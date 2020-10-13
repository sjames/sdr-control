// std and main are not available for bare metal software
#![no_std]
#![no_main]

use sdr_control as _; // global logger + panicking-behavior + memory layout

// SI5351
// SCL connected to PB8
// SDA connected to PB9

use cortex_m::asm::{delay, wfi};
use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions

use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use si5351;
use si5351::{Si5351, Si5351Device};

// For use in interrupt handlers
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
    // This must be enabled first. The HAL provides some abstractions for
    // us: First get a handle to the RCC peripheral:
    let mut rcc = dp.RCC.constrain();
    // Now we have access to the RCC's registers. The GPIOC can be enabled in
    // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
    // register to the `split` function.
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    defmt::info!("sdr_control");

    // This gives us an exclusive handle to the GPIOC peripheral. To get the
    // handle to a single pin, we need to configure the pin first. Pin C13
    // is usually connected to the Bluepills onboard LED.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Now we need a delay object. The delay is of course depending on the clock
    // frequency of the microcontroller, so we need to fix the frequency
    // first. The system frequency is set via the FLASH_ACR register, so we
    // need to get a handle to the FLASH peripheral first:
    let mut flash = dp.FLASH.constrain();
    // Now we can set the controllers frequency to 8 MHz:
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());
    // The `clocks` handle ensures that the clocks are now configured and gives
    // the `Delay::new` function access to the configured frequency. With
    // this information it can later calculate how many cycles it has to
    // wait. The function also consumes the System Timer peripheral, so that no
    // other function can access it. Otherwise the timer could be reset during a
    // delay.
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 200_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        &mut rcc.apb1,
        1000,
        10,
        1000,
        1000,
    );

    let mut clock = Si5351Device::new(i2c, false, 25_000_000);

    defmt::info!("Going to init 5351");
    if let Ok(_) = clock.init(si5351::CrystalLoad::_10) {
        defmt::debug!("init done");
        /*
        if let Ok(_)= clock.set_frequency(si5351::PLL::A, si5351::ClockOutput::Clk0, 14_175_000) {
        } else {
            defmt::error!("set_frequency failed");
            sdr_control::exit();
        }
        */

        if let Ok(_) = clock.set_quadrature_clock_freq(
            si5351::PLL::A,
            (si5351::ClockOutput::Clk1, si5351::ClockOutput::Clk0),
            48_550_000,
        ) {
           defmt::info!("VCO freq:{:?}",clock.get_vco_frequency()) ;
        } else {
            defmt::error!("set_frequency failed");
            sdr_control::exit();
        }
    } else {
        defmt::error!("5351 init failed");
        sdr_control::exit();
    }

    defmt::error!("Blah");

    // USB Initialization

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    cortex_m::asm::delay(clocks.sysclk().0 / 100);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };
    unsafe {
        USB_BUS = Some(UsbBus::new(usb));
        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Sojan")
            .product("SDR Controller")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        USB_DEVICE = Some(usb_dev);
    }

    //let mut nvic = cp.NVIC;
    //nvic.enable(Interrupt::USB_HP_CAN_TX);
    //nvic.enable(Interrupt::USB_LP_CAN_RX0);

    loop {
        defmt::debug!("loop");
        led.set_high().ok();
        delay.delay_ms(1_00_u16);
        //wfi();
        led.set_low().ok();
        delay.delay_ms(1_000_u16);
    }

    #[interrupt]
    fn USB_HP_CAN_TX() {
        usb_interrupt();
    }

    #[interrupt]
    fn USB_LP_CAN_RX0() {
        usb_interrupt();
    }

    fn usb_interrupt() {
        let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
        let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

        if !usb_dev.poll(&mut [serial]) {
            return;
        }
        //led.set_high().ok();
        let mut buf = [0u8; 8];
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }
                serial.write(&buf[0..count]).ok();
            }
            _ => {}
        }
        //led.set_low().ok();
    }

    /*
    loop {
        led.set_high().ok();
        delay.delay_ms(1_00_u16);
        led.set_low().ok();
        delay.delay_ms(1_000_u16);
    }
    */
}
