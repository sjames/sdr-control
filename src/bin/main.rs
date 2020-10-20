// std and main are not available for bare metal software
#![no_std]
#![no_main]

use sdr_control as _; // global logger + panicking-behavior + memory layout

// SI5351
// SCL connected to PB8
// SDA connected to PB9

// Rotary
// P1 -> A8
// P2 -> A9
// TIM1 used as rotary encoder. Inputs are pulled-up

use cortex_m::asm::{delay, wfi};
use cortex_m_rt::entry; // The runtime
use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
use stm32f1xx_hal::stm32::{interrupt, tim1, Interrupt, TIM1};
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use si5351;
use si5351::{Si5351, Si5351Device};

use sdr_control::vco::{self, Vco};

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
        .sysclk(72.mhz())
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
    /*
    if let Ok(_) = clock.set_quadrature_clock_freq(
        si5351::PLL::A,
        (si5351::ClockOutput::Clk0, si5351::ClockOutput::Clk1),
        7_465_000,
    ) {
       defmt::info!("VCO freq:{:?}",clock.get_vco_frequency()) ;
    } else {
        defmt::error!("set_frequency failed");
        sdr_control::exit();
    }
    */
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

    let tim1 = dp.TIM1;
    let _tim1 = Timer::tim1(tim1, &clocks, &mut rcc.apb2);
    let tim1 = setup_tim1_for_rotary();
    // pull-up the timer1 inputs to connect the rotary.
    let _tim1_ch1 = gpioa.pa8.into_pull_up_input(&mut gpioa.crh);
    let _tim1_ch2 = gpioa.pa9.into_pull_up_input(&mut gpioa.crh);


    let mut vco = Vco::create(14_000_000);
    vco.set_multiplier(vco::Multiplier::Hundred);

    loop {
        vco.run_once(
            || tim1.cnt.read().bits() as u16,
            |freq| {
                led.set_low().ok();
                if let Ok(_) = clock.set_quadrature_clock_freq(
                    si5351::PLL::A,
                    (si5351::ClockOutput::Clk0, si5351::ClockOutput::Clk1),
                    freq,
                ) {
                    defmt::info!("VCO freq:{:?} freq:{:?}", clock.get_vco_frequency(), freq);
                } else {
                    defmt::error!("set_frequency failed");
                    sdr_control::exit();
                }
                led.set_high().ok();
            },
        );
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
    }
}

fn setup_tim1_for_rotary() -> &'static tim1::RegisterBlock {
    let tim1 = unsafe { &*TIM1::ptr() };

    tim1.smcr.write(|s| s.sms().encoder_mode_1());
    tim1.arr.write(|a| a.arr().bits(65535));
    tim1.ccer.write(|c| {
        c.cc1p().clear_bit(); // rising edge
        c.cc2p().clear_bit(); // rising edge
        c.cc1np().clear_bit();
        c.cc2np().clear_bit()
    });
    tim1.ccmr1_input_mut().write(|c| {
        c.ic2f().bits(0);
        c.cc1s().ti2();
        c.cc2s().ti1();
        c.ic1f().bits(0)
    });

    tim1.ccmr2_input_mut().write(|c| c.cc4s().ti4());

    tim1.cr1.write(|c| c.cen().set_bit());
    tim1.cnt.write(|w| unsafe { w.bits(65535 / 2) });
    tim1
}
