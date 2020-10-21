use core::convert::TryInto;
use core::mem;
use usb_device::class_prelude::*;
use usb_device::Result;

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_INTERFACE_CLASS_AUDIO: u8 = 0x01;

const USB_AUDIO_SUBCLASS_UNDEFINED:u8 = 0x0;
const USB_AUDIO_SUBCLASS_AUDIOCONTROL:u8 = 0x01;
const USB_AUDIO_SUBCLASS_AUDIOSTREAMING:u8 = 0x02;
const USB_AUDIO_SUBCLASS_MIDISTREAMING:u8 = 0x03;

const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02;
const USB_AUDIO_PROTOCOL_NONE: u8 = 0x00;

const CS_DEVICE: u8 = 0x21;
const CS_CONFIGURATION: u8 = 0x22;
const CS_STRING: u8 = 0x23;
const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

const EP_GENERAL : u8 = 0x1;

const AC_DESC_TYPE_HEADER : u8 = 0x1;
const AC_DESC_TYPE_INPUT_TERMINAL : u8 = 0x2;
const AC_DESC_TYPE_OUTPUT_TERMINAL : u8 = 0x3;
const AC_DESC_TYPE_MIXER_UNIT : u8 = 0x4;
const AC_DESC_TYPE_SELECTOR_UNIT : u8 = 0x5;
const AC_DESC_TYPE_FEATURE_UNIT : u8 = 0x6;
const AC_DESC_TYPE_PROCESSING_UNIT : u8 = 0x7;
const AC_DESC_TYPE_EXTENSION_UNIT : u8 = 0x8;

const AC_DESC_ST_GENERAL: u8 = 0x1;
const AC_DESC_ST_FORMAT_TYPE: u8 = 0x2;
const AC_DESC_ST_FORMAT_SPECIFIC: u8 = 0x3;

const AUDIO_SUB_TYPE_HEADER: u8 = 0x01;
const AUDIO_SUB_TYPE_INPUT_TERMINAL : u8 = 0x02;
const AUDIO_SUB_TYPE_FEATURE_UNIT : u8 = 0x06;
const AUDIO_SUB_TYPE_OUTPUT_TERMINAL : u8 = 0x03;
const AUDIO_SUB_TYPE_AS_GENERAL : u8 = 0x01;
const AUDIO_SUB_TYPE_FORMAT_TYPE : u8 = 0x02;

/// Packet level implementation of a CDC-ACM serial port.
///
/// This class can be used directly and it has the least overhead due to directly reading and
/// writing USB packets with no intermediate buffers, but it will not act like a stream-like serial
/// port. The following constraints must be followed if you use this class directly:
///
/// - `read_packet` must be called with a buffer large enough to hold max_packet_size bytes, and the
///   method will return a `WouldBlock` error if there is no packet to be read.
/// - `write_packet` must not be called with a buffer larger than max_packet_size bytes, and the
///   method will return a `WouldBlock` error if the previous packet has not been sent yet.
/// - If you write a packet that is exactly max_packet_size bytes long, it won't be processed by the
///   host operating system until a subsequent shorter packet is sent. A zero-length packet (ZLP)
///   can be sent if there is no other data to send. This is because USB bulk transactions must be
///   terminated with a short packet, even if the bulk endpoint is used for stream-like data.
pub struct UsbAudioClass<'a,B: UsbBus> {

    audio_control_if : InterfaceNumber,
    audio_stream : InterfaceNumber,
    alt_audio_stream : InterfaceNumber,
    audio_in_ep:EndpointIn<'a, B>,


    /*
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    line_coding: LineCoding,
    dtr: bool,
    rts: bool,
    */
}

impl<B: UsbBus> UsbAudioClass<'_, B> {
    /// Creates a new UsbAudioClass with the provided UsbBus and max_packet_size in bytes. For
    /// full-speed devices, max_packet_size has to be one of 8, 16, 32 or 64.
    pub fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> UsbAudioClass<'_, B> {
        UsbAudioClass {
            audio_control_if : alloc.interface(),
            audio_stream : alloc.interface(),
            alt_audio_stream : alloc.interface(),
            audio_in_ep : alloc.bulk(max_packet_size),
        }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        self.audio_in_ep.max_packet_size()
    }

    /// Writes a single packet into the IN endpoint.
    pub fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
        self.audio_in_ep.write(data)
    }

    /// Gets the address of the IN endpoint.
    pub(crate) fn write_ep_address(&self) -> EndpointAddress {
        self.audio_in_ep.address()
    }
}

impl<B: UsbBus> UsbClass<B> for UsbAudioClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        
        writer.iad(
            self.audio_control_if,
            3,
            0x0, // device defined at interface level
            0x0, // subclass unused
            USB_AUDIO_PROTOCOL_NONE)?;

        writer.interface(
            self.audio_control_if,
            USB_INTERFACE_CLASS_AUDIO,
            USB_AUDIO_SUBCLASS_AUDIOCONTROL,
            USB_AUDIO_PROTOCOL_NONE)?;

        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_HEADER, // bDescriptorSubtype
                0x00, 0x01, // bcdADC (1.0),
                0x00, 43, // wTotalLength (Compute this!)
                0x01, // bInCollection (1 streaming interface)
                0x01, // baInterfaceNr (Interface 1 is stream)
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_INPUT_TERMINAL, // bDescriptorSubtype
                0x01, // bTerminalID 1,
                0x07, 0x10, // wTerminalType (radio receiver)
                0x00, // bAssocTerminal (none)
                0x02, // bNrChannels - 2 for I and Q
                0x00, 0x03, // wChannelConfig (left, right)
                0x00, // iChannelNames (none)
                0x00, // iTerminal (none)
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_FEATURE_UNIT, // bDescriptorSubtype
                0x02, // bUnitID,
                0x01, // bSourceID (input terminal 1)
                0x02, // bControlSize (2 bytes)
                0x00, 0x01, // Master controls
                0x00, 0x00, // Channel 0 controls
                0x00, 0x00, // Channel 1 controls
                0x00, // iFeature (none)
            ])?;

        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_OUTPUT_TERMINAL, // bDescriptorSubtype
                0x03, // bTerminalID,
                0x01, 0x01, // wTerminalType (USB Streaming)
                0x00, // bAssocTerminal (none)
                0x02, // bSourceID (feature unit 2)
                0x00, // iTerminal (none)
            ])?;

        writer.interface(
            self.audio_stream,
            USB_INTERFACE_CLASS_AUDIO,
            USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
            USB_AUDIO_PROTOCOL_NONE)?;
        
        //alternate audio stream
        writer.interface(
            self.alt_audio_stream,
            USB_INTERFACE_CLASS_AUDIO,
            USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
            USB_AUDIO_PROTOCOL_NONE)?;

        // Audio Stream Audio Class Descriptor
        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_AS_GENERAL, // bDescriptorSubtype
                0x03, // bTerminalID,
                0x00, // bDelay
                0x00, 0x01, // wFormatTag (PCM Format)
            ])?;
        
        // Format Type Audio Descriptor
        writer.write(
            CS_INTERFACE,
            &[
                AUDIO_SUB_TYPE_FORMAT_TYPE, // bDescriptorSubtype
                0x01, // bFormatType (TYPE_I)
                0x02, // bNrChannels (2)
                0x02, // bSubFrameSize (2)
                0x10, // bBitResolution (16 bits)
                0x01, // bSamFreqType (1 sample frequency)
                0x80, // 8*2 = 16 KHz byte 0
                0x3E, // 8*2 = 16 KHz byte 1
                0x00, // 8*2 = 16 KHz byte 2
            ])?;
         

        // TODO: Set the necessary flags for Isochronous
        writer.endpoint(&self.audio_in_ep)?;

        // Isochronous endpoint Audio Class descriptor
        writer.write(
            CS_ENDPOINT,
            &[
                EP_GENERAL, // bDescriptorSubtype
                0x00, // bmAttributes (none)
                0x02, // bLockDelayUnits (PCM Samples)
                0x00, 0x00, // wLockDelay (0)  - should be zero for asynchronous
            ])?;
        Ok(())
    }

    fn reset(&mut self) {
        
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.audio_control_if) as u16)
        {
            return;
        }

        match req.request {
            /*
            REQ_GET_LINE_CODING if req.length == 7 => {
                xfer.accept(|data| {
                    data[0..4].copy_from_slice(&self.line_coding.data_rate.to_le_bytes());
                    data[4] = self.line_coding.stop_bits as u8;
                    data[5] = self.line_coding.parity_type as u8;
                    data[6] = self.line_coding.data_bits;

                    Ok(7)
                }).ok();
            },
            */
            _ => { xfer.reject().ok(); },
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.audio_control_if) as u16)
        {
            return;
        }

        match req.request {
            /*
            REQ_SEND_ENCAPSULATED_COMMAND => {
                // We don't actually support encapsulated commands but pretend we do for standards
                // compatibility.
                xfer.accept().ok();
            },*/
     
            _ => { xfer.reject().ok(); }
        };  
    }
}


