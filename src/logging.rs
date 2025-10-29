extern crate alloc;

use alloc::format;
use core::mem::MaybeUninit;

use stm32f4xx_hal as hal;
use hal::otg_fs;
use hal::otg_fs::UsbBus as HalUsbBus;
use hal::gpio::alt::otg_fs::{Dm, Dp};

use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use usb_device::device::UsbDevice;

// --- Public types & globals used by main.rs ---

// The HAL's USB bus type
pub type HalUsbBusType = HalUsbBus<otg_fs::USB>;

// Bus allocator, CDC class, and UsbDevice live in statics so they get 'static lifetime.
pub static mut USB_ALLOC: MaybeUninit<UsbBusAllocator<HalUsbBusType>> = MaybeUninit::uninit();
pub static mut USB_SERIAL: MaybeUninit<SerialPort<HalUsbBusType>> = MaybeUninit::uninit();
pub static mut USB_DEV: MaybeUninit<UsbDevice<HalUsbBusType>> = MaybeUninit::uninit();

#[unsafe(link_section = ".axisram")]
pub static mut EP_MEMORY: [u32; 1024] = [0; 1024];

// --- Logging rate limiting state ---
const LOG_PERIOD_TICKS: u32 = 200_000;
static mut LAST_LOG_TICK: u32 = 0;

// --- Public API ---

/// Initialize the USB peripheral, allocator, CDC class, and USB device.
/// Call this once from main() after clocks are configured and GPIOA is split.
/// Returns nothing; fills the statics above.
pub unsafe fn usb_init(
    usb_global: hal::pac::OTG_FS_GLOBAL,
    usb_device: hal::pac::OTG_FS_DEVICE,
    usb_pwrclk: hal::pac::OTG_FS_PWRCLK,
    clocks: &hal::rcc::Clocks,
    dm: Dm,
    dpin: Dp,
) {
    // Build the low-level OTG FS peripheral from the pieces we were given
    let usb_periph = otg_fs::USB {
        usb_global,
        usb_device,
        usb_pwrclk,
        pin_dm: dm,
        pin_dp: dpin,
        hclk: clocks.hclk(),
    };

    // Allocate the bus using our endpoint buffer
    #[allow(static_mut_refs)]
    let bus = otg_fs::UsbBus::new(usb_periph, &mut EP_MEMORY);
    #[allow(static_mut_refs)]
    USB_ALLOC.write(bus);

    // Create CDC serial class
    #[allow(static_mut_refs)]
    let serial = SerialPort::new(USB_ALLOC.assume_init_ref());
    #[allow(static_mut_refs)]
    USB_SERIAL.write(serial);

    // Create the UsbDevice (VID/PID etc.)
    #[allow(static_mut_refs)]
    let usb_dev = UsbDeviceBuilder::new(
        USB_ALLOC.assume_init_ref(),
        UsbVidPid(0x16c0, 0x27dd),
    )
    .device_class(USB_CLASS_CDC)
    .build();

    #[allow(static_mut_refs)]
    USB_DEV.write(usb_dev);
}

/// Throttled USB log with timestamp.
/// You pass in the current `tick` and a message, and this will:
/// - rate limit
/// - poll USB to keep the host happy
/// - write one line with "[00012345] your message"
pub fn usb_log(tick: u32, text: &str) {
    unsafe {
        // Rate limit
        if tick.wrapping_sub(LAST_LOG_TICK) < LOG_PERIOD_TICKS {
            return;
        }
        LAST_LOG_TICK = tick;

        // Build message
        let msg = format!("[{:08}] {}\r\n", tick, text);

        // Access global USB device and class
        #[allow(static_mut_refs)]
        let dev_ref: &mut UsbDevice<HalUsbBusType> = USB_DEV.assume_init_mut();
        #[allow(static_mut_refs)]
        let serial_ref: &mut SerialPort<HalUsbBusType> = USB_SERIAL.assume_init_mut();

        // Poll and write
        let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsbBusType>; 1] =
            [serial_ref];
        if dev_ref.poll(&mut classes) {
            let _ = serial_ref.write(msg.as_bytes());
        }
    }
}