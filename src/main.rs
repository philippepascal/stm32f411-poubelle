#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;

use cortex_m_rt::entry;
use panic_halt as _;

use hal::gpio::alt::otg_fs::{Dm, Dp};
use hal::{otg_fs, pac, prelude::*};
use stm32f4xx_hal as hal;

use stm32f4xx_hal::otg_fs::UsbBus as HalUsbBus;
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::mem::MaybeUninit;
use usb_device::device::UsbDevice; // helper to block on nb::Result

/// Queue a USB log message if enough time has passed.
/// Safe wrapper that can be called from main loop and (later) other modules.
/// `tick` is the current timestamp/counter.
/// `text` is the log body (no newline needed).
pub fn usb_log(tick: u32, text: &str) {
    unsafe {
        // Simple rate limiter: only log if (tick - LAST_LOG_TICK) >= LOG_PERIOD_TICKS
        // if tick.wrapping_sub(LAST_LOG_TICK) < LOG_PERIOD_TICKS {
        //     return;
        // }
        LAST_LOG_TICK = tick;

        // Build timestamped message
        let msg = alloc::format!("[{:08}] {}\r\n", tick, text);

        // Get the global USB device + serial
        #[allow(static_mut_refs)]
        let dev_ref: &mut UsbDevice<HalUsbBusType> = USB_DEV.assume_init_mut();
        #[allow(static_mut_refs)]
        let serial_ref: &mut SerialPort<HalUsbBusType> = USB_SERIAL.assume_init_mut();

        // Poll USB so host traffic stays serviced
        let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsbBusType>; 1] =
            [serial_ref];
    
        let _ = dev_ref.poll(&mut classes);

        // Then attempt to write no matter what
        let _ = serial_ref.write(msg.as_bytes());

    }
}
pub fn usb_poll_once() {
    unsafe {
        #[allow(static_mut_refs)]
        let dev_ref: &mut UsbDevice<HalUsbBusType> = USB_DEV.assume_init_mut();
        #[allow(static_mut_refs)]
        let serial_ref: &mut SerialPort<HalUsbBusType> = USB_SERIAL.assume_init_mut();

        let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsbBusType>; 1] =
            [serial_ref];
        let _ = dev_ref.poll(&mut classes);
    }
}

type HalUsbBusType = HalUsbBus<otg_fs::USB>;
// We'll treat this as the "bus allocator" the stack expects.
static mut USB_ALLOC: MaybeUninit<UsbBusAllocator<HalUsbBusType>> = MaybeUninit::uninit();
// CDC-ACM class and UsbDevice
static mut USB_SERIAL: MaybeUninit<SerialPort<HalUsbBusType>> = MaybeUninit::uninit();
static mut USB_DEV: MaybeUninit<UsbDevice<HalUsbBusType>> = MaybeUninit::uninit();

#[unsafe(link_section = ".axisram")]
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

mod door_state;
mod mem;

use crate::door_state::*;
use crate::mem::*;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[entry]
fn main() -> ! {
    // Take ownership of the peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        // .use_hse(8.MHz()) //that fails.
        .sysclk(100.MHz())
        .require_pll48clk()
        .freeze();

    // Enable GPIOC (for the LED on PC13 on many BlackPill-style boards)
    let gpioc = dp.GPIOC.split();

    // USB setup

    // configure PA11/PA12 for OTG FS using the HAL's strong pin wrappers
    let gpioa = dp.GPIOA.split();
    // Put PA11/PA12 into AF10 (USB OTG FS)
    let pa11_usb = gpioa.pa11.into_alternate::<10>();
    let pa12_usb = gpioa.pa12.into_alternate::<10>();

    // Wrap them in the HAL's strong USB pin types
    let dm = Dm::PA11(pa11_usb);
    let dpin = Dp::PA12(pa12_usb);

    // Build the low-level OTG FS peripheral
    let usb_periph = otg_fs::USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: dm,
        pin_dp: dpin,
        hclk: clocks.hclk(),
    };

    // Move everything into statics so they have 'static lifetime
    unsafe {
        // Build whatever the HAL thinks the "bus" is

        #[allow(static_mut_refs)]
        let bus = otg_fs::UsbBus::new(usb_periph, &mut EP_MEMORY);

        // Make an allocator from that bus
        // IMPORTANT: we are NOT trying to store `bus` separately anymore.
        // USB_ALLOC.write(UsbBusAllocator::new(bus));

        #[allow(static_mut_refs)]
        USB_ALLOC.write(bus);

        // Build SerialPort from the allocator we just wrote

        #[allow(static_mut_refs)]
        let serial = SerialPort::new(USB_ALLOC.assume_init_ref());

        #[allow(static_mut_refs)]
        USB_SERIAL.write(serial);

        // Build UsbDevice from the same allocator

        #[allow(static_mut_refs)]
        let usb_dev = UsbDeviceBuilder::new(USB_ALLOC.assume_init_ref(), UsbVidPid(0x16c0, 0x27dd))
            .device_class(USB_CLASS_CDC)
            .build();

        #[allow(static_mut_refs)]
        USB_DEV.write(usb_dev);
    }

    unsafe {
        // Pick a heap size. Let's say 8 KB for now.
        // cortex_m_rt gives us `extern "C" { static _sheap: u8; }` / `heap_start()`,
        // but `CortexMHeap` provides helper.
        const HEAP_SIZE: usize = 8 * 1024;
        ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE);
    }

    // // Read calibration data from flash
    let calib_data = DoorCalibrationData::read_from_flash();
    let mut state: Box<dyn State> = match calib_data.is_valid() {
        true => {
            // Use the calibration data
            init_state(calib_data.max_open_steps)
        }
        false => {
            // Handle invalid calibration data (e.g., set defaults or enter error state)
            calibrate()
        }
    };

    // On many STM32F411 "BlackPill"-style boards, PC13 is the user LED.
    let mut led = gpioc.pc13.into_push_pull_output();
    let mut tick: u32 = 0;

    // Simple blink loop
    loop {
        tick = tick.wrapping_add(1);

        // keep USB stack alive no matter what
        usb_poll_once();

        // blink LED
        if tick % 100_000 == 0 {
            led.set_high();
        }
        if tick % 200_000 == 0 {
            led.set_low();
        }

        // throttle logging
        if tick % 10_000 == 0 {
            usb_log(tick, "main loop alive");
        }

        // eventually: state = state.run();
    }
}
