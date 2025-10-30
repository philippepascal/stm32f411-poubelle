#![no_std]
#![no_main]

use stm32f4xx_hal as hal;
use hal::{
    gpio::{Output, PushPull, gpioa::PA0, gpioa::PA1, gpioc::PC13},
    otg_fs,
    pac,
    prelude::*,
};
use rtic::app;
use panic_halt as _;
use usb_device::{prelude::*, class_prelude::UsbBusAllocator};
use usbd_serial::SerialPort;
use stm32f4xx_hal::otg_fs::UsbBus as HalUsbBus;
use cortex_m::asm;

mod flash_store;
use flash_store::{Config, load_config, save_config_blocking};

// --- Type aliases for USB stack
type HalUsb = HalUsbBus<otg_fs::USB>;
type UsbDev<'a> = usb_device::device::UsbDevice<'a,HalUsb>;
type Serial<'a> = usbd_serial::SerialPort<'a,HalUsb>;


#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    // Resources shared across tasks (RTIC locks these automatically)
    #[shared]
    struct Shared {
        usb_dev: UsbDev<'static>,
        serial: SerialPort<'static,HalUsb>,
        dir_pin: PA0<Output<PushPull>>,
        step_pin: PA1<Output<PushPull>>,
        led_pin: PC13<Output<PushPull>>,
        cfg: Config,
    }

    // Resources local to specific tasks (not shared)
    #[local]
    struct Local {
        // e.g., LED pin, counters, etc.
    }

    #[init(local = [
        ep_memory: [u32; 1024] = [0; 0x400],
        usb_bus: Option<UsbBusAllocator<HalUsb>> = None,
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut core = ctx.core;
        let dp: pac::Peripherals = ctx.device;

        // RCC / clocks
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(100.MHz())
            .require_pll48clk()
            .freeze();

        // GPIO
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();
        let dir_pin = gpioa.pa0.into_push_pull_output();
        let step_pin = gpioa.pa1.into_push_pull_output();
        let mut led_pin = gpioc.pc13.into_push_pull_output();
        // Many F411 "Black Pill" boards have the LED on PC13 active-low; start LED off.
        let _ = led_pin.set_high();

        // USB pins
        let pa11 = gpioa.pa11.into_alternate::<10>();
        let pa12 = gpioa.pa12.into_alternate::<10>();

        // USB peripheral
        let usb_periph = otg_fs::USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: hal::gpio::alt::otg_fs::Dm::PA11(pa11),
            pin_dp: hal::gpio::alt::otg_fs::Dp::PA12(pa12),
            hclk: clocks.hclk(),
        };

        // Create the USB bus allocator in an init-local 'static slot so we can get
        // a &'static UsbBusAllocator for SerialPort/UsbDevice lifetimes.
        *ctx.local.usb_bus = Some(HalUsb::new(usb_periph, ctx.local.ep_memory));
        let bus: &'static UsbBusAllocator<HalUsb> = ctx.local.usb_bus.as_ref().unwrap();

        let serial: Serial<'static> = SerialPort::new(bus);
        let usb_dev: UsbDev<'static> = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        // Try load or use defaults
        let cfg = load_config().unwrap_or(Config {
            // step_rate_hz: 1000,
            // direction_default_opening: true,
            // pulses_per_rev: 200,
            travel_step_count: 0,
        });

        // Later: store flash_writer in Shared so you can call save_config_blocking()

        // Configure SysTick (optional if you plan to use it for periodic work)
        core.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        core.SYST.set_reload(100_000 - 1); // ~1 kHz at 100 MHz
        core.SYST.clear_current();
        core.SYST.enable_interrupt();
        core.SYST.enable_counter();

        (
            Shared { usb_dev, serial, dir_pin, step_pin, led_pin, cfg },
            Local { }
        )
    }

    // Bind to the USB interrupt to keep the stack serviced
    #[task(binds = OTG_FS, shared = [usb_dev, serial])]
    fn usb_irq(ctx: usb_irq::Context) {
        // Poll the device/class
        (ctx.shared.usb_dev, ctx.shared.serial).lock(|dev, serial| {
            let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsb>; 1] = [serial];
            let _ = dev.poll(&mut classes);
        });
    }

    // Example: periodic task via SysTick to do logging or stepping
    #[task(binds = SysTick, local = [ticks: u32 = 0, led_on: bool = false], shared = [dir_pin, step_pin, serial, usb_dev, led_pin])]
    fn systick(mut ctx: systick::Context) {
        // Count milliseconds (assuming reload was set for 1 kHz)
        *ctx.local.ticks += 1;
        let mut do_blink = false;
        if *ctx.local.ticks >= 500 {
            *ctx.local.ticks = 0;
            do_blink = true;
        }

        // Keep USB serviced every tick; emit message when we blink
        (ctx.shared.usb_dev, ctx.shared.serial).lock(|dev, serial| {
            let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsb>; 1] = [serial];
            let _ = dev.poll(&mut classes);
            if do_blink {
                let _ = serial.write(b"blink\r\n");
            }
        });

        // Toggle LED on blink (accounting for typical active-low PC13 on BlackPill).
        if do_blink {
            (ctx.shared.led_pin).lock(|led| {
                if *ctx.local.led_on {
                    let _ = led.set_high(); // off if active-low
                } else {
                    let _ = led.set_low();  // on if active-low
                }
                *ctx.local.led_on = !*ctx.local.led_on;
            });
        }

        // Optional: one step pulse every tick (kept from your example)
        (ctx.shared.step_pin).lock(|step| {
            step.set_high();
            asm::delay(1000);
            step.set_low();
        });
    }

    // Your `operate` can be a normal function, or a task you `spawn`.
    // As a task, it gets safe mutable access to the pins:
    #[task(shared = [dir_pin, step_pin])]
    async fn operate(mut ctx: operate::Context, steps: u16, opening: bool) {
        ctx.shared.dir_pin.lock(|dir| {
            if opening { dir.set_high(); } else { dir.set_low(); }
        });

        ctx.shared.step_pin.lock(|step| {
            for _ in 0..steps {
                step.set_high();
                asm::delay(1000);
                step.set_low();
                asm::delay(1000);
            }
        });
    }

    #[task]
    async fn save_cfg(_ctx: save_cfg::Context, cfg: Config) {
        let _ = save_config_blocking(&cfg);
    }

    #[task(shared = [cfg])]
    async fn persist_cfg(mut ctx: persist_cfg::Context, new_cfg: Config) {
        ctx.shared.cfg.lock(|c| *c = new_cfg);
        let _ = save_config_blocking(&new_cfg);
    }
}