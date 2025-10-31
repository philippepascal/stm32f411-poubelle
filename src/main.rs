#![no_std]
#![no_main]

use stm32f4xx_hal as hal;
use hal::{
    gpio::{Output, PushPull, Input, ExtiPin, Edge, gpioa::PA0, gpioa::PA1, gpioa::PA7, gpioc::PC13},
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
        button_event: bool,
    }

    // Resources local to specific tasks (not shared)
    #[local]
    struct Local {
        button_pin: PA7<Input>,
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
        let mut button_pin = gpioa.pa7.into_pull_up_input();
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

        // Configure EXTI on PA7 (lines 5..9 use EXTI9_5 interrupt)
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;
        button_pin.make_interrupt_source(&mut syscfg);
        button_pin.trigger_on_edge(&mut exti, Edge::Falling);
        button_pin.enable_interrupt(&mut exti);

        // Later: store flash_writer in Shared so you can call save_config_blocking()

        // Configure SysTick (optional if you plan to use it for periodic work)
        core.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        core.SYST.set_reload(100_000 - 1); // ~1 kHz at 100 MHz
        core.SYST.clear_current();
        core.SYST.enable_interrupt();
        core.SYST.enable_counter();

        // Kick off the state machine
        let _ = start_door::spawn();

        (
            Shared { usb_dev, serial, dir_pin, step_pin, led_pin, cfg, button_event: false },
            Local { button_pin }
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
    // Async logger: write a static message to USB serial safely
    #[task(shared = [usb_dev, serial])]
    async fn log(ctx: log::Context, msg: &'static [u8]) {
        (ctx.shared.usb_dev, ctx.shared.serial).lock(|dev, serial| {
            let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsb>; 1] = [serial];
            let _ = dev.poll(&mut classes);
            let _ = serial.write(msg);
        });
    }
    #[task(binds = EXTI9_5, local = [button_pin], shared = [button_event])]
    fn exti_irq(mut ctx: exti_irq::Context) {
        // Clear the interrupt first
        ctx.local.button_pin.clear_interrupt_pending_bit();
        // Set the shared event flag
        ctx.shared.button_event.lock(|f| *f = true);
    }

    // Example: periodic task via SysTick to do logging or stepping
    #[task(binds = SysTick, local = [ticks: u32 = 0, led_on: bool = false], shared = [dir_pin, step_pin, serial, usb_dev, led_pin, button_event])]
    fn systick(mut ctx: systick::Context) {
        // Count milliseconds (assuming reload was set for 1 kHz)
        *ctx.local.ticks += 1;
        let mut do_blink = false;
        if *ctx.local.ticks >= 500 {
            *ctx.local.ticks = 0;
            do_blink = true;
        }

        // // Check and consume button event (edge-triggered)
        // let mut got_button = false;
        // ctx.shared.button_event.lock(|f| { if *f { *f = false; got_button = true; } });

        // Keep USB serviced every tick; emit message when we blink
        (ctx.shared.usb_dev, ctx.shared.serial).lock(|dev, serial| {
            let mut classes: [&mut dyn usb_device::class_prelude::UsbClass<HalUsb>; 1] = [serial];
            let _ = dev.poll(&mut classes);
            if do_blink { let _ = serial.write(b"blink\r\n"); }
            // if got_button { let _ = serial.write(b"button\r\n"); }
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
        // keep commented for testing
        // (ctx.shared.step_pin).lock(|step| {
        //     step.set_high();
        //     asm::delay(1000);
        //     step.set_low();
        // });
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

    // --- State machine bootstrap ---
    #[task(shared = [cfg])]
    async fn start_door(mut ctx: start_door::Context) {
        let _ = log::spawn(b"-> starting\r\n");
        let mut steps = 0u32;
        ctx.shared.cfg.lock(|c| {
            steps = c.travel_step_count as u32;
        });

        if steps == 0 {
            // No calibration yet → go calibrate
            let _ = calibrate::spawn();
        } else {
            // Known travel distance → wait for closed state
            let _ = wait_closed::spawn();
        }
    }

    // Fake calibrate: set travel_step_count in RAM only (no flash write), then wait for closed
    #[task(shared = [cfg])]
    async fn calibrate(mut ctx: calibrate::Context) {
        let _ = log::spawn(b"-> calibrating\r\n");
        ctx.shared.cfg.lock(|c| {
            c.travel_step_count = 2000;
        });
        let _ = wait_closed::spawn();
    }

    // Wait for button event, then spawn opening task
    #[task(shared = [button_event, usb_dev, serial])]
    async fn wait_closed(mut ctx: wait_closed::Context) {
        let _ = log::spawn(b"-> wait closed\r\n");
        let mut ms = 0u32;
        loop {
            // Consume the edge-triggered button event
            let mut got = false;
            ctx.shared.button_event.lock(|f| { if *f { *f = false; got = true; } });

            if got {
                let _ = opening::spawn();
                return; // end wait_closed
            }

            // Heartbeat log every ~1000 ms
            ms = ms.wrapping_add(1);
            if ms >= 1000 {
                ms = 0;
                let _ = log::spawn(b"-> wait closed\r\n");
            }

            // Small cooperative delay to avoid tight spinning (~1ms at 100 MHz)
            asm::delay(100_000);
        }
    }

    // Open by stepping `travel_step_count` steps with a trapezoidal profile:
    // 10% accel (slow -> fast), 80% cruise (fast), 10% decel (fast -> slow).
    // Uses busy-wait delays (asm::delay cycles) for simplicity.
    #[task(shared = [cfg, dir_pin, step_pin, usb_dev, serial])]
    async fn opening(mut ctx: opening::Context) {
        // Read number of steps from config
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);
        if total_steps == 0 {
            // No motion needed; transition immediately
            let _ = wait_open::spawn();
            return;
        }

        // Log start
        let _ = log::spawn(b"-> opening\r\n");

        // Timing constants (tune for your mechanics and sysclk)
        const SLOW_DELAY: u32 = 20_000; // ~200 µs half-period (≈ 2.5 kHz)
        const FAST_DELAY: u32 = 3_000;  // ~30  µs half-period  (≈ 16.7 kHz)

        // Phase partitioning: 10% accel, 80% cruise, 10% decel
        let mut accel = core::cmp::max(1, total_steps / 10);
        let mut decel = accel;
        if accel * 2 > total_steps { accel = total_steps / 2; decel = total_steps - accel; }
        let cruise = total_steps.saturating_sub(accel + decel);

        // Set direction HIGH once
        ctx.shared.dir_pin.lock(|dir| { dir.set_high(); });

        // Progress tracking (log at 10%,20%,...,90%)
        let mut steps_done: u32 = 0;
        let mut next_pct: u32 = 10;
        let marks: [&[u8]; 9] = [
            b"10%\r\n", b"20%\r\n", b"30%\r\n", b"40%\r\n", b"50%\r\n",
            b"60%\r\n", b"70%\r\n", b"80%\r\n", b"90%\r\n",
        ];
        let mut mark_idx = 0usize;

        // Helper: do one pulse and maybe log progress
        let mut pulse = |delay: u32, ctx: &mut opening::Context| {
            // emit one step pulse
            ctx.shared.step_pin.lock(|step| {
                step.set_high();
                asm::delay(delay);
                step.set_low();
                asm::delay(delay);
            });
            steps_done = steps_done.saturating_add(1);

            // check progress milestone
            if mark_idx < marks.len() && steps_done.saturating_mul(100) / total_steps >= next_pct {
                let _ = log::spawn(marks[mark_idx]);
                next_pct += 10;
                mark_idx += 1;
            }
        };

        // Accel: linearly interpolate SLOW -> FAST
        if accel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..accel {
                let d = if accel > 1 { SLOW_DELAY.saturating_sub((span * i) / (accel - 1)) } else { SLOW_DELAY };
                pulse(d, &mut ctx);
            }
        }

        // Cruise at FAST
        for _ in 0..cruise { pulse(FAST_DELAY, &mut ctx); }

        // Decel: linearly interpolate FAST -> SLOW
        if decel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..decel {
                let d = if decel > 1 { FAST_DELAY.saturating_add((span * i) / (decel - 1)) } else { SLOW_DELAY };
                pulse(d, &mut ctx);
            }
        }

        // Transition to next state once motion completes
        let _ = wait_open::spawn();
    }

    // Wait for button event while door is open; on event, spawn `closing` and exit
    #[task(shared = [button_event, usb_dev, serial])]
    async fn wait_open(mut ctx: wait_open::Context) {
        loop {
            let mut got = false;
            ctx.shared.button_event.lock(|f| { if *f { *f = false; got = true; } });

            if got {
                let _ = log::spawn(b"-> closing\r\n");
                let _ = closing::spawn();
                return;
            }

            asm::delay(100_000); // ~1ms at 100 MHz; adjust as needed
        }
    }

    // Close by stepping `travel_step_count` steps with the same trapezoidal profile as `opening`,
    // but with direction LOW, then transition to `wait_closed`.
    #[task(shared = [cfg, dir_pin, step_pin, usb_dev, serial])]
    async fn closing(mut ctx: closing::Context) {
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);
        if total_steps == 0 {
            let _ = wait_closed::spawn();
            return;
        }

        // Log start
        let _ = log::spawn(b"-> closing\r\n");

        const SLOW_DELAY: u32 = 20_000;
        const FAST_DELAY: u32 = 3_000;

        let mut accel = core::cmp::max(1, total_steps / 10);
        let mut decel = accel;
        if accel * 2 > total_steps { accel = total_steps / 2; decel = total_steps - accel; }
        let cruise = total_steps.saturating_sub(accel + decel);

        ctx.shared.dir_pin.lock(|dir| { dir.set_low(); });

        let mut steps_done: u32 = 0;
        let mut next_pct: u32 = 10;
        let marks: [&[u8]; 9] = [
            b"10%\r\n", b"20%\r\n", b"30%\r\n", b"40%\r\n", b"50%\r\n",
            b"60%\r\n", b"70%\r\n", b"80%\r\n", b"90%\r\n",
        ];
        let mut mark_idx = 0usize;

        let mut pulse = |delay: u32, ctx: &mut closing::Context| {
            ctx.shared.step_pin.lock(|step| {
                step.set_high();
                asm::delay(delay);
                step.set_low();
                asm::delay(delay);
            });
            steps_done = steps_done.saturating_add(1);
            if mark_idx < marks.len() && steps_done.saturating_mul(100) / total_steps >= next_pct {
                let _ = log::spawn(marks[mark_idx]);
                next_pct += 10;
                mark_idx += 1;
            }
        };

        if accel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..accel {
                let d = if accel > 1 { SLOW_DELAY.saturating_sub((span * i) / (accel - 1)) } else { SLOW_DELAY };
                pulse(d, &mut ctx);
            }
        }
        for _ in 0..cruise { pulse(FAST_DELAY, &mut ctx); }
        if decel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..decel {
                let d = if decel > 1 { FAST_DELAY.saturating_add((span * i) / (decel - 1)) } else { SLOW_DELAY };
                pulse(d, &mut ctx);
            }
        }

        let _ = wait_closed::spawn();
    }
}