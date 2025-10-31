#![no_std]
#![no_main]

use hal::{
    gpio::{
        Edge, ExtiPin, Input, Output, PushPull, gpioa::PA0, gpioa::PA1, gpioa::PA6, gpioa::PA7, gpioc::PC13,
    },
    otg_fs, pac,
    prelude::*,
};
use rtic::app;
use stm32f4xx_hal as hal;

use cortex_m::asm;
use panic_halt as _;
use stm32f4xx_hal::otg_fs::UsbBus as HalUsbBus;
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

mod flash_store;
use flash_store::{Config, load_config, save_config_blocking};

// --- Type aliases for USB stack
type HalUsb = HalUsbBus<otg_fs::USB>;
type UsbDev<'a> = usb_device::device::UsbDevice<'a, HalUsb>;
type Serial<'a> = usbd_serial::SerialPort<'a, HalUsb>;

use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1000);

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
)]
mod app {

    use super::*;
    use fugit::ExtU32;
    use rtic_monotonics::Monotonic; // enables 10_u32.millis(), 500_u32.millis()

    // Resources shared across tasks (RTIC locks these automatically)
    #[shared]
    struct Shared {
        usb_dev: UsbDev<'static>,
        serial: SerialPort<'static, HalUsb>,
        dir_pin: PA0<Output<PushPull>>,
        step_pin: PA1<Output<PushPull>>,
        led_pin: PC13<Output<PushPull>>,
        sleep_pin: PA6<Output<PushPull>>, // DRV8825 SLEEP (HIGH = awake, LOW = sleep)
        cfg: Config,
        button_event: bool,
        last_button_ms: u32,
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
        let core = ctx.core;
        let dp: pac::Peripherals = ctx.device;

        // RCC / clocks
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(100.MHz()).require_pll48clk().freeze();

        // Initialize the systick interrupt & obtain the token to prove that we did
        Mono::start(core.SYST, clocks.sysclk().to_Hz());

        // GPIO
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();
        let dir_pin = gpioa.pa0.into_push_pull_output();
        let step_pin = gpioa.pa1.into_push_pull_output();
        let mut button_pin = gpioa.pa7.into_pull_up_input();
        let mut led_pin = gpioc.pc13.into_push_pull_output();
        // Many F411 "Black Pill" boards have the LED on PC13 active-low; start LED off.
        let _ = led_pin.set_high();

        let mut sleep_pin = gpioa.pa6.into_push_pull_output();
        // Default driver to sleep until we intentionally wake for motion
        let _ = sleep_pin.set_low();

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

        // Kick off the state machine
        let _ = heartbeat::spawn();
        let _ = start_door::spawn();

        (
            Shared {
                usb_dev,
                serial,
                dir_pin,
                step_pin,
                led_pin,
                sleep_pin,
                cfg,
                button_event: false,
                last_button_ms: 0,
            },
            Local { button_pin },
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

    #[task(binds = EXTI9_5, local = [button_pin], shared = [button_event, last_button_ms])]
    fn exti_irq(ctx: exti_irq::Context) {
        // Clear the line's pending flag first
        ctx.local.button_pin.clear_interrupt_pending_bit();

        // Simple debounce: only accept one event per 50 ms window
        let now_ms: u32 = Mono::now().ticks(); // 1 kHz monotonic → ticks are milliseconds?
        let mut accept = false;
        (ctx.shared.last_button_ms, ctx.shared.button_event).lock(|last_ms, evt| {
            let dt = now_ms.wrapping_sub(*last_ms);
            if dt >= 500 {
                // 500ms? debounce
                *last_ms = now_ms;
                *evt = true;
                accept = true;
            }
        });
    }

    #[task(shared = [led_pin, usb_dev, serial])]
    async fn heartbeat(mut ctx: heartbeat::Context) {
        let mut led_on = false;
        let mut divider = 0;
        loop {
            // sleep/yield to the RTIC executor
            Mono::delay(500_u32.millis()).await;

            divider += 1;

            // toggle LED (PC13 is typically active-low on BlackPill)
            ctx.shared.led_pin.lock(|led| {
                if led_on {
                    let _ = led.set_high();
                } else {
                    let _ = led.set_low();
                }
                led_on = !led_on;
            });

            // keep USB serviced and (optionally) log a blink line
            if divider % 10 == 0 {
                let _ = log::spawn(b"-> heartbeat\r\n");
                divider = 0;
            }
        }
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

    // Open by stepping `travel_step_count` steps with a trapezoidal profile:
    // 10% accel (slow -> fast), 80% cruise (fast), 10% decel (fast -> slow).
    // Uses busy-wait delays (asm::delay cycles) for simplicity.
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, usb_dev, serial])]
    async fn opening(mut ctx: opening::Context) {
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);

        move_trapezoid(
            || { ctx.shared.dir_pin.lock(|dir| { dir.set_high(); }); },
            |delay| {
                ctx.shared.step_pin.lock(|step| {
                    step.set_high();
                    asm::delay(delay);
                    step.set_low();
                    asm::delay(delay);
                });
            },
            || { let _ = wait_open::spawn(); },
            |on| { ctx.shared.sleep_pin.lock(|s| { let _ = if on { s.set_high() } else { s.set_low() }; }); },
            total_steps,
            b"-> opening\r\n",
            b"-> opened\r\n",
        ).await;
    }

    // Close by stepping `travel_step_count` steps with the same trapezoidal profile as `opening`,
    // but with direction LOW, then transition to `wait_closed`.
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, usb_dev, serial])]
    async fn closing(mut ctx: closing::Context) {
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);

        move_trapezoid(
            || { ctx.shared.dir_pin.lock(|dir| { dir.set_low(); }); },
            |delay| {
                ctx.shared.step_pin.lock(|step| {
                    step.set_high();
                    asm::delay(delay);
                    step.set_low();
                    asm::delay(delay);
                });
            },
            || { let _ = wait_closed::spawn(); },
            |on| { ctx.shared.sleep_pin.lock(|s| { let _ = if on { s.set_high() } else { s.set_low() }; }); },
            total_steps,
            b"-> closing\r\n",
            b"-> closed\r\n",
        ).await;
    }

    #[task(shared = [button_event, usb_dev, serial])]
    async fn wait_open(mut ctx: wait_open::Context) {
        wait_state!(
            ctx,
            closing,
            b"-> transitioned to wait open\r\n",
            b"-> wait open\r\n",
            b"-> transitioned out of wait open\r\n"
        );
    }

    #[task(shared = [button_event, usb_dev, serial])]
    async fn wait_closed(mut ctx: wait_closed::Context) {
        wait_state!(
            ctx,
            opening,
            b"-> transitioned to wait closed\r\n",
            b"-> wait closed\r\n",
            b"-> transitioned out of wait closed\r\n"
        );
    }

    // shared async helper 
    async fn wait_common(
        mut take_event: impl FnMut() -> bool,
        mut spawn_next: impl FnMut(),
        enter: &'static [u8],
        beat: &'static [u8],
        exit: &'static [u8],
    ) {
        let _ = log::spawn(enter);
        let mut ms = 0u32;
        loop {
            Mono::delay(10_u32.millis()).await;
            ms = ms.wrapping_add(10);

            if take_event() {
                spawn_next();
                let _ = log::spawn(exit);
                return;
            }
            if ms >= 1000 {
                ms = 0;
                let _ = log::spawn(beat);
            }
        }
    }

    // macro that *uses* the helper inside a task
    macro_rules! wait_state {
        ($ctx:ident, $next:ident, $enter:expr, $beat:expr, $exit:expr $(,)?) => {{
            wait_common(
                || {
                    let mut got = false;
                    $ctx.shared.button_event.lock(|f| {
                        if *f {
                            *f = false;
                            got = true;
                        }
                    });
                    got
                },
                || {
                    let _ = $next::spawn();
                },
                $enter,
                $beat,
                $exit,
            )
            .await;
        }};
    }


    // Reusable trapezoid motion helper used by `opening` and `closing`
    async fn move_trapezoid<SetDir, Pulse, Next, SetSleep>(
        mut set_direction: SetDir,
        mut pulse: Pulse,
        mut spawn_next: Next,
        mut set_sleep: SetSleep,
        total_steps: u32,
        start_label: &'static [u8],
        done_label: &'static [u8],
    )
    where
        SetDir: FnMut(),
        Pulse: FnMut(u32),
        Next: FnMut(),
        SetSleep: FnMut(bool),
    {
        if total_steps == 0 {
            // No motion needed; transition immediately
            spawn_next();
            return;
        }

        set_sleep(true); // wake driver

        let _ = log::spawn(start_label);
        Mono::delay(10_u32.millis()).await; // let USB breathe

        // Timing constants (tune for your mechanics and sysclk)
        const SLOW_DELAY: u32 = 120_000; // ~60 µs half-period
        const FAST_DELAY: u32 = 25_000;  // ~30 µs half-period

        // Phase partitioning: 10% accel, 80% cruise, 10% decel
        let mut accel = core::cmp::max(1, total_steps / 10);
        let mut decel = accel;
        if accel * 2 > total_steps {
            accel = total_steps / 2;
            decel = total_steps - accel;
        }
        let cruise = total_steps.saturating_sub(accel + decel);

        // Direction once up-front
        set_direction();

        // Accel: SLOW -> FAST
        if accel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..accel {
                let d = if accel > 1 {
                    SLOW_DELAY.saturating_sub((span * i) / (accel - 1))
                } else {
                    SLOW_DELAY
                };
                pulse(d);
            }
        }

        // Cruise at FAST
        for _ in 0..cruise {
            pulse(FAST_DELAY);
        }

        // Decel: FAST -> SLOW
        if decel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..decel {
                let d = if decel > 1 {
                    FAST_DELAY.saturating_add((span * i) / (decel - 1))
                } else {
                    SLOW_DELAY
                };
                pulse(d);
            }
        }

        let _ = log::spawn(done_label);
        set_sleep(false); // sleep driver
        spawn_next();
    }
}
