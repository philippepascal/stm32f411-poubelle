#![no_std]
#![no_main]

use hal::{
    gpio::{
        Edge, ExtiPin, Input, Output, PushPull, gpioa::PA0, gpioa::PA1, gpioa::PA2, gpioa::PA3, gpioa::PA6, gpioa::PA7, gpioa::PA15, gpioc::PC13,
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
        in_motion: bool,
        door_is_fully_closed: bool,
        door_is_fully_open: bool,
        manual_opening: bool,
        manual_closing: bool,
        opening_direction: bool, // true: HIGH is open, false: LOW is open
        fully_closed_switch_debouncing_flag: bool,
        fully_closed_last_switch_state: bool,
        fully_open_switch_debouncing_flag: bool,
        fully_open_last_switch_state: bool,
    }

    // Resources local to specific tasks (not shared)
    #[local]
    struct Local {
        button_pin: PA7<Input>,
        fault_pin: PA15<Input>,
        closed_position_switch: PA2<Input>,
        open_position_switch: PA3<Input>,
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

        // DRV8825 FAULT (active-low) on PA15
        let mut fault_pin = gpioa.pa15.into_pull_up_input();

        // Position switches
        let mut closed_position_switch = gpioa.pa2.into_pull_up_input(); // PA2: closed-limit / manual opening
        let mut open_position_switch = gpioa.pa3.into_pull_up_input();   // PA3: open-limit / manual closing

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

        // Configure EXTI on PA15 for FAULT (lines 10..15 use EXTI15_10 interrupt)
        fault_pin.make_interrupt_source(&mut syscfg);
        fault_pin.trigger_on_edge(&mut exti, Edge::Falling);
        fault_pin.enable_interrupt(&mut exti);

        // Configure EXTI on PA2 and PA3 (separate vectors EXTI2 and EXTI3)
        closed_position_switch.make_interrupt_source(&mut syscfg);
        closed_position_switch.trigger_on_edge(&mut exti, Edge::RisingFalling);
        closed_position_switch.enable_interrupt(&mut exti);

        open_position_switch.make_interrupt_source(&mut syscfg);
        open_position_switch.trigger_on_edge(&mut exti, Edge::RisingFalling);
        open_position_switch.enable_interrupt(&mut exti);

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
                in_motion: false,
                door_is_fully_closed: false,
                door_is_fully_open: false,
                manual_opening: false,
                manual_closing: false,
                opening_direction: false, // default
                fully_closed_switch_debouncing_flag: false,
                fully_closed_last_switch_state: false,
                fully_open_switch_debouncing_flag: false,
                fully_open_last_switch_state: false,
            },
            Local { button_pin, fault_pin, closed_position_switch, open_position_switch },
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

    // button trigger, replace with motion detector later
    #[task(binds = EXTI9_5, local = [button_pin], shared = [button_event, in_motion])]
    fn exti_irq(ctx: exti_irq::Context) {
        // Clear the line's pending flag first
        ctx.local.button_pin.clear_interrupt_pending_bit();

        // If we're currently moving, ignore button events
        (ctx.shared.in_motion, ctx.shared.button_event).lock(|moving, evt| {
            if !*moving {
                *evt = true;
            }
        });
    }

    // EXTI2: PA2 (closed-limit / manual opening)
    #[task(binds = EXTI2, local = [closed_position_switch], shared = [fully_closed_switch_debouncing_flag, fully_closed_last_switch_state])]
    fn exti_pa2(mut ctx: exti_pa2::Context) {
        // Clear pending first
        ctx.local.closed_position_switch.clear_interrupt_pending_bit();

        ctx.shared.fully_closed_last_switch_state.lock(|p| {
            *p = ctx.local.closed_position_switch.is_low(); // inverted logic: LOW = switch closed
        });

        ctx.shared.fully_closed_switch_debouncing_flag.lock(|f| {
            // Set debouncing flag
            if !*f {
                *f = true;
                let _ = debounce_closed_position_switch::spawn();
            }
        });
    }

    #[task(shared = [in_motion,door_is_fully_closed, manual_opening, fully_closed_switch_debouncing_flag, fully_closed_last_switch_state])]
    async fn debounce_closed_position_switch(mut ctx: debounce_closed_position_switch::Context) {
        // Wait 50ms
        Mono::delay(50_u32.millis()).await;
        // let _ = log::spawn(b"-> debounce closed switch\r\n");
        // Mono::delay(50_u32.millis()).await;

        // If pin is high → fully closed; if low → not fully closed and/or manual opening
        let level_high = ctx.shared.fully_closed_last_switch_state.lock(|p| *p);
        if level_high {
            ctx.shared.door_is_fully_closed.lock(|f| *f = true);
        } else {
            ctx.shared.door_is_fully_closed.lock(|f| *f = false);
            (ctx.shared.in_motion, ctx.shared.manual_opening).lock(|im, mo| {
                *mo = !*im;
            });
        }
        // Clear debouncing flag
        ctx.shared.fully_closed_switch_debouncing_flag.lock(|f| *f = false);
    }

    // EXTI3: PA3 (open-limit / manual closing)
    #[task(binds = EXTI3, local = [open_position_switch], shared = [fully_open_switch_debouncing_flag, fully_open_last_switch_state])]
    fn exti_pa3(mut ctx: exti_pa3::Context) {
        // Clear pending first
        ctx.local.open_position_switch.clear_interrupt_pending_bit();

        ctx.shared.fully_open_last_switch_state.lock(|p| {
            *p = ctx.local.open_position_switch.is_low(); // inverted logic: LOW = switch closed
        });

        ctx.shared.fully_open_switch_debouncing_flag.lock(|f| {
            // Set debouncing flag
            if !*f {
                *f = true;
                let _ = debounce_open_position_switch::spawn();
            }
        });
    }

    #[task(shared = [in_motion,door_is_fully_open, manual_closing, fully_open_switch_debouncing_flag, fully_open_last_switch_state])]
    async fn debounce_open_position_switch(mut ctx: debounce_open_position_switch::Context) {
        // Wait 50ms
        Mono::delay(50_u32.millis()).await;
        // let _ = log::spawn(b"-> debounce open switch\r\n");
        // Mono::delay(50_u32.millis()).await;

        // If pin is high → fully open; if low → not fully open and/or manual closing
        let level_high = ctx.shared.fully_open_last_switch_state.lock(|p| *p);
        if level_high {
            ctx.shared.door_is_fully_open.lock(|f| *f = true);
        } else {
            ctx.shared.door_is_fully_open.lock(|f| *f = false);
            (ctx.shared.in_motion, ctx.shared.manual_closing).lock(|im, mc| {
                *mc = !*im;
            });
        }
        // Clear debouncing flag
        ctx.shared.fully_open_switch_debouncing_flag.lock(|f| *f = false);
    }
    
    // FAULT handler: log the event for now. 
    // Later: disable driver outputs, stop motion, etc.
    // Note: EXTI lines 10..15 share the same interrupt vector.
    // Make sure only to bind this if no other pins on those lines are used.
    // If multiple pins on those lines are used, you'll need to check
    // which one triggered the interrupt.
    // This doesn't seem to detect blockage... not sure it's worth doing much with it
    #[task(binds = EXTI15_10, local = [fault_pin])]
    fn exti_fault(ctx: exti_fault::Context) {
        // Clear pending first to avoid immediate retrigger
        ctx.local.fault_pin.clear_interrupt_pending_bit();
        let _ = log::spawn(b"-> FAULT asserted\r\n");
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

    // Calibrate: probe both directions and measure travel
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, door_is_fully_closed, door_is_fully_open, manual_opening, manual_closing, opening_direction])]
    async fn calibrate(mut ctx: calibrate::Context) {
        // 2.1 wait 10 seconds to allow users to clear the area
        let _ = log::spawn(b"-> calibrating: waiting 10s\r\n");
        Mono::delay(10_u32.secs()).await;

        // Wake driver for calibration moves
        ctx.shared.sleep_pin.lock(|s| { let _ = s.set_high(); });

        // 2.2 set the direction on the controller to HIGH
        ctx.shared.dir_pin.lock(|dir| { dir.set_high(); });

        // Helper: perform one slow step (very conservative timing)
        let slow_step = |step_pin: &mut PA1<Output<PushPull>>| {
            step_pin.set_high();
            asm::delay(150_000); // ~75us (depends on sysclk); adjust as needed
            step_pin.set_low();
            asm::delay(150_000);
        };

        // 2.3 move slowly until either fully_closed or fully_open is set
        let _ = log::spawn(b"-> calibrating: probing HIGH direction\r\n");
        // `high_is_open` = true if DIR=HIGH leads to fully_open, false if it leads to fully_closed
        let high_is_open: bool = 'probe_high: loop {
            // Check flags
            let fc = ctx.shared.door_is_fully_closed.lock(|p| *p);
            let fo = ctx.shared.door_is_fully_open.lock(|p| *p);
            if fc { break 'probe_high false; }
            if fo { break 'probe_high true; }

            // Step once
            ctx.shared.step_pin.lock(|step| slow_step(step));
            Mono::delay(5_u32.millis()).await; // small delay between steps
        };

        // 2.4 decide opening_direction based on result of HIGH probe
        if high_is_open {
            // HIGH moved us to fully_open ⇒ opening is HIGH
            ctx.shared.opening_direction.lock(|d| *d = true);
            let _ = log::spawn(b"-> calibrating: opening_direction = HIGH\r\n");
        } else {
            // HIGH moved us to fully_closed ⇒ opening is LOW
            ctx.shared.opening_direction.lock(|d| *d = false);
            let _ = log::spawn(b"-> calibrating: opening_direction = LOW\r\n");
        }

        // 2.5 set direction to LOW and move slowly while counting steps until either end-stop is hit
        ctx.shared.dir_pin.lock(|dir| { dir.set_low(); });

        Mono::delay(1_u32.secs()).await;

        let _ = log::spawn(b"-> calibrating: measuring travel (LOW)\r\n");
        let mut steps: u32 = 0;
        'measure: loop {
            // Check end stops
            if high_is_open && ctx.shared.door_is_fully_closed.lock(|p| *p) {
                // HIGH is open, LOW is closed, and we're fully closed
                break 'measure;
            }
            if !high_is_open && ctx.shared.door_is_fully_open.lock(|p| *p) {
                // HIGH is closed, LOW is open, and we're fully open
                break 'measure;
            }

            // step and count
            ctx.shared.step_pin.lock(|step| slow_step(step));
            steps = steps.saturating_add(1);
            Mono::delay(5_u32.millis()).await; // small delay between steps

            // Optionally: put a hard cap to avoid endless loops if switches are miswired
            if steps > 2_000_000 { let _ = log::spawn(b"-> calibrating: aborting, no end-stop\r\n"); break 'measure; }
        }

        Mono::delay(1_u32.secs()).await;
        // save cfg.travel_step_count
        ctx.shared.cfg.lock(|c| { c.travel_step_count = steps as u32; });
        let _ = log::spawn(b"-> calibrating: travel_step_count set\r\n");
        Mono::delay(1_u32.secs()).await;

        // 2.6 transition based on current end-stop
        let fc2 = ctx.shared.door_is_fully_closed.lock(|p| *p);
        let fo2 = ctx.shared.door_is_fully_open.lock(|p| *p);
        let (fc2, fo2) = (fc2, fo2);

        // Put driver back to sleep after calibration moves
        ctx.shared.sleep_pin.lock(|s| { let _ = s.set_low(); });

        if fo2 {
            let _ = closing::spawn();
        } else if fc2 {
            let _ = wait_closed::spawn();
        } else {
            // If neither asserted, default to wait_closed (safe)
            let _ = wait_closed::spawn();
        }
    }

    // Open by stepping `travel_step_count` steps with a trapezoidal profile:
    // 10% accel (slow -> fast), 80% cruise (fast), 10% decel (fast -> slow).
    // Uses busy-wait delays (asm::delay cycles) for simplicity.
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, in_motion, usb_dev, serial, opening_direction, door_is_fully_open])]
    async fn opening(mut ctx: opening::Context) {
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);
        ctx.shared.in_motion.lock(|m| *m = true);

        move_trapezoid(
            || {
                ctx.shared.dir_pin.lock(|dir| {
                    let is_opening_high = ctx.shared.opening_direction.lock(|d| *d);
                    if is_opening_high { dir.set_high(); } else { dir.set_low(); }
                });
            },
            |delay| {
                ctx.shared.step_pin.lock(|step| {
                    step.set_high();
                    asm::delay(delay);
                    step.set_low();
                    asm::delay(delay);
                });
            },
            || { ctx.shared.door_is_fully_open.lock(|f| *f) },
            || { let _ = wait_open::spawn(); },
            |on| { ctx.shared.sleep_pin.lock(|s| { let _ = if on { s.set_high() } else { s.set_low() }; }); },
            total_steps,
            b"-> opening\r\n",
            b"-> opened\r\n",
        ).await;
        ctx.shared.in_motion.lock(|m| *m = false);
    }

    // Close by stepping `travel_step_count` steps with the same trapezoidal profile as `opening`,
    // but with direction LOW, then transition to `wait_closed`.
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, in_motion, usb_dev, serial, opening_direction, door_is_fully_closed])]
    async fn closing(mut ctx: closing::Context) {
        let total_steps: u32 = ctx.shared.cfg.lock(|c| c.travel_step_count as u32);
        ctx.shared.in_motion.lock(|m| *m = true);

        move_trapezoid(
            || {
                ctx.shared.dir_pin.lock(|dir| {
                    let is_opening_high = ctx.shared.opening_direction.lock(|d| *d);
                    if is_opening_high { dir.set_low(); } else { dir.set_high(); }
                });
            },
            |delay| {
                ctx.shared.step_pin.lock(|step| {
                    step.set_high();
                    asm::delay(delay);
                    step.set_low();
                    asm::delay(delay);
                });
            },
            || { ctx.shared.door_is_fully_closed.lock(|f| *f) },
            || { let _ = wait_closed::spawn(); },
            |on| { ctx.shared.sleep_pin.lock(|s| { let _ = if on { s.set_high() } else { s.set_low() }; }); },
            total_steps,
            b"-> closing\r\n",
            b"-> closed\r\n",
        ).await;
        ctx.shared.in_motion.lock(|m| *m = false);
    }

    #[task(shared = [button_event, manual_closing, usb_dev, serial])]
    async fn wait_open(mut ctx: wait_open::Context) {
        wait_state!(
            ctx,
            closing,
            manual = manual_closing,
            b"-> transitioned to wait open\r\n",
            b"-> wait open\r\n",
            b"-> transitioned out of wait open\r\n"
        );
    }

    #[task(shared = [button_event, manual_opening, usb_dev, serial])]
    async fn wait_closed(mut ctx: wait_closed::Context) {
        wait_state!(
            ctx,
            opening,
            manual = manual_opening,
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
        ($ctx:ident, $next:ident, manual = $manual_flag:ident, $enter:expr, $beat:expr, $exit:expr $(,)?) => {{
            wait_common(
                || {
                    let mut got = false;
                    $ctx.shared.button_event.lock(|f| {
                        if *f {
                            *f = false;
                            got = true;
                        }
                    });
                    if got { true }
                    else {
                        $ctx.shared.$manual_flag.lock(|f| {
                            if *f {
                                *f = false;
                                got = true;
                            }
                        });
                        got
                    }
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
    async fn move_trapezoid<SetDir, Pulse, ShouldAbort, Next, SetSleep>(
        mut set_direction: SetDir,
        mut pulse: Pulse,
        mut should_abort: ShouldAbort,
        mut spawn_next: Next,
        mut set_sleep: SetSleep,
        total_steps: u32,
        start_label: &'static [u8],
        done_label: &'static [u8],
    )
    where
        SetDir: FnMut(),
        Pulse: FnMut(u32),
        ShouldAbort: FnMut() -> bool,
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

        let mut aborted = false;

        // Accel: SLOW -> FAST
        if accel > 0 {
            let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
            for i in 0..accel {
                if should_abort() { aborted = true; break; }
                let d = if accel > 1 {
                    SLOW_DELAY.saturating_sub((span * i) / (accel - 1))
                } else {
                    SLOW_DELAY
                };
                pulse(d);
                Mono::delay(1_u32.millis()).await; // small delay between steps
            }
        }

        // Cruise at FAST
        if !aborted {
            for _ in 0..cruise {
                if should_abort() { aborted = true; break; }
                pulse(FAST_DELAY);
                Mono::delay(1_u32.millis()).await; // small delay between steps
            }
        }

        // Decel: FAST -> SLOW
        if !aborted {
            if decel > 0 {
                let span = SLOW_DELAY.saturating_sub(FAST_DELAY);
                for i in 0..decel {
                    if should_abort() { break; }
                    let d = if decel > 1 {
                        FAST_DELAY.saturating_add((span * i) / (decel - 1))
                    } else {
                        SLOW_DELAY
                    };
                    pulse(d);
                    Mono::delay(1_u32.millis()).await; // small delay between steps
                }
            }
        }

        if aborted { let _ = log::spawn(b"-> limit hit\r\n"); }
        let _ = log::spawn(done_label);
        set_sleep(false); // sleep driver
        spawn_next();
    }
}
