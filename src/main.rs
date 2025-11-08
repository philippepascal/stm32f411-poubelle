#![no_std]
#![no_main]

use hal::{
    gpio::{
        Edge, ExtiPin, Input, Output, PushPull,
        gpioa::PA0, gpioa::PA1, gpioa::PA2, gpioa::PA3, gpioa::PA6, gpioa::PA7, gpioa::PA15,
        gpiob::PB0, gpiob::PB1, gpiob::PB5, gpiob::PB6,
        gpioc::PC13,
    },
    otg_fs, pac,
    prelude::*,
    i2c::I2c,
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

use vl53l1x_uld::{VL53L1X, IOVoltage, DistanceMode, RangeStatus};

// --- Type aliases for USB stack
type HalUsb = HalUsbBus<otg_fs::USB>;
type UsbDev<'a> = usb_device::device::UsbDevice<'a, HalUsb>;
type Serial<'a> = usbd_serial::SerialPort<'a, HalUsb>;

use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1000);

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum FootState {
    Idle,
    Inside,              // entered, waiting for exit
    IgnoredLongPresence, // stayed too long; ignore until exit
    Cooldown,            // short ignore after a valid transient
}

const ENTER_MM: u16      = 80;   // “inside” threshold (tune 60–120)
const WINDOW_MIN_MS: u32 = 200;   // min time inside to count as motion (de-glitch)
const WINDOW_MAX_MS: u32 = 2000;  // max time inside before we ignore lingerers
const COOLDOWN_MS:   u32 = 1000;  // suppress re-triggers after a valid transient

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
)]
mod app {
    use super::*;
    use fugit::ExtU32;
    
    use rtic_monotonics::{Monotonic, TimerQueueBasedMonotonic};
    use vl53l1x_uld::{Polarity, threshold::{Threshold, Window}}; // enables 10_u32.millis(), 500_u32.millis()

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
        // I2C1 for VL53L1X
        i2c1: I2c<pac::I2C1>,
        sensor1_present: bool,
        sensor2_present: bool,
        // VL53L1X GPIO1 levels and periodic loop counter
        s1_gpio_high: bool,
        s2_gpio_high: bool,
        s1_ranging: bool,
        s2_ranging: bool,
        s1_state: FootState,
        s1_enter_time: Option<<Mono as TimerQueueBasedMonotonic>::Instant>,
        s2_state: FootState,
        s2_enter_time: Option<<Mono as TimerQueueBasedMonotonic>::Instant>,

    }

    // Resources local to specific tasks (not shared)
    #[local]
    struct Local {
        button_pin: PA7<Input>,
        fault_pin: PA15<Input>,
        closed_position_switch: PA2<Input>,
        open_position_switch: PA3<Input>,
        // VL53L1X control/interrupt pins
        xshut1: PB0<Output<PushPull>>, // Sensor 1 XSHUT
        xshut2: PB1<Output<PushPull>>, // Sensor 2 XSHUT
        int1: PB5<Input>,              // Sensor 1 GPIO1
        int2: PB6<Input>,              // Sensor 2 GPIO1
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
        let gpiob = dp.GPIOB.split();
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

        // I2C1 on PB8 (SCL) / PB9 (SDA)
        let scl = gpiob.pb8.into_alternate::<4>().set_open_drain();
        let sda = gpiob.pb9.into_alternate::<4>().set_open_drain();
        let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

        // VL53L1X control pins (keep sensors held in reset initially)
        let mut xshut1 = gpiob.pb0.into_push_pull_output();
        let mut xshut2 = gpiob.pb1.into_push_pull_output();
        let _ = xshut1.set_low();
        let _ = xshut2.set_low();

        // VL53L1X GPIO1 interrupt pins
        let mut int1 = gpiob.pb5.into_pull_up_input();
        let mut int2 = gpiob.pb6.into_pull_up_input();

        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI9_5); }

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

        // Route PB5/PB6 to EXTI and enable interrupts (EXTI9_5 vector)
        int1.make_interrupt_source(&mut syscfg);
        int1.trigger_on_edge(&mut exti, Edge::Rising);
        int1.enable_interrupt(&mut exti);

        int2.make_interrupt_source(&mut syscfg);
        int2.trigger_on_edge(&mut exti, Edge::Rising);
        int2.enable_interrupt(&mut exti);

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
                i2c1,
                sensor1_present: false,
                sensor2_present: false,
                s1_gpio_high: false,
                s2_gpio_high: false,
                s1_ranging: false,
                s2_ranging: false,
                s1_state: FootState::Idle,
                s1_enter_time: None,
                s2_state: FootState::Idle,
                s2_enter_time: None,
            },
            Local {
                button_pin,
                fault_pin,
                closed_position_switch,
                open_position_switch,
                xshut1,
                xshut2,
                int1,
                int2,
            },
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

    // PB5 / PB6 (VL53L1X GPIO1) and PA7 (button) share EXTI9_5
    #[task(binds = EXTI9_5, local = [button_pin, int1, int2], shared = [button_event, in_motion])]
    fn exti_irq(ctx: exti_irq::Context) {
        // PB5 / PB6 (VL53L1X GPIO1) and PA7 (button) share EXTI9_5
        // Clear pending on any that fired, and react accordingly.

        // Button on PA7
        if ctx.local.button_pin.check_interrupt() {
            ctx.local.button_pin.clear_interrupt_pending_bit();
            (ctx.shared.in_motion, ctx.shared.button_event).lock(|moving, evt| {
                if !*moving { *evt = true; }
            });
        }

        // Sensor 1 on PB5
        if ctx.local.int1.check_interrupt() {
            ctx.local.int1.clear_interrupt_pending_bit();
            let _ = sensor1_motion::spawn();
        }

        // Sensor 2 on PB6
        if ctx.local.int2.check_interrupt() {
            ctx.local.int2.clear_interrupt_pending_bit();
            let _ = sensor2_motion::spawn();
        }
    }

    #[task(shared = [i2c1, s1_state, s1_enter_time, usb_dev, serial])]
    async fn sensor1_motion(mut ctx: sensor1_motion::Context) {
        // Read one sample & clear IRQ
        let (rs, d) = ctx.shared.i2c1.lock(|i2c| {
            let s = VL53L1X::new(0x2A);
            let rs = s.get_range_status(i2c).unwrap_or(RangeStatus::None);
            let d  = s.get_distance(i2c).unwrap_or(u16::MAX);
            let _  = s.clear_interrupt(i2c);
            (rs, d)
        });
        ctx.shared.serial.lock(|serial| {
            let _ = serial.write(b"S1:In State MAchine Reading: ");
            print_u8(serial, "RS", rs as u8);
            print_u16(serial, "D", d);
            let _ = serial.write(b"\r\n");
        });
        Mono::delay(1_u32.millis()).await;

        // Ignore invalid measurements
        if rs as u8 != 0 {  let _ = log::spawn(b"S1:READ FAILED\r\n"); Mono::delay(1.millis()).await; return;}

        let now = Mono::now();

        // State transition
        let mut to_start_timeout = false;

        let _ = log::spawn(b"S1:starting state machine\r\n");
        Mono::delay(1.millis()).await;

        let foot_state = ctx.shared.s1_state.lock(|st| { *st});
        match foot_state {
            FootState::Idle => {
                let _ = log::spawn(b"S1:Idle!\r\n");
                Mono::delay(1.millis()).await;
                if d < ENTER_MM {
                    let _ = log::spawn(b"S1:changing to Inside\r\n");
                    Mono::delay(1.millis()).await;
                    ctx.shared.s1_state.lock(|st| {
                        *st = FootState::Inside;
                    });
                    // record entry time
                    ctx.shared.s1_enter_time.lock(|t| *t = Some(now));
                    to_start_timeout = true; // arm WINDOW_MAX timer
                }
            }
            FootState::Inside => {
                let _ = log::spawn(b"S1:already inside\r\n");
                Mono::delay(1.millis()).await;
                // we only get GPIO1 on enter; we also need to detect exit:
                // poll distance once more here: if already outside, treat as exit
                // (Alternatively, rely on your periodic range loop to call a small “exit check” task.)
            }
            FootState::IgnoredLongPresence | FootState::Cooldown => {
                let _ = log::spawn(b"S1:Ignored long presence\r\n");
                Mono::delay(1.millis()).await;
                // Do nothing on extra enters while suppressed
            }
        }
        if to_start_timeout {
            // We no longer have `spawn_after` (removed with rtic-monotonics). Arm a small
            // async timer task that sleeps, then spawns the timeout handler.
            let _ = s1_inside_timeout_alarm::spawn();
        }
    }

    // Small helper task: wait WINDOW_MAX, then trigger inside-timeout evaluation.
    #[task]
    async fn s1_inside_timeout_alarm(_ctx: s1_inside_timeout_alarm::Context) {
        let _ = log::spawn(b"S1:ENTER\r\n");
        Mono::delay(WINDOW_MAX_MS.millis()).await;
        let _ = s1_inside_timeout::spawn();
    }

    // Fired WINDOW_MAX after entering; decides whether we ignore long presence
    #[task(shared = [i2c1, s1_state, s1_enter_time, usb_dev, serial])]
    async fn s1_inside_timeout(mut ctx: s1_inside_timeout::Context) {
        // Re-check distance (are we still inside?)
        let (rs, d) = ctx.shared.i2c1.lock(|i2c| {
            let s = VL53L1X::new(0x2A);
            let rs = s.get_range_status(i2c).unwrap_or(RangeStatus::None);
            let d  = s.get_distance(i2c).unwrap_or(u16::MAX);
            (rs, d)
        });

        if rs as u8 != 0 { return; }

        // If still inside after WINDOW_MAX → ignore until exit
        if d < ENTER_MM {
            ctx.shared.s1_state.lock(|st| *st = FootState::IgnoredLongPresence);
            let _ = log::spawn(b"S1:LONG_PRESENCE\r\n");
            Mono::delay(1_u32.millis()).await;
        }
    }


    #[task]
    async fn sensor2_motion(_ctx: sensor2_motion::Context) { 
        // let _ = log::spawn(b"-> VL53 S2 motion\r\n"); 
        Mono::delay(1_u32.millis()).await;
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
        // 2.1 wait 10 seconds to allow users to clear the area
        let _ = log::spawn(b"-> calibrating: waiting 10s\r\n");
        Mono::delay(10_u32.secs()).await;

        let _ = log::spawn(b"-> starting\r\n");
        Mono::delay(1_u32.millis()).await;
        let _ = vl53_init::spawn();
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

    // Bring up both VL53L1X sensors on PB0/PB1 XSHUT. This is where you'd call into a real driver
    // to change I2C addresses and start continuous ranging. For now, we just power them up staggered.
    #[task(shared = [i2c1, sensor1_present, sensor2_present, s1_ranging, s2_ranging], local = [xshut1, xshut2])]
    async fn vl53_init(mut ctx: vl53_init::Context) {
        let _ = log::spawn(b"-> VL53 init start\r\n");
        Mono::delay(1_u32.millis()).await;

        // Hold both low
        let _ = ctx.local.xshut1.set_low();
        let _ = ctx.local.xshut2.set_low();
        Mono::delay(10_u32.millis()).await;

        // Probe while both sensors are held in reset
        let mut cold = [0u8; 1];
        let cold_ack = ctx.shared.i2c1.lock(|i2c| i2c.read(0x29u8, &mut cold).is_ok());
        if cold_ack { let _ = log::spawn(b"I2C: unexpected ACK @0x29 with XSHUT=LOW\r\n"); }
        else { let _ = log::spawn(b"I2C: no ACK @0x29 with XSHUT=LOW\r\n"); }
        Mono::delay(1_u32.millis()).await;

        // Bring up sensor 1
        let _ = ctx.local.xshut1.set_high();
        Mono::delay(20_u32.millis()).await;
        ctx.shared.sensor1_present.lock(|p| *p = true);

        // I2C probe for default VL53L1X address (0x29) after S1 up
        let mut tmp = [0u8; 1];
        let s1_ack = ctx.shared.i2c1.lock(|i2c| i2c.read(0x29u8, &mut tmp).is_ok());
        if s1_ack {
            let _ = log::spawn(b"I2C: S1 ACK @0x29\r\n");
            Mono::delay(1_u32.millis()).await;
            // Read VL53L1X Model ID (0x010F expected 0xEA)
            let mid_ok = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x29u8, 0x010F).is_ok());
            if mid_ok { let _ = log::spawn(b"I2C: S1 model id ok\r\n"); } else { let _ = log::spawn(b"I2C: S1 model id read fail\r\n"); }
            Mono::delay(1_u32.millis()).await;

            // Change S1 I2C address to 0x2A while S2 is still in reset
            let set_ok = ctx.shared.i2c1.lock(|i2c| i2c_write_u8(i2c, 0x29u8, 0x0001, 0x2Au8).is_ok());
            if set_ok { let _ = log::spawn(b"I2C: S1 addr -> 0x2A\r\n"); } else { let _ = log::spawn(b"I2C: S1 addr change FAIL\r\n"); }
            Mono::delay(5_u32.millis()).await;

            // Confirm S1 now ACKs at 0x2A and no longer at 0x29
            let s1_new_ack = ctx.shared.i2c1.lock(|i2c| i2c.read(0x2Au8, &mut [0]).is_ok());
            if s1_new_ack { let _ = log::spawn(b"I2C: S1 ACK @0x2A\r\n"); } else { let _ = log::spawn(b"I2C: S1 NAK @0x2A\r\n"); }
            Mono::delay(1_u32.millis()).await;
            let s1_old_nak = ctx.shared.i2c1.lock(|i2c| i2c.read(0x29u8, &mut [0]).is_err());
            if s1_old_nak { let _ = log::spawn(b"I2C: S1 no longer @0x29\r\n"); }
            Mono::delay(1_u32.millis()).await;
        } else {
            let _ = log::spawn(b"I2C: S1 NAK @0x29\r\n");
        }
        Mono::delay(1_u32.millis()).await;

        // Bring up sensor 2 with robust boot polling and clearer logs
        let _ = ctx.local.xshut2.set_high();
        Mono::delay(30_u32.millis()).await; // give more time to boot power-on sequencer
        ctx.shared.sensor2_present.lock(|p| *p = true);

        // Probe default address (0x29) now that S2 is up
        let mut tmp2 = [0u8; 1];
        let ack_29 = ctx.shared.i2c1.lock(|i2c| i2c.read(0x29u8, &mut tmp2).is_ok());
        if ack_29 { let _ = log::spawn(b"I2C: S2 ACK @0x29\r\n"); } else { let _ = log::spawn(b"I2C: S2 NAK @0x29\r\n"); }
        Mono::delay(5_u32.millis()).await;

        // Poll boot state (0x0006) up to ~100ms until it reports ready (1)
        let mut boot_ready = false;
        for _ in 0..10u8 {
            let ok = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x29u8, 0x0006).ok() == Some(1));
            if ok { boot_ready = true; break; }
            Mono::delay(10_u32.millis()).await;
        }
        if boot_ready { let _ = log::spawn(b"I2C: S2 boot state ready\r\n"); }
        else { let _ = log::spawn(b"I2C: S2 boot state NOT ready\r\n"); }
        Mono::delay(1_u32.millis()).await;

        // Try model id a couple of times with small delays
        let mut s2_mid_ok = false;
        for _ in 0..3u8 {
            let ok = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x29u8, 0x010F).is_ok());
            if ok { s2_mid_ok = true; break; }
            Mono::delay(5_u32.millis()).await;
        }
        if s2_mid_ok { let _ = log::spawn(b"I2C: S2(0x29) model id ok\r\n"); }
        else { let _ = log::spawn(b"I2C: S2(0x29) model id FAIL\r\n"); }
        Mono::delay(1_u32.millis()).await;

        // Also re-verify S1 model id at its new address
        let s1_mid_ok = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x2Au8, 0x010F).is_ok());
        if s1_mid_ok { let _ = log::spawn(b"I2C: S1(0x2A) model id ok\r\n"); }
        Mono::delay(1_u32.millis()).await;

        // Configure both sensors via the ULD driver (loads defaults, sets timing, starts ranging)
        let (mut s1_ok, mut s2_ok) = (false, false);
        ctx.shared.i2c1.lock(|i2c| {
            // S1 @ 0x2A
            if ctx.shared.sensor1_present.lock(|p| *p) {
                let s1 = VL53L1X::new(0x2A);
                    let _ = s1.init(i2c,IOVoltage::Volt2_8);
                    let _ = s1.set_distance_mode(i2c,DistanceMode::Short);
                    let _ = s1.set_timing_budget_ms(i2c,50);
                    let _ = s1.set_inter_measurement_period_ms(i2c,100);
                    let _ = s1.set_distance_threshold(i2c, Threshold::new(0, ENTER_MM, Window::In));
                    let _ = s1.set_interrupt_polarity(i2c, Polarity::ActiveHigh);
                    let _ = s1.start_ranging(i2c);
                    s1_ok = true;
            }

            // S2 @ 0x29
            if ctx.shared.sensor2_present.lock(|p| *p) {
                let s2 = VL53L1X::new(0x29);
                    let _ = s2.init(i2c,IOVoltage::Volt2_8);
                    let _ = s2.set_distance_mode(i2c,DistanceMode::Short);
                    let _ = s2.set_timing_budget_ms(i2c,50);
                    let _ = s2.set_inter_measurement_period_ms(i2c,100);
                    let _ = s2.set_distance_threshold(i2c, Threshold::new(0, ENTER_MM, Window::In));
                    let _ = s2.set_interrupt_polarity(i2c, Polarity::ActiveHigh);
                    let _ = s2.start_ranging(i2c);
                    s2_ok = true;
            }
        });
        if s1_ok { let _ = log::spawn(b"S1: ranging started\r\n"); Mono::delay(1_u32.millis()).await; }
        if s2_ok { let _ = log::spawn(b"S2: ranging started\r\n"); Mono::delay(1_u32.millis()).await; }
        ctx.shared.s1_ranging.lock(|f| *f = s1_ok);
        ctx.shared.s2_ranging.lock(|f| *f = s2_ok);

        
        // TODO: configure both sensors for continuous ranging + GPIO1 interrupt behavior
        let _ = log::spawn(b"-> VL53 init done\r\n");
        // Start distance ranging + periodic GPIO/I2C checks
        let _ = vl53_range_loop::spawn();
        Mono::delay(1_u32.millis()).await;
        // let _ = vl53_loopcheck::spawn();
        // Mono::delay(1_u32.millis()).await;
    }

    // Calibrate: probe both directions and measure travel
    #[task(shared = [cfg, dir_pin, step_pin, sleep_pin, door_is_fully_closed, door_is_fully_open, manual_opening, manual_closing, opening_direction])]
    async fn calibrate(mut ctx: calibrate::Context) {
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

    // ---- Minimal I2C register helpers for VL53L1X ----
    fn i2c_write_u8<I>(i2c: &mut I, addr: u8, reg: u16, val: u8) -> Result<(), ()>
    where
        I: embedded_hal::blocking::i2c::Write,
    {
        let buf = [(reg >> 8) as u8, (reg & 0xFF) as u8, val];
        i2c.write(addr, &buf).map_err(|_| ())
    }

    fn i2c_read_u8<I>(i2c: &mut I, addr: u8, reg: u16) -> Result<u8, ()>
    where
        I: embedded_hal::blocking::i2c::WriteRead,
    {
        let regs = [(reg >> 8) as u8, (reg & 0xFF) as u8];
        let mut out = [0u8; 1];
        i2c.write_read(addr, &regs, &mut out).map_err(|_| ())?;
        Ok(out[0])
    }
    // fn i2c_write_u16<I>(i2c: &mut I, addr: u8, reg: u16, val: u16) -> Result<(), ()>
    // where
    //     I: embedded_hal::blocking::i2c::Write,
    // {
    //     let buf = [
    //         (reg >> 8) as u8, (reg & 0xFF) as u8,
    //         (val >> 8) as u8, (val & 0xFF) as u8,
    //     ];
    //     i2c.write(addr, &buf).map_err(|_| ())
    // }

    // fn i2c_read_u16<I>(i2c: &mut I, addr: u8, reg: u16) -> Result<u16, ()>
    // where
    //     I: embedded_hal::blocking::i2c::WriteRead,
    // {
    //     let regs = [(reg >> 8) as u8, (reg & 0xFF) as u8];
    //     let mut out = [0u8; 2];
    //     i2c.write_read(addr, &regs, &mut out).map_err(|_| ())?;
    //     Ok(((out[0] as u16) << 8) | (out[1] as u16))
    // }

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

    // Periodic loop-check for VL53L1X GPIO1 pins and I2C bus presence
    #[task(shared = [s1_gpio_high, s2_gpio_high, i2c1])]
    async fn vl53_loopcheck(mut ctx: vl53_loopcheck::Context) {
        let mut elapsed_ms: u32 = 0;
        loop {
            // Sample PB5/PB6 levels via GPIOB IDR to avoid pin ownership issues
            let (s1, s2) = unsafe {
                let idr = (*pac::GPIOB::ptr()).idr().read();
                (idr.idr5().bit_is_set(), idr.idr6().bit_is_set())
            };

            let mut changed = false;
            ctx.shared.s1_gpio_high.lock(|last| { if *last != s1 { *last = s1; changed = true; } });
            ctx.shared.s2_gpio_high.lock(|last| { if *last != s2 { *last = s2; changed = true; } });

            if changed {
                if s1 { let _ = log::spawn(b"VL53 S1 GPIO1=HIGH\r\n"); } else { let _ = log::spawn(b"VL53 S1 GPIO1=LOW\r\n"); }
                Mono::delay(1_u32.millis()).await;
                if s2 { let _ = log::spawn(b"VL53 S2 GPIO1=HIGH\r\n"); } else { let _ = log::spawn(b"VL53 S2 GPIO1=LOW\r\n"); }
                Mono::delay(1_u32.millis()).await;
            }

            Mono::delay(100_u32.millis()).await;
            elapsed_ms = elapsed_ms.saturating_add(100);
            if elapsed_ms >= 1000 {
                elapsed_ms = 0;
                // Log current GPIO1 levels once per second even if stable
                let (s1_now, s2_now) = unsafe {
                    let idr = (*pac::GPIOB::ptr()).idr().read();
                    (idr.idr5().bit_is_set(), idr.idr6().bit_is_set())
                };
                if s1_now { let _ = log::spawn(b"VL53 S1 GPIO1=HIGH\r\n"); } else { let _ = log::spawn(b"VL53 S1 GPIO1=LOW\r\n"); }
                Mono::delay(1_u32.millis()).await;
                if s2_now { let _ = log::spawn(b"VL53 S2 GPIO1=HIGH\r\n"); } else { let _ = log::spawn(b"VL53 S2 GPIO1=LOW\r\n"); }
                Mono::delay(1_u32.millis()).await;
                let mut byte = [0u8;1];
                let s1_ok = ctx.shared.i2c1.lock(|i2c| i2c.read(0x2Au8, &mut byte).is_ok());
                let s2_ok = ctx.shared.i2c1.lock(|i2c| i2c.read(0x29u8, &mut byte).is_ok());
                if s1_ok { let _ = log::spawn(b"I2C ping 0x2A OK\r\n"); } else { let _ = log::spawn(b"I2C ping 0x2A FAIL\r\n"); }
                Mono::delay(1_u32.millis()).await;
                if s2_ok { let _ = log::spawn(b"I2C ping 0x29 OK\r\n"); } else { let _ = log::spawn(b"I2C ping 0x29 FAIL\r\n"); }
                Mono::delay(1_u32.millis()).await;
            }
        }
    }

    // Minimal u32 -> ASCII writer; returns number of bytes written
    fn u32_to_ascii(mut v: u32, out: &mut [u8]) -> usize {
        if out.is_empty() { return 0; }
        if v == 0 { out[0] = b'0'; return 1; }
        let mut tmp = [0u8; 10]; // u32 max 10 digits
        let mut i = 0;
        while v > 0 && i < tmp.len() {
            tmp[i] = b'0' + (v % 10) as u8;
            v /= 10;
            i += 1;
        }
        // reverse into out
        let n = core::cmp::min(i, out.len());
        for j in 0..n { out[j] = tmp[i - 1 - j]; }
        n
    }

    // tiny helpers to print diagnostic triples
    fn print_u8(serial: &mut usbd_serial::SerialPort<'static, HalUsb>, label: &str, v: u8) {
        let _ = serial.write(label.as_bytes());
        let _ = serial.write(b"=");
        let mut buf = [0u8; 8];
        let n = u32_to_ascii(v as u32, &mut buf);
        let _ = serial.write(&buf[..n]);
        let _ = serial.write(b" ");
    }
    fn print_u16(serial: &mut usbd_serial::SerialPort<'static, HalUsb>, label: &str, v: u16) {
        let _ = serial.write(label.as_bytes());
        let _ = serial.write(b"=");
        let mut buf = [0u8; 8];
        let n = u32_to_ascii(v as u32, &mut buf);
        let _ = serial.write(&buf[..n]);
        let _ = serial.write(b" ");
    }

    // Start continuous ranging and read distances.
    // Key registers (ST):
    //   SYSTEM__MODE_START (0x0087)             -> write 0x40 to start continuous ranging
    //   SYSTEM__INTERRUPT_CLEAR (0x0086)        -> write 0x01 after reading to clear "data ready"
    //   RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 (0x0096) -> 16-bit distance in mm
    //   RESULT__RANGE_STATUS (0x0089)           -> optional status
    #[task(shared = [i2c1, sensor1_present, sensor2_present, serial, s1_ranging, s2_ranging, s1_state, s1_enter_time, in_motion, button_event])]
    async fn vl53_range_loop(mut ctx: vl53_range_loop::Context) {
        // track whether we've started continuous ranging per sensor
        let mut s1_started = false;
        let mut s2_started = false;

        // 1-in-5 log gating to reduce chatter
        let mut s1_gate: u8 = 0;
        let mut s2_gate: u8 = 2; // offset so S1/S2 aren’t in lock-step
        loop {
            // Re-check presence flags each iteration; start continuous ranging once per sensor
            let s1 = ctx.shared.sensor1_present.lock(|p| *p);
            if s1 && !s1_started {
                ctx.shared.i2c1.lock(|i2c| {
                    let _ = i2c_write_u8(i2c, 0x2A, 0x0086, 0x01); // clear any stale IRQ
                    let _ = i2c_write_u8(i2c, 0x2A, 0x0087, 0x40); // continuous ranging
                });
                // read back SYSTEM__MODE_START
                let mode = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x2A, 0x0087).ok());
                if let Some(m) = mode {
                    ctx.shared.serial.lock(|serial| { let _ = serial.write(b"S1 start mode=0x");
                        let mut buf = [0u8; 8]; let n = u32_to_ascii(m as u32, &mut buf); let _ = serial.write(&buf[..n]); let _ = serial.write(b"\r\n"); });
                }
                s1_started = true;
            }
            let s2 = ctx.shared.sensor2_present.lock(|p| *p);
            if s2 && !s2_started {
                ctx.shared.i2c1.lock(|i2c| {
                    let _ = i2c_write_u8(i2c, 0x29, 0x0086, 0x01);
                    let _ = i2c_write_u8(i2c, 0x29, 0x0087, 0x40);
                });
                let mode = ctx.shared.i2c1.lock(|i2c| i2c_read_u8(i2c, 0x29, 0x0087).ok());
                if let Some(m) = mode {
                    ctx.shared.serial.lock(|serial| { let _ = serial.write(b"S2 start mode=0x");
                        let mut buf = [0u8; 8]; let n = u32_to_ascii(m as u32, &mut buf); let _ = serial.write(&buf[..n]); let _ = serial.write(b"\r\n"); });
                }
                s2_started = true;
            }

            if s1 {
                if ctx.shared.s1_ranging.lock(|f| *f) {
                    let (stat, dist) = ctx.shared.i2c1.lock(|i2c| {
                        let s1 = VL53L1X::new(0x2A);
                        let rs = s1.get_range_status(i2c).unwrap_or(RangeStatus::None);
                        let mm = s1.get_distance(i2c).unwrap_or(0);
                        let _ = s1.clear_interrupt(i2c); // arm next sample
                        (rs, mm)
                    });
                    
                    if s1_gate == 0 {
                        ctx.shared.serial.lock(|serial| {
                            let _ = serial.write(b"S1:");
                            print_u8(serial, "RS", stat as u8);
                            print_u16(serial, "D", dist);
                            let _ = serial.write(b"\r\n");
                        });
                        Mono::delay(1_u32.millis()).await;
                    }
                    s1_gate = s1_gate.wrapping_add(1) % 5;

                    // --- S1 exit logic ---
                    let now = Mono::now();
                    let (state, entered_at) = (
                        ctx.shared.s1_state.lock(|s| *s),
                        ctx.shared.s1_enter_time.lock(|t| *t),
                    );

                    match state {
                        FootState::Inside => {
                            if dist >= ENTER_MM {
                                if let Some(t0) = entered_at {
                                    let dt_ms = (now - t0).to_millis();
                                    if dt_ms >= WINDOW_MIN_MS && dt_ms <= WINDOW_MAX_MS {
                                        // transient motion detected!
                                        let _ = log::spawn(b"FOOT: S1 TRANSIENT\r\n");
                                        Mono::delay(1_u32.millis()).await;

                                        // trigger action here (open/flag/etc.)
                                        let should_trigger = ctx.shared.in_motion.lock(|moving| !*moving);
                                        if should_trigger {
                                            ctx.shared.button_event.lock(|evt| *evt = true);
                                        }

                                        // go to cooldown
                                        ctx.shared.s1_state.lock(|s| *s = FootState::Cooldown);
                                        let _ = s1_cooldown_alarm::spawn();
                                    } else {
                                        // too short or too long → ignore, back to idle
                                        if dt_ms < WINDOW_MIN_MS {
                                            let _ = log::spawn(b"S1:EXIT_SHORT\r\n");
                                        } else {
                                            let _ = log::spawn(b"S1:EXIT_LONG\r\n");
                                        }
                                        Mono::delay(1_u32.millis()).await;
                                        ctx.shared.s1_state.lock(|s| *s = FootState::Idle);
                                        ctx.shared.s1_enter_time.lock(|t| *t = None);
                                    }
                                } else {
                                    // no timestamp? reset
                                    ctx.shared.s1_state.lock(|s| *s = FootState::Idle);
                                }
                            }
                        }
                        FootState::IgnoredLongPresence => {
                            // We’re waiting for exit to return to Idle without event
                            if dist >= ENTER_MM {
                                let _ = log::spawn(b"S1:EXIT_AFTER_LONG\r\n");
                                Mono::delay(1_u32.millis()).await;
                                ctx.shared.s1_state.lock(|s| *s = FootState::Idle);
                                ctx.shared.s1_enter_time.lock(|t| *t = None);
                            }
                        }
                        _ => {}
                    }
                }
            }

            if s2 {
                if ctx.shared.s2_ranging.lock(|f| *f) {
                    let (stat, dist) = ctx.shared.i2c1.lock(|i2c| {
                        let s2 = VL53L1X::new(0x29);
                        let rs = s2.get_range_status(i2c).unwrap_or(RangeStatus::None);
                        let mm =s2.get_distance(i2c).unwrap_or(0);
                        let _ = s2.clear_interrupt(i2c);
                        (rs,mm)
                    });

                    if s2_gate == 0 {
                        ctx.shared.serial.lock(|serial| {
                            let _ = serial.write(b"S2:");
                            print_u8(serial, "RS", stat as u8);
                            print_u16(serial, "D", dist);
                            let _ = serial.write(b"\r\n");
                        });
                        Mono::delay(1_u32.millis()).await;
                    }
                    s2_gate = s2_gate.wrapping_add(1) % 5;
                }
            }

            Mono::delay(100_u32.millis()).await; // ~10 Hz
        }
    }

    #[task]
    async fn s1_cooldown_alarm(_ctx: s1_cooldown_alarm::Context) {
        Mono::delay(COOLDOWN_MS.millis()).await;
        let _ = s1_cooldown_done::spawn();
    }

    #[task(shared = [s1_state, s1_enter_time])]
    async fn s1_cooldown_done(mut ctx: s1_cooldown_done::Context) {
        ctx.shared.s1_state.lock(|s| *s = FootState::Idle);
        ctx.shared.s1_enter_time.lock(|t| *t = None);
        let _ = log::spawn(b"S1:COOLDOWN_DONE\r\n");
        Mono::delay(1_u32.millis()).await;
    }
}