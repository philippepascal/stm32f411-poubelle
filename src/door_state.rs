// going with the assumption that we are taking around 2000 steps (10 wheel turns) in 2 seconds
// that's 1kHz or 1 step per millisecond. We are going to use delays to do it linerarly within the loop.
// that is also ok since the only thing that can modify the operation is a fault detection, which is an interrupt.
// we'll need a flag to indicate a fault has happened, loop needs to be cancelled and state changed (that part in the interrupt handler)

use alloc::boxed::Box;
use cortex_m::asm;
use crate::usb_log;

pub fn init_state(max_open: u16) -> Box<dyn State> {
    Box::new(ClosedState{keep_running: true, position: Position{max_open, current: 0}})
}

pub fn calibrate() -> Box<dyn State> {
    Box::new(CalibratingState{})
}

#[derive(Clone,Copy)]
struct Position {
    //really is a count of all microsteps from fully closed
    max_open: u16,
    //always in reference to fully closed, so 0 = fully closed
    current: u16,
}

pub trait State {
    fn run(self: Box<Self>) -> Box<dyn State>;
}

struct UndefinedState {
}
impl State for UndefinedState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        Box::new(UndefinedState{})
    }
}
struct OpenState {
    keep_running: bool,
    position: Position,
}
impl State for OpenState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        usb_log(0, "starting Open State run()");
        let mut elapsed_ms = 0;
        while self.keep_running && elapsed_ms < 5000 {
            // roughly 1ms delay at 100 MHz
            asm::delay(100_000);
            elapsed_ms += 1;
        }
        usb_log(0, "stoping Open State run()");
        Box::new(ClosingState{position: self.position})
    }
}

struct ClosingState {
    position: Position,
}
impl State for ClosingState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        usb_log(0, "starting Closing State run()");
        Box::new(ClosedState{keep_running: true, position: self.position})
    }
}
struct ClosedState {
    keep_running: bool,
    position: Position,
}
impl State for ClosedState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        usb_log(0, "starting Closed State run()");
        let mut elapsed_ms = 0;
        while self.keep_running && elapsed_ms < 5000 {
            // roughly 1ms delay at 100 MHz
            asm::delay(100_000);
            elapsed_ms += 1;
        }
        usb_log(0, "closing Closed State run()");
        Box::new(OpeningState{position: self.position})
    }
}

struct OpeningState {
    position: Position,
}
impl State for OpeningState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        usb_log(0, "starting Opening State run()");
        let mut step_count = self.position.current;
        let mut elapsed_ms = 0;
        while step_count > 0 && elapsed_ms < 5000 {
            step_count -= 1;
            // roughly 1ms delay at 100 MHz
            asm::delay(100_000);
            elapsed_ms += 1;
        }
        usb_log(0, "closing Opening State run()");
        Box::new(OpenState{keep_running: true, position: self.position})
    }
}

struct CalibratingState {
}
impl State for CalibratingState {
    fn run(self: Box<Self>) -> Box<dyn State> {
        //faking it for now
        // 1. 2 turns open
        // 2. 2 turns close
        Box::new(ClosedState{keep_running: true, position: Position { max_open: 2000, current: 0 }})
    }
}