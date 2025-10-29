#![no_std]
#![no_main]

extern crate alloc;
use alloc_cortex_m::CortexMHeap;
use alloc::boxed::Box;

use cortex_m_rt::entry;
use panic_halt as _;

use stm32f4xx_hal as hal;
use hal::{
    pac,
    prelude::*,
};

mod mem;
mod door_state;

use crate::mem::*;
use crate::door_state::*;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[entry]
fn main() -> ! {
    // Take ownership of the peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(100.MHz()).freeze();

    // Enable GPIOC (for the LED on PC13 on many BlackPill-style boards)
    let gpioc = dp.GPIOC.split();

    unsafe {
        // Pick a heap size. Let's say 8 KB for now.
        // cortex_m_rt gives us `extern "C" { static _sheap: u8; }` / `heap_start()`,
        // but `CortexMHeap` provides helper.
        const HEAP_SIZE: usize = 8 * 1024;
        ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE);
    }


    // Read calibration data from flash
    let calib_data = DoorCalibrationData::read_from_flash();
    let mut state: Box<dyn State> = match calib_data.is_valid() {
        true => {
            // Use the calibration data
            init_state(calib_data.max_open_steps)
        },
        false => {
            // Handle invalid calibration data (e.g., set defaults or enter error state)
            calibrate()
        },
    };

    // On many STM32F411 "BlackPill"-style boards, PC13 is the user LED.
    let mut led = gpioc.pc13.into_push_pull_output();

    // Simple blink loop
    loop {
        led.set_high();
        cortex_m::asm::delay(50_000_00); // crude delay
        led.set_low();
        cortex_m::asm::delay(50_000_00);

        state = state.run();
    }
}