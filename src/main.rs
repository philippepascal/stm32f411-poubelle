#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use stm32f4xx_hal as hal;
use hal::{
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    // Take ownership of the peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(100.MHz()).freeze();

    // Enable GPIOC (for the LED on PC13 on many BlackPill-style boards)
    let gpioc = dp.GPIOC.split();

    // On many STM32F411 "BlackPill"-style boards, PC13 is the user LED.
    let mut led = gpioc.pc13.into_push_pull_output();

    // Simple blink loop
    loop {
        led.set_high();
        cortex_m::asm::delay(50_000_00); // crude delay
        led.set_low();
        cortex_m::asm::delay(50_000_00);
    }
}