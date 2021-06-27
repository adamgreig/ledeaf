//! Implement RollingTimer for Choreographer's timebase.

use groundhog::RollingTimer;
use stm32f4xx_hal::stm32;

/// Configure TIM5 to provide a 32-bit 1MHz timebase.
pub fn setup_tim5(clocks: &stm32f4xx_hal::rcc::Clocks, tim5: &stm32::TIM5) {
    // Divide clock to 1MHz, so that each tick is 1µs.
    let psc = (clocks.pclk1().0 / 1_000_000) as u16;
    tim5.psc.write(|w| w.psc().bits(psc - 1));

    // Trigger update event.
    tim5.egr.write(|w| w.ug().update());

    // Enable TIM5.
    tim5.cr1.write(|w| w.cen().enabled());
}

/// A Timer used to provide the RollingTimer required by Choreographer sequences.
#[derive(Copy, Clone, Default)]
pub struct Timer;

impl RollingTimer for Timer {
    type Tick = u32;
    const TICKS_PER_SECOND: Self::Tick = 1_000_000u32;
    fn is_initialized(&self) -> bool {
        unsafe { (*stm32::TIM5::ptr()).cr1.read().cen().bit_is_set() }
    }

    fn get_ticks(&self) -> Self::Tick {
        unsafe { (*stm32::TIM5::ptr()).cnt.read().cnt().bits() }
    }
}
