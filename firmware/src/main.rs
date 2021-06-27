//! Firmware for LEDEAF project.
//! Copyright 2021 Adam Greig.
//! Licensed under the Mozilla Public License v2.

#![no_std]
#![no_main]

use panic_halt as _;
use heapless::Vec;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{Spi, NoSck, NoMiso};
use stm32f4xx_hal::stm32;
use ws2812_spi::Ws2812;
use smart_leds::{brightness, SmartLedsWrite, RGB8, hsv::{Hsv, hsv2rgb}};
use choreographer::engine::{LoopBehavior, Sequence, ActionBuilder};
use groundhog::RollingTimer;

/// Number of LED triangles to drive.
const N_LEDS: usize = 9;

/// Number of colours on the colour wheel.
const N_SPOKES: usize = 12;

/// Brightness scaling factor.
const BRIGHTNESS: u8 = 25;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up STM32 and peripherals.
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(20.mhz()).sysclk(48.mhz()).freeze();
    let gpiob = dp.GPIOB.split();
    let led = gpiob.pb5.into_alternate_af5();
    let spi = Spi::spi1(dp.SPI1, (NoSck, NoMiso, led), ws2812_spi::MODE, 3_000_000.hz(), clocks);
    let mut ws = Ws2812::new(spi);
    Timer::setup(cp.SYST);

    // Storage for LED patterns.
    let mut leds: [Sequence<Timer, {N_SPOKES * 2}>; N_LEDS] = Default::default();
    let mut driver_led: Sequence<Timer, 8> = Default::default();
    let mut data: [RGB8; 1 + N_LEDS] = Default::default();

    // Clear LEDs at startup.
    ws.write(data.iter().cloned()).ok();
    cortex_m::asm::delay(20_000);

    // Make a colourwheel of fully saturated hues.
    let mut wheel: [RGB8; N_SPOKES] = Default::default();
    for (idx, spoke) in wheel.iter_mut().enumerate() {
        let hue = ((idx * 255) / (N_SPOKES - 1)) as u8;
        *spoke = hsv2rgb(Hsv { hue, sat: 255, val: 255 });
    }

    // Set LED scripts to fade into each new colour then hold for a short duration.
    for (idx, led) in leds.iter_mut().enumerate() {
        let mut actions: Vec<_, {N_SPOKES * 2}> = Vec::new();
        for color in wheel.iter().cycle().skip(N_LEDS - idx).take(N_SPOKES) {
            actions.push(ActionBuilder::new().once().seek().color(*color).for_ms(150).finish()).ok();
            actions.push(ActionBuilder::new().once().solid().color(*color).for_ms(300).finish()).ok();
        }
        led.set(&actions, LoopBehavior::LoopForever);
    }

    loop {
        // Set driver LED.
        if let Some(rgb) = driver_led.poll() {
            data[0] = rgb;
        }

        // Set triangle LEDs.
        for (led, entry) in leds.iter_mut().zip(&mut data[1..].iter_mut()) {
            if let Some(rgb) = led.poll() {
                *entry = rgb;
            }
        }

        // Write out LED data, at scaled brightness.
        ws.write(brightness(data.iter().cloned(), BRIGHTNESS)).ok();
        cortex_m::asm::delay(20_000);
    }
}


/// A simple SysTick-based Timer used to provide the RollingTimer
/// required by Choreographer sequences.
#[derive(Copy, Clone, Default)]
struct Timer;

impl Timer {
    /// Initialise SysTick to provide a 24-bit downcounter.
    ///
    /// On STM32F4, with SysTick set to external clock, it runs at HCLK/8.
    pub fn setup(mut syst: cortex_m::peripheral::SYST) {
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
        syst.set_reload(0x00ff_ffff);
        syst.clear_current();
        syst.enable_counter();
    }
}

impl RollingTimer for Timer {
    type Tick = u32;

    // 48MHz/8 = 6MHz. Adjust if HCLK is changed from 48MHz.
    const TICKS_PER_SECOND: Self::Tick = 6_000_000u32;

    fn is_initialized(&self) -> bool { true }

    fn get_ticks(&self) -> Self::Tick {
        // Systick counts down, so reverse to count up instead.
        0x00ff_ffff - cortex_m::peripheral::SYST::get_current()
    }

    fn ticks_since(&self, rhs: Self::Tick) -> Self::Tick {
        // Wrap around after 24 bits, the size of the systick counter.
        self.get_ticks().wrapping_sub(rhs) & 0x00ff_ffff
    }
}
