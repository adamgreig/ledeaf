//! Firmware for LEDEAF project.
//! Copyright 2021 Adam Greig.
//! Licensed under the Mozilla Public License v2.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};
use panic_halt as _;
use heapless::Vec;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{Spi, NoSck, NoMiso};
use stm32f4xx_hal::stm32::{self, interrupt};
use ws2812_spi::Ws2812;
use smart_leds::{brightness, SmartLedsWrite, RGB8, hsv::{Hsv, hsv2rgb}};
use choreographer::engine::{LoopBehavior, Sequence, ActionBuilder};
use groundhog::RollingTimer;
use rtt_target::{rtt_init_print, rprintln};

mod ir;
mod timer;

use ir::{setup_tim3, process_tim3, Decoder as IRDecoder};
use timer::{setup_tim5, Timer};

/// Number of LED triangles to drive.
const N_LEDS: usize = 9;

/// Number of colours on the colour wheel.
const N_SPOKES: usize = 12;

/// Brightness scaling factor.
static BRIGHTNESS: AtomicU8 = AtomicU8::new(150);

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up STM32 and peripherals.
    rtt_init_print!();
    let dp = stm32::Peripherals::take().unwrap();
    dp.RCC.apb1enr.modify(|_, w| w.tim3en().enabled().tim5en().enabled());
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(20.mhz()).sysclk(48.mhz()).freeze();
    let gpiob = dp.GPIOB.split();
    let led = gpiob.pb5.into_alternate_af5();
    let _ir = gpiob.pb4.into_alternate_af2();
    let spi = Spi::spi1(dp.SPI1, (NoSck, NoMiso, led), ws2812_spi::MODE, 3_000_000.hz(), clocks);
    let mut ws = Ws2812::new(spi);

    // Set up TIM3 CH1 for IR pulse capture, and TIM5 to provide a timebase.
    setup_tim3(&clocks, &dp.TIM3);
    setup_tim5(&clocks, &dp.TIM5);

    // Storage for LED patterns.
    let mut leds: [Sequence<Timer, {N_SPOKES * 2}>; N_LEDS] = Sequence::new_array();
    let mut driver_led: Sequence<Timer, 8> = Sequence::empty();
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

    let loop_timer = Timer {};
    loop {
        let t0 = loop_timer.get_ticks();

        // Run driver LED sequence.
        if let Some(rgb) = driver_led.poll() {
            data[0] = rgb;
        }

        // Run triangle LED sequences.
        for (led, entry) in leds.iter_mut().zip(&mut data[1..].iter_mut()) {
            if let Some(rgb) = led.poll() {
                *entry = rgb;
            }
        }

        // Write out LED data, at scaled brightness.
        // Disable interrupt processing while we send the SPI data to prevent timing glitches.
        let level = BRIGHTNESS.load(Ordering::Relaxed);
        cortex_m::interrupt::free(|_|
            ws.write(brightness(data.iter().cloned(), level)).ok()
        );

        // Maintain 100fps; no need to render any faster than that.
        while loop_timer.millis_since(t0) < 10 {}
    }
}

/// We run an IR protocol decoder in the TIM3 interrupt handler,
/// capturing the received pulse widths and decoding them to commands.
#[interrupt]
fn TIM3() {
    static mut DECODER: IRDecoder = IRDecoder::new();
    process_tim3(DECODER);
}
