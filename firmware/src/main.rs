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

use ir::Decoder as IRDecoder;

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

/// Configure TIM3 to capture rising and falling edges from an IR receiver.
fn setup_tim3(clocks: &stm32f4xx_hal::rcc::Clocks, tim3: &stm32::TIM3) {
    // Enable interrupt on update, CC1, and CC2.
    tim3.dier.write(|w| w.uie().enabled().cc1ie().enabled().cc2ie().enabled());
    unsafe { cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::TIM3) };

    // Set up input capture from TI1 on both CC1 and CC2.
    tim3.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti1());

    // Set CC1 to falling edge and CC2 to rising edge.
    tim3.ccer.write(|w| w.cc1e().set_bit().cc1p().set_bit().cc2e().set_bit());

    // Enable slave mode to reset on falling edge.
    tim3.smcr.write(|w| w.ts().ti1fp1().sms().reset_mode());

    // Divide clock to 1MHz, so that each tick is 1µs and we overflow every 65ms.
    let psc = (clocks.pclk1().0 / 1_000_000) as u16;
    tim3.psc.write(|w| w.psc().bits(psc - 1));

    // Trigger update event.
    tim3.egr.write(|w| w.ug().update());

    // Enable counter, and set URS=1 to only generate updates on overflow,
    // not on slave-mode resets.
    tim3.cr1.write(|w| w.cen().enabled().urs().counter_only());
}

/// Configure TIM5 to provide a 32-bit 1MHz timebase.
fn setup_tim5(clocks: &stm32f4xx_hal::rcc::Clocks, tim5: &stm32::TIM5) {
    // Divide clock to 1MHz, so that each tick is 1µs.
    let psc = (clocks.pclk1().0 / 1_000_000) as u16;
    tim5.psc.write(|w| w.psc().bits(psc - 1));

    // Trigger update event.
    tim5.egr.write(|w| w.ug().update());

    // Enable TIM5.
    tim5.cr1.write(|w| w.cen().enabled());
}

/// We run an IR protocol decoder in the TIM3 interrupt handler,
/// capturing the received pulse widths and decoding them to commands.
#[interrupt]
fn TIM3() {
    static mut DECODER: IRDecoder = IRDecoder::new();

    let ptr = stm32::TIM3::ptr();

    // Clear interrupt bits to acknowledge the interrupt.
    let sr = unsafe { (*ptr).sr.read() };
    unsafe { (*ptr).sr.modify(|_, w|
        w.uif().clear_bit().cc1if().clear_bit().cc2if().clear_bit()
    )};

    if sr.uif().bit_is_set() {
        // Reset decoder state machine on timer overflow.
        // The timer is reset after each pulse, so an overflow means
        // no edge was received for the entire timer period of 65ms.
        DECODER.reset();
    }

    if sr.cc1if().bit_is_set() {
        // Falling edge detected, indicating the end of a space pulse.
        // The timer is re
        let cc1 = unsafe { (*ptr).ccr1.read().ccr().bits() };
        match DECODER.space(cc1) {
            Some(cmd) => match cmd {
                0xF7 => { BRIGHTNESS.fetch_sub(5, Ordering::Relaxed); },
                0xA5 => { BRIGHTNESS.fetch_add(5, Ordering::Relaxed); },
                _    => (),
            },
            None => (),
        }
    }

    if sr.cc2if().bit_is_set() {
        // Rising edge detected, indicating the end of a mark pulse.
        let cc2 = unsafe { (*ptr).ccr2.read().ccr().bits() };
        DECODER.mark(cc2);
    }
}


/// A Timer used to provide the RollingTimer required by Choreographer sequences.
#[derive(Copy, Clone, Default)]
struct Timer;

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
