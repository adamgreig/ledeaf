//! Firmware for LEDEAF project.
//! Copyright 2021 Adam Greig.
//! Licensed under the Mozilla Public License v2.

#![no_std]
#![no_main]

use panic_halt as _;
use heapless::spsc::Queue;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{Spi, NoSck, NoMiso};
use stm32f4xx_hal::stm32::{self, interrupt};
use ws2812_spi::Ws2812;
use smart_leds::{brightness, SmartLedsWrite, RGB8, colors::BLACK};
use choreographer::{script, engine::{LoopBehavior, Sequence}};
use groundhog::RollingTimer;
use rtt_target::{rtt_init_print, rprintln};

mod ir;
mod timer;
mod patterns;

use ir::{setup_tim3, process_tim3, Decoder as IRDecoder, Command as IRCommand};
use timer::{setup_tim5, Timer};
use patterns::Pattern;

/// Number of LED triangles to drive.
const N_LEDS: usize = 9;

/// Queue for moving received IR commands into the main thread.
static mut IRCOMMAND_Q: Queue<IRCommand, 8> = Queue::new();

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

    // Grab the consumer half of the IR command queue.
    let mut ir_consumer = unsafe { IRCOMMAND_Q.split().1 };

    // Set up TIM3 CH1 for IR pulse capture, and TIM5 to provide a timebase.
    setup_tim3(&clocks, &dp.TIM3);
    setup_tim5(&clocks, &dp.TIM5);

    // Storage for LED patterns.
    let mut leds: [Sequence<Timer, 32>; N_LEDS] = Sequence::new_array();
    let mut driver_led: Sequence<Timer, 8> = Sequence::empty();
    let mut data: [RGB8; 1 + N_LEDS] = Default::default();

    // Clear all LEDs at startup.
    ws.write(data.iter().cloned()).ok();
    cortex_m::asm::delay(20_000);

    // Start on the colour wheel pattern.
    Pattern::Wheel.set(&mut leds);

    let loop_timer = Timer {};
    let mut level = 100u8;
    let mut running = true;

    loop {
        let t0 = loop_timer.get_ticks();

        // Process any incoming IR commands.
        if let Some(cmd) = ir_consumer.dequeue() {
            match cmd {
                IRCommand::On if !running   => { running = true; Pattern::Wheel.set(&mut leds); },
                IRCommand::Off if running   => { running = false; Pattern::Off.set(&mut leds); },
                IRCommand::Mode1            => { Pattern::Wheel.set(&mut leds); },
                IRCommand::Mode2            => { Pattern::Smooth.set(&mut leds); },
                IRCommand::DimUp            => { level = level.saturating_add(10); },
                IRCommand::DimDown          => { level = level.saturating_sub(10); },
                _                           => (),
            }

            driver_led.set(&script!(
                |    action |  color | duration_ms | period_ms_f | phase_offset_ms | repeat |
                |   fade_up |  GREEN |          50 |         0.0 |               0 |   once |
                |     solid |  GREEN |          50 |         0.0 |               0 |   once |
                |      seek |  BLACK |          50 |         0.0 |               0 |   once |
            ), LoopBehavior::OneShot);
        }

        // Run driver LED sequence.
        data[0] = driver_led.poll().unwrap_or(BLACK);

        // Run triangle LED sequences.
        for (led, entry) in leds.iter_mut().zip(&mut data[1..].iter_mut()) {
            *entry = led.poll().unwrap_or(BLACK);
        }

        // Write out LED data, at scaled brightness.
        // Disable interrupt processing while we send the SPI data to prevent timing glitches.
        cortex_m::interrupt::free(|_| ws.write(brightness(data.iter().cloned(), level)).ok());

        // Maintain 100fps; no need to render any faster than that.
        while loop_timer.millis_since(t0) < 10 {}
    }
}

/// We run an IR protocol decoder in the TIM3 interrupt handler,
/// capturing the received pulse widths and decoding them to commands.
#[interrupt]
fn TIM3() {
    static mut DECODER: IRDecoder = IRDecoder::new();
    let producer = unsafe { IRCOMMAND_Q.split().0 };
    process_tim3(DECODER, producer);
}
