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

/// Number of LED triangles to drive.
const N_LEDS: usize = 9;

/// Number of colours on the colour wheel.
const N_SPOKES: usize = 12;

/// Brightness scaling factor.
static BRIGHTNESS: AtomicU8 = AtomicU8::new(50);

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up STM32 and peripherals.
    rtt_init_print!();
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    dp.RCC.apb1enr.modify(|_, w| w.tim3en().enabled());
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(20.mhz()).sysclk(48.mhz()).freeze();
    let gpiob = dp.GPIOB.split();
    let led = gpiob.pb5.into_alternate_af5();
    let _ir = gpiob.pb4.into_alternate_af2();
    let spi = Spi::spi1(dp.SPI1, (NoSck, NoMiso, led), ws2812_spi::MODE, 3_000_000.hz(), clocks);
    let mut ws = Ws2812::new(spi);
    Timer::setup(cp.SYST);

    // Set up TIM3 CH1 for IR pulse capture.
    setup_tim3(&clocks, &dp.TIM3);

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
        // Disable interrupt processing while we send the SPI data to
        // prevent timing glitches.
        let level = BRIGHTNESS.load(Ordering::Relaxed);
        cortex_m::interrupt::free(|_|
            ws.write(brightness(data.iter().cloned(), level)).ok()
        );
        cortex_m::asm::delay(20_000);
    }
}

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

    // Enable counter, and set URS=1 to only generate updates on overflow,
    // not on slave-mode resets.
    tim3.cr1.write(|w| w.cen().enabled().urs().counter_only());
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
enum IRCommand {
    On          = 0xBA,
    Off         = 0xB8,
    Timer       = 0xB9,
    Mode1       = 0xBB,
    Mode2       = 0xBC,
    Mode3       = 0xF8,
    Mode4       = 0xF6,
    Mode5       = 0xE9,
    Mode6       = 0xF2,
    Mode7       = 0xF3,
    Mode8       = 0xA1,
    DimDown     = 0xF7,
    DimUp       = 0xA5,
}

#[derive(Copy, Clone, Debug)]
enum NecState {
    Reset,
    Addr { idx: usize, addr: u8, naddr: u8 },
    Cmd  { idx: usize, cmd: u8,  ncmd: u8 },
}

struct NecDecoder {
    state: NecState,
    last_mark: Option<u16>,
}

impl NecDecoder {
    const HEADER_MARK: i32 = 9000;
    const HEADER_SPACE: i32 = 4500;
    const BIT_MARK: i32 = 562;
    const SPACE_0: i32 = 1687;
    const SPACE_1: i32 = 562;
    const TOL: i32 = 100;

    pub const fn new() -> Self {
        NecDecoder { state: NecState::Reset, last_mark: None }
    }

    /// Call when a mark pulse is received, with the width in µs.
    pub fn mark(&mut self, width: u16) {
        match self.last_mark {
            Some(_) => self.reset(),
            None    => self.last_mark = Some(width),
        }
    }

    /// Call when a space pulse is received, with the total mark+space time in µs.
    ///
    /// If a valid command has been fully decoded, returns Some(u8).
    pub fn space(&mut self, width: u16) -> Option<u8> {
        match self.last_mark {
            Some(mark) => {
                self.last_mark = None;
                self.process(mark as i32, (width - mark) as i32)
            },
            None => {
                self.reset();
                None
            },
        }
    }

    /// Call after a timeout to reset the decoder state.
    pub fn reset(&mut self) {
        self.state = NecState::Reset;
        self.last_mark = None;
    }

    fn process_bit(mark: i32, space: i32) -> Option<u8> {
        if (mark - Self::BIT_MARK).abs() < Self::TOL {
            if (space - Self::SPACE_0).abs() < Self::TOL {
                Some(0)
            } else if (space - Self::SPACE_1).abs() < Self::TOL {
                Some(1)
            } else {
                None
            }
        } else {
            None
        }
    }

    fn process_word(mark: i32, space: i32, idx: usize, mut word: u8, mut nword: u8)
        -> Option<(u8, u8)>
    {
        let bit = match Self::process_bit(mark, space) {
            Some(bit) => bit,
            None      => return None,
        };
        if idx < 8 {
            word |= bit << idx;
        } else {
            nword |= bit << (idx - 8);
        }
        Some((word, nword))
    }

    fn process(&mut self, mark: i32, space: i32) -> Option<u8> {
        //rprintln!("process state={:?} mark={} space={}", self.state, mark, space);
        match self.state {
            NecState::Reset => {
                if ((mark  - Self::HEADER_MARK ).abs() < Self::TOL) &&
                   ((space - Self::HEADER_SPACE).abs() < Self::TOL)
                {
                    self.state = NecState::Addr { idx: 0, addr: 0, naddr: 0 };
                }
            },
            NecState::Addr { idx, addr, naddr } => {
                let (addr, naddr) = match Self::process_word(mark, space, idx, addr, naddr) {
                    Some((addr, naddr)) => (addr, naddr),
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    if addr == !naddr {
                        self.state = NecState::Cmd { idx: 0, cmd: 0, ncmd: 0 };
                    } else {
                        self.reset();
                    }
                } else {
                    self.state = NecState::Addr { idx: idx + 1, addr, naddr };
                }
            },
            NecState::Cmd { idx, cmd, ncmd  } => {
                let (cmd, ncmd) = match Self::process_word(mark, space, idx, cmd, ncmd) {
                    Some((cmd, ncmd)) => (cmd, ncmd),
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    self.reset();
                    if cmd == !ncmd {
                        rprintln!("Got command: {:02X}", cmd);
                        return Some(cmd);
                    }
                } else {
                    self.state = NecState::Cmd { idx: idx + 1, cmd, ncmd };
                }
            },
        }
        None
    }
}

#[interrupt]
fn TIM3() {
    static mut DECODER: NecDecoder = NecDecoder::new();

    let ptr = stm32::TIM3::ptr();

    // Clear interrupt bits to acknowledge the interrupt.
    let sr = unsafe { (*ptr).sr.read() };
    unsafe { (*ptr).sr.modify(|_, w|
        w.uif().clear_bit().cc1if().clear_bit().cc2if().clear_bit()
    )};

    if sr.uif().bit_is_set() {
        DECODER.reset();
    }

    if sr.cc1if().bit_is_set() {
        // Falling edge detected, indicating the end of a space pulse.
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
