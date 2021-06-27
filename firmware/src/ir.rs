//! NEC protocol IR decoding.

use core::convert::TryFrom;
use stm32f4xx_hal::stm32;

/// Configure TIM3 to capture rising and falling edges from an IR receiver.
pub fn setup_tim3(clocks: &stm32f4xx_hal::rcc::Clocks, tim3: &stm32::TIM3) {
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

    // Set ARR to max.
    tim3.arr.write(|w| w.arr().bits(0xFFFF));

    // Trigger update event.
    tim3.egr.write(|w| w.ug().update());

    // Enable counter, and set URS=1 to only generate updates on overflow,
    // not on slave-mode resets.
    tim3.cr1.write(|w| w.cen().enabled().urs().counter_only());
}

pub fn process_tim3(decoder: &mut Decoder, mut producer: heapless::spsc::Producer<Command, 8>) {
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
        decoder.reset();
    }

    if sr.cc2if().bit_is_set() {
        // Rising edge detected, indicating the end of a mark pulse.
        let cc2 = unsafe { (*ptr).ccr2.read().ccr().bits() };
        decoder.mark(cc2);
    }

    if sr.cc1if().bit_is_set() {
        // Falling edge detected, indicating the end of a space pulse.
        // The timer is re
        let cc1 = unsafe { (*ptr).ccr1.read().ccr().bits() };
        if let Some(cmd) = decoder.space(cc1) {
            if let Ok(cmd) = Command::try_from(cmd) {
                producer.enqueue(cmd).ok();
            }
        }
    }
}

#[derive(Copy, Clone, Debug, num_enum::IntoPrimitive, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum Command {
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
enum State {
    Reset,
    Addr { idx: usize, addr: u8, naddr: u8 },
    Cmd  { idx: usize, cmd: u8,  ncmd: u8 },
}

pub struct Decoder {
    state: State,
    last_mark: Option<u16>,
}

impl Decoder {
    const HEADER_MARK: i32 = 9000;
    const HEADER_SPACE: i32 = 4500;
    const BIT_MARK: i32 = 562;
    const SPACE_0: i32 = 1687;
    const SPACE_1: i32 = 562;
    const TOL: i32 = 150;

    pub const fn new() -> Self {
        Self { state: State::Reset, last_mark: None }
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
        self.state = State::Reset;
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
            State::Reset => {
                if ((mark  - Self::HEADER_MARK ).abs() < Self::TOL) &&
                   ((space - Self::HEADER_SPACE).abs() < Self::TOL)
                {
                    self.state = State::Addr { idx: 0, addr: 0, naddr: 0 };
                }
            },
            State::Addr { idx, addr, naddr } => {
                let (addr, naddr) = match Self::process_word(mark, space, idx, addr, naddr) {
                    Some((addr, naddr)) => (addr, naddr),
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    if addr == !naddr {
                        self.state = State::Cmd { idx: 0, cmd: 0, ncmd: 0 };
                    } else {
                        self.reset();
                    }
                } else {
                    self.state = State::Addr { idx: idx + 1, addr, naddr };
                }
            },
            State::Cmd { idx, cmd, ncmd  } => {
                let (cmd, ncmd) = match Self::process_word(mark, space, idx, cmd, ncmd) {
                    Some((cmd, ncmd)) => (cmd, ncmd),
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    self.reset();
                    if cmd == !ncmd {
                        return Some(cmd);
                    }
                } else {
                    self.state = State::Cmd { idx: idx + 1, cmd, ncmd };
                }
            },
        }
        None
    }
}
