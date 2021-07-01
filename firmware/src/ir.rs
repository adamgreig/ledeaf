//! NEC protocol IR decoding.

use core::convert::TryFrom;
use stm32f4xx_hal::stm32;
use groundhog::RollingTimer;
use crate::timer::Timer;

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
    DimUp       = 0x00,
    DimDown     = 0x01,
    Off         = 0x02,
    On          = 0x03,
    Red         = 0x04,
    Green       = 0x05,
    Blue        = 0x06,
    White       = 0x07,
    LightRed    = 0x08,
    LightGreen  = 0x09,
    LightBlue   = 0x0A,
    Flash       = 0x0B,
    Orange      = 0x0C,
    Cyan        = 0x0D,
    Purple      = 0x0E,
    Strobe      = 0x0F,
    LightOrange = 0x10,
    Cerulean    = 0x11,
    LightPurple = 0x12,
    Fade        = 0x13,
    Yellow      = 0x14,
    SeaGreen    = 0x15,
    Pink        = 0x16,
    Smooth      = 0x17,

}

#[derive(Copy, Clone, Debug)]
enum State {
    Reset,
    Addr { idx: usize, addr: u16 },
    Cmd  { idx: usize, cmd: u16 },
}

pub struct Decoder {
    state: State,
    last_mark: Option<u16>,
    last_cmd: Option<(u32, u8)>,
}

impl Decoder {
    const HEADER_MARK: i32 = 9000;
    const HEADER_SPACE: i32 = 4500;
    const REPEAT_SPACE: i32 = 2250;
    const BIT_MARK: i32 = 562;
    const SPACE_0: i32 = 1687;
    const SPACE_1: i32 = 562;
    const HEADER_TOL: i32 = 300;
    const BIT_TOL: i32 = 150;

    pub const fn new() -> Self {
        Self { state: State::Reset, last_mark: None, last_cmd: None }
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

    fn process_bit(mark: i32, space: i32) -> Option<u16> {
        if (mark - Self::BIT_MARK).abs() < Self::BIT_TOL {
            if (space - Self::SPACE_0).abs() < Self::BIT_TOL {
                Some(0)
            } else if (space - Self::SPACE_1).abs() < Self::BIT_TOL {
                Some(1)
            } else {
                None
            }
        } else {
            None
        }
    }

    fn process_word(mark: i32, space: i32, idx: usize, mut word: u16)
        -> Option<u16>
    {
        let bit = match Self::process_bit(mark, space) {
            Some(bit) => bit,
            None      => return None,
        };
        word |= bit << idx;
        Some(word)
    }

    fn process(&mut self, mark: i32, space: i32) -> Option<u8> {
        // Discard stale last_cmds.
        let timer = Timer {};
        if let Some((t0, _)) = self.last_cmd {
            if timer.millis_since(t0) > 200 {
                self.last_cmd = None;
            }
        }

        match self.state {
            State::Reset => {
                if ((mark  - Self::HEADER_MARK ).abs() < Self::HEADER_TOL) &&
                   ((space - Self::HEADER_SPACE).abs() < Self::HEADER_TOL)
                {
                    // Header received, prepare to receive an address.
                    self.state = State::Addr { idx: 0, addr: 0 };
                } else if ((mark  - Self::HEADER_MARK ).abs() < Self::HEADER_TOL) &&
                          ((space - Self::REPEAT_SPACE).abs() < Self::HEADER_TOL)
                {
                    // Repeat command received, check if we have a recent command to repeat.
                    if let Some((_, cmd)) = self.last_cmd {
                        // Reset timeout and return the last command.
                        self.last_cmd = Some((timer.get_ticks(), cmd));
                        return Some(cmd);
                    }
                }
            },

            State::Addr { idx, addr } => {
                let addr = match Self::process_word(mark, space, idx, addr) {
                    Some(addr) => addr,
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    if addr == 0x10FF {
                        self.state = State::Cmd { idx: 0, cmd: 0 };
                    } else {
                        self.reset();
                    }
                } else {
                    self.state = State::Addr { idx: idx + 1, addr };
                }
            },

            State::Cmd { idx, cmd  } => {
                let cmd = match Self::process_word(mark, space, idx, cmd) {
                    Some(cmd) => cmd,
                    None => { self.reset(); return None; },
                };

                if idx == 15 {
                    self.reset();
                    if !((cmd >> 8) as u8) == ((cmd & 0xFF) as u8) {
                        // Store this command as our most recent command, in case
                        // we receive a repeat command later.
                        self.last_cmd = Some((timer.get_ticks(), (cmd >> 8) as u8));
                        return Some((cmd >> 8) as u8);
                    }
                } else {
                    self.state = State::Cmd { idx: idx + 1, cmd };
                }
            },
        }
        None
    }
}
