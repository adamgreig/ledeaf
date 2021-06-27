//! NEC protocol IR decoding.

#[derive(Copy, Clone, Debug)]
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
    const TOL: i32 = 100;

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
