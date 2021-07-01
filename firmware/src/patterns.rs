use heapless::Vec;
use choreographer::engine::{LoopBehavior, Sequence, ActionBuilder};
use smart_leds::{RGB8, hsv::{Hsv, hsv2rgb}};
use crate::{N_LEDS, timer::Timer};

#[derive(Copy, Clone, Debug)]
pub enum Pattern {
    Off,
    Flash,
    Strobe,
    Fade,
    Smooth,
    Solid(RGB8),
}

impl Pattern {
    pub fn set<const N: usize>(self, leds: &mut [Sequence<Timer, N>]) {
        match self {
            Pattern::Off        => set_off(leds),
            Pattern::Flash      => set_flash(leds),
            Pattern::Strobe     => set_strobe(leds),
            Pattern::Fade       => set_fade(leds),
            Pattern::Smooth     => set_smooth(leds),
            Pattern::Solid(rgb) => set_solid(leds, rgb),
        }
    }
}

fn set_off<const N: usize>(leds: &mut [Sequence<Timer, N>]) {
    for led in leds.iter_mut() {
        led.clear();
    }
}

fn set_strobe<const N: usize>(leds: &mut [Sequence<Timer, N>]) {
    // Number of colours on the colour wheel.
    const N_SPOKES: usize = 12;

    // Time to fade into new colour.
    const T_FADE: u32 = 150;

    // Time to hold new colour.
    const T_HOLD: u32 = 300;

    // Make a colourwheel of fully saturated hues.
    let wheel = make_wheel::<N_SPOKES>();

    // Set LED scripts to fade into each new colour then hold for a short duration.
    for (idx, led) in leds.iter_mut().enumerate() {
        let mut actions: Vec<_, {N_SPOKES * 2}> = Vec::new();
        for color in wheel.iter().cycle().skip(N_LEDS - idx).take(N_SPOKES) {
            actions.extend_from_slice(&[
                ActionBuilder::new().once().seek().color(*color).for_ms(T_FADE).finish(),
                ActionBuilder::new().once().solid().color(*color).for_ms(T_HOLD).finish(),
            ]).ok();
        }
        led.set(&actions, LoopBehavior::LoopForever);
    }
}

fn set_flash<const N: usize>(leds: &mut [Sequence<Timer, N>]) {
    // Number of colours on the colour wheel.
    const N_SPOKES: usize = 12;

    // Time to hold new colour.
    const T_HOLD: u32 = 1000;

    // Make a colourwheel of fully saturated hues.
    let wheel = make_wheel::<N_SPOKES>();

    // Set LED scripts to fade into each new colour then hold for a short duration.
    for led in leds.iter_mut() {
        let mut actions: Vec<_, N_SPOKES> = Vec::new();
        for color in wheel.iter() {
            actions.extend_from_slice(&[
                ActionBuilder::new().once().solid().color(*color).for_ms(T_HOLD).finish(),
            ]).ok();
        }
        led.set(&actions, LoopBehavior::LoopForever);
    }
}

fn set_smooth<const N: usize>(leds: &mut [Sequence<Timer, N>]) {
    // Number of colours on the colour wheel.
    const N_SPOKES: usize = 12;

    // Time to fade into new colour.
    const T_FADE: u32 = 150;

    // Make a colourwheel of fully saturated hues.
    let wheel = make_wheel::<N_SPOKES>();

    // Set LED scripts to fade into each new colour.
    for (idx, led) in leds.iter_mut().enumerate() {
        let mut actions: Vec<_, {N_SPOKES * 2}> = Vec::new();
        for color in wheel.iter().cycle().skip(N_LEDS - idx).take(N_SPOKES) {
            actions.push(
                ActionBuilder::new().once().seek().color(*color).for_ms(T_FADE).finish(),
            ).ok();
        }
        led.set(&actions, LoopBehavior::LoopForever);
    }
}

fn set_fade<const N: usize>(leds: &mut [Sequence<Timer, N>]) {
    // Number of colours on the colour wheel.
    const N_SPOKES: usize = 12;

    // Time to fade into new colour.
    const T_FADE: u32 = 2000;

    // Make a colourwheel of fully saturated hues.
    let wheel = make_wheel::<N_SPOKES>();

    // Set LED scripts to fade into each new colour.
    for led in leds.iter_mut() {
        let mut actions: Vec<_, N_SPOKES> = Vec::new();
        for color in wheel.iter() {
            actions.push(
                ActionBuilder::new().once().seek().color(*color).for_ms(T_FADE).finish(),
            ).ok();
        }
        led.set(&actions, LoopBehavior::LoopForever);
    }
}

fn set_solid<const N: usize>(leds: &mut [Sequence<Timer, N>], rgb: RGB8) {
    for led in leds.iter_mut() {
        led.set(&[ActionBuilder::new().once().solid().color(rgb).for_ms(100).finish()],
                LoopBehavior::LoopForever);
    }
}

/// Create a colour wheel with N_SPOKES distinct hues at full saturation and value.
fn make_wheel<const N_SPOKES: usize>() -> [RGB8; N_SPOKES] {
    let mut wheel: [RGB8; N_SPOKES] = [RGB8::new(0, 0, 0); N_SPOKES];
    for (idx, spoke) in wheel.iter_mut().enumerate() {
        let hue = ((idx * 255) / (N_SPOKES - 1)) as u8;
        *spoke = hsv2rgb(Hsv { hue, sat: 255, val: 255 });
    }
    wheel
}
