#![no_std]
#![no_main]

use panic_halt as _;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{Spi, NoSck, NoMiso};
use stm32f4xx_hal::stm32;
use ws2812_spi::Ws2812;
use smart_leds::{brightness, SmartLedsWrite, RGB8, hsv::{Hsv, hsv2rgb}};
use choreographer::engine::{LoopBehavior, Sequence, ActionBuilder};
use groundhog::RollingTimer;

#[derive(Copy, Clone, Default)]
struct Timer;

impl Timer {
    pub fn setup(mut syst: cortex_m::peripheral::SYST) {
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
        syst.set_reload(0x00ff_ffff);
        syst.clear_current();
        syst.enable_counter();
    }
}

impl RollingTimer for Timer {
    type Tick = u32;
    const TICKS_PER_SECOND: Self::Tick = 6_000_000u32;

    fn is_initialized(&self) -> bool { true }

    fn get_ticks(&self) -> Self::Tick {
        0x00ff_ffff - cortex_m::peripheral::SYST::get_current()
    }

    fn ticks_since(&self, rhs: Self::Tick) -> Self::Tick {
        self.get_ticks().wrapping_sub(rhs) & 0x00ff_ffff
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(20.mhz()).sysclk(48.mhz()).freeze();
    Timer::setup(cp.SYST);
    let gpiob = dp.GPIOB.split();
    let led = gpiob.pb5.into_alternate_af5();
    let spi = Spi::spi1(dp.SPI1, (NoSck, NoMiso, led), ws2812_spi::MODE, 3_000_000.hz(), clocks);
    let mut ws = Ws2812::new(spi);
    const N_LEDS: usize = 12;
    let mut leds: [Sequence<Timer, 8>; N_LEDS] = Default::default();
    let mut data: [RGB8; N_LEDS] = Default::default();

    // Clear LEDs at startup.
    ws.write(data.iter().cloned()).ok();
    cortex_m::asm::delay(20_000);

    // Make a 10-colour wheel of hues.
    const N_SPOKES: usize = 10;
    let mut wheel: [RGB8; N_SPOKES] = Default::default();
    for (idx, spoke) in wheel.iter_mut().enumerate() {
        let hue = ((idx * 255) / (N_SPOKES - 1)) as u8;
        *spoke = hsv2rgb(Hsv { hue, sat: 255, val: 255 });
    }

    // Set LED scripts.
    for (idx, led) in leds.iter_mut().enumerate() {
        led.set(&[
            ActionBuilder::new().once().seek() .color(wheel[(idx + 0) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 0) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 1) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 1) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 2) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 2) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 3) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 3) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 4) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 4) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 5) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 5) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 6) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 6) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 7) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 7) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 8) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 8) % N_SPOKES]).for_ms(300).finish(),
            ActionBuilder::new().once().seek() .color(wheel[(idx + 9) % N_SPOKES]).for_ms(150).finish(),
            ActionBuilder::new().once().solid().color(wheel[(idx + 9) % N_SPOKES]).for_ms(300).finish(),
        ], LoopBehavior::LoopForever);
    }

    // Play LED scripts.
    loop {
        for (led, entry) in leds.iter_mut().zip(data.iter_mut()) {
            if let Some(rgb) = led.poll() {
                *entry = rgb;
            }
        }
        ws.write(brightness(data.iter().cloned(), 50)).ok();
        cortex_m::asm::delay(20_000);
    }
}
