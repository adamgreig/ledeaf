#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m::peripheral::Peripherals;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::spi::{Spi, NoSck, NoMiso};
use stm32f4xx_hal::stm32;
use ws2812_spi::Ws2812;
use smart_leds::{brightness, SmartLedsWrite, RGB8};

#[cortex_m_rt::entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (stm32::Peripherals::take(), Peripherals::take()) {
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(20.mhz()).sysclk(48.mhz()).freeze();
        let gpiob = dp.GPIOB.split();
        let led = gpiob.pb5.into_alternate_af5();
        let mut delay = Delay::new(cp.SYST, clocks);
        let spi = Spi::spi1(dp.SPI1, (NoSck, NoMiso, led), ws2812_spi::MODE, 3_000_000.hz(), clocks);
        let mut ws = Ws2812::new(spi);
        let mut data = [RGB8::default(); 12];

        const BRIGHTNESS: u8 = 255;

        loop {
            for i in 0..25 {
                let drop = wheel(i * 10);
                for led in 0..11 {
                    for j in 0..11 {
                        if j == led {
                            data[j] = drop;
                        } else {
                            data[j] = RGB8::default();
                        }
                    }
                    ws.write(brightness(data.iter().cloned(), BRIGHTNESS)).unwrap();
                    delay.delay_ms(50u8);
                }
                data[10] = RGB8::default();
                data[11] = drop;
                ws.write(brightness(data.iter().cloned(), BRIGHTNESS)).unwrap();
                delay.delay_ms(50u8);

                for _ in 0..=255 {
                    if data[11].r > 0 {
                        data[11].r -= 1;
                    }
                    if data[11].g > 0 {
                        data[11].g -= 1;
                    }
                    if data[11].b > 0 {
                        data[11].b -= 1;
                    }
                    ws.write(brightness(data.iter().cloned(), BRIGHTNESS)).unwrap();
                    delay.delay_ms(1u8);
                }
            }
        }
    }
    loop {}
}

fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}
