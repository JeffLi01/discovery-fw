#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, Ordering};

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::delay::Delay;
use cortex_m::Peripherals;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f3_discovery::leds::{Direction as LedDirection, Leds};
use stm32f3_discovery::stm32f3xx_hal::pac;
use stm32f3_discovery::stm32f3xx_hal::prelude::*;
use stm32f3_discovery::switch_hal::OutputSwitch;

static BOARD_MODE: AtomicU8 = AtomicU8::new(0);

struct Discovery {
    delay: Delay,
    leds: Leds,
}

impl Discovery {
    fn new() -> Self {
        let core_peripherals = Peripherals::take().unwrap();
        let device_periph = pac::Peripherals::take().unwrap();

        let mut rcc = device_periph.RCC.constrain();

        let syst = core_peripherals.SYST;
        let delay = Delay::new(syst, 8_000_000);

        let gpioe = device_periph.GPIOE.split(&mut rcc.ahb);
        let mut moder = gpioe.moder;
        let mut otyper = gpioe.otyper;
        let leds = stm32f3_discovery::leds::Leds::new(
            gpioe.pe8,
            gpioe.pe9,
            gpioe.pe10,
            gpioe.pe11,
            gpioe.pe12,
            gpioe.pe13,
            gpioe.pe14,
            gpioe.pe15,
            &mut moder,
            &mut otyper,
        );

        Discovery { delay, leds }
    }

    fn mainloop(&mut self) -> ! {
        loop {
            let mode = BOARD_MODE.load(Ordering::Relaxed);
            hprintln!("Mode {}", mode).unwrap();
            match mode {
                0 => self.breath_led(LedDirection::North),
                1 => {}
                _ => BOARD_MODE.store(0, Ordering::Relaxed),
            }
        }
    }

    fn breath_led(&mut self, direction: LedDirection) {
        let orig_mode = BOARD_MODE.load(Ordering::Relaxed);
        let mut up = true;
        let mut idle = 0;
        let cycle = 1000;
        let step = 1;
        while BOARD_MODE.load(Ordering::Relaxed) == orig_mode {
            self.leds.for_direction(direction).on().unwrap();
            self.delay.delay_us(idle);
            self.leds.for_direction(direction).off().unwrap();
            self.delay.delay_us(cycle - idle);
            match up {
                true => idle += step,
                false => idle -= step,
            }
            if idle == cycle {
                up = false;
            } else if idle == 0 {
                up = true;
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let mut discovery = Discovery::new();

    discovery.mainloop();
}
