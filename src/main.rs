#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

// pick a panicking behavior
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::Peripherals;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f3::stm32f303::{interrupt, Interrupt, NVIC};
use stm32f3_discovery::leds::{Direction as LedDirection, Leds};
use stm32f3_discovery::stm32f3xx_hal::gpio::{Edge, Gpioa, Input, Pin, U};
use stm32f3_discovery::stm32f3xx_hal::pac;
use stm32f3_discovery::stm32f3xx_hal::prelude::*;
use stm32f3_discovery::switch_hal::OutputSwitch;

static BOARD_MODE: AtomicU8 = AtomicU8::new(0);

static PA0: Mutex<RefCell<Option<Pin<Gpioa, U<0>, Input>>>> = Mutex::new(RefCell::new(None));

struct Discovery {
    delay: Delay,
    leds: Leds,
}

impl Discovery {
    fn new() -> Self {
        let core_peripherals = Peripherals::take().unwrap();
        let device_periph = pac::Peripherals::take().unwrap();

        let mut rcc = device_periph.RCC.constrain();
        let mut syscfg = device_periph.SYSCFG.constrain(&mut rcc.apb2);

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

        let mut gpioa = device_periph.GPIOA.split(&mut rcc.ahb);
        let mut exti = device_periph.EXTI;
        let mut pa0 = gpioa.pa0.into_input(&mut gpioa.moder);
        pa0.trigger_on_edge(&mut exti, Edge::Rising);
        pa0.make_interrupt_source(&mut syscfg);
        pa0.enable_interrupt(&mut exti);

        cortex_m::interrupt::free(|cs| PA0.borrow(cs).replace(Some(pa0)));

        Discovery { delay, leds }
    }

    fn mainloop(&mut self) -> ! {
        unsafe {
            NVIC::unmask(Interrupt::EXTI0);
        }
        loop {
            let mode = BOARD_MODE.load(Ordering::Relaxed);
            hprintln!("Mode {}", mode).unwrap();
            match mode {
                0 => self.breath_led(LedDirection::North),
                1 => self.breath_leds(),
                _ => BOARD_MODE.store(0, Ordering::Relaxed),
            }
        }
    }

    fn breath_leds(&mut self) {
        let orig_mode = BOARD_MODE.load(Ordering::Relaxed);
        let mut up = true;
        let mut duty = 0;
        let cycle = 1000;
        let step = 1;
        while BOARD_MODE.load(Ordering::Relaxed) == orig_mode {
            for led in &mut self.leds {
                led.on().unwrap();
            }
            self.delay.delay_us(duty);
            for led in &mut self.leds {
                led.off().unwrap();
            }
            self.delay.delay_us(cycle - duty);
            match up {
                true => duty += step,
                false => duty -= step,
            }
            if duty == cycle {
                up = false;
            } else if duty == 0 {
                up = true;
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

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(pa0) = (*PA0.borrow(cs).borrow_mut()).as_mut() {
            pa0.clear_interrupt_pending_bit();
        }
        BOARD_MODE.fetch_add(1, Ordering::Relaxed);
    });
}
