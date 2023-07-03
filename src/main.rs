#![no_std]
#![no_main]

use accelerometer::vector::F32x3;
use core::cell::RefCell;
use core::f32::consts::PI;
use core::sync::atomic::{AtomicU8, Ordering};
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::Peripherals;
use cortex_m_rt::entry;
#[cfg(debug_assertions)]
use cortex_m_semihosting::hprintln;
use l3gd20::L3gd20;
use lsm303dlhc::Lsm303dlhc;
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32f3xx_hal::gpio::{
    Alternate, Edge, Gpioa, Gpiob, Gpioe, Input, OpenDrain, Output, Pin, PushPull, U,
};
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::pac::{self, interrupt, Interrupt, NVIC, SPI1};
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::spi;
use stm32f3xx_hal::spi::Spi;
use stm32f3xx_hal::time::rate::Hertz;
use switch_hal::OutputSwitch;

mod board;

use board::{Direction, Leds};

static BOARD_MODE: AtomicU8 = AtomicU8::new(0);

static PA0: Mutex<RefCell<Option<Pin<Gpioa, U<0>, Input>>>> = Mutex::new(RefCell::new(None));

type L3gd20Alias = L3gd20<
    Spi<
        SPI1,
        (
            Pin<Gpioa, U<5>, Alternate<PushPull, 5>>,
            Pin<Gpioa, U<6>, Alternate<PushPull, 5>>,
            Pin<Gpioa, U<7>, Alternate<PushPull, 5>>,
        ),
    >,
    Pin<Gpioe, U<3>, Output<PushPull>>,
>;

type Lsm303dlhcAlias = Lsm303dlhc<
    I2c<
        pac::I2C1,
        (
            Pin<Gpiob, U<6>, Alternate<OpenDrain, 4>>,
            Pin<Gpiob, U<7>, Alternate<OpenDrain, 4>>,
        ),
    >,
>;

struct Discovery {
    delay: Delay,
    leds: Leds,
    l3gd20: L3gd20Alias,
    lsm303dlhc: Lsm303dlhcAlias,
}

impl Discovery {
    fn new() -> Self {
        let core_peripherals = Peripherals::take().unwrap();

        let syst = core_peripherals.SYST;
        let delay = Delay::new(syst, 8_000_000);

        let device_periph = pac::Peripherals::take().unwrap();

        let mut rcc = device_periph.RCC.constrain();
        let mut syscfg = device_periph.SYSCFG.constrain(&mut rcc.apb2);

        let mut gpioa = device_periph.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = device_periph.GPIOB.split(&mut rcc.ahb);
        let mut gpioe = device_periph.GPIOE.split(&mut rcc.ahb);

        let leds = Leds::new(
            gpioe.pe8,
            gpioe.pe9,
            gpioe.pe10,
            gpioe.pe11,
            gpioe.pe12,
            gpioe.pe13,
            gpioe.pe14,
            gpioe.pe15,
            &mut gpioe.moder,
            &mut gpioe.otyper,
        );

        let mut exti = device_periph.EXTI;
        let mut pa0 = gpioa.pa0.into_input(&mut gpioa.moder);
        pa0.trigger_on_edge(&mut exti, Edge::Rising);
        syscfg.select_exti_interrupt_source(&pa0);
        pa0.enable_interrupt(&mut exti);

        cortex_m::interrupt::free(|cs| PA0.borrow(cs).replace(Some(pa0)));

        let mut flash = device_periph.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let sck_pin =
            gpioa
                .pa5
                .into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let mosi_pin =
            gpioa
                .pa6
                .into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let miso_pin =
            gpioa
                .pa7
                .into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let config = spi::config::Config::default();
        let spi = Spi::new(
            device_periph.SPI1,
            (sck_pin, mosi_pin, miso_pin),
            config,
            clocks,
            &mut rcc.apb2,
        );
        let cs = gpioe
            .pe3
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        let l3gd20 = L3gd20::new(spi, cs).unwrap();

        let scl_pin =
            gpiob
                .pb6
                .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let sda_pin =
            gpiob
                .pb7
                .into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let i2c = I2c::new(
            device_periph.I2C1,
            (scl_pin, sda_pin),
            Hertz::new(400_000),
            clocks,
            &mut rcc.apb1,
        );
        let lsm303dlhc = Lsm303dlhc::new(i2c).unwrap();

        Discovery {
            delay,
            l3gd20,
            leds,
            lsm303dlhc,
        }
    }

    fn mainloop(&mut self) -> ! {
        unsafe {
            NVIC::unmask(Interrupt::EXTI0);
        }
        loop {
            let mode = BOARD_MODE.load(Ordering::Relaxed);
            #[cfg(debug_assertions)]
            hprintln!("Mode {}", mode).unwrap();
            match mode {
                0 => self.breath_led(Direction::North),
                1 => self.breath_leds(),
                2 => self.run_leveller(),
                3 => self.cycle_leds(),
                4 => self.run_gyroscope(),
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

    fn breath_led(&mut self, direction: Direction) {
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

    fn cycle_leds(&mut self) {
        for led in &mut self.leds {
            led.off().unwrap();
        }

        let orig_mode = BOARD_MODE.load(Ordering::Relaxed);
        let mut index = 0;
        let directions = [
            Direction::North,
            Direction::NorthEast,
            Direction::East,
            Direction::SouthEast,
            Direction::South,
            Direction::SouthWest,
            Direction::West,
            Direction::NorthWest,
        ];
        while BOARD_MODE.load(Ordering::Relaxed) == orig_mode {
            self.leds.for_direction(directions[index]).on().ok();
            self.delay.delay_ms(1000);
            self.leds.for_direction(directions[index]).off().ok();
            index += 1;
            index %= directions.len();
        }
    }

    fn run_leveller(&mut self) {
        let orig_mode = BOARD_MODE.load(Ordering::Relaxed);
        let mut old_direction: Option<Direction> = None;
        let threshold = 0.2;

        while BOARD_MODE.load(Ordering::Relaxed) == orig_mode {
            let i16x3 = self.lsm303dlhc.accel().unwrap();
            if let Some(direction) = old_direction {
                self.leds.for_direction(direction).off().ok();
            }
            let f32x3 = F32x3::new(i16x3.x as f32, i16x3.y as f32, i16x3.z as f32);
            let direction = calculate_direction(f32x3, threshold);
            old_direction = direction;
            if let Some(d) = direction {
                self.leds.for_direction(d).on().ok();
            }
        }
    }

    fn run_gyroscope(&mut self) {
        let orig_mode = BOARD_MODE.load(Ordering::Relaxed);

        while BOARD_MODE.load(Ordering::Relaxed) == orig_mode {
            let _i16x3 = self.l3gd20.gyro().unwrap();
            #[cfg(debug_assertions)]
            hprintln!("{:?}", _i16x3).unwrap();
        }
    }
}

///
///               N(-y)
///               ^
///               |
/// W(-x)   <-----+----->    E(+x)
///               |
///               v
///               S(+y)
///
fn calculate_direction(f32x3: F32x3, threshold: f32) -> Option<Direction> {
    let acc = f32x3.x * f32x3.x + f32x3.y * f32x3.y;
    if acc < threshold * threshold {
        return None;
    }
    let acc = libm::sqrtf(acc);
    let mut radians_by_pi = libm::acosf(f32x3.x / acc) / PI;
    if f32x3.y > 0.0 {
        radians_by_pi = -radians_by_pi;
    }
    let direction = match radians_by_pi {
        _ if 5. / 8. < radians_by_pi && radians_by_pi <= 7. / 8. => Direction::NorthWest,
        _ if 3. / 8. < radians_by_pi && radians_by_pi <= 5. / 8. => Direction::North,
        _ if 1. / 8. < radians_by_pi && radians_by_pi <= 3. / 8. => Direction::NorthEast,
        _ if -1. / 8. < radians_by_pi && radians_by_pi <= 1. / 8. => Direction::East,
        _ if -3. / 8. < radians_by_pi && radians_by_pi <= -1. / 8. => Direction::SouthEast,
        _ if -5. / 8. < radians_by_pi && radians_by_pi <= -3. / 8. => Direction::South,
        _ if -7. / 8. < radians_by_pi && radians_by_pi <= -5. / 8. => Direction::SouthWest,
        _ => Direction::West,
    };
    Some(direction)
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
            pa0.clear_interrupt();
        }
        BOARD_MODE.fetch_add(1, Ordering::Relaxed);
    });
}
