//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::bank0::*,
    gpio::pin::{Pin, PushPullOutput},
    pac,
    sio::Sio,
    timer::{Alarm0, Timer},
    watchdog::Watchdog,
};

use pac::interrupt;

use pico_seven_seg_2::{ElapsedFlag, Led, LedWithPin, SevenSegmentLed2Digits};
use seven_segment::SevenSegmentPins;

type SharedObject<T> = Mutex<RefCell<Option<T>>>;

static TIMER: SharedObject<Timer> = Mutex::new(RefCell::new(None));
static ALARM_0: SharedObject<Alarm0> = Mutex::new(RefCell::new(None));

type RPOutputPin<I> = Pin<I, PushPullOutput>;
static SEVEN_SEG_LEDS: SharedObject<
    SevenSegmentLed2Digits<
        RPOutputPin<Gpio22>,
        RPOutputPin<Gpio21>,
        RPOutputPin<Gpio20>,
        RPOutputPin<Gpio19>,
        RPOutputPin<Gpio18>,
        RPOutputPin<Gpio17>,
        RPOutputPin<Gpio16>,
        RPOutputPin<Gpio15>,
        RPOutputPin<Gpio14>,
    >,
> = Mutex::new(RefCell::new(None));

static ELAPSED_125MS_FLAG: SharedObject<ElapsedFlag> = Mutex::new(RefCell::new(None));

static TIMER_INTERVAL_US: u32 = 250;

#[derive(Debug)]
enum AppState {
    ChangeNumber,
    LedTurnOff,
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = LedWithPin::new(pins.led.into_push_pull_output());

    let seven_seg_pins = SevenSegmentPins {
        a: pins.gpio22.into_push_pull_output(),
        b: pins.gpio21.into_push_pull_output(),
        c: pins.gpio20.into_push_pull_output(),
        d: pins.gpio19.into_push_pull_output(),
        e: pins.gpio18.into_push_pull_output(),
        f: pins.gpio17.into_push_pull_output(),
        g: pins.gpio16.into_push_pull_output(),
    };
    let seven_seg = seven_seg_pins.with_common_cathode();

    let mut seven_seg_drive_pin1 = pins.gpio15.into_push_pull_output();
    let mut seven_seg_drive_pin2 = pins.gpio14.into_push_pull_output();

    seven_seg_drive_pin1.set_low().unwrap();
    seven_seg_drive_pin2.set_low().unwrap();

    let mut seven_seg_leds =
        SevenSegmentLed2Digits::new(seven_seg, seven_seg_drive_pin1, seven_seg_drive_pin2);
    let mut count = 0u8;

    seven_seg_leds.set_number(count);
    led.turn_on();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm_0 = timer.alarm_0().unwrap();
    alarm_0.schedule(TIMER_INTERVAL_US.microseconds()).unwrap();
    alarm_0.enable_interrupt(&mut timer);

    // 割り込みハンドラと変数を共有する
    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
        ALARM_0.borrow(cs).replace(Some(alarm_0));
        SEVEN_SEG_LEDS.borrow(cs).replace(Some(seven_seg_leds));
        ELAPSED_125MS_FLAG
            .borrow(cs)
            .replace(Some(ElapsedFlag::new()));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut next_state = AppState::LedTurnOff;
    loop {
        if cortex_m::interrupt::free(|cs| {
            ELAPSED_125MS_FLAG
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .clear_if_set()
        }) {
            match next_state {
                AppState::ChangeNumber => {
                    if count >= 99 {
                        count = 0;
                    } else {
                        count += 1;
                    }

                    cortex_m::interrupt::free(|cs| {
                        let mut seven_seg_leds_shared = SEVEN_SEG_LEDS.borrow(cs).borrow_mut();
                        seven_seg_leds_shared.as_mut().unwrap().set_number(count);
                    });

                    led.turn_on();

                    next_state = AppState::LedTurnOff;
                }
                AppState::LedTurnOff => {
                    led.turn_off();
                    next_state = AppState::ChangeNumber;
                }
            }
        }
    }
}

/// タイマ割込み（250 usごとに実行）
#[interrupt]
fn TIMER_IRQ_0() {
    const COUNT_125MS_MAX: u32 = (125 * 1000 / 250) - 1;
    static COUNT_125MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

    cortex_m::interrupt::free(|cs| {
        let mut timer = TIMER.borrow(cs).borrow_mut();
        let mut alarm_0 = ALARM_0.borrow(cs).borrow_mut();
        alarm_0
            .as_mut()
            .unwrap()
            .clear_interrupt(timer.as_mut().unwrap());
        alarm_0
            .as_mut()
            .unwrap()
            .schedule(TIMER_INTERVAL_US.microseconds())
            .unwrap();

        let count_125ms_ref = COUNT_125MS.borrow(cs);
        let count_125ms_val = count_125ms_ref.get();
        if count_125ms_val >= COUNT_125MS_MAX {
            count_125ms_ref.set(0);

            let mut elapsed_125ms_flag = ELAPSED_125MS_FLAG.borrow(cs).borrow_mut();
            elapsed_125ms_flag.as_mut().unwrap().set();
        } else {
            count_125ms_ref.set(count_125ms_val + 1);
        }

        let mut seven_seg_leds = SEVEN_SEG_LEDS.borrow(cs).borrow_mut();
        seven_seg_leds.as_mut().unwrap().turn_on_next_digit();
    });
}
