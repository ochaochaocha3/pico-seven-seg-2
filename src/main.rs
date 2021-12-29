//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::{borrow::Borrow, cell::RefCell};

use cortex_m::interrupt;
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
    timer::Timer,
    watchdog::Watchdog,
};

use pico_seven_seg_2::{ElapsedFlag, SevenSegmentLeds};
use seven_segment::SevenSegmentPins;

type RPOutputPin<I> = Pin<I, PushPullOutput>;
static SEVEN_SEG_LEDS: Mutex<
    RefCell<
        Option<
            SevenSegmentLeds<
                RPOutputPin<Gpio22>,
                RPOutputPin<Gpio21>,
                RPOutputPin<Gpio20>,
                RPOutputPin<Gpio19>,
                RPOutputPin<Gpio18>,
                RPOutputPin<Gpio17>,
                RPOutputPin<Gpio16>,
                RPOutputPin<Gpio14>,
                RPOutputPin<Gpio15>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(Option::None));

static ELAPSED_1MS: Mutex<RefCell<Option<ElapsedFlag>>> = Mutex::new(RefCell::new(None));

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

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

    let mut seven_seg_drive_pin1 = pins.gpio14.into_push_pull_output();
    let mut seven_seg_drive_pin2 = pins.gpio15.into_push_pull_output();

    seven_seg_drive_pin1.set_low().unwrap();
    seven_seg_drive_pin2.set_low().unwrap();

    let seven_seg_leds =
        SevenSegmentLeds::new(seven_seg, seven_seg_drive_pin1, seven_seg_drive_pin2);

    interrupt::free(|cs| {
        SEVEN_SEG_LEDS.borrow(cs).replace(Some(seven_seg_leds));
        ELAPSED_1MS.borrow(cs).replace(Some(ElapsedFlag::new()))
    });

    let mut alarm_0 = Timer::new(pac.TIMER, &mut pac.RESETS).alarm_0().unwrap();
    alarm_0.schedule(250.microseconds()).unwrap();

    let mut count_ms = 0u32;
    loop {
        let elapsed_1ms = interrupt::free(|cs| {
            ELAPSED_1MS.borrow(cs).borrow().
        });
    }
}
