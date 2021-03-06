#![no_std]

use core::fmt::Debug;

use embedded_hal::digital::v2::OutputPin;
use seven_segment::{Cathode, SevenSegment};

#[derive(Debug)]
pub struct ElapsedFlag(bool);

impl ElapsedFlag {
    pub fn new() -> Self {
        Self(false)
    }

    pub fn is_set(&self) -> bool {
        self.0
    }

    pub fn set(&mut self) {
        self.0 = true;
    }

    pub fn clear(&mut self) {
        self.0 = false;
    }

    pub fn clear_if_set(&mut self) -> bool {
        if self.is_set() {
            self.clear();
            true
        } else {
            false
        }
    }
}

#[derive(Debug)]
pub struct LedWithPin<T> {
    pin: T,
}

impl<T> LedWithPin<T>
where
    T: OutputPin,
    T::Error: Debug,
{
    pub fn new(pin: T) -> Self {
        Self { pin }
    }
}

pub trait Led {
    fn turn_on(&mut self);
    fn turn_off(&mut self);
}

impl<T> Led for LedWithPin<T>
where
    T: OutputPin,
    T::Error: Debug,
{
    fn turn_on(&mut self) {
        self.pin.set_high().unwrap();
    }

    fn turn_off(&mut self) {
        self.pin.set_low().unwrap();
    }
}

#[derive(Debug)]
struct SevenSegDigitWithPin<T> {
    drive_pin: T,
    number: u8,
}

impl<T> SevenSegDigitWithPin<T>
where
    T: OutputPin,
    T::Error: Debug,
{
    pub fn new(drive_pin: T) -> Self {
        Self {
            drive_pin,
            number: 0u8,
        }
    }
}

trait SevenSegDigit {
    fn get_number(&self) -> u8;
    fn set_number(&mut self, number: u8);
    fn turn_on(&mut self);
    fn turn_off(&mut self);
}

impl<T> SevenSegDigit for SevenSegDigitWithPin<T>
where
    T: OutputPin,
    T::Error: Debug,
{
    fn get_number(&self) -> u8 {
        self.number
    }

    fn set_number(&mut self, number: u8) {
        self.number = number;
    }

    fn turn_on(&mut self) {
        self.drive_pin.set_high().unwrap();
    }

    fn turn_off(&mut self) {
        self.drive_pin.set_low().unwrap();
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SevenSegDigitIndex {
    Digit1,
    Digit2,
}

pub struct SevenSegmentLed2Digits<A, B, C, D, E, F, G, DrivePin1, DrivePin2> {
    seven_seg: SevenSegment<A, B, C, D, E, F, G, Cathode>,
    digit_1: SevenSegDigitWithPin<DrivePin1>,
    digit_2: SevenSegDigitWithPin<DrivePin2>,
    current_digit_index: SevenSegDigitIndex,
}

impl<A, B, C, D, E, F, G, DrivePin1, DrivePin2>
    SevenSegmentLed2Digits<A, B, C, D, E, F, G, DrivePin1, DrivePin2>
where
    A: OutputPin,
    A::Error: Debug,
    B: OutputPin<Error = A::Error>,
    C: OutputPin<Error = A::Error>,
    D: OutputPin<Error = A::Error>,
    E: OutputPin<Error = A::Error>,
    F: OutputPin<Error = A::Error>,
    G: OutputPin<Error = A::Error>,
    DrivePin1: OutputPin,
    DrivePin1::Error: Debug,
    DrivePin2: OutputPin<Error = DrivePin1::Error>,
{
    pub fn new(
        seven_seg: SevenSegment<A, B, C, D, E, F, G, Cathode>,
        drive_pin_1: DrivePin1,
        drive_pin_2: DrivePin2,
    ) -> Self {
        Self {
            seven_seg,
            digit_1: SevenSegDigitWithPin::new(drive_pin_1),
            digit_2: SevenSegDigitWithPin::new(drive_pin_2),
            current_digit_index: SevenSegDigitIndex::Digit1,
        }
    }

    pub fn set_number(&mut self, number: u8) {
        self.digit_1.set_number(number / 10);
        self.digit_2.set_number(number % 10);
    }

    pub fn turn_off_all_digits(&mut self) {
        self.digit_1.turn_off();
        self.digit_2.turn_off();
    }

    pub fn turn_on_next_digit(&mut self) {
        let next_digit_index = match self.current_digit_index {
            SevenSegDigitIndex::Digit1 => SevenSegDigitIndex::Digit2,
            SevenSegDigitIndex::Digit2 => SevenSegDigitIndex::Digit1,
        };

        self.turn_off_all_digits();

        let next_digit_number = self.get_digit(next_digit_index).get_number();
        self.seven_seg.set(next_digit_number).unwrap();
        self.get_mutable_digit(next_digit_index).turn_on();

        self.current_digit_index = next_digit_index;
    }

    fn get_digit(&self, index: SevenSegDigitIndex) -> &dyn SevenSegDigit {
        match index {
            SevenSegDigitIndex::Digit1 => &self.digit_1 as &dyn SevenSegDigit,
            SevenSegDigitIndex::Digit2 => &self.digit_2 as &dyn SevenSegDigit,
        }
    }

    fn get_mutable_digit(&mut self, index: SevenSegDigitIndex) -> &mut dyn SevenSegDigit {
        match index {
            SevenSegDigitIndex::Digit1 => &mut self.digit_1 as &mut dyn SevenSegDigit,
            SevenSegDigitIndex::Digit2 => &mut self.digit_2 as &mut dyn SevenSegDigit,
        }
    }
}
