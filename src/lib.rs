//! A platform agnostic driver to interface the [`HC-SR04`][2] (ultrasonic distance sensor).
//!
//! This driver is built using [`embedded-hal`][1] traits.
//!
//! # Usage
//! Currently the sensor is implemented with interrupts in mind. This allows
//! us to accurately measure the pulse width, but at the cost of needing
//! external support for calling `HcSr04::update`.
//!
//! See the `examples` folder for further information.
//!
//! [1]: https://crates.io/crates/embedded-hal
//! [2]: http://www.micropik.com/PDF/HCSR04.pdf

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate void;
use void::Void;

extern crate embedded_hal as hal;
use hal::blocking::delay::DelayUs;
use hal::digital::OutputPin;

extern crate nb;
/// Publicly re-export `nb::Error` for easier usage down-stream
pub use nb::Error;

/// Wrapper for return value of sensor
#[derive(Debug, Copy, Clone)]
pub struct Distance(u32);

impl Distance {
    /// Get distance as centimeters.
    pub fn cm(&self) -> u32 {
        self.0 / 10
    }

    /// Get distance as millimeters.
    pub fn mm(&self) -> u32 {
        self.0
    }
}

/// Possible error returned by sensor.
#[derive(Debug, Copy, Clone)]
pub enum SensorError {
    /// Sensor is in wrong mode for update to take place.
    WrongMode,
}

/// Sensor Mode
enum Mode {
    /// Ready to start new measurement
    Idle,
    /// Sensor has been triggered, waiting for return
    Triggered,
    /// Measurement is in progress
    Measurement,
    /// Measurement is completed
    Completed,
    /// Measurement timed out
    Timedout,
}

/// HC-SR04 device
pub struct HcSr04<Pin, Delay> {
    /// Output pin to trigger sensor
    pin: Pin,
    /// Delay to wait on for sensor trigger
    delay: Delay,
    /// Internal mode of sensor
    mode: Mode,
    ///  Frequency (Hz) of clock ticks passed to capture method
    hz: u32,
    /// Echo rising edge timestamp
    t1: u32,
    /// Echo falling edge timestamp
    t2: u32,
}

impl<Pin, Delay> HcSr04<Pin, Delay>
where
    Pin: OutputPin,
    Delay: DelayUs<u32>,
{
    /// Create a new driver.
    ///
    /// # Arguments
    /// - `trigger` is the `OutputPin` connected to the sensor used to trigger
    /// the sensor into taking a measurement.
    /// - `delay` is a timer used to wait for the sensor to trigger.
    /// - `hz` frequency (Hz) of clock ticks passed to capture method
    pub fn new(trigger: Pin, delay: Delay, hz: u32) -> Self {
        // Ensure that our starting state is valid, if the pin was already
        // high then all internal methods would have to account for that
        // possibility, by defensively setting it low all internal states
        // can assume it is low.
        let mut trigger = trigger;
        trigger.set_low();
        HcSr04 {
            pin: trigger,
            delay: delay,
            mode: Mode::Idle,
            hz: hz,
            t1: 0,
            t2: 0,
        }
    }

    /// Trigger sensor reading and return the resulting `Distance`.
    ///
    /// This function uses [`nb::Error::WouldBlock`][1] to signal that a
    /// measurement is taking place. Once the measurement is taken, currently
    /// user responsibility of calling `update` on interrupt, the function
    /// will return the distance.
    ///
    /// # Note
    /// This method will not return another error except [`WouldBlock`][1].
    ///
    /// [1]: https://docs.rs/nb/0.1.1/nb/enum.Error.html
    pub fn distance(&mut self) -> nb::Result<Option<Distance>, Void> {
        match self.mode {
            // Start a new sensor measurement
            Mode::Idle => {
                self.trigger();
                Err(Error::WouldBlock)
            }
            // Waiting for the rising edge of echo pulse
            Mode::Triggered => Err(Error::WouldBlock),
            // Waiting for the falling edge of echo pulse
            Mode::Measurement => Err(Error::WouldBlock),
            // Measurement timedout
            Mode::Timedout => {
                self.reset();
                Ok(None)
            }
            // Measurement completed
            Mode::Completed => {
                // Divisions to avoid u32 overflow
                let d = self.t2.wrapping_sub(self.t1) * (171_605 / 1000) / (self.hz / 1000);
                self.reset();
                Ok(Some(Distance(d)))
            }
        }
    }

    /// Update the internal state noting that an external interrupt tracking
    /// echo response pulse has occurred.
    ///
    /// This function updates the internal state in response to an external
    /// interrupt caused by the sensor. This interface will be removed once
    /// abstract interrupt handling is supported.
    ///
    /// # Return
    /// This function will return `Result::Ok` if called in the correct
    /// state. Otherwise it will return `Result::Err`.
    pub fn capture(&mut self, ts: u32) -> Result<(), SensorError> {
        self.mode = match self.mode {
            Mode::Triggered => {
                self.t1 = ts;
                Mode::Measurement
            }
            Mode::Measurement => {
                self.t2 = ts;
                Mode::Completed
            }
            _ => return Err(SensorError::WrongMode),
        };
        Ok(())
    }

    /// Update the internal state noting that measurement has timed out.
    ///
    /// This function updates the internal state in response to an external
    /// interrupt caused by some guard timer or any other appropriate type
    /// of watcher.
    ///
    /// # Return
    /// This function will return `Result::Ok` if called in the correct
    /// state. Otherwise it will return `Result::Err`.
    pub fn timedout(&mut self) -> Result<(), SensorError> {
        self.mode = match self.mode {
            Mode::Triggered => Mode::Timedout,
            Mode::Measurement => Mode::Timedout,
            Mode::Completed => Mode::Completed,
            _ => return Err(SensorError::WrongMode),
        };
        Ok(())
    }

    /// Trigger sensor starting a measurement
    fn trigger(&mut self) {
        self.pin.set_high();
        self.delay.delay_us(10);
        self.pin.set_low();
        self.mode = Mode::Triggered;
    }

    /// Reset internal state
    fn reset(&mut self) {
        self.t1 = 0;
        self.t2 = 0;
        self.mode = Mode::Idle;
    }
}
