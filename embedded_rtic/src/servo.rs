//! Defines a generic servo abstraction.

use defmt::trace;
use nrf52840_hal::{
    gpio::{Output, Pin, PushPull},
    pwm::{Channel, Instance, Pwm},
    time::U32Ext,
};
use shared::controller::Channel as ControllerChannel;

use self::sealed::Remap;
use crate::wrapper::{Degrees, Exti32};

/// Enumerates the errors that can occur when using the [`Servo`]
/// abstraction.
#[derive(Debug)]
pub enum Error {
    /// Thrown when the requested duty cycle is invalid for the
    /// [`Pwm`].
    InvalidDutyCycle(u16),

    /// Thrown when the requested angle is not achievable.
    InvalidAngle(i32),
}

/// Our neat little wrapper around the servo.
pub struct Servo<PWM: Instance> {
    pwm: Pwm<PWM>,
}

impl<PWM: Instance> Servo<PWM> {
    /// Maximum duty cycle for the [`Pwm`].
    const MAXIMUM_DUTY_CYCLE: u16 = 2500;
    /// The maximum angle for steering actuation.
    const MAX_ANGLE: i32 = 15;
    /// The minimum angle for steering actuation.
    const MIN_ANGLE: i32 = -15;
    /// The steering seems to be offset by some constant factor.
    const STEERING_ERROR: i32 = 0;

    /// Creates a new servo from a [`Pwm`] [`Instance`] and the associated
    /// [`Pin`].
    ///
    /// This allows use of any [`Pwm`] peripheral and [`Pin`] on the board.
    pub fn new(pwm: PWM, pin: Pin<Output<PushPull>>) -> Self {
        let pwm = Pwm::new(pwm);

        // Pwm configuration.
        //
        // Sets the freq to 50 Hz and binds
        // 100 -> fully extended to the left.
        // 275 -> fully extended to the right.
        {
            pwm.set_prescaler(nrf52840_hal::pwm::Prescaler::Div128);
            pwm.set_period(50.hz()).set_output_pin(Channel::C0, pin);
            pwm.set_max_duty(Self::MAXIMUM_DUTY_CYCLE);
            pwm.enable();
        }

        let mut ret = Self { pwm };

        // Center the servo after initiation.
        ret.angle(0.deg()).unwrap();
        ret
    }

    /// Sets the angle of the servo.
    ///
    /// The angle has to be between [`MIN_ANGLE`](Self::MIN_ANGLE) to
    /// [`MAX_ANGLE`](Self::MAX_ANGLE) degrees, this is simply because
    /// this is the maximum actuation distance for the turning.
    pub fn angle<T: Into<Degrees>>(&mut self, angle: T) -> Result<(), Error> {
        let angle = angle.into();
        let value = angle.consume();
        trace!("Setting angle to {}", value);
        // value += Self::STEERING_ERROR;
        if value < Self::MIN_ANGLE /* + Self::STEERING_ERROR */ || value > Self::MAX_ANGLE {
            return Err(Error::InvalidAngle(value + Self::STEERING_ERROR));
        }

        let value: i16 = (value + Self::STEERING_ERROR)
            .try_into()
            .map_err(|_err| Error::InvalidAngle(value))?;

        // If rust ever expands the const generics functionality we could
        // ensure that this is valid at compile-time but for now we have to
        // unwrap here.
        let value = value
            // .remap::<-60, 60, 100, 275>()
            .remap::<-60, 60, 0, 325>()
            .expect("Servo Remap Broken");

        // Dirty inversion.
        self.pwm
            .set_duty_on(Channel::C0, Self::MAXIMUM_DUTY_CYCLE - value);
        Ok(())
    }
}

impl<I: Instance> ControllerChannel<Error> for Servo<I> {
    type Output = f32;

    fn set(&mut self, value: Self::Output) -> Result<(), Error> {
        self.angle((value as i32).deg())
    }
}

mod sealed {

    pub trait Remap<Target>
    where
        Self: Sized,
    {
        /// Remaps the value in to the new specified range.
        ///
        /// Using floating point this would be equivalent to
        ///
        /// ```no_run
        /// let mut ret = (value - OLD_MIN)/(OLD_MAX - OLD_MIN);
        /// ret = ret*(NEW_MAX-NEW_MIN) + NEW_MIN;
        /// ```
        ///
        /// If the operation is not successful the variable `value` will be
        /// returned as an error.
        fn remap<const OLD_MIN: i32, const OLD_MAX: i32, const NEW_MIN: i32, const NEW_MAX: i32>(
            self,
        ) -> Result<Target, Self>;
    }

    impl Remap<u16> for i16 {
        fn remap<const OLD_MIN: i32, const OLD_MAX: i32, const NEW_MIN: i32, const NEW_MAX: i32>(
            self,
        ) -> Result<u16, Self> {
            assert!(OLD_MIN <= OLD_MAX, "Old min has to be less than Old max");
            assert!(NEW_MIN <= NEW_MAX, "New min has to be less than New max");
            assert!(
                OLD_MAX <= i16::MAX as i32,
                "Old max cannot be larger than i16 max"
            );
            assert!(
                OLD_MIN >= i16::MIN as i32,
                "Old min cannot be less than i16 min"
            );

            let (old_range, new_range) = (OLD_MAX - OLD_MIN, NEW_MAX - NEW_MIN);

            let value: i32 = self.into();
            let mut normalized = value - OLD_MIN;

            // Scale ut to avoid decimals mid calculation.
            normalized *= old_range.max(new_range);

            normalized /= old_range;

            normalized *= new_range;

            normalized /= new_range.max(old_range);

            (normalized + NEW_MIN).try_into().map_err(|_err| self)
        }
    }
}
