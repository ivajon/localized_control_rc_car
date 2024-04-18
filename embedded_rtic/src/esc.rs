//! Defines a generic [`Esc`] abstraction, this allows us to
//! easily specify the speed that the car should run at.

use defmt::{trace, warn};
use nrf52840_hal::{
    gpio::{Output, Pin, PushPull},
    pwm::{Channel, Instance, Pwm},
    time::U32Ext,
};

use self::sealed::Remap;

/// Enumerates the errors that can occur when using the [`Esc`]
/// abstraction.
#[derive(Debug)]
pub enum Error {
    /// Thrown when the requested duty cycle is invalid for the
    /// pwm.
    InvalidDutyCycle(u16),

    /// Thrown when the requested velocity is not achievable.
    InvalidVelocity(i32),
}

// ESC 1000-2000 uS 1000 is stopped or reverse and 2000 is 100% speed

/// Defines an easy to use abstraction over the Electronic speed controller.
pub struct Esc<PWM: Instance> {
    pwm: Pwm<PWM>,
}

impl<PWM: Instance> Esc<PWM> {
    /// The maximum duty cycle.
    const MAXIMUM_DUTY_CYCLE: u16 = 2500;
    /// The maximum velocity.
    const MAX_VELOCITY: i32 = 1000;
    /// The minimum velocity.
    const MIN_VELOCITY: i32 = -1000;

    /// Instantiates a new [`Esc`] that is controlled over PWM.
    pub fn new(pwm: PWM, pin: Pin<Output<PushPull>>) -> Self {
        trace!(
            "Instantiating a new ESC on pwm : {:?} and pin : {:?}",
            pwm,
            pin
        );
        let pwm = Pwm::new(pwm);

        // PWM configuration, sets the period to 50 Hz and specifies a duty cycle
        // that allows us to use the desired values.
        //
        // 125 -> min velocity.
        // 250 -> max velocity.
        {
            pwm.set_prescaler(nrf52840_hal::pwm::Prescaler::Div128);
            pwm.set_period(50.hz()).set_output_pin(Channel::C0, pin);
            pwm.set_max_duty(Self::MAXIMUM_DUTY_CYCLE);
            pwm.enable();
        }

        let period = pwm.period();

        trace!("Esc instantiated : with period {:?} Hz", period.0);
        let mut ret = Self { pwm };

        ret.speed(0).unwrap();
        ret
    }

    /// Sets the angle of the servo.
    pub fn speed(&mut self, velocity: i32) -> Result<(), Error> {
        trace!("Setting the speed to {:?}", velocity);

        // A few safeguards.
        if velocity > Self::MAX_VELOCITY || velocity < Self::MIN_VELOCITY {
            return Err(Error::InvalidVelocity(velocity));
        }

        if velocity > -30 && velocity < 15 {
            warn!("Flaky esc might cause problems around the origin, please use larger control signals.");
        }

        let value: i16 = velocity
            .try_into()
            .map_err(|_err| Error::InvalidVelocity(velocity))?;

        let value = value
            .remap::<-1000, 1000, 125, 250>()
            .expect("Remap is broken");

        // Dirty inversion.
        self.pwm
            .set_duty_on(Channel::C0, Self::MAXIMUM_DUTY_CYCLE - value);
        Ok(())
    }
}

impl<PWM: Instance> shared::controller::Channel<Error> for Esc<PWM> {
    type Output = f32;

    fn set(&mut self, value: Self::Output) -> Result<(), Error> {
        trace!("Setting vel to {:?}", value);
        self.speed(value as i32)
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
        #[inline(always)]
        fn remap<const OLD_MIN: i32, const OLD_MAX: i32, const NEW_MIN: i32, const NEW_MAX: i32>(
            self,
        ) -> Result<u16, Self> {
            // The asserts are optimized out during release builds.
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

            // Scale to avoid decimals mid calculation.
            normalized *= old_range.max(new_range);

            normalized /= old_range;

            normalized *= new_range;

            normalized /= new_range.max(old_range);

            (normalized + NEW_MIN).try_into().map_err(|_err| self)
        }
    }
}
