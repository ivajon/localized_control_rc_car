//! Defines a generic servo abstraction

use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::info;
use nrf52840_hal::{
    gpio::{Output, Pin, PushPull},
    pwm::{Channel, Instance, Pwm},
    time::U32Ext,
};

use self::sealed::Remap;

/// Enumerates the errors that can occur when using the [`Servo`]
/// abstraction.
#[derive(Debug)]
pub enum Error {
    /// Thrown when the requested dutycyle is invalid for the
    /// pwm.
    InvalidDutyCycle(u16),

    /// Thrown when the requested velocity is not achievable.
    InvalidVelocity(i32),
}

// ESC 1000-2000 uS 1000 is stopped or reverse and 2000 is 100% speed

/// Our neat little wrapper around the esc.
pub struct Esc<PWM: Instance> {
    pwm: Pwm<PWM>,
}

impl<PWM: Instance> Esc<PWM> {
    // TODO! Set the actual max value here.
    /// MAX_VALUE for the servo is 2100 us, with a period of 250 hz.
    // const MAX_VALUE: <Controller as Pwm>::Duty = { ((0x7FFF as u32 * 512) / 1000) as u16 };
    /// MIN_VALUE for the servo is 2100 us, with a period of 250 hz.
    // const MIN_VALUE: <Controller as Pwm>::Duty = { ((0x7FFF as u32 * 225) / 1000)
    // as u16 };

    pub fn new(pwm: PWM, pin: Pin<Output<PushPull>>) -> Self {
        let pwm = Pwm::new(pwm);
        pwm.set_prescaler(nrf52840_hal::pwm::Prescaler::Div32);
        pwm.set_period(500.hz()).set_output_pin(Channel::C0, pin);
        pwm.set_max_duty(1000);
        // Set it to 250 hz in accordance to
        // https://www.blue-bird-model.com/index.php?/products_detail/310.htm
        pwm.enable();

        let period = pwm.get_period();
        info!("Esc instantianted : with period {:?} Hz", period.0);
        Self { pwm }
    }
}

impl<PWM: Instance> Esc<PWM> {
    /// Sets the angle of the servo.
    pub fn speed(&mut self, velocity: i32) -> Result<(), Error> {
        if velocity > 1000 || velocity < -1000 {
            return Err(Error::InvalidVelocity(velocity));
        }

        // Reduce granularity around the origin.

        let value: i16 = velocity
            .try_into()
            .map_err(|_err| Error::InvalidVelocity(velocity))?;

        let value = value
            .remap::<-1000, 1000,
        // { ((0x7FFF as u32 * 225) / 1000) as i32 }, 
            500,
        // { ((0x7FFF as u32 * 512) / 1000) as i32 }>()
            1000>()
            .expect("Remap is broken");

        self.pwm.set_duty(Channel::C0, value);
        let real_value = self.pwm.get_duty(Channel::C0);
        info!("Set the duty cycle to {:?}", real_value);
        info!("Expected the duty cycle to be {:?}", value);
        let period = self.pwm.get_period().0;
        info!("With period of {:?}", period);
        Ok(())
    }
}

mod sealed {

    pub trait Remap<Target>
    where
        Self: Sized,
    {
        /// Remaps the value in to the new specified range.
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
