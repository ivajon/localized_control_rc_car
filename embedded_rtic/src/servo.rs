use nrf52840_hal::{
    prelude::_embedded_hal_Pwm as Pwm,
    pwm::Channel,
    time::{Hertz, U32Ext},
};

use self::sealed::{GetPwm, Remap};
use crate::wrapper::Degrees;

/// Enumerates the errors that can occur when using the [`Servo`]
/// abstraction.
pub enum Error<Controller: Pwm> {
    /// Thrown when the requested dutycyle is invalid for the
    /// pwm.
    InvalidDutyCycle(Controller::Duty),

    /// Thrown when the requested angle is not achievable.
    InvalidAngle(i32),
}

/// A generic pwm controlled servo.
pub trait ServoInterface<Controller>: GetPwm<Controller>
where
    Controller: Pwm<Time = Hertz, Channel = Channel>,
    Controller::Duty: core::cmp::PartialOrd,
{
    /// The maximum value that can be set.
    const MAX_VALUE: Controller::Duty;
    /// The minimum vaue that can be set.
    const MIN_VALUE: Controller::Duty;

    /// Instantiates a new motor controller.
    fn new(pwm: Controller, channel: Controller::Channel) -> Self;
}

// ESC 1000-2000 uS 1000 is stopped or reverse and 2000 is 100% speed

/// Our neat little wrapper around the servo.
pub struct Servo<PWM: Pwm> {
    pwm: PWM,
    channel: PWM::Channel,
}

impl<PWM: Pwm> sealed::GetPwm<PWM> for Servo<PWM> {
    fn get_pwm<'a>(&'a mut self) -> &'a mut PWM {
        &mut self.pwm
    }

    fn get_channel<'a>(&'a self) -> &'a <PWM as Pwm>::Channel {
        &self.channel
    }
}

impl<Controller> ServoInterface<Controller> for Servo<Controller>
where
    Controller: Pwm<Time = Hertz, Channel = Channel, Duty = u16>,
    Controller::Duty: core::cmp::PartialOrd,
{
    // TODO! Set the actual max value here.
    /// MAX_VALUE for the servo is 2100 us, with a period of 250 hz.
    const MAX_VALUE: <Controller as Pwm>::Duty = { ((0x7FFF as u32 * 512) / 1000) as u16 };
    /// MIN_VALUE for the servo is 2100 us, with a period of 250 hz.
    const MIN_VALUE: <Controller as Pwm>::Duty = { ((0x7FFF as u32 * 225) / 1000) as u16 };

    fn new(pwm: Controller, channel: Controller::Channel) -> Self {
        let mut pwm = pwm;
        // Set it to 250 hz in accordance to
        // https://www.blue-bird-model.com/index.php?/products_detail/310.htm
        pwm.set_period(250u32.hz());
        pwm.set_duty(channel, (Self::MAX_VALUE + Self::MIN_VALUE) / 2);

        Self { channel, pwm }
    }
}

impl<Controller> Servo<Controller>
where
    Controller: Pwm<Time = Hertz, Channel = Channel, Duty = u16>,
    Controller::Duty: core::cmp::PartialOrd,
    Self: ServoInterface<Controller>,
{
    /// Sets the angle of the servo.
    pub fn angle(&mut self, angle: Degrees) -> Result<(), Error<Controller>> {
        let mut value = angle.consume();
        if value < -60 || value > 60 {
            return Err(Error::InvalidAngle(value));
        }

        // Reduce granularity around the origin.
        if value > -5 && value < 5 {
            value = 0;
        }

        let value: i16 = value
            .try_into()
            .map_err(|_err| Error::InvalidAngle(value))?;

        let value = value
            .remap::<-60, 60,  { ((0x7FFF as u32 * 225) / 1000) as i32 }, { ((0x7FFF as u32 * 512) / 1000) as i32 }>()
            .expect("Remap is broken");

        self.pwm.set_duty(self.channel, value);
        Ok(())
    }
}

mod sealed {
    use super::Pwm;

    /// Returns a refference to the pwm interface.
    pub trait GetPwm<Controller: Pwm> {
        /// Returns a refference to the pwm interface.
        fn get_pwm<'a>(&'a mut self) -> &'a mut Controller;

        /// Returns the channel that the motor is connected to.
        fn get_channel<'a>(&'a self) -> &'a Controller::Channel;
    }

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
                OLD_MIN <= i16::MIN as i32,
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
