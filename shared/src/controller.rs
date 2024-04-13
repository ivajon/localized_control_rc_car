//! Defines a PID controller.

use core::{
    convert::Infallible,
    fmt::Debug,
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Mul, Sub},
};

use arraydeque::ArrayDeque;

/// Provides a light weight interface for setting the output of a value
pub trait Channel<Error: Debug> {
    /// The value type that the channel accepts.
    type Output: Sized;
    /// Sets the output value for the type.
    fn set(&mut self, value: Self::Output) -> Result<(), Error>;
}

pub trait DoubleSize {
    /// A type with double the size of self.
    type Ret: Sized;
    /// Potential errors that can occur.
    type Error: Debug;
    /// Doubles the size of self.
    fn double_size(self) -> Self::Ret;

    /// Halves the size of the value.
    ///
    /// Returns error if the value does not fit.
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized;
}

/// Enumerates the error cases for the controller.
#[derive(Debug)]
pub enum ControllerError<Error: Debug, ConversionError: Debug> {
    /// User tried to use the controller before assigning a control sequence.
    BufferEmpty,
    /// The value written to the channel caused some error.
    ChannelError(Error),
    /// Thrown when a conversion is non successful.
    ConversionError(ConversionError),
    /// Value to large.
    ///
    /// This is thrown when a value is to large to fit in half the size.
    ValueToLarge,
}

/// This assumes that we have a i32 as data.
pub struct Pid<
    Error: Debug,
    Interface: Channel<Error>,
    Output: Sized,
    const BUFFER: usize,
    const KP: i32,
    const KI: i32,
    const KD: i32,
    const TS: i32,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
    const FIXED_POINT: u32,
> {
    err: PhantomData<Error>,
    out: Interface,
    reference: ArrayDeque<Output, BUFFER>,
    previous: Output,
    // This might need to be changed in to i64 to not cause errors.
    integral: Output,
    measurement: (u32, Output),
    previous_actuation: Output,
}

/// Wraps the info about a specific time step in the control sequence.
#[derive(Debug)]
pub struct ControlInfo<Output: Sized> {
    /// The expected value.
    pub reference: Output,
    /// The actual value read from the [`Channel`].
    pub measured: Output,
    /// The actuation applied to the [`Channel`].
    pub actuation: Output,
    /// The contribution from the p term.
    pub p: Output,
    /// The contribution from the i term.
    pub i: Output,
    /// The contribution from the d term.
    pub d: Output,
    /// The output pre-threshold.
    pub pre_threshold: Output,
}

impl<
        Error: Debug,
        Interface: Channel<Error, Output = Output>,
        Output: Sized,
        ConversionError: Debug,
        const BUFFER: usize,
        const KP: i32,
        const KI: i32,
        const KD: i32,
        const TS: i32,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
        const FIXED_POINT: u32,
    >
    Pid<
        Error,
        Interface,
        Output,
        BUFFER,
        KP,
        KI,
        KD,
        TS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    >
where
    Output: Sub<Output, Output = Output>
        + Mul<Output, Output = Output>
        + Div<Output, Output = Output>
        + DoubleSize
        + Default
        + AddAssign<Output>
        + Add<Output, Output = Output>
        + Copy
        + PartialOrd
        + CmpExt,
    Output::Ret: Add<Output::Ret, Output = Output::Ret> + Div<Output::Ret, Output = Output::Ret>,
    i32: Convert<Output, Error = ConversionError>,
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(channel: Interface) -> Self {
        Self {
            out: channel,
            err: PhantomData,
            reference: ArrayDeque::new(),
            previous: Output::default(),
            integral: Output::default(),
            measurement: (0, Output::default()),
            previous_actuation: Output::default(),
        }
    }

    /// Completely erases previous control signals.
    pub fn follow<I: IntoIterator<Item = Output>>(&mut self, values: I) {
        self.reference.clear();
        self.reference.extend(values);
    }

    /// Extends the reference signal with new values.
    pub fn extend<I: IntoIterator<Item = Output>>(&mut self, values: I) {
        self.reference.extend(values);
    }

    /// Registers the most recent measurement.
    pub fn register_measurement(&mut self, value: Output, time_step: u32) {
        self.measurement = (time_step, value);
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    pub fn actuate_rate_limited(
        &mut self,
        rate_limit: Output,
    ) -> Result<ControlInfo<Output>, ControllerError<Error, ConversionError>>
    where
        <Output as DoubleSize>::Ret: Debug + Copy,
        Output: Debug + Copy,
    {
        let mut output_pre_rate_limit = self.compute_output()?;

        let output = match (
            output_pre_rate_limit.actuation > (self.previous_actuation + rate_limit),
            output_pre_rate_limit.actuation < (self.previous_actuation - rate_limit),
        ) {
            (true, _) => self.previous_actuation + rate_limit,
            (_, true) => self.previous_actuation - rate_limit,
            (_, _) => output_pre_rate_limit.actuation,
        };

        self.out
            .set(output)
            .map_err(|err| ControllerError::ChannelError(err))?;
        self.previous_actuation = output;
        output_pre_rate_limit.actuation = output;
        Ok(output_pre_rate_limit)
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    pub fn actuate(
        &mut self,
    ) -> Result<ControlInfo<Output>, ControllerError<Error, ConversionError>>
    where
        <Output as DoubleSize>::Ret: Debug + Copy,
        Output: Debug + Copy,
    {
        let output = self.compute_output()?;
        self.out
            .set(output.actuation)
            .map_err(|err| ControllerError::ChannelError(err))?;
        self.previous_actuation = output.actuation;

        Ok(output)
    }
    fn compute_output(
        &mut self,
    ) -> Result<ControlInfo<Output>, ControllerError<Error, ConversionError>> {
        let target: Output = match self.reference.pop_front() {
            Some(value) => value,
            None => return Err(ControllerError::BufferEmpty),
        };

        let kp: Output = KP.convert()?;
        let ki: Output = KI.convert()?;
        let kd: Output = KD.convert()?;

        let time_scale: Output = TIMESCALE.convert()?;
        let ts: Output = TS.convert()?;

        let threshold_min = THRESHOLD_MIN.convert()?;
        let threshold_max = THRESHOLD_MAX.convert()?;

        let fixed_point = { 10i32.pow(FIXED_POINT) }.convert()?;

        let actual: Output = self.measurement.1;

        let error = target - actual;

        let p = error * kp;

        // Integral is approximated as a sum of discrete signals.
        let avg: Output::Ret = self.previous.double_size() + error.double_size();
        let two: Output = 2.convert()?;
        let two = two.double_size();
        self.integral +=
            (Output::half_size(avg / two).map_err(|_| ControllerError::ValueToLarge)?) * ts
                / time_scale;
        let i = self.integral * ki;

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous) / ts;

        self.previous = error;

        let output = ((p + i + d) / fixed_point)
            .max(threshold_min)
            .min(threshold_max);

        Ok(ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        })
    }
}

#[macro_export]
/// Makes the instantiation of a new [`PID`](Pid) controller a bit more readable.
///
/// Syntax :
///
/// ```rust
///   use shared::controller::prelude::*;
///   let channel = 0f32;
///   let target = [6f32, 6f32];
///   let mut pid = pid!(
///     buffer_size:2, // This is optional, leaving it out will generate a buffer of size 1.
///     kp:2,
///     ki:1,
///     kd:1,
///     sample_time:1,
///     threshold_max:100,
///     threshold_min:0,
///     time_scale:1,
///     fixed_point:0,
///     output:channel
///   );
///   pid.follow(target);
///
///   pid.register_measurement(1f32, 0);
///   let actuation = pid.actuate().unwrap();
///   let expected = 2f32 * (actuation.reference - actuation.measured) + 5f32 / 2f32 + 5f32;
///   assert!(actuation.actuation == expected);
///
///   pid.register_measurement(2f32, 1);
///   let actuation = pid.actuate().unwrap();
///   let expected =
///     2f32 * (actuation.reference - actuation.measured) + (5f32 + 4f32) / 2f32 + 2.5f32 - 1f32;
///   // Accumulated sum + average from previous time step to the current time step.
///   assert!(actuation.actuation == expected);
/// ```
macro_rules! pid {
    (
        // Error,
        // Interface,
        // Output,
        buffer_size:$buffer:literal,
        kp:$kp:literal,
        ki:$ki:literal,
        kd:$kd:literal,
        sample_time:$ts:literal,
        threshold_max:$max:literal,
        threshold_min:$min:literal,
        time_scale:$time_scale:literal,
        fixed_point:$fixed_point:literal,
        output:$channel:tt
    ) => {{
        let ret: Pid<_, _, _, $buffer, $kp, $ki, $kd, $ts, $max, $min, $time_scale, $fixed_point> =
            Pid::new($channel);
        ret
    }};
    (
        // Error,
        // Interface,
        // Output,
        kp:$kp:literal,
        ki:$ki:literal,
        kd:$kd:literal,
        sample_time:$ts:literal,
        threshold_max:$max:literal,
        threshold_min:$min:literal,
        time_scale:$time_scale:literal,
        fixed_point:$fixed_point:literal,
        output:$channel:tt
    ) => {{
        let ret: Pid<_, _, _, 1, $kp, $ki, $kd, $ts, $max, $min, $time_scale, $fixed_point> =
            Pid::new($channel);
        ret
    }};
}

pub mod prelude {

    impl Channel<()> for f32 {
        type Output = f32;
        fn set(&mut self, value: f32) -> Result<(), ()> {
            *self = value;
            Ok(())
        }
    }
    pub use super::Channel;
    pub use super::Pid;
    pub use crate::pid;
}

mod sealed {
    use core::convert::Infallible;
    use core::fmt::Debug;

    use super::ControllerError;

    /// Converts a value in to the destination type if possible.
    pub trait Convert<Dest: Sized> {
        /// The error value returned.
        type Error: core::fmt::Debug;
        /// Converts the value in to the destination type.
        fn convert(self) -> Result<Dest, Self::Error>;
    }

    impl Convert<i32> for i32 {
        type Error = Infallible;
        fn convert(self) -> Result<i32, Self::Error> {
            Ok(self)
        }
    }

    impl Convert<f32> for i32 {
        type Error = Infallible;
        fn convert(self) -> Result<f32, Self::Error> {
            Ok(self as f32)
        }
    }
    pub trait CmpExt {
        fn max(self, other: Self) -> Self
        where
            Self: PartialOrd<Self> + Sized,
        {
            match self > other {
                true => self,
                false => other,
            }
        }
        fn min(self, other: Self) -> Self
        where
            Self: PartialOrd<Self> + Sized,
        {
            match self < other {
                true => self,
                false => other,
            }
        }
    }

    macro_rules! ext {
        ($($ty:ident),*) => {
            $(
                impl CmpExt for $ty {}
            )*
        };
    }
    ext!(i64, i32, i16, i8, u64, u32, u16, u8, f64, f32);

    impl<Error: Debug, ConversionError: Debug> From<ConversionError>
        for ControllerError<Error, ConversionError>
    {
        fn from(value: ConversionError) -> Self {
            Self::ConversionError(value)
        }
    }
}
use sealed::*;

impl DoubleSize for i32 {
    type Ret = i64;
    type Error = ();
    fn double_size(self) -> Self::Ret {
        self.into()
    }
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        value.try_into().map_err(|_| ())
    }
}

impl DoubleSize for f32 {
    type Ret = f64;
    type Error = Infallible;
    fn double_size(self) -> Self::Ret {
        self.into()
    }
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        Ok(value as f32)
    }
}

#[cfg(feature = "std")]
#[cfg(test)]
mod test {
    use super::prelude::*;

    impl<Iter: Iterator<Item = i32>> Channel<()> for (Iter, i32) {
        type Output = i32;
        fn set(&mut self, value: i32) -> Result<(), ()> {
            self.1 = value;
            Ok(())
        }
    }

    #[test]
    fn test_p() {
        let channel = ([1, 2, 3, 5, 6].into_iter(), 0);
        let target = [6, 6, 6, 6, 6];
        let mut pid: Pid<_, _, _, 5, 2, 0, 0, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        while let Ok(actuation) = pid.actuate() {
            assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured));
        }
    }

    #[test]
    fn test_pi() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 2, 1, 0, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured) + 5 / 2);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation == 2 * (actuation.reference - actuation.measured) + (5 + 4) / 2 + 2
        );
    }

    #[test]
    fn test_pid() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 2, 1, 1, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured) + 5 / 2 + 5);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation
                == 2 * (actuation.reference - actuation.measured) + (5 + 4) / 2 + 2 - 1
        );
    }

    #[test]
    fn test_pid_fixed_point() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 21, 21, 11, 1, 100, 0, 1, 1> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        let expected =
            (21 * (actuation.reference - actuation.measured) + 21 * (5 / 2) + 11 * 5) / 10;
        println!("Actuation : {:?} == {expected:?}", actuation);
        assert!(actuation.actuation == expected);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        let expected =
            (21 * (actuation.reference - actuation.measured) + 21 * ((5 + 4) / 2) + 11 * 2) / 10;
        println!("Actuation : {:?} == {expected:?}", actuation);
        // Accumulated sum + average from previous time step to the current time step.
        assert!(actuation.actuation == expected);
    }

    #[test]
    fn test_pid_float() {
        let channel = 0f32;
        let target = [6f32, 6f32];
        let mut pid = pid!(
            buffer_size:2,
            kp:2,
            ki:1,
            kd:1,
            sample_time:1,
            threshold_max:100,
            threshold_min:0,
            time_scale:1,
            fixed_point:0,
            output:channel
        );
        pid.follow(target);

        pid.register_measurement(1f32, 0);
        let actuation = pid.actuate().unwrap();
        let expected = 2f32 * (actuation.reference - actuation.measured) + 5f32 / 2f32 + 5f32;
        println!("Actuation : {:?} == {expected:?}", actuation);
        assert!(actuation.actuation == expected);

        pid.register_measurement(2f32, 1);
        let actuation = pid.actuate().unwrap();
        let expected =
            2f32 * (actuation.reference - actuation.measured) + (5f32 + 4f32) / 2f32 + 2.5f32
                - 1f32;
        println!("Actuation : {:?} == {expected:?}", actuation);
        // Accumulated sum + average from previous time step to the current time step.
        assert!(actuation.actuation == expected);
    }
}
