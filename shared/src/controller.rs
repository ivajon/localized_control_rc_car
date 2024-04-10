//! Defines a PID controller.

use core::{fmt::Debug, marker::PhantomData};

use arraydeque::ArrayDeque;

/// Provides a light weight interface for setting the output of a value
pub trait Channel<Error: Debug> {
    /// Sets the output value for the type.
    fn set(&mut self, value: i32) -> Result<(), Error>;

    /// Gets the latest measured refference.
    fn read(&mut self) -> Result<i32, Error>;
}

/// Enumerates the error cases for the controller.
#[derive(Debug)]
pub enum ControllerError<Error: Debug> {
    /// User tried to use the controller before assigning a control sequence.
    BufferEmpty,
    /// The value written to the channel caused some error.
    ChannelError(Error),
}

impl<Error: Debug> From<Error> for ControllerError<Error> {
    fn from(value: Error) -> Self {
        Self::ChannelError(value)
    }
}

/// This assumes that we have a i32 as data.
pub struct Pid<
    Error: Debug,
    Interface: Channel<Error>,
    const BUFFER: usize,
    const KP: i32,
    const KI: i32,
    const KD: i32,
    const TS: i32,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
> {
    err: PhantomData<Error>,
    out: Interface,
    refference: ArrayDeque<i32, BUFFER>,
    previous: i32,

    // This might need to be changed in to i64 to not cause errors.
    integral: i32,
}

/// Wraps the info about a specific time step in the control sequence.
#[derive(Debug)]
pub struct ControlInfo {
    /// The expected value.
    pub refference: i32,
    /// The actual value read from the [`Channel`].
    pub measured: i32,
    /// The actuation applied to the [`Channel`].
    pub actuation: i32,
}

impl<
        Error: Debug,
        Interface: Channel<Error>,
        const BUFFER: usize,
        const KP: i32,
        const KI: i32,
        const KD: i32,
        const TS: i32,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
    > Pid<Error, Interface, BUFFER, KP, KI, KD, TS, THRESHOLD_MAX, THRESHOLD_MIN, TIMESCALE>
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(channel: Interface) -> Self {
        Self {
            out: channel,
            err: PhantomData,
            refference: ArrayDeque::new(),
            previous: 0,
            integral: 0,
        }
    }

    /// Completely erases previous control signals.
    pub fn follow<I: IntoIterator<Item = i32>>(&mut self, values: I) {
        self.refference.clear();
        self.refference.extend(values.into_iter());
    }

    /// Extends the refference signal with new values.
    pub fn extend<I: IntoIterator<Item = i32>>(&mut self, values: I) {
        self.refference.extend(values.into_iter());
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successfull it returns the expected value and the read value.
    pub fn actuate(&mut self) -> Result<ControlInfo, ControllerError<Error>> {
        let target = match self.refference.pop_front() {
            Some(value) => value,
            None => return Err(ControllerError::BufferEmpty),
        };

        let actual = self.out.read()?;

        let error = target - actual;

        let p = error * KP;

        // Integral is approximated as a sum of discrete signals.
        let avg: i64 = self.previous as i64 + error as i64;
        self.integral += ((avg / 2) as i32) * TS / TIMESCALE;

        let i = self.integral * KI;

        // Compute the rate of change between previous timestep and this timestep.
        let d = KD * TIMESCALE * (error - self.previous) / TS;

        self.previous = error;

        let output = (p + i + d).max(THRESHOLD_MIN).min(THRESHOLD_MAX);

        self.out
            .set(output)
            .map_err(|err| ControllerError::ChannelError(err))?;

        Ok(ControlInfo {
            refference: target,
            measured: actual,
            actuation: output,
        })
    }
}

impl<Iter: Iterator<Item = i32>> Channel<()> for (Iter, i32) {
    fn set(&mut self, value: i32) -> Result<(), ()> {
        self.1 = value;
        Ok(())
    }
    fn read(&mut self) -> Result<i32, ()> {
        match self.0.next() {
            Some(value) => Ok(value),
            None => Err(()),
        }
    }
}

#[cfg(test)]
mod test {
    use super::Pid;

    #[test]
    fn test_p() {
        let channel = ([1, 2, 3, 5, 6].into_iter(), 0);
        let target = [6, 6, 6, 6, 6];
        let mut pid: Pid<(), _, 5, 2, 0, 0, 1, 100, 0, 1> = Pid::new(channel);
        pid.follow(target);

        while let Ok(actuation) = pid.actuate() {
            assert!(actuation.actuation == 2 * (actuation.refference - actuation.measured));
        }
    }

    #[test]
    fn test_pi() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, 2, 2, 1, 0, 1, 100, 0, 1> = Pid::new(channel);
        pid.follow(target);

        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.refference - actuation.measured) + 5 / 2);

        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation
                == 2 * (actuation.refference - actuation.measured) + (5 + 4) / 2 + 2
        );
    }

    #[test]
    fn test_pid() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, 2, 2, 1, 1, 1, 100, 0, 1> = Pid::new(channel);
        pid.follow(target);

        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.refference - actuation.measured) + 5 / 2 + 5);

        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation
                == 2 * (actuation.refference - actuation.measured) + (5 + 4) / 2 + 2 - 1
        );
    }
}
