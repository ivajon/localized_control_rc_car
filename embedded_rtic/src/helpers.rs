//! Provides a few helper functions.
#![allow(async_fn_in_trait)]

use core::ops::Sub;

use arraydeque::{ArrayDeque, Wrapping};
use embedded_hal::digital::{ErrorType, OutputPin};
use nrf52840_hal::gpio::{Output, Pin, PushPull};
use rtic_monotonics::nrf::timer::{ExtU64, Timer0};

use super::car::constants::SMOOTHING;

/// Computes sum of the smoothing window with a recency bias.
pub fn sum(window: &ArrayDeque<f32, SMOOTHING, Wrapping>) -> f32 {
    let sum = (0..(window.len()))
        .map(|idx| (idx as f32) / { SMOOTHING as f32 })
        .sum::<f32>();
    window
        .iter()
        .enumerate()
        .map(|(idx, value)| (idx as f32) / { SMOOTHING as f32 } * value)
        .sum::<f32>()
        / sum
}

/// Denotes the absolute value of a variable.
pub trait Abs {
    /// Denotes the absolute value of a variable.
    fn abs(self) -> Self;
}
/// Rejects outliers in the data.
pub trait OutlierRejection: PartialOrd + Sized + Sub<Self, Output = Self> + Clone + Abs {
    /// Rejects the value if it is an outlier.
    fn reject<'to_continue, const VOTING_THRESHOLD: usize>(
        &self,
        previous_value: &mut Self,
        outlier_counter: &mut usize,
        threshold: &Self,
    ) -> bool {
        let value = (self.clone() - previous_value.clone()).abs();

        if value > *threshold && *outlier_counter < VOTING_THRESHOLD {
            *outlier_counter += 1;
            return true;
        }
        *outlier_counter = 0;
        *previous_value = self.clone();
        false
    }
}

/// Denotes that the type can send a trigger pulse.
pub trait Trigger {
    /// The error that can be thrown.
    type Error;
    /// Sends a pulse on that channel.
    async fn trigger(&mut self) -> Result<(), Self::Error>;
}

impl Trigger for Pin<Output<PushPull>> {
    type Error = <Self as ErrorType>::Error;

    async fn trigger(&mut self) -> Result<(), <Self as ErrorType>::Error> {
        self.set_high()?;

        Timer0::delay(25.micros()).await;

        self.set_low()?;
        Ok(())
    }
}

impl Trigger for (Pin<Output<PushPull>>, Pin<Output<PushPull>>) {
    type Error = <Pin<Output<PushPull>> as ErrorType>::Error;

    async fn trigger(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()?;
        self.1.set_high()?;

        Timer0::delay(25.micros()).await;

        self.0.set_low()?;
        self.1.set_low()?;
        Ok(())
    }
}

impl Abs for f32 {
    fn abs(self) -> Self {
        if self < 0. {
            return -self;
        }
        self
    }
}
impl OutlierRejection for f32 {}
