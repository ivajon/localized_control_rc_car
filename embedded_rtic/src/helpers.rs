//! Provides a few helper functions.
#![allow(async_fn_in_trait)]

use core::{
    future::Future,
    ops::{AsyncFn, Sub},
};

use arraydeque::{ArrayDeque, Wrapping};
use defmt::info;
use embedded_hal::digital::{ErrorType, OutputPin};
use nrf52840_hal::gpio::{Output, Pin, PushPull};
use rtic_monotonics::{
    nrf::timer::{ExtU64, Timer0},
    systick::fugit::Duration,
    Monotonic,
};
use rtic_sync::channel::{Receiver, Sender};

use super::car::{
    constants::{CAPACITY, SMOOTHING},
    wrappers::Mono,
};

/// Computes sum of the smoothing window with a recency bias.
pub fn sum(window: &ArrayDeque<f32, SMOOTHING, Wrapping>) -> f32 {
    // let sum = (0..(window.len()))
    // .map(|idx| (idx as f32) / { SMOOTHING as f32 })
    // .sum::<f32>();
    window
        .iter()
        .enumerate()
        .map(|(_idx, value)| /* (idx as f32) / { SMOOTHING as f32 } */ * value)
        .sum::<f32>()
        / window.len() as f32
}

/// Sends a message on the channel after the given time.
pub async fn timeout(
    channel: &mut Sender<'static, f32, CAPACITY>,
    kill_channel: &mut Receiver<'static, (), 1>,
    timeout: Duration<u64, 1, 1_000_000>,
) {
    let now = Mono::now();
    Mono::delay_until(now + timeout).await;
    if kill_channel.try_recv().is_ok() {
        return;
    }
    channel.send(0.).await.unwrap();
}

/// Measures the time required for the function call.
pub async fn tictoc<ReturnType: Future, F: AsyncFn() -> ReturnType>(function: F) -> ReturnType {
    let start_time = Mono::now();
    let ret = function().await;
    let end_time = Mono::now();
    let duration = end_time.checked_duration_since(start_time).unwrap();
    info!("Call took {:?}", duration.to_micros());
    ret
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

        Timer0::delay(50.micros()).await;

        self.set_low()?;
        Ok(())
    }
}

impl Trigger for (Pin<Output<PushPull>>, Pin<Output<PushPull>>) {
    type Error = <Pin<Output<PushPull>> as ErrorType>::Error;

    async fn trigger(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()?;
        self.1.set_high()?;

        Timer0::delay(50.micros()).await;

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
