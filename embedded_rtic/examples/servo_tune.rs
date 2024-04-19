//! This example shows how to use [`Servo`](controller::servo::Servo).
//!
//! It simply steps the angle from the right most turn to the left most turn.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout

/// An [`Iterator`] that never runs out.
pub struct CircularBuffer<I: Iterator> {
    source: I,
    item: I,
}

impl<I: Iterator + Clone> CircularBuffer<I> {
    /// Creates a new [`CircularBuffer`]
    pub fn new(iter: I) -> Self {
        Self {
            source: iter.clone(),
            item: iter.clone(),
        }
    }
}

impl<I: Iterator + Clone> Iterator for CircularBuffer<I> {
    type Item = I::Item;

    fn next(&mut self) -> Option<Self::Item> {
        match self.item.next() {
            None => {
                self.item = self.source.clone();
                self.item.next()
            }
            val => val,
        }
    }
}

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2]
)]
mod app {
    use controller::{servo::Servo, wrapper::Exti32};
    use defmt::info;
    use embedded_hal::digital::InputPin;
    use nrf52840_hal::{clocks::Clocks, gpio};

    use crate::CircularBuffer;

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {}

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    #[no_mangle]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();

        // Button 1
        let mut button = p0.p0_11.into_pullup_input().degrade();
        let buffer = CircularBuffer::new(
            (0..15)
                .chain((0..15).rev())
                .chain(((-15)..0).rev())
                .chain((-15)..0),
        );
        info!("Motor started");
        let mut servo = Servo::new(cx.device.PWM0, motor);
        for angle in buffer {
            while button.is_low().unwrap() {}

            info!("Button pressed");
            info!("Setting angle to {:?}", angle);
            let angle = angle.deg();
            servo.angle(angle).unwrap();
            while button.is_high().unwrap() {}
        }
        unreachable!()
    }
}
