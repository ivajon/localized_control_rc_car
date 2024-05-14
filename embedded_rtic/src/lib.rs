//! Defines some panic behavior and common utilities for the application.
//!
//!
//! ## [Esc](esc)
//!
//! We define a wrapper for the Electronic speed controller.
//! You can use this abstraction like so.
//!
//! ```no_run
//!         let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();
//!         let mut esc = Esc::new(cx.device.PWM0, motor);
//!
//!         loop {
//!             info!("BACKWARDS");
//!             esc.speed(-29).unwrap();
//!
//!             delay(50000000);
//!             info!("FORWARDS!");
//!             esc.speed(20).unwrap();
//!
//!             delay(50000000);
//!         }
//! ```
//!
//! ## [Servo](servo)
//!
//! We define a simple wrapper for the steering servo that only allows you to
//! set the angle of the servo within reasonable limits.
//! It can be used like so.
//!
//! ```no_run
//!         let p0 = gpio::p0::Parts::new(cx.device.P0);
//!         let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();
//!
//!         let mut servo = Servo::new(cx.device.PWM0, motor);
//!         loop {
//!             for i in 15..(-15) {
//!                 servo.angle(i.deg()).unwrap();
//!                 delay(10000000);
//!             }
//!             for i in (-15)..15 {
//!                 servo.angle(i.deg()).unwrap();
//!                 delay(10000000);
//!             }
//!         }
//! ```
//!
//! ## [Wrapper](wrapper)
//!
//! This module mainly exports a few helper types for unit conversion such as
//! [degrees](wrapper::Degrees).

#![no_main]
#![no_std]
#![deny(warnings)]
#![feature(async_fn_traits)]
#![feature(async_closure)]
#![deny(missing_docs)]
#![deny(clippy::all)]
#![allow(clippy::manual_range_contains)]
use core::sync::atomic::{AtomicUsize, Ordering};

use car::wrappers::Instant;
use defmt::{trace, warn};
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use nrf52840_hal::gpio::{Input, Pin, PullDown};
// global logger
use panic_probe as _;
use rtic_sync::channel::Sender;

use crate::car::constants::{MAGNET_SPACING, RADIUS}; // memory layout

pub mod car;
pub mod esc;
pub mod helpers;
pub mod servo;
pub mod wrapper;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

/// Computes the distance for a specific sensor channel and sends that in the
/// specified [`Sender`].
#[inline(always)]
pub fn compute_distance(
    sonar: usize,
    channels: &mut [Sender<'static, f32, { car::constants::CAPACITY }>],
    pins: &mut [Pin<Input<PullDown>>],
    times: &mut [Instant],
    time: Instant,
) {
    let zero = Instant::from_ticks(0);

    if times[sonar] == zero {
        if pins[sonar].is_high().is_ok_and(|el| el) {
            times[sonar] = time;
        } else {
            warn!("Tried to set time on falling edge");
        }
    } else {
        let distance = time
            .checked_duration_since(times[sonar])
            .unwrap()
            .to_micros() as f32
            / 56.;
        match channels[sonar].try_send(distance) {
            Ok(_) => {}
            _ => {
                defmt::error!(
                    "Distance did not fit in to the buffer, scheduling is probably broken."
                );
            }
        }
        times[sonar] = zero;
    }
}

/// Computes the velocity of the car based on time stamps in micro seconds.
#[inline(always)]
pub fn compute_velocity(
    previous_time: &mut Option<u64>,
    time: u64,
    // sender: &mut Sender<'static, i32, CAPACITY>,
    vel: &mut (f32, u64),
) {
    match previous_time {
        Some(value) => {
            let dt = time - *value;
            let angvel = (MAGNET_SPACING as u64) * 1_000_000 / dt;
            let angvel = angvel / 10_000;
            trace!("Angular velocity {:?}", angvel);
            let velocity = RADIUS * angvel;
            trace!("Velocity : {:?} cm/s", vel);
            *previous_time = Some(time);

            *vel = (velocity as f32, time);
            // let _ = sender.try_send(vel as i32);
        }
        None => {
            *previous_time = Some(time);
        }
    }
}
