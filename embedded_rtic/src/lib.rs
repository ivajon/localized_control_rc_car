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
#![deny(missing_docs)]
#![deny(clippy::all)]
#![allow(clippy::manual_range_contains)]
use core::sync::atomic::{AtomicUsize, Ordering};

use car::{constants::CAPACITY, wrappers::Instant};
use defmt_rtt as _; // global logger
use panic_probe as _;
use rtic_sync::channel::Sender; // memory layout

pub mod car;
pub mod esc;
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
pub fn compute_distance(
    sonar: usize,
    channels: &mut [Sender<'static, u32, CAPACITY>],
    times: &mut [Instant],
    time: Instant,
) {
    let zero = Instant::from_ticks(0);

    if times[sonar] == zero {
        times[sonar] = time;
    } else {
        let distance = time
            .checked_duration_since(times[sonar])
            .unwrap()
            .to_micros()
            / 56;
        match channels[sonar].try_send(distance as u32) {
            Ok(_) => {}
            _ => {
                defmt::error!(
                    "Distance FORWARD did not fit in to the buffer, scheduling is probably broken."
                );
            }
        }
        times[sonar] = zero;
    }
}
