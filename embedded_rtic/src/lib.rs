//! Defines some drivers and panic behavior for our app
#![no_main]
#![no_std]
#![deny(warnings)]
#![deny(missing_docs)]
#![deny(clippy::all)]
#![feature(noop_waker)]

// use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _; // global logger
use embassy_nrf as _;
use panic_probe as _; // memory layout

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

// static COUNT: AtomicUsize = AtomicUsize::new(0);
// defmt::timestamp!("{=usize}", {
//     // NOTE(no-CAS) `timestamps` runs with interrupts disabled
//     let n = COUNT.load(Ordering::Relaxed);
//     COUNT.store(n + 1, Ordering::Relaxed);
//     n
// });
//
/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[macro_export]
/// Calls an async function in a blocking manner.
macro_rules! block {
    ($call:expr) => {
        'a: {
            use core::{future::Future,task::{Poll},pin::pin};

            let unpinned = $call;
            let waker = core::task::Waker::noop();
            let mut ctx = core::task::Context::from_waker(waker);
            loop {
                let future_clone = pin!(unpinned.clone());
                let ret = future_clone.poll(&mut ctx);
                if let Poll::Ready(ret) = ret {
                   break 'a ret;
                }
            }
        }
    };
    ($mod:ident::$func:ident($($arg:expr),*)) => {
        'a: {
            use core::{future::Future,task::{Poll},pin::pin};

            let unpinned = $mod::$func($($arg),*);
            let waker = core::task::Waker::noop();
            let mut ctx = core::task::Context::from_waker(waker);
            let future = pin!( unpinned);
            loop {
                let ret = future.poll(&mut ctx);
                if let Poll::Ready(ret) = ret {
                   break 'a ret;
                }
            }
        }
    };
}
