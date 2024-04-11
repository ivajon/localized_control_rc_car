//! Defines a simple distance measurement example.
//!
//! This example measures the distance to a nearby object, prefferably a wall
//! using a sonar sensor. It then smooths the result over a few timestamps to
//! avoid small peaks in the measured distance.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use test_app as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2]
)]
mod app {
    use cortex_m::asm::delay;
    use defmt::info;
    use nrf52840_hal::{clocks::Clocks, gpio};
    use test_app::{servo::Servo, wrapper::Exti32};
    // use rtic_monotonics::nrf::timer::Timer0 as Mono;

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {}

    // For future pin refference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    #[no_mangle]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();

        info!("Motor started");
        let mut servo = Servo::new(cx.device.PWM0, motor);
        loop {
            for i in 15..(-15) {
                servo.angle(i.deg()).unwrap();
                info!("Set angle to {:?} degrees", i);
                delay(10000000);
            }
            for i in (-15)..15 {
                servo.angle(i.deg()).unwrap();
                info!("Set angle to {:?} degrees", i);
                delay(10000000);
            }
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        info!("idle");

        loop {
            continue;
        }
    }
}
