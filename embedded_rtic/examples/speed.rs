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
    use nrf52840_hal::{
        clocks::Clocks,
        gpio,
        prelude::*,
        pwm::{Channel, Pwm},
    };
    use test_app::{
        servo::{Servo, ServoInterface},
        wrapper::Exti32,
    };
    // use rtic_monotonics::nrf::timer::Timer0 as Mono;

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {}

    // For future pin refference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();

        let pwm = Pwm::new(cx.device.PWM0);
        pwm.set_prescaler(nrf52840_hal::pwm::Prescaler::Div128);
        pwm.set_period(250u32.hz())
            .set_output_pin(nrf52840_hal::pwm::Channel::C0, motor);

        pwm.set_max_duty(1000);
        let max_duty = pwm.max_duty();
        info!("Maximum duty is {:?}", max_duty);

        pwm.enable();
        info!("Motor started");
        let mut servo = Servo::new(pwm, Channel::C0);
        servo.angle((60).deg()).unwrap();
        delay(50000000);
        servo.angle((30).deg()).unwrap();
        delay(50000000);
        servo.angle((0).deg()).unwrap();
        delay(50000000);
        servo.angle((-30).deg()).unwrap();
        delay(50000000);
        servo.angle((-60).deg()).unwrap();
        delay(50000000);

        // let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();

        loop {}

        // Mono::start(cx.device.TIMER0, token);
        //
        // (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        info!("idle");

        loop {
            continue;
        }
    }
}
