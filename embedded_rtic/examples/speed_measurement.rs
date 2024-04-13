//! Defines a simple showcase for how to use the [`Esc`](controller::esc::Esc).
//!
//! It simply changes the direction we are moving in.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(noop_waker)]
#![deny(clippy::all)]
#![allow(dead_code)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2]
)]
mod app {
    use arraydeque::{behavior::Wrapping, ArrayDeque};
    use controller::{esc::Esc, servo::Servo, wrapper::Exti32};
    use cortex_m::asm::delay;
    use defmt::info;
    use nrf52840_hal::{
        clocks::Clocks,
        gpio,
        gpiote::Gpiote,
        pac::{PWM0, PWM1},
        ppi,
        prelude::*,
    };
    use rtic_monotonics::{systick::Systick as Mono, Monotonic};

    #[shared]
    struct Shared {
        velocity: u32,
    }

    // Local resources go here
    #[local]
    struct Local {
        esc: Esc<PWM0>,
        servo: Servo<PWM1>,
        gpiote: Gpiote,
        queue: ArrayDeque<<Mono as Monotonic>::Instant, 10, Wrapping>,
    }

    const DIFF: u32 = 2 * 31415 / 3;

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let _source = p0.p0_15.into_push_pull_output(gpio::Level::High).degrade();
        let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();
        let servo_motor = p0.p0_06.into_push_pull_output(gpio::Level::High).degrade();

        let mut esc = Esc::new(cx.device.PWM0, motor);
        let mut servo = Servo::new(cx.device.PWM1, servo_motor);

        let echo = p0.p0_11.into_pullup_input().degrade();

        let gpiote = {
            let gpiote = Gpiote::new(cx.device.GPIOTE);
            gpiote
                .channel0()
                .input_pin(&echo)
                .lo_to_hi()
                .enable_interrupt();

            let ppi_channels = ppi::Parts::new(cx.device.PPI);
            let mut ppi0 = ppi_channels.ppi0;
            ppi0.set_event_endpoint(gpiote.channel0().event());
            ppi0.set_task_endpoint(gpiote.channel0().task_out());
            ppi0.enable();
            gpiote.port().enable_interrupt();
            gpiote
        };

        let systick_token = rtic_monotonics::create_systick_token!();
        Mono::start(cx.core.SYST, 12_000_000, systick_token);
        esc.speed(25).unwrap();
        servo.angle((-7i32).deg()).unwrap();

        let queue = ArrayDeque::new();

        (Shared { velocity: 0 }, Local {
            queue,
            esc,
            gpiote,
            servo,
        })
    }

    #[task(local=[gpiote,queue,prev_time:Option<u32> = None,flank:bool = false],shared=[velocity],binds=GPIOTE)]
    /// Computes the velocity using a rotary encoder.
    fn compute_vel(cx: compute_vel::Context) {
        let time = Mono::now().duration_since_epoch().to_micros();

        if *cx.local.flank {
            *cx.local.flank = false;
            cx.local.gpiote.reset_events();
            cx.local.gpiote.channel0().clear();
            cx.local.gpiote.port().reset_events();
        }
        match cx.local.prev_time {
            Some(value) => {
                // info!("value : {:?} time : {:?}", value, time);
                let dt = time - *value;
                let angvel = (DIFF as u64) * 1_000_000 / (dt as u64);
                let angvel = angvel / 10_000/* (DIFF as u64 / 3) */;
                info!("Angular velocity {:?}", angvel);
                let vel = 3 * angvel;
                info!("Velocity : {:?} cm/s", vel);
                *cx.local.prev_time = Some(time);
            }
            None => {
                *cx.local.prev_time = Some(time);
            }
        };
        cx.local.gpiote.reset_events();
        cx.local.gpiote.channel0().clear();
        cx.local.gpiote.port().reset_events();

        // info!("Interrupt time {:?}", time);
    }

    #[idle(local = [esc,servo])]
    fn idle(cx: idle::Context) -> ! {
        let servo = cx.local.servo;
        loop {
            for i in 20..23 {
                cx.local.esc.speed(i).unwrap();
                info!("Set speed to {:?}", i);

                delay(10000000);
            }
            for i in ((-10)..10).rev() {
                servo.angle(i.deg()).unwrap();
                info!("Set angle to {:?} degrees", i);
                delay(10000000);
            }

            for i in (-10)..10 {
                servo.angle(i.deg()).unwrap();
                info!("Set angle to {:?} degrees", i);
                delay(10000000);
            }

            for i in (20..23).rev() {
                cx.local.esc.speed(i).unwrap();
                info!("Set speed to {:?}", i);
                delay(10000000);
            }
        }
    }
}
