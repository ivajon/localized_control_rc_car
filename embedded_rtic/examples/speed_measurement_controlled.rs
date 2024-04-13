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
    use controller::{
        esc::{Error, Esc},
        servo::Servo,
        wrapper::Exti32,
    };
    use cortex_m::asm::delay;
    use defmt::{debug, info};
    use nrf52840_hal::{
        clocks::Clocks,
        gpio,
        gpiote::Gpiote,
        pac::{PWM0, PWM1},
        ppi,
        prelude::*,
    };
    use rtic_monotonics::{
        systick::{ExtU32, Systick as Mono},
        Monotonic,
    };
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use shared::controller::Pid;

    const KP: i32 = 23;
    const KI: i32 = 21;
    const KD: i32 = 11;
    // 10 ^ 1
    const SCALE: u32 = 1;

    // Sample time
    const TS: i32 = 200;
    // us
    const TIMESCALE: i32 = 1_000_000;

    #[shared]
    struct Shared {
        measurement: (i32, u32),
        refference: i32,
    }

    // Local resources go here
    #[local]
    struct Local {
        controller: Pid<Error, Esc<PWM0>, i32, 1, KP, KI, KD, TS, -100, 100, TIMESCALE, SCALE>,
        servo: Servo<PWM1>,
        gpiote: Gpiote,

        // Sliding smoothing.
        queue: ArrayDeque<i32, 10, Wrapping>,

        // Sends velocity updates
        sender: Sender<'static, i32, 10>,

        reciever: Receiver<'static, i32, 10>,
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

        esc.speed(25).unwrap();
        servo.angle((-7i32).deg()).unwrap();

        let queue = ArrayDeque::new();

        let controller = Pid::new(esc);

        let (sender, reciever) = make_channel!(i32, 10);

        let systick_token = rtic_monotonics::create_systick_token!();
        Mono::start(cx.core.SYST, 12_000_000, systick_token);
        (
            Shared {
                measurement: (0, 0),
                refference: 0,
            },
            Local {
                sender,
                reciever,
                queue,
                controller,
                gpiote,
                servo,
            },
        )
    }

    #[task(local=[gpiote,prev_time:Option<u32> = None,flank:bool = false,sender],binds=GPIOTE)]
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

                cx.local
                    .sender
                    .try_send(vel as i32)
                    .expect("The message channel is broken");
            }
            None => {
                *cx.local.prev_time = Some(time);
            }
        };

        cx.local.gpiote.reset_events();
        cx.local.gpiote.channel0().clear();
        cx.local.gpiote.port().reset_events();
    }

    #[task(local = [queue, reciever], shared = [measurement],priority=2)]
    async fn intermediary(mut cx: intermediary::Context) {
        while let Ok(vel) = cx.local.reciever.recv().await {
            cx.local.queue.push_back(vel);
            let avg = cx.local.queue.iter().sum::<i32>() / cx.local.queue.len() as i32;
            cx.shared
                .measurement
                .lock(|measurement| *measurement = (avg, measurement.1 + 1));
        }
        debug!("Channel closed, returning from intermediary");
    }

    #[task(local = [controller], shared = [measurement,refference],priority=5)]
    async fn controll_loop(mut cx: controll_loop::Context) {
        let measurement = cx.shared.measurement.lock(|measurement| *measurement);
        let refference = cx.shared.refference.lock(|refference| *refference);
        let controller = cx.local.controller;

        controller.register_measurement(measurement.0, measurement.1);
        controller.follow([refference]);
        controller
            .actuate()
            .expect("Example is broken this should work");

        Mono::delay({ TS as u32 }.micros()).await;
        controll_loop::spawn().ok();
    }

    #[task(shared = [refference],priority=1)]
    async fn set_refference(mut cx: set_refference::Context) {
        let refferences = [200, 400, 600, 800, 1000, 1500, 2000, 1500, 1000, 0, 0];
        loop {
            for vel in refferences {
                cx.shared.refference.lock(|refference| *refference = vel);
                Mono::delay(5.secs()).await;
            }
        }
    }

    #[idle(local = [servo])]
    fn idle(cx: idle::Context) -> ! {
        let servo = cx.local.servo;
        loop {
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
        }
    }
}
