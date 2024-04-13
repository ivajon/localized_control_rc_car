//! Defines a simple showcase for how to use the [`Esc`](controller::esc::Esc)
//! with a [`PID`](shared::controller::Pid) controller.
//!
//! It sets a few reference velocities and tries to reach the velocity using a
//! PID control law.

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
        car::{
            constants::{ESC_PID_PARAMS, MAGNET_SPACING, RADIUS},
            wrappers::MotorController,
        },
        esc::Esc,
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

    #[shared]
    struct Shared {
        measurement: (i32, u32),
        reference: i32,
    }

    // Local resources go here
    #[local]
    struct Local {
        controller: MotorController<PWM0>,
        servo: Servo<PWM1>,
        gpiote: Gpiote,

        // Sliding smoothing.
        queue: ArrayDeque<i32, 10, Wrapping>,

        // Sends velocity updates
        sender: Sender<'static, i32, 10>,

        receiver: Receiver<'static, i32, 10>,
    }

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

        esc.speed(0).unwrap();
        servo.angle(0.deg()).unwrap();

        let queue = ArrayDeque::new();

        let controller = Pid::new(esc);

        let (sender, receiver) = make_channel!(i32, 10);

        let systick_token = rtic_monotonics::create_systick_token!();
        Mono::start(cx.core.SYST, 12_000_000, systick_token);
        (
            Shared {
                measurement: (0, 0),
                reference: 0,
            },
            Local {
                sender,
                receiver,
                queue,
                controller,
                gpiote,
                servo,
            },
        )
    }

    #[task(
        local=[
            gpiote,
            sender,
            // The time for the previous magnet.
            prev_time:Option<u32> = None,
            // Wether or not we are on a rising or falling edge of the signal.
            flank:bool = false
        ],
        binds=GPIOTE,
        priority = 6
    )]
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
                let angvel = (MAGNET_SPACING as u64) * 1_000_000 / (dt as u64);
                let angvel = angvel / 10_000/* (DIFF as u64 / 3) */;
                info!("Angular velocity {:?}", angvel);
                let vel = RADIUS * angvel;
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

    #[task(local = [queue, receiver], shared = [measurement],priority=2)]
    /// Acts as a trampoline for data processing thus, hopefully reducing the
    /// time spent in `compute_vel`.
    async fn intermediary(mut cx: intermediary::Context) {
        // Wait for a new velocity reading, smooth it using the queue and then set the
        // average value in measurement.
        while let Ok(vel) = cx.local.receiver.recv().await {
            cx.local.queue.push_back(vel);
            let avg = cx.local.queue.iter().sum::<i32>() / cx.local.queue.len() as i32;
            cx.shared
                .measurement
                .lock(|measurement| *measurement = (avg, measurement.1 + 1));
        }
        debug!("Channel closed, returning from intermediary");
    }

    #[task(local = [controller], shared = [measurement,reference],priority=5)]
    /// Highest priority as this should be ran no matter what is running (aside
    /// from measurements).
    ///
    /// Uses the PID to set an appropriate control signal for the motor.
    /// Will re-run every sample time.
    ///
    /// NOTE! If we notice that we this takes too long, use [`Symex`](https://github.com/ivario123/symex) to get the
    /// longest possible time the PID takes and subtract that form TS.
    async fn control_loop(mut cx: control_loop::Context) {
        let measurement = cx.shared.measurement.lock(|measurement| *measurement);
        let reference = cx.shared.reference.lock(|reference| *reference);
        let controller = cx.local.controller;

        controller.register_measurement(measurement.0, measurement.1);
        controller.follow([reference]);
        controller
            .actuate()
            .expect("Example is broken this should work");

        Mono::delay({ ESC_PID_PARAMS.TS as u32 }.micros()).await;
        control_loop::spawn().ok();
    }

    #[task(shared = [reference],priority=1)]
    /// Sets the new reference value, this should probably be done using SPI and
    /// DMA for the real thing.
    async fn set_reference(mut cx: set_reference::Context) {
        let references = [200, 400, 600, 800, 1000, 1500, 2000, 1500, 1000, 0, 0];
        loop {
            for vel in references {
                cx.shared.reference.lock(|reference| *reference = vel);
                Mono::delay(5.secs()).await;
            }
        }
    }

    #[idle(local = [servo])]
    /// Turns a bit every now and then.
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
