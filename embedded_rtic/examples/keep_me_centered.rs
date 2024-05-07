//! Defines a distance measurement example for multiple sonars.
//!
//! This example measures the distance to nearby objects, preferably a wall
//! using a sonar sensors on the front, left and right side of the car. It then
//! smooths the result over a few timestamps to avoid small peaks in the
//! measured distance.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2,TIMER1,TIMER2,TIMER3]
)]

mod app {

    use arraydeque::{ArrayDeque, Wrapping};
    use controller::{
        car::{
            constants::{Sonar, CAPACITY, ESC_PID_PARAMS, MAGNET_SPACING, MIN_VEL, RADIUS},
            event::{EventManager, GpioEvents},
            pin_map::PinMapping,
            wrappers::{MotorController, ServoController},
        },
        compute_distance,
        compute_velocity,
    };
    use defmt::{debug, info};
    use embedded_hal::digital::OutputPin;
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Output, Pin, PushPull},
        gpiote::*,
        pac::{PWM0, PWM1},
        ppi,
    };
    use rtic_monotonics::{
        nrf::timer::{fugit::ExtU64, Timer0 as Mono},
        *,
    };
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };

    /// The message queue capacity
    const SMOOTHING: usize = 10;

    /// The duration type
    type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

    #[shared]
    struct Shared {
        velocity: (f32, u64),
        velocity_reference: f32,
        difference: f32,
        left_distance: (f32, u64),
        right_distance: (f32, u64),
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        //Sonar 1
        // trig_forward: Pin<Output<PushPull>>,
        // receiver_forward: Receiver<'static, u32, CAPACITY>,

        // Sonar 2
        trig_left: Pin<Output<PushPull>>,
        receiver_left: Receiver<'static, u32, CAPACITY>,

        // Sonar 3
        trig_right: Pin<Output<PushPull>>,
        receiver_right: Receiver<'static, u32, CAPACITY>,

        // receiver_sonar_packets: Receiver<'static, ((u32, u32), u32), CAPACITY>,
        sender_sonar_packets: Sender<'static, ((u32, u32), u32), CAPACITY>,

        receiver_velocity: Receiver<'static, i32, CAPACITY>,
        sender_velocity: Sender<'static, i32, CAPACITY>,

        senders: [Sender<'static, u32, CAPACITY>; 3],
        times: [Instant; 3],

        servo: ServoController<PWM1>,
        esc: MotorController<PWM0>,

        queue: ArrayDeque<i32, SMOOTHING, Wrapping>,

        event_manager: EventManager,
    }

    #[allow(dead_code)]
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let p1 = gpio::p1::Parts::new(cx.device.P1);

        let pins = PinMapping::new(p0, p1);

        let (pins, servo) = pins.servo_controller(cx.device.PWM1);
        let (pins, esc) = pins.esc_controller(cx.device.PWM0);

        // Configure GPIOTE and PPI for all sonars
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        let ppi_channels = ppi::Parts::new(cx.device.PPI);
        let (pins, event_manager) = pins.configure_events(gpiote, ppi_channels);

        let (sonar_forward, sonar_left, sonar_right, _hal_effect) = pins.consume();

        let (_trig_forward, _echo_forward) = sonar_forward.split();
        let (trig_left, _echo_left) = sonar_left.split();
        let (trig_right, _echo_right) = sonar_right.split();

        // Pair forward sonars sender and receiver
        let (sender_forward, _receiver_forward) = make_channel!(u32, CAPACITY);
        // Pair left sonars sender and receiver
        let (sender_left, receiver_left) = make_channel!(u32, CAPACITY);
        // Pair right sonars sender and receiver
        let (sender_right, receiver_right) = make_channel!(u32, CAPACITY);
        // Channel for packed sonar data.
        let (sender_sonar_packets, _receiver_sonar_packets) =
            make_channel!(((u32, u32), u32), CAPACITY);

        let (sender_velocity, receiver_velocity) = make_channel!(i32, CAPACITY);

        // Array with respective senders
        let senders = [sender_forward, sender_left, sender_right];

        let times = [Instant::from_ticks(0); 3];

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        trigger_timestamped::spawn().unwrap_or_else(|_| panic!());
        controll_loop_steering::spawn().unwrap_or_else(|_| panic!());
        controll_loop_velocity::spawn().unwrap_or_else(|_| panic!());
        // intermediary::spawn().unwrap_or_else(|_| panic!());

        (
            Shared {
                velocity: (0., 0),
                velocity_reference: 30.,
                difference: 0.,
                left_distance: (0., 0),
                right_distance: (0., 0),
            },
            Local {
                //Sonar 1
                // trig_forward,
                // receiver_forward: Receiver<'static, u32, CAPACITY>,

                // Sonar 2
                trig_left,
                receiver_left,

                // Sonar 3
                trig_right,
                receiver_right,

                // receiver_sonar_packets,
                sender_sonar_packets,

                sender_velocity,
                receiver_velocity,

                senders,
                times,

                servo,
                esc,

                queue: ArrayDeque::new(),

                event_manager,
            },
        )
    }

    #[task(binds = GPIOTE,priority = 7, local=[
           senders,
           times,
           event_manager,
           flank:bool=false,
           prev_time:Option<u64> = None,
           sender_velocity
    ],shared = [velocity,left_distance,right_distance])]
    /// Reads the echo pin and check which sonar that triggered an event using
    /// their channels
    ///
    /// Whenever the echo pin goes high or low this interrupt is triggered.
    /// On the falling edge this interrupt computes the "width" in time
    /// of the square wave thus allowing us to compute the time it took
    /// to echo back to us.
    fn echo(mut cx: echo::Context) {
        // Get time ASAP for highest granularity
        let time = Mono::now();

        // Check which sonar triggered the event and store it in the array
        let _channels = cx.local.senders;
        let times = cx.local.times;

        // Now automagiaclly clears the pending bit after a case is handled.
        for event in cx.local.event_manager.events() {
            match event {
                GpioEvents::Encoder => {
                    if *cx.local.flank {
                        *cx.local.flank = false;
                        continue;
                    }
                    *cx.local.flank = true;
                    cx.shared.velocity.lock(|vel| {
                        compute_velocity(
                            cx.local.prev_time,
                            time.duration_since_epoch().to_micros(),
                            vel,
                        )
                    });
                }
                GpioEvents::Sonar(sonar) => match sonar {
                    Sonar::Forward => { /* compute_distance(0, channels, times, time) */ }
                    Sonar::Left => {
                        cx.shared
                            .left_distance
                            .lock(|left_dist| compute_distance(1, left_dist, times, time));
                    }
                    Sonar::Right => {
                        cx.shared
                            .right_distance
                            .lock(|right_dist| compute_distance(2, right_dist, times, time));
                        // compute_distance(2, channels, times, time)
                    }
                },
            }
        }
        // This should not be needed.
        cx.local.event_manager.clear();
    }

    #[task(local = [queue, receiver_velocity], shared = [velocity],priority=1)]
    /// Acts as a trampoline for data processing thus, hopefully reducing the
    /// time spent in `compute_vel`.
    async fn intermediary(mut cx: intermediary::Context) {
        // info!("Waiting for message");
        // Wait for a new velocity reading, smooth it using the queue and then set the
        // average value in measurement.

        // If we have scheduling issues again we should probably remove this.
        // it is not impossible either that the issue is th

        while let Ok(vel) = cx.local.receiver_velocity.recv().await {
            cx.local.queue.push_back(vel);
            let avg = cx.local.queue.iter().sum::<i32>() / { SMOOTHING as i32 };
            cx.shared
                .velocity
                .lock(|measurement| *measurement = (avg as f32, measurement.1 + 1));
        }
        debug!("Channel closed.");
    }

    #[task(local = [esc], shared = [velocity,velocity_reference],priority=4)]
    /// Highest priority as this should be ran no matter what is running (aside
    /// from measurements).
    ///
    /// Uses the PID to set an appropriate control signal for the motor.
    /// Will re-run every sample time.
    ///
    /// NOTE! If we notice that we this takes too long, use [`Symex`](https://github.com/ivario123/symex) to get the
    /// longest possible time the PID takes and subtract that form TS.
    async fn controll_loop_velocity(mut cx: controll_loop_velocity::Context) {
        let controller = cx.local.esc;
        let mut previous = (0f32, 0);
        loop {
            let time = Mono::now();
            let mut measurement = cx.shared.velocity.lock(|measurement| *measurement);
            let reference = cx.shared.velocity_reference.lock(|reference| *reference);

            if measurement.1 == previous.1 {
                debug!("Measured velocity must be faster than {:?} cm/s to be accurately sampled in this manner.",MIN_VEL);

                // Lets say that the minimum speed we want to achive is 10 cm/s.
                let angle = { 1.0f32 * MAGNET_SPACING as f32 / 10_000f32 };
                let angvel =
                    angle * { ESC_PID_PARAMS.TIMESCALE as f32 } / (ESC_PID_PARAMS.TS as f32);

                let vel = RADIUS as f32 * angvel;

                measurement.0 -= vel;

                if measurement.0 < 0f32 {
                    measurement.0 = 0f32;
                }
            } else {
                previous = measurement;
            }

            // info!("Previous speed {:?} target : {:?}", measurement, reference);

            controller.register_measurement(measurement.0, measurement.1 as u32);
            controller.follow([reference]);
            let _actuation = controller
                .actuate()
                .expect("Example is broken this should work");

            let outer = measurement;

            cx.shared.velocity.lock(|measurement| *measurement = outer);
            // Delay between entry time and actuation time.
            Mono::delay_until(time + { ESC_PID_PARAMS.TS as u64 }.micros()).await;
        }
    }

    #[task(priority = 4, local = [servo], shared = [difference])]
    async fn controll_loop_steering(mut cx: controll_loop_steering::Context) {
        let mut prev = 0.;
        let mut prev_modified = 0.;

        loop {
            let start_time = Mono::now();
            // This should ensure that the sampling time is correct.

            // Dequeue all of the measured values and grab the latest one
            let mut latest = cx.shared.difference.lock(|diff| *diff);

            // TODO! Add in some way to cope with to slow measurements.
            if latest == prev {
                // info!("Servo controll too fast");
                // prev_modified = prev_modified / 2.;
                latest = prev_modified;
            } else {
                prev = latest;
                prev_modified = latest;
            }

            // Previous check makes this safe.

            let diff = latest;

            // info!("Distance difference : {:?}", diff);

            // Register measurement and actuate.
            cx.local.servo.register_measurement(diff as f32, 0);
            // TODO! Follow something else here.
            cx.local.servo.follow([0.]);
            cx.local.servo.actuate().unwrap_or_else(|_| panic!());

            Mono::delay_until(
                start_time + { controller::car::constants::SERVO_PID_PARAMS.TS as u64 }.micros(),
            )
            .await;
        }
    }

    #[task(priority = 4,local = [
           receiver_left,trig_left,
           receiver_right,trig_right,
           sender_sonar_packets
    ],
    shared = [difference,left_distance,right_distance])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_timestamped(mut cx: trigger_timestamped::Context) {
        // let mut time_stamp: u32 = 0;
        let mut prev_dist_left = (0., 0);
        let mut prev_dist_right = (0., 0);
        let mut left_outlier = 0;
        let mut right_outlier = 0;

        const OUTLIER_LIMIT: f32 = 150.;
        const VOTE_THRESH: usize = 2;
        // Do not expect any value larger than 300.
        // const MAX_VALUE: f32 = 300.;

        const SMOOTHING: usize = 10;
        let mut window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();

        'main: loop {
            // Receive data from the second sonar
            cx.local.trig_left.set_high().unwrap();
            cx.local.trig_right.set_high().unwrap();
            let now = Mono::now();
            Mono::delay_until(now + 25.micros()).await;

            cx.local.trig_left.set_low().unwrap();
            cx.local.trig_right.set_low().unwrap();

            let mut count = 0;
            loop {
                if count >= 10 {
                    continue 'main;
                }
                let mut new = (0., 0);
                cx.shared.left_distance.lock(|left| {
                    new = *left;
                });
                if new != prev_dist_left {
                    // if new.0 > MAX_VALUE {
                    // window.drain(..);
                    // cx.shared.difference.lock(|diff| *diff = 0.);
                    // continue 'main;
                    // }
                    let mut val = new.0 - prev_dist_left.0;
                    if val < 0. {
                        val = -val;
                    }
                    if val > OUTLIER_LIMIT && left_outlier < VOTE_THRESH {
                        left_outlier += 1;
                        continue 'main;
                    }

                    prev_dist_left = new;
                    break;
                }
                Mono::delay(1.millis()).await;
                count += 1;
            }

            let mut count = 0;
            loop {
                if count >= 10 {
                    continue 'main;
                }
                let mut new = (0., 0);
                cx.shared.right_distance.lock(|right| {
                    new = *right;
                });
                if new != prev_dist_right {
                    // if new.0 > MAX_VALUE {
                    // window.drain(..);
                    // cx.shared.difference.lock(|diff| *diff = 0.);
                    // continue 'main;
                    // }
                    let mut val = new.0 - prev_dist_right.0;
                    if val < 0. {
                        val = -val;
                    }
                    if val > OUTLIER_LIMIT && right_outlier < VOTE_THRESH {
                        right_outlier += 1;
                        continue 'main;
                    }
                    prev_dist_right = new;
                    break;
                }
                Mono::delay(1.millis()).await;
                count += 1;
            }
            // info!("Right measured");

            let distance_left = prev_dist_left; //cx.local.receiver_left.recv().await.unwrap();
                                                // info!("Left distance : {:?}", distance_left);
            let distance_right = prev_dist_right; //cx.local.receiver_right.recv().await.unwrap();
                                                  // info!("Right distance : {:?}", distance_right);
            let difference = distance_left.0 - distance_right.0;

            window.push_back(difference);
            let sum = (0..(window.len()))
                .map(|idx| (idx as f32) / { SMOOTHING as f32 })
                .sum::<f32>();
            let avg = window
                .iter()
                .enumerate()
                .map(|(idx, value)| (idx as f32) / { SMOOTHING as f32 } * value)
                .sum::<f32>()
                / sum;
            info!("New diff set to : {:?}", avg);
            cx.shared.difference.lock(|diff| *diff = avg);

            // Mono::delay(50.millis()).await;
            // time_stamp += 1;
        }
    }
}
