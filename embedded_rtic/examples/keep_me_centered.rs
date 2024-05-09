//! Defines a simple PID controller for both steering and velocity controll.

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
            constants::{
                Sonar,
                CAPACITY,
                ESC_PID_PARAMS,
                MAGNET_SPACING,
                MIN_VEL,
                OUTLIER_LIMIT,
                RADIUS,
                SERVO_PID_PARAMS,
                SMOOTHING,
                VOTE_THRESH,
            },
            event::{EventManager, GpioEvents},
            pin_map::PinMapping,
            wrappers::{MotorController, ServoController},
        },
        compute_distance,
        compute_velocity,
        helpers::{sum, timeout, OutlierRejection, Trigger},
    };
    use defmt::{debug, error, info};
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

    /// The duration type
    type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

    #[shared]
    struct Shared {
        velocity: (f32, u64),
        velocity_reference: f32,
        difference: f32,
        left_distance: (f32, u64),
        left_distance_2: (f32, u64),
        right_distance: (f32, u64),
        right_distance_2: (f32, u64),

        side_lock: (),
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        first_pair: (Pin<Output<PushPull>>, Pin<Output<PushPull>>),
        second_pair: (Pin<Output<PushPull>>, Pin<Output<PushPull>>),

        receiver_left_1: Receiver<'static, f32, CAPACITY>,
        sender_left_1_clone: Sender<'static, f32, CAPACITY>,
        receiver_left_2: Receiver<'static, f32, CAPACITY>,
        sender_left_2_clone: Sender<'static, f32, CAPACITY>,

        receiver_right_1: Receiver<'static, f32, CAPACITY>,
        sender_right_1_clone: Sender<'static, f32, CAPACITY>,
        receiver_right_2: Receiver<'static, f32, CAPACITY>,
        sender_right_2_clone: Sender<'static, f32, CAPACITY>,

        wall_difference_sender_1: Sender<'static, f32, CAPACITY>,
        wall_difference_sender_2: Sender<'static, f32, CAPACITY>,
        wall_difference_recv: Receiver<'static, f32, CAPACITY>,

        // receiver_sonar_packets: Receiver<'static, ((u32, u32), u32), CAPACITY>,
        sender_sonar_packets: Sender<'static, ((u32, u32), u32), CAPACITY>,

        senders: [Sender<'static, f32, CAPACITY>; 4],
        times: [Instant; 4],

        servo: ServoController<PWM1>,
        esc: MotorController<PWM0>,

        event_manager: EventManager,
    }

    #[allow(dead_code)]
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let p1 = gpio::p1::Parts::new(cx.device.P1);

        // ===================== CONFIGURE PINS =====================
        let pins = PinMapping::new(p0, p1);

        // ===================== CONFIGURE CONTROLLERS =====================
        let (pins, servo) = pins.servo_controller(cx.device.PWM1);
        let (pins, esc) = pins.esc_controller(cx.device.PWM0);

        // ===================== CONFIGURE INTERRUPTS =====================
        // Configure GPIOTE and PPI for all sonars
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        let ppi_channels = ppi::Parts::new(cx.device.PPI);
        let (pins, event_manager) = pins.configure_events(gpiote, ppi_channels);

        // ===================== CONFIGURE SONARS =====================
        let (sonar_forward, sonars_left, sonars_right, _hal_effect) = pins.consume();

        let (_trig_forward, _echo_forward) = sonar_forward.split();
        let (trig_left_1, _echo_left) = sonars_left.0.split();
        let (trig_left_2, _echo_left) = sonars_left.1.split();
        let (trig_right_1, _echo_right) = sonars_right.0.split();
        let (trig_right_2, _echo_right) = sonars_right.1.split();

        // ===================== CONFIGURE CHANNLES =====================
        // Pair left sonars sender and receiver
        let (sender_left_1, receiver_left_1) = make_channel!(f32, CAPACITY);
        let sender_left_1_clone = sender_left_1.clone();
        let (sender_left_2, receiver_left_2) = make_channel!(f32, CAPACITY);
        let sender_left_2_clone = sender_left_2.clone();
        // Pair right sonars sender and receiver
        let (sender_right_1, receiver_right_1) = make_channel!(f32, CAPACITY);
        let sender_right_1_clone = sender_right_1.clone();
        let (sender_right_2, receiver_right_2) = make_channel!(f32, CAPACITY);
        let sender_right_2_clone = sender_right_2.clone();

        let (wall_difference_sender_1, wall_difference_recv) = make_channel!(f32, CAPACITY);
        let wall_difference_sender_2 = wall_difference_sender_1.clone();
        // Channel for packed sonar data.
        let (sender_sonar_packets, _receiver_sonar_packets) =
            make_channel!(((u32, u32), u32), CAPACITY);

        // Array with respective senders
        let senders = [sender_left_1, sender_left_2, sender_right_1, sender_right_2];

        let times = [Instant::from_ticks(0); 4];

        trigger_timestamped::spawn().unwrap_or_else(|_| panic!());
        // trigger_timestamped_2::spawn().unwrap_or_else(|_| panic!());
        controll_loop_steering::spawn().unwrap_or_else(|_| panic!());
        controll_loop_velocity::spawn().unwrap_or_else(|_| panic!());

        (
            Shared {
                velocity: (0., 0),
                velocity_reference: 30.,
                difference: 0.,
                left_distance: (0., 0),
                left_distance_2: (0., 0),
                right_distance: (0., 0),
                right_distance_2: (0., 0),
                side_lock: (),
            },
            Local {
                first_pair: (trig_left_1, trig_right_1),
                receiver_left_1,
                sender_left_1_clone,
                receiver_right_1,
                sender_right_1_clone,

                second_pair: (trig_left_2, trig_right_2),
                receiver_left_2,
                sender_left_2_clone,
                receiver_right_2,
                sender_right_2_clone,

                wall_difference_recv,
                wall_difference_sender_1,
                wall_difference_sender_2,

                sender_sonar_packets,

                senders,
                times,
                event_manager,

                servo,
                esc,
            },
        )
    }

    #[task(binds = GPIOTE,priority = 7, local=[
           senders,
           times,
           event_manager,
           flank:bool=false,
           prev_time:Option<u64> = None,
    ],shared = [
        velocity,
        left_distance,
        left_distance_2,
        right_distance,
        right_distance_2
    ])]
    /// Reads the interrupts and computes distance or velocity depending on
    /// which interrupt was triggered.
    fn echo(mut cx: echo::Context) {
        // Get time ASAP for highest granularity
        let time = Mono::now();

        // Check which sonar triggered the event and store it in the array
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
                GpioEvents::Sonar(sonar) => {
                    match sonar {
                        Sonar::Forward => { /* compute_distance(0, channels, times, time) */ }
                        Sonar::Left => compute_distance(0, cx.local.senders, times, time),
                        Sonar::Left2 => compute_distance(1, cx.local.senders, times, time),
                        Sonar::Right => compute_distance(2, cx.local.senders, times, time),
                        Sonar::Right2 => compute_distance(3, cx.local.senders, times, time),
                    }
                }
            }
        }
        // This should not be needed.
        cx.local.event_manager.clear();
    }

    #[task(local = [esc], shared = [velocity,velocity_reference],priority=4)]
    /// Provides a PID controller for the ESC.
    async fn controll_loop_velocity(mut cx: controll_loop_velocity::Context) {
        info!("Controll loop velocity spawned");
        let controller = cx.local.esc;
        let mut previous = (0f32, 0);
        loop {
            let time = Mono::now();
            let mut measurement = cx.shared.velocity.lock(|measurement| *measurement);
            let reference = cx.shared.velocity_reference.lock(|reference| *reference);

            // MIIIIIIGHT be better to just wait here?
            if measurement.1 == previous.1 {
                debug!("Measured velocity must be faster than {:?} cm/s to be accurately sampled in this manner.",MIN_VEL);

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

            controller.register_measurement(measurement.0, measurement.1 as u32);
            controller.follow([reference]);
            let _actuation = controller
                .actuate()
                .expect("Example is broken this should work");

            let outer = measurement;

            cx.shared.velocity.lock(|measurement| *measurement = outer);

            // Mono::delay(1.secs()).await;
            let duration = Mono::now()
                .checked_duration_since(time)
                .unwrap()
                .to_micros();
            if duration > ESC_PID_PARAMS.TS as u64 {
                defmt::error!("ESC controll loop took {:?} us", duration);
            }

            // Delay between entry time and actuation time.
            Mono::delay_until(time + { ESC_PID_PARAMS.TS as u64 }.micros()).await;
        }
    }

    #[task(priority = 4, local = [servo,wall_difference_recv], shared = [difference])]
    async fn controll_loop_steering(cx: controll_loop_steering::Context) {
        info!("Controll loop steering spawned");
        loop {
            let start_time = Mono::now();

            // Read from the channel, if sensor values are slow we will miss deadlines.
            let latest = match cx.local.wall_difference_recv.recv().await {
                Ok(val) => val,
                Err(_) => {
                    defmt::error!("WALL DIFFERENCE CHANNEL BROKEN");
                    break;
                }
            };

            // // TODO! Add in some way to cope with to slow measurements.
            // if latest == prev {
            //     // info!("Servo controll too fast");
            //     prev_modified = prev_modified / 1.001;
            //     latest = prev_modified;
            //     info!("Using err = {:?}", latest);
            // } else {
            //     prev = latest;
            //     prev_modified = latest;
            // }

            info!("Latest difference : {:?}", latest);
            // Register measurement and actuate.
            cx.local.servo.register_measurement(latest as f32, 0);
            // TODO! Follow something else here.
            cx.local.servo.follow([0.]);
            cx.local.servo.actuate().unwrap_or_else(|_| panic!());

            let now = Mono::now();
            let took = now.checked_duration_since(start_time).unwrap().to_micros();
            if took > SERVO_PID_PARAMS.TS as u64 {
                // defmt::error!("Controll loop for servo out of sync, took {:?} uS", took);
                continue;
            }
            // info!("Controll loop for servo out of sync, took {:?} uS", took);

            Mono::delay_until(
                start_time + { controller::car::constants::SERVO_PID_PARAMS.TS as u64 }.micros(),
            )
            .await;
        }
    }

    #[task(priority = 5)]
    async fn timeout_task(
        _cx: timeout_task::Context,
        mut channel: Sender<'static, f32, CAPACITY>,
        mut kill_channel: Receiver<'static, (), 1>,
    ) {
        timeout(&mut channel, &mut kill_channel, 25.micros()).await;
    }

    #[task(priority = 4,local = [
           receiver_left_1,
           sender_left_1_clone,

           receiver_right_1,
           sender_right_1_clone,
           wall_difference_sender_1,
           sender_sonar_packets,
           first_pair
    ],
    shared = [
        difference,
        left_distance,
        right_distance,
        side_lock,
    ])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_timestamped(mut cx: trigger_timestamped::Context) {
        info!("Trigger for first pair spawned");
        // let mut time_stamp: u32 = 0;
        let mut prev_dist_left = 0.;
        let mut prev_dist_right = 0.;
        let mut left_outlier = 0;
        let mut right_outlier = 0;

        let mut left_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();
        let mut right_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();
        let mut diff_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();

        const MAX_POLL: usize = 500;
        let mut poll_counter;

        'main: loop {
            let mut found_outlier = false;
            cx.shared
                .side_lock
                .lock(|_| async {
                    let now = Mono::now();
                    Mono::delay_until(now + 6.millis()).await;
                    cx.local.first_pair.trigger().await.unwrap();
                })
                .await;

            // ======================= LEFT DISTANCE =========================

            // THIS IS BAD, FIX LATER
            //

            poll_counter = 0;
            let left_distance = 'poll: loop {
                if poll_counter >= MAX_POLL {
                    error!("Max polling exceeded");
                    break 'poll 0.;
                }
                match cx.local.receiver_left_1.try_recv() {
                    Ok(value) => {
                        let mut val = value;
                        while let Ok(inner_val) = cx.local.receiver_left_1.try_recv() {
                            val = inner_val;
                        }
                        break 'poll val;
                    }
                    Err(_) => {
                        poll_counter += 1;
                        Mono::delay(1.millis()).await;
                        continue;
                        // continue 'main;
                    }
                };
            };
            if left_distance == 0. {
                found_outlier = true;
            }

            found_outlier |= left_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_left,
                &mut left_outlier,
                &OUTLIER_LIMIT,
            );

            // ======================= RIGHT DISTANCE =========================
            poll_counter = 0;
            let right_distance = 'poll: loop {
                if poll_counter >= MAX_POLL {
                    error!("Max poll exceeded");
                    break 'poll 0.;
                }
                match cx.local.receiver_right_1.try_recv() {
                    Ok(value) => {
                        let mut val = value;
                        // Consume all enqueued messages.
                        while let Ok(inner_val) = cx.local.receiver_right_1.try_recv() {
                            val = inner_val;
                        }
                        break 'poll val;
                    }
                    Err(rtic_sync::channel::ReceiveError::Empty) => {
                        poll_counter += 1;
                        Mono::delay(1.millis()).await;
                        continue;
                        // continue 'main;
                    }
                    _ => panic!(),
                };
            };

            if right_distance == 0. {
                found_outlier = true;
            }

            found_outlier |= right_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_right,
                &mut right_outlier,
                &OUTLIER_LIMIT,
            );

            // DISCARD OUTLIERS
            if found_outlier {
                // Mono::delay(250.millis()).await;
                continue 'main;
            }

            info!("Distance ({:?},{:?})",left_distance,right_distance);

            left_window.push_back(left_distance);
            right_window.push_back(right_distance);

            let difference = sum(&left_window) - sum(&right_window);
            diff_window.push_back(difference);
            let difference = sum(&diff_window);

            match cx.local.wall_difference_sender_1.send(difference).await {
                Ok(_) => {}
                Err(_) => {
                    error!("difference sender 1 channel full.");
                    continue 'main;
                }
            };
        }
    }

    #[task(priority = 4,local = [
           second_pair,
           receiver_left_2,
           sender_left_2_clone,
           receiver_right_2,
           sender_right_2_clone,
           wall_difference_sender_2
    ],
    shared = [
        difference,
        left_distance_2,
        right_distance_2,
        side_lock,
    ])]
    /// Triggers the second set of sonars.
    async fn trigger_timestamped_2(mut cx: trigger_timestamped_2::Context) {
        info!("Trigger for second pair spawned");
        let mut prev_dist_left = 0.;
        let mut prev_dist_right = 0.;
        let mut left_outlier = 0;
        let mut right_outlier = 0;

        let mut left_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();
        let mut right_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();
        let mut diff_window: ArrayDeque<f32, SMOOTHING, Wrapping> = ArrayDeque::new();

        const MAX_POLL: usize = 100;
        let mut poll_counter;

        'main: loop {
            let mut found_outlier = false;
            cx.shared
                .side_lock
                .lock(|_| async {
                    let now = Mono::now();
                    Mono::delay_until(now + 6.millis()).await;
                    cx.local.second_pair.trigger().await.unwrap();
                })
                .await;

            // ======================= LEFT DISTANCE =========================

            poll_counter = 0;
            let left_distance = 'poll: loop {
                if poll_counter >= MAX_POLL {
                    break 'poll 0.;
                }
                match cx.local.receiver_left_2.try_recv() {
                    Ok(value) => {
                        break 'poll value;
                    }
                    Err(_) => {
                        poll_counter += 1;
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                };
            };

            if left_distance == 0. {
                found_outlier = true;
            }

            found_outlier |= left_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_left,
                &mut left_outlier,
                &OUTLIER_LIMIT,
            );

            // ======================= RIGHT DISTANCE =========================

            poll_counter = 0;
            let right_distance = 'poll: loop {
                if poll_counter >= MAX_POLL {
                    break 'poll 0.;
                }
                match cx.local.receiver_right_2.try_recv() {
                    Ok(value) => {
                        break 'poll value;
                    }
                    Err(_) => {
                        poll_counter += 1;
                        Mono::delay(1.millis()).await;
                        continue;
                    }
                };
            };

            if right_distance == 0. {
                found_outlier = true;
            }

            found_outlier |= right_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_right,
                &mut right_outlier,
                &OUTLIER_LIMIT,
            );

            // DISCARD OUTLIERS
            if found_outlier {
                Mono::delay(250.millis()).await;
                continue 'main;
            }

            left_window.push_back(left_distance);
            right_window.push_back(right_distance);

            let difference = sum(&left_window) - sum(&right_window);
            diff_window.push_back(difference);
            let difference = sum(&diff_window);

            match cx.local.wall_difference_sender_2.send(difference).await {
                Ok(_) => {}
                Err(_) => {
                    info!("difference sender 2 channel full.");
                    continue 'main;
                }
            };
        }
    }
}
