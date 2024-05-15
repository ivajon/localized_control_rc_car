//! Defines the race-day binary.

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
                BUFFER_SIZE,
                CAPACITY,
                ESC_PID_PARAMS,
                MAGNET_SPACING,
                MIN_VEL,
                OHSHIT_MAP,
                OUTLIER_LIMIT,
                RADIUS,
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
        servo::Servo,
    };
    use defmt::{debug, error, info};
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Input, Output, Pin, PullDown, PushPull},
        gpiote::*,
        pac::{PWM0, PWM1, SPIS0},
        ppi,
        spis::Transfer,
    };
    use rtic_monotonics::{
        nrf::timer::{fugit::ExtU64, Timer0 as Mono},
        *,
    };
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use shared::protocol::{
        v0_0_1::{Payload, V0_0_1},
        Message,
        Parse,
    };

    /// The duration type
    type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

    #[shared]
    struct Shared {
        velocity: (f32, u64),
        velocity_reference: f32,
        pose_reference: f32,
        safety_velocity_reference: Option<f32>,
        cruch_velocity_reference: Option<f32>,
        difference: f32,
        left_distance: (f32, u64),
        right_distance: (f32, u64),
        forward_distance: f32,

        actual_velocity_ref: f32,

        side_lock: (),
        times: [Instant; 3],
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        first_pair: (Pin<Output<PushPull>>, Pin<Output<PushPull>>),
        trig_forward: Pin<Output<PushPull>>,

        receiver_left: Receiver<'static, f32, CAPACITY>,
        sender_left_clone: Sender<'static, f32, CAPACITY>,

        receiver_forward: Receiver<'static, f32, CAPACITY>,

        receiver_right: Receiver<'static, f32, CAPACITY>,
        sender_right_clone: Sender<'static, f32, CAPACITY>,

        wall_difference_sender: Sender<'static, (f32, (f32, f32)), CAPACITY>,
        wall_difference_recv: Receiver<'static, (f32, (f32, f32)), CAPACITY>,

        // receiver_sonar_packets: Receiver<'static, ((u32, u32), u32), CAPACITY>,
        sender_sonar_packets: Sender<'static, ((u32, u32), u32), CAPACITY>,

        echo_pins: [Pin<Input<PullDown>>; 3],

        senders: [Sender<'static, f32, CAPACITY>; 3],

        servo: ServoController<Servo<PWM1>>,
        esc: MotorController<PWM0>,

        event_manager: EventManager,

        spis: Option<Transfer<SPIS0, &'static mut [u8]>>,
    }

    #[allow(dead_code)]
    #[init(local = [
            #[link_section = ".uninit.buffer"]
            RXBUF: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let p1 = gpio::p1::Parts::new(cx.device.P1);

        // ===================== CONFIGURE PINS =====================
        let pins = PinMapping::new(p0, p1);

        // ===================== CONFIGURE SPI ====================
        let (pins, spis) = pins.spi(cx.device.SPIS0, cx.local.RXBUF);

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

        let (trig_forward, echo_forward) = sonar_forward.split();
        let (trig_left, echo_left) = sonars_left.0.split();
        let (trig_right, echo_right) = sonars_right.0.split();

        // ===================== CONFIGURE CHANNLES =====================
        // Pair left sonars sender and receiver
        let (sender_left, receiver_left) = make_channel!(f32, CAPACITY);
        let sender_left_clone = sender_left.clone();
        // Pair right sonars sender and receiver
        let (sender_right, receiver_right) = make_channel!(f32, CAPACITY);
        let sender_right_clone = sender_right.clone();

        // Channel for forward measurements.
        let (sender_forward, receiver_forward) = make_channel!(f32, CAPACITY);

        let (wall_difference_sender, wall_difference_recv) =
            make_channel!((f32, (f32, f32)), CAPACITY);
        let wall_difference_sender = wall_difference_sender.clone();
        // Channel for packed sonar data.
        let (sender_sonar_packets, _receiver_sonar_packets) =
            make_channel!(((u32, u32), u32), CAPACITY);

        // Array with respective senders
        let senders = [sender_forward, sender_left, sender_right];

        let times = [Instant::from_ticks(0); 3];

        trigger_timestamped::spawn().unwrap_or_else(|_| panic!());
        trigger_timestamped_forward::spawn().unwrap_or_else(|_| panic!());
        controll_loop_steering::spawn().unwrap_or_else(|_| panic!());
        controll_loop_velocity::spawn().unwrap_or_else(|_| panic!());

        let velocity_reference = 40.;
        (
            Shared {
                velocity: (0., 0),
                velocity_reference,
                pose_reference: 0.,
                safety_velocity_reference: None,
                cruch_velocity_reference: None,
                difference: 0.,
                left_distance: (0., 0),
                right_distance: (0., 0),
                forward_distance: f32::MAX,
                side_lock: (),
                times,
                actual_velocity_ref: velocity_reference,
            },
            Local {
                first_pair: (trig_left, trig_right),
                trig_forward,
                receiver_left,
                sender_left_clone,
                receiver_right,
                sender_right_clone,
                receiver_forward,

                wall_difference_recv,
                wall_difference_sender,

                sender_sonar_packets,
                spis: Some(spis),

                senders,
                echo_pins: [echo_forward, echo_left, echo_right],
                event_manager,

                servo,
                esc,
            },
        )
    }

    #[task(shared = [velocity,left_distance,right_distance,velocity_reference,pose_reference], local = [
           spis,
    ],priority = 3,binds = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0)]
    fn register_measurement(mut cx: register_measurement::Context) {
        info!("Waiting for message");
        let (buff, transfer) = cx.local.spis.take().unwrap_or_else(|| panic!()).wait();

        // Convert in to a message.
        match Message::<V0_0_1>::try_parse(&mut buff.iter().cloned()) {
            Some(message) => {
                let payload = message.payload();
                info!("Got message {:?}", payload);
                match payload {
                    Payload::SetSpeed {
                        velocity,
                        hold_for_us: _hold,
                    } => {
                        cx.shared
                            .velocity_reference
                            .lock(|vel| *vel = velocity as f32);
                    }
                    // Payload::SetPosition { position } => {
                    //     cx.shared
                    //         .pose_reference
                    //         .lock(|pose_ref| *pose_ref = position as f32);
                    // }
                    _ => {}
                }
            }
            None => debug!("SPI got malformed packet"),
        }

        // let new_velocity = cx.shared.velocity.lock(|measurement| *measurement);
        // let left_distance = cx.shared.left_distance;
        // let right_distance = cx.shared.right_distance;
        // let new_pose = (left_distance, right_distance).lock(|dl, dr| dr.0 - dl.0);

        // Enqueue all of the bytes needed for the transfer.
        // let new_message = Message::<V0_0_1>::new(Payload::CurrentState {
        // velocity: new_velocity.0 as u32,
        // position: new_pose as i32,
        // distance: 0,
        // time_us: new_velocity.1,
        // });

        // for (idx, el) in new_message.enumerate() {
        //     if idx >= BUFFER_SIZE {
        //         warn!("BUFFER OVERFLOW");
        //         break;
        //     }
        //     buff[idx] = el;
        // }

        transfer.reset_events();

        // Enqueue the next packet.
        *cx.local.spis = transfer.transfer(buff).ok();
    }

    #[task(binds = GPIOTE,priority = 7, local=[
           senders,
           event_manager,
           flank:bool=false,
           prev_time:Option<u64> = None,
           echo_pins
    ],shared = [
        velocity,
        left_distance,
        right_distance,
        times,

    ])]
    /// Reads the interrupts and computes distance or velocity depending on
    /// which interrupt was triggered.
    fn echo(mut cx: echo::Context) {
        // Get time ASAP for highest granularity
        let time = Mono::now();

        // Check which sonar triggered the event and store it in the array
        cx.shared.times.lock(|times| {
            let echo_pins = cx.local.echo_pins;
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
                        let idx = match sonar {
                            Sonar::Forward => 0,
                            Sonar::Left => 1,
                            Sonar::Right => 2,
                            Sonar::Left2 => continue,
                            Sonar::Right2 => continue,
                        };

                        compute_distance(idx, cx.local.senders, echo_pins, times, time)
                    }
                }
            }
        });
        // This should not be needed.
    }

    #[task(local = [esc], shared = [
           velocity,
           velocity_reference,
           safety_velocity_reference,
           cruch_velocity_reference,
           actual_velocity_ref
    ],priority=4
    )]
    /// Provides a PID controller for the ESC.
    async fn controll_loop_velocity(mut cx: controll_loop_velocity::Context) {
        info!("Controll loop velocity spawned");
        let controller = cx.local.esc;
        let mut previous = (0f32, 0);
        // let mut counter = 0;
        Mono::delay(100.millis()).await;

        loop {
            let time = Mono::now();
            let mut measurement = cx.shared.velocity.lock(|measurement| *measurement);
            let mut reference = cx.shared.velocity_reference.lock(|reference| *reference);

            // MIIIIIIGHT be better to just wait here?
            if measurement.1 == previous.1 {
                debug!("Measured velocity must be faster than {:?} cm/s to be accurately sampled in this manner.",MIN_VEL);

                let angle = { 2.0f32 * MAGNET_SPACING as f32 / 10_000f32 };
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

            cx.shared
                .cruch_velocity_reference
                .lock(|cruch_velocity_reference| {
                    if let Some(cruch) = cruch_velocity_reference {
                        if *cruch < reference {
                            // warn!("Limited to {:?}", cruch);
                            reference = *cruch;
                        }
                    }
                });

            cx.shared
                .safety_velocity_reference
                .lock(|safety_velocity_reference| {
                    if let Some(safe) = safety_velocity_reference {
                        if *safe < reference {
                            reference = *safe;
                        }
                    }
                });

            cx.shared.actual_velocity_ref.lock(|actual_velocity_ref| {
                if reference == 0. {
                    controller.reset();
                }
                *actual_velocity_ref = reference
            });

            info!("REF : {:?}", reference);

            controller.register_measurement(measurement.0, measurement.1 as u32);
            controller.follow([reference]);
            let _actuation = controller
                .actuate()
                .expect("Example is broken this should work");

            // info!("{:?}", actuation);
            let outer = measurement;

            cx.shared.velocity.lock(|measurement| *measurement = outer);

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

    #[task(priority = 4, local = [servo,wall_difference_recv], shared = [
           difference,
           velocity,
           forward_distance,
           pose_reference,
           actual_velocity_ref
    ])]
    async fn controll_loop_steering(mut cx: controll_loop_steering::Context) {
        info!("Controll loop steering spawned");
        let mut prev = Instant::from_ticks(0);
        loop {
            // Read from the channel, if sensor values are slow we will miss deadlines.
            let latest = match cx.local.wall_difference_recv.recv().await {
                Ok(val) => val,
                Err(_) => {
                    defmt::error!("WALL DIFFERENCE CHANNEL BROKEN");
                    break;
                }
            };

            // Lock the needed index variables, this should allow us to use multiple index
            // gain scheduling.
            // if latest.0 > 1. {
            //     latest.0 = 1.;
            // }
            // if latest.0 < -1. {
            //     latest.0 = -1.;
            // }
            // info!("Latest difference : {:?}", latest);
            // Register measurement and actuate.
            cx.local.servo.register_measurement(
                latest.0 as f32,
                Mono::now().duration_since_epoch().to_micros() as u32,
            );
            cx.shared
                .pose_reference
                .lock(|pose_reference| cx.local.servo.follow([*pose_reference]));
            let actuation = cx
                .local
                .servo
                .actuate(
                    Mono::now()
                        .checked_duration_since(prev)
                        .unwrap()
                        .to_micros(),
                )
                .unwrap_or_else(|_| panic!());
            info!("Actuated : {:?}", actuation);
            prev = Mono::now();
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
           receiver_left,
           sender_left_clone,

           receiver_right,
           sender_right_clone,
           wall_difference_sender,
           sender_sonar_packets,
           first_pair
    ],
    shared = [
        difference,
        left_distance,
        right_distance,
        side_lock,
        times,
        cruch_velocity_reference,
    ])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_timestamped(mut cx: trigger_timestamped::Context) {
        info!("Trigger for side-fireing sonars spawned.");
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

            cx.local.first_pair.trigger().await.unwrap();
            // ======================= LEFT DISTANCE =========================

            poll_counter = 0;
            let zero = Instant::from_ticks(0);
            let left_distance = 'poll: loop {
                // Wait until we have found a rising edge.
                while cx.shared.times.lock(|times| times[1] == zero) {
                    if poll_counter >= MAX_POLL {
                        error!("Max polling exceeded");
                        break 'poll 0.;
                    }
                    Mono::delay(1.millis()).await;
                    if let Ok(value) = cx.local.receiver_left.try_recv() {
                        break 'poll value;
                    }
                    poll_counter += 1;
                }

                // When we have found a rising edge we wait until the falling ege comes.
                if poll_counter >= MAX_POLL {
                    error!("Max polling exceeded for falling edge");
                    break 'poll 0.;
                }
                match cx.local.receiver_left.try_recv() {
                    Ok(value) => {
                        let mut val = value;
                        while let Ok(inner_val) = cx.local.receiver_left.try_recv() {
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
                cx.shared.times.lock(|times| {
                    times[1] = zero;
                    times[2] = zero;
                });
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
                while cx.shared.times.lock(|times| times[2] == zero) {
                    if poll_counter >= MAX_POLL {
                        error!("Max polling exceeded");
                        break 'poll 0.;
                    }
                    Mono::delay(1.millis()).await;
                    if let Ok(value) = cx.local.receiver_right.try_recv() {
                        break 'poll value;
                    }
                    poll_counter += 1;
                }
                if poll_counter >= MAX_POLL {
                    error!("Max polling exceeded for falling edge");
                    break 'poll 0.;
                }
                match cx.local.receiver_right.try_recv() {
                    Ok(value) => {
                        let mut val = value;
                        // Consume all enqueued messages.
                        while let Ok(inner_val) = cx.local.receiver_right.try_recv() {
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
                cx.shared.times.lock(|times| {
                    times[1] = zero;
                    times[2] = zero;
                });
            }

            found_outlier |= right_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_right,
                &mut right_outlier,
                &OUTLIER_LIMIT,
            );

            // DISCARD OUTLIERS
            if found_outlier {
                // if we have found an outlier set the
                // info!("OUTLIER");
                // Mono::delay(250.millis()).await;
                continue 'main;
            }

            left_window.push_back(left_distance);
            right_window.push_back(right_distance);

            let left_distance = sum(&left_window);
            let right_distance = sum(&right_window);
            let difference = left_distance - right_distance; //(left_distance + right_distance).min(400.);
            diff_window.push_back(difference);
            let difference = sum(&diff_window);

            match cx
                .local
                .wall_difference_sender
                .send((difference, (left_distance, right_distance)))
                .await
            {
                Ok(_) => {}
                Err(_) => {
                    error!("difference sender 1 channel full.");
                    continue 'main;
                }
            };
        }
    }

    #[task(priority = 3,local = [
           receiver_forward,
           trig_forward
    ],
    shared = [
        safety_velocity_reference,
        times,
        forward_distance
    ])]
    /// Triggers the forward sensor.
    async fn trigger_timestamped_forward(mut cx: trigger_timestamped_forward::Context) {
        let mut prev_dist_forward = 0.;
        let mut forward_outlier = 0;

        let mut forward_window: ArrayDeque<f32, 40, Wrapping> = ArrayDeque::new();

        const MAX_POLL: usize = 100;
        let mut poll_counter;

        let mut limiting: bool = false;

        'main: loop {
            let mut found_outlier = false;

            // ======================= FORWARD DISTANCE =========================
            cx.local.trig_forward.trigger().await.unwrap();

            poll_counter = 0;
            let zero = Instant::from_ticks(0);
            let forward_distance = 'poll: loop {
                while cx.shared.times.lock(|times| times[0] == zero) {
                    if poll_counter >= MAX_POLL {
                        error!("Max polling exceeded");
                        break 'poll 0.;
                    }
                    Mono::delay(1.millis()).await;
                    if let Ok(value) = cx.local.receiver_forward.try_recv() {
                        break 'poll value;
                    }
                    poll_counter += 1;
                }
                if poll_counter >= MAX_POLL {
                    error!("Max polling exceeded for falling edge");
                    break 'poll 0.;
                }
                match cx.local.receiver_forward.try_recv() {
                    Ok(value) => {
                        let mut val = value;
                        // Consume all enqueued messages.
                        while let Ok(inner_val) = cx.local.receiver_forward.try_recv() {
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

            if forward_distance == 0. {
                found_outlier = true;
                cx.shared.times.lock(|times| {
                    times[0] = zero;
                });
                Mono::delay(100.millis()).await;
            }

            found_outlier |= forward_distance.reject::<VOTE_THRESH>(
                &mut prev_dist_forward,
                &mut forward_outlier,
                &OUTLIER_LIMIT,
            );

            // info!("Distance ({:?})", forward_distance);
            // DISCARD OUTLIERS
            if found_outlier {
                continue 'main;
            }

            forward_window.push_back(forward_distance);

            cx.shared
                .forward_distance
                .lock(|fwd| *fwd = forward_distance);

            let distance = sum(&forward_window);

            let mut none_checked = true;
            for (thresh_distance, speed) in OHSHIT_MAP.iter().rev() {
                if distance < *thresh_distance {
                    continue;
                }
                none_checked = false;
                if speed.is_none() {
                    cx.shared
                        .safety_velocity_reference
                        .lock(|safety_velocity_reference| *safety_velocity_reference = None);
                    break;
                }
                if let Some(velocity) = speed {
                    cx.shared
                        .safety_velocity_reference
                        .lock(|safety_velocity_reference| {
                            *safety_velocity_reference = Some(*velocity);
                            limiting = true;
                        });
                }
                continue 'main;
            }
            if none_checked {
                let speed = OHSHIT_MAP[0].1;
                if let Some(velocity) = speed {
                    cx.shared
                        .safety_velocity_reference
                        .lock(|safety_velocity_reference| {
                            *safety_velocity_reference = Some(velocity);
                            limiting = true;
                        });
                    continue 'main;
                }
            }
            cx.shared
                .safety_velocity_reference
                .lock(|safety_velocity_reference| *safety_velocity_reference = None);
            limiting = false;
        }
    }
}
