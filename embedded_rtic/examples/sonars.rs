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
    dispatchers = [RTC0,RTC1,RTC2]
)]

mod app {

    use arraydeque::{behavior::Wrapping, ArrayDeque};
    use controller::car::constants::Sonar;
    use defmt::{error, info};
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Input, Output, Pin, PullDown, PushPull},
        gpiote::*,
        ppi,
        prelude::*,
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
    const CAPACITY: usize = 5;

    /// The duration type
    type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        //Sonar 1
        trig: Pin<Output<PushPull>>,
        echo: Pin<Input<PullDown>>,
        receiver_forward: Receiver<'static, u32, CAPACITY>,

        // Sonar 2
        trig2: Pin<Output<PushPull>>,
        echo2: Pin<Input<PullDown>>,
        receiver_left: Receiver<'static, u32, CAPACITY>,

        // Sonar 3
        trig3: Pin<Output<PushPull>>,
        echo3: Pin<Input<PullDown>>,
        receiver_right: Receiver<'static, u32, CAPACITY>,

        senders: [Sender<'static, u32, CAPACITY>; 3],
        times: [Instant; 3],
    }

    const NUM_SONARS: usize = 3; // Number of sonars

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[allow(dead_code)]
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);

        // Sonar 1
        let trig = p0.p0_12.into_push_pull_output(gpio::Level::Low).degrade();
        let echo = p0.p0_11.into_pulldown_input().degrade();

        // Sonar 2
        let trig2 = p0.p0_14.into_push_pull_output(gpio::Level::Low).degrade();
        let echo2 = p0.p0_13.into_pulldown_input().degrade();

        // Sonar 3
        let trig3 = p0.p0_16.into_push_pull_output(gpio::Level::Low).degrade();
        let echo3 = p0.p0_15.into_pulldown_input().degrade();

        // Enable interrupts
        //
        // The nrf52840 is a bit strange when it comes to pin interrupts,
        // but for now we simply connect the echo pins to the channel 0, 1 and 2.

        // Configure GPIOTE and PPI for all sonars
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        let ppi_channels = ppi::Parts::new(cx.device.PPI);

        // Sonar 1 -> Channel0
        // Sonar 2 -> Channel1
        // Sonar 3 -> Channel2
        let gpiote = {
            gpiote
                .channel0()
                .input_pin(&echo)
                .toggle()
                .enable_interrupt();

            gpiote
                .channel1()
                .input_pin(&echo2)
                .toggle()
                .enable_interrupt();

            gpiote
                .channel2()
                .input_pin(&echo3)
                .toggle()
                .enable_interrupt();

            let mut ppi0 = ppi_channels.ppi0;
            ppi0.set_event_endpoint(gpiote.channel0().event());
            ppi0.set_task_endpoint(gpiote.channel0().task_out());
            ppi0.enable();

            let mut ppi1 = ppi_channels.ppi1;
            ppi1.set_event_endpoint(gpiote.channel1().event());
            ppi1.set_task_endpoint(gpiote.channel1().task_out());
            ppi1.enable();

            let mut ppi2 = ppi_channels.ppi2;
            ppi2.set_event_endpoint(gpiote.channel2().event());
            ppi2.set_task_endpoint(gpiote.channel2().task_out());
            ppi2.enable();

            gpiote.port().enable_interrupt();
            gpiote
        };

        // Pair forward sonars sender and receiver
        let (sender, receiver_forward) = make_channel!(u32, CAPACITY);

        // Pair left sonars sender and receiver
        let (sender2, receiver_left) = make_channel!(u32, CAPACITY);

        // Pair right sonars sender and receiver
        let (sender3, receiver_right) = make_channel!(u32, CAPACITY);

        // Array with respective senders
        let senders = [sender, sender2, sender3];

        //
        let times = [Instant::from_ticks(0); 3];

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        let _ = trigger_trampoline1::spawn();
        let _ = trigger_trampoline2::spawn();
        let _ = trigger_trampoline3::spawn();

        (Shared { gpiote }, Local {
            trig,
            echo,
            trig2,
            echo2,
            trig3,
            echo3,
            receiver_forward,
            receiver_left,
            receiver_right,
            senders,
            times,
        })
    }

    //#[task(binds local=[echo,sender,echo2,sender2,echo3,sender3,senders,times],
    #[task(binds = GPIOTE,priority = 4, local=[echo,echo2,echo3,senders,times],shared = [gpiote])]
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
        let zero: Instant = Instant::from_ticks(0);

        // Array to store triggered sonars
        let mut triggered_sonars = [None; NUM_SONARS];

        // Check which sonar triggered the event and store it in the array
        //let debug_time = time.duration_since_epoch().to_micros();
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                //info!("Interrupt on channel 0, {:?}", debug_time);
                triggered_sonars[0] = Some(Sonar::Forward);
            }
            if gpiote.channel1().is_event_triggered() {
                //info!("Interrupt on channel 1, {:?}", debug_time);
                triggered_sonars[1] = Some(Sonar::Left);
            }
            if gpiote.channel2().is_event_triggered() {
                //info!("Interrupt on channel 2, {:?}", debug_time);
                triggered_sonars[2] = Some(Sonar::Right);
            }
        });

        // Map triggered sonars and match with respective one to track distance
        for sonar in triggered_sonars.iter().filter_map(|x| *x) {
            match sonar {
                Sonar::Forward => {
                    if cx.local.times[0] == zero {
                        cx.local.times[0] = time;
                    } else {
                        let distance = time
                            .checked_duration_since(cx.local.times[0])
                            .unwrap()
                            .to_micros()
                            / 56;
                        match cx.local.senders[0].try_send(distance as u32) {
                            Ok(_) => {}
                            _ => {
                                error!("Distance FORWARD did not fit in to the buffer, scheduling is probably broken.");
                                // This is not fatal so we simply continue with
                                // our life
                            }
                        }
                        cx.local.times[0] = zero;
                    }
                }
                Sonar::Left => {
                    if cx.local.times[1] == zero {
                        cx.local.times[1] = time;
                    } else {
                        let distance2 = time
                            .checked_duration_since(cx.local.times[1])
                            .unwrap()
                            .to_micros()
                            / 56;
                        cx.local.times[1] = zero;
                        match cx.local.senders[1].try_send(distance2 as u32) {
                            Ok(_) => {}
                            _ => {
                                error!("Distance LEFT did not fit in to the buffer, scheduling is probably broken.");
                                // This is not fatal so we simply continue with
                                // our life
                            }
                        }
                    }
                }
                Sonar::Right => {
                    if cx.local.times[2] == zero {
                        cx.local.times[2] = time;
                    } else {
                        let distance3 = time
                            .checked_duration_since(cx.local.times[2])
                            .unwrap()
                            .to_micros()
                            / 56;
                        cx.local.times[2] = zero;
                        match cx.local.senders[2].try_send(distance3 as u32) {
                            Ok(_) => {}
                            _ => {
                                error!("Distance RIGHT did not fit in to the buffer, scheduling is probably broken.");
                                // This is not fatal so we simply continue with
                                // our life
                            }
                        }
                    }
                }
            }
        }
        // Clear interrupt pending bits.
        cx.shared.gpiote.lock(|gpiote| {
            gpiote.reset_events();
            gpiote.channel0().clear();
            gpiote.channel1().clear();
            gpiote.channel2().clear();
        });
    }

    #[task(priority = 2, local = [trig], shared = [gpiote])]
    /// Send a small pulse to the sonars.
    ///
    /// The sonars will then notify us in echo when the sound waves are
    /// received.
    async fn trigger1(cx: trigger1::Context) {
        // Set high is always valid.
        cx.local.trig.set_high().unwrap();

        let now = Mono::now();
        Mono::delay_until(now + 20.micros()).await;

        // Set low is always valid.
        cx.local.trig.set_low().unwrap();
    }

    #[task(priority = 2, local = [trig2], shared = [gpiote])]
    /// Send a small pulse to the sonars.
    ///
    /// The sonars will then notify us in echo when the sound waves are
    /// received.
    async fn trigger2(cx: trigger2::Context) {
        // Set high is always valid.
        cx.local.trig2.set_high().unwrap();

        let now = Mono::now();
        Mono::delay_until(now + 20.micros()).await;

        // Set low is always valid.
        cx.local.trig2.set_low().unwrap();
    }

    #[task(priority = 2, local = [trig3], shared = [gpiote])]
    /// Send a small pulse to sonar 3.
    ///
    /// The sonar will then notify us in echo when the sound wave are
    /// received.
    async fn trigger3(cx: trigger3::Context) {
        // Set high is always valid.
        cx.local.trig3.set_high().unwrap();

        let now = Mono::now();
        Mono::delay_until(now + 20.micros()).await;

        // Set low is always valid.
        cx.local.trig3.set_low().unwrap();
    }

    #[task(priority = 1,local = [receiver_forward])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_trampoline1(cx: trigger_trampoline1::Context) {
        // Apply a sliding window-like smoothing to the measurements
        let mut window: ArrayDeque<u32, 20, Wrapping> = ArrayDeque::new();
        let outlier_offset = 100;
        let mut prev_avg = u32::MAX - outlier_offset;
        loop {
            // Receive data from the first sonar
            match trigger1::spawn() {
                Ok(_) => {
                    let distance = cx.local.receiver_forward.recv().await.unwrap();
                    // Discard outliers
                    if distance < prev_avg + outlier_offset {
                        let _ = window.push_back(distance);
                    }
                    let avg = window.iter().sum::<u32>() / (window.len() as u32);
                    prev_avg = avg;

                    info!(
                        "Sonar 1 Distance : {:?} cm (average : {:?} cm)",
                        distance, avg
                    );
                }
                Err(_) => {
                    error!("Failed to receive data from sonar 1");
                }
            }
            Mono::delay(1000.millis()).await; // Adjust delay as needed
        }
    }

    #[task(priority = 1,local = [receiver_left])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_trampoline2(cx: trigger_trampoline2::Context) {
        let mut window: ArrayDeque<u32, 20, Wrapping> = ArrayDeque::new();
        let outlier_offset = 100;
        let mut prev_avg = u32::MAX - outlier_offset;
        loop {
            // Receive data from the second sonar
            match trigger2::spawn() {
                Ok(_) => {
                    let distance = cx.local.receiver_left.recv().await.unwrap();
                    // Discard outliers
                    if distance < prev_avg + outlier_offset {
                        let _ = window.push_back(distance);
                    }
                    let avg = window.iter().sum::<u32>() / (window.len() as u32);
                    prev_avg = avg;

                    info!(
                        "Sonar 2 Distance : {:?} cm (average : {:?} cm)",
                        distance, avg
                    );
                }
                Err(_) => {
                    error!("Failed to receive data from sonar 2");
                }
            }
            Mono::delay(1000.millis()).await; // Adjust delay as needed
        }
    }

    #[task(priority = 1,local = [receiver_right])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_trampoline3(cx: trigger_trampoline3::Context) {
        // Apply a sliding window-like smoothing to the measurements
        let mut window: ArrayDeque<u32, 20, Wrapping> = ArrayDeque::new();
        let outlier_offset = 100;
        let mut prev_avg = u32::MAX - outlier_offset;
        loop {
            // Receive data from the third sonar
            match trigger3::spawn() {
                Ok(_) => {
                    let distance = cx.local.receiver_right.recv().await.unwrap();
                    // Discard outliers
                    if distance < prev_avg + outlier_offset {
                        let _ = window.push_back(distance);
                    }
                    let avg = window.iter().sum::<u32>() / (window.len() as u32);
                    prev_avg = avg;

                    info!(
                        "Sonar 3 Distance : {:?} cm (average : {:?} cm)",
                        distance, avg
                    );
                }
                Err(_) => {
                    error!("Failed to receive data from sonar 3");
                }
            }
            Mono::delay(1000.millis()).await; // Adjust delay as needed
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
