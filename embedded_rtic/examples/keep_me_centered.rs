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

    use core::hint::unreachable_unchecked;

    use controller::{car::wrappers::ServoController, compute_distance, servo::Servo};
    use defmt::info;
    use embedded_hal::digital::OutputPin;
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Input, Output, Pin, PullDown, PushPull},
        gpiote::*,
        pac::PWM0,
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
        // receiver_forward: Receiver<'static, u32, CAPACITY>,

        // Sonar 2
        trig2: Pin<Output<PushPull>>,
        echo2: Pin<Input<PullDown>>,
        receiver_left: Receiver<'static, u32, CAPACITY>,

        // Sonar 3
        trig3: Pin<Output<PushPull>>,
        echo3: Pin<Input<PullDown>>,
        receiver_right: Receiver<'static, u32, CAPACITY>,

        receiver_sonar_packets: Receiver<'static, ((u32, u32), u32), CAPACITY>,
        sender_sonar_packets: Sender<'static, ((u32, u32), u32), CAPACITY>,

        senders: [Sender<'static, u32, CAPACITY>; 3],
        times: [Instant; 3],

        servo_controller: ServoController<PWM0>,
    }

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

        let servo_controller = {
            let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();
            let servo = Servo::new(cx.device.PWM0, motor);
            let mut controller = ServoController::new(servo);
            controller.follow([0]);
            controller
        };

        // Pair forward sonars sender and receiver
        let (sender, _receiver_forward) = make_channel!(u32, CAPACITY);

        // Pair left sonars sender and receiver
        let (sender2, receiver_left) = make_channel!(u32, CAPACITY);

        // Pair right sonars sender and receiver
        let (sender3, receiver_right) = make_channel!(u32, CAPACITY);

        // Channel for packed sonar data.
        let (sender_sonar_packets, receiver_sonar_packets) =
            make_channel!(((u32, u32), u32), CAPACITY);

        // Array with respective senders
        let senders = [sender, sender2, sender3];

        //
        let times = [Instant::from_ticks(0); 3];

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        trigger_timestamped::spawn().ok();
        controll_loop::spawn().ok();

        (Shared { gpiote }, Local {
            trig,
            echo,
            trig2,
            echo2,
            trig3,
            echo3,
            // receiver_forward,
            receiver_left,
            receiver_right,
            sender_sonar_packets,
            receiver_sonar_packets,
            senders,
            times,
            servo_controller,
        })
    }

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

        // Check which sonar triggered the event and store it in the array
        let channels = cx.local.senders;
        let times = cx.local.times;
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                compute_distance(0, channels, times, time);
            }
            if gpiote.channel1().is_event_triggered() {
                compute_distance(1, channels, times, time);
            }
            if gpiote.channel2().is_event_triggered() {
                compute_distance(2, channels, times, time);
            }
        });

        // Clear pending bits.
        cx.shared.gpiote.lock(|gpiote| {
            gpiote.reset_events();
            gpiote.channel0().clear();
            gpiote.channel1().clear();
            gpiote.channel2().clear();
        });
    }

    #[task(priority = 1, local = [receiver_sonar_packets,servo_controller])]
    async fn controll_loop(cx: controll_loop::Context) {
        let prev = None;
        loop {
            let start_time = Mono::now();
            // This should ensure that the sampling time is correct.

            // Dequeue all of the measured values and grab the latest one
            let mut latest = prev.clone();
            // If we have more than one value recorded we discard the previous values.
            while let Ok(new) = cx.local.receiver_sonar_packets.try_recv() {
                latest = Some(new);
            }

            if latest.is_none() {
                Mono::delay(1.secs()).await;
                continue;
            }

            // TODO! Add in some way to cope with to slow measurements.
            if latest == prev {
                todo!()
            }

            // Previous check makes this safe.
            let latest = unsafe { latest.unwrap_or_else(|| unreachable_unchecked()) };

            let ((dl, dr), ts) = latest;

            let diff = (dl as i32) - (dr as i32);

            // Register measurement and actuate.
            cx.local.servo_controller.register_measurement(diff, ts);
            cx.local
                .servo_controller
                .actuate()
                .unwrap_or_else(|_| panic!());

            Mono::delay_until(
                start_time + { controller::car::constants::SERVO_PID_PARAMS.TS as u64 }.micros(),
            )
            .await;
        }
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
    async fn trigger_left(cx: trigger_left::Context) {
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
    async fn trigger_right(cx: trigger_right::Context) {
        // Set high is always valid.
        cx.local.trig3.set_high().unwrap();

        let now = Mono::now();
        Mono::delay_until(now + 20.micros()).await;

        // Set low is always valid.
        cx.local.trig3.set_low().unwrap();
    }
    #[task(priority = 2,local = [receiver_left,receiver_right,sender_sonar_packets])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_timestamped(cx: trigger_timestamped::Context) {
        let mut time_stamp: u32 = 0;
        loop {
            // Receive data from the second sonar
            trigger_left::spawn().unwrap_or_else(|_| panic!());
            trigger_right::spawn().unwrap_or_else(|_| panic!());
            let distance_left = cx.local.receiver_left.recv().await.unwrap();
            let distance_right = cx.local.receiver_left.recv().await.unwrap();

            cx.local
                .sender_sonar_packets
                .send(((distance_left, distance_right), time_stamp))
                .await
                .unwrap_or_else(|_| panic!());

            time_stamp += 1;
        }
    }
}
