//! Defines a simple distance measurement example.
//!
//! This example measures the distance to a nearby object, preferably a wall
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

    use arraydeque::{behavior::Wrapping, ArrayDeque};
    use defmt::{error, info};
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Input, Output, Pin, PullDown, PushPull},
        gpiote::Gpiote,
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
        // TODO! Add in temperature
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        trig: Pin<Output<PushPull>>,
        echo: Pin<Input<PullDown>>,
        sender: Sender<'static, u32, CAPACITY>,
        receiver: Receiver<'static, u32, CAPACITY>,
        previous_time: Instant,
    }

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let trig = p0.p0_12.into_push_pull_output(gpio::Level::Low).degrade();
        let echo = p0.p0_11.into_pulldown_input().degrade();

        // Enable interrupts
        //
        // The nrf52840 is a bit strange when it comes to pin interrupts,
        // but for now we simply connect the echo pin to the channel 1.
        let gpiote = {
            let gpiote = Gpiote::new(cx.device.GPIOTE);
            gpiote
                .channel0()
                .input_pin(&echo)
                .toggle()
                .enable_interrupt();

            let ppi_channels = ppi::Parts::new(cx.device.PPI);
            let mut ppi0 = ppi_channels.ppi0;
            ppi0.set_event_endpoint(gpiote.channel1().event());
            ppi0.set_task_endpoint(gpiote.channel1().task_out());
            ppi0.enable();
            gpiote.port().enable_interrupt();
            gpiote
        };

        let (sender, receiver) = make_channel!(u32, CAPACITY);
        let previous_time = Instant::from_ticks(0);

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        let _ = trigger_trampoline::spawn();
        (Shared { gpiote }, Local {
            trig,
            echo,
            sender,
            receiver,
            previous_time,
        })
    }

    #[task(binds = GPIOTE,priority = 4, local=[echo,sender,previous_time],shared = [gpiote])]
    /// Reads the echo pin
    ///
    /// When ever the echo pin goes high or low this interrupt is triggered.
    /// On the falling edge this interrupt computes the "width" in time
    /// of the square wave thus allowing us to compute the time it took
    /// to echo back to us.
    fn echo(cx: echo::Context) {
        // Get time ASAP for highest granularity
        let time = Mono::now();
        let zero: Instant = Instant::from_ticks(0);

        // If this is the first interrupt we are on rising edge.
        // Then we simply return early after setting the time.
        if *cx.local.previous_time == zero {
            *cx.local.previous_time = time;
            let mut gpiote = cx.shared.gpiote;

            gpiote.lock(|gpiote| {
                gpiote.reset_events();
                gpiote.channel1().clear();
            });
            return;
        }
        // If not we compute the distance based on the datasheet.
        let start = cx.local.previous_time.clone();
        *cx.local.previous_time = zero;

        let distance = time.checked_duration_since(start).unwrap().to_micros() / 56;

        match cx.local.sender.try_send(distance as u32) {
            Ok(_) => {}
            _ => {
                error!("Distance did not fit in to the buffer, scheduling is probably broken.");
                // This is not fatal so we simply continue with our life
            }
        }
        let mut gpiote = cx.shared.gpiote;

        // Clear interrupt pending bits.
        gpiote.lock(|gpiote| {
            gpiote.reset_events();
            gpiote.channel1().clear();
        });
    }

    #[task(priority = 2, local = [trig], shared = [gpiote])]
    /// Send a small pulse to the sonar.
    ///
    /// The sonar will then notify us in echo when the sound wave is
    /// recieved.
    async fn trigger(cx: trigger::Context) {
        // Set high is allways valid.
        cx.local.trig.set_high().unwrap();

        let now = Mono::now();
        Mono::delay_until(now + 20.micros()).await;

        // Set low is allways valid.
        cx.local.trig.set_low().unwrap();
    }

    #[task(priority = 1,local = [receiver])]
    /// Re-spawn trigger after every new distance is correctly measured.
    async fn trigger_trampoline(cx: trigger_trampoline::Context) {
        // Apply a sliding window like smoothing to the measurements
        let mut window: ArrayDeque<u32, 20, Wrapping> = ArrayDeque::new();
        let outlier_offset = 100;
        let mut prev_avg = u32::MAX - outlier_offset;
        loop {
            // let _start = Mono::now();
            match trigger::spawn() {
                Ok(_) => {
                    let distance = cx.local.receiver.recv().await.unwrap();
                    // Discard outliers
                    if distance < prev_avg + outlier_offset {
                        // let _end = Mono::now();

                        let _ = window.push_back(distance);
                    }
                    let avg = window.iter().sum::<u32>() / (window.len() as u32);
                    prev_avg = avg;

                    info!("Distance : {:?} cm (average : {:?} cm)", distance, avg);
                    // let dt = end.checked_duration_since(start).unwrap().
                    // to_millis(); info!("Measurement took
                    // : {:?} ms", dt);

                    // info!(
                    //     "This gives us {:?} samples/second",
                    //     100/dt,
                    // );
                }
                _ => {
                    // If no task is spawned we need to wait a bit before respawning it.
                    Mono::delay(500.micros()).await;
                }
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
