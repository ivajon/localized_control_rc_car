//! Defines a simple distance measurement

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use test_app as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [RTC0]
)]
mod app {
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, Output, Pin, PushPull},
        gpiote::Gpiote,
        ppi,
        prelude::*,
    };
    use rtic_monotonics::{
        systick::{fugit::Instant, fugit::*, Systick},
        *,
    };

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        start_time: Instant<u32, 1, 1000>,
        // TODO! Add in temperature
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        trig: Pin<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        let p0 = gpio::p0::Parts::new(cx.device.P0);
        let trig = p0.p0_00.into_push_pull_output(gpio::Level::Low).degrade();
        let echo = p0.p0_01.into_pulldown_input().degrade();

        // Enable interrupts
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&echo)
            .toggle()
            .enable_interrupt();


        let ppi_channels = ppi::Parts::new(cx.device.PPI);
        let mut ppi0 = ppi_channels.ppi0;
        ppi0.set_event_endpoint(gpiote.channel0().event());
        ppi0.set_task_endpoint(gpiote.channel1().task_out());
        ppi0.set_event_endpoint(gpiote.channel2().event());
        ppi0.enable();

        // Enable interrupt for port event
        gpiote.port().enable_interrupt();

        // TODO setup monotonic if used
        let token = rtic_monotonics::create_systick_token!();
        rtic_monotonics::systick::Systick::start(cx.core.SYST, 32_000_000, token);

        task1::spawn().ok();

        (
            Shared {
                gpiote,
                start_time: Systick::now(),
            },
            Local { trig },
        )
    }

    #[task(shared = [gpiote,start_time],priority = 1)]
    async fn calc_distance(mut cx: calc_distance::Context, time: Instant<u32, 1, 1000>) {
        let mut gpiote = cx.shared.gpiote;
        gpiote.lock(|gpiote| {
            if !gpiote.channel0().is_event_triggered() {
                defmt::error!("Unexpected interrupt triggerd");
                panic!();
            }
            gpiote.channel0().reset_events();
        });
        let time = cx.shared.start_time.lock(|start_time| {
            time.checked_duration_since(start_time.clone())
                .unwrap()
                .to_nanos();
        });

        defmt::info!("Sonar took : {:?} ns", time);
    }

    #[task(binds = GPIOTE,priority = 2)]
    fn echo(_cx: echo::Context) {
        defmt::info!("Interrupt recieved!");
        let time = Systick::now();
        calc_distance::spawn(time).ok();
    }

    #[task(priority = 1, local = [trig],shared=[start_time,gpiote])]
    async fn trigger(mut cx: trigger::Context) {
        defmt::info!("Starting trigger");
        cx.shared.start_time.lock(|time| *time = Systick::now());
        cx.local.trig.set_high().unwrap();
        defmt::info!("Waiting for 10 us");

        let now_ms = Systick::now();
        Systick::delay_until(now_ms + 10_u32.micros()).await;
        defmt::info!("Pulling trigger low again");

        cx.local.trig.set_low().unwrap();
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        defmt::info!("Sending trigger");
        trigger::spawn().ok().unwrap();

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }
}