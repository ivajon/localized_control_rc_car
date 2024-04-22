//! Composite example where we set the reference value via SPI and read the
//! latest measurement over SPI.
//!
//! This example can be ran with or without a host, if you want to run it
//! without a host, connect all of spis pins to the spim pins and spawn the send
//! directive task.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]
#![allow(static_mut_refs)]

use controller as _;
use shared::protocol::{v0_0_1::V0_0_1, Message};

const BUFFER_SIZE: usize = Message::<V0_0_1>::MAX_SIZE;

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2,PWM0]
)]
mod app {

    use arraydeque::{ArrayDeque, Wrapping};
    use controller::{
        car::{
            constants::{ESC_PID_PARAMS, MAGNET_SPACING, MIN_VEL, RADIUS},
            wrappers::MotorController,
        },
        esc::Esc,
    };
    use defmt::{debug, info, trace};
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{self, p0, p1, Level, Output, Pin, PushPull},
        gpiote::Gpiote,
        pac::{PWM0, SPIM1, SPIS0},
        ppi::Ppi,
        prelude::*,
        spim::{self, Frequency},
        spis,
        Spim,
    };
    use rtic_monotonics::{
        nrf::timer::{fugit::ExtU64, Timer0 as Mono},
        Monotonic,
    };
    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use shared::{
        controller::Pid,
        protocol::{
            v0_0_1::{Payload, V0_0_1},
            Message,
            Parse,
        },
    };

    /// How big the smoothing window should be.
    const SMOOTHING: usize = 10;

    #[shared]
    struct Shared {
        measurement: (f32, u32),
        reference: f32,
    }

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        spis: Option<spis::Transfer<SPIS0, &'static mut [u8; super::BUFFER_SIZE]>>,
        spim: Spim<SPIM1>,
        spim_cs: Pin<Output<PushPull>>,
        controller: MotorController<PWM0>,

        gpiote: Gpiote,

        // Sliding smoothing.
        queue: ArrayDeque<i32, SMOOTHING, Wrapping>,

        // Sends velocity updates
        sender: Sender<'static, i32, 30>,

        receiver: Receiver<'static, i32, 30>,

        command_receiver: Receiver<'static, Payload, 30>,
        command_sender: Sender<'static, Payload, 30>,
    }

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init(local = [
            #[link_section = ".uninit.buffer"]
            RXBUF: [u8; super::BUFFER_SIZE] = [0; super::BUFFER_SIZE],
            #[link_section = ".uninit.buffer"]
            TXBUF: [u8; super::BUFFER_SIZE] = [0; super::BUFFER_SIZE],
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();

        let p0 = p0::Parts::new(cx.device.P0);
        let p1 = p1::Parts::new(cx.device.P1);

        let (controller, sender, receiver, queue, gpiote) = {
            let _source = p0.p0_15.into_push_pull_output(gpio::Level::High).degrade();
            let motor = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();

            let mut esc = Esc::new(cx.device.PWM0, motor);

            let hal_effect = p0.p0_11.into_pullup_input().degrade();

            let gpiote = {
                let gpiote = Gpiote::new(cx.device.GPIOTE);
                gpiote
                    .channel0()
                    .input_pin(&hal_effect)
                    .lo_to_hi()
                    .enable_interrupt();

                let ppi_channels = nrf52840_hal::ppi::Parts::new(cx.device.PPI);
                let mut ppi0 = ppi_channels.ppi0;
                ppi0.set_event_endpoint(gpiote.channel0().event());
                ppi0.set_task_endpoint(gpiote.channel0().task_out());
                ppi0.enable();
                gpiote.port().enable_interrupt();
                gpiote
            };

            esc.speed(0).unwrap();

            let queue = ArrayDeque::new();

            let controller = Pid::new(esc);

            let (sender, receiver) = make_channel!(i32, 30);

            intermediary::spawn().ok();
            control_loop::spawn().ok();

            (controller, sender, receiver, queue, gpiote)
        };

        let mosi = p1.p1_12.into_floating_input().degrade();
        let miso = p1.p1_13.into_floating_input().degrade();
        let sck = p1.p1_14.into_floating_input().degrade();
        let cs = p1.p1_11.into_floating_input().degrade();

        let spis_pins = spis::Pins {
            sck,
            cipo: Some(miso),
            copi: Some(mosi),
            cs,
        };
        let spi = spis::Spis::new(cx.device.SPIS0, spis_pins);
        spi.enable_interrupt(spis::SpisEvent::End);
        let spis = spi.transfer(cx.local.RXBUF).unwrap_or_else(|_| panic!());

        let spim_mosi = p1.p1_07.into_push_pull_output(Level::Low).degrade();
        let spim_miso = p1.p1_08.into_floating_input().degrade();
        let spim_sck = p1.p1_10.into_push_pull_output(Level::Low).degrade();
        let spim_cs = p0.p0_18.into_push_pull_output(Level::Low).degrade();

        let spim_pins = spim::Pins {
            sck: Some(spim_sck),
            mosi: Some(spim_mosi),
            miso: Some(spim_miso),
        };
        let spim = Spim::new(cx.device.SPIM1, spim_pins, Frequency::M1, spim::MODE_0, 0);

        let (command_sender, command_receiver) = make_channel!(Payload, 30);

        // send_directive::spawn().ok();
        reference_setter::spawn().ok();

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        (
            Shared {
                measurement: (0f32, 0),
                reference: 0f32,
            },
            Local {
                spis: Some(spis),
                spim,
                spim_cs,
                controller,
                sender,
                receiver,
                queue,
                gpiote,
                command_sender,
                command_receiver,
            },
        )
    }

    #[task(shared = [measurement], local = [
           spis,
           command_sender,
           previous:(f32,u32) = (0.,0)
    ],priority = 3,binds = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0)]
    fn register_measurement(mut cx: register_measurement::Context) {
        info!("Waiting for message");
        let (buff, transfer) = cx.local.spis.take().unwrap_or_else(|| panic!()).wait();

        // Convert in to a message.
        match Message::<V0_0_1>::try_parse(&mut buff.iter().cloned()) {
            Some(message) => {
                let payload = message.payload();
                info!("Got message {:?}", payload);
                cx.local
                    .command_sender
                    .try_send(payload)
                    .unwrap_or_else(|_| panic!());
            }
            None => debug!("SPI got malformed packet"),
        }

        let new_measurement = cx.shared.measurement.lock(|measurement| *measurement);

        // Enqueue all of the bytes needed for the transfer.
        if new_measurement != *cx.local.previous {
            let new_message = Message::<V0_0_1>::new(Payload::CurrentVelocity {
                velocity: new_measurement.0 as u32,
                time_us: new_measurement.1 as u64,
            });
            for (idx, el) in new_message.enumerate() {
                buff[idx] = el;
            }

            *cx.local.previous = new_measurement;
        } else {
            buff.fill(0);
        }

        transfer.reset_events();

        // Enqueue the next packet.
        *cx.local.spis = transfer.transfer(buff).ok();
    }

    #[task(shared = [], local = [
            spim,
            spim_cs,
            #[link_section = ".uninit.buffer"]
            BUF: [u8; super::BUFFER_SIZE] = [0;super::BUFFER_SIZE],
    ],priority= 1)]
    /// This mimics the host side SPI, if we are connected to the real device we
    /// should not spawn this.
    async fn send_directive(cx: send_directive::Context) {
        let commands = [
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 0,
                hold_for_us: 10,
            }),
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 10,
                hold_for_us: 10,
            }),
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 20,
                hold_for_us: 10,
            }),
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 30,
                hold_for_us: 10,
            }),
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 40,
                hold_for_us: 10,
            }),
            Message::<V0_0_1>::new(Payload::SetSpeed {
                velocity: 60,
                hold_for_us: 10,
            }),
        ];
        loop {
            for el in commands.clone().iter() {
                let msg = el.clone();
                for (idx, el) in msg.enumerate() {
                    cx.local.BUF[idx] = el;
                    info!("Cx : {:?}", cx.local.BUF[idx]);
                }
                info!("Writing {:?}", cx.local.BUF);

                cx.local
                    .spim
                    .transfer(cx.local.spim_cs, cx.local.BUF)
                    .unwrap_or_else(|_| panic!());

                info!("Buffer after transfer {:?}", cx.local.BUF);
                // Convert in to a message.
                match Message::<V0_0_1>::try_parse(&mut cx.local.BUF.iter().cloned()) {
                    Some(message) => {
                        let payload = message.payload();
                        info!("Host got message : {:?}", payload);
                    }
                    None => debug!("SPI got malformed packet"),
                }
                for _ in 0..100 {
                    cx.local.BUF.fill(0);
                    cx.local
                        .spim
                        .transfer(cx.local.spim_cs, cx.local.BUF)
                        .unwrap_or_else(|_| panic!());

                    info!("Buffer after transfer {:?}", cx.local.BUF);
                    // Convert in to a message.
                    match Message::<V0_0_1>::try_parse(&mut cx.local.BUF.iter().cloned()) {
                        Some(message) => {
                            let payload = message.payload();
                            info!("Host got message : {:?}", payload);
                        }
                        None => debug!("SPI got malformed packet"),
                    }
                    Mono::delay(100.millis()).await;
                }

                if !cx.local.BUF.map(|val| val != 255).iter().all(|val| *val) {
                    info!("Transfer did not work");
                }
            }
        }
    }

    #[task(shared = [reference],local=[command_receiver],priority = 3)]
    async fn reference_setter(mut cx: reference_setter::Context) {
        while let Ok(payload) = cx.local.command_receiver.recv().await {
            if let Payload::SetSpeed {
                velocity,
                hold_for_us: _hold_for_us,
            } = payload
            {
                cx.shared
                    .reference
                    .lock(|reference| *reference = velocity as f32);
            }
        }
    }

    #[task(
        local=[
            gpiote,
            sender,
            // The time for the previous magnet.
            prev_time:Option<u64> = None,
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
                let dt = time - *value;
                let angvel = (MAGNET_SPACING as u64) * 1_000_000 / dt;
                let angvel = angvel / 10_000;
                trace!("Angular velocity {:?}", angvel);
                let vel = RADIUS * angvel;
                trace!("Velocity : {:?} cm/s", vel);
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

    #[task(local = [queue, receiver], shared = [measurement],priority=4)]
    /// Acts as a trampoline for data processing thus, hopefully reducing the
    /// time spent in `compute_vel`.
    async fn intermediary(mut cx: intermediary::Context) {
        info!("Waiting for message");
        // Wait for a new velocity reading, smooth it using the queue and then set the
        // average value in measurement.

        while let Ok(vel) = cx.local.receiver.recv().await {
            cx.local.queue.push_back(vel);
            let avg = cx.local.queue.iter().sum::<i32>() / { SMOOTHING as i32 };
            cx.shared
                .measurement
                .lock(|measurement| *measurement = (avg as f32, measurement.1 + 1));
        }
        debug!("Channel closed.");
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
        let controller = cx.local.controller;
        let mut previous = (0f32, 0);
        loop {
            let time = Mono::now();
            let mut measurement = cx.shared.measurement.lock(|measurement| *measurement);
            let reference = cx.shared.reference.lock(|reference| *reference);

            if measurement.1 == previous.1 {
                debug!("Measured velocity must be faster than {:?} cm/s to be accurately sampled in this manner.",MIN_VEL);

                let angle = { 0.1f32 * MAGNET_SPACING as f32 / 10_000f32 };
                let angvel =
                    angle * { ESC_PID_PARAMS.TIMESCALE as f32 } / (ESC_PID_PARAMS.TS as f32);
                let vel = RADIUS as f32 * angvel;

                info!("Expected min vel to be  {:?}", vel);
                // Go a little bit closer to 0 as we are sampling faster than
                // the wheel rotates.
                //
                // We could do something like subtract towards the minimum speed here.
                // I am not sure on how we do this in a good way.
                measurement.0 -= vel;
                if measurement.0 < 0f32 {
                    measurement.0 = 0f32;
                }
            } else {
                previous = measurement;
            }

            info!("Previous speed {:?} target : {:?}", measurement, reference);

            controller.register_measurement(measurement.0, measurement.1);
            controller.follow([reference]);
            let _actuation = controller
                .actuate()
                .expect("Example is broken this should work");

            let outer = measurement;

            cx.shared
                .measurement
                .lock(|measurement| *measurement = outer);

            // Delay between entry time and actuation time.
            Mono::delay_until(time + { ESC_PID_PARAMS.TS as u64 }.micros()).await;
        }
    }
}
