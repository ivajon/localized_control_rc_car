//! Defines a simple showcase for how to use the [`Esc`](controller::esc::Esc).
//!
//! It simply changes the direction we are moving in.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(noop_waker)]
#![feature(const_refs_to_static)]
#![feature(const_mut_refs)]
#![deny(clippy::all)]
#![deny(warnings)]
// use core::ptr::addr_of_mut;
#![allow(static_mut_refs)]

use controller as _; // global logger + panicking-behavior + memory layout

const BUFFER_SIZE: usize = 100;

// #[link_section = ".uninit.buffer"]
// static mut WRITE_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
// #[link_section = ".uninit.buffer"]
// static mut READ_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

// static WRITE_BUFFER_PTR: &'static mut [u8] = unsafe { &mut WRITE_BUFFER };
// static READ_BUFFER_PTR: &'static mut [u8] = unsafe { &mut READ_BUFFER };

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2]
)]
mod app {
    use defmt::info;
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::{p0, Level, Output, Pin, PushPull},
        pac::{SPIM0, SPIS0},
        spim::{self, Frequency},
        spis,
        Spim,
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

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        spis: Option<spis::Transfer<SPIS0, &'static mut [u8; super::BUFFER_SIZE]>>,
        spim: Spim<SPIM0>,
        spim_cs: Pin<Output<PushPull>>,
        sender: Sender<'static, Payload, 100>,
        receiver: Receiver<'static, Payload, 100>,
    }

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init(local = [
            #[link_section = ".uninit.buffer"]
            BUF: [u8; super::BUFFER_SIZE] = [0; super::BUFFER_SIZE],
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = p0::Parts::new(cx.device.P0);

        let mosi = p0.p0_23.into_floating_input().degrade();
        let miso = p0.p0_22.into_floating_input().degrade();
        let sck = p0.p0_24.into_floating_input().degrade();
        let cs = p0.p0_25.into_floating_input().degrade();

        let spis_pins = spis::Pins {
            sck,
            cipo: Some(miso),
            copi: Some(mosi),
            cs,
        };
        let spi = spis::Spis::new(cx.device.SPIS0, spis_pins);
        let spis = spi.transfer(cx.local.BUF).unwrap_or_else(|_| panic!());

        let spim_mosi = p0.p0_15.into_push_pull_output(Level::Low).degrade();
        let spim_miso = p0.p0_16.into_floating_input().degrade();
        let spim_sck = p0.p0_17.into_push_pull_output(Level::Low).degrade();
        let spim_cs = p0.p0_18.into_push_pull_output(Level::Low).degrade();

        let spim_pins = spim::Pins {
            sck: Some(spim_sck),
            mosi: Some(spim_mosi),
            miso: Some(spim_miso),
        };
        let spim = Spim::new(cx.device.SPIM0, spim_pins, Frequency::K125, spim::MODE_0, 0);

        register_measurement::spawn().ok();
        send_directive::spawn().ok();

        let (sender, receiver) = make_channel!(Payload, 100);

        (Shared {}, Local {
            spis: Some(spis),
            spim,
            spim_cs,
            sender,
            receiver,
        })
    }

    #[task(shared = [], local = [spis,receiver],priority = 1)]
    async fn register_measurement(cx: register_measurement::Context) {
        loop {
            info!("Waiting for message");
            let (buff, transfer) = cx.local.spis.take().unwrap_or_else(|| panic!()).wait();
            info!("Read {:?}", buff);

            let next_msg = cx.local.receiver.recv().await.unwrap_or_else(|_| panic!());

            let iter = Message::<V0_0_1>::new(next_msg);
            let iter = iter.into_iter();
            let mut ptr = 0;
            for el in iter {
                buff[ptr] = el;
                ptr += 1;
            }
            *cx.local.spis = transfer.transfer(buff).ok()
        }
    }

    #[task(shared = [], local = [spim,spim_cs],priority= 2)]
    async fn send_directive(cx: send_directive::Context) {
        loop {
            let mut transfer_buffer = [0; super::BUFFER_SIZE];
            info!("Writing {:?}", transfer_buffer);
            cx.local
                .spim
                .transfer(cx.local.spim_cs, &mut transfer_buffer)
                .ok();

            let data = Message::<V0_0_1>::try_parse(&mut transfer_buffer.into_iter());
            match data {
                Some(message) => info!("Got message {:?}", message.payload()),
                None => {}
            };
            info!("Buffer after transfer {:?}", transfer_buffer);
        }
    }

    #[task(local=[sender],priority = 1)]
    async fn sender(cx: sender::Context) {
        let sender = cx.local.sender;
        sender
            .send(Payload::CurrentVelocity {
                velocity: 10,
                time_us: 1,
            })
            .await
            .unwrap_or_else(|_| panic!());
        sender
            .send(Payload::CurrentAngle {
                angle: -10,
                time_us: 1,
            })
            .await
            .unwrap_or_else(|_| panic!());
        sender
            .send(Payload::CurrentDistance {
                distance: 10,
                time_us: 1,
            })
            .await
            .unwrap_or_else(|_| panic!());
        loop {}
    }
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        sender::spawn().unwrap_or_else(|_| panic!());
        loop {}
    }
}
