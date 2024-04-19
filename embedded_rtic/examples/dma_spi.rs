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

const BUFFER_SIZE: usize = 10;

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

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        spis: Option<spis::Transfer<SPIS0, &'static mut [u8; super::BUFFER_SIZE]>>,
        spim: Spim<SPIM0>,
        spim_cs: Pin<Output<PushPull>>,
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
        (Shared {}, Local {
            spis: Some(spis),
            spim,
            spim_cs,
        })
    }

    #[task(shared = [], local = [spis],priority = 1)]
    async fn register_measurement(cx: register_measurement::Context) {
        loop {
            info!("Waiting for message");
            let (buff, transfer) = cx.local.spis.take().unwrap_or_else(|| panic!()).wait();
            info!("Read {:?}", buff);

            buff.copy_from_slice(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
            *cx.local.spis = transfer.transfer(buff).ok()
        }
    }

    #[task(shared = [], local = [spim,spim_cs],priority= 2)]
    async fn send_directive(cx: send_directive::Context) {
        loop {
            let mut transfer_buffer = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
            info!("Writing {:?}", transfer_buffer);
            cx.local
                .spim
                .transfer(cx.local.spim_cs, &mut transfer_buffer)
                .ok();
            info!("Buffer after transfer {:?}", transfer_buffer);
            if !transfer_buffer.map(|val| val == 0).iter().all(|val| *val) {
                info!("Transfer did not work");
                return;
            }
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }
}
