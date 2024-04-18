//! Defines a simple showcase for how to use the [`Esc`](controller::esc::Esc).
//!
//! It simply changes the direction we are moving in.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(noop_waker)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout

const BUFFER_SIZE: usize = 10;

#[link_section = ".BUFFER"]
static WRITE_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
#[link_section = ".BUFFER"]
static READ_BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,RTC1,RTC2]
)]
mod app {
    use defmt::info;
    use nrf52840_hal::{
        clocks::Clocks,
        gpio::p0,
        pac::SPIS0,
        spis_async::{Config, Pins, Spi},
    };

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        spi: Spi<SPIS0, true, true, false, { super::BUFFER_SIZE }>,
    }

    // For future pin reference look at https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpin.html&cp=3_0_0_6_0
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");
        let _clocks = Clocks::new(cx.device.CLOCK).enable_ext_hfosc();

        let p0 = p0::Parts::new(cx.device.P0);

        let mosi = p0.p0_23.into_floating_input().degrade();
        let miso = p0.p0_22.into_floating_input().degrade();
        let sck = p0.p0_24.into_floating_input().degrade();

        let pins = Pins::duplex(sck, miso, mosi);
        let spi = Spi::new(
            cx.device.SPIS0,
            (
                pins,
                super::WRITE_BUFFER.as_ptr() as *mut [u8; super::BUFFER_SIZE],
                super::READ_BUFFER.as_ptr() as *const [u8; super::BUFFER_SIZE],
            ),
            Config::default(),
        )
        .unwrap_or_else(|_| panic!());

        (Shared {}, Local { spi })
    }

    #[task(shared = [], local = [spi])]
    async fn register_measurement(cx: register_measurement::Context) {
        // wait for the spis_duplex to be freed.
        cx.local.spi.write(&[0, 1, 2, 3, 4]).await.ok();
    }
}
