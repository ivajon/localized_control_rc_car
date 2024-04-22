//! Simple example where we use external loopback to test the SPI peripherals.

#![no_main]
#![no_std]
#![deny(clippy::all)]
#![deny(warnings)]
#![feature(type_alias_impl_trait)]

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
        gpio::{p0, p1, Level, Output, Pin, PushPull},
        pac::{SPIM1, SPIS0},
        spim::{self, Frequency},
        spis, /* , spis_async::{Config, Pins, Spi} */
        Spim,
    };
    use rtic_monotonics::nrf::timer::{fugit::ExtU64, Timer0 as Mono};

    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    #[allow(dead_code)]
    struct Local {
        spis: Option<spis::Transfer<SPIS0, &'static mut [u8; super::BUFFER_SIZE]>>,
        // spis: Spi<SPIS0, true, true, true, { crate::BUFFER_SIZE }>,
        spim: Spim<SPIM1>,
        spim_cs: Pin<Output<PushPull>>,
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

        // let spis_pins = Pins::duplex_cs(sck, miso, mosi, cs);
        // let spis = Spi::new(
        //     cx.device.SPIS0,
        //     (
        //         spis_pins,
        //         cx.local.TXBUF.as_ptr() as *mut [u8; super::BUFFER_SIZE],
        //         cx.local.RXBUF.as_ptr() as *const [u8; super::BUFFER_SIZE],
        //     ),
        //     Config::default(),
        // )
        // .unwrap_or_else(|_| panic!());

        let spim_mosi = p1.p1_07.into_push_pull_output(Level::Low).degrade();
        let spim_miso = p1.p1_08.into_floating_input().degrade();
        let spim_sck = p1.p1_10.into_push_pull_output(Level::Low).degrade();
        let spim_cs = p0.p0_18.into_push_pull_output(Level::Low).degrade();

        let spim_pins = spim::Pins {
            sck: Some(spim_sck),
            mosi: Some(spim_mosi),
            miso: Some(spim_miso),
        };
        let spim = Spim::new(cx.device.SPIM1, spim_pins, Frequency::M32, spim::MODE_0, 0);

        send_directive::spawn().ok();
        // register_measurement::spawn().ok();

        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(cx.device.TIMER0, token);

        (Shared {}, Local {
            // spis,
            spis: Some(spis),
            spim,
            spim_cs,
        })
    }
    #[task(shared = [], local = [spis],priority = 3,binds = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0)]
    fn register_measurement(cx: register_measurement::Context) {
        info!("Waiting for message");
        let (buff, transfer) = cx.local.spis.take().unwrap_or_else(|| panic!()).wait();
        // if buff[0] == 0 {
        info!("Read {:?}", buff);
        // }
        buff.copy_from_slice(&[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]);
        let was_end = transfer.is_event_triggered(spis::SpisEvent::End);
        info!("Was end ? : {:?}", was_end);
        let was_end = transfer.is_event_triggered(spis::SpisEvent::EndRx);
        info!("Was end ? : {:?}", was_end);
        // transfer.reset_event(spis::SpisEvent::End);
        // transfer.reset_event(spis::SpisEvent::EndRx);
        transfer.reset_events();

        *cx.local.spis = transfer.transfer(buff).ok();
        // info!("Starting transfer from device side.");
        // let result = cx
        //     .local
        //     .spis
        //     .transfer(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        //     .await
        //     .unwrap_or_else(|_| panic!());
        // info!("Result : {:?}", result);
        // info!("Transfer done!");
    }

    #[task(shared = [], local = [spim,spim_cs,
            #[link_section = ".uninit.buffer"]
            BUF: [u8; super::BUFFER_SIZE] = [1,2,3,4,5,6,7,8,9,10],
    ],priority= 1)]
    async fn send_directive(cx: send_directive::Context) {
        loop {
            // let mut transfer_buffer = [0; BUFFER_SIZE * 10];
            // info!("Writing {:?}", transfer_buffer);
            for (idx, el) in (10..(10 + super::BUFFER_SIZE)).enumerate() {
                cx.local.BUF[idx] = el as u8;
                info!("Cx : {:?}", cx.local.BUF[idx]);
            }
            info!("Writing {:?}", cx.local.BUF);

            // embedded_hal::spi::SpiBus::transfer(cx.local.spim, cx.local.BUF, &[
            //     0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
            // ])
            // .unwrap_or_else(|_| panic!());
            cx.local
                .spim
                .transfer(cx.local.spim_cs, cx.local.BUF)
                .unwrap_or_else(|_| panic!());
            info!("Buffer after transfer {:?}", cx.local.BUF);
            if !cx.local.BUF.map(|val| val != 255).iter().all(|val| *val) {
                info!("Transfer did not work");
            }
            Mono::delay(1.secs()).await;
        }
    }
}
