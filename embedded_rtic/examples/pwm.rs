#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

#[rtic::app(device = embassy_nrf, peripherals = false, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use cortex_m::asm::delay;
    use defmt::info;
    use embassy_nrf::pwm::{Prescaler, SimplePwm};
    use embassy_time::Timer;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        info!("Hello World!");

        let p = embassy_nrf::init(Default::default());
        let mut pwm = SimplePwm::new_1ch(p.PWM0, p.P0_05);
        // sg90 microervo requires 50hz or 20ms period
        // set_period can only set down to 125khz so we cant use it directly
        // Div128 is 125khz or 0.000008s or 0.008ms, 20/0.008 is 2500 is top
        pwm.set_prescaler(Prescaler::Div128);
        pwm.set_max_duty(2500);
        info!("pwm initialized!");

        // 1ms 0deg (1/.008=125), 1.5ms 90deg (1.5/.008=187.5), 2ms 180deg (2/.008=250),
        loop {
            info!("45 deg");
            // poor mans inverting, subtract our value from max_duty
            pwm.set_duty(0, 0);
            delay(5000000);

            info!("90 deg");
            pwm.set_duty(0, 1000);
            delay(5000000);

            info!("135 deg");
            pwm.set_duty(0, 1500);
            delay(5000000);

            info!("180 deg");
            pwm.set_duty(0, 2000);
            delay(5000000);

            info!("0 deg");
            pwm.set_duty(0, 2500);
            delay(5000000);
        }
    }
}
