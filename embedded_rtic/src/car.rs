//! BSP for our car, defines some specifics for out car.

pub mod event;
pub mod pin_map;

#[allow(non_snake_case)]
/// A collection of PID parameters.
pub struct PidParams {
    /// The proportional error coefficient.
    pub KP: i32,
    /// The integral of error coefficient.
    pub KI: i32,
    /// The derivative of error coefficient.
    pub KD: i32,
    // 10 ^ 1
    /// log 10 of scale.
    pub SCALE: u32,

    /// Sample time.
    pub TS: i32,
    /// Fraction of seconds, so uS => 10^6.
    pub TIMESCALE: i32,
}

/// Defines a few intrinsic constants for the car.
pub mod constants {
    use defmt::Format;
    use shared::gain_scheduling::GainParams;

    use super::PidParams;

    /// The PID parameters for the ESC.
    pub const ESC_PID_PARAMS: PidParams = PidParams {
        KP: 30,
        KI: 60,
        KD: 10,
        // 10^2
        SCALE: 2,
        TS: 50_000,
        TIMESCALE: 1_000_000,
    };

    /// The PID parameters for the ESC.
    // pub const SERVO_PID_PARAMS: PidParams = PidParams {
    // KP: 150,
    // KI: 5,
    // KD: 60,
    // 10^3
    // SCALE: 3,
    // This is not used.
    // TS: 60_000, // 4 Hz should probly be higher
    // TIMESCALE: 1_000_000,
    // };

    pub const SERVO_SCALE: u32 = 2;

    /// The type of gain store used.
    pub type GAINSTORE = [(
        f32,
        [Option<(f32, [Option<GainParams<SERVO_SCALE>>; 2])>; 2],
    ); 2];

    /// The scheduling ranges.
    pub const SERVO_PID_PARAMS_SCHEDULE: GAINSTORE = [
        // 0..50 cm from the wall we simply use a really harsh PID.
        (0., [
            Some((0., [
                Some(GainParams {
                    kp: 150,
                    ki: 0,
                    kd: 0,
                    max_value: f32::MAX,
                    min_value: 0.,
                }),
                None,
            ])),
            None,
        ]),
        // Distance from wall to start using the really harsh PID.
        (50., [
            Some((0., [
                Some(GainParams {
                    kp: 10,
                    ki: 3,
                    kd: 12,
                    max_value: 60.,
                    min_value: 2.,
                }),
                Some(GainParams {
                    kp: { 20 / 3 },
                    ki: { 4 / 2 },
                    kd: { 15 / 3 },
                    max_value: 150.,
                    min_value: 60.,
                }),
            ])),
            // If the average distance to the valls is large we use a really light PID.
            Some((80., [
                Some(GainParams {
                    kp: 150,
                    ki: 0,
                    kd: 0,
                    max_value: f32::MAX,
                    min_value: 0.,
                }),
                None,
            ])),
        ]),
    ];

    // Some(GainParams {
    //     kp: { 20 / 3 },
    //     ki: { 4 / 2 },
    //     kd: { 15 / 3 },
    //     max_value: 2.,
    //     min_value: 1.,
    // }),
    /// The message queue capacity.
    pub const CAPACITY: usize = 100;

    /// How much smoothing should be applied to signals.
    pub const SMOOTHING: usize = 10;

    /// The magnet spacing in the rotary encoder.
    pub const MAGNET_SPACING: u32 = 31415/4/* 2 * 31415 / 3 */;

    /// The wheel radius in centimeters.
    pub const RADIUS: u64 = 3;

    /// How much can a sonar value increase/decrease wihout being concidered an
    /// outlier.
    pub const OUTLIER_LIMIT: f32 = 150.;

    /// How many outliers in a row have to occur before we accept that it is the
    /// truth?
    pub const VOTE_THRESH: usize = 2;

    /// How far before we should slow the car down a notch?
    pub const OHSHIT_MAP: [(f32, Option<f32>); 6] = [
        (30., Some(0.)),
        (40., Some(0.)),
        (60., Some(10.)),
        (100., Some(20.)),
        (150., Some(40.)),
        (170., None),
    ];

    /// Sonar Channels for multiple.
    #[derive(Copy, Clone, Format)]
    pub enum Sonar {
        /// Sonar positions: Forward
        Forward,
        /// Sonar position: Left
        Left,
        /// Second Sonar position: Left
        Left2,
        /// Sonar positions: Right
        Right,
        /// Second Sonar position: Right
        Right2,
    }

    /// The minimum measurable velocity in cm / s.
    ///
    /// Any velocity lower than this will potentially cause
    /// re-use of previous velocity when controlling.
    ///
    /// One possible fix for this is to apply an averaging filter but this
    pub const MIN_VEL: i32 = ((RADIUS as f32
        * ((MAGNET_SPACING as f32 / 10_000f32)
            / (ESC_PID_PARAMS.TS as f32 / ESC_PID_PARAMS.TIMESCALE as f32)))
        * 100f32) as i32;
}

/// Defines wrapper types that make the [`rtic`] code easier to read.
pub mod wrappers {
    use nrf52840_hal::pac::{PWM0, PWM1, SPIS0};
    pub use rtic_monotonics::nrf::timer::Timer0 as Mono;
    use shared::{controller::Pid, gain_scheduling::GainScheduler};

    use super::constants::{ESC_PID_PARAMS, GAINSTORE, SERVO_SCALE};

    /// The spi device used.
    pub type SpiInstance = SPIS0;

    /// The PWM in charge of controlling the [`Esc`](crate::esc::Esc)
    pub type EscPwm = PWM0;
    /// The PWM in charge of controlling the [`Servo`](crate::servo::Servo)
    pub type ServoPwm = PWM1;

    /// The duration type.
    pub type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

    /// A wrapper around the [`Pid`] controller with predefined coefficients.
    pub type MotorController<PWM> = Pid<
        crate::esc::Error,
        crate::esc::Esc<PWM>,
        f32,
        1,
        { ESC_PID_PARAMS.KP },
        { ESC_PID_PARAMS.KI },
        { ESC_PID_PARAMS.KD },
        { ESC_PID_PARAMS.TS },
        1000,
        -1000,
        { ESC_PID_PARAMS.TIMESCALE },
        { ESC_PID_PARAMS.SCALE },
    >;

    /// A wrapper around the [`Pid`] controller with predefined coefficients.
    pub type ServoController<PWM> = GainScheduler<
        crate::servo::Error,
        crate::servo::Servo<PWM>,
        GAINSTORE,
        10,
        -10,
        { ESC_PID_PARAMS.TIMESCALE },
        SERVO_SCALE,
    >;
}
