//! BSP for our car, defines some specifics for out car.

use shared::controller::Pid;

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
    use super::PidParams;
    use defmt::Format;

    /// The PID parameters for the ESC.
    pub const ESC_PID_PARAMS: PidParams = PidParams {
        KP: 23,
        KI: 21,
        KD: 11,
        // 10^1
        SCALE: 1,
        TS: 200,
        TIMESCALE: 1_000_000,
    };

    /// The magnet spacing in the rotary encoder.
    pub const MAGNET_SPACING: u32 = 2 * 31415 / 3;

    /// The wheel radius in centimeters.
    pub const RADIUS: u64 = 3;

    /// Sonar Channels for multiple.
    #[derive(Copy, Clone, Format)]
    pub enum Sonar {
        /// Sonar positions: Forward
        Forward,
        /// Sonar position: Left
        Left,
        /// Sonar positions: Right
        Right,
    }
}

/// Defines wrapper types that make the [`rtic`] code easier to read.
pub mod wrappers {
    use super::{constants::ESC_PID_PARAMS, Pid};

    /// A wrapper around the [`Pid`] controller with predefined coefficients.
    pub type MotorController<PWM> = Pid<
        crate::esc::Error,
        crate::esc::Esc<PWM>,
        i32,
        1,
        { ESC_PID_PARAMS.KP },
        { ESC_PID_PARAMS.KI },
        { ESC_PID_PARAMS.KD },
        { ESC_PID_PARAMS.TS },
        -100,
        100,
        { ESC_PID_PARAMS.TIMESCALE },
        { ESC_PID_PARAMS.SCALE },
    >;
}
