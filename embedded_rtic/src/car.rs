//! defines specifics for the car
use defmt::Format;
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
