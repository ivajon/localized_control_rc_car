//! Defines a few wrapper types.

/// Represents an angle in degrees.
#[derive(Clone, Copy)]
pub struct Degrees(i32);
/// Represents an angle in radians.
#[derive(Clone, Copy)]
pub struct Radians(i32);

/// Probides a few helper functions, mainly semantics.
pub trait Exti32 {
    /// Wraps the u32 in a Degrees type.
    fn deg(self) -> Degrees;
    /// Wraps the u32 in a Radians type.
    fn rad(self) -> Radians;
}

impl Exti32 for i32 {
    fn deg(self) -> Degrees {
        Degrees(self)
    }

    fn rad(self) -> Radians {
        Radians(self)
    }
}

impl From<Radians> for Degrees {
    fn from(value: Radians) -> Self {
        // Approximate coersion.
        let value = (value.0 as i64 * 180 * 10000 / 31415) as i32;
        Self(value)
    }
}

impl From<Degrees> for Radians {
    fn from(value: Degrees) -> Self {
        // Approximate coersion.
        let value = ((value.0 as i64 * 31415 / 100) / 1000) as i32;
        Self(value)
    }
}

impl Degrees {
    // Generally we should avoid pub(crate) but we need to
    // be able to deconstruct the wrapper types.
    pub(crate) fn consume(self) -> i32 {
        self.0
    }
}

#[allow(dead_code)]
impl Radians {
    // Generally we should avoid pub(crate) but we need to
    // be able to deconstruct the wrapper types.
    pub(crate) fn consume(self) -> i32 {
        self.0
    }
}
