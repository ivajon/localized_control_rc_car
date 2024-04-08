//! Defines a few wrapper types.

/// Represents an angle in degrees.
#[derive(Clone, Copy)]
pub struct Degrees(i32);

/// Represents an angle in radians.
#[derive(Clone, Copy)]
pub struct Radians(i32);

/// Provides helpers to create [`Degrees`] or [`Radians`] from i32.
pub trait Exti32 {
    /// Wraps the u32 in a Degrees type.
    fn deg(self) -> Degrees;
    /// Wraps the u32 in a Radians type.
    fn rad(self) -> Radians;
}

impl Degrees {
    /// Unwraps the type returning the underlying value.
    pub fn consume(self) -> i32 {
        self.0
    }
}

#[allow(dead_code)]
impl Radians {
    /// Unwraps the type returning the underlying value.
    pub fn consume(self) -> i32 {
        self.0
    }
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
        // Approximate coercion.
        //
        // This is not entirely correct but it is correct enough
        let value = (value.0 as i64 * 180 * 10000 / 31415) as i32;
        Self(value)
    }
}

impl From<Degrees> for Radians {
    fn from(value: Degrees) -> Self {
        // Approximate coercion.
        //
        // This is not entirely correct but it is correct enough
        let value = ((value.0 as i64 * 31415 / 100) / 1000) as i32;
        Self(value)
    }
}
