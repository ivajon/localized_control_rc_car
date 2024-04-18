use core::mem::size_of;

use crate::OwnedItterator;

use super::{Parse, Version};

/// The first version of the protocol.
pub type V0_0_1 = ();

impl Version for V0_0_1 {
    const VERSION_ID: Self::BusItem = 0b00_00_00_01;
    const PACKET_SIZE: usize = Payload::MAX_BUFFER_REQUIRED + Self::HEADER_SIZE;
    type BusItem = u8;
    type Payload = Payload;
}

/// The payload for [`V0_0_1`] of the protocol.
#[derive(PartialEq, Clone, Copy)]
pub enum Payload {
    /// Sets the speed of the esc to the specified velocity in cm/s.
    ///
    /// If hold_for_us is 0 the speed will be held until a new speed is requested.
    SetSpeed { velocity: u32, hold_for_us: u64 },
    /// Sets the steering angle of the motor.
    ///
    /// If hold_for_us is 0 the angle will be held until a new angle is requested.
    SetSteeringAngle { angle: i32, hold_for_us: u64 },
    /// The current velocity of the car, timestamped with the measurement on the car.
    CurrentVelocity { velocity: u32, time_us: u64 },
    /// The current angle of the steering servo on the car.
    ///
    /// Timestamped on the car.
    CurrentAngle { angle: i32, time_us: u64 },
    /// The current distance that the car has traveled.
    ///
    /// Timestamped on the car.
    CurrentDistance { distance: u32, time_us: u64 },
    /// Clears all requested control signals, this is used when we have planned a path until
    /// completion.
    ClearQueue,
}

impl Payload {
    /// For this payload we need one u64, one i32 or u32 and on byte to indicate payload type.
    const MAX_BUFFER_REQUIRED: usize = { size_of::<u64>() + size_of::<i32>() + 1 };
}

impl IntoIterator for Payload {
    type Item = u8;
    type IntoIter = OwnedItterator<Self::Item, { Self::MAX_BUFFER_REQUIRED }>;

    fn into_iter(self) -> Self::IntoIter {
        let mut ret = [0; Self::MAX_BUFFER_REQUIRED];
        let mut ptr: usize = 0;
        let extend = |target: &mut [u8], src: &[u8], ptr: &mut usize| {
            for el in src {
                target[*ptr] = *el;
                *ptr += 1;
            }
        };
        match self {
            Self::SetSpeed {
                velocity,
                hold_for_us,
            } => {
                ret[ptr] = 0;
                ptr += 1;
                extend(&mut ret, &mut velocity.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut hold_for_us.to_be_bytes(), &mut ptr);
            }
            Self::SetSteeringAngle { angle, hold_for_us } => {
                ret[ptr] = 1;
                ptr += 1;
                extend(&mut ret, &mut angle.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut hold_for_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentVelocity { velocity, time_us } => {
                ret[ptr] = 2;
                ptr += 1;
                extend(&mut ret, &mut velocity.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentAngle { angle, time_us } => {
                ret[ptr] = 3;
                ptr += 1;
                extend(&mut ret, &mut angle.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentDistance { distance, time_us } => {
                ret[ptr] = 4;
                ptr += 1;
                extend(&mut ret, &mut distance.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }

            Self::ClearQueue => {
                ret[ptr] = 5;
            }
        };
        OwnedItterator::new(ret)
    }
}

impl Parse for Payload {
    type Item = u8;
    fn try_parse<T: Iterator<Item = Self::Item>>(stream: &mut T) -> Option<Self>
    where
        Self: Sized,
    {
        let packet_indication = stream.next()?;
        match packet_indication {
            0 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let velocity: u32 = u32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let hold_for_us: u64 = u64::from_be_bytes(next);
                Some(Self::SetSpeed {
                    velocity,
                    hold_for_us,
                })
            }
            1 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let angle: i32 = i32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let hold_for_us: u64 = u64::from_be_bytes(next);
                Some(Self::SetSteeringAngle { angle, hold_for_us })
            }
            2 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let velocity: u32 = u32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let time_us: u64 = u64::from_be_bytes(next);
                Some(Self::CurrentVelocity { velocity, time_us })
            }
            3 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let angle: i32 = i32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let time_us: u64 = u64::from_be_bytes(next);
                Some(Self::CurrentAngle { angle, time_us })
            }
            4 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let distance: u32 = u32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let time_us: u64 = u64::from_be_bytes(next);
                Some(Self::CurrentDistance { distance, time_us })
            }
            5 => Some(Self::ClearQueue),
            _ => None,
        }
    }
}
