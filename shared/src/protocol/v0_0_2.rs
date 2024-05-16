use core::mem::size_of;

use super::{Parse, Version};
use crate::OwnedItterator;

/// The first version of the protocol.
pub struct V0_0_2 {}

impl Version for V0_0_2 {
    type BusItem = u8;
    type Payload = Payload;

    const PACKET_SIZE: usize = Payload::MAX_BUFFER_REQUIRED + Self::HEADER_SIZE;
    const VERSION_ID: Self::BusItem = 0b00_00_00_10;
}

/// The payload for [`V0_0_2`] of the protocol.
#[derive(defmt::Format, PartialEq, Clone, Copy)]
pub enum Payload {
    /// Sets the speed of the esc to the specified velocity in cm/s.
    SetSpeed { velocity: u32 },

    /// Sets the desired position for the car in terms of the difference in
    /// distance between the right wall and the left wall. (cm)
    SetPosition { position: i32 },

    /// The current velocity of the car, timestamped with the measurement on the
    /// car.
    CurrentVelocity { velocity: u32, time_us: u64 },

    /// The current position of the car, i.e. the difference between the
    /// distance to the right wall and the distance to the left wall.
    CurrentPosition { position: i32, time_us: u64 },

    /// The current distance that the car has traveled.
    ///
    /// Timestamped on the car.
    CurrentDistance { distance: u32, time_us: u64 },

    /// The current state of the car.
    CurrentState {
        velocity: u32,
        position: i32,
        distance: u32,
        time_us: u64,
    },
}

impl Payload {
    /// For this payload we need one u64, one i32 or u32 and on byte to indicate
    /// payload type.
    const MAX_BUFFER_REQUIRED: usize = { size_of::<u64>() + size_of::<i32>() * 3 + 1 };
}

impl IntoIterator for Payload {
    type IntoIter = OwnedItterator<Self::Item, { Self::MAX_BUFFER_REQUIRED }>;
    type Item = u8;

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
            Self::SetSpeed { velocity } => {
                ret[ptr] = 0;
                ptr += 1;
                extend(&mut ret, &mut velocity.to_be_bytes(), &mut ptr);
            }
            Self::SetPosition { position } => {
                ret[ptr] = 1;
                ptr += 1;
                extend(&mut ret, &mut position.to_be_bytes(), &mut ptr);
            }
            Self::CurrentVelocity { velocity, time_us } => {
                ret[ptr] = 2;
                ptr += 1;
                extend(&mut ret, &mut velocity.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentPosition { position, time_us } => {
                ret[ptr] = 3;
                ptr += 1;
                extend(&mut ret, &mut position.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentDistance { distance, time_us } => {
                ret[ptr] = 4;
                ptr += 1;
                extend(&mut ret, &mut distance.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
            }
            Self::CurrentState {
                velocity,
                position,
                distance,
                time_us,
            } => {
                ret[ptr] = 5;
                ptr += 1;
                extend(&mut ret, &mut velocity.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut position.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut distance.to_be_bytes(), &mut ptr);
                extend(&mut ret, &mut time_us.to_be_bytes(), &mut ptr);
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

                Some(Self::SetSpeed { velocity })
            }
            1 => {
                let next = match stream.next_chunk::<4>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let position: i32 = i32::from_be_bytes(next);

                Some(Self::SetPosition { position })
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
                let position: i32 = i32::from_be_bytes(next);

                let next = match stream.next_chunk::<8>() {
                    Ok(next) => next,
                    _ => return None,
                };
                let time_us: u64 = u64::from_be_bytes(next);
                Some(Self::CurrentPosition { position, time_us })
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
            5 => {
                // velocity,
                // position,
                // distance,
                // time_us,
                let next = match stream.next_chunk() {
                    Ok(next) => next,
                    _ => return None,
                };
                let velocity = u32::from_be_bytes(next);

                let next = match stream.next_chunk() {
                    Ok(next) => next,
                    _ => return None,
                };
                let position = i32::from_be_bytes(next);

                let next = match stream.next_chunk() {
                    Ok(next) => next,
                    _ => return None,
                };
                let distance = u32::from_be_bytes(next);

                let next = match stream.next_chunk() {
                    Ok(next) => next,
                    _ => return None,
                };
                let time_us = u64::from_be_bytes(next);

                Some(Self::CurrentState {
                    velocity,
                    position,
                    distance,
                    time_us,
                })
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod test {

    use super::Payload;
    use crate::protocol::Parse;

    #[test]
    fn test_set_speed() {
        let source = Payload::SetSpeed { velocity: 123 };
        let mut to_send = source.clone().into_iter();

        let received = Payload::try_parse(&mut to_send).unwrap();

        assert!(source == received);
    }

    #[test]
    fn test_set_angle() {
        let source = Payload::SetPosition { position: 123 };
        let mut to_send = source.clone().into_iter();

        let received = Payload::try_parse(&mut to_send).unwrap();

        assert!(source == received);
    }

    #[test]
    fn test_get_speed() {
        let source = Payload::CurrentVelocity {
            velocity: 123,
            time_us: 10_000,
        };
        let mut to_send = source.clone().into_iter();

        let received = Payload::try_parse(&mut to_send).unwrap();

        assert!(source == received);
    }

    #[test]
    fn test_get_angle() {
        let source = Payload::CurrentPosition {
            position: 123,
            time_us: 10_000,
        };
        let mut to_send = source.clone().into_iter();

        let received = Payload::try_parse(&mut to_send).unwrap();

        assert!(source == received);
    }

    #[test]
    fn test_get_distance() {
        let source = Payload::CurrentDistance {
            distance: 123,
            time_us: 10_000,
        };
        let mut to_send = source.clone().into_iter();

        let received = Payload::try_parse(&mut to_send).unwrap();

        assert!(source == received);
    }
}
