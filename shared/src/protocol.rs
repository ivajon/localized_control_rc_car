//! Defines the binary protocol used in communication inbetween the boards.

use core::marker::PhantomData;
pub mod v0_0_1;

pub trait Version {
    const HEADER_SIZE: usize = 10;
    const PACKET_SIZE: usize;
    const VERSION_ID: Self::BusItem;

    type BusItem: Sized + Clone + PartialEq;
    /// The payload can be converted to a u8
    type Payload: IntoIterator<Item = Self::BusItem>
        + Parse<Item = Self::BusItem>
        + Clone
        + defmt::Format;
}

// A generic message.
#[derive(Clone)]
pub struct Message<V: Version>
where
    <V::Payload as IntoIterator>::IntoIter: Clone,
{
    version: PhantomData<V>,
    payload: V::Payload,
    version_sent: bool,
    iterator: Option<<V::Payload as IntoIterator>::IntoIter>,
}

impl<V: Version> Message<V>
where
    <V::Payload as IntoIterator>::IntoIter: Clone,
{
    /// The maximum packet size.
    pub const MAX_SIZE: usize = { V::PACKET_SIZE + 1 };

    pub fn new(payload: V::Payload) -> Self {
        Self {
            version: PhantomData,
            payload,
            version_sent: false,
            iterator: None,
        }
    }

    pub fn payload(self) -> V::Payload {
        self.payload
    }
}

pub trait Parse {
    type Item: Sized;

    fn try_parse<T: Iterator<Item = Self::Item>>(stream: &mut T) -> Option<Self>
    where
        Self: Sized;
}

impl<V: Version> Iterator for Message<V>
where
    <V::Payload as IntoIterator>::IntoIter: Clone,
{
    type Item = <V::Payload as IntoIterator>::Item;
    fn next(&mut self) -> Option<Self::Item> {
        if !self.version_sent {
            self.version_sent = true;
            return Some(V::VERSION_ID);
        }

        if self.iterator.is_none() {
            self.iterator = Some(self.payload.clone().into_iter());
        }

        if let Some(iter) = self.iterator.as_mut() {
            let to_send = iter.next()?;
            Some(to_send)
        } else {
            // This should not happen.
            None
        }
    }
}

impl<V: Version> Parse for Message<V>
where
    <V::Payload as IntoIterator>::IntoIter: Clone,
{
    type Item = V::BusItem;
    fn try_parse<T: Iterator<Item = Self::Item>>(stream: &mut T) -> Option<Self>
    where
        Self: Sized,
    {
        let version_id = stream.next()?;

        if version_id != V::VERSION_ID {
            return None;
        }

        Some(Self::new(V::Payload::try_parse(stream)?))
    }
}
