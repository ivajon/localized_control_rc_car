use std::{io::Read, path::Path};

use shared::protocol::{Message, Parse, Version};
use spidev::{spidevioctl::spi_ioc_transfer, SpiModeFlags, Spidev, SpidevOptions};

/// Enumerates the accounted for edge-cases when using the [`Spi`].
#[derive(Debug)]
pub enum Error {
    /// Thrown when there is no data to read on the device.
    NothingToRead,
    /// Thrown when the data was not a value [`Message`]
    InvalidData,

    /// A generic io error.
    IoError(std::io::Error),
}

pub struct Spi<V: Version> {
    spi: Spidev,
    buffered: Vec<V::Payload>,
}

impl<V: Version<BusItem = u8>> Spi<V> {
    /// Creates a neat SPI abstraction that allows us to read/write protocol packets.
    pub fn init<P: AsRef<Path>>(path: P) -> Result<Self, Error> {
        let mut spi = match Spidev::open(path) {
            Ok(value) => value,
            Err(e) => return Err(Error::IoError(e)),
        };

        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(1_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build();

        match spi.configure(&options) {
            Ok(_) => {}
            Err(e) => return Err(Error::IoError(e)),
        }

        Ok(Self {
            spi,
            buffered: Vec::new(),
        })
    }

    /// Reads a new value from the device returning the new value any any values buffered
    /// from [`write`](Self::write).
    pub fn read(&mut self) -> Result<Vec<V::Payload>, Error>
    where
        <<V as Version>::Payload as IntoIterator>::IntoIter: Clone,
    {
        let mut data = [0; 100];
        let read = self.spi.read(&mut data).map_err(Error::IoError)?;
        if read == 0 {
            return Err(Error::NothingToRead);
        }

        match shared::protocol::Message::<V>::try_parse(&mut data.into_iter()) {
            Some(message) => self.buffered.push(message.payload()),
            _ => {
                return Err(Error::InvalidData);
            }
        }
        Ok(self.buffered.drain(..).collect())
    }

    /// Writes to the client, buffering any read values to be sent whenever [`read`](Self::read) is
    /// called.
    pub fn write(&mut self, payload: V::Payload) -> Result<(), Error>
    where
        <<V as Version>::Payload as IntoIterator>::IntoIter: Clone,
    {
        let target: Vec<u8> = Message::<V>::new(payload).collect();
        // Make a really large buffer, this is a "fulhack" but it is fine for now.
        let mut rx_buff: Vec<u8> = target.iter().map(|_| 0).collect();

        let mut xfer = spi_ioc_transfer::read_write(target.as_slice(), &mut rx_buff);
        self.spi.transfer(&mut xfer).map_err(Error::IoError)?;

        if let Some(message) = shared::protocol::Message::<V>::try_parse(&mut rx_buff.into_iter()) {
            self.buffered.push(message.payload())
        }

        Ok(())
    }
}
