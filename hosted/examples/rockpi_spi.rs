use std::{fmt, io, path::Path};

use embedded_hal::spi::{FullDuplex, Mode};
use gpio_cdev::{Chip, LineRequestFlags};

/// Define a struct to represent the SPI device
pub struct SpiDevice {
    spidev: spidev::Spidev,
    cs_pin: gpio_cdev::Line,
}

impl SpiDevice {
    // struct for rock pi pins
    pub fn open<P>(
        path: P,
        mosi_pin_num: u32,
        miso_pin_num: u32,
        sck_pin_num: u32,
        cs_pin_num: u32,
    ) -> Result<Self, SPIError>
    where
        // cheap reference-to-reference for the path to rock pi
        P: AsRef<Path>,
    {
        // Open SPI device
        let spidev = spidev::Spidev::open(path)?;

        // Configure MOSI, MISO, SCK and CS pins
        let mut chip = Chip::new("/dev/spi0")?; // i think this is the path

        // CNF mosi pin
        let mosi_pin = chip.get_line(mosi_pin_num)?;
        mosi_pin.request(LineRequestFlags::OUTPUT, 21, "mosi")?;

        // CNF miso pin
        let miso_pin = chip.get_line(miso_pin_num)?;
        miso_pin.request(LineRequestFlags::INPUT, 19, "miso")?;

        // CNF sck pin
        let sck_pin = chip.get_line(sck_pin_num)?;
        sck_pin.request(LineRequestFlags::OUTPUT, 23, "sck")?;

        // Set up CS pin
        let cs_pin = chip.get_line(cs_pin_num)?;
        cs_pin.request(LineRequestFlags::OUTPUT, 24, "cs_pin")?;

        Ok(SpiDevice { spidev, cs_pin })
    }

    fn set_cs_active(&mut self, active: bool) -> io::Result<()> {
        self.cs_pin.set_value(if active { 0 } else { 1 })?;
        Ok(())
    }
}

// implements Full Duplex which means we can send and receive data
impl FullDuplex<u8> for SpiDevice {
    type Error = io::Error;

    fn send_data(&mut self, word: u8) -> Result<(), Self::Error> {
        // Set CS pin active
        self.set_cs_active(true)?;

        // Send data over SPI
        self.spidev.write(&[word])?;

        // Set CS pin inactive
        self.set_cs_active(false)?;

        Ok(())
    }

    fn read_data(&mut self) -> Result<u8, Self::Error> {
        // Set CS pin active
        self.set_cs_active(true)?;

        // actual read
        let mut buf = [0];
        self.spidev.read(&mut buf)?;

        // Set CS pin inactive
        self.set_cs_active(false)?;

        Ok(buf[0])
    }
}

fn main() -> io::Result<()> {
    // Open SPI device
    let mut spi = SpiDevice::open(
        "/dev/gpiochip0",
        mosi_pin_num,
        miso_pin_num,
        sck_pin_num,
        cs_pin,
    )?;

    // Define data to be sent from Rock Pi
    let response_data: u8 = 0x55;

    // Create a buffer for received data
    let mut received_data = [0u8; 3];

    // sends data
    for byte in &mut received_data {
        spi.send_data(response_data)?;
        *byte = spi.read_data()?;
    }

    // Display received data
    println!("Received data: {:?}", received_data);

    Ok(())
}

// Here we can define custom methods for IO errors
#[derive(Debug)]
pub struct SPIError {
    err: io::Error,
}

// fmt error trait for SPIError
impl fmt::Display for SPIError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SPI error: {}", self.err)
    }
}

// Standard error trait for SPIError
impl std::error::Error for SPIError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.err)
    }
}
