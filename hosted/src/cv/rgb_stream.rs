extern crate jpeg_decoder as jpeg;
use std::{io::BufReader, marker::PhantomData};

use eye_hal::{
    stream::Descriptor,
    traits::{Context, Device, Stream},
    Error, PlatformContext,
};

use super::{buffer::Buffer, color_code::ColorCode};

pub struct VideoStream<'device, Color: ColorCode> {
    stream: eye_hal::platform::Stream<'device>,
    description: Descriptor,
    color: PhantomData<Color>,
}

impl<'device, Color: ColorCode> VideoStream<'device, Color> {
    pub fn new() -> Result<Self, Error> {
        let ctx = PlatformContext::default();

        // Query for available devices.
        let devices = ctx.devices()?;

        // First, we need a capture device to read images from. For this example, let's
        // just choose whatever device is first in the list.
        let dev = ctx.open_device(&devices[0].uri)?;

        // Query for available streams and just choose the first one.
        let streams = dev.streams()?;
        let description = streams[0].clone();

        let stream = dev.start_stream(&description)?;

        Ok(Self {
            stream,
            color: PhantomData,
            description,
        })
    }

    pub fn interval(&self) -> std::time::Duration {
        self.description.interval
    }
    pub fn dimensions(&self) -> (usize, usize) {
        (
            self.description.width as usize,
            self.description.height as usize,
        )
    }
}

impl<'device, Color: ColorCode> Iterator for &mut VideoStream<'device, Color> {
    type Item = Buffer<Color>;

    fn next(&mut self) -> Option<Self::Item> {
        let frame = match self.stream.next() {
            Some(v) => match v {
                Ok(v) => v,
                _ => return None,
            },
            _ => return None,
        };

        let mut decoder = jpeg::Decoder::new(BufReader::new(frame.clone()));

        let pixels = match decoder.decode() {
            Ok(v) => v,
            _ => return None,
        };

        Some(Buffer::new(
            Color::from_rgb(&pixels),
            self.description.width as usize,
            self.description.height as usize,
        ))
    }
}
