//! Defines a few exports that allow us to do simple computer vision tasks.
#![feature(const_trait_impl)]
use buffer::{Buffer, GrayScale};
use image::{GrayImage, ImageResult, Luma};

pub mod buffer;
pub mod graphical;
pub mod kernel;
pub mod rgb_stream;
pub mod transform;

pub(crate) mod sealed {
    pub trait Conversions<Target: Sized> {
        fn convert_to(self) -> Target;
        fn convert_from(value: Target) -> Self;
    }
}
use sealed::*;

pub struct HighLight {
    x: usize,
    y: usize,
}

pub fn display_buffer(buffer: &Buffer<'_, GrayScale>, path: &'static str) -> ImageResult<()> {
    let width = buffer.width;
    let mut image_buffer = GrayImage::new(buffer.width as u32, buffer.height as u32);
    for (y, row) in image_buffer.rows_mut().enumerate() {
        for (x, pixel) in row.enumerate() {
            let idx = y * width + x;

            *pixel = Luma([buffer.buffer[idx]]);
        }
    }

    image_buffer.save(path)
}

impl Conversions<i32> for i8 {
    fn convert_to(self) -> i32 {
        self as i32
    }

    // This can panic and should really be handled.
    fn convert_from(value: i32) -> Self {
        value as i8
    }
}

impl Conversions<i32> for i32 {
    fn convert_to(self) -> i32 {
        self as i32
    }

    // This can panic and should really be handled.
    fn convert_from(value: i32) -> Self {
        value as i32
    }
}
impl Conversions<i32> for f32 {
    fn convert_to(self) -> i32 {
        self as i32
    }

    // This can panic and should really be handled.
    fn convert_from(value: i32) -> Self {
        value as f32
    }
}
