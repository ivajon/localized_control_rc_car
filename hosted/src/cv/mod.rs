//! Defines a few exports that allow us to do simple computer vision tasks.
use buffer::Buffer;
use color_code::ColorCode;
use image::{GrayImage, ImageResult, Luma};

pub mod buffer;
pub mod camera_monitor;
pub mod color_code;
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
use ratatui::widgets::{
    canvas::{Canvas, Points, Shape},
    Widget,
};
use sealed::*;

pub struct HighLight {
    x: usize,
    y: usize,
}

pub fn display_buffer<Color: ColorCode<Marker = u8>>(
    buffer: &Buffer<Color>,
    path: &'static str,
) -> ImageResult<()> {
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

pub fn draw_on_canvas<Color: ColorCode<Marker = u8>>(
    buffer: &Buffer<Color>,
    features: Vec<impl Shape>,
) -> impl Widget {
    let points: Vec<HighLight> = buffer.into();
    Canvas::default()
        .x_bounds([0., buffer.width as f64])
        .y_bounds([0., buffer.height as f64])
        .paint(move |ctx| {
            points.iter().for_each(|HighLight { x, y }| {
                ctx.draw(&Points {
                    coords: &[(*x as f64, *y as f64)],
                    color: Color::get_color(&Color::highlight()),
                })
            });

            features.iter().for_each(|el| ctx.draw(el));
        })
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
