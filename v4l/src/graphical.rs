use std::{
    fmt::{Debug, Display},
    ops::IndexMut,
};

use crate::{buffer::Buffer, color_code::ColorCode};

pub trait Draw<Color: ColorCode> {
    fn draw(&self, buffer: &mut Buffer<Color>, marker: Color::Marker);
}

pub struct Circle {
    pub center: (isize, isize),
    radius: isize,
}

impl Circle {
    pub fn new(center: (isize, isize), radius: isize) -> Self {
        Self { center, radius }
    }
}

#[derive(Debug)]
pub struct Line {
    rho: isize,
    theta: f32,
    pub(crate) start: (usize, usize),
    pub(crate) stop: (usize, usize),
}

impl Line {
    pub fn new(rho: isize, theta: f32, start: (usize, usize), stop: (usize, usize)) -> Self {
        Self {
            rho,
            theta,
            start,
            stop,
        }
    }
}

impl<Color: ColorCode> Draw<Color> for Line
where
    Buffer<Color>: IndexMut<(usize, usize), Output = Color::Marker>,
    Color::Marker: Debug + Display,
{
    fn draw(&self, buffer: &mut Buffer<Color>, marker: Color::Marker) {
        let ypx =
            (self.stop.1 as f32 - self.start.1 as f32) / (self.stop.0 as f32 - self.start.0 as f32);

        if self.start.0 == self.stop.0 {
            let x = self.start.0;

            for y in self.start.1..((self.stop.1 + 1).min(buffer.height)) {
                buffer[(x, y)] = marker.clone();
            }
            return;
        }
        let mut y = self.start.1 as f32;
        for x in self.start.0..((self.stop.0 + 1).min(buffer.width)) {
            /*
            let (theta, rho, x_inner) = (self.theta, self.rho as f32, x as f32);
            let y = ((-theta.cos() / theta.sin()) * x_inner + rho / theta.sin());
            println!("Y float {y}");
            let y = y.abs() as usize;
            */

            if y < buffer.height as f32 {
                buffer[(x, y.abs() as usize)] = marker.clone();
            } else {
                // THIS SHOULD BE UNREACHABLE
            }
            y += ypx;
        }
    }
}

impl<Color: ColorCode> Draw<Color> for Circle
where
    Buffer<Color>: IndexMut<(usize, usize), Output = Color::Marker>,
{
    fn draw(&self, buffer: &mut Buffer<Color>, marker: Color::Marker) {
        let x_min = match self.center.0 <= self.radius {
            true => 0,
            false => self.center.0 - self.radius,
        };

        let x_max = match (self.center.0 + self.radius) as usize > buffer.width {
            true => buffer.width,
            false => (self.center.0 + self.radius) as usize,
        };

        let y_min = match self.center.1 <= self.radius {
            true => 0,
            false => self.center.1 - self.radius,
        };

        let y_max = match (self.center.1 + self.radius) as usize > buffer.height {
            true => buffer.height,
            false => (self.center.1 + self.radius) as usize,
        };

        for x in x_min..(x_max as isize) {
            for y in y_min..(y_max as isize) {
                let (x, y) = (x as f32, y as f32);
                let radius = ((x - self.center.0 as f32).powf(2.)
                    + (y - self.center.1 as f32).powf(2.))
                .sqrt() as isize;

                if radius == self.radius {
                    buffer[(x as usize, y as usize)] = marker.clone();
                }
            }
        }
    }
}
