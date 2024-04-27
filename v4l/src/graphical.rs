use std::{fmt::Debug, ops::IndexMut};

use crate::buffer::{Buffer, ColorCode};

pub trait Draw<'buffer, Color: ColorCode> {
    fn draw(&self, buffer: &mut Buffer<'buffer, Color>, marker: Color::Marker);
}

pub struct Circle {
    center: (usize, usize),
    radius: usize,
}

impl Circle {
    pub fn new(center: (usize, usize), radius: usize) -> Self {
        Self { center, radius }
    }
}

#[derive(Debug)]
pub struct Line {
    rho: usize,
    theta: f32,
    start: (usize, usize),
    stop: (usize, usize),
}

impl Line {
    pub fn new(rho: usize, theta: f32, start: (usize, usize), stop: (usize, usize)) -> Self {
        Self {
            rho,
            theta,
            start,
            stop,
        }
    }
}

impl<'buffer, Color: ColorCode> Draw<'buffer, Color> for Line
where
    Buffer<'buffer, Color>: IndexMut<(usize, usize), Output = Color::Marker>,
    Color::Marker: Debug,
{
    fn draw(&self, buffer: &mut Buffer<'buffer, Color>, marker: Color::Marker) {
        let ypx =
            (self.stop.1 as f32 - self.start.1 as f32) / (self.stop.0 as f32 - self.start.0 as f32);

        if self.start.0 == self.stop.0 {
            let x = self.start.0;
            for y in self.start.1..((self.stop.1 + 1).min(buffer.height)) {
                buffer[(x, y)] = marker.clone();
            }
            return;
        }

        for x in self.start.0..((self.stop.0 + 1).min(buffer.width)) {
            let (theta, rho, x_inner) = (self.theta, self.rho as f32, x as f32);
            for i in 0..((1. / ypx) as usize) {
                let y = (ypx * (i as f32) * (x as f32 - self.start.0 as f32)) as usize; //((-theta.cos() / theta.sin()) * x_inner + rho / theta.sin()).max(0.) as usize;

                if y < buffer.height && self.start.1 < y && self.stop.1 > y {
                    buffer[(x, y)] = marker.clone();
                } else {
                }
            }
        }
    }
}

impl<'buffer, Color: ColorCode> Draw<'buffer, Color> for Circle
where
    Buffer<'buffer, Color>: IndexMut<(usize, usize), Output = Color::Marker>,
{
    fn draw(&self, buffer: &mut Buffer<'buffer, Color>, marker: Color::Marker) {
        let x_min = match self.center.0 <= self.radius {
            true => 0,
            false => self.center.0 - self.radius,
        };
        let x_max = match self.center.0 + self.radius > buffer.width {
            true => buffer.width,
            false => self.center.0 + self.radius,
        };

        let y_min = match self.center.1 <= self.radius {
            true => 0,
            false => self.center.1 - self.radius,
        };
        let y_max = match self.center.1 + self.radius > buffer.height {
            true => buffer.height,
            false => self.center.1 + self.radius,
        };

        for x in x_min..x_max {
            for y in y_min..y_max {
                let (x, y) = (x as f32, y as f32);
                let radius = ((x - self.center.0 as f32).powf(2.)
                    + (y - self.center.1 as f32).powf(2.))
                .sqrt() as usize;

                if radius == self.radius {
                    buffer[(x as usize, y as usize)] = marker.clone();
                }
            }
        }
    }
}
