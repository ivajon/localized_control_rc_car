use std::{
    borrow::Borrow,
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Index, IndexMut, Mul},
};

use crate::{kernel::Kernel, sealed::Conversions, transform::Transform, HighLight};

pub trait ColorCode {
    type Marker: Clone;
    fn into_grayscale(current: Vec<u8>) -> Vec<u8>;
    fn from_rgb(current: Vec<u8>) -> Vec<u8>;
}

pub struct RGB {}
impl ColorCode for RGB {
    type Marker = [u8; 3];

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::new();
        let mut cnt = 1;
        let mut sum = 0;
        for el in current {
            sum += el;
            if cnt % 3 == 0 {
                let avg = sum / 3;
                ret.push(avg);
                sum = 0;
            }

            cnt += 1;
        }
        ret
    }

    fn from_rgb(current: Vec<u8>) -> Vec<u8> {
        current
    }
}

pub struct GrayScale {}
impl ColorCode for GrayScale {
    type Marker = u8;

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        current
    }

    fn from_rgb(current: Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::new();
        let mut cnt = 1;
        let mut sum: u32 = 0;
        for el in current {
            sum += el as u32;
            if cnt % 3 == 0 {
                let avg = sum / 3;
                ret.push(avg as u8);
                sum = 0;
            }

            cnt += 1;
        }
        ret
    }
}

pub struct Buffer<'buffer, Color: ColorCode> {
    pub width: usize,
    pub height: usize,
    pub buffer: &'buffer mut [u8],
    color: PhantomData<Color>,
}

fn remap(source: usize, prev_max: usize, new_max: usize) -> usize {
    ((source as f32) / ((prev_max - 1) as f32) * ((new_max - 1) as f32)) as usize
}

impl<'buffer> Buffer<'buffer, GrayScale> {
    pub fn new(buffer: &'buffer mut [u8], width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            buffer,
            color: PhantomData,
        }
    }

    /// Dead simple implementation of a convolution.
    ///
    /// TODO! Break this in to threaded convolutions, we could have a thread
    /// manager that spawns say 4 threads where each region only has a ptr
    /// to some region that does not overlap with other regions.
    /// This is unsafe but we can make it safe if we do it properly.
    pub fn conv<
        T: Sized
            + Mul<T, Output = T>
            + Conversions<i32>
            + AddAssign<T>
            + Default
            + Copy
            + Div<T, Output = T>,
        const KERNEL_WIDTH: usize,
        const KERNEL_HEIGHT: usize,
        const AVERAGE: bool,
    >(
        &mut self,
        kernel: &Kernel<T, KERNEL_WIDTH, KERNEL_HEIGHT, AVERAGE>,
    ) {
        let mut target: Vec<u8> = (0..(self.width * self.height)).map(|_| 0).collect();

        for x in 0..self.width {
            for y in 0..self.height {
                kernel.apply(self, x, y, &mut target)
            }
        }

        self.buffer
            .iter_mut()
            .zip(target)
            .for_each(|(target, source)| *target = source);
    }

    pub fn down_sample<'target, const SCALING: usize>(
        self,
        target: &'target mut Vec<u8>,
    ) -> Buffer<'target, GrayScale> {
        let target_height = self.height / SCALING;
        let target_width = self.width / SCALING;

        for _el in 0..(target_height * target_width) {
            target.push(0);
        }

        for x in 0..target_width {
            for y in 0..target_height {
                // No smoothing
                let idx = remap(x, target_width, self.width)
                    + remap(y, target_height, self.height) * self.width;
                target[y * target_width + x] = self.buffer[idx];
            }
        }

        Buffer::new(target, target_width, target_height)
    }

    pub fn histogram(&self) -> [u32; 256] {
        let mut ret = [0; 256];
        for el in self.buffer.iter() {
            ret[*el as usize] += 1;
        }
        ret
    }

    pub fn threshold_percentile<const PERCENTILE: usize>(&mut self) {
        assert!(PERCENTILE <= 100);
        let target_count = PERCENTILE * self.width * self.height / 100;
        let hist = self.histogram();
        let mut histogram = hist.iter().enumerate().rev();

        let mut count = 0;
        let mut threshold = 255;
        while count < target_count {
            match histogram.next() {
                Some((new_thresh, to_add)) => {
                    count += *to_add as usize;
                    threshold = new_thresh as u8;
                }
                None => return,
            }
        }

        for el in self.buffer.iter_mut() {
            *el = match *el >= threshold {
                true => 255,
                false => 0,
            }
        }
    }

    pub fn threshold_lower_percentile<const PERCENTILE: usize>(&mut self) {
        assert!(PERCENTILE <= 100);
        let target_count = PERCENTILE * self.width * self.height / 100;
        let hist = self.histogram();
        let mut histogram = hist.iter().enumerate();

        let mut count = 0;
        let mut threshold = 255;
        while count < target_count {
            match histogram.next() {
                Some((new_thresh, to_add)) => {
                    count += *to_add as usize;
                    threshold = new_thresh as u8;
                }
                None => return,
            }
        }

        for el in self.buffer.iter_mut() {
            *el = match *el >= threshold {
                true => 255,
                false => 0,
            }
        }
    }

    pub fn limit_upper(&mut self, limit: u8) {
        for el in self.buffer.iter_mut() {
            if *el > limit {
                *el = limit;
            }
        }
    }

    pub fn limit_lower(&mut self, limit: u8) {
        for el in self.buffer.iter_mut() {
            if *el < limit {
                *el = 0;
            }
        }
    }
}

impl<'buffer> From<&Buffer<'buffer, GrayScale>> for Vec<HighLight> {
    fn from(value: &Buffer<'buffer, GrayScale>) -> Self {
        let mut x = 0;
        let mut y = 0;
        let next = |y: &mut usize, x: &mut usize| {
            *x += 1;
            if *x >= value.width {
                *x = 0;
                *y += 1
            }
        };
        let mut ret = Vec::new();
        for element in value.buffer.iter() {
            if *element == 255 {
                ret.push(HighLight { x, y });
            };
            next(&mut y, &mut x);
        }
        ret
    }
}

impl<'buffer> Index<(usize, usize)> for Buffer<'buffer, GrayScale> {
    type Output = <GrayScale as ColorCode>::Marker;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        let (x, y) = index;
        let idx = x + y * self.width;
        &self.buffer[idx]
    }
}
impl<'buffer> IndexMut<(usize, usize)> for Buffer<'buffer, GrayScale> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        let (x, y) = index;
        let idx = x + y * self.width;
        self.buffer.index_mut(idx)
    }
}

impl<'mine, 'theirs, Color: ColorCode> AddAssign<&Buffer<'theirs, Color>> for Buffer<'mine, Color> {
    fn add_assign(&mut self, rhs: &Buffer<'theirs, Color>) {
        assert!(rhs.width == self.width, "Operands must have same width");
        assert!(rhs.height == self.height, "Operands must have same height");

        self.buffer
            .iter_mut()
            .zip(rhs.buffer.iter())
            .for_each(|(mine, theirs)| *mine = ((*mine as u16 + *theirs as u16) / 2) as u8);
    }
}
