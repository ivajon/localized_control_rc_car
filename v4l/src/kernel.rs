use std::ops::{AddAssign, Div, Mul};

use crate::{buffer::Buffer, color_code::ColorCode, sealed::Conversions};

pub static SOBEL: Kernel<i32, 3, 3, true> = Kernel {
    data: [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]],
};
pub static PREWIT: Kernel<i32, 3, 3, true> = Kernel {
    data: [[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]],
};

pub static LAPLACIAN: Kernel<i32, 3, 3, false> = Kernel {
    data: [[0, 1, 0], [1, -4, 1], [0, 1, 0]],
};

pub static GAUSSIAN: Kernel<f32, 3, 3, false> = Kernel {
    data: [
        [1. / 16., 1. / 8., 1. / 16.],
        [1. / 8., 1. / 4., 1. / 8.],
        [1. / 16., 1. / 8., 1. / 16.],
    ],
};

pub static AVERAGING: Kernel<i32, 5, 5, true> = Kernel {
    data: [
        [1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1],
    ],
};

pub const fn averaging<const SIZE: usize>() -> Kernel<i32, SIZE, SIZE, true> {
    Kernel {
        data: [[1; SIZE]; SIZE],
    }
}

#[derive(Clone)]
pub struct Kernel<
    T: Sized
        + Mul<T, Output = T>
        + Conversions<i32>
        + AddAssign<T>
        + Default
        + Copy
        + Div<T, Output = T>,
    const WIDTH: usize,
    const HEIGHT: usize,
    const AVERAGE: bool,
> {
    pub data: [[T; WIDTH]; HEIGHT],
}

impl<
        T: Sized
            + Mul<T, Output = T>
            + Conversions<i32>
            + AddAssign<T>
            + Default
            + Copy
            + Div<T, Output = T>,
        const WIDTH: usize,
        const HEIGHT: usize,
        const AVERAGE: bool,
    > Kernel<T, WIDTH, HEIGHT, AVERAGE>
{
    const X_CENTER: usize = { WIDTH / 2 };
    const Y_CENTER: usize = { HEIGHT / 2 };

    pub fn apply<Color: ColorCode<Marker = u8>>(
        &self,
        buffer: &mut Buffer<Color>,
        center_x: usize,
        center_y: usize,
        target: &mut [u8],
    ) {
        // LETS BE STUPID FOR NOW and discard single pixel boundaries.

        let img_width = buffer.width;
        let img_height = buffer.height;

        // This does nothing to the edges, not optimal but we can move this in to the
        // loop if it causes errors.
        if center_x + Self::X_CENTER >= img_width || Self::X_CENTER >= center_x {
            return;
        }

        if center_y + Self::Y_CENTER >= img_height || Self::Y_CENTER >= center_y {
            return;
        }

        let corner_x = center_x - Self::X_CENTER;
        let corner_y = center_y - Self::Y_CENTER;

        let mut sum: T = T::default();
        for x in 0..WIDTH {
            for y in 0..HEIGHT {
                let img_x = corner_x + x;
                let img_y = corner_y + y;
                //let idx = img_x + img_y * img_width;

                sum += T::convert_from(buffer[(img_x, img_y)] as i32) * self.data[y][x];
            }
        }
        let idx = center_x + center_y * img_width;

        if AVERAGE {
            sum = sum / T::convert_from({ WIDTH * HEIGHT } as i32);
        }

        let sum: i32 = sum.convert_to();
        target[idx] = match sum >= 0 {
            true => sum as u8,
            false => 0,
        };
    }
}

impl<const WIDTH: usize, const HEIGHT: usize, const AVERAGE: bool>
    Kernel<i32, WIDTH, HEIGHT, AVERAGE>
{
    pub const fn transpose(self) -> Kernel<i32, HEIGHT, WIDTH, AVERAGE> {
        let mut ret = [[0; HEIGHT]; WIDTH];
        let mut h = 0;
        while h < { HEIGHT } {
            let mut w = 0;
            while w < { WIDTH } {
                ret[h][w] = self.data[w][h];
                w += 1;
            }
            h += 1;
        }
        Kernel { data: ret }
    }
}

impl<const WIDTH: usize, const HEIGHT: usize, const AVERAGE: bool>
    Kernel<f32, WIDTH, HEIGHT, AVERAGE>
{
    pub const fn transpose(self) -> Kernel<f32, HEIGHT, WIDTH, AVERAGE> {
        let mut ret = [[0.; HEIGHT]; WIDTH];
        let mut h = 0;
        while h < { HEIGHT } {
            let mut w = 0;
            while w < { WIDTH } {
                ret[h][w] = self.data[w][h];
                w += 1;
            }
            h += 1;
        }
        Kernel { data: ret }
    }
}
