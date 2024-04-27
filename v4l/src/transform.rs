use std::{collections::HashMap, f32::consts::PI, ops::Range};

use crate::{
    buffer::{Buffer, ColorCode, GrayScale},
    graphical::{Circle, Line},
};

pub trait Transform<Color: ColorCode> {
    type Output: Sized;
    fn apply<'buffer>(&self, buffer: &Buffer<'buffer, Color>) -> Self::Output;
}

/// Ignore the outermost 10 pixels in all directions.
const PADDING: usize = 10;

pub struct HoughLines<RhoIter: Iterator<Item = usize>, ThetaIter: Iterator<Item = isize>> {
    rho: RhoIter,
    theta: ThetaIter,
    voting_threshold: u32,
    line_length: (usize, usize),
}

impl<RhoIter: Iterator<Item = usize>, ThetaIter: Iterator<Item = isize>>
    HoughLines<RhoIter, ThetaIter>
{
    pub fn new(
        rho: RhoIter,
        theta: ThetaIter,
        voting_threshold: u32,
        line_length: (usize, usize),
    ) -> Self {
        Self {
            rho,
            theta,
            voting_threshold,
            line_length,
        }
    }
}

impl<RhoIter: Iterator<Item = usize> + Clone, ThetaIter: Iterator<Item = isize> + Clone>
    Transform<GrayScale> for HoughLines<RhoIter, ThetaIter>
{
    type Output = Vec<Line>;

    fn apply<'buffer>(&self, buffer: &Buffer<'buffer, GrayScale>) -> Self::Output {
        let theta_width: usize = 100;
        let rho_min = self.rho.clone().min().unwrap();
        let rho_max = self.rho.clone().max().unwrap();

        let mut param_space: Vec<Vec<(u32, (usize, usize), (usize, usize))>> = (rho_min..rho_max)
            .clone()
            .map(|_| {
                //self.theta
                (0..theta_width)
                    .clone()
                    .map(|_| (0u32, (usize::MAX, usize::MAX), (0usize, 0usize)))
                    .collect()
            })
            .collect();

        let rho_steps = self.rho.clone().count();
        let step_size = (rho_max - rho_min) / rho_steps;
        let theta_min = self.theta.clone().min().unwrap();

        // ASSUMING THAT THE IMAGE IS 0 or 255 at this point.
        for x in PADDING..(buffer.width - PADDING) {
            for y in PADDING..(buffer.height - PADDING) {
                if buffer[(x, y)] == 255 {
                    for theta_counter in 0isize..(theta_width as isize) {
                        //self.theta.clone() {
                        let theta = PI / 2. - PI / (theta_counter as f32);
                        let rho = (theta.cos() * x as f32 + theta.sin() * y as f32) as usize;

                        if rho >= rho_max || rho < rho_min {
                            //println!("GENERATED INVALID RHO");
                            continue;
                        }

                        let (mut count, (mut x_min, mut y_min), (mut x_max, mut y_max)) =
                            param_space[(rho - rho_min) as usize][theta_counter as usize];
                        count += 1;

                        let dist = ((x.pow(2) + y.pow(2)) as f32).sqrt();
                        let max_dist =
                            (((x_max as f32).powf(2.) + (y_max as f32).powf(2.)) as f32).sqrt();
                        let min_dist = ((x_min as f32).powf(2.) + (y_min as f32).powf(2.)).sqrt();

                        if dist > max_dist {
                            x_max = x;
                            y_max = y;
                        }

                        if dist < min_dist {
                            x_min = x;
                            y_min = y;
                        }

                        param_space[(rho - rho_min) as usize][theta_counter as usize] =
                            (count, (x_min, y_min), (x_max, y_max));
                    }
                }
            }
        }

        let mut ret = Vec::new();
        for (rho, thetas) in param_space.iter().enumerate() {
            for (theta, (votes, (x_min, y_min), (x_max, y_max))) in thetas.iter().enumerate() {
                if *votes > self.voting_threshold {
                    let x_dist = *x_max as f32 - *x_min as f32;
                    let y_dist = *y_max as f32 - *y_min as f32;
                    let dist = (x_dist.powf(2.) + y_dist.powf(2.)).sqrt() as usize;
                    //println!("VOTES : {votes:?} DIST {dist:?}");
                    if dist >= self.line_length.0 && dist <= self.line_length.1 {
                        ret.push(Line::new(
                            rho + rho_min,
                            PI / 2. - PI / (theta as f32),
                            //theta as isize + theta_min,
                            (*x_min, *y_min),
                            (*x_max, *y_max),
                        ))
                    }
                }
            }
        }

        ret
    }
}

pub struct HoughCircles {
    a: Range<usize>,
    b: Range<usize>,
    radius: Range<usize>,
    voting_threshold: u32,
}

impl HoughCircles {
    pub fn new(
        a: Range<usize>,
        b: Range<usize>,
        radius: Range<usize>,
        voting_threshold: u32,
    ) -> Self {
        Self {
            a,
            b,
            radius,
            voting_threshold,
        }
    }
}

impl Transform<GrayScale> for HoughCircles {
    type Output = Vec<Circle>;

    fn apply<'buffer>(&self, buffer: &Buffer<'buffer, GrayScale>) -> Self::Output {
        let mut param_space: Vec<Vec<Vec<u32>>> = self
            .a
            .clone()
            .map(|_| {
                self.b
                    .clone()
                    .map(|_| self.radius.clone().map(|_| 0u32).collect())
                    .collect()
            })
            .collect();

        let a_min = self.a.clone().min().unwrap();
        let a_max = self.a.clone().max().unwrap();
        let b_min = self.b.clone().min().unwrap();
        let b_max = self.b.clone().max().unwrap();
        let radius_min = self.radius.clone().min().unwrap();
        let radius_max = self.radius.clone().max().unwrap();

        // ASSUMING THAT THE IMAGE IS 0 or 255 at this point.
        for x in PADDING..(buffer.width - PADDING) {
            for y in PADDING..(buffer.height - PADDING) {
                if buffer[(x, y)] == 255 {
                    for radius in self.radius.clone() {
                        for angle in 0..360 {
                            let a = (x as f32 - (radius as f32 * (angle as f32).to_radians().cos()))
                                as usize;
                            let b = (y as f32 - (radius as f32 * (angle as f32).to_radians().sin()))
                                as usize;
                            if a >= a_min && a < a_max && b >= b_min && b < b_max {
                                param_space[a - a_min][b - b_min][radius - radius_min] += 1;
                            }
                        }
                    }
                }
            }
        }

        let mut ret = Vec::new();
        let mut taken: HashMap<(usize, usize), ()> = HashMap::new();
        for (a, thetas) in param_space.iter().enumerate() {
            for (b, radii) in thetas.iter().enumerate() {
                for (radius, votes) in radii.iter().enumerate() {
                    if *votes > self.voting_threshold
                        && taken.get(&(a + a_min, b + b_min)).is_none()
                    {
                        taken.insert((a + a_min, b + b_min), ());
                        ret.push(Circle::new((a + a_min, b + b_min), radius + radius_min))
                    }
                }
            }
        }

        ret
    }
}
