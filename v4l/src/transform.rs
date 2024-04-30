use std::{
    collections::{vec_deque, HashMap, HashSet},
    f32::consts::PI,
    ops::Range,
};

use crate::{
    buffer::Buffer,
    color_code::{ColorCode, GrayScale},
    graphical::{Circle, Line},
    HighLight,
};

pub trait Transform<Color: ColorCode> {
    type Output: Sized;
    fn apply<'buffer>(&self, buffer: &Vec<HighLight>) -> Self::Output;
}

/// Ignore the outermost 10 pixels in all directions.
const PADDING: usize = 10;

const VOTE_LIMIT: usize = 500;

pub struct HoughLines<RhoIter: Iterator<Item = isize>, ThetaIter: Iterator<Item = isize>> {
    rho: RhoIter,
    theta: ThetaIter,
    voting_threshold: u32,
    line_length: (usize, usize),
}

impl<RhoIter: Iterator<Item = isize>, ThetaIter: Iterator<Item = isize>>
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

impl<RhoIter: Iterator<Item = isize> + Clone, ThetaIter: Iterator<Item = isize> + Clone>
    Transform<GrayScale> for HoughLines<RhoIter, ThetaIter>
{
    type Output = Vec<Line>;

    fn apply<'buffer>(&self, highlights: &Vec<HighLight>) -> Self::Output {
        let rho_min = self.rho.clone().min().unwrap();
        let rho_max = self.rho.clone().max().unwrap();

        let cs: Vec<(f32, f32, isize)> = self
            .theta
            .clone()
            .map(|angle| {
                (
                    (angle as f32).to_radians().cos(),
                    (angle as f32).to_radians().sin(),
                    angle,
                )
            })
            .collect();

        let mut param_space =
            HashMap::<isize, HashMap<isize, (u32, (usize, usize), (usize, usize))>>::new();

        // ASSUMING THAT THE IMAGE IS 0 or 255 at this point.
        for HighLight { x, y } in highlights {
            let mut checked = HashSet::<(i32, i32)>::new();
            for (cos, sin, theta) in cs.iter() {
                let (cos_i, sin_i) = (*cos as i32, *sin as i32);

                if !checked.insert((cos_i, sin_i)) {
                    //continue;
                }

                //self.theta.clone() {
                let rho = (cos * *x as f32 + sin * *y as f32) as isize;

                //println!("THETA : {theta} -> {rho} for {x},{y}");

                if rho >= rho_max || rho < rho_min {
                    //println!("GENERATED INVALID RHO");
                    continue;
                }

                let thetas = match param_space.get_mut(&rho) {
                    Some(value) => value,
                    None => {
                        param_space.insert(rho, HashMap::new());
                        unsafe { param_space.get_mut(&rho).unwrap_unchecked() }
                    }
                };

                let (count, (x_min, y_min), (x_max, y_max)) = match thetas.get_mut(&theta) {
                    Some(value) => value,
                    None => {
                        thetas.insert(*theta, (0, (usize::MAX, usize::MAX), (0, 0)));
                        unsafe { thetas.get_mut(&theta).unwrap_unchecked() }
                    }
                };

                *count += 1;

                let dist = ((x.pow(2) + y.pow(2)) as f32).sqrt();
                let max_dist =
                    (((*x_max as f32).powf(2.) + (*y_max as f32).powf(2.)) as f32).sqrt();
                let min_dist = ((*x_min as f32).powf(2.) + (*y_min as f32).powf(2.)).sqrt();

                let length_if_new_min = ((*x_max as f32 - *x as f32).powf(2.)
                    + (*y_max as f32 - *y as f32).powf(2.))
                .sqrt();
                let length_if_new_max = ((*x as f32 - *x_min as f32).powf(2.)
                    + (*y as f32 - *y_min as f32).powf(2.))
                .sqrt();

                if dist > max_dist && length_if_new_max < self.line_length.1 as f32 {
                    *x_max = *x;
                    *y_max = *y;
                }
                if (*x_min == *y_min && *x_min == usize::MAX)
                    || dist < min_dist && length_if_new_min < self.line_length.1 as f32
                {
                    *x_min = *x;
                    *y_min = *y;
                }
            }
        }

        let mut ret = Vec::new();

        let mut taken: HashMap<usize, HashMap<usize, Vec<(usize, usize)>>> = HashMap::new();
        for (rho, thetas) in param_space.iter() {
            for (theta, (votes, (x_min, y_min), (x_max, y_max))) in thetas.iter() {
                if *votes > self.voting_threshold {
                    let ys = match taken.get_mut(x_min) {
                        Some(ys) => ys,
                        None => {
                            taken.insert(*x_min, HashMap::new());
                            unsafe { taken.get_mut(x_min).unwrap_unchecked() }
                        }
                    };
                    let end_points = match ys.get_mut(y_min) {
                        Some(endpoints) => endpoints,
                        None => {
                            ys.insert(*y_min, Vec::new());
                            unsafe { ys.get_mut(y_min).unwrap_unchecked() }
                        }
                    };

                    if end_points.contains(&(*x_max, *y_max)) {
                        continue;
                    } else {
                        end_points.push((*x_max, *y_max));
                    }
                    let x_dist = *x_max as f32 - *x_min as f32;
                    let y_dist = *y_max as f32 - *y_min as f32;
                    let dist = (x_dist.powf(2.) + y_dist.powf(2.)).sqrt() as usize;

                    if dist >= self.line_length.0 && dist <= self.line_length.1 {
                        ret.push(Line::new(
                            rho + rho_min,
                            //PI / 2. - PI / (*theta as f32),
                            *theta as f32,
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
    a: Range<isize>,
    b: Range<isize>,
    radius: Range<usize>,
    voting_threshold: u32,
}

impl HoughCircles {
    pub fn new(
        a: Range<isize>,
        b: Range<isize>,
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

    fn apply<'buffer>(&self, highlights: &Vec<HighLight>) -> Self::Output {
        let mut param_space: HashMap<isize, HashMap<isize, HashMap<i32, u32>>> = HashMap::new();

        let a_min = self.a.clone().min().unwrap();
        let b_min = self.b.clone().min().unwrap();
        let a_max = self.a.clone().max().unwrap();
        let b_max = self.b.clone().max().unwrap();

        let cs: Vec<(f32, f32)> = (0..360)
            .step_by(2)
            .map(|angle| {
                (
                    (angle as f32).to_radians().cos(),
                    (angle as f32).to_radians().sin(),
                )
            })
            .collect();

        // ASSUMING THAT THE IMAGE IS 0 or 255 at this point.
        for HighLight { x, y } in highlights {
            let mut taken = HashSet::new();
            for radius in self.radius.clone() {
                for (cosine, sine) in cs.iter() {
                    let a = (*x as f32 - (radius as f32 * cosine)) as isize;
                    if a >= a_max || a <= a_min {
                        continue;
                    }
                    let b = (*y as f32 - (radius as f32 * sine)) as isize;
                    if b >= b_max || b <= b_min {
                        continue;
                    }

                    if !taken.insert((a, b)) {
                        //println!("({a},{b}) taken");
                        continue;
                    }
                    let bs = match param_space.get_mut(&a) {
                        Some(bs) => bs,
                        None => {
                            param_space.insert(a, HashMap::new());
                            unsafe { param_space.get_mut(&a).unwrap_unchecked() }
                        }
                    };

                    let radii = match bs.get_mut(&b) {
                        Some(radii) => radii,
                        None => {
                            bs.insert(b, HashMap::new());
                            unsafe { bs.get_mut(&b).unwrap_unchecked() }
                        }
                    };

                    let vote = match radii.get_mut(&(radius as i32)) {
                        Some(vote) => vote,
                        None => {
                            radii.insert(radius as i32, 0);
                            unsafe { radii.get_mut(&(radius as i32)).unwrap_unchecked() }
                        }
                    };

                    *vote += 1;
                }
                /*
                let a = (*x as f32 - (radius as f32)) as isize;
                let b = (*y as f32 - (radius as f32)) as isize;

                if !taken.insert((a, b)) {
                    println!("({a},{b}) taken");
                    continue;
                }
                let bs = match param_space.get_mut(&a) {
                    Some(bs) => bs,
                    None => {
                        param_space.insert(a, HashMap::new());
                        unsafe { param_space.get_mut(&a).unwrap_unchecked() }
                    }
                };

                let radii = match bs.get_mut(&b) {
                    Some(radii) => radii,
                    None => {
                        bs.insert(b, HashMap::new());
                        unsafe { bs.get_mut(&b).unwrap_unchecked() }
                    }
                };

                let vote = match radii.get_mut(&(radius as i32)) {
                    Some(vote) => vote,
                    None => {
                        radii.insert(radius as i32, 0);
                        unsafe { radii.get_mut(&(radius as i32)).unwrap_unchecked() }
                    }
                };

                *vote += 1;

                let a = (*x as f32 + (radius as f32)) as isize;
                let b = (*y as f32 + (radius as f32)) as isize;

                if !taken.insert((a, b)) {
                    println!("({a},{b}) taken");
                    continue;
                }

                let bs = match param_space.get_mut(&a) {
                    Some(bs) => bs,
                    None => {
                        param_space.insert(a, HashMap::new());
                        unsafe { param_space.get_mut(&a).unwrap_unchecked() }
                    }
                };

                let radii = match bs.get_mut(&b) {
                    Some(radii) => radii,
                    None => {
                        bs.insert(b, HashMap::new());
                        unsafe { bs.get_mut(&b).unwrap_unchecked() }
                    }
                };

                let vote = match radii.get_mut(&(radius as i32)) {
                    Some(vote) => vote,
                    None => {
                        radii.insert(radius as i32, 0);
                        unsafe { radii.get_mut(&(radius as i32)).unwrap_unchecked() }
                    }
                };

                *vote += 1;
                */
            }
        }

        let mut top: Option<(u32, (isize, isize, i32))> = None;
        let mut top3 = vec![0, 0, 0];

        let mut taken: HashMap<(isize, isize), (i32, u32)> = HashMap::new();
        for (a, bs) in param_space.iter() {
            for (b, radii) in bs.iter() {
                for (radius, votes) in radii.iter() {
                    //println!("Circle with center ({a},{b}) and radius {radius} has {votes}
                    // votes");

                    for (idx, v) in top3.clone().iter().enumerate() {
                        if *v <= *votes {
                            if idx < 2 {
                                top3[idx + 1] = *v;
                            }
                            top3[idx] = *votes;
                            break;
                        }
                    }
                    if *votes > self.voting_threshold
                        && taken.get(&(a + a_min, b + b_min)).is_none()
                    {
                        //println!("ACCEPTED CIRCLE with center ({a},{b}) and radius {radius} has
                        // {votes} votes");
                        match taken.get(&(a + a_min, b + b_min)) {
                            Some((_old_radius, old_votes)) => {
                                if old_votes < votes {
                                    taken.insert((a + a_min, b + b_min), (*radius, *votes));
                                }
                            }
                            None => {
                                taken.insert((a + a_min, b + b_min), (*radius, *votes));
                            }
                        };

                        if let Some(top_inner) = top {
                            if *votes > top_inner.0 {
                                top = Some((*votes, (*a, *b, *radius)));
                            }
                        } else {
                            top = Some((*votes, (*a, *b, *radius)));
                        }
                    }
                }
            }
        }
        println!("TOP 3 VOTES {top3:?}");
        /*if let Some((votes, (a, b, r))) = top {
            vec![Circle::new(
                ((a as isize + a_min), (b as isize + b_min)),
                (r + radius_min as i32) as isize,
            )]
        } else {
            vec![]
        }*/
        taken
            .iter()
            .map(|((a, b), (radius, votes))| Circle::new((*a, *b), *radius as isize))
            .collect()
    }
}
