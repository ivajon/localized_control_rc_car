//! Defines a gain scheduling PID controller.
use core::{fmt::Debug, marker::PhantomData};

use defmt::info;

use crate::controller::{Channel, ControlInfo};

pub trait PidParams<const FIXED_POINT: u32>: Copy {
    fn get_kp(&self) -> i32;

    fn get_ki(&self) -> i32;

    fn get_kd(&self) -> i32;

    fn get_min(&self) -> f32;

    fn get_max(&self) -> f32;
}

pub trait GainGetter<const FIXED_POINT: u32> {
    type Idx: Sized + Clone;
    type PrevIdx: Sized + Clone;
    fn get(&self, prev_idx: &mut Self::PrevIdx, idx: Self::Idx) -> impl PidParams<FIXED_POINT>;
}

#[derive(Copy, Clone)]
pub struct GainParams<const FIXED_POINT: u32> {
    pub kp: i32,
    pub ki: i32,
    pub kd: i32,
    pub max_value: f32,
    pub min_value: f32,
}

pub struct GainScheduler<
    Error: Debug,
    Interface: Channel<Error, Output = f32>,
    ParamsStore: GainGetter<FIXED_POINT>,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
    const FIXED_POINT: u32,
> {
    parameters: ParamsStore,
    prev_idx: ParamsStore::PrevIdx,
    interface: Interface,
    measurement: (f32, u64),
    reference: f32,
    prev_time: u64,
    bucketer: ParamsStore::Idx,
    previous_actuation: f32,
    previous_error: f32,
    integral: f32,
    _err: PhantomData<Error>,
}

impl<const FIXED_POINT: u32> PidParams<FIXED_POINT> for GainParams<FIXED_POINT> {
    fn get_kp(&self) -> i32 {
        self.kp
    }

    fn get_ki(&self) -> i32 {
        self.ki
    }

    fn get_kd(&self) -> i32 {
        self.kd
    }

    fn get_min(&self) -> f32 {
        self.min_value
    }

    fn get_max(&self) -> f32 {
        self.max_value
    }
}

impl<
        Error: Debug,
        Interface: Channel<Error, Output = f32>,
        ParamsStore: GainGetter<FIXED_POINT>,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
        const FIXED_POINT: u32,
    >
    GainScheduler<
        Error,
        Interface,
        ParamsStore,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    >
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(channel: Interface, params: ParamsStore) -> Self
    where
        ParamsStore::Idx: Default,
        ParamsStore::PrevIdx: Default,
    {
        GainScheduler {
            parameters: params,
            prev_time: 0,
            measurement: (0., 0),
            reference: 0.,
            prev_idx: ParamsStore::PrevIdx::default(),
            bucketer: ParamsStore::Idx::default(),
            previous_actuation: 0.,
            previous_error: 0.,
            integral: 0.,
            interface: channel,
            _err: PhantomData,
        }
    }

    pub fn follow(&mut self, reference: f32) {
        self.reference = reference;
    }

    pub fn set_bucket(&mut self, bucketer: ParamsStore::Idx) {
        self.bucketer = bucketer;
    }

    pub fn register_measurement(&mut self, measurement: (f32, u64)) {
        self.prev_time = self.measurement.1;
        self.measurement = measurement;
    }

    pub fn get_gain(&mut self) -> impl PidParams<FIXED_POINT> + '_ {
        self.parameters
            .get(&mut self.prev_idx, self.bucketer.clone())
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    pub fn actuate(&mut self) -> Result<ControlInfo<f32>, ()> {
        let output = self.compute_output()?;
        info!("Applying {:?}", output);

        self.interface.set(output.actuation).unwrap();
        self.previous_actuation = output.actuation;

        Ok(output)
    }

    pub fn compute_output(&mut self) -> Result<ControlInfo<f32>, ()> {
        let target: f32 = self.reference;

        let gain = self.get_gain();

        let kp = gain.get_kp() as f32;
        let ki = gain.get_ki() as f32;
        let kd = gain.get_kd() as f32;

        let time_scale = TIMESCALE as f32;
        let ts = (self.measurement.1 - self.prev_time) as f32;

        let threshold_min = THRESHOLD_MIN as f32;
        let threshold_max = THRESHOLD_MAX as f32;

        let fixed_point = { 10i32.pow(FIXED_POINT) as f32 };

        let actual = self.measurement.0;

        let error = target - actual;

        let p = error * kp;

        // Integral is approximated as a sum of discrete signals.
        let avg = self.previous_error as f64 + error as f64;
        self.integral += ((avg / 2.) as f32) * ts / time_scale;

        self.integral = self.integral.max(threshold_min).min(threshold_max);

        let i = self.integral * ki;

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous_error) / ts;

        self.previous_error = error;

        let output = ((p + i + d) / fixed_point)
            .max(threshold_min)
            .min(threshold_max);

        Ok(ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        })
    }
}

impl<Params: PidParams<FIXED_POINT>, const FIXED_POINT: u32, const LEN: usize>
    GainGetter<FIXED_POINT> for [Params; LEN]
{
    type Idx = f32;
    type PrevIdx = usize;

    fn get(&self, prev_idx: &mut usize, idx: f32) -> impl PidParams<FIXED_POINT> {
        let mut sugested_idx = 0;
        for (inner_idx, param_set) in self.iter().enumerate().rev() {
            if idx > param_set.get_min() as f32 * 1.05 {
                sugested_idx = inner_idx;
            }
        }

        if sugested_idx < *prev_idx {
            if idx > self[*prev_idx].get_min() as f32 * 0.95 {
                sugested_idx = *prev_idx;
            }
        }
        *prev_idx = sugested_idx;

        self[sugested_idx]
    }
}

impl<
        Params: PidParams<FIXED_POINT>,
        const FIXED_POINT: u32,
        const LEN1: usize,
        const LEN2: usize,
    > GainGetter<FIXED_POINT> for [(f32, [Option<Params>; LEN2]); LEN1]
{
    type Idx = (f32, f32);
    type PrevIdx = (usize, usize);

    fn get(&self, prev_idx: &mut (usize, usize), idx: (f32, f32)) -> impl PidParams<FIXED_POINT> {
        let mut sugested_row = 0;
        for (row, (thresh, _)) in self.iter().enumerate().rev() {
            if idx.0 > *thresh as f32 * 1.05 {
                sugested_row = row;
            }
        }

        if sugested_row < prev_idx.0 {
            if idx.0 > self[prev_idx.0].0 * 0.95 {
                sugested_row = prev_idx.0;
            }
        }

        let mut sugested_col = 0;
        let mut col_iter = self[sugested_row].1.iter().enumerate().rev();
        while let Some((col, Some(parameters))) = col_iter.next() {
            if idx.1 > parameters.get_min() as f32 * 1.05 {
                sugested_col = col;
            }
        }

        if sugested_col < prev_idx.1 {
            if let Some(value) = self[prev_idx.0].1[prev_idx.1] {
                if idx.1 > value.get_min() as f32 * 0.95 {
                    sugested_col = prev_idx.1;
                }
            }
        }
        *prev_idx = (sugested_row, sugested_col);

        // It should be safe to assume that the value exists here.
        self[sugested_row].1[sugested_col].unwrap()
    }
}

impl<
        Params: PidParams<FIXED_POINT>,
        const FIXED_POINT: u32,
        const LEN1: usize,
        const LEN2: usize,
        const LEN3: usize,
    > GainGetter<FIXED_POINT> for [(f32, [Option<(f32, [Option<Params>; LEN3])>; LEN2]); LEN1]
{
    type Idx = (f32, f32, f32);
    type PrevIdx = (usize, usize, usize);

    fn get(
        &self,
        prev_idx: &mut (usize, usize, usize),
        idx: (f32, f32, f32),
    ) -> impl PidParams<FIXED_POINT> {
        let mut sugested_row = 0;
        for (row, (thresh, _)) in self.iter().enumerate().rev() {
            if idx.0 > *thresh as f32 * 1.05 {
                sugested_row = row;
            }
        }

        if sugested_row < prev_idx.0 {
            if idx.0 > self[prev_idx.0].0 * 0.95 {
                sugested_row = prev_idx.0;
            }
        }

        let mut sugested_col = 0;
        let mut col_iter = self[sugested_row].1.iter().enumerate().rev();
        while let Some((col, Some(parameters))) = col_iter.next() {
            if idx.1 > parameters.0 as f32 * 1.05 {
                sugested_col = col;
            }
        }

        if sugested_col < prev_idx.1 {
            if let Some(value) = self[prev_idx.0].1[prev_idx.1] {
                if idx.1 > value.0 as f32 * 0.95 {
                    sugested_col = prev_idx.1;
                }
            }
        }

        let mut sugested_depth = 0;
        let col = self[sugested_row].1[sugested_col].unwrap();
        let mut col_iter = col.1.iter().enumerate().rev();

        while let Some((col, Some(parameters))) = col_iter.next() {
            if idx.1 > parameters.get_min() as f32 * 1.05 {
                sugested_depth = col;
            }
        }

        if sugested_col < prev_idx.1 {
            if let Some(value) = self[prev_idx.0].1[prev_idx.1] {
                if idx.1 > value.1[sugested_depth].unwrap().get_min() as f32 * 0.95 {
                    sugested_col = prev_idx.1;
                }
            }
        }
        *prev_idx = (sugested_row, sugested_col, sugested_depth);

        // It should be safe to assume that the value exists here.
        self[sugested_row].1[sugested_col].unwrap().1[sugested_depth].unwrap()
    }
}
