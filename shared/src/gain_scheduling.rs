//! Defines a gain scheduling PID controller.
use core::{fmt::Debug, marker::PhantomData};

use defmt::{info, warn, Format};

use crate::controller::{Channel, ControlInfo};

pub trait PidParams<const FIXED_POINT: u32>: Copy {
    /// Gets the proportional gain for the control law.
    fn get_kp(&self) -> i32;
    /// Gets the integral gain for the control law.
    fn get_ki(&self) -> i32;
    /// Gets the derivative gain for the conrol law.
    fn get_kd(&self) -> i32;
    /// Gets the minimum value where the control law is valid.
    fn get_min(&self) -> f32;
    /// Gets the maximum value where the control law is valid.
    fn get_max(&self) -> f32;
}

/// A generic way to access the gain values used in [gain
/// scheduling](https://se.mathworks.com/help/control/ug/gain-scheduled-control-systems.html)
pub trait GainGetter<const FIXED_POINT: u32> {
    /// The type used to index the getter.
    type Idx: Sized + Clone + defmt::Format;
    /// The internally used index.
    type PrevIdx: Sized + Clone;
    /// Retrieves the current set of [`PidParams`].
    fn get(&self, prev_idx: &mut Self::PrevIdx, idx: Self::Idx) -> impl PidParams<FIXED_POINT>;
}

#[derive(Copy, Clone)]
/// Simple pure gain parameters.
pub struct GainParams<const FIXED_POINT: u32> {
    /// Proportional gain.
    pub kp: i32,
    /// Integral gain.
    pub ki: i32,
    /// Derivative gain.
    pub kd: i32,
    /// Maximum value where this configuration is valid.
    pub max_value: f32,
    /// Minimum value where this configuration is valid.
    pub min_value: f32,
}

/// A PID controller that supports basic gain scheduling.
///
/// This can be dynamically created or statically, typically though it will be
/// used with a compile time defined set of parameters.
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
where
    ParamsStore::PrevIdx: Format,
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

    /// Sets a new reference to follow.
    pub fn follow(&mut self, reference: f32) {
        self.reference = reference;
    }

    /// Sets the bucket i.e. which set of [`PidParams`] to use.
    pub fn set_bucket(&mut self, bucketer: ParamsStore::Idx) {
        self.bucketer = bucketer;
    }

    /// Registers a new measurement, this is a tuple of (value, time in
    /// TIMESCALE)
    pub fn register_measurement(&mut self, measurement: (f32, u64)) {
        self.prev_time = self.measurement.1;
        self.measurement = measurement;
    }

    /// Returns the current [`PidParams`].
    pub fn get_gain(&mut self) -> impl PidParams<FIXED_POINT> + '_ {
        let params = self
            .parameters
            .get(&mut self.prev_idx, self.bucketer.clone());
        params
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    pub fn actuate(&mut self) -> Result<ControlInfo<f32>, Error> {
        let output = self.compute_output();
        info!("USING IDX {:?}", self.prev_idx);

        self.interface.set(output.actuation)?;
        self.previous_actuation = output.actuation;

        Ok(output)
    }

    pub fn compute_output(&mut self) -> ControlInfo<f32> {
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

        ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        }
    }
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
impl<Params: PidParams<FIXED_POINT>, const FIXED_POINT: u32, const LEN: usize>
    GainGetter<FIXED_POINT> for [Params; LEN]
{
    type Idx = f32;
    type PrevIdx = usize;

    fn get(&self, prev_idx: &mut usize, idx: f32) -> impl PidParams<FIXED_POINT> {
        let mut suggested_idx = 0;
        for (inner_idx, param_set) in self.iter().enumerate().rev() {
            if idx > param_set.get_min() * 1.05 {
                suggested_idx = inner_idx;
                break;
            }
        }

        if suggested_idx < *prev_idx && idx > self[*prev_idx].get_min() * 0.95 {
            suggested_idx = *prev_idx;
        }
        *prev_idx = suggested_idx;

        self[suggested_idx]
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
        let mut suggested_row = 0;
        for (row, (thresh, _)) in self.iter().enumerate().rev() {
            if idx.0 > *thresh * 1.05 {
                suggested_row = row;
                break;
            }
        }

        if suggested_row < prev_idx.0 && idx.0 > self[prev_idx.0].0 * 0.95 {
            suggested_row = prev_idx.0;
        }

        let mut suggested_col = 0;
        let mut col_iter = self[suggested_row].1.iter().enumerate().rev();
        while let Some((col, Some(parameters))) = col_iter.next() {
            if idx.1 > parameters.get_min() * 1.05 {
                suggested_col = col;
                break;
            }
        }

        if suggested_col < prev_idx.1 {
            if let Some(value) = self[prev_idx.0].1[prev_idx.1] {
                if idx.1 > value.get_min() * 0.95 {
                    suggested_col = prev_idx.1;
                }
            }
        }
        *prev_idx = (suggested_row, suggested_col);

        // It should be safe to assume that the value exists here.
        self[suggested_row].1[suggested_col].unwrap()
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
        warn!("USING GETTER IDX : {:?}", idx);
        let mut suggested_row = 0;
        for (row, (thresh, _)) in self.iter().enumerate().rev() {
            info!("Checking thresh {:?}", thresh);
            if idx.0 > *thresh * 1.05 {
                suggested_row = row;
                break;
            }
        }

        if suggested_row < prev_idx.0 && idx.0 > self[prev_idx.0].0 * 0.95 {
            suggested_row = prev_idx.0;
        }

        let mut suggested_col = 0;
        let mut col_iter = self[suggested_row].1.iter().enumerate().rev();
        while let Some(col) = col_iter.next() {
            info!("Checking cols");
            if let (col, Some(parameters)) = col {
                info!("Checking col {:?}", col);
                if idx.1 > parameters.0 * 1.05 {
                    suggested_col = col;
                    break;
                }
            }
        }

        if suggested_col < prev_idx.1 {
            if let Some(value) = self[suggested_row].1[prev_idx.1] {
                if idx.1 > value.0 * 0.95 && self[suggested_row].1[prev_idx.1].is_some() {
                    suggested_col = prev_idx.1;
                }
            }
        }

        let mut suggested_depth = 0;
        info!("Sugested col : {:?}", suggested_col);
        let col = self[suggested_row].1[suggested_col].unwrap();
        let mut col_iter = col.1.iter().enumerate().rev();

        while let Some((col, Some(parameters))) = col_iter.next() {
            if idx.2 > parameters.get_min() * 1.05 {
                suggested_depth = col;
                break;
            }
        }

        if suggested_col < prev_idx.1 {
            if let Some(value) = self[suggested_row].1[suggested_col] {
                if self[suggested_row].1[suggested_col].unwrap().1[prev_idx.2].is_some()
                    && idx.2 > value.1[prev_idx.2].unwrap().get_min() * 0.95
                {
                    suggested_col = prev_idx.2;
                }
            }
        }
        *prev_idx = (suggested_row, suggested_col, suggested_depth);

        // It should be safe to assume that the value exists here.
        self[suggested_row].1[suggested_col].unwrap().1[suggested_depth].unwrap()
    }
}
