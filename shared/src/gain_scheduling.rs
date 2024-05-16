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
    ParamsStore: PidParams<FIXED_POINT>,
    const REGIONS: usize,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
    const FIXED_POINT: u32,
> {
    index_map: [(f32, usize); REGIONS],
    parameters: [ParamsStore; REGIONS],
    prev_idx: usize,
    interface: Interface,
    measurement: (f32, u64),
    reference: f32,
    prev_time: u64,
    bucketer: f32,
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
        ParamsStore: PidParams<FIXED_POINT>,
        const REGIONS: usize,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
        const FIXED_POINT: u32,
    >
    GainScheduler<
        Error,
        Interface,
        ParamsStore,
        REGIONS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    >
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(
        channel: Interface,
        params: [ParamsStore; REGIONS],
    ) -> GainScheduler<
        Error,
        Interface,
        ParamsStore,
        REGIONS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    > {
        let mut index_map = [(0., 0); REGIONS];
        let mut prev_thresh = f32::MIN;
        for (idx, el) in params.iter().enumerate() {
            if el.get_min() <= prev_thresh {
                panic!()
            }
            prev_thresh = el.get_min();
            index_map[idx] = (prev_thresh, idx);
        }

        GainScheduler {
            index_map,
            parameters: params,
            prev_time: 0,
            measurement: (0., 0),
            reference: 0.,
            prev_idx: 0,
            bucketer: 0.,
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

    pub fn set_bucket(&mut self, bucketer: f32) {
        self.bucketer = bucketer;
    }

    pub fn register_measurement(&mut self, measurement: (f32, u64)) {
        self.prev_time = self.measurement.1;
        self.measurement = measurement;
    }

    pub fn get_gain(&self) -> ParamsStore {
        let mut min_idx = 0;

        for (idx, (lower_bound, _target_idx)) in self.index_map.iter().enumerate() {
            if self.bucketer as f32 > *lower_bound * 1.05 {
                info!("IDX : {:?}",idx);
                min_idx = idx;
            } else {
                break;
            }
        }
        if min_idx < self.prev_idx {
            if self.bucketer >= self.index_map[self.prev_idx].0 * 0.95 {
                min_idx = self.prev_idx;
            }
        }

        info!("using idx : {:?}",min_idx);

        self.parameters[self.index_map[min_idx].1]
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    pub fn actuate(&mut self) -> Result<ControlInfo<f32>, ()> {
        let output = self.compute_output()?;
        info!("Applying {:?}",output);

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

        let output = -((p + i + d) / fixed_point)
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
