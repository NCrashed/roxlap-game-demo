use std::time::{Duration, Instant};

use legion::*;

use crate::Dt;

const PERIOD: Duration = Duration::from_secs(1);

pub struct PerformanceInfo {
    pub fps: u64,
    pub frame_time_us: u64,
    update_timer: Instant,
}

impl PerformanceInfo {
    pub fn new() -> Self {
        Self {
            fps: 0,
            frame_time_us: 0,
            update_timer: Instant::now(),
        }
    }
}

#[system]
pub fn update_info(#[resource] dt: &Dt, #[resource] info: &mut PerformanceInfo) {
    if info.update_timer.elapsed() >= PERIOD {
        info.fps = dt.0.recip() as u64;
        info.frame_time_us = (dt.0 * 1_000_000.0) as u64;
        info.update_timer = Instant::now();
    }
}
