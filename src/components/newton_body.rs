use glam::{DQuat, DVec3};

use crate::Dt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NewtonBody {
    pub mass: f64,
    pub pos: DVec3,
    pub acc: DVec3,
    pub vel: DVec3,
    pub orientation: DQuat,
    pub angular_vel: DVec3,
}

impl NewtonBody {
    pub fn update_a(&mut self, dt: &Dt) {
        self.orientation =
            (DQuat::from_scaled_axis(self.angular_vel * dt.0) * self.orientation).normalize();
    }
}
