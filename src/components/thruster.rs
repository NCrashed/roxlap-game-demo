use glam::DVec3;

pub struct ThrusterBank {
    pub command: DVec3,
    pub linear_command: DVec3,
    pub max_rot_accel: f64,
    pub max_lin_accel: f64,
}

impl ThrusterBank {
    /// Build the bank, baking in sphere inertia `I = 2/5 · m · r²`.
    /// `max_rot_accel = 5·F / (m·r)` (two opposing nozzles per axis).
    /// `max_lin_accel = linear_force / mass`.
    pub fn new(mass: f64, radius: f64, force_per_thruster: f64, linear_force: f64) -> Self {
        Self {
            command: DVec3::ZERO,
            linear_command: DVec3::ZERO,
            max_rot_accel: (5.0 * force_per_thruster) / (mass * radius),
            max_lin_accel: linear_force / mass,
        }
    }
}
