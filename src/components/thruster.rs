use glam::DVec3;

/// 12 rotational nozzles (4 per body axis, pure couple — zero net force) plus
/// 6 linear nozzles (±X, ±Y, ±Z, pure force — zero net torque).
pub struct ThrusterBank {
    /// Body-space unit torque axes for the 12 rotational nozzles.
    pub torques: [DVec3; 12],
    /// Body-space unit axes for the 6 linear nozzles (±X, ±Y, ±Z).
    pub linear_axes: [DVec3; 6],
    /// Accumulated body-space angular-acceleration request for this frame.
    pub command: DVec3,
    /// Accumulated body-space linear-acceleration request for this frame.
    pub linear_command: DVec3,
    /// Thrust force per rotational nozzle (N).
    pub force_per_thruster: f64,
    /// Thrust force per linear nozzle (N).
    pub linear_force: f64,
    /// Arm length from body centre to each mount point (m).
    pub radius: f64,
}

impl ThrusterBank {
    /// Build the bank.
    ///
    /// Rotational nozzles: 4 per body axis, each pair a balanced couple
    /// (computed as `(mount × radius).cross(fire_dir).normalize()`).
    /// Linear nozzles: one per face, firing radially outward.
    pub fn new(radius: f64, force_per_thruster: f64, linear_force: f64) -> Self {
        // (mount_axis, fire_dir) pairs — each produces a unit torque along one body axis.
        let rot_nozzles: [(DVec3, DVec3); 12] = [
            // Z rotation — mount ±X, fire ±Y
            (DVec3::X, DVec3::Y),
            (DVec3::NEG_X, DVec3::NEG_Y),
            (DVec3::X, DVec3::NEG_Y),
            (DVec3::NEG_X, DVec3::Y),
            // Y rotation — mount ±Z, fire ±X
            (DVec3::Z, DVec3::X),
            (DVec3::NEG_Z, DVec3::NEG_X),
            (DVec3::Z, DVec3::NEG_X),
            (DVec3::NEG_Z, DVec3::X),
            // X rotation — mount ±Y, fire ±Z
            (DVec3::Y, DVec3::Z),
            (DVec3::NEG_Y, DVec3::NEG_Z),
            (DVec3::Y, DVec3::NEG_Z),
            (DVec3::NEG_Y, DVec3::Z),
        ];

        let torques = rot_nozzles.map(|(mount, fire)| (mount * radius).cross(fire).normalize());

        let linear_axes = [
            DVec3::X,
            DVec3::NEG_X,
            DVec3::Y,
            DVec3::NEG_Y,
            DVec3::Z,
            DVec3::NEG_Z,
        ];

        Self {
            torques,
            linear_axes,
            command: DVec3::ZERO,
            linear_command: DVec3::ZERO,
            force_per_thruster,
            linear_force,
            radius,
        }
    }

    /// Angular acceleration one rotational thruster produces (rad/s²).
    #[inline]
    pub fn accel_per_thruster(&self, mass: f64) -> f64 {
        let inertia = (2.0 / 5.0) * mass * self.radius * self.radius; // solid-sphere moment of inertia: I = 2/5 · m · r²
        self.force_per_thruster * self.radius / inertia
    }

    /// Maximum angular acceleration: 2 aligned nozzles fire per direction.
    #[inline]
    pub fn max_accel(&self, mass: f64) -> f64 {
        self.accel_per_thruster(mass) * 2.0
    }

    /// Maximum linear acceleration: 1 nozzle fires per direction.
    #[inline]
    pub fn max_linear_accel(&self, mass: f64) -> f64 {
        self.linear_force / mass
    }
}
