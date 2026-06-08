use glam::DVec3;

/// Six sphere-surface mount points (±X, ±Y, ±Z body axes), each with four
/// tangential nozzles.  Any subset producing a pure couple (zero net force)
/// can be selected by projecting the command onto each nozzle's torque axis.
pub struct ThrusterBank {
    /// Precomputed body-space unit torque axis for each nozzle.
    pub torques: Vec<DVec3>,
    /// Accumulated body-space angular-acceleration request for this frame.
    /// Written by input / autopilot systems; consumed and zeroed by the thruster system.
    pub command: DVec3,
    /// Thrust force each nozzle produces (N).
    pub force_per_thruster: f64,
    /// Arm length from body centre to each mount point (m).
    pub radius: f64,
}

impl ThrusterBank {
    pub fn new(radius: f64, force_per_thruster: f64) -> Self {
        let groups: [(DVec3, [DVec3; 4]); 6] = [
            (DVec3::X, [DVec3::Y, DVec3::NEG_Y, DVec3::Z, DVec3::NEG_Z]),
            (
                DVec3::NEG_X,
                [DVec3::Y, DVec3::NEG_Y, DVec3::Z, DVec3::NEG_Z],
            ),
            (DVec3::Y, [DVec3::X, DVec3::NEG_X, DVec3::Z, DVec3::NEG_Z]),
            (
                DVec3::NEG_Y,
                [DVec3::X, DVec3::NEG_X, DVec3::Z, DVec3::NEG_Z],
            ),
            (DVec3::Z, [DVec3::X, DVec3::NEG_X, DVec3::Y, DVec3::NEG_Y]),
            (
                DVec3::NEG_Z,
                [DVec3::X, DVec3::NEG_X, DVec3::Y, DVec3::NEG_Y],
            ),
        ];
        let mut torques = Vec::with_capacity(24);
        for (axis, tans) in &groups {
            let offset = *axis * radius;
            for &dir in tans {
                torques.push(offset.cross(dir).normalize());
            }
        }
        Self {
            torques,
            command: DVec3::ZERO,
            force_per_thruster,
            radius,
        }
    }

    /// Angular acceleration one fully-activated thruster produces (rad/s²).
    /// Uses solid-sphere inertia: I = (2/5) · mass · radius².
    #[inline]
    pub fn accel_per_thruster(&self, mass: f64) -> f64 {
        let inertia = (2.0 / 5.0) * mass * self.radius * self.radius;
        self.force_per_thruster * self.radius / inertia
    }

    /// Maximum angular acceleration when four aligned thrusters fire together (rad/s²).
    #[inline]
    pub fn max_accel(&self, mass: f64) -> f64 {
        self.accel_per_thruster(mass) * 4.0
    }
}
