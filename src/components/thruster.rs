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
    /// Max angular-velocity change a single fully-activated thruster contributes per step.
    pub accel_per_thruster: f64,
}

impl ThrusterBank {
    pub fn new(radius: f64, accel_per_thruster: f64) -> Self {
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
            accel_per_thruster,
        }
    }

    /// Maximum angular acceleration achievable when four aligned thrusters fire together.
    #[inline]
    pub fn max_accel(&self) -> f64 {
        self.accel_per_thruster * 4.0
    }
}
