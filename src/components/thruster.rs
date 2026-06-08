use glam::DVec3;

#[allow(dead_code)]
pub struct Thruster {
    pub offset: DVec3,
    pub force_dir: DVec3,
    /// Precomputed body-space unit torque axis: (offset × force_dir).normalize()
    pub torque: DVec3,
}

impl Thruster {
    fn new(offset: DVec3, force_dir: DVec3) -> Self {
        let raw = offset.cross(force_dir);
        let torque = if raw.length_squared() > 1e-12 {
            raw.normalize()
        } else {
            DVec3::ZERO
        };
        Self { offset, force_dir, torque }
    }
}

/// Six sphere-surface mount points (±X, ±Y, ±Z body axes), each with four
/// tangential nozzles.  Any subset producing a pure couple (zero net force)
/// can be selected by projecting the command onto each nozzle's torque axis.
pub struct ThrusterBank {
    pub thrusters: Vec<Thruster>,
    /// Accumulated body-space angular-velocity delta request (rad/s) for this
    /// frame.  Written by input / autopilot systems; consumed and zeroed by
    /// the thruster system.
    pub command: DVec3,
    /// Max |angular_vel| change a single fully-activated thruster contributes
    /// per step.  Four thrusters perfectly aligned with the command axis give
    /// 4 × accel_per_thruster of achievable change.
    pub accel_per_thruster: f64,
}

impl ThrusterBank {
    pub fn new(radius: f64, accel_per_thruster: f64) -> Self {
        let r = radius;
        let groups: [(DVec3, [DVec3; 4]); 6] = [
            (DVec3::X,     [DVec3::Y,     DVec3::NEG_Y, DVec3::Z,     DVec3::NEG_Z]),
            (DVec3::NEG_X, [DVec3::Y,     DVec3::NEG_Y, DVec3::Z,     DVec3::NEG_Z]),
            (DVec3::Y,     [DVec3::X,     DVec3::NEG_X, DVec3::Z,     DVec3::NEG_Z]),
            (DVec3::NEG_Y, [DVec3::X,     DVec3::NEG_X, DVec3::Z,     DVec3::NEG_Z]),
            (DVec3::Z,     [DVec3::X,     DVec3::NEG_X, DVec3::Y,     DVec3::NEG_Y]),
            (DVec3::NEG_Z, [DVec3::X,     DVec3::NEG_X, DVec3::Y,     DVec3::NEG_Y]),
        ];
        let mut thrusters = Vec::with_capacity(24);
        for (axis, tans) in &groups {
            let offset = *axis * r;
            for &dir in tans {
                thrusters.push(Thruster::new(offset, dir));
            }
        }
        Self { thrusters, command: DVec3::ZERO, accel_per_thruster }
    }
}
