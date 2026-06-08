use legion::{world::SubWorld, *};

use crate::components::{newton_body::NewtonBody, thruster::ThrusterBank};

/// Project `bank.command` (body-space angular-velocity delta) onto each
/// thruster's torque axis, fire proportionally, apply world-space
/// angular_vel change, then zero the command.
pub fn apply_thrusters(body: &mut NewtonBody, bank: &mut ThrusterBank) {
    let mag = bank.command.length();
    if mag < 1e-15 {
        bank.command = glam::DVec3::ZERO;
        return;
    }
    let dir = bank.command / mag;
    let max_accel = 4.0 * bank.accel_per_thruster;
    let throttle = (mag / max_accel).min(1.0);

    for t in &bank.thrusters {
        let activation = dir.dot(t.torque).max(0.0) * throttle;
        // t.torque is body-space; transform to world-space before applying.
        body.angular_vel += body.orientation * (t.torque * (activation * bank.accel_per_thruster));
    }
    bank.command = glam::DVec3::ZERO;
}

#[system]
#[write_component(NewtonBody)]
#[write_component(ThrusterBank)]
pub fn thruster(world: &mut SubWorld) {
    let mut query = <(&mut NewtonBody, &mut ThrusterBank)>::query();
    for (body, bank) in query.iter_mut(world) {
        apply_thrusters(body, bank);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::thruster::ThrusterBank;
    use glam::{DQuat, DVec3};

    fn make_body() -> NewtonBody {
        NewtonBody {
            mass: 1.0,
            pos: DVec3::ZERO,
            vel: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_vel: DVec3::ZERO,
        }
    }

    #[test]
    fn command_zeroed_after_apply() {
        let mut body = make_body();
        let mut bank = ThrusterBank::new(1.0, 0.75);
        bank.command = DVec3::Z;
        apply_thrusters(&mut body, &mut bank);
        assert_eq!(bank.command, DVec3::ZERO);
    }

    #[test]
    fn zero_command_leaves_body_unchanged() {
        let mut body = make_body();
        body.angular_vel = DVec3::new(1.0, 2.0, 3.0);
        let before = body.angular_vel;
        let mut bank = ThrusterBank::new(1.0, 0.75);
        apply_thrusters(&mut body, &mut bank);
        assert_eq!(body.angular_vel, before);
    }

    #[test]
    fn angular_vel_moves_in_commanded_direction() {
        for &dir in &[DVec3::X, DVec3::Y, DVec3::Z, DVec3::NEG_X, DVec3::NEG_Y, DVec3::NEG_Z] {
            let mut body = make_body();
            let mut bank = ThrusterBank::new(1.0, 0.75);
            bank.command = dir * 3.0;
            apply_thrusters(&mut body, &mut bank);
            let dot = body.angular_vel.dot(dir);
            assert!(dot > 0.5, "angular_vel not in commanded direction {dir:?}: dot={dot}");
        }
    }

    #[test]
    fn no_nan_or_inf() {
        let mut body = make_body();
        let mut bank = ThrusterBank::new(1.0, 0.75);
        bank.command = DVec3::new(0.3, -0.1, 0.7);
        apply_thrusters(&mut body, &mut bank);
        assert!(body.angular_vel.is_finite());
    }

    #[test]
    fn pos_vel_unchanged() {
        let mut body = make_body();
        body.pos = DVec3::new(1.0, 2.0, 3.0);
        body.vel = DVec3::new(4.0, 5.0, 6.0);
        let mut bank = ThrusterBank::new(1.0, 0.75);
        bank.command = DVec3::X;
        apply_thrusters(&mut body, &mut bank);
        assert_eq!(body.pos, DVec3::new(1.0, 2.0, 3.0));
        assert_eq!(body.vel, DVec3::new(4.0, 5.0, 6.0));
    }
}
