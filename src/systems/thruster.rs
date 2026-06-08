use legion::{world::SubWorld, *};

use crate::{
    components::{newton_body::NewtonBody, thruster::ThrusterBank},
    Dt,
};

/// Project `bank.command` (body-space thrust direction) onto each torque axis
/// and apply a frame-rate-independent angular_vel change.
pub fn apply_thrusters(body: &mut NewtonBody, bank: &mut ThrusterBank, dt: f64) {
    let mag = bank.command.length();
    if mag < 1e-15 {
        bank.command = glam::DVec3::ZERO;
        return;
    }
    let dir = bank.command / mag;
    let throttle = (mag / bank.max_accel()).min(1.0);

    for &torque in &bank.torques {
        let activation = dir.dot(torque).max(0.0) * throttle;
        // torque is body-space; rotate to world-space before adding to angular_vel.
        body.angular_vel +=
            body.orientation * (torque * (activation * bank.accel_per_thruster * dt));
    }
    bank.command = glam::DVec3::ZERO;
}

#[system]
#[write_component(NewtonBody)]
#[write_component(ThrusterBank)]
pub fn thruster(world: &mut SubWorld, #[resource] dt: &Dt) {
    let mut query = <(&mut NewtonBody, &mut ThrusterBank)>::query();
    for (body, bank) in query.iter_mut(world) {
        apply_thrusters(body, bank, dt.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::thruster::ThrusterBank;
    use glam::{DQuat, DVec3};

    fn make_body() -> NewtonBody {
        NewtonBody {
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
        apply_thrusters(&mut body, &mut bank, 1.0 / 60.0);
        assert_eq!(bank.command, DVec3::ZERO);
    }

    #[test]
    fn zero_command_leaves_body_unchanged() {
        let mut body = make_body();
        body.angular_vel = DVec3::new(1.0, 2.0, 3.0);
        let before = body.angular_vel;
        let mut bank = ThrusterBank::new(1.0, 0.75);
        apply_thrusters(&mut body, &mut bank, 1.0 / 60.0);
        assert_eq!(body.angular_vel, before);
    }

    #[test]
    fn angular_vel_moves_in_commanded_direction() {
        // Use dt=1.0 so the angular_vel change is large enough to check direction reliably.
        for &dir in &[
            DVec3::X,
            DVec3::Y,
            DVec3::Z,
            DVec3::NEG_X,
            DVec3::NEG_Y,
            DVec3::NEG_Z,
        ] {
            let mut body = make_body();
            let mut bank = ThrusterBank::new(1.0, 0.75);
            bank.command = dir * 3.0;
            apply_thrusters(&mut body, &mut bank, 1.0);
            let dot = body.angular_vel.dot(dir);
            assert!(
                dot > 0.5,
                "angular_vel not in commanded direction {dir:?}: dot={dot}"
            );
        }
    }

    #[test]
    fn no_nan_or_inf() {
        let mut body = make_body();
        let mut bank = ThrusterBank::new(1.0, 0.75);
        bank.command = DVec3::new(0.3, -0.1, 0.7);
        apply_thrusters(&mut body, &mut bank, 1.0 / 60.0);
        assert!(body.angular_vel.is_finite());
    }

    #[test]
    fn pos_vel_unchanged() {
        let mut body = make_body();
        body.pos = DVec3::new(1.0, 2.0, 3.0);
        body.vel = DVec3::new(4.0, 5.0, 6.0);
        let mut bank = ThrusterBank::new(1.0, 0.75);
        bank.command = DVec3::X;
        apply_thrusters(&mut body, &mut bank, 1.0 / 60.0);
        assert_eq!(body.pos, DVec3::new(1.0, 2.0, 3.0));
        assert_eq!(body.vel, DVec3::new(4.0, 5.0, 6.0));
    }
}
