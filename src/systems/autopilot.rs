use glam::{DQuat, DVec3};
use legion::{world::SubWorld, *};

use crate::{
    components::{
        camera::CameraComponent, miner::Miner, newton_body::NewtonBody, thruster::ThrusterBank,
    },
    Dt, MouseDelta, ScreenState,
};

/// Proportional angular gain (rad/s per radian of error).
const STEER_GAIN: f64 = 4.0;
/// Lerp fraction of (desired − current) angular_vel applied per 60 Hz tick.
const STEER_SMOOTH: f64 = 0.18;
/// Maximum rotation speed the autopilot can command (rad/s).
const MAX_ANGULAR_SPEED: f64 = 3.0;
/// Mouse sensitivity when rotating the target direction (rad/pixel).
const MOUSE_SENSITIVITY: f64 = 0.003;

/// Compute the body-space angular-velocity delta needed to steer toward
/// `target_dir` and accumulate it into `bank.command`.
/// Does not write `body.angular_vel` — that is the thruster system's job.
pub fn apply_autopilot(body: &NewtonBody, bank: &mut ThrusterBank, target_dir: DVec3, dt: f64) {
    let ship_fwd = body.orientation * DVec3::NEG_Z;

    let steer_cross = ship_fwd.cross(target_dir);
    let steer_sin = steer_cross.length();
    let steer_cos = ship_fwd.dot(target_dir);
    let steer_angle = steer_sin.atan2(steer_cos);

    let smooth = (STEER_SMOOTH * dt * 60.0).min(1.0);

    if steer_angle.abs() > 0.001 {
        let steer_axis_world = if steer_sin > 1e-9 {
            steer_cross / steer_sin
        } else {
            let alt = if ship_fwd.x.abs() < 0.9 {
                DVec3::X
            } else {
                DVec3::Y
            };
            ship_fwd.cross(alt).normalize()
        };
        let desired_speed = (steer_angle * STEER_GAIN).min(MAX_ANGULAR_SPEED);
        let desired_world = steer_axis_world * desired_speed;
        let delta_world = (desired_world - body.angular_vel) * smooth;
        bank.command += body.orientation.inverse() * delta_world;
    } else {
        // Damp residual spin toward zero.
        let delta_world = -body.angular_vel * smooth;
        bank.command += body.orientation.inverse() * delta_world;
    }
}

#[system]
#[read_component(Miner)]
#[read_component(CameraComponent)]
#[read_component(NewtonBody)]
#[write_component(ThrusterBank)]
pub fn autopilot(
    world: &mut SubWorld,
    #[resource] screen: &mut ScreenState,
    #[resource] mouse_delta: &MouseDelta,
    #[resource] dt: &Dt,
) {
    if mouse_delta.x != 0.0 || mouse_delta.y != 0.0 {
        let cam_axes = {
            let mut q = <(&Miner, &CameraComponent)>::query();
            q.iter(world).next().map(|(_, cam)| {
                let c = &cam.0;
                (DVec3::from(c.right), -DVec3::from(c.down))
            })
        };
        if let Some((cam_right, cam_up)) = cam_axes {
            let yaw = -(mouse_delta.x as f64) * MOUSE_SENSITIVITY;
            let pitch = -(mouse_delta.y as f64) * MOUSE_SENSITIVITY;
            let yaw_rot = DQuat::from_axis_angle(cam_up, yaw);
            let pitch_rot = DQuat::from_axis_angle(cam_right, pitch);
            screen.target_dir = (yaw_rot * pitch_rot * screen.target_dir).normalize();
        }
    }

    let target_dir = screen.target_dir;
    let dt_val = dt.0;

    let mut q = <(&Miner, &NewtonBody, &mut ThrusterBank)>::query();
    for (_, body, bank) in q.iter_mut(world) {
        apply_autopilot(body, bank, target_dir, dt_val);
    }
}

#[cfg(test)]
mod tests {
    use super::apply_autopilot;
    use crate::{
        components::{newton_body::NewtonBody, thruster::ThrusterBank},
        systems::thruster::apply_thrusters,
        Dt,
    };
    use glam::{DQuat, DVec3};
    use proptest::prelude::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn dir(yaw: f64, pitch: f64) -> DVec3 {
        DVec3::new(
            pitch.cos() * yaw.sin(),
            pitch.sin(),
            -pitch.cos() * yaw.cos(),
        )
        .normalize()
    }

    fn simulate(mut body: NewtonBody, target: DVec3, seconds: f64) -> NewtonBody {
        let dt = 1.0 / 60.0;
        let dt_obj = Dt(dt);
        for _ in 0..(seconds / dt) as usize {
            let mut bank = ThrusterBank::new(1.0, 0.75);
            apply_autopilot(&body, &mut bank, target, dt);
            apply_thrusters(&mut body, &mut bank);
            body.integrate_rotation(&dt_obj);
        }
        body
    }

    fn heading_error(body: &NewtonBody, target: DVec3) -> f64 {
        let heading = body.orientation * DVec3::NEG_Z;
        heading.dot(target).clamp(-1.0, 1.0).acos()
    }

    // ── apply_autopilot must not write pos, vel, angular_vel, or orientation ─

    #[test]
    fn does_not_write_body() {
        let pos = DVec3::new(42.0, -7.0, 100.0);
        let vel = DVec3::new(5.0, -3.0, 2.0);
        let ang = DVec3::new(0.1, 0.2, 0.3);
        let body = NewtonBody {
            mass: 1.0,
            pos,
            vel,
            orientation: DQuat::IDENTITY,
            angular_vel: ang,
        };
        let mut bank = ThrusterBank::new(1.0, 0.75);
        apply_autopilot(&body, &mut bank, DVec3::X, 1.0 / 60.0);
        assert_eq!(body.pos, pos);
        assert_eq!(body.vel, vel);
        assert_eq!(body.angular_vel, ang);
    }

    // ── No NaN / inf in command under arbitrary single-step inputs ──────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(300))]
        #[test]
        fn no_nan_or_inf(
            tgt_yaw   in -PI..PI,
            tgt_pitch in -FRAC_PI_2..FRAC_PI_2,
            ang_x in -10.0f64..10.0,
            ang_y in -10.0f64..10.0,
            ang_z in -10.0f64..10.0,
            dt in 0.005f64..0.05,
        ) {
            let target = dir(tgt_yaw, tgt_pitch);
            let body = NewtonBody {
                mass: 1.0,
                pos: DVec3::ZERO,
                vel: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                angular_vel: DVec3::new(ang_x, ang_y, ang_z),
            };
            let mut bank = ThrusterBank::new(1.0, 0.75);
            apply_autopilot(&body, &mut bank, target, dt);
            prop_assert!(bank.command.is_finite(), "command NaN/inf");
        }
    }

    // ── Heading must converge to target within 5 s ─────────────────────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(50))]
        #[test]
        fn heading_converges(
            tgt_yaw   in -PI..PI,
            tgt_pitch in -FRAC_PI_2..FRAC_PI_2,
        ) {
            let target = dir(tgt_yaw, tgt_pitch);
            let body = NewtonBody {
                mass: 1.0,
                pos: DVec3::ZERO,
                vel: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                angular_vel: DVec3::ZERO,
            };
            let err = heading_error(&simulate(body, target, 5.0), target);
            prop_assert!(
                err < 0.05,
                "heading_error={:.4} rad after 5 s; tgt_pitch={:.3}",
                err, tgt_pitch,
            );
        }
    }
}
