use glam::{DQuat, DVec3};
use legion::{world::SubWorld, *};

use crate::{
    components::{camera::CameraComponent, miner::Miner, newton_body::NewtonBody},
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

/// Rotate the ship's `angular_vel` toward `target_dir`.
/// Reads and writes `angular_vel` only — never touches `vel` or `pos`.
pub fn apply_autopilot(body: &mut NewtonBody, target_dir: DVec3, dt: f64) {
    let ship_fwd = body.orientation * DVec3::NEG_Z;

    let steer_cross = ship_fwd.cross(target_dir);
    let steer_sin = steer_cross.length();
    let steer_cos = ship_fwd.dot(target_dir);
    let steer_angle = steer_sin.atan2(steer_cos);

    let smooth = (STEER_SMOOTH * dt * 60.0).min(1.0);
    if steer_angle.abs() > 0.001 {
        // When ship_fwd ≈ -target_dir the cross product is ~0 but angle ≈ π;
        // pick an arbitrary perpendicular axis rather than normalising a zero vector.
        let steer_axis = if steer_sin > 1e-9 {
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
        let desired = steer_axis * desired_speed;
        body.angular_vel += (desired - body.angular_vel) * smooth;
    } else {
        // Already aligned — damp residual spin.
        body.angular_vel *= 1.0 - smooth;
    }
}

#[system]
#[read_component(Miner)]
#[read_component(CameraComponent)]
#[write_component(NewtonBody)]
pub fn autopilot(
    world: &mut SubWorld,
    #[resource] screen: &mut ScreenState,
    #[resource] mouse_delta: &MouseDelta,
    #[resource] dt: &Dt,
) {
    // Update world-space target direction from mouse input, using the current
    // camera axes so the rotation feels screen-relative.
    if mouse_delta.x != 0.0 || mouse_delta.y != 0.0 {
        // Read camera axes from the first miner entity.
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

    let mut q = <(&Miner, &mut NewtonBody)>::query();
    for (_, body) in q.iter_mut(world) {
        apply_autopilot(body, target_dir, dt.0);
    }
}

#[cfg(test)]
mod tests {
    use super::apply_autopilot;
    use crate::{components::newton_body::NewtonBody, Dt};
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

    /// Simulate autopilot + orientation integration; pos and vel are untouched.
    fn simulate(mut body: NewtonBody, target: DVec3, seconds: f64) -> NewtonBody {
        let dt = 1.0 / 60.0;
        let dt_obj = Dt(dt);
        for _ in 0..(seconds / dt) as usize {
            apply_autopilot(&mut body, target, dt);
            body.integrate_rotation(&dt_obj);
        }
        body
    }

    fn heading_error(body: &NewtonBody, target: DVec3) -> f64 {
        let heading = body.orientation * DVec3::NEG_Z;
        heading.dot(target).clamp(-1.0, 1.0).acos()
    }

    // ── apply_autopilot must not write pos or vel ───────────────────────────

    #[test]
    fn does_not_write_pos_or_vel() {
        let pos = DVec3::new(42.0, -7.0, 100.0);
        let vel = DVec3::new(5.0, -3.0, 2.0);
        let mut body = NewtonBody {
            mass: 1.0,
            pos,
            vel,
            orientation: DQuat::IDENTITY,
            angular_vel: DVec3::ZERO,
        };
        apply_autopilot(&mut body, DVec3::X, 1.0 / 60.0);
        assert_eq!(body.pos, pos, "apply_autopilot wrote pos");
        assert_eq!(body.vel, vel, "apply_autopilot wrote vel");
    }

    // ── No NaN / inf under arbitrary single-step inputs ────────────────────

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
            let mut body = NewtonBody {
                mass: 1.0,
                pos: DVec3::ZERO,
                vel: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                angular_vel: DVec3::new(ang_x, ang_y, ang_z),
            };
            apply_autopilot(&mut body, target, dt);
            prop_assert!(body.angular_vel.is_finite(), "angular_vel NaN/inf");
            prop_assert_eq!(body.vel, DVec3::ZERO, "vel must not change");
            prop_assert_eq!(body.pos, DVec3::ZERO, "pos must not change");
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
