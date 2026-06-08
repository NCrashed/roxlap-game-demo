use glam::DVec3;

/// Remove the component of `v` along `axis` (assumed unit vector).
#[inline]
pub fn reject(v: DVec3, axis: DVec3) -> DVec3 {
    v - axis * v.dot(axis)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reject_removes_parallel_component() {
        let v = DVec3::new(1.0, 2.0, 3.0);
        let axis = DVec3::Y;
        let r = reject(v, axis);
        assert!(r.dot(axis).abs() < 1e-12, "parallel component not removed");
        assert!((r - DVec3::new(1.0, 0.0, 3.0)).length() < 1e-12);
    }

    #[test]
    fn reject_perpendicular_is_identity() {
        let v = DVec3::X;
        let r = reject(v, DVec3::Y);
        assert!((r - v).length() < 1e-12);
    }

    #[test]
    fn reject_parallel_is_zero() {
        let r = reject(DVec3::Y * 5.0, DVec3::Y);
        assert!(r.length() < 1e-12);
    }
}
