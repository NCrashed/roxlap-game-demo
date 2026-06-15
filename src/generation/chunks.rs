use std::collections::HashSet;

use glam::{DVec3, IVec3};

/// Side length of one chunk in world units.
pub const CHUNK_SIZE: i32 = 64;

/// Radius (in chunks) of the loaded sphere around the player.
pub const LOAD_RADIUS: i32 = 8;

/// Convert a world-space position to the chunk coordinate that contains it.
pub fn world_to_chunk(world_pos: DVec3) -> IVec3 {
    (world_pos / CHUNK_SIZE as f64).floor().as_ivec3()
}

/// Return all chunk coords within `radius` chunks of `center` (inclusive, Manhattan
/// approximated by axis-aligned cube; swap for sphere if needed).
pub fn chunks_in_sphere(center: IVec3, radius: i32) -> impl Iterator<Item = IVec3> {
    let r2 = radius * radius;
    (-radius..=radius).flat_map(move |dx| {
        (-radius..=radius).flat_map(move |dy| {
            (-radius..=radius).filter_map(move |dz| {
                let d = IVec3::new(dx, dy, dz);
                (d.dot(d) <= r2).then_some(center + d)
            })
        })
    })
}

/// Return chunk coords within `radius` chunks of `ship_pos` that are not yet generated.
pub fn missing_chunks(ship_pos: DVec3, radius: i32, generated: &HashSet<IVec3>) -> Vec<IVec3> {
    let center = world_to_chunk(ship_pos);
    chunks_in_sphere(center, radius)
        .filter(|c| !generated.contains(c))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_to_chunk_origin() {
        assert_eq!(world_to_chunk(DVec3::ZERO), IVec3::ZERO);
    }

    #[test]
    fn world_to_chunk_positive() {
        // 65 world units into chunk 1
        assert_eq!(
            world_to_chunk(DVec3::new(65.0, 0.0, 0.0)),
            IVec3::new(1, 0, 0)
        );
    }

    #[test]
    fn world_to_chunk_negative() {
        // -1 world units is in chunk -1 (floor division)
        assert_eq!(
            world_to_chunk(DVec3::new(-1.0, 0.0, 0.0)),
            IVec3::new(-1, 0, 0)
        );
    }

    #[test]
    fn world_to_chunk_boundary() {
        // exactly at a boundary belongs to the higher chunk
        assert_eq!(
            world_to_chunk(DVec3::new(64.0, 0.0, 0.0)),
            IVec3::new(1, 0, 0)
        );
    }

    #[test]
    fn chunks_in_sphere_radius0_is_just_center() {
        let result: Vec<_> = chunks_in_sphere(IVec3::ZERO, 0).collect();
        assert_eq!(result, vec![IVec3::ZERO]);
    }

    #[test]
    fn chunks_in_sphere_radius1_count() {
        // unit sphere in 3D integer grid: center + 6 face neighbours = 7
        let count = chunks_in_sphere(IVec3::ZERO, 1).count();
        assert_eq!(count, 7);
    }

    #[test]
    fn chunks_in_sphere_all_within_radius() {
        let radius = 3;
        let r2 = radius * radius;
        for c in chunks_in_sphere(IVec3::ZERO, radius) {
            assert!(c.dot(c) <= r2, "{c} is outside radius {radius}");
        }
    }

    #[test]
    fn chunks_in_sphere_no_duplicates() {
        let seen: HashSet<IVec3> = chunks_in_sphere(IVec3::ZERO, 3).collect();
        let count = chunks_in_sphere(IVec3::ZERO, 3).count();
        assert_eq!(seen.len(), count);
    }

    #[test]
    fn missing_chunks_empty_generated_returns_full_sphere() {
        let generated = HashSet::new();
        let missing = missing_chunks(DVec3::ZERO, 1, &generated);
        assert_eq!(missing.len(), 7);
    }

    #[test]
    fn missing_chunks_excludes_generated() {
        let center = IVec3::ZERO;
        let mut generated: HashSet<IVec3> = chunks_in_sphere(center, 1).collect();
        generated.remove(&IVec3::new(1, 0, 0));
        let missing = missing_chunks(DVec3::ZERO, 1, &generated);
        assert_eq!(missing, vec![IVec3::new(1, 0, 0)]);
    }

    #[test]
    fn missing_chunks_all_generated_returns_empty() {
        let generated: HashSet<IVec3> = chunks_in_sphere(IVec3::ZERO, 2).collect();
        let missing = missing_chunks(DVec3::ZERO, 2, &generated);
        assert!(missing.is_empty());
    }
}
