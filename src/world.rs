use rand::RngExt;
use roxlap_cavegen::pack_dense_grid_to_vxl;
use roxlap_formats::{edit::MAXZDIM, vxl::Vxl};

pub const VSID: u32 = 64;

/// Z-coord of the (one-voxel-thick) ground plane. Voxlap is **z-down**:
/// small z is up, large z is down.
pub const GROUND_Z: i32 = 200;

pub const CUBE_VXL_VSID: u32 = 16;
pub const CUBE_VXL_EDGE: i32 = 16;

/// Holds both the static ground world and the pre-lit cube VXL.
/// Bundled into one resource to stay within Legion's 8-resource-per-system limit.
pub struct Worlds {
    pub base: Vxl,
    pub cube: Vxl,
}

fn random_voxel_colour(rng: &mut impl rand::Rng) -> u32 {
    0x80_00_00_00 | (rng.random::<u32>() & 0x00_FF_FF_FF)
}

pub fn build_world() -> Vxl {
    let vsid_u = VSID as usize;
    let maxz_u = MAXZDIM as usize;
    let cells = vsid_u * vsid_u * maxz_u;

    let mut mask = vec![0u8; cells];
    let mut colour = vec![0u32; cells];
    let idx = |x: usize, y: usize, z: usize| -> usize { (y * vsid_u + x) * maxz_u + z };
    let mut rng = rand::rng();
    for y in 0..vsid_u {
        for x in 0..vsid_u {
            let i = idx(x, y, GROUND_Z as usize);
            mask[i] = 1;
            colour[i] = random_voxel_colour(&mut rng);
        }
    }
    pack_dense_grid_to_vxl(&mask, &colour, VSID)
}

pub fn build_cube_vxl() -> Vxl {
    let vsid = CUBE_VXL_VSID as usize;
    let maxz_u = MAXZDIM as usize;
    let cells = vsid * vsid * maxz_u;

    let mut mask = vec![0u8; cells];
    let mut colour = vec![0u32; cells];
    let idx = |x: usize, y: usize, z: usize| -> usize { (y * vsid + x) * maxz_u + z };

    let center = CUBE_VXL_VSID as f64 / 2.0;
    let radius = center - 0.5;

    let mut rng = rand::rng();
    for y in 0..vsid {
        for x in 0..vsid {
            for z in 0..vsid {
                let dx = x as f64 + 0.5 - center;
                let dy = y as f64 + 0.5 - center;
                let dz = z as f64 + 0.5 - center;
                if dx * dx + dy * dy + dz * dz <= radius * radius {
                    mask[idx(x, y, z)] = 1;
                    colour[idx(x, y, z)] = random_voxel_colour(&mut rng);
                }
            }
        }
    }
    pack_dense_grid_to_vxl(&mask, &colour, CUBE_VXL_VSID)
}
