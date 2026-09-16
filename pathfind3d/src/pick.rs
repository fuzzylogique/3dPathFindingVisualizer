//! Turning a mouse ray into a cell.
//!
//! The old editor picked cells by raycasting a slab of translucent "ghost"
//! boxes that the user had to position first, one grid layer at a time. That
//! made every edit a two-step operation and made empty space unclickable.
//!
//! Here the front end hands over the camera ray and gets back cells. A ray is
//! marched in sub-cell steps and each sample is mapped straight back to a
//! cell through `Domain::world_to_cell`, so curved domains (the torus) work
//! the same way lattice ones do without a second code path.
//!
//! Two results come out of one march:
//!
//! * `solid` — the first wall the ray meets, i.e. what a click erases.
//! * `free`  — the open cell just before it, i.e. where a click builds.
//!
//! When the ray meets no wall at all there is nothing to anchor against, so
//! `on_plane` intersects the ray with the camera-facing plane through a
//! reference cell instead. That is what makes dragging the start and goal
//! markers through open space feel direct.

use crate::domain::Domain;
use crate::{T_OBST, T_VOID};

/// Sub-cell march increment. Cells are one world unit across, so five
/// samples per cell never skips one in practice.
const STEP: f32 = 0.2;

#[derive(Clone, Copy, Default)]
pub struct Hit {
    /// First blocked cell along the ray, or `-1`.
    pub solid: i32,
    /// Last open, existing cell before `solid` (or before the ray left the
    /// domain), or `-1`.
    pub free: i32,
    /// First existing cell of any kind, or `-1`.
    pub any: i32,
}

impl Hit {
    fn miss() -> Hit {
        Hit {
            solid: -1,
            free: -1,
            any: -1,
        }
    }
}

/// Clip a ray against the domain's bounding box, padded by a cell so grazing
/// hits on the outer shell still register. Returns the `t` interval to march,
/// or `None` if the ray misses the box entirely.
fn clip(dom: &Domain, o: [f32; 3], d: [f32; 3]) -> Option<(f32, f32)> {
    // A curved domain's cells wrap around the origin, so the axis-aligned box
    // is the right conservative bound there too.
    let mut t0 = 0.0f32;
    let mut t1 = f32::INFINITY;
    for a in 0..3 {
        let lo = dom.bmin[a] - 1.0;
        let hi = dom.bmax[a] + 1.0;
        if d[a].abs() < 1e-6 {
            if o[a] < lo || o[a] > hi {
                return None;
            }
            continue;
        }
        let mut ta = (lo - o[a]) / d[a];
        let mut tb = (hi - o[a]) / d[a];
        if ta > tb {
            std::mem::swap(&mut ta, &mut tb);
        }
        t0 = t0.max(ta);
        t1 = t1.min(tb);
        if t0 > t1 {
            return None;
        }
    }
    Some((t0, t1))
}

fn normalize(d: [f32; 3]) -> [f32; 3] {
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len < 1e-9 {
        [0.0, 0.0, 1.0]
    } else {
        [d[0] / len, d[1] / len, d[2] / len]
    }
}

/// March the ray and report the first wall plus the open cell in front of it.
///
/// `stop_weighted` widens "wall" to include any cell carrying a weight of 2 or
/// more. The weight brush uses that so it builds outward from an existing
/// energy field exactly the way the wall brush builds outward from a wall,
/// instead of shooting straight through it.
pub fn ray(
    dom: &Domain,
    terrain: &[u8],
    origin: [f32; 3],
    dir: [f32; 3],
    stop_weighted: bool,
) -> Hit {
    let d = normalize(dir);
    let Some((t0, t1)) = clip(dom, origin, d) else {
        return Hit::miss();
    };
    let mut hit = Hit::miss();
    let mut last = u32::MAX;
    let mut t = t0;
    while t <= t1 {
        let p = [
            origin[0] + d[0] * t,
            origin[1] + d[1] * t,
            origin[2] + d[2] * t,
        ];
        t += STEP;
        let Some(cell) = dom.world_to_cell(p[0], p[1], p[2]) else {
            continue;
        };
        if cell == last || !dom.exists_at(cell) {
            continue;
        }
        last = cell;
        if hit.any < 0 {
            hit.any = cell as i32;
        }
        match terrain[cell as usize] {
            T_VOID => continue,
            T_OBST => {
                hit.solid = cell as i32;
                return hit;
            }
            w if stop_weighted && w >= 2 => {
                hit.solid = cell as i32;
                return hit;
            }
            _ => hit.free = cell as i32,
        }
    }
    hit
}

/// The cell where the ray crosses the camera-facing plane through `anchor`.
/// `dir` doubles as the plane normal, which is what keeps a dragged marker
/// tracking the cursor instead of sliding toward or away from the camera.
pub fn on_plane(dom: &Domain, origin: [f32; 3], dir: [f32; 3], anchor: u32) -> Option<u32> {
    let d = normalize(dir);
    let a = dom.world_of(anchor);
    let denom = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if denom < 1e-9 {
        return None;
    }
    let t = ((a[0] - origin[0]) * d[0] + (a[1] - origin[1]) * d[1] + (a[2] - origin[2]) * d[2])
        / denom;
    if t <= 0.0 {
        return None;
    }
    dom.world_to_cell(
        origin[0] + d[0] * t,
        origin[1] + d[1] * t,
        origin[2] + d[2] * t,
    )
}

/// Nearest existing, unblocked cell to `cell`, searched in growing shells.
/// `avoid` keeps the two endpoints from landing on each other. `-1` when
/// nothing suitable is within reach.
pub fn snap(dom: &Domain, terrain: &[u8], cell: u32, avoid: i32) -> i32 {
    let ok = |j: u32| {
        dom.exists_at(j) && terrain[j as usize] != T_OBST && j as i32 != avoid
    };
    if (cell as usize) < dom.total() && ok(cell) {
        return cell as i32;
    }
    let (cx, cy, cz) = dom.coords(cell);
    for r in 1..=6i32 {
        let mut best = -1i32;
        let mut best_d = i32::MAX;
        for dz in -r..=r {
            for dy in -r..=r {
                for dx in -r..=r {
                    if dx.abs().max(dy.abs()).max(dz.abs()) != r {
                        continue;
                    }
                    let (Some(x), Some(y), Some(z)) = (
                        dom.fold(cx + dx, 0),
                        dom.fold(cy + dy, 1),
                        dom.fold(cz + dz, 2),
                    ) else {
                        continue;
                    };
                    let j = dom.idx(x, y, z);
                    let dd = dx * dx + dy * dy + dz * dz;
                    if dd < best_d && ok(j) {
                        best_d = dd;
                        best = j as i32;
                    }
                }
            }
        }
        if best >= 0 {
            return best;
        }
    }
    -1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::Preset;

    #[test]
    fn ray_finds_the_first_wall_and_the_cell_in_front_of_it() {
        let dom = Domain::preset(Preset::Cube, 10);
        let mut terrain = vec![1u8; dom.total()];
        // A wall plane at x = 5; shoot down +x from outside.
        for z in 0..dom.nz {
            for y in 0..dom.ny {
                terrain[dom.idx(5, y, z) as usize] = T_OBST;
            }
        }
        let target = dom.idx(5, 4, 4);
        let p = dom.world_of(target);
        let hit = ray(&dom, &terrain, [p[0] - 20.0, p[1], p[2]], [1.0, 0.0, 0.0], false);
        assert_eq!(hit.solid, target as i32, "must stop at the wall");
        assert_eq!(
            hit.free,
            dom.idx(4, 4, 4) as i32,
            "must report the open cell in front of the wall"
        );
    }

    #[test]
    fn ray_through_open_space_reports_no_wall() {
        let dom = Domain::preset(Preset::Cube, 8);
        let terrain = vec![1u8; dom.total()];
        let p = dom.world_of(dom.idx(4, 4, 4));
        let hit = ray(&dom, &terrain, [p[0] - 20.0, p[1], p[2]], [1.0, 0.0, 0.0], false);
        assert_eq!(hit.solid, -1);
        assert!(hit.free >= 0, "an open cell should still be reported");
    }

    #[test]
    fn weighted_cells_stop_the_ray_only_for_the_weight_brush() {
        let dom = Domain::preset(Preset::Cube, 10);
        let mut terrain = vec![1u8; dom.total()];
        let heavy = dom.idx(5, 4, 4);
        terrain[heavy as usize] = 6;
        let p = dom.world_of(heavy);
        let o = [p[0] - 20.0, p[1], p[2]];

        let wall_brush = ray(&dom, &terrain, o, [1.0, 0.0, 0.0], false);
        assert_eq!(wall_brush.solid, -1, "a weighted cell is not a wall");

        let weight_brush = ray(&dom, &terrain, o, [1.0, 0.0, 0.0], true);
        assert_eq!(weight_brush.solid, heavy as i32);
        assert_eq!(weight_brush.free, dom.idx(4, 4, 4) as i32);
    }

    #[test]
    fn ray_that_misses_the_domain_hits_nothing() {
        let dom = Domain::preset(Preset::Cube, 8);
        let terrain = vec![1u8; dom.total()];
        let hit = ray(&dom, &terrain, [0.0, 500.0, 0.0], [1.0, 0.0, 0.0], false);
        assert_eq!(hit.solid, -1);
        assert_eq!(hit.free, -1);
        assert_eq!(hit.any, -1);
    }

    #[test]
    fn every_preset_round_trips_cell_to_world_and_back() {
        for preset in [
            Preset::Cube,
            Preset::Prism,
            Preset::Pyramid,
            Preset::Sphere,
            Preset::Torus,
        ] {
            let dom = Domain::preset(preset, 14);
            for i in 0..dom.total() {
                if dom.exists[i] == 0 {
                    continue;
                }
                let p = dom.world_of(i as u32);
                assert_eq!(
                    dom.world_to_cell(p[0], p[1], p[2]),
                    Some(i as u32),
                    "{:?}: cell {i} did not round-trip",
                    preset
                );
            }
        }
    }

    #[test]
    fn picking_works_on_the_curved_torus_too() {
        let dom = Domain::preset(Preset::Torus, 14);
        let mut terrain: Vec<u8> = dom
            .exists
            .iter()
            .map(|&e| if e == 1 { 1 } else { T_VOID })
            .collect();
        // Aim straight down at the top of the ring.
        let target = {
            let mut best = 0u32;
            let mut best_y = f32::NEG_INFINITY;
            for i in 0..dom.total() {
                if dom.exists[i] == 1 && dom.py[i] > best_y {
                    best_y = dom.py[i];
                    best = i as u32;
                }
            }
            best
        };
        terrain[target as usize] = T_OBST;
        let p = dom.world_of(target);
        let hit = ray(&dom, &terrain, [p[0], p[1] + 20.0, p[2]], [0.0, -1.0, 0.0], false);
        assert_eq!(hit.solid, target as i32);
    }

    #[test]
    fn snap_moves_off_a_wall_to_the_nearest_open_cell() {
        let dom = Domain::preset(Preset::Cube, 8);
        let mut terrain = vec![1u8; dom.total()];
        let blocked = dom.idx(4, 4, 4);
        terrain[blocked as usize] = T_OBST;
        let got = snap(&dom, &terrain, blocked, -1);
        assert!(got >= 0 && got as u32 != blocked);
        assert_ne!(terrain[got as usize], T_OBST);
    }

    #[test]
    fn snap_will_not_land_on_the_avoided_cell() {
        let dom = Domain::preset(Preset::Cube, 8);
        let mut terrain = vec![1u8; dom.total()];
        let blocked = dom.idx(4, 4, 4);
        terrain[blocked as usize] = T_OBST;
        let neighbour = dom.idx(5, 4, 4) as i32;
        let got = snap(&dom, &terrain, blocked, neighbour);
        assert!(got >= 0);
        assert_ne!(got, neighbour);
    }

    #[test]
    fn on_plane_tracks_the_cursor_across_the_camera_facing_plane() {
        let dom = Domain::preset(Preset::Cube, 12);
        let anchor = dom.idx(6, 6, 6);
        let a = dom.world_of(anchor);
        // A ray aimed straight at the anchor resolves to the anchor itself.
        let origin = [a[0], a[1], a[2] + 30.0];
        let dir = [0.0, 0.0, -1.0];
        assert_eq!(on_plane(&dom, origin, dir, anchor), Some(anchor));
        // Sliding the origin sideways slides the result by the same cells.
        let origin2 = [a[0] + 2.0, a[1] - 1.0, a[2] + 30.0];
        assert_eq!(
            on_plane(&dom, origin2, dir, anchor),
            Some(dom.idx(8, 5, 6)),
            "the plane hit should track the ray offset"
        );
    }
}
