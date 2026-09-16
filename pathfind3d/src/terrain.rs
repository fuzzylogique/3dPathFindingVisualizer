//! World generation.
//!
//! Terrain bytes: `T_VOID` outside the domain mask, `1..=W_MAX` = a cell of
//! that weight, `T_OBST` = a wall. Everything comes from one seeded PRNG, so
//! a seed reproduces a world exactly.
//!
//! The generator layers perforated cross-section walls, floating ellipsoidal
//! obstacle clusters, isolated scatter, and weighted "dense energy" blobs,
//! then carves the start/goal neighbourhoods back to clean ground so the
//! endpoints can never be boxed in.

use crate::domain::Domain;
use crate::{T_OBST, T_VOID, W_MAX};

/// mulberry32, bit-identical to the JS reference implementation so seeds
/// produce the same worlds they always did.
pub struct Rng {
    a: u32,
}

impl Rng {
    pub fn new(seed: u32) -> Rng {
        Rng { a: seed }
    }

    /// Next value in the unit interval `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        self.a = self.a.wrapping_add(0x6D2B_79F5);
        let a = self.a;
        let mut t = (a ^ (a >> 15)).wrapping_mul(1 | a);
        t = t.wrapping_add((t ^ (t >> 7)).wrapping_mul(61 | t)) ^ t;
        ((t ^ (t >> 14)) as f64) / 4_294_967_296.0
    }

    /// `(rnd() * n) | 0` — truncating, matching the JS call sites.
    fn below(&mut self, n: i32) -> i32 {
        (self.unit() * n as f64) as i32
    }
}

pub fn generate(dom: &Domain, density_pct: u32, seed: u32, empty: bool) -> Vec<u8> {
    let (nx, ny, nz) = (dom.nx, dom.ny, dom.nz);
    let total = dom.total();
    let density = density_pct as f64 / 100.0;
    let mut rnd = Rng::new(seed);
    let mut terrain: Vec<u8> = dom
        .exists
        .iter()
        .map(|&e| if e == 1 { 1 } else { T_VOID })
        .collect();
    let dims = [nx, ny, nz];
    let id = |x: i32, y: i32, z: i32| (x + nx * (y + ny * z)) as usize;

    if !empty {
        let max_dim = nx.max(ny).max(nz);

        // Perforated walls: full cross-sections with punched holes. On the
        // torus this is a disc across the bore — the search has to find a
        // hole or go the other way around the ring.
        let wall_count = if max_dim >= 18 { 3 } else { 2 };
        let mut used: Vec<i32> = Vec::new();
        for _ in 0..wall_count {
            let axis = rnd.below(3) as usize;
            let n = dims[axis];
            if n < 5 {
                continue;
            }
            let mut p = 2 + rnd.below(n - 4);
            let taken = |used: &Vec<i32>, p: i32| {
                let k = axis as i32 * 1000 + p;
                used.contains(&k) || used.contains(&(k + 1)) || used.contains(&(k - 1))
            };
            let mut guard = 0;
            while guard < 24 && taken(&used, p) {
                p = 2 + rnd.below(n - 4);
                guard += 1;
            }
            used.push(axis as i32 * 1000 + p);

            // The two grid axes the wall plane spans.
            let uu = match axis {
                0 => [1usize, 2],
                1 => [0, 2],
                _ => [0, 1],
            };
            let (du, dv) = (dims[uu[0]], dims[uu[1]]);
            let put = |terrain: &mut Vec<u8>, u: i32, v: i32, val: u8| {
                let c = match axis {
                    0 => (p, u, v),
                    1 => (u, p, v),
                    _ => (u, v, p),
                };
                let i = id(c.0, c.1, c.2);
                if dom.exists[i] == 1 {
                    terrain[i] = val;
                }
            };
            for u in 0..du {
                for v in 0..dv {
                    put(&mut terrain, u, v, T_OBST);
                }
            }
            let holes = 1 + rnd.below(2);
            for _ in 0..holes {
                let hu = 1 + rnd.below(du - 2);
                let hv = 1 + rnd.below(dv - 2);
                // Identical arms on purpose: the small-domain case has to
                // consume a random value, the large one must not.
                #[allow(clippy::if_same_then_else)]
                let r = if max_dim >= 16 {
                    1
                } else if rnd.unit() < 0.5 {
                    1
                } else {
                    0
                };
                for d_u in -r..=r {
                    for d_v in -r..=r {
                        let (cu, cv) = (hu + d_u, hv + d_v);
                        if cu >= 0 && cv >= 0 && cu < du && cv < dv {
                            put(&mut terrain, cu, cv, 1);
                        }
                    }
                }
            }
            for u in 0..du {
                for v in 0..dv {
                    if rnd.unit() < 0.05 {
                        put(&mut terrain, u, v, 1);
                    }
                }
            }
        }

        // Floating obstacle clusters: ellipsoidal blobs suspended in the volume.
        let blobs = 2 + (density * 14.0).round() as i32 + max_dim / 9;
        for _ in 0..blobs {
            let cx = 1.0 + rnd.unit() * (nx - 2) as f64;
            let cy = 1.0 + rnd.unit() * (ny - 2) as f64;
            let cz = 1.0 + rnd.unit() * (nz - 2) as f64;
            let rx = 1.0 + rnd.unit() * nx as f64 * 0.11;
            let ry = 1.0 + rnd.unit() * ny as f64 * 0.11;
            let rz = 1.0 + rnd.unit() * nz as f64 * 0.11;
            let x0 = 0.max((cx - rx).floor() as i32);
            let x1 = (nx - 1).min((cx + rx).ceil() as i32);
            let y0 = 0.max((cy - ry).floor() as i32);
            let y1 = (ny - 1).min((cy + ry).ceil() as i32);
            let z0 = 0.max((cz - rz).floor() as i32);
            let z1 = (nz - 1).min((cz + rz).ceil() as i32);
            for z in z0..=z1 {
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        let dd = ((x as f64 - cx) / rx).powi(2)
                            + ((y as f64 - cy) / ry).powi(2)
                            + ((z as f64 - cz) / rz).powi(2);
                        let i = id(x, y, z);
                        if dd <= 1.0 && rnd.unit() < 0.9 && dom.exists[i] == 1 {
                            terrain[i] = T_OBST;
                        }
                    }
                }
            }
        }

        // Isolated scatter voxels.
        let scatter = density * 0.35;
        for (i, t) in terrain.iter_mut().enumerate() {
            if dom.exists[i] == 1 && rnd.unit() < scatter {
                *t = T_OBST;
            }
        }

        // Weighted terrain: dense-energy blobs the weighted algorithms can
        // route around or pay to push through. Weight 1 stays normal ground.
        let w_blobs = 2 + (total as f64 / 1400.0).round() as i32;
        for _ in 0..w_blobs {
            let w = 3 + rnd.below(W_MAX as i32 - 2) as u8;
            let cx = 1.0 + rnd.unit() * (nx - 2) as f64;
            let cy = 1.0 + rnd.unit() * (ny - 2) as f64;
            let cz = 1.0 + rnd.unit() * (nz - 2) as f64;
            let rr = 1.2 + rnd.unit() * max_dim as f64 * 0.14;
            let x0 = 0.max((cx - rr).floor() as i32);
            let x1 = (nx - 1).min((cx + rr).ceil() as i32);
            let y0 = 0.max((cy - rr).floor() as i32);
            let y1 = (ny - 1).min((cy + rr).ceil() as i32);
            let z0 = 0.max((cz - rr).floor() as i32);
            let z1 = (nz - 1).min((cz + rr).ceil() as i32);
            for z in z0..=z1 {
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        let dd = ((x as f64 - cx).powi(2)
                            + (y as f64 - cy).powi(2)
                            + (z as f64 - cz).powi(2))
                            / (rr * rr);
                        let i = id(x, y, z);
                        if dd <= 1.0 && dom.exists[i] == 1 && terrain[i] != T_OBST {
                            // soft core: full weight inside, lighter fringe
                            terrain[i] = if dd < 0.55 { w } else { 2.max(w >> 1) };
                        }
                    }
                }
            }
        }
    }

    carve(dom, &mut terrain, dom.start);
    carve(dom, &mut terrain, dom.goal);
    terrain
}

/// Clamp stray bytes into the encoding: void and obstacle markers survive,
/// everything else becomes a weight in `1..=W_MAX`.
pub fn sanitize(terrain: &mut [u8]) {
    for t in terrain.iter_mut() {
        if *t == T_VOID || *t == T_OBST {
            continue;
        }
        *t = (*t).clamp(1, W_MAX);
    }
}

/// The endpoints themselves must never be obstacles, whatever the generator
/// produced. Void endpoints stay void: that is "no path".
pub fn free_endpoints(terrain: &mut [u8], start: u32, goal: u32) {
    for e in [start, goal] {
        let i = e as usize;
        if i < terrain.len() && terrain[i] == T_OBST {
            terrain[i] = 1;
        }
    }
}

/// Reset a cell and its (wrap-aware) 26-neighbourhood to clean weight-1
/// ground, so an endpoint can never sit inside a wall or be sealed in.
pub fn carve(dom: &Domain, terrain: &mut [u8], cell: u32) {
    let (cx, cy, cz) = dom.coords(cell);
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (Some(x), Some(y), Some(z)) = (
                    dom.fold(cx + dx, 0),
                    dom.fold(cy + dy, 1),
                    dom.fold(cz + dz, 2),
                ) else {
                    continue;
                };
                let i = dom.idx(x, y, z) as usize;
                if dom.exists[i] == 1 {
                    terrain[i] = 1;
                }
            }
        }
    }
}
