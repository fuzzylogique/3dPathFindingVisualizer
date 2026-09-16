//! Domain geometry — the shape of the world.
//!
//! A domain is per-axis dims, per-axis toroidal wrap, a per-cell existence
//! mask, and a world-space position for every cell. Positions live here
//! (rather than in the renderer) because picking needs the *inverse* map,
//! world point -> cell, and the two must agree exactly. The torus is the
//! reason: it bends grid axis x around a ring, so "cell 7 is at x=7" is
//! only true for the lattice presets.
//!
//! Index convention everywhere: `idx = x + nx*(y + ny*z)`.

use std::f32::consts::PI;

/// The domain floats this far above the renderer's floor grid.
pub const LIFT: f32 = 2.6;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Preset {
    Cube,
    Prism,
    Pyramid,
    Sphere,
    Torus,
}

impl Preset {
    pub fn parse(s: &str) -> Preset {
        match s {
            "prism" => Preset::Prism,
            "pyramid" => Preset::Pyramid,
            "sphere" => Preset::Sphere,
            "torus" => Preset::Torus,
            _ => Preset::Cube,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Preset::Cube => "cube",
            Preset::Prism => "prism",
            Preset::Pyramid => "pyramid",
            Preset::Sphere => "sphere",
            Preset::Torus => "torus",
        }
    }
}

pub struct Domain {
    pub preset: Preset,
    pub nx: i32,
    pub ny: i32,
    pub nz: i32,
    pub wrap: [bool; 3],
    /// 1 = the cell is part of the domain, 0 = void (untraversable, unrendered).
    pub exists: Vec<u8>,
    /// Per-cell world position, and the Y rotation that aligns a cell's cube
    /// with the ring frame (zero for every lattice preset).
    pub px: Vec<f32>,
    pub py: Vec<f32>,
    pub pz: Vec<f32>,
    pub rot: Vec<f32>,
    /// True when positions are not a plain lattice, so `world_to_cell` has to
    /// undo a curve instead of a translation.
    pub curved: bool,
    pub ring_r: f32,
    pub start: u32,
    pub goal: u32,
    pub count: u32,
    pub label: String,
    /// World-space bounds of the existing cell centres.
    pub bmin: [f32; 3],
    pub bmax: [f32; 3],
}

impl Domain {
    /// Build one of the named presets at nominal size `n`.
    pub fn preset(preset: Preset, n: u32) -> Domain {
        let n = n.max(4) as i32;
        let nf = n as f64;
        match preset {
            Preset::Prism => {
                let ny = 6.max((nf * 0.5).round() as i32);
                let nz = 8.max((nf * 0.8).round() as i32);
                Domain::build(
                    preset,
                    n,
                    ny,
                    nz,
                    [false, false, false],
                    |_, _, _| true,
                    (1, 1, 1),
                    (n - 2, ny - 2, nz - 2),
                    format!("{n}×{ny}×{nz}"),
                )
            }
            Preset::Pyramid => {
                let ny = 6.max((nf * 0.75).round() as i32);
                // Keep the apex at least 3 cells wide or the top is unwalkable.
                let max_shrink = (n - 3) / 2;
                let shrink = move |y: i32| (max_shrink * y) / (ny - 1);
                let c = n / 2;
                Domain::build(
                    preset,
                    n,
                    ny,
                    n,
                    [false, false, false],
                    move |x, y, z| {
                        let m = shrink(y);
                        x >= m && x <= n - 1 - m && z >= m && z <= n - 1 - m
                    },
                    (1, 0, 1),
                    (c, ny - 1, c),
                    format!("{n} base · {ny} tall"),
                )
            }
            Preset::Sphere => {
                let c = (n - 1) as f64 / 2.0;
                let r = c + 0.2;
                let rc = c.round() as i32;
                Domain::build(
                    preset,
                    n,
                    n,
                    n,
                    [false, false, false],
                    move |x, y, z| {
                        let (dx, dy, dz) = (x as f64 - c, y as f64 - c, z as f64 - c);
                        dx * dx + dy * dy + dz * dz <= r * r
                    },
                    (rc, 1, rc),
                    (rc, n - 2, rc),
                    format!("r ≈ {r:.1}"),
                )
            }
            Preset::Torus => {
                let nu = 24.max((nf * 2.4).round() as i32);
                let d = 5.max(((nf * 0.42).round() as i32) | 1); // odd -> a true centre cell
                let c = (d - 1) as f64 / 2.0;
                let r = c + 0.2;
                let ci = c as i32;
                // Endpoints straddle the wrap seam: the short way is through it,
                // the long way is all the way around the ring.
                Domain::build(
                    preset,
                    nu,
                    d,
                    d,
                    [true, false, false],
                    move |_, y, z| {
                        let (dy, dz) = (y as f64 - c, z as f64 - c);
                        dy * dy + dz * dz <= r * r
                    },
                    (2, ci, ci),
                    (nu - 3, ci, ci),
                    format!("ring {nu} · bore {d}"),
                )
            }
            Preset::Cube => Domain::build(
                preset,
                n,
                n,
                n,
                [false, false, false],
                |_, _, _| true,
                (1, 1, 1),
                (n - 2, n - 2, n - 2),
                format!("{n}³"),
            ),
        }
    }

    /// A plain full lattice with no mask — the shape the solver tests use.
    pub fn raw(nx: u32, ny: u32, nz: u32, wrap: [bool; 3]) -> Domain {
        let (nx, ny, nz) = (nx as i32, ny as i32, nz as i32);
        Domain::build(
            Preset::Cube,
            nx,
            ny,
            nz,
            wrap,
            |_, _, _| true,
            (0, 0, 0),
            (nx - 1, ny - 1, nz - 1),
            format!("{nx}×{ny}×{nz}"),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn build<F: Fn(i32, i32, i32) -> bool>(
        preset: Preset,
        nx: i32,
        ny: i32,
        nz: i32,
        wrap: [bool; 3],
        exists_fn: F,
        start: (i32, i32, i32),
        goal: (i32, i32, i32),
        label: String,
    ) -> Domain {
        let total = (nx * ny * nz) as usize;
        let mut exists = vec![0u8; total];
        let mut count = 0u32;
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    if exists_fn(x, y, z) {
                        exists[(x + nx * (y + ny * z)) as usize] = 1;
                        count += 1;
                    }
                }
            }
        }
        let id = |c: (i32, i32, i32)| (c.0 + nx * (c.1 + ny * c.2)) as u32;
        let mut dom = Domain {
            preset,
            nx,
            ny,
            nz,
            wrap,
            exists,
            px: vec![0.0; total],
            py: vec![0.0; total],
            pz: vec![0.0; total],
            rot: vec![0.0; total],
            curved: preset == Preset::Torus,
            ring_r: 0.0,
            start: id(start),
            goal: id(goal),
            count,
            label,
            bmin: [0.0; 3],
            bmax: [0.0; 3],
        };
        dom.build_positions();
        dom
    }

    /// Per-cell world positions. Lattice presets sit on a centred grid; the
    /// torus bends grid axis x around a ring so the wrap seam is a physical
    /// seam (grid y stays vertical, grid z becomes radial depth).
    fn build_positions(&mut self) {
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        let total = (nx * ny * nz) as usize;
        if self.curved {
            let r = (nx as f32 / (2.0 * PI)) * 1.06;
            self.ring_r = r;
            let c = (ny - 1) as f32 / 2.0;
            let cy = LIFT + 0.5 + c;
            for i in 0..total {
                let ii = i as i32;
                let u = ii % nx;
                let v = (ii / nx) % ny;
                let w = ii / (nx * ny);
                let th = u as f32 * 2.0 * PI / nx as f32;
                let rad = r + (w as f32 - c);
                self.px[i] = rad * th.cos();
                self.pz[i] = rad * th.sin();
                self.py[i] = cy + (v as f32 - c);
                self.rot[i] = -th;
            }
        } else {
            let hx = (nx - 1) as f32 / 2.0;
            let hz = (nz - 1) as f32 / 2.0;
            for i in 0..total {
                let ii = i as i32;
                self.px[i] = (ii % nx) as f32 - hx;
                self.py[i] = LIFT + 0.5 + ((ii / nx) % ny) as f32;
                self.pz[i] = (ii / (nx * ny)) as f32 - hz;
            }
        }
        let mut bmin = [f32::INFINITY; 3];
        let mut bmax = [f32::NEG_INFINITY; 3];
        for i in 0..total {
            if self.exists[i] == 0 {
                continue;
            }
            for (a, v) in [self.px[i], self.py[i], self.pz[i]].iter().enumerate() {
                if *v < bmin[a] {
                    bmin[a] = *v;
                }
                if *v > bmax[a] {
                    bmax[a] = *v;
                }
            }
        }
        self.bmin = bmin;
        self.bmax = bmax;
    }

    #[inline]
    pub fn total(&self) -> usize {
        (self.nx * self.ny * self.nz) as usize
    }

    #[inline]
    pub fn idx(&self, x: i32, y: i32, z: i32) -> u32 {
        (x + self.nx * (y + self.ny * z)) as u32
    }

    #[inline]
    pub fn coords(&self, idx: u32) -> (i32, i32, i32) {
        let i = idx as i32;
        (
            i % self.nx,
            (i / self.nx) % self.ny,
            i / (self.nx * self.ny),
        )
    }

    #[inline]
    pub fn exists_at(&self, idx: u32) -> bool {
        (idx as usize) < self.total() && self.exists[idx as usize] == 1
    }

    /// Per-axis separation, `min(|d|, n - |d|)` on a wrapped axis.
    #[inline]
    pub fn axis_delta(&self, a: i32, b: i32, axis: usize) -> f64 {
        let n = [self.nx, self.ny, self.nz][axis];
        let d = (a - b).abs();
        (if self.wrap[axis] { d.min(n - d) } else { d }) as f64
    }

    /// Fold a coordinate into range on a wrapped axis; `None` off the end of
    /// an unwrapped one.
    #[inline]
    pub fn fold(&self, v: i32, axis: usize) -> Option<i32> {
        let n = [self.nx, self.ny, self.nz][axis];
        if v >= 0 && v < n {
            Some(v)
        } else if self.wrap[axis] {
            Some((v % n + n) % n)
        } else {
            None
        }
    }

    /// The neighbour `(dx, dy, dz)` away, wrap-aware, or `None` if it falls
    /// outside the box or outside the mask.
    pub fn neighbor(&self, from: u32, dx: i32, dy: i32, dz: i32) -> Option<u32> {
        let (x, y, z) = self.coords(from);
        let nx = self.fold(x + dx, 0)?;
        let ny = self.fold(y + dy, 1)?;
        let nz = self.fold(z + dz, 2)?;
        let j = self.idx(nx, ny, nz);
        if j == from || !self.exists_at(j) {
            None
        } else {
            Some(j)
        }
    }

    /// Inverse of `build_positions`: the cell whose centre is nearest a world
    /// point. Returns `None` when the point lands outside the box (a wrapped
    /// axis never does — it folds).
    pub fn world_to_cell(&self, wx: f32, wy: f32, wz: f32) -> Option<u32> {
        let (fx, fy, fz) = if self.curved {
            let mut th = wz.atan2(wx);
            if th < 0.0 {
                th += 2.0 * PI;
            }
            let c = (self.ny - 1) as f32 / 2.0;
            let cy = LIFT + 0.5 + c;
            let rad = (wx * wx + wz * wz).sqrt();
            (
                th * self.nx as f32 / (2.0 * PI),
                wy - cy + c,
                rad - self.ring_r + c,
            )
        } else {
            (
                wx + (self.nx - 1) as f32 / 2.0,
                wy - LIFT - 0.5,
                wz + (self.nz - 1) as f32 / 2.0,
            )
        };
        let x = self.fold(fx.round() as i32, 0)?;
        let y = self.fold(fy.round() as i32, 1)?;
        let z = self.fold(fz.round() as i32, 2)?;
        Some(self.idx(x, y, z))
    }

    #[inline]
    pub fn world_of(&self, idx: u32) -> [f32; 3] {
        let i = idx as usize;
        [self.px[i], self.py[i], self.pz[i]]
    }

    /// Longest bounding-box edge, used for framing and ray-march limits.
    pub fn span(&self) -> f32 {
        let mut s: f32 = 0.0;
        for a in 0..3 {
            s = s.max(self.bmax[a] - self.bmin[a]);
        }
        s + 1.0
    }
}
