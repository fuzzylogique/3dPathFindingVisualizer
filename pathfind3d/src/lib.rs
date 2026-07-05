//! Volumetric A* core for the 3D pathfinding visualizer.
//!
//! Index convention everywhere: `idx = x + n*(y + n*z)`.
//! The JS fallback solver in `web/index.html` mirrors this file exactly
//! (same constructor arguments, same state constants, same costs).

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use wasm_bindgen::prelude::*;

// Cell states surfaced through `state()`. Must match the JS mirror.
pub const FREE: u8 = 0;
pub const OBST: u8 = 1;
pub const OPEN: u8 = 2;
pub const CLOSED: u8 = 3;
pub const PATH: u8 = 4;

const SQRT_2: f64 = std::f64::consts::SQRT_2;
const SQRT_3: f64 = 1.732_050_807_568_877_2;
const NO_PARENT: u32 = u32::MAX;
// Guard against float churn re-pushing a node whose g only "improved" by rounding noise.
const EPS: f64 = 1e-12;

/// Heap entry. `BinaryHeap` has no decrease-key, so improved nodes are pushed
/// again as duplicates; stale entries are recognised on pop because the cell
/// is already CLOSED.
#[derive(Copy, Clone)]
struct Node {
    f: f64,
    h: f64,
    idx: u32,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Node {}

impl Ord for Node {
    // f64 isn't Ord; f and h are never NaN here. Comparison is reversed on
    // (f, then h) so the lowest-f node counts as "greatest" and pops first
    // out of the max-heap; final idx tiebreak keeps the order total.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.h.partial_cmp(&self.h).unwrap_or(Ordering::Equal))
            .then_with(|| self.idx.cmp(&other.idx))
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[wasm_bindgen]
pub struct Solver {
    n: i32,
    cells: Vec<u8>,
    g: Vec<f64>,
    parent: Vec<u32>,
    heap: BinaryHeap<Node>,
    start: u32,
    goal: u32,
    diagonals: bool,
    expanded: u32,
    frontier: u32,
    done: bool,
    found: bool,
    cost: f64,
    path: Vec<u32>,
}

#[wasm_bindgen]
impl Solver {
    /// `blocked` must hold exactly `n^3` bytes (non-zero = obstacle),
    /// indexed as `x + n*(y + n*z)`.
    #[wasm_bindgen(constructor)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        n: u32,
        blocked: &[u8],
        sx: u32,
        sy: u32,
        sz: u32,
        gx: u32,
        gy: u32,
        gz: u32,
        diagonals: bool,
    ) -> Solver {
        assert!(n >= 2, "grid side must be >= 2");
        assert!(sx < n && sy < n && sz < n, "start out of bounds");
        assert!(gx < n && gy < n && gz < n, "goal out of bounds");
        let total = (n * n * n) as usize;
        assert_eq!(blocked.len(), total, "blocked must have n^3 entries");

        let start = sx + n * (sy + n * sz);
        let goal = gx + n * (gy + n * gz);

        let mut cells = vec![FREE; total];
        for (i, &b) in blocked.iter().enumerate() {
            if b != 0 {
                cells[i] = OBST;
            }
        }
        // The endpoints themselves must never be obstacles, whatever the
        // generator produced.
        cells[start as usize] = FREE;
        cells[goal as usize] = FREE;

        let mut s = Solver {
            n: n as i32,
            cells,
            g: vec![f64::INFINITY; total],
            parent: vec![NO_PARENT; total],
            heap: BinaryHeap::new(),
            start,
            goal,
            diagonals,
            expanded: 0,
            frontier: 0,
            done: false,
            found: false,
            cost: f64::NAN,
            path: Vec::new(),
        };
        s.g[start as usize] = 0.0;
        let h = s.heuristic(start);
        s.cells[start as usize] = OPEN;
        s.frontier = 1;
        s.heap.push(Node { f: h, h, idx: start });
        s
    }

    /// Advance the search by at most `steps` node expansions.
    pub fn step_many(&mut self, steps: u32) {
        for _ in 0..steps {
            if self.done {
                break;
            }
            self.step_once();
        }
    }

    /// Per-cell state snapshot (copy): FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4.
    pub fn state(&self) -> Vec<u8> {
        self.cells.clone()
    }

    /// Zero-copy alternative to `state()`: pointer into wasm linear memory.
    /// View it with `new Uint8Array(memory.buffer, ptr, state_len())`;
    /// rebuild the view after any call that may allocate (memory can grow).
    pub fn state_ptr(&self) -> *const u8 {
        self.cells.as_ptr()
    }

    pub fn state_len(&self) -> u32 {
        self.cells.len() as u32
    }

    /// Cell indices from start to goal; empty until a path is found.
    pub fn path(&self) -> Vec<u32> {
        self.path.clone()
    }

    pub fn expanded(&self) -> u32 {
        self.expanded
    }

    pub fn frontier(&self) -> u32 {
        self.frontier
    }

    pub fn is_done(&self) -> bool {
        self.done
    }

    pub fn found(&self) -> bool {
        self.found
    }

    /// Total path cost; NaN until a path is found.
    pub fn cost(&self) -> f64 {
        self.cost
    }
}

impl Solver {
    fn coords(&self, idx: u32) -> (i32, i32, i32) {
        let n = self.n;
        let i = idx as i32;
        (i % n, (i / n) % n, i / (n * n))
    }

    /// Exact minimum cost across an empty grid, hence admissible (and
    /// consistent). 26-connected: sort |deltas| as a >= b >= c, then
    /// (a-b)*1 + (b-c)*sqrt2 + c*sqrt3. 6-connected: Manhattan.
    fn heuristic(&self, idx: u32) -> f64 {
        let (x, y, z) = self.coords(idx);
        let (gx, gy, gz) = self.coords(self.goal);
        let mut d = [
            (x - gx).abs() as f64,
            (y - gy).abs() as f64,
            (z - gz).abs() as f64,
        ];
        if !self.diagonals {
            return d[0] + d[1] + d[2];
        }
        d.sort_by(|a, b| b.partial_cmp(a).unwrap());
        (d[0] - d[1]) + (d[1] - d[2]) * SQRT_2 + d[2] * SQRT_3
    }

    fn step_once(&mut self) {
        while let Some(node) = self.heap.pop() {
            let ui = node.idx as usize;
            if self.cells[ui] != OPEN {
                // Stale duplicate: this cell was already closed via a
                // cheaper entry.
                continue;
            }
            self.cells[ui] = CLOSED;
            self.frontier -= 1;
            self.expanded += 1;
            if node.idx == self.goal {
                self.done = true;
                self.found = true;
                self.cost = self.g[ui];
                self.build_path();
                return;
            }
            self.expand_from(node.idx);
            return;
        }
        // Open set exhausted before reaching the goal.
        self.done = true;
        self.found = false;
    }

    fn expand_from(&mut self, idx: u32) {
        let n = self.n;
        let (x, y, z) = self.coords(idx);
        let g_here = self.g[idx as usize];
        for dz in -1i32..=1 {
            let nz = z + dz;
            if nz < 0 || nz >= n {
                continue;
            }
            for dy in -1i32..=1 {
                let ny = y + dy;
                if ny < 0 || ny >= n {
                    continue;
                }
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let k = dx.abs() + dy.abs() + dz.abs();
                    if !self.diagonals && k > 1 {
                        continue;
                    }
                    let nx = x + dx;
                    if nx < 0 || nx >= n {
                        continue;
                    }
                    let ni = (nx + n * (ny + n * nz)) as usize;
                    let st = self.cells[ni];
                    if st == OBST || st == CLOSED {
                        continue;
                    }
                    let step = match k {
                        1 => 1.0,
                        2 => SQRT_2,
                        _ => SQRT_3,
                    };
                    let ng = g_here + step;
                    if ng < self.g[ni] - EPS {
                        self.g[ni] = ng;
                        self.parent[ni] = idx;
                        let h = self.heuristic(ni as u32);
                        if st != OPEN {
                            self.cells[ni] = OPEN;
                            self.frontier += 1;
                        }
                        self.heap.push(Node {
                            f: ng + h,
                            h,
                            idx: ni as u32,
                        });
                    }
                }
            }
        }
    }

    fn build_path(&mut self) {
        let mut cur = self.goal;
        let mut rev = vec![cur];
        while cur != self.start {
            cur = self.parent[cur as usize];
            rev.push(cur);
        }
        rev.reverse();
        for &i in &rev {
            self.cells[i as usize] = PATH;
        }
        self.path = rev;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idx(n: u32, x: u32, y: u32, z: u32) -> u32 {
        x + n * (y + n * z)
    }

    fn run(mut s: Solver) -> Solver {
        for _ in 0..1_000_000 {
            if s.is_done() {
                break;
            }
            s.step_many(1024);
        }
        assert!(s.is_done(), "search did not terminate");
        s
    }

    #[test]
    fn empty_3_cube_diagonals_optimal() {
        let blocked = vec![0u8; 27];
        let s = run(Solver::new(3, &blocked, 0, 0, 0, 2, 2, 2, true));
        assert!(s.found());
        let want = 2.0 * SQRT_3;
        assert!(
            (s.cost() - want).abs() < 1e-9,
            "cost {} want {}",
            s.cost(),
            want
        );
        assert_eq!(s.path().len(), 3);
        assert_eq!(s.path()[0], idx(3, 0, 0, 0));
        assert_eq!(s.path()[2], idx(3, 2, 2, 2));
    }

    #[test]
    fn empty_3_cube_no_diagonals_manhattan() {
        let blocked = vec![0u8; 27];
        let s = run(Solver::new(3, &blocked, 0, 0, 0, 2, 2, 2, false));
        assert!(s.found());
        assert!((s.cost() - 6.0).abs() < 1e-9, "cost {} want 6", s.cost());
        assert_eq!(s.path().len(), 7);
    }

    #[test]
    fn threads_through_single_hole_in_blocked_plane() {
        let n = 5u32;
        let mut blocked = vec![0u8; 125];
        for x in 0..n {
            for y in 0..n {
                if !(x == 2 && y == 2) {
                    blocked[idx(n, x, y, 2) as usize] = 1;
                }
            }
        }
        let s = run(Solver::new(n, &blocked, 0, 0, 0, 4, 4, 4, true));
        assert!(s.found());
        let hole = idx(n, 2, 2, 2);
        assert!(
            s.path().contains(&hole),
            "path must pass through the only hole in the z=2 plane"
        );
        // The straight space diagonal threads the hole exactly, so the
        // optimum is the unobstructed octile bound.
        let want = 4.0 * SQRT_3;
        assert!(
            (s.cost() - want).abs() < 1e-9,
            "cost {} want {}",
            s.cost(),
            want
        );
    }

    #[test]
    fn walled_off_goal_reports_no_path() {
        let n = 4u32;
        let mut blocked = vec![0u8; 64];
        // Block every cell of the 2x2x2 corner block except the goal itself:
        // exactly the goal's 26-neighbourhood that lies in bounds.
        for x in 2..n {
            for y in 2..n {
                for z in 2..n {
                    if !(x == 3 && y == 3 && z == 3) {
                        blocked[idx(n, x, y, z) as usize] = 1;
                    }
                }
            }
        }
        let s = run(Solver::new(n, &blocked, 0, 0, 0, 3, 3, 3, true));
        assert!(s.is_done());
        assert!(!s.found());
        assert!(s.path().is_empty());
    }

    #[test]
    fn stepped_execution_matches_batch() {
        let n = 8u32;
        let mut blocked = vec![0u8; 512];
        let mut lcg: u64 = 42;
        for b in blocked.iter_mut() {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            if (lcg >> 33) % 100 < 25 {
                *b = 1;
            }
        }
        let mut a = Solver::new(n, &blocked, 0, 0, 0, 7, 7, 7, true);
        let mut b = Solver::new(n, &blocked, 0, 0, 0, 7, 7, 7, true);
        while !a.is_done() {
            a.step_many(1);
        }
        b.step_many(1_000_000);
        assert_eq!(a.found(), b.found());
        assert_eq!(a.expanded(), b.expanded());
        if a.found() {
            assert!((a.cost() - b.cost()).abs() < 1e-9);
            assert_eq!(a.path(), b.path());
        }
    }

    #[test]
    fn path_steps_are_valid_neighbour_moves() {
        let n = 7u32;
        let mut blocked = vec![0u8; 343];
        for x in 0..n {
            for y in 0..n {
                if !(x == 1 && y == 5) {
                    blocked[idx(n, x, y, 3) as usize] = 1;
                }
            }
        }
        let s = run(Solver::new(n, &blocked, 0, 0, 0, 6, 6, 6, true));
        assert!(s.found());
        let p = s.path();
        let mut acc = 0.0;
        for w in p.windows(2) {
            let (ax, ay, az) = s.coords(w[0]);
            let (bx, by, bz) = s.coords(w[1]);
            let k = (ax - bx).abs() + (ay - by).abs() + (az - bz).abs();
            assert!((bx - ax).abs() <= 1 && (by - ay).abs() <= 1 && (bz - az).abs() <= 1);
            assert!(k >= 1);
            acc += match k {
                1 => 1.0,
                2 => SQRT_2,
                _ => SQRT_3,
            };
        }
        assert!((acc - s.cost()).abs() < 1e-9, "path cost must sum to cost()");
    }
}
