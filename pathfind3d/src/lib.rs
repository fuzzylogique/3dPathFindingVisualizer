//! Volumetric pathfinding core for the 3D visualizer — edition 2.
//!
//! Index convention everywhere: `idx = x + nx*(y + ny*z)`.
//! The JS fallback solver in `web/index.html` mirrors this file exactly
//! (same constructor arguments, same state constants, same costs, same
//! expansion order), so keep the two in lock-step.
//!
//! One stepping loop drives every algorithm. An algorithm is a frontier
//! structure (priority queue / FIFO queue / LIFO stack) plus a priority
//! `f = g_w * g + h_w * h`:
//!
//! | algo                | frontier | g_w | h_w | optimal?      |
//! |---------------------|----------|-----|-----|---------------|
//! | A*                  | PQ       | 1   | 1   | yes           |
//! | Dijkstra            | PQ       | 1   | 0   | yes           |
//! | Greedy Best-First   | PQ       | 0   | 1   | no            |
//! | Swarm               | PQ       | 1   | 1.5 | no            |
//! | Convergent Swarm    | PQ       | 1   | 3   | no            |
//! | Bidirectional Swarm | 2×PQ     | 1   | 1.5 | no            |
//! | Breadth-First       | FIFO     | (hops)  | fewest hops   |
//! | Depth-First         | LIFO     | (hops)  | no            |
//!
//! Cells carry a weight `w ∈ [1, W_MAX]`; entering a cell costs
//! `base_step_cost(dir) * w` where base cost is Euclidean (1, √2, √3).
//! Weights never drop below 1, so the Euclidean/octile heuristic never
//! overestimates and A*/Dijkstra stay optimal. BFS/DFS ignore weights and
//! count hops; their reported `cost()` is still the true weighted cost of
//! the path they return (re-summed), so comparisons stay honest.
//!
//! Domains generalize the cube: per-axis dims, a per-cell existence mask
//! (terrain value 0 = void), and per-axis toroidal wrap. On a wrapped axis
//! the heuristic uses `min(|d|, n - |d|)` per axis — the raw delta would
//! overestimate and break admissibility.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::VecDeque;

use wasm_bindgen::prelude::*;

// Cell states surfaced through `state()`. Must match the JS mirror.
pub const FREE: u8 = 0;
pub const OBST: u8 = 1;
pub const OPEN: u8 = 2;
pub const CLOSED: u8 = 3;
pub const PATH: u8 = 4;

// Terrain encoding (constructor input). Must match the JS mirror.
pub const T_VOID: u8 = 0; // not part of the domain: untraversable, unrendered
pub const T_OBST: u8 = 255; // hard-blocked obstacle (infinite weight)
pub const W_MAX: u8 = 10; // valid weights are 1..=W_MAX

// Algorithm ids. Must match the JS mirror and the UI dropdown.
pub const ALGO_ASTAR: u8 = 0;
pub const ALGO_DIJKSTRA: u8 = 1;
pub const ALGO_GREEDY: u8 = 2;
pub const ALGO_SWARM: u8 = 3;
pub const ALGO_CONVERGENT: u8 = 4;
pub const ALGO_BISWARM: u8 = 5;
pub const ALGO_BFS: u8 = 6;
pub const ALGO_DFS: u8 = 7;

// Heuristic weights that make the swarm variants visibly distinct.
const SWARM_H: f64 = 1.5;
const CONVERGENT_H: f64 = 3.0;

const SQRT_2: f64 = std::f64::consts::SQRT_2;
const SQRT_3: f64 = 1.732_050_807_568_877_2;
const STEP_COST: [f64; 4] = [0.0, 1.0, SQRT_2, SQRT_3]; // indexed by axes touched
const NO_PARENT: u32 = u32::MAX;
// Guard against float churn re-pushing a node whose g only "improved" by rounding noise.
const EPS: f64 = 1e-12;

// Frontier structures.
const FR_PQ: u8 = 0;
const FR_FIFO: u8 = 1;
const FR_LIFO: u8 = 2;

// Per-search cell status (separate from the shared viz buffer so the two
// bidirectional searches never confuse each other's bookkeeping).
const UNSEEN: u8 = 0;
const S_OPEN: u8 = 1;
const S_CLOSED: u8 = 2;

/// Heap entry. `BinaryHeap` has no decrease-key, so improved nodes are pushed
/// again as duplicates; stale entries are recognised on pop because the cell
/// is no longer open in that search.
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

/// FIFO/LIFO frontier entry. The LIFO stack allows duplicates, so the entry
/// carries the (parent, g) captured at push time; they are committed to the
/// arrays when the entry is actually closed.
#[derive(Copy, Clone)]
struct QEntry {
    idx: u32,
    parent: u32,
    g: f64,
}

/// One directional search: its own frontier, scores, parents and status.
/// Single-frontier algorithms use exactly one; Bidirectional Swarm uses two
/// (index 0 from the start, index 1 from the goal) that alternate expansions.
struct Search {
    heap: BinaryHeap<Node>,
    fifo: VecDeque<QEntry>,
    lifo: Vec<QEntry>,
    g: Vec<f64>,
    parent: Vec<u32>,
    stat: Vec<u8>,
    target: u32, // heuristic target (goal for the forward search, start for the reverse)
}

impl Search {
    fn new(total: usize, target: u32) -> Search {
        Search {
            heap: BinaryHeap::new(),
            fifo: VecDeque::new(),
            lifo: Vec::new(),
            g: vec![f64::INFINITY; total],
            parent: vec![NO_PARENT; total],
            stat: vec![UNSEEN; total],
            target,
        }
    }
}

#[wasm_bindgen]
pub struct Solver {
    nx: i32,
    ny: i32,
    nz: i32,
    wrap: [bool; 3],
    terrain: Vec<u8>,
    cells: Vec<u8>, // shared viz buffer: FREE/OBST/OPEN/CLOSED/PATH
    searches: Vec<Search>,
    turn: usize,
    diagonals: bool,
    algo: u8,
    frontier_kind: u8,
    g_w: f64,
    h_w: f64,
    start: u32,
    goal: u32,
    expanded: u32,
    frontier: u32,
    done: bool,
    found: bool,
    cost: f64,
    path: Vec<u32>,
}

#[wasm_bindgen]
impl Solver {
    /// `terrain` must hold exactly `nx*ny*nz` bytes, indexed `x + nx*(y + ny*z)`:
    /// `0` = void (outside the domain), `1..=W_MAX` = existing cell with that
    /// weight, `255` = obstacle. Other values are clamped into `1..=W_MAX`.
    /// Obstacles at the endpoints are carved free (weight 1); an endpoint on a
    /// void cell makes the search report "no path" immediately.
    #[wasm_bindgen(constructor)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        nx: u32,
        ny: u32,
        nz: u32,
        terrain: &[u8],
        wrap_x: bool,
        wrap_y: bool,
        wrap_z: bool,
        sx: u32,
        sy: u32,
        sz: u32,
        gx: u32,
        gy: u32,
        gz: u32,
        diagonals: bool,
        algo: u8,
    ) -> Solver {
        assert!(nx >= 1 && ny >= 1 && nz >= 1, "dims must be >= 1");
        assert!(sx < nx && sy < ny && sz < nz, "start out of bounds");
        assert!(gx < nx && gy < ny && gz < nz, "goal out of bounds");
        assert!(algo <= ALGO_DFS, "unknown algorithm id");
        let total = (nx as usize) * (ny as usize) * (nz as usize);
        assert_eq!(terrain.len(), total, "terrain must have nx*ny*nz entries");

        let start = sx + nx * (sy + ny * sz);
        let goal = gx + nx * (gy + ny * gz);

        // Sanitise terrain: keep void/obstacle markers, clamp weights into range.
        let mut terrain: Vec<u8> = terrain
            .iter()
            .map(|&t| {
                if t == T_VOID || t == T_OBST {
                    t
                } else if t > W_MAX {
                    W_MAX
                } else {
                    t
                }
            })
            .collect();
        // The endpoints themselves must never be obstacles, whatever the
        // generator produced. Void endpoints stay void: that is "no path".
        if terrain[start as usize] == T_OBST {
            terrain[start as usize] = 1;
        }
        if terrain[goal as usize] == T_OBST {
            terrain[goal as usize] = 1;
        }

        let mut cells = vec![FREE; total];
        for (i, &t) in terrain.iter().enumerate() {
            if t == T_OBST {
                cells[i] = OBST;
            }
        }

        let (frontier_kind, g_w, h_w) = match algo {
            ALGO_ASTAR => (FR_PQ, 1.0, 1.0),
            ALGO_DIJKSTRA => (FR_PQ, 1.0, 0.0),
            ALGO_GREEDY => (FR_PQ, 0.0, 1.0),
            ALGO_SWARM => (FR_PQ, 1.0, SWARM_H),
            ALGO_CONVERGENT => (FR_PQ, 1.0, CONVERGENT_H),
            ALGO_BISWARM => (FR_PQ, 1.0, SWARM_H),
            ALGO_BFS => (FR_FIFO, 1.0, 0.0),
            _ => (FR_LIFO, 1.0, 0.0),
        };

        let mut searches = vec![Search::new(total, goal)];
        if algo == ALGO_BISWARM {
            searches.push(Search::new(total, start));
        }

        let mut s = Solver {
            nx: nx as i32,
            ny: ny as i32,
            nz: nz as i32,
            wrap: [wrap_x, wrap_y, wrap_z],
            terrain,
            cells,
            searches,
            turn: 0,
            diagonals,
            algo,
            frontier_kind,
            g_w,
            h_w,
            start,
            goal,
            expanded: 0,
            frontier: 0,
            done: false,
            found: false,
            cost: f64::NAN,
            path: Vec::new(),
        };

        // Degenerate cases resolved at construction.
        if s.terrain[start as usize] == T_VOID || s.terrain[goal as usize] == T_VOID {
            s.done = true;
            s.found = false;
            return s;
        }
        if start == goal {
            s.done = true;
            s.found = true;
            s.cost = 0.0;
            s.path = vec![start];
            s.cells[start as usize] = PATH;
            return s;
        }

        let origins: Vec<u32> = if algo == ALGO_BISWARM {
            vec![start, goal]
        } else {
            vec![start]
        };
        for (si, &origin) in origins.iter().enumerate() {
            let h = s.heuristic(origin, s.searches[si].target);
            let (g_w, h_w, kind) = (s.g_w, s.h_w, s.frontier_kind);
            let sr = &mut s.searches[si];
            sr.g[origin as usize] = 0.0;
            sr.stat[origin as usize] = S_OPEN;
            match kind {
                FR_PQ => sr.heap.push(Node {
                    f: g_w * 0.0 + h_w * h,
                    h,
                    idx: origin,
                }),
                FR_FIFO => sr.fifo.push_back(QEntry {
                    idx: origin,
                    parent: NO_PARENT,
                    g: 0.0,
                }),
                _ => sr.lifo.push(QEntry {
                    idx: origin,
                    parent: NO_PARENT,
                    g: 0.0,
                }),
            }
            s.frontier += 1;
            if s.cells[origin as usize] == FREE {
                s.cells[origin as usize] = OPEN;
            }
        }
        s
    }

    /// Advance the search by at most `steps` node expansions (closes).
    pub fn step_many(&mut self, steps: u32) {
        for _ in 0..steps {
            if self.done {
                break;
            }
            self.step_once();
        }
    }

    /// Per-cell state snapshot (copy): FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4.
    /// Void cells always read FREE; they never enter any frontier or path.
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

    /// Total node closures so far. For Bidirectional Swarm both directions
    /// count, and the meeting cell is closed by both searches, so a
    /// successful run reports one more than the number of distinct cells.
    pub fn expanded(&self) -> u32 {
        self.expanded
    }

    /// Current open-set size (sum of both fronts for Bidirectional Swarm).
    pub fn frontier(&self) -> u32 {
        self.frontier
    }

    pub fn is_done(&self) -> bool {
        self.done
    }

    pub fn found(&self) -> bool {
        self.found
    }

    /// True weighted cost of the returned path (re-summed along the path, so
    /// BFS/DFS report honest weighted costs too); NaN until a path is found.
    pub fn cost(&self) -> f64 {
        self.cost
    }

    pub fn algo(&self) -> u8 {
        self.algo
    }

    pub fn start(&self) -> u32 {
        self.start
    }

    pub fn goal(&self) -> u32 {
        self.goal
    }
}

impl Solver {
    fn coords(&self, idx: u32) -> (i32, i32, i32) {
        let i = idx as i32;
        (i % self.nx, (i / self.nx) % self.ny, i / (self.nx * self.ny))
    }

    fn axis_delta(&self, a: i32, b: i32, n: i32, wrapped: bool) -> f64 {
        let d = (a - b).abs();
        let d = if wrapped { d.min(n - d) } else { d };
        d as f64
    }

    /// Exact minimum base cost across an empty grid, hence admissible (and
    /// consistent) since every weight is >= 1. On wrapped axes the per-axis
    /// delta is min(|d|, n - |d|) — the raw delta would overestimate.
    /// 26-connected: sort |deltas| as a >= b >= c, then
    /// (a-b)*1 + (b-c)*sqrt2 + c*sqrt3. 6-connected: Manhattan.
    fn heuristic(&self, idx: u32, target: u32) -> f64 {
        let (x, y, z) = self.coords(idx);
        let (tx, ty, tz) = self.coords(target);
        let mut d = [
            self.axis_delta(x, tx, self.nx, self.wrap[0]),
            self.axis_delta(y, ty, self.ny, self.wrap[1]),
            self.axis_delta(z, tz, self.nz, self.wrap[2]),
        ];
        if !self.diagonals {
            return d[0] + d[1] + d[2];
        }
        d.sort_by(|a, b| b.partial_cmp(a).unwrap());
        (d[0] - d[1]) + (d[1] - d[2]) * SQRT_2 + d[2] * SQRT_3
    }

    /// Pop the next closable entry from search `si`, skipping stale duplicates.
    /// Returns (idx, parent, g) — parent/g are only meaningful for the LIFO
    /// frontier, whose entries commit their bookkeeping at close time.
    fn pop_valid(&mut self, si: usize) -> Option<QEntry> {
        let sr = &mut self.searches[si];
        match self.frontier_kind {
            FR_PQ => loop {
                let node = sr.heap.pop()?;
                if sr.stat[node.idx as usize] == S_OPEN {
                    return Some(QEntry {
                        idx: node.idx,
                        parent: sr.parent[node.idx as usize],
                        g: sr.g[node.idx as usize],
                    });
                }
            },
            FR_FIFO => sr.fifo.pop_front(),
            _ => loop {
                let e = sr.lifo.pop()?;
                if sr.stat[e.idx as usize] != S_CLOSED {
                    return Some(e);
                }
            },
        }
    }

    fn step_once(&mut self) {
        // Pick the next search with work left, starting from whose turn it is.
        let ns = self.searches.len();
        let mut si = self.turn % ns;
        let mut entry = None;
        for _ in 0..ns {
            entry = self.pop_valid(si);
            if entry.is_some() {
                break;
            }
            si = (si + 1) % ns;
        }
        let Some(e) = entry else {
            // Every frontier exhausted before the searches met / reached the goal.
            self.done = true;
            self.found = false;
            return;
        };

        let ui = e.idx as usize;
        {
            let sr = &mut self.searches[si];
            if self.frontier_kind == FR_LIFO {
                // Commit the branch this entry actually came from.
                sr.parent[ui] = e.parent;
                sr.g[ui] = e.g;
            }
            sr.stat[ui] = S_CLOSED;
        }
        self.cells[ui] = CLOSED;
        self.frontier -= 1;
        self.expanded += 1;
        self.turn = (si + 1) % ns;

        if ns == 2 {
            // Bidirectional termination: the closed sets first intersect.
            if self.searches[1 - si].stat[ui] == S_CLOSED {
                self.finish_bidirectional(e.idx);
                return;
            }
        } else if e.idx == self.goal {
            self.done = true;
            self.found = true;
            self.path = self.walk_parents(0, self.goal);
            self.finish_path();
            return;
        }

        self.expand_from(si, e.idx);
    }

    fn expand_from(&mut self, si: usize, idx: u32) {
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        let (x, y, z) = self.coords(idx);
        let g_here = self.searches[si].g[idx as usize];
        let target = self.searches[si].target;
        for dz in -1i32..=1 {
            let mut z2 = z + dz;
            if z2 < 0 || z2 >= nz {
                if !self.wrap[2] {
                    continue;
                }
                z2 = (z2 + nz) % nz;
            }
            for dy in -1i32..=1 {
                let mut y2 = y + dy;
                if y2 < 0 || y2 >= ny {
                    if !self.wrap[1] {
                        continue;
                    }
                    y2 = (y2 + ny) % ny;
                }
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let k = (dx.abs() + dy.abs() + dz.abs()) as usize;
                    if !self.diagonals && k > 1 {
                        continue;
                    }
                    let mut x2 = x + dx;
                    if x2 < 0 || x2 >= nx {
                        if !self.wrap[0] {
                            continue;
                        }
                        x2 = (x2 + nx) % nx;
                    }
                    let ni = (x2 + nx * (y2 + ny * z2)) as u32;
                    if ni == idx {
                        continue; // wrap on a 1-cell axis folds onto itself
                    }
                    let t = self.terrain[ni as usize];
                    if t == T_VOID || t == T_OBST {
                        continue;
                    }
                    let sr = &mut self.searches[si];
                    let nu = ni as usize;
                    if sr.stat[nu] == S_CLOSED {
                        continue;
                    }
                    match self.frontier_kind {
                        FR_PQ => {
                            let ng = g_here + STEP_COST[k] * t as f64;
                            if ng < sr.g[nu] - EPS {
                                sr.g[nu] = ng;
                                sr.parent[nu] = idx;
                                if sr.stat[nu] == UNSEEN {
                                    sr.stat[nu] = S_OPEN;
                                    self.frontier += 1;
                                    if self.cells[nu] == FREE {
                                        self.cells[nu] = OPEN;
                                    }
                                }
                                let h = self.heuristic(ni, target);
                                let f = self.g_w * ng + self.h_w * h;
                                self.searches[si].heap.push(Node { f, h, idx: ni });
                            }
                        }
                        FR_FIFO => {
                            // Unweighted hop search: first discovery is the
                            // fewest-hops route, never re-pushed.
                            if sr.stat[nu] == UNSEEN {
                                sr.stat[nu] = S_OPEN;
                                sr.g[nu] = g_here + 1.0;
                                sr.parent[nu] = idx;
                                sr.fifo.push_back(QEntry {
                                    idx: ni,
                                    parent: idx,
                                    g: g_here + 1.0,
                                });
                                self.frontier += 1;
                                if self.cells[nu] == FREE {
                                    self.cells[nu] = OPEN;
                                }
                            }
                        }
                        _ => {
                            // Depth-first: duplicates allowed so the newest
                            // branch is always explored first; each entry
                            // remembers the branch it came from.
                            if sr.stat[nu] == UNSEEN {
                                sr.stat[nu] = S_OPEN;
                                self.frontier += 1;
                                if self.cells[nu] == FREE {
                                    self.cells[nu] = OPEN;
                                }
                            }
                            sr.lifo.push(QEntry {
                                idx: ni,
                                parent: idx,
                                g: g_here + 1.0,
                            });
                        }
                    }
                }
            }
        }
    }

    /// Walk a search's parent chain from `from` back to its origin, returning
    /// the cells ordered origin -> `from`.
    fn walk_parents(&self, si: usize, from: u32) -> Vec<u32> {
        let mut rev = vec![from];
        let mut cur = from;
        while self.searches[si].parent[cur as usize] != NO_PARENT {
            cur = self.searches[si].parent[cur as usize];
            rev.push(cur);
        }
        rev.reverse();
        rev
    }

    /// Stitch the two half-paths at the meeting cell. The forward half runs
    /// start -> meet; the reverse search's parent chain runs meet -> goal.
    fn finish_bidirectional(&mut self, meet: u32) {
        self.done = true;
        self.found = true;
        let mut path = self.walk_parents(0, meet); // start .. meet
        let mut cur = self.searches[1].parent[meet as usize];
        while cur != NO_PARENT {
            path.push(cur);
            cur = self.searches[1].parent[cur as usize];
        }
        self.path = path;
        self.finish_path();
    }

    /// True weighted cost of a path: per step, the Euclidean base cost of the
    /// (wrap-aware) move times the weight of the cell being entered.
    fn path_cost(&self, path: &[u32]) -> f64 {
        let mut acc = 0.0;
        for w in path.windows(2) {
            let (ax, ay, az) = self.coords(w[0]);
            let (bx, by, bz) = self.coords(w[1]);
            let k = (self.axis_delta(ax, bx, self.nx, self.wrap[0])
                + self.axis_delta(ay, by, self.ny, self.wrap[1])
                + self.axis_delta(az, bz, self.nz, self.wrap[2])) as usize;
            acc += STEP_COST[k] * self.terrain[w[1] as usize] as f64;
        }
        acc
    }

    fn finish_path(&mut self) {
        self.cost = self.path_cost(&self.path);
        for &i in &self.path {
            self.cells[i as usize] = PATH;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- helpers -------------------------------------------------

    fn idx(nx: u32, ny: u32, x: u32, y: u32, z: u32) -> u32 {
        x + nx * (y + ny * z)
    }

    /// Uniform-weight terrain from an edition-1 style `blocked` array.
    fn terrain_from_blocked(blocked: &[u8]) -> Vec<u8> {
        blocked.iter().map(|&b| if b != 0 { T_OBST } else { 1 }).collect()
    }

    fn cube(
        n: u32,
        blocked: &[u8],
        s: (u32, u32, u32),
        g: (u32, u32, u32),
        diagonals: bool,
        algo: u8,
    ) -> Solver {
        Solver::new(
            n, n, n,
            &terrain_from_blocked(blocked),
            false, false, false,
            s.0, s.1, s.2,
            g.0, g.1, g.2,
            diagonals, algo,
        )
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

    fn lcg_terrain(total: usize, seed: u64, obst_pct: u64, weighted: bool) -> Vec<u8> {
        let mut lcg = seed;
        let mut next = || {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            lcg >> 33
        };
        (0..total)
            .map(|_| {
                let r = next();
                if r % 100 < obst_pct {
                    T_OBST
                } else if weighted {
                    1 + (next() % W_MAX as u64) as u8
                } else {
                    1
                }
            })
            .collect()
    }

    /// Path validity: correct endpoints, every cell existing and unblocked,
    /// every step a legal (wrap-aware) neighbour move, and the solver's cost
    /// equal to the re-summed weighted cost.
    fn assert_valid_path(s: &Solver, path: &[u32]) {
        assert!(!path.is_empty());
        assert_eq!(path[0], s.start, "path must start at start");
        assert_eq!(*path.last().unwrap(), s.goal, "path must end at goal");
        for &i in path {
            let t = s.terrain[i as usize];
            assert!(t != T_VOID && t != T_OBST, "path crosses void/obstacle");
        }
        for w in path.windows(2) {
            let (ax, ay, az) = s.coords(w[0]);
            let (bx, by, bz) = s.coords(w[1]);
            let dx = s.axis_delta(ax, bx, s.nx, s.wrap[0]);
            let dy = s.axis_delta(ay, by, s.ny, s.wrap[1]);
            let dz = s.axis_delta(az, bz, s.nz, s.wrap[2]);
            assert!(dx <= 1.0 && dy <= 1.0 && dz <= 1.0, "illegal step");
            let k = dx + dy + dz;
            assert!(k >= 1.0, "path repeats a cell");
            if !s.diagonals {
                assert!(k <= 1.0, "diagonal step while diagonals are off");
            }
        }
        assert!(
            (s.path_cost(path) - s.cost()).abs() < 1e-9,
            "cost() must equal the re-summed weighted path cost"
        );
    }

    fn optimal_cost(
        dims: (u32, u32, u32),
        terrain: &[u8],
        wrap: (bool, bool, bool),
        s: (u32, u32, u32),
        g: (u32, u32, u32),
        diagonals: bool,
    ) -> Option<f64> {
        let a = run(Solver::new(
            dims.0, dims.1, dims.2, terrain, wrap.0, wrap.1, wrap.2,
            s.0, s.1, s.2, g.0, g.1, g.2, diagonals, ALGO_DIJKSTRA,
        ));
        if a.found() { Some(a.cost()) } else { None }
    }

    // ---------- edition-1 invariants (updated constructor) --------------

    #[test]
    fn empty_3_cube_diagonals_optimal() {
        let blocked = vec![0u8; 27];
        let s = run(cube(3, &blocked, (0, 0, 0), (2, 2, 2), true, ALGO_ASTAR));
        assert!(s.found());
        let want = 2.0 * SQRT_3;
        assert!((s.cost() - want).abs() < 1e-9, "cost {} want {}", s.cost(), want);
        assert_eq!(s.path().len(), 3);
        assert_eq!(s.path()[0], idx(3, 3, 0, 0, 0));
        assert_eq!(s.path()[2], idx(3, 3, 2, 2, 2));
    }

    #[test]
    fn empty_3_cube_no_diagonals_manhattan() {
        let blocked = vec![0u8; 27];
        let s = run(cube(3, &blocked, (0, 0, 0), (2, 2, 2), false, ALGO_ASTAR));
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
                    blocked[idx(n, n, x, y, 2) as usize] = 1;
                }
            }
        }
        let s = run(cube(n, &blocked, (0, 0, 0), (4, 4, 4), true, ALGO_ASTAR));
        assert!(s.found());
        let hole = idx(n, n, 2, 2, 2);
        assert!(
            s.path().contains(&hole),
            "path must pass through the only hole in the z=2 plane"
        );
        let want = 4.0 * SQRT_3;
        assert!((s.cost() - want).abs() < 1e-9, "cost {} want {}", s.cost(), want);
    }

    #[test]
    fn walled_off_goal_reports_no_path() {
        let n = 4u32;
        let mut blocked = vec![0u8; 64];
        for x in 2..n {
            for y in 2..n {
                for z in 2..n {
                    if !(x == 3 && y == 3 && z == 3) {
                        blocked[idx(n, n, x, y, z) as usize] = 1;
                    }
                }
            }
        }
        let s = run(cube(n, &blocked, (0, 0, 0), (3, 3, 3), true, ALGO_ASTAR));
        assert!(s.is_done());
        assert!(!s.found());
        assert!(s.path().is_empty());
    }

    #[test]
    fn stepped_execution_matches_batch_for_every_algorithm() {
        let n = 8u32;
        let terrain = lcg_terrain(512, 42, 25, true);
        for algo in [
            ALGO_ASTAR, ALGO_DIJKSTRA, ALGO_GREEDY, ALGO_SWARM,
            ALGO_CONVERGENT, ALGO_BISWARM, ALGO_BFS, ALGO_DFS,
        ] {
            let mk = || Solver::new(
                n, n, n, &terrain, false, false, false,
                0, 0, 0, 7, 7, 7, true, algo,
            );
            let mut a = mk();
            let mut b = mk();
            while !a.is_done() {
                a.step_many(1);
            }
            b.step_many(1_000_000);
            assert_eq!(a.found(), b.found(), "algo {algo}: found mismatch");
            assert_eq!(a.expanded(), b.expanded(), "algo {algo}: expanded mismatch");
            if a.found() {
                assert!((a.cost() - b.cost()).abs() < 1e-9, "algo {algo}: cost mismatch");
                assert_eq!(a.path(), b.path(), "algo {algo}: path mismatch");
            }
        }
    }

    #[test]
    fn path_steps_are_valid_neighbour_moves() {
        let n = 7u32;
        let mut blocked = vec![0u8; 343];
        for x in 0..n {
            for y in 0..n {
                if !(x == 1 && y == 5) {
                    blocked[idx(n, n, x, y, 3) as usize] = 1;
                }
            }
        }
        let s = run(cube(n, &blocked, (0, 0, 0), (6, 6, 6), true, ALGO_ASTAR));
        assert!(s.found());
        assert_valid_path(&s, &s.path());
    }

    // ---------- edition-2: weighted cells --------------------------------

    #[test]
    fn dijkstra_equals_astar_on_weighted_random_grids() {
        let n = 8u32;
        for seed in [7u64, 99, 1234, 555, 2026] {
            let terrain = lcg_terrain(512, seed, 20, true);
            let a = run(Solver::new(
                n, n, n, &terrain, false, false, false,
                0, 0, 0, 7, 7, 7, true, ALGO_ASTAR,
            ));
            let d = run(Solver::new(
                n, n, n, &terrain, false, false, false,
                0, 0, 0, 7, 7, 7, true, ALGO_DIJKSTRA,
            ));
            assert_eq!(a.found(), d.found(), "seed {seed}: found mismatch");
            if a.found() {
                assert!(
                    (a.cost() - d.cost()).abs() < 1e-9,
                    "seed {seed}: A* {} != Dijkstra {}",
                    a.cost(),
                    d.cost()
                );
                assert_valid_path(&a, &a.path());
                assert_valid_path(&d, &d.path());
            }
        }
    }

    #[test]
    fn heavy_cell_forces_detour_light_cell_does_not() {
        // 5x3x3 corridor, start (0,1,1) -> goal (4,1,1), no diagonals.
        // The straight line passes through (2,1,1); a detour via y=0 costs 6.
        // Straight-through cost is 3 + W (entering 4 cells, one of weight W).
        let (nx, ny, nz) = (5u32, 3u32, 3u32);
        let total = (nx * ny * nz) as usize;
        let mid = idx(nx, ny, 2, 1, 1);
        for (w, want, through) in [(10u8, 6.0, false), (2u8, 5.0, true)] {
            let mut terrain = vec![1u8; total];
            terrain[mid as usize] = w;
            let s = run(Solver::new(
                nx, ny, nz, &terrain, false, false, false,
                0, 1, 1, 4, 1, 1, false, ALGO_ASTAR,
            ));
            assert!(s.found());
            assert!(
                (s.cost() - want).abs() < 1e-9,
                "weight {w}: cost {} want {want}",
                s.cost()
            );
            assert_eq!(
                s.path().contains(&mid),
                through,
                "weight {w}: through-mid should be {through}"
            );
            assert_valid_path(&s, &s.path());
            // Dijkstra agrees.
            let d = run(Solver::new(
                nx, ny, nz, &terrain, false, false, false,
                0, 1, 1, 4, 1, 1, false, ALGO_DIJKSTRA,
            ));
            assert!((d.cost() - want).abs() < 1e-9);
        }
    }

    // ---------- edition-2: BFS/DFS hop semantics --------------------------

    #[test]
    fn bfs_returns_fewest_hops_not_least_cost() {
        // 3x3x1, diagonals on, heavy centre. BFS takes the 2-hop diagonal
        // route through the centre; A* pays fewer coins on a 3-hop detour.
        let (nx, ny, nz) = (3u32, 3u32, 1u32);
        let mut terrain = vec![1u8; 9];
        terrain[idx(nx, ny, 1, 1, 0) as usize] = 10;
        let bfs = run(Solver::new(
            nx, ny, nz, &terrain, false, false, false,
            0, 0, 0, 2, 2, 0, true, ALGO_BFS,
        ));
        assert!(bfs.found());
        assert_eq!(bfs.path().len(), 3, "fewest hops is 2 (3 cells)");
        assert_valid_path(&bfs, &bfs.path());
        let astar = run(Solver::new(
            nx, ny, nz, &terrain, false, false, false,
            0, 0, 0, 2, 2, 0, true, ALGO_ASTAR,
        ));
        assert!(astar.found());
        assert!(astar.path().len() > bfs.path().len(), "A* takes more hops here");
        assert!(astar.cost() < bfs.cost(), "least cost beats fewest hops here");
    }

    #[test]
    fn bfs_fewest_hops_hand_checked() {
        // Empty 4^3, corner to corner, diagonals on: 3 hops (4 cells).
        let blocked = vec![0u8; 64];
        let s = run(cube(4, &blocked, (0, 0, 0), (3, 3, 3), true, ALGO_BFS));
        assert!(s.found());
        assert_eq!(s.path().len(), 4);
        // Diagonals off: Manhattan distance 9 hops (10 cells).
        let s = run(cube(4, &blocked, (0, 0, 0), (3, 3, 3), false, ALGO_BFS));
        assert!(s.found());
        assert_eq!(s.path().len(), 10);
    }

    // ---------- edition-2: non-optimal algorithms stay honest -------------

    #[test]
    fn suboptimal_algorithms_return_valid_paths_never_beating_optimal() {
        let n = 8u32;
        for seed in [3u64, 77, 909] {
            for diagonals in [true, false] {
                let terrain = lcg_terrain(512, seed, 18, true);
                let opt = optimal_cost(
                    (n, n, n), &terrain, (false, false, false),
                    (0, 0, 0), (7, 7, 7), diagonals,
                );
                for algo in [
                    ALGO_GREEDY, ALGO_SWARM, ALGO_CONVERGENT,
                    ALGO_BISWARM, ALGO_BFS, ALGO_DFS,
                ] {
                    let s = run(Solver::new(
                        n, n, n, &terrain, false, false, false,
                        0, 0, 0, 7, 7, 7, diagonals, algo,
                    ));
                    assert_eq!(
                        s.found(),
                        opt.is_some(),
                        "seed {seed} algo {algo}: reachability must match Dijkstra"
                    );
                    if let Some(opt) = opt {
                        assert_valid_path(&s, &s.path());
                        assert!(
                            s.cost() >= opt - 1e-9,
                            "seed {seed} algo {algo}: cost {} beats optimal {opt}",
                            s.cost()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn bidirectional_returns_valid_connected_path() {
        let n = 10u32;
        let terrain = lcg_terrain(1000, 4242, 22, true);
        let s = run(Solver::new(
            n, n, n, &terrain, false, false, false,
            0, 0, 0, 9, 9, 9, true, ALGO_BISWARM,
        ));
        assert!(s.found(), "grid should be solvable");
        assert_valid_path(&s, &s.path());
    }

    // ---------- edition-2: wraparound -------------------------------------

    #[test]
    fn wrap_x_shortcut_beats_unwrapped_route() {
        // 9x3x3 corridor, endpoints on opposite x faces. Without wrap the
        // best route costs 8; with wrap_x it is a single step.
        let (nx, ny, nz) = (9u32, 3u32, 3u32);
        let terrain = vec![1u8; (nx * ny * nz) as usize];
        let flat = run(Solver::new(
            nx, ny, nz, &terrain, false, false, false,
            0, 1, 1, 8, 1, 1, false, ALGO_ASTAR,
        ));
        let wrapped = run(Solver::new(
            nx, ny, nz, &terrain, true, false, false,
            0, 1, 1, 8, 1, 1, false, ALGO_ASTAR,
        ));
        assert!((flat.cost() - 8.0).abs() < 1e-9);
        assert!((wrapped.cost() - 1.0).abs() < 1e-9);
        assert!(wrapped.cost() < flat.cost());
        assert_valid_path(&wrapped, &wrapped.path());
    }

    #[test]
    fn astar_stays_optimal_with_wrap_on_weighted_grids() {
        // The wrap-aware heuristic must keep A* == Dijkstra.
        let (nx, ny, nz) = (9u32, 7u32, 8u32);
        let total = (nx * ny * nz) as usize;
        for seed in [11u64, 313, 9000] {
            let terrain = lcg_terrain(total, seed, 20, true);
            let a = run(Solver::new(
                nx, ny, nz, &terrain, true, true, true,
                0, 0, 0, 8, 6, 7, true, ALGO_ASTAR,
            ));
            let d = run(Solver::new(
                nx, ny, nz, &terrain, true, true, true,
                0, 0, 0, 8, 6, 7, true, ALGO_DIJKSTRA,
            ));
            assert_eq!(a.found(), d.found());
            if a.found() {
                assert!(
                    (a.cost() - d.cost()).abs() < 1e-9,
                    "seed {seed}: wrapped A* {} != Dijkstra {}",
                    a.cost(),
                    d.cost()
                );
                assert_valid_path(&a, &a.path());
            }
        }
    }

    // ---------- edition-2: domain mask -------------------------------------

    #[test]
    fn void_cells_never_searched_and_masked_goal_is_unreachable() {
        // Ellipsoid mask inside a 9^3 box; everything outside is void.
        let n = 9u32;
        let total = (n * n * n) as usize;
        let c = 4.0;
        let inside = |x: u32, y: u32, z: u32| {
            let (dx, dy, dz) = (x as f64 - c, y as f64 - c, z as f64 - c);
            (dx * dx + dy * dy + dz * dz).sqrt() <= 4.2
        };
        let mut terrain = vec![T_VOID; total];
        for z in 0..n {
            for y in 0..n {
                for x in 0..n {
                    if inside(x, y, z) {
                        terrain[idx(n, n, x, y, z) as usize] = 1 + ((x + y + z) % 3) as u8;
                    }
                }
            }
        }
        assert!(inside(4, 4, 0) && inside(4, 4, 8), "endpoints must exist");
        for algo in [ALGO_ASTAR, ALGO_BFS, ALGO_DFS, ALGO_BISWARM] {
            let s = run(Solver::new(
                n, n, n, &terrain, false, false, false,
                4, 4, 0, 4, 4, 8, true, algo,
            ));
            assert!(s.found(), "algo {algo}: path must exist inside the mask");
            let state = s.state();
            for i in 0..total {
                if terrain[i] == T_VOID {
                    assert_eq!(
                        state[i], FREE,
                        "algo {algo}: void cell {i} entered the search"
                    );
                    assert!(!s.path().contains(&(i as u32)));
                }
            }
            assert_valid_path(&s, &s.path());
        }
        // A goal placed outside the mask reports no path immediately.
        assert!(!inside(0, 0, 0));
        let s = run(Solver::new(
            n, n, n, &terrain, false, false, false,
            4, 4, 0, 0, 0, 0, true, ALGO_ASTAR,
        ));
        assert!(s.is_done());
        assert!(!s.found());
        assert!(s.path().is_empty());
    }

    // ---------- misc edge cases --------------------------------------------

    #[test]
    fn start_equals_goal_is_a_zero_cost_path() {
        let terrain = vec![1u8; 27];
        let s = Solver::new(
            3, 3, 3, &terrain, false, false, false,
            1, 1, 1, 1, 1, 1, true, ALGO_ASTAR,
        );
        assert!(s.is_done() && s.found());
        assert_eq!(s.path(), vec![idx(3, 3, 1, 1, 1)]);
        assert!(s.cost().abs() < 1e-12);
    }

    #[test]
    fn non_cubic_dims_route_correctly() {
        let (nx, ny, nz) = (6u32, 3u32, 4u32);
        let terrain = vec![1u8; (nx * ny * nz) as usize];
        let s = run(Solver::new(
            nx, ny, nz, &terrain, false, false, false,
            0, 0, 0, 5, 2, 3, true, ALGO_ASTAR,
        ));
        assert!(s.found());
        // deltas (5,2,3) sorted desc: (5-3)*1 + (3-2)*sqrt2 + 2*sqrt3
        let want = 2.0 + SQRT_2 + 2.0 * SQRT_3;
        assert!((s.cost() - want).abs() < 1e-9, "cost {} want {}", s.cost(), want);
        assert_valid_path(&s, &s.path());
    }
}
