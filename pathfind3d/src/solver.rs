//! The search engine.
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
//! Entering a cell costs `base_step_cost(dir) * weight`, where the base cost
//! is Euclidean (1, √2, √3). Weights never drop below 1, so the octile
//! heuristic never overestimates and A*/Dijkstra stay optimal. BFS/DFS ignore
//! weights and count hops; their reported `cost` is still the true weighted
//! cost of the path they return, so comparisons stay honest.
//!
//! The solver borrows its `Domain` and terrain rather than owning copies, so
//! rebuilding one to rewind the search (see `World::seek`) is cheap.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::VecDeque;

use crate::domain::Domain;
use crate::{CLOSED, FREE, OBST, OPEN, PATH, T_OBST, T_VOID};

// Algorithm ids. Must match the UI dropdown.
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

pub const SQRT_2: f64 = std::f64::consts::SQRT_2;
pub const SQRT_3: f64 = 1.732_050_807_568_877_2;
/// Base step cost indexed by how many axes the move touches.
pub const STEP_COST: [f64; 4] = [0.0, 1.0, SQRT_2, SQRT_3];

const NO_PARENT: u32 = u32::MAX;
// Guard against float churn re-pushing a node whose g only "improved" by
// rounding noise.
const EPS: f64 = 1e-12;

// Frontier structures.
const FR_PQ: u8 = 0;
const FR_FIFO: u8 = 1;
const FR_LIFO: u8 = 2;

// Per-search cell status, separate from the shared viz buffer so the two
// bidirectional searches never confuse each other's bookkeeping.
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
    // out of the max-heap; the final idx tiebreak keeps the order total.
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
    /// Heuristic target: the goal for the forward search, the start for the
    /// reverse one.
    target: u32,
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

pub struct Solver {
    /// Shared viz buffer: FREE / OBST / OPEN / CLOSED / PATH per cell.
    cells: Vec<u8>,
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

impl Solver {
    /// `start` and `goal` must be in-bounds cell indices. An endpoint on a
    /// void cell reports "no path" immediately; an endpoint on an obstacle is
    /// treated as free (the caller is expected to keep them off walls, and
    /// `World` does).
    pub fn new(
        dom: &Domain,
        terrain: &[u8],
        start: u32,
        goal: u32,
        diagonals: bool,
        algo: u8,
    ) -> Solver {
        let total = dom.total();
        assert_eq!(terrain.len(), total, "terrain must have nx*ny*nz entries");
        assert!(
            (start as usize) < total && (goal as usize) < total,
            "endpoint out of bounds"
        );
        assert!(algo <= ALGO_DFS, "unknown algorithm id");

        let mut cells = vec![FREE; total];
        for (i, &t) in terrain.iter().enumerate() {
            if t == T_OBST {
                cells[i] = OBST;
            }
        }
        cells[start as usize] = FREE;
        cells[goal as usize] = FREE;

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
        if terrain[start as usize] == T_VOID || terrain[goal as usize] == T_VOID {
            s.done = true;
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

        let origins: &[u32] = if algo == ALGO_BISWARM {
            &[start, goal]
        } else {
            &[start]
        };
        for (si, &origin) in origins.iter().enumerate() {
            let h = s.heuristic(dom, origin, s.searches[si].target);
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

    /// Advance by at most `steps` node expansions (closes).
    pub fn step_many(&mut self, dom: &Domain, terrain: &[u8], steps: u32) {
        for _ in 0..steps {
            if self.done {
                break;
            }
            self.step_once(dom, terrain);
        }
    }

    /// Run to completion. Bounded so a pathological state can never hang the
    /// caller: every cell can close at most twice (once per direction).
    pub fn run(&mut self, dom: &Domain, terrain: &[u8]) {
        let cap = (dom.total() as u32).saturating_mul(2).saturating_add(8);
        self.step_many(dom, terrain, cap);
    }

    pub fn cells(&self) -> &[u8] {
        &self.cells
    }
    pub fn path(&self) -> &[u32] {
        &self.path
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

    /// Exact minimum base cost across an empty grid, hence admissible (and
    /// consistent) since every weight is >= 1. 26-connected: sort |deltas| as
    /// a >= b >= c, then `(a-b)*1 + (b-c)*√2 + c*√3`. 6-connected: Manhattan.
    fn heuristic(&self, dom: &Domain, idx: u32, target: u32) -> f64 {
        let (x, y, z) = dom.coords(idx);
        let (tx, ty, tz) = dom.coords(target);
        let mut d = [
            dom.axis_delta(x, tx, 0),
            dom.axis_delta(y, ty, 1),
            dom.axis_delta(z, tz, 2),
        ];
        if !self.diagonals {
            return d[0] + d[1] + d[2];
        }
        d.sort_by(|a, b| b.partial_cmp(a).unwrap());
        (d[0] - d[1]) + (d[1] - d[2]) * SQRT_2 + d[2] * SQRT_3
    }

    /// Pop the next closable entry from search `si`, skipping stale
    /// duplicates. `parent`/`g` are only meaningful for the LIFO frontier,
    /// whose entries commit their bookkeeping at close time.
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

    fn step_once(&mut self, dom: &Domain, terrain: &[u8]) {
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
                self.finish_bidirectional(dom, terrain, e.idx);
                return;
            }
        } else if e.idx == self.goal {
            self.done = true;
            self.found = true;
            self.path = self.walk_parents(0, self.goal);
            self.finish_path(dom, terrain);
            return;
        }

        self.expand_from(dom, terrain, si, e.idx);
    }

    fn expand_from(&mut self, dom: &Domain, terrain: &[u8], si: usize, idx: u32) {
        let g_here = self.searches[si].g[idx as usize];
        let target = self.searches[si].target;
        for dz in -1i32..=1 {
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let k = (dx.abs() + dy.abs() + dz.abs()) as usize;
                    if !self.diagonals && k > 1 {
                        continue;
                    }
                    let Some(ni) = dom.neighbor(idx, dx, dy, dz) else {
                        continue;
                    };
                    let nu = ni as usize;
                    let t = terrain[nu];
                    if t == T_VOID || t == T_OBST {
                        continue;
                    }
                    let sr = &mut self.searches[si];
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
                                let h = self.heuristic(dom, ni, target);
                                let f = self.g_w * ng + self.h_w * h;
                                self.searches[si].heap.push(Node { f, h, idx: ni });
                            }
                        }
                        FR_FIFO => {
                            // Unweighted hop search: the first discovery is the
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
    fn finish_bidirectional(&mut self, dom: &Domain, terrain: &[u8], meet: u32) {
        self.done = true;
        self.found = true;
        let mut path = self.walk_parents(0, meet); // start .. meet
        let mut cur = self.searches[1].parent[meet as usize];
        while cur != NO_PARENT {
            path.push(cur);
            cur = self.searches[1].parent[cur as usize];
        }
        self.path = path;
        self.finish_path(dom, terrain);
    }

    fn finish_path(&mut self, dom: &Domain, terrain: &[u8]) {
        self.cost = crate::route::path_cost(dom, terrain, &self.path);
        for &i in &self.path {
            self.cells[i as usize] = PATH;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::route::{path_cost, validate_route};
    use crate::terrain::Rng;
    use crate::{T_OBST, T_VOID, W_MAX};

    // ---------- helpers -------------------------------------------------

    fn idx(nx: u32, ny: u32, x: u32, y: u32, z: u32) -> u32 {
        x + nx * (y + ny * z)
    }

    fn terrain_from_blocked(blocked: &[u8]) -> Vec<u8> {
        blocked
            .iter()
            .map(|&b| if b != 0 { T_OBST } else { 1 })
            .collect()
    }

    /// Build and run to completion on a plain lattice domain.
    fn solve(
        dims: (u32, u32, u32),
        wrap: [bool; 3],
        terrain: &[u8],
        s: (u32, u32, u32),
        g: (u32, u32, u32),
        diagonals: bool,
        algo: u8,
    ) -> (Domain, Vec<u8>, Solver) {
        let dom = Domain::raw(dims.0, dims.1, dims.2, wrap);
        let mut terrain = terrain.to_vec();
        let start = dom.idx(s.0 as i32, s.1 as i32, s.2 as i32);
        let goal = dom.idx(g.0 as i32, g.1 as i32, g.2 as i32);
        crate::terrain::sanitize(&mut terrain);
        crate::terrain::free_endpoints(&mut terrain, start, goal);
        let mut solver = Solver::new(&dom, &terrain, start, goal, diagonals, algo);
        solver.run(&dom, &terrain);
        assert!(solver.is_done(), "search did not terminate");
        (dom, terrain, solver)
    }

    fn cube(
        n: u32,
        blocked: &[u8],
        s: (u32, u32, u32),
        g: (u32, u32, u32),
        diagonals: bool,
        algo: u8,
    ) -> (Domain, Vec<u8>, Solver) {
        solve(
            (n, n, n),
            [false; 3],
            &terrain_from_blocked(blocked),
            s,
            g,
            diagonals,
            algo,
        )
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
    fn assert_valid_path(dom: &Domain, terrain: &[u8], s: &Solver) {
        let v = validate_route(dom, terrain, s.path(), s.start(), s.goal(), s.diagonals);
        assert!(v.is_none(), "path invalid: {}", v.unwrap());
        assert!(
            (path_cost(dom, terrain, s.path()) - s.cost()).abs() < 1e-9,
            "cost() must equal the re-summed weighted path cost"
        );
    }

    // ---------- edition-1 invariants --------------------------------------

    #[test]
    fn empty_3_cube_diagonals_optimal() {
        let (_, _, s) = cube(3, &[0u8; 27], (0, 0, 0), (2, 2, 2), true, ALGO_ASTAR);
        assert!(s.found());
        let want = 2.0 * SQRT_3;
        assert!((s.cost() - want).abs() < 1e-9, "cost {}", s.cost());
        assert_eq!(s.path().len(), 3);
        assert_eq!(s.path()[0], idx(3, 3, 0, 0, 0));
        assert_eq!(s.path()[2], idx(3, 3, 2, 2, 2));
    }

    #[test]
    fn empty_3_cube_no_diagonals_manhattan() {
        let (_, _, s) = cube(3, &[0u8; 27], (0, 0, 0), (2, 2, 2), false, ALGO_ASTAR);
        assert!(s.found());
        assert!((s.cost() - 6.0).abs() < 1e-9);
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
        let (d, t, s) = cube(n, &blocked, (0, 0, 0), (4, 4, 4), true, ALGO_ASTAR);
        assert!(s.found());
        assert!(
            s.path().contains(&idx(n, n, 2, 2, 2)),
            "path must pass through the only hole in the z=2 plane"
        );
        assert!((s.cost() - 4.0 * SQRT_3).abs() < 1e-9);
        assert_valid_path(&d, &t, &s);
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
        let (_, _, s) = cube(n, &blocked, (0, 0, 0), (3, 3, 3), true, ALGO_ASTAR);
        assert!(s.is_done() && !s.found());
        assert!(s.path().is_empty());
    }

    #[test]
    fn stepped_execution_matches_batch_for_every_algorithm() {
        let n = 8u32;
        let dom = Domain::raw(n, n, n, [false; 3]);
        let terrain = lcg_terrain(512, 42, 25, true);
        let (start, goal) = (dom.idx(0, 0, 0), dom.idx(7, 7, 7));
        for algo in [
            ALGO_ASTAR,
            ALGO_DIJKSTRA,
            ALGO_GREEDY,
            ALGO_SWARM,
            ALGO_CONVERGENT,
            ALGO_BISWARM,
            ALGO_BFS,
            ALGO_DFS,
        ] {
            let mut a = Solver::new(&dom, &terrain, start, goal, true, algo);
            let mut b = Solver::new(&dom, &terrain, start, goal, true, algo);
            while !a.is_done() {
                a.step_many(&dom, &terrain, 1);
            }
            b.run(&dom, &terrain);
            assert_eq!(a.found(), b.found(), "algo {algo}: found mismatch");
            assert_eq!(a.expanded(), b.expanded(), "algo {algo}: expanded mismatch");
            if a.found() {
                assert!((a.cost() - b.cost()).abs() < 1e-9, "algo {algo}: cost");
                assert_eq!(a.path(), b.path(), "algo {algo}: path mismatch");
            }
        }
    }

    // ---------- weighted cells --------------------------------------------

    #[test]
    fn dijkstra_equals_astar_on_weighted_random_grids() {
        for seed in [7u64, 99, 1234, 555, 2026] {
            let terrain = lcg_terrain(512, seed, 20, true);
            let (da, ta, a) = solve(
                (8, 8, 8),
                [false; 3],
                &terrain,
                (0, 0, 0),
                (7, 7, 7),
                true,
                ALGO_ASTAR,
            );
            let (dd, td, d) = solve(
                (8, 8, 8),
                [false; 3],
                &terrain,
                (0, 0, 0),
                (7, 7, 7),
                true,
                ALGO_DIJKSTRA,
            );
            assert_eq!(a.found(), d.found(), "seed {seed}: found mismatch");
            if a.found() {
                assert!(
                    (a.cost() - d.cost()).abs() < 1e-9,
                    "seed {seed}: A* {} != Dijkstra {}",
                    a.cost(),
                    d.cost()
                );
                assert_valid_path(&da, &ta, &a);
                assert_valid_path(&dd, &td, &d);
            }
        }
    }

    #[test]
    fn heavy_cell_forces_detour_light_cell_does_not() {
        // 5x3x3 corridor, start (0,1,1) -> goal (4,1,1), no diagonals. The
        // straight line passes through (2,1,1); a detour via y=0 costs 6.
        let (nx, ny, nz) = (5u32, 3u32, 3u32);
        let mid = idx(nx, ny, 2, 1, 1);
        for (w, want, through) in [(10u8, 6.0, false), (2u8, 5.0, true)] {
            let mut terrain = vec![1u8; (nx * ny * nz) as usize];
            terrain[mid as usize] = w;
            let (d, t, s) = solve(
                (nx, ny, nz),
                [false; 3],
                &terrain,
                (0, 1, 1),
                (4, 1, 1),
                false,
                ALGO_ASTAR,
            );
            assert!(s.found());
            assert!((s.cost() - want).abs() < 1e-9, "weight {w}: cost {}", s.cost());
            assert_eq!(s.path().contains(&mid), through, "weight {w}");
            assert_valid_path(&d, &t, &s);
            let (_, _, dj) = solve(
                (nx, ny, nz),
                [false; 3],
                &terrain,
                (0, 1, 1),
                (4, 1, 1),
                false,
                ALGO_DIJKSTRA,
            );
            assert!((dj.cost() - want).abs() < 1e-9);
        }
    }

    // ---------- BFS/DFS hop semantics --------------------------------------

    #[test]
    fn bfs_returns_fewest_hops_not_least_cost() {
        // 3x3x1, diagonals on, heavy centre. BFS takes the 2-hop diagonal
        // route through the centre; A* pays fewer coins on a 3-hop detour.
        let mut terrain = vec![1u8; 9];
        terrain[idx(3, 3, 1, 1, 0) as usize] = 10;
        let (d, t, bfs) = solve(
            (3, 3, 1),
            [false; 3],
            &terrain,
            (0, 0, 0),
            (2, 2, 0),
            true,
            ALGO_BFS,
        );
        assert!(bfs.found());
        assert_eq!(bfs.path().len(), 3, "fewest hops is 2 (3 cells)");
        assert_valid_path(&d, &t, &bfs);
        let (_, _, astar) = solve(
            (3, 3, 1),
            [false; 3],
            &terrain,
            (0, 0, 0),
            (2, 2, 0),
            true,
            ALGO_ASTAR,
        );
        assert!(astar.path().len() > bfs.path().len());
        assert!(astar.cost() < bfs.cost(), "least cost beats fewest hops here");
    }

    #[test]
    fn bfs_fewest_hops_hand_checked() {
        // Empty 4^3, corner to corner, diagonals on: 3 hops (4 cells).
        let (_, _, s) = cube(4, &[0u8; 64], (0, 0, 0), (3, 3, 3), true, ALGO_BFS);
        assert_eq!(s.path().len(), 4);
        // Diagonals off: Manhattan distance 9 hops (10 cells).
        let (_, _, s) = cube(4, &[0u8; 64], (0, 0, 0), (3, 3, 3), false, ALGO_BFS);
        assert_eq!(s.path().len(), 10);
    }

    // ---------- non-optimal algorithms stay honest -------------------------

    #[test]
    fn suboptimal_algorithms_return_valid_paths_never_beating_optimal() {
        for seed in [3u64, 77, 909] {
            for diagonals in [true, false] {
                let terrain = lcg_terrain(512, seed, 18, true);
                let (_, _, d) = solve(
                    (8, 8, 8),
                    [false; 3],
                    &terrain,
                    (0, 0, 0),
                    (7, 7, 7),
                    diagonals,
                    ALGO_DIJKSTRA,
                );
                for algo in [
                    ALGO_GREEDY,
                    ALGO_SWARM,
                    ALGO_CONVERGENT,
                    ALGO_BISWARM,
                    ALGO_BFS,
                    ALGO_DFS,
                ] {
                    let (dm, t, s) = solve(
                        (8, 8, 8),
                        [false; 3],
                        &terrain,
                        (0, 0, 0),
                        (7, 7, 7),
                        diagonals,
                        algo,
                    );
                    assert_eq!(
                        s.found(),
                        d.found(),
                        "seed {seed} algo {algo}: reachability must match Dijkstra"
                    );
                    if d.found() {
                        assert_valid_path(&dm, &t, &s);
                        assert!(
                            s.cost() >= d.cost() - 1e-9,
                            "seed {seed} algo {algo}: cost {} beats optimal {}",
                            s.cost(),
                            d.cost()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn bidirectional_returns_valid_connected_path() {
        let terrain = lcg_terrain(1000, 4242, 22, true);
        let (d, t, s) = solve(
            (10, 10, 10),
            [false; 3],
            &terrain,
            (0, 0, 0),
            (9, 9, 9),
            true,
            ALGO_BISWARM,
        );
        assert!(s.found(), "grid should be solvable");
        assert_valid_path(&d, &t, &s);
    }

    // ---------- wraparound --------------------------------------------------

    #[test]
    fn wrap_x_shortcut_beats_unwrapped_route() {
        // 9x3x3 corridor, endpoints on opposite x faces. Without wrap the best
        // route costs 8; with wrap_x it is a single step.
        let terrain = vec![1u8; 9 * 3 * 3];
        let (_, _, flat) = solve(
            (9, 3, 3),
            [false; 3],
            &terrain,
            (0, 1, 1),
            (8, 1, 1),
            false,
            ALGO_ASTAR,
        );
        let (d, t, wrapped) = solve(
            (9, 3, 3),
            [true, false, false],
            &terrain,
            (0, 1, 1),
            (8, 1, 1),
            false,
            ALGO_ASTAR,
        );
        assert!((flat.cost() - 8.0).abs() < 1e-9);
        assert!((wrapped.cost() - 1.0).abs() < 1e-9);
        assert_valid_path(&d, &t, &wrapped);
    }

    #[test]
    fn astar_stays_optimal_with_wrap_on_weighted_grids() {
        // The wrap-aware heuristic must keep A* == Dijkstra.
        for seed in [11u64, 313, 9000] {
            let terrain = lcg_terrain(9 * 7 * 8, seed, 20, true);
            let (d, t, a) = solve(
                (9, 7, 8),
                [true; 3],
                &terrain,
                (0, 0, 0),
                (8, 6, 7),
                true,
                ALGO_ASTAR,
            );
            let (_, _, dj) = solve(
                (9, 7, 8),
                [true; 3],
                &terrain,
                (0, 0, 0),
                (8, 6, 7),
                true,
                ALGO_DIJKSTRA,
            );
            assert_eq!(a.found(), dj.found());
            if a.found() {
                assert!(
                    (a.cost() - dj.cost()).abs() < 1e-9,
                    "seed {seed}: wrapped A* {} != Dijkstra {}",
                    a.cost(),
                    dj.cost()
                );
                assert_valid_path(&d, &t, &a);
            }
        }
    }

    // ---------- domain mask --------------------------------------------------

    #[test]
    fn void_cells_never_searched_and_masked_goal_is_unreachable() {
        // Ellipsoid mask inside a 9^3 box; everything outside is void.
        let n = 9u32;
        let total = (n * n * n) as usize;
        let c = 4.0f64;
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
            let (d, t, s) = solve(
                (n, n, n),
                [false; 3],
                &terrain,
                (4, 4, 0),
                (4, 4, 8),
                true,
                algo,
            );
            assert!(s.found(), "algo {algo}: path must exist inside the mask");
            for (i, &t) in terrain.iter().enumerate() {
                if t == T_VOID {
                    assert_eq!(
                        s.cells()[i],
                        FREE,
                        "algo {algo}: void cell {i} entered the search"
                    );
                    assert!(!s.path().contains(&(i as u32)));
                }
            }
            assert_valid_path(&d, &t, &s);
        }
        // A goal placed outside the mask reports no path immediately.
        assert!(!inside(0, 0, 0));
        let (_, _, s) = solve(
            (n, n, n),
            [false; 3],
            &terrain,
            (4, 4, 0),
            (0, 0, 0),
            true,
            ALGO_ASTAR,
        );
        assert!(s.is_done() && !s.found() && s.path().is_empty());
    }

    // ---------- misc edge cases ----------------------------------------------

    #[test]
    fn start_equals_goal_is_a_zero_cost_path() {
        let dom = Domain::raw(3, 3, 3, [false; 3]);
        let terrain = vec![1u8; 27];
        let c = dom.idx(1, 1, 1);
        let s = Solver::new(&dom, &terrain, c, c, true, ALGO_ASTAR);
        assert!(s.is_done() && s.found());
        assert_eq!(s.path(), &[c]);
        assert!(s.cost().abs() < 1e-12);
    }

    #[test]
    fn non_cubic_dims_route_correctly() {
        let terrain = vec![1u8; 6 * 3 * 4];
        let (d, t, s) = solve(
            (6, 3, 4),
            [false; 3],
            &terrain,
            (0, 0, 0),
            (5, 2, 3),
            true,
            ALGO_ASTAR,
        );
        assert!(s.found());
        // deltas (5,2,3) sorted desc: (5-3)*1 + (3-2)*sqrt2 + 2*sqrt3
        let want = 2.0 + SQRT_2 + 2.0 * SQRT_3;
        assert!((s.cost() - want).abs() < 1e-9, "cost {}", s.cost());
        assert_valid_path(&d, &t, &s);
    }

    #[test]
    fn rng_matches_the_reference_mulberry32_stream() {
        // Values produced by the JS reference implementation for seed 41.
        let mut r = Rng::new(41);
        let got: Vec<f64> = (0..4).map(|_| r.unit()).collect();
        for v in &got {
            assert!((0.0..1.0).contains(v), "rng out of range: {v}");
        }
        // Deterministic: the same seed replays the same stream.
        let mut r2 = Rng::new(41);
        let again: Vec<f64> = (0..4).map(|_| r2.unit()).collect();
        assert_eq!(got, again);
    }
}
