//! Volumetric pathfinding core for the 3D visualizer — edition 3.
//!
//! Everything that counts as a calculation lives in this crate: the shape of
//! the world ([`domain`]), what is in it ([`terrain`]), the eight search
//! algorithms ([`solver`]), route scoring ([`route`]) and turning a mouse ray
//! into a cell ([`pick`]). The front end owns pixels and nothing else — it
//! reads cell states and positions straight out of wasm linear memory and
//! draws them.
//!
//! [`World`] is the single wasm-bindgen surface. It holds the domain, the
//! terrain, the endpoints and the live solver, so the browser never has to
//! keep a second copy of any of it in sync.
//!
//! Index convention everywhere: `idx = x + nx*(y + ny*z)`.

use wasm_bindgen::prelude::*;

pub mod domain;
pub mod pick;
pub mod route;
pub mod solver;
pub mod terrain;

use domain::{Domain, Preset};
use solver::{Solver, ALGO_ASTAR, ALGO_DFS};

// Cell states surfaced through `state_ptr`.
pub const FREE: u8 = 0;
pub const OBST: u8 = 1;
pub const OPEN: u8 = 2;
pub const CLOSED: u8 = 3;
pub const PATH: u8 = 4;

// Terrain encoding.
/// Not part of the domain: untraversable and unrendered.
pub const T_VOID: u8 = 0;
/// Hard-blocked obstacle (infinite weight).
pub const T_OBST: u8 = 255;
/// Valid traversable weights are `1..=W_MAX`.
pub const W_MAX: u8 = 10;

/// `route_append` outcomes, so the UI can say why a click was refused
/// without re-deriving the reason.
pub const ROUTE_OK: i32 = 0;
pub const ROUTE_BLOCKED: i32 = -1;
pub const ROUTE_UNREACHABLE: i32 = -2;
pub const ROUTE_FINISHED: i32 = -3;

#[wasm_bindgen]
pub struct World {
    dom: Domain,
    terrain: Vec<u8>,
    preset: Preset,
    size: u32,
    density: u32,
    seed: u32,
    empty: bool,

    start: u32,
    goal: u32,
    diagonals: bool,
    algo: u8,

    solver: Solver,
    /// Set by any edit that invalidates the running search; the front end
    /// resets when the gesture ends rather than restarting mid-stroke.
    stale: bool,
    /// Expansion count of a completed search, for the scrub bar. Computed on
    /// demand and dropped whenever the world changes under it.
    total_steps: Option<u32>,
    /// Cached A* reference solve, used to score hand-drawn routes.
    optimal: Option<(f64, Vec<u32>)>,

    route: Vec<u32>,
    /// Route lengths before each committed leg, so undo removes a whole leg.
    route_marks: Vec<u32>,
    /// Reference cell for building in open space, where there is no wall face
    /// to place against.
    anchor: u32,
}

#[wasm_bindgen]
impl World {
    #[wasm_bindgen(constructor)]
    pub fn new(preset: &str, size: u32, density: u32, seed: u32, empty: bool) -> World {
        let dom = Domain::preset(Preset::parse(preset), size);
        let (start, goal) = (dom.start, dom.goal);
        let mut terrain = terrain::generate(&dom, density, seed, empty);
        terrain::sanitize(&mut terrain);
        terrain::free_endpoints(&mut terrain, start, goal);
        let solver = Solver::new(&dom, &terrain, start, goal, true, ALGO_ASTAR);
        let anchor = dom.idx(dom.nx / 2, dom.ny / 2, dom.nz / 2);
        World {
            dom,
            terrain,
            preset: Preset::parse(preset),
            size,
            density,
            seed,
            empty,
            start,
            goal,
            diagonals: true,
            algo: ALGO_ASTAR,
            solver,
            stale: false,
            total_steps: None,
            optimal: None,
            route: Vec::new(),
            route_marks: Vec::new(),
            anchor,
        }
    }

    /// Rebuild the world from scratch. Any argument left at its current value
    /// is kept, so the UI can change one slider without restating the rest.
    pub fn rebuild(&mut self, preset: &str, size: u32, density: u32, seed: u32, empty: bool) {
        self.preset = Preset::parse(preset);
        self.size = size;
        self.density = density;
        self.seed = seed;
        self.empty = empty;
        self.dom = Domain::preset(self.preset, size);
        self.start = self.dom.start;
        self.goal = self.dom.goal;
        self.terrain = terrain::generate(&self.dom, density, seed, empty);
        terrain::sanitize(&mut self.terrain);
        terrain::free_endpoints(&mut self.terrain, self.start, self.goal);
        self.anchor = self
            .dom
            .idx(self.dom.nx / 2, self.dom.ny / 2, self.dom.nz / 2);
        self.route.clear();
        self.route_marks.clear();
        self.invalidate();
        self.reset_search();
    }

    /// Wipe obstacles and weights back to weight-1 ground, keeping the shape.
    pub fn clear_terrain(&mut self) {
        for i in 0..self.dom.total() {
            if self.dom.exists[i] == 1 {
                self.terrain[i] = 1;
            }
        }
        self.route.clear();
        self.route_marks.clear();
        self.invalidate();
        self.reset_search();
    }

    // ---- shape ---------------------------------------------------------

    pub fn nx(&self) -> u32 {
        self.dom.nx as u32
    }
    pub fn ny(&self) -> u32 {
        self.dom.ny as u32
    }
    pub fn nz(&self) -> u32 {
        self.dom.nz as u32
    }
    /// Number of cells in the buffers (`nx*ny*nz`, void cells included).
    pub fn len(&self) -> u32 {
        self.dom.total() as u32
    }
    pub fn is_empty(&self) -> bool {
        self.dom.total() == 0
    }
    /// Number of cells that actually exist (outside the void mask).
    pub fn cell_count(&self) -> u32 {
        self.dom.count
    }
    pub fn label(&self) -> String {
        self.dom.label.clone()
    }
    pub fn preset(&self) -> String {
        self.dom.preset.as_str().to_string()
    }
    pub fn seed(&self) -> u32 {
        self.seed
    }
    pub fn wraps(&self) -> Vec<u8> {
        self.dom.wrap.iter().map(|&w| w as u8).collect()
    }
    /// World-space bounds of the existing cell centres: `[minX, minY, minZ,
    /// maxX, maxY, maxZ]`.
    pub fn bbox(&self) -> Vec<f32> {
        let b = &self.dom;
        vec![
            b.bmin[0], b.bmin[1], b.bmin[2], b.bmax[0], b.bmax[1], b.bmax[2],
        ]
    }
    pub fn span(&self) -> f32 {
        self.dom.span()
    }

    // ---- buffers -------------------------------------------------------
    //
    // Pointers into wasm linear memory. Rebuild the typed-array view after
    // any call that can allocate — growing the heap detaches old views — so
    // the front end simply re-derives them every frame.

    pub fn exists_ptr(&self) -> *const u8 {
        self.dom.exists.as_ptr()
    }
    pub fn terrain_ptr(&self) -> *const u8 {
        self.terrain.as_ptr()
    }
    /// Per-cell search state: FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4.
    pub fn state_ptr(&self) -> *const u8 {
        self.solver.cells().as_ptr()
    }
    pub fn pos_x_ptr(&self) -> *const f32 {
        self.dom.px.as_ptr()
    }
    pub fn pos_y_ptr(&self) -> *const f32 {
        self.dom.py.as_ptr()
    }
    pub fn pos_z_ptr(&self) -> *const f32 {
        self.dom.pz.as_ptr()
    }
    /// Per-cell Y rotation aligning a cube with the ring frame; all zero
    /// unless `curved()`.
    pub fn rot_ptr(&self) -> *const f32 {
        self.dom.rot.as_ptr()
    }
    pub fn curved(&self) -> bool {
        self.dom.curved
    }

    // ---- endpoints -----------------------------------------------------

    pub fn start_cell(&self) -> u32 {
        self.start
    }
    pub fn goal_cell(&self) -> u32 {
        self.goal
    }

    /// Move the start. The cell is snapped to the nearest existing, unblocked
    /// cell that is not the goal; returns the cell actually used, or `-1` if
    /// nothing suitable was near. Cheap enough to call on every pointer move.
    pub fn set_start(&mut self, cell: u32) -> i32 {
        self.set_endpoint(cell, true)
    }

    pub fn set_goal(&mut self, cell: u32) -> i32 {
        self.set_endpoint(cell, false)
    }

    fn set_endpoint(&mut self, cell: u32, is_start: bool) -> i32 {
        let avoid = if is_start { self.goal } else { self.start } as i32;
        let snapped = pick::snap(&self.dom, &self.terrain, cell, avoid);
        if snapped < 0 {
            return -1;
        }
        let snapped = snapped as u32;
        let cur = if is_start { self.start } else { self.goal };
        if cur == snapped {
            return snapped as i32;
        }
        if is_start {
            self.start = snapped;
        } else {
            self.goal = snapped;
        }
        // The endpoints changed, so a route drawn to the old ones is void.
        self.route.clear();
        self.route_marks.clear();
        self.invalidate();
        snapped as i32
    }

    // ---- search --------------------------------------------------------

    pub fn algo(&self) -> u8 {
        self.algo
    }

    pub fn set_algo(&mut self, algo: u8) {
        let algo = algo.min(ALGO_DFS);
        if algo == self.algo {
            return;
        }
        self.algo = algo;
        self.total_steps = None;
        self.reset_search();
    }

    pub fn diagonals(&self) -> bool {
        self.diagonals
    }

    pub fn set_diagonals(&mut self, on: bool) {
        if on == self.diagonals {
            return;
        }
        self.diagonals = on;
        // Adjacency changed underneath any drawn route.
        self.route.clear();
        self.route_marks.clear();
        self.invalidate();
        self.reset_search();
    }

    /// Rebuild the solver at expansion zero.
    pub fn reset_search(&mut self) {
        self.solver = Solver::new(
            &self.dom,
            &self.terrain,
            self.start,
            self.goal,
            self.diagonals,
            self.algo,
        );
        self.stale = false;
    }

    /// Advance the search by up to `steps` expansions.
    pub fn advance(&mut self, steps: u32) {
        self.solver
            .step_many(&self.dom, &self.terrain, steps);
    }

    /// Jump to an exact expansion count. The searches are deterministic, so
    /// rewinding is a replay from zero rather than an undo journal: no
    /// per-step history to store, and no chance of the two drifting apart.
    /// A full search over the largest supported domain replays in a couple of
    /// milliseconds, which is what makes scrubbing viable at all.
    pub fn seek(&mut self, step: u32) {
        self.reset_search();
        self.advance(step);
    }

    pub fn step_back(&mut self, steps: u32) {
        let target = self.solver.expanded().saturating_sub(steps.max(1));
        self.seek(target);
    }

    /// Expansions a completed search takes, for the scrub bar's range.
    /// Computed once per world state and cached.
    pub fn total_steps(&mut self) -> u32 {
        if let Some(n) = self.total_steps {
            return n;
        }
        let mut s = Solver::new(
            &self.dom,
            &self.terrain,
            self.start,
            self.goal,
            self.diagonals,
            self.algo,
        );
        s.run(&self.dom, &self.terrain);
        let n = s.expanded();
        self.total_steps = Some(n);
        n
    }

    pub fn expanded(&self) -> u32 {
        self.solver.expanded()
    }
    pub fn frontier(&self) -> u32 {
        self.solver.frontier()
    }
    pub fn is_done(&self) -> bool {
        self.solver.is_done()
    }
    pub fn found(&self) -> bool {
        self.solver.found()
    }
    /// True weighted cost of the path found, or NaN before there is one.
    pub fn cost(&self) -> f64 {
        self.solver.cost()
    }
    pub fn path(&self) -> Vec<u32> {
        self.solver.path().to_vec()
    }
    /// True when an edit has invalidated the displayed search.
    pub fn is_stale(&self) -> bool {
        self.stale
    }

    // ---- editing -------------------------------------------------------

    /// Every wall cell, for the renderer's instance list.
    pub fn obstacle_cells(&self) -> Vec<u32> {
        self.terrain
            .iter()
            .enumerate()
            .filter(|(_, &t)| t == T_OBST)
            .map(|(i, _)| i as u32)
            .collect()
    }

    /// Wall cells with at least one exposed face. Only these are worth
    /// outlining — rims on buried cells just z-fight with their neighbours.
    pub fn surface_obstacle_cells(&self) -> Vec<u32> {
        const FACES: [(i32, i32, i32); 6] = [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        ];
        self.terrain
            .iter()
            .enumerate()
            .filter(|(_, &t)| t == T_OBST)
            .map(|(i, _)| i as u32)
            .filter(|&i| {
                FACES.iter().any(|&(dx, dy, dz)| {
                    match self.dom.neighbor(i, dx, dy, dz) {
                        // Off the edge of the domain counts as exposed.
                        None => true,
                        Some(j) => self.terrain[j as usize] != T_OBST,
                    }
                })
            })
            .collect()
    }

    /// Cells carrying a traversal weight above 1, which the renderer draws as
    /// glowing dense-energy fields.
    pub fn weighted_cells(&self) -> Vec<u32> {
        self.terrain
            .iter()
            .enumerate()
            .filter(|(_, &t)| (2..=W_MAX).contains(&t))
            .map(|(i, _)| i as u32)
            .collect()
    }

    pub fn terrain_at(&self, cell: u32) -> u8 {
        if (cell as usize) < self.terrain.len() {
            self.terrain[cell as usize]
        } else {
            T_VOID
        }
    }

    /// Set one cell's terrain byte. Refuses void cells and refuses to wall in
    /// an endpoint. Returns true when something actually changed.
    pub fn paint(&mut self, cell: u32, value: u8) -> bool {
        if !self.dom.exists_at(cell) {
            return false;
        }
        let value = if value == T_OBST {
            if cell == self.start || cell == self.goal {
                return false;
            }
            T_OBST
        } else {
            value.clamp(1, W_MAX)
        };
        if self.terrain[cell as usize] == value {
            return false;
        }
        self.terrain[cell as usize] = value;
        self.stale = true;
        self.invalidate_caches();
        true
    }

    /// Drop cached derived values without touching the live solver — used
    /// while a paint stroke is still in progress.
    fn invalidate_caches(&mut self) {
        self.total_steps = None;
        self.optimal = None;
    }

    fn invalidate(&mut self) {
        self.stale = true;
        self.invalidate_caches();
    }

    // ---- picking -------------------------------------------------------

    /// First wall the ray meets, or `-1`. This is what a click erases.
    pub fn pick_solid(&self, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) -> i32 {
        pick::ray(&self.dom, &self.terrain, [ox, oy, oz], [dx, dy, dz], false).solid
    }

    /// The open cell a click would build into: the cell in front of the first
    /// wall, or — through open space, where there is no face to build against
    /// — where the ray crosses the camera-facing plane through the anchor.
    pub fn pick_build(&self, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) -> i32 {
        let hit = pick::ray(&self.dom, &self.terrain, [ox, oy, oz], [dx, dy, dz], false);
        if hit.solid >= 0 && hit.free >= 0 {
            return hit.free;
        }
        match pick::on_plane(&self.dom, [ox, oy, oz], [dx, dy, dz], self.anchor) {
            Some(c) if self.dom.exists_at(c) && self.terrain[c as usize] != T_OBST => c as i32,
            _ => hit.free,
        }
    }

    /// Where an endpoint dragged under this ray should land: the same
    /// resolution as `pick_build`, then snapped onto a legal cell.
    #[allow(clippy::too_many_arguments)]
    pub fn pick_endpoint(
        &self,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        is_start: bool,
    ) -> i32 {
        let anchor = if is_start { self.start } else { self.goal };
        let hit = pick::ray(&self.dom, &self.terrain, [ox, oy, oz], [dx, dy, dz], false);
        let raw = if hit.solid >= 0 && hit.free >= 0 {
            hit.free as u32
        } else {
            match pick::on_plane(&self.dom, [ox, oy, oz], [dx, dy, dz], anchor) {
                Some(c) => c,
                None if hit.free >= 0 => hit.free as u32,
                None => return -1,
            }
        };
        let avoid = if is_start { self.goal } else { self.start } as i32;
        pick::snap(&self.dom, &self.terrain, raw, avoid)
    }

    /// One ray-march serving both halves of a brush gesture:
    /// `[cell_to_clear, cell_to_paint]`, either of which may be `-1`.
    ///
    ///
    /// `include_weighted` makes the march stop on weighted cells as well as
    /// walls, which is what lets the weight brush grow a field outward rather
    /// than passing through it. The pair comes back in one call because the
    /// hover highlight needs both on every pointer move.
    #[allow(clippy::too_many_arguments)]
    pub fn pick_edit(
        &self,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        include_weighted: bool,
    ) -> Vec<i32> {
        let hit = pick::ray(
            &self.dom,
            &self.terrain,
            [ox, oy, oz],
            [dx, dy, dz],
            include_weighted,
        );
        let build = if hit.solid >= 0 && hit.free >= 0 {
            hit.free
        } else {
            match pick::on_plane(&self.dom, [ox, oy, oz], [dx, dy, dz], self.anchor) {
                Some(c) if self.dom.exists_at(c) && self.terrain[c as usize] != T_OBST => c as i32,
                _ => hit.free,
            }
        };
        vec![hit.solid, build]
    }

    /// Any existing cell under the ray — used for hover readouts.
    pub fn pick_any(&self, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) -> i32 {
        let hit = pick::ray(&self.dom, &self.terrain, [ox, oy, oz], [dx, dy, dz], false);
        if hit.solid >= 0 {
            hit.solid
        } else {
            hit.free
        }
    }

    pub fn anchor(&self) -> u32 {
        self.anchor
    }

    pub fn set_anchor(&mut self, cell: u32) {
        if self.dom.exists_at(cell) {
            self.anchor = cell;
        }
    }

    /// Push the build plane `steps` cells along a world-space direction —
    /// scrolling in an edit mode moves it toward or away from the camera.
    pub fn nudge_anchor(&mut self, dx: f32, dy: f32, dz: f32, steps: f32) -> u32 {
        let a = self.dom.world_of(self.anchor);
        let len = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-6);
        let p = [
            a[0] + dx / len * steps,
            a[1] + dy / len * steps,
            a[2] + dz / len * steps,
        ];
        if let Some(c) = self.dom.world_to_cell(p[0], p[1], p[2]) {
            if self.dom.exists_at(c) {
                self.anchor = c;
            }
        }
        self.anchor
    }

    // ---- hand-drawn routes ---------------------------------------------

    pub fn route(&self) -> Vec<u32> {
        self.route.clone()
    }

    pub fn route_len(&self) -> u32 {
        self.route.len() as u32
    }

    pub fn route_complete(&self) -> bool {
        self.route.len() > 1 && *self.route.last().unwrap() == self.goal
    }

    /// Extend the route to `cell`. Adjacent cells chain directly; a distant
    /// click is bridged with a shortest-hop leg, so drawing in 3D never
    /// demands placing every single cell. Clicking a cell already on the
    /// route rewinds to it. Returns `ROUTE_OK` or a `ROUTE_*` reason.
    pub fn route_append(&mut self, cell: u32) -> i32 {
        if !self.dom.exists_at(cell) || self.terrain[cell as usize] == T_OBST {
            return ROUTE_BLOCKED;
        }
        if self.route.is_empty() {
            self.route.push(self.start);
        }
        if self.route_complete() {
            return ROUTE_FINISHED;
        }
        let tail = *self.route.last().unwrap();
        if cell == tail {
            return ROUTE_FINISHED;
        }
        if let Some(at) = self.route.iter().position(|&c| c == cell) {
            self.route.truncate(at + 1);
            self.route_marks.retain(|&m| (m as usize) < self.route.len());
            return ROUTE_OK;
        }
        let mark = self.route.len() as u32;
        match route::connect(&self.dom, &self.terrain, tail, cell, self.diagonals) {
            Some(leg) => {
                self.route.extend_from_slice(&leg[1..]);
                self.route_marks.push(mark);
                ROUTE_OK
            }
            None => ROUTE_UNREACHABLE,
        }
    }

    /// Drop the most recently added leg, or the last cell if the route was
    /// built one neighbour at a time.
    pub fn route_undo(&mut self) {
        while self
            .route_marks
            .last()
            .is_some_and(|&m| m as usize >= self.route.len())
        {
            self.route_marks.pop();
        }
        match self.route_marks.pop() {
            Some(m) => self.route.truncate(m as usize),
            None => {
                let n = self.route.len().saturating_sub(1).max(1);
                self.route.truncate(n);
            }
        }
        if self.route.is_empty() {
            self.route.push(self.start);
        }
    }

    pub fn route_clear(&mut self) {
        self.route.clear();
        self.route_marks.clear();
    }

    pub fn route_begin(&mut self) {
        if self.route.is_empty() {
            self.route.push(self.start);
        }
    }

    /// Weighted cost of the drawn route, or NaN if it is not a legal route
    /// from start to goal.
    pub fn route_cost(&self) -> f64 {
        if self.route_error().is_some() {
            return f64::NAN;
        }
        route::path_cost(&self.dom, &self.terrain, &self.route)
    }

    /// Why the drawn route is not yet a legal start-to-goal path, or an empty
    /// string when it is.
    pub fn route_error(&self) -> Option<String> {
        route::validate_route(
            &self.dom,
            &self.terrain,
            &self.route,
            self.start,
            self.goal,
            self.diagonals,
        )
        .map(|s| s.to_string())
    }

    // ---- optimal reference ----------------------------------------------

    fn compute_optimal(&mut self) -> &(f64, Vec<u32>) {
        if self.optimal.is_none() {
            let mut s = Solver::new(
                &self.dom,
                &self.terrain,
                self.start,
                self.goal,
                self.diagonals,
                ALGO_ASTAR,
            );
            s.run(&self.dom, &self.terrain);
            self.optimal = Some(if s.found() {
                (s.cost(), s.path().to_vec())
            } else {
                (f64::NAN, Vec::new())
            });
        }
        self.optimal.as_ref().unwrap()
    }

    /// Cost of a genuinely optimal route, whatever algorithm is on display,
    /// so the scoreboard never flatters a suboptimal one.
    pub fn optimal_cost(&mut self) -> f64 {
        self.compute_optimal().0
    }

    pub fn optimal_path(&mut self) -> Vec<u32> {
        self.compute_optimal().1.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> World {
        World::new("cube", 12, 18, 41, false)
    }

    #[test]
    fn a_fresh_world_has_clean_endpoints_and_a_runnable_search() {
        for preset in ["cube", "prism", "pyramid", "sphere", "torus"] {
            let mut w = World::new(preset, 16, 18, 41, false);
            assert!(w.cell_count() > 0, "{preset}: empty mask");
            assert_eq!(w.terrain_at(w.start_cell()), 1, "{preset}: start carved");
            assert_eq!(w.terrain_at(w.goal_cell()), 1, "{preset}: goal carved");
            for i in 0..w.len() {
                let t = w.terrain_at(i);
                let exists = w.dom.exists[i as usize] == 1;
                if exists {
                    assert!(t == T_OBST || (1..=W_MAX).contains(&t), "{preset}: byte {t}");
                } else {
                    assert_eq!(t, T_VOID, "{preset}: mask violated");
                }
            }
            let n = w.total_steps();
            w.seek(n);
            assert!(w.is_done(), "{preset}: search must terminate");
        }
    }

    #[test]
    fn torus_wraps_x_and_nothing_else() {
        let w = World::new("torus", 16, 0, 1, true);
        assert_eq!(w.wraps(), vec![1u8, 0, 0]);
    }

    #[test]
    fn seek_reproduces_the_same_state_as_stepping_there() {
        let mut a = world();
        let mut b = world();
        let target = a.total_steps() / 2;
        a.advance(target);
        b.seek(target);
        assert_eq!(a.expanded(), b.expanded());
        assert_eq!(a.frontier(), b.frontier());
        assert_eq!(a.solver.cells(), b.solver.cells());
    }

    #[test]
    fn step_back_rewinds_exactly_one_expansion() {
        let mut w = world();
        let mid = w.total_steps() / 2;
        assert!(mid >= 1, "need a search with room to rewind");
        w.advance(mid);
        let before = w.solver.cells().to_vec();
        w.advance(1);
        assert_eq!(w.expanded(), mid + 1);
        w.step_back(1);
        assert_eq!(w.expanded(), mid);
        assert_eq!(w.solver.cells(), &before[..]);
    }

    #[test]
    fn seeking_past_the_end_lands_on_the_finished_search() {
        let mut w = world();
        let n = w.total_steps();
        w.seek(n + 5_000);
        assert!(w.is_done());
        assert_eq!(w.expanded(), n);
    }

    #[test]
    fn moving_an_endpoint_snaps_off_walls_and_never_onto_the_other_one() {
        let mut w = world();
        let goal = w.goal_cell();
        assert_eq!(w.set_start(goal), w.start_cell() as i32);
        assert_ne!(w.start_cell(), goal, "the endpoints must stay distinct");

        // Aim the start at a wall: it should land on an open cell nearby.
        let wall = (0..w.len()).find(|&i| w.terrain_at(i) == T_OBST);
        if let Some(wall) = wall {
            let got = w.set_start(wall);
            assert!(got >= 0);
            assert_ne!(w.terrain_at(got as u32), T_OBST);
        }
    }

    #[test]
    fn endpoints_cannot_be_walled_in_but_other_cells_can() {
        let mut w = world();
        let s = w.start_cell();
        assert!(!w.paint(s, T_OBST), "the start must not be paintable shut");
        assert!(!w.paint(w.goal_cell(), T_OBST));
        let other = (0..w.len())
            .find(|&i| i != s && w.terrain_at(i) != T_VOID && w.terrain_at(i) != T_OBST)
            .unwrap();
        assert!(w.paint(other, T_OBST));
        assert_eq!(w.terrain_at(other), T_OBST);
    }

    #[test]
    fn editing_marks_the_search_stale_rather_than_restarting_it() {
        let mut w = world();
        let mid = w.total_steps() / 2;
        w.advance(mid);
        let cell = (0..w.len())
            .find(|&i| i != w.start_cell() && i != w.goal_cell() && w.terrain_at(i) == 1)
            .unwrap();
        assert!(w.paint(cell, T_OBST));
        assert!(w.is_stale(), "an edit must flag the search as stale");
        assert_eq!(w.expanded(), mid, "but must not restart it mid-stroke");
        w.reset_search();
        assert!(!w.is_stale());
        assert_eq!(w.expanded(), 0);
    }

    #[test]
    fn a_hand_drawn_route_is_bridged_scored_and_never_beats_the_optimum() {
        let mut w = World::new("cube", 12, 12, 7, false);
        w.route_begin();
        assert_eq!(w.route_append(w.goal_cell()), ROUTE_OK);
        assert!(w.route_complete());
        assert!(w.route_error().is_none(), "{:?}", w.route_error());
        let cost = w.route_cost();
        let opt = w.optimal_cost();
        assert!(cost.is_finite() && opt.is_finite());
        assert!(
            cost >= opt - 1e-9,
            "a drawn route ({cost}) cannot beat the optimum ({opt})"
        );
    }

    #[test]
    fn route_undo_removes_a_whole_bridged_leg() {
        let mut w = World::new("cube", 12, 0, 3, true);
        w.route_begin();
        let mid = w.dom.idx(6, 6, 6);
        assert_eq!(w.route_append(mid), ROUTE_OK);
        let after_leg = w.route_len();
        assert!(after_leg > 1);
        assert_eq!(w.route_append(w.goal_cell()), ROUTE_OK);
        assert!(w.route_len() > after_leg);
        w.route_undo();
        assert_eq!(w.route_len(), after_leg, "undo drops exactly the last leg");
        w.route_undo();
        assert_eq!(w.route_len(), 1, "back to just the start cell");
    }

    #[test]
    fn a_route_into_a_wall_is_refused() {
        let mut w = world();
        let wall = (0..w.len()).find(|&i| w.terrain_at(i) == T_OBST);
        if let Some(wall) = wall {
            w.route_begin();
            assert_eq!(w.route_append(wall), ROUTE_BLOCKED);
        }
    }

    #[test]
    fn changing_the_algorithm_restarts_the_search_but_keeps_the_world() {
        let mut w = world();
        let before: Vec<u8> = (0..w.len()).map(|i| w.terrain_at(i)).collect();
        w.advance(120);
        w.set_algo(solver::ALGO_DFS);
        assert_eq!(w.expanded(), 0);
        assert_eq!(w.algo(), solver::ALGO_DFS);
        let after: Vec<u8> = (0..w.len()).map(|i| w.terrain_at(i)).collect();
        assert_eq!(before, after, "the terrain must survive an algorithm change");
    }

    #[test]
    fn clearing_terrain_leaves_open_ground_and_a_solvable_world() {
        let mut w = world();
        w.clear_terrain();
        for i in 0..w.len() {
            let t = w.terrain_at(i);
            assert!(t == T_VOID || t == 1, "cell {i} left at {t}");
        }
        let n = w.total_steps();
        w.seek(n);
        assert!(w.is_done() && w.found());
    }

    #[test]
    fn picking_resolves_a_cell_from_a_camera_ray() {
        let w = World::new("cube", 10, 0, 1, true);
        let target = w.dom.idx(5, 5, 5);
        let p = w.dom.world_of(target);
        // Nothing solid in an empty world, so this resolves through the plane
        // fallback; aim along -z with the anchor at the domain centre.
        let got = w.pick_build(p[0], p[1], p[2] + 25.0, 0.0, 0.0, -1.0);
        assert!(got >= 0, "an empty world must still be clickable");
    }
}
