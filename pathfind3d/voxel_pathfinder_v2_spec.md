# Edition 2: 3D pathfinding visualizer, feature expansion

This extends the existing project, it does not replace it. **Read the current
code first** and map these features onto what's already there, preserving the
established conventions (the flat index scheme, the cell-state constants, the
stepped-execution + state-buffer model that drives the animation, the
light-grid aesthetic, and the passing test suite). Everything below should slot
into that architecture, not fork it.

The prime directive from edition 1 still holds: this is a **true volumetric**
visualizer. None of the new features may reduce it to a tilted plane. Rotating
the camera must still reveal a real 3D volume.

Build the shared foundations first (Sections 1 to 3); the user-facing features
(Sections 4 to 7) are mostly thin layers on top of them.

---

## 1. Weighted cells

Introduce a per-cell **weight** so "weighted" algorithms have something to act
on. A cell has a weight `w >= 1`; entering it costs `base_step_cost(dir) * w`,
where `base_step_cost` is the existing Euclidean move cost (`1`, `√2`, `√3`).
Obstacles remain a separate hard-blocked state (effectively infinite weight).

- Weight range `[1, W_max]` (e.g. `W_max = 10`), surfaced as a brush value in
  the editor. Weight `1` is normal terrain.
- **Admissibility note (important):** keeping the minimum weight at `1` means the
  existing Euclidean/octile heuristic never overestimates, so A* and Dijkstra
  stay optimal. Do **not** allow weights below `1`, or the heuristic breaks and
  the "guarantees shortest path" claims become false.

---

## 2. One stepping loop, swappable frontier + priority

Unify all algorithms behind a single stepped search so the animation, the
`OPEN`/`CLOSED`/`PATH` state buffer, and the visibility system in Section 6 work
identically no matter which algorithm is selected. An algorithm is defined by
two things:

1. its **frontier structure** (priority queue, FIFO queue, or LIFO stack), and
2. its **priority function** over `g` (cost so far) and `h` (heuristic to goal).

The weighted algorithms are all instances of `f = g_weight * g + h_weight * h`:

| Algorithm                 | frontier    | g_weight | h_weight | optimal? |
|---------------------------|-------------|----------|----------|----------|
| Dijkstra                  | priority Q  | 1        | 0        | yes      |
| A*                        | priority Q  | 1        | 1        | yes      |
| Greedy Best-First         | priority Q  | 0        | 1        | no       |
| Swarm                     | priority Q  | 1        | ~1.5     | no       |
| Convergent Swarm          | priority Q  | 1        | ~3       | no       |
| Bidirectional Swarm       | 2 priority Qs | 1      | ~1.5     | no       |
| Breadth-First (unweighted)| FIFO queue  | (hops)   | 0        | fewest hops |
| Depth-First (unweighted)  | LIFO stack  | (hops)   | 0        | no       |

Notes:

- The **Swarm** family isn't a formally specified algorithm; it's weighted A*
  (heuristic weighted above 1), which produces the broad, goal-biased "swarming"
  spread and non-optimal paths. Convergent Swarm just leans harder on the
  heuristic, so it funnels toward the goal faster and explores less. Pick weights
  that make the three swarm variants visibly distinct in the animation.
- **Bidirectional Swarm** runs two weighted searches at once, one from the start
  and one from the goal, alternating expansions, stopping when their closed sets
  first intersect, then stitching the two half-paths at the meeting cell. Since
  the swarm variants are already non-optimal, first-touch termination is
  acceptable here; just make sure the reconstructed path is actually connected
  and collision-free.
- **BFS/DFS are unweighted:** they ignore cell weights and treat every move as
  one hop (diagonal moves still allowed if diagonals are on). BFS therefore
  returns the fewest-hops path, which is not the same as the least-cost path once
  weights or diagonal costs are in play. Surface that honestly in the UI (see
  Section 5) so the comparison stays fair.
- Each algorithm must report the same stats the current one does (expanded,
  frontier size, path length, path cost, status) so the HUD and comparison
  feature are algorithm-agnostic.

A dropdown selects the active algorithm, with a one-line description of each
(guarantees shortest / faster but not optimal / etc.), matching the framing in
the list the user provided.

---

## 3. Domain abstraction (geometry beyond the cube)

Generalize "cube of side N" into a **Domain**:

- **dims** `(nx, ny, nz)` — allows non-cubic bounding boxes. Generalize the index
  scheme to `idx = x + nx*(y + ny*z)` and make neighbor bounds per-axis.
- **existence mask** — a per-cell bit for whether the cell is part of the domain.
  Cells outside the mask are *void*: not traversable, not rendered, not editable.
- **wrap flags** `(wrap_x, wrap_y, wrap_z)` — per-axis toroidal connectivity. On a
  wrapped axis, neighbor indexing is modular, so a path can leave one face and
  re-enter the opposite one.

Ship these domain presets:

- **Rectangular prism** — full box, no wrap, non-cubic dims allowed.
- **Pyramid** — mask whose cross-section shrinks with height.
- **Sphere / ellipsoid** — mask by radius from center.
- **Torus** — a donut mask (major/minor radius) **with wrap around the major
  circumference**, so routes genuinely wrap around the ring. This is the headline
  case; it should be obvious when watching a path that it wrapped.

**Wraparound heuristic (important gotcha):** on any wrapped axis, the per-axis
delta must be `min(|a - b|, n_axis - |a - b|)` before combining the three axis
deltas into the heuristic. If you use the raw delta on a wrapped axis the
heuristic overestimates and A*/Dijkstra lose optimality. Add a test for this
(Section 8).

Rendering: only existing cells are drawn and paintable; the bounding cage / grid
adapts to the domain (box for the prism, an appropriate wireframe hull otherwise).
A domain selector rebuilds the grid; obstacles/weights reset or remap sensibly.

---

## 4. Interactive editing

The user should be able to build scenarios directly, not just randomize.

**Edit modes** (a mode selector): Move Start, Move Goal, Paint Obstacle, Paint
Weight, Erase.

- **Movable start / goal:** drag the start and goal markers to any existing cell
  (raycast to pick, snap to nearest valid cell). Repositioning must not rebuild
  the whole scene, just reset the search.
- **Painting into 3D space (the real UX problem):** a screen click is
  depth-ambiguous, so provide two placement methods and let the user use either:
  1. **Active build slice** — a highlighted plane at a selectable layer along a
     chosen axis; clicks paint on that plane. Scroll or a slider moves the slice
     through the volume so the user can work layer by layer.
  2. **Face-adjacency** — when hovering an existing block, a click adds a cell on
     the hovered face (place-on-surface, like a voxel editor); shift-click or
     right-click removes. This makes it easy to extend structures.
- **Click-drag to paint** runs of cells. The Weight brush uses a value slider
  (cost `1`–`W_max`), shown as a color ramp so heavier cells read as "denser".
- **Empty grid:** a toggle to start from an empty domain, plus a Clear button.
  Keep Randomize as well.
- Recompute the path on edit-release or on an explicit Run, not on every mouse
  move, to keep it responsive.

---

## 5. Compare your path vs the algorithm

Reproduce the classic "guess the path, then race the algorithm" feature, adapted
honestly to 3D.

- A **Challenge / Draw mode:** the user builds their own attempted route by
  clicking a sequence of cells from start to goal (click-drag to extend along
  grid-adjacent cells). Waypoints beat free-dragging in 3D because free-drag has
  no depth; if you want to allow non-adjacent clicks, auto-connect consecutive
  waypoints with a short internal BFS so the user isn't forced to place every
  cell.
- **Validate** the user route: it must start at start, end at goal, stay on
  existing cells, and avoid obstacles. Flag invalid routes clearly.
- **Score and compare:** always compute the optimal reference cost (via
  Dijkstra/A*, regardless of which algorithm is selected for display) and show:
  user cost vs optimal cost, the difference, and the ratio (e.g. "your route is
  18% longer than optimal"). Draw both paths at once in distinct colors.
- If the selected display algorithm is non-optimal (Greedy, Swarm, DFS), show its
  cost too, so the user sees the three-way spread: their guess, the algorithm's
  result, and the true optimum.

---

## 6. Seeing the path and cloud through obstacles

Occlusion is the central readability problem in a 3D volume. Implement a layered
solution rather than one trick:

- **Auto-dim while searching (the user's request):** when a search is running,
  drop obstacle and weight opacity to a low value (x-ray) so the explored cloud
  and path read through them; restore opacity when it finishes or is paused. Make
  the dimmed opacity a slider.
- **Always-visible path/start/goal:** render the path, start, and goal in a pass
  with depth testing disabled (or an overlay pass) so they're never fully hidden
  behind geometry, with a slightly desaturated tint on the occluded portions so
  depth is still legible. The light-wall trail from edition 1 should remain
  visible through obstacles this way.
- **Clipping-plane slice (headline tool):** a slider that slices away the front of
  the volume along a chosen axis (Three.js `clippingPlanes`), letting the user cut
  into the domain and see the interior directly. This is the most reliable way to
  inspect a dense 3D search.
- **Camera-facing peel (optional):** fade cells that sit between the camera and
  the current frontier so you're never looking through a wall of closed cells.

Provide these as toggles/sliders; the auto-dim and the clip slice are the two
that matter most.

---

## 7. Keep the edition-1 character

- True volumetric behavior, 26-connectivity (with the 6-connected toggle), the
  admissible heuristic, stepped animation, and the existing HUD.
- The light-grid / lightcycle aesthetic and its mechanics (path as a light wall, a
  marker traversing the solved route, rez-in cells, dark-void staging, the
  two-color energy language). New elements (weights, multiple domains, user path)
  should be themed to fit, not bolted on in a clashing style.
- Reduced-motion respected, responsive down to a phone, WASM core with the JS
  mirror so it still runs without a build step, static hosting unchanged.

---

## 8. Tests (extend the existing suite; all must pass)

Add unit tests for the new invariants:

1. **Dijkstra == A\*** total cost on a weighted random grid (both optimal).
2. **Weighted routing:** a high-weight wall with a cheap detour causes the optimal
   path to go around; lowering the wall's weight makes it go through. Costs match
   expectation.
3. **BFS = fewest hops** on a small hand-checked case (and note it may differ from
   least-cost when weights/diagonals are present).
4. **Greedy / DFS** return a valid connected start-to-goal path whose cost is
   `>=` optimal (never a false "shorter than optimal").
5. **Wraparound:** with `wrap_x` on and start/goal near opposite x-faces, the path
   cost via wrap is less than with wrap off; and A* cost still equals Dijkstra
   cost with wrap on (heuristic stays admissible).
6. **Domain mask:** void cells never appear in any closed set or path; a goal
   placed in a masked-out region reports no path.
7. **Bidirectional** returns a valid connected path on a solvable grid.

---

## Build discipline (for Claude Code)

Assume a clean machine: check for and install the toolchain (Rust, the
`wasm32-unknown-unknown` target, wasm-pack) before building, and say what you're
installing. Do not consider the task done until `cargo test` passes and
`wasm-pack build --target web` completes clean. Preview the frontend and confirm
each new feature works before finishing: switch algorithms, drag start/goal,
paint obstacles and weights on a slice, draw a comparison path, slice the volume
with the clip plane, and load each domain preset including the torus wrap.
