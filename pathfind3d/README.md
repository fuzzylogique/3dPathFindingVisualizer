# pathfind3d — volumetric pathfinding light-trail visualizer

A genuinely volumetric 3D pathfinding visualizer: a dense voxel **domain** where
cells are blocked, weighted, or void independently along all three axes,
searched in true 3D by **eight algorithms** with full 26-connectivity. The
explored set grows as a glowing 3D cloud you can orbit around and see into;
the found route renders as a neon **light wall** — a vertical ribbon extruded
below the path curve — with a glowing cycle running the path.

- **Weighted cells** `w ∈ [1, 10]` — entering a cell costs
  `base_step_cost(dir) × w`; obstacles stay a separate hard-blocked state.
- **Eight algorithms** behind one stepping loop: A\*, Dijkstra, Greedy
  Best-First, Swarm, Convergent Swarm, Bidirectional Swarm, BFS, DFS.
- **Domains beyond the cube**: rectangular prism, pyramid, sphere, and a
  **torus** whose x axis genuinely wraps (rendered as a real donut — routes
  cross the modular seam).
- **Direct-manipulation editing**: drag the start and goal markers anywhere,
  in any tool. Wall and weight brushes preview the exact cell they will hit.
- **Full playback control**: run, pause, step forward, step *back*, and scrub
  to any point in the search. Pausing survives edits and option changes.
- **Challenge mode**: draw your own route (distant clicks auto-connect), then
  race the algorithm — your cost vs the true optimum vs the displayed
  algorithm's result, all three paths drawn at once.

## Architecture: Rust computes, the browser draws

**Every calculation lives in Rust**, compiled to WebAssembly: the domain
shapes and their world-space layout, world generation, all eight searches,
route validation and scoring, and turning a mouse ray into a cell. The
browser holds no copy of the model. It reads cell states and positions
straight out of wasm linear memory and renders them with Three.js.

```
pathfind3d/
├── Cargo.toml
├── src/
│   ├── lib.rs          # `World` — the single wasm-bindgen surface
│   ├── domain.rs       # presets, existence masks, cell ↔ world-space mapping
│   ├── terrain.rs      # seeded world generation (mulberry32)
│   ├── solver.rs       # the unified search engine
│   ├── route.rs        # route validation, costing, auto-connect
│   └── pick.rs         # ray-march picking
├── web/
│   ├── index.html      # markup only
│   ├── css/style.css   # all styling
│   ├── js/             # rendering and input — no model logic
│   │   ├── main.js     #   boot + frame loop
│   │   ├── scene.js    #   renderer, camera, markers, set dressing
│   │   ├── volume.js   #   voxel meshes, explored cloud, hover highlight
│   │   ├── trails.js   #   light wall, route and optimal tubes
│   │   ├── input.js    #   pointer + keyboard → World calls
│   │   └── ui.js       #   control panel
│   └── pkg/            # wasm-pack output (generated, not committed)
└── tests/
    ├── core.test.mjs   # the wasm boundary, from JavaScript
    ├── e2e.test.mjs    # headless-browser tests over DevTools protocol
    └── server.mjs      # static server for development and e2e
```

Earlier editions shipped a hand-maintained JavaScript mirror of the solver
as a fallback. It has been removed: one implementation, no drift.

## Build and run

The WASM core is required — there is no fallback.

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-pack                                   # once
wasm-pack build --target web --out-dir web/pkg --release
node tests/server.mjs                                     # → http://127.0.0.1:8080
```

The page must be served over HTTP: `file://` blocks ES modules, and WASM
needs the `application/wasm` MIME type (the bundled server sets it; so does
`python -m http.server 8080 -d web`). If the core is missing, the page says
so and shows the build command instead of failing silently.

## `World` API

Index convention **everywhere**: `idx = x + nx*(y + ny*z)`.

```js
const world = new World(preset, size, densityPct, seed, startEmpty);
```

`preset` is `cube | prism | pyramid | sphere | torus`. Terrain bytes are `0` =
**void** (outside the domain), `1..10` = a cell of that **weight**, `255` =
**obstacle**.

| Area | Methods |
|---|---|
| World | `rebuild(preset, size, density, seed, empty)`, `clear_terrain()`, `nx/ny/nz()`, `len()`, `cell_count()`, `preset()`, `seed()`, `label()`, `wraps()`, `curved()`, `bbox()`, `span()` |
| Buffers | `state_ptr()`, `terrain_ptr()`, `exists_ptr()`, `pos_x/y/z_ptr()`, `rot_ptr()` — all `len()` long. **Rebuild typed-array views after any call that can allocate.** |
| Instance lists | `obstacle_cells()`, `surface_obstacle_cells()`, `weighted_cells()` |
| Endpoints | `start_cell()`, `goal_cell()`, `set_start(cell)`, `set_goal(cell)` — snap to the nearest legal cell and return the one used |
| Search | `set_algo(id)`, `set_diagonals(on)`, `reset_search()`, `advance(n)`, `step_back(n)`, `seek(step)`, `total_steps()`, `expanded()`, `frontier()`, `is_done()`, `found()`, `cost()`, `path()`, `is_stale()` |
| Editing | `paint(cell, value)`, `terrain_at(cell)` — refuses void cells and walling in an endpoint |
| Picking | `pick_edit(ray…, includeWeighted)` → `[cellToClear, cellToPaint]`, `pick_endpoint(ray…, isStart)`, `pick_any(ray…)`, `pick_solid(ray…)`, `set_anchor(cell)`, `nudge_anchor(dir…, steps)` |
| Routes | `route_begin()`, `route_append(cell)` → `0` ok / `-1` blocked / `-2` unreachable / `-3` finished, `route_undo()`, `route_clear()`, `route()`, `route_len()`, `route_complete()`, `route_cost()`, `route_error()`, `optimal_cost()`, `optimal_path()` |

Cell states in the state buffer: `FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4`.

### How rewinding works

The searches are deterministic, so `seek(n)` rebuilds the solver and replays
`n` expansions rather than keeping an undo journal. There is no per-step
history to store and no way for a journal to drift from the forward pass. A
full search over the largest domain replays in a few milliseconds, which is
fast enough to scrub.

### How picking works

A click into a 3D volume is depth-ambiguous. The core marches the camera ray
in sub-cell steps, mapping each sample back to a cell through the inverse of
the domain layout — which is why the curved torus needs no special case. One
march yields the first wall hit (what a clear-click removes) and the open cell
in front of it (where a paint-click builds). Through empty space, where there
is no wall to build against, it intersects the ray with a camera-facing plane
through an anchor cell instead; dragging a marker uses the same fallback
anchored on the marker itself.

## The algorithms — one loop, swappable frontier + priority

Every algorithm is the same stepping loop with a frontier structure and a
priority `f = g_w·g + h_w·h` (`g` = cost so far, `h` = admissible heuristic):

| id | algorithm | frontier | g_w | h_w | optimal? |
|---|---|---|---|---|---|
| 0 | A\* | priority queue | 1 | 1 | **yes** |
| 1 | Dijkstra | priority queue | 1 | 0 | **yes** |
| 2 | Greedy Best-First | priority queue | 0 | 1 | no |
| 3 | Swarm | priority queue | 1 | 1.5 | no |
| 4 | Convergent Swarm | priority queue | 1 | 3 | no |
| 5 | Bidirectional Swarm | 2 × priority queue | 1 | 1.5 | no |
| 6 | Breadth-First | FIFO queue | (hops) | — | fewest **hops** |
| 7 | Depth-First | LIFO stack | (hops) | — | no |

- Move cost between neighbours is Euclidean — a step touching `k` axes costs
  `√k` (`1`, `√2`, `√3`) — times the **weight of the cell being entered**.
  Weights never drop below 1, so the heuristic (exact minimum cost across an
  empty grid; octile-style for 26-connectivity, Manhattan for 6) never
  overestimates and A\*/Dijkstra stay optimal on weighted grids.
- The swarm family is weighted A\* (heuristic scaled above 1): broad,
  goal-biased exploration, paths within a bounded factor of optimal but not
  optimal. Bidirectional Swarm runs one search from each endpoint,
  alternating expansions, stopping when the closed sets first intersect and
  stitching the halves at the meeting cell.
- BFS/DFS are unweighted hop searches (diagonals still count one hop when
  enabled). BFS returns the **fewest-hops** path — *not* the least-cost path
  once weights or diagonal costs are in play; the UI says so, and `cost()`
  still reports the honest weighted cost of whatever path they return.
- **Wraparound heuristic gotcha**: on a wrapped axis the per-axis delta is
  `min(|a−b|, n−|a−b|)` before combining. Raw deltas would overestimate and
  break A\*'s optimality (tested).
- `BinaryHeap` has no decrease-key, so improved nodes are pushed as
  duplicates; a popped node that is no longer open in that search is skipped.
  Ties break on lower `h`, then index, so runs are fully deterministic —
  which is what lets the scrub bar rewind by replaying.

## Controls

**Playback** — `⏮` rewind, `◀` step back, `RUN`/`PAUSE`, `▶` step forward,
`⏭` run to the end, plus a scrub bar across every expansion. A change never
resumes playback on its own: edits, algorithm and option changes restart the
search but leave it paused if it was paused.

**Endpoints** — drag the glowing start (cyan) or goal (amber) marker. It works
in every tool; hovering a marker rings it to show it can be grabbed. Markers
snap to the nearest open cell and never land in a wall or on each other.

**Tools**

| Tool | Key | Click | Alt-click / right-click |
|---|---|---|---|
| View | `V` | orbit | — |
| Wall | `W` | build a wall against the face you point at | remove the wall |
| Weight | `E` | paint a cell at the brush weight | reset to normal ground |
| Route | `R` | extend your route; far clicks auto-connect | — |

In Wall, Weight and Route a highlight shows exactly which cell a click will
affect, and the bottom-left readout gives its coordinates and contents.
Shift-scroll pushes the build plane deeper when there is nothing to build
against. Edits apply on release.

**Keyboard** — `Space` run/pause, `,` `.` step back/forward, `Home`/`End`
rewind/run to end, `V W E R` tools, `N` randomize, `Esc` back to View, arrow
keys orbit, `+`/`-` zoom.

**Seeing through the volume** — walls auto-dim to x-ray while a search runs;
a clip plane slices the volume open along any axis; the path, markers and
routes draw a faint through-wall pass so they are never fully hidden.

**Challenge mode** — the route starts at the start marker. Reaching the goal
scores it against the true optimum (always A\*, whatever is on display), with
the percentage over optimal and the displayed algorithm's cost. Your route is
cyan, the optimum pale white, the algorithm's trail amber.

Reduced motion (`prefers-reduced-motion`) disables flicker, auto-rotate and
the auto-traversing cycle.

The aesthetic is an original light-grid / lightcycle homage — no logos,
characters, or vehicle designs are reproduced.

## Tests

```sh
cargo test                     # 45 tests: algorithms, domains, picking, routes, World
node tests/core.test.mjs       # 20 tests: the wasm boundary, driven from JavaScript
node tests/e2e.test.mjs        # 17 tests: the real page in a headless browser
```

`cargo test` carries the algorithm canon — known-optimal costs, Dijkstra ==
A\* on weighted grids, BFS fewest-hops, wrap-aware admissibility, void cells
never searched, stepped == batch for every algorithm — plus the picking
round-trip for every preset (including the torus), seek/step-back exactness,
and edits marking the search stale instead of restarting it.

`core.test.mjs` checks what Rust tests cannot: that the exported API behaves
from JavaScript and that buffer views stay correct across heap growth.

`e2e.test.mjs` needs a Chromium-based browser (Edge or Chrome are found
automatically; set `BROWSER=` otherwise). It drives real pointer input over
the DevTools protocol — dragging both markers, hover-previewing and placing a
wall, alt-click removal, keyboard shortcuts — and asserts that pausing
survives edits and that every preset renders without console errors.

## Hosting

Everything under `web/` is static, but `web/pkg/` is generated and not
committed, so the host must build it or you must deploy a built copy.

- **Cloudflare Pages:** build command
  `wasm-pack build --target web --out-dir web/pkg --release` (needs Rust in
  the build image), output directory `web`.
- **GitHub Pages:** build locally or in an Action, then publish `web/`
  including `web/pkg/`.
- Any static host works; the requirements are HTTP (not `file://`) and the
  `application/wasm` MIME type.
