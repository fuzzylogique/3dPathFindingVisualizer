# pathfind3d — volumetric pathfinding light-trail visualizer

A genuinely volumetric 3D pathfinding visualizer: a dense voxel **domain** where
cells are blocked, weighted, or void independently along all three axes,
searched in true 3D by **eight algorithms** with full 26-connectivity. The
explored set grows as a glowing 3D cloud you can orbit around and see into;
the found route renders as a neon **light wall** — a vertical ribbon extruded
below the path curve — with a glowing cycle running the path.

Edition 2 adds:

- **Weighted cells** `w ∈ [1, 10]` — entering a cell costs
  `base_step_cost(dir) × w`; obstacles stay a separate hard-blocked state.
- **Eight algorithms** behind one stepping loop: A\*, Dijkstra, Greedy
  Best-First, Swarm, Convergent Swarm, Bidirectional Swarm, BFS, DFS.
- **Domains beyond the cube**: rectangular prism, pyramid, sphere, and a
  **torus** whose x axis genuinely wraps (rendered as a real donut — routes
  cross the modular seam).
- **Interactive editing**: move start/goal, paint obstacles and weights into
  3D space via a build-slice ghost layer or face-adjacency (voxel-editor
  style), erase, clear, empty worlds.
- **Challenge mode**: draw your own route (distant clicks auto-connect with an
  internal BFS), then race the algorithm — your cost vs the true optimum vs
  the displayed algorithm's result, all three paths drawn at once.
- **Seeing through the volume**: auto-dim x-ray while searching, a clipping
  plane that slices the volume open, and depth-test-free ghost passes so the
  path, start, and goal are never fully hidden.

The solver is written in Rust and compiles to WebAssembly. The frontend is a
single static `web/index.html` (Three.js r128 from CDN, custom orbit camera,
no build step, no backend) that ships a line-for-line JS mirror of the Rust
solver, so the demo runs immediately and the WASM core drops in when built.

```
pathfind3d/
├── Cargo.toml
├── src/lib.rs               # unified search engine + wasm-bindgen API + unit tests
├── web/index.html           # Three.js frontend + JS mirror solver
├── tests/mirror.test.mjs    # Node tests for the JS mirror (same invariants)
└── README.md
```

## Run it now (no Rust required)

The page must be served over HTTP — `file://` blocks module scripts, and WASM
needs the `application/wasm` MIME type.

```sh
cd pathfind3d
python -m http.server 8080 -d web
# or: npx serve web
```

Open http://localhost:8080. The HUD shows `CORE JS` — the JS mirror solver is
running. Behaviour, constants, costs, and API are identical to the Rust core.

## Build the WASM core

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-pack          # once
wasm-pack build --target web --out-dir web/pkg
```

Reload the page — the HUD flips to `CORE WASM`. Nothing else changes.

### How the swap works

`web/index.html` boots with the JS mirror and upgrades itself if `web/pkg/`
exists. The swap is exactly three lines, already wired inside a `try/catch`:

```js
const wasm = await import('./pkg/pathfind3d.js'); // 1. load the bindings
await wasm.default();                             // 2. instantiate the .wasm
SolverCtor = wasm.Solver;                         // 3. use it instead of the mirror
```

## Solver API (identical in Rust/WASM and the JS mirror)

Index convention **everywhere**: `idx = x + nx*(y + ny*z)`.

```js
new Solver(nx, ny, nz, terrain,
           wrapX, wrapY, wrapZ,
           sx, sy, sz, gx, gy, gz,
           diagonals, algo)
```

`terrain` is a `Uint8Array` of `nx*ny*nz` bytes: `0` = **void** (outside the
domain — untraversable, unrendered), `1..10` = existing cell with that
**weight**, `255` = **obstacle**. Obstacles at the endpoints are carved free;
an endpoint on a void cell reports *no path* immediately. `wrap*` enable
per-axis toroidal connectivity (neighbour indexing goes modular).

| Method | Returns | Notes |
|---|---|---|
| `step_many(steps)` | — | Advance by at most `steps` node expansions (stepped execution for animation). |
| `state()` | `Uint8Array` | Per-cell: `FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4`. Void cells always read FREE. WASM returns a copy; the JS mirror a live view. |
| `path()` | `Uint32Array` | Cell indices start→goal; empty until solved. |
| `expanded()` | `u32` | Nodes closed so far (both directions for Bidirectional Swarm). |
| `frontier()` | `u32` | Open-set size (sum of both fronts for Bidirectional Swarm). |
| `is_done()` / `found()` | `bool` | Done + not found = no path. |
| `cost()` | `f64` | **True weighted cost of the returned path** (re-summed, so BFS/DFS report honest weighted costs); NaN until found. |
| `algo()` / `start()` / `goal()` | ids | Introspection. |
| `state_ptr()`, `state_len()` | ptr, `u32` | Zero-copy view into wasm memory (rebuild after any allocating call). |
| `free()` | — | wasm-bindgen destructor (no-op in the mirror). |

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
  Ties break on lower `h`, then index, keeping JS and WASM bit-identical.

## Frontend

- **Controls:** Run/Pause, Step, Reset, Randomize, Clear; algorithm and domain
  dropdowns (with one-line descriptions); sliders for speed, size, density,
  weight brush (1–10, colour-ramped), build-slice layer, x-ray opacity, clip
  depth; toggles for explored cloud, auto-rotate, 26- vs 6-connectivity,
  start-empty, slice ghost, clip plane. `SPACE` run/pause, `S` step,
  `R` randomize, `ESC` back to view mode, `[` `]` move the slice, arrow keys
  orbit, `+`/`-` zoom.
- **Edit modes:** VIEW (orbit), START/GOAL (click to move, snaps to the
  nearest valid cell), WALL / WEIGHT / ERASE (paint via the slice ghost or by
  clicking block faces; shift-click removes), ROUTE (challenge mode). In any
  edit mode the right mouse button still orbits and the wheel moves the
  slice. Edits recompute the path on release.
- **Challenge mode:** the route starts at the start marker; each click
  extends it (adjacent cells chain directly, distant clicks auto-connect via
  BFS over open cells; clicking an earlier route cell rewinds). Reaching the
  goal scores it: your cost vs the optimal cost (always computed with A\*,
  whatever is on display) with the % over optimal, plus the displayed
  algorithm's cost after it runs — user route in cyan, optimum in pale white,
  algorithm trail in amber.
- **Occlusion tools:** while a search runs, obstacles and weights auto-dim to
  the x-ray opacity and recover on pause/finish; the clip slider slices the
  volume open along any axis (Three.js clipping planes); path, start, goal,
  and route render a second depth-test-free pass, desaturated where occluded,
  so they read through geometry.
- **Scene:** instanced voxels for obstacles (matte, neon-rimmed), weighted
  cells as amber energy (brighter + larger = denser), frontier vs closed as
  additive glowing clouds with rez-in flicker, Catmull-Rom light wall +
  travelling cycle, emissive start/goal markers, per-domain wireframe hull
  (box, pyramid cage, sphere rings, donut grid with a seam marker), floor
  grid and lit horizon. Custom orbit camera; auto-rotate stops on
  interaction.
- Reduced motion (`prefers-reduced-motion`) disables flicker, auto-rotate,
  and the auto-traversing cycle. No storage; all state is in memory.

The aesthetic is an original light-grid / lightcycle homage — no logos,
characters, or vehicle designs are reproduced.

## Tests

```sh
cargo test                    # 17 Rust tests
node tests/mirror.test.mjs    # 19 JS-mirror tests (same invariants)
```

Beyond the edition-1 canon (known-optimal costs on empty cubes, threading a
one-hole wall, walled-off goal, stepped == batch execution, legal path
steps), the suite covers the edition-2 invariants:

1. Dijkstra == A\* total cost on weighted random grids (both optimal).
2. A high-weight cell forces the optimal path around it; lowering the weight
   sends the path through. Exact costs asserted.
3. BFS returns fewest hops on hand-checked cases — and demonstrably *not*
   the least-cost route when weights punish its hop-path.
4. Greedy / swarms / DFS return valid connected start-goal paths whose cost
   is never below optimal, and agree with Dijkstra on reachability.
5. Wraparound: endpoints on opposite x-faces cost 1 wrapped vs 8 flat, and
   A\* still equals Dijkstra with wrap on (the heuristic stays admissible).
6. Void cells never appear in any closed set or path; a goal in a masked-out
   region reports no path.
7. Bidirectional Swarm returns a valid connected stitched path.
8. Stepped and batch execution match for **every** algorithm (including the
   bidirectional alternation).

The JS mirror suite additionally checks the domain presets (masks, carved
endpoints, torus wrap flags and that wrapping genuinely shortens routes) and
the route validator used by challenge mode.

## Hosting

Everything under `web/` is static.

- **Cloudflare Pages:** framework preset *None*, build command *(empty)* — or
  `wasm-pack build --target web --out-dir web/pkg` if the image has Rust —
  output directory `web`.
- **GitHub Pages:** serve the repo root and link to `/pathfind3d/web/`, or copy
  `web/` to `docs/` and enable Pages → `docs`.
- Any static file host works; the only requirements are HTTP (not `file://`)
  and, if you built the WASM core, a correct `.wasm` MIME type.
