# pathfind3d — volumetric A\* light-trail visualizer

A genuinely volumetric 3D pathfinding visualizer: a dense `N³` voxel grid where
cells are blocked or free independently along all three axes, searched by A\*
with full 26-connectivity. The explored set grows as a glowing 3D cloud you can
orbit around and see into; obstacles are perforated walls and floating clusters
suspended in space; the found route renders as a neon **light wall** — a
vertical ribbon extruded below the path curve — with a glowing cycle running
the path and the wall rezzing in behind it.

The solver is written in Rust and compiles to WebAssembly. The frontend is a
single static `web/index.html` (Three.js r128 from CDN, custom orbit camera,
no build step, no backend) that ships a line-for-line JS mirror of the Rust
solver, so the demo runs immediately and the WASM core drops in when built.

```
pathfind3d/
├── Cargo.toml
├── src/lib.rs        # A* solver + wasm-bindgen API + unit tests
├── web/index.html    # Three.js frontend + JS mirror solver
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

> If your Python serves `.wasm` with the wrong MIME type (rare, broken Windows
> registry associations), use `npx serve web` instead.

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

If the import fails (no `pkg/` built), the catch falls back to the JS mirror
and the demo keeps working.

## Solver API (identical in Rust/WASM and the JS mirror)

Index convention **everywhere**: `idx = x + N*(y + N*z)`.

| Method | Returns | Notes |
|---|---|---|
| `new Solver(n, blocked, sx,sy,sz, gx,gy,gz, diagonals)` | — | `blocked`: `Uint8Array` of `n³` (non-zero = obstacle). Start/goal cells are force-carved free. |
| `step_many(steps)` | — | Advance by at most `steps` node expansions (stepped execution for animation). |
| `state()` | `Uint8Array(n³)` | Per-cell: `FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4`. WASM returns a copy; the JS mirror returns a live read-only view. |
| `path()` | `Uint32Array` | Cell indices start→goal; empty until solved. |
| `expanded()` | `u32` | Unique nodes closed so far. |
| `frontier()` | `u32` | Current open-set size (unique cells). |
| `is_done()` | `bool` | True when solved **or** exhausted. |
| `found()` | `bool` | False + `is_done()` = **no path** (open set emptied). |
| `cost()` | `f64` | Total path cost; `NaN` until found. |
| `state_ptr()`, `state_len()` | ptr, `u32` | Zero-copy escape hatch, see below. |
| `free()` | — | wasm-bindgen destructor (no-op in the mirror). |

### Zero-copy `state()` for large N

Copying `N³` bytes per frame is fine at interactive sizes (26³ ≈ 17 KB). For
much larger grids, view the solver's cell buffer directly in wasm memory:

```js
const wasm = await import('./pkg/pathfind3d.js');
const { memory } = await wasm.default();
const view = new Uint8Array(memory.buffer, solver.state_ptr(), solver.state_len());
```

Rebuild the view after any call that could allocate — wasm memory growth
detaches old buffers. The shipped frontend uses plain `state()`.

## Algorithm

- A\* over a dense grid, **26-connectivity** (all `dx,dy,dz ∈ {−1,0,1}` minus
  origin), with a 6-connectivity (face-neighbour) toggle.
- Move cost is Euclidean: a step touching `k` axes costs `√k` → `1`, `√2`, `√3`.
- Heuristic is the exact minimum cost across an empty 26-connected grid —
  admissible and consistent, so A\* stays optimal. With absolute deltas sorted
  `a ≥ b ≥ c`: `h = (a−b)·1 + (b−c)·√2 + c·√3`. In 6-connected mode: Manhattan.
- `std::collections::BinaryHeap` has no decrease-key, so improved nodes are
  pushed as duplicates; a popped node whose cell is already CLOSED is skipped.
  The heap node carries a manual `Ord` (lowest `f`, tie-break lowest `h`,
  reversed for the max-heap) because `f64` isn't `Ord`.
- **No path** is detected and reported when the open set empties first.
- Diagonal moves may cut blocked corners — deliberate for a visualizer (paths
  read smoother). To forbid it, reject a diagonal step in `expand_from` /
  `_expand` unless its orthogonal sub-moves are free.
- Ties: JS and WASM cores always agree on `cost()` and on found/not-found;
  among equally-optimal paths the tie-breaking of heap-internal order may pick
  a different (equal-cost) path.

## Frontend

- **Controls:** Run/Pause, Step, Reset, Randomize; sliders for speed
  (expansions/frame), grid size (10³–26³), obstacle density; toggles for the
  explored cloud, auto-rotate, and 26- vs 6-connectivity. `SPACE` run/pause,
  `S` step, `R` randomize.
- **HUD:** expanded count, frontier size, path node count, path cost, status
  (`READY / SEARCHING / PATH FOUND / NO PATH`), active core, seed.
- **Scene:** instanced voxels for obstacles (matte, neon-rimmed), frontier
  (bright cyan) vs closed (deep blue) as an additive glowing cloud with
  rez-in flicker, Catmull-Rom path curve rendered as the amber light wall +
  travelling cycle, emissive start/goal markers, bounding cage, floor grid
  receding to a lit horizon. Custom orbit camera: drag to rotate, scroll or
  pinch to zoom, optional auto-rotate that stops on interaction.
- **Maze generator:** seeded PRNG (mulberry32) builds perforated full-section
  walls plus floating ellipsoidal clusters plus scatter, then carves the
  start/goal 26-neighbourhoods free so endpoints can never be boxed in. Same
  seed + sliders → same maze. High densities can legitimately produce NO PATH.
- Reduced motion (`prefers-reduced-motion`) disables flicker, auto-rotate, and
  the auto-traversing cycle (the wall appears fully built). No
  `localStorage`/`sessionStorage`; all state is in memory.

The aesthetic is an original light-grid / lightcycle homage — no logos,
characters, or vehicle designs are reproduced.

## Tests

```sh
cargo test
```

Six tests, including the four canonical ones:

1. 3³ empty grid, corner→corner, diagonals on: `cost == 2·√3`, path length 3.
2. Same, diagonals off: `cost == 6` (Manhattan), path length 7.
3. 5³ grid with the `z = 2` plane fully blocked except one central hole: the
   path passes through that hole (and costs exactly `4·√3`).
4. Goal corner walled off on all 26 approaches: `found()` is false.

Plus: stepped (`step_many(1)`) and batch execution produce identical results,
and returned paths are contiguous legal moves whose step costs sum to `cost()`.

The JS mirror passes the same four canonical tests plus maze-generator and
path-validity invariants (extract the block between `=== SOLVER CORE BEGIN/END
===` markers in `web/index.html` and run it under Node).

## Hosting

Everything under `web/` is static.

- **Cloudflare Pages:** framework preset *None*, build command *(empty)* — or
  `wasm-pack build --target web --out-dir web/pkg` if the image has Rust —
  output directory `web`.
- **GitHub Pages:** serve the repo root and link to `/pathfind3d/web/`, or copy
  `web/` to `docs/` and enable Pages → `docs`. GitHub Pages serves `.wasm` with
  the correct `application/wasm` MIME type.
- Any static file host works; the only requirements are HTTP (not `file://`)
  and, if you built the WASM core, a correct `.wasm` MIME type.
