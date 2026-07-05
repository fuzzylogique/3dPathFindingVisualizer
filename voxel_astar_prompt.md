# One-shot build: True volumetric 3D pathfinding visualizer (Rust + WASM)

Build a **genuinely volumetric** 3D pathfinding visualizer and deliver it as a
complete, hostable project in a single response. The core algorithm is written
in Rust and compiled to WebAssembly; the frontend renders it in real 3D with
Three.js and is fully static (no backend).

## The one constraint that matters

Almost every "3D" pathfinder online is fake: A* on a 2D grid with a tilted
camera, or a heightmap. **Do not build that.** This is a dense voxel grid where
cells are blocked or free independently along all three axes, agents move
*through* the volume, and the correctness test is visual:

- The explored set fills in as a **3D cloud you can orbit around and see into**,
  not a flat sheet.
- Obstacles are **volumetric** — floating blocks and perforated walls suspended
  in space — never a floor with height.
- The resulting path **threads up, down, and around** through 3D space; it does
  not trace a single surface.

If a reviewer rotates the camera and the scene collapses to a plane, the build
has failed.

## Stack

- **Pathfinding core:** Rust, compiled with `wasm-pack build --target web`
  (`wasm-bindgen`). `crate-type = ["cdylib", "rlib"]`.
- **Frontend:** single `index.html` using Three.js (CDN, e.g. r128 from cdnjs).
  Implement a small custom orbit camera (spherical coords: drag to rotate,
  scroll to zoom) rather than depending on an OrbitControls CDN path.
- **Hosting:** static. Deployable to GitHub Pages / Cloudflare Pages as-is.

## Algorithm spec (be precise — these are the details that get fumbled)

- Dense grid of side `N`. **Index convention everywhere:** `idx = x + N*(y + N*z)`.
- A* with **26-connectivity** (full 3D stencil: all `dx,dy,dz ∈ {−1,0,1}` minus
  origin). Provide a toggle for 6-connectivity (face neighbours only).
- **Move cost** between neighbours is Euclidean: for a step touching `k` axes
  (`k = |dx|+|dy|+|dz|`), cost is `√k` → `1`, `√2`, or `√3`.
- **Heuristic (admissible, so A* stays optimal):** the exact minimum cost across
  an empty 26-connected grid. Take the absolute axis deltas to the goal, sort
  them `a ≥ b ≥ c`, then

  ```
  h = (a − b)·1 + (b − c)·√2 + c·√3
  ```

  In 6-connected mode, use Manhattan (`a + b + c`).
- **Stepped execution** so the frontier can be animated: expose a way to advance
  the search by N expansions per call. Use `std::collections::BinaryHeap`; since
  it has no decrease-key, push duplicate entries and skip a popped node if it is
  already CLOSED.
- Detect and report **no path** (open set empties before reaching the goal).

## WASM API (frontend reads state through this)

Expose a `Solver` with roughly this surface:

```
Solver::new(n, blocked: &[u8], sx,sy,sz, gx,gy,gz, diagonals: bool)
  step_many(steps: u32)
  state()    -> Vec<u8>    // per-cell: FREE=0 OBST=1 OPEN=2 CLOSED=3 PATH=4
  path()     -> Vec<u32>   // cell indices, empty until solved
  expanded() -> u32
  frontier() -> u32
  is_done()  -> bool
  found()    -> bool
  cost()     -> f64
```

`state()` copying N³ bytes per frame is fine at interactive N. If you support
large N, expose a memory pointer for a zero-copy `Uint8Array` view and say so.

## Rendering spec

- Bounding cage for the volume (wireframe cube) and a faint floor grid for
  spatial reference.
- **Obstacles:** instanced cubes, matte/dark, generated as perforated walls with
  gaps plus scattered floating clusters so the optimal path is forced to detour
  in 3D. Seeded PRNG so "randomize" is reproducible.
- **Explored set:** instanced cubes, semi-transparent, additive blending, so the
  search reads as a glowing cloud. Distinguish **frontier (open)** from
  **closed** by colour.
- **Path:** a Catmull-Rom curve through the cell centres, rendered per the
  aesthetic direction (light-wall ribbon + travelling marker); highlight the
  path voxels brightly as well.
- **Start/goal:** distinct emissive markers.
- Use `InstancedMesh` for the voxel sets (grid up to ~22³ should stay smooth).
- Orbit camera with optional slow auto-rotate that stops on interaction.

## Controls (compact instrument panel)

Play / Pause, Step, Reset, Randomize; sliders for speed (expansions/frame),
grid size, obstacle density; toggles for show-explored-cloud, auto-rotate, and
diagonals on/off. Live HUD: expanded count, frontier size, path node count,
path cost, status (READY / SEARCHING / PATH FOUND / NO PATH).

## Aesthetic

**Direction: light-grid / lightcycle homage.** Build an original take on the
neon-grid, glowing-light-trail look — do not reproduce any real logo, named
characters, or specific vehicle designs; evoke the *style*, don't copy assets.
Lean into the fact that the theme maps onto the mechanics:

- **The path is a light wall, not a line.** Render the found route as a glowing
  vertical ribbon extruded below the path curve — a trail laid down through the
  volume. This is the signature element; make it the thing the piece is
  remembered for, and keep everything else quiet around it.
- **A cycle runs the solved path.** Once a path is found, animate a small glowing
  marker travelling along the curve with the light wall rezzing in behind it.
- **Voxels rez in.** The explored/frontier cubes should appear with a brief
  scanline or flicker rather than popping, so the search reads as energy
  propagating through the grid.
- **Staging:** dark void, a glowing grid floor plane receding toward a lit
  horizon; sharp neon wireframe edges on obstacles; heavy additive glow (a cheap
  bloom pass if feasible).
- **Colour:** a cool search energy (cyan/blue) against one hot accent
  (orange/amber) for the goal and/or trail — the classic two-team tension. Commit
  to it; don't dilute with a third neon.

Hold a quality floor regardless: legible HUD, responsive down to a phone,
visible focus states, reduced-motion respected (fall back to no flicker / no
auto-traverse when reduced motion is requested).

## Deliverables

```
pathfind3d/
├── Cargo.toml
├── src/lib.rs        # solver + wasm-bindgen API + #[cfg(test)] tests
├── web/index.html    # Three.js frontend
└── README.md         # build, host, and API notes
```

The frontend must run **without a build step** by shipping a JS `Solver` that
mirrors the Rust one exactly (same constructor args, same state constants), so
the demo works immediately and swapping in the compiled WASM is a three-line
change. Document that swap in the README.

## Verification (include these tests, they must pass)

Add `#[cfg(test)]` unit tests with known-optimal answers:

1. 3³ empty grid, corner (0,0,0) → (2,2,2), diagonals on: cost `== 2·√3`, path
   length 3.
2. Same, diagonals off: cost `== 6` (Manhattan).
3. 5³ grid with the `z = 2` plane fully blocked except one hole at its centre:
   the returned path must pass through that hole.
4. Goal corner walled off on all approaches: `found()` is false.

## Pitfalls to avoid (learned the hard way)

- In the neighbour-stencil loops, annotate the ranges as `i32` (e.g.
  `for dz in -1i32..=1`) or `.abs()` fails to resolve on an ambiguous numeric
  type.
- `f64` isn't `Ord`: give the heap node a manual `Ord` that orders by lowest `f`
  (tie-break lowest `h`) and reverses for `BinaryHeap`'s max-heap.
- Carve the start/goal cells (and their immediate neighbourhood) free after
  generating obstacles, or they get boxed in and every run reports no path.
- wasm needs an `application/wasm` MIME type — serve over http, not `file://`.
- No `localStorage`/`sessionStorage` in the artifact; keep all state in memory.
- Diagonal moves may cut blocked corners (fine for a visualizer, smoother paths).
  If you disallow it, check the orthogonal sub-moves are free before accepting a
  diagonal neighbour, and make it a toggle.

Produce the full project. Get the algorithm right the first time.
