# 3D Pathfinder

An interactive pathfinding visualizer that searches in **three dimensions**. Eight
algorithms explore a voxel world you can orbit around, look inside and edit.
You watch each search spread outward as a glowing cloud, and the route it
finds lights up as a neon trail.

The search engine is written in Rust and compiled to WebAssembly. The visuals
are rendered in the browser with Three.js.

**[Try it live →](https://fuzzylogique.github.io/3dPathFindingVisualizer/)**

## Why I built it

Pathfinding visualizers are a popular way to learn how A\*, Dijkstra and
breadth-first search work, and there are plenty of good ones. Almost all of
them are **flat**: a 2D grid of squares where you draw walls and watch the
search fill in the gaps.

I went looking for a 3D version and found very little. That seemed like a
real gap, because a third dimension changes how the algorithms behave:

- **Every cell has 26 neighbours instead of 8.** A search has far more
  directions to try, so the difference between a well-guided search (A\*) and
  an unguided one (Dijkstra, BFS) becomes much easier to see.
- **Obstacles are volumes, not lines.** A wall in 3D can be walked over,
  under or around, so a detour doesn't have to stay in a single plane.
- **The world doesn't have to be a box.** A sphere or a pyramid changes which
  routes are available, and on a torus a route can leave one side of the
  world and come back in on the other.

The hard part of a 3D visualizer is that you can't see inside a solid block
of cells, and you can't tell which cell you're clicking on. A lot of this
project is about solving those two problems.

## What you can do

- **Pick an algorithm.** A\*, Dijkstra, Greedy Best-First, three Swarm
  variants, Breadth-First and Depth-First Search.
- **Pick a world.** Cube, rectangular prism, pyramid, sphere, or a torus whose
  ends connect.
- **Move the start and goal.** Drag either glowing marker anywhere in the
  world.
- **Build obstacles.** Place walls, or paint weighted terrain that costs more
  to cross, and see which algorithms route around it and which push through.
- **Control time.** Run, pause, step one move forward or back, or drag a
  scrub bar to any point in the search.
- **See inside.** Walls fade to x-ray while a search runs, and a clipping
  plane slices the world open along any axis.
- **Race the algorithm.** Draw your own route from start to goal and see how
  its cost compares with the true shortest path.

## How it works

### Searching in 3D

The world is a grid of cubes called voxels. Each voxel is either void (outside
the world's shape), a wall, open ground, or weighted ground with a cost from 2
to 10.

A search moves from a cell to any of its 26 neighbours. A straight step costs
1, a diagonal across one face costs √2, and a diagonal through a corner costs
√3. That base cost is multiplied by the weight of the cell being entered, so
heavy terrain is expensive to cross.

All eight algorithms share a single stepping loop. They differ in only two
things: the order in which they choose the next cell, and how much they're
drawn toward the goal.

| Algorithm | How it chooses the next cell | Shortest path guaranteed? |
|---|---|---|
| A\* | cost so far + estimated distance to the goal | yes |
| Dijkstra | cost so far only | yes |
| Greedy Best-First | estimated distance to the goal only | no |
| Swarm / Convergent Swarm | like A\*, but pulled harder toward the goal | no |
| Bidirectional Swarm | two searches, one from each end, that meet in the middle | no |
| Breadth-First | fewest steps, ignoring weights | fewest steps only |
| Depth-First | dives down one branch as far as it can | no |

The distance estimate is calculated so that it can never overestimate the
real cost. That is what guarantees A\* finds the shortest path, and it needs
extra care on the torus, where the short way round might go through the
seam.

### Rust does the work, the browser draws it

All of the logic lives in Rust: the shape of each world, generating
obstacles, running the searches, scoring routes, and working out which cell
the mouse is pointing at. It compiles to WebAssembly and runs in the browser
at close to native speed.

The JavaScript layer only draws. Each frame it reads the state of every cell
straight out of the WebAssembly memory, without copying, and renders it with
Three.js. Because the model exists in exactly one place, what you see can't
drift out of sync with what the algorithm is doing.

### Clicking into a 3D world

In 2D, a click lands on exactly one square. In 3D, a click could mean any of
the cells stacked up behind the cursor.

To resolve this, the mouse position becomes a ray pointing into the world.
The Rust core steps along that ray a fifth of a cell at a time, turning each
point back into grid coordinates, until it hits something. The cell it hits
is what a click would remove, and the open cell just in front of it is where
a click would build. Before you click, a highlight shows you which cell that
is.

On the torus, the grid is bent into a ring. The same method still works,
because each point along the ray is converted back through the ring shape
rather than assuming a flat grid.

### Rewinding a search

Every algorithm is deterministic: given the same world, it makes the same
choices every time. So instead of recording a history to step backward
through, rewinding simply replays the search from the beginning up to the
chosen step. A complete search replays in a few milliseconds, which is fast
enough to drag the scrub bar smoothly.

## The look

The visual style is inspired by **Tron Legacy**: a glowing grid floor
stretching to a lit horizon, cyan and amber neon on near-black, and the found
path drawn as a light wall with a light cycle riding along it. Explored cells
flicker in as they're discovered, like the Grid coming online.

It's an original tribute to the film's aesthetic. No logos, characters or
vehicle designs are reproduced.

## Running it locally

You'll need [Rust](https://rustup.rs), [wasm-pack](https://rustwasm.github.io/wasm-pack/)
and [Node.js](https://nodejs.org).

```sh
cd pathfind3d
rustup target add wasm32-unknown-unknown
cargo install wasm-pack
wasm-pack build --target web --out-dir web/pkg --release
npx serve web
```

Then open the address it prints.

## Built with

- **Rust**, compiled to **WebAssembly** with wasm-bindgen: the search engine
  and all computation
- **Three.js**: 3D rendering
- **HTML and CSS**: the interface

For the API, the full list of controls, and how to run the test suites, see
the [technical README](pathfind3d/README.md).
