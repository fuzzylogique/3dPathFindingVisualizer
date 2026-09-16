// Tests for the wasm boundary.
//
// The algorithms themselves are covered by `cargo test`, which runs the same
// Rust the browser runs. What cannot be checked there is the crossing: that
// the exported `World` API behaves from JavaScript, that the pointer-plus-
// length buffers line up, and that views survive a heap that grows underneath
// them. Those are exactly the failures that would ship silently.
//
// Run with:  node tests/core.test.mjs   (after wasm-pack build)

import { readFileSync } from 'node:fs';
import assert from 'node:assert/strict';
import init, { World } from '../web/pkg/pathfind3d.js';

const bytes = readFileSync(new URL('../web/pkg/pathfind3d_bg.wasm', import.meta.url));
const wasm = await init({ module_or_path: bytes });

const PRESETS = ['cube', 'prism', 'pyramid', 'sphere', 'torus'];
const T_VOID = 0, T_OBST = 255;
const FREE = 0, OBST = 1, OPEN = 2, CLOSED = 3, PATH = 4;

let passed = 0;
function test(name, fn) {
  try {
    fn();
    passed++;
    console.log(`ok  ${name}`);
  } catch (err) {
    console.error(`FAIL ${name}`);
    console.error(err);
    process.exitCode = 1;
  }
}

/** Views must be re-derived after any call that can allocate, exactly as the
 *  front end does. This mirrors `mem` in web/js/main.js. */
function views(w) {
  const n = w.len(), b = wasm.memory.buffer;
  return {
    state: new Uint8Array(b, w.state_ptr(), n),
    terrain: new Uint8Array(b, w.terrain_ptr(), n),
    exists: new Uint8Array(b, w.exists_ptr(), n),
    px: new Float32Array(b, w.pos_x_ptr(), n),
    py: new Float32Array(b, w.pos_y_ptr(), n),
    pz: new Float32Array(b, w.pos_z_ptr(), n),
    rot: new Float32Array(b, w.rot_ptr(), n),
  };
}

test('every preset exposes buffers of the right length and finite positions', () => {
  for (const preset of PRESETS) {
    const w = new World(preset, 16, 18, 41, false);
    const n = w.len();
    assert.equal(n, w.nx() * w.ny() * w.nz(), `${preset}: len must be nx*ny*nz`);
    assert.ok(w.cell_count() > 0 && w.cell_count() <= n, `${preset}: cell count`);
    const v = views(w);
    assert.equal(v.state.length, n);
    let existing = 0;
    for (let i = 0; i < n; i++) {
      assert.ok(Number.isFinite(v.px[i]) && Number.isFinite(v.py[i]) && Number.isFinite(v.pz[i]),
        `${preset}: cell ${i} has a non-finite position`);
      if (v.exists[i]) existing++;
      else assert.equal(v.terrain[i], T_VOID, `${preset}: void cell carries terrain`);
    }
    assert.equal(existing, w.cell_count(), `${preset}: mask disagrees with cell_count`);
    w.free();
  }
});

test('only the torus wraps, and only on x', () => {
  for (const preset of PRESETS) {
    const w = new World(preset, 16, 0, 1, true);
    assert.deepEqual(Array.from(w.wraps()),
      preset === 'torus' ? [1, 0, 0] : [0, 0, 0], preset);
    w.free();
  }
});

test('the torus is the only curved domain and its cells carry rotations', () => {
  const flat = new World('cube', 14, 0, 1, true);
  assert.equal(flat.curved(), false);
  assert.ok(Array.from(views(flat).rot).every(r => r === 0));
  flat.free();

  const ring = new World('torus', 14, 0, 1, true);
  assert.equal(ring.curved(), true);
  const v = views(ring);
  assert.ok(Array.from(v.rot).some(r => r !== 0), 'ring cells must be rotated');
  ring.free();
});

test('a full run reaches the goal and marks the path in the state buffer', () => {
  const w = new World('cube', 16, 18, 41, false);
  w.seek(w.total_steps());
  assert.ok(w.is_done());
  assert.ok(w.found(), 'the generator guarantees a solvable world');
  const path = w.path();
  assert.ok(path.length >= 2);
  assert.equal(path[0], w.start_cell());
  assert.equal(path[path.length - 1], w.goal_cell());
  const st = views(w).state;
  for (const i of path) assert.equal(st[i], PATH, `cell ${i} should read PATH`);
  assert.ok(Number.isFinite(w.cost()) && w.cost() > 0);
  w.free();
});

test('state buffer only ever holds the five documented values', () => {
  const w = new World('sphere', 16, 22, 9, false);
  w.seek(Math.floor(w.total_steps() / 2));
  const st = views(w).state;
  const seen = new Set(st);
  for (const s of seen) {
    assert.ok([FREE, OBST, OPEN, CLOSED, PATH].includes(s), `unexpected state ${s}`);
  }
  w.free();
});

test('seek and step_back are exact, and views survive the reallocation', () => {
  const w = new World('cube', 18, 20, 7, false);
  const mid = Math.floor(w.total_steps() / 2);
  assert.ok(mid > 2, 'need a search long enough to scrub');

  w.seek(mid);
  const snapshot = Uint8Array.from(views(w).state); // a copy, not a view
  assert.equal(w.expanded(), mid);

  w.seek(w.total_steps());       // reallocates the solver's buffers
  w.seek(mid);                   // and again
  const after = views(w).state;  // re-derived, as the front end does
  assert.equal(after.length, snapshot.length, 'the view must still be the right size');
  assert.deepEqual(Array.from(after), Array.from(snapshot),
    'scrubbing back to a step must reproduce that step exactly');

  w.advance(1);
  w.step_back(1);
  assert.equal(w.expanded(), mid);
  assert.deepEqual(Array.from(views(w).state), Array.from(snapshot));
  w.free();
});

test('seeking past the end stops at the end rather than overrunning', () => {
  const w = new World('cube', 14, 18, 3, false);
  const n = w.total_steps();
  w.seek(n + 10000);
  assert.equal(w.expanded(), n);
  assert.ok(w.is_done());
  w.free();
});

test('every algorithm terminates and none beats the optimum', () => {
  const w = new World('cube', 14, 20, 2026, false);
  const opt = w.optimal_cost();
  assert.ok(Number.isFinite(opt), 'the reference world must be solvable');
  for (let algo = 0; algo <= 7; algo++) {
    w.set_algo(algo);
    w.seek(w.total_steps());
    assert.ok(w.is_done(), `algo ${algo} did not terminate`);
    assert.ok(w.found(), `algo ${algo} should find the same reachable goal`);
    assert.ok(w.cost() >= opt - 1e-9, `algo ${algo} cost ${w.cost()} beats optimal ${opt}`);
  }
  w.free();
});

test('painting updates the terrain, the instance lists and the caches', () => {
  const w = new World('cube', 14, 0, 5, true); // empty world: no walls at all
  assert.equal(w.obstacle_cells().length, 0);
  assert.equal(w.weighted_cells().length, 0);
  const before = w.total_steps();

  const cell = w.nx() * w.ny() * 4 + 30; // some interior cell
  assert.ok(w.paint(cell, T_OBST));
  assert.equal(views(w).terrain[cell], T_OBST);
  assert.deepEqual(Array.from(w.obstacle_cells()), [cell]);
  assert.deepEqual(Array.from(w.surface_obstacle_cells()), [cell],
    'a lone wall is entirely surface');
  assert.ok(w.is_stale(), 'painting must flag the displayed search as stale');

  assert.ok(w.paint(cell, 6));
  assert.deepEqual(Array.from(w.weighted_cells()), [cell]);
  assert.equal(w.obstacle_cells().length, 0);

  w.paint(cell, 1);
  w.reset_search();
  assert.equal(w.total_steps(), before, 'an undone edit restores the search length');
  w.free();
});

test('the endpoints cannot be walled in and cannot collide', () => {
  const w = new World('cube', 14, 18, 41, false);
  assert.equal(w.paint(w.start_cell(), T_OBST), false);
  assert.equal(w.paint(w.goal_cell(), T_OBST), false);
  const goal = w.goal_cell();
  const landed = w.set_start(goal);
  assert.ok(landed >= 0);
  assert.notEqual(landed, goal, 'the start must be pushed off the goal');
  assert.equal(w.start_cell(), landed);
  w.free();
});

test('moving an endpoint changes where the search ends', () => {
  const w = new World('cube', 14, 0, 5, true);
  w.seek(w.total_steps());
  const firstGoal = w.goal_cell();
  const firstCost = w.cost();

  const moved = w.set_goal(w.nx() * w.ny() * 2 + w.nx() * 2 + 2);
  assert.ok(moved >= 0);
  assert.notEqual(moved, firstGoal);
  w.reset_search();
  w.seek(w.total_steps());
  assert.equal(w.path()[w.path().length - 1], moved, 'the path must end at the new goal');
  assert.notEqual(w.cost(), firstCost);
  w.free();
});

test('a ray fired at a wall picks that wall, and the cell in front of it', () => {
  const w = new World('cube', 14, 0, 5, true);
  const target = w.nx() * w.ny() * 7 + w.nx() * 7 + 7; // (7,7,7)
  w.paint(target, T_OBST);
  const v = views(w);
  // Straight down +x from well outside the domain.
  const [solid, build] = w.pick_edit(
    v.px[target] - 40, v.py[target], v.pz[target], 1, 0, 0, false);
  assert.equal(solid, target, 'the wall must be picked');
  assert.ok(build >= 0 && build !== target, 'and a build cell in front of it');
  assert.equal(views(w).terrain[build], 1, 'the build cell must be open ground');
  w.free();
});

test('picking works through the curved torus frame', () => {
  const w = new World('torus', 14, 0, 5, true);
  const v = views(w);
  // The highest existing cell, aimed at from directly above.
  let top = -1, topY = -Infinity;
  for (let i = 0; i < w.len(); i++) {
    if (v.exists[i] && v.py[i] > topY) { topY = v.py[i]; top = i; }
  }
  w.paint(top, T_OBST);
  const v2 = views(w);
  const [solid] = w.pick_edit(v2.px[top], v2.py[top] + 30, v2.pz[top], 0, -1, 0, false);
  assert.equal(solid, top);
  w.free();
});

test('a ray into the void picks nothing rather than guessing', () => {
  const w = new World('cube', 12, 0, 1, true);
  const [solid, build] = w.pick_edit(0, 900, 0, 1, 0, 0, false);
  assert.equal(solid, -1);
  assert.equal(build, -1);
  w.free();
});

test('a drawn route is bridged, validated and scored against the optimum', () => {
  const w = new World('cube', 14, 12, 77, false);
  w.route_begin();
  assert.equal(w.route_len(), 1);
  assert.equal(w.route_error(), 'must reach the goal');

  assert.equal(w.route_append(w.goal_cell()), 0);
  assert.ok(w.route_complete());
  assert.equal(w.route_error(), undefined, 'a bridged route must be legal');

  const route = w.route();
  assert.equal(route[0], w.start_cell());
  assert.equal(route[route.length - 1], w.goal_cell());
  const cost = w.route_cost();
  assert.ok(Number.isFinite(cost));
  assert.ok(cost >= w.optimal_cost() - 1e-9, 'a drawn route cannot beat the optimum');
  w.free();
});

test('route_append refuses walls and reports why', () => {
  const w = new World('cube', 14, 25, 11, false);
  const wall = w.obstacle_cells()[0];
  assert.ok(wall !== undefined, 'this density should produce walls');
  w.route_begin();
  assert.equal(w.route_append(wall), -1, 'ROUTE_BLOCKED');
  w.free();
});

test('route_undo drops a whole bridged leg', () => {
  const w = new World('cube', 14, 0, 5, true);
  w.route_begin();
  w.route_append(w.nx() * w.ny() * 7 + w.nx() * 7 + 7);
  const afterLeg = w.route_len();
  assert.ok(afterLeg > 1);
  w.route_append(w.goal_cell());
  assert.ok(w.route_len() > afterLeg);
  w.route_undo();
  assert.equal(w.route_len(), afterLeg);
  w.route_undo();
  assert.equal(w.route_len(), 1);
  w.free();
});

test('rebuild keeps the world consistent and reseeds reproducibly', () => {
  const w = new World('cube', 14, 20, 1, false);
  w.rebuild('pyramid', 18, 25, 999, false);
  assert.equal(w.preset(), 'pyramid');
  assert.equal(w.seed(), 999);
  assert.equal(w.len(), w.nx() * w.ny() * w.nz());
  const a = Array.from(views(w).terrain);

  w.rebuild('cube', 14, 20, 1, false);
  w.rebuild('pyramid', 18, 25, 999, false);
  assert.deepEqual(Array.from(views(w).terrain), a, 'a seed must reproduce its world');
  w.free();
});

test('clear_terrain leaves open ground everywhere inside the mask', () => {
  const w = new World('sphere', 16, 30, 4, false);
  w.clear_terrain();
  const v = views(w);
  for (let i = 0; i < w.len(); i++) {
    assert.equal(v.terrain[i], v.exists[i] ? 1 : T_VOID, `cell ${i}`);
  }
  assert.equal(w.obstacle_cells().length, 0);
  assert.equal(w.weighted_cells().length, 0);
  w.free();
});

test('bbox covers every existing cell', () => {
  for (const preset of PRESETS) {
    const w = new World(preset, 14, 0, 1, true);
    const b = w.bbox();
    const v = views(w);
    for (let i = 0; i < w.len(); i++) {
      if (!v.exists[i]) continue;
      assert.ok(v.px[i] >= b[0] - 1e-4 && v.px[i] <= b[3] + 1e-4, `${preset}: x`);
      assert.ok(v.py[i] >= b[1] - 1e-4 && v.py[i] <= b[4] + 1e-4, `${preset}: y`);
      assert.ok(v.pz[i] >= b[2] - 1e-4 && v.pz[i] <= b[5] + 1e-4, `${preset}: z`);
    }
    w.free();
  }
});

console.log(`\n${passed} tests passed${process.exitCode ? ' (with failures)' : ''}`);
