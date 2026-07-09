// Node tests for the JS mirror solver embedded in web/index.html.
//
// The block between the `=== SOLVER CORE BEGIN/END ===` markers is
// dependency-free (no DOM, no THREE); this harness extracts it, appends
// exports, imports it as a module, and re-runs the same invariants the
// Rust suite in src/lib.rs checks — same expected costs, same semantics.
//
// Run with:  node tests/mirror.test.mjs

import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, '..', 'web', 'index.html'), 'utf8');

const beginTag = '=== SOLVER CORE BEGIN ===';
const endTag = '/* === SOLVER CORE END === */';
const beginAt = html.indexOf(beginTag);
const endAt = html.indexOf(endTag);
assert.ok(beginAt > 0 && endAt > beginAt, 'solver core markers must exist');
const afterBegin = html.indexOf('*/', beginAt) + 2;
const src = html.slice(afterBegin, endAt) + `
export {
  JsSolver, MinHeap, makeDomain, generateTerrain, pathCostOf, validateRoute,
  mulberry32, FREE, OBST, OPEN, CLOSED, PATH, T_VOID, T_OBST, W_MAX,
  ALGO_ASTAR, ALGO_DIJKSTRA, ALGO_GREEDY, ALGO_SWARM, ALGO_CONVERGENT,
  ALGO_BISWARM, ALGO_BFS, ALGO_DFS, SQRT_2, SQRT_3, STEP_COST,
};
`;
const core = await import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));
const {
  JsSolver, makeDomain, generateTerrain, pathCostOf, validateRoute, mulberry32,
  FREE, OPEN, CLOSED, PATH, T_VOID, T_OBST, W_MAX,
  ALGO_ASTAR, ALGO_DIJKSTRA, ALGO_GREEDY, ALGO_SWARM, ALGO_CONVERGENT,
  ALGO_BISWARM, ALGO_BFS, ALGO_DFS, SQRT_2, SQRT_3,
} = core;

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

const near = (a, b, eps = 1e-9) => Math.abs(a - b) < eps;
const idx = (nx, ny, x, y, z) => x + nx * (y + ny * z);

function run(s) {
  for (let i = 0; i < 1e6 && !s.is_done(); i++) s.step_many(1024);
  assert.ok(s.is_done(), 'search did not terminate');
  return s;
}

function cube(n, terrain, s, g, diagonals, algo) {
  return run(new JsSolver(n, n, n, terrain, false, false, false,
    s[0], s[1], s[2], g[0], g[1], g[2], diagonals, algo));
}

function uniform(total) { return new Uint8Array(total).fill(1); }

function randTerrain(total, seed, obstPct, weighted) {
  const rnd = mulberry32(seed);
  const t = new Uint8Array(total);
  for (let i = 0; i < total; i++) {
    if (rnd() * 100 < obstPct) t[i] = T_OBST;
    else t[i] = weighted ? 1 + ((rnd() * W_MAX) | 0) : 1;
  }
  return t;
}

function domView(s) {
  return { nx: s.nx, ny: s.ny, nz: s.nz, wrap: s.wrap, terrain: s.terrain };
}

// Path validity + cost() must equal the re-summed weighted path cost.
function assertValidPath(s) {
  const p = Array.from(s.path());
  const v = validateRoute(domView(s), p, s.start(), s.goal(), s.diagonals);
  assert.ok(v.ok, `path invalid: ${v.reason}`);
  assert.ok(near(pathCostOf(domView(s), p), s.cost()), 'cost() != re-summed path cost');
}

/* ---- edition-1 invariants ------------------------------------------ */

test('empty 3³, diagonals: cost 2·√3, path length 3', () => {
  const s = cube(3, uniform(27), [0, 0, 0], [2, 2, 2], true, ALGO_ASTAR);
  assert.ok(s.found());
  assert.ok(near(s.cost(), 2 * SQRT_3));
  assert.equal(s.path().length, 3);
  assert.equal(s.path()[0], idx(3, 3, 0, 0, 0));
  assert.equal(s.path()[2], idx(3, 3, 2, 2, 2));
});

test('empty 3³, no diagonals: Manhattan cost 6, path length 7', () => {
  const s = cube(3, uniform(27), [0, 0, 0], [2, 2, 2], false, ALGO_ASTAR);
  assert.ok(s.found());
  assert.ok(near(s.cost(), 6));
  assert.equal(s.path().length, 7);
});

test('threads through the single hole in a blocked plane', () => {
  const n = 5, t = uniform(125);
  for (let x = 0; x < n; x++) for (let y = 0; y < n; y++) {
    if (!(x === 2 && y === 2)) t[idx(n, n, x, y, 2)] = T_OBST;
  }
  const s = cube(n, t, [0, 0, 0], [4, 4, 4], true, ALGO_ASTAR);
  assert.ok(s.found());
  assert.ok(Array.from(s.path()).includes(idx(n, n, 2, 2, 2)));
  assert.ok(near(s.cost(), 4 * SQRT_3));
  assertValidPath(s);
});

test('walled-off goal reports no path', () => {
  const n = 4, t = uniform(64);
  for (let x = 2; x < n; x++) for (let y = 2; y < n; y++) for (let z = 2; z < n; z++) {
    if (!(x === 3 && y === 3 && z === 3)) t[idx(n, n, x, y, z)] = T_OBST;
  }
  const s = cube(n, t, [0, 0, 0], [3, 3, 3], true, ALGO_ASTAR);
  assert.ok(s.is_done() && !s.found());
  assert.equal(s.path().length, 0);
});

test('stepped execution matches batch for every algorithm', () => {
  const n = 8, terrain = randTerrain(512, 42, 25, true);
  for (const algo of [ALGO_ASTAR, ALGO_DIJKSTRA, ALGO_GREEDY, ALGO_SWARM,
                      ALGO_CONVERGENT, ALGO_BISWARM, ALGO_BFS, ALGO_DFS]) {
    const mk = () => new JsSolver(n, n, n, terrain, false, false, false,
      0, 0, 0, 7, 7, 7, true, algo);
    const a = mk(), b = mk();
    while (!a.is_done()) a.step_many(1);
    b.step_many(1e6);
    assert.equal(a.found(), b.found(), `algo ${algo}: found`);
    assert.equal(a.expanded(), b.expanded(), `algo ${algo}: expanded`);
    if (a.found()) {
      assert.ok(near(a.cost(), b.cost()), `algo ${algo}: cost`);
      assert.deepEqual(Array.from(a.path()), Array.from(b.path()), `algo ${algo}: path`);
    }
  }
});

/* ---- weighted cells -------------------------------------------------- */

test('Dijkstra == A* on weighted random grids', () => {
  for (const seed of [7, 99, 1234, 555, 2026]) {
    const terrain = randTerrain(512, seed, 20, true);
    const a = cube(8, terrain, [0, 0, 0], [7, 7, 7], true, ALGO_ASTAR);
    const d = cube(8, terrain, [0, 0, 0], [7, 7, 7], true, ALGO_DIJKSTRA);
    assert.equal(a.found(), d.found(), `seed ${seed}`);
    if (a.found()) {
      assert.ok(near(a.cost(), d.cost()), `seed ${seed}: ${a.cost()} != ${d.cost()}`);
      assertValidPath(a);
      assertValidPath(d);
    }
  }
});

test('heavy cell forces a detour; a light one does not', () => {
  const [nx, ny, nz] = [5, 3, 3];
  const mid = idx(nx, ny, 2, 1, 1);
  for (const [w, want, through] of [[10, 6, false], [2, 5, true]]) {
    const t = uniform(nx * ny * nz);
    t[mid] = w;
    const s = run(new JsSolver(nx, ny, nz, t, false, false, false,
      0, 1, 1, 4, 1, 1, false, ALGO_ASTAR));
    assert.ok(s.found());
    assert.ok(near(s.cost(), want), `w=${w}: cost ${s.cost()} want ${want}`);
    assert.equal(Array.from(s.path()).includes(mid), through, `w=${w}`);
    assertValidPath(s);
  }
});

/* ---- BFS/DFS hop semantics ------------------------------------------- */

test('BFS returns fewest hops, not least cost', () => {
  const t = uniform(9);
  t[idx(3, 3, 1, 1, 0)] = 10; // heavy centre of a 3×3×1 board
  const mk = algo => run(new JsSolver(3, 3, 1, t, false, false, false,
    0, 0, 0, 2, 2, 0, true, algo));
  const bfs = mk(ALGO_BFS), astar = mk(ALGO_ASTAR);
  assert.equal(bfs.path().length, 3, 'fewest hops is 2 (3 cells)');
  assert.ok(astar.path().length > bfs.path().length);
  assert.ok(astar.cost() < bfs.cost());
  assertValidPath(bfs);
});

test('BFS fewest hops, hand-checked empty 4³', () => {
  assert.equal(cube(4, uniform(64), [0, 0, 0], [3, 3, 3], true, ALGO_BFS).path().length, 4);
  assert.equal(cube(4, uniform(64), [0, 0, 0], [3, 3, 3], false, ALGO_BFS).path().length, 10);
});

/* ---- non-optimal algorithms stay honest ------------------------------ */

test('greedy/swarms/BFS/DFS: valid connected paths, never beating optimal', () => {
  for (const seed of [3, 77, 909]) {
    for (const diagonals of [true, false]) {
      const terrain = randTerrain(512, seed, 18, true);
      const d = cube(8, terrain, [0, 0, 0], [7, 7, 7], diagonals, ALGO_DIJKSTRA);
      const opt = d.found() ? d.cost() : NaN;
      for (const algo of [ALGO_GREEDY, ALGO_SWARM, ALGO_CONVERGENT,
                          ALGO_BISWARM, ALGO_BFS, ALGO_DFS]) {
        const s = cube(8, terrain, [0, 0, 0], [7, 7, 7], diagonals, algo);
        assert.equal(s.found(), d.found(), `seed ${seed} algo ${algo}: reachability`);
        if (d.found()) {
          assertValidPath(s);
          assert.ok(s.cost() >= opt - 1e-9, `seed ${seed} algo ${algo}: ${s.cost()} < ${opt}`);
        }
      }
    }
  }
});

test('bidirectional swarm returns a valid connected path', () => {
  const terrain = randTerrain(1000, 4242, 22, true);
  const s = cube(10, terrain, [0, 0, 0], [9, 9, 9], true, ALGO_BISWARM);
  assert.ok(s.found());
  assertValidPath(s);
});

/* ---- wraparound ------------------------------------------------------- */

test('wrap_x shortcut: cost 1 wrapped vs 8 flat', () => {
  const t = uniform(9 * 3 * 3);
  const flat = run(new JsSolver(9, 3, 3, t, false, false, false, 0, 1, 1, 8, 1, 1, false, ALGO_ASTAR));
  const wrapped = run(new JsSolver(9, 3, 3, t, true, false, false, 0, 1, 1, 8, 1, 1, false, ALGO_ASTAR));
  assert.ok(near(flat.cost(), 8));
  assert.ok(near(wrapped.cost(), 1));
  assertValidPath(wrapped);
});

test('A* stays optimal with wrap on weighted grids (admissible heuristic)', () => {
  for (const seed of [11, 313, 9000]) {
    const terrain = randTerrain(9 * 7 * 8, seed, 20, true);
    const mk = algo => run(new JsSolver(9, 7, 8, terrain, true, true, true,
      0, 0, 0, 8, 6, 7, true, algo));
    const a = mk(ALGO_ASTAR), d = mk(ALGO_DIJKSTRA);
    assert.equal(a.found(), d.found());
    if (a.found()) {
      assert.ok(near(a.cost(), d.cost()), `seed ${seed}: ${a.cost()} != ${d.cost()}`);
      assertValidPath(a);
    }
  }
});

/* ---- domain mask ------------------------------------------------------- */

test('void cells never enter the search; masked-out goal is unreachable', () => {
  const n = 9, total = n * n * n, c = 4;
  const inside = (x, y, z) => Math.hypot(x - c, y - c, z - c) <= 4.2;
  const t = new Uint8Array(total); // T_VOID
  for (let z = 0; z < n; z++) for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
    if (inside(x, y, z)) t[idx(n, n, x, y, z)] = 1 + ((x + y + z) % 3);
  }
  for (const algo of [ALGO_ASTAR, ALGO_BFS, ALGO_DFS, ALGO_BISWARM]) {
    const s = cube(n, t, [4, 4, 0], [4, 4, 8], true, algo);
    assert.ok(s.found(), `algo ${algo}`);
    const st = s.state(), p = new Set(s.path());
    for (let i = 0; i < total; i++) {
      if (t[i] === T_VOID) {
        assert.equal(st[i], FREE, `algo ${algo}: void cell ${i} searched`);
        assert.ok(!p.has(i));
      }
    }
    assertValidPath(s);
  }
  const s = cube(n, t, [4, 4, 0], [0, 0, 0], true, ALGO_ASTAR);
  assert.ok(s.is_done() && !s.found() && s.path().length === 0);
});

test('start == goal is a zero-cost single-cell path', () => {
  const s = cube(3, uniform(27), [1, 1, 1], [1, 1, 1], true, ALGO_ASTAR);
  assert.ok(s.is_done() && s.found());
  assert.deepEqual(Array.from(s.path()), [idx(3, 3, 1, 1, 1)]);
  assert.ok(near(s.cost(), 0, 1e-12));
});

/* ---- domain presets & terrain generator -------------------------------- */

test('every domain preset yields existing, unblocked endpoints', () => {
  for (const preset of ['cube', 'prism', 'pyramid', 'sphere', 'torus']) {
    const dom = makeDomain(preset, 22);
    assert.ok(dom.count > 0, `${preset}: empty mask`);
    const sIdx = dom.start[0] + dom.nx * (dom.start[1] + dom.ny * dom.start[2]);
    const gIdx = dom.goal[0] + dom.nx * (dom.goal[1] + dom.ny * dom.goal[2]);
    assert.ok(dom.exists[sIdx], `${preset}: start in mask`);
    assert.ok(dom.exists[gIdx], `${preset}: goal in mask`);
    const terrain = generateTerrain(dom, 18, 41, false);
    assert.equal(terrain[sIdx], 1, `${preset}: start carved clean`);
    assert.equal(terrain[gIdx], 1, `${preset}: goal carved clean`);
    for (let i = 0; i < terrain.length; i++) {
      if (!dom.exists[i]) assert.equal(terrain[i], T_VOID, `${preset}: mask violated`);
      else assert.ok(terrain[i] === T_OBST || (terrain[i] >= 1 && terrain[i] <= W_MAX));
    }
    // and the generated world is solvable end to end
    const s = run(new JsSolver(dom.nx, dom.ny, dom.nz, terrain,
      dom.wrap[0], dom.wrap[1], dom.wrap[2],
      dom.start[0], dom.start[1], dom.start[2],
      dom.goal[0], dom.goal[1], dom.goal[2], true, ALGO_ASTAR));
    assert.ok(s.is_done(), `${preset}: terminated`);
  }
});

test('torus preset wraps x and the wrap makes routes shorter', () => {
  const dom = makeDomain('torus', 22);
  assert.deepEqual(dom.wrap, [true, false, false]);
  const t = generateTerrain(dom, 0, 1, true); // empty ground
  const mk = wrapX => run(new JsSolver(dom.nx, dom.ny, dom.nz, t,
    wrapX, false, false,
    dom.start[0], dom.start[1], dom.start[2],
    dom.goal[0], dom.goal[1], dom.goal[2], true, ALGO_ASTAR));
  const wrapped = mk(true), flat = mk(false);
  assert.ok(wrapped.found() && flat.found());
  assert.ok(wrapped.cost() < flat.cost(), 'wrapping must beat the long way round');
  assertValidPath(wrapped);
});

test('pyramid cross-section shrinks with height', () => {
  const dom = makeDomain('pyramid', 22);
  const rowWidth = y => {
    let w = 0;
    for (let x = 0; x < dom.nx; x++) {
      if (dom.exists[x + dom.nx * (y + dom.ny * Math.floor(dom.nz / 2))]) w++;
    }
    return w;
  };
  assert.ok(rowWidth(0) > rowWidth(dom.ny - 1), 'base must be wider than apex');
  assert.ok(rowWidth(dom.ny - 1) >= 3, 'apex must still be walkable');
});

test('empty generation leaves no obstacles; validateRoute flags bad routes', () => {
  const dom = makeDomain('cube', 10);
  const t = generateTerrain(dom, 30, 7, true);
  for (let i = 0; i < t.length; i++) assert.ok(t[i] !== T_OBST);
  const dv = { nx: 10, ny: 10, nz: 10, wrap: [false, false, false], terrain: t };
  const a = 0, b = idx(10, 10, 1, 0, 0), far = idx(10, 10, 5, 5, 5);
  assert.ok(validateRoute(dv, [a, b], a, b, true).ok);
  assert.ok(!validateRoute(dv, [a, far], a, far, true).ok, 'jump must fail');
  t[b] = T_OBST;
  assert.ok(!validateRoute(dv, [a, b], a, b, true).ok, 'obstacle must fail');
});

console.log(`\n${passed} tests passed${process.exitCode ? ' (with failures)' : ''}`);
