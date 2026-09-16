/* Boot and frame loop.

   The whole model — domain, terrain, endpoints, search, routes — lives in the
   Rust `World`. This file owns three things the core cannot: when to ask it to
   advance, how to get its buffers on screen, and what a button press means. */

import init, { World } from '../pkg/pathfind3d.js';
import { createStage, REDUCED, clamp } from './scene.js';
import { Volume, updateClipPlane } from './volume.js';
import { Trails } from './trails.js';
import { attachInput } from './input.js';
import { createPanel, showError } from './ui.js';

const BUILD_HINT = 'wasm-pack build --target web --out-dir web/pkg --release';

async function boot() {
  if (typeof THREE === 'undefined') {
    showError('Three.js failed to load — check your network or CDN access.');
    return;
  }

  let wasm;
  try {
    wasm = await init();
  } catch (err) {
    console.error(err);
    showError(
      'The WebAssembly core failed to load.<br>Build it, then serve this folder over HTTP:',
      BUILD_HINT);
    return;
  }

  let stage;
  try {
    stage = createStage();
  } catch (err) {
    console.error(err);
    showError('WebGL is unavailable in this browser.');
    return;
  }

  const ui = {
    playing: true,          // the page should not open on a still frame
    speed: 8,
    tool: 'view',
    brushW: 5,
    autoRotate: !REDUCED,
    showCloud: true,
    xray: 0.12,
    clipOn: false,
    clipAxis: 2,
    clipT: 1,
    emptyOnRebuild: false,
    preset: 'cube',
    size: 22,
    density: 18,
  };

  const world = new World(ui.preset, ui.size, ui.density, 41, ui.emptyOnRebuild);

  /* Typed-array views into wasm linear memory, rebuilt on every access.
     Growing the heap detaches old views, and almost every core call can
     allocate, so caching them would be a use-after-free waiting to happen.
     Constructing a view is nearly free; reading through one is a direct read
     of the core's own memory with no copy. */
  const mem = {
    get state() { return new Uint8Array(wasm.memory.buffer, world.state_ptr(), world.len()); },
    get terrain() { return new Uint8Array(wasm.memory.buffer, world.terrain_ptr(), world.len()); },
    get px() { return new Float32Array(wasm.memory.buffer, world.pos_x_ptr(), world.len()); },
    get py() { return new Float32Array(wasm.memory.buffer, world.pos_y_ptr(), world.len()); },
    get pz() { return new Float32Array(wasm.memory.buffer, world.pos_z_ptr(), world.len()); },
    get rot() { return new Float32Array(wasm.memory.buffer, world.rot_ptr(), world.len()); },
  };

  const volume = new Volume(stage.scene, world, mem);
  const trails = new Trails(stage.scene, mem, stage.cycle);

  // Search bookkeeping the renderer needs but the core does not track.
  let totalSteps = 0;       // expansions in a completed search, for the scrub bar
  let shownDone = false;    // whether the finished-path visuals are up
  let pathIdx = null;
  let compareOn = false;

  const app = {
    canvas: stage.canvas, camera: stage.camera, cam: stage.cam,
    stage, world, volume, trails, ui,
  };

  // ---- helpers ---------------------------------------------------------

  function markerTo(marker, cell) {
    marker.grp.position.set(mem.px[cell], mem.py[cell], mem.pz[cell]);
  }

  function placeMarkers() {
    markerTo(stage.startM, world.start_cell());
    markerTo(stage.goalM, world.goal_cell());
  }

  /** Re-read the scrub range. This runs a throwaway full search inside the
   *  core, so it is called on commits rather than every frame. */
  function refreshRange() {
    totalSteps = world.total_steps();
  }

  /** Tear the finished-path visuals down and put the search back to zero,
   *  preserving whether we were playing. Edits and option changes go through
   *  here, which is why pausing now survives them. */
  function resetSearch() {
    world.reset_search();
    trails.clearPath();
    volume.resetTrails();
    volume.setCloudEmphasis(false);
    shownDone = false;
    pathIdx = null;
    refreshRange();
  }

  function onDone(now) {
    shownDone = true;
    ui.playing = false;
    if (world.found()) {
      pathIdx = world.path();
      trails.showPath(pathIdx, world.cost(), now);
      volume.setCloudEmphasis(true); // quiet the cloud so the trail owns the frame
    } else {
      pathIdx = null;
    }
    refreshCompare();
  }

  function refreshCompare() {
    const complete = world.route_complete() && world.route_error() === undefined;
    compareOn = complete;
    const algCost = world.is_done() && world.found() ? world.cost() : NaN;
    panel.showCompare(complete, algCost);
    if (complete) trails.showOptimal(world.optimal_path());
    else trails.hideOptimal();
  }

  function applyVisibility() {
    volume.xray = ui.xray;
    volume.setCloudVisible(ui.showCloud);
    updateClipPlane(ui, world.bbox(), panel.clipOut);
    volume.applyDim(ui.playing && !world.is_done(), 0, true);
  }

  /** Rebuild every per-domain resource. Only the shape-changing paths need
   *  this; a paint stroke just refreshes the static meshes. */
  function rebuildScene(reframe) {
    volume.rebuild();
    trails.clearPath();
    trails.clearRoute();
    placeMarkers();
    if (reframe) stage.frame(world.bbox(), world.span());
    applyVisibility();
    panel.showWorld();
    shownDone = false;
    pathIdx = null;
    refreshRange();
  }

  // ---- actions the UI and input layers call ----------------------------

  Object.assign(app, {
    togglePlay() {
      if (!ui.playing && world.is_done()) resetSearch(); // rerun from a finished state
      ui.playing = !ui.playing;
    },
    stepForward() {
      ui.playing = false;
      if (world.is_done()) return;
      world.advance(1);
    },
    stepBack() {
      ui.playing = false;
      if (world.expanded() === 0) return;
      world.step_back(1);
      trails.clearPath();
      volume.resetTrails();
      volume.setCloudEmphasis(false);
      shownDone = false;
      pathIdx = null;
    },
    seek(step) {
      ui.playing = false;
      const target = clamp(step, 0, totalSteps);
      if (target === world.expanded()) return;
      world.seek(target);
      // Seeking backwards invalidates whatever trail was on screen; the frame
      // loop puts it back if the new position is also a finished search.
      trails.clearPath();
      volume.resetTrails();
      volume.setCloudEmphasis(false);
      shownDone = false;
      pathIdx = null;
    },
    rewind() { app.seek(0); },
    runToEnd() { app.seek(totalSteps); },

    setTool(tool) {
      ui.tool = tool;
      if (tool === 'route') world.route_begin();
      volume.setHover(-1);
      panel.showTool(tool);
      panel.showCell(-1);
    },

    setAutoRotate(on) {
      ui.autoRotate = on;
      panel.setRotateChecked(on);
    },

    randomize() {
      world.rebuild(ui.preset, ui.size, ui.density,
        (Math.random() * 0x7fffffff) | 0, ui.emptyOnRebuild);
      rebuildScene(true);
      refreshCompare();
    },

    rebuildWorld() {
      world.rebuild(ui.preset, ui.size, ui.density, world.seed(), ui.emptyOnRebuild);
      rebuildScene(true);
      refreshCompare();
    },

    clearWorld() {
      world.clear_terrain();
      volume.rebuildStatics();
      trails.clearPath();
      trails.clearRoute();
      applyVisibility();
      resetSearch();
      refreshCompare();
    },

    /** Mid-stroke: the terrain changed but the gesture is not over yet. */
    onTerrainChanged() {
      volume.rebuildStatics();
      volume.applyDim(ui.playing && !world.is_done(), 0, true);
    },

    onEndpointMoved() {
      placeMarkers();
      trails.clearRoute();
      refreshCompare();
    },

    /** Pointer released after an edit or an endpoint drag. */
    onEditCommitted() {
      resetSearch();
      trails.showRoute(world.route());
      refreshCompare();
    },

    onRouteChanged() {
      trails.showRoute(world.route());
      refreshCompare();
    },

    onSearchReset() { resetSearch(); },

    onVisibilityChanged() { applyVisibility(); },

    warn(msg) { panel.warn(msg); },
    setHint(text) { panel.setHint(text); },
    showCell(cell) { panel.showCell(cell); },
  });

  const panel = createPanel(app);
  const input = attachInput(app);

  // ---- boot ------------------------------------------------------------

  rebuildScene(true);
  app.setTool('view');

  // Read-only hook for the browser checks in tests/.
  window.__pg = {
    get world() { return world; },
    get ui() { return ui; },
    get totalSteps() { return totalSteps; },
    app,
    screenOf(i) {
      const v = new THREE.Vector3(mem.px[i], mem.py[i], mem.pz[i]).project(stage.camera);
      return { x: (v.x + 1) / 2 * innerWidth, y: (1 - v.y) / 2 * innerHeight, z: v.z };
    },
  };

  let last = performance.now() / 1000;
  function frame() {
    requestAnimationFrame(frame);
    const now = performance.now() / 1000;
    const dt = Math.min(0.05, now - last);
    last = now;

    if (ui.playing && !world.is_done()) world.advance(ui.speed);

    const done = world.is_done();
    if (done && !shownDone) onDone(now);

    volume.updateClouds(now);
    volume.updatePathCells(now, pathIdx);
    trails.update(now);
    stage.animateMarkers(now, dt);
    volume.applyDim(ui.playing && !done, dt, false);

    // Starts off under prefers-reduced-motion, but an explicit opt-in has to
    // actually rotate.
    if (ui.autoRotate && input.pointerCount() === 0) stage.cam.theta += dt * 0.1;
    stage.applyCamera();
    stage.renderer.render(stage.scene, stage.camera);

    panel.showStatus();
    panel.showStats(pathIdx ? pathIdx.length : 0, world.found() ? world.cost() : NaN);
    panel.showTransport(totalSteps);
    if (compareOn) panel.showCompare(true, done && world.found() ? world.cost() : NaN);
  }
  frame();
}

boot().catch(err => {
  console.error(err);
  showError(`Failed to start: ${err && err.message ? err.message : err}`);
});
