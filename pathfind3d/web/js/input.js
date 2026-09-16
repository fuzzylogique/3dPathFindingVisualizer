/* Pointer and keyboard handling.

   Cells are never worked out here. The pointer is turned into a world-space
   ray and handed to the Rust core, which answers with cell indices; this
   module decides only what a gesture means.

   Two deliberate departures from the old editor:

   * The start and goal are dragged directly. They are not tools you switch
     into, so moving an endpoint never costs you the tool you were using.
   * Every brush shows the cell it will affect before you commit, because a
     click into a 3D volume is depth-ambiguous and guessing is miserable. */

const RAY_NDC = new THREE.Vector2();
const RAYCASTER = new THREE.Raycaster();
const DIR = new THREE.Vector3();

/** Tool → the terrain byte a plain click writes, and what a clearing click
 *  (Alt, or the right button) writes back. */
const BRUSH = {
  wall: { paint: () => 255, clear: () => 1, weighted: false },
  weight: { paint: ui => ui.brushW, clear: () => 1, weighted: true },
};

export function attachInput(app) {
  const { canvas, camera, cam, stage, world, volume, ui } = app;
  const pointers = new Map();
  let gesture = null;      // {type:'orbit'|'edit'|'drag', which?, moved?}
  let pinchDist = 0, pinchRadius = 0;
  let edited = false;      // the terrain or an endpoint changed in this gesture

  /** Camera ray through a screen point, as plain numbers for the core. */
  function rayAt(cx, cy) {
    RAY_NDC.set((cx / innerWidth) * 2 - 1, -(cy / innerHeight) * 2 + 1);
    RAYCASTER.setFromCamera(RAY_NDC, camera);
    const o = RAYCASTER.ray.origin, d = RAYCASTER.ray.direction;
    return [o.x, o.y, o.z, d.x, d.y, d.z];
  }

  /** Which endpoint marker, if any, is under the pointer. Their grab spheres
   *  are the only raycast targets left in the scene. */
  function markerUnder(cx, cy) {
    RAY_NDC.set((cx / innerWidth) * 2 - 1, -(cy / innerHeight) * 2 + 1);
    RAYCASTER.setFromCamera(RAY_NDC, camera);
    const hits = RAYCASTER.intersectObjects(
      [stage.startM.grab, stage.goalM.grab], false);
    if (!hits.length) return null;
    return hits[0].object === stage.startM.grab ? 'start' : 'goal';
  }

  /** The cell the current tool would act on, plus what it would do to it. */
  function targetFor(r, clearing) {
    if (ui.tool === 'route') {
      const [, build] = world.pick_edit(r[0], r[1], r[2], r[3], r[4], r[5], false);
      return { cell: build, clearing: false };
    }
    const brush = BRUSH[ui.tool];
    if (!brush) return { cell: -1, clearing: false };
    const [solid, build] = world.pick_edit(
      r[0], r[1], r[2], r[3], r[4], r[5], brush.weighted);
    return clearing ? { cell: solid, clearing: true } : { cell: build, clearing: false };
  }

  function applyEdit(e) {
    const clearing = e.altKey || e.buttons === 2 || e.button === 2;
    const r = rayAt(e.clientX, e.clientY);
    const { cell } = targetFor(r, clearing);
    if (cell < 0) return;

    if (ui.tool === 'route') {
      const status = world.route_append(cell);
      if (status === -1) app.warn('ROUTE MUST STAY ON OPEN CELLS');
      else if (status === -2) app.warn('NO OPEN ROUTE TO THAT CELL');
      if (status === 0) app.onRouteChanged();
      return;
    }
    const brush = BRUSH[ui.tool];
    if (!brush) return;
    const value = clearing ? brush.clear(ui) : brush.paint(ui);
    if (!world.paint(cell, value)) {
      // The only refusals are void cells and walling in an endpoint; the
      // second is worth saying out loud, the first is just a miss.
      if (cell === world.start_cell() || cell === world.goal_cell()) {
        app.warn('THE START AND GOAL CANNOT BE WALLED IN');
      }
      return;
    }
    // Build outward from where you last drew, so a stroke through open space
    // keeps its depth instead of snapping back to the middle of the volume.
    world.set_anchor(cell);
    edited = true;
    app.onTerrainChanged();
  }

  function dragEndpoint(e, which) {
    const r = rayAt(e.clientX, e.clientY);
    const cell = world.pick_endpoint(
      r[0], r[1], r[2], r[3], r[4], r[5], which === 'start');
    if (cell < 0) return;
    const got = which === 'start' ? world.set_start(cell) : world.set_goal(cell);
    if (got < 0) return;
    edited = true;
    app.onEndpointMoved();
  }

  function endGesture() {
    if (edited) {
      edited = false;
      app.onEditCommitted();
    }
    gesture = null;
  }

  // ---- pointer ---------------------------------------------------------

  canvas.addEventListener('contextmenu', e => e.preventDefault());

  canvas.addEventListener('pointerdown', e => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

    if (pointers.size === 2) {
      // A second finger always means camera: abandon any edit stroke.
      endGesture();
      gesture = { type: 'orbit' };
      const [a, b] = [...pointers.values()];
      pinchDist = Math.hypot(a.x - b.x, a.y - b.y);
      pinchRadius = cam.radius;
      document.body.dataset.dragging = '0';
      return;
    }

    const marker = e.button === 0 ? markerUnder(e.clientX, e.clientY) : null;
    if (marker) {
      gesture = { type: 'drag', which: marker };
      document.body.dataset.dragging = '1';
      app.setHint(marker === 'start'
        ? 'MOVING THE START — RELEASE TO DROP IT'
        : 'MOVING THE GOAL — RELEASE TO DROP IT');
    } else if (ui.tool !== 'view' && (e.button === 0 || e.button === 2)) {
      gesture = { type: 'edit' };
      applyEdit(e);
    } else {
      gesture = { type: 'orbit' };
    }
    // Any deliberate interaction stops the idle spin.
    if (ui.autoRotate) app.setAutoRotate(false);
  });

  canvas.addEventListener('pointermove', e => {
    const p = pointers.get(e.pointerId);

    if (!p) {
      // Hover: show what a click would hit, and light up a grabbable marker.
      const marker = markerUnder(e.clientX, e.clientY);
      document.body.dataset.grab = marker ? '1' : '0';
      stage.highlightMarker(marker);
      if (marker || ui.tool === 'view') {
        volume.setHover(-1);
        app.showCell(-1);
      } else {
        const clearing = e.altKey;
        const { cell } = targetFor(rayAt(e.clientX, e.clientY), clearing);
        volume.setHover(cell, clearing ? 0xff6a4d : 0xffffff);
        app.showCell(cell);
      }
      return;
    }

    const dx = e.clientX - p.x, dy = e.clientY - p.y;
    p.x = e.clientX; p.y = e.clientY;

    if (pointers.size === 1) {
      if (gesture && gesture.type === 'drag') dragEndpoint(e, gesture.which);
      else if (gesture && gesture.type === 'edit') applyEdit(e);
      else {
        cam.theta -= dx * 0.0055;
        cam.phi -= dy * 0.0045;
      }
    } else if (pointers.size === 2 && pinchDist > 0) {
      const [a, b] = [...pointers.values()];
      const d = Math.hypot(a.x - b.x, a.y - b.y);
      if (d > 1) cam.radius = pinchRadius * pinchDist / d;
    }
  });

  const release = e => {
    pointers.delete(e.pointerId);
    if (pointers.size === 0) {
      endGesture();
      document.body.dataset.dragging = '0';
      app.setHint(null);
    }
  };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('pointerleave', () => {
    if (!pointers.size) {
      volume.setHover(-1);
      app.showCell(-1);
      document.body.dataset.grab = '0';
      stage.highlightMarker(null);
    }
  });

  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    if (e.shiftKey && ui.tool !== 'view') {
      // Push the build plane toward or away from the camera. It only matters
      // where there is no wall to build against, but it is the escape hatch
      // for drawing in wide-open space.
      camera.getWorldDirection(DIR);
      world.nudge_anchor(DIR.x, DIR.y, DIR.z, -Math.sign(e.deltaY));
      app.warn(`BUILD PLANE ${Math.sign(e.deltaY) < 0 ? 'FURTHER' : 'NEARER'}`);
      return;
    }
    cam.radius *= Math.exp(e.deltaY * 0.0011);
  }, { passive: false });

  // ---- keyboard --------------------------------------------------------

  addEventListener('keydown', e => {
    if (e.key === 'Escape') { app.setTool('view'); return; } // works from any focus
    if (e.target.closest && e.target.closest('input,button,select,textarea,summary')) return;
    if (e.ctrlKey || e.metaKey) return;

    switch (e.key) {
      case ' ': e.preventDefault(); app.togglePlay(); return;
      case ',': e.preventDefault(); app.stepBack(); return;
      case '.': e.preventDefault(); app.stepForward(); return;
      case 'Home': e.preventDefault(); app.rewind(); return;
      case 'End': e.preventDefault(); app.runToEnd(); return;
      case 'v': case 'V': app.setTool('view'); return;
      case 'w': case 'W': app.setTool('wall'); return;
      case 'e': case 'E': app.setTool('weight'); return;
      case 'r': case 'R': app.setTool('route'); return;
      case 'n': case 'N': app.randomize(); return;
      case 'ArrowLeft': e.preventDefault(); cam.theta += 0.12; return;
      case 'ArrowRight': e.preventDefault(); cam.theta -= 0.12; return;
      case 'ArrowUp': e.preventDefault(); cam.phi -= 0.08; return;
      case 'ArrowDown': e.preventDefault(); cam.phi += 0.08; return;
      case '+': case '=': cam.radius *= 0.92; return;
      case '-': cam.radius *= 1.08; return;
      default:
    }
  });

  return { pointerCount: () => pointers.size };
}
