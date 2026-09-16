/* The voxel volume: walls, weighted cells, the explored cloud, the domain
   hull and the hover highlight.

   Every cell position, every instance list and every membership test comes
   from the Rust core through `mem`. This module decides how those cells look
   and nothing else. */

import {
  LIFT, OPEN_OP, CLOSED_OP, WEIGHT_OP, REDUCED,
  clipPlane, unitBox, dummy, clamp,
} from './scene.js';

const FREE = 0, OBST = 1, OPEN = 2, CLOSED = 3, PATH = 4;
const W_MAX = 10;
const REZ_T = 0.24; // voxel rez-in duration (s)

/** Cheap 32-bit hash, used to desynchronise the rez-in flicker per cell. */
function hash01(i) {
  i = (i ^ 61) ^ (i >>> 16);
  i = (i + (i << 3)) | 0;
  i = i ^ (i >>> 4);
  i = Math.imul(i, 0x27d4eb2d);
  i = i ^ (i >>> 15);
  return (i >>> 0) / 4294967296;
}

export class Volume {
  constructor(scene, world, mem) {
    this.scene = scene;
    this.world = world;
    this.mem = mem;
    this.group = null;
    this.obsMesh = this.edgeLines = this.weightMesh = null;
    this.openMesh = this.closedMesh = this.pathMesh = null;
    this.hover = null;
    this.prevState = null;
    this.born = null;
    this.dimK = 0;      // 0 = solid walls, 1 = fully x-rayed
    this.dimApplied = -1;
    this.xray = 0.12;
    this.showCloud = true;
  }

  /** Full teardown and rebuild — used when the domain's shape changes. */
  rebuild() {
    this.dispose();
    const { world } = this;
    const total = world.len();
    this.group = new THREE.Group();
    this.scene.add(this.group);
    this.group.add(this._buildHull());
    this.rebuildStatics();

    this.openMesh = this._cloud(0x53f4ff, OPEN_OP, 2, total);
    this.closedMesh = this._cloud(0x1c55ff, CLOSED_OP, 2, total);
    this.pathMesh = this._cloud(0xffb054, 0.9, 3, total);
    this.openMesh.visible = this.closedMesh.visible = this.showCloud;

    // Hover highlight: a bright wire cage plus a faint fill, both drawn
    // through walls so the cell a click will hit is never hidden by one.
    this.hover = new THREE.Group();
    const wire = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(1.04, 1.04, 1.04)),
      new THREE.LineBasicMaterial({
        color: 0xffffff, transparent: true, opacity: 0.95,
        depthTest: false, depthWrite: false,
      }));
    const fill = new THREE.Mesh(unitBox, new THREE.MeshBasicMaterial({
      color: 0x4ff2ff, transparent: true, opacity: 0.22,
      blending: THREE.AdditiveBlending, depthTest: false, depthWrite: false,
    }));
    this.hover.add(wire, fill);
    this.hover.renderOrder = 12;
    this.hover.visible = false;
    this.hover.traverse(o => { o.raycast = () => {}; });
    this.scene.add(this.hover);
    this.hoverWire = wire;

    this.prevState = new Uint8Array(total);
    this.prevState.set(this.mem.state);
    this.born = new Float32Array(total).fill(-9);
  }

  /** Walls, neon rims and weighted cells, rebuilt from the terrain whenever a
   *  paint stroke changes it. Instance lists come from the core. */
  rebuildStatics() {
    const { world, mem, group } = this;
    if (this.obsMesh) {
      group.remove(this.obsMesh, this.edgeLines, this.weightMesh);
      this.obsMesh.dispose(); this.obsMesh.material.dispose();
      this.edgeLines.geometry.dispose(); this.edgeLines.material.dispose();
      this.weightMesh.dispose(); this.weightMesh.material.dispose();
    }
    const walls = world.obstacle_cells();
    const surface = world.surface_obstacle_cells();
    const weighted = world.weighted_cells();

    this.obsMesh = new THREE.InstancedMesh(unitBox, new THREE.MeshLambertMaterial({
      color: 0x0c1826, transparent: true, opacity: 1, clippingPlanes: [clipPlane],
    }), Math.max(1, walls.length));
    for (let n = 0; n < walls.length; n++) this._place(this.obsMesh, n, walls[n], 0.86);
    this.obsMesh.count = walls.length;
    this.obsMesh.instanceMatrix.needsUpdate = true;
    this.obsMesh.frustumCulled = false;
    this.obsMesh.raycast = () => {}; // picking is a Rust ray-march, not a raycast
    group.add(this.obsMesh);

    // Neon rims on exposed wall faces.
    const hb = 0.46;
    const E = [
      [-hb,-hb,-hb, hb,-hb,-hb], [-hb, hb,-hb, hb, hb,-hb], [-hb,-hb, hb, hb,-hb, hb], [-hb, hb, hb, hb, hb, hb],
      [-hb,-hb,-hb,-hb, hb,-hb], [ hb,-hb,-hb, hb, hb,-hb], [-hb,-hb, hb,-hb, hb, hb], [ hb,-hb, hb, hb, hb, hb],
      [-hb,-hb,-hb,-hb,-hb, hb], [ hb,-hb,-hb, hb,-hb, hb], [-hb, hb,-hb,-hb, hb, hb], [ hb, hb,-hb, hb, hb, hb],
    ];
    const { px, py, pz, rot } = mem;
    const epos = new Float32Array(surface.length * E.length * 6);
    let o = 0;
    for (const i of surface) {
      const ca = Math.cos(rot[i]), sa = Math.sin(rot[i]);
      for (const e of E) {
        // Rotate the axis-aligned edge template into the cell's ring frame.
        const ax = e[0] * ca + e[2] * sa, az = -e[0] * sa + e[2] * ca;
        const bx = e[3] * ca + e[5] * sa, bz = -e[3] * sa + e[5] * ca;
        epos[o++] = px[i] + ax; epos[o++] = py[i] + e[1]; epos[o++] = pz[i] + az;
        epos[o++] = px[i] + bx; epos[o++] = py[i] + e[4]; epos[o++] = pz[i] + bz;
      }
    }
    const eg = new THREE.BufferGeometry();
    eg.setAttribute('position', new THREE.BufferAttribute(epos, 3));
    this.edgeLines = new THREE.LineSegments(eg, new THREE.LineBasicMaterial({
      color: 0x1f8fb4, transparent: true, opacity: 0.5,
      blending: THREE.AdditiveBlending, depthWrite: false,
      clippingPlanes: [clipPlane],
    }));
    this.edgeLines.frustumCulled = false;
    this.edgeLines.renderOrder = 1;
    this.edgeLines.raycast = () => {};
    group.add(this.edgeLines);

    // Weighted cells: hot-amber energy — brighter and larger means costlier.
    this.weightMesh = new THREE.InstancedMesh(unitBox, new THREE.MeshBasicMaterial({
      color: 0xffffff, transparent: true, opacity: WEIGHT_OP,
      blending: THREE.AdditiveBlending, depthWrite: false,
      clippingPlanes: [clipPlane],
    }), Math.max(1, weighted.length));
    const cLo = new THREE.Color(0x552d12), cHi = new THREE.Color(0xff6a4d);
    const cc = new THREE.Color();
    const terrain = mem.terrain;
    for (let n = 0; n < weighted.length; n++) {
      const i = weighted[n], w = terrain[i];
      this._place(this.weightMesh, n, i, 0.48 + 0.035 * w);
      cc.copy(cLo).lerp(cHi, (w - 2) / (W_MAX - 2));
      this.weightMesh.setColorAt(n, cc);
    }
    this.weightMesh.count = weighted.length;
    this.weightMesh.instanceMatrix.needsUpdate = true;
    if (this.weightMesh.instanceColor) this.weightMesh.instanceColor.needsUpdate = true;
    this.weightMesh.frustumCulled = false;
    this.weightMesh.renderOrder = 1;
    this.weightMesh.raycast = () => {};
    group.add(this.weightMesh);

    this.dimApplied = -1; // fresh materials: reapply the x-ray state next frame
  }

  /** Redraw the explored cloud from the core's per-cell state buffer. */
  updateClouds(now) {
    const st = this.mem.state;
    const { px, py, pz, rot } = this.mem;
    const total = st.length;
    let no = 0, nc = 0;
    for (let i = 0; i < total; i++) {
      const s = st[i];
      if (s !== this.prevState[i]) {
        const was = this.prevState[i];
        if (was === FREE || was === OBST) this.born[i] = now; // a fresh rez, not a re-tint
        this.prevState[i] = s;
      }
      if (s !== OPEN && s !== CLOSED) continue;
      let k = 1;
      if (!REDUCED) {
        const age = now - this.born[i];
        if (age < REZ_T) {
          const t = Math.max(0, age / REZ_T);
          k = t * t * (3 - 2 * t);
          if (age < 0.16) k *= 0.62 + 0.5 * hash01(i * 131 + ((now * 90) | 0) * 911);
        }
      }
      dummy.position.set(px[i], py[i], pz[i]);
      dummy.rotation.set(0, rot[i], 0);
      dummy.scale.setScalar((s === OPEN ? 0.62 : 0.46) * k);
      dummy.updateMatrix();
      if (s === OPEN) this.openMesh.setMatrixAt(no++, dummy.matrix);
      else this.closedMesh.setMatrixAt(nc++, dummy.matrix);
    }
    this.openMesh.count = no;
    this.closedMesh.count = nc;
    this.openMesh.instanceMatrix.needsUpdate = true;
    this.closedMesh.instanceMatrix.needsUpdate = true;
  }

  /** The pulsing amber cubes sitting on the found path. */
  updatePathCells(now, path) {
    if (!path || !path.length) {
      if (this.pathMesh.count !== 0) {
        this.pathMesh.count = 0;
        this.pathMesh.instanceMatrix.needsUpdate = true;
      }
      return;
    }
    const { px, py, pz, rot } = this.mem;
    let n = 0;
    for (const i of path) {
      const pulse = REDUCED ? 1 : 1 + 0.12 * Math.sin(now * 5 + n * 0.55);
      dummy.position.set(px[i], py[i], pz[i]);
      dummy.rotation.set(0, rot[i], 0);
      dummy.scale.setScalar(0.5 * pulse);
      dummy.updateMatrix();
      this.pathMesh.setMatrixAt(n++, dummy.matrix);
    }
    this.pathMesh.count = n;
    this.pathMesh.instanceMatrix.needsUpdate = true;
  }

  /** Park the highlight on a cell, or hide it with `cell < 0`. */
  setHover(cell, color) {
    if (!this.hover) return;
    if (cell < 0) { this.hover.visible = false; return; }
    const { px, py, pz, rot } = this.mem;
    this.hover.visible = true;
    this.hover.position.set(px[cell], py[cell], pz[cell]);
    this.hover.rotation.set(0, rot[cell], 0);
    if (color !== undefined) this.hoverWire.material.color.setHex(color);
  }

  /** While a search runs, walls and weights fade to x-ray so the cloud and
   *  the path read through them; they come back when it stops. */
  applyDim(active, dt, force) {
    const target = active ? 1 : 0;
    const prev = this.dimK;
    if (!force) {
      this.dimK += (target - this.dimK) * Math.min(1, (dt || 0.016) * 6);
      if (Math.abs(this.dimK - target) < 0.01) this.dimK = target;
    }
    if (!this.obsMesh) return;
    if (!force && this.dimK === prev && this.dimK === target && this.dimApplied === target) return;
    this.dimApplied = this.dimK;
    const k = this.dimK;
    const om = this.obsMesh.material;
    om.opacity = 1 + (this.xray - 1) * k;
    om.depthWrite = k < 0.5; // x-rayed walls must not occlude the cloud
    this.edgeLines.material.opacity = 0.5 - 0.38 * k;
    this.weightMesh.material.opacity = WEIGHT_OP * (1 - 0.55 * k);
  }

  setCloudVisible(on) {
    this.showCloud = on;
    if (this.openMesh) this.openMesh.visible = this.closedMesh.visible = on;
  }

  /** Quieten the search cloud once a path is on screen, so the trail owns
   *  the frame; `reset` puts it back for the next run. */
  setCloudEmphasis(found) {
    if (!this.openMesh) return;
    this.openMesh.material.opacity = found ? 0.26 : OPEN_OP;
    this.closedMesh.material.opacity = found ? 0.10 : CLOSED_OP;
  }

  /** Forget the rez-in timings after a reset or a scrub, so cells that were
   *  already lit do not replay their birth animation. */
  resetTrails() {
    if (!this.born) return;
    this.born.fill(-9);
    this.prevState.set(this.mem.state);
  }

  dispose() {
    if (this.hover) {
      this.scene.remove(this.hover);
      this.hover.traverse(o => {
        if (o.geometry && o.geometry !== unitBox) o.geometry.dispose();
        if (o.material) o.material.dispose();
      });
      this.hover = null;
    }
    if (!this.group) return;
    this.group.traverse(o => {
      if (o.isInstancedMesh) o.dispose();
      if (o.geometry && o.geometry !== unitBox) o.geometry.dispose();
      if (o.material) o.material.dispose();
    });
    this.scene.remove(this.group);
    this.group = null;
    this.obsMesh = this.edgeLines = this.weightMesh = null;
  }

  // ---- internals -------------------------------------------------------

  _place(mesh, slot, i, scale) {
    const { px, py, pz, rot } = this.mem;
    dummy.position.set(px[i], py[i], pz[i]);
    dummy.rotation.set(0, rot[i], 0);
    dummy.scale.setScalar(scale);
    dummy.updateMatrix();
    mesh.setMatrixAt(slot, dummy.matrix);
  }

  _cloud(color, opacity, order, total) {
    const m = new THREE.InstancedMesh(unitBox, new THREE.MeshBasicMaterial({
      color, transparent: true, opacity,
      blending: THREE.AdditiveBlending, depthWrite: false,
      clippingPlanes: [clipPlane],
    }), total);
    m.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    m.count = 0;
    m.frustumCulled = false;
    m.renderOrder = order;
    m.raycast = () => {};
    this.group.add(m);
    return m;
  }

  /** Wireframe hull matched to the domain: a box cage for box-like presets,
   *  rings for the sphere, a slant cage for the pyramid, a donut for the
   *  torus (with its wrap seam marked). */
  _buildHull() {
    const { world } = this;
    const preset = world.preset();
    const nx = world.nx(), ny = world.ny(), nz = world.nz();
    const grp = new THREE.Group();
    const mat = () => new THREE.LineBasicMaterial({
      color: 0x2fb6d8, transparent: true, opacity: 0.45,
    });
    const midY = LIFT + 0.5 + (ny - 1) / 2;
    if (preset === 'torus') {
      const R = (nx / (2 * Math.PI)) * 1.06;
      const tg = new THREE.TorusGeometry(R, (ny - 1) / 2 + 0.75, 8, Math.min(64, nx));
      const lines = new THREE.LineSegments(new THREE.EdgesGeometry(tg, 12), mat());
      lines.rotation.x = Math.PI / 2;
      lines.position.y = midY;
      grp.add(lines);
      // Seam marker: the modular boundary where x wraps from nx-1 back to 0.
      const sg = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(R - ny / 2 - 0.8, midY, 0),
        new THREE.Vector3(R + ny / 2 + 0.8, midY, 0),
      ]);
      grp.add(new THREE.Line(sg, new THREE.LineBasicMaterial({
        color: 0xffb054, transparent: true, opacity: 0.7,
      })));
    } else if (preset === 'sphere') {
      const r = (nx - 1) / 2 + 0.8;
      const pts = [];
      for (let s = 0; s <= 64; s++) {
        const a = s / 64 * 2 * Math.PI;
        pts.push(new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0));
      }
      for (const rot of [[0, 0, 0], [Math.PI / 2, 0, 0], [0, Math.PI / 2, 0]]) {
        const ring = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat());
        ring.rotation.set(rot[0], rot[1], rot[2]);
        ring.position.y = midY;
        grp.add(ring);
      }
    } else if (preset === 'pyramid') {
      const m = Math.floor((nx - 3) / 2);
      const hb = (nx - 1) / 2 + 0.575, ht = (nx - 1 - 2 * m) / 2 + 0.575;
      const y0 = LIFT + 0.5 - 0.575, y1 = LIFT + 0.5 + (ny - 1) + 0.575;
      const q = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
      const pts = [];
      for (let s = 0; s < 4; s++) {
        const [ax, az] = q[s], [bx, bz] = q[(s + 1) % 4];
        pts.push(ax * hb, y0, az * hb, bx * hb, y0, bz * hb); // base edge
        pts.push(ax * ht, y1, az * ht, bx * ht, y1, bz * ht); // apex edge
        pts.push(ax * hb, y0, az * hb, ax * ht, y1, az * ht); // slant
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pts), 3));
      grp.add(new THREE.LineSegments(g, mat()));
    } else {
      const cg = new THREE.EdgesGeometry(new THREE.BoxGeometry(nx + 0.15, ny + 0.15, nz + 0.15));
      const box = new THREE.LineSegments(cg, mat());
      box.position.set(0, midY, 0);
      grp.add(box);
    }
    grp.traverse(o => { o.raycast = () => {}; });
    return grp;
  }
}

/** Clip slice: shaves the front of the volume away along an axis so the
 *  interior of a dense search is directly inspectable. */
export function updateClipPlane(ui, bbox, out) {
  const a = ui.clipAxis;
  const n = new THREE.Vector3(a === 0 ? -1 : 0, a === 1 ? -1 : 0, a === 2 ? -1 : 0);
  if (!ui.clipOn) {
    clipPlane.set(n, 1e9);
    if (out) out.textContent = 'OFF';
    return;
  }
  const lo = bbox[a] - 0.6, hi = bbox[a + 3] + 0.6;
  clipPlane.set(n, lo + clamp(ui.clipT, 0, 1) * (hi - lo));
  if (out) out.textContent = `${Math.round(ui.clipT * 100)}%`;
}
