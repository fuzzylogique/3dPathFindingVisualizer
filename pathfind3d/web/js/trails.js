/* Light trails: the algorithm's path (amber light-wall + tube, ridden by the
   cycle), the route the user drew (cyan), and the optimal reference (pale).

   These take cell-index arrays from the Rust core and turn them into curves.
   No path is computed here — only drawn. */

import { WALL_DROP, REDUCED, clamp } from './scene.js';

const TUBE_RADIAL = 5;

function makeWallMaterial(color, alphaMul, xray) {
  return new THREE.ShaderMaterial({
    uniforms: {
      uColor: { value: new THREE.Color(color) },
      uTime: { value: 0 },
      uMul: { value: alphaMul },
    },
    vertexShader: `
      varying vec2 vUv;
      void main() {
        vUv = uv;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }`,
    fragmentShader: `
      uniform vec3 uColor;
      uniform float uTime;
      uniform float uMul;
      varying vec2 vUv;
      void main() {
        float fade = vUv.y;                     /* 0 bottom -> 1 at the path */
        float hot  = pow(fade, 4.0);            /* searing top edge */
        float scan = 0.85 + 0.15 * sin(vUv.x * 170.0 - uTime * 2.6);
        float a = (0.10 + 0.42 * fade + 0.6 * hot) * scan * uMul;
        gl_FragColor = vec4(uColor * (0.5 + 1.15 * hot + 0.35 * fade), a);
      }`,
    transparent: true,
    depthWrite: false,
    depthTest: !xray,
    side: THREE.DoubleSide,
    blending: THREE.AdditiveBlending,
  });
}

/** A tube plus a desaturated depth-test-free twin, so a route stays legible
 *  behind walls while depth still reads. Both share one geometry, so
 *  drawRange reveals stay in sync. */
function makeTubePair(geo, color, opacity, order) {
  const grp = new THREE.Group();
  const solid = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
    color, transparent: true, opacity,
    blending: THREE.AdditiveBlending, depthWrite: false,
  }));
  const seen = new THREE.Color(color).lerp(new THREE.Color(0x9fb2ba), 0.45);
  const ghost = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
    color: seen, transparent: true, opacity: opacity * 0.32,
    blending: THREE.AdditiveBlending, depthWrite: false, depthTest: false,
  }));
  solid.renderOrder = order;
  ghost.renderOrder = order + 5;
  solid.frustumCulled = ghost.frustumCulled = false;
  solid.raycast = ghost.raycast = () => {};
  grp.add(solid, ghost);
  return grp;
}

function disposeGroup(scene, grp) {
  if (!grp) return null;
  scene.remove(grp);
  grp.traverse(o => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) o.material.dispose();
  });
  return null;
}

export class Trails {
  constructor(scene, mem, cycle) {
    this.scene = scene;
    this.mem = mem;
    this.cycle = cycle;
    this.curve = null;
    this.wall = this.wallGeo = this.wallMat = null;
    this.wallGhost = this.wallGhostMat = null;
    this.wallSegs = 0;
    this.tube = this.tubeGeo = null;
    this.tubeSegs = 0;
    this.routeGrp = null;
    this.optGrp = null;
    this.cycleT0 = 0;
    this.travelT = 4;
    this._v = new THREE.Vector3();
  }

  _points(cells) {
    const { px, py, pz } = this.mem;
    return Array.from(cells, i => new THREE.Vector3(px[i], py[i], pz[i]));
  }

  /** Build the amber light-wall and tube for a finished search. */
  showPath(path, cost, now) {
    this.clearPath();
    const pts = this._points(path);
    if (pts.length < 2) return;
    this.curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');

    const S = Math.min(560, Math.max(48, pts.length * 8));
    this.wallSegs = S - 1;
    const pos = new Float32Array(S * 2 * 3);
    const uv = new Float32Array(S * 2 * 2);
    const index = new Uint32Array(this.wallSegs * 6);
    for (let i = 0; i < S; i++) {
      const p = this.curve.getPointAt(i / (S - 1));
      const o = i * 6;
      pos[o] = p.x; pos[o + 1] = p.y; pos[o + 2] = p.z;
      pos[o + 3] = p.x; pos[o + 4] = p.y - WALL_DROP; pos[o + 5] = p.z;
      const u = i / (S - 1), uo = i * 4;
      uv[uo] = u; uv[uo + 1] = 1;
      uv[uo + 2] = u; uv[uo + 3] = 0;
    }
    for (let i = 0; i < this.wallSegs; i++) {
      const a = i * 2, b = a + 1, c = a + 2, d = a + 3, o = i * 6;
      index[o] = a; index[o + 1] = b; index[o + 2] = c;
      index[o + 3] = b; index[o + 4] = d; index[o + 5] = c;
    }
    this.wallGeo = new THREE.BufferGeometry();
    this.wallGeo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    this.wallGeo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    this.wallGeo.setIndex(new THREE.BufferAttribute(index, 1));
    this.wallMat = makeWallMaterial(0xff9b3d, 1, false);
    this.wall = new THREE.Mesh(this.wallGeo, this.wallMat);
    this.wall.frustumCulled = false;
    this.wall.renderOrder = 4;
    this.wall.raycast = () => {};
    this.scene.add(this.wall);

    // The same wall drawn through walls, faint and desaturated.
    this.wallGhostMat = makeWallMaterial(0xc9a06a, 0.3, true);
    this.wallGhost = new THREE.Mesh(this.wallGeo, this.wallGhostMat);
    this.wallGhost.frustumCulled = false;
    this.wallGhost.renderOrder = 9;
    this.wallGhost.raycast = () => {};
    this.scene.add(this.wallGhost);

    this.tubeSegs = Math.min(400, Math.max(32, pts.length * 6));
    this.tubeGeo = new THREE.TubeGeometry(this.curve, this.tubeSegs, 0.075, TUBE_RADIAL, false);
    this.tube = makeTubePair(this.tubeGeo, 0xffd9a0, 0.95, 4);
    this.scene.add(this.tube);

    this.cycleT0 = now;
    this.travelT = clamp(cost * 0.14, 2.6, 9);
    this._reveal(REDUCED ? 1 : 0);
    this.cycle.visible = true;
  }

  _reveal(p) {
    if (this.wallGeo) {
      this.wallGeo.setDrawRange(0, Math.max(0, Math.floor(p * this.wallSegs)) * 6);
    }
    if (this.tubeGeo) {
      this.tubeGeo.setDrawRange(0, Math.max(0, Math.floor(p * this.tubeSegs)) * TUBE_RADIAL * 6);
    }
  }

  clearPath() {
    if (this.wall) {
      this.scene.remove(this.wall, this.wallGhost);
      this.wallGeo.dispose(); this.wallMat.dispose(); this.wallGhostMat.dispose();
      this.wall = this.wallGeo = this.wallMat = this.wallGhost = this.wallGhostMat = null;
    }
    if (this.tube) {
      this.scene.remove(this.tube);
      this.tubeGeo.dispose();
      this.tube.children.forEach(c => c.material.dispose());
      this.tube = this.tubeGeo = null;
    }
    this.curve = null;
    this.cycle.visible = false;
  }

  showRoute(route) {
    this.routeGrp = disposeGroup(this.scene, this.routeGrp);
    if (!route || route.length < 2) return;
    const curve = new THREE.CatmullRomCurve3(this._points(route), false, 'centripetal');
    const segs = Math.min(400, Math.max(16, route.length * 6));
    const geo = new THREE.TubeGeometry(curve, segs, 0.11, TUBE_RADIAL, false);
    this.routeGrp = makeTubePair(geo, 0x4ff2ff, 0.9, 6);
    this.scene.add(this.routeGrp);
  }

  showOptimal(path) {
    this.optGrp = disposeGroup(this.scene, this.optGrp);
    if (!path || path.length < 2) return;
    const curve = new THREE.CatmullRomCurve3(this._points(path), false, 'centripetal');
    const segs = Math.min(400, Math.max(16, path.length * 6));
    const geo = new THREE.TubeGeometry(curve, segs, 0.05, TUBE_RADIAL, false);
    this.optGrp = makeTubePair(geo, 0xd9f2f7, 0.6, 5);
    this.scene.add(this.optGrp);
  }

  hideOptimal() {
    this.optGrp = disposeGroup(this.scene, this.optGrp);
  }

  clearRoute() {
    this.routeGrp = disposeGroup(this.scene, this.routeGrp);
    this.hideOptimal();
  }

  /** Advance the reveal wipe and ride the cycle along the curve. */
  update(now) {
    if (!this.curve) return;
    let reveal, t;
    if (REDUCED) {
      reveal = 1;
      t = 0; // parked at the start, no auto-traverse
    } else {
      const elapsed = now - this.cycleT0;
      reveal = Math.min(1, elapsed / this.travelT);
      const lapT = this.travelT + 0.9; // a brief hold at the goal between laps
      t = Math.min(1, (elapsed % lapT) / this.travelT);
    }
    this._reveal(reveal);
    const p = this.curve.getPointAt(t);
    this.cycle.position.copy(p);
    const tan = this.curve.getTangentAt(clamp(t, 0.001, 0.999));
    this.cycle.lookAt(this._v.copy(p).add(tan));
    if (this.wallMat) this.wallMat.uniforms.uTime.value = REDUCED ? 0 : now;
    if (this.wallGhostMat) this.wallGhostMat.uniforms.uTime.value = REDUCED ? 0 : now;
  }

  dispose() {
    this.clearPath();
    this.clearRoute();
  }
}
