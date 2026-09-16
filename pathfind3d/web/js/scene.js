/* Stage: renderer, camera, lighting and the permanent set dressing.
   Nothing here knows what a search is — it owns pixels only. */

export const LIFT = 2.6;          // must match domain::LIFT in the Rust core
export const WALL_DROP = 1.6;     // light-wall ribbon depth below the path curve
export const OPEN_OP = 0.6, CLOSED_OP = 0.26, WEIGHT_OP = 0.4;
export const REDUCED = matchMedia('(prefers-reduced-motion: reduce)').matches;

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

/** One clip plane shared by every volume material. "Disabled" parks it far
 *  outside the domain, which avoids recompiling materials to toggle it. */
export const clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e9);

export const unitBox = new THREE.BoxGeometry(1, 1, 1);
export const dummy = new THREE.Object3D();

const glowTex = (() => {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.25, 'rgba(255,255,255,0.55)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
})();

export function makeGlow(color, scale) {
  const s = new THREE.Sprite(new THREE.SpriteMaterial({
    map: glowTex, color, transparent: true, opacity: 0.85,
    blending: THREE.AdditiveBlending, depthWrite: false, depthTest: false,
  }));
  s.scale.set(scale, scale, 1);
  s.raycast = () => {}; // glows must never swallow a picking ray
  return s;
}

/** Start/goal marker. `grab` is an invisible sphere several cells across: it
 *  is the only thing in the marker that raycasts, so grabbing the endpoint
 *  stays forgiving even when the octahedron is small on screen. */
function makeMarker(bodyColor, glowColor, glowScale) {
  const grp = new THREE.Group();
  const octa = new THREE.Mesh(
    new THREE.OctahedronGeometry(0.52),
    new THREE.MeshBasicMaterial({ color: bodyColor }));
  // Through-wall ghost, so endpoints stay legible inside dense volumes.
  const ghost = new THREE.Mesh(
    new THREE.OctahedronGeometry(0.52),
    new THREE.MeshBasicMaterial({
      color: bodyColor, transparent: true, opacity: 0.3,
      depthTest: false, depthWrite: false,
    }));
  ghost.renderOrder = 9;
  ghost.raycast = () => {};
  const glow = makeGlow(glowColor, glowScale);
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.95, 0.055, 6, 28),
    new THREE.MeshBasicMaterial({
      color: glowColor, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false, depthTest: false,
    }));
  ring.renderOrder = 10;
  ring.raycast = () => {};
  // Drawn as nothing (no colour, no depth) rather than `visible: false`,
  // because an invisible object is skipped by the raycaster in some three
  // versions and this sphere exists purely to be hit.
  const grab = new THREE.Mesh(
    new THREE.SphereGeometry(1.35, 8, 6),
    new THREE.MeshBasicMaterial({
      colorWrite: false, depthWrite: false, depthTest: false,
    }));
  grab.renderOrder = -1;
  grp.add(octa, ghost, glow, ring, grab, new THREE.PointLight(glowColor, 0.9, 14));
  return { grp, octa, ghost, glow, ring, grab, glowScale };
}

export function createStage() {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
  renderer.setSize(innerWidth, innerHeight);
  renderer.localClippingEnabled = true;
  renderer.domElement.id = 'scene';
  document.body.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x020409);
  scene.fog = new THREE.Fog(0x020409, 70, 620);
  const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 1200);

  scene.add(new THREE.AmbientLight(0x2a3f55, 1.0));
  const dirLight = new THREE.DirectionalLight(0x9fd8ff, 0.55);
  dirLight.position.set(40, 90, 30);
  scene.add(dirLight);

  // Floor grid.
  const gridMinor = new THREE.GridHelper(900, 150, 0x0b4a5c, 0x07293a);
  gridMinor.material.transparent = true;
  gridMinor.material.opacity = 0.55;
  scene.add(gridMinor);
  const gridMajor = new THREE.GridHelper(900, 30, 0x12809b, 0x0d5468);
  gridMajor.material.transparent = true;
  gridMajor.material.opacity = 0.75;
  gridMajor.position.y = 0.02;
  scene.add(gridMajor);

  // Horizon band: vertex-colour brightness fades to black, which is invisible
  // under additive blending — no alpha texture needed.
  {
    const hg = new THREE.CylinderGeometry(440, 440, 120, 72, 6, true);
    const posAttr = hg.attributes.position;
    const cols = new Float32Array(posAttr.count * 3);
    const base = new THREE.Color(0x2fd4ff);
    for (let i = 0; i < posAttr.count; i++) {
      const t = posAttr.getY(i) / 120 + 0.5; // 0 bottom -> 1 top
      const k = Math.pow(1 - t, 2.4) * 0.5;
      cols[i * 3] = base.r * k; cols[i * 3 + 1] = base.g * k; cols[i * 3 + 2] = base.b * k;
    }
    hg.setAttribute('color', new THREE.BufferAttribute(cols, 3));
    const horizon = new THREE.Mesh(hg, new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, blending: THREE.AdditiveBlending,
      side: THREE.BackSide, depthWrite: false, fog: false,
    }));
    horizon.position.y = 56;
    horizon.raycast = () => {};
    scene.add(horizon);
  }

  const startM = makeMarker(0x86f7ff, 0x53f4ff, 3.2);
  const goalM = makeMarker(0xffcf8e, 0xffa04d, 3.4);
  scene.add(startM.grp, goalM.grp);

  // The cycle: a travelling marker that rides the finished path.
  const cycle = new THREE.Group();
  const cycleBody = new THREE.Mesh(
    new THREE.OctahedronGeometry(0.42),
    new THREE.MeshBasicMaterial({ color: 0xffd9a0 }));
  cycleBody.scale.set(0.55, 0.55, 1.6);
  cycle.add(cycleBody, makeGlow(0xffa04d, 2.8), new THREE.PointLight(0xff9b3d, 1.5, 26));
  cycle.visible = false;
  cycle.traverse(o => { o.raycast = () => {}; });
  scene.add(cycle);

  const cam = {
    theta: 0.85, phi: 1.02, radius: 55,
    rMin: 20, rMax: 300,
    target: new THREE.Vector3(0, LIFT + 11, 0),
  };

  function applyCamera() {
    cam.phi = clamp(cam.phi, 0.12, 1.52);
    cam.radius = clamp(cam.radius, cam.rMin, cam.rMax);
    const sp = Math.sin(cam.phi);
    camera.position.set(
      cam.target.x + cam.radius * sp * Math.sin(cam.theta),
      cam.target.y + cam.radius * Math.cos(cam.phi),
      cam.target.z + cam.radius * sp * Math.cos(cam.theta));
    camera.lookAt(cam.target);
  }

  /** Frame the camera on a domain's bounding box. */
  function frame(bbox, span) {
    cam.target.set(
      (bbox[0] + bbox[3]) / 2,
      (bbox[1] + bbox[4]) / 2,
      (bbox[2] + bbox[5]) / 2);
    cam.rMin = span * 1.05;
    cam.rMax = Math.min(400, span * 7 + 40);
    cam.radius = clamp(span * 2.3, cam.rMin, cam.rMax);
    scene.fog.near = span * 3.2;
  }

  addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  function animateMarkers(now, dt) {
    if (REDUCED) return;
    startM.octa.rotation.y += dt * 1.2;
    goalM.octa.rotation.y -= dt * 1.2;
    startM.ghost.rotation.copy(startM.octa.rotation);
    goalM.ghost.rotation.copy(goalM.octa.rotation);
    const gp = 1 + 0.22 * Math.sin(now * 3.1);
    goalM.glow.scale.set(3.4 * gp, 3.4 * gp, 1);
    const sp = 1 + 0.15 * Math.sin(now * 2.3 + 1.7);
    startM.glow.scale.set(3.2 * sp, 3.2 * sp, 1);
    for (const m of [startM, goalM]) {
      m.ring.rotation.x = Math.PI / 2;
      m.ring.rotation.z += dt * 0.8;
    }
  }

  /** Halo the marker the pointer can grab, so "this is draggable" is visible
   *  before the drag rather than discovered by accident. */
  function highlightMarker(which) {
    startM.ring.material.opacity = which === 'start' ? 0.9 : 0;
    goalM.ring.material.opacity = which === 'goal' ? 0.9 : 0;
    const s = which === 'start' ? 1.25 : 1, g = which === 'goal' ? 1.25 : 1;
    startM.grp.scale.setScalar(s);
    goalM.grp.scale.setScalar(g);
  }

  return {
    renderer, canvas: renderer.domElement, scene, camera, cam,
    applyCamera, frame, startM, goalM, cycle, animateMarkers, highlightMarker,
  };
}
