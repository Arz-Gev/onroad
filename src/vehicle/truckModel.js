import * as THREE from 'three';
import { createTireMaterial } from './tireMaterial.js';
import { createMaterials, makeRubberTexture } from './truckMaterials.js';
import { mesh, bar, mergeStatic, mergeGeometries } from './geom.js';
import { buildExterior } from './truckBody.js';
import { buildInterior } from './truckInterior.js';
import { buildLightRig } from './truckLights.js';

export { createMaterials };

let _tireGeo = null;
export function tireGeometry(R = 0.42) {
  if (_tireGeo) return _tireGeo;
  const prof = [[0.205, -0.112], [0.235, -0.125], [0.29, -0.133], [0.345, -0.134], [0.378, -0.128], [0.392, -0.116],
    [0.398, -0.095], [0.399, 0.0], [0.398, 0.095], [0.392, 0.116], [0.378, 0.128], [0.345, 0.134], [0.29, 0.133], [0.235, 0.125], [0.205, 0.112]];
  const carcass = new THREE.LatheGeometry(prof.map(([r, y]) => new THREE.Vector2(r, y)), 72);
  carcass.rotateZ(-Math.PI / 2);
  const parts = [carcass];
  const NP = 22;
  const lug = (axial, radial0, radial1, circ, ax, angle, skew = 0) => {
    const g = new THREE.BoxGeometry(axial, radial1 - radial0, circ);
    g.translate(0, (radial0 + radial1) / 2, 0);
    if (skew) g.rotateY(skew);
    g.translate(ax, 0, 0);
    g.rotateX(angle);
    return g;
  };
  for (let k = 0; k < NP; k++) {
    const a = (k / NP) * Math.PI * 2, b = a + Math.PI / NP;

    parts.push(lug(0.085, 0.392, R, 0.078, -0.088, a, 0.12));
    parts.push(lug(0.085, 0.392, R, 0.078, 0.088, b, -0.12));

    parts.push(lug(0.028, 0.345, 0.405, 0.06, -0.128, a));
    parts.push(lug(0.028, 0.345, 0.405, 0.06, 0.128, b));

    parts.push(lug(0.055, 0.395, R - 0.002, 0.05, -0.022, b + 0.05, 0.4));
    parts.push(lug(0.055, 0.395, R - 0.002, 0.05, 0.022, a + 0.05, -0.4));
  }
  const nonIndexed = parts.map(g => { const ng = g.index ? g.toNonIndexed() : g; ng.deleteAttribute('uv'); return ng; });
  _tireGeo = mergeGeometries(nonIndexed);
  _tireGeo.computeVertexNormals();
  _tireGeo.computeBoundingSphere();
  return _tireGeo;
}

const _barrelMats = new WeakMap();
function barrelMat(m) {
  if (!_barrelMats.has(m)) { const c = m.clone(); c.side = THREE.DoubleSide; c.onBeforeCompile = m.onBeforeCompile; c.customProgramCacheKey = m.customProgramCacheKey; _barrelMats.set(m, c); }
  return _barrelMats.get(m);
}

function rimGroup(mats) {
  const g = new THREE.Group();

  const barrel = mesh(new THREE.CylinderGeometry(0.205, 0.205, 0.2, 36, 1, true), barrelMat(mats.steel));
  barrel.rotation.z = Math.PI / 2;
  g.add(barrel);
  const lip = mesh(new THREE.TorusGeometry(0.203, 0.009, 8, 40), mats.steel, 0.1, 0, 0);
  lip.rotation.y = Math.PI / 2;
  g.add(lip);
  const face = new THREE.Shape();
  face.absarc(0, 0, 0.2, 0, Math.PI * 2, false);
  for (let k = 0; k < 8; k++) {
    const a = (k + 0.5) / 8 * Math.PI * 2;
    const h = new THREE.Path();
    h.absarc(Math.cos(a) * 0.135, Math.sin(a) * 0.135, 0.027, 0, Math.PI * 2, true);
    face.holes.push(h);
  }
  const fg = new THREE.ExtrudeGeometry(face, { depth: 0.01, bevelEnabled: true, bevelThickness: 0.005, bevelSize: 0.005, bevelSegments: 2, curveSegments: 20 });
  fg.rotateY(Math.PI / 2);
  g.add(mesh(fg, mats.steel, 0.04, 0, 0));

  const dome = mesh(new THREE.CylinderGeometry(0.1, 0.115, 0.03, 28).rotateZ(Math.PI / 2), mats.steel, 0.06, 0, 0);
  g.add(dome);
  const back = mesh(new THREE.CylinderGeometry(0.203, 0.203, 0.01, 36), mats.chassis, 0.0, 0, 0);
  back.rotation.z = Math.PI / 2;
  g.add(back);
  const cap = mesh(new THREE.CylinderGeometry(0.042, 0.05, 0.035, 18), mats.blackMetal, 0.085, 0, 0);
  cap.rotation.z = Math.PI / 2;
  g.add(cap);
  for (let k = 0; k < 5; k++) {
    const a = k / 5 * Math.PI * 2;
    const nut = mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.026, 6), mats.zinc, 0.08, Math.cos(a) * 0.07, Math.sin(a) * 0.07);
    nut.rotation.z = Math.PI / 2;
    g.add(nut);
  }
  return g;
}

function buildWheel(mats, side, deformable) {
  const steer = new THREE.Group();
  const spin = new THREE.Group();
  steer.add(spin);
  const tireMat = deformable ? createTireMaterial(mats.rubberTex) : new THREE.MeshStandardMaterial({ color: 0xffffff, map: mats.rubberTex, roughness: 0.88 });
  const tire = mesh(tireGeometry(), tireMat);
  spin.add(tire);
  const rim = rimGroup(mats);
  mergeStatic(rim);
  if (side < 0) rim.rotation.y = Math.PI;
  spin.add(rim);
  return { steer, spin, tire, tireMat, rim };
}

function coilGeometry(radius, wire, turns) {
  const pts = [];
  const n = turns * 24;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const a = t * turns * Math.PI * 2;
    pts.push(new THREE.Vector3(Math.cos(a) * radius, t, Math.sin(a) * radius));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), n, wire, 6, false);
}

export function buildTruck(P) {
  const mats = createMaterials();
  mats.rubberTex = makeRubberTexture();
  const root = new THREE.Group();
  root.name = 'truck';
  const body = new THREE.Group();
  root.add(body);
  const add = (o, parent = body) => { parent.add(o); return o; };

  const ext = buildExterior(mats, body);
  const cab = buildInterior(mats, body);

  const spare = buildWheel(mats, 1, false);
  spare.steer.position.set(0, 1.17, 2.33 + 0.245);
  spare.steer.rotation.y = -Math.PI / 2;
  add(spare.steer);

  const wheels = [];
  const axles = [];
  const suspension = [];
  const t2 = P.track / 2;
  for (let ai = 0; ai < 2; ai++) {
    const ap = P.axles[ai];
    const axle = new THREE.Group();
    axle.position.set(0, ap.droopY, ap.z);
    root.add(axle);
    add(bar([-t2 + 0.12, 0, 0], [t2 - 0.12, 0, 0], 0.05, mats.chassis, 12), axle);
    const diffX = ai === 0 ? 0.12 : 0;
    const diff = add(mesh(new THREE.SphereGeometry(0.15, 18, 12), mats.chassis, diffX, 0, 0), axle);
    diff.scale.set(1, 0.9, 0.75);
    add(mesh(new THREE.CylinderGeometry(0.05, 0.06, 0.14, 12).rotateX(Math.PI / 2), mats.chassis, diffX, 0.0, ai === 0 ? 0.15 : -0.15), axle);
    for (const s of [-1, 1]) {
      const hub = add(mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.14, 16), mats.chassis, s * (t2 - 0.17), 0, 0), axle);
      hub.rotation.z = Math.PI / 2;
      add(mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.02, 20).rotateZ(Math.PI / 2), mats.steel, s * (t2 - 0.1), 0, 0), axle);
      add(new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.03, 0.16), mats.chassis), axle).position.set(s * (ap.springTrack / 2), 0.055, 0);
    }
    axles.push(axle);

    for (const s of [-1, 1]) {
      const sx = s * ap.springTrack / 2;
      const topY = ap.droopY + ap.travel + 0.36;
      const coil = add(mesh(coilGeometry(0.085, 0.014, 7), mats.blackMetal), root);
      add(mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.03, 16), mats.chassis, sx, topY, ap.z), root);

      const shockX = s * (ap.damperTrack ? ap.damperTrack / 2 : ap.springTrack / 2 + 0.16);
      const shockZ = ap.z + (ai === 0 ? 0.12 : -0.12);
      const shockBody = add(mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.3, 12).translate(0, -0.15, 0), mats.yellow), root);
      const shockRod = add(mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.32, 8).translate(0, 0.16, 0), mats.chrome), root);
      suspension.push({ ai, s, coil, sx, topY, shockBody, shockRod, shockX, shockZ, shockTop: topY + 0.02 });
    }

    const links = [];
    for (const s of [-1, 1]) {
      const link = add(mesh(new THREE.BoxGeometry(0.06, 0.06, 1), mats.chassis), root);
      links.push({ link, s, pivot: new THREE.Vector3(s * 0.4, 0.5, ap.z + (ai === 0 ? 1.05 : -1.05)), end: new THREE.Vector3(s * 0.45, -0.06, 0) });
    }
    axle.userData.links = links;
    const pan = add(mesh(new THREE.BoxGeometry(1, 0.04, 0.04), mats.chassis), root);
    axle.userData.panhard = { mesh: pan, bodyEnd: new THREE.Vector3(-0.45, 0.62, ap.z + (ai === 0 ? 0.22 : -0.22)), axleEnd: new THREE.Vector3(0.55, 0.06, ai === 0 ? 0.22 : -0.22) };

    for (const s of [-1, 1]) {
      const w = buildWheel(mats, s, true);
      w.steer.position.set(s * t2, 0, 0);
      w.steer.userData.isWheel = true;
      axle.add(w.steer);
      w.side = s;
      w.axle = ai;
      wheels.push(w);
    }
    if (ai === 0) {
      const tie = add(mesh(new THREE.CylinderGeometry(0.018, 0.018, 1, 8), mats.chassis), axle);
      tie.rotation.z = Math.PI / 2;
      tie.position.set(0, -0.05, 0.16);
      tie.scale.y = 2 * t2 - 0.3;
      axle.userData.tie = tie;
    }
  }
  const props = [];
  for (let ai = 0; ai < 2; ai++) props.push(add(mesh(new THREE.CylinderGeometry(0.03, 0.03, 1, 10), mats.zinc), root));

  const dynamic = new Set([...cab.dynamic, spare.steer]);
  mergeStatic(body, o => dynamic.has(o) || o.userData.keep);
  mergeStatic(spare.steer);
  for (const ax of axles) mergeStatic(ax, o => o.userData.isWheel || o === ax.userData.tie);
  mergeStatic(cab.steeringWheel);

  const lights = buildLightRig(root, ext.anchors);

  root.traverse(o => { if (o.isMesh) o.frustumCulled = true; });

  return {
    root, body, wheels, axles, suspension, props, mats, lights,
    needles: cab.needles, gaugeMat: cab.gaugeMat, needleMat: cab.needleMat, warnMat: cab.warnMat,
    steeringWheel: cab.steeringWheel, gearLever: cab.gearLever, transferLever: cab.transferLever, spare,
    driverEye: cab.driverEye,
  };
}
