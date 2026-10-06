// Turns a downloaded car glTF/GLB (Sketchfab export) into a game-ready GLB:
// - every node transform baked into the vertices, metres, vehicle frame (+x right, +y up, -z forward,
//   y = 0 ground under the tyres, x = 0 centre line, z = 0 mid-wheelbase)
// - the scene split into `body`, `wheel_FL/FR/RL/RR` (spinning parts, origin at the hub) and
//   `caliper_FL/...` (steer with the wheel but don't spin, origin at the hub)
// - primitives merged per material inside each group, textures resized to webp
// Usage: node tools/prepcar.mjs <in.glb> <out.glb> <config.json>
// The config names the wheel materials and the real wheelbase (scale reference); see tools/cars/*.json.
import fs from 'node:fs';
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { flatten, clearNodeTransform, getBounds, join, dedup, weld, prune, textureCompress, transformPrimitive } from '@gltf-transform/functions';
import sharp from 'sharp';
import * as THREE from 'three';

const [inPath, outPath, cfgPath] = process.argv.slice(2);
const cfg = JSON.parse(fs.readFileSync(cfgPath, 'utf8'));
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
const doc = await io.read(inPath);
const root = doc.getRoot();
const scene = root.getDefaultScene() || root.listScenes()[0];

const spinMats = new Set(cfg.spin), caliperMats = new Set(cfg.caliper || []);
const matName = p => p.getMaterial()?.getName() || '';

// 1. world transforms into the vertices (each mesh node becomes a direct child of the scene)
await doc.transform(flatten());
for (const n of scene.listChildren()) if (n.getMesh()) clearNodeTransform(n);
// shared primitives would get transformed twice below: give every node its own copy
const seen = new Set();
for (const n of scene.listChildren()) {
  const mesh = n.getMesh(); if (!mesh) continue;
  for (const p of mesh.listPrimitives()) {
    if (seen.has(p)) { const c = p.clone(); mesh.swap(p, c); seen.add(c); } else seen.add(p);
  }
}

const primBounds = p => {
  const a = p.getAttribute('POSITION'), min = [1e9, 1e9, 1e9], max = [-1e9, -1e9, -1e9], v = [0, 0, 0];
  for (let i = 0; i < a.getCount(); i++) { a.getElement(i, v); for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], v[k]); max[k] = Math.max(max[k], v[k]); } }
  return { min, max, c: min.map((m, k) => (m + max[k]) / 2) };
};

// 2. hubs from the tyre primitives (the source's own units and axes)
const tyres = [];
for (const n of scene.listChildren()) for (const p of n.getMesh()?.listPrimitives() || []) if (matName(p) === cfg.tyre) tyres.push(primBounds(p));
if (tyres.length !== 4) throw new Error(`expected 4 '${cfg.tyre}' primitives, found ${tyres.length}`);
const fwd = cfg.forward;                 // source axis (index) that points forward, with its sign
const ax = fwd.axis, sg = fwd.sign;
tyres.sort((a, b) => sg * (b.c[ax] - a.c[ax]));   // front pair first
const front = tyres.slice(0, 2), rear = tyres.slice(2);
const zF = (front[0].c[ax] + front[1].c[ax]) / 2, zR = (rear[0].c[ax] + rear[1].c[ax]) / 2;
const wbUnits = Math.abs(zF - zR);
const s = cfg.wheelbase / wbUnits;
const ground = Math.min(...tyres.map(t => t.min[1]));
const latAx = 3 - ax - 1;               // the remaining horizontal axis (x if forward is z, and vice versa)
const centre = tyres.reduce((acc, t) => acc + t.c[latAx], 0) / 4;
const mid = (zF + zR) / 2;
// source -> vehicle frame: forward source axis maps to -z, the lateral axis to x (right-handed with y up)
const M = new THREE.Matrix4();
if (ax === 2) {
  // forward = +z (sign 1): rotate 180 about y -> x' = -(x - c), z' = -(z - mid)
  M.set(-sg * s, 0, 0, sg * s * centre,
        0, s, 0, -s * ground,
        0, 0, -sg * s, sg * s * mid,
        0, 0, 0, 1);
} else {
  // forward = +x (sign 1): x' = (z - c) * sg, z' = -(x - mid) * sg  (rotation about y)
  M.set(0, 0, sg * s, -sg * s * centre,
        0, s, 0, -s * ground,
        -sg * s, 0, 0, sg * s * mid,
        0, 0, 0, 1);
}
const toCar = v => new THREE.Vector3(...v).applyMatrix4(M);
const hubs = tyres.map(t => toCar(t.c));
// name them: front first, then left (x < 0) before right
const named = {};
for (const h of hubs) named[(h.z < 0 ? 'F' : 'R') + (h.x < 0 ? 'L' : 'R')] = h;
if (Object.keys(named).length !== 4) throw new Error('could not tell the four wheels apart');

const mArr = M.toArray();
for (const n of scene.listChildren()) for (const p of n.getMesh()?.listPrimitives() || []) transformPrimitive(p, mArr);

// 3. each tyre's real axle: the source may pose the wheels steered or cambered. The axle is the
// direction of least spread of the tyre's vertices (a tyre is a flat ring). Every wheel is stored
// straight, so the view can steer it from zero.
const covEig = pts => {
  const n = pts.length / 3, m = [0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) m[k] += pts[i * 3 + k] / n;
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < n; i++) for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) A[r][c] += (pts[i * 3 + r] - m[r]) * (pts[i * 3 + c] - m[c]) / n;
  // Jacobi rotations
  const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 50; sweep++) for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
    if (Math.abs(A[p][q]) < 1e-12) continue;
    const th = 0.5 * Math.atan2(2 * A[p][q], A[q][q] - A[p][p]), c = Math.cos(th), sn = Math.sin(th);
    for (let k = 0; k < 3; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - sn * akq; A[k][q] = sn * akp + c * akq; }
    for (let k = 0; k < 3; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - sn * aqk; A[q][k] = sn * apk + c * aqk; }
    for (let k = 0; k < 3; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - sn * vkq; V[k][q] = sn * vkp + c * vkq; }
  }
  const i = [0, 1, 2].reduce((b, k) => (A[k][k] < A[b][b] ? k : b), 0);
  return { mean: new THREE.Vector3(...m), axle: new THREE.Vector3(V[0][i], V[1][i], V[2][i]).normalize() };
};
const nearestHub = c => Object.entries(named).reduce((best, [k, h]) => {
  const d = Math.hypot(c[0] - h.x, c[1] - h.y, c[2] - h.z); return d < best.d ? { k, d } : best;
}, { k: null, d: Infinity });
// A tyre can be asymmetric (sidewall detail on one side) and tilt that fit by a few degrees, which makes
// the wheel wobble as it spins: `cfg.axle` names a flat part (brake disc face) to take the axle from.
const axleRef = {};
if (cfg.axle) {
  const pts = {};
  for (const n of scene.listChildren()) for (const p of n.getMesh()?.listPrimitives() || []) {
    if (matName(p) !== cfg.axle) continue;
    const { k } = nearestHub(primBounds(p).c);
    (pts[k] ||= []).push(...p.getAttribute('POSITION').getArray());
  }
  for (const [k, a] of Object.entries(pts)) axleRef[k] = covEig(a).axle;
}
const pose = {};    // wheel key -> quaternion that puts the straight wheel back in its posed camber / toe
const size = {};    // wheel key -> { r, w } of the straight tyre
const local = {};   // wheel key -> matrix from car frame to the straight wheel frame (origin at the hub)
for (const n of scene.listChildren()) for (const p of n.getMesh()?.listPrimitives() || []) {
  if (matName(p) !== cfg.tyre) continue;
  const { k } = nearestHub(primBounds(p).c);
  const fit = covEig(p.getAttribute('POSITION').getArray());
  const mean = fit.mean, axle = axleRef[k] || fit.axle;
  if (axle.x < 0) axle.negate();
  const L = new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromUnitVectors(axle, new THREE.Vector3(1, 0, 0)))
    .multiply(new THREE.Matrix4().makeTranslation(-mean.x, -mean.y, -mean.z));
  const a = p.getAttribute('POSITION'), v = new THREE.Vector3(), min = new THREE.Vector3(1e9, 1e9, 1e9), max = new THREE.Vector3(-1e9, -1e9, -1e9);
  for (let i = 0; i < a.getCount(); i++) { v.fromArray(a.getElement(i, [])).applyMatrix4(L); min.min(v); max.max(v); }
  const b = min.clone().add(max).multiplyScalar(0.5);
  local[k] = new THREE.Matrix4().makeTranslation(-b.x, -b.y, -b.z).multiply(L);
  named[k] = b.clone().applyMatrix4(L.clone().invert());
  size[k] = { r: (max.y - min.y) / 2, w: max.x - min.x };
  pose[k] = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), axle).toArray().map(x => +x.toFixed(5));
  const toe = Math.atan2(axle.z, axle.x) * 180 / Math.PI, camber = Math.asin(axle.y) * 180 / Math.PI;
  console.log(`${k}: hub ${named[k].toArray().map(x => x.toFixed(3))}  posed steer ${toe.toFixed(2)} deg, camber ${camber.toFixed(2)} deg, size ${(max.y - min.y).toFixed(3)} x ${(max.x - min.x).toFixed(3)}`);
}
const tyreR = Math.max(...Object.values(size).map(t => t.r)), tyreW = Math.max(...Object.values(size).map(t => t.w));

// 4. regroup: body, wheel_XX, caliper_XX
const group = name => { const g = doc.createNode(name); scene.addChild(g); return g; };
const groups = { body: group('body') };
for (const k of ['FL', 'FR', 'RL', 'RR']) {
  groups['wheel_' + k] = group('wheel_' + k).setTranslation(named[k].toArray());
  groups['caliper_' + k] = group('caliper_' + k).setTranslation(named[k].toArray());
}
let moved = 0;
for (const n of [...scene.listChildren()]) {
  const mesh = n.getMesh(); if (!mesh) continue;
  for (const p of mesh.listPrimitives()) {
    const name = matName(p);
    let dest = 'body';
    if (spinMats.has(name) || caliperMats.has(name)) {
      const { k, d } = nearestHub(primBounds(p).c);
      if (d < tyreR * 1.2) {
        dest = (spinMats.has(name) ? 'wheel_' : 'caliper_') + k;
        transformPrimitive(p, local[k].toArray());
      }
    }
    const m = doc.createMesh(n.getName() + '_' + dest).addPrimitive(p);
    groups[dest].addChild(doc.createNode(n.getName()).setMesh(m));
    mesh.removePrimitive(p);
    if (dest !== 'body') moved++;
  }
  n.dispose();
}
// 5. clean-up and merge per material inside each group
await doc.transform(
  prune(),
  dedup(),
  weld(),
  join({ keepNamed: false }),
  prune(),
  textureCompress({ encoder: sharp, targetFormat: 'webp', resize: [cfg.maxTexture || 2048, cfg.maxTexture || 2048] }),
);
// drop the empty intermediate nodes join() leaves behind
for (const g of Object.values(groups)) for (const c of g.listChildren()) if (!c.getMesh() && !c.listChildren().length) c.dispose();

const b = getBounds(scene);
scene.setExtras({
  car: cfg.name, credit: cfg.credit,
  wheelbase: cfg.wheelbase,
  hubs: Object.fromEntries(Object.entries(named).map(([k, h]) => [k, h.toArray().map(v => +v.toFixed(4))])),
  pose,
  tyres: Object.fromEntries(Object.entries(size).map(([k, t]) => [k, { r: +t.r.toFixed(4), w: +t.w.toFixed(4) }])),
  tyreRadius: +tyreR.toFixed(4), tyreWidth: +tyreW.toFixed(4),
  bounds: { min: b.min.map(v => +v.toFixed(3)), max: b.max.map(v => +v.toFixed(3)) },
});
await io.write(outPath, doc);

let tris = 0, prims = 0;
for (const m of root.listMeshes()) for (const p of m.listPrimitives()) { prims++; tris += (p.getIndices()?.getCount() ?? p.getAttribute('POSITION').getCount()) / 3; }
console.log(JSON.stringify(scene.getExtras(), null, 1));
console.log(`scale ${s.toFixed(5)} m/unit, ${moved} wheel primitives, ${prims} draw calls, ${tris | 0} triangles, ${(fs.statSync(outPath).size / 1e6).toFixed(1)} MB`);
