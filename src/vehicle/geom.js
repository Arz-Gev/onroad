import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export function mesh(geo, mat, x = 0, y = 0, z = 0) {
  const o = new THREE.Mesh(geo, mat);
  o.position.set(x, y, z);
  o.castShadow = true;
  o.receiveShadow = true;
  return o;
}

export const rbox = (w, h, d, r, mat, x = 0, y = 0, z = 0, seg = 2) =>
  mesh(new RoundedBoxGeometry(w, h, d, seg, Math.min(r, w / 2 - 1e-4, h / 2 - 1e-4, d / 2 - 1e-4)), mat, x, y, z);
export const box = (w, h, d, mat, x = 0, y = 0, z = 0) => mesh(new THREE.BoxGeometry(w, h, d), mat, x, y, z);

export function span(x0, x1, y0, y1, z0, z1, mat, r = 0) {
  if (x0 > x1) [x0, x1] = [x1, x0];
  if (y0 > y1) [y0, y1] = [y1, y0];
  if (z0 > z1) [z0, z1] = [z1, z0];
  const w = x1 - x0, h = y1 - y0, d = z1 - z0;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, cz = (z0 + z1) / 2;
  return r > 0 ? rbox(w, h, d, r, mat, cx, cy, cz) : box(w, h, d, mat, cx, cy, cz);
}

const _up = new THREE.Vector3(0, 1, 0);

export function bar(a, b, r, mat, seg = 10, r2 = r) {
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
  const len = A.distanceTo(B);
  const m = mesh(new THREE.CylinderGeometry(r2, r, len, seg), mat);
  m.position.copy(A).add(B).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(_up, B.clone().sub(A).normalize());
  return m;
}

export function pipe(points, r, mat, rc = 0.06, seg = 8, closed = false) {
  const P = points.map(p => new THREE.Vector3(...p));
  const path = new THREE.CurvePath();
  const n = P.length;
  const corner = (i) => {
    const a = P[(i - 1 + n) % n], b = P[i], c = P[(i + 1) % n];
    const da = a.clone().sub(b), dc = c.clone().sub(b);
    const k = Math.min(rc, da.length() * 0.45, dc.length() * 0.45);
    return [b.clone().addScaledVector(da.normalize(), k), b, b.clone().addScaledVector(dc.normalize(), k)];
  };
  let prev = closed ? corner(0)[2] : P[0];
  const last = closed ? n : n - 1;
  for (let i = 1; i <= last; i++) {
    const idx = i % n;
    if (!closed && i === n - 1) { path.add(new THREE.LineCurve3(prev, P[idx])); break; }
    const [p0, p1, p2] = corner(idx);
    path.add(new THREE.LineCurve3(prev, p0));
    path.add(new THREE.QuadraticBezierCurve3(p0, p1, p2));
    prev = p2;
  }
  const lenTotal = path.getLength();
  const geo = new THREE.TubeGeometry(path, Math.max(8, Math.ceil(lenTotal / 0.05)), r, seg, false);
  return mesh(geo, mat);
}

export function extrudeProfile(points, depth, mat, bevel = 0.015, holes = [], curveSegments = 6) {
  const shape = new THREE.Shape(points.map(([z, y]) => new THREE.Vector2(z, y)));
  for (const h of holes) shape.holes.push(h);
  const geo = new THREE.ExtrudeGeometry(shape, { depth: depth - 2 * bevel, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelOffset: -bevel, bevelSegments: 2, curveSegments });
  geo.translate(0, 0, -(depth - 2 * bevel) / 2);
  geo.rotateY(-Math.PI / 2);
  return mesh(geo, mat);
}

export function extrudeFront(points, depth, mat, bevel = 0.01, holes = [], curveSegments = 8) {
  const shape = new THREE.Shape(points.map(([x, y]) => new THREE.Vector2(x, y)));
  for (const h of holes) shape.holes.push(h);
  const geo = new THREE.ExtrudeGeometry(shape, { depth: Math.max(1e-4, depth - 2 * bevel), bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelOffset: -bevel, bevelSegments: 2, curveSegments });
  geo.translate(0, 0, -(depth - 2 * bevel) / 2);
  return mesh(geo, mat);
}

export function extrudePlan(points, height, mat, bevel = 0.01, curveSegments = 8) {
  const shape = new THREE.Shape(points.map(([x, z]) => new THREE.Vector2(x, -z)));
  const geo = new THREE.ExtrudeGeometry(shape, { depth: Math.max(1e-4, height - 2 * bevel), bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelOffset: -bevel, bevelSegments: 2, curveSegments });
  geo.translate(0, 0, bevel);
  geo.rotateX(-Math.PI / 2);
  return mesh(geo, mat);
}

export function roundRect(shapeOrPath, x0, y0, x1, y1, r) {
  const s = shapeOrPath;
  r = Math.min(r, (x1 - x0) / 2, (y1 - y0) / 2);
  s.moveTo(x0 + r, y0);
  s.lineTo(x1 - r, y0); s.quadraticCurveTo(x1, y0, x1, y0 + r);
  s.lineTo(x1, y1 - r); s.quadraticCurveTo(x1, y1, x1 - r, y1);
  s.lineTo(x0 + r, y1); s.quadraticCurveTo(x0, y1, x0, y1 - r);
  s.lineTo(x0, y0 + r); s.quadraticCurveTo(x0, y0, x0 + r, y0);
  return s;
}
export const rrShape = (x0, y0, x1, y1, r) => roundRect(new THREE.Shape(), x0, y0, x1, y1, r);
export const rrPath = (x0, y0, x1, y1, r) => roundRect(new THREE.Path(), x0, y0, x1, y1, r);

export function plate(x0, y0, x1, y1, r, depth, mat, bevel = 0.004, holes = []) {
  const s = rrShape(x0, y0, x1, y1, r);
  for (const h of holes) s.holes.push(h);
  const geo = new THREE.ExtrudeGeometry(s, { depth: Math.max(1e-4, depth - 2 * bevel), bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelOffset: -bevel, bevelSegments: 1, curveSegments: 6 });
  geo.translate(0, 0, -(depth - 2 * bevel) / 2);
  return mesh(geo, mat);
}

export function frame(x0, y0, x1, y1, r, w, depth, mat, bevel = 0.003) {
  return plate(x0 - w, y0 - w, x1 + w, y1 + w, r + w, depth, mat, bevel, [rrPath(x0, y0, x1, y1, r)]);
}

export function warpGroup(group, fn) {
  group.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(group.matrixWorld).invert();
  const m = new THREE.Matrix4(), nm = new THREE.Matrix3();
  const p = new THREE.Vector3(), n = new THREE.Vector3();
  group.traverse(o => {
    if (!o.isMesh) return;
    m.multiplyMatrices(inv, o.matrixWorld);
    const g = o.geometry.clone();
    g.applyMatrix4(m);
    nm.getNormalMatrix(m);
    const pos = g.attributes.position, nor = g.attributes.normal;
    for (let i = 0; i < pos.count; i++) {
      p.fromBufferAttribute(pos, i);
      n.fromBufferAttribute(nor, i);
      fn(p, n);
      pos.setXYZ(i, p.x, p.y, p.z);
      nor.setXYZ(i, n.x, n.y, n.z);
    }
    o.geometry = g;

    const parentInv = new THREE.Matrix4().copy(o.parent.matrixWorld).invert();
    const local = new THREE.Matrix4().multiplyMatrices(parentInv, group.matrixWorld);
    local.decompose(o.position, o.quaternion, o.scale);
  });
}

export function tumblehome(y0, k) {
  return (p, n) => {
    const a = 1 - k * Math.max(0, p.y - y0);
    const active = p.y > y0 ? 1 : 0;
    const x = p.x;
    p.x = x * a;

    if (active) {
      const nx = n.x / a, ny = n.y + (k * x * n.x) / a;
      n.set(nx, ny, n.z).normalize();
    }
  };
}

export function mergeStatic(group, skip = () => false) {
  group.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(group.matrixWorld).invert();
  const buckets = new Map();
  const remove = [];
  const visit = (o) => {
    for (const c of [...o.children]) {
      if (skip(c)) continue;
      if (c.isMesh && c.children.length === 0 && !c.isInstancedMesh) {
        const g = c.geometry.index ? c.geometry.toNonIndexed() : c.geometry.clone();
        g.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, c.matrixWorld));
        for (const k of Object.keys(g.attributes)) if (k !== 'position' && k !== 'normal' && k !== 'uv') g.deleteAttribute(k);
        if (!g.attributes.uv) g.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2));
        g.morphAttributes = {};
        const key = c.material.uuid + (c.castShadow ? 'c' : 'n');
        if (!buckets.has(key)) buckets.set(key, { mat: c.material, geos: [], cast: c.castShadow });
        buckets.get(key).geos.push(g);
        remove.push(c);
      } else if (!c.isLight) visit(c);
    }
  };
  visit(group);
  for (const c of remove) c.parent.remove(c);
  const out = [];
  for (const { mat, geos, cast } of buckets.values()) {
    const merged = mergeGeometries(geos);
    merged.computeBoundingSphere();
    const m = new THREE.Mesh(merged, mat);
    m.castShadow = cast && !mat.transparent;
    m.receiveShadow = true;
    group.add(m);
    out.push(m);
  }
  return out;
}

export { mergeGeometries };

export function roundPoly(target, pts, r) {
  const n = pts.length;
  const V = pts.map(([a, b]) => new THREE.Vector2(a, b));
  if (!(r > 0)) {
    V.forEach((p, i) => (i === 0 ? target.moveTo(p.x, p.y) : target.lineTo(p.x, p.y)));
    target.closePath();
    return target;
  }
  for (let i = 0; i < n; i++) {
    const p = V[i], a = V[(i - 1 + n) % n], c = V[(i + 1) % n];
    const da = a.clone().sub(p), dc = c.clone().sub(p);
    const k = Math.min(r, da.length() * 0.45, dc.length() * 0.45);
    const p0 = p.clone().addScaledVector(da.normalize(), k), p2 = p.clone().addScaledVector(dc.normalize(), k);
    if (i === 0) target.moveTo(p0.x, p0.y); else target.lineTo(p0.x, p0.y);
    target.quadraticCurveTo(p.x, p.y, p2.x, p2.y);
  }
  target.closePath();
  return target;
}
export const polyShape = (pts, r = 0) => roundPoly(new THREE.Shape(), pts, r);
export const polyPath = (pts, r = 0) => roundPoly(new THREE.Path(), pts, r);

export function extrudeShape(shape, depth, mat, plane = 'zy', bevel = 0.006, curveSegments = 6) {
  const geo = new THREE.ExtrudeGeometry(shape, { depth: Math.max(1e-4, depth - 2 * bevel), bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelOffset: -bevel, bevelSegments: 2, curveSegments });
  geo.translate(0, 0, -(depth - 2 * bevel) / 2);
  if (plane === 'zy') geo.rotateY(-Math.PI / 2);
  else if (plane === 'xz') geo.rotateX(Math.PI / 2);
  return mesh(geo, mat);
}
