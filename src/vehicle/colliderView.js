import * as THREE from 'three';
import { D } from './truckDims.js';

// X-ray view of the physics: chassis collision boxes, wheel side cylinders, tyre outlines against the
// arch tops, suspension travel gauges, tyre contact patches and the points where the body touches the
// ground or an obstacle (the touching box turns red). Drawn on top of everything (no depth test).
//
// Two switches: 'physics' (Body and colliders tab) shows all of the above; 'susp' (Suspension tab) leaves
// out the collision boxes, wheel cylinders and body contacts and adds the springs (colour: load), the
// damper forces, static ride height and recent peak on the travel gauges, tyre load arrows and the axle
// lines. With 'susp' on, corners[] holds smoothed per-corner numbers for the panel.
//
// Nothing exists until it is first switched on, and while off the group is invisible and update() returns
// at once, so it costs nothing in normal play.

const COL = {
  box: new THREE.Color(0.25, 0.85, 1.0),
  hit: new THREE.Color(1.0, 0.22, 0.15),
  sel: new THREE.Color(1.0, 0.72, 0.28),
  wheel: new THREE.Color(0.55, 0.6, 1.0),
  tyre: new THREE.Color(0.85, 0.9, 0.95),
  ok: new THREE.Color(0.43, 0.86, 0.55),
  stop: new THREE.Color(1.0, 0.7, 0.28),
  cool: new THREE.Color(0.35, 0.7, 1.0),
  peak: new THREE.Color(1, 1, 1),
};
const G = 9.81;
const _v = new THREE.Vector3(), _q = new THREE.Quaternion(), _q2 = new THREE.Quaternion(), _m = new THREE.Matrix4();
const _p = new THREE.Vector3(), _cq = new THREE.Quaternion();

// Contacts of a collider after world.step(): the solver contacts are cleared by then, so this reads the
// manifold points and keeps those closer than 1 cm. cb(worldPoint) per point; returns the count.
export function touchPoints(world, c, cb) {
  let n = 0;
  world.contactPairsWith(c, other => {
    world.contactPair(c, other, (m, flipped) => {
      for (let j = 0, k = m.numContacts(); j < k; j++) {
        if (m.contactDist(j) > 0.01) continue;
        n++;
        if (cb) {
          const lp = flipped ? m.localContactPoint2(j) : m.localContactPoint1(j);
          const t = c.translation(), r = c.rotation();
          _cq.set(r.x, r.y, r.z, r.w);
          cb(_p.set(lp.x, lp.y, lp.z).applyQuaternion(_cq).add(_v.set(t.x, t.y, t.z)));
        }
      }
    });
  });
  return n;
}
const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);

const lineMat = (color, opacity = 1) => new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false, toneMapped: false });
const fillMat = (color, opacity) => new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false, toneMapped: false, side: THREE.DoubleSide });
const onTop = o => { o.renderOrder = 999; o.frustumCulled = false; return o; };

// unit circle in the y-z plane (a wheel seen from the side), n segments
function circle(n = 48) {
  const p = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2, b = ((i + 1) / n) * Math.PI * 2;
    p.push(0, Math.cos(a), Math.sin(a), 0, Math.cos(b), Math.sin(b));
  }
  return new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
}
// cylinder outline along x: two rims and four lines
function cylinderLines(n = 32) {
  const p = [];
  for (const x of [-1, 1]) for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2, b = ((i + 1) / n) * Math.PI * 2;
    p.push(x, Math.cos(a), Math.sin(a), x, Math.cos(b), Math.sin(b));
  }
  for (let k = 0; k < 4; k++) { const a = k * Math.PI / 2; p.push(-1, Math.cos(a), Math.sin(a), 1, Math.cos(a), Math.sin(a)); }
  return new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
}
// coil spring: unit radius, along y from 0 to 1
function coil(turns = 7, n = 14) {
  const p = [], N = turns * n;
  for (let i = 0; i < N; i++) {
    const a = (i / n) * Math.PI * 2, b = ((i + 1) / n) * Math.PI * 2;
    p.push(Math.cos(a), i / N, Math.sin(a), Math.cos(b), (i + 1) / N, Math.sin(b));
  }
  return new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
}
const segs = n => onTop(new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(n * 6), 3)), lineMat(0xffffff, 0.8)));

export class ColliderView {
  constructor(scene, model, vehicle) {
    this.scene = scene; this.m = model; this.v = vehicle;
    this.enabled = false;           // either view on
    this.physics = false;           // full physics view (boxes, cylinders, contacts)
    this.susp = false;              // suspension view
    this.corners = [];              // per corner, while 'susp' is on (see suspCorner)
    this.selected = -1;
    this.built = false;
    this.touching = new Set();      // chassis box indices touching something this frame
    this.contacts = 0;
  }

  setEnabled(on) { this.physics = !!on; this.setMode(); }
  setSusp(on) { this.susp = !!on; this.setMode(); }

  setMode() {
    const on = this.physics || this.susp;
    this.enabled = on;
    if (on && !this.built) this.build();
    if (!this.built) return;
    this.body.visible = on; this.world.visible = on;
    this.boxGroup.visible = this.physics;
    this.dots.visible = this.physics;
    for (const o of this.wheels) o.cyl.visible = this.physics;
    this.suspBody.visible = this.susp; this.suspWorld.visible = this.susp;
    for (const g of this.gauges) g.extra.visible = this.susp;
    if (!this.physics) this.touching.clear();
    if (this.susp) { this.corners = []; this._t = 0; }
    if (this.physics) this.rebuildBoxes();
  }

  build() {
    this.built = true;
    // body frame: lives under the truck root, so it follows the interpolated pose
    this.body = onTop(new THREE.Group()); this.body.name = 'collider-view';
    this.m.root.add(this.body);
    this.world = onTop(new THREE.Group()); this.world.name = 'collider-view-world';
    this.scene.add(this.world);
    this.boxGroup = new THREE.Group(); this.body.add(this.boxGroup);
    this.boxes = [];
    // wheels: side cylinder, tyre outline, travel gauge
    this.cyl = cylinderLines(); this.circ = circle();
    this.wheels = this.v.wheels.map(() => {
      const cyl = onTop(new THREE.LineSegments(this.cyl, lineMat(COL.wheel, 0.75)));
      const tyre = onTop(new THREE.LineSegments(this.circ, lineMat(COL.tyre, 0.9)));
      this.body.add(cyl, tyre);
      return { cyl, tyre };
    });
    // arch tops: a short line over each wheel at D.ARCH_TOP
    const ap = [];
    for (const z of [D.ARCH_F, D.ARCH_R]) for (const x of [-1, 1]) ap.push(x * 0.78, D.ARCH_TOP, z - 0.34, x * 0.78, D.ARCH_TOP, z + 0.34);
    this.arches = onTop(new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(ap, 3)), lineMat(0xffffff, 0.6)));
    this.body.add(this.arches);
    // travel gauges, outside the body next to each wheel: droop..bump, bump stop zone, current position
    this.gauges = this.v.wheels.map(w => {
      const g = new THREE.Group();
      const rail = onTop(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 0)]), lineMat(0xffffff, 0.45)));
      const stop = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.035, 1).translate(0, 0.5, 0), fillMat(COL.stop, 0.55)));
      const mark = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.09, 0.022), fillMat(COL.ok, 0.95)));
      // suspension view: static ride height and the recent peak compression
      const extra = new THREE.Group();
      const still = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.15, 0.01), fillMat(COL.cool, 0.9)));
      const peak = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.13, 0.008), fillMat(COL.peak, 0.85)));
      extra.add(still, peak);
      g.add(rail, stop, mark, extra);
      g.rotation.y = Math.PI / 2;
      this.body.add(g);
      return { g, rail, stop, mark, extra, still, peak, side: w.side };
    });
    // contact patches (world): a disc per wheel, size by load, colour by slip
    this.patches = this.v.wheels.map(() => {
      const d = onTop(new THREE.Mesh(new THREE.CircleGeometry(1, 24), fillMat(COL.ok, 0.7)));
      this.world.add(d);
      return d;
    });
    // body contact points (world)
    this.dots = onTop(new THREE.InstancedMesh(new THREE.SphereGeometry(0.045, 10, 8), fillMat(COL.hit, 0.95), 48));
    this.dots.count = 0;
    this.world.add(this.dots);
    this.buildSusp();
  }

  // suspension view: springs + damper bars (body frame), axle lines, roll centres, tyre load arrows (world)
  buildSusp() {
    this.suspBody = new THREE.Group(); this.body.add(this.suspBody);
    this.suspWorld = new THREE.Group(); this.world.add(this.suspWorld);
    const cg = coil();
    const bar = new THREE.PlaneGeometry(0.03, 1).translate(0, 0.5, 0);
    this.springs = this.v.wheels.map(() => {
      const spring = onTop(new THREE.LineSegments(cg, lineMat(COL.ok, 0.95)));
      const damper = onTop(new THREE.Mesh(bar, fillMat(COL.cool, 0.85)));
      damper.rotation.y = Math.PI / 2;
      this.suspBody.add(spring, damper);
      return { spring, damper };
    });
    this.axleLines = segs(2); this.suspBody.add(this.axleLines);
    this.rc = segs(4); this.rc.material.color.copy(COL.stop); this.suspBody.add(this.rc);
    const up = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 0)]);
    const tick = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-0.07, 0, 0), new THREE.Vector3(0.07, 0, 0), new THREE.Vector3(0, 0, -0.07), new THREE.Vector3(0, 0, 0.07)]);
    this.arrows = this.v.wheels.map(() => {
      const g = new THREE.Group();
      const line = onTop(new THREE.Line(up, lineMat(COL.ok, 0.95)));
      const still = onTop(new THREE.LineSegments(tick, lineMat(COL.cool, 0.9)));
      still.position.y = 0.6;
      g.add(line, still);
      this.suspWorld.add(g);
      return { g, line };
    });
  }

  // one corner's suspension state: travel, rates and forces at the spring seat, tyre load. Same
  // formulas as Vehicle.axleSubstep / cornerSubstep, read after the step.
  suspCorner(w, k) {
    const v = this.v, P = v.P, ax = w.axle, p = ax.p;
    let comp, rate, rateD;
    if (v.indep) { comp = w.c; rate = rateD = w.vz - w.vMountU; }
    else {
      const cd = ax.vz - ax.vMountU, cs = Math.cos(ax.phi) * (ax.Om - ax.rollRateBody);
      comp = v.wheelComp(w);
      rate = cd + w.side * (p.springTrack / 2) * cs;
      rateD = cd + w.side * (p.damperTrack / 2) * cs;
    }
    const spring = v.springForce(p, comp, rate, ax.k), damper = v.damperForce(p, rateD);
    const bar = w.side * (p.arb || 0) * v.axleRoll(ax.i) / P.track;
    const ff = (P.wheelbase / 2 - P.com[2]) / P.wheelbase, frac = ax.i === 0 ? ff : 1 - ff;
    const sprung = P.bodyMass * G * frac / 2;
    const load = w.contact ? w.FnAvg : 0;
    const c = this.corners[w.i] || (this.corners[w.i] = { comp, rate, spring, damper, bar, load, peak: comp });
    c.travel = p.travel; c.stop = p.travel - 0.05;
    c.sprung = sprung; c.staticComp = Math.max(0, sprung / ax.k - (p.preload || 0));
    c.staticLoad = (P.bodyMass * frac + p.mass) * G / 2;
    c.compNow = comp; c.springNow = spring; c.dampNow = damper; c.loadNow = load;
    c.comp += (comp - c.comp) * k; c.rate += (rate - c.rate) * k; c.spring += (spring - c.spring) * k;
    c.damper += (damper - c.damper) * k; c.bar += (bar - c.bar) * k; c.load += (load - c.load) * k;
    c.onStop = comp >= c.stop; c.topped = comp < 0.003;
    return c;
  }

  // chassis boxes from the vehicle's params (after an edit or a setup change)
  rebuildBoxes() {
    if (!this.built) return;
    for (const b of this.boxes) { this.boxGroup.remove(b.edges, b.fill); b.edges.geometry.dispose(); b.fill.geometry.dispose(); }
    this.boxes = this.v.P.colliders.map(([cx, cy, cz, hx, hy, hz]) => {
      const geo = new THREE.BoxGeometry(2 * hx, 2 * hy, 2 * hz);
      const edges = onTop(new THREE.LineSegments(new THREE.EdgesGeometry(geo), lineMat(COL.box, 0.95)));
      const fill = onTop(new THREE.Mesh(geo, fillMat(COL.box, 0.07)));
      edges.position.set(cx, cy, cz); fill.position.set(cx, cy, cz);
      this.boxGroup.add(edges, fill);
      return { edges, fill };
    });
  }

  update() {
    if (!this.enabled) return;
    const v = this.v, P = v.P, world = v.world, phys = this.physics, susp = this.susp;
    const now = performance.now(), dt = this._t ? Math.min(0.1, (now - this._t) / 1000) : 0;
    this._t = now;
    // which chassis boxes touch something, and where
    if (phys) this.updateBoxes(world, v);
    // wheels: side cylinders (body frame, same pose as Vehicle.updateGeometry), tyre outlines, gauges
    const R = v.R, ke = dt ? 1 - Math.exp(-dt / 0.12) : 1;
    v.wheels.forEach((w, i) => {
      const ax = w.axle, ap = ax.p, o = this.wheels[i];
      _q.setFromAxisAngle(Z, v.wheelRoll(w));
      v.hubLocal(w, _v);
      _q2.copy(_q).multiply(_q.setFromAxisAngle(Y, -w.steer));
      if (phys) {
        o.cyl.position.copy(_v); o.cyl.quaternion.copy(_q2);
        const cr = R - 0.12 * R / 0.42;
        o.cyl.scale.set(P.tire.width * 0.42, cr, cr);
      }
      o.tyre.position.copy(_v); o.tyre.quaternion.copy(_q2); o.tyre.scale.setScalar(R);
      // tyre top against the arch top (body frame): red when it reaches the arch
      const top = _v.y + R;
      const rub = top > D.ARCH_TOP - 0.01;
      o.tyre.material.color.copy(rub ? COL.hit : COL.tyre);
      // side cylinder touching a rock / log / tree
      if (phys) {
        const hitW = w.sideCollider ? touchPoints(world, w.sideCollider) > 0 : false;
        o.cyl.material.color.copy(hitW ? COL.hit : COL.wheel);
      }
      // travel gauge: spring compression at this side, 0 = full droop, travel = hard stop
      const g = this.gauges[i];
      const comp = v.wheelComp(w);
      g.g.position.set(w.side * (D.W + 0.12), ax.droopY, ap.z + (ax.i === 0 ? -0.62 : 0.62));
      g.rail.scale.y = ap.travel;
      g.stop.position.y = ap.travel - 0.05; g.stop.scale.y = 0.05;
      g.mark.position.y = Math.max(-0.02, Math.min(ap.travel + 0.02, comp));
      g.mark.material.color.copy(comp > ap.travel - 0.05 ? (comp >= ap.travel - 0.005 ? COL.hit : COL.stop) : COL.ok);
      // contact patch
      const d = this.patches[i];
      d.visible = w.contact && w.FnAvg > 0;
      if (d.visible) {
        d.position.copy(w.P).addScaledVector(w.n, 0.01);
        d.quaternion.setFromUnitVectors(Z, w.n);
        d.scale.setScalar(0.05 + 0.12 * Math.sqrt(w.FnAvg / 5000));
        const s = Math.min(1, w.slipNorm || 0);
        d.material.color.copy(COL.ok).lerp(COL.hit, s);
      }
      if (susp) this.updateSuspCorner(w, i, ke, dt);
    });
    if (susp) this.updateAxles();
  }

  updateBoxes(world, v) {
    this.touching.clear();
    let n = 0;
    const cols = v.chassisColliders;
    for (let i = 0; i < cols.length; i++) {
      const k = touchPoints(world, cols[i], p => { if (n < 48) this.dots.setMatrixAt(n++, _m.makeTranslation(p.x, p.y, p.z)); });
      if (k > 0) this.touching.add(i);
    }
    this.contacts = n;
    this.dots.count = n;
    this.dots.instanceMatrix.needsUpdate = true;
    this.boxes.forEach((b, i) => {
      const hit = this.touching.has(i), sel = i === this.selected;
      const c = hit ? COL.hit : sel ? COL.sel : COL.box;
      b.edges.material.color.copy(c);
      b.fill.material.color.copy(c);
      b.fill.material.opacity = hit ? 0.28 : sel ? 0.2 : 0.06;
    });
  }

  updateSuspCorner(w, i, ke, dt) {
    const v = this.v, P = v.P, ax = w.axle, ap = ax.p;
    const c = this.suspCorner(w, ke);
    // gauge ticks: static ride height, peak compression of the last couple of seconds
    const g = this.gauges[i];
    c.peak = Math.max(c.compNow, c.peak - dt * 0.06);
    g.still.position.y = Math.min(ap.travel, c.staticComp);
    g.peak.position.y = Math.max(0, Math.min(ap.travel + 0.02, c.peak));
    g.peak.material.color.copy(c.peak >= c.stop ? COL.hit : COL.peak);
    // spring: seat on the axle / corner up to a fixed seat on the body; colour by load against static
    const s = this.springs[i];
    const x = v.indep ? w.side * (P.track / 2 - 0.2) : w.side * (ap.springTrack ?? P.track * 0.7) / 2;
    const y0 = ax.droopY + c.compNow, y1 = ax.droopY + ap.travel + 0.1;
    s.spring.position.set(x, y0, ap.z);
    s.spring.scale.set(0.065, Math.max(0.02, y1 - y0), 0.065);
    const r = c.springNow / Math.max(1, c.sprung);
    const col = s.spring.material.color;
    if (c.onStop) col.copy(COL.hit);
    else if (r < 1) col.copy(COL.cool).lerp(COL.ok, Math.max(0, r));
    else col.copy(COL.ok).lerp(COL.stop, Math.min(1, (r - 1) / 1.2));
    // damper force bar beside the spring: up = bump (blue), down = rebound (orange), 0.1 m per kN
    const fd = c.damper, len = Math.min(0.4, Math.abs(fd) / 10000);
    s.damper.visible = len > 0.004;
    s.damper.position.set(x - w.side * 0.12, (y0 + y1) / 2, ap.z);
    s.damper.scale.y = fd >= 0 ? len : -len;
    s.damper.material.color.copy(fd >= 0 ? COL.cool : COL.stop);
    // tyre load arrow along the ground normal: 0.6 m (the blue tick) = static load
    const a = this.arrows[i];
    a.g.visible = w.contact && c.loadNow > 0;
    if (a.g.visible) {
      a.g.position.copy(w.P);
      a.g.quaternion.setFromUnitVectors(Y, w.n);
      const q = c.loadNow / Math.max(1, c.staticLoad);
      a.line.scale.y = Math.min(2, 0.6 * q);
      a.line.material.color.copy(q < 1 ? COL.cool : COL.ok).lerp(q < 1 ? COL.ok : COL.stop, q < 1 ? q : Math.min(1, (q - 1) / 0.8));
    }
  }

  // a line through the hub centres of each axle (axle articulation), roll centres of independent axles
  updateAxles() {
    const v = this.v, P = v.P;
    const pa = this.axleLines.geometry.attributes.position;
    for (let a = 0; a < 2; a++) for (let s = 0; s < 2; s++) {
      v.hubLocal(v.wheels[a * 2 + s], _v);
      pa.setXYZ(a * 2 + s, _v.x, _v.y, _v.z);
    }
    pa.needsUpdate = true;
    this.rc.visible = v.indep;
    if (v.indep) {
      const pr = this.rc.geometry.attributes.position;
      P.axles.forEach((p, a) => {
        const y = p.rcY ?? 0, z = p.z, k = a * 4;
        pr.setXYZ(k, -0.08, y, z); pr.setXYZ(k + 1, 0.08, y, z);
        pr.setXYZ(k + 2, 0, y - 0.08, z); pr.setXYZ(k + 3, 0, y + 0.08, z);
      });
      pr.needsUpdate = true;
    }
  }

  dispose() {
    if (!this.built) return;
    this.m.root.remove(this.body); this.scene.remove(this.world);
  }
}
