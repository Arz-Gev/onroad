import * as THREE from 'three';

// X-ray view of the physics: chassis collision boxes, wheel side cylinders, suspension travel gauges,
// tyre contact patches and the points where the body touches the
// ground or an obstacle (the touching box turns red). Drawn on top of everything (no depth test).
//
// Two switches: 'physics' (Body and colliders tab) shows all of the above; 'susp' (Suspension tab) leaves
// out the collision boxes, wheel cylinders and body contacts and adds the damper
// forces, static ride height on the travel gauges and the roll centres. With 'susp' on, corners[] holds smoothed per-corner numbers for the panel.
//
// Nothing exists until it is first switched on, and while off the group is invisible and update() returns
// at once, so it costs nothing in normal play.

const COL = {
  box: new THREE.Color(0.25, 0.85, 1.0),
  hit: new THREE.Color(1.0, 0.22, 0.15),
  sel: new THREE.Color(1.0, 0.72, 0.28),
  wheel: new THREE.Color(0.55, 0.6, 1.0),
  ok: new THREE.Color(0.43, 0.86, 0.55),
  stop: new THREE.Color(1.0, 0.7, 0.28),
  cool: new THREE.Color(0.35, 0.7, 1.0),
};
const G = 9.81;
const PATCH_R = 0.2;   // contact patch radius at static load (m)
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
    // wheels: side cylinder, travel gauge
    this.cyl = cylinderLines();
    this.wheels = this.v.wheels.map(() => {
      const cyl = onTop(new THREE.LineSegments(this.cyl, lineMat(COL.wheel, 0.75)));
      this.body.add(cyl);
      return { cyl };
    });
    // travel gauges, outside the body next to each wheel: droop..bump, bump stop zone, current position
    this.gauges = this.v.wheels.map(w => {
      const g = new THREE.Group();
      const rail = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.014, 1).translate(0, 0.5, 0), fillMat(0xffffff, 0.85)));
      const stop = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.035, 1).translate(0, 0.5, 0), fillMat(COL.stop, 0.9)));
      const mark = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.09, 0.022), fillMat(COL.ok, 1)));
      // suspension view: static ride height
      const extra = new THREE.Group();
      const still = onTop(new THREE.Mesh(new THREE.PlaneGeometry(0.15, 0.01), fillMat(COL.cool, 1)));
      extra.add(still);
      g.add(rail, stop, mark, extra);
      g.rotation.y = Math.PI / 2;
      this.body.add(g);
      return { g, rail, stop, mark, extra, still, side: w.side };
    });
    // contact patches (world): a disc per wheel, radius in proportion to the load (the ring = static
    // load), colour by slip
    const ring = new THREE.RingGeometry(PATCH_R - 0.012, PATCH_R, 48);
    this.patches = this.v.wheels.map(() => {
      const g = new THREE.Group();
      const disc = onTop(new THREE.Mesh(new THREE.CircleGeometry(1, 36), fillMat(COL.ok, 0.35)));
      g.add(disc, onTop(new THREE.Mesh(ring, fillMat(0xffffff, 0.9))));
      this.world.add(g);
      return { g, disc };
    });
    // body contact points (world)
    this.dots = onTop(new THREE.InstancedMesh(new THREE.SphereGeometry(0.045, 10, 8), fillMat(COL.hit, 0.95), 48));
    this.dots.count = 0;
    this.world.add(this.dots);
    this.buildSusp();
  }

  // suspension view: damper bars (body frame), roll centres
  buildSusp() {
    this.suspBody = new THREE.Group(); this.body.add(this.suspBody);
    this.suspWorld = new THREE.Group(); this.world.add(this.suspWorld);
    const bar = new THREE.PlaneGeometry(0.03, 1).translate(0, 0.5, 0);
    this.springs = this.v.wheels.map(() => {
      const damper = onTop(new THREE.Mesh(bar, fillMat(COL.cool, 0.85)));
      damper.rotation.y = Math.PI / 2;
      this.suspBody.add(damper);
      return { damper };
    });
    // roll centres (orange) and the centre of mass (white): balls in the body frame
    const ball = new THREE.SphereGeometry(1, 16, 12);
    this.rc = [0, 1].map(() => { const m = onTop(new THREE.Mesh(ball, fillMat(COL.stop, 0.95))); m.scale.setScalar(0.045); this.suspBody.add(m); return m; });
    this.com = onTop(new THREE.Mesh(ball, fillMat(0xffffff, 0.95))); this.com.scale.setScalar(0.06); this.suspBody.add(this.com);
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
    const c = this.corners[w.i] || (this.corners[w.i] = { comp, rate, spring, damper, bar, load });
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
    // wheels: side cylinders (body frame, same pose as Vehicle.updateGeometry), gauges
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
      // side cylinder touching a rock / log / tree
      if (phys) {
        const hitW = w.sideCollider ? touchPoints(world, w.sideCollider) > 0 : false;
        o.cyl.material.color.copy(hitW ? COL.hit : COL.wheel);
      }
      // travel gauge: spring compression at this side, 0 = full droop, travel = hard stop
      const g = this.gauges[i];
      const comp = v.wheelComp(w);
      g.g.position.set(w.side * (P.track / 2 + P.tire.width / 2 + 0.15), ax.droopY, ap.z);   // just outside the tyre, at the axle
      g.rail.scale.y = ap.travel;
      g.stop.position.y = ap.travel - 0.05; g.stop.scale.y = 0.05;
      g.mark.position.y = Math.max(-0.02, Math.min(ap.travel + 0.02, comp));
      g.mark.material.color.copy(comp > ap.travel - 0.05 ? (comp >= ap.travel - 0.005 ? COL.hit : COL.stop) : COL.ok);
      // contact patch
      const d = this.patches[i];
      d.g.visible = w.contact && w.FnAvg > 0;
      if (d.g.visible) {
        d.g.position.copy(w.P).addScaledVector(w.n, 0.01);
        d.g.quaternion.setFromUnitVectors(Z, w.n);
        const ff = (P.wheelbase / 2 - P.com[2]) / P.wheelbase, frac = ax.i === 0 ? ff : 1 - ff;
        const still = (P.bodyMass * frac + ap.mass) * G / 2;
        d.disc.scale.setScalar(Math.max(0.02, Math.min(0.8, PATCH_R * w.FnAvg / still)));
        const s = Math.min(1, w.slipNorm || 0);
        d.disc.material.color.copy(COL.ok).lerp(COL.hit, s);
      }
      if (susp) this.updateSuspCorner(w, i, ke, dt);
    });
    if (susp) this.updateRollCentres();
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
    // gauge tick: static ride height
    const g = this.gauges[i];
    g.still.position.y = Math.min(ap.travel, c.staticComp);
    // damper force bar where the spring sits: up = bump (blue), down = rebound (orange), 0.1 m per kN
    const s = this.springs[i];
    const x = v.indep ? w.side * (P.track / 2 - 0.2) : w.side * (ap.springTrack ?? P.track * 0.7) / 2;
    const y0 = ax.droopY + c.compNow, y1 = ax.droopY + ap.travel + 0.1;
    const fd = c.damper, len = Math.min(0.4, Math.abs(fd) / 10000);
    s.damper.visible = len > 0.004;
    s.damper.position.set(x - w.side * 0.12, (y0 + y1) / 2, ap.z);
    s.damper.scale.y = fd >= 0 ? len : -len;
    s.damper.material.color.copy(fd >= 0 ? COL.cool : COL.stop);
  }

  // roll centres of independent axles (they follow the Roll centre sliders) and the centre of mass
  updateRollCentres() {
    const v = this.v, P = v.P;
    P.axles.forEach((p, a) => { this.rc[a].visible = v.indep; this.rc[a].position.set(0, p.rcY ?? 0, p.z); });
    this.com.position.set(P.com[0], P.com[1], P.com[2]);
  }

  dispose() {
    if (!this.built) return;
    this.m.root.remove(this.body); this.scene.remove(this.world);
  }
}
