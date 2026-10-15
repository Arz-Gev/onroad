import * as THREE from 'three';
import { Drivetrain } from './drivetrain.js';
import { SURFACES, tireCoefs, tireForces, tireRelax } from './tire.js';
import { SteerFilter } from './steerFilter.js';

const V3 = THREE.Vector3;
const G = 9.81;
const DEG = Math.PI / 180;
const FAN = [];
for (let i = -6; i <= 6; i++) FAN.push((i / 6) * (80 * Math.PI / 180));

const _q = new THREE.Quaternion();
const _q2 = new THREE.Quaternion();
const _v = new V3(), _v2 = new V3(), _v3 = new V3();
const X = new V3(1, 0, 0), Y = new V3(0, 1, 0), Z = new V3(0, 0, 1);

const GROUND_BIT = 0x0001, WHEEL_BIT = 0x0002;
export const GROUP_GROUND = (GROUND_BIT << 16) | 0xffff;
const GROUP_WHEEL_SIDE = (WHEEL_BIT << 16) | (0xffff & ~GROUND_BIT);

export const STEER_ASSIST = { strong: { slip: 1.0 }, light: { slip: 1.6 } };
const HAND_RATE = 23;
// Front-wheel rad/s the hands may move at with the BeamNG filter, which already limits the rate itself.
const BEAM_RACK_RATE = 6;

export class Vehicle {
  constructor(RAPIER, world, P, opts = {}) {
    this.RAPIER = RAPIER;
    this.world = world;
    this.P = P;
    this.surfaceAt = opts.surfaceAt || (() => SURFACES.dirt);
    this.indep = P.suspension === 'independent';
    this.substeps = opts.substeps || 4;
    this.pressures = [P.tire.pressure, P.tire.pressure];

    this.R = P.tire.radius;

    const unsprung = P.axles[0].mass + P.axles[1].mass;
    this.totalMass = P.bodyMass + unsprung;
    const pos = opts.position || { x: 0, y: 1, z: 0 };
    const rot = new THREE.Quaternion().setFromAxisAngle(Y, opts.yaw || 0);
    const desc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(pos.x, pos.y, pos.z)
      .setRotation({ x: rot.x, y: rot.y, z: rot.z, w: rot.w })
      .setAdditionalMassProperties(this.totalMass, { x: P.com[0], y: P.com[1], z: P.com[2] },
        { x: P.bodyInertia[0], y: P.bodyInertia[1], z: P.bodyInertia[2] }, { x: 0, y: 0, z: 0, w: 1 })
      .setCanSleep(false)
      .setCcdEnabled(true)
      .setAngularDamping(0.02);
    this.body = world.createRigidBody(desc);
    this.chassisColliders = [];
    this.setColliders(P.colliders);

    this.drivetrain = new Drivetrain(P);
    this.ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });

    const L = P.wheelbase;
    const frontFrac = (L / 2 - P.com[2]) / L;
    this.axles = P.axles.map((p, i) => {
      const load = P.bodyMass * G * (i === 0 ? frontFrac : 1 - frontFrac) / 2;
      return {
        p, i, droopY: p.droopY, k: p.k,
        c: Math.min(p.travel * 0.9, load / p.k - p.preload),
        vz: 0, phi: 0, Om: 0,
        A: new V3(), q: new THREE.Quaternion(), mount: new V3(), vMountU: 0, rollRateBody: 0,
        S: [0, 0], accS: [0, 0], accD: [0, 0], accArb: 0,
      };
    });

    this.wheels = [];
    for (let i = 0; i < 4; i++) {
      const ax = this.axles[i < 2 ? 0 : 1];
      this.wheels.push({
        i, axle: ax, side: i % 2 === 0 ? -1 : 1, steer: 0,
        hub: new V3(), q: new THREE.Quaternion(), spin: 0,
        spinAxis: new V3(), fwd: new V3(), up: new V3(),
        contact: false, pen: -1, penDot: 0, n: new V3(0, 1, 0), nLocal: new V3(0, 1, 0), P: new V3(),
        latOff: 0, surf: SURFACES.dirt, fc: new V3(), sc: new V3(), vcx: 0, vcy: 0, vHubPerp: new V3(),
        ux: 0, uy: 0, Fx: 0, Fy: 0, Fn: 0, Re: P.tire.radius, slipNorm: 0, slipSteady: 0,
        co: {}, accF: new V3(), FnAvg: 0, slipVel: 0, collider: null,

        c: ax.c, vz: 0, vMountU: 0, S: 0, accU: 0, accLat: 0, accJL: 0, accLong: 0, accJF: 0,
      });
    }

    world.forEachCollider(c => {
      if (c.shape.type === RAPIER.ShapeType.HeightField) c.setCollisionGroups(GROUP_GROUND);
    });
    this.makeWheelColliders();

    this.ctl = { throttle: 0, brake: 0, steer: 0, clutch: 0, handbrake: 0 };
    this.steerAngle = 0;
    this.steerComp = 0;
    this.alignM = 0; this.handsOff = false;
    this.steerVel = 0; this.hand = 0; this.grip = 1;
    this.revTimer = 0;
    this.gripG = 0.63; this.gripSlip = 0.16;
    this.steerAssist = 'strong';
    this.arcadeAuto = false;
    this.speed = 0;
    this.tireT = new Float64Array(4);
    this.rrT = new Float64Array(4);
    this.brakeT = new Float64Array(4);
    this.absFactor = new Float64Array([1, 1, 1, 1]);
    this.abs = true;
    this.hbMode = 'hold';
    this.hbLatched = false;
    this._hbPrev = 0;
    this._hbPress = 0;
    this._hbWas = false;
    this.absActive = 0;
    this.tc = P.layout !== 'rwd';
    this.tcActive = 0;
    this.tcT = new Float64Array(4);
    this.time = 0;

    this.pos = new V3(); this.quat = new THREE.Quaternion(); this.vel = new V3(); this.angVel = new V3(); this.com = new V3();
    this.right = new V3(); this.up = new V3(); this.back = new V3(); this.fwd = new V3();
    this.accel = new V3(); this._lastVel = new V3();
    this.readBody();
    this.updateGeometry();
  }

  setColliders(list) {
    const { RAPIER, world } = this;
    for (const c of this.chassisColliders) world.removeCollider(c, false);
    this.chassisColliders = list.map(([cx, cy, cz, hx, hy, hz, r]) => {
      r = Math.max(0, Math.min(r, hx - 0.002, hy - 0.002, hz - 0.002));
      const cd = RAPIER.ColliderDesc.roundCuboid(hx - r, hy - r, hz - r, r)
        .setTranslation(cx, cy, cz).setDensity(0).setFriction(0.55).setRestitution(0.05);
      return world.createCollider(cd, this.body);
    });
  }

  makeWheelColliders() {
    const { RAPIER, world, P } = this;
    this.wheelColR = this.R;
    this.wheelColW = P.tire.width * 0.42;
    for (const w of this.wheels) {
      if (w.sideCollider) world.removeCollider(w.sideCollider, false);
      const cd = RAPIER.ColliderDesc.cylinder(P.tire.width * 0.42, this.R - 0.12 * this.R / 0.42)
        .setDensity(0).setFriction(0.35).setRestitution(0.0)
        .setCollisionGroups(GROUP_WHEEL_SIDE);
      w.sideCollider = world.createCollider(cd, this.body);
    }
    if (this.quat) this.updateGeometry();
  }

  updateMass(snap = false) {
    const P = this.P;
    this.massTarget = [P.bodyMass + P.axles[0].mass + P.axles[1].mass, ...P.com, ...P.bodyInertia];
    if (snap || !this.massNow) this.massNow = [...this.massTarget];
    this.applyMass();
  }
  applyMass() {
    const m = this.massNow;
    this.totalMass = m[0];
    this.body.setAdditionalMassProperties(m[0], { x: m[1], y: m[2], z: m[3] }, { x: m[4], y: m[5], z: m[6] }, { x: 0, y: 0, z: 0, w: 1 }, true);
  }

  retune({ colliders = true, snap = false } = {}) {
    this.updateMass(snap);
    if (colliders) this.setColliders(this.P.colliders);
    this.drivetrain.updateInertia();
    if (snap) this.snapGeometry();
    else if (Math.abs(this.P.tire.width * 0.42 - this.wheelColW) > 1e-4) this.makeWheelColliders();
  }

  snapGeometry() {
    this.R = this.P.tire.radius;
    for (const ax of this.axles) { ax.droopY = ax.p.droopY; ax.k = ax.p.k; }
    if (this.massTarget && this.massNow.some((x, i) => x !== this.massTarget[i])) { this.massNow = [...this.massTarget]; this.applyMass(); }
    if (Math.abs(this.R - this.wheelColR) > 1e-4 || Math.abs(this.P.tire.width * 0.42 - this.wheelColW) > 1e-4) this.makeWheelColliders();
  }

  morphGeometry(h) {
    const up = 0.15 * h, down = 0.05 * h;
    const R = this.P.tire.radius;
    if (this.R !== R) {
      this.R += Math.max(-down, Math.min(up, R - this.R));
      if (Math.abs(this.R - this.wheelColR) > 0.004 || this.R === R) this.makeWheelColliders();
    }

    for (const ax of this.axles) if (ax.droopY !== ax.p.droopY) ax.droopY += Math.max(-up, Math.min(down, ax.p.droopY - ax.droopY));

    const b = Math.min(1, h * 5);
    for (const ax of this.axles) if (ax.k !== ax.p.k) ax.k = Math.abs(ax.p.k - ax.k) < 50 ? ax.p.k : ax.k + (ax.p.k - ax.k) * b;
    const mt = this.massTarget, mn = this.massNow;
    if (mt && mn.some((x, i) => x !== mt[i])) {
      let close = true;
      for (let i = 0; i < 7; i++) { mn[i] += (mt[i] - mn[i]) * b; if (Math.abs(mt[i] - mn[i]) > 1e-3 * (1 + Math.abs(mt[i]))) close = false; }
      if (close) for (let i = 0; i < 7; i++) mn[i] = mt[i];
      this.applyMass();
    }
  }

  get rideRaise() {
    const base = 0.42;
    return (this.P.tire.radius - base) + (0.275 - this.P.axles[0].droopY);
  }

  get pressure() { return (this.pressures[0] + this.pressures[1]) / 2; }
  setPressure(psi, axle) {
    const t = this.P.tire, lim = p => Math.max(t.minPressure, Math.min(t.maxPressure, p));
    if (axle === 0 || axle === 1) { this.pressures[axle] = lim(psi); return; }
    const d = psi - this.pressure;
    this.pressures = this.pressures.map(p => lim(p + d));
  }

  handbrakeLogic(raw, h) {
    const key = raw.handbrake > 0.5, prev = this._hbPrev > 0;
    this._hbPrev = key ? 1 : 0;
    if (this.hbMode === 'hold') { this.hbLatched = false; return raw.handbrake; }
    if (this.hbMode === 'toggle') { if (key && !prev) this.hbLatched = !this.hbLatched; return this.hbLatched ? 1 : 0; }
    if (key && !prev) { this._hbPress = 0; this._hbWas = this.hbLatched; }
    if (key) { this._hbPress += h; return 1; }
    if (prev) this.hbLatched = this._hbPress < 0.3 ? !this._hbWas : false;
    return this.hbLatched ? 1 : 0;
  }

  applyInput(raw, h) {
    const dt = this.drivetrain;
    const c = this.ctl;
    let throttle = raw.throttle, brake = raw.brake;
    if (dt.mode === 'auto') {
      const v = this.speed;

      if (this.arcadeAuto && dt.selector === 'R') { throttle = raw.brake; brake = raw.throttle; }
      if (this.arcadeAuto && Math.abs(v) < 0.8) {
        const wantR = raw.brake > 0.5 && raw.throttle < 0.05 && dt.selector !== 'R';
        const wantD = raw.throttle > 0.5 && raw.brake < 0.05 && dt.selector !== 'D';
        if (wantR || wantD) this.revTimer += h; else this.revTimer = 0;
        if (this.revTimer > 0.45) { dt.setSelector(wantR ? 'R' : 'D'); this.revTimer = 0; }
      } else this.revTimer = 0;
    }
    c.throttle = throttle;
    c.brake = brake;
    c.clutch = raw.clutch;
    c.handbrake = this.handbrakeLogic(raw, h);

    const S = this.P.steer, v = Math.abs(this.speed), maxA = S.maxAngle;
    this.updateGripEstimate(h);
    const key = raw.steerKey ?? raw.steer;
    const beam = this.steerAssist === 'beamng';
    let held = raw.analogSteer || key !== 0 || this.speed < -1;
    let target = 0;
    if (beam) {
      // BeamNG's filter does the rate limiting and the assists; the hands just hold what it outputs.
      const lockKey = S.ratio * maxA;
      if (!this.steerFilter || this.steerFilterKey !== lockKey) { this.steerFilter = new SteerFilter(this.P); this.steerFilterKey = lockKey; }
      const car = this._filterCar || (this._filterCar = { fronts: [this.wheels[0], this.wheels[1]] });
      car.vel = this.vel; car.right = this.right; car.fwd = this.fwd; car.accel = this.accel;
      car.wheelSpeed = Math.abs(this.speed); car.mu = this.gripG / 0.8;
      car.steerAngle = this.steerAngle; car.wheelbase = this.P.wheelbase;
      target = this.steerFilter.update(raw.analogSteer ? raw.steer : key, raw.analogSteer, h, car) * maxA;
      held = true;
    } else if (raw.analogSteer) target = raw.steer * maxA / (1 + Math.max(0, v - 8) / 30);
    else if (key !== 0) {
      let lim = maxA;
      const a = STEER_ASSIST[this.steerAssist];
      const vx = this.vel.dot(this.fwd);
      if (a && vx > 3) {
        const useful = this.P.wheelbase * this.gripG * G / (vx * vx) + a.slip * this.gripSlip;
        lim = maxA + (Math.min(maxA, useful) - maxA) * Math.min(1, (vx - 3) / 3);
      }
      target = raw.steer * lim;
    }
    const rate = beam ? BEAM_RACK_RATE : HAND_RATE / S.ratio;
    let w = this.steerVel;
    if (S.inertia != null) {
      this.grip += ((held ? 1 : 0) - this.grip) * Math.min(1, h / (held ? 0.05 : 0.25));
      this.hand += Math.max(-rate * h, Math.min(rate * h, (held ? target : this.steerAngle) - this.hand));
      const Mh = this.grip * Math.max(-S.handTorque, Math.min(S.handTorque, S.handStiffness * (this.hand - this.steerAngle) - S.handDamping * w));
      const Fz = Math.max(0, this.wheels[0].Fn) + Math.max(0, this.wheels[1].Fn);
      const M = Mh - this.alignM - Fz * S.kpiOffset * Math.sin(S.kpi * DEG) * Math.sin(this.steerAngle) - S.friction * Math.tanh(w / 0.05);
      w = (w + h * M / S.inertia) / (1 + h * S.damping / S.inertia);
    } else {
      const cmd = Math.max(-rate, Math.min(rate, (target - this.steerAngle) / 0.05));
      w += (cmd - w) * Math.min(1, h / 0.025);
    }
    let a = this.steerAngle + w * h;
    if (Math.abs(a) >= maxA) { a = Math.sign(a) * maxA; w = 0; }
    this.steerAngle = a; this.steerVel = w;
    const free = !held;
    this.handsOff = free;
    c.steer = this.steerAngle;
  }

  camber(w, sinC, sinK) {
    const p = w.axle.p;
    let lean = Math.asin(Math.max(-1, Math.min(1, w.up.dot(w.sc)))) + w.side * (p.camber ?? 0) * DEG;
    if (p.steered) lean += sinC * Math.sin(w.steer) + w.side * sinK * (1 - Math.cos(w.steer));
    w.camber = lean;
    const ta = Math.abs(w.vcy) / Math.max(Math.abs(w.vcx), 1);
    const b = Math.min(1, ta / (w.aPk || w.surf.aPeak));
    const into = -Math.sign(w.vcy) * lean;
    w.camberGrip = Math.max(0.6, 1 - b * 8 * (into - 0.035) ** 2 - (1 - b) * 4 * lean * lean);
  }

  updateGripEstimate(h) {
    let mu = 0, n = 0, ap = 0, nf = 0;
    for (const w of this.wheels) {
      if (!w.contact || !(w.Fn > 0) || !w.co.mu) continue;
      mu += w.co.mu; n++;
      if (w.i < 2) { ap += w.aPk || w.surf.aPeak; nf++; }
    }
    if (!n) return;
    const k = Math.min(1, h / 0.3);
    this.gripG += (0.80 * mu / n - this.gripG) * k;
    if (nf) this.gripSlip += (ap / nf - this.gripSlip) * k;
  }

  readBody() {
    const b = this.body;
    const t = b.translation(), r = b.rotation(), lv = b.linvel(), av = b.angvel(), com = b.worldCom();
    this.pos.set(t.x, t.y, t.z);
    this.quat.set(r.x, r.y, r.z, r.w);
    this.vel.set(lv.x, lv.y, lv.z);
    this.angVel.set(av.x, av.y, av.z);
    this.com.set(com.x, com.y, com.z);
    this.right.copy(X).applyQuaternion(this.quat);
    this.up.copy(Y).applyQuaternion(this.quat);
    this.back.copy(Z).applyQuaternion(this.quat);
    this.fwd.copy(this.back).negate();
  }

  pointVel(p, out) {
    return out.subVectors(p, this.com).crossVectors(this.angVel, out).add(this.vel);
  }

  steerAngles() {
    const d = this.steerAngle + this.steerComp;
    if (Math.abs(d) < 1e-4) return [d, d];
    const L = this.P.wheelbase, T = this.P.steer.kingpinTrack;
    const Rt = L / Math.tan(Math.abs(d));
    const inner = Math.atan(L / (Rt - T / 2)), outer = Math.atan(L / (Rt + T / 2));
    const s = Math.sign(d);

    const k = this.P.steer.ackermann ?? 1, ad = Math.abs(d);
    const i2 = ad + (inner - ad) * k, o2 = ad + (outer - ad) * k;

    return s > 0 ? [o2 * s, i2 * s] : [i2 * s, o2 * s];
  }

  hubLocal(w, out) {
    const ax = w.axle;
    if (this.indep) return out.set(w.side * this.P.track / 2, ax.droopY + w.c, ax.p.z);
    out.set(w.side * this.P.track / 2, 0, 0).applyAxisAngle(Z, ax.phi);
    out.y += ax.droopY + ax.c; out.z += ax.p.z;
    return out;
  }
  wheelRoll(w) { return this.indep ? 0 : w.axle.phi; }
  wheelComp(w) {
    const ax = w.axle;
    return this.indep ? w.c : ax.c + w.side * (ax.p.springTrack / 2) * Math.sin(ax.phi);
  }
  wheelCompRate(w) { return this.indep ? w.vz - w.vMountU : w.axle.vz - w.axle.vMountU; }

  axleRoll(i) {
    if (!this.indep) return this.axles[i].phi;
    return (this.wheels[i * 2 + 1].c - this.wheels[i * 2].c) / this.P.track;
  }

  updateGeometry() {
    const P = this.P;
    const [sl, sr] = this.steerAngles();
    if (this.indep) { this.updateGeometryIndep(sl, sr); return; }
    for (const ax of this.axles) {
      const p = ax.p;
      ax.A.set(0, ax.droopY + ax.c, p.z).applyQuaternion(this.quat).add(this.pos);
      _q.setFromAxisAngle(Z, ax.phi);
      ax.q.copy(this.quat).multiply(_q);
    }
    for (const w of this.wheels) {
      const ax = w.axle;

      w.steer = (ax.p.steered ? (w.side < 0 ? sl : sr) : 0) + Math.sign(ax.p.z) * -(ax.p.rollSteer || 0) * ax.phi;
      w.hub.set(w.side * P.track / 2, 0, 0).applyQuaternion(ax.q).add(ax.A);
      _q.setFromAxisAngle(Y, -w.steer);
      w.q.copy(ax.q).multiply(_q);
      w.spinAxis.copy(X).applyQuaternion(w.q);
      w.up.copy(Y).applyQuaternion(w.q);
      w.fwd.copy(Z).applyQuaternion(w.q).negate();
      if (w.sideCollider) {
        const ap = ax.p;
        _q2.setFromAxisAngle(Z, ax.phi);
        _v.set(w.side * P.track / 2, 0, 0).applyQuaternion(_q2);
        _v.y += ax.droopY + ax.c; _v.z += ap.z;
        w.sideCollider.setTranslationWrtParent({ x: _v.x, y: _v.y, z: _v.z });
        _q2.multiply(_q.setFromAxisAngle(Y, -w.steer)).multiply(_q.setFromAxisAngle(Z, -Math.PI / 2));
        w.sideCollider.setRotationWrtParent({ x: _q2.x, y: _q2.y, z: _q2.z, w: _q2.w });
      }
    }
  }

  updateGeometryIndep(sl, sr) {
    for (const w of this.wheels) {
      const ax = w.axle;
      w.steer = ax.p.steered ? (w.side < 0 ? sl : sr) : 0;
      this.hubLocal(w, _v);
      w.hub.copy(_v).applyQuaternion(this.quat).add(this.pos);
      _q.setFromAxisAngle(Y, -w.steer);
      w.q.copy(this.quat).multiply(_q);
      w.spinAxis.copy(X).applyQuaternion(w.q);
      w.up.copy(Y).applyQuaternion(w.q);
      w.fwd.copy(Z).applyQuaternion(w.q).negate();
      if (w.sideCollider) {
        w.sideCollider.setTranslationWrtParent({ x: _v.x, y: _v.y, z: _v.z });
        _q2.copy(_q).multiply(_q.setFromAxisAngle(Z, -Math.PI / 2));
        w.sideCollider.setRotationWrtParent({ x: _q2.x, y: _q2.y, z: _q2.z, w: _q2.w });
      }
    }
  }

  castContact(w) {
    const T = this.P.tire;
    const R = this.R, margin = 0.06;
    const ray = this.ray;
    let maxPen = -1e9, sumW = 0, nx = 0, ny = 0, nz = 0, lat = 0, snLat = 0;
    let deep = null, deepX = 0, deepY = 0, deepZ = 0;
    for (let row = -1; row <= 1; row++) {
      const o = row * 0.34 * T.width;
      const ox = w.hub.x + w.spinAxis.x * o, oy = w.hub.y + w.spinAxis.y * o, oz = w.hub.z + w.spinAxis.z * o;
      for (let k = 0; k < FAN.length; k++) {
        const th = FAN[k];
        const cs = Math.cos(th), sn = Math.sin(th);
        const dx = -w.up.x * cs + w.fwd.x * sn, dy = -w.up.y * cs + w.fwd.y * sn, dz = -w.up.z * cs + w.fwd.z * sn;
        ray.origin.x = ox; ray.origin.y = oy; ray.origin.z = oz;
        ray.dir.x = dx; ray.dir.y = dy; ray.dir.z = dz;
        const hit = this.world.castRayAndGetNormal(ray, R + margin, true, undefined, undefined, undefined, this.body);
        if (!hit) continue;
        const pen = R - hit.timeOfImpact;
        const wg = (pen + margin) * (pen + margin);
        if (pen > maxPen) {
          maxPen = pen; deep = hit.collider;
          deepX = ox + dx * hit.timeOfImpact; deepY = oy + dy * hit.timeOfImpact; deepZ = oz + dz * hit.timeOfImpact;
        }
        nx -= dx * wg; ny -= dy * wg; nz -= dz * wg;
        lat += o * wg;
        const hn = hit.normal;
        snLat += (hn.x * w.spinAxis.x + hn.y * w.spinAxis.y + hn.z * w.spinAxis.z) * wg;
        sumW += wg;
      }
    }
    if (sumW === 0) {
      w.contact = false; w.pen = -1; w.collider = null;
      return;
    }
    const n = w.n.set(nx, ny, nz).normalize();

    n.addScaledVector(w.spinAxis, snLat / sumW).normalize();
    w.pen = maxPen;
    w.latOff = lat / sumW;
    w.contact = maxPen > -margin;
    w.P.copy(w.hub).addScaledVector(w.spinAxis, w.latOff).addScaledVector(n, -(R - Math.max(maxPen, 0)));
    w.collider = deep;
    _v.set(deepX, deepY, deepZ);
    w.surf = this.surfaceAt(deep, _v) || SURFACES.dirt;
  }

  step(h, raw) {
    const P = this.P, dt = this.drivetrain, T = P.tire;
    this.time += h;
    this.morphGeometry(h);
    this.readBody();
    this.accel.subVectors(this.vel, this._lastVel).divideScalar(h);
    this._lastVel.copy(this.vel);
    this.speed = this.vel.dot(this.fwd);
    if (raw) this.applyInput(raw, h);
    const c = this.ctl;
    dt.control(h, c, this.speed);

    const up = this.up;
    const gU = -G * up.y;

    for (const ax of this.axles) {
      const p = ax.p;
      ax.mount.set(0, ax.droopY + ax.c, p.z).applyQuaternion(this.quat).add(this.pos);
      this.pointVel(ax.mount, _v);
      ax.vMountU = _v.dot(up);
      ax.rollRateBody = this.angVel.dot(this.back);
      ax.accS[0] = ax.accS[1] = 0; ax.accD[0] = ax.accD[1] = 0; ax.accArb = 0;
    }
    if (this.indep) for (const w of this.wheels) {
      this.hubLocal(w, _v2).applyQuaternion(this.quat).add(this.pos);
      w.vMountU = this.pointVel(_v2, _v).dot(up);
      w.accU = w.accLat = w.accJL = w.accLong = w.accJF = 0;
    }
    this.updateGeometry();

    const R0 = this.R;

    const sinC = Math.sin(Math.atan((P.steer.casterTrail || 0) / R0)), sinK = Math.sin((P.steer.kpi ?? 0) * DEG);
    for (const w of this.wheels) {
      this.castContact(w);
      tireCoefs(T, this.pressures[w.i < 2 ? 0 : 1], w.surf, w.co);
      w.Re = R0 - Math.max(0, w.pen) * 0.33;
      if (w.contact) {
        const n = w.n;
        w.fc.copy(w.fwd).addScaledVector(n, -n.dot(w.fwd)).normalize();
        w.sc.crossVectors(w.fc, n);
        this.pointVel(w.P, _v);
        w.vcx = _v.dot(w.fc);
        w.vcy = _v.dot(w.sc);
        this.camber(w, sinC, sinK);
      } else { w.vcx = w.vcy = 0; w.camberGrip = 1; }
      this.pointVel(w.hub, _v);
      w.vHubPerp.copy(_v).addScaledVector(up, -_v.dot(up));
      w.nLocal.copy(w.n).applyQuaternion(_q2.copy(this.quat).invert());
      w.accF.set(0, 0, 0);
      w.FnAvg = 0;
    }

    const S = this.substeps, hs = h / S;
    const hbT = c.handbrake * P.brakes.handbrake;

    const holdT = c.handbrake > 0.5 ? P.brakes.handbrakeHold * Math.max(0, Math.min(1, 2 - Math.abs(this.speed))) : 0;

    for (let i = 0; i < 4; i++) {
      const w = this.wheels[i];
      let f = this.absFactor[i];
      const vx = Math.abs(w.vcx);
      const slip = vx > 0.5 ? (dt.w[2 + i] * w.Re - w.vcx) / vx * Math.sign(w.vcx) : 0;
      if (this.abs && c.brake > 0.05 && vx > 1.8 && w.Fn > 0 && slip < -0.16) { f = Math.max(0.05, f - 9 * h); this.absActive = 0.15; }
      else f = Math.min(1, f + 3.5 * h);
      this.absFactor[i] = f;
      this.brakeT[i] = c.brake * f * (i < 2 ? P.brakes.front : P.brakes.rear);
    }
    this.absActive = Math.max(0, this.absActive - h);

    for (let i = 0; i < 4; i++) {
      const w = this.wheels[i];
      let T = this.tcT[i];
      if (this.tc && c.throttle > 0.05 && dt.running) {
        const surfV = dt.w[2 + i] * w.Re;
        const ref = w.contact && w.Fn > 0 ? w.vcx : this.speed;
        const excess = Math.sign(surfV) === Math.sign(ref) || Math.abs(ref) < 0.05 ? Math.abs(surfV) - Math.abs(ref) : Math.abs(surfV) + Math.abs(ref);
        const thr = 0.7 + 0.12 * Math.abs(ref);

        const tMax = 2400 * Math.max(0.15, Math.min(1, 1 - (Math.abs(this.speed) - 8) / 12));
        if (excess > thr) { T = Math.min(tMax, T + h * 9000 * (excess - thr) + h * 600); this.tcActive = 0.3; }
        else T = Math.max(0, T - h * (excess < thr * 0.5 ? 9000 : 3000));
      } else T = Math.max(0, T - h * 8000);
      this.tcT[i] = T;
      this.brakeT[i] = Math.max(this.brakeT[i], T);
    }
    this.tcActive = Math.max(0, this.tcActive - h);
    let tpf = 0, tpr = 0, engA = 0;

    for (let k = 0; k < S; k++) {
      for (const w of this.wheels) {
        this.hubPenDot(w);
        if (w.contact && w.pen > 0) {
          let Fn = w.co.kt * w.pen + T.damping * w.penDot;
          const rimLim = (this.R - T.rimRadius * this.R / T.radius) * 0.62;
          if (w.pen > rimLim) Fn += 2.5e6 * (w.pen - rimLim) + 3000 * Math.max(0, w.penDot);
          w.Fn = Math.max(0, Fn);
        } else w.Fn = 0;
      }

      for (let i = 0; i < 4; i++) {
        const w = this.wheels[i];
        const om = dt.w[2 + i];
        if (w.Fn > 0) {
          const vsx = om * w.Re - w.vcx;
          tireForces(w, w.Fn, vsx, -w.vcy, Math.hypot(w.vcx, w.vcy), w.surf, w.co);
        } else { w.Fx = 0; w.Fy = 0; }
        this.tireT[i] = -w.Fx * w.Re;
        this.rrT[i] = (w.Fn > 0 ? w.co.crr * w.Fn * w.Re : 0) + 2.5;
      }

      dt.substep(hs, this.tireT, this.rrT, this.brakeT, hbT, holdT);
      tpf += dt.propTorque[0]; tpr += dt.propTorque[1]; engA += dt.engineAlpha;

      for (let i = 0; i < 4; i++) {
        const w = this.wheels[i];
        if (w.Fn > 0) {
          const vsx = dt.w[2 + i] * w.Re - w.vcx;
          tireRelax(w, hs, vsx, -w.vcy, Math.abs(w.vcx), w.surf, w.co);
          w.slipVel = Math.hypot(vsx, w.vcy);
        } else {
          const d = Math.exp(-hs * 30);
          w.ux *= d; w.uy *= d; w.slipVel = 0;
        }
      }

      if (this.indep) for (const ax of this.axles) this.cornerSubstep(ax, hs, gU);
      else for (const ax of this.axles) this.axleSubstep(ax, hs, gU, ax.i === 0 ? dt.propTorque[0] : dt.propTorque[1]);

      for (const w of this.wheels) {
        this.hubPenDot(w);
        if (w.contact) w.pen += hs * w.penDot;
      }
    }

    const b = this.body;
    b.resetForces(true);
    b.resetTorques(true);
    const inv = 1 / S;
    if (this.indep) this.applyCornerForces(inv);
    else for (const w of this.wheels) {
      w.accF.multiplyScalar(inv);
      w.FnAvg *= inv;
      b.addForceAtPoint(w.accF, w.hub, true);
    }
    if (!this.indep) for (const ax of this.axles) {
      const p = ax.p;
      for (let s = 0; s < 2; s++) {
        const side = s === 0 ? -1 : 1;
        _v2.set(side * p.springTrack / 2, ax.droopY + ax.c, p.z).applyQuaternion(this.quat).add(this.pos);
        _v.copy(up).multiplyScalar(ax.accS[s] * inv);
        b.addForceAtPoint(_v, _v2, true);
        _v2.set(side * p.damperTrack / 2, ax.droopY + ax.c, p.z).applyQuaternion(this.quat).add(this.pos);
        _v.copy(up).multiplyScalar(ax.accD[s] * inv);
        b.addForceAtPoint(_v, _v2, true);
      }

      _v.copy(up).multiplyScalar(p.mass * G * up.y);
      b.addForceAtPoint(_v, ax.A, true);
    }

    let rollT = 0;
    for (const ax of this.axles) rollT -= ax.accArb * inv;

    rollT -= (P.engine.inertia * engA + (this.indep ? 0 : tpf + tpr)) * inv;
    _v.copy(this.back).multiplyScalar(rollT);
    b.addTorque(_v, true);

    const v2 = this.vel.lengthSq();
    if (v2 > 0.01) {
      _v.copy(this.vel).multiplyScalar(-0.5 * P.aero.rho * P.aero.cdA * Math.sqrt(v2));
      b.addForce(_v, true);
    }

    for (let i = 0; i < 4; i++) this.wheels[i].spin += dt.w[2 + i] * h;
    this.steerCompliance(h);
  }

  steerCompliance(h) {
    const S = this.P.steer;
    let M = 0;
    for (let i = 0; i < 2; i++) {
      const w = this.wheels[i];
      if (w.Fn <= 0) continue;
      const tp = S.pneuTrail * Math.max(0, 1 - w.slipNorm);
      M += w.Fy * (S.casterTrail + tp);
    }
    this.alignM = M;
    if (!S.stiffness || S.inertia != null) { this.steerComp = 0; return; }

    const target = this.handsOff ? 0 : Math.max(-0.06, Math.min(0.06, -M / S.stiffness));
    this.steerComp += (target - this.steerComp) * Math.min(1, h * 60);
  }

  hubPenDot(w) {
    const ax = w.axle;
    const vU = this.indep ? w.vz : ax.vz + w.side * (this.P.track / 2) * Math.cos(ax.phi) * ax.Om;
    const n = w.n, up = this.up;
    const v = w.vHubPerp;
    w.penDot = -((v.x + up.x * vU) * n.x + (v.y + up.y * vU) * n.y + (v.z + up.z * vU) * n.z);
  }

  springForce(p, x, xd, k = p.k) {
    let F = Math.max(0, k * (x + p.preload));
    const bs = x - (p.travel - 0.05);
    if (bs > 0) {
      const [b1, b2] = p.bumpStop ?? [220000, 8e6];
      const el = b1 * bs + b2 * bs * bs;
      F += el * (0.775 + 0.225 * Math.tanh(xd / 0.05)) + (xd > 0 ? 6000 * xd * Math.min(1, bs / 0.02) : 0);
    }
    if (x > p.travel) F += 3e6 * (x - p.travel) + (xd > 0 ? 25000 * xd : 0);
    if (x < 0) F += 1.6e6 * x + (xd < 0 ? 12000 * xd : 0);
    return F;
  }

  damperForce(p, xd) {
    const cd = xd > 0 ? p.bump : p.rebound;
    const v = Math.abs(xd), knee = p.damperKnee;
    const Fd = v < knee ? cd * v : cd * (knee + p.damperHigh * (v - knee));
    return xd > 0 ? Fd : -Fd;
  }

  axleSubstep(ax, hs, gU, propT) {
    const p = ax.p, up = this.up;
    const wl = this.wheels[ax.i * 2], wr = this.wheels[ax.i * 2 + 1];

    let FuL = 0, FuR = 0, tyreRoll = 0;
    const back = this.back;
    for (const w of [wl, wr]) {
      const Fn = w.Fn, Fx = w.Fx, Fy = w.Fy;
      _v.copy(w.n).multiplyScalar(Fn).addScaledVector(w.fc, Fx).addScaledVector(w.sc, Fy);
      _v2.subVectors(w.P, ax.A);
      _v3.crossVectors(_v2, _v);
      tyreRoll += _v3.dot(back);
      const fu = _v.dot(up);
      _v.addScaledVector(up, -fu);
      w.accF.add(_v);
      w.FnAvg += Fn;
      if (w === wl) FuL = fu; else FuR = fu;
    }
    const s2 = p.springTrack / 2, t2 = this.P.track / 2;
    const sn = Math.sin(ax.phi), cs = Math.cos(ax.phi);
    const cd = ax.vz - ax.vMountU;
    const pd = ax.Om - ax.rollRateBody;
    const SL = this.springForce(p, ax.c - s2 * sn, cd - s2 * cs * pd, ax.k);
    const SR = this.springForce(p, ax.c + s2 * sn, cd + s2 * cs * pd, ax.k);
    const d2 = p.damperTrack / 2;
    const DL = this.damperForce(p, cd - d2 * cs * pd);
    const DR = this.damperForce(p, cd + d2 * cs * pd);
    const arb = -p.arb * ax.phi;
    const heave = (FuL + FuR - SL - SR - DL - DR) / p.mass + gU;
    const roll = (tyreRoll - (SR - SL) * s2 * cs - (DR - DL) * d2 * cs + arb + propT) / p.rollInertia;
    ax.vz += hs * heave;
    ax.Om += hs * roll;
    ax.c += hs * (ax.vz - ax.vMountU);
    ax.phi += hs * (ax.Om - ax.rollRateBody);
    ax.S[0] = SL + DL; ax.S[1] = SR + DR;
    ax.accS[0] += SL; ax.accS[1] += SR;
    ax.accD[0] += DL; ax.accD[1] += DR;
    ax.accArb += arb;
  }

  cornerSubstep(ax, hs, gU) {
    const P = this.P, p = ax.p, up = this.up, right = this.right, back = this.back;
    const wl = this.wheels[ax.i * 2], wr = this.wheels[ax.i * 2 + 1];
    const t2 = P.track / 2, m = p.mass / 2;
    const tanL = (p.rcY ?? 0) / t2, tanF = (p.anti || 0) * P.com[1] / P.wheelbase, fs = -Math.sign(p.z);
    const bar = (p.arb || 0) * (wr.c - wl.c) / (P.track * P.track);
    for (const w of [wl, wr]) {
      _v.copy(w.n).multiplyScalar(w.Fn).addScaledVector(w.fc, w.Fx).addScaledVector(w.sc, w.Fy);
      const fu = _v.dot(up), fl = _v.dot(right), fb = _v.dot(back);
      const jl = -w.side * fl * tanL, jf = fs * fb * tanF;
      const cd = w.vz - w.vMountU;
      const S = this.springForce(p, w.c, cd, ax.k) + this.damperForce(p, cd) + w.side * bar;
      w.vz += hs * ((fu - S - jl - jf) / m + gU);
      w.c += hs * (w.vz - w.vMountU);
      w.S = S;
      w.accU += S; w.accLat += fl; w.accJL += jl; w.accLong += fb; w.accJF += jf;
      w.FnAvg += w.Fn;
    }
  }

  applyCornerForces(inv) {
    const b = this.body, up = this.up;
    for (const w of this.wheels) {
      const p = w.axle.p;
      w.FnAvg *= inv;

      this.hubLocal(w, _v2).applyQuaternion(this.quat).add(this.pos);
      _v.copy(up).multiplyScalar(w.accU * inv + p.mass / 2 * G * up.y);
      b.addForceAtPoint(_v, _v2, true);

      _v2.set(0, p.rcY ?? 0, p.z).applyQuaternion(this.quat).add(this.pos);
      _v.copy(this.right).multiplyScalar(w.accLat * inv).addScaledVector(up, w.accJL * inv);
      b.addForceAtPoint(_v, _v2, true);

      _v2.set(w.side * this.P.track / 2, 0, p.z).applyQuaternion(this.quat).add(this.pos);
      _v.copy(this.back).multiplyScalar(w.accLong * inv).addScaledVector(up, w.accJF * inv);
      b.addForceAtPoint(_v, _v2, true);
    }
    for (const ax of this.axles) {
      const wl = this.wheels[ax.i * 2], wr = this.wheels[ax.i * 2 + 1];
      ax.c = (wl.c + wr.c) / 2; ax.phi = 0;
      ax.S[0] = wl.S; ax.S[1] = wr.S;
    }
  }

  reset(position, yaw) {
    const b = this.body;
    const q = new THREE.Quaternion().setFromAxisAngle(Y, yaw);
    b.setTranslation(position, true);
    b.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true);
    b.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const ax of this.axles) { ax.vz = 0; ax.Om = 0; ax.phi = 0; }
    for (const w of this.wheels) { w.ux = w.uy = 0; w.vz = 0; }
    const d = this.drivetrain;
    for (let i = 2; i < 6; i++) d.w[i] = 0;

    d.lockup = 0;
    if (d.mode === 'auto') d.w[1] = 0;
    if (d.running) d.startGrace = Math.max(d.startGrace, 0.6);
    this.steerFilter?.reset();
    this.snapGeometry();
    this.readBody();
    this._lastVel.set(0, 0, 0);
    this.updateGeometry();
  }

  yaw() {
    return Math.atan2(-this.fwd.x, -this.fwd.z);
  }
}
