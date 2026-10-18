// Assetto Corsa's suspensions on the ODE bodies: the wheel carrier ("hub") is its own rigid body, held to the car
// body by rods (DBall joints). DWB: five rods (two wishbones and the steering rod). STRUT: two lower rods, the
// steering rod, and a strut body that slides on the hub and pivots on a ball joint at the top mount. Springs,
// dampers and bump stops push between hub and body. Also the anti-roll bar. Ported from rustyAC
// (car/suspension.rs, suspension_strut.rs, antiroll_bar.rs, body.rs).

import { IniReader } from './ini.js';
import { fromAxisAngle, mul, fromBody, toBodyR, v3 } from './mat.js';
import { PARAM_CFM, PARAM_ERP } from './ode.js';

const nz = x => x < 0 || x > 0;
const sign = x => (x > 0 ? 1 : x >= 0 ? 0 : -1);

// PhysicsCore helpers on top of the ODE world
export class Core {
  constructor(world) { this.world = world; }
  createBody() {
    const b = this.world.bodyCreate();
    this.world.bodySetFiniteRotationMode(b, true);
    this.world.bodySetFiniteRotationAxis(b, 0, 0, 0);
    return b;
  }
  // dMassSetBoxTotal
  setMassBox(b, mass, x, y, z) {
    const k = mass / 12;
    this.world.bodySetMass(b, { mass, c: [0, 0, 0], I: [(z * z + y * y) * k, 0, 0, 0, 0, (x * x + z * z) * k, 0, 0, 0, 0, (x * x + y * y) * k, 0] });
  }
  matrix(b) { return fromBody(b.R, b.pos); }
  setRotation(b, m) { this.world.bodySetRotation(b, toBodyR(m)); }
  setPosition(b, p) { this.world.bodySetPosition(b, p[0], p[1], p[2]); }
  localToWorld(b, p) { return this.world.getRelPointPos(b, p); }
  worldToLocal(b, p) { return this.world.getPosRelPoint(b, p); }
  localToWorldNormal(b, v) { return this.world.vectorToWorld(b, v); }
  worldToLocalNormal(b, v) { return this.world.vectorFromWorld(b, v); }
  velocity(b) { return this.world.getRelPointVel(b, [0, 0, 0]); }
  localVelocity(b) { return this.worldToLocalNormal(b, this.velocity(b)); }
  angularVelocity(b) { return b.avel.slice(); }
  localPointVelocity(b, p) { return this.world.getRelPointVel(b, p); }
  pointVelocity(b, p) { return this.world.getPointVel(b, p); }
  mass(b) { return b.mass.mass; }
  stop(b) { b.lvel = [0, 0, 0]; b.avel = [0, 0, 0]; b.facc = [0, 0, 0]; b.tacc = [0, 0, 0]; }
  addForceAtPos(b, f, p) { this.world.addForceAtPos(b, f, p); }
  addForceAtLocalPos(b, f, p) { this.world.addForceAtRelPos(b, f, p); }
  addLocalForce(b, f) { this.world.addRelForceAtRelPos(b, f, [0, 0, 0]); }
  addLocalForceAtLocalPos(b, f, p) { this.world.addRelForceAtRelPos(b, f, p); }
  addTorque(b, t) { this.world.addTorque(b, t); }
  addLocalTorque(b, t) { this.world.addRelTorque(b, t); }
  distanceJoint(b1, b2, p1, p2) {
    const w = this.world, j = w.jointCreateDBall();
    w.jointAttach(j, b1, b2);
    w.jointSetDBallAnchor1(j, p1[0], p1[1], p1[2]);
    w.jointSetDBallAnchor2(j, p2[0], p2[1], p2[2]);
    return { j, distance: w.jointGetDBallDistance(j) };
  }
  fixedJoint(b1, b2) { const w = this.world, j = w.jointCreateFixed(); w.jointAttach(j, b1, b2); w.jointSetFixed(j); return j; }
  sliderJoint(b1, b2, axis) {
    const w = this.world, j = w.jointCreateSlider();
    w.jointAttach(j, b1, b2);
    const l = v3.len(axis), a = nz(l) ? v3.scale(axis, 1 / l) : axis;
    w.jointSetSliderAxis(j, a[0], a[1], a[2]);
    return j;
  }
  ballJoint(b1, b2, anchor) { const w = this.world, j = w.jointCreateBall(); w.jointAttach(j, b1, b2); w.jointSetBallAnchor(j, anchor[0], anchor[1], anchor[2]); return j; }
  // the rod's anchors move to new body-local points, its length stays
  reseat(rod, p1, p2) {
    const w = this.world, b1 = w.jointGetBody(rod.j, 0), b2 = w.jointGetBody(rod.j, 1);
    const w1 = w.getRelPointPos(b1, p1), w2 = w.getRelPointPos(b2, p2);
    w.jointSetDBallAnchor1(rod.j, w1[0], w1[1], w1[2]);
    w.jointSetDBallAnchor2(rod.j, w2[0], w2[1], w2[2]);
    w.jointSetDBallDistance(rod.j, rod.distance);
  }
  jointErpCfm(j, erp, cfm) {
    if (erp > 0) this.world.jointSetParam(j, PARAM_ERP, erp);
    if (cfm > 0) this.world.jointSetParam(j, PARAM_CFM, cfm);
  }
}

export class Damper {
  constructor() { this.reboundSlow = 5000; this.reboundFast = 300; this.bumpSlow = 2000; this.bumpFast = 300; this.thresholdBump = 0.2; this.thresholdRebound = 0.2; }
  static load(ini, s) {
    const d = new Damper();
    d.bumpSlow = ini.getFloat(s, 'DAMP_BUMP'); d.reboundSlow = ini.getFloat(s, 'DAMP_REBOUND');
    d.bumpFast = ini.getFloat(s, 'DAMP_FAST_BUMP'); d.reboundFast = ini.getFloat(s, 'DAMP_FAST_REBOUND');
    d.thresholdBump = ini.getFloat(s, 'DAMP_FAST_BUMPTHRESHOLD'); d.thresholdRebound = ini.getFloat(s, 'DAMP_FAST_REBOUNDTHRESHOLD');
    if (!nz(d.thresholdBump)) d.thresholdBump = 0.2;
    if (!nz(d.thresholdRebound)) d.thresholdRebound = 0.2;
    if (!nz(d.bumpFast)) d.bumpFast = d.bumpSlow;
    if (!nz(d.reboundFast)) d.reboundFast = d.reboundSlow;
    return d;
  }
  force(v) {
    if (v > 0) { const t = this.thresholdBump; return v > t ? -((v - t) * this.bumpFast + t * this.bumpSlow) : -(v * this.bumpSlow); }
    const t = this.thresholdRebound;
    return Math.abs(v) > t ? t * this.reboundSlow - (t + v) * this.reboundFast : -(v * this.reboundSlow);
  }
}

// the shared part of suspensions.ini for wheel `index` (0 FL, 1 FR, 2 RL, 3 RR)
function readCommon(sus, ini, index, keys) {
  const version = ini.getInt('HEADER', 'VERSION');
  const wheelbase = ini.getFloat('BASIC', 'WHEELBASE'), cg = ini.getFloat('BASIC', 'CG_LOCATION');
  const fy = ini.getFloat('FRONT', 'BASEY'), ft = ini.getFloat('FRONT', 'TRACK');
  const ry = ini.getFloat('REAR', 'BASEY'), rt = ini.getFloat('REAR', 'TRACK');
  const fz = (1 - cg) * wheelbase, rz = -(cg * wheelbase);
  const refs = [[ft * 0.5, fy, fz], [-(ft * 0.5), fy, fz], [rt * 0.5, ry, rz], [-(rt * 0.5), ry, rz]];
  const s = index < 2 ? 'FRONT' : 'REAR';
  const w = { refPoint: refs[index] };
  for (const [name, key] of keys) w[name] = ini.getFloat3(s, key);
  if (version > 1) {
    const rim = -ini.getFloat(s, 'RIM_OFFSET');
    if (nz(rim)) for (const [name] of keys) w[name][0] += rim;
  }
  w.hubMass = ini.getFloat(s, 'HUB_MASS');
  w.hubInertiaBox = [0, 0, 0];
  const b = sus.base;
  b.bumpStopUp = ini.getFloat(s, 'BUMPSTOP_UP');
  b.bumpStopDn = -ini.getFloat(s, 'BUMPSTOP_DN');
  b.rodLength = ini.getFloat(s, 'ROD_LENGTH');
  b.toeOutLinear = ini.getFloat(s, 'TOE_OUT');
  b.k = ini.getFloat(s, 'SPRING_RATE');
  b.progressiveK = ini.getFloat(s, 'PROGRESSIVE_SPRING_RATE');
  sus.damper = Damper.load(ini, s);
  b.bumpStopRate = ini.getFloat(s, 'BUMP_STOP_RATE');
  if (!nz(b.bumpStopRate)) b.bumpStopRate = 500000;
  b.staticCamber = -(ini.getFloat(s, 'STATIC_CAMBER') * 0.017453);
  if (index % 2) b.staticCamber = -b.staticCamber;
  b.packerRange = ini.getFloat(s, 'PACKER_RANGE');
  // a right wheel (positive x) mirrors the file's points
  if (!(0 >= w.refPoint[0])) for (const [name] of keys) w[name][0] = -w[name][0];
  return w;
}

class SuspensionBase {
  constructor(core, carBody, index) {
    this.core = core;
    this.carBody = carBody;
    this.index = index;
    this.base = { k: 90000, progressiveK: 0, bumpStopRate: 0, bumpStopProgressive: 0, staticCamber: 0, bumpStopUp: 0, bumpStopDn: 0, rodLength: 0, toeOutLinear: 0, packerRange: 0, baseCfm: 1e-7 };
    this.status = { travel: 0, damperSpeed: 0 };
    this.steerTorque = 0;
    this.damage = { amount: 0, direction: 1 };
  }
  getHubWorldMatrix() { return mul(fromAxisAngle([0, 0, 1], this.base.staticCamber), this.core.matrix(this.hub)); }
  getPointVelocity(p) { return this.core.pointVelocity(this.hub, p); }
  getHubAngularVelocity() { return this.core.angularVelocity(this.hub); }
  getVelocity() { return this.core.velocity(this.hub); }
  get mass() { return this.core.mass(this.hub); }
  getBasePosition() { return this.basePosition; }
  stop() { this.core.stop(this.hub); }
  addTorque(t) {
    this.core.addTorque(this.hub, t);
    const [, axis] = this.getSteerBasis();
    this.steerTorque = ((axis[1] * t[1] + axis[0] * t[0]) + axis[2] * t[2]) + this.steerTorque;
  }
  setSteerLengthOffset(offset) {
    const side = sign(this.wheel.refPoint[0]) * this.base.toeOutLinear;
    const b = this.baseCarSteer;
    this.body.carSteer = [((this.damage.direction * this.damage.amount + offset) + side) + b[0], b[1], b[2]];
    this.core.reseat(this.steerRod, this.body.carSteer, this.wheel.tyreSteer);
  }
  setErpCfm(erp, cfm) { for (const j of this.joints()) this.core.jointErpCfm(j, erp, cfm); }
  // the hub's mass box and the rods, from the points in body coordinates
  makeHub(massShare) {
    let mass = this.wheel.hubMass;
    if (!(mass > 0)) mass = 20;
    this.hubMassTotal = mass;
    this.hub = this.core.createBody();
    this.core.setMassBox(this.hub, mass * massShare, 0.2, 0.6, 0.6);
  }
}

export class Dwb extends SuspensionBase {
  constructor(core, carBody, files, index) {
    super(core, carBody, index);
    const ini = new IniReader(files, 'suspensions.ini');
    this.wheel = readCommon(this, ini, index, [['carTopF', 'WBCAR_TOP_FRONT'], ['carTopR', 'WBCAR_TOP_REAR'], ['carBottomF', 'WBCAR_BOTTOM_FRONT'],
      ['carBottomR', 'WBCAR_BOTTOM_REAR'], ['tyreTop', 'WBTYRE_TOP'], ['tyreBottom', 'WBTYRE_BOTTOM'], ['tyreSteer', 'WBTYRE_STEER'], ['carSteer', 'WBCAR_STEER']]);
    const s = index < 2 ? 'FRONT' : 'REAR';
    if (ini.hasKey(s, 'BUMP_STOP_PROGRESSIVE')) this.base.bumpStopProgressive = ini.getFloat(s, 'BUMP_STOP_PROGRESSIVE');
    this.kind = 'DWB';
    this.makeHub(1);
    this.basePosition = this.wheel.refPoint;
    this.rods = [];
    this.attach();
    this.baseCarSteer = this.body.carSteer;
    this.setSteerLengthOffset(0);
    for (const j of this.joints()) core.jointErpCfm(j, 0.3, this.base.baseCfm);
  }
  get steerRod() { return this.rods[4]; }
  joints() { return this.rods.map(r => r.j); }
  bodies() { return [this.hub]; }
  get casterRad() { const w = this.wheel; return Math.atan((w.tyreTop[2] - w.tyreBottom[2]) / (w.tyreTop[1] - w.tyreBottom[1])); }
  attach() {
    const c = this.core, m = c.matrix(this.carBody);
    c.setRotation(this.hub, m);
    c.setPosition(this.hub, c.localToWorld(this.carBody, this.basePosition));
    if (this.rods.length) return;
    const w = this.wheel, toBody = p => c.localToWorld(this.carBody, v3.add(w.refPoint, p));
    const b = this.body = {};
    for (const k of ['carBottomF', 'carBottomR', 'carTopF', 'carTopR', 'tyreBottom', 'tyreTop', 'carSteer', 'tyreSteer']) b[k] = toBody(w[k]);
    for (const [car, tyre] of [[b.carTopR, b.tyreTop], [b.carTopF, b.tyreTop], [b.carBottomR, b.tyreBottom], [b.carBottomF, b.tyreBottom], [b.carSteer, b.tyreSteer]])
      this.rods.push(c.distanceJoint(this.carBody, this.hub, car, tyre));
  }
  addForceAtPos(f, p, driven, addToSteerTorque) {
    this.core.addForceAtPos(this.hub, f, p);
    if (addToSteerTorque) {
      const [centre, axis] = this.getSteerBasis(), r = v3.sub(p, centre);
      this.steerTorque = (((f[0] * r[2] - f[2] * r[0]) * axis[1] + (f[2] * r[1] - f[1] * r[2]) * axis[0]) + (f[1] * r[0] - f[0] * r[1]) * axis[2]) + this.steerTorque;
    }
  }
  getSteerBasis() {
    const a = this.core.localToWorld(this.hub, this.wheel.tyreTop), b = this.core.localToWorld(this.hub, this.wheel.tyreBottom);
    let axis = v3.sub(a, b);
    const l = v3.len(axis);
    if (nz(l)) axis = v3.scale(axis, 1 / l);
    return [[(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5], axis];
  }
  step() {
    const c = this.core, base = this.base, ref = this.wheel.refPoint;
    this.steerTorque = 0;
    const hubPos = this.hub.pos.slice();
    const hubLocal = c.worldToLocal(this.carBody, hubPos);
    const m = c.matrix(this.carBody), up = [m[4], m[5], m[6]];
    const dy = hubLocal[1] - ref[1];
    const travel = dy + base.rodLength;
    this.status.travel = travel;
    let force = (travel * base.progressiveK + base.k) * travel;
    if (nz(base.packerRange) && travel > base.packerRange && nz(base.k)) {
      const d = travel - base.packerRange;
      force += (d * base.bumpStopProgressive + base.bumpStopRate) * d;
    }
    if (force > 0) {
      this.addForceAtPos(v3.scale(up, -force), hubPos, false, false);
      c.addLocalForceAtLocalPos(this.carBody, [0, force, 0], ref);
    }
    const hv = c.velocity(this.hub), pv = c.localPointVelocity(this.carBody, ref);
    const speed = ((hv[1] - pv[1]) * up[1] + (hv[0] - pv[0]) * up[0]) + (hv[2] - pv[2]) * up[2];
    const f = this.damper.force(speed);
    this.status.damperSpeed = speed;
    const onHub = v3.scale(up, f);
    this.addForceAtPos(onHub, hubPos, false, false);
    c.addForceAtLocalPos(this.carBody, v3.scale(onHub, -1), ref);
    if (nz(base.bumpStopUp) && dy > base.bumpStopUp && nz(base.k)) {
      const d = dy - base.bumpStopUp, fb = (d * base.bumpStopProgressive + base.bumpStopRate) * d;
      this.addForceAtPos(v3.scale(up, -fb), hubPos, false, false);
      c.addLocalForceAtLocalPos(this.carBody, [0, fb, 0], hubLocal);
    }
    if (nz(base.bumpStopDn) && !(dy >= base.bumpStopDn) && nz(base.k)) {
      const d = dy - base.bumpStopDn, fb = (d * base.bumpStopProgressive + base.bumpStopRate) * d;
      this.addForceAtPos(v3.scale(up, -fb), hubPos, false, false);
      c.addLocalForceAtLocalPos(this.carBody, [0, fb, 0], hubLocal);
    }
  }
}

export class Strut extends SuspensionBase {
  constructor(core, carBody, files, index) {
    super(core, carBody, index);
    const ini = new IniReader(files, 'suspensions.ini');
    this.wheel = readCommon(this, ini, index, [['carStrut', 'STRUT_CAR'], ['tyreStrut', 'STRUT_TYRE'], ['carBottomF', 'WBCAR_BOTTOM_FRONT'],
      ['carBottomR', 'WBCAR_BOTTOM_REAR'], ['tyreBottom', 'WBTYRE_BOTTOM'], ['tyreSteer', 'WBTYRE_STEER'], ['carSteer', 'WBCAR_STEER']]);
    this.kind = 'STRUT';
    this.makeHub(0.8);
    this.strutBody = core.createBody();
    this.strutBodyLength = 0.2;
    core.setMassBox(this.strutBody, this.hubMassTotal * 0.2, 0.05, 0.5, 0.2);
    this.basePosition = this.wheel.refPoint;
    this.rods = [];
    this.body = { carStrut: [0, 0, 0] };
    this.attach();
    this.baseCarSteer = this.body.carSteer;
    this.setSteerLengthOffset(0);
  }
  get steerRod() { return this.rods[2]; }
  joints() { return this.rods.map(r => r.j).concat(this.slider ? [this.slider] : [], this.ball ? [this.ball] : []); }
  bodies() { return [this.hub, this.strutBody]; }
  get casterRad() { const w = this.wheel; return Math.atan((w.carStrut[2] - w.tyreStrut[2]) / (w.carStrut[1] - w.tyreStrut[1])); }
  setPositions() {
    const c = this.core, m = c.matrix(this.carBody);
    c.setRotation(this.hub, m);
    c.setPosition(this.hub, c.localToWorld(this.carBody, this.basePosition));
    const top = c.localToWorld(this.carBody, this.body.carStrut), foot = c.localToWorld(this.hub, this.wheel.tyreStrut);
    let d = v3.sub(foot, top);
    const l = v3.len(d);
    if (nz(l)) d = v3.scale(d, 1 / l);
    const n = [-m[8], -m[9], -m[10]];
    const side = [n[2] * d[1] - n[1] * d[2], n[0] * d[2] - n[2] * d[0], n[1] * d[0] - n[0] * d[1]];
    let front = [side[2] * d[1] - side[1] * d[2], side[0] * d[2] - side[2] * d[0], side[1] * d[0] - side[0] * d[1]];
    const fl = v3.len(front);
    if (nz(fl)) front = v3.scale(front, 1 / fl);
    c.setRotation(this.strutBody, [front[0], front[1], front[2], 0, side[0], side[1], side[2], 0, -d[0], -d[1], -d[2], 0, 0, 0, 0, 1]);
    const L = this.strutBodyLength;
    c.setPosition(this.strutBody, [d[0] * L * 0.5 + top[0], d[1] * L * 0.5 + top[1], d[2] * L * 0.5 + top[2]]);
  }
  strutLine() {
    const top = this.core.localToWorld(this.carBody, this.body.carStrut), foot = this.core.localToWorld(this.hub, this.wheel.tyreStrut);
    const d = v3.sub(foot, top);
    return [top, foot, d, (d[1] * d[1] + d[0] * d[0]) + d[2] * d[2]];
  }
  attach() {
    const c = this.core;
    this.setPositions();
    if (this.rods.length) return;
    const w = this.wheel, toBody = p => c.localToWorld(this.carBody, v3.add(w.refPoint, p)), b = this.body;
    for (const k of ['carBottomF', 'carBottomR', 'carStrut', 'tyreBottom', 'tyreStrut', 'carSteer', 'tyreSteer']) b[k] = toBody(w[k]);
    this.setPositions();
    for (const [car, tyre] of [[b.carBottomR, b.tyreBottom], [b.carBottomF, b.tyreBottom], [b.carSteer, b.tyreSteer]])
      this.rods.push(c.distanceJoint(this.carBody, this.hub, car, tyre));
    const [, , d, sq] = this.strutLine();
    this.strutBaseLength = nz(sq) ? Math.sqrt(sq) : 0;
    this.slider = c.sliderJoint(this.strutBody, this.hub, d);
    this.ball = c.ballJoint(this.carBody, this.strutBody, c.localToWorld(this.carBody, b.carStrut));
  }
  addForceAtPos(f, p, driven, addToSteerTorque) {
    this.core.addForceAtPos(this.hub, f, p);
    if (addToSteerTorque) {
      const [centre, axis] = this.getSteerBasis(), r = v3.sub(p, centre);
      this.steerTorque -= ((f[2] * r[0] - f[0] * r[2]) * axis[1] + (f[1] * r[2] - f[2] * r[1]) * axis[0]) + (f[0] * r[1] - f[1] * r[0]) * axis[2];
    }
  }
  getSteerBasis() {
    const a = this.core.localToWorld(this.carBody, this.body.carStrut), b = this.core.localToWorld(this.hub, this.wheel.tyreStrut);
    let axis = v3.sub(a, b);
    const l = v3.len(axis);
    if (nz(l)) axis = v3.scale(axis, 1 / l);
    return [[(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5], axis];
  }
  step() {
    const c = this.core, base = this.base;
    this.steerTorque = 0;
    const [top, foot, d0, sq] = this.strutLine();
    const length = nz(sq) ? Math.sqrt(sq) : 0;
    let dir = d0;
    if (nz(Math.sqrt(sq))) dir = v3.scale(d0, 1 / Math.sqrt(sq));
    const travel = (this.strutBaseLength + base.rodLength) - length;
    this.status.travel = travel;
    const spring = (travel * base.progressiveK + base.k) * travel;
    let force = spring > 0 ? spring : 0;
    if (nz(base.packerRange) && travel > base.packerRange) force += (travel - base.packerRange) * base.bumpStopRate;
    if (force > 0) {
      this.addForceAtPos(v3.scale(dir, force), foot, false, false);
      c.addForceAtPos(this.carBody, v3.scale(dir, -force), top);
    }
    const hubPos = this.hub.pos.slice(), hubLocal = c.worldToLocal(this.carBody, hubPos);
    const m = c.matrix(this.carBody), up = [m[4], m[5], m[6]];
    const dy = hubLocal[1] - this.wheel.refPoint[1];
    if (dy > base.bumpStopUp) {
      const fb = (dy - base.bumpStopUp) * 500000;
      this.addForceAtPos(v3.scale(up, -fb), this.hub.pos.slice(), false, false);
      c.addLocalForceAtLocalPos(this.carBody, [0, fb, 0], hubLocal);
    }
    if (!(dy >= base.bumpStopDn)) {
      const fb = (dy - base.bumpStopDn) * 500000;
      this.addForceAtPos(v3.scale(up, -fb), this.hub.pos.slice(), false, false);
      c.addLocalForceAtLocalPos(this.carBody, [0, fb, 0], hubLocal);
    }
    const fv = c.localPointVelocity(this.hub, this.wheel.tyreStrut), tv = c.localPointVelocity(this.carBody, this.body.carStrut);
    const speed = ((fv[1] - tv[1]) * dir[1] + (fv[0] - tv[0]) * dir[0]) + (fv[2] - tv[2]) * dir[2];
    this.status.damperSpeed = speed;
    const onHub = v3.scale(dir, this.damper.force(speed));
    this.addForceAtPos(onHub, foot, false, false);
    c.addForceAtPos(this.carBody, v3.scale(onHub, -1), top);
  }
}

export class AntirollBar {
  constructor(k = 0) { this.k = k; }
  step(core, body, left, right) {
    if (0 >= this.k) return;
    const m0 = left.getHubWorldMatrix(), m1 = right.getHubWorldMatrix();
    const p0 = [m0[12], m0[13], m0[14]], p1 = [m1[12], m1[13], m1[14]];
    const l0 = core.worldToLocal(body, p0), l1 = core.worldToLocal(body, p1);
    const delta = l1[1] - l0[1];
    const bm = core.matrix(body);
    let n = [bm[4], bm[5], bm[6]];
    const l = v3.len(n);
    if (nz(l)) n = v3.scale(n, 1 / l);
    const k = this.k;
    left.addForceAtPos(v3.scale(n, delta * k), p0, false, false);
    core.addLocalForceAtLocalPos(body, [0, -(delta * k), 0], l0);
    right.addForceAtPos(v3.scale(n, -delta * k), p1, false, false);
    core.addLocalForceAtLocalPos(body, [0, delta * k, 0], l1);
  }
}
