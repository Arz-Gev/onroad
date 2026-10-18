// The Assetto Corsa car (src/ac/car.js) inside onroad: its tyres cast their rays into the Rapier track, its body
// touches walls through a ring of contact points, and it shows the rest of the game (view, HUD, audio, effects,
// camera, tuning panel) the fields the old Vehicle had.
//
// Frames: AC's car frame is +x left, +y up, +z forward; onroad's is +x right, +y up, -z forward. The two differ by a
// half turn about y, so world coordinates are shared as they are (the track is in AC's own coordinates).
// `pos` / `quat` are the onroad car frame: the model's origin (ground under the wheels, mid-wheelbase).

import * as THREE from 'three';
import { AcCar, defaultEnv } from './car.js';
import { acSurface } from './surfaces.js';
import { IniReader } from './ini.js';
import { surfaceParameters, CONTACT_APPROX1_1, CONTACT_APPROX1_2, CONTACT_SOFT_CFM, CONTACT_SOFT_ERP, CONTACT_BOUNCE } from './ode.js';
import { SURFACES } from '../vehicle/tire.js';

const STEP = 0.003;
const HALF_TURN = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
const FX_OF = { ROAD: 'asphalt', KERB: 'kerb', GRASS: 'grass', SAND: 'sand', WALL: 'concrete', WAL: 'concrete' };
// AC's contact materials (PhysicsCore::onCollision): the car's collider mesh against the world, and a floor box on
// the road (kept only when it pushes nearly straight up as seen from the car)
const MESH_MATERIAL = surfaceParameters({ mode: CONTACT_APPROX1_1 | CONTACT_APPROX1_2 | 0x4000 | CONTACT_SOFT_CFM | CONTACT_BOUNCE, mu: 0.25, bounce: 0.01, softCfm: 1e-4 });
const FLOOR_MATERIAL = surfaceParameters({ mode: CONTACT_APPROX1_1 | CONTACT_APPROX1_2 | 0x4000 | CONTACT_SOFT_CFM | CONTACT_SOFT_ERP | CONTACT_BOUNCE, mu: 0.1, softCfm: 1 / 1050, softErp: 5 / 7 });

const _v = new THREE.Vector3(), _q = new THREE.Quaternion(), _m = new THREE.Matrix4();

// AC's DirectX matrix (rows = axes, row 3 = position) as a three.js Matrix4
function fromDx(m, out = new THREE.Matrix4()) {
  return out.set(m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14], 0, 0, 0, 1);
}

export class AcVehicle {
  // files: the car's data files; opts: { position {x,y,z} (ground), yaw, rayGround(org, len) -> {pos, normal, collider} | null,
  // surfaceKey(collider) -> 'ROAD'..., isGround(collider), projectPoint(p) -> {point, collider} | null, bodyBox: Box3 in the model frame,
  // modelWheels: [{ rest }] of the visual model, name }
  constructor(files, opts) {
    this.opts = opts;
    this.name = opts.name || 'AC car';
    this.stepSize = STEP;
    this.indep = true;
    this.env = defaultEnv();
    this.car = new AcCar(files, { ground: this.#ground(), env: this.env });
    this.sessionTime = 0;
    const c = this.car;

    const sus = new IniReader(files, 'suspensions.ini');
    const wheelbase = sus.getFloat('BASIC', 'WHEELBASE'), cg = sus.getFloat('BASIC', 'CG_LOCATION');
    const front = c.suspensions[0].basePosition, rear = c.suspensions[2].basePosition;
    // the model's origin in the AC body frame: mid-wheelbase, the visual wheel centres on the physics ones
    const restY = opts.modelWheels ? opts.modelWheels.reduce((s, w) => s + w.rest.y, 0) / 4 : c.tyres[0].data.radius;
    this.originAC = new THREE.Vector3(0, (front[1] + rear[1]) / 2 - restY, (0.5 - cg) * wheelbase);
    const eng = c.drivetrain.engine;
    this.P = {
      name: this.name, model: 's15', wheelbase, track: Math.abs(front[0] * 2), finalDrive: c.drivetrain.finalRatio,
      tire: { radius: c.tyres[0].data.radius, width: c.tyres[0].data.width },
      engine: { limiterRpm: eng.limiterRpm, redlineRpm: eng.limiterRpm, idleRpm: eng.minimum, maxPowerRpm: eng.maxPowerRpm },
      steer: { maxAngle: (c.steerLock / c.steerRatio) * Math.PI / 180 }, axles: [{ travel: 0.12 }, { travel: 0.12 }], colliders: [],
    };

    this.pos = new THREE.Vector3(); this.quat = new THREE.Quaternion();
    this.vel = new THREE.Vector3(); this.angVel = new THREE.Vector3(); this.accel = new THREE.Vector3();
    this.fwd = new THREE.Vector3(0, 0, -1); this.right = new THREE.Vector3(1, 0, 0); this.up = new THREE.Vector3(0, 1, 0);
    this.speed = 0;
    this.ctl = { throttle: 0, brake: 0, handbrake: 0, steer: 0, clutch: 0 };
    this.steerAngle = 0;
    this.tcActive = 0; this.absActive = 0;
    this.keySteer = 0;
    this.pendingShift = 0;
    this.lastVel = new THREE.Vector3();
    this.body = { translation: () => this.pos, rotation: () => this.quat };
    this.wheels = [0, 1, 2, 3].map(i => ({
      i, side: i % 2 ? 1 : -1, axle: { p: { travel: 0.12, z: i < 2 ? -wheelbase / 2 : wheelbase / 2 } },
      P: new THREE.Vector3(), contact: false, FnAvg: 0, Fx: 0, Fy: 0, slipNorm: 0, slipVel: 0, vcx: 0, pen: 0, steer: 0, spin: 0,
      c: 0, cRate: 0, S: 0, surf: SURFACES.asphalt, fc: new THREE.Vector3(0, 0, -1), hubLocalAC: new THREE.Vector3(),
    }));
    this.contactPoints = this.#contactPoints(opts.bodyBox);
    this.drivetrain = new AcDrivetrainView(this);
    this.reset(opts.position, opts.yaw || 0);
  }

  #ground() {
    const self = this;
    return {
      hasRayCaster: true,
      rayCast(org, dir, length) {
        const hit = self.opts.rayGround(org, length);
        if (!hit) return null;
        return { surface: acSurface(self.opts.surfaceKey(hit.collider)), pos: hit.pos, normal: hit.normal };
      },
    };
  }

  // sample points on the body's outline (AC body frame) that stand in for AC's collider mesh against walls, and floor
  // points for the colliders.ini floor box
  #contactPoints(box) {
    const o = this.originAC, pts = [];
    const b = box || new THREE.Box3(new THREE.Vector3(-0.85, 0.15, -2.2), new THREE.Vector3(0.85, 1.3, 2.2));
    const r = 0.12;
    const toAC = (x, y, z) => [-x + o.x, y + o.y, -z + o.z];
    const x0 = b.min.x + r, x1 = b.max.x - r, z0 = b.min.z + r, z1 = b.max.z - r, yl = b.min.y + 0.25, yh = b.min.y + 0.65 * (b.max.y - b.min.y);
    for (const y of [yl, yh]) {
      for (const x of [x0, 0, x1]) { pts.push({ p: toAC(x, y, z0), r, kind: 'mesh' }); pts.push({ p: toAC(x, y, z1), r, kind: 'mesh' }); }
      for (const z of [z0 * 0.5, 0, z1 * 0.5]) { pts.push({ p: toAC(x0, y, z), r, kind: 'mesh' }); pts.push({ p: toAC(x1, y, z), r, kind: 'mesh' }); }
    }
    pts.push({ p: toAC(0, b.max.y - r, 0), r, kind: 'mesh' });
    // the floor (colliders.ini: a thin box under the car)
    for (const x of [x0 * 0.7, x1 * 0.7]) for (const z of [z0 * 0.6, 0, z1 * 0.6]) pts.push({ p: toAC(x, b.min.y + 0.05, z), r: 0.05, kind: 'floor' });
    return pts;
  }

  // the body against the world: AC's contact joints from the nearest track surface to each sample point
  #collide(car) {
    const project = this.opts.projectPoint;
    if (!project || this.sessionTime < 0.75) return;
    const c = car.core, b = car.body, up = c.matrix(b).slice(4, 7), rolled = up[1] < 0.5;
    for (const s of this.contactPoints) {
      const w = c.localToWorld(b, s.p);
      const hit = project(w);
      if (!hit) continue;
      const dx = w[0] - hit.point[0], dy = w[1] - hit.point[1], dz = w[2] - hit.point[2];
      const d = Math.hypot(dx, dy, dz);
      if (d >= s.r || d < 1e-6) continue;
      const n = [dx / d, dy / d, dz / d];
      const ground = this.opts.isGround(hit.collider);
      if (s.kind === 'floor') {
        if (!ground) continue;
        if (!(c.worldToLocalNormal(b, n)[1] >= 0.9)) continue;
        car.addBodyContact(hit.point, n, s.r - d, FLOOR_MATERIAL);
      } else {
        if (ground && !rolled) continue;
        car.addBodyContact(hit.point, n, s.r - d, MESH_MATERIAL);
      }
    }
  }

  // the aids the car's electronics.ini has (a car without them keeps them off)
  get tc() { const t = this.car.aids.tc; return t.present && t.active; }
  set tc(on) { const t = this.car.aids.tc; if (t.present) t.active = !!on; }
  get abs() { const a = this.car.aids.abs; return a.present && a.active; }
  set abs(on) { const a = this.car.aids.abs; if (a.present) a.active = !!on; }

  get pressure() { return this.car.tyres[0].status.pressureStatic; }
  get pressures() { return [this.car.tyres[0].status.pressureStatic, this.car.tyres[2].status.pressureStatic]; }
  set pressures(p) { /* AC pressures come from tyres.ini (and the setup): ignored */ }
  setPressure(p) {
    const v = Math.max(10, Math.min(45, p));
    for (const t of this.car.tyres) { t.status.pressureStatic = v; }
  }
  yaw() { return Math.atan2(-this.fwd.x, -this.fwd.z); }

  reset(position, yaw) {
    const c = this.car;
    c.forceRotation([Math.sin(yaw), 0, Math.cos(yaw)]);
    c.forcePosition([position.x, position.y, position.z]);
    if (!this.started) { c.sessionStart(); this.started = true; }
    this.sessionTime = 0;
    for (const w of this.wheels) w.spin = 0;
    this.#read(0);
  }

  // one game step of `dt` (the caller steps at stepSize)
  step(dt, raw) {
    const c = this.car, ctl = c.controls, d = this.drivetrain;
    // steering: a gamepad's stick is the wheel; keys turn it at AC's keyboard rates, less lock at speed
    let steer = raw.steer || 0;
    if (!raw.analogSteer && raw.steerKey !== undefined) {
      const key = raw.steerKey, s = this.keySteer;
      const rate = key === 0 ? 1.8 : Math.sign(key) !== Math.sign(s) && s !== 0 ? 2.5 : 1.75;
      this.keySteer += Math.max(-rate * dt, Math.min(rate * dt, key - s));
      steer = this.keySteer / (1 + this.speed * 0.75 * 0.06);
    }
    ctl.steer = Math.max(-1, Math.min(1, steer));
    let gas = raw.throttle || 0, brake = raw.brake || 0;
    // the arcade automatic: hold the brake at a stop to reverse, in reverse the pedals swap
    ctl.requestedGear = -1;
    if (d.mode === 'auto') {
      const box = c.drivetrain, gear = box.currentGear, stopped = Math.abs(this.speed) < 1;
      if (stopped && brake > 0.3 && gas < 0.05 && gear >= 2 && !box.isChangingGear) this.revTimer = (this.revTimer || 0) + dt; else this.revTimer = 0;
      if (this.revTimer > 0.4) { ctl.requestedGear = 0; this.revTimer = 0; }
      else if (gear === 0 && stopped && gas > 0.3) ctl.requestedGear = 2;
      else if (gear === 1 && gas > 0.05 && !box.isChangingGear) ctl.requestedGear = 2;
      if (gear === 0) [gas, brake] = [brake, gas];
    }
    ctl.gas = gas;
    ctl.brake = brake;
    ctl.handBrake = raw.handbrake || 0;
    ctl.clutch = d.mode === 'manual' && !d.clutchAssist ? 1 - (raw.clutch || 0) : 1;
    ctl.gearUp = this.pendingShift > 0;
    ctl.gearDn = this.pendingShift < 0;
    this.pendingShift = 0;
    this.sessionTime += dt;
    c.step(STEP, this.sessionTime * 1000 + 60000, car => this.#collide(car));
    this.ctl.throttle = gas; this.ctl.brake = brake; this.ctl.handbrake = ctl.handBrake; this.ctl.steer = ctl.steer; this.ctl.clutch = 1 - ctl.clutch;
    this.#read(dt);
  }

  // copy the AC car's state into the onroad-facing fields
  #read(dt) {
    const c = this.car, core = c.core, b = c.body;
    const bm = fromDx(core.matrix(b), _m);
    _q.setFromRotationMatrix(bm);
    this.quat.copy(_q).multiply(HALF_TURN);
    _v.copy(this.originAC).applyMatrix4(bm);
    this.pos.copy(_v);
    const v = core.velocity(b);
    this.vel.set(v[0], v[1], v[2]);
    this.angVel.set(b.avel[0], b.avel[1], b.avel[2]);
    if (dt > 0) this.accel.subVectors(this.vel, this.lastVel).divideScalar(dt).add(_v.set(0, 9.806, 0));
    this.lastVel.copy(this.vel);
    this.fwd.set(bm.elements[8], bm.elements[9], bm.elements[10]);
    this.right.set(-bm.elements[0], -bm.elements[1], -bm.elements[2]);
    this.up.set(bm.elements[4], bm.elements[5], bm.elements[6]);
    this.speed = this.vel.dot(this.fwd);
    this.steerAngle = c.finalSteerAngleSignal * Math.PI / 180;
    const aids = c.aids;
    this.tcActive = aids.tc.inAction ? 1 : 0;
    this.absActive = c.tyres.some(t => t.absOverride < 1) ? 1 : 0;
    const inv = core.matrix(b);
    c.tyres.forEach((t, i) => {
      const w = this.wheels[i], s = t.status, sus = c.suspensions[i];
      w.contact = s.load > 0;
      w.FnAvg = s.load; w.Fx = s.fx; w.Fy = s.fy;
      w.slipNorm = s.ndSlip;
      w.slipVel = Math.hypot(t.slidingVelocityX, t.slidingVelocityY);
      w.vcx = t.roadVelocityX;
      w.pen = s.depth;
      w.P.set(t.contactPoint[0], t.contactPoint[1], t.contactPoint[2]);
      w.fc.set(t.roadHeading[0], t.roadHeading[1], t.roadHeading[2]);
      w.surf = SURFACES[FX_OF[t.surface?.name] || 'asphalt'];
      w.steer = i < 2 ? this.steerAngle : 0;
      w.spin += s.angularVelocity * dt;
      const prev = w.c;
      w.c = sus.status.travel;
      w.cRate = dt > 0 ? (w.c - prev) / dt : 0;
      w.S = sus.base.k * sus.status.travel;
      const hl = core.worldToLocal(b, sus.hub.pos);
      w.hubLocalAC.set(hl[0], hl[1], hl[2]);
      void inv;
    });
  }

  // what the HUD and audio read
  wheelComp(w) { return w.c; }
  wheelCompRate(w) { return w.cRate; }
  axleRoll(i) {
    const l = this.wheels[2 * i].hubLocalAC, r = this.wheels[2 * i + 1].hubLocalAC;
    return Math.atan2(l.y - r.y, Math.abs(l.x - r.x));
  }
  wheelRoll() { return 0; }
  hubLocal(w, out) { return out.set(-(w.hubLocalAC.x - this.originAC.x), w.hubLocalAC.y - this.originAC.y, -(w.hubLocalAC.z - this.originAC.z)); }

  // the hub's world matrix (three.js), static camber included: for the view
  hubMatrix(i, out = new THREE.Matrix4()) { return fromDx(this.car.suspensions[i].getHubWorldMatrix(), out); }
  bodyMatrix(out = new THREE.Matrix4()) { return fromDx(this.car.core.matrix(this.car.body), out); }

  retune() {}
}

// the drivetrain as the HUD, audio, menu and key actions see it
class AcDrivetrainView {
  constructor(v) {
    this.v = v;
    this.mode = 'manual';
    this.clutchAssist = true;
    this.running = true; this.cranking = false; this.stalled = false; this.fire = 1;
    this.frontLock = false; this.centerLock = false; this.fixedRwd = true; this.rwd = true; this.range = 'high';
    this.message = null; this.grind = 0; this.shift = null; this.propTorque = [0, 0];
    this.seq = 0;
  }
  get ac() { return this.v.car.drivetrain; }
  get rpm() { return this.ac.engineRpm; }
  get thr() { return this.v.car.drivetrain.engine.lastInput.gas; }
  get load() { return Math.max(0, Math.min(1, this.ac.engine.outTorque / Math.max(1, this.ac.engine.maxTorqueNm * (1 + this.ac.engine.turboBoost)))); }
  get Tcomb() { return this.ac.engine.outTorque; }
  get clutchPedal() { return 1 - this.v.car.controls.clutch; }
  get clutchSlip() { return this.ac.clutchOpen ? (this.ac.engineEl.v - this.ac.rootVelocity) : 0; }
  get lockup() { return 1; }
  get rearLock() { return this.ac.spool; }
  get rearDiff() { return this.ac.spool ? 'welded' : 'lsd'; }
  get manualGear() { const g = this.ac.currentGear; return g === 0 ? -1 : g - 1; }
  get selector() { const g = this.ac.currentGear; return g === 0 ? 'R' : g === 1 ? 'N' : 'D'; }
  get w() { const t = this.v.car.tyres; return [0, 0, ...t.map(x => x.status.angularVelocity)]; }
  gearLabel() { return this.ac.gearLabel; }
  currentRatio() { return this.ac.ratio; }
  wheelMean() { const t = this.v.car.tyres; return (t[2].status.angularVelocity + t[3].status.angularVelocity) / 2; }
  say(id, params) { this.message = { id, params, seq: ++this.seq }; }
  requestShift(dir) { this.v.pendingShift = dir; }
  toggleMode() {
    this.mode = this.mode === 'auto' ? 'manual' : 'auto';
    this.v.car.autoShifter.active = this.mode === 'auto';
    if (this.mode === 'auto' && this.ac.currentGear === 1) this.v.car.controls.requestedGear = 2;
  }
  toggleClutchAssist() { this.clutchAssist = !this.clutchAssist; this.v.car.autoclutch.useOnStart = this.clutchAssist; }
  toggleRange() {} toggleCenterLock() {} cycleAxleLockers() {} toggleRwd() {}
  startEngine() {} stopEngine() {}
}
