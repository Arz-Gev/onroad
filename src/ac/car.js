// An Assetto Corsa car: the body, the fuel tank, four suspensions with their tyres, heave springs, anti-roll
// bars, steering, brakes, engine and drivetrain, shift helpers, aero and aids, stepped in the order of
// acs.exe's Car::step / Car::stepComponents, then the ODE world. Built from a car's data files (car.ini,
// suspensions.ini, tyres.ini, engine.ini, drivetrain.ini, brakes.ini, aero.ini, electronics.ini, setup.ini and
// the .lut files they name). Ported from rustyAC (car/chassis.rs, heave_spring.rs, vanilla_car.rs).
//
// AC's physics frame is right-handed: +y up, +z forward, +x the car's left (wheel 0, front left, sits at +x).
// forceRotation(h) takes h = minus the forward direction (as acs.exe does): h = [0, 0, -1] faces the car to +z.
// A positive wheel speed rolls the car forward.
//
// Units: metres, seconds, newtons, kg. Time steps are AC's 3 ms (333 Hz).
//
//   const car = new AcCar(files, { ground, env });
//   car.forceRotation([0, 0, -1]); car.forcePosition([x, y, z]); car.sessionStart();
//   car.controls.gas = 1; car.step(0.003, timeMs);

import { World, surfaceParameters } from './ode.js';
import { IniReader } from './ini.js';
import { v3 } from './mat.js';
import { Core, Dwb, Strut, Damper, AntirollBar } from './suspension.js';
import { Tyre, defaultSurface } from './tyre.js';
import { Controller, Brakes, Engine, Drivetrain, Autoclutch, AutoBlip, AutoShifter, GearChanger, Aero, Aids } from './powertrain.js';

const nz = x => x < 0 || x > 0;
const clamp01 = x => (x > 1 ? 1 : x >= 0 ? x : 0);

export const defaultEnv = () => ({
  ambientTemperature: 26, roadTemperature: 30, dynamicGripLevel: 1, tyreConsumptionRate: 1, mechanicalDamageRate: 1,
  fuelConsumptionRate: 1, allowTyreBlankets: true, flatSpotFFGain: 0.05, gyroWheelGain: 0, ffFilter: 0,
  mzLowSpeedReductionSpeedKmh: 3, mzLowSpeedReductionMinValue: 0.01, sessionStartTimeMs: 0,
  get airDensity() { return 1.2922 - this.ambientTemperature * 0.0041; },
});

// a flat endless road at y = 0 (for tests): the ground of rustyAC's test rig (straight down, up to 3 m)
export const flatGround = (surface = defaultSurface()) => ({
  hasRayCaster: false,
  rayCast(org) {
    return org[1] >= 0 && org[1] <= 3 ? { surface, pos: [org[0], 0, org[2]], normal: [0, 1, 0] } : null;
  },
});

class HeaveSpring {
  constructor(ini, front) {
    this.k = 0; this.progressiveK = 0; this.rodLength = 0; this.travel = 0; this.present = false;
    const s = front ? 'HEAVE_FRONT' : 'HEAVE_REAR';
    if (!ini.ready || !ini.hasSection(s)) return;
    this.present = true;
    this.bumpStopUp = ini.getFloat(s, 'BUMPSTOP_UP'); this.bumpStopDn = -ini.getFloat(s, 'BUMPSTOP_DN');
    this.rodLength = ini.getFloat(s, 'ROD_LENGTH'); this.k = ini.getFloat(s, 'SPRING_RATE'); this.progressiveK = ini.getFloat(s, 'PROGRESSIVE_SPRING_RATE');
    this.damper = Damper.load(ini, s);
    this.bumpStopRate = ini.getFloat(s, 'BUMP_STOP_RATE') || 500000;
    this.packerRange = ini.getFloat(s, 'PACKER_RANGE');
  }
  step(core, body, left, right) {
    const l0 = core.worldToLocal(body, left.hub.pos), l1 = core.worldToLocal(body, right.hub.pos);
    const ref0 = left.basePosition, ref1 = right.basePosition;
    if (nz(left.base.k) || nz(right.base.k)) this.rodLength = (right.base.rodLength + left.base.rodLength) * 0.5;
    const avg = (l1[1] + l0[1]) * 0.5, travel = (avg - ref0[1]) + this.rodLength;
    this.travel = travel;
    let force = (travel * this.progressiveK + this.k) * travel;
    if (nz(this.packerRange) && travel > this.packerRange) force += (travel - this.packerRange) * this.bumpStopRate;
    const m = core.matrix(body), up = [m[4], m[5], m[6]];
    const push = f => {
      left.addForceAtPos(v3.scale(up, -f), left.hub.pos.slice(), false, false);
      right.addForceAtPos(v3.scale(up, -f), right.hub.pos.slice(), false, false);
      core.addLocalForceAtLocalPos(body, [0, f, 0], ref0);
      core.addLocalForceAtLocalPos(body, [0, f, 0], ref1);
    };
    push(force);
    const d = avg - ref0[1];
    if (nz(this.bumpStopUp) && d > this.bumpStopUp) push((d - this.bumpStopUp) * 500000);
    if (nz(this.bumpStopDn) && !(d >= this.bumpStopDn)) push((d - this.bumpStopDn) * 500000);
    const v1 = core.velocity(right.hub), v0 = core.velocity(left.hub), p1 = core.localPointVelocity(body, ref1), p0 = core.localPointVelocity(body, ref0);
    const speed = ((v0[1] + v1[1]) * 0.5 - (p0[1] + p1[1]) * 0.5) * up[1] + ((v1[0] + v0[0]) * 0.5 - (p0[0] + p1[0]) * 0.5) * up[0] + ((v0[2] + v1[2]) * 0.5 - (p0[2] + p1[2]) * 0.5) * up[2];
    const onHub = v3.scale(up, this.damper.force(speed));
    left.addForceAtPos(onHub, left.hub.pos.slice(), false, false);
    right.addForceAtPos(onHub, right.hub.pos.slice(), false, false);
    core.addForceAtLocalPos(body, v3.scale(onHub, -1), ref0);
    core.addForceAtLocalPos(body, v3.scale(onHub, -1), ref1);
  }
}

export class AcCar {
  constructor(files, { ground = flatGround(), env = defaultEnv() } = {}) {
    this.files = files;
    this.ground = ground;
    this.env = env;
    this.world = World.assettoCorsa();
    this.core = new Core(this.world);
    this.contactGroup = this.world.jointGroupCreate();
    const c = this.core;
    this.body = c.createBody();
    this.fuelTankBody = c.createBody();

    const ini = new IniReader(files, 'car.ini');
    if (!ini.ready) throw new Error('car.ini is missing');
    this.mass = ini.getFloat('BASIC', 'TOTALMASS');
    if (ini.hasSection('EXPLICIT_INERTIA')) throw new Error('car.ini: [EXPLICIT_INERTIA] is not ported');
    this.bodyInertia = ini.getFloat3('BASIC', 'INERTIA');
    if (!this.bodyInertia.some(nz)) throw new Error('car.ini: [BASIC] INERTIA is missing');
    c.setMassBox(this.body, this.mass, ...this.bodyInertia);
    this.fuelKg = ini.hasSection('FUEL_EXT') ? ini.getFloat('FUEL_EXT', 'KG_PER_LITER') : 0.74;
    this.ffMult = ini.getFloat('CONTROLS', 'FFMULT') * 0.001;
    this.steerLock = ini.getFloat('CONTROLS', 'STEER_LOCK');
    this.steerRatio = ini.getFloat('CONTROLS', 'STEER_RATIO');
    this.steerLinearRatio = ini.getFloat('CONTROLS', 'LINEAR_STEER_ROD_RATIO') || 0.003;
    this.steerAssist = ini.getFloat('CONTROLS', 'STEER_ASSIST') || 1;
    this.fuelConsumptionK = ini.getFloat('FUEL', 'CONSUMPTION');
    this.fuel = ini.getFloat('FUEL', 'FUEL');
    this.maxFuel = ini.getFloat('FUEL', 'MAX_FUEL') || 30;
    if (this.fuel === 0) { this.fuel = 30; this.requestedFuel = 30; } else this.requestedFuel = this.fuel;
    this.fuelTankPos = ini.getFloat3('FUELTANK', 'POSITION');
    c.setMassBox(this.fuelTankBody, 1, 0.5, 0.5, 0.5);
    c.setPosition(this.fuelTankBody, this.fuelTankPos);
    this.fuelTankJoint = c.fixedJoint(this.fuelTankBody, this.body);

    this.controls = { gearUp: false, gearDn: false, requestedGear: -1, handBrake: 0, gas: 0, brake: 0, steer: 0, clutch: 0 };
    this.finalSteerAngleSignal = 0;
    this.speed = 0;
    this.accG = [0, 0, 0];
    this.lastVelocity = [0, 0, 0];
    this.sleepingFrames = 0;
    this.framesToSleep = 50;
    this.physicsTime = 0;
    this.lastBodyMassUpdate = -1e8;
    this.ballast = 0;
    this.mzCurrent = 0; this.lastFF = 0; this.lastSteerPosition = 0;
    this.fuelPressure = 1;

    // the wheels: suspension, then its tyre
    const sus = new IniReader(files, 'suspensions.ini');
    if (sus.getString('REAR', 'TYPE') === 'AXLE') throw new Error('suspensions.ini: the rigid rear axle (AXLE) is not ported yet');
    this.suspensions = [];
    this.tyres = [];
    const tyreCar = this.tyreCar();
    for (let i = 0; i < 4; i++) {
      const kind = sus.getString(i < 2 ? 'FRONT' : 'REAR', 'TYPE');
      let s;
      if (kind === 'STRUT') s = new Strut(c, this.body, files, i);
      else if (kind === 'DWB') s = new Dwb(c, this.body, files, i);
      else throw new Error(`suspensions.ini: suspension type ${kind || '(none)'} for wheel ${i} is not ported${kind === 'COSMIC' ? ' (COSMIC needs Custom Shaders Patch)' : ''}`);
      this.suspensions.push(s);
      const t = new Tyre();
      t.init(s, files, i, tyreCar);
      this.tyres.push(t);
    }
    this.heaveSprings = this.suspensions.every(s => s.kind === 'DWB') ? [new HeaveSpring(sus, true), new HeaveSpring(sus, false)] : [];
    this.antirollBars = [new AntirollBar(sus.getFloat('ARB', 'FRONT')), new AntirollBar(sus.getFloat('ARB', 'REAR'))];
    this.arbControllers = ['ctrl_arb_front.ini', 'ctrl_arb_rear.ini'].map(n => Controller.load(files, n));
    this.ctrl4ws = Controller.load(files, 'ctrl_4ws.ini');
    this.updateBodyMass();

    this.brakes = new Brakes(files);
    this.drivetrain = new Drivetrain(this, files, new Engine(files));
    this.autoclutch = new Autoclutch(files);
    this.autoBlip = new AutoBlip(files);
    this.autoShifter = new AutoShifter(files);
    this.gearChanger = new GearChanger();
    this.aero = new Aero(files);
    this.aids = new Aids(files, this.drivetrain.traction);
  }

  // what a tyre asks of the car
  tyreCar() {
    const car = this;
    return {
      get isSleeping() { return car.sleepingFrames > car.framesToSleep; },
      get speed() { return car.speed; },
      get dynamicGripLevel() { return car.env.dynamicGripLevel; },
      get tyreConsumptionRate() { return car.env.tyreConsumptionRate; },
      get mechanicalDamageRate() { return car.env.mechanicalDamageRate; },
      get ambientTemperature() { return car.env.ambientTemperature; },
      get roadTemperature() { return car.env.roadTemperature; },
      get allowTyreBlankets() { return car.env.allowTyreBlankets; },
      bodyVelocity: () => car.core.velocity(car.body),
      bodyMass: () => car.core.mass(car.body),
      bodyAddForceAtLocalPos: (f, p) => car.core.addForceAtLocalPos(car.body, f, p),
    };
  }

  get isSleeping() { return this.sleepingFrames > this.framesToSleep; }
  get totalMass() { return this.core.mass(this.body) + this.suspensions.reduce((s, x) => s + x.mass, 0) + this.fuel * this.fuelKg; }
  updateBodyMass(force = false) {
    if (!force && !(this.physicsTime - this.lastBodyMassUpdate > 1000)) return;
    const hubs = this.suspensions.reduce((s, x) => s + x.mass, 0);
    this.core.setMassBox(this.body, (this.mass - hubs) + this.ballast, ...this.bodyInertia);
    const fuelMass = this.fuelKg * this.fuel;
    this.core.setMassBox(this.fuelTankBody, fuelMass > 0.1 ? fuelMass : 0.1, 0.5, 0.5, 0.5);
    this.lastBodyMassUpdate = this.physicsTime;
  }
  get baseCarHeight() {
    const f = -(this.suspensions[0].basePosition[1] - this.tyres[0].data.radius), r = -(this.suspensions[2].basePosition[1] - this.tyres[2].data.radius);
    return f > r ? f : r;
  }

  // the car faces `heading` (unit vector, world)
  forceRotation(h) {
    let x = h[1] * 0 - h[2], y = h[2] * 0 - h[0] * 0, z = h[0] - h[1] * 0;
    const l = Math.sqrt(y * y + x * x + z * z);
    if (nz(l)) { x /= l; y /= l; z /= l; }
    const m = [x, y, z, 0, 0, 1, 0, 0, -h[0], -h[1], -h[2], 0, 0, 0, 0, 1];
    this.core.setRotation(this.body, m);
    this.core.setRotation(this.fuelTankBody, m);
    for (const s of this.suspensions) s.attach();
    this.core.stop(this.body);
    this.core.stop(this.fuelTankBody);
  }
  // the car stands on its wheels with its ground point at `pos`
  forcePosition(p) {
    const pos = [p[0], p[1] + this.baseCarHeight + 0.01, p[2]];
    this.framesToSleep = 50;
    this.fuel = this.requestedFuel;
    this.core.stop(this.body);
    this.core.setPosition(this.body, pos);
    this.core.setPosition(this.fuelTankBody, this.core.localToWorld(this.body, this.fuelTankPos));
    for (const s of this.suspensions) { s.stop(); s.attach(); }
    this.drivetrain.reset();
    this.brakes.reset(this.env.ambientTemperature);
    const tc = this.tyreCar();
    this.tyres.forEach((t, i) => t.reset(this.suspensions[i], tc));
    this.drivetrain.setCurrentGear(1, true, this);
    this.core.stop(this.body);
    this.core.stop(this.fuelTankBody);
  }
  // the session's start: the default compound (the setup screen's rounding of setup values is not ported)
  sessionStart() {
    const t = new IniReader(this.files, 'tyres.ini');
    const compound = t.getInt('COMPOUND_DEFAULT', 'INDEX');
    const tc = this.tyreCar();
    this.tyres.forEach((ty, i) => ty.setCompound(compound, this.suspensions[i], tc));
  }

  onGearRequest(kind) {
    this.autoclutch.onGearRequest(kind);
    this.autoBlip.onGearRequest(kind, this.controls.clutch, this.physicsTime);
  }

  // one physics step: Car::step, then the rigid-body step. `collide(car)` may add contact joints first.
  step(dt, physicsTime, collide = null) {
    const c = this.core, ctl = this.controls;
    this.physicsTime = physicsTime;
    // stepPreCacheValues
    const v = c.velocity(this.body);
    const sq = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    this.speed = nz(sq) ? Math.sqrt(sq) : 0;
    // the first car softens its joints' error correction below 1 m/s
    const erp = sq >= 1 ? 0.3 : 0.9;
    for (const s of this.suspensions) s.setErpCfm(erp, sq >= 1 ? s.base.baseCfm : 1e-7);
    c.jointErpCfm(this.fuelTankJoint, erp, -1);
    // controls are already in this.controls (the caller's device); the air; the fuel burn
    this.aero.airDensity = this.env.airDensity;
    const d = this.drivetrain, e = d.engine;
    const boost = e.turboBoost >= 0 ? e.turboBoost : 0;
    this.fuel -= Math.abs(d.engineRpm) * dt * e.gasUsage * (boost + 1) * this.fuelConsumptionK * 0.001 * this.env.fuelConsumptionRate;
    if (this.fuel > 0) this.fuelPressure = 1; else { this.fuel = 0; this.fuelPressure = 0; }
    e.fuelPressure = this.fuelPressure;
    this.updateBodyMass();
    // the steering wheel as a road-wheel angle
    let signal = this.steerLock * ctl.steer / this.steerRatio;
    if (!Number.isFinite(signal)) signal = 0;
    this.finalSteerAngleSignal = signal;
    const allLoaded = this.tyres.every(t => !(0 >= t.status.load));
    this.autoclutch.step(ctl, this.speed, d.engineEl.v, d.currentGear, dt);
    // the sleeping rule: a car standing still, out of gear or with the clutch open, freezes
    let asleep = false;
    if (this.speed >= 0.5) this.sleepingFrames = 0;
    else {
      const w = c.angularVelocity(this.body);
      if ((w[0] * w[0] + w[1] * w[1]) + w[2] * w[2] >= 1) this.sleepingFrames = 0;
      else {
        const driving = !(0.01 >= ctl.gas) && !(0.01 >= ctl.clutch) && d.currentGear !== 1;
        this.sleepingFrames = !driving && allLoaded ? this.sleepingFrames + 1 : 0;
        asleep = this.sleepingFrames > this.framesToSleep;
      }
    }
    if (asleep) { c.stop(this.body); c.stop(this.fuelTankBody); }

    // body acceleration in g, body axes
    const vel = c.velocity(this.body), inv = 1 / dt;
    const acc = [0, 1, 2].map(k => ((vel[k] - this.lastVelocity[k]) * inv) * 0.101978384);
    this.lastVelocity = vel;
    this.accG = c.worldToLocalNormal(this.body, acc);

    // Car::stepComponents
    this.brakes.step(this, dt);
    this.aids.stepEdl(this);
    for (const s of this.suspensions) s.step();
    const tc = this.tyreCar();
    this.tyres.forEach((t, i) => t.step(dt, this.suspensions[i], this.ground, tc));
    this.lastFF = this.steerFF();
    if (nz(this.env.mzLowSpeedReductionSpeedKmh)) {
      const k = clamp01((this.speed * 3.6) / this.env.mzLowSpeedReductionSpeedKmh), min = this.env.mzLowSpeedReductionMinValue;
      this.lastFF *= (1 - min) * k + min;
    }
    this.heaveSprings.forEach((h, axle) => { if (nz(h.k)) h.step(c, this.body, this.suspensions[2 * axle], this.suspensions[2 * axle + 1]); });
    this.aero.step(this);
    const offset = -(this.finalSteerAngleSignal * this.steerLinearRatio);
    this.suspensions[0].setSteerLengthOffset(offset);
    this.suspensions[1].setSteerLengthOffset(offset);
    if (this.ctrl4ws.ready) { const o = this.ctrl4ws.eval(this); this.suspensions[2].setSteerLengthOffset(o); this.suspensions[3].setSteerLengthOffset(o); }
    this.autoBlip.step(ctl, false, this.speed, this.physicsTime);
    this.autoShifter.step(this, dt);
    this.gearChanger.step(this);
    d.step(this, dt);
    this.antirollBars.forEach((bar, axle) => {
      if (this.arbControllers[axle].ready) bar.k = this.arbControllers[axle].eval(this);
      bar.step(c, this.body, this.suspensions[2 * axle], this.suspensions[2 * axle + 1]);
    });
    this.aids.step(this, dt);
    this.aids.stepStability(this);

    // PhysicsCore::step: contacts, then dWorldStep
    this.world.jointGroupEmpty(this.contactGroup);
    if (collide) collide(this);
    this.world.step(dt);
  }

  // add a contact between the car body and the static world (point and normal in world space; the normal points
  // from the world into the car). surface: ODE surface parameters (see contacts.js).
  addBodyContact(pos, normal, depth, surface) {
    const j = this.world.jointCreateContact(this.contactGroup, { surface, geom: { pos, normal, depth } });
    this.world.jointAttach(j, this.body, null);
  }

  // Car::getSteerFF: the steering torque the driver's wheel would feel
  steerFF() {
    const t = this.tyres, steerTorque = this.suspensions[1].steerTorque + this.suspensions[0].steerTorque;
    const position = this.steerLock * this.controls.steer;
    this.mzCurrent = (this.mzCurrent - steerTorque) * this.env.ffFilter + steerTorque;
    let gyro = (position - this.lastSteerPosition) * 333.33334 / Math.abs(this.steerRatio);
    gyro *= (Math.abs(t[1].status.angularVelocity) + Math.abs(t[0].status.angularVelocity)) * t[0].data.angularInertia * this.env.gyroWheelGain;
    this.lastSteerPosition = position;
    let ff = -(this.mzCurrent + gyro) * 1.4;
    if (this.steerAssist !== 1) {
      const level = Math.pow(Math.abs(ff * this.ffMult), this.steerAssist);
      ff = Math.sign(ff) * level;
    } else ff *= this.ffMult;
    return -ff;
  }

  // what the rest of the game reads
  get position() { return this.body.pos; }
  get gear() { return this.drivetrain.currentGear - 1; }
  get rpm() { return this.drivetrain.engineRpm; }
}

export { surfaceParameters, defaultSurface };
