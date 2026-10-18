// Assetto Corsa's powertrain and the systems around it: dynamic controllers (ctrl_*.ini), brakes, the engine with
// turbos, the 2WD drivetrain with its clutch, gearbox and LSD, the shift helpers (auto clutch, auto blip, auto
// shifter, gear changer), aero (wings and fins, or the old single-body DATA) and the aids (traction control,
// ABS, EDL, stability control, pit limiter). Ported from rustyAC (car/dynamic_controller.rs, brakes.rs,
// engine.rs, drivetrain.rs, shift_assists.rs, aero.rs, aids.rs).
//
// Each system's step(car, dt) reads and writes the car the way acs.exe's does: car.controls, car.tyres[i].inputs,
// car.tyres[i].status, car.speed, car.accG, car.core / car.body, car.env.

import { IniReader, Curve, fileText } from './ini.js';
import { v3 } from './mat.js';

const nz = x => x < 0 || x > 0;
const sat = x => (x > 1 ? 1 : !(x >= 0) ? 0 : x);
const K2PI = 0.15915494309189535; // 1 / (2 pi)
const INV_DT = 333.33333;
const exists = (files, name) => fileText(files, name) != null;

// --- dynamic controllers -------------------------------------------------------------------------------------

const INPUTS = ['BRAKE', 'GAS', 'STEER', 'LATG', 'LONG', 'SPEED_KMH', 'GEAR', 'SLIPRATIO_MAX', 'SLIPRATIO_AVG', 'SLIPANGLE_FRONT_AVG',
  'SLIPANGLE_FRONT_MAX', 'SLIPANGLE_REAR_AVG', 'SLIPANGLE_REAR_MAX', 'OVERSTEER_FACTOR', 'REAR_SPEED_RATIO', 'STEER_DEG', 'CONST', 'RPMS',
  'WHEEL_STEER_DEG', 'LOAD_SPREAD_LF', 'LOAD_SPREAD_RF', 'AVG_TRAVEL_REAR', 'SUS_TRAVEL_LR', 'SUS_TRAVEL_RR'];

export class Controller {
  constructor() { this.stages = []; this.ready = false; }
  static load(files, name) {
    const c = new Controller();
    if (!exists(files, name)) return c;
    const ini = new IniReader(files, name);
    if (!ini.ready) return c;
    for (let i = 0; ini.hasSection(`CONTROLLER_${i}`); i++) {
      const s = `CONTROLLER_${i}`, input = ini.getString(s, 'INPUT');
      const st = { input: INPUTS.includes(input) ? input : 'UNDEFINED', combinator: ini.getString(s, 'COMBINATOR'), lut: new Curve(), constValue: 0, value: 0 };
      if (st.input === 'CONST') st.constValue = ini.getFloat(s, 'CONST_VALUE');
      else st.lut = ini.getCurve(s, 'LUT');
      st.filter = (1 / 0.003) * 0.004 * (1 - ini.getFloat(s, 'FILTER')) * (1 / 0.003);
      st.up = ini.getFloat(s, 'UP_LIMIT');
      st.down = ini.getFloat(s, 'DOWN_LIMIT');
      c.stages.push(st);
    }
    c.ready = true;
    return c;
  }
  eval(car) {
    let out = 0;
    for (const st of this.stages) {
      let value = st.input === 'CONST' ? st.constValue : st.lut.getValue(input(st.input, car));
      const prev = st.value;
      if (Math.abs(value - prev) >= 0.001) value = (value - prev) * sat(st.filter * 0.003) + prev;
      st.value = value;
      if (st.combinator === 'ADD') out += value;
      else if (st.combinator === 'MULT') out *= value;
      else out = 0;
      if (nz(st.down) || nz(st.up)) { if (out > st.up) out = st.up; else if (!(out >= st.down)) out = st.down; }
    }
    return out;
  }
}

function input(name, car) {
  const t = car.tyres, sr = i => t[i].status.slipRatio, sa = i => t[i].status.slipAngleRad, ld = i => t[i].status.load;
  const tr = i => car.suspensions[i].status.travel, max = (a, b) => (a > b ? a : b);
  const dt = car.drivetrain, traction = dt ? dt.traction : 'RWD';
  switch (name) {
    case 'BRAKE': return car.controls.brake;
    case 'GAS': return car.controls.gas;
    case 'LATG': return car.accG[0];
    case 'LONG': return car.accG[2];
    case 'STEER': return car.controls.steer;
    case 'SPEED_KMH': return car.speed * 3.6;
    case 'GEAR': return (dt ? dt.currentGear : 1) - 1;
    case 'SLIPRATIO_MAX': return traction === 'FWD' ? max(sr(0), sr(1)) : max(sr(2), sr(3));
    case 'SLIPRATIO_AVG': return traction === 'FWD' ? (sr(1) + sr(0)) * 0.5 : (sr(3) + sr(2)) * 0.5;
    case 'SLIPANGLE_FRONT_AVG': return (sa(1) + sa(0)) * 57.29578 * 0.5;
    case 'SLIPANGLE_REAR_AVG': return (sa(3) + sa(2)) * 57.29578 * 0.5;
    case 'SLIPANGLE_FRONT_MAX': return max(Math.abs(sa(0)), Math.abs(sa(1))) * 57.29578;
    case 'SLIPANGLE_REAR_MAX': return max(Math.abs(sa(2)), Math.abs(sa(3))) * 57.29578;
    case 'OVERSTEER_FACTOR': return ((Math.abs(sa(3)) + Math.abs(sa(2))) * 0.5 - (Math.abs(sa(0)) + Math.abs(sa(1))) * 0.5) * 57.29578;
    case 'REAR_SPEED_RATIO': {
      const w = i => t[i].status.angularVelocity, f = (w(1) + w(0)) * 0.5;
      return nz(f) ? (w(3) + w(2)) * 0.5 / f : 0;
    }
    case 'STEER_DEG': return car.steerLock * car.controls.steer;
    case 'RPMS': return dt ? dt.engineRpm : 0;
    case 'WHEEL_STEER_DEG': return car.finalSteerAngleSignal;
    case 'LOAD_SPREAD_LF': return ld(0) / (ld(0) + ld(1));
    case 'LOAD_SPREAD_RF': return ld(1) / (ld(1) + ld(0));
    case 'AVG_TRAVEL_REAR': return (tr(3) + tr(2)) * 0.5 * 1000;
    case 'SUS_TRAVEL_LR': return tr(2) * 1000;
    case 'SUS_TRAVEL_RR': return tr(3) * 1000;
  }
  return 0;
}

// --- brakes --------------------------------------------------------------------------------------------------

export class Brakes {
  constructor(files) {
    this.frontBias = 0.7; this.powerMultiplier = 1; this.electronicOverride = 0; this.rearCorrectionTorque = 0;
    this.handBrakeTorque = 0; this.ebbInstant = 0.5; this.limitDown = 0; this.limitUp = 1; this.biasOverride = -1;
    this.hasCockpitBias = true; this.biasStep = 0.005; this.ebbMode = 'off'; this.ebbFrontMultiplier = 1.1;
    this.discs = [0, 1, 2, 3].map(() => ({ t: 0, coolTransfer: 0.005, torqueK: 0.1, coolSpeedFactor: 0.001, perf: new Curve() }));
    this.hasTemps = false;
    const ini = new IniReader(files, 'brakes.ini');
    this.power = ini.getFloat('DATA', 'MAX_TORQUE');
    this.frontBias = ini.getFloat('DATA', 'FRONT_SHARE');
    this.handBrakeTorque = ini.getFloat('DATA', 'HANDBRAKE_TORQUE');
    this.hasCockpitBias = ini.getInt('DATA', 'COCKPIT_ADJUSTABLE') !== 0;
    this.biasStep = ini.getFloat('DATA', 'ADJUST_STEP') * 0.01;
    if (ini.hasSection('EBB')) { this.ebbMode = 'internal'; const m = ini.getFloat('EBB', 'FRONT_SHARE_MULTIPLIER'); this.ebbFrontMultiplier = m >= 1.1 ? m : 1.1; }
    this.steerBrake = exists(files, 'steer_brake_controller.ini') ? Controller.load(files, 'steer_brake_controller.ini') : null;
    ['TEMPS_FRONT', 'TEMPS_REAR'].forEach((s, axle) => {
      if (ini.hasSection(s)) {
        for (const d of this.discs.slice(2 * axle, 2 * axle + 2)) {
          d.torqueK = ini.getFloat(s, 'TORQUE_K'); d.perf = ini.getCurve(s, 'PERF_CURVE');
          d.coolTransfer = ini.getFloat(s, 'COOL_TRANSFER'); d.coolSpeedFactor = ini.getFloat(s, 'COOL_SPEED_FACTOR');
        }
        this.hasTemps = true;
      } else this.hasTemps = false;
    });
    const setup = new IniReader(files, 'setup.ini');
    if (setup.ready && setup.hasSection('FRONT_BIAS')) { this.limitDown = setup.getFloat('FRONT_BIAS', 'MIN') * 0.01; this.limitUp = setup.getFloat('FRONT_BIAS', 'MAX') * 0.01; }
    if (exists(files, 'ctrl_ebb.ini')) { this.ebbController = Controller.load(files, 'ctrl_ebb.ini'); this.ebbMode = 'controller'; }
  }
  get bias() { return nz(this.biasOverride + 1) ? this.biasOverride : this.frontBias; }
  reset(ambient) { this.biasOverride = -1; for (const d of this.discs) d.t = ambient; }
  step(car, dt) {
    const t = car.tyres;
    let bias = nz(this.biasOverride + 1) ? this.biasOverride : this.frontBias;
    if (this.ebbMode === 'internal') {
      const front = t[1].status.load + t[0].status.load, total = (t[3].status.load + t[2].status.load) + front;
      this.ebbInstant = nz(total) && car.speed * 3.6 > 10 ? sat(front / total * this.ebbFrontMultiplier) : this.frontBias;
      bias = this.ebbInstant;
    } else if (this.ebbMode === 'controller') bias = this.ebbController.eval(car);
    if (bias > this.limitUp) bias = this.limitUp; else if (!(bias >= this.limitDown)) bias = this.limitDown;
    const pedal = this.electronicOverride > car.controls.brake ? this.electronicOverride : car.controls.brake;
    const torque = this.power * this.powerMultiplier * pedal, front = torque * bias;
    t[0].inputs.brakeTorque = front; t[1].inputs.brakeTorque = front;
    let rear = (1 - bias) * torque - this.rearCorrectionTorque;
    if (!(rear >= 0)) rear = 0;
    t[2].inputs.brakeTorque = rear; t[3].inputs.brakeTorque = rear;
    t[2].inputs.handBrakeTorque = car.controls.handBrake * this.handBrakeTorque;
    t[3].inputs.handBrakeTorque = car.controls.handBrake * this.handBrakeTorque;
    if (this.steerBrake && this.steerBrake.ready) {
      const extra = this.steerBrake.eval(car);
      if (extra >= 0) t[3].inputs.brakeTorque += extra; else t[2].inputs.brakeTorque -= extra;
    }
    if (this.hasTemps && 1 >= t[0].aiMult) {
      this.discs.forEach((d, i) => {
        t[i].inputs.brakeTorque = d.perf.getValue(d.t) * t[i].inputs.brakeTorque;
        const cool = ((car.speed * 3.6) * d.coolSpeedFactor + 1) * d.coolTransfer;
        const cooled = ((car.env.ambientTemperature - d.t) * cool) * dt + d.t;
        d.t = ((Math.abs(t[i].status.angularVelocity) * t[i].inputs.brakeTorque * d.torqueK) * 0.001) * dt + cooled;
      });
    }
    this.electronicOverride = 0;
  }
}

// --- engine --------------------------------------------------------------------------------------------------

export class Engine {
  constructor(files) {
    this.coastTorqueMultiplier = 1; this.limiterMultiplier = 1; this.fuelPressure = 1; this.bov = 0; this.restrictor = 0;
    this.lastInput = { gas: 0, rpm: 0 }; this.inertia = 1; this.limiterOn = 0; this.electronicOverride = 1;
    this.maxPowerWDynamic = -1; this.gasUsage = 0; this.lifeLeft = 0; this.outTorque = 0; this.turboBoost = 0;
    this.data = { power: new Curve(), coast2: 1e-6, coast1: 0, coast0: 0, minimum: 1000, limiter: 18000, limiterCycles: 50, overlapFreq: 1, overlapGain: 0, overlapIdealRpm: 6000 };
    this.turbos = []; this.turboControllers = []; this.throttle = new Curve(); this.throttleMax = new Curve(); this.throttleMaxRef = 6000;
    this.turboDamageThreshold = 0; this.turboDamageK = 0; this.rpmDamageThreshold = 0; this.rpmDamageK = 0; this.bovThreshold = 0.2;
    this.gasCoastOffset = 0; this.gasCoastCurve = new Curve(); this.coastDefault = 0; this.coastEntryRpm = 0;
    const ini = new IniReader(files, 'engine.ini'), d = this.data;
    const curve = ini.getString('HEADER', 'POWER_CURVE');
    d.power = Curve.fromLut(fileText(files, curve));
    if (!d.power.count) throw new Error(`engine.ini: the power curve ${curve} is missing or empty`);
    d.minimum = ini.getInt('ENGINE_DATA', 'MINIMUM') || 1000;
    if (ini.getString('HEADER', 'COAST_CURVE') === 'FROM_COAST_REF') {
      const rpm = ini.getFloat('COAST_REF', 'RPM'), torque = ini.getFloat('COAST_REF', 'TORQUE'), nl = ini.getFloat('COAST_REF', 'NON_LINEARITY');
      const lin = (1 - nl) * rpm - d.minimum, quad = nl * rpm;
      d.coast1 = nz(lin) ? -(torque / lin) : 0;
      d.coast2 = nz(quad) ? torque / (quad * quad) : 0;
    }
    this.inertia = ini.getFloat('ENGINE_DATA', 'INERTIA');
    this.defaultLimiter = ini.getInt('ENGINE_DATA', 'LIMITER');
    d.limiter = this.defaultLimiter;
    const hz = ini.getInt('ENGINE_DATA', 'LIMITER_HZ');
    d.limiterCycles = hz === 0 ? 50 : Math.trunc(Math.trunc(1000 / hz) / 3);
    if (ini.hasSection('COAST_SETTINGS')) {
      this.gasCoastCurve = ini.getCurve('COAST_SETTINGS', 'LUT');
      this.coastDefault = ini.getInt('COAST_SETTINGS', 'DEFAULT');
      this.setCoastSettings(this.coastDefault);
      this.coastEntryRpm = ini.getInt('COAST_SETTINGS', 'ACTIVATION_RPM') + d.minimum;
    }
    for (let i = 0; ini.hasSection(`TURBO_${i}`); i++) {
      const s = `TURBO_${i}`;
      this.turbos.push({
        userSetting: 1, rotation: 0,
        lagDn: ((1 - ini.getFloat(s, 'LAG_DN')) * 1.3333334) * 333.33334, lagUp: ((1 - ini.getFloat(s, 'LAG_UP')) * 1.3333334) * 333.33334,
        maxBoost: ini.getFloat(s, 'MAX_BOOST'), rpmRef: ini.getFloat(s, 'REFERENCE_RPM'), gamma: ini.getFloat(s, 'GAMMA'),
        wastegate: ini.getFloat(s, 'WASTEGATE'), adjustable: ini.getInt(s, 'COCKPIT_ADJUSTABLE') !== 0,
      });
    }
    if (this.turbos.some(t => t.adjustable)) this.setTurboBoostLevel(ini.getFloat('ENGINE_DATA', 'DEFAULT_TURBO_ADJUSTMENT'));
    if (ini.hasSection('OVERLAP')) { d.overlapFreq = ini.getFloat('OVERLAP', 'FREQUENCY'); d.overlapGain = ini.getFloat('OVERLAP', 'GAIN'); d.overlapIdealRpm = ini.getFloat('OVERLAP', 'IDEAL_RPM'); }
    if (exists(files, 'throttle.lut')) this.throttle = Curve.fromLut(fileText(files, 'throttle.lut'));
    if (this.defaultLimiter !== 0) { this.rpmDamageThreshold = this.defaultLimiter * 1.05; this.rpmDamageK = 10; }
    if (ini.hasSection('DAMAGE')) {
      if (this.turbos.length) { this.turboDamageThreshold = ini.getFloat('DAMAGE', 'TURBO_BOOST_THRESHOLD'); this.turboDamageK = ini.getFloat('DAMAGE', 'TURBO_DAMAGE_K'); }
      this.rpmDamageThreshold = ini.getFloat('DAMAGE', 'RPM_THRESHOLD');
      this.rpmDamageK = ini.getFloat('DAMAGE', 'RPM_DAMAGE_K');
    }
    if (ini.hasSection('BOV')) this.bovThreshold = ini.getFloat('BOV', 'PRESSURE_THRESHOLD');
    this.turbos.forEach((_, i) => {
      for (const [name, wastegate] of [['ctrl_turbo', false], ['ctrl_wastegate', true]])
        if (exists(files, `${name}${i}.ini`)) this.turboControllers.push({ turbo: i, controller: Controller.load(files, `${name}${i}.ini`), wastegate });
    });
    if (ini.hasSection('THROTTLE_RESPONSE')) { this.throttleMaxRef = ini.getFloat('THROTTLE_RESPONSE', 'RPM_REFERENCE'); this.throttleMax = ini.getCurve('THROTTLE_RESPONSE', 'LUT'); }
    this.reset();
    // the maximum power and torque on a 50 rpm grid
    this.maxPowerW = 0; this.maxTorqueNm = 0; this.maxPowerRpm = 0; this.maxTorqueRpm = 0;
    const last = d.power.refs[d.power.refs.length - 1];
    for (let rpm = 0; last > 0 && rpm < last; rpm += 50) {
      const tq = d.power.getValue(rpm), p = rpm * tq * 0.1047;
      if (tq > this.maxTorqueNm) { this.maxTorqueNm = tq; this.maxTorqueRpm = rpm; }
      if (p > this.maxPowerW) { this.maxPowerW = p; this.maxPowerRpm = rpm; }
    }
  }
  get limiterRpm() { return Math.trunc(this.data.limiter * this.limiterMultiplier); }
  get minimum() { return this.data.minimum; }
  get maxPower() {
    if (this.maxPowerWDynamic > 0) return this.maxPowerWDynamic;
    return (this.turbos.reduce((s, t) => s + t.maxBoost, 0) + 1) * this.maxPowerW;
  }
  setCoastSettings(i) { if (i >= 0 && i <= this.gasCoastCurve.count) this.gasCoastOffset = this.gasCoastCurve.getValue(i); }
  setTurboBoostLevel(level) { for (const t of this.turbos) t.userSetting = t.adjustable ? level : 1; }
  reset() { for (const t of this.turbos) t.rotation = 0; this.lifeLeft = 1000; }
  throttleResponse(gas, rpm) {
    if (this.throttle.count && this.throttleMax.count) {
      const lo = sat(this.throttle.getValue(gas * 100) * 0.01), hi = sat(this.throttleMax.getValue(gas * 100) * 0.01);
      const r = rpm / this.throttleMaxRef, k = r > 1 ? 1 : r >= 0 ? r : 0;
      return (hi - lo) * k + lo;
    }
    if (this.throttle.count) return sat(this.throttle.getValue(gas * 100) * 0.01);
    return gas;
  }
  // torque at an rpm with the current coast model (used by the HUD's dyno and by tools)
  torqueAt(rpm, gas) {
    const d = this.data, last = this.lastInput.rpm;
    let coast = d.coast0;
    if (nz(d.coast1)) coast += d.coast1 * last * this.coastTorqueMultiplier;
    if (nz(d.coast2)) coast -= d.coast2 * this.coastTorqueMultiplier * last * last * Math.sign(last);
    return (d.power.getValue(rpm) - coast) * gas + coast;
  }
  step(gasIn, rpm, dt, car) {
    const d = this.data, minimum = d.minimum;
    this.lastInput = { gas: this.throttleResponse(gasIn, rpm), rpm };
    if (this.gasCoastOffset > 0) {
      const offset = this.gasCoastOffset * sat((rpm - minimum) / this.coastEntryRpm);
      this.lastInput.gas = sat((1 - offset) * this.lastInput.gas + offset);
    }
    if (d.limiter !== 0 && !(d.limiter * this.limiterMultiplier >= rpm)) this.limiterOn = d.limiterCycles;
    if (this.limiterOn > 0) { this.limiterOn--; this.lastInput.gas = 0; }
    if (0 >= this.lifeLeft) this.fuelPressure = 0;
    const gas = this.electronicOverride * this.lastInput.gas;
    this.lastInput.gas = gas;
    this.gasUsage = gas;
    let torque = d.power.getValue(rpm), coast = 0;
    if (nz(d.coast1)) coast = (rpm - minimum) * d.coast1;
    // turbos (the controllers first; a turbo always steps 3 ms)
    for (const e of this.turboControllers) {
      const v = e.controller.eval(car);
      if (e.wastegate) this.turbos[e.turbo].wastegate = v; else this.turbos[e.turbo].maxBoost = v;
    }
    this.turboBoost = 0;
    const trpm = !(0 >= this.lastInput.rpm) ? this.lastInput.rpm : 0, tgas = this.lastInput.gas;
    for (const t of this.turbos) {
      let target = 0;
      if (trpm > 0 && tgas > 0) target = Math.pow(sat(tgas * trpm / t.rpmRef), t.gamma);
      const lag = target > t.rotation ? 0.003 * t.lagUp : 0.003 * t.lagDn;
      t.rotation = (target - t.rotation) * (lag > 1 ? 1 : lag >= 0 ? lag : 0) + t.rotation;
      if (nz(t.wastegate)) { const limit = t.wastegate * t.userSetting; if (t.maxBoost * t.rotation > limit) t.rotation = limit / t.maxBoost; }
      this.turboBoost = t.maxBoost * t.rotation * this.fuelPressure + this.turboBoost;
    }
    const boost = this.turboBoost;
    if (nz(boost)) torque *= boost + 1;
    if (nz(d.coast2)) { const above = rpm - minimum; coast -= above * above * d.coast2 * (rpm > 0 ? 1 : rpm >= 0 ? 0 : -1); }
    if (!(rpm > minimum)) coast = 0;
    this.bov = (1 - gas) * boost > this.bovThreshold ? 1 : 0;
    const damageRate = car.env.mechanicalDamageRate;
    if (nz(this.turboDamageThreshold) && boost > this.turboDamageThreshold) this.lifeLeft -= (boost - this.turboDamageThreshold) * this.turboDamageK * 0.003 * damageRate;
    if (nz(this.rpmDamageThreshold) && rpm > this.rpmDamageThreshold) this.lifeLeft -= (rpm - this.rpmDamageThreshold) * this.rpmDamageK * 0.003 * damageRate;
    let air = car.env.airDensity * 0.82630974;
    if (this.restrictor > 0) { air -= this.restrictor * rpm * 0.0001 * gas; if (!(air >= 0)) air = 0; }
    const blended = ((torque - coast) * gas + coast) * air;
    this.outTorque = blended;
    if (!(0 >= this.fuelPressure)) {
      if (rpm >= minimum) {
        if (nz(d.overlapGain)) {
          const phase = car.physicsTime * 0.001 * d.overlapFreq * rpm * 0.00033333333333333;
          this.outTorque = (Math.sin(phase) * 0.5 - 0.5) * Math.abs(rpm - d.overlapIdealRpm) * d.overlapGain + blended;
        }
      } else this.outTorque = blended >= 15 ? blended : 15;
    }
    if (!(this.fuelPressure >= 1)) { const drag = rpm * -0.01; this.outTorque = (this.outTorque - drag) * this.fuelPressure + drag; }
    this.isLimiterOn = this.limiterOn !== 0;
    this.electronicOverride = 1;
    const p = rpm * this.outTorque * 0.10471975803375244;
    if (p > this.maxPowerWDynamic) this.maxPowerWDynamic = p;
  }
}

// --- drivetrain (2WD: RWD and FWD) ---------------------------------------------------------------------------

export class Drivetrain {
  constructor(car, files, engine) {
    this.engine = engine;
    this.finalRatio = 4; this.engineEl = { v: 0, inertia: 0.009999999776482582 }; this.drive = { v: 0, inertia: 0.009999999776482582 };
    this.outL = { v: 0, old: 0, inertia: car.tyres[2].data.angularInertia }; this.outR = { v: 0, old: 0, inertia: car.tyres[3].data.angularInertia };
    this.gears = []; this.rootVelocity = 0; this.clutchOpen = false; this.ratio = 12; this.diffPower = 0.7; this.diffCoast = 0.2; this.diffPreload = 0;
    this.spool = false; this.cutOff = 0; this.clutchMaxTorque = 0; this.totalTorque = 0; this.currentClutchTorque = 0; this.traction = 'RWD';
    this.currentGear = 0; this.lastRatio = -1; this.tyreLeft = 2; this.tyreRight = 3;
    this.request = { kind: 'none', time: 0, timeout: 200, gear: -1 };
    this.gearUpTime = 0.10000000149011612; this.gearDnTime = 0.15000000596046448; this.autoCutOffTime = 0;
    this.validShiftRpmWindow = 0; this.controlsWindowGain = 0; this.damageRpmWindow = 0; this.orgRpmWindow = 0;
    this.clutchInertia = 1; this.locClutch = 1; this.isGearGrinding = false;
    this.downshift = { active: false, overrev: 0, lockN: true };
    const ini = new IniReader(files, 'drivetrain.ini');
    if (ini.hasSection('HEADER') && ini.getInt('HEADER', 'VERSION') >= 2 && ini.hasSection('DAMAGE')) this.damageRpmWindow = ini.getFloat('DAMAGE', 'RPM_WINDOW_K');
    const count = ini.getInt('GEARS', 'COUNT');
    this.gears.push({ ratio: ini.getFloat('GEARS', 'GEAR_R'), name: 'R' }, { ratio: 0, name: 'N' });
    for (let i = 1; i <= count; i++) this.gears.push({ ratio: ini.getFloat('GEARS', `GEAR_${i}`), name: String(i) });
    this.finalRatio = ini.getFloat('GEARS', 'FINAL');
    this.currentGear = 1;
    this.diffPower = ini.getFloat('DIFFERENTIAL', 'POWER');
    this.diffCoast = ini.getFloat('DIFFERENTIAL', 'COAST');
    if (!(1 > this.diffPower) && !(1 > this.diffCoast)) this.spool = true;
    this.diffPreload = ini.getFloat('DIFFERENTIAL', 'PRELOAD');
    const traction = ini.getString('TRACTION', 'TYPE');
    if (traction === 'FWD') {
      this.traction = 'FWD'; this.tyreLeft = 0; this.tyreRight = 1;
      this.outL.inertia = car.tyres[0].data.angularInertia; this.outR.inertia = car.tyres[1].data.angularInertia;
      car.tyres.forEach((t, i) => { t.driven = i < 2; });
    } else if (traction === 'RWD') {
      car.tyres.forEach((t, i) => { t.driven = i >= 2; });
    } else throw new Error(`drivetrain.ini: [TRACTION] TYPE=${traction} is not ported yet (only RWD and FWD)`);
    this.gearUpTime = ini.getFloat('GEARBOX', 'CHANGE_UP_TIME') * 0.001;
    this.gearDnTime = ini.getFloat('GEARBOX', 'CHANGE_DN_TIME') * 0.001;
    if (!nz(this.gearDnTime)) this.gearDnTime = 0.15;
    if (!nz(this.gearUpTime)) this.gearUpTime = 0.1;
    this.autoCutOffTime = ini.getFloat('GEARBOX', 'AUTO_CUTOFF_TIME') * 0.001;
    this.shifterSupported = ini.getInt('GEARBOX', 'SUPPORTS_SHIFTER') !== 0;
    if (ini.hasSection('DOWNSHIFT_PROTECTION') && !this.shifterSupported) {
      this.downshift.active = ini.getInt('DOWNSHIFT_PROTECTION', 'ACTIVE') !== 0;
      this.downshift.overrev = ini.getInt('DOWNSHIFT_PROTECTION', 'OVERREV');
      this.downshift.lockN = ini.getInt('DOWNSHIFT_PROTECTION', 'LOCK_N') !== 0;
    } else this.downshift.active = !this.shifterSupported;
    this.clutchMaxTorque = ini.getFloat('CLUTCH', 'MAX_TORQUE');
    if (!nz(this.clutchMaxTorque)) this.clutchMaxTorque = 450;
    this.validShiftRpmWindow = ini.getFloat('GEARBOX', 'VALID_SHIFT_RPM_WINDOW');
    this.orgRpmWindow = this.validShiftRpmWindow;
    this.controlsWindowGain = ini.getFloat('GEARBOX', 'CONTROLS_WINDOW_GAIN');
    if (!nz(this.validShiftRpmWindow)) this.validShiftRpmWindow = 500;
    const inertia = ini.getFloat('GEARBOX', 'INERTIA');
    if (nz(inertia)) { this.clutchInertia = inertia; this.drive.inertia = inertia; }
    this.singleDiffLock = this.traction === 'RWD' && exists(files, 'ctrl_single_lock.ini') ? Controller.load(files, 'ctrl_single_lock.ini') : null;
  }
  get engineRpm() { return this.engineEl.v * K2PI * 60; }
  get gearLabel() { return this.gears[this.currentGear].name; }
  get isChangingGear() { return this.request.kind !== 'none'; }
  inertiaFromEngine() {
    if (!nz(this.ratio)) return this.engineEl.inertia;
    return ((this.outL.inertia + this.drive.inertia) + this.outR.inertia) / Math.abs(this.ratio * this.ratio) + (this.clutchInertia + this.engineEl.inertia);
  }
  inertiaFromWheels() {
    if (!nz(this.ratio)) return (this.outL.inertia + this.drive.inertia) + this.outR.inertia;
    const r2 = Math.abs(this.ratio * this.ratio);
    return (((r2 * (this.clutchOpen ? this.clutchInertia : this.clutchInertia + this.engineEl.inertia)) + this.drive.inertia) + this.outL.inertia) + this.outR.inertia;
  }
  accelerateBlock(acc) { this.drive.v += acc; this.outR.v += acc; this.outL.v += acc; }
  reallignSpeeds() {
    const r = this.ratio;
    if (!nz(r)) return;
    const drive = this.drive.v;
    if (!(0.9 >= this.locClutch)) {
      const root = this.rootVelocity, k = 1 - this.engineEl.inertia / this.inertiaFromEngine();
      this.rootVelocity = root - (k * (root / r - drive)) * Math.abs(r);
    } else this.rootVelocity = drive * r;
    this.accelerateBlock(this.rootVelocity / r - drive);
    if (!this.clutchOpen) this.engineEl.v = this.rootVelocity;
  }
  step(car, dt) {
    this.outL.old = this.outL.v; this.outR.old = this.outR.v;
    this.locClutch = Math.pow(car.controls.clutch, 1.5);
    this.currentClutchTorque = 0;
    if (this.singleDiffLock) { this.diffPower = 0; this.diffCoast = 0; this.diffPreload = this.singleDiffLock.eval(car); }
    const L = this.tyreLeft, R = this.tyreRight, t = car.tyres;
    const rq = this.request;
    if (rq.kind !== 'none' && !(rq.timeout >= rq.time)) { rq.kind = 'none'; this.currentGear = rq.gear; }
    if (rq.kind !== 'none') rq.time += dt;
    this.ratio = this.finalRatio * this.gears[this.currentGear].ratio;
    this.engineEl.inertia = this.engine.inertia;
    if (this.lastRatio < this.ratio || this.lastRatio > this.ratio) { this.reallignSpeeds(); this.lastRatio = this.ratio; }
    let gas;
    if (this.cutOff > 0) { gas = 0; this.cutOff -= dt; } else gas = car.controls.gas;
    this.engine.step(gas, this.engineEl.v * K2PI * 60, dt, car);
    const torque = this.engine.outTorque, lc = this.locClutch;
    if (!(lc >= 1)) this.clutchOpen = true;
    else if (nz(this.engineEl.v)) this.clutchOpen = Math.abs(this.rootVelocity / this.engineEl.v - 1) >= 0.1;
    else this.clutchOpen = nz(this.rootVelocity);
    const ei = this.engineEl.inertia, r = this.ratio, ife = this.inertiaFromEngine(), ifw = this.inertiaFromWheels(), inGear = nz(r);
    let clutchTorque = 0;
    if (this.clutchOpen) {
      const es = this.engineEl.v, root = this.rootVelocity, slip = es - root;
      clutchTorque = -((slip / (Math.abs(slip) + 4)) * lc * this.clutchMaxTorque);
      this.currentClutchTorque = clutchTorque;
      if (inGear) {
        this.engineEl.v = ((clutchTorque + torque) / ei) * dt + es;
        const d = dt * (-clutchTorque / (ife - ei));
        this.rootVelocity = d + root;
        this.accelerateBlock(d / r);
      } else { const s = (torque / ei) * dt + es; this.engineEl.v = s; this.rootVelocity = s; }
    } else if (inGear) {
      const d = dt * (torque / ife);
      this.rootVelocity += d;
      this.accelerateBlock(d / r);
    } else this.rootVelocity += (torque / ife) * dt;
    const fl = t[L].status.feedbackTorque, fr = t[R].status.feedbackTorque, fb = fr + fl;
    const a = dt * (fb / ifw);
    if (inGear) this.rootVelocity += a * this.ratio;
    this.drive.v += a; this.outR.v += a; this.outL.v += a;
    if (this.spool) { this.outL.v = this.drive.v; this.outR.v = this.drive.v; }
    else {
      let inputTorque = lc * torque;
      if (nz(clutchTorque)) inputTorque = -clutchTorque;
      const ar = Math.abs(this.ratio);
      let lock = inputTorque > 0 ? ar * (this.diffPower * inputTorque) : Math.abs(ar * (this.diffCoast * inputTorque));
      let left = this.outL.v, right = this.outR.v;
      const carrier = this.drive.v;
      lock += this.diffPreload;
      const friction = -(((left - right) / (Math.abs(left - right) + 0.009999999776482582)) * lock);
      const slipping = Math.abs(left - carrier) >= 0.10000000149011612 || Math.abs(fr - fl) > lock;
      if (!slipping) { this.outR.v = carrier; this.outL.v = carrier; }
      else {
        const x = dt * ((friction / this.outL.inertia) * 0.5);
        left += x; right -= x;
        const y = dt * (((fr - fl) / this.outR.inertia) * 0.5);
        this.outL.v = left - y; this.outR.v = y + right;
      }
    }
    if (t[L].status.isLocked && t[R].status.isLocked) {
      const brake = t[L].absOverride * t[L].inputs.brakeTorque + t[L].inputs.handBrakeTorque + t[R].absOverride * t[R].inputs.brakeTorque + t[R].inputs.handBrakeTorque;
      if (Math.abs(Math.abs(this.ratio) * torque) > brake || !(1 >= car.speed)) { t[L].status.isLocked = false; t[R].status.isLocked = false; }
      else if (this.clutchOpen) { this.rootVelocity = 0; this.drive.v = 0; this.outL.v = 0; this.outR.v = 0; }
    }
    if (!this.clutchOpen) this.engineEl.v = this.rootVelocity;
    t[L].status.angularVelocity = this.outL.v;
    t[R].status.angularVelocity = this.outR.v;
    t[L].stepRotationMatrix(dt, car);
    t[R].stepRotationMatrix(dt, car);
    this.totalTorque = nz(this.ratio) ? Math.abs(Math.abs(this.ratio) * (torque * lc) - (fl + fr)) : Math.abs(torque * lc);
  }
  gearUp(car) {
    if (this.currentGear < 0 || this.currentGear >= this.gears.length - 1 || this.request.kind !== 'none') return false;
    Object.assign(this.request, { timeout: this.gearUpTime, kind: 'up', time: 0, gear: this.currentGear + 1 });
    car.onGearRequest('up');
    if (nz(this.autoCutOffTime)) this.cutOff = this.autoCutOffTime;
    this.currentGear = 1;
    return true;
  }
  gearDown(car) {
    if (this.downshift.active && this.currentGear > 0) {
      const target = this.gears[this.currentGear - 1].ratio;
      if (nz(target) && nz(this.gears[this.currentGear].ratio)) {
        const accL = Math.min(0, (this.outL.v - this.outL.old) * INV_DT), accR = Math.min(0, (this.outR.v - this.outR.old) * INV_DT);
        const wheels = ((this.gearDnTime * accR + this.outR.v) + (this.gearDnTime * accL + this.outL.v)) * 0.5;
        const rpm = target * wheels * this.finalRatio * K2PI * 60;
        if (rpm > this.engine.limiterRpm + this.downshift.overrev) return false;
      } else if (this.downshift.lockN && !nz(target) && car.speed * 3.6 > 2) return false;
    }
    if (this.currentGear <= 0 || this.request.kind !== 'none') return false;
    Object.assign(this.request, { kind: 'down', timeout: this.gearDnTime, time: 0, gear: this.currentGear - 1 });
    car.onGearRequest('down');
    this.currentGear = 1;
    return true;
  }
  setCurrentGear(index, force, car) {
    this.isGearGrinding = false;
    if (index < 0 || index >= this.gears.length || index === this.currentGear) return;
    if (index === 1 || force) { this.currentGear = index; return; }
    const diff = Math.abs(this.engineEl.v - this.gears[index].ratio * this.drive.v * this.finalRatio);
    const closed = this.locClutch * diff;
    const mismatch = ((car.controls.gas * this.locClutch * diff - closed) * this.controlsWindowGain + closed) * K2PI * 60;
    if (!(mismatch >= this.validShiftRpmWindow)) { this.currentGear = index; return; }
    this.isGearGrinding = true;
    if (this.validShiftRpmWindow > 0 && car.env.mechanicalDamageRate > 0)
      this.validShiftRpmWindow -= this.damageRpmWindow * 0.003000000026077032 * car.env.mechanicalDamageRate;
  }
  reset() {
    this.clutchOpen = true; this.rootVelocity = 0; this.engineEl.v = 0; this.outL.v = 0; this.outR.v = 0; this.drive.v = 0;
    this.request.kind = 'none'; this.validShiftRpmWindow = this.orgRpmWindow;
    this.engine.reset();
  }
}

// --- shift helpers -------------------------------------------------------------------------------------------

export class Autoclutch {
  constructor(files) {
    this.rpmMin = 1500; this.rpmMax = 2500; this.speed = 1; this.useOnStart = true; this.useOnChange = true; this.forced = false;
    this.sequence = { curve: new Curve(), time: 0, done: true }; this.up = new Curve(); this.down = new Curve(); this.signal = 0;
    const ini = new IniReader(files, 'drivetrain.ini');
    if (ini.hasSection('HEADER') && ini.getInt('HEADER', 'VERSION') > 2) this.forced = ini.getInt('AUTOCLUTCH', 'FORCED_ON') !== 0;
    const upName = ini.getString('AUTOCLUTCH', 'UPSHIFT_PROFILE'), dnName = ini.getString('AUTOCLUTCH', 'DOWNSHIFT_PROFILE');
    this.useOnChange = ini.getInt('AUTOCLUTCH', 'USE_ON_CHANGES') !== 0;
    for (const [name, p] of [[upName, this.up], [dnName, this.down]]) if (name !== 'NONE') {
      p.addValue(0, 1);
      p.addValue(ini.getFloat(name, 'POINT_0') * 0.001, 0);
      p.addValue(ini.getFloat(name, 'POINT_1') * 0.001, 0);
      p.addValue(ini.getFloat(name, 'POINT_2') * 0.001, 1);
    }
    this.rpmMin = ini.getInt('AUTOCLUTCH', 'MIN_RPM');
    this.rpmMax = ini.getInt('AUTOCLUTCH', 'MAX_RPM');
    if (this.rpmMin === 0 || this.rpmMax === 0) { this.rpmMin = 1500; this.rpmMax = 2500; }
  }
  step(controls, speed, engineVelocity, gear, dt) {
    const sq = this.sequence;
    if (!sq.done) {
      if (speed * 3.6 > 5) {
        this.signal = sq.curve.getValue(sq.time);
        sq.time += dt;
        if (!(sq.curve.refs[sq.curve.refs.length - 1] >= sq.time)) sq.done = true;
        controls.clutch = sat(this.signal);
        return;
      }
      sq.done = true;
    }
    if (!this.useOnStart && !this.forced) return;
    let target = 1, toZero = false;
    const rpm = engineVelocity * K2PI * 60;
    if (gear === 0 || gear === 2) {
      if (rpm >= this.rpmMin && !(rpm > this.rpmMax)) { target = (rpm - this.rpmMin) / (this.rpmMax - this.rpmMin); this.signal = target; }
      if (rpm > this.rpmMax) target = 1;
      toZero = !(rpm >= this.rpmMin);
    } else if (gear === 1) {
      if (!(speed * 3.6 >= 5)) { if (0.2 >= controls.gas) toZero = true; else this.signal = 1; }
    } else toZero = !(rpm >= this.rpmMin);
    if (toZero) { target = 0; this.signal = 0; }
    const step = dt * this.speed;
    if (Math.abs(target - this.signal) >= step) this.signal += target > this.signal ? step : -step;
    else this.signal = target;
    controls.clutch = sat(this.signal);
  }
  onGearRequest(kind) {
    if (!this.useOnChange) return;
    const closed = this.signal > 0.01;
    if (kind === 'down' && closed && this.down.count === 4) this.sequence = { curve: this.down, time: 0, done: false };
    if (kind === 'up' && closed && this.up.count === 4) this.sequence = { curve: this.up, time: 0, done: false };
  }
}

export class AutoBlip {
  constructor(files) {
    const ini = new IniReader(files, 'drivetrain.ini'), level = ini.getFloat('AUTOBLIP', 'LEVEL');
    this.profile = new Curve([[0, 0], [ini.getFloat('AUTOBLIP', 'POINT_0'), level], [ini.getFloat('AUTOBLIP', 'POINT_1'), level], [ini.getFloat('AUTOBLIP', 'POINT_2'), 0]]);
    this.performTime = this.profile.refs[3];
    this.electronic = nz(ini.getFloat('AUTOBLIP', 'ELECTRONIC'));
    this.startTime = 0;
    this.active = true;
  }
  onGearRequest(kind, clutch, time) { if (kind === 'down' && !(0.1 >= clutch)) this.startTime = time; }
  step(controls, locked, speed, time) {
    if (locked || !(speed * 3.6 >= 5) || (!this.electronic && !this.active)) return;
    const since = time - this.startTime;
    if (!(since >= 0) || since >= this.performTime || this.profile.count !== 4) return;
    controls.gas = sat(Math.max(controls.gas, this.profile.getValue(since)));
  }
}

export class AutoShifter {
  constructor(files) {
    this.active = false; this.up = 0; this.down = 4000; this.slipThreshold = 0.8; this.gasCutoff = 0; this.gasCutoffTime = 0.5;
    const ini = new IniReader(files, 'drivetrain.ini');
    if (ini.ready && ini.hasSection('AUTO_SHIFTER')) {
      this.up = ini.getInt('AUTO_SHIFTER', 'UP'); this.down = ini.getInt('AUTO_SHIFTER', 'DOWN');
      this.slipThreshold = ini.getFloat('AUTO_SHIFTER', 'SLIP_THRESHOLD'); this.gasCutoffTime = ini.getFloat('AUTO_SHIFTER', 'GAS_CUTOFF_TIME');
    } else {
      const ai = new IniReader(files, 'ai.ini');
      if (ai.ready) { this.up = ai.getInt('GEARS', 'UP'); this.down = ai.getInt('GEARS', 'DOWN'); this.slipThreshold = ai.getFloat('GEARS', 'SLIP_THRESHOLD'); this.gasCutoffTime = ai.getFloat('GEARS', 'GAS_CUTOFF_TIME'); }
    }
  }
  step(car, dt) {
    const d = car.drivetrain, c = car.controls;
    if (!this.active || car.physicsTime < 300 + car.env.sessionStartTimeMs) return;
    const gear = d.currentGear;
    if (gear === 0) return;
    if (this.up === 0) {
      const e = d.engine, lower = Math.min(e.limiterRpm, Math.trunc(e.maxPowerRpm));
      this.up = Math.trunc(lower * 0.98);
      this.down = Math.trunc(e.maxTorqueRpm * 1.1);
    }
    if (c.gearUp || c.gearDn) return;
    const first = d.traction === 'FWD' ? 0 : 2;
    const slip = Math.max(car.tyres[first].status.ndSlip, car.tyres[first + 1].status.ndSlip);
    const slipping = slip > this.slipThreshold && car.speed > 5;
    if (!d.isChangingGear && (c.clutch > 0.99 || gear === 1) && !slipping) {
      const rpm = d.engineRpm, top = d.gears.length - 1;
      if (rpm > this.up && gear < top && c.gas > 0.2 && 0 >= this.gasCutoff) { c.gearUp = true; this.gasCutoff = this.gasCutoffTime; }
      const down = gear === 3 ? Math.trunc(this.down * 0.65) : this.down;
      if (!(rpm >= down) && gear > 2 && c.clutch > 0.85 && 0 >= this.gasCutoff) c.gearDn = true;
    }
    if (!(car.speed >= 2) && !d.isChangingGear && !(c.gas >= 0.1) && 0 >= this.gasCutoff && gear > 2) c.gearDn = true;
    if (this.gasCutoff > 0) { this.gasCutoff -= dt; c.gas = 0; }
  }
}

export class GearChanger {
  constructor() { this.lastUp = false; this.lastDn = false; }
  step(car) {
    const d = car.drivetrain, c = car.controls;
    if (c.requestedGear !== -1) { d.setCurrentGear(c.requestedGear, false, car); return; }
    if (c.gearUp && !this.lastUp) d.gearUp(car);
    if (c.gearDn && !this.lastDn) d.gearDown(car);
    this.lastUp = c.gearUp;
    this.lastDn = c.gearDn;
  }
}

// --- aero ----------------------------------------------------------------------------------------------------

function plane(p0, p1, p2) {
  const a = v3.sub(p2, p0), b = v3.sub(p1, p0);
  const n = [a[2] * b[1] - a[1] * b[2], a[0] * b[2] - a[2] * b[0], a[1] * b[0] - a[0] * b[1]];
  const sq = n[1] * n[1] + n[0] * n[0] + n[2] * n[2];
  if (!(sq > 0)) return [0, 0, 0, 0];
  const k = 1 / Math.sqrt(sq), x = n[0] * k, y = n[1] * k, z = n[2] * k;
  return [x, y, z, -(y * p0[1] + x * p0[0] + z * p0[2])];
}
function groundHeight(car, p) {
  const t = car.tyres;
  const h = pl => {
    const den = -pl[1];
    if (!nz(den)) return 0;
    return -((p[1] * pl[1] + pl[0] * p[0] + pl[2] * p[2] + pl[3]) / den) * -1 + p[1];
  };
  const a = h(plane(t[0].contactPoint, t[1].contactPoint, t[2].contactPoint)), b = h(plane(t[0].contactPoint, t[1].contactPoint, t[3].contactPoint));
  return ((p[1] - b) + (p[1] - a)) * 0.5;
}

export class Aero {
  constructor(files) {
    const ini = new IniReader(files, 'aero.ini');
    this.referenceArea = 1; this.frontShare = 0.5; this.cda = 0.1; this.airDensity = 1.221; this.cd = 0; this.cl = 0; this.cdx = 0; this.cdy = 0;
    this.wings = [];
    const version = ini.getInt('HEADER', 'VERSION');
    for (const [prefix, vertical] of [['WING', false], ['FIN', true]]) for (let i = 0; ini.hasSection(`${prefix}_${i}`); i++) {
      const s = `${prefix}_${i}`, lut = k => Curve.fromLut(fileText(files, ini.getString(s, k)));
      const w = {
        name: ini.getString(s, 'NAME'), vertical, area: ini.getFloat(s, 'CHORD') * ini.getFloat(s, 'SPAN'), position: ini.getFloat3(s, 'POSITION'),
        aoaCl: lut('LUT_AOA_CL'), aoaCd: lut('LUT_AOA_CD'), ghCl: lut('LUT_GH_CL'), ghCd: lut('LUT_GH_CD'),
        cdGain: ini.getFloat(s, 'CD_GAIN'), clGain: ini.getFloat(s, 'CL_GAIN'), angle: ini.getFloat(s, 'ANGLE'), angleMult: 1,
        yawGain: version >= 3 ? ini.getFloat(s, 'YAW_CL_GAIN') : 0, controllers: [],
        aoa: 0, yawAngle: 0, cd: 0, cl: 0, dragKg: 0, liftKg: 0, groundHeight: 0,
      };
      w.inputAngle = w.angle;
      this.wings.push(w);
    }
    if (!this.wings.length) {
      if (!ini.hasSection('DATA')) throw new Error('aero.ini does not contain WINGS nor DATA');
      this.referenceArea = ini.getFloat('DATA', 'REFERENCE_AREA'); this.cd = ini.getFloat('DATA', 'CD'); this.cl = ini.getFloat('DATA', 'CL');
      this.frontShare = ini.getFloat('DATA', 'FRONT_SHARE'); this.cdx = ini.getFloat('DATA', 'CDX'); this.cdy = ini.getFloat('DATA', 'CDY');
    }
    for (let i = 0; ini.hasSection(`DYNAMIC_CONTROLLER_${i}`); i++) {
      const s = `DYNAMIC_CONTROLLER_${i}`, wing = ini.getInt(s, 'WING');
      if (wing < 0 || wing >= this.wings.length) continue;
      this.wings[wing].controllers.push({
        input: ini.getString(s, 'INPUT'), combinator: ini.getString(s, 'COMBINATOR'), lut: Curve.fromLut(fileText(files, ini.getString(s, 'LUT'))),
        filter: ((1 - ini.getFloat(s, 'FILTER')) * 1.3333334) * 333.33334, up: ini.getFloat(s, 'UP_LIMIT'), down: ini.getFloat(s, 'DOWN_LIMIT'), out: 0,
      });
    }
  }
  step(car) {
    if (!this.wings.length) { this.#body(car); return; }
    for (const w of this.wings) this.#wing(car, w);
  }
  #wing(car, w) {
    const c = car.core, b = car.body;
    if (w.controllers.length) {
      let angle = w.inputAngle;
      for (const ct of w.controllers) {
        const inp = { BRAKE: car.controls.brake, GAS: car.controls.gas, LATG: car.accG[0], LONG: car.accG[2], STEER: car.controls.steer, SPEED_KMH: car.speed * 3.6,
          SUS_TRAVEL_LR: car.suspensions[2].status.travel * 1000, SUS_TRAVEL_RR: car.suspensions[3].status.travel * 1000 }[ct.input] ?? 0;
        let v = ct.lut.getValue(inp);
        if (Math.abs(v - ct.out) >= 0.001) v = (v - ct.out) * sat(ct.filter * 0.003) + ct.out;
        ct.out = v;
        if (ct.combinator === 'ADD') angle += v; else if (ct.combinator === 'MULT') angle *= v;
        if (angle > ct.up) angle = ct.up; else if (!(angle >= ct.down)) angle = ct.down;
      }
      w.angle = angle;
    }
    const lv = c.localPointVelocity(b, w.position);
    // wind is zero here (no weather); AC adds the ground wind vector to the world velocity
    const v = c.worldToLocalNormal(b, lv);
    w.groundHeight = groundHeight(car, c.localToWorld(b, w.position));
    if (!nz(v[2])) { w.aoa = 0; w.cd = 0; w.yawAngle = 0; w.cl = 0; return; }
    const inv = 1 / v[2];
    w.aoa = Math.atan(inv * v[1]) * 57.29578;
    w.yawAngle = Math.atan(inv * v[0]) * 57.29578;
    const flow = w.vertical ? w.yawAngle : w.aoa, rho = this.airDensity;
    // drag
    const sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const n = nz(Math.sqrt(sq)) ? v3.scale(v, 1 / Math.sqrt(sq)) : v;
    w.cd = w.aoaCd.getValue(w.angleMult * w.angle + flow) * w.cdGain;
    if (w.ghCd.count) w.cd *= w.ghCd.getValue(w.groundHeight);
    const drag = sq * w.cd * rho * w.area * 0.5;
    w.dragKg = drag * 0.101978384;
    if (nz(sq)) c.addLocalForceAtLocalPos(b, v3.scale(n, -drag), w.position);
    // lift
    const across = w.vertical ? v[0] : v[1], sql = across * across + v[2] * v[2];
    w.cl = w.aoaCl.getValue(w.angleMult * w.angle + flow) * w.clGain;
    if (0 > v[2]) w.cl = 0;
    if (nz(w.yawGain) && !w.vertical) w.cl *= sat(Math.sin(Math.abs(w.yawAngle) * 0.017453) * w.yawGain + 1);
    if (w.ghCl.count) w.cl *= w.ghCl.getValue(w.groundHeight);
    const lift = sql * w.cl * rho * w.area * 0.5;
    w.liftKg = lift * 0.101978384;
    if (!nz(sql)) return;
    const u = nz(Math.sqrt(sq)) ? v3.scale(v, 1 / Math.sqrt(sq)) : v;
    // the lift acts across the flow: (0, vz, -vy) for a wing, (vz... ) for a fin
    const dir = w.vertical ? [-u[2], 0, u[0]] : [0, u[2], -u[1]];
    c.addLocalForceAtLocalPos(b, v3.scale(dir, -lift), w.position);
  }
  #body(car) {
    const c = car.core, b = car.body, v = c.localVelocity(b);
    const sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (nz(sq)) {
      const n = v3.scale(v, 1 / Math.sqrt(sq));
      const dcd = (Math.abs(n[0]) * this.cd * this.cdx + this.cd) + Math.abs(n[1]) * this.cd * this.cdy;
      c.addLocalForce(b, v3.scale(n, -(dcd * sq * this.airDensity * this.referenceArea * 0.5)));
      const w = c.angularVelocity(b), spin = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
      if (nz(spin)) c.addLocalTorque(b, v3.scale(w, -spin / Math.sqrt(spin) * this.cda));
    }
    const zz = v[2] * v[2];
    if (!nz(zz)) return;
    const lift = zz * this.cl * this.airDensity * this.referenceArea * 0.5;
    c.addLocalForceAtLocalPos(b, [0, -(lift * this.frontShare), 0], [0, 0, 0]);
    c.addLocalForceAtLocalPos(b, [0, -((1 - this.frontShare) * lift), 0], [0, 0, 0]);
  }
  get dragKg() { return this.wings.reduce((s, w) => s + w.dragKg, 0); }
  get liftKg() { return this.wings.reduce((s, w) => s + w.liftKg, 0); }
}

// --- aids ----------------------------------------------------------------------------------------------------

export class Aids {
  constructor(files, traction) {
    const has = exists(files, 'electronics.ini'), ini = has ? new IniReader(files, 'electronics.ini') : null;
    const rate = (s, d) => { const r = ini.getFloat(s, 'RATE_HZ'); return nz(r) ? 1 / r : d; };
    this.tc = { present: true, active: true, limit: 0.2, inAction: false, frequency: 0.05, minSpeed: 0, acc: 0 };
    this.abs = { present: true, active: true, limit: 0.2, frequency: 0.05, channels: 4, acc: 0 };
    this.edl = { present: false, active: false, gainPower: 0.01, gainCoast: 0.01, deadPower: 0, deadCoast: 0, torquePower: 2000, torqueCoast: 2000, left: 2, right: 3 };
    this.stability = { gain: 0, useBeta: false, maxGain: 30 };
    if (traction === 'FWD') { this.edl.left = 0; this.edl.right = 1; }
    if (ini && ini.ready) {
      const s = 'TRACTION_CONTROL', t = this.tc;
      t.limit = ini.getFloat(s, 'SLIP_RATIO_LIMIT'); t.minSpeed = ini.getFloat(s, 'MIN_SPEED_KMH') * 0.2777778;
      t.present = ini.getInt(s, 'PRESENT') !== 0; t.active = ini.getInt(s, 'ACTIVE') !== 0; t.frequency = rate(s, 0.05);
      const a = this.abs, as = ini.hasSection('ABS_V2') ? 'ABS_V2' : 'ABS';
      a.limit = ini.getFloat(as, 'SLIP_RATIO_LIMIT'); a.present = ini.getInt(as, 'PRESENT') !== 0; a.active = ini.getInt(as, 'ACTIVE') !== 0;
      a.frequency = rate(as, 0.05);
      if (as === 'ABS_V2') a.channels = ini.getInt(as, 'CHANNELS');
      if (ini.hasSection('EDL')) {
        const e = this.edl;
        e.present = ini.getInt('EDL', 'PRESENT') !== 0; e.active = ini.getInt('EDL', 'ACTIVE') !== 0;
        e.torquePower = ini.getFloat('EDL', 'BRAKE_TORQUE_POWER'); e.torqueCoast = ini.getFloat('EDL', 'BRAKE_TORQUE_COAST');
        e.deadCoast = ini.getFloat('EDL', 'DEAD_ZONE_COAST'); e.deadPower = ini.getFloat('EDL', 'DEAD_ZONE_POWER');
        e.gainPower = 1 / (ini.getFloat('EDL', 'MAX_SPIN_POWER') - e.deadPower); e.gainCoast = 1 / (ini.getFloat('EDL', 'MAX_SPIN_COAST') - e.deadCoast);
      }
    }
  }
  stepEdl(car) {
    const e = this.edl;
    if (!e.active || !e.present) return;
    const wl = Math.abs(car.tyres[e.left].status.angularVelocity), wr = Math.abs(car.tyres[e.right].status.angularVelocity);
    if (!nz(wl) || !nz(wr)) return;
    const [torque, dead, gain] = 0 >= car.accG[2] ? [e.torqueCoast, e.deadCoast, e.gainCoast] : [e.torquePower, e.deadPower, e.gainPower];
    const [excess, fast] = wl > wr ? [(wl / wr - 1) - dead, e.left] : [(wr / wl - 1) - dead, e.right];
    const level = sat(gain * Math.max(0, excess));
    car.tyres[fast].inputs.brakeTorque += level * torque;
  }
  step(car, dt) {
    // ABS
    const a = this.abs;
    a.acc += dt;
    if (a.acc >= a.frequency) {
      a.acc = 0;
      a.active = a.active && a.present;
      if (a.active && car.speed * 3.6 > 20) {
        const lock = i => -a.limit > car.tyres[i].status.slipRatio;
        if (a.channels === 4) car.tyres.forEach((t, i) => { t.absOverride = lock(i) ? 0 : 1; });
        else {
          const front = lock(0) || lock(1) ? 0 : 1, rear = lock(2) || lock(3) ? 0 : 1;
          if (a.channels === 2) car.tyres.forEach((t, i) => { t.absOverride = i < 2 ? front : rear; });
          else { const both = Math.min(front, rear); car.tyres.forEach(t => { t.absOverride = both; }); }
        }
      } else car.tyres.forEach(t => { t.absOverride = 1; });
    }
    // traction control
    const t = this.tc, e = car.drivetrain && car.drivetrain.engine;
    if (!t.present) { t.inAction = false; t.active = false; }
    else if (!(v3.len(car.core.velocity(car.body)) >= t.minSpeed)) { t.inAction = false; t.acc = 0; }
    else {
      t.acc += dt;
      if (!(t.acc >= t.frequency)) { if (t.inAction && e) e.electronicOverride = 0; }
      else {
        t.acc = 0;
        if (t.active && !(1 >= car.speed)) {
          let largest = 0;
          for (const ty of car.tyres) if (ty.driven && ty.status.slipRatio > largest) largest = ty.status.slipRatio;
          if (largest > t.limit && e) e.electronicOverride = 0;
        }
        t.inAction = e ? nz(e.electronicOverride - 1) : false;
      }
    }
    // pit limiter
    if (car.tyres.some(ty => ty.surface && ty.surface.isPitlane) && car.speed * 3.6 > 80 && e) e.electronicOverride = 0;
  }
  stepStability(car) {
    const s = this.stability;
    if (!(s.gain > 0)) return;
    const sa = i => car.tyres[i].status.slipAngleRad;
    const diff = (Math.abs(sa(3) + sa(2)) - Math.abs(sa(1) + sa(0))) * 0.5;
    if (!(diff > 0 && car.speed * 3.6 > 5)) return;
    const torque = -(car.core.mass(car.body) * diff * s.gain * s.maxGain * Math.sign(sa(2)));
    car.core.addLocalTorque(car.body, [0, torque, 0]);
  }
}
