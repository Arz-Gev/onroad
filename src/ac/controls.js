// How the player's keys and pad become the AC car's steering, gas and brake. Three choices (the "Steering help"
// setting):
//
// - 'ac': Assetto Corsa's own input classes, ported from rustyAC (input/keyboard.rs, input/pad.rs, aids.rs).
//   Keyboard (KeyboardCarControl): the steering moves towards a lock that shrinks with speed, slower at speed and
//   faster back to the centre; the gas ramps up and backs off (not below 65 %) while a driven tyre is past its
//   grip; the brake goes straight to the pedal the tyres can take (RaceEngineer::getOptimalBrake).
//   Pad (JoypadCarControl, with AC's stock gamepad preset): dead zone, gamma, a steering speed limit and filter
//   per physics step, and less lock at speed (speed sensitivity).
// - 'aga': Advanced Gamepad Assist 1.5.5 by adam10603 (MIT, github.com/adam10603/AC-Advanced-Gamepad-Assist), a
//   Custom Shaders Patch "Gamepad FX" script: the steering aims at the front tyres' best slip angle, the car
//   countersteers by itself in a slide (self-steer, like caster), and on keys the gas and brake back off at the
//   grip limit. Its clutch, shifting and trigger rumble extras are left out (the game has its own).
// - 'off': the game's raw keys and stick, straight to the wheels.
//
// AC body frame throughout: +x left, +y up, +z forward; positive steer = right.

import { IniReader } from './ini.js';
import { AcCar, flatGround } from './car.js';

const clamp = (x, a, b) => (x < a ? a : x > b ? b : x);
const sat = x => clamp(x, 0, 1);
const sgn = x => (x > 0 ? 1 : x < 0 ? -1 : 0);
const lerp = (a, b, t) => a + (b - a) * t;
const lerpInvSat = (x, a, b) => sat((x - a) / (b - a));
const DEG = 180 / Math.PI;

// --- Assetto Corsa --------------------------------------------------------------------------------------------

// KeyboardCarControl with the stock controls.ini [KEYBOARD] values
export class AcKeyboard {
  constructor({ steerSpeed = 1.75, opposite = 2.5, reset = 1.8 } = {}) {
    this.steerSpeed = steerSpeed; this.oppositeFactor = opposite; this.resetFactor = reset;
    this.gasPedalSpeed = 4;
    this.intSteer = 0; this.intGas = 0;
    this.steerDecelerator = 1; this.speedDecFactor = 1; this.turnDecFactor = 0;
  }

  reset() { this.intSteer = 0; this.intGas = 0; }

  // getKeyboardSteering: the target is a lock of pi / (v / 2 + 1) (the AI-line term is 0, as in rustyAC)
  steering(left, right, dt, speed) {
    const m = Math.PI / (speed * 0.5 + 1);
    return this.movement(left ? -m : right ? m : 0, dt, speed);
  }

  // steeringMovement
  movement(target, dt, speed) {
    const cur = this.intSteer, diff = target - cur, stepMax = dt * this.steerSpeed;
    if (!(Math.abs(diff) >= stepMax)) this.intSteer = target;
    else if (target === 0) {
      const n = sgn(diff) * stepMax * this.resetFactor + cur;
      this.intSteer = sgn(cur) !== sgn(n) ? 0 : n;
    } else if (sgn(cur) === sgn(target) || cur === 0) {
      const a = 2 / (Math.exp(speed * -0.05) + 1) - 1;
      this.speedDecFactor = clamp(1 - Math.max(a, 0), 0.1, 1);
      if (speed * 3.6 > 100) this.steerDecelerator = this.speedDecFactor;
      else {
        this.turnDecFactor = clamp(Math.abs(target) * 0.63661975, 0.1, 1);
        this.steerDecelerator = (this.turnDecFactor + this.speedDecFactor) * 0.5;
      }
      this.intSteer = sgn(target - cur) * stepMax * this.steerDecelerator + cur;
    } else this.intSteer = sgn(diff) * stepMax * this.oppositeFactor + cur;
    this.intSteer = clamp(this.intSteer, -1, 1);
    return this.intSteer;
  }

  // computeGasCoefficient: the larger nd slip of the driven pair; past 0.99 (2 above 100 km/h) the pedal backs off
  gas(held, car, dt) {
    if (!held) { this.intGas = 0; return 0; }
    const d = car.drivetrain, step = dt * this.gasPedalSpeed;
    const l = car.tyres[d.tyreLeft].status.ndSlip, r = car.tyres[d.tyreRight].status.ndSlip;
    const slip = !(l >= r) ? r : l;
    const slipping = car.speed * 3.6 < 100 ? slip > 0.99 : slip > 2;
    this.intGas = slipping ? clamp(this.intGas - step, 0.65, 1) : sat(this.intGas + step);
    return sat(this.intGas);
  }
}

// RaceEngineer::getOptimalBrake: the pedal that just reaches the grip of the axle that locks first
export function optimalBrake(car) {
  const t = car.tyres, grip = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) grip[i] = t[i].correctedD(tyreDx(t[i], t[i].status.load), false);
  const load = i => t[i].status.load;
  const front = (grip[1] * load(1) + grip[0] * load(0)) * 0.5 * t[0].status.loadedRadius;
  const rear = (grip[3] * load(3) + grip[2] * load(2)) * 0.5 * t[2].status.loadedRadius;
  const power = car.brakes.power, bias = car.brakes.frontBias;
  const f = front / (power * bias), r = rear / (power * (1 - bias));
  return f >= r ? r : f;
}

// tyre_get_dx (aids.rs)
function tyreDx(tyre, load) {
  const m = tyre.modelData, b = sat(tyre.status.blister * 0.01), k = b * 0.2 + 1;
  if (m.dxLoadCurve.count > 0) return m.dxLoadCurve.getCubicSplineValue(load) / k;
  if (m.lsExpX !== 0) return (load !== 0 ? Math.pow(load, m.lsExpX) * m.lsMultX / load : 0) / k;
  return (load * 0.0005 * m.dx1 + m.dx0) / k;
}

// getAxisValue: dead zone, then the gamma curve
export function padAxis(value, gamma, deadZone) {
  if (!(Math.abs(value) >= deadZone)) return 0;
  const v = (value > 0 ? value - deadZone : value + deadZone) / (1 - deadZone);
  return Math.pow(Math.abs(v), gamma) * sgn(v);
}

// JoypadCarControl's steering, with AC's stock gamepad preset (STEER_SPEED 0.2, GAMMA 2, FILTER 0.7, DEADZONE 0.05,
// SPEED_SENSITIVITY 0.5). Called once per physics step: its rates are per call.
export class AcPad {
  constructor({ steerSpeed = 0.2, gamma = 2, filter = 0.7, deadZone = 0.05, speedSensitivity = 0.5 } = {}) {
    this.steerSpeed = clamp(steerSpeed * 0.1, 0.01, 0.1);
    this.gamma = Math.max(1, gamma);
    this.filter = filter * 0.99;
    this.deadZone = deadZone; this.speedSensitivity = speedSensitivity;
  }

  // the speed-dependent limit (the current code, not USE_LEGACY_CODE): the full stick gives the car's lock at a
  // standstill and, at sensitivity 1, 20 degrees of steering wheel from 200 km/h on
  maxSteer(speed, lock) {
    const t = sat(speed * 3.6 * 0.005), s = this.speedSensitivity;
    const a = (90 - lock) * s + lock, b = (20 - a) * s + a;
    return ((b - lock) * t + lock) / lock;
  }

  steer(stick, prev, speed, lock) {
    const a = padAxis(stick, this.gamma, this.deadZone), m = this.maxSteer(speed, lock);
    const d = clamp(a * m - prev, -this.steerSpeed, this.steerSpeed);
    return clamp((1 - this.filter) * d + prev, -1, 1);
  }
}

// --- Advanced Gamepad Assist ----------------------------------------------------------------------------------

// AGALib.SmoothTowards
class SmoothTowards {
  constructor(rate, linearity, min, max, start = 0) { this.rate = rate; this.linearity = linearity; this.range = max - min; this.state = start; this.start = start; }
  get(val, dt, rate = this.rate) {
    const lsq = this.linearity * this.linearity;
    const r = rate / (1 - 1 / (lsq + 1 / 0.75)) * 0.5;
    const diff = Math.abs((val - this.state) / this.range);
    const adj = (diff * (1 - lsq) + lsq) * r;
    this.state += sgn(val - this.state) * Math.min(diff, dt * adj) * this.range;
    return this.state;
  }
  getWithRateMult(val, dt, mult) { return this.get(val, dt, this.rate * mult); }
  reset() { this.state = this.start; }
}

// AGALib.clampEased: a clamp with parabolic easing into the limits
function clampEased(val, min, max, window) {
  const w = window * (max - min), h = w * 0.5;
  if (val < min - h) return min;
  if (val > max + h) return max;
  if (val < min + h) { const t = (val - (min - h)) / w; return min + t * t * h; }
  if (val > max - h) { const t = (val - (max - h) - w) / w; return max - t * t * h; }
  return val;
}
const signedPow = (x, y) => sgn(x) * Math.pow(Math.abs(x), y);
const inverseLerp = (a, b, v) => (Math.abs(b - a) > 1e-60 ? (v - a) / (b - a) : 0);
const zeroGuard = v => 1 / Math.max(Math.min(1 / v, 1e15), -1e15);
const guard = (v, alt = 0) => (Number.isFinite(v) ? v : alt);
const wavg = (a, b, wa, wb) => (wa + wb !== 0 ? (a * wa + b * wb) / (wa + wb) : 0);
const pointVel = (p, w, v) => [w[1] * p[2] - w[2] * p[1] + v[0], w[2] * p[0] - w[0] * p[2] + v[1], w[0] * p[1] - w[1] * p[0] + v[2]];

// the front wheels' average steering angle in degrees (+ = right), from the hubs' forward axes in the body frame
function steeringAngleDeg(car) {
  const c = car.core, l = c.worldToLocalNormal(car.body, c.matrix(car.suspensions[0].hub).slice(8, 11));
  const r = c.worldToLocalNormal(car.body, c.matrix(car.suspensions[1].hub).slice(8, 11));
  return -guard(Math.atan2(l[0] + r[0], l[2] + r[2]) * DEG);
}

// performCalibration, run on a copy of the car standing on a flat floor with the brakes on: sweeps the steering to
// full lock and back and fits the steering curve. Returns { lock (deg), exponent, wheels (body-frame hub positions) }.
export function calibrateSteering(files) {
  const car = new AcCar(files, { ground: flatGround() });
  car.forceRotation([0, 0, -1]); car.forcePosition([0, 0, 0]);
  car.sessionStart();
  const ctl = car.controls, dt = 0.003;
  ctl.brake = 1; ctl.handBrake = 1; ctl.gas = 0; ctl.clutch = 0;
  let t = 0;
  const step = () => { car.step(dt, 60000 + t * 1000, null); t += dt; };
  for (let i = 0; i < 200; i++) step();
  const wheels = [0, 1, 2, 3].map(i => car.core.worldToLocal(car.body, car.suspensions[i].hub.pos));
  const smoother = new SmoothTowards(1.2, 1, -1, 1, 0), samples = [];
  let lockHold = 0;
  for (let i = 0; i < 2000 && lockHold < 0.2; i++) {
    const angle = Math.abs(steeringAngleDeg(car));
    if (!samples.length || Math.abs(angle - samples[samples.length - 1][1]) > 1e-6) samples.push([smoother.state, angle]);
    ctl.steer = clamp(smoother.get(1, dt), -1, 1);
    if (1 - ctl.steer < 1e-10) lockHold += dt;
    step();
  }
  let lock = Math.abs(steeringAngleDeg(car));
  const fallbackLock = Math.abs(car.steerLock / car.steerRatio);
  if (!(lock > 3 && lock < 90)) lock = fallbackLock;
  // evaluateSteeringCurve
  const lo = clamp(3.5 / lock, 0.1, 0.25), hi = clamp(10 / lock, 0.45, 0.6);
  let acc = 0, n = 0;
  for (const [x, y] of samples) {
    if (x < lo || x > hi) continue;
    const vy = clamp(y / lock, 0.001, 0.999), e = Math.log10(1 - vy) / Math.log10(1 - x);
    if (Number.isFinite(e)) { acc += e; n++; }
  }
  let exponent = n ? acc / n : 0.95;
  if (!(exponent > 0.5 && exponent < 2)) exponent = 0.95;
  return { lock, exponent, wheels };
}

// CarPerformanceData's target front slip angle: an estimate from tyres.ini (VERSION 10), then learned while
// cornering near the limit
class TargetSlip {
  constructor(files) {
    const ini = new IniReader(files, 'tyres.ini');
    const idx = ini.getInt('COMPOUND_DEFAULT', 'INDEX'), s = idx === 0 ? 'FRONT' : `FRONT_${idx}`;
    const f = (k, d) => (ini.hasKey(s, k) ? ini.getFloat(s, k) : d);
    this.v10 = ini.getInt('HEADER', 'VERSION') === 10 && f('FRICTION_LIMIT_ANGLE', 0) !== 0;
    this.t = { limit: f('FRICTION_LIMIT_ANGLE', 6.3), camberGain: f('CAMBER_GAIN', 0.213), pIdeal: f('PRESSURE_IDEAL', 23),
      pFlex: f('PRESSURE_FLEX_GAIN', 0.3), pD: f('PRESSURE_D_GAIN', 0.0052), flex: f('FLEX_GAIN', 0.029), fz0: f('FZ0', 3451) };
    this.target = new SmoothTowards(0.12, 0.05, 0, 15, 0);
    this.loadAdd = new SmoothTowards(1.4, 0.05, 0, 2, 0);
    this.fastLearningTime = 0;
  }

  loadAdditive(d) {
    const t = this.t, [w0, w1] = d.w;
    const avg = (w0.load + w1.load) * 0.5, min = avg * (Math.sqrt(lerpInvSat(d.hVel, 0, 15)) * 0.6 + 1);
    const flexMult = 1 + 12 * t.flex, ref = 2 * t.fz0;
    const a0 = Math.sqrt(Math.max(min, w0.load) / ref) * flexMult, a1 = Math.sqrt(Math.max(min, w1.load) / ref) * flexMult;
    return guard(wavg(a0, a1, d.fw[0], d.fw[1]));
  }

  initialEstimate(d) {
    const t = this.t, [w0, w1] = d.w;
    const cam0 = t.camberGain * Math.sin(w0.camber / DEG), cam1 = t.camberGain * Math.sin(w1.camber / DEG);
    const pfm = t.pFlex / 3.7;
    const pd0 = t.pIdeal - w0.pressure, pd1 = t.pIdeal - w1.pressure;
    const pg0 = pd0 * pfm - Math.abs(pd0 * (w0.pressure < t.pIdeal ? 1.5 : 1)) * t.pD * 1.7 - 0.005 * Math.abs(w0.core - w0.optimum);
    const pg1 = pd1 * pfm - Math.abs(pd1 * (w1.pressure < t.pIdeal ? 1.5 : 1)) * t.pD * 1.7 - 0.005 * Math.abs(w1.core - w1.optimum);
    const lm = 1 / (2 * t.fz0) * (5.5 * (t.flex - 0.03));
    const min = (w0.load + w1.load) * 0.5 * 1.6;
    const l0 = Math.max(min, w0.load) * lm + pg0, l1 = Math.max(min, w1.load) * lm + pg1;
    return clamp(guard(wavg(t.limit + cam0 + l0, t.limit + cam1 + l1, d.fw[0], d.fw[1])), 6, 11);
  }

  update(d, initialSteering, dt) {
    const [w0, w1] = d.w, oneFrontGrounded = w0.load > 0 || w1.load > 0;
    const learning = d.frontNd > 0.25 && d.frontNd < 1.5 && d.fwdVel > 8 && d.rearNd > 0.1 && d.rearNd < 1.5 &&
      w0.road && w1.road && Math.abs(initialSteering) > 0.6 && Math.abs(w0.slipRatio) < 0.1 && Math.abs(w1.slipRatio) < 0.1 && oneFrontGrounded;
    let raw = 0;
    if (this.v10 && oneFrontGrounded) { raw = this.loadAdditive(d); this.loadAdd.get(raw, dt); }
    if (this.target.state === 0) {
      if (this.v10) { this.fastLearningTime = 999; this.target.state = this.initialEstimate(d) - raw; }
      else { this.fastLearningTime = 0; this.target.state = 7; }
    }
    if (learning) {
      let mult = 1;
      if (this.fastLearningTime < 1.5) { this.fastLearningTime += dt; mult = 4; }
      const ideal = Math.abs(d.frontSlipDeg) / d.frontNd - this.loadAdd.state;
      this.target.getWithRateMult(this.v10 ? clamp(ideal, 0.5, 14) : clamp(ideal, 4, 14), dt, mult);
    }
  }

  get() { return this.target.state + this.loadAdd.state; }
}

// script.update of assist.lua, stepped at a game-frame rate. `strength` is the "simplified settings" slider
// (filterSetting, 0..1): higher = more self-steer and a tighter steering limit, lower = looser.
export class AgaAssist {
  constructor(files) {
    this.files = files;
    this.cal = null;
    this.strength = 0.5;
    this.steeringRate = 0.5;
    this.steering = new SmoothTowards(7, 0.13, -1, 1, 0);
    this.absSteering = new SmoothTowards(7, 0.13, -1, 1, 0);
    this.kbThrottle = new SmoothTowards(12, 1, 0, 1, 0);
    this.kbBrake = new SmoothTowards(12, 1, 0, 1, 0);
    this.kbSteer = new SmoothTowards(7, 1, -1, 1, 0);
    this.selfSteer = new SmoothTowards(7, 0.13, -1, 1, 0);
    this.limit = new SmoothTowards(11, 0.01, 0, 32, 32);
    this.grounded = new SmoothTowards(4, 1, 0, 1, 1);
    this.counter = new SmoothTowards(12, 1, 0, 1, 0);
    this.brakeTarget = 1; this.throttleTarget = 1; this.prevBrakeNd = 0; this.prevThrottleNd = 0; this.brakeIn = 0;
    this.targetSlip = new TargetSlip(files);
    this.out = { steer: 0, gas: 0, brake: 0 };
    this.info = { selfSteer: 0, limitReduction: 0, rearAngle: 0 };
  }

  reset() {
    for (const s of [this.steering, this.absSteering, this.kbThrottle, this.kbBrake, this.kbSteer, this.selfSteer, this.limit, this.grounded, this.counter]) s.reset();
    this.brakeTarget = 1; this.throttleTarget = 1; this.prevBrakeNd = 0; this.prevThrottleNd = 0;
  }

  // updateConfig with useFilter on
  config() {
    const f = this.strength;
    this.rateIncreaseWithSpeed = (1 - f) * 0.2 - 0.1;
    this.selfSteerResponse = f * 0.5 + 0.12;
    this.dampingStrength = this.selfSteerResponse;
    this.maxSelfSteerAngle = 90;
    this.maxDynamicLimitReduction = 3 * f + 3.5;
    this.countersteerResponse = (1 - f) * 0.2 + 0.1;
    this.targetSlipMult = 0.95 - (f - 0.5) * 0.04;
  }

  // getVehicleData
  data(car, tc, abs) {
    if (!this.cal) this.cal = calibrateSteering(this.files);
    const c = car.core, b = car.body, cal = this.cal;
    const w = car.tyres.map(t => {
      const s = t.status;
      return { load: s.load, slipDeg: s.slipAngleRad * DEG, nd: s.ndSlip, slipRatio: s.slipRatio, road: t.surface?.name === 'ROAD',
        camber: s.camberRad * DEG, pressure: s.pressureDynamic, core: t.thermal.coreTemp, optimum: t.data.optimumTemp };
    });
    const fw = [zeroGuard(w[0].load), zeroGuard(w[1].load)], rw = [zeroGuard(w[2].load), zeroGuard(w[3].load)];
    const vel = c.worldToLocalNormal(b, c.velocity(b)), av = c.worldToLocalNormal(b, b.avel);
    const P = cal.wheels, mid = (i, j) => [(P[i][0] + P[j][0]) / 2, (P[i][1] + P[j][1]) / 2, (P[i][2] + P[j][2]) / 2];
    const fAxle = mid(0, 1), rAxle = mid(2, 3);
    const avgPos = [0, 1, 2].map(k => Math.round((P[0][k] + P[1][k] + P[2][k] + P[3][k]) * 250) / 1000);
    const midVel = pointVel(avgPos, av, vel), fVel = pointVel(fAxle, av, vel), rVel = pointVel(rAxle, av, vel);
    const wheelbase = Math.abs(fAxle[2] - rAxle[2]);
    return {
      w, fw, rw, av, wheelbaseFactor: wheelbase / 2.5,
      hVel: Math.hypot(midVel[0], midVel[2]), travelDir: guard(Math.atan2(midVel[0], midVel[2]) * DEG),
      frontSlipDeg: guard(wavg(w[0].slipDeg, w[1].slipDeg, fw[0], fw[1])), rearSlipDeg: guard(wavg(w[2].slipDeg, w[3].slipDeg, rw[0], rw[1])),
      frontNd: guard(wavg(w[0].nd, w[1].nd, fw[0], fw[1])), rearNd: guard(wavg(w[2].nd, w[3].nd, rw[0], rw[1])),
      totalNd: guard((w[0].nd * fw[0] + w[1].nd * fw[1] + w[2].nd * rw[0] + w[3].nd * rw[1]) / (fw[0] + fw[1] + rw[0] + rw[1])),
      fwdVel: Math.max(0, midVel[2]), lock: cal.lock, exponent: cal.exponent, fVel, rVel, fHVel: Math.hypot(fVel[0], fVel[2]),
      frontGrounded: 0, fwd: car.drivetrain.traction === 'FWD', tc, abs,
    };
  }

  rateMult(speed, lock) {
    const r = lock / Math.min(65 / (Math.max(speed, 8) - 7.3) + 3.5, lock);
    return Math.pow(r, this.rateIncreaseWithSpeed) * this.steeringRate;
  }

  // calcCorrectedSteering
  corrected(d, targetSlipDeg, initial, absInitial, fadeIn, dt) {
    const fAng = guard(Math.atan2(d.fVel[0], Math.abs(d.fVel[2])) * DEG);
    const rAng = guard(Math.atan2(d.rVel[0], Math.abs(d.rVel[2])) * DEG);
    const inputSign = sgn(initial);
    const midSpeedFade = lerpInvSat(d.hVel, 10 * d.wheelbaseFactor, 20 * d.wheelbaseFactor);

    const corrExp = 1 + (1 - Math.log10(10 * (this.selfSteerResponse * 0.9 + 0.1)));
    const corrBase = signedPow(clamp(-rAng / 72, -1, 1), corrExp) * 72 / d.lock;
    const cap = sat(this.maxSelfSteerAngle / d.lock);
    const strength = Math.max(1, d.frontGrounded) * fadeIn;
    const damping = d.av[1] * this.dampingStrength * 0.15 * (30 / d.lock);
    const capT = Math.min(1, 4 / (2 * cap));
    const rawSelf = clampEased(corrBase, -cap, cap, capT) + damping;
    const selfSteer = clamp(this.selfSteer.get(rawSelf, dt), -2, 2) * strength;

    const finalTarget = targetSlipDeg * this.targetSlipMult;
    const maxReduction = lerp(finalTarget * 0.4, finalTarget * 0.75, sat(this.maxDynamicLimitReduction / 10));
    const angleSub = lerp(maxReduction, maxReduction * 0.9, this.brakeIn);
    const clampedF = clampEased(inputSign * fAng, -d.lock - 15, angleSub, (angleSub * 0.4) / (d.lock + 15 + angleSub));
    if (d.hVel > 1e-15) { this.info.rearAngle = rAng; this.info.limitReduction = Math.max(clampedF, 0); }

    const counter = inputSign !== sgn(zeroGuard(d.rVel[0])) && Math.abs(initial) > 1e-6;
    const rawCounter = counter ? clampEased(inverseLerp(4.5, 10, Math.abs(rAng)), 0, 1, 0.6) : 0;
    const returnRate = lerp(0.8, 0.3, lerpInvSat(Math.abs(fAng - rAng), 2, 10));
    const counterInd = this.counter.getWithRateMult(rawCounter, dt, rawCounter < this.counter.state ? returnRate : 1) * midSpeedFade;

    const antiSelf = absInitial * -selfSteer;
    const targetInward = finalTarget - clampedF;
    const counterMult = lerp(lerpInvSat(-inputSign * rAng, 0, 30) / 3 + 2 / 3, 1, this.countersteerResponse);
    const targetCounter = finalTarget * (this.countersteerResponse * counterMult * 0.7 + 0.1) - inputSign * rAng;
    const targetAngle = lerp(clamp(targetInward, 0, d.lock), clamp(targetCounter, 0, d.lock), counterInd);
    const notForward = Math.pow(Math.sin(clamp(d.travelDir * 2 / 3 / DEG, -Math.PI / 2, Math.PI / 2)), 16);
    const limit = lerp(this.limit.get(targetAngle, dt) / d.lock, 1, notForward);
    this.info.selfSteer = strength * (1 - absInitial);
    return clamp(initial * limit + selfSteer + antiSelf, -1, 1);
  }

  // in: { left, right, gas, brake (keys, booleans), stick (-1..1, null without a pad), padGas, padBrake }
  update(car, inp, tc, abs, dt) {
    this.config();
    const d = this.data(car, tc, abs);
    d.frontGrounded = this.grounded.get(d.w[0].load === 0 && d.w[1].load === 0 ? 0 : 1, dt);
    const rate = this.rateMult(d.hVel, d.lock);

    // processInitialInput (keyboard mode 3: gas and brake help)
    const kbRaw = (inp.right ? 1 : 0) - (inp.left ? 1 : 0);
    const kbMult = Math.abs(this.kbSteer.state) - sgn(this.kbSteer.state) * kbRaw > 0 ? 1.5 : 1;
    const kbSteer = this.kbSteer.getWithRateMult(kbRaw, dt, rate * kbMult);
    const stick = inp.stick == null ? 0 : padAxis(inp.stick, 1.4, 0.12);
    const raw = clamp(stick + kbSteer, -1, 1);
    let centering = 1;
    if (rate > 0 && rate < 0.5) {
      const s = this.steering.state;
      if ((Math.abs(raw) < Math.abs(s) && sgn(raw) === sgn(s)) || sgn(raw) !== sgn(s)) centering = (rate * 0.5 + 0.25) / rate;
    }
    const initial = this.steering.getWithRateMult(raw, dt, rate * centering);
    const absInitial = this.absSteering.getWithRateMult(Math.abs(raw), dt, rate * centering);

    const kbGas = this.kbThrottle.get(inp.gas ? 1 : 0, dt), kbBrake = this.kbBrake.get(inp.brake ? 1 : 0, dt);
    const brakeNd = d.totalNd + 0.15;
    const slipSub = lerp(0.25, 0.35, sat(inverseLerp(40, 160, d.hVel * 3.6)));
    const throttleNd = (d.fwd ? d.frontNd : d.rearNd) - slipSub;
    let bt = 1, tt = 1;
    if (!abs) {
      const ext = clamp(brakeNd + dt * 5000 * (brakeNd - this.prevBrakeNd), 0, 2);
      this.brakeTarget = sat(Math.max(0.3, this.brakeTarget + dt * 10 * (ext < 1 ? 1 - ext : 3 - 3 * ext)));
      bt = this.brakeTarget; this.prevBrakeNd = brakeNd;
    }
    if (!tc) {
      const ext = clamp(throttleNd + dt * 5000 * (throttleNd - this.prevThrottleNd), 0, 2);
      const tMin = lerp(0.6, 0.8, sat(inverseLerp(40, 160, d.hVel * 3.6)));
      this.throttleTarget = sat(Math.max(tMin, this.throttleTarget + dt * 10 * (ext < 1 ? 1 - ext : 3 - 3 * ext)));
      tt = this.throttleTarget; this.prevThrottleNd = throttleNd;
    }
    const brake = sat((inp.padBrake || 0) + kbBrake * bt), gas = sat((inp.padGas || 0) + kbGas * tt);
    this.brakeIn = brake;

    this.targetSlip.update(d, initial, dt);
    const fadeIn = lerpInvSat(d.fHVel, 2 * d.wheelbaseFactor, 6 * d.wheelbaseFactor);
    const processed = this.corrected(d, this.targetSlip.get(), initial, absInitial, fadeIn, dt);
    const desired = lerp(initial, processed, fadeIn);
    // normalizedSteeringToInput
    const steer = clamp(sgn(desired) * (1 - Math.pow(1 - Math.abs(desired), 1 / d.exponent)), -1, 1);
    this.out.steer = guard(steer); this.out.gas = guard(gas); this.out.brake = guard(brake);
    return this.out;
  }
}
