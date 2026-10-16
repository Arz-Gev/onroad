// Steering input filter ported from BeamNG.drive 0.36 (lua/vehicle/input.lua, bCDDL 1.1).
//
// Turns the raw key or stick value (-1..1, + = right) into a steering command (-1..1) the way BeamNG does:
// - keyboard: the wheel turns in slower the harder the car is already cornering (less understeer) and
//   comes back faster when you steer out of the turn (catches oversteer);
// - gamepad: a rate that follows how far and how fast the stick moved;
// then the four BeamNG assistants: slower steering and a smaller steering range at speed (both off while
// sliding), oversteer reduction (adds countersteer towards where the car is going) and understeer
// reduction (pulls the request back to the tightest turn the tyres can make, on hard surfaces only).

import { SURFACES } from './tire.js';

const sign = x => (x > 0 ? 1 : x < 0 ? -1 : 0);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const G = 9.81;

// BeamNG's temporalSmoothing: moves towards the sample at a fixed rate per second.
class Smoother {
  constructor(v = 0) { this.state = v; }
  getWithRate(sample, dt, rate) {
    const dif = sample - this.state;
    this.state += dif * Math.min(rate * dt / (Math.abs(dif) + 1e-300), 1);
    return this.state;
  }
  set(v) { this.state = v; }
  value() { return this.state; }
}

// BeamNG's getRate: smooth harder the further the smoother is from where it should be.
function getRate(cur, want, a1, b1, c1, a2, b2, c2) {
  const d = cur - want, ad = Math.abs(d);
  if (a2 !== undefined && d > 0) return a2 + b2 * ad + (c2 * ad) ** 2;
  return a1 + b1 * ad + (c1 * ad) ** 2;
}

// Surfaces where understeer reduction kicks in; on loose ones digging the fronts in is how you get grip.
const HARD = new Set(['asphalt', 'kerb', 'concrete', 'rock', 'wood'].map(k => SURFACES[k]));

// The assist settings as the tuning setup stores them (speeds in m/s). BeamNG keeps these in its options
// menu; these are our stock values. The keyboard oversteer help is half BeamNG's 0.7 (countersteering
// snapped the car the other way). A strength of 0 turns an assistant off; for slowdown and limit,
// a multiplier of 1 does.
export const ASSIST_STOCK = {
  kbdIn: 2.2, kbdOut: 1.6, kbdUndersteer: 0.7, kbdOversteer: 0.35,
  stab: 0.8, stabSpeed: 10,
  under: 0.8,
  slow: 0.6, slowStart: 15, slowEnd: 45,
  limit: 1, limitStart: 10, limitEnd: 40,
  // Slip-angle steering (Vehicle.slipSteer): share of the tyres' peak slip a full key asks for, how much a
  // released key follows the front axle's path, how far (s) the wheels lead its swing, and the speed (m/s)
  // from which it fully takes over.
  slipGain: 1, slipFollow: 1, slipDamp: 0.08, slipSpeed: 3,
};

// Setup values → the filter's options.
export const assistOptions = a => ({
  kbdInRate: a.kbdIn, kbdOutRate: a.kbdOut, kbdUndersteerMult: a.kbdUndersteer, kbdOversteerMult: a.kbdOversteer,
  stabilization: a.stab > 0, stabilizationEndSpeed: a.stabSpeed, stabilizationMultiplier: a.stab,
  understeerReduction: a.under > 0, understeerReductionMultiplier: a.under,
  slowdown: a.slow < 1, slowdownStartSpeed: a.slowStart, slowdownEndSpeed: Math.max(a.slowEnd, a.slowStart + 1), slowdownMultiplier: a.slow,
  limit: a.limit < 1, limitStartSpeed: a.limitStart, limitEndSpeed: Math.max(a.limitEnd, a.limitStart + 1), limitMultiplier: a.limit,
  slipGain: a.slipGain, slipFollow: a.slipFollow, slipDamp: a.slipDamp, slipSpeed: a.slipSpeed,
});

export const STEER_FILTER_DEFAULTS = assistOptions(ASSIST_STOCK);

export class SteerFilter {
  constructor(params, opts = {}) {
    this.o = { ...STEER_FILTER_DEFAULTS, ...opts };
    // Rates are per steering-wheel lock, as in BeamNG (450° lock = 1).
    const lockDeg = (params.steer.ratio * params.steer.maxAngle * 180) / Math.PI;
    this.rateMult = 450 / lockDeg;
    this.kbdOutRateMult = Math.min(this.o.kbdOutRate * this.rateMult, 2.68);
    this.kbdInRateMult = Math.min(this.o.kbdInRate * this.rateMult, 3.68);
    const line = (start, end, mult) => { const m = (mult - 1) / (1e-30 + end - start); return [m, 1 - m * start]; };
    [this.slowM, this.slowB] = line(this.o.slowdownStartSpeed, this.o.slowdownEndSpeed, this.o.slowdownMultiplier);
    [this.limM, this.limB] = line(this.o.limitStartSpeed, this.o.limitEndSpeed, this.o.limitMultiplier);
    this.reset();
  }

  reset() {
    this.kbd = new Smoother(); this.pad = new Smoother();
    this.kbdG = new Smoother(); this.padS = new Smoother();
    this.smU1 = new Smoother(); this.smU2 = new Smoother(); this.smO = new Smoother();
    this.gx = 0;
    this.inMult = 1; this.outMult = 1;
    this.assistance = 0; this.countersteer = 0;
  }

  rateKbd(v, dt, curx) {
    const gx = this.gx, absgx = Math.abs(gx);
    let gs = this.kbdG.getWithRate(0, dt, 3);
    if (absgx > gs) { gs = absgx; this.kbdG.set(gs); }
    if (v === 0) return this.inMult * this.kbdInRateMult;
    if (sign(curx) === -sign(gx) && sign(v) === -sign(gx)) {
      this.kbdG.set(0);
      return this.outMult * (this.kbdOutRateMult - this.kbdOutRateMult * this.o.kbdUndersteerMult * Math.min(absgx, G) / G);
    }
    if (sign(v) === sign(gx)) return this.inMult * (this.kbdOutRateMult + this.kbdOutRateMult * this.o.kbdOversteerMult * Math.min(gs, G) / G);
    return this.outMult * this.kbdOutRateMult;
  }

  ratePad(v, dt, curx) {
    let ps = this.padS.getWithRate(0, dt, 0.2);
    const diff = v - curx, absdiff = Math.abs(diff) * 0.9;
    if (absdiff > ps) { ps = absdiff; this.padS.set(ps); }
    const base = Math.min(absdiff * 1.7, 3) + ps + 0.35;
    if (diff * sign(curx) < 0) return this.inMult * Math.min(base * 2, 5) * this.rateMult;
    return this.outMult * base * this.rateMult;
  }

  // car: { vel, right, fwd, accel (world, m/s²), wheelSpeed, mu, steerAngle, wheelbase, fronts: [{ Fn, surf }] }
  update(input, pad, dt, car) {
    // gx as BeamNG's sensor reads it: opposite in sign to the steering in a steady turn. Lightly smoothed,
    // because our acceleration comes from differencing velocities at 240 Hz.
    const gxRaw = -car.accel.dot(car.right);
    this.gx += (gxRaw - this.gx) * Math.min(1, dt / 0.03);

    const v = clamp(input, -1, 1);
    let st;
    if (pad) st = clamp(this.pad.getWithRate(v, dt, this.ratePad(v, dt, this.pad.value())), -1, 1);
    else st = clamp(this.kbd.getWithRate(v, dt, this.rateKbd(v, dt, this.kbd.value())), -1, 1);
    // Keep the two filters level so switching devices mid-drive doesn't jump.
    if (pad) this.kbd.set(st); else this.pad.set(st);
    const out = this.stabilize(st, dt, car);
    // A bad number would stick in the smoothers for good; start over instead of passing it on.
    if (!Number.isFinite(out) || !Number.isFinite(this.gx)) { this.reset(); return 0; }
    return out;
  }

  stabilize(st, dt, car) {
    const o = this.o;
    this.inMult = 1; this.outMult = 1;
    const vel = car.vel, velSq = vel.lengthSq(), velLen = Math.sqrt(velSq);
    const wheelSpeed = car.wheelSpeed;
    const lowSpeedCoef = Math.min(Math.abs(wheelSpeed), velSq, 10) * 0.1;
    const speedThreshold = 13;
    const side = vel.dot(car.right) / (velLen + 1e-10);
    const oversteer = lowSpeedCoef * Math.min(velLen / speedThreshold, 1) * side;

    if (o.slowdown) {
      const speedMult = Math.min(1, Math.max(o.slowdownMultiplier, this.slowM * Math.min(velLen, wheelSpeed) + this.slowB));
      this.outMult = Math.max(clamp(5 * Math.abs(oversteer), 0, 1), speedMult);
      this.inMult = this.outMult;
    }
    if (o.limit) {
      const speedMult = Math.min(1, Math.max(o.limitMultiplier, this.limM * Math.min(velLen, wheelSpeed) + this.limB));
      st *= Math.max(clamp(5 * Math.abs(oversteer), 0, 1), speedMult);
    }
    this.countersteer = 0;
    if (o.stabilization) {
      const om = lowSpeedCoef * Math.min(velLen / (o.stabilizationEndSpeed + 1e-10), 1) * side;
      const cs = om * o.stabilizationMultiplier;
      this.countersteer = sign(cs) * Math.max(0, 1 - st * st) * Math.min(1, Math.abs(cs));
      st += this.countersteer;
    }
    this.assistance = 0;
    if (o.understeerReduction) {
      // Tightest turn the tyres allow at this speed, faded out at low speed and on loose ground.
      let fz = 0, hard = 0;
      for (const w of car.fronts) { const n = Math.max(0, w.Fn); fz += n; if (HARD.has(w.surf)) hard += n; }
      const best = clamp(velSq / (car.mu * G), 0, 100000);
      const target = best * clamp(0.072 * velLen, 0, 1) * (hard / (fz + 1e-10)) * o.understeerReductionMultiplier;
      const a = Math.abs(car.steerAngle);
      const requested = a > 1e-6 ? clamp(car.wheelbase / Math.tan(a), 0, 100000) : 100000;
      const ratio = requested === 0 ? 0 : target / requested;
      // BeamNG's empirical radius→input conversion ("from hundreds of tests").
      const conversion = clamp(-3 + 2600 / (Math.max(13, 3.6 * velLen) - 13), 5, 40) * 0.01;
      const factor = ratio * conversion;
      const mult = this.smU1.getWithRate(factor, dt, getRate(this.smU1.value(), factor, 1.5, 1.5, 0.75));
      const multSm = clamp(this.smU2.getWithRate(mult, dt, getRate(this.smU2.value(), mult, 0, 0.25, 2.5, 0.15, 0.5, 5)), 0, 1);
      // No understeer help while sliding, spinning or reversing.
      const margin = sign(st) === sign(oversteer) ? -1 : 0.15;
      // Clamped: Rapier's single-precision rotation can make fwd a hair longer than 1, and driving straight
      // would then put sqrt(1 - fwdDot) below zero (NaN).
      const fwdDot = Math.min(1, vel.dot(car.fwd) / (velLen + 1e-10));
      const overMult = 1 - clamp(5 * (Math.abs(lowSpeedCoef * Math.min(velLen / speedThreshold, 1) * Math.sqrt(1 - Math.max(0, fwdDot))) - margin), 0, 1);
      const overSm = this.smO.getWithRate(overMult, dt, 3);
      this.assistance = multSm * overSm;
      st *= 1 - this.assistance;
      this.inMult *= 1 + 3.5 * this.assistance;
    }
    return clamp(st, -1, 1);
  }
}
