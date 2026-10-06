// Vehicle tuning: a player "setup" (engine, gearing, tyres, suspension, brakes, mass, steering, body
// colliders) and how it maps onto the car's physics parameters (src/cars/<id>.js makeParams).
//
// applySetup(P, setup) writes the setup into an existing params object in place, so the Vehicle,
// Drivetrain and view that hold P see the new values on their next step. The stock setup reproduces the
// car's makeParams() exactly (tools/simtest.mjs baselines are unchanged).
//
// What changes how (Vehicle.retune):
// - read every step, so live: engine curve / limiter / shift points, gear ratios, final drive, transfer,
//   springs, dampers, anti-roll bars, travel, brakes, steering, tyre grip and pressure;
// - geometry (tyre radius, lift = axle droop height) eases to the new value at 0.15 m/s: the body rises
//   on its springs like an air suspension instead of the wheels being teleported into the ground;
// - mass, centre of mass and inertia: body.setAdditionalMassProperties, live;
// - colliders (chassis boxes, wheel side cylinders): rebuilt on the same rigid body in a few microseconds.
// Nothing needs a new rigid body, so the car keeps its position and speed through every change.
//
// Every car has its own profile, built from its module in src/cars/ (`tuning`: engines, tyre sizes, slider
// ranges, panel texts; `capabilities`: transfer case, rear diff types, live switches). carTuning(id) gives
// the profile's STOCK / RANGES / ENGINES / sanitize; applySetup, analyze, geometry, staticRide and rideRaise
// pick the profile from P.model (the car id the car's makeParams() writes).

import { tireRadialStiffness } from './tire.js';
import { engineFriction } from './engineCurve.js';
import { findCar } from '../cars/index.js';

export { engineFriction };

const G = 9.81;
const DEG = Math.PI / 180;
const clamp = (x, a, b) => (x < a ? a : x > b ? b : x);
const lerpTable = (t, x) => {
  if (x <= t[0][0]) return t[0][1];
  for (let i = 1; i < t.length; i++) if (x <= t[i][0]) { const a = t[i - 1], b = t[i]; return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]); }
  return t[t.length - 1][1];
};

// ---------------------------------------------------------------- car profiles
// A car's `tuning` (src/cars/<id>.js): engines / engineOrder / stockEngine · named(base params, engine,
// preset) · tyre: stock size (overall diameter, inches) and width (widthUnit), the chip options,
// wheelMass(inches, width) · ranges: slider ranges that differ from BASE_RANGES · cargoPos / roofPos: where
// the load sits · mu / traction / crr / surface: the test surface for the brake and 0-100 estimates ·
// defaultSetup: an exported setup that replaces the stock one · migrate(setup, suspension): fixes for old
// saved setups · ui: what the panel shows and its texts.
const BASE_RANGES = {
  'engine.torque': [0.5, 2, 0.05], 'engine.revs': [-1000, 1500, 50],
  'gearbox.ratio': [0.5, 6, 0.01], 'gearbox.final': [2.5, 6, 0.01], 'gearbox.high': [0.9, 1.8, 0.001], 'gearbox.low': [1.8, 5.5, 0.01],
  'tyres.size': [31, 37, 1], 'tyres.width': [9.5, 13.5, 0.5], 'tyres.press': [6, 38, 1], 'tyres.grip': [0.7, 1.3, 0.01],
  'suspension.lift': [0, 0.15, 0.005], 'suspension.k': [15000, 100000, 500], 'suspension.bump': [1000, 10000, 100],
  'suspension.rebound': [1500, 16000, 100], 'suspension.arb': [0, 30000, 500], 'suspension.travel': [0.14, 0.36, 0.005],
  'brakes.force': [0.4, 1.8, 0.05], 'brakes.bias': [0.4, 0.85, 0.01], 'brakes.handbrake': [0.3, 2, 0.05],
  'mass.cargo': [0, 900, 10], 'mass.roof': [0, 300, 5], 'mass.comY': [-0.15, 0.4, 0.01],
  'steering.lock': [25, 48, 0.5], 'steering.ratio': [11, 24, 0.5],
  'diff.preload': [0, 300, 5], 'diff.power': [0, 1, 0.01], 'diff.coast': [0, 1, 0.01],
  'box.c': [-3.5, 3.5, 0.005], 'box.h': [0.02, 2.2, 0.005], 'box.r': [0, 0.2, 0.005],
};
const DIFF_TYPES = ['welded', 'lsd', 'open'];

// a car without a transfer case drives through a 1:1 "high range"
const NO_TRANSFER = { high: 1, low: 1 };
export const transferOf = P => P.transfer || NO_TRANSFER;
// the panel's live driveline switches a car has (its capabilities)
const liveSwitches = cap => [cap.transfer && 'range', cap.centreDiff && 'centreLock', ...(cap.lockers === 'axles' ? ['rearLock', 'frontLock'] : []), 'traction', 'abs', cap.selectableRwd && 'rwd'].filter(Boolean);

function makeProfile(id) {
  const car = findCar(id);
  if (!car) throw new Error(`tuning: unknown car '${id}'`);
  const C = car.tuning, cap = car.capabilities, BASE = car.makeParams();
  const ui = { ...C.ui, transfer: !!cap.transfer, diff: cap.lockers === 'rearDiff', live: liveSwitches(cap) };
  const RANGES = { ...BASE_RANGES, ...C.ranges };
  const ENGINES = C.engines;
  const tyreRadius = inches => BASE.tire.radius * inches / C.tyre.size;
  const wheelMass = C.wheelMass;
  // static spring compression of an axle (m), from the sprung load it carries
  const staticCompression = (P, a) => {
    const L = P.wheelbase, i = P.axles.indexOf(a);
    const frac = i === 0 ? (L / 2 - P.com[2]) / L : (L / 2 + P.com[2]) / L;
    return clamp(P.bodyMass * G * frac / 2 / a.k - a.preload, 0, a.travel);
  };
  // unsprung axle mass position and the sprung COM implied by the stock total COM (makeParams)
  const SPRUNG_COM = (() => {
    const P = BASE, ms = P.bodyMass, m = ms + P.axles[0].mass + P.axles[1].mass;
    const ay = a => a.droopY + staticCompression(P, a);
    return [
      (m * P.com[0]) / ms,
      (m * P.com[1] - P.axles.reduce((s, a) => s + a.mass * ay(a), 0)) / ms,
      (m * P.com[2] - P.axles.reduce((s, a) => s + a.mass * a.z, 0)) / ms,
    ];
  })();

  function stockSetup() {
    const P = BASE, [f, r] = P.axles;
    const axle = a => ({ k: a.k, bump: a.bump, rebound: a.rebound, arb: a.arb, travel: a.travel });
    const s = {
      v: 2,
      car: id,
      engine: { preset: C.stockEngine, torque: 1, revs: 0 },
      gearbox: {
        auto: [...P.auto.ratios], autoRev: P.auto.reverse,
        manual: [...P.manual.ratios], manualRev: P.manual.reverse,
        final: P.finalDrive,
        ...(P.transfer ? { high: P.transfer.high, low: P.transfer.low } : {}),
      },
      tyres: { size: C.tyre.size, width: C.tyre.width, pressF: P.tire.pressure, pressR: P.tire.pressure, grip: 1 },
      suspension: { lift: 0, front: axle(f), rear: axle(r) },
      brakes: { force: 1, bias: P.brakes.front / (P.brakes.front + P.brakes.rear), handbrake: 1 },
      mass: { cargo: 0, roof: 0, comY: 0 },
      steering: { lock: P.steer.maxAngle / DEG, ratio: P.steer.ratio },
      colliders: P.colliders.map((box, i) => ({ name: C.colliderNames[i] || `Box ${i + 1}`, box: [...box] })),
    };
    if (ui.diff) { const L = P.lsd || { preload: 80, power: 0.6, coast: 0.35 }; s.diff = { type: P.rearDiff || 'welded', preload: L.preload, power: L.power, coast: L.coast }; }
    return s;
  }

  // Fill a (possibly partial or foreign) setup from a base setup and clamp every number to its range.
  function fill(base, s) {
    const out = clone(base);
    if (!s || typeof s !== 'object' || (s.car && s.car !== id)) return out;
    const num = (v, key, def) => {
      const [a, b] = RANGES[key];
      return Number.isFinite(v) ? clamp(v, a, b) : def;
    };
    const e = s.engine || {};
    if (ENGINES[e.preset]) out.engine.preset = e.preset;
    out.engine.torque = num(e.torque, 'engine.torque', out.engine.torque);
    out.engine.revs = num(e.revs, 'engine.revs', out.engine.revs);
    const g = s.gearbox || {};
    for (const k of ['auto', 'manual']) if (Array.isArray(g[k]) && g[k].length === out.gearbox[k].length) out.gearbox[k] = g[k].map((v, i) => num(v, 'gearbox.ratio', out.gearbox[k][i]));
    for (const k of ['autoRev', 'manualRev']) out.gearbox[k] = num(g[k], 'gearbox.ratio', out.gearbox[k]);
    out.gearbox.final = num(g.final, 'gearbox.final', out.gearbox.final);
    if (ui.transfer) for (const k of ['high', 'low']) out.gearbox[k] = num(g[k], 'gearbox.' + k, out.gearbox[k]);
    const t = s.tyres || {};
    out.tyres.size = Math.round(num(t.size, 'tyres.size', out.tyres.size));
    out.tyres.width = num(t.width, 'tyres.width', out.tyres.width);
    out.tyres.pressF = num(t.pressF, 'tyres.press', out.tyres.pressF);
    out.tyres.pressR = num(t.pressR, 'tyres.press', out.tyres.pressR);
    out.tyres.grip = num(t.grip, 'tyres.grip', out.tyres.grip);
    const su = clone(s.suspension || {});
    C.migrate?.(s, su);
    out.suspension.lift = num(su.lift, 'suspension.lift', out.suspension.lift);
    for (const ax of ['front', 'rear']) for (const k of ['k', 'bump', 'rebound', 'arb', 'travel']) out.suspension[ax][k] = num(su[ax]?.[k], 'suspension.' + k, out.suspension[ax][k]);
    const b = s.brakes || {};
    for (const k of ['force', 'bias', 'handbrake']) out.brakes[k] = num(b[k], 'brakes.' + k, out.brakes[k]);
    const m = s.mass || {};
    for (const k of ['cargo', 'comY']) out.mass[k] = num(m[k], 'mass.' + k, out.mass[k]);
    if (ui.roof) out.mass.roof = num(m.roof, 'mass.roof', out.mass.roof);
    const st = s.steering || {};
    for (const k of ['lock', 'ratio']) out.steering[k] = num(st[k], 'steering.' + k, out.steering[k]);
    if (out.diff) {
      const df = s.diff || {};
      if (DIFF_TYPES.includes(df.type)) out.diff.type = df.type;
      for (const k of ['preload', 'power', 'coast']) out.diff[k] = num(df[k], 'diff.' + k, out.diff[k]);
    }
    if (Array.isArray(s.colliders) && s.colliders.length && s.colliders.length <= 24) {
      out.colliders = s.colliders.filter(c => Array.isArray(c?.box) && c.box.length === 7).map((c, i) => {
        const [cx, cy, cz, hx, hy, hz, r] = c.box;
        const box = [num(cx, 'box.c', 0), num(cy, 'box.c', 1), num(cz, 'box.c', 0), num(hx, 'box.h', 0.3), num(hy, 'box.h', 0.3), num(hz, 'box.h', 0.3), 0];
        box[6] = clamp(num(r, 'box.r', 0.03), 0, Math.min(box[3], box[4], box[5]) - 0.005);
        return { name: String(c.name || `Box ${i + 1}`).slice(0, 40), box };
      });
      if (!out.colliders.length) out.colliders = clone(base.colliders);
    }
    return out;
  }

  // ------------------------------------------------------------ setup -> params
  function applySetup(P, s) {
    const B = BASE;
    // engine
    const E = ENGINES[s.engine.preset] || ENGINES[C.stockEngine], k = s.engine.torque, dr = s.engine.revs;
    Object.assign(P.engine, {
      name: E.label, preset: s.engine.preset, fuel: E.fuel,
      torque: E.torque.map(([r, t]) => [r, t * k]),
      idleRpm: E.idleRpm, inertia: E.inertia,
      limiterRpm: E.limiterRpm + dr, redlineRpm: E.redlineRpm + dr, shiftRpm: E.shiftRpm + dr,
    });
    P.name = C.named(B, E, s.engine.preset);
    // a stronger engine gets a clutch and a lock-up clutch that hold it (stock parts hold the stock engine)
    const peak = Math.max(...P.engine.torque.map(x => x[1]));
    P.clutch.capacity = Math.max(B.clutch.capacity, peak * 1.45);
    P.auto.lockupCapacity = Math.max(B.auto.lockupCapacity, peak * 1.6);
    P.auto.shiftCapacity = Math.max(B.auto.shiftCapacity, peak * 1.9);
    // gearing
    const g = s.gearbox;
    P.auto.ratios = [...g.auto]; P.auto.reverse = g.autoRev;
    P.manual.ratios = [...g.manual]; P.manual.reverse = g.manualRev;
    P.finalDrive = g.final;
    if (P.transfer) { P.transfer.high = g.high; P.transfer.low = g.low; }
    // tyres: size scales the whole wheel (rim too, so the sidewall keeps its proportion)
    const t = s.tyres, R = tyreRadius(t.size), sc = R / B.tire.radius;
    const wm = wheelMass(t.size, t.width), wm0 = wheelMass(C.tyre.size, C.tyre.width);
    P.tire.size = t.size; P.tire.widthIn = t.width;
    P.tire.radius = R;
    P.tire.rimRadius = B.tire.rimRadius * sc;
    P.tire.width = B.tire.width * t.width / C.tyre.width;
    P.tire.inertia = B.tire.inertia * (wm / wm0) * sc * sc;
    P.tire.grip = t.grip;
    P.tire.pressure = t.pressF;
    // suspension: lift lowers the axles relative to the body (longer springs / spacers), springs and
    // dampers per axle; travel is the bump travel to the hard stop
    const su = s.suspension;
    P.axles.forEach((a, i) => {
      const b = B.axles[i], q = i === 0 ? su.front : su.rear;
      a.droopY = b.droopY - su.lift;
      a.k = q.k; a.bump = q.bump; a.rebound = q.rebound; a.arb = q.arb; a.travel = q.travel;
      a.mass = b.mass + 2 * (wm - wm0);
    });
    P.lift = su.lift;
    // brakes
    const br = s.brakes, tot = (B.brakes.front + B.brakes.rear) * br.force;
    P.brakes.front = Math.round(tot * br.bias * 100) / 100; P.brakes.rear = Math.round(tot * (1 - br.bias) * 100) / 100;
    P.brakes.handbrake = B.brakes.handbrake * br.handbrake;
    // steering
    P.steer.maxAngle = s.steering.lock * DEG; P.steer.ratio = s.steering.ratio;
    // rear diff (RWD cars): the drivetrain reads P.lsd every step; the type goes in through the panel
    if (s.diff) { P.rearDiff = s.diff.type; P.lsd = { preload: s.diff.preload, power: s.diff.power, coast: s.diff.coast }; }
    // mass: sprung body + cargo + roof load; COM height offset moves the sprung COM
    const m = s.mass, ms0 = B.bodyMass, CP = C.cargoPos, RP = C.roofPos;
    P.bodyMass = ms0 + m.cargo + m.roof;
    const sc0 = [SPRUNG_COM[0], SPRUNG_COM[1] + m.comY, SPRUNG_COM[2]];
    const sprung = [0, 1, 2].map(j => (ms0 * sc0[j] + m.cargo * CP[j] + m.roof * RP[j]) / P.bodyMass);
    P.sprungCom = sprung;
    const mu = P.axles[0].mass + P.axles[1].mass, mt = P.bodyMass + mu;
    const axY = a => a.droopY + staticCompression(P, a);
    const com = [
      (P.bodyMass * sprung[0]) / mt,
      (P.bodyMass * sprung[1] + P.axles.reduce((x, a) => x + a.mass * axY(a), 0)) / mt,
      (P.bodyMass * sprung[2] + P.axles.reduce((x, a) => x + a.mass * a.z, 0)) / mt,
    ];
    // stock: exactly the makeParams numbers (no rounding drift in the baselines)
    const isStockMass = m.cargo === 0 && m.roof === 0 && m.comY === 0 && su.lift === 0 && t.size === C.tyre.size && t.width === C.tyre.width;
    P.com = isStockMass ? [...B.com] : com;
    // inertia: stock body + parallel-axis terms of the added masses about the new COM + the COM shift
    const I = [...B.bodyInertia];
    if (!isStockMass) {
      const add = (mass, p) => {
        const dx = p[0] - P.com[0], dy = p[1] - P.com[1], dz = p[2] - P.com[2];
        I[0] += mass * (dy * dy + dz * dz); I[1] += mass * (dx * dx + dz * dz); I[2] += mass * (dx * dx + dy * dy);
      };
      const m0 = ms0 + B.axles[0].mass + B.axles[1].mass;
      add(m0, B.com);
      add(m.cargo, CP); add(m.roof, RP);
      I[0] += m.cargo * (0.5 * 0.5 + 1.0 * 1.0) / 12; I[1] += m.cargo * (1.2 * 1.2 + 1.0 * 1.0) / 12; I[2] += m.cargo * (1.2 * 1.2 + 0.5 * 0.5) / 12;
      I[1] += m.roof * (1.4 * 1.4 + 2 * 2) / 12; I[2] += m.roof * 1.4 * 1.4 / 12;
      P.axles.forEach((a, i) => add(a.mass - B.axles[i].mass, [0, axY(a), a.z]));
    }
    P.bodyInertia = I;
    // colliders
    P.colliders = s.colliders.map(c => [...c.box]);
    P.colliderNames = s.colliders.map(c => c.name);
    return P;
  }

  // The car's stock setup. Paste an exported setup (panel: Export) as `tuning.defaultSetup` in the car's
  // module to make it the new stock: the Stock button, new players and the panel's "stock" marks all use it.
  const STOCK = C.defaultSetup ? fill(stockSetup(), C.defaultSetup) : stockSetup();
  return {
    id, car, C, BASE, RANGES, ENGINES, ENGINE_ORDER: C.engineOrder, STOCK, ui, tyre: C.tyre,
    make: car.makeParams, staticCompression, tyreRadius, wheelMass, applySetup,
    sanitize: x => fill(STOCK, x),
    // how much higher the body sits than stock at static ride (bigger tyres + lift); spawn / teleport height
    rideRaise: P => (P.tire.radius - BASE.tire.radius) + (BASE.axles[0].droopY - P.axles[0].droopY),
  };
}

export const clone = o => JSON.parse(JSON.stringify(o));

const profiles = {};
// the tuning profile of a car (by id, see src/cars/index.js); throws for an unknown id
export const carTuning = id => profiles[id] || (profiles[id] = makeProfile(id));
// the profile of the car a params object belongs to
const of = P => carTuning(P.model);

export const applySetup = (P, s) => of(P).applySetup(P, s);
export const rideRaise = P => of(P).rideRaise(P);
export function makeTunedParams(setup, id) { const T = carTuning(id); return T.applySetup(T.make(), setup); }
const staticCompression = (P, a) => of(P).staticCompression(P, a);

// ---------------------------------------------------------------- consequences (panel readouts)
const peakOf = (P, f) => {
  let best = 0, at = 0;
  for (let r = 600; r <= P.engine.limiterRpm; r += 25) { const v = f(r); if (v > best) { best = v; at = r; } }
  return [best, at];
};
export const netTorque = (P, rpm) => Math.max(0, lerpTable(P.engine.torque, rpm) - engineFriction(rpm));

// Body outline in the side view (lowest point of every collider along z), for the ground angles.
function colliderUnderside(P) {
  const pts = [];
  for (const [cx, cy, cz, hx, hy, hz, r] of P.colliders) {
    for (let i = 0; i <= 40; i++) {
      const z = cz - hz + (2 * hz * i) / 40;
      const e = Math.max(0, Math.abs(z - cz) - (hz - r));
      pts.push([z, cy - hy + (r - Math.sqrt(Math.max(0, r * r - e * e)))]);
    }
  }
  return pts;
}
// angle (deg) of the line through a point d metres outboard of the axle at height h that touches a
// tyre of radius R whose centre is at height c (the ground under the tyre is y = 0)
function tangentAngle(d, h, R, c) {
  if (d <= 0) return 90;
  const B = c - h, L = Math.hypot(d, B);
  if (L <= R) return 90;    // over the tyre: the ramp can't reach it
  return (Math.asin(R / L) - Math.atan2(B, d)) / DEG;
}

// static ride: spring compression, tyre squash and the ground line in the body frame
export function staticRide(P, pressF = P.tire.pressure, pressR = pressF) {
  const L = P.wheelbase, mt = P.bodyMass + P.axles[0].mass + P.axles[1].mass;
  const fFront = (L / 2 - P.com[2]) / L;
  return P.axles.map((a, i) => {
    const c = staticCompression(P, a);
    const wheelLoad = mt * G * (i === 0 ? fFront : 1 - fFront) / 2;
    const squash = wheelLoad / tireRadialStiffness(i === 0 ? pressF : pressR);
    const hubY = a.droopY + c;               // hub height in the body frame
    return { c, wheelLoad, squash, hubY, groundY: hubY - (P.tire.radius - squash), hubH: P.tire.radius - squash };
  });
}

// pts: [z, y] underside outline in the body frame (default: the collision boxes)
export function geometry(P, pressF, pressR, pts0 = null) {
  const ride = staticRide(P, pressF, pressR);
  const [f, r] = P.axles, R = P.tire.radius;
  const zf = f.z, zr = r.z, gf = ride[0].groundY, gr = ride[1].groundY;
  const ground = z => gf + (z - zf) * (gr - gf) / (zr - zf);   // ground line under the truck (body frame)
  const pts = (pts0 || colliderUnderside(P)).map(([z, y]) => [z, y - ground(z)]);
  let app = 90, dep = 90, brk = 180, low = 9;
  for (const [z, h] of pts) {
    low = Math.min(low, h);
    if (z < zf) app = Math.min(app, tangentAngle(zf - z, h, R, ride[0].hubH));
    else if (z > zr) dep = Math.min(dep, tangentAngle(z - zr, h, R, ride[1].hubH));
    else brk = Math.min(brk, tangentAngle(z - zf, h, R, ride[0].hubH) + tangentAngle(zr - z, h, R, ride[1].hubH));
  }
  // axle diff housing (r 0.15 x 0.9) is the lowest part of a beam-axle truck
  const diff = Math.min(ride[0].hubH, ride[1].hubH) - 0.135;
  // tyre top vs the arch top (body frame) at full bump, and with one wheel pushed up by
  // full axle articulation (one spring on its stop, the other at full droop)
  const archTop = of(P).car.body.archTop;
  const arch = P.axles.map(a => {
    const bump = archTop - (a.droopY + a.travel + R);
    const twist = archTop - (a.droopY + a.travel / 2 * (1 + P.track / (a.springTrack ?? P.track)) + R);
    return { bump, twist };
  });
  return { ride, approach: app, departure: dep, breakover: brk, bodyClear: low, diffClear: diff, arch };
}

export function analyze(P, setup, gearbox = 'auto') {
  const t = setup.tyres;
  const out = {};
  const mt = P.bodyMass + P.axles[0].mass + P.axles[1].mass;
  out.mass = mt;
  // engine
  const [pk, pkAt] = peakOf(P, r => netTorque(P, r) * r * Math.PI / 30);
  const [tq, tqAt] = peakOf(P, r => netTorque(P, r));
  out.engine = { kw: pk / 1000, kwAt: pkAt, hp: pk / 735.5, nm: tq, nmAt: tqAt, kgPerHp: mt / (pk / 735.5) };
  // weight distribution and stability
  const L = P.wheelbase;
  out.front = (L / 2 - P.com[2]) / L;
  const geo = geometry(P, t.pressF, t.pressR);
  out.geo = geo;
  const gAvg = (geo.ride[0].groundY + geo.ride[1].groundY) / 2;
  out.comH = P.com[1] - gAvg;
  out.ssf = P.track / (2 * out.comH);            // static stability factor = rollover threshold in g
  out.tiltDeg = Math.atan(out.ssf) / DEG;         // side slope where it tips over (rigid, no body roll)
  // suspension: heave frequency and damping per axle (spring in series with the tyre)
  out.susp = P.axles.map((a, i) => {
    const kt = tireRadialStiffness(i === 0 ? t.pressF : t.pressR);
    const kRide = (a.k * kt) / (a.k + kt);
    const ms = P.bodyMass * G * (i === 0 ? out.front : 1 - out.front) / G;
    const f = Math.sqrt(2 * kRide / ms) / (2 * Math.PI);
    const zeta = (a.bump + a.rebound) / (2 * Math.sqrt(2 * a.k * ms)); // two dampers, average of bump/rebound
    return { f, zeta, kRide, rollStiff: a.k * (a.springTrack ?? P.track) ** 2 / 2 + a.arb };
  });
  // gearing: road speed at the limiter in every gear, high and low range
  const R = P.tire.radius, Re = R - 0.012;
  const box = gearbox === 'manual' ? P.manual.ratios : P.auto.ratios;
  const vAt = (ratio, tr) => (P.engine.limiterRpm * Math.PI / 30) / (ratio * tr * P.finalDrive) * Re * 3.6;
  const TR = transferOf(P);
  out.gears = box.map(g => ({ ratio: g, high: vAt(g, TR.high), low: vAt(g, TR.low) }));
  out.crawl = box[0] * TR.low * P.finalDrive;
  // top speed: power against drag + rolling resistance, capped by the limiter in top gear
  const C = of(P).C;
  const crr = C.crr * Math.sqrt(28 / Math.max(t.pressF, 4));
  let vmax = 0;
  for (let v = 5; v < 90; v += 0.25) {
    // best gear at this speed
    let best = 0;
    for (const g of box) {
      const w = v / Re * g * TR.high * P.finalDrive, rpm = w * 30 / Math.PI;
      if (rpm > P.engine.limiterRpm || rpm < 900) continue;
      best = Math.max(best, netTorque(P, rpm) * g * TR.high * P.finalDrive * 0.9 / Re);
    }
    const need = 0.5 * P.aero.rho * P.aero.cdA * v * v + crr * mt * G;
    if (best >= need) vmax = v; else if (vmax > 0) break;
  }
  out.vmax = vmax * 3.6;
  // brakes: deceleration the brakes alone can make (all four at full torque) and which axle locks first
  const brakeG = 2 * (P.brakes.front + P.brakes.rear) / Re / (mt * G);
  const mu = C.mu;
  const a = Math.min(brakeG, mu);
  const idealFront = out.front + a * out.comH / L;  // dynamic front share at that deceleration
  const bias = P.brakes.front / (P.brakes.front + P.brakes.rear);
  out.brakes = { brakeG, dirtG: a, bias, ideal: idealFront, first: bias >= idealFront ? 'front' : 'rear' };
  // steering
  const d = P.steer.maxAngle;
  out.turnDiameter = 2 * (L / Math.sin(d) + P.track / 2 + 0.15);
  out.lockToLock = 2 * d * P.steer.ratio / (2 * Math.PI);
  out.accel = estimateAccel(P, gearbox, t.pressF);
  return out;
}

// Quick 0-100 km/h estimate (the panel recomputes it on every change; "Measure" runs the full physics).
// Quasi-static: engine torque through the converter (auto) or a slipping clutch (manual), gearing,
// tyre traction limit, drag and rolling resistance, rotating inertia, shift pauses.
const TC_K = [[0, 1], [0.3, 1.02], [0.5, 1.07], [0.7, 1.17], [0.8, 1.3], [0.87, 1.55], [0.92, 2.0], [0.96, 3.0], [0.985, 5.5], [1, 9]];
const TC_TR = [[0, 2.1], [0.2, 1.86], [0.4, 1.6], [0.6, 1.35], [0.8, 1.1], [0.87, 1.0], [1, 1.0]];
export function estimateAccel(P, gearbox = 'auto', psi = 20) {
  const E = P.engine, auto = gearbox !== 'manual';
  const box = auto ? P.auto.ratios : P.manual.ratios;
  const mt = P.bodyMass + P.axles[0].mass + P.axles[1].mass;
  const Re = P.tire.radius - 0.012, tr = transferOf(P).high * P.finalDrive;
  const C = of(P).C;
  const crr = C.crr * Math.sqrt(28 / Math.max(psi, 4));
  const traction = C.traction * mt * G * (P.tire.grip ?? 1);   // what traction control lets through on the test surface
  const toRpm = 30 / Math.PI;
  let v = 0, t = 0, gi = 0, shift = 0, t60 = null, we = E.idleRpm / toRpm;
  while (t < 60 && v < 100 / 3.6) {
    const dt = t < 3 ? 0.01 : 0.02;
    const G4 = box[gi] * tr;
    const wt = v / Re * G4;                                   // gearbox input speed
    let Tin;
    if (auto) {
      // engine speed where the converter absorbs what the engine makes
      let lo = Math.max(wt, 1), hi = 900;
      for (let it = 0; it < 22; it++) {
        const m = 0.5 * (lo + hi), sr = Math.min(1, wt / m);
        const K = P.auto.stallK * lerpTable(TC_K, sr), Tp = (m * toRpm / K) ** 2;
        if (Tp > netTorque(P, m * toRpm)) hi = m; else lo = m;
      }
      we = Math.min(lo, E.limiterRpm / toRpm);
      const sr = Math.min(1, wt / we), K = P.auto.stallK * lerpTable(TC_K, sr);
      Tin = (we * toRpm / K) ** 2 * lerpTable(TC_TR, sr);
    } else {
      // slipping clutch at ~2200 rpm until the gear catches the engine
      we = Math.max(wt, Math.min(2200, E.limiterRpm * 0.6) / toRpm);
      Tin = netTorque(P, we * toRpm);
    }
    if (shift > 0) { Tin *= auto ? 0.45 : 0; shift -= dt; }
    const Iw = 4 * P.tire.inertia / (Re * Re) + (auto ? 0 : E.inertia * G4 * G4 / (Re * Re));
    const F = Math.min(traction, Tin * G4 / Re);
    const drag = 0.5 * P.aero.rho * P.aero.cdA * v * v + crr * mt * G;
    v = Math.max(0, v + dt * (F - drag) / (mt + Iw));
    t += dt;
    if (t60 === null && v >= 60 / 3.6) t60 = t;
    const rpm = (auto ? v / Re * G4 : we) * toRpm;
    if (gi < box.length - 1 && rpm > E.shiftRpm && shift <= 0) { gi++; shift = auto ? P.auto.shiftTime : 0.36; }
  }
  return { t100: v >= 100 / 3.6 ? t : null, t60 };
}

// Export: the setup as JSON. Pasting it as `tuning.defaultSetup` in the car's module makes it the new stock.
export function exportJSON(setup) {
  const s = clone(setup);
  const round = x => (typeof x === 'number' ? Math.round(x * 1e4) / 1e4 : x);
  const walk = o => { for (const k in o) { if (o[k] && typeof o[k] === 'object') walk(o[k]); else o[k] = round(o[k]); } };
  walk(s);
  return JSON.stringify(s, (k, v) => v, 2).replace(/\[\s+([-\d.,\s]+?)\s+\]/g, (m, a) => '[' + a.split(/,\s*/).join(', ') + ']');
}
