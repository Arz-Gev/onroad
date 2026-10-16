import { makeCar, keyboard, step, state, launchTo, holdSpeed, H, DEG } from './sim.mjs';
import { SURFACES, tireForces, tireCoefs } from '../../src/vehicle/tire.js';
import { makeS15Params } from '../../src/vehicle/params.js';
import { Vector3 } from 'three';
import { SteerFilter } from '../../src/vehicle/steerFilter.js';

const only = process.argv[2];
const results = [];
function test(name, fn) {
  if (only && !name.includes(only)) return;
  const t0 = performance.now();
  const checks = [];
  const expect = (label, value, lo, hi) => checks.push({ label, value, ok: value >= lo && value <= hi, range: `[${lo}, ${hi}]` });
  try { fn(expect); } catch (e) { checks.push({ label: 'threw ' + e.message, value: NaN, ok: false, range: '' }); }
  const ok = checks.every(c => c.ok);
  results.push(ok);
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name} (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
  for (const c of checks) console.log(`   ${c.ok ? ' ' : '✗'} ${c.label}: ${Number.isFinite(c.value) ? +c.value.toFixed(3) : c.value} ${c.range}`);
}

const run = (c, secs, input) => { const t0 = c.t; while (c.t - t0 < secs) step(c, keyboard(c, input(c.t - t0))); };

test('tyre: lateral force peaks near the peak slip angle and a full slide keeps most of it', expect => {
  const surf = SURFACES.asphalt, co = tireCoefs(makeS15Params().tire, 30, surf, {});
  const Fn = 3500, w = { ux: 0, uy: 0 };
  const force = a => { w.uy = co.Ly * Math.tan(a); tireForces(w, Fn, 0, 0, 20, surf, co); return Math.abs(w.Fy); };
  let peak = 0, at = 0;
  for (let a = 0.005; a < 0.6; a += 0.005) { const f = force(a); if (f > peak) { peak = f; at = a; } }
  expect('peak slip angle (deg)', at / DEG, 4, 10);
  expect('peak lateral mu', peak / Fn, 0.9, 1.2);
  expect('force at 35 deg / peak', force(35 * DEG) / peak, 0.8, 0.97);
  w.ux = co.Lx * 0.3; w.uy = co.Ly * Math.tan(10 * DEG);
  tireForces(w, Fn, 0, 0, 20, surf, co);
  expect('combined slip stays inside the friction circle', Math.hypot(w.Fx, w.Fy) / (co.mu * Fn * 1.12), 0, 1.0001);
});

test('S15 static ride: weight split and suspension travel left both ways', expect => {
  const c = makeCar();
  run(c, 3, () => ({}));
  const v = c.v, Fn = v.wheels.map(w => w.FnAvg), tot = Fn.reduce((a, b) => a + b, 0);
  expect('front weight share', (Fn[0] + Fn[1]) / tot, 0.5, 0.54);
  expect('left/right difference (N)', Math.abs(Fn[0] - Fn[1]) + Math.abs(Fn[2] - Fn[3]), 0, 60);
  const p = v.P.axles;
  for (const w of v.wheels) {
    const ax = p[w.i < 2 ? 0 : 1];
    expect(`wheel ${w.i} droop travel (cm)`, w.c * 100, 4, 10);
    expect(`wheel ${w.i} free bump travel (cm)`, (ax.travel - 0.05 - w.c) * 100, 3, 8);
  }
  let moved = 0;
  const y0 = v.pos.y;
  run(c, 1, () => ({}));
  moved = Math.abs(v.pos.y - y0);
  expect('body still moving after settling (mm)', moved * 1000, 0, 1);
});

test('S15 steady cornering at 70 km/h: grip, roll and no lifted wheel', expect => {
  const c = makeCar();
  launchTo(c, 70, 3);
  let minFn = Infinity, minC = Infinity;
  run(c, 4, t => ({ steer: -1, throttle: holdSpeed(c, 70) }));
  const rolls = [];
  run(c, 1.5, t => {
    minFn = Math.min(minFn, ...c.v.wheels.map(w => w.FnAvg));
    minC = Math.min(minC, ...c.v.wheels.map(w => w.c));
    rolls.push(state(c).roll);
    return { steer: -1, throttle: holdSpeed(c, 70) };
  });
  const s = state(c);
  expect('lateral g', Math.abs(s.latG), 0.85, 1.05);
  expect('roll per g (deg/g)', Math.abs(s.roll / s.latG), 1.5, 4);
  expect('roll wobble while steady (deg)', Math.max(...rolls) - Math.min(...rolls), 0, 0.3);
  expect('lightest wheel load (N)', minFn, 600, 1e9);
  expect('inner wheel droop left (cm)', minC * 100, 0.5, 100);
  expect('body slip angle (deg)', Math.abs(s.beta), 0, 3);
});

test('S15 slalom at 70 km/h: weight moves side to side without topping out a wheel', expect => {
  const c = makeCar();
  launchTo(c, 70, 3);
  let minFn = Infinity, minC = Infinity, maxB = 0;
  run(c, 4, t => {
    minFn = Math.min(minFn, ...c.v.wheels.map(w => w.FnAvg));
    minC = Math.min(minC, ...c.v.wheels.map(w => w.c));
    maxB = Math.max(maxB, Math.abs(state(c).beta));
    return { steer: Math.floor(t / 0.8) % 2 ? 1 : -1, throttle: holdSpeed(c, 70) };
  });
  expect('lightest wheel load (N)', minFn, 400, 1e9);
  expect('least droop travel left (cm)', minC * 100, 0.5, 100);
  expect('largest body slip (deg)', maxB, 0, 6);
});

test('S15 hands-off steering: no shimmy at any speed after a 3 deg kick', expect => {
  for (const kmh of [15, 30, 60, 100, 140, 180]) {
    const c = makeCar();
    launchTo(c, kmh, kmh > 120 ? 5 : kmh > 70 ? 4 : 3);
    c.v.steerAngle = 3 * DEG; c.v.steerVel = 0;
    let late = 0;
    run(c, 2.5, t => { if (t > 1.5) late = Math.max(late, Math.abs(c.v.steerAngle / DEG)); return { throttle: holdSpeed(c, kmh) }; });
    expect(`${kmh} km/h: steering left after 1.5 s (deg)`, late, 0, 0.3);
  }
});

test('S15 lets go of the wheel after a corner: returns to centre and drives straight', expect => {
  const c = makeCar();
  launchTo(c, 70, 3);
  run(c, 3, () => ({ steer: -1, throttle: holdSpeed(c, 70) }));
  let rate = 0, prev = c.v.steerAngle, t1 = null, t2 = null;
  run(c, 2.5, t => {
    rate = Math.max(rate, Math.abs(c.v.steerAngle - prev) / H); prev = c.v.steerAngle;
    if (t1 === null && Math.abs(c.v.steerAngle) < 1 * DEG) t1 = t;
    if (t2 === null && t > 0.2 && Math.abs(state(c).yaw) < 3 && Math.abs(c.v.steerAngle) < 1 * DEG) t2 = t;
    return { throttle: holdSpeed(c, 70) };
  });
  expect('time to centre the wheel (s)', t1 ?? 99, 0.05, 0.8);
  expect('time to drive straight (s)', t2 ?? 99, 0.1, 1.5);
  expect('fastest steering motion (rad/s)', rate, 0, 3);
  expect('heading drift after 2.5 s (deg/s)', Math.abs(state(c).yaw), 0, 1);
});

const slide = (thr, release = 0.6) => {
  const c = makeCar();
  c.v.P.assist.stabilization = false; // the car's own slide, without the oversteer assistant catching it
  launchTo(c, 50, 2);
  // About 15 degrees of lock at 50 km/h, what the old keyboard assist allowed; full lock (45) would scrub the car to a crawl.
  run(c, 2, () => ({ steer: -0.35, throttle: holdSpeed(c, 50) }));
  run(c, release, () => ({ steer: -0.35, throttle: thr }));
  const v = c.v, L = v.P.wheelbase, cz = v.P.com[2];
  const trace = [];
  run(c, 2.5, () => {
    const vx = v.vel.dot(v.fwd), vy = v.vel.dot(v.right), r = v.angVel.dot(v.up);
    trace.push({ beta: state(c).beta, steer: v.steerAngle / DEG, fa: Math.atan2(vy - r * (L / 2 + cz), vx) / DEG });
    return { throttle: thr };
  });
  return trace;
};

test('S15 power oversteer: more throttle, more slide', expect => {
  const peak = {};
  for (const thr of [0.5, 0.7, 1]) peak[thr] = Math.max(...slide(thr).map(p => Math.abs(p.beta)));
  expect('50 % throttle: body slip (deg)', peak[0.5], 0, 4);
  expect('70 % throttle: body slip (deg)', peak[0.7], peak[0.5], 20);
  expect('full throttle: body slip (deg)', peak[1], 30, 180);
});

test('S15 braking from 100 km/h with ABS: straight and short', expect => {
  const c = makeCar();
  launchTo(c, 100, 4);
  const z0 = c.v.pos.clone(), yaw0 = c.v.yaw();
  let t = 0;
  while (c.v.speed > 0.3 && t < 8) { step(c, keyboard(c, { brake: 1 })); t += H; }
  const dist = c.v.pos.distanceTo(z0);
  expect('stopping distance (m)', dist, 30, 48);
  expect('heading change (deg)', Math.abs(c.v.yaw() - yaw0) / DEG, 0, 2);
});

test('welded diff ties the rear wheels; open diff lets them differ in a turn', expect => {
  for (const diff of ['welded', 'open']) {
    const c = makeCar({ tweak: s => { s.diff.type = diff; } });
    c.v.drivetrain.rearDiff = diff; c.v.drivetrain.rearLock = diff === 'welded';
    launchTo(c, 30, 2);
    run(c, 3, () => ({ steer: -1, throttle: holdSpeed(c, 30) }));
    const w = c.v.drivetrain.w, d = Math.abs(w[4] - w[5]) / Math.abs(w[4] + w[5]) * 2;
    if (diff === 'welded') expect('welded: rear wheel speed difference', d, 0, 0.002);
    else expect('open: rear wheel speed difference', d, 0.03, 0.5);
  }
});

test('Defender still drives: launch and a dirt corner without rolling over', expect => {
  const c = makeCar({ car: 'defender', surface: 'dirt' });
  expect('time to 50 km/h (s)', launchTo(c, 50, 3, 1), 3, 12);
  let minUp = 1;
  run(c, 4, () => { minUp = Math.min(minUp, c.v.up.y); return { steer: 1, throttle: holdSpeed(c, 50) }; });
  expect('lowest body up.y (1 = level)', minUp, 0.9, 1);
  expect('lateral g on dirt', Math.abs(state(c).latG), 0.4, 0.8);
});

test('BeamNG steering filter: driving straight with a forward axis a hair over unit length stays finite', expect => {
  const f = new SteerFilter(makeS15Params());
  const car = { vel: new Vector3(0, 0, -25), right: new Vector3(1, 0, 0), fwd: new Vector3(0, 0, -(1 + 1e-7)), accel: new Vector3(),
    wheelSpeed: 25, mu: 1.05, steerAngle: 0, wheelbase: 2.525, fronts: [{ Fn: 3000, surf: SURFACES.asphalt }, { Fn: 3000, surf: SURFACES.asphalt }] };
  for (let i = 0; i < 10; i++) f.update(0, false, H, car);
  car.fwd.set(0, 0, -1);
  let out = 0;
  for (let i = 0; i < 240; i++) out = f.update(0.5, false, H, car);
  expect('filter output after a second of steering', out, 0.3, 0.6);
});

const failed = results.filter(r => !r).length;
console.log(`\n${results.length - failed}/${results.length} passed`);
process.exit(failed ? 1 : 0);
