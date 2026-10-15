import { makeCar, keyboard, step, state, launchTo, holdSpeed, H, DEG } from './sim.mjs';
import { SURFACES, tireForces, tireCoefs } from '../../src/vehicle/tire.js';
import { makeS15Params } from '../../src/vehicle/params.js';

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

test('hands off at speed: no shimmy, no weave after a 3 deg kick', expect => {
  for (const [car, kmh, gear] of [['s15', 30, 2], ['s15', 100, 4], ['s15', 180, 5], ['defender', 60, 3], ['defender', 110, 4]]) {
    const c = makeCar({ car });
    launchTo(c, kmh, gear, car === 'defender' ? 1 : 0.7);
    c.v.steerAngle = 3 * DEG; c.v.steerVel = 0;
    let steer = 0, yaw = 0;
    run(c, 4, t => {
      if (t > 3) { steer = Math.max(steer, Math.abs(c.v.steerAngle / DEG)); yaw = Math.max(yaw, Math.abs(state(c).yaw)); }
      return { throttle: holdSpeed(c, kmh) };
    });
    expect(`${car} ${kmh} km/h: steering after 3 s (deg)`, steer, 0, 0.2);
    expect(`${car} ${kmh} km/h: yaw rate after 3 s (deg/s)`, yaw, 0, 0.5);
  }
});

const frontTravel = v => {
  const L = v.P.wheelbase, cz = v.P.com[2], maxA = v.P.steer.maxAngle;
  const vx = v.vel.dot(v.fwd), vy = v.vel.dot(v.right) - v.angVel.dot(v.up) * (L / 2 + cz);
  return Math.max(-maxA, Math.min(maxA, Math.atan2(vy * Math.sign(vx), Math.abs(vx))));
};
const handsOff = (c, secs, thr) => {
  const v = c.v, yaw0 = v.yaw();
  let lined = null, share = 0;
  run(c, secs, t => {
    if (lined === null && Math.abs(v.steerAngle - frontTravel(v)) < 1 * DEG) lined = t;
    if (lined !== null && t > lined + 0.1) {
      const f = v.wheels.slice(0, 2), grip = f.reduce((s, w) => s + w.co.mu * w.Fn, 0);
      if (grip > 0) share = Math.max(share, Math.abs(f[0].Fy + f[1].Fy) / grip);
    }
    return { throttle: thr(t) };
  });
  return { lined: lined ?? 99, share, turn: Math.abs(v.yaw() - yaw0) / DEG };
};

test('S15 hands off: the wheels go where the front of the car goes, the car is not steered by them', expect => {
  const cases = {
    'let go in a 30 km/h corner': c => { launchTo(c, 30, 2); run(c, 2, () => ({ steer: -1, throttle: holdSpeed(c, 30) })); return () => holdSpeed(c, 30); },
    'let go in a 70 km/h corner': c => { launchTo(c, 70, 3); run(c, 2, () => ({ steer: -1, throttle: holdSpeed(c, 70) })); return () => holdSpeed(c, 70); },
    'let go in a slide, 80 % throttle': c => { launchTo(c, 50, 2); run(c, 2, () => ({ steer: -1, throttle: holdSpeed(c, 50) })); run(c, 0.6, () => ({ steer: -1, throttle: 0.8 })); return () => 0.8; },
    'let go in a slide, full throttle': c => { launchTo(c, 50, 2); run(c, 2, () => ({ steer: -1, throttle: holdSpeed(c, 50) })); run(c, 0.6, () => ({ steer: -1, throttle: 1 })); return () => 1; },
    'wheels left at 20 deg going straight at 60 km/h': c => { launchTo(c, 60, 3); c.v.steerAngle = 20 * DEG; c.v.steerVel = 0; return () => holdSpeed(c, 60); },
  };
  for (const [name, setup] of Object.entries(cases)) {
    const c = makeCar();
    const r = handsOff(c, 2.5, setup(c));
    expect(`${name}: wheels lined up with the front's travel after (s)`, r.lined, 0, 0.1);
    expect(`${name}: front tyres' side force once lined up (share of their grip)`, r.share, 0, 0.25);
    if (name.startsWith('wheels left')) expect(`${name}: heading change (deg)`, r.turn, 0, 4);
    else if (name.includes('corner')) expect(`${name}: yaw rate 2.5 s later (deg/s)`, Math.abs(state(c).yaw), 0, 0.5);
  }
});

test('S15 standing still hands off: the wheels stay where they were left', expect => {
  const c = makeCar();
  run(c, 1, () => ({}));
  run(c, 1.2, () => ({ steer: 1 }));
  const a = c.v.steerAngle;
  run(c, 2, () => ({}));
  expect('wheel moved (deg)', Math.abs(c.v.steerAngle - a) / DEG, 0, 0.2);
});

const slide = thr => {
  const c = makeCar();
  launchTo(c, 50, 2);
  run(c, 2, () => ({ steer: -1, throttle: holdSpeed(c, 50) }));
  run(c, 0.6, () => ({ steer: -1, throttle: thr }));
  let peak = 0;
  run(c, 2.5, () => { peak = Math.max(peak, Math.abs(state(c).beta)); return { throttle: thr }; });
  return peak;
};

test('S15 power oversteer: more throttle, more slide', expect => {
  const peak = {};
  for (const thr of [0.5, 0.7, 1]) peak[thr] = slide(thr);
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

const failed = results.filter(r => !r).length;
console.log(`\n${results.length - failed}/${results.length} passed`);
process.exit(failed ? 1 : 0);
