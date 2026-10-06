// S15 drift check on flat asphalt (PLAN.md step 1 proof): settle, pull away in 2nd, turn in, kick the rear
// out with the handbrake, then a simple driver holds a target body slip angle with countersteer and
// throttle. Prints every 0.5 s: speed, rpm, body slip angle (beta), yaw rate, road-wheel angle, throttle.
//   node tools/drifttest.mjs [targetBetaDeg=30] [diff=welded|lsd|open] [kmh=55] [gear=2]
// DRIFT_KEYS=1: a keyboard driver in the hold (no steer key = the game's caster self-steer; a key only
// when the angle is far off), throttle as below.
// Driver gains (defaults from a sweep that holds 30 deg): DRIFT_GAINS='{"kp":0.3,"kd":0.25,"tb":0.3,"kt":2,"ki":0.5,"kv":0.02}'
// A held drift = beta stays near the target with the speed roughly steady for the whole hold phase.
import RAPIER from '@dimforge/rapier3d-compat';
import { Vehicle } from '../src/vehicle/Vehicle.js';
import { makeS15Params } from '../src/vehicle/params.js';
import { SURFACES } from '../src/vehicle/tire.js';
await RAPIER.init();
const H = 1 / 240;
const keys = process.env.DRIFT_KEYS === '1';   // keyboard driver: steer keys -1 / 0 / 1 and the game's drift help
const target = +(process.argv[2] || 30) / 57.3, diff = process.argv[3] || 'welded', kmh = +(process.argv[4] || 55), gear = +(process.argv[5] || 2);
const K = { kp: 0.3, kd: 0.25, tb: 0.3, kt: 2.0, ki: 0.5, kv: 0.02, ...(process.env.DRIFT_GAINS ? JSON.parse(process.env.DRIFT_GAINS) : {}) };
const P = makeS15Params();
P.rearDiff = diff;
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 }); world.timestep = H;
const n = 200, N = n + 1;
world.createCollider(RAPIER.ColliderDesc.heightfield(n, n, new Float32Array(N * N), { x: 3000, y: 1, z: 3000 }));
world.step();
const v = new Vehicle(RAPIER, world, P, { position: { x: 0, y: 0.05, z: 1300 }, surfaceAt: () => SURFACES.asphalt });
const d = v.drivetrain;
if (d.mode !== 'manual') d.toggleMode();
const deg = x => (x * 57.3).toFixed(1).padStart(6);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
console.log(`S15 drift test: target beta ${deg(target)} deg, diff ${diff}, entry ${kmh} km/h in gear ${gear}`);
console.log('    t phase     km/h   rpm  gear  beta   yaw   steer  thr  rearSpin');
let phase = 'settle', t = 0, lastPrint = -1, held = [], iBeta = 0, betaPrev = 0, betaDot = 0;
for (let i = 0; i < 240 * 26; i++, t += H) {
  const vx = v.vel.dot(v.fwd), vy = v.vel.dot(v.right), r = v.angVel.dot(v.up);
  const beta = Math.atan2(vy, Math.max(Math.abs(vx), 0.5));   // > 0: moving to the right of the heading
  const sp = v.speed * 3.6;
  let thr = 0, steer = 0, hb = 0, brake = 0;
  if (phase === 'settle') { if (t > 1.5) { phase = 'launch'; d.requestShift(1, v.ctl); } }
  else if (phase === 'launch') {
    const spin = 0.5 * (d.w[4] + d.w[5]) * v.R / Math.max(1, Math.abs(vx));
    thr = clamp((kmh - sp) * 0.2, 0, 1) * (spin > 1.12 ? 0.25 : 1);   // feather it: no burnout
    if (d.manualGear < gear && !d.shift && sp > 38 * d.manualGear) d.requestShift(1, v.ctl);
    if (sp > kmh - 2 && d.manualGear >= gear) { phase = 'turn'; t = 0; }
  } else if (phase === 'turn') {
    // left turn-in, handbrake to break the rear loose
    steer = -0.6; thr = 0.3; hb = t > 0.25 && t < 0.6 ? 1 : 0;
    if (t > 0.6) { phase = 'hold'; t = 0; }
  } else if (phase === 'hold') {
    // a left turn slides with the car moving to the right of its nose (beta > 0): countersteer to the
    // right, throttle holds the angle (more throttle = more angle)
    const err = target - beta;
    iBeta = clamp(iBeta + err * H, -0.5, 0.5);
    const kin = Math.atan2(vy - r * 1.2, Math.max(Math.abs(vx), 1));   // front axle travel direction
    betaDot += ((beta - betaPrev) / H - betaDot) * Math.min(1, H / 0.05);
    // countersteer: follow the front axle's travel, less of it to add angle, more when the angle grows fast
    const road = kin - K.kp * err + K.kd * betaDot;
    // keys: tap into the turn only when the angle is well short and not growing; countersteer key when well past
    steer = keys ? (err > 0.15 && betaDot < 0.1 ? -1 : err < -0.15 ? 1 : 0) : clamp(road / P.steer.maxAngle, -1, 1);
    thr = clamp(K.tb + K.kt * err + K.ki * iBeta + K.kv * (kmh - 10 - sp), 0, 1);
    if (t > 2) held.push({ beta, sp });
    if (t > 12) break;
  }
  betaPrev = beta;
  v.step(H, { throttle: thr, brake, steer, clutch: 0, handbrake: hb, analogSteer: !(keys && phase === 'hold') });
  world.step();
  if (v.up.y < 0.4) { console.log('ROLLED'); break; }
  const tick = Math.floor(t * 2);
  if (phase !== 'settle' && tick !== lastPrint) {
    lastPrint = tick;
    console.log(`${t.toFixed(1).padStart(5)} ${phase.padEnd(7)} ${sp.toFixed(0).padStart(5)} ${d.rpm.toFixed(0).padStart(5)} ${d.gearLabel().padStart(4)} ${deg(beta)} ${deg(r)} ${deg(v.steerAngle)} ${thr.toFixed(2).padStart(5)} ${(0.5 * (d.w[4] + d.w[5]) * v.R / Math.max(1, Math.abs(vx))).toFixed(2).padStart(5)}`);
  }
}
if (phase === 'settle' || phase === 'launch') console.log('never reached the entry speed');
const hub = v.axles.map(a => (a.droopY + a.c).toFixed(3));
if (held.length) {
  const b = held.map(x => x.beta * 57.3), s = held.map(x => x.sp);
  const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
  const sd = a => Math.sqrt(mean(a.map(x => (x - mean(a)) ** 2)));
  console.log(`hold (t > 2 s): beta ${mean(b).toFixed(1)} ± ${sd(b).toFixed(1)} deg, speed ${mean(s).toFixed(0)} ± ${sd(s).toFixed(1)} km/h, ${Math.abs(mean(b) - target * 57.3) < 6 && sd(b) < 5 ? 'HELD' : 'NOT HELD'}`);
}
console.log(`hub heights (front, rear): ${hub.join(', ')}`);
