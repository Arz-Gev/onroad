import RAPIER from '@dimforge/rapier3d-compat';
import { Vehicle } from '../../src/vehicle/Vehicle.js';
import { carTuning, clone } from '../../src/vehicle/tuning.js';
import { SURFACES } from '../../src/vehicle/tire.js';

await RAPIER.init();

export const H = 1 / 240;
export const DEG = Math.PI / 180;

export function makeCar({ car = 's15', surface = 'asphalt', setup = null, tweak = null, gearbox = 'manual' } = {}) {
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  world.timestep = H;
  const n = 100, N = n + 1;
  world.createCollider(RAPIER.ColliderDesc.heightfield(n, n, new Float32Array(N * N), { x: 4000, y: 1, z: 4000 }));
  world.step();
  const T = carTuning(car);
  const s = setup ? T.sanitize(setup) : clone(T.STOCK);
  if (tweak) tweak(s);
  const P = T.applySetup(T.make(), s);
  const v = new Vehicle(RAPIER, world, P, { position: { x: 0, y: 0.05 + T.rideRaise(P), z: 0 }, surfaceAt: () => SURFACES[surface] });
  v.pressures = [s.tyres.pressF, s.tyres.pressR];
  if (gearbox === 'manual' && v.drivetrain.mode !== 'manual') v.drivetrain.toggleMode();
  const car_ = { world, v, P, t: 0, keys: { steer: 0, thr: 0, brk: 0 } };
  return car_;
}

export function keyboard(c, { steer = 0, throttle = 0, brake = 0, handbrake = 0 }) {
  const k = c.keys;
  const rate = steer === 0 ? 6 : (Math.sign(steer) !== Math.sign(k.steer) && k.steer !== 0 ? 8 : 4);
  k.steer += Math.max(-rate * H, Math.min(rate * H, steer - k.steer));
  k.thr = throttle > k.thr ? Math.min(throttle, k.thr + 6.5 * H) : Math.max(throttle, k.thr - 10 * H);
  k.brk = brake > k.brk ? Math.min(brake, k.brk + 8 * H) : Math.max(brake, k.brk - 12 * H);
  return { throttle: k.thr, brake: k.brk, steer: k.steer, steerKey: steer, clutch: 0, handbrake, analogSteer: false };
}

export function step(c, raw) {
  c.v.step(H, raw);
  c.world.step();
  c.t += H;
}

export function state(c) {
  const v = c.v;
  const vx = v.vel.dot(v.fwd), vy = v.vel.dot(v.right);
  return {
    t: c.t,
    kmh: v.speed * 3.6,
    beta: Math.atan2(vy, Math.max(Math.abs(vx), 0.5)) / DEG,
    yaw: v.angVel.dot(v.up) / DEG,
    roll: Math.asin(Math.max(-1, Math.min(1, v.right.y))) / DEG,
    pitch: Math.asin(Math.max(-1, Math.min(1, v.fwd.y))) / DEG,
    latG: v.accel.dot(v.right) / 9.81,
    lonG: v.accel.dot(v.fwd) / 9.81,
    steer: v.steerAngle / DEG,
    Fn: v.wheels.map(w => w.FnAvg),
    gear: v.drivetrain.manualGear,
    rpm: v.drivetrain.rpm,
  };
}

export function holdSpeed(c, kmh, gain = 0.25) {
  const e = kmh - c.v.speed * 3.6;
  return Math.max(0, Math.min(1, 0.15 + e * gain));
}

export function launchTo(c, kmh, gear = 2, thr = 0.7) {
  const d = c.v.drivetrain;
  for (let i = 0; i < 240; i++) step(c, keyboard(c, {}));
  d.requestShift(1);
  const t0 = c.t;
  while (c.v.speed * 3.6 < kmh && c.t - t0 < 30) {
    if (d.manualGear < gear && d.rpm > c.v.P.engine.limiterRpm - 500 && !d.shift) d.requestShift(1);
    step(c, keyboard(c, { throttle: thr }));
  }
  while (d.manualGear < gear) { d.requestShift(1); while (d.shift) step(c, keyboard(c, { throttle: holdSpeed(c, kmh) })); }
  return c.t - t0;
}
