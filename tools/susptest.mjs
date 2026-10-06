// S15 suspension check on flat asphalt: static loads and ride height, then (each from a fresh car)
// steady cornering roll, braking dive, full-throttle squat and a 5 cm drop onto all four wheels.
//   node tools/susptest.mjs
import RAPIER from '@dimforge/rapier3d-compat';
import { Vehicle } from '../src/vehicle/Vehicle.js';
import { makeS15Params } from '../src/cars/s15.js';
import { SURFACES } from '../src/vehicle/tire.js';
await RAPIER.init();
const H = 1 / 240, D = 57.3;
const mean = a => a.reduce((x, y) => x + y, 0) / a.length;

function car() {
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 }); world.timestep = H;
  const n = 200, N = n + 1;
  world.createCollider(RAPIER.ColliderDesc.heightfield(n, n, new Float32Array(N * N), { x: 3000, y: 1, z: 3000 }));
  world.step();
  const v = new Vehicle(RAPIER, world, makeS15Params(), { position: { x: 0, y: 0.05, z: 1300 }, surfaceAt: () => SURFACES.asphalt });
  if (v.drivetrain.mode !== 'manual') v.drivetrain.toggleMode();
  const run = (secs, ctl, each) => {
    for (let i = 0; i < secs / H; i++) {
      const c = typeof ctl === 'function' ? ctl(v, i * H) : ctl;
      const d = v.drivetrain;
      if (!c.clutch && d.rpm > 7300 && d.manualGear >= 1 && d.manualGear < 5 && !d.shift) d.requestShift(1, v.ctl);
      v.step(H, { throttle: 0, brake: 0, steer: 0, clutch: 0, handbrake: 0, analogSteer: true, ...c });
      world.step();
      each?.(v, i * H);
    }
  };
  run(2, {});
  v.drivetrain.requestShift(1, v.ctl);
  run(0.5, {});
  return { v, run, world };
}
const roll = v => Math.asin(v.right.y) * D;      // > 0: right side up
const pitch = v => Math.asin(v.fwd.y) * D;       // > 0: nose up
const hubs = v => v.wheels.map(w => (v.hubLocal(w, w.hub.clone()).y * 1000).toFixed(0)).join(' ');
const loads = v => v.wheels.map(w => (w.FnAvg / 9.81).toFixed(0)).join(' ');

{ // static
  const { v } = car();
  const sum = v.wheels.reduce((s, w) => s + w.FnAvg, 0);
  console.log(`static: loads FL FR RL RR ${loads(v)} kg (sum ${(sum / 9.81).toFixed(0)} of ${v.totalMass} kg), front ${(100 * (v.wheels[0].FnAvg + v.wheels[1].FnAvg) / sum).toFixed(1)} %`);
  console.log(`        hubs ${hubs(v)} mm (body frame), body y ${v.pos.y.toFixed(3)}, pitch ${pitch(v).toFixed(2)} deg`);
}
{ // steady cornering: hold speed, ramp the steering, report roll per g
  const { v, run } = car();
  const kmh = 70;
  run(10, (v, t) => ({ throttle: Math.max(0, Math.min(1, (kmh - v.speed * 3.6) * 0.15 + 0.15)) }));
  for (const st of (process.env.STEERS || "0.02,0.04,0.06").split(",").map(Number)) {
    const s = [];
    run(4, v => ({ throttle: Math.max(0, Math.min(1, (kmh - v.speed * 3.6) * 0.15 + 0.2)), steer: st / v.P.steer.maxAngle }),
      (v, t) => { if (t > 2.5) s.push([v.accel.dot(v.right) / 9.81, roll(v), v.wheels.map(w => w.FnAvg)]); });
    const ay = mean(s.map(x => x[0])), r = mean(s.map(x => x[1]));
    const fl = mean(s.map(x => x[2][0])), fr = mean(s.map(x => x[2][1])), rl = mean(s.map(x => x[2][2])), rr = mean(s.map(x => x[2][3]));
    const ltF = Math.abs(fr - fl) / (fr + fl), ltR = Math.abs(rr - rl) / (rr + rl);
    console.log(`corner ${kmh} km/h steer ${(st * D).toFixed(1)} deg: lat ${ay.toFixed(2)} g, roll ${r.toFixed(2)} deg (${(r / Math.max(Math.abs(ay), 1e-3)).toFixed(2)} deg/g), load transfer F ${(ltF * 100).toFixed(0)} % R ${(ltR * 100).toFixed(0)} %`);
  }
}
{ // braking from 100 km/h
  const { v, run } = car();
  run(14, (v, t) => ({ throttle: v.speed * 3.6 < 100 ? 0.6 : 0 }));
  const v0 = v.speed * 3.6;
  const s = [];
  run(1.2, { brake: 1, clutch: 1 }, (v, t) => { if (t > 0.4 && t < 1.0) s.push([-v.accel.dot(v.fwd) / 9.81, pitch(v)]); });
  console.log(`braking from ${v0.toFixed(0)} km/h: decel ${mean(s.map(x => x[0])).toFixed(2)} g, pitch ${mean(s.map(x => x[1])).toFixed(2)} deg (nose down < 0)`);
}
{ // full throttle in 2nd
  const { v, run } = car();
  run(3, v => ({ throttle: v.speed * 3.6 < 40 ? 0.3 : 0.1 }));
  const s = [];
  run(1.0, { throttle: 1 }, (v, t) => { if (t > 0.3) s.push([v.accel.dot(v.fwd) / 9.81, pitch(v)]); });
  console.log(`full throttle: accel ${mean(s.map(x => x[0])).toFixed(2)} g, pitch ${mean(s.map(x => x[1])).toFixed(2)} deg (squat: nose up > 0)`);
}
{ // drop: body vertical motion after a 5 cm drop, to see the damping
  const { v, run } = car();
  const y0 = v.pos.y;
  v.body.setTranslation({ x: v.pos.x, y: v.pos.y + 0.05, z: v.pos.z }, true);
  const ys = [];
  run(2, {}, v => ys.push(v.pos.y - y0));
  let peaks = [];
  for (let i = 1; i < ys.length - 1; i++) if ((ys[i] - ys[i - 1]) * (ys[i + 1] - ys[i]) < 0) peaks.push(`${(i * H).toFixed(2)}s ${(ys[i] * 1000).toFixed(1)}mm`);
  console.log(`drop 5 cm: extremes ${peaks.slice(0, 6).join(', ')}; after 2 s ${(ys.at(-1) * 1000).toFixed(1)} mm`);
}
