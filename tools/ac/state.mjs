// The car's state after a step, in the column order acref (tools/ac/ref) writes.
export const COLUMNS = ['step', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'upx', 'upy', 'upz', 'zx', 'zy', 'zz',
  ...[0, 1, 2, 3].flatMap(i => ['hubx', 'huby', 'hubz', 'load', 'sr', 'sa', 'fx', 'fy', 'w', 'travel'].map(k => `${k}${i}`)),
  'rpm', 'gear', 'boost', 'clutch'];

export function stateRow(car, step) {
  const c = car.core, b = car.body, m = c.matrix(b), vel = c.velocity(b), w = b.avel;
  const row = [step, ...b.pos, ...vel, ...w, m[4], m[5], m[6], m[8], m[9], m[10]];
  for (let i = 0; i < 4; i++) {
    const h = car.suspensions[i].hub.pos, t = car.tyres[i].status;
    row.push(h[0], h[1], h[2], t.load, t.slipRatio, t.slipAngleRad, t.fx, t.fy, t.angularVelocity, car.suspensions[i].status.travel);
  }
  const d = car.drivetrain;
  row.push(d.engineRpm, d.currentGear, d.engine.turboBoost, car.controls.clutch);
  return row;
}
