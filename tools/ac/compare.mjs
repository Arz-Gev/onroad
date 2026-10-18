// Drive the same scripted inputs through rustyAC (the reference, tools/ac/ref) and through the JS port
// (src/ac), and compare the two cars after every 3 ms step.
//   node tools/ac/compare.mjs <scenario> [car data folder]      scenarios: see SCENARIOS below
// Prints, per quantity, the largest difference and the step where they first part by more than a tolerance.
import { execFileSync } from 'node:child_process';
import { writeFileSync, readFileSync, existsSync, mkdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { AcCar } from '../../src/ac/car.js';
import { readCarFolder } from './files.mjs';
import { COLUMNS, stateRow } from './state.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '../..');
const STEP = 0.003;
const s = t => Math.round(t / STEP);

// a scenario: seconds -> { gas, brake, steer, clutch, handbrake, up, dn } (clutch 1 = pedal released... the
// device's clutch is AC's: 1 engaged; the auto clutch takes over at standstill)
export const SCENARIOS = {
  settle: { secs: 3, at: () => ({}) },
  launch: { secs: 8, at: t => ({ gas: t > 0.5 ? 1 : 0, clutch: 1, up: [1, 2.5, 4.5].some(x => t > x && t < x + 0.05) }) },
  brake: { secs: 9, at: t => ({ gas: t > 0.5 && t < 5 ? 1 : 0, brake: t >= 5 ? 1 : 0, clutch: 1, up: [1, 2.5].some(x => t > x && t < x + 0.05) }) },
  corner: { secs: 10, at: t => ({ gas: t > 0.5 ? 0.45 : 0, steer: t > 4 ? -0.25 : 0, clutch: 1, up: [1, 2.5].some(x => t > x && t < x + 0.05) }) },
  slalom: { secs: 10, at: t => ({ gas: t > 0.5 ? 0.5 : 0, steer: t > 4 ? 0.2 * Math.sin((t - 4) * 3) : 0, clutch: 1, up: [1, 2.5].some(x => t > x && t < x + 0.05) }) },
  handbrake: { secs: 8, at: t => ({ gas: t > 0.5 && t < 4 ? 0.8 : 0, steer: t > 4 ? -0.4 : 0, handbrake: t > 4 && t < 4.6 ? 1 : 0, clutch: 1, up: [1].some(x => t > x && t < x + 0.05) }) },
  powerslide: { secs: 9, at: t => ({ gas: t > 0.5 && t < 4.5 ? 0.6 : t >= 4.5 ? 1 : 0, steer: t > 4 ? -0.5 : 0, clutch: 1, up: [1].some(x => t > x && t < x + 0.05) }) },
};

export function controlsLines(sc) {
  const lines = [];
  for (let i = 0; i < s(sc.secs); i++) {
    const c = sc.at(i * STEP);
    lines.push([c.gas ?? 0, c.brake ?? 0, c.steer ?? 0, c.clutch ?? 0, c.handbrake ?? 0, c.up ? 1 : 0, c.dn ? 1 : 0].join(' '));
  }
  return lines;
}

export function runJs(files, lines, onRow) {
  const car = new AcCar(files);
  car.forceRotation([0, 0, -1]);
  car.forcePosition([0, 0, 0]);
  car.sessionStart();
  const rows = [];
  lines.forEach((line, step) => {
    const v = line.split(' ').map(Number), c = car.controls;
    c.gas = v[0]; c.brake = v[1]; c.steer = v[2]; c.clutch = v[3]; c.handBrake = v[4]; c.gearUp = v[5] !== 0; c.gearDn = v[6] !== 0;
    car.step(STEP, 60000 + (step + 1) * 3);
    rows.push(stateRow(car, step));
    onRow?.(car, step);
  });
  return rows;
}

export function runRef(carDir, lines, name) {
  const bin = join(here, 'ref/target/release/acref');
  if (!existsSync(bin)) throw new Error('build the reference first: cargo build --release --manifest-path tools/ac/ref/Cargo.toml');
  const tmp = join(root, 'tools/ac/out');
  mkdirSync(tmp, { recursive: true });
  const input = join(tmp, `${name}.controls`), output = join(tmp, `${name}.ref.csv`);
  writeFileSync(input, lines.join('\n') + '\n');
  execFileSync(bin, [carDir, input, output]);
  return readFileSync(output, 'utf8').trim().split('\n').map(l => l.split(' ').map(Number));
}

// how far two runs may part before it counts: absolute + relative
const TOL = { p: 1e-3, v: 1e-2, w: 1e-2, up: 1e-4, z: 1e-4, hub: 1e-3, load: 20, sr: 5e-3, sa: 5e-3, fx: 30, fy: 30, travel: 5e-4, rpm: 10, gear: 0.5, boost: 1e-3, clutch: 1e-3 };
const tolFor = name => TOL[Object.keys(TOL).sort((a, b) => b.length - a.length).find(k => name.replace(/\d$/, '').startsWith(k))] ?? 1e-3;

export function compare(ref, js) {
  const n = Math.min(ref.length, js.length), out = [];
  COLUMNS.forEach((name, k) => {
    if (!k) return;
    let max = 0, at = -1, first = -1;
    for (let i = 0; i < n; i++) {
      const d = Math.abs(ref[i][k] - js[i][k]), tol = tolFor(name) + 1e-3 * Math.abs(ref[i][k]);
      if (!(d <= max)) { max = d; at = i; }
      if (first < 0 && !(d <= tol)) first = i;
    }
    out.push({ name, max, at, first });
  });
  return out;
}

function runOne(name, carDir, verbose) {
  const sc = SCENARIOS[name];
  if (!sc) throw new Error(`no scenario ${name}; have ${Object.keys(SCENARIOS).join(', ')}`);
  const lines = controlsLines(sc);
  const ref = runRef(carDir, lines, name);
  const t0 = performance.now();
  const js = runJs(readCarFolder(carDir), lines);
  const ms = (performance.now() - t0) / lines.length;
  const res = compare(ref, js);
  const bad = res.filter(r => r.first >= 0).sort((a, b) => a.first - b.first);
  if (verbose) for (const r of res) console.log(`${r.name.padEnd(8)} max |diff| ${r.max.toExponential(2)} at ${(r.at * STEP).toFixed(3)} s${r.first >= 0 ? `   parts at ${(r.first * STEP).toFixed(3)} s` : ''}`);
  const k = n => COLUMNS.indexOf(n), last = Math.min(ref.length, js.length) - 1;
  const drift = Math.hypot(ref[last][k('px')] - js[last][k('px')], ref[last][k('pz')] - js[last][k('pz')]);
  console.log(`${name.padEnd(10)} ${String(lines.length).padStart(5)} steps  JS ${ms.toFixed(3)} ms/step  ` +
    (bad.length ? `together until ${(bad[0].first * STEP).toFixed(2)} s (${bad.slice(0, 3).map(r => r.name).join(', ')} part first)` : 'together all the way') +
    `, ${(sc.secs).toFixed(0)} s later ${(drift * 100).toFixed(1)} cm apart`);
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const name = process.argv[2] || 'all';
  const carDir = process.argv[3] || join(root, 'reference/ac-cars/declan_byrne_nissan_silvia_s14');
  if (name === 'all') for (const n of Object.keys(SCENARIOS)) runOne(n, carDir, false);
  else runOne(name, carDir, true);
}
