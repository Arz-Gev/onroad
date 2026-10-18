// Pack an Assetto Corsa car's data folder into one JSON file the game fetches: { file name: text }.
//   node tools/ac/pack.mjs <car data folder> <out.json>
// The AC car mods in reference/ac-cars are packed into public/ac/cars/ (npm run ac:pack).
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import { readCarFolder } from './files.mjs';

const [src, out] = process.argv.slice(2);
if (!src || !out) { console.error('usage: node tools/ac/pack.mjs <car data folder> <out.json>'); process.exit(2); }
const files = readCarFolder(src);
// only what the physics reads: inis and the tables they name (no sounds, cameras or lights)
const keep = /^(car|suspensions|tyres|engine|drivetrain|brakes|aero|electronics|setup|ai|ctrl_.*|steer_brake_controller|drs)\.ini$|\.(lut|rto)$/;
const packed = Object.fromEntries(Object.entries(files).filter(([k]) => keep.test(k)));
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(packed));
console.log(`${out}: ${Object.keys(packed).length} files, ${(JSON.stringify(packed).length / 1024).toFixed(0)} KB`);
