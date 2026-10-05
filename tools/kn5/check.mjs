// Is a car or track convertible? node tools/kn5/check.mjs <folder or .kn5> [...]
// Prints each .kn5 found and whether Custom Shaders Patch encrypted it (encrypted ones can't be converted).
import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join } from 'node:path';

const files = [];
for (const arg of process.argv.slice(2)) {
  if (statSync(arg).isDirectory()) {
    (function rec(d) { for (const f of readdirSync(d)) { const p = join(d, f); if (statSync(p).isDirectory()) rec(p); else if (/\.kn5$/i.test(f)) files.push(p); } })(arg);
  } else files.push(arg);
}
const MARK = Buffer.from('__AC_SHADERS_PATCH_KN5ENC');
for (const f of files) {
  const b = readFileSync(f);
  const ok = b.subarray(0, 6).toString() === 'sc6969';
  const enc = b.includes(MARK);
  console.log(`${!ok ? 'not a kn5 ' : enc ? 'ENCRYPTED ' : 'ok        '} ${(b.length / 1e6).toFixed(1).padStart(6)} MB  ${f}`);
}
