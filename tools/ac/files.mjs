// Read an Assetto Corsa car's data folder (the unpacked `data/`) as { lower-case name: text } for src/ac.
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join } from 'node:path';

const TEXT = /\.(ini|lut|rto|csv|txt)$/i;

export function readCarFolder(dir) {
  const files = {};
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    if (statSync(p).isFile() && TEXT.test(name)) files[name.toLowerCase()] = readFileSync(p, 'latin1');
  }
  return files;
}
