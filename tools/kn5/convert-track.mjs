import { readFileSync, writeFileSync, mkdirSync, existsSync, readdirSync } from 'node:fs';
import { join, basename } from 'node:path';
import { parseKn5, walk } from './kn5.mjs';
import { writeGlb } from './glb.mjs';
import { ddsInfo, needsTranscode, decodeDds, resize, nearestPot, encodePng } from './dds.mjs';

const src = process.argv[2];
if (!src) { console.error('usage: convert-track.mjs <ac track folder> [out root]'); process.exit(1); }
const id = basename(src.replace(/\/+$/, ''));
const out = join(process.argv[3] || 'public/tracks', id);
mkdirSync(join(out, 'textures'), { recursive: true });

function readIni(file) {
  const res = {}; let sec = null;
  if (!existsSync(file)) return res;
  for (let line of readFileSync(file, 'latin1').split(/\r?\n/)) {
    line = line.replace(/;.*$/, '').trim();
    const s = line.match(/^\[(.+)\]$/);
    if (s) { sec = res[s[1]] = res[s[1]] || {}; continue; }
    const kv = line.match(/^([^=]+)=(.*)$/);
    if (kv && sec) sec[kv[1].trim()] = kv[2].trim();
  }
  return res;
}

const kn5s = readdirSync(src).filter((f) => f.toLowerCase().endsWith('.kn5'));
const kn5File = kn5s.find((f) => f.toLowerCase() === `${id.toLowerCase()}.kn5`)
  || kn5s.sort((a, b) => readFileSync(join(src, b)).length - readFileSync(join(src, a)).length)[0];
const buf = readFileSync(join(src, kn5File));
if (buf.includes(Buffer.from('__AC_SHADERS_PATCH_KN5ENC'))) {
  console.error(`${kn5File} is CSP-encrypted; it can't be converted.`); process.exit(2);
}
const k = parseKn5(buf);
if (k.bytesRead !== k.bytesTotal) throw new Error(`parse stopped at ${k.bytesRead} of ${k.bytesTotal}`);

const surfaces = {
  ROAD: { friction: 1, physics: 'asphalt' }, WALL: { friction: 1, physics: 'wall' },
  KERB: { friction: 0.92, physics: 'kerb' }, GRASS: { friction: 0.6, physics: 'grass' },
  SAND: { friction: 0.8, physics: 'sand' }, GRAVEL: { friction: 0.7, physics: 'gravel' },
};
for (const s of Object.values(readIni(join(src, 'data/surfaces.ini')))) {
  if (!s.KEY) continue;
  const key = s.KEY.toUpperCase();
  surfaces[key] = { ...(surfaces[key] || {}), friction: +s.FRICTION || 1, damping: +s.DAMPING || 0,
    validTrack: s.IS_VALID_TRACK === '1', physics: surfaces[key]?.physics || guessPhysics(key) };
}
function guessPhysics(key) {
  if (/WAL/.test(key)) return 'wall';
  if (/GRASS/.test(key)) return 'grass';
  if (/SAND|GRAVEL|DIRT/.test(key)) return 'sand';
  if (/KERB/.test(key)) return 'kerb';
  return 'asphalt';
}
const surfaceKeys = Object.keys(surfaces).sort((a, b) => b.length - a.length);
function surfaceOf(name) {
  const m = name.match(/^\d+(.*)$/);
  if (!m) return null;
  const rest = m[1].toUpperCase();
  const key = surfaceKeys.find((key) => rest.startsWith(key));
  if (key) return key;
  if (rest.startsWith('WAL')) return 'WALL';
  return null;
}

const I = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
function mul(a, b) {
  const r = new Array(16);
  for (let c = 0; c < 4; c++) for (let rr = 0; rr < 4; rr++) {
    let s = 0; for (let t = 0; t < 4; t++) s += a[t * 4 + rr] * b[c * 4 + t];
    r[c * 4 + rr] = s;
  }
  return r;
}
const groups = new Map();
const markers = {};
let skipped = 0;
(function rec(node, M, active) {
  const m = node.matrix ? mul(M, node.matrix) : M;
  const on = active && node.active;
  if (node.name.startsWith('AC_')) {
    if (node.matrix) markers[node.name] = m;
    return;
  }
  if (node.mesh && on) {
    const ms = node.mesh;
    const surface = surfaceOf(node.name);
    const visible = !!(ms.visible && ms.renderable);
    if (!visible && !surface) { skipped++; }
    else {
      const key = `${ms.material}|${surface || ''}|${visible ? 1 : 0}`;
      if (!groups.has(key)) groups.set(key, { material: ms.material, surface, visible, parts: [], names: [] });
      groups.get(key).parts.push({ ms, m }); groups.get(key).names.push(node.name);
    }
  }
  for (const c of node.children) rec(c, m, on);
})(k.root, I, true);

const meshes = [];
let tris = 0;
for (const g of groups.values()) {
  let nv = 0, ni = 0;
  for (const { ms } of g.parts) { nv += ms.pos.length / 3; ni += ms.idx.length; }
  const pos = new Float32Array(nv * 3), nrm = new Float32Array(nv * 3), uv = new Float32Array(nv * 2), idx = new Uint32Array(ni);
  let vo = 0, io = 0;
  for (const { ms, m } of g.parts) {
    const n = ms.pos.length / 3;
    for (let i = 0; i < n; i++) {
      const x = ms.pos[i * 3], y = ms.pos[i * 3 + 1], z = ms.pos[i * 3 + 2];
      pos[(vo + i) * 3] = m[0] * x + m[4] * y + m[8] * z + m[12];
      pos[(vo + i) * 3 + 1] = m[1] * x + m[5] * y + m[9] * z + m[13];
      pos[(vo + i) * 3 + 2] = m[2] * x + m[6] * y + m[10] * z + m[14];
      const a = ms.nrm[i * 3], b = ms.nrm[i * 3 + 1], c = ms.nrm[i * 3 + 2];
      let nx = m[0] * a + m[4] * b + m[8] * c, ny = m[1] * a + m[5] * b + m[9] * c, nz = m[2] * a + m[6] * b + m[10] * c;
      const l = Math.hypot(nx, ny, nz) || 1;
      nrm[(vo + i) * 3] = nx / l; nrm[(vo + i) * 3 + 1] = ny / l; nrm[(vo + i) * 3 + 2] = nz / l;
      uv[(vo + i) * 2] = ms.uv[i * 2]; uv[(vo + i) * 2 + 1] = ms.uv[i * 2 + 1];
    }
    for (let i = 0; i < ms.idx.length; i++) idx[io + i] = ms.idx[i] + vo;
    vo += n; io += ms.idx.length;
  }
  tris += ni / 3;
  const mat = k.materials[g.material];
  meshes.push({
    name: `${mat.name}${g.surface ? ` [${g.surface}]` : ''}${g.visible ? '' : ' (hidden)'}`,
    pos, nrm, uv, idx, material: g.material,
    extras: { surface: g.surface, visible: g.visible, sources: g.names },
  });
}

const texFile = {};
let transcoded = 0;
for (const t of k.textures) {
  const safe = t.name.replace(/[^A-Za-z0-9._-]/g, '_');
  const dds = ddsInfo(t.data);
  if (dds && needsTranscode(dds)) {
    const img = resize(decodeDds(t.data), nearestPot(dds.width), nearestPot(dds.height));
    texFile[t.name] = `textures/${safe.replace(/\.dds$/i, '')}.png`;
    writeFileSync(join(out, texFile[t.name]), encodePng(img));
    transcoded++;
  } else {
    texFile[t.name] = `textures/${safe}`;
    writeFileSync(join(out, texFile[t.name]), t.data);
  }
}
const materials = k.materials.map((m) => ({
  name: m.name,
  extras: {
    shader: m.shader, blend: m.blend, alphaTested: m.alphaTested, depthMode: m.depthMode,
    props: Object.fromEntries(Object.entries(m.props).map(([n, v]) => [n, [v.A, ...v.B, ...v.C, ...v.D].map((x) => +x.toFixed(5))])),
    textures: Object.fromEntries(Object.entries(m.samplers).map(([s, v]) => [s, texFile[v.texture] || null])),
  },
}));

let line = null;
const aiFile = join(src, 'ai/fast_lane.ai');
if (existsSync(aiFile)) {
  const a = readFileSync(aiFile); const dv = new DataView(a.buffer, a.byteOffset, a.byteLength);
  const n = dv.getInt32(4, true);
  if (n > 1 && 16 + n * 20 <= a.length) {
    line = [];
    for (let i = 0; i < n; i++) {
      const o = 16 + i * 20;
      line.push(...[0, 4, 8].map((d) => +dv.getFloat32(o + d, true).toFixed(2)));
    }
  }
}

function tangentNear(p) {
  if (!line) return null;
  let best = 0, bd = Infinity;
  for (let i = 0; i < line.length / 3; i++) {
    const d = (line[i * 3] - p[0]) ** 2 + (line[i * 3 + 2] - p[2]) ** 2;
    if (d < bd) { bd = d; best = i; }
  }
  const n = line.length / 3, j = (best + 1) % n;
  return [line[j * 3] - line[best * 3], line[j * 3 + 2] - line[best * 3 + 2]];
}
const spawns = {};
for (const [name, m] of Object.entries(markers)) {
  const pos = [m[12], m[13], m[14]].map((v) => +v.toFixed(3));
  let fwd = [m[8], m[9], m[10]];
  const t = tangentNear(pos);
  if (t && fwd[0] * t[0] + fwd[2] * t[1] < 0) fwd = fwd.map((v) => -v);
  spawns[name] = { pos, forward: fwd.map((v) => +v.toFixed(4)) };
}

const ui = existsSync(join(src, 'ui/ui_track.json')) ? JSON.parse(readFileSync(join(src, 'ui/ui_track.json'), 'latin1').replace(/[\r\n\t]+/g, ' ')) : {};
const info = {
  id, name: ui.name || id, author: ui.author || null, country: ui.country || null, url: ui.url || null,
  source: kn5File, glb: 'track.glb', surfaces, spawns, racingLine: line,
};
writeFileSync(join(out, 'track.json'), JSON.stringify(info));
const glb = writeGlb({ meshes, materials, extras: { track: id } });
writeFileSync(join(out, 'track.glb'), glb);
console.log(`${id}: ${meshes.length} meshes (from ${Object.keys(k.materials).length} materials), ${tris} tris, `
  + `${k.textures.length} textures (${transcoded} to PNG), ${Object.keys(spawns).length} markers, racing line ${line ? line.length / 3 : 0} pts, `
  + `skipped ${skipped} hidden non-physical meshes, glb ${(glb.length / 1e6).toFixed(1)} MB → ${out}`);
