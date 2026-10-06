// Converts an unencrypted Assetto Corsa track (.kn5) into a GLB for the game.
// - node transforms baked (AC's frame is three.js's: +y up, same handedness)
// - one primitive per kn5 mesh, merged per material and per group (visual / physics surface)
// - DDS textures decoded with macOS `sips`, then resized to webp
// - materials: base colour from txDiffuse, alpha test for the AT / tree shaders, normal maps. Multilayer
//   materials (road, grass, kerbs) keep their detail texture as a separate webp next to the GLB
//   (<out>/<name>.webp); material extras.detail says which, the tile size in metres and its mean colour
// - physics meshes (names like 1ROAD_main, 1GRASS_x, 1KERB, 1WALL_x) go to `phys_<SURFACE>` groups and are also
//   rendered; everything else goes to `visual`. Scene extras list the surfaces.
// Usage: node tools/kn5glb.mjs <track.kn5> <out.glb> [maxTexture]
// CSP-encrypted kn5 files (car mods marked __AC_SHADERS_PATCH_KN5ENC_v1__) are refused.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { Document, NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { dedup, join, prune, weld, textureCompress } from '@gltf-transform/functions';
import sharp from 'sharp';
import * as THREE from 'three';

const [inPath, outPath, maxTexArg] = process.argv.slice(2);
const maxTex = +(maxTexArg || 2048);
const buf = fs.readFileSync(inPath);
if (buf.subarray(-64).toString('latin1').includes('KN5ENC')) throw new Error('encrypted kn5: not supported');

// ---------------------------------------------------------------- kn5 parse
let p = 0;
const i32 = () => { const v = buf.readInt32LE(p); p += 4; return v; };
const f32 = () => { const v = buf.readFloatLE(p); p += 4; return v; };
const u8 = () => buf[p++];
const str = () => { const n = i32(); const s = buf.toString('utf8', p, p + n); p += n; return s; };
if (buf.toString('latin1', 0, 6) !== 'sc6969') throw new Error('not a kn5');
p = 6;
const version = i32(); if (version > 5) i32();
const textures = [];
for (let i = 0, n = i32(); i < n; i++) { i32(); const name = str(); const size = i32(); textures.push({ name, data: buf.subarray(p, p + size) }); p += size; }
const materials = [];
for (let i = 0, n = i32(); i < n; i++) {
  const m = { name: str(), shader: str(), props: {}, maps: {} };
  m.blend = u8(); m.alphaTested = u8(); m.depthMode = i32();
  for (let j = 0, np = i32(); j < np; j++) { const k = str(); const a = f32(); const b = [f32(), f32()]; const c = [f32(), f32(), f32()]; const d = [f32(), f32(), f32(), f32()]; m.props[k] = { a, b, c, d }; }
  for (let j = 0, nm = i32(); j < nm; j++) { const k = str(); i32(); m.maps[k] = str(); }
  materials.push(m);
}
const meshes = [];
const markers = [];   // AC_START_n, AC_PIT_n, AC_HOTLAP_START_n, AC_TIME_n_L/R: spawn and timing points
const readNode = (parentM) => {
  const cls = i32(), name = str(), nc = i32(); u8();
  let M = parentM;
  if (cls === 1) {
    const a = []; for (let k = 0; k < 16; k++) a.push(f32());
    M = parentM.clone().multiply(new THREE.Matrix4().fromArray(a));
    if (name.startsWith('AC_')) markers.push({ name, M });
  } else if (cls === 2) {
    u8(); u8(); u8();
    const nv = i32(), pos = new Float32Array(nv * 3), nrm = new Float32Array(nv * 3), uv = new Float32Array(nv * 2);
    for (let v = 0; v < nv; v++) {
      pos[v * 3] = f32(); pos[v * 3 + 1] = f32(); pos[v * 3 + 2] = f32();
      nrm[v * 3] = f32(); nrm[v * 3 + 1] = f32(); nrm[v * 3 + 2] = f32();
      uv[v * 2] = f32(); uv[v * 2 + 1] = f32();
      p += 12; // tangent
    }
    const ni = i32(), idx = new Uint32Array(ni);
    for (let k = 0; k < ni; k++) { idx[k] = buf.readUInt16LE(p); p += 2; }
    const mat = i32(); i32(); f32(); f32(); p += 16; u8();
    meshes.push({ name, M: parentM, pos, nrm, uv, idx, mat });
  } else if (cls === 3) {
    u8(); u8(); u8();
    for (let b = 0, nb = i32(); b < nb; b++) { str(); p += 64; }
    const nv = i32(); p += nv * 76; const ni = i32(); p += ni * 2; i32(); i32(); f32(); f32();
    console.warn(`skipped skinned mesh ${name}`);
  } else throw new Error(`unknown node class ${cls} at ${p}`);
  for (let c = 0; c < nc; c++) readNode(M);
};
readNode(new THREE.Matrix4());
if (p !== buf.length) console.warn(`parsed ${p} of ${buf.length} bytes`);

// ---------------------------------------------------------------- textures: DDS -> PNG
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'kn5tex-'));
const used = new Set(materials.flatMap(m => [m.maps.txDiffuse, m.maps.txNormal, m.maps.txDetailR].filter(Boolean)));
const png = new Map();
for (const t of textures) {
  if (!used.has(t.name)) continue;
  const head = t.data.toString('latin1', 0, 4);
  const src = path.join(tmp, 'in_' + t.name.replace(/[^\w.-]/g, '_'));
  fs.writeFileSync(src, t.data);
  if (head === 'DDS ') {
    const out = src + '.png';
    execFileSync('sips', ['-s', 'format', 'png', src, '--out', out], { stdio: 'ignore' });
    png.set(t.name, fs.readFileSync(out));
  } else png.set(t.name, t.data);   // png / jpg as is
}

// ---------------------------------------------------------------- glTF
const doc = new Document();
const bufferG = doc.createBuffer();
const scene = doc.createScene('track');
doc.getRoot().setDefaultScene(scene);
const texCache = new Map();
const tex = name => {
  if (!name || !png.has(name)) return null;
  if (!texCache.has(name)) texCache.set(name, doc.createTexture(name).setImage(png.get(name)).setMimeType(name.toLowerCase().endsWith('.jpg') ? 'image/jpeg' : 'image/png'));
  return texCache.get(name);
};
const outDir = outPath.replace(/\.glb$/, '');
const details = new Map();   // texture name -> file name, written after the GLB
const detailFile = name => { if (!details.has(name)) details.set(name, name.replace(/\.\w+$/, '').replace(/[^\w-]/g, '_') + '.webp'); return details.get(name); };
const ALPHA_TEST = /AT|Tree|Grass/;   // ksPerPixelAT, ksPerPixelAT_NM, ksTree, ksGrass ...
const gMats = materials.map(m => {
  const mt = doc.createMaterial(m.name).setRoughnessFactor(0.85).setMetallicFactor(0);
  // ksDiffuse / ksAmbient are AC lighting gains, not albedo: the textures carry the colour
  mt.setBaseColorFactor([1, 1, 1, 1]);
  const d = tex(m.maps.txDiffuse);
  if (d) mt.setBaseColorTexture(d);
  const n = tex(m.maps.txNormal);
  if (n && /NM/.test(m.shader)) mt.setNormalTexture(n);
  if (ALPHA_TEST.test(m.shader) || m.alphaTested) mt.setAlphaMode('MASK').setAlphaCutoff(m.props.ksAlphaRef?.a || 0.5).setDoubleSided(true);
  else if (m.blend === 1) mt.setAlphaMode('BLEND');
  const e = m.props.ksEmissive?.c;
  if (e && (e[0] + e[1] + e[2]) > 0) mt.setEmissiveFactor(e.map(x => Math.min(1, x)));
  const spec = m.props.ksSpecular?.a ?? 0, exp = m.props.ksSpecularEXP?.a ?? 1;
  if (spec > 0.05) mt.setRoughnessFactor(Math.max(0.25, Math.min(0.9, 1.2 - Math.log2(exp + 1) / 8)));
  const extras = { shader: m.shader };
  if (/Multilayer/.test(m.shader) && m.maps.txDetailR && png.has(m.maps.txDetailR)) {
    extras.detail = { file: detailFile(m.maps.txDetailR), scale: /grass/i.test(m.name) ? 4 : 3 };
  }
  mt.setExtras(extras);
  return mt;
});

// physics surface from the mesh name: "1ROAD_main" -> ROAD; kerbs named inside a road mesh -> KERB
const SURF = /^\d+([A-Z]+)/;
const surfaceOf = name => {
  const s = SURF.exec(name)?.[1];
  if (!s) return null;
  if (/kerb/i.test(name)) return 'KERB';
  return s;
};
const groups = new Map();
const group = key => {
  if (!groups.has(key)) { const g = doc.createNode(key); scene.addChild(g); groups.set(key, g); }
  return groups.get(key);
};
const v = new THREE.Vector3(), nm = new THREE.Matrix3();
let tris = 0;
const surfaces = {};
for (const m of meshes) {
  const M = m.M;
  nm.getNormalMatrix(M);
  const pos = new Float32Array(m.pos.length), nrm = new Float32Array(m.nrm.length);
  for (let i = 0; i < pos.length; i += 3) {
    v.fromArray(m.pos, i).applyMatrix4(M); v.toArray(pos, i);
    v.fromArray(m.nrm, i).applyMatrix3(nm).normalize(); v.toArray(nrm, i);
  }
  const idx = m.idx;
  const prim = doc.createPrimitive()
    .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(pos).setBuffer(bufferG))
    .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(nrm).setBuffer(bufferG))
    .setAttribute('TEXCOORD_0', doc.createAccessor().setType('VEC2').setArray(m.uv).setBuffer(bufferG))
    .setIndices(doc.createAccessor().setType('SCALAR').setArray(idx).setBuffer(bufferG))
    .setMaterial(gMats[m.mat]);
  const s = surfaceOf(m.name);
  const key = s ? 'phys_' + s : 'visual';
  if (s) surfaces[s] = (surfaces[s] || 0) + idx.length / 3;
  group(key).addChild(doc.createNode(m.name).setMesh(doc.createMesh(m.name).addPrimitive(prim)));
  tris += idx.length / 3;
}

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
await doc.transform(
  weld(),
  dedup(),
  join({ keepNamed: false }),
  prune(),
  textureCompress({ encoder: sharp, targetFormat: 'webp', resize: [maxTex, maxTex] }),
);
// bounds for the game (spawn, camera far plane, minimap)
const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
for (const m of doc.getRoot().listMeshes()) for (const pr of m.listPrimitives()) {
  const a = pr.getAttribute('POSITION'); const lo = a.getMin([]), hi = a.getMax([]);
  for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], lo[k]); max[k] = Math.max(max[k], hi[k]); }
}
// markers: position and the marker's local z axis (AC cars spawn facing it)
const r3 = x => +x.toFixed(3);
const markerOut = Object.fromEntries(markers.map(({ name, M }) => {
  const W = M;
  const pos = new THREE.Vector3().setFromMatrixPosition(W);
  const z = new THREE.Vector3(0, 0, 1).transformDirection(W);
  return [name, { pos: pos.toArray().map(r3), dir: z.toArray().map(r3) }];
}));
scene.setExtras({ source: path.basename(inPath), surfaces: Object.keys(surfaces), bounds: { min, max }, markers: markerOut });
await io.write(outPath, doc);
// detail textures: webp, max 1024, with their mean linear colour (the shader divides by it, so the detail
// only adds texture and keeps the base map's brightness)
fs.mkdirSync(outDir, { recursive: true });
const toLin = c => Math.pow(c / 255, 2.2);
for (const [name, file] of details) {
  const img = sharp(png.get(name)).resize(1024, 1024, { fit: 'inside' });
  await img.clone().webp({ quality: 88 }).toFile(path.join(outDir, file));
  const st = await img.clone().removeAlpha().stats();
  const mean = st.channels.slice(0, 3).map(c => +toLin(c.mean).toFixed(4));
  for (const mt of doc.getRoot().listMaterials()) {
    const ex = mt.getExtras(); if (ex.detail?.file === file) ex.detail.mean = mean;
  }
}
await io.write(outPath, doc);
fs.rmSync(tmp, { recursive: true, force: true });
let prims = 0; for (const m of doc.getRoot().listMeshes()) prims += m.listPrimitives().length;
console.log(`${meshes.length} meshes, ${tris | 0} triangles -> ${prims} draw calls; surfaces ${JSON.stringify(surfaces)}; bounds ${min.map(x => x.toFixed(1))} .. ${max.map(x => x.toFixed(1))}; ${(fs.statSync(outPath).size / 1e6).toFixed(1)} MB`);
