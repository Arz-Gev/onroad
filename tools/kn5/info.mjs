import { readFileSync } from 'node:fs';
import { parseKn5, walk } from './kn5.mjs';

const file = process.argv[2];
const showTree = process.argv.includes('--tree');
const showMats = process.argv.includes('--materials');
const k = parseKn5(readFileSync(file));

let meshes = 0, skinned = 0, tris = 0, transforms = 0;
walk(k.root, (n, _p, d) => {
  if (n.cls === 1) transforms++;
  if (n.mesh) { meshes++; if (n.cls === 3) skinned++; tris += n.mesh.idx.length / 3; }
  if (showTree) {
    const extra = n.mesh ? ` [${n.mesh.idx.length / 3} tris, mat ${k.materials[n.mesh.material]?.name}]`
      : n.matrix ? ` @ ${[...n.matrix.slice(12, 15)].map((v) => v.toFixed(3)).join(', ')}` : '';
    console.log(`${'  '.repeat(d)}${n.name}${n.active ? '' : ' (inactive)'}${extra}`);
  }
});

const fmt = {};
let texBytes = 0;
for (const t of k.textures) {
  const d = t.data;
  let f = 'other';
  if (d && d[0] === 0x44 && d[1] === 0x44 && d[2] === 0x53) {
    const fourcc = String.fromCharCode(d[84], d[85], d[86], d[87]);
    f = /^[A-Z0-9]{4}$/.test(fourcc) ? `DDS ${fourcc}` : `DDS raw${new DataView(d.buffer, d.byteOffset).getUint32(88, true)}bpp`;
  } else if (d && d[0] === 0x89 && d[1] === 0x50) f = 'PNG';
  else if (d && d[0] === 0xff && d[1] === 0xd8) f = 'JPG';
  fmt[f] = (fmt[f] || 0) + 1;
  texBytes += t.size;
}

console.log({
  file, version: k.version, parsedToEnd: k.bytesRead === k.bytesTotal, bytesRead: k.bytesRead, bytesTotal: k.bytesTotal,
  textures: k.textures.length, textureMB: +(texBytes / 1e6).toFixed(1), textureFormats: fmt,
  materials: k.materials.length, transforms, meshes, skinned, tris,
});
if (showMats) for (const m of k.materials) console.log(m.name, m.shader, Object.fromEntries(Object.entries(m.samplers).map(([s, v]) => [s, v.texture])));
