// Minimal GLB writer: triangle meshes (POSITION, NORMAL, TEXCOORD_0, uint32 indices), one node each,
// materials as plain pbrMetallicRoughness with `extras` carrying whatever the caller wants.

export function writeGlb({ meshes, materials, extras = {} }) {
  const chunks = [];
  let offset = 0;
  const bufferViews = [], accessors = [];

  function addView(typed, target) {
    const bytes = new Uint8Array(typed.buffer, typed.byteOffset, typed.byteLength);
    const pad = (4 - (bytes.byteLength % 4)) % 4;
    bufferViews.push({ buffer: 0, byteOffset: offset, byteLength: bytes.byteLength, target });
    chunks.push(bytes);
    if (pad) chunks.push(new Uint8Array(pad));
    offset += bytes.byteLength + pad;
    return bufferViews.length - 1;
  }
  function addAccessor(typed, type, componentType, target, withBounds) {
    const n = { VEC3: 3, VEC2: 2, SCALAR: 1 }[type];
    const acc = { bufferView: addView(typed, target), componentType, count: typed.length / n, type };
    if (withBounds) {
      const min = Array(n).fill(Infinity), max = Array(n).fill(-Infinity);
      for (let i = 0; i < typed.length; i += n) for (let a = 0; a < n; a++) {
        const v = typed[i + a]; if (v < min[a]) min[a] = v; if (v > max[a]) max[a] = v;
      }
      acc.min = min; acc.max = max;
    }
    accessors.push(acc);
    return accessors.length - 1;
  }

  const gltfMeshes = [], nodes = [];
  for (const m of meshes) {
    const attributes = {
      POSITION: addAccessor(m.pos, 'VEC3', 5126, 34962, true),
      NORMAL: addAccessor(m.nrm, 'VEC3', 5126, 34962),
      TEXCOORD_0: addAccessor(m.uv, 'VEC2', 5126, 34962),
    };
    const indices = addAccessor(m.idx, 'SCALAR', 5125, 34963);
    gltfMeshes.push({ name: m.name, primitives: [{ attributes, indices, material: m.material }] });
    nodes.push({ name: m.name, mesh: gltfMeshes.length - 1, extras: m.extras });
  }

  const json = {
    asset: { version: '2.0', generator: 'onroad kn5 converter' },
    scene: 0,
    scenes: [{ nodes: nodes.map((_, i) => i), extras }],
    nodes, meshes: gltfMeshes,
    materials: materials.map((m) => ({
      name: m.name,
      pbrMetallicRoughness: { baseColorFactor: [0.8, 0.8, 0.8, 1], metallicFactor: 0, roughnessFactor: 0.8 },
      doubleSided: true,
      extras: m.extras,
    })),
    accessors, bufferViews,
    buffers: [{ byteLength: offset }],
  };

  let jsonBytes = new TextEncoder().encode(JSON.stringify(json));
  const jsonPad = (4 - (jsonBytes.length % 4)) % 4;
  const jsonChunk = new Uint8Array(jsonBytes.length + jsonPad).fill(0x20);
  jsonChunk.set(jsonBytes);
  const total = 12 + 8 + jsonChunk.length + 8 + offset;
  const out = new Uint8Array(total);
  const dv = new DataView(out.buffer);
  dv.setUint32(0, 0x46546c67, true); dv.setUint32(4, 2, true); dv.setUint32(8, total, true);
  dv.setUint32(12, jsonChunk.length, true); dv.setUint32(16, 0x4e4f534a, true);
  out.set(jsonChunk, 20);
  let p = 20 + jsonChunk.length;
  dv.setUint32(p, offset, true); dv.setUint32(p + 4, 0x004e4942, true); p += 8;
  for (const c of chunks) { out.set(c, p); p += c.byteLength; }
  return out;
}
