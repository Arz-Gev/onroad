// Assetto Corsa .kn5 reader (model files for cars and tracks).
//
// Layout (little endian, strings are int32 length + UTF-8):
//   "sc6969" magic, int32 version, int32 extra (version > 5)
//   textures:  int32 count, each { int32 active, string name, int32 size, bytes (DDS/PNG/JPG) }
//   materials: int32 count, each { string name, string shader, u8 blend, u8 alphaTested,
//              int32 depthMode, int32 nProps × { string, f32 A, vec2 B, vec3 C, vec4 D },
//              int32 nSamplers × { string slotName, int32 slot, string texture } }
//   nodes:     one root, recursive { int32 class, string name, int32 nChildren, u8 active, body, children }
//              class 1 = transform (4×4, row-vector DirectX layout = three.js column-major array)
//              class 2 = mesh, class 3 = skinned mesh
// Vertices: position 3f, normal 3f, uv 2f, tangent 3f (44 bytes); skinned adds 4 weights + 4 bone indices (f32).

export function parseKn5(buf, { keepTextureData = true } = {}) {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let p = 0;
  const td = new TextDecoder();
  const u8 = () => dv.getUint8(p++);
  const i32 = () => { const v = dv.getInt32(p, true); p += 4; return v; };
  const f32 = () => { const v = dv.getFloat32(p, true); p += 4; return v; };
  const str = () => { const n = i32(); const s = td.decode(buf.subarray(p, p + n)); p += n; return s; };
  const floats = (n) => { const a = new Float32Array(n); for (let i = 0; i < n; i++) a[i] = f32(); return a; };

  const magic = td.decode(buf.subarray(0, 6)); p = 6;
  if (magic !== 'sc6969') throw new Error(`not a kn5 (magic ${JSON.stringify(magic)})`);
  const version = i32();
  if (version > 5) i32();

  const textures = [];
  for (let n = i32(), i = 0; i < n; i++) {
    const active = i32(); const name = str(); const size = i32();
    textures.push({ active, name, size, data: keepTextureData ? buf.subarray(p, p + size) : null });
    p += size;
  }

  const materials = [];
  for (let n = i32(), i = 0; i < n; i++) {
    const m = { name: str(), shader: str(), blend: u8(), alphaTested: u8(), depthMode: i32(), props: {}, samplers: {} };
    for (let k = i32(), j = 0; j < k; j++) {
      const name = str();
      m.props[name] = { A: f32(), B: floats(2), C: floats(3), D: floats(4) };
    }
    for (let k = i32(), j = 0; j < k; j++) {
      const slotName = str(); const slot = i32(); const texture = str();
      m.samplers[slotName] = { slot, texture };
    }
    materials.push(m);
  }

  function readMesh(skinned) {
    const mesh = { castShadows: u8(), visible: u8(), transparent: u8() };
    if (skinned) {
      mesh.bones = [];
      for (let n = i32(), i = 0; i < n; i++) mesh.bones.push({ name: str(), matrix: floats(16) });
    }
    const nv = i32();
    const stride = skinned ? 76 : 44;
    // Copy vertex data out (the source offset is not 4-byte aligned in general).
    const raw = new Float32Array(nv * (stride / 4));
    new Uint8Array(raw.buffer).set(buf.subarray(p, p + nv * stride));
    p += nv * stride;
    const pos = new Float32Array(nv * 3), nrm = new Float32Array(nv * 3), uv = new Float32Array(nv * 2);
    const s = stride / 4;
    for (let i = 0; i < nv; i++) {
      const o = i * s;
      pos[i * 3] = raw[o]; pos[i * 3 + 1] = raw[o + 1]; pos[i * 3 + 2] = raw[o + 2];
      nrm[i * 3] = raw[o + 3]; nrm[i * 3 + 1] = raw[o + 4]; nrm[i * 3 + 2] = raw[o + 5];
      uv[i * 2] = raw[o + 6]; uv[i * 2 + 1] = raw[o + 7];
    }
    const ni = i32();
    const idx = new Uint16Array(ni);
    for (let i = 0; i < ni; i++) { idx[i] = dv.getUint16(p, true); p += 2; }
    Object.assign(mesh, { pos, nrm, uv, idx, material: i32(), layer: i32(), lodIn: f32(), lodOut: f32() });
    if (!skinned) {
      mesh.sphere = floats(4);
      mesh.renderable = u8();
    }
    return mesh;
  }

  function readNode() {
    const node = { cls: i32(), name: str(), children: [] };
    const nChildren = i32();
    node.active = u8();
    if (node.cls === 1) node.matrix = floats(16);
    else if (node.cls === 2) node.mesh = readMesh(false);
    else if (node.cls === 3) node.mesh = readMesh(true);
    else throw new Error(`unknown node class ${node.cls} at ${p} (${node.name})`);
    for (let i = 0; i < nChildren; i++) node.children.push(readNode());
    return node;
  }

  const root = readNode();
  return { version, textures, materials, root, bytesRead: p, bytesTotal: buf.byteLength };
}

export function walk(node, fn, parent = null, depth = 0) {
  fn(node, parent, depth);
  for (const c of node.children) walk(c, fn, node, depth + 1);
}
