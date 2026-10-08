import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { SURFACES } from '../vehicle/tire.js';
import { GROUP_GROUND } from '../vehicle/Vehicle.js';

const SURFACE_OF = { ROAD: 'asphalt', KERB: 'kerb', GRASS: 'grass', SAND: 'sand', WALL: 'concrete', WAL: 'concrete' };
const GROUND = new Set(['ROAD', 'KERB', 'GRASS', 'SAND']);

const loader = new GLTFLoader();
const texLoader = new THREE.TextureLoader();

function addDetail(m, dir, aniso) {
  const d = m.userData?.detail;
  if (!d) return;
  const t = texLoader.load(dir + d.file);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = aniso;
  const uniforms = { uDetail: { value: t }, uDetailScale: { value: 1 / d.scale }, uDetailMean: { value: new THREE.Vector3(...(d.mean || [0.25, 0.25, 0.25])) } };
  m.onBeforeCompile = sh => {
    Object.assign(sh.uniforms, uniforms);
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec2 vTrXZ;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvTrXZ = (modelMatrix * vec4(transformed, 1.0)).xz;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\nvarying vec2 vTrXZ;\nuniform sampler2D uDetail;\nuniform float uDetailScale;\nuniform vec3 uDetailMean;')
      .replace('#include <map_fragment>', '#include <map_fragment>\ndiffuseColor.rgb *= clamp(texture2D(uDetail, vTrXZ * uDetailScale).rgb / uDetailMean, 0.0, 2.5);');
  };
  m.customProgramCacheKey = () => 'track-detail';
}

export async function buildTrack(url, { RAPIER, world, colliderSurface, renderer, trees = false }) {
  const gltf = await loader.loadAsync(url);
  const root = gltf.scene;
  const info = root.userData || {};
  const aniso = Math.min(8, renderer.capabilities.getMaxAnisotropy());

  const drop = [];
  root.traverse(o => { if (o.isMesh && (/^AC_/.test(o.name) || (!trees && /^TREE/.test(o.material.name)))) drop.push(o); });
  for (const o of drop) { o.removeFromParent(); o.geometry.dispose(); }
  root.updateMatrixWorld(true);
  const dir = url.replace(/\.glb$/, '/');

  const ground = new Set();
  const v = new THREE.Vector3();
  root.traverse(o => {
    if (!o.isMesh) return;
    const m = o.material;
    for (const t of [m.map, m.normalMap]) if (t) { t.anisotropy = aniso; t.needsUpdate = true; }
    if (!m.userData.detailDone) { addDetail(m, dir, aniso); m.userData.detailDone = true; }
    o.receiveShadow = true;
    o.castShadow = m.alphaTest > 0 || !m.transparent;
  });

  for (const g of root.children) {
    const key = /^phys_(\w+)/.exec(g.name)?.[1];
    if (!key) continue;
    const surf = SURFACES[SURFACE_OF[key] || 'concrete'];
    g.traverse(o => {
      if (!o.isMesh) return;
      const geo = o.geometry, pos = geo.attributes.position;
      const verts = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) { v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld); v.toArray(verts, i * 3); }
      const idx = geo.index ? Uint32Array.from(geo.index.array) : Uint32Array.from({ length: pos.count }, (_, i) => i);
      const desc = RAPIER.ColliderDesc.trimesh(verts, idx).setFriction(GROUND.has(key) ? 0.9 : 0.4).setRestitution(0.05);
      if (GROUND.has(key)) desc.setCollisionGroups(GROUP_GROUND);
      const col = world.createCollider(desc);
      colliderSurface.set(col.handle, surf);
      if (GROUND.has(key)) ground.add(col.handle);
    });
  }

  world.step();

  const ray = new RAPIER.Ray({ x: 0, y: 600, z: 0 }, { x: 0, y: -1, z: 0 });
  const isGround = c => ground.has(c.handle);
  const heightAt = (x, z) => {
    ray.origin.x = x; ray.origin.y = 600; ray.origin.z = z;
    const hit = world.castRay(ray, 1200, true, undefined, undefined, undefined, undefined, isGround);
    return hit ? 600 - hit.timeOfImpact : -50;
  };
  const surfaceAt = () => SURFACES.grass;
  const terrain = { heightAt, surfaceAt, waterLevelAt: () => -Infinity, trailCurves: [] };

  const markers = info.markers || {};
  const spot = (name, label, tag, desc) => {
    const m = markers[name];
    if (!m) return null;
    return { name: label, tag, title: label, desc, x: m.pos[0], z: m.pos[2], yaw: Math.atan2(-m.dir[0], -m.dir[2]) };
  };
  const teleports = [
    spot('AC_START_0', 'Start line', 'Grid', 'The start of the course, on the grid.'),
    spot('AC_HOTLAP_START_0', 'Hotlap start', 'Grid', 'A rolling start just before the line.'),
    spot('AC_PIT_0', 'Pit area', 'Paddock', 'The paddock. Room to warm up the tyres.'),
  ].filter(Boolean);

  return { group: root, terrain, teleports, info, spawn: teleports[0] || { x: 0, z: 0, yaw: 0 } };
}
