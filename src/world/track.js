import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { SURFACES } from '../vehicle/tire.js';
import { GROUP_GROUND } from '../vehicle/Vehicle.js';

// A track converted from Assetto Corsa by tools/kn5glb.mjs. The GLB has a `visual` group and one
// `phys_<SURFACE>` group per AC physics surface (1ROAD_x, 1GRASS_x, 1KERB, 1WALL_x...). The physics groups
// are rendered too and become fixed trimesh colliders; ground surfaces go into the GROUND collision group
// (the wheel side cylinders ignore it, see Vehicle.js), walls stay in the default group so the wheels and
// the chassis hit them. Scene extras carry the AC markers (AC_START_n, AC_PIT_n...).
//
// Also returns a stand-in for the offroad Terrain (heightAt / surfaceAt / waterLevelAt) for the camera,
// spawning and the effects: heights come from a ray cast against the ground colliders.

const SURFACE_OF = { ROAD: 'asphalt', KERB: 'kerb', GRASS: 'grass', SAND: 'sand', WALL: 'concrete', WAL: 'concrete' };
const GROUND = new Set(['ROAD', 'KERB', 'GRASS', 'SAND']);

const loader = new GLTFLoader();
const texLoader = new THREE.TextureLoader();

// AC multilayer materials (road, grass, kerbs): the base map is a low-res colour map over the whole
// track; a detail texture tiled in world XZ gives the close-up grain. Divided by its mean colour so it
// only adds texture.
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
  // dropped: the AC marker boxes (AC_* meshes, invisible in AC; spawns come from the scene extras) and,
  // unless asked for, the tree cards (TREE.* materials). Both are visual only, no colliders.
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
  // colliders from the physics groups (vertices in world space; the converter baked all transforms)
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

  // scene queries see new colliders only after a step (the world holds nothing else yet); without it the
  // first heightAt (the spawn height) misses the ground and the car starts 50 m under the track
  world.step();

  // ground height under (x, z): the highest ground collider below y = 600
  const ray = new RAPIER.Ray({ x: 0, y: 600, z: 0 }, { x: 0, y: -1, z: 0 });
  const isGround = c => ground.has(c.handle);
  const heightAt = (x, z) => {
    ray.origin.x = x; ray.origin.y = 600; ray.origin.z = z;
    const hit = world.castRay(ray, 1200, true, undefined, undefined, undefined, undefined, isGround);
    return hit ? 600 - hit.timeOfImpact : -50;
  };
  const surfaceAt = () => SURFACES.grass;
  const terrain = { heightAt, surfaceAt, waterLevelAt: () => -Infinity, trailCurves: [] };

  // spawn points: AC cars spawn facing the marker's z axis; game yaw turns -z (the car's forward) to it
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

// ?map=flat: a 20 km square of asphalt at y = 0 with a 10 m grid painted on it, for testing the car (drift,
// handling) with no walls or kerbs in the way. Same return shape as buildTrack.
export function buildFlat({ RAPIER, world, colliderSurface }) {
  const size = 20000;
  const col = world.createCollider(RAPIER.ColliderDesc.cuboid(size / 2, 1, size / 2).setTranslation(0, -1, 0)
    .setFriction(0.9).setRestitution(0.05).setCollisionGroups(GROUP_GROUND));
  colliderSurface.set(col.handle, SURFACES.asphalt);

  // grid texture: dark asphalt, thin light lines every 10 m, a brighter one every 100 m
  const c = document.createElement('canvas');
  c.width = c.height = 1024;
  const g = c.getContext('2d');
  g.fillStyle = '#2b2b2d'; g.fillRect(0, 0, 1024, 1024);
  for (let i = 0; i < 10; i++) {
    g.fillStyle = i === 0 ? '#b8b8b0' : '#55555a';
    const w = i === 0 ? 6 : 3;
    g.fillRect(i * 102.4, 0, w, 1024); g.fillRect(0, i * 102.4, 1024, w);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(size / 100, size / 100);
  tex.anisotropy = 8;
  tex.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(size, size).rotateX(-Math.PI / 2),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 0.9 }));
  mesh.receiveShadow = true;
  const group = new THREE.Group();
  group.add(mesh);

  const terrain = { heightAt: () => 0, surfaceAt: () => SURFACES.asphalt, waterLevelAt: () => -Infinity, trailCurves: [] };
  const spawn = { name: 'Centre', tag: 'Flat', title: 'Centre', desc: 'The middle of the test pad.', x: 0, z: 0, yaw: 0 };
  return { group, terrain, teleports: [spawn], info: {}, spawn };
}
