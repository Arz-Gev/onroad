import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Sky } from 'three/examples/jsm/objects/Sky.js';
import { loadAcTrack } from './world/acTrack.js';

const q = new URLSearchParams(location.search);
const id = q.get('track') || 'ebisu_north';
const info = document.getElementById('info');

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.NeutralToneMapping;
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 5000);
const sky = new Sky(); sky.scale.setScalar(4000);
const sunDir = new THREE.Vector3().setFromSphericalCoords(1, THREE.MathUtils.degToRad(55), THREE.MathUtils.degToRad(140));
sky.material.uniforms.sunPosition.value.copy(sunDir);
scene.add(sky);
scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x5a5040, 1.1));
const sun = new THREE.DirectionalLight(0xfff2dd, 2.6);
scene.add(sun, sun.target);
scene.fog = new THREE.Fog(0xb8c8d8, 300, 2500);

const controls = new OrbitControls(camera, renderer.domElement);
addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });

const t0 = performance.now();
const track = await loadAcTrack(id, { renderer });
scene.add(track.scene);
const s = track.info.spawns.AC_HOTLAP_START_0 || track.info.spawns.AC_START_0 || { pos: [0, 0, 0], forward: [0, 0, 1] };
const p = new THREE.Vector3(...s.pos), f = new THREE.Vector3(...s.forward);
const view = q.get('view') || 'start';
if (view === 'top') {
  const box = new THREE.Box3();
  for (const ph of track.physical) if (ph.surface === 'ROAD') box.expandByObject(ph.mesh);
  scene.fog = null;
  const c = box.getCenter(new THREE.Vector3());
  camera.position.set(c.x, c.y + Math.max(box.max.x - box.min.x, box.max.z - box.min.z) * 0.9, c.z + 1);
  controls.target.copy(c);
} else {
  camera.position.copy(p).addScaledVector(f, -9).add(new THREE.Vector3(0, 3.2, 0));
  controls.target.copy(p).addScaledVector(f, 12).add(new THREE.Vector3(0, 0.5, 0));
}
sun.position.copy(p).addScaledVector(sunDir, 300); sun.target.position.copy(p);
sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -120, right: 120, top: 120, bottom: -120, far: 800 });
controls.update();

let tris = 0; track.scene.traverse((o) => { if (o.isMesh && o.visible) tris += o.geometry.index.count / 3; });
info.textContent = `${track.info.name} by ${track.info.author || '?'}: ${Math.round(tris / 1000)}k tris, loaded in ${((performance.now() - t0) / 1000).toFixed(1)} s`;
window.viewer = { scene, camera, renderer, track, ready: true };
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
