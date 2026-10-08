import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { patch } from './truckMaterials.js';
import { buildLightRig } from './truckLights.js';

export const WHEEL_KEYS = ['FL', 'FR', 'RL', 'RR'];

const LOOKS = {
  BODY1: m => new THREE.MeshPhysicalMaterial({
    color: m.color, roughness: 0.32, metalness: 0.0, specularIntensity: 0.4, clearcoat: 0.6, clearcoatRoughness: 0.08,
    side: m.side,
  }),
};

const LAMPS = {
  hlgl: ['head', 0xfff1de], hlmat: ['head', 0xfff1de], hlmat1: ['head', 0xfff1de], material_42: ['head', 0xfff1de],
  tlgl: ['tail', 0xff1a08], material_36: ['tail', 0xff1a08], lic_li: ['tail', 0xfff4e0],
  revgl: ['reverse', 0xffffff],
  signal: ['amber', 0xff8a10],
};

const loader = new GLTFLoader();

export async function loadCarModel(url, opts = {}) {
  const gltf = await loader.loadAsync(url);
  const src = gltf.scene;
  const info = src.userData || {};

  const root = new THREE.Group();
  root.name = info.car || 'car';
  const body = src.getObjectByName('body');
  root.add(body);

  const mats = new Map();
  const lamps = { head: [], tail: [], reverse: [], amber: [] };
  const fix = m => {
    if (mats.has(m)) return mats.get(m);
    let out = LOOKS[m.name] ? LOOKS[m.name](m) : m;
    out.name = m.name;
    const glass = m.transparent || m.alphaMode === 'BLEND';
    if (!glass) patch(out, { env: m.name === 'BODY1' ? 0.6 : 1.0, dirt: m.name === 'BODY1' ? 0.25 : 0 });
    const lamp = LAMPS[m.name];
    if (lamp) {
      out.emissive = new THREE.Color(lamp[1]);
      out.emissiveIntensity = 0;
      lamps[lamp[0]].push(out);
    }
    mats.set(m, out);
    return out;
  };
  const prep = o => {
    if (!o.isMesh) return;
    o.material = fix(o.material);
    const glass = o.material.transparent;
    o.castShadow = !glass;
    o.receiveShadow = !glass;
  };
  body.traverse(prep);

  const hubs = info.hubs || {};
  const wheels = WHEEL_KEYS.map(k => {
    const hub = new THREE.Group(), steer = new THREE.Group(), pose = new THREE.Group(), spin = new THREE.Group();
    hub.name = 'hub_' + k;
    hub.add(steer); steer.add(pose); pose.add(spin);
    const w = src.getObjectByName('wheel_' + k), c = src.getObjectByName('caliper_' + k);
    for (const g of [w, c]) if (g) { g.traverse(prep); g.position.set(0, 0, 0); }
    if (w) spin.add(w);
    if (c) pose.add(c);
    if (info.pose?.[k]) pose.quaternion.fromArray(info.pose[k]);
    const rest = new THREE.Vector3().fromArray(hubs[k] || [0, 0, 0]);
    hub.position.copy(rest);
    root.add(hub);
    return { key: k, hub, steer, pose, spin, rest };
  });

  root.traverse(o => { if (o.isMesh) o.frustumCulled = true; });

  const lights = buildLightRig(root, opts.anchors || { head: [0, 0.6, -2.5], bar: [0, 1.3, -0.4], rear: [0, 0.7, 2.4] });

  return {
    root, body, wheels, lamps, lights, info,
    driverEye: new THREE.Vector3(...(opts.driverEye || [0.36, 1.0, 0.25])),
  };
}
