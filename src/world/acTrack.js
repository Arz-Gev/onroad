import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DDSLoader } from 'three/examples/jsm/loaders/DDSLoader.js';

const COLOR_SLOTS = new Set(['txDiffuse', 'txDetail', 'txDetailR', 'txDetailG', 'txDetailB', 'txDetailA']);

export async function loadAcTrack(id, { base = 'tracks/', renderer, onProgress } = {}) {
  const root = `${base}${id}/`;
  const info = await (await fetch(`${root}track.json`)).json();
  const gltf = await new GLTFLoader().loadAsync(`${root}${info.glb}`, onProgress);

  const dds = new DDSLoader();
  const img = new THREE.TextureLoader();
  const cache = new Map();
  const aniso = renderer ? renderer.capabilities.getMaxAnisotropy() : 1;
  function tex(path, color) {
    const key = `${path}|${color}`;
    if (!cache.has(key)) {
      const t = path.endsWith('.png') ? img.load(`${root}${path}`) : dds.load(`${root}${path}`);
      t.flipY = false;
      t.wrapS = t.wrapT = THREE.RepeatWrapping;
      t.colorSpace = color ? THREE.SRGBColorSpace : THREE.NoColorSpace;
      t.anisotropy = Math.min(8, aniso);
      cache.set(key, t);
    }
    return cache.get(key);
  }

  const mats = new Map();
  function material(src) {
    if (mats.has(src)) return mats.get(src);
    const x = src.userData;
    const p = x.props || {};
    const T = x.textures || {};
    const m = new THREE.MeshStandardMaterial({ name: src.name, roughness: 0.85, metalness: 0, side: THREE.DoubleSide });
    if (T.txDiffuse) m.map = tex(T.txDiffuse, COLOR_SLOTS.has('txDiffuse'));
    const em = p.ksEmissive ? p.ksEmissive.slice(3, 6) : [0, 0, 0];
    if (em.some((v) => v > 0)) {
      m.emissive.setRGB(...em.map((v) => Math.min(v, 4)));
      if (m.map) m.emissiveMap = m.map;
    }
    if (x.alphaTested) { m.alphaTest = p.ksAlphaRef?.[0] > 0 ? p.ksAlphaRef[0] : 0.5; }
    if (x.blend === 1) {
      m.transparent = true; m.depthWrite = false;
      m.polygonOffset = true; m.polygonOffsetFactor = -1; m.polygonOffsetUnits = -4;
    }
    if (/^ksTree/.test(x.shader)) { m.alphaTest = Math.max(m.alphaTest, 0.5); m.roughness = 1; }
    if (/^ksMultilayer/.test(x.shader)) addMultilayer(m, T, p, tex);
    m.userData.ac = x;
    mats.set(src, m);
    return m;
  }

  const scene = gltf.scene;
  const physical = [];
  scene.traverse((o) => {
    if (!o.isMesh) return;
    const e = o.userData;
    o.material = material(o.material);
    o.visible = e.visible !== false;
    o.castShadow = o.receiveShadow = o.visible && !o.material.transparent;
    o.matrixAutoUpdate = false; o.updateMatrix();
    if (e.surface) physical.push({ mesh: o, surface: e.surface, physics: info.surfaces[e.surface]?.physics || 'asphalt', friction: info.surfaces[e.surface]?.friction ?? 1 });
  });

  return { info, scene, physical };
}

function addMultilayer(m, T, p, tex) {
  if (!T.txMask) return;
  const slots = ['R', 'G', 'B', 'A'];
  const uniforms = { acMask: { value: tex(T.txMask, false) } };
  for (const s of slots) {
    uniforms[`acDetail${s}`] = { value: T[`txDetail${s}`] ? tex(T[`txDetail${s}`], true) : null };
    uniforms[`acMult${s}`] = { value: p[`mult${s}`]?.[0] || 1 };
  }
  m.onBeforeCompile = (sh) => {
    Object.assign(sh.uniforms, uniforms);
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <map_pars_fragment>', `#include <map_pars_fragment>
uniform sampler2D acMask, acDetailR, acDetailG, acDetailB, acDetailA;
uniform float acMultR, acMultG, acMultB, acMultA;`)
      .replace('#include <map_fragment>', `#include <map_fragment>
{
  vec4 mk = texture2D(acMask, vMapUv);
  float wsum = max(mk.r + mk.g + mk.b + mk.a, 1e-3);
  vec3 det = (texture2D(acDetailR, vMapUv * acMultR).rgb * mk.r + texture2D(acDetailG, vMapUv * acMultG).rgb * mk.g
            + texture2D(acDetailB, vMapUv * acMultB).rgb * mk.b + texture2D(acDetailA, vMapUv * acMultA).rgb * mk.a) / wsum;
  diffuseColor.rgb *= det * 2.0;
}`);
  };
  m.customProgramCacheKey = () => 'acMultilayer';
}
