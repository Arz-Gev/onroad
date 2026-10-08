import * as THREE from 'three';

export function createTireMaterial(map) {
  const mat = new THREE.MeshStandardMaterial({ color: 0xffffff, map, roughness: 0.86, metalness: 0.0 });
  const uniforms = {
    uPlaneP: { value: new THREE.Vector3(0, -10, 0) },
    uPlaneN: { value: new THREE.Vector3(0, 1, 0) },
    uDefl: { value: 0 },
  };
  mat.userData.uniforms = uniforms;
  mat.onBeforeCompile = (sh) => {
    Object.assign(sh.uniforms, uniforms);
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
uniform vec3 uPlaneP;
uniform vec3 uPlaneN;
uniform float uDefl;`)
      .replace('#include <beginnormal_vertex>', `#include <beginnormal_vertex>
vec3 defPos = position;
{
  float d = dot(defPos - uPlaneP, uPlaneN);
  float near = 1.0 - smoothstep(0.0, 0.2, max(d, 0.0));
  float ax = defPos.x;
  float side = smoothstep(0.05, 0.13, abs(ax));
  float r = length(defPos.yz);
  float wall = 1.0 - smoothstep(0.36, 0.415, r);
  float bulge = uDefl * 0.6 * near * side * (0.35 + 0.65 * wall);
  defPos.x += sign(ax) * bulge;
  float flt = 0.0;
  if (d < 0.0) { defPos -= uPlaneN * d; flt = 1.0; }
  objectNormal = normalize(mix(objectNormal, uPlaneN, flt * 0.85) + vec3(sign(ax) * bulge * 4.0, 0.0, 0.0));
}`)
      .replace('#include <begin_vertex>', 'vec3 transformed = defPos;');
  };
  mat.customProgramCacheKey = () => 'tire-deform-v1';
  return mat;
}
