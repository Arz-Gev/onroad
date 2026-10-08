import * as THREE from 'three';
import { installCascadeShadowChunks } from './cascadeShadow.js';

export const ATMO = {
  atmoSun: { x: 0, y: 1, z: 0, w: 0.02 },

  atmoAway: { x: 0.5, y: 0.6, z: 0.7, w: 0 },

  atmoSide: { x: 0.5, y: 0.6, z: 0.7, w: 0.0003 },

  atmoToward: { x: 0.6, y: 0.6, z: 0.6, w: 1 },

  atmoGlow: { x: 0, y: 0, z: 0, w: 0.7 },
};

export function atmosphereUniforms() {
  const u = {};
  for (const k in ATMO) u[k] = { value: ATMO[k] };
  return u;
}

export const setVec4 = (o, x, y, z, w) => { o.x = x; o.y = y; o.z = z; if (w !== undefined) o.w = w; };

const FOG_PARS_VERTEX =`
#ifdef USE_FOG
	varying float vFogDepth;
	varying vec3 vFogWorldDir;
#endif
`;

const FOG_VERTEX =`
#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
	vFogWorldDir = ( vec4( mvPosition.xyz, 0.0 ) * viewMatrix ).xyz;
#endif
`;

const FOG_PARS_FRAGMENT =`
#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	varying vec3 vFogWorldDir;
	#ifdef FOG_EXP2
		uniform float fogDensity;
		uniform vec4 atmoSun;
		uniform vec4 atmoAway;
		uniform vec4 atmoSide;
		uniform vec4 atmoToward;
		uniform vec4 atmoGlow;
		float atmoFogFactor( vec3 d ) {
			float dist = length( d );
			float b = atmoSun.w;
			float dens0 = fogDensity * exp( - b * ( cameraPosition.y - atmoAway.w ) );
			float dy = clamp( d.y * b, - 40.0, 40.0 );
			float hf = abs( dy ) > 1e-3 ? ( 1.0 - exp( - dy ) ) / dy : 1.0 - 0.5 * dy;
			float od = dist * ( min( dens0 * hf, 0.2 ) + atmoSide.w );
			return min( 1.0 - exp( - od ), atmoToward.w );
		}
		vec3 atmoFogColor( vec3 v ) {
			vec2 hz = normalize( v.xz + vec2( 1e-5 ) );
			vec2 sz = normalize( atmoSun.xz + vec2( 1e-5 ) );
			float a = dot( hz, sz );
			vec3 col = a > 0.0 ? mix( atmoSide.rgb, atmoToward.rgb, a ) : mix( atmoSide.rgb, atmoAway.rgb, - a );
			float g = atmoGlow.w;
			float mu = dot( v, atmoSun.xyz );
			float hg = ( 1.0 - g * g ) / pow( max( 1.0 + g * g - 2.0 * g * mu, 1e-4 ), 1.5 );
			return col + atmoGlow.rgb * hg * 0.0795775;
		}
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif
`;

const FOG_FRAGMENT =`
#ifdef USE_FOG
	#ifdef FOG_EXP2
		{
			vec3 fogD = vFogWorldDir;
			float fogFactor = atmoFogFactor( fogD );
			vec3 fogCol = atmoFogColor( fogD / max( length( fogD ), 1e-4 ) );
			gl_FragColor.rgb = mix( gl_FragColor.rgb, fogCol, fogFactor );
		}
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
		gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
	#endif
#endif
`;

let installed = false;
export function installShaderPatches() {
  if (installed) return;
  installed = true;
  const C = THREE.ShaderChunk;
  C.fog_pars_vertex = FOG_PARS_VERTEX;
  C.fog_vertex = FOG_VERTEX;
  C.fog_pars_fragment = FOG_PARS_FRAGMENT;
  C.fog_fragment = FOG_FRAGMENT;
  installCascadeShadowChunks();

  for (const k in THREE.ShaderLib) {
    const u = THREE.ShaderLib[k].uniforms;
    if (u && 'fogDensity' in u) Object.assign(u, atmosphereUniforms());
  }
  THREE.UniformsLib.fog = { ...THREE.UniformsLib.fog, ...atmosphereUniforms() };

  let lf = C.lights_fragment_begin;
  const pointRE = /(\t\tgetPointLightInfo\( pointLight, geometryPosition, directLight \);\n)([\s\S]*?)(\t\tRE_Direct\( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight \);\n)/;
  const spotRE = /(\t\tgetSpotLightInfo\( spotLight, geometryPosition, directLight \);\n)([\s\S]*?)(\t\tRE_Direct\( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight \);\n)/;
  const wrap = (m, a, body, re) => `${a}\t\tif ( directLight.visible ) {\n${body}${re}\t\t}\n`;
  if (pointRE.test(lf) && spotRE.test(lf)) {
    lf = lf.replace(pointRE, wrap).replace(spotRE, wrap);

    C.lights_fragment_begin = lf;
  } else {
    console.warn('shaderPatches: lights_fragment_begin layout changed, light skipping disabled');
  }
}
