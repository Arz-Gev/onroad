// Assetto Corsa surfaces (SurfaceDef) by KEY. ROAD and WALL are AC's defaults (system/data/surfaces.ini); KERB,
// GRASS and SAND are Ebisu North's own data/surfaces.ini. A track mesh named <digit><KEY>... is that surface.
//   gripMod: FRICTION, damping: DAMPING (a drag on the car body, x its mass, per m/s), dirtAdditiveK: DIRT_ADDITIVE,
//   sinHeight / sinLength: the bumps the tyre feels (SIN_HEIGHT, SIN_LENGTH), vibration*: the force feedback rumble.
import { defaultSurface } from './tyre.js';

const def = o => ({ ...defaultSurface(), ...o });

export const AC_SURFACES = {
  ROAD: def({ name: 'ROAD', gripMod: 1, isValidTrack: true }),
  KERB: def({ name: 'KERB', gripMod: 0.92, isValidTrack: true, vibrationGain: 0.1, vibrationLength: 0.2 }),
  GRASS: def({ name: 'GRASS', gripMod: 0.6, isValidTrack: false, sinHeight: 0.03, sinLength: 0.5, vibrationGain: 0.2, vibrationLength: 0.6 }),
  SAND: def({ name: 'SAND', gripMod: 0.9, damping: 0.05, dirtAdditiveK: 0.5, isValidTrack: true, sinHeight: 0.01, sinLength: 2, vibrationGain: 0.1, vibrationLength: 0.2 }),
  WALL: def({ name: 'WALL', gripMod: 1, isValidTrack: false }),
};
AC_SURFACES.WAL = AC_SURFACES.WALL;

export const acSurface = key => AC_SURFACES[key] || AC_SURFACES.ROAD;
