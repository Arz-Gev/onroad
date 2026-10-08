export const SURFACES = {
  dirt:     { name: 'Dirt',     mu: 0.72, crr: 0.026, kPeak: 0.15, aPeak: 0.16, C: 1.35, soft: 0.5, dust: 1.0, color: [0.55, 0.45, 0.33] },
  grass:    { name: 'Grass',    mu: 0.58, crr: 0.034, kPeak: 0.14, aPeak: 0.15, C: 1.40, soft: 0.4, dust: 0.25, color: [0.36, 0.40, 0.24] },
  rock:     { name: 'Rock',     mu: 0.98, crr: 0.013, kPeak: 0.11, aPeak: 0.13, C: 1.55, soft: 0.0, dust: 0.15, color: [0.5, 0.5, 0.5] },
  mud:      { name: 'Mud',      mu: 0.42, crr: 0.10,  kPeak: 0.30, aPeak: 0.22, C: 1.05, soft: 1.0, dust: 0.0, mud: 1, color: [0.25, 0.18, 0.11] },
  sand:     { name: 'Sand',     mu: 0.62, crr: 0.085, kPeak: 0.24, aPeak: 0.20, C: 1.15, soft: 1.0, dust: 1.2, color: [0.75, 0.66, 0.48] },
  wood:     { name: 'Wood',     mu: 0.68, crr: 0.012, kPeak: 0.10, aPeak: 0.13, C: 1.50, soft: 0.0, dust: 0.0, color: [0.4, 0.3, 0.2] },
  concrete: { name: 'Concrete', mu: 0.98, crr: 0.011, kPeak: 0.10, aPeak: 0.12, C: 1.60, soft: 0.0, dust: 0.2, color: [0.6, 0.6, 0.58] },

  asphalt:  { name: 'Asphalt',  mu: 1.05, crr: 0.011, kPeak: 0.10, aPeak: 0.105, C: 1.35, soft: 0.0, dust: 0.0, color: [0.2, 0.2, 0.21] },
  kerb:     { name: 'Kerb',     mu: 0.94, crr: 0.013, kPeak: 0.10, aPeak: 0.11, C: 1.35, soft: 0.0, dust: 0.0, color: [0.7, 0.2, 0.2] },
};
for (const s of Object.values(SURFACES)) s.B = Math.tan(Math.PI / (2 * s.C));

const FN_REF = 5200;

const LAT_LOAD_EXP = 0.35;
export const latPeak = (surf, Fn) => surf.aPeak * Math.pow(Math.max(0.25, Math.min(2.5, Fn / FN_REF)), LAT_LOAD_EXP);

export function tireRadialStiffness(psi) {
  return 46000 + 6300 * psi;
}

export function tireCoefs(tire, psi, surf, out) {
  const low = Math.max(0, Math.min(1, (30 - psi) / 24));

  const gripGain = 1 + low * (0.10 + 0.14 * surf.soft) + (surf === SURFACES.rock ? low * 0.08 : 0);
  out.mu = surf.mu * gripGain * (tire.grip ?? 1);

  const hard = 1 - surf.soft;
  out.crr = surf.crr * (hard * Math.sqrt(28 / Math.max(psi, 4)) + surf.soft * Math.pow(Math.max(psi, 4) / 28, 0.6));
  out.Lx = tire.relaxX * (1 + 0.6 * low);
  out.Ly = tire.relaxY * (1 + 0.7 * low);
  out.kt = tireRadialStiffness(psi);
  return out;
}

export function tireForces(w, Fn, vsx, vsy, speed, surf, co) {
  if (Fn <= 0) { w.Fx = 0; w.Fy = 0; w.slipNorm = 0; return; }
  const loadFactor = Math.max(0.75, Math.min(1.12, 1 - 0.14 * (Fn / FN_REF - 1)));
  const mu = co.mu * loadFactor * (w.camberGrip ?? 1);
  const sx = w.ux / (co.Lx * surf.kPeak);
  w.aPk = latPeak(surf, Fn);
  const sy = w.uy / (co.Ly * w.aPk);
  const s = Math.hypot(sx, sy);
  let Fx = 0, Fy = 0;
  if (s > 1e-9) {
    const F = mu * Fn * Math.sin(surf.C * Math.atan(surf.B * s));
    Fx = F * sx / s;
    Fy = 0.95 * F * sy / s;
  }

  const fade = Math.max(0, 1 - speed / 2.5);
  if (fade > 0 && s < 1) {
    const c = 2.6 * Fn * fade;
    Fx += c * vsx;
    Fy += c * vsy;
  }
  const lim = mu * Fn;
  const m = Math.hypot(Fx, Fy);
  if (m > lim) { Fx *= lim / m; Fy *= lim / m; }
  w.Fx = Fx; w.Fy = Fy;
  w.slipNorm = s;
}

export function tireRelax(w, h, vsx, vsy, avx, surf, co) {
  w.ux = (w.ux + h * vsx) / (1 + h * avx / co.Lx);
  w.uy = (w.uy + h * vsy) / (1 + h * avx / co.Ly);
  const sx = w.ux / (co.Lx * surf.kPeak);
  const aPk = w.aPk || surf.aPeak;
  const sy = w.uy / (co.Ly * aPk);
  const s = Math.hypot(sx, sy);
  const vref = Math.max(avx, 0.6);
  const ss = Math.hypot((vsx / vref) / surf.kPeak, (vsy / vref) / aPk);
  const smax = Math.max(1, ss);
  if (s > smax) { const k = smax / s; w.ux *= k; w.uy *= k; }
  w.slipSteady = ss;
}
