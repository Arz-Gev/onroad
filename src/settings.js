const KEY = 'onroad.settings.v1';

export const DEFAULTS = {
  gearbox: 'manual',
  autoClutch: true,
  arcadeAuto: false,
  handbrake: 'hold',
  steerAssist: 'strong',
  camera: 'chase',
  time: 13,
  muted: false,
  volume: 1,
  speedUnit: 'kmh',
  pressureUnit: 'psi',
  cluster: 'auto',
  hudScale: 1,
  hints: true,
  suspension: false,
  telemetry: false,
  fps: false,
  autoPause: true,
  quality: 'auto',
  renderScale: 1,

  gDpr: 1.5,
  gAA: 'msaa2',
  gShadows: 'high',
  gSSAO: 'low',
  gBloom: true,
  gViewDist: 1,
  gTerrain: 2,
  gTreeShadows: true,

  gGrass: 0.15,
  gGrassHeight: 3.25,
  gGrassWidth: 2,
  gGrassNear: 60,
  gGrassDist: 239,
  gGrassFarWidth: 1.65,
  gGrassFarSpacing: 1.1,
  gGrassFarGrow: 0,
  gBushes: 0.65,
  gBushHeight: 1.6,
  gBushDist: 1.55,
  vegetation: true,

  uiPalette: 'onroad',
  uiAccent: '#ededed',
  uiCool: '#4cc2ff',
  uiHot: '#ff5c35',
  uiGlass: 0.5,
};

const OLD_TIMES = { day: 13, dusk: 19.5, night: 23 };

const CHOICES = {
  uiPalette: ['onroad', 'ice', 'neon', 'mint', 'violet', 'mono', 'custom'],
  gearbox: ['auto', 'manual'],
  handbrake: ['hold', 'toggle', 'auto'],
  steerAssist: ['strong', 'light', 'off'],
  camera: ['chase', 'cockpit', 'hood', 'wheel', 'orbit'],
  speedUnit: ['kmh', 'mph'],
  pressureUnit: ['psi', 'bar'],
  cluster: ['auto', 'full', 'compact'],
  quality: ['auto', 'low', 'medium', 'high', 'ultra', 'custom'],
  gAA: ['off', 'fxaa', 'msaa2', 'msaa4'],
  gShadows: ['off', 'low', 'medium', 'high', 'ultra'],
  gSSAO: ['off', 'low', 'high'],
  gTerrain: [0, 1, 2],
};
const RANGES = {
  time: [0, 24], volume: [0, 1], hudScale: [0.7, 1.5], renderScale: [0.5, 1],
  gDpr: [1, 2], gViewDist: [0.6, 1.5],

  gGrass: [0.04, 0.96], gGrassHeight: [1.8, 3.9], gGrassWidth: [0.8, 2.4], gGrassNear: [42, 72], gGrassDist: [119, 287],
  gGrassFarWidth: [1.3, 3.6], gGrassFarSpacing: [0.85, 1.6], gGrassFarGrow: [0, 1.5],
  gBushes: [0.05, 2.2], gBushHeight: [1.1, 2.6], gBushDist: [1, 2.25],
  uiGlass: [0.2, 0.85],
};
const HEX = /^#[0-9a-f]{6}$/i;
const PATTERNS = { uiAccent: HEX, uiCool: HEX, uiHot: HEX };

export const storage = {
  get(k) { try { return window.localStorage.getItem(k); } catch { return null; } },
  set(k, v) { try { window.localStorage.setItem(k, v); return true; } catch { return false; } },
  remove(k) { try { window.localStorage.removeItem(k); } catch { } },
};

function valid(k, v) {
  if (!(k in DEFAULTS) || typeof v !== typeof DEFAULTS[k]) return false;
  if (CHOICES[k]) return CHOICES[k].includes(v);
  if (RANGES[k]) return Number.isFinite(v) && v >= RANGES[k][0] && v <= RANGES[k][1];
  if (PATTERNS[k]) return PATTERNS[k].test(v);
  return true;
}

export class Settings {
  constructor() {
    this.v = { ...DEFAULTS };
    try {
      const saved = JSON.parse(storage.get(KEY) || '{}');
      if (typeof saved.time === 'string') saved.time = OLD_TIMES[saved.time];
      for (const k in saved) if (valid(k, saved[k])) this.v[k] = saved[k];
    } catch { }
    this.subs = [];
  }

  get(k) { return this.v[k]; }
  get all() { return this.v; }

  set(k, value, opts = {}) {
    if (!valid(k, value)) return false;
    if (this.v[k] === value && !opts.force) return false;
    this.v[k] = value;
    this.save();
    for (const f of this.subs) f(k, value, opts);
    return true;
  }

  onChange(f) { this.subs.push(f); }
  applyAll(opts = {}) { for (const k in this.v) for (const f of this.subs) f(k, this.v[k], { silent: true, ...opts }); }
  save() { storage.set(KEY, JSON.stringify(this.v)); }
  reset() { for (const k in DEFAULTS) this.set(k, DEFAULTS[k], { silent: true, reset: true }); }
}
