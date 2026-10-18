// Assetto Corsa's tyre: Tyre::step with the ground ray, the radial spring, the slip state (relaxation length),
// the force model (SCTM for tyres.ini VERSION >= 10, the brush curve below), rolling resistance, the 36-patch
// thermal model, wear, dirt, grain and blisters, and the tyres.ini loader. Ported from rustyAC
// (tyre/vanilla_tyre.rs, sctm.rs, brush.rs, thermal.rs, data/tyres_ini.rs), which transcribes acs.exe.
//
// Interfaces the tyre talks to (the chassis provides them):
//   hub:    getHubWorldMatrix(), getPointVelocity(p), addForceAtPos(f, p, driven, addToSteerTorque),
//           addTorque(t), getHubAngularVelocity()
//   ground: rayCast(org, dir, length) -> { surface, pos, normal } | null, hasRayCaster
//   car:    isSleeping, speed, dynamicGripLevel, tyreConsumptionRate, mechanicalDamageRate,
//           ambientTemperature, roadTemperature, allowTyreBlankets, bodyVelocity(), bodyMass(),
//           bodyAddForceAtLocalPos(f, p)

import { IniReader, Curve, fileText } from './ini.js';
import { fromAxisAngle, mul, v3 } from './mat.js';

const DEG = 0.017453;
const sign = x => (x > 0 ? 1 : x >= 0 ? 0 : -1);
const nz = x => x < 0 || x > 0;
const clamp01 = x => (x > 1 ? 1 : x >= 0 ? x : 0);
const badNum = x => !Number.isFinite(x);

export const defaultSurface = () => ({
  gripMod: 1, dirtAdditiveK: 0, sinHeight: 0, sinLength: 0, damping: 0, granularity: 0,
  isValidTrack: true, isPitlane: false, vibrationGain: 0, vibrationLength: 0, name: '',
});

// --- the force curves ----------------------------------------------------------------------------------------

class BrushModel {
  constructor() { this.cf = 1200; this.xu = 0; this.cf1 = -10; this.fz0 = 2000; this.maxSlip0 = 0.2; this.maxSlip1 = 0.4; this.falloffSpeed = 2; }
  static cfFromSlipAngle(angle) { return ((1 / Math.tan(angle * DEG)) * 3) * 78.125; }
  solve(slip, friction, load, cf1Mix, asy) {
    const cf = ((load * 0.0005 - 1) * this.cf1) * cf1Mix + this.cf;
    const third = this.cf * (1 / 3);
    const c = third > cf ? third : cf;
    const k = c * 2 * 0.08 * 0.08;
    const slipMax = 1 / (k / (friction * 3));
    const nd = slip / slipMax;
    const force = slip > slipMax ? (1 - asy) / ((slip - slipMax) * 2 + 1) + asy
      : ((1 - nd) * (1 - nd)) * (k * slip) * (friction * this.xu + 1) + (3 - nd * 2) * (nd * nd);
    return { force, slip: nd };
  }
  solveV5(slip, load, asy) {
    const peak = ((load - this.fz0) / this.fz0) * (this.maxSlip1 - this.maxSlip0) + this.maxSlip0;
    const k = ((1 / peak) * 3 * 78.125) * 2 * 0.0064;
    const slipMax = 1 / (k * (1 / 3));
    const nd = slip / slipMax;
    const force = slip > slipMax ? (1 - asy) / ((slip - slipMax) * this.falloffSpeed + 1) + asy
      : ((1 - nd) * (1 - nd)) * (k * slip) + (3 - nd * 2) * (nd * nd);
    return { force, slip: nd };
  }
}

class BrushSlipProvider {
  constructor(maxAngle = null, flex = 0) {
    this.model = new BrushModel();
    this.asy = 1;
    this.version = 0;
    this.maximum = 0;
    this.maxSlip = 0;
    if (maxAngle != null) {
      this.model.cf = BrushModel.cfFromSlipAngle(maxAngle);
      this.model.cf1 = flex * -50000;
      this.recomputeMaximum();
    }
  }
  recomputeMaximum() {
    let maximum = 0, maxSlip = 0;
    for (let slip = 0; slip < 1; slip += 0.001) {
      const out = this.version < 5 ? this.model.solve(slip, 1, 2000, 1, this.asy) : this.model.solveV5(slip, 2000, this.asy);
      if (out.force > maximum) { maximum = out.force; maxSlip = slip; }
    }
    this.maximum = maximum;
    this.maxSlip = maxSlip;
  }
  getSlipForce(slip, load, useAsy) {
    return this.version >= 5 ? this.model.solveV5(slip, load, useAsy ? this.asy : 1) : this.model.solve(slip, 1, load, 1, 1);
  }
}

// SCTM: the tyre model of tyres.ini VERSION >= 10
class Sctm {
  constructor(def) {
    const m = def.modelData, b = def.slipProvider.model;
    this.falloffSpeed = b.falloffSpeed; this.asy = def.slipProvider.asy; this.fz0 = b.fz0;
    this.lsExpX = m.lsExpX; this.lsExpY = m.lsExpY; this.lsMultX = m.lsMultX; this.lsMultY = m.lsMultY;
    this.maxSlip0 = b.maxSlip0; this.maxSlip1 = b.maxSlip1;
    this.speedSensitivity = m.speedSensitivity; this.camberGain = m.camberGain;
    this.dcamber0 = m.dcamber0; this.dcamber1 = m.dcamber1;
    this.dyLoadCurve = m.dyLoadCurve; this.dxLoadCurve = m.dxLoadCurve;
    this.pressureCfGain = m.pressureFlexGain; this.cfXMult = m.cfXMult; this.brakeDxMod = m.brakeDxMod;
    this.dCamberCurve = m.dCamberCurve; this.smoothDCamber = m.smoothDCamber; this.dCamberBlend = 1;
    this.combinedFactor = m.combinedFactor;
  }
  staticDy(load) {
    if (this.dyLoadCurve.count < 1) return load !== 0 ? Math.pow(load, this.lsExpY) * this.lsMultY / load : 0;
    return this.dyLoadCurve.getCubicSplineValue(load);
  }
  staticDx(load) {
    if (this.dxLoadCurve.count < 1) return load !== 0 ? Math.pow(load, this.lsExpX) * this.lsMultX / load : 0;
    return this.dxLoadCurve.getCubicSplineValue(load);
  }
  pureF(cf, slip, asy) {
    const stiffness = cf * 2 * 0.0064, peak = 1 / (stiffness / 3);
    if (!(peak >= slip)) return (1 / ((slip - peak) * this.falloffSpeed + 1)) * (1 - asy) + asy;
    const nd = slip / peak, om = 1 - nd;
    return om * om * (stiffness * slip) + (3 - nd * 2) * (nd * nd);
  }
  solve(i) {
    const out = { fy: 0, fx: 0, mz: 0, trail: 0, ndSlip: 0, dy: 0, dx: 0 };
    if (0 >= i.load || (i.slipAngle === 0 && i.slipRatio === 0 && i.camber === 0)) return out;
    const asy = i.simple ? 1 : this.asy;
    const sa = i.slipAngle;
    const camberedSa = Math.sin(i.camber) * this.camberGain + sa;
    const tanSa = Math.tan(camberedSa), sinSa = Math.sin(sa);
    const blister = clamp01(i.blister * 0.01), blisterDiv = blister * 0.2 + 1;
    out.dy = this.staticDy(i.load) / blisterDiv;
    const sdx = this.staticDx(i.load) / blisterDiv;
    let dY = i.u * out.dy, dX = i.u * sdx;
    out.dx = sdx;
    if (0 > i.slipRatio) dX *= this.brakeDxMod;
    const camber = i.camber;
    let cabs = Math.abs(camber);
    if (!(camber >= 0 && camberedSa >= 0) && (!(0 >= camber) || !(0 >= camberedSa))) cabs = -cabs;
    const cs = -cabs;
    if (this.dCamberCurve.count === 0) {
      let loss = cs * this.dcamber0 - cs * cs * this.dcamber1;
      if (-1 >= loss) loss = -0.9;
      dY += (dY / (loss + 1) - dY) * this.dCamberBlend;
    } else if (!this.smoothDCamber) dY *= this.dCamberCurve.getValue(cs * 57.29578);
    else dY *= this.dCamberCurve.getCubicSplineValue(cs * 57.29578);
    const speed = i.speed, sr = i.slipRatio, cosSa = Math.cos(sa);
    const slideY = speed * sinSa, slideX = cosSa * (speed * sr);
    const slideSpeed = Math.sqrt(slideY * slideY + slideX * slideX);
    const srLim = sr > -0.99999 ? sr : -0.99999;
    const speedDiv = slideSpeed * this.speedSensitivity + 1;
    const pressureCf = this.pressureCfGain * i.pressureRatio + 1;
    dY /= speedDiv; dX /= speedDiv;
    let maxSlip = ((i.load - this.fz0) / this.fz0) * (this.maxSlip1 - this.maxSlip0) + this.maxSlip0;
    maxSlip *= (i.u - 1) * 0.75 + 1;
    const cf = (((1 / maxSlip) * 3 * 78.125) / (i.grain * 0.01 + 1)) * pressureCf;
    const div = srLim + 1, slipX = sr / div, slipY = tanSa / div;
    const f = this.combinedFactor;
    const slip = 0 >= f || f === 2 ? Math.sqrt(slipY * slipY + slipX * slipX)
      : Math.pow(Math.pow(Math.abs(slipY), f) + Math.pow(Math.abs(slipX), f), 1 / f);
    const forceX = this.pureF(cf * this.cfXMult, slip, asy) * dX;
    out.fy = (this.pureF(cf, slip, asy) * dY) * (slipY / slip) * i.load;
    out.fx = (slipX / slip) * forceX * i.load;
    const nd = slip / (1 / ((cf * 2 * 0.0064) / 3));
    out.ndSlip = nd;
    const ts = clamp01(1 - nd * 0.8);
    const trail = ((3 - ts * 2) * (ts * ts) * 1.1 - 0.1) * i.cpLength * 0.12 * clamp01(speed);
    out.dy = dY; out.dx = dX; out.trail = trail;
    out.mz = -(trail * out.fy);
    return out;
  }
}

// --- the thermal model: 12 elements around x 3 stripes across, and a core -----------------------------------

class Thermal {
  constructor() {
    this.elements = 0; this.stripes = 0; this.patches = []; this.phase = 0;
    this.patchData = { surfaceTransfer: 0.3, patchTransfer: 0.2, patchCoreTransfer: 0.2, internalCoreTransfer: 0.004, coolFactorGain: 0 };
    this.coreTemp = 0; this.performanceCurve = new Curve(); this.isActive = true; this.thermalMultD = 1;
    this.practicalTemp = 0; this.camberSpreadK = 1.4; this.coreTInput = 0;
  }
  init(elements, stripes, car) {
    this.elements = elements; this.stripes = stripes; this.phase = 0;
    this.coreTemp = car ? car.ambientTemperature : 26;
    this.isActive = true;
    this.patches = [];
    for (let i = 0; i < elements * stripes; i++) this.patches.push({ connections: [], t: 0, inputT: 0 });
    for (let s = 0; s < stripes; s++) for (let e = 0; e < elements; e++) {
      const self = this.idx(s, e), p = this.patches[self];
      p.t = car ? car.ambientTemperature : 26;
      const link = o => { p.connections.push(o); this.patches[o].connections.push(self); };
      if (s > 0) link(this.idx(s - 1, e));
      if (e > 0) link(this.idx(s, e - 1));
      if (e === elements - 1) link(this.idx(s, 0));
    }
  }
  idx(stripe, element) { return this.elements * stripe + element; }
  contactElement() { return Math.floor(this.phase * (1 / (2 * Math.PI)) * this.elements) % this.elements; }
  reset(car) { this.coreTemp = car ? car.ambientTemperature : 26; for (const p of this.patches) p.t = this.coreTemp; this.phase = 0; }
  setTemperature(t) { this.coreTemp = t; for (const p of this.patches) p.t = t; }
  correctedD(d) { return this.isActive ? d * this.thermalMultD : d; }
  addCoreInput(t) { this.coreTInput += t; }
  addInput(xpos, pressureRel, temp, car) {
    temp += car ? car.roadTemperature : 26;
    const spread = Math.max(-1, Math.min(1, xpos * this.camberSpreadK));
    const pg = pressureRel * -0.5 + 1, edge = pressureRel * 0.1, half = edge * 0.5, e = this.contactElement();
    this.patches[this.idx(0, e)].inputT += (spread + 1 - half) * pg * temp;
    this.patches[this.idx(1, e)].inputT += (edge + 1) * pg * temp;
    this.patches[this.idx(2, e)].inputT += (1 - spread - half) * pg * temp;
  }
  cpTemp(camber) {
    const spread = Math.max(-1, Math.min(1, camber * this.camberSpreadK)), e = this.contactElement();
    return ((spread + 1) * this.patches[this.idx(0, e)].t + this.patches[this.idx(1, e)].t + (1 - spread) * this.patches[this.idx(2, e)].t) * (1 / 3);
  }
  imo() {
    const out = [0, 0, 0];
    for (let s = 0; s < 3; s++) { for (let e = 0; e < 12; e++) out[s] += this.patches[this.idx(s, e)].t; out[s] *= 1 / 12; }
    return out;
  }
  step(dt, angularSpeed, camber, car) {
    this.phase += dt * angularSpeed;
    if (this.phase > 100000) this.phase -= 100000;
    if (!(this.phase >= 0)) this.phase += 100000;
    const floor = car ? car.ambientTemperature : 26;
    const coreInput = floor > this.coreTInput ? floor : this.coreTInput;
    this.coreTInput = 0;
    const d = this.patchData;
    this.coreTemp = (coreInput - this.coreTemp) * (dt * d.internalCoreTransfer) + this.coreTemp;
    const speed = car ? car.speed : 0, ambient = car ? car.ambientTemperature : 26;
    const patchCore = dt * d.patchCoreTransfer;
    const cooling = car ? (speed * speed * d.coolFactorGain + 1) * d.surfaceTransfer * dt : (d.coolFactorGain * 300 + 1) * d.surfaceTransfer * dt;
    for (let i = 0; i < this.patches.length; i++) {
      const p = this.patches[i];
      if (p.inputT > ambient) p.t = (p.inputT - p.t) * (dt * d.surfaceTransfer) + p.t;
      else p.t = (ambient - p.t) * cooling + p.t;
      p.inputT = 0;
      let t = p.t;
      for (const c of p.connections) { t += (this.patches[c].t - t) * (dt * d.patchTransfer); p.t = t; }
      p.t = (this.coreTemp - p.t) * patchCore + p.t;
      this.coreTemp = (p.t - this.coreTemp) * patchCore + this.coreTemp;
    }
    if (this.isActive && this.performanceCurve.count > 0) {
      this.practicalTemp = (this.cpTemp(camber) - this.coreTemp) * 0.25 + this.coreTemp;
      this.thermalMultD = this.performanceCurve.getValue(this.practicalTemp);
    }
  }
}

// --- tyres.ini -----------------------------------------------------------------------------------------------

export function loadTyresIni(files, index) {
  const ini = new IniReader(files, 'tyres.ini');
  if (!ini.ready) throw new Error('tyres.ini cannot be opened');
  const prefix = index < 2 ? 'FRONT' : 'REAR', tprefix = index < 2 ? 'THERMAL_FRONT' : 'THERMAL_REAR';
  const out = { version: ini.getInt('HEADER', 'VERSION'), compounds: [] };
  if (ini.hasSection('EXPLOSION')) out.explosionTemperature = ini.getFloat('EXPLOSION', 'TEMPERATURE');
  if (ini.hasSection('VIRTUALKM')) out.useLoadForVkm = ini.getInt('VIRTUALKM', 'USE_LOAD') !== 0;
  if (ini.hasSection('ADDITIONAL1')) {
    out.blanketTemperature = ini.getFloat('ADDITIONAL1', 'BLANKETS_TEMP');
    out.pressureTemperatureGain = ini.getFloat('ADDITIONAL1', 'PRESSURE_TEMPERATURE_GAIN');
    const spread = ini.getFloat('ADDITIONAL1', 'CAMBER_TEMP_SPREAD_K');
    if (spread !== 0) out.camberSpreadK = spread;
  }
  for (let n = 0; ; n++) {
    const section = n ? `${prefix}_${n}` : prefix;
    if (!ini.hasSection(section)) break;
    const def = loadCompound(ini, files, section, n ? `${tprefix}_${n}` : tprefix);
    def.index = out.compounds.length;
    out.compounds.push(def);
  }
  return out;
}

function loadCompound(ini, files, section, tsection) {
  const f = k => ini.getFloat(section, k), t = k => ini.getFloat(tsection, k);
  const version = ini.getInt('HEADER', 'VERSION');
  const m = {
    version, dy0: 1.4, dy1: -0.145, dx0: 1.68, dx1: -0.0145, fz0: 2000, flexK: 0.005, speedSensitivity: 0.003,
    relaxationLength: 0.8, rr0: 500, rr1: 0.0075, rrSa: 0, rrSr: 0, rrSlip: 0, camberGain: 0, pressureSpringGain: 20000,
    pressureFlexGain: 1, pressureRrGain: 0.5, pressureGainD: 0, idealPressure: 0, pressureRef: 26, wearCurve: new Curve(),
    dcamber0: 0.1, dcamber1: -0.8, dyLoadCurve: new Curve(), dxLoadCurve: new Curve(), lsMultY: 0, lsExpY: 0, lsMultX: 0,
    lsExpX: 0, maxWearKm: 0, maxWearMult: 0, asy: 1, cfXMult: 1, brakeDxMod: 1, dCamberCurve: new Curve(), smoothDCamber: false,
    combinedFactor: 0,
  };
  const data = {
    width: 0.15, radius: 0.3, k: 220000, d: 400, angularInertia: 1.6, thermalFrictionK: 0.03, thermalRollingK: 0.5,
    thermalRollingSurfaceK: 0, grainThreshold: 0, blisterThreshold: 9000, grainGamma: 1, blisterGamma: 1, grainGain: 0,
    blisterGain: 0, rimRadius: 0, optimumTemp: 80, softnessIndex: 0, radiusRaiseK: 0,
  };
  const def = { section, modelData: m, data, pressureStatic: 26, name: ini.getString(section, 'NAME'), shortName: '',
    thermalPatchData: { surfaceTransfer: 0.3, patchTransfer: 0.2, patchCoreTransfer: 0.2, internalCoreTransfer: 0.004, coolFactorGain: 0 },
    thermalPerformanceCurve: new Curve() };
  if (version > 3) def.shortName = ini.getString(section, 'SHORT_NAME');
  data.width = f('WIDTH'); if (data.width <= 0) data.width = 0.15;
  data.radius = f('RADIUS');
  data.rimRadius = version < 3 ? 0.13 : f('RIM_RADIUS');
  m.flexK = f('FLEX');
  let limitAngle = f('FRICTION_LIMIT_ANGLE'), xmu = f('XMU');
  if (limitAngle === 0) limitAngle = 7.5;
  if (version >= 5) xmu = 0;
  const sp = new BrushSlipProvider(limitAngle, m.flexK);
  if (version > 9) {
    m.cfXMult = f('CX_MULT');
    data.radiusRaiseK = f('RADIUS_ANGULAR_K') * 0.001;
    if (ini.hasKey(section, 'BRAKE_DX_MOD')) { const b = f('BRAKE_DX_MOD'); m.brakeDxMod = b === 0 ? 1 : b + 1; }
    if (ini.hasKey(section, 'COMBINED_FACTOR')) m.combinedFactor = f('COMBINED_FACTOR');
  }
  if (version < 5) {
    m.dy0 = f('DY0'); m.dy1 = f('DY1'); m.dx0 = f('DX0'); m.dx1 = f('DX1');
    sp.asy = 0.85; sp.model.xu = xmu;
  } else {
    const fz0 = f('FZ0');
    m.lsExpX = f('LS_EXPX'); m.lsExpY = f('LS_EXPY'); m.dx0 = f('DX_REF'); m.dy0 = f('DY_REF');
    m.lsMultX = (m.dx0 * fz0) / Math.pow(fz0, m.lsExpX);
    m.lsMultY = (m.dy0 * fz0) / Math.pow(fz0, m.lsExpY);
    sp.asy = 0.92; sp.model.fz0 = fz0;
    const flexGain = f('FLEX_GAIN');
    sp.model.maxSlip0 = Math.tan(limitAngle * DEG);
    sp.model.maxSlip1 = Math.tan((flexGain + 1) * limitAngle * DEG);
    sp.version = 5;
    if (ini.hasKey(section, 'DY_CURVE')) m.dyLoadCurve = ini.getCurve(section, 'DY_CURVE');
    if (ini.hasKey(section, 'DX_CURVE')) m.dxLoadCurve = ini.getCurve(section, 'DX_CURVE');
  }
  sp.recomputeMaximum();
  if (version > 6) { sp.asy = f('FALLOFF_LEVEL'); sp.model.falloffSpeed = f('FALLOFF_SPEED'); }
  def.slipProvider = sp;
  m.speedSensitivity = f('SPEED_SENSITIVITY');
  m.relaxationLength = f('RELAXATION_LENGTH');
  m.rr0 = f('ROLLING_RESISTANCE_0'); m.rr1 = f('ROLLING_RESISTANCE_1');
  if (version === 1) { m.rrSa = f('ROLLING_RESISTANCE_SA'); m.rrSr = f('ROLLING_RESISTANCE_SR'); }
  else m.rrSlip = f('ROLLING_RESISTANCE_SLIP');
  m.camberGain = f('CAMBER_GAIN');
  m.dcamber0 = f('DCAMBER_0'); m.dcamber1 = f('DCAMBER_1');
  if (m.dcamber0 === 0 || m.dcamber1 === 0) { m.dcamber0 = 0.1; m.dcamber1 = -0.8; }
  if (ini.hasKey(section, 'DCAMBER_LUT')) { m.dCamberCurve = ini.getCurve(section, 'DCAMBER_LUT'); m.smoothDCamber = ini.getInt(section, 'DCAMBER_LUT_SMOOTH') !== 0; }
  data.angularInertia = f('ANGULAR_INERTIA'); data.d = f('DAMP'); data.k = f('RATE');
  if (data.angularInertia === 0) data.angularInertia = 1.2;
  if (data.d === 0) data.d = 400;
  if (data.k === 0) data.k = 220000;
  if (m.dx0 === 0) m.dx0 = m.dy0 * 1.2;
  if (m.dx1 === 0) m.dx1 = m.dy1 * 0.1;
  def.pressureStatic = f('PRESSURE_STATIC'); if (def.pressureStatic === 0) def.pressureStatic = 26;
  m.pressureRef = def.pressureStatic;
  m.pressureSpringGain = f('PRESSURE_SPRING_GAIN'); if (m.pressureSpringGain === 0) m.pressureSpringGain = 1000;
  m.pressureFlexGain = f('PRESSURE_FLEX_GAIN'); m.pressureRrGain = f('PRESSURE_RR_GAIN'); m.pressureGainD = f('PRESSURE_D_GAIN');
  m.idealPressure = f('PRESSURE_IDEAL'); if (m.idealPressure === 0) m.idealPressure = 26;
  if (ini.hasSection(tsection)) {
    const p = def.thermalPatchData;
    p.surfaceTransfer = t('SURFACE_TRANSFER'); p.patchTransfer = t('PATCH_TRANSFER'); p.patchCoreTransfer = t('CORE_TRANSFER');
    data.thermalFrictionK = t('FRICTION_K'); data.thermalRollingK = t('ROLLING_K');
    if (version > 4) {
      p.internalCoreTransfer = t('INTERNAL_CORE_TRANSFER');
      if (ini.hasKey(tsection, 'COOL_FACTOR')) p.coolFactorGain = (t('COOL_FACTOR') - 1) * 0.000324;
    }
    if (version > 5) data.thermalRollingSurfaceK = t('SURFACE_ROLLING_K');
    def.thermalPerformanceCurve = Curve.fromLut(fileText(files, ini.getString(tsection, 'PERFORMANCE_CURVE')));
  }
  m.wearCurve = Curve.fromLut(fileText(files, ini.getString(section, 'WEAR_CURVE')));
  m.wearCurve.scale(0.01);
  const perf = def.thermalPerformanceCurve;
  for (let i = 0; i < perf.count; i++) { const [temp, v] = perf.pairAt(i); if (v >= 1) { data.grainThreshold = temp; break; } }
  for (let i = perf.count - 1; i > 0; i--) { const [temp, v] = perf.pairAt(i); if (v >= 1) { data.blisterThreshold = temp; data.optimumTemp = temp; break; } }
  m.maxWearMult = 100;
  for (let i = 0; i < m.wearCurve.count; i++) { const [km, v] = m.wearCurve.pairAt(i); if (!(v >= m.maxWearMult)) { m.maxWearMult = v; m.maxWearKm = km; } }
  if (version > 2) { data.blisterGamma = t('BLISTER_GAMMA'); data.blisterGain = t('BLISTER_GAIN'); data.grainGamma = t('GRAIN_GAMMA'); data.grainGain = t('GRAIN_GAIN'); }
  const d3000 = version < 5 ? 3000 * 0.0005 * m.dy1 + m.dy0 : Math.pow(3000, m.lsExpY) * m.lsMultY / 3000;
  data.softnessIndex = d3000 - 1;
  if (!(data.softnessIndex >= 0)) data.softnessIndex = 0;
  return def;
}

// --- the tyre ------------------------------------------------------------------------------------------------

const slipAngleRad = (vy, vx) => (nz(vx) ? Math.atan(-(vy / Math.abs(vx))) : 0);
export function contactPatchLength(radius, deflection) {
  const u = radius - deflection;
  return u > 0 && radius > u ? Math.sqrt(radius * radius - u * u) * 2 : 0;
}
function camberRad(n, m) {
  const d = m[1] * n[1] + m[0] * n[0] + m[2] * n[2];
  return d > -1 && !(d >= 1) ? -Math.asin(d) : -1.5707964;
}

export class Tyre {
  constructor() {
    this.inputs = { brakeTorque: 0, handBrakeTorque: 0, electricTorque: 0 };
    this.status = {
      depth: 0, load: 0, camberRad: 0, slipAngleRad: 0, slipRatio: 0, angularVelocity: 0, fy: 0, fx: 0, mz: 0,
      isLocked: false, slipFactor: 0, ndSlip: 0, distToGround: 0, dy: 0, dx: 0, d: 0, dirtyLevel: 0,
      rollingResistance: 0, thermalInput: 0, feedbackTorque: 0, loadedRadius: 0, effectiveRadius: 0, liveRadius: 0,
      pressureStatic: 26, pressureDynamic: 26, virtualKm: 0, lastTempImo: [-200, -200, -200], grain: 0, blister: 0,
      inflation: 0, flatSpot: 0, lastGrain: 0, lastBlister: 0, wearMult: 0,
    };
    this.worldRotation = null;
    this.worldPosition = [0, 0, 0];
    this.contactPoint = [0, 0, 0];
    this.contactNormal = [0, 1, 0];
    this.surface = null;
    this.absOverride = 0;
    this.thermal = new Thermal();
    this.compounds = [];
    this.aiMult = 1;
    this.roadRight = [0, 0, 0];
    this.roadHeading = [0, 0, 0];
    this.useLoadForVkm = false;
    this.driven = false;
    this.oldAngularVelocity = 0;
    this.localWheelRotation = null;
    this.slidingVelocityX = 0; this.slidingVelocityY = 0; this.roadVelocityX = 0; this.roadVelocityY = 0;
    this.totalHubVelocity = 0;
    this.index = 0;
    this.compoundIndex = 0;
    this.blanketsOn = true;
    this.flatSpotK = 0.15;
    this.explosionTemperature = 350;
    this.blanketTemperature = 80;
    this.pressureTemperatureGain = 0.16;
    this.localMx = 0;
  }

  init(hub, files, index, car) {
    this.thermal.init(12, 3, car);
    this.index = index;
    this.localWheelRotation = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    this.absOverride = 1;
    const t = loadTyresIni(files, index);
    if (!t.compounds.length) throw new Error(`tyres.ini: no compound for wheel ${index}`);
    if (t.version < 10) throw new Error(`tyres.ini VERSION=${t.version}: only VERSION 10 and later (the SCTM tyre) is ported`);
    if (t.explosionTemperature != null) this.explosionTemperature = t.explosionTemperature;
    if (t.useLoadForVkm != null) this.useLoadForVkm = t.useLoadForVkm;
    if (t.blanketTemperature != null) this.blanketTemperature = t.blanketTemperature;
    if (t.pressureTemperatureGain != null) this.pressureTemperatureGain = t.pressureTemperatureGain;
    if (t.camberSpreadK != null) this.thermal.camberSpreadK = t.camberSpreadK;
    this.compounds = t.compounds;
    this.setCompound(0, hub, car);
  }

  setCompound(i, hub, car) {
    if (i < 0 || i >= this.compounds.length) return false;
    const def = this.compounds[i];
    this.modelData = { ...def.modelData };
    this.data = { ...def.data };
    this.thermal.patchData = { ...def.thermalPatchData };
    this.thermal.performanceCurve = def.thermalPerformanceCurve;
    this.slipProvider = def.slipProvider;
    this.compoundIndex = i;
    this.status.pressureStatic = def.pressureStatic;
    this.status.pressureDynamic = def.pressureStatic;
    this.status.dirtyLevel = 0;
    this.status.virtualKm = 0;
    this.thermal.reset(car);
    this.reset(hub, car);
    this.modelData.asy = this.slipProvider.asy;
    this.model = new Sctm(def);
    return true;
  }

  reset(hub, car) {
    const s = this.status;
    s.lastGrain = s.grain; s.lastBlister = s.blister;
    s.isLocked = true; s.inflation = 1; s.flatSpot = 0; s.slipRatio = 0; s.angularVelocity = 0; s.slipAngleRad = 0;
    s.fy = 0; s.fx = 0; s.mz = 0; s.grain = 0; s.blister = 0;
    this.readHubMatrix(hub);
    if (!(0.001 >= s.virtualKm)) s.lastTempImo = this.thermal.imo();
    s.virtualKm = 0; s.dirtyLevel = 0; s.feedbackTorque = 0;
    this.absOverride = 1;
    this.thermal.reset(car);
    this.blanketsOn = true;
  }

  readHubMatrix(hub) {
    const m = hub.getHubWorldMatrix().slice();
    this.worldPosition = [m[12], m[13], m[14]];
    m[12] = 0; m[13] = 0; m[14] = 0;
    this.worldRotation = m;
  }

  step(dt, hub, ground, car) {
    const s = this.status;
    s.feedbackTorque = 0; s.fx = 0; s.mz = 0; s.slipFactor = 0; s.rollingResistance = 0;
    this.slidingVelocityY = 0; this.slidingVelocityX = 0; this.totalHubVelocity = 0;
    this.surface = null;
    this.readHubMatrix(hub);
    if (badNum(s.angularVelocity)) s.angularVelocity = 0;
    const wp = this.worldPosition, R = this.worldRotation;
    const hit = ground ? ground.rayCast([wp[0], wp[1] + 2, wp[2]], [0, -1, 0], ground.hasRayCaster ? 3 : 2) : null;
    if (hit && !(0.35 >= R[5])) {
      this.surface = hit.surface;
      const surface = hit.surface, up = [R[4], R[5], R[6]];
      const [nx, ny, nz_] = hit.normal;
      let normal = hit.normal.slice();
      const dot = ny * up[1] + nx * up[0] + nz_ * up[2];
      if (dot > 0.96) {
        const dist = ny * (hit.pos[1] - wp[1]) + nx * (hit.pos[0] - wp[0]) + nz_ * (hit.pos[2] - wp[2]);
        this.contactPoint = [nx * dist + wp[0], ny * dist + wp[1], nz_ * dist + wp[2]];
      } else {
        const angle = (dot > -1 && !(dot >= 1) ? Math.acos(dot) : 0) - Math.acos(0.96);
        const axis = v3.norm([up[2] * ny - up[1] * nz_, up[0] * nz_ - up[2] * nx, up[1] * nx - up[0] * ny]);
        const r = fromAxisAngle(axis, angle);
        normal = [r[0] * nx + r[4] * ny + r[8] * nz_ + r[12], r[1] * nx + r[5] * ny + r[9] * nz_ + r[13], r[2] * nx + r[6] * ny + r[10] * nz_ + r[14]];
        this.contactPoint = hit.pos.slice();
      }
      const cp = this.contactPoint;
      if (nz(surface.sinHeight)) cp[1] -= (Math.sin(surface.sinLength * cp[0]) * Math.cos(surface.sinLength * cp[2]) + 1) * surface.sinHeight;
      if (nz(surface.granularity)) {
        let y = cp[1];
        for (const [l, h] of [[1, 0.005], [5.8, 0.005], [11.4, 0.01]]) y += (Math.sin(l * cp[0]) * Math.cos(l * cp[2]) + 1) * h * -0.6;
        cp[1] = y;
      }
      this.contactNormal = normal;
      this.addGroundContact(cp, normal, hub);
      this.addTyreForcesV10(cp, normal, surface, dt, hub, car);
      if (surface.damping > 0 && car) {
        const v = car.bodyVelocity(), m = car.bodyMass(), k = -surface.damping;
        car.bodyAddForceAtLocalPos([k * v[0] * m, k * v[1] * m, k * v[2] * m], [0, 0, 0]);
      }
    } else {
      s.ndSlip = 0;
      s.fy = 0;
    }
    let brake = this.inputs.brakeTorque * this.absOverride;
    if (!(brake > this.inputs.handBrakeTorque)) brake = this.inputs.handBrakeTorque;
    s.feedbackTorque = s.rollingResistance - (sign(s.angularVelocity) * brake + this.localMx) + this.inputs.electricTorque;
    if (!this.driven) {
      this.updateAngularSpeed(dt);
      this.stepRotationMatrix(dt, car);
    } else {
      this.updateLockedState();
      if (sign(this.oldAngularVelocity) !== sign(s.angularVelocity) && 1 > this.totalHubVelocity) s.isLocked = true;
      this.oldAngularVelocity = s.angularVelocity;
    }
    if (!(this.totalHubVelocity >= 10)) s.slipFactor = Math.abs(this.totalHubVelocity * 0.1) * s.slipFactor;
    this.stepThermal(dt, car);
    s.pressureDynamic = (this.thermal.coreTemp - 26) * this.pressureTemperatureGain + s.pressureStatic;
    this.stepGrainBlister(dt, this.totalHubVelocity, car);
    this.stepFlatSpot(dt, this.totalHubVelocity, car);
  }

  addGroundContact(pos, normal, hub) {
    const s = this.status, wp = this.worldPosition;
    const dx = wp[0] - pos[0], dy = wp[1] - pos[1], dz = wp[2] - pos[2];
    const sq = dy * dy + dx * dx + dz * dz;
    s.distToGround = nz(sq) ? Math.sqrt(sq) : 0;
    const vel = hub.getPointVelocity(pos);
    let radius = nz(this.data.radiusRaiseK) ? Math.abs(s.angularVelocity) * this.data.radiusRaiseK + this.data.radius : this.data.radius;
    if (!(s.inflation >= 1)) radius = (radius - this.data.rimRadius) * s.inflation + this.data.rimRadius;
    s.liveRadius = radius;
    s.effectiveRadius = radius;
    if (s.distToGround > radius) {
      s.loadedRadius = radius; s.depth = 0; s.load = 0; s.fx = 0; s.mz = 0; s.fy = 0; s.ndSlip = 0;
    } else {
      const depth = radius - s.distToGround, loaded = radius - depth;
      s.depth = depth;
      s.loadedRadius = loaded;
      let k = 200000;
      if (loaded > this.data.rimRadius) {
        k = (s.pressureDynamic - this.modelData.pressureRef) * this.modelData.pressureSpringGain + this.data.k;
        if (!(k >= 0)) k = 0;
      }
      const load = -((vel[0] * normal[0] + vel[1] * normal[1] + vel[2] * normal[2]) * this.data.d) + depth * k;
      s.load = load;
      hub.addForceAtPos([load * normal[0], load * normal[1], load * normal[2]], pos, this.driven, false);
      if (0 > s.load) s.load = 0;
    }
  }

  addTyreForcesV10(pos, normal, surface, dt, hub, car) {
    const s = this.status, m = this.worldRotation, md = this.modelData;
    const h = [-m[8], -m[9], -m[10]], r = [m[0], m[1], m[2]];
    let d = v3.dot(h, normal);
    this.roadHeading = v3.norm([h[0] - d * normal[0], h[1] - d * normal[1], h[2] - d * normal[2]]);
    d = v3.dot(r, normal);
    this.roadRight = v3.norm([r[0] - d * normal[0], r[1] - d * normal[1], r[2] - d * normal[2]]);
    const havel = hub.getHubAngularVelocity();
    const wheelSpeed = havel[1] * r[1] + havel[0] * r[0] + havel[2] * r[2] + s.angularVelocity;
    const v = hub.getPointVelocity(pos);
    this.slidingVelocityY = v3.dot(v, this.roadRight);
    this.roadVelocityX = -v3.dot(v, this.roadHeading);
    let slipAngle = slipAngleRad(this.slidingVelocityY, this.roadVelocityX);
    this.slidingVelocityX = wheelSpeed * s.effectiveRadius - this.roadVelocityX;
    const roadSpeed = Math.abs(this.roadVelocityX);
    let slipRatio = nz(roadSpeed) ? this.slidingVelocityX / roadSpeed : 0;
    const camber = camberRad(this.contactNormal, m);
    s.camberRad = camber;
    const load = s.load;
    const syy = this.slidingVelocityY * this.slidingVelocityY;
    const hubSpeed = Math.sqrt(this.roadVelocityX * this.roadVelocityX + syy);
    this.totalHubVelocity = hubSpeed;
    const nd = clamp01(s.ndSlip);
    const rl = md.relaxationLength;
    const loadedLength = ((load / md.fz0 * rl - rl) * 0.3 + rl) * 2;
    const relaxation = (rl - loadedLength) * nd + loadedLength;
    if (!(hubSpeed >= 1)) {
      slipRatio = Math.max(-1, Math.min(1, this.slidingVelocityX * 0.5));
      slipAngle = Math.max(-1, Math.min(1, this.slidingVelocityY * -5.5));
    }
    const lag = (target, old) => {
      if (!nz(relaxation)) return target;
      const rate = hubSpeed * dt / relaxation;
      if (rate > 1) return target;
      if (!(rate >= 0.04)) return 0.04 * (target - old) + old;
      if (!(rate >= 1)) return rate * (target - old) + old;
      return target;
    };
    s.slipRatio = lag(slipRatio, s.slipRatio);
    s.slipAngleRad = lag(slipAngle, s.slipAngleRad);
    if (!(load > 0)) { s.slipAngleRad = 0; s.slipRatio = 0; }
    const grip = car ? car.dynamicGripLevel : 1;
    const slideSpeed = Math.sqrt(this.slidingVelocityX * this.slidingVelocityX + syy);
    const out = this.model.solve({
      load, slipAngle: s.slipAngleRad, slipRatio: s.slipRatio, camber, speed: hubSpeed,
      u: this.correctedD(1, true) * surface.gripMod * grip,
      cpLength: contactPatchLength(s.liveRadius, s.depth), grain: s.grain, blister: s.blister,
      pressureRatio: s.pressureDynamic / md.idealPressure - 1, simple: !(1 >= this.aiMult),
    });
    s.dy = out.dy; s.dx = out.dx;
    s.fy = out.fy * this.aiMult;
    s.fx = -out.fx;
    this.stepDirtyLevel(dt, Math.abs(s.effectiveRadius * s.angularVelocity), surface);
    this.stepPuncture(car);
    s.mz = out.mz;
    const fx = s.fx, fy = s.fy, rr = this.roadRight, rh = this.roadHeading;
    hub.addForceAtPos([fy * rr[0] + fx * rh[0], fy * rr[1] + fx * rh[1], fy * rr[2] + fx * rh[2]], pos, this.driven, true);
    this.localMx = -(s.loadedRadius * s.fx);
    hub.addTorque([out.mz * normal[0], out.mz * normal[1], out.mz * normal[2]]);
    this.stepRollingResistance();
    if (car) {
      const lm = this.useLoadForVkm ? s.load / md.fz0 : 1;
      s.virtualKm += slideSpeed * dt * car.tyreConsumptionRate * lm * 0.001;
    }
    s.d = this.model.staticDy(s.load);
    s.ndSlip = out.ndSlip;
  }

  stepRollingResistance() {
    const s = this.status, md = this.modelData, w = s.angularVelocity, aw = Math.abs(w);
    if (!(aw > 1)) return;
    const er = s.effectiveRadius, rs = er * w, p = s.pressureDynamic;
    let pm = (md.idealPressure / p - 1) * md.pressureRrGain + 1;
    if (!(p > 0)) pm = 0;
    let res = (rs * rs * md.rr1 + md.rr0) * sign(rs) * pm;
    if (aw > 20) {
      const slip = md.version >= 2 ? clamp01(s.ndSlip) * md.rrSlip * pm
        : clamp01(Math.abs(s.slipRatio)) * pm * md.rrSr + Math.abs(s.slipAngleRad * 57.29578) * pm * md.rrSa;
      res *= slip * 0.001 + 1;
    }
    s.rollingResistance = -(s.load * 0.001 * res * er);
  }

  correctedD(d, storeWear) {
    const s = this.status, md = this.modelData;
    let c = this.thermal.correctedD(d) / (Math.abs(s.pressureDynamic - md.idealPressure) * md.pressureGainD + 1);
    if (md.wearCurve.count !== 0) {
      const w = md.wearCurve.getValue(s.virtualKm);
      c *= w;
      if (storeWear) s.wearMult = w;
    }
    return c;
  }

  stepDirtyLevel(dt, hubSpeed, surface) {
    const s = this.status;
    if (!(s.dirtyLevel >= 5)) s.dirtyLevel = hubSpeed * surface.dirtAdditiveK * 0.03 * dt + s.dirtyLevel;
    if (!nz(surface.dirtAdditiveK)) {
      if (s.dirtyLevel > 0) s.dirtyLevel -= hubSpeed * 0.015 * dt;
      if (0 > s.dirtyLevel) s.dirtyLevel = 0;
    }
    const c = clamp01(s.dirtyLevel * 0.05), g = 1 - c > 0.8 ? 1 - c : 0.8;
    if (this.aiMult === 1) { s.mz *= g; s.fx *= g; s.fy *= g; }
  }

  stepPuncture(car) {
    if (!car || 0 >= car.mechanicalDamageRate) return;
    const imo = this.thermal.imo(), l = this.explosionTemperature;
    if (imo[0] > l || imo[1] > l || imo[2] > l) this.status.inflation = 0;
  }

  updateLockedState() {
    const s = this.status;
    if (!s.isLocked) return;
    let brake = this.absOverride * this.inputs.brakeTorque;
    if (!(brake > this.inputs.handBrakeTorque)) brake = this.inputs.handBrakeTorque;
    s.isLocked = Math.abs(brake) >= Math.abs(s.loadedRadius * s.fx) && !(Math.abs(s.angularVelocity) >= 1) && !this.driven;
  }

  updateAngularSpeed(dt) {
    const s = this.status;
    this.updateLockedState();
    const w = s.feedbackTorque / this.data.angularInertia * dt + s.angularVelocity;
    s.angularVelocity = w;
    if (sign(this.oldAngularVelocity) !== sign(w)) s.isLocked = true;
    this.oldAngularVelocity = w;
    if (s.isLocked) s.angularVelocity = 0;
    if (!(Math.abs(s.angularVelocity) >= 1)) s.angularVelocity *= 0.9;
  }

  stepRotationMatrix(dt, car) {
    if (!car) return;
    if (Math.abs(this.status.angularVelocity) > 0.1 && !car.isSleeping) {
      const m = mul(fromAxisAngle([1, 0, 0], dt * this.status.angularVelocity), this.localWheelRotation);
      m[12] = 0; m[13] = 0; m[14] = 0;
      this.localWheelRotation = m;
    }
  }

  stepThermal(dt, car) {
    const s = this.status, md = this.modelData;
    const grip = car ? car.dynamicGripLevel : 1;
    const slide = Math.sqrt(this.slidingVelocityX * this.slidingVelocityX + this.slidingVelocityY * this.slidingVelocityY);
    s.thermalInput = slide * (s.d * s.load * this.data.thermalFrictionK) * grip;
    if (this.surface) s.thermalInput *= this.surface.gripMod;
    if (!badNum(s.thermalInput)) {
      const p = s.pressureDynamic, ideal = md.idealPressure;
      const pm = (ideal / p - 1) * md.pressureRrGain + 1;
      let rollingK = this.data.thermalRollingK;
      if (p >= 0) rollingK *= pm;
      if (md.version < 5) s.thermalInput += rollingK * s.angularVelocity * s.load * 0.001;
      if (md.version >= 6) s.thermalInput += pm * this.data.thermalRollingSurfaceK * s.angularVelocity * s.load * 0.001;
      this.thermal.addInput(s.camberRad, p / ideal - 1, s.thermalInput, car);
      if (md.version >= 5) this.thermal.addCoreInput(rollingK * s.angularVelocity * s.load * 0.001);
      this.thermal.step(dt, s.angularVelocity, s.camberRad, car);
    }
    if (car && car.allowTyreBlankets && this.blanketsOn) {
      if (car.speed * 3.6 > 10) this.blanketsOn = false;
      this.thermal.setTemperature(this.blanketTemperature >= this.data.optimumTemp ? this.data.optimumTemp : this.blanketTemperature);
    }
  }

  stepGrainBlister(dt, hubVelocity, car) {
    const s = this.status, d = this.data;
    if (!car || 0 >= car.tyreConsumptionRate) { s.grain = 0; s.blister = 0; return; }
    const rate = car.tyreConsumptionRate;
    if (0 >= s.load) return;
    const nd = s.ndSlip >= 2.5 ? 2.5 : s.ndSlip;
    const core = this.thermal.coreTemp, slipGain = Math.pow(nd, d.grainGamma), sf = this.surface;
    if (sf) {
      if (d.grainGain > 0 && !(core >= d.grainThreshold) && hubVelocity > 2 && sf.gripMod >= 0.95) {
        const amount = sf.gripMod * hubVelocity * d.grainGain * ((d.grainThreshold - core) * 0.0001);
        if (!badNum(amount)) s.grain += slipGain * dt * amount * rate;
      }
      const heal = hubVelocity * sf.gripMod * d.grainGain * 5e-5;
      if (!badNum(heal) && heal > 0) s.grain -= slipGain * dt * heal * rate;
      if (d.blisterGain > 0 && core > d.blisterThreshold && hubVelocity > 2 && sf.gripMod >= 0.95) {
        const amount = sf.gripMod * this.totalHubVelocity * d.blisterGain * ((core - d.blisterThreshold) * 0.0001);
        if (!badNum(amount)) s.blister += Math.pow(nd, d.blisterGamma) * dt * amount * rate;
      }
    }
    s.grain = Math.max(0, Math.min(100, s.grain));
    s.blister = Math.max(0, Math.min(100, s.blister));
  }

  stepFlatSpot(dt, hubVelocity, car) {
    const s = this.status;
    const locked = !(Math.abs(s.angularVelocity) > 0.3) || -0.98 > s.slipRatio;
    if (!this.surface || !car || !locked || !(hubVelocity > 3)) return;
    if (!nz(car.mechanicalDamageRate) || !(this.surface.gripMod >= 0.95)) return;
    const amount = hubVelocity * this.flatSpotK * s.load * this.surface.gripMod;
    s.flatSpot = Math.min(1, amount * 1e-5 * dt * car.mechanicalDamageRate * this.data.softnessIndex + s.flatSpot);
  }
}
