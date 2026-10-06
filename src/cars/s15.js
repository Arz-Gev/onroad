// Nissan Silvia S15 (model: public/models/s15.glb). Numbers from the Assetto Corsa S14/180SX drift setups in
// "cars and tracks" (SR20DET torque curve and turbo, 5-speed gearing, 2.525 m wheelbase, 52 % front, 1300 kg).
// Rear-wheel drive with a welded diff (the AC setups run POWER = COAST = 1.0); the diff key switches
// to a 1.5-way LSD or an open diff. Model and view: vehicle/carModel.js and vehicle/carView.js.
//
// Vehicle-local frame: +x right, +y up, -z forward. y = 0 is the ground at static ride height,
// z = 0 is the middle of the wheelbase.

import { gross } from '../vehicle/engineCurve.js';

// ---------------------------------------------------------------- engines (tuning panel)
// SR20DET net torque without boost (the AC curve). AC multiplies it by (1 + boost); full boost from refRpm.
const SR20_NA = [[0, 0], [500, 176], [1000, 182], [1500, 198], [2000, 208], [2500, 217], [3000, 222], [3500, 247],
  [4000, 255], [4500, 261], [5000, 266], [5500, 267], [6000, 261], [6500, 254], [7000, 239], [7500, 228],
  [8000, 214], [8500, 190], [9000, 60]];
const sr20 = (bar, refRpm, gamma = 2.75) => gross(SR20_NA.map(([r, t]) => [r, r === 0 ? 0 : t * (1 + bar * Math.min(1, Math.pow(r / refRpm, gamma)))]));
const ENGINES = {
  // The S14 mod runs 2.0 bar (a ~650 hp VDC car); this is a built street SR20: 0.75 bar, full boost from
  // 3800 rpm (AC REFERENCE_RPM 3800, GAMMA 2.75) -> ~470 Nm, ~400 hp. Stock (makeS15Params uses it).
  street: {
    label: 'SR20DET', name: 'SR20DET, built street', fuel: 'petrol', note: 'Stock for this car: 0.75 bar, full boost from 3800 rpm. About 400 hp, revs to 8200.',
    torque: sr20(0.75, 3800),
    idleRpm: 950, limiterRpm: 8200, redlineRpm: 7800, shiftRpm: 7500, inertia: 0.16,
  },
  factory: {
    label: 'SR20 factory', name: 'SR20DET, factory', fuel: 'petrol', note: 'The Spec R as it left the factory: 250 PS, 275 Nm. Light, easy to balance.',
    torque: sr20(0.05, 3500, 2.2),
    idleRpm: 850, limiterRpm: 7600, redlineRpm: 7200, shiftRpm: 7000, inertia: 0.16,
  },
  bigTurbo: {
    label: 'SR20 big turbo', name: 'SR20DET, big single turbo', fuel: 'petrol', note: '1.5 bar on a big single: lag below 4500, then about 570 hp. Hard to meter out.',
    torque: sr20(1.5, 4800, 3.6),
    idleRpm: 1000, limiterRpm: 8600, redlineRpm: 8200, shiftRpm: 8000, inertia: 0.15,
  },
  jz: {
    label: '2JZ-GTE', name: '2JZ-GTE swap, single turbo', fuel: 'petrol', note: 'The classic swap: 3.0 straight six, about 650 Nm from 4000 rpm, 530 hp. Heavier nose, huge torque.',
    torque: gross([[0, 0], [500, 200], [1000, 250], [2000, 330], [3000, 480], [3500, 600], [4000, 640], [5000, 650],
      [6000, 620], [6500, 580], [7000, 520], [7500, 400], [8000, 150]]),
    idleRpm: 800, limiterRpm: 7600, redlineRpm: 7200, shiftRpm: 7000, inertia: 0.24,
  },
};

// ---------------------------------------------------------------- physics parameters
export function makeS15Params() {
  const wheelbase = 2.525;
  const track = 1.56;
  const E = ENGINES.street;
  return {
    name: 'Silvia S15',
    model: 's15',
    layout: 'rwd',              // no transfer case or front drive (drivetrain.js fixedRwd)
    rearDiff: 'welded',
    lsd: { preload: 80, power: 0.6, coast: 0.35 },
    bodyMass: 1120,
    // box inertia (AC car.ini INERTIA 1.72 x 1.195 x 4.72 m) for the sprung mass: pitch, yaw, roll
    bodyInertia: [2210, 2360, 410],
    com: [0, 0.45, -0.05],       // 52 % front
    aero: { cdA: 0.62, rho: 1.2 },
    wheelbase,
    track,
    // Independent suspension: MacPherson struts at the front, multilink at the rear (Vehicle.cornerSubstep).
    // Per corner: k is the wheel rate (N/m at the wheel), bump / rebound the damping at the wheel, mass the
    // axle's unsprung mass (both corners). arb is the bar's roll stiffness (N·m/rad), rcY the roll centre
    // height (a lowered strut car's sits near the ground), anti the anti-dive / anti-squat fraction.
    suspension: 'independent',
    axles: [
      {
        name: 'front', z: -wheelbase / 2, droopY: 0.29, travel: 0.11,
        mass: 90, k: 70000, preload: 0.0,
        bump: 2600, rebound: 4500, damperKnee: 0.15, damperHigh: 0.5,
        arb: 9000, rcY: 0.05, anti: 0.15,
        steered: true,
      },
      {
        name: 'rear', z: wheelbase / 2, droopY: 0.277, travel: 0.11,
        mass: 90, k: 45000, preload: 0.0,
        bump: 2200, rebound: 3800, damperKnee: 0.15, damperHigh: 0.5,
        arb: 3000, rcY: 0.09, anti: 0.3,
        steered: false,
      },
    ],
    tire: {
      radius: 0.33,              // 265/35R18 (the model's tyres)
      width: 0.245,
      rimRadius: 0.241,
      inertia: 1.8,
      pressure: 30, minPressure: 15, maxPressure: 45,   // psi
      damping: 400,              // radial damping N*s/m
      relaxX: 0.15, relaxY: 0.4, // relaxation lengths (m)
    },
    // quick rack: a slide is caught with the wheel, ~3.5 rad/s at the road wheels (lock to lock in ~0.4 s)
    steer: { maxAngle: 0.75, kingpinTrack: 1.45, ratio: 13, rate: 3.5, ackermann: 0.2, casterTrail: 0.025, pneuTrail: 0.03, stiffness: 40000 },
    brakes: { front: 1500, rear: 850, handbrake: 2500, handbrakeHold: 8000 },
    engine: {
      name: E.label,
      idleRpm: E.idleRpm, limiterRpm: E.limiterRpm, redlineRpm: E.redlineRpm, stallRpm: 300, shiftRpm: E.shiftRpm,
      inertia: E.inertia,
      torque: E.torque.map(([r, t]) => [r, t]),
      // starting (drivetrain.js startEngine): the values the player tuned by ear on the Defender's V8
      starterTorque: 260, starterRpm: 265, compressionTorque: 120, startFlare: 1000,
      catchMin: 0.35, catchSpread: 0.15, fireRamp: 0.2,
    },
    clutch: { capacity: 800, inputInertia: 0.02 },
    manual: { ratios: [2.833, 1.764, 1.289, 1.0, 0.833], reverse: 3.818 },
    // the S15 Spec S option: 4-speed automatic (RE4R01A ratios) with a torque converter
    auto: {
      ratios: [2.785, 1.545, 1.0, 0.694],
      reverse: 2.272,
      stallK: 120,               // converter capacity factor at stall (rpm / sqrt(Nm)): ~2600 rpm on boost
      turbineInertia: 0.04,
      lockupCapacity: 700,
      shiftTime: 0.35,
      shiftCapacity: 850,
    },
    finalDrive: 3.91,
    // chassis boxes fitted to the model: lower body, cabin, rear wing
    colliders: [
      [0, 0.45, 0.025, 0.94, 0.31, 2.2, 0.08],
      [0, 0.98, 0.2, 0.7, 0.2, 0.95, 0.08],
      [0, 1.03, 2.36, 0.8, 0.07, 0.11, 0.02],
    ],
  };
}

// ---------------------------------------------------------------- the car (registry entry, see index.js)
export default {
  id: 's15',
  name: 'Silvia S15',
  makeParams: makeS15Params,
  // what the car has: the game, HUD, menu, controls list and tuning panel read these, never the id
  capabilities: {
    drive: 'rwd',          // rear-wheel drive only (HUD: rear diff only)
    transfer: false,
    centreDiff: false,
    lockers: 'rearDiff',   // the lockers key cycles the rear diff: welded -> LSD -> open (tuning: Differential)
    selectableRwd: false,
    lightBar: false,
    tyrePressure: false,   // pressures are set in the tuning panel, not on the move
  },
  body: { archTop: 0.74, halfWidth: 0.86 },
  tuning: {
    engines: ENGINES, engineOrder: ['street', 'factory', 'bigTurbo', 'jz'], stockEngine: 'street',
    named: B => B.name,
    // 265/35R18 on the model is ~26" across; the physics tyre is 0.245 m wide
    tyre: { size: 26, width: 245, widthUnit: 'mm', sizes: [24, 25, 26, 27], widths: [215, 225, 235, 245, 255, 265, 275] },
    wheelMass: (inches, width) => 9 + 13 * Math.pow(inches / 26, 2.2) * (width / 245),
    colliderNames: ['Lower body', 'Cabin', 'Rear wing'],
    ranges: {
      'tyres.size': [24, 27, 1], 'tyres.width': [215, 275, 10], 'tyres.press': [15, 45, 1],
      'suspension.lift': [-0.05, 0.05, 0.005], 'suspension.k': [20000, 160000, 500], 'suspension.bump': [800, 8000, 100],
      'suspension.rebound': [1200, 12000, 100], 'suspension.arb': [0, 30000, 500], 'suspension.travel': [0.05, 0.16, 0.005],
      'mass.cargo': [0, 200, 5], 'mass.comY': [-0.1, 0.15, 0.01],
      'steering.lock': [30, 70, 0.5], 'steering.ratio': [9, 18, 0.5],
    },
    cargoPos: [0, 0.5, 1.75],     // spare-wheel well in the boot
    roofPos: [0, 1.25, 0.2],
    mu: 1.05, traction: 1.1, crr: 0.011, surface: 'asphalt', surfaceName: 'asphalt',
    defaultSetup: null,
    ui: {
      noun: 'car', lift: 'Ride height', cargo: 'Ballast (boot)', roof: false, crawl: false, offroadGeo: false,
      hints: {
        final: 'Axle ratio. Shorter (higher) keeps a drift in the power band on a tight track.',
        pressure: 'Lower: a softer, more forgiving slide. Higher: sharper response, less grip.',
        grip: 'Grip on every surface. Street tyres = 1; drifters often run less at the rear.',
        lift: 'Lower: less roll and a lower centre of mass, less travel before the bump stops.',
        cargo: 'In the spare-wheel well: more weight on the rear tyres.',
        lock: 'Road-wheel angle at full lock. An angle kit (55–70°) holds deeper drifts.',
        ride: 'street ~1.5–2 Hz, track 2–3 Hz',
      },
    },
  },
  async build(vehicle) {
    const [{ loadCarModel }, { CarView }] = await Promise.all([import('../vehicle/carModel.js'), import('../vehicle/carView.js')]);
    const model = await loadCarModel(`${import.meta.env.BASE_URL}models/s15.glb`);
    return { model, view: new CarView(model, vehicle) };
  },
};
