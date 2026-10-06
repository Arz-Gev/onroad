// Defender 110: the inherited offroad truck. A lifted 4x4 on solid beam axles with a transfer case,
// lockable centre diff, axle lockers, a roof light bar and tyres you air down on the trail.
// Model, view and lamps: vehicle/truckModel.js (+ truckBody, truckInterior, truckLights, truckMaterials,
// truckDims) and vehicle/vehicleView.js, loaded only when this car is driven.
//
// Vehicle-local frame: +x right, +y up, -z forward. y = 0 is the ground at static ride height,
// z = 0 is the middle of the wheelbase.

import { gross } from '../vehicle/engineCurve.js';
import { D } from '../vehicle/truckDims.js';

export function makeDefenderParams() {
  const wheelbase = 2.794;
  const track = 1.56;
  return {
    name: 'Defender 110 V8',
    model: 'defender',
    bodyMass: 1880,                       // sprung mass incl. roof rack, winch, spare
    // principal inertia about the COM: pitch (x), yaw (y), roll (z). The axles' mass sits in this body, so
    // yaw includes them at +-1.4 m (they swing sideways with the body); pitch and roll only their small
    // horizontal part (their vertical motion is the axle DOF).
    bodyInertia: [3400, 4400, 1040],
    // centre of mass (local): 51 % on the front axle (engine, winch and bumper ahead of it)
    com: [0, 0.90, -0.04],
    aero: { cdA: 2.15, rho: 1.2 },
    wheelbase,
    track,

    // Solid beam axles on coil springs (front: radius arms + panhard, rear: trailing links + A-frame).
    // y values are the axle centre height (local) at full droop; c is compression measured at the springs.
    axles: [
      {
        name: 'front', z: -wheelbase / 2, droopY: 0.275, travel: 0.24,
        mass: 185, rollInertia: 75,
        springTrack: 1.0, k: 43000, preload: 0.0,
        // dampers sit outboard, close to the wheels; digressive above the knee (m/s)
        damperTrack: 1.24, bump: 3800, rebound: 6000, damperKnee: 0.22, damperHigh: 0.55,
        // roll stiffness ~53 % front, a little over its weight share: a stable, mildly understeering truck
        arb: 11000,
        steered: true,
      },
      {
        name: 'rear', z: wheelbase / 2, droopY: 0.27, travel: 0.25,
        mass: 175, rollInertia: 72,
        springTrack: 1.04, k: 47000, preload: 0.0,
        damperTrack: 1.24, bump: 4000, rebound: 6300, damperKnee: 0.22, damperHigh: 0.55,
        arb: 4000,
        // trailing links + A-frame: the axle turns slightly into the bend as the body rolls (rad / rad)
        rollSteer: 0.06,
        steered: false,
      },
    ],

    tire: {
      radius: 0.42,            // 33x10.5R16 mud terrain
      width: 0.27,
      rimRadius: 0.205,
      inertia: 3.3,            // wheel + tyre + hub + half shaft
      pressure: 20,            // psi (default, adjustable in game)
      minPressure: 6, maxPressure: 38,
      damping: 650,            // radial damping N*s/m
      relaxX: 0.22,            // longitudinal relaxation length (m) at reference pressure
      relaxY: 0.55,            // lateral relaxation length (m)
    },

    steer: {
      maxAngle: 0.62, kingpinTrack: 1.38, ratio: 17,
      // compliance: side force x (caster trail + pneumatic trail) about the kingpins over the stiffness of
      // the steering box, drag link and track rod (N·m/rad at the road wheels, both wheels together)
      casterTrail: 0.022, pneuTrail: 0.035, stiffness: 50000,
    },

    brakes: { front: 2700, rear: 1350, handbrake: 3200, handbrakeHold: 24000 }, // max torque per wheel (handbrake: at rear axle; hold: whole transfer output at a stop)

    engine: {
      name: '4.6 V8',
      idleRpm: 720, limiterRpm: 5350, redlineRpm: 5000, stallRpm: 280,
      shiftRpm: 4800,           // automatic: upshift at full throttle (lighter throttle shifts earlier)
      inertia: 0.22,
      // gross torque (Nm) at full throttle; friction and pumping losses are subtracted separately
      torque: [[0, 0], [400, 150], [800, 300], [1200, 365], [1600, 405], [2000, 435], [2500, 455], [3000, 462],
               [3500, 458], [4000, 445], [4500, 420], [5000, 385], [5500, 330], [6000, 250], [6500, 120]],
      starterTorque: 260, starterRpm: 265,  // starter at the crank: stall torque (Nm) and no-load speed (rpm); cranks at ~200 rpm
      compressionTorque: 120, startFlare: 1000, // compression strokes (Nm peak) when not firing; idle flare on start (rpm)
      catchMin: 0.35, catchSpread: 0.15, fireRamp: 0.2, // cranking before it fires (s, + random), all cylinders firing after (s)
    },

    clutch: { capacity: 680, inputInertia: 0.035 },

    manual: {   // R380 style 5 speed
      ratios: [3.585, 2.301, 1.507, 1.0, 0.83],
      reverse: 3.70,
    },
    auto: {     // ZF 6HP style 6 speed + torque converter
      ratios: [4.17, 2.34, 1.52, 1.14, 0.87, 0.69],
      reverse: 3.40,
      stallK: 112,               // converter capacity factor at stall (rpm / sqrt(Nm))
      turbineInertia: 0.06,
      lockupCapacity: 750,
      shiftTime: 0.38,
      shiftCapacity: 900,
    },
    transfer: { high: 1.211, low: 3.32 },
    finalDrive: 3.54,

    // chassis collision boxes: [cx, cy, cz, hx, hy, hz, rounding], fitted to the visible model (truckBody.js)
    // on Oct 3: the old front bumper box sat 10 cm lower and 11 cm further forward than the bumper you see
    // (an invisible nose: approach 32° instead of 38°). Names in tuning.js (COLLIDER_NAMES), same order.
    colliders: [
      [0, 1.33, 0.745, 0.89, 0.67, 1.585, 0.06],    // cabin / rear body: sill 0.66 to roof 2.0, bulkhead to rear face 2.33
      [0, 1.00, -1.36, 0.87, 0.38, 0.64, 0.06],     // engine bay / wings: 0.62 to the raised bonnet centre 1.38, wing face -2.0
      [0, 0.6775, -2.0875, 0.985, 0.1225, 0.1375, 0.03], // winch bumper plate 0.555-0.80, face at -2.225 (D-ring shackles hang below: soft)
      [0, 0.63, 2.4025, 0.955, 0.095, 0.0925, 0.03],  // rear bumper 0.535-0.725, face 2.495
      [0, 0.54, 0.205, 0.48, 0.08, 2.145, 0.03],     // chassis rails + crossmembers 0.46-0.62, -1.94 to 2.35
      [0, 2.20, 0.70, 0.84, 0.13, 1.58, 0.03],       // roof rack (top 2.33)
      [0, 1.18, 2.62, 0.36, 0.38, 0.15, 0.06],       // spare wheel
      [0, 0.51, -0.625, 0.30, 0.09, 1.125, 0.04],    // belly: sump, gearbox, transfer case (lowest, 0.42)
      [0.58, 0.56, 1.91, 0.22, 0.08, 0.29, 0.03],    // fuel tank, right of the rear rails
    ],
  };
}

// ---------------------------------------------------------------- engines (tuning panel)
// curves: gross torque (Nm) at full throttle. Real figures are net (flywheel) torque: gross() adds
// the friction the drivetrain subtracts again. shiftRpm: automatic upshift at full throttle.
const ENGINES = {
  v8: {
    label: '4.6 V8', name: '4.6 V8 petrol', fuel: 'petrol', note: 'Stock. Rover V8, 225 PS. Smooth and quick.',
    torque: [[0, 0], [400, 150], [800, 300], [1200, 365], [1600, 405], [2000, 435], [2500, 455], [3000, 462],
      [3500, 458], [4000, 445], [4500, 420], [5000, 385], [5500, 330], [6000, 250], [6500, 120]],
    idleRpm: 720, limiterRpm: 5350, redlineRpm: 5000, shiftRpm: 4800, inertia: 0.22,
  },
  works: {
    label: '5.0 V8', name: 'Works V8 5.0 petrol', fuel: 'petrol', note: 'Land Rover Classic Works V8: 405 PS, 515 Nm. Revs to 6800.',
    torque: gross([[0, 0], [400, 150], [800, 330], [1200, 380], [2000, 430], [3000, 475], [4000, 505], [4500, 515],
      [5000, 510], [5500, 495], [6000, 474], [6500, 430], [7000, 300], [7500, 120]]),
    idleRpm: 650, limiterRpm: 6800, redlineRpm: 6500, shiftRpm: 6300, inertia: 0.2,
  },
  v35: {
    label: '3.5 V8', name: '3.5 V8 petrol (carburettor)', fuel: 'petrol', note: 'The 1983 110: 114 PS, 251 Nm. Slow but sweet.',
    torque: gross([[0, 0], [400, 80], [800, 170], [1200, 210], [1600, 230], [2000, 243], [2500, 251], [3000, 247],
      [3500, 232], [4000, 203], [4500, 170], [5000, 125], [5500, 70], [6000, 20]]),
    idleRpm: 700, limiterRpm: 5000, redlineRpm: 4750, shiftRpm: 4500, inertia: 0.24,
  },
  tdi300: {
    label: '300Tdi', name: '2.5 300Tdi diesel', fuel: 'diesel', note: '1994–98: 111 PS, 265 Nm at 1800. Pulls low, runs out of breath at 4000.',
    torque: gross([[0, 0], [400, 50], [800, 130], [1200, 200], [1500, 245], [1800, 265], [2400, 255], [3000, 235],
      [3500, 215], [4000, 198], [4400, 120], [4700, 40]]),
    idleRpm: 720, limiterRpm: 4400, redlineRpm: 4000, shiftRpm: 3900, inertia: 0.32,
  },
  td5: {
    label: 'Td5', name: '2.5 Td5 diesel', fuel: 'diesel', note: '1998–2007: 122 PS, 300 Nm at 1950. Pulls from low revs, takes its time to 100.',
    torque: gross([[0, 0], [400, 60], [800, 150], [1200, 215], [1500, 265], [1950, 300], [2500, 292], [3000, 270],
      [3500, 245], [4000, 215], [4200, 205], [4500, 150], [4800, 60]]),
    idleRpm: 760, limiterRpm: 4600, redlineRpm: 4300, shiftRpm: 4200, inertia: 0.3,
  },
  puma: {
    label: '2.4 TDCi', name: '2.4 TDCi Puma diesel', fuel: 'diesel', note: '2007–11: 122 PS, 360 Nm at 2000. The most torque of the diesels.',
    torque: gross([[0, 0], [400, 60], [800, 140], [1200, 230], [1500, 310], [2000, 360], [2500, 355], [3000, 315],
      [3500, 250], [4000, 170], [4300, 60]]),
    idleRpm: 780, limiterRpm: 4200, redlineRpm: 4000, shiftRpm: 3800, inertia: 0.3,
  },
};

// ---------------------------------------------------------------- the car (registry entry, see index.js)
export default {
  id: 'defender',
  name: 'Defender 110',
  makeParams: makeDefenderParams,
  // what the car has: the game, HUD, menu, controls list and tuning panel read these, never the id
  capabilities: {
    drive: '4x4',          // all four wheels driven (HUD: three-diff diagram)
    transfer: true,        // HIGH / LOW transfer case
    centreDiff: true,      // lockable centre diff
    lockers: 'axles',      // the lockers key: front / rear axle lockers
    selectableRwd: true,   // 2WD / 4WD switch
    lightBar: true,        // roof light bar
    tyrePressure: true,    // air down / up while driving
  },
  // body outline for the physics overlay (colliderView): arch-top height and lower-body half width
  body: { archTop: D.ARCH_TOP, halfWidth: D.W },
  // tuning profile (vehicle/tuning.js makeProfile): engines, tyre sizes, slider ranges, panel texts
  tuning: {
    engines: ENGINES, engineOrder: ['v8', 'works', 'v35', 'tdi300', 'td5', 'puma'], stockEngine: 'v8',
    named: (B, E, preset) => (preset === 'v8' ? B.name : 'Defender 110 ' + E.label),
    tyre: { size: 33, width: 10.5, widthUnit: 'in', sizes: [31, 32, 33, 34, 35, 37], widths: [9.5, 10.5, 11.5, 12.5, 13.5] },
    // steel rim + hub part plus a tyre that grows with diameter and width; 42 kg stock
    wheelMass: (inches, width) => 20 + 22 * Math.pow(inches / 33, 2.2) * (width / 10.5),
    colliderNames: ['Cabin / rear body', 'Engine bay', 'Front bumper', 'Rear bumper', 'Chassis rails', 'Roof rack', 'Spare wheel', 'Belly', 'Fuel tank'],
    ranges: {},
    cargoPos: [0, 0.98, 1.45],    // load bay floor, behind the rear seats
    roofPos: [0, 2.45, 0.75],     // on the rack
    mu: 0.72, traction: 0.76, crr: 0.026, surface: 'dirt', surfaceName: 'dirt',
    // an exported setup (panel: Export) pasted here becomes the stock setup (null = the params above)
    defaultSetup: null,
    // v1 setups carry the Oct 2 anti-roll bars (8000 / 8500, rear-biased: the truck spun out when you lifted
    // off at speed). Untouched stock values there take the current stock bars.
    migrate(s, su) { if (!(s.v >= 2) && su.front?.arb === 8000 && su.rear?.arb === 8500) { delete su.front.arb; delete su.rear.arb; } },
    ui: {
      noun: 'truck', lift: 'Lift', cargo: 'Cargo', roof: true, crawl: true, offroadGeo: true,
      hints: {
        final: 'Axle ratio. 4.10 or 4.75 is the usual fix for big tyres.',
        pressure: 'Lower: bigger footprint, more grip off-road, softer ride. [ ] change both.',
        grip: 'Grip on every surface. Mud terrain = 1.',
        lift: 'Longer springs: more ground clearance and room in the arches, higher centre of mass.',
        cargo: 'In the load bay behind the rear seats.',
        lock: 'Road-wheel angle at full lock: a tighter turning circle.',
        ride: 'comfort ~1.0–1.5 Hz',
      },
    },
  },
  // model + view, loaded on demand (three.js modules stay out of the headless tools)
  async build(vehicle) {
    const [{ buildTruck }, { VehicleView }] = await Promise.all([import('../vehicle/truckModel.js'), import('../vehicle/vehicleView.js')]);
    const model = buildTruck(makeDefenderParams());   // modelled stock; the view scales the wheels and follows the lift
    return { model, view: new VehicleView(model, vehicle) };
  },
};
