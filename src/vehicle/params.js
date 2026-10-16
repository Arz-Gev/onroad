export const PSI = 6894.76;

export function makeDefenderParams() {
  const wheelbase = 2.794;
  const track = 1.56;
  return {
    name: 'Defender 110 V8',
    model: 'defender',
    bodyMass: 1880,

    bodyInertia: [3400, 4400, 1040],

    com: [0, 0.90, -0.04],
    aero: { cdA: 2.15, rho: 1.2 },
    wheelbase,
    track,

    axles: [
      {
        name: 'front', z: -wheelbase / 2, droopY: 0.275, travel: 0.24,
        mass: 185, rollInertia: 75,
        springTrack: 1.0, k: 43000, preload: 0.0,

        damperTrack: 1.24, bump: 3800, rebound: 6000, damperKnee: 0.22, damperHigh: 0.55,

        arb: 11000,
        steered: true,
      },
      {
        name: 'rear', z: wheelbase / 2, droopY: 0.27, travel: 0.25,
        mass: 175, rollInertia: 72,
        springTrack: 1.04, k: 47000, preload: 0.0,
        damperTrack: 1.24, bump: 4000, rebound: 6300, damperKnee: 0.22, damperHigh: 0.55,
        arb: 4000,

        rollSteer: 0.06,
        steered: false,
      },
    ],

    tire: {
      radius: 0.42,
      width: 0.27,
      rimRadius: 0.205,
      inertia: 3.3,
      pressure: 20,
      minPressure: 6, maxPressure: 38,
      damping: 650,
      relaxX: 0.22,
      relaxY: 0.55,
    },

    steer: {
      maxAngle: 0.62, kingpinTrack: 1.38, ratio: 17,

      casterTrail: 0.022, pneuTrail: 0.035, stiffness: 50000,
    },

    brakes: { front: 2700, rear: 1350, handbrake: 3200, handbrakeHold: 24000 },

    engine: {
      name: '4.6 V8',
      idleRpm: 720, limiterRpm: 5350, redlineRpm: 5000, stallRpm: 280,
      shiftRpm: 4800,
      inertia: 0.22,

      torque: [[0, 0], [400, 150], [800, 300], [1200, 365], [1600, 405], [2000, 435], [2500, 455], [3000, 462],
               [3500, 458], [4000, 445], [4500, 420], [5000, 385], [5500, 330], [6000, 250], [6500, 120]],
      starterTorque: 260, starterRpm: 265,
      compressionTorque: 120, startFlare: 1000,
      catchMin: 0.35, catchSpread: 0.15, fireRamp: 0.2,
    },

    clutch: { capacity: 680, inputInertia: 0.035 },

    manual: {
      ratios: [3.585, 2.301, 1.507, 1.0, 0.83],
      reverse: 3.70,
    },
    auto: {
      ratios: [4.17, 2.34, 1.52, 1.14, 0.87, 0.69],
      reverse: 3.40,
      stallK: 112,
      turbineInertia: 0.06,
      lockupCapacity: 750,
      shiftTime: 0.38,
      shiftCapacity: 900,
    },
    transfer: { high: 1.211, low: 3.32 },
    finalDrive: 3.54,

    colliders: [
      [0, 1.33, 0.745, 0.89, 0.67, 1.585, 0.06],
      [0, 1.00, -1.36, 0.87, 0.38, 0.64, 0.06],
      [0, 0.6775, -2.0875, 0.985, 0.1225, 0.1375, 0.03],
      [0, 0.63, 2.4025, 0.955, 0.095, 0.0925, 0.03],
      [0, 0.54, 0.205, 0.48, 0.08, 2.145, 0.03],
      [0, 2.20, 0.70, 0.84, 0.13, 1.58, 0.03],
      [0, 1.18, 2.62, 0.36, 0.38, 0.15, 0.06],
      [0, 0.51, -0.625, 0.30, 0.09, 1.125, 0.04],
      [0.58, 0.56, 1.91, 0.22, 0.08, 0.29, 0.03],
    ],
  };
}

export function makeS15Params() {
  const P = makeDefenderParams();
  const wheelbase = 2.525;
  const track = 1.56;

  const friction = rpm => 12 + 0.0085 * rpm + 1.2e-6 * rpm * rpm;
  const boost = rpm => 0.75 * Math.min(1, Math.pow(rpm / 3800, 2.75));
  const sr20 = [[0, 0], [500, 176], [1000, 182], [1500, 198], [2000, 208], [2500, 217], [3000, 222], [3500, 247],
    [4000, 255], [4500, 261], [5000, 266], [5500, 267], [6000, 261], [6500, 254], [7000, 239], [7500, 228],
    [8000, 214], [8500, 190], [9000, 60]];
  return {
    ...P,
    name: 'Silvia S15',
    model: 's15',
    layout: 'rwd',
    rearDiff: 'welded',
    lsd: { preload: 80, power: 0.6, coast: 0.35 },
    bodyMass: 1120,

    bodyInertia: [2210, 2360, 410],
    com: [0, 0.45, -0.05],
    aero: { cdA: 0.62, rho: 1.2 },
    wheelbase,
    track,

    suspension: 'independent',
    axles: [
      {
        name: 'front', z: -wheelbase / 2, droopY: 0.261, travel: 0.16,
        mass: 90, k: 55000, preload: 0,
        bump: 4800, rebound: 8000, damperKnee: 0.15, damperHigh: 0.5,
        arb: 32000, rcY: 0.05, anti: 0.15, camber: -1,
        bumpStop: [40000, 1.2e7],
        steered: true,
      },
      {
        name: 'rear', z: wheelbase / 2, droopY: 0.27, travel: 0.16,
        mass: 90, k: 45000, preload: 0,
        bump: 3600, rebound: 7500, damperKnee: 0.15, damperHigh: 0.5,
        arb: 9500, rcY: 0.09, anti: 0.3, camber: -1,
        bumpStop: [40000, 1.2e7],
        steered: false,
      },
    ],
    tire: {
      ...P.tire,
      radius: 0.33,
      width: 0.245,
      rimRadius: 0.241,
      inertia: 1.8,
      pressure: 30, minPressure: 15, maxPressure: 45,
      damping: 400,
      relaxX: 0.15, relaxY: 0.4,
    },

    steer: { maxAngle: 0.65, kingpinTrack: 1.40, ratio: 15, ackermann: 0.7, casterTrail: 0.0376, pneuTrail: 0.03,
      kpi: 13, kpiOffset: 0.06, inertia: 3, damping: 200, friction: 4, handStiffness: 8000, handDamping: 60, handTorque: 800 },
    brakes: { front: 1360, rear: 990, handbrake: 2500, handbrakeHold: 8000 },
    engine: {
      ...P.engine,
      name: 'SR20DET',
      idleRpm: 950, limiterRpm: 8200, redlineRpm: 7800, stallRpm: 300, shiftRpm: 7500,
      inertia: 0.16,
      torque: sr20.map(([r, t]) => [r, r === 0 ? 0 : Math.round(t * (1 + boost(r)) + friction(r))]),
    },
    clutch: { capacity: 800, inputInertia: 0.02 },
    manual: { ratios: [2.833, 1.764, 1.289, 1.0, 0.833], reverse: 3.818 },
    transfer: { high: 1.0, low: 1.0 },
    finalDrive: 3.91,

    colliders: [
      [0, 0.45, 0.025, 0.94, 0.31, 2.2, 0.08],
      [0, 0.98, 0.2, 0.7, 0.2, 0.95, 0.08],
      [0, 1.03, 2.36, 0.8, 0.07, 0.11, 0.02],
    ],
  };
}
