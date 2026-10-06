// Engine torque curves. The drivetrain subtracts friction and pumping losses from the gross curve, so a
// published (net, at the flywheel) curve needs them added back.

// engine friction + pumping at full throttle (drivetrain.js substep, thr = 1)
export const engineFriction = rpm => 12 + 0.0085 * Math.abs(rpm) + 1.2e-6 * rpm * rpm;
// a published (net, at the flywheel) torque curve -> the gross curve the drivetrain uses
export const gross = net => net.map(([r, t]) => [r, t > 0 ? Math.round(t + engineFriction(r)) : 0]);
