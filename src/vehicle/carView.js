import * as THREE from 'three';
import { VehicleView } from './vehicleView.js';
import { shared } from './truckMaterials.js';

// Drives a GLB car (carModel.js) from the physics: body pose, hubs from the axle state, steering, spin,
// lamps. The model keeps its own track: each wheel sits at the physics hub plus the model's lateral
// offset from track / 2.

const _v = new THREE.Vector3();
const Z = new THREE.Vector3(0, 0, 1);
const reversingNow = d => (d.mode === 'manual' ? d.manualGear < 0 : d.selector === 'R');

export class CarView extends VehicleView {
  constructor(model, vehicle) {
    super(model, vehicle);
    // lateral offset of each model wheel from the physics hub (model track vs P.track)
    this.dx = model.wheels.map((w, i) => w.rest.x - vehicle.wheels[i].side * this.P.track / 2);
  }

  update(pos, quat, dt, env) {
    const m = this.m, v = this.v, P = this.P;
    m.root.position.copy(pos);
    m.root.quaternion.copy(quat);
    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i], mw = m.wheels[i], a = w.axle;
      // axle frame (as the physics: centre at droopY + c, rolled by phi about z) -> body frame
      _v.set(w.side * P.track / 2 + this.dx[i], 0, 0).applyAxisAngle(Z, a.phi);
      mw.hub.position.set(_v.x, _v.y + a.droopY + a.c, mw.rest.z);
      mw.hub.rotation.set(0, 0, a.phi);
      mw.steer.rotation.y = -w.steer;
      mw.spin.rotation.x = -w.spin;
    }
    m.root.updateMatrixWorld(true);
    this.updateLights(dt, env, quat);
  }

  updateLights(dt, env, quat) {
    const m = this.m, v = this.v, L = m.lights, ls = this.lights, lamps = m.lamps;
    const head = ls.head;
    const braking = v.ctl.brake > 0.05;
    const reversing = reversingNow(v.drivetrain);
    const tailOn = head > 0;
    const live = env.night || head > 0;
    for (const l of [L.head, L.rear]) l.visible = live;
    L.bar.visible = false;
    const k = Math.min(1, Math.max(0.12, 0.42 / Math.max(this.ambientLevel(), 1e-3)));

    const hp = L.head.userData.peak;
    L.head.map = head === 2 ? L.cookies.high : L.cookies.low;
    L.head.intensity = head === 0 ? 0 : (head === 1 ? hp.low : hp.high) * k;
    L.head.distance = head === 2 ? 260 : 150;
    L.head.shadow.autoUpdate = live && head > 0 && env.shadows !== false;
    L.head.shadow.camera.up.set(0, 1, 0).applyQuaternion(quat);

    const glow = (list, x) => { for (const mt of list) mt.emissiveIntensity = x; };
    glow(lamps.head, head === 0 ? 0 : head === 1 ? 2.6 : 5.0);
    glow(lamps.tail, braking ? 3.2 : tailOn ? 1.2 : 0);
    glow(lamps.reverse, reversing ? 4.0 : 0);
    this.blink += dt;
    glow(lamps.amber, ls.hazard && (this.blink % 0.8) < 0.4 ? 5.0 : 0);

    const red = (braking ? 0.5 : 0) + (tailOn ? 0.08 : 0);
    const white = reversing ? 9 : 0;
    L.rear.intensity = (red + white) * k;
    if (red + white > 0) L.rear.color.setRGB(1, 0.16 + 0.81 * white / (red + white), 0.06 + 0.86 * white / (red + white));

    const dark = env.darkness ?? (env.night ? 1 : 0);
    shared.uEnvSpec.value = 1.0 + 0.6 * dark;
  }
}
