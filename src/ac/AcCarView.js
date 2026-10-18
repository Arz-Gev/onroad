// The car model driven by the Assetto Corsa car: the body at the AC body (through AcVehicle.pos / quat), each
// wheel at its AC wheel carrier: height and fore-aft from the physics, sideways at the model's own track (the S14
// data is wider than the S15 body), turned and cambered by the carrier's attitude, spun by the tyre.
import * as THREE from 'three';
import { CarView } from '../vehicle/carView.js';

const _b = new THREE.Matrix4(), _h = new THREE.Matrix4(), _inv = new THREE.Matrix4(), _q = new THREE.Quaternion(), _v = new THREE.Vector3();
const HALF_TURN = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);

export class AcCarView extends CarView {
  constructor(model, vehicle) {
    super(model, vehicle);
    this.restLocal = vehicle.wheels.map(w => vehicle.hubLocal(w, new THREE.Vector3()));
  }

  update(pos, quat, dt, env) {
    const m = this.m, v = this.v;
    m.root.position.copy(pos);
    m.root.quaternion.copy(quat);
    v.bodyMatrix(_b);
    _inv.copy(_b).invert();
    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i], mw = m.wheels[i];
      v.hubLocal(w, _v);
      mw.hub.position.set(mw.rest.x + (_v.x - this.restLocal[i].x), _v.y, _v.z);
      // the carrier's attitude relative to the body, in the model's frame (a half turn about y from AC's)
      _h.multiplyMatrices(_inv, v.hubMatrix(i, _h));
      _q.setFromRotationMatrix(_h);
      mw.hub.quaternion.copy(HALF_TURN).multiply(_q).multiply(HALF_TURN);
      mw.steer.rotation.y = 0;
      mw.spin.rotation.x = -w.spin;
    }
    m.root.updateMatrixWorld(true);
    this.updateLights(dt, env, quat);
  }
}
