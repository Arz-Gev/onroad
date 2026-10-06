// Car registry. Everything car-specific lives in the car's own module here: physics params, tuning
// profile (engines, tyre sizes, slider ranges, panel texts), capabilities, body outline and the model /
// view loader. The game, HUD, menu, controls list and tuning panel read the active car's `capabilities`
// and never compare car ids. A new car is a new module in this folder plus one line in CARS.
//
// A car module's default export:
//   id, name                 registry key (also P.model, the tuning storage key) and display name
//   makeParams()             fresh physics params (vehicle frame, see DEVNOTES)
//   capabilities             { drive: '4x4' | 'rwd', transfer, centreDiff, lockers: 'axles' | 'rearDiff' | false,
//                              selectableRwd, lightBar, tyrePressure }
//   body                     { archTop, halfWidth }: arch-top height and lower-body half width (physics overlay)
//   tuning                   the profile vehicle/tuning.js builds the setup system from
//   build(vehicle)           async: loads the model and returns { model, view } (three.js only loads here)

import defender from './defender.js';
import s15 from './s15.js';

export const CARS = [s15, defender];
export const DEFAULT_CAR = s15.id;

const BY_ID = Object.fromEntries(CARS.map(c => [c.id, c]));

// the car with this id, or undefined
export const findCar = id => BY_ID[id];
// the car with this id; unknown ids get the default car
export const getCar = id => BY_ID[id] || BY_ID[DEFAULT_CAR];
// ?car=<id> picks the car; the default car otherwise
export const carFromURL = (search = globalThis.location?.search || '') => getCar(new URLSearchParams(search).get('car'));
