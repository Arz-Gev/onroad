# Assetto Corsa physics in onroad

`src/ac/` is a JavaScript port of Assetto Corsa's car physics, taken from
[rustyAC](https://github.com/thesupersupersigma/rustyAC) (a line-by-line Rust port of `acs.exe`, rev `78b8990`).
The S15 drives on it by default, built from an AC car's data files.

Settings → Car → Physics (Assetto Corsa / Onroad) and AC car data (S14 / S13 / 180SX, packed in `public/ac/cars/`)
pick them; `?physics=ac|old` and `?acdata=s14|s13|180sx` override them for one load.

## What is ported

| File | What |
|---|---|
| `ode.js` | The part of ODE 0.13.1 (the rigid-body solver) AC uses: bodies, rods (DBall), ball, slider, fixed and contact joints, `dWorldStep` with the Dantzig LCP solver |
| `ini.js` | AC's `.ini` and `.lut` reading, with the game's quirks |
| `tyre.js` | `Tyre::step`: ground ray, radial spring, slip with relaxation length, the SCTM force model (tyres.ini VERSION 10+), rolling resistance, 36-patch heat model, wear, dirt, grain, blisters, flat spots |
| `suspension.js` | Double wishbone (5 rods) and MacPherson strut (2 rods, steering rod, strut body on a slider and ball joint), springs, dampers, bump stops, packers, anti-roll bars |
| `powertrain.js` | Engine with turbos and `ctrl_*.ini` controllers, 2WD drivetrain (clutch, gearbox, LSD / spool), auto clutch, auto blip, auto shifter, brakes, aero (wings, fins, or the DATA body), aids (TC, ABS, EDL, stability, pit limiter) |
| `car.js` | The car, stepped at 3 ms in `Car::step` / `Car::stepComponents` order, then the ODE step |
| `controls.js` | Keys and pad to steering / gas / brake, the menu's "Steering help": AC's own `KeyboardCarControl` (speed-limited steering, gas that backs off on wheelspin, brake at `RaceEngineer::getOptimalBrake`) and `JoypadCarControl` (stock gamepad preset); or a port of [Advanced Gamepad Assist](https://github.com/adam10603/AC-Advanced-Gamepad-Assist) 1.5.5 (MIT, `LICENSE-advanced-gamepad-assist`): steering limit at the front tyres' peak slip angle, self-steer in slides, keyboard gas / brake help, with its steering calibration run on a copy of the car at load; or the raw input |
| `AcVehicle.js`, `AcCarView.js` | The bridge to the game: tyre rays into the Rapier track, AC surfaces (`surfaces.js`), body contacts against walls, the fields the HUD / audio / effects / camera read, the model's wheels on the AC wheel carriers, Y / B as AC's TC / ABS assist "on" (fits the aid to a car without it; ABS starts on), AC's stability control assist (Settings → Driving aids) |

Not ported yet: rigid rear axle (AXLE) and multilink (ML) suspensions, AWD / AWD2, tyres.ini below VERSION 10,
the setup screen's rounding of setup values, collision damage, KERS / ERS, DRS, wind, slipstream. AC's car
collider mesh (`collider.kn5`) is not available for these mods, so a ring of contact points on the model's
bounding box stands in for it against walls.

## How it was checked

`tools/ac/ref` drives rustyAC's car with a script; `tools/ac/compare.mjs` drives the JS port with the same script
and compares both cars after every 3 ms step (body, wheel carriers, tyre loads, slips, forces, wheel speeds,
suspension travel, rpm, gear, boost, clutch).

```
npm run ac:ref        # build the reference (needs Rust)
npm run ac:compare    # all scenarios; `node tools/ac/compare.mjs launch` for one, with every quantity
```

With the S14 data: standing still matches the whole way; the driving scenarios (launch, braking, corner, slalom,
handbrake turn, power slide) stay together for 2.3 to 6.9 s, and end 0 to 6.5 cm apart after 8 to 10 s. They part
where the car is unstable (an oscillating auto clutch, a spin): rustyAC computes in 32-bit floats like the game, the
port in 64-bit, and the tiny rounding differences grow there. No quantity jumps apart, so nothing is missing.

## Licence

rustyAC's physics is GPL-3.0-or-later (its ODE port BSD-3-Clause), and `src/ac/` is derived from it. The Advanced
Gamepad Assist part of `controls.js` is MIT (© 2023 Adam D., `src/ac/LICENSE-advanced-gamepad-assist`).
