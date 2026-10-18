# Assetto Corsa physics in onroad

`src/ac/` is a JavaScript port of Assetto Corsa's car physics, taken from
[rustyAC](https://github.com/thesupersupersigma/rustyAC) (a line-by-line Rust port of `acs.exe`, rev `78b8990`).
The S15 drives on it by default, built from an AC car's data files.

- `?acdata=s14` (default), `s13`, `180sx`: which AC car's data the S15 uses (packed in `public/ac/cars/`).
- `?physics=old`: the old hand-made S15 physics.

## What is ported

| File | What |
|---|---|
| `ode.js` | The part of ODE 0.13.1 (the rigid-body solver) AC uses: bodies, rods (DBall), ball, slider, fixed and contact joints, `dWorldStep` with the Dantzig LCP solver |
| `ini.js` | AC's `.ini` and `.lut` reading, with the game's quirks |
| `tyre.js` | `Tyre::step`: ground ray, radial spring, slip with relaxation length, the SCTM force model (tyres.ini VERSION 10+), rolling resistance, 36-patch heat model, wear, dirt, grain, blisters, flat spots |
| `suspension.js` | Double wishbone (5 rods) and MacPherson strut (2 rods, steering rod, strut body on a slider and ball joint), springs, dampers, bump stops, packers, anti-roll bars |
| `powertrain.js` | Engine with turbos and `ctrl_*.ini` controllers, 2WD drivetrain (clutch, gearbox, LSD / spool), auto clutch, auto blip, auto shifter, brakes, aero (wings, fins, or the DATA body), aids (TC, ABS, EDL, stability, pit limiter) |
| `car.js` | The car, stepped at 3 ms in `Car::step` / `Car::stepComponents` order, then the ODE step |
| `AcVehicle.js`, `AcCarView.js` | The bridge to the game: tyre rays into the Rapier track, AC surfaces (`surfaces.js`), body contacts against walls, the fields the HUD / audio / effects / camera read, the model's wheels on the AC wheel carriers, Y / B as AC's TC / ABS assist "on" (fits the aid to a car without it; ABS starts on) |

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

rustyAC's physics is GPL-3.0-or-later (its ODE port BSD-3-Clause), and `src/ac/` is derived from it.
