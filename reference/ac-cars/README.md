# Assetto Corsa car physics (reference)

Physics data (`data/` folder) from three AC drift car mods: `car.ini` (mass, inertia), `engine.ini` +
`power.lut` (torque curve, turbo boost), `drivetrain.ini` (gears, final drive, diff lock), `suspensions.ini`
(wheelbase, track, springs, camber), `tyres.ini`, `brakes.ini`, `aero.ini`, `setup.ini` (setup ranges).

| Folder | Car |
|---|---|
| `13_pub_nakamura` | Nissan Silvia S13 |
| `declan_byrne_nissan_silvia_s14` | Nissan Silvia S14 (VDC drift build) |
| `duane_mckeever25_nissan_180sx` | Nissan 180SX (drift build) |

Only the physics is kept: the 3D models are CSP-encrypted (`__AC_SHADERS_PATCH_KN5ENC_v1__`) and can't be
used, so the archives were dropped. `makeS15Params()` in `src/vehicle/params.js` is built from the S14 data.

Reading tips: `power.lut` is torque (Nm) without boost; AC multiplies it by `1 + boost`, where each
`[TURBO_n]` gives `MAX_BOOST * min(1, (rpm / REFERENCE_RPM) ^ GAMMA)`. `SPRING_RATE` is the wheel rate (N/m).
`DIFFERENTIAL POWER/COAST = 1.0` is a welded diff.
