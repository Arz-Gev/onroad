# Onroad: JDM touge drifting

Goal: a JDM touge drifting game (mountain pass, night runs, RWD cars, handbrake entries, drift scoring), built on the Offroad Defender engine.

**Starting point:** a full copy of `Arz-Gev/offroad` at commit `6a07292` (2026-10-06), not a fork, with no shared history. The game runs (`npm run dev`, port 5175; `npm run simtest` passes). It now starts with the Silvia S15 on the Ebisu North track; `?car=defender` and `?map=offroad` bring back the Defender and the offroad map. Each step below replaces a piece while the game keeps running. `DEVNOTES.md` is the offroad notes, kept because the conventions, traps and testing workflow still apply. The old player task list is in `docs/offroad-TASKS.md`. `reference.webp` (the Defender photo) was left out; comments in `truckBody.js`, `truckDims.js` and `truckModel.js` still mention it.

## What carries over

| Area | Files | Verdict |
|---|---|---|
| Render pipeline, quality presets, cascaded shadows, shader patches | `src/render/*` | **Keep as is** |
| Sky, atmosphere, time of day (night touge is the main mood) | `world/sky.js`, `atmosphere.js`, `environment.js` | **Keep** |
| Trees, foliage, grass, undergrowth, noise, textures, materials | `world/trees.js`, `foliage.js`, `grass.js`, `undergrowth.js`, `noise.js`, `textures.js`, `materials.js` | **Keep** to dress the mountain road (Japanese cedar and pine forest) |
| Collider streaming | `world/colliderStream.js` | **Keep** |
| Tyre model (relaxation length, combined slip, load sensitivity) | `vehicle/tire.js` | **Keep, retune** for road tyres on asphalt: higher μ, a slip-angle peak that falls off past the limit (slides need to be controllable, not grip-and-snap) |
| Drivetrain PGS solver: engine, clutch, manual gearbox, rev-match, handbrake | `vehicle/drivetrain.js` | **Keep, simplify**: 2WD RWD layout, add an **LSD** (1.5/2-way clutch-type, and welded), remove transfer case and lockers. Handbrake must lock the rear wheels (hand-brake turns) |
| Vehicle core (ray-fan contact, force averaging, 240 Hz plus substeps) | `vehicle/Vehicle.js` | **Keep the core, rewrite the suspension**: the solid beam axles become **independent** suspension per wheel (strut front, multi-link rear), using the same heave DOF per corner instead of per axle |
| Params / tuning / tuning panel | `vehicle/params.js`, `tuning.js`, `tuningPanel.js` | **Keep the system**: new car presets (AE86, S13/S15, FD RX-7, R32, Chaser JZX100). Swap offroad knobs (lift, tyre pressure, lockers) for drift ones: camber, toe, steering lock (angle kits), LSD lock, tyre compound, brake bias |
| Input, settings, menu, HUD, camera rig | `input.js`, `settings.js`, `menu.js`, `hud.js`, `ui.css`, `cameraRig.js` | **Keep**: remove offroad bindings (transfer, lockers, pressure, light bar). Add a drift HUD: angle, speed, combo score |
| Audio: AudioWorklet engine synth | `audio/engine-worklet.js`, `audio/audio.js` | **Keep the engine**: new firing patterns (I4 4A-GE, I6 RB26/2JZ, 13B rotary), turbo spool and blow-off, **tyre squeal** keyed to slip (replaces gravel and mud) |
| Effects | `effects.js` | **Keep the system**: dust becomes **tyre smoke**, tracks become **skid marks** on asphalt |
| Lights | `vehicle/truckLights.js` | **Adapt** to a new car (pop-up headlights are a must) |
| Headless test tools | `tools/simtest.mjs`, `dyno.mjs`, `handling.mjs`, `yawtest.mjs`, `steerassist.mjs`, `cdp.mjs`, `profile.mjs` | **Keep**: retarget the baselines. Add a drift test (hold a steady slide at a fixed angle) |
| Offroad-only tools | `climb.mjs`, `ramptest.mjs`, `bumptest.mjs`, `traillap.mjs`, `trailprofile.mjs`, `junction.mjs`, `terraintest.mjs` | Delete once the new map exists |

## What gets replaced

- **Car model**: `vehicle/truck*.js`, `geom.js`, `vehicleView.js`. Procedural Defender body and interior. Replace with a JDM coupe: either a procedural build like the truck, or a GLB from Sketchfab (the API key is in `~/keys/`). `geom.js` and `truckMaterials.js` (car paint, glass, rim shaders) are worth reusing.
- **Map**: `world/terrain.js`, `terrainView.js`, `props.js`, `water.js`. The 1 km offroad heightfield with trails and a proving ground. Replace with a **touge course**: a spline road with hairpins, esses and elevation change, carved into a mountain heightfield. Add a real road mesh (asphalt, centre line, guardrails, drainage gutters, the gutter "hook" if wanted), plus vending machines, a tunnel and a parking area at the summit. Keep the terrain heightfield for the hillsides.
- **Collision against the scenery**: guardrails and rock walls need colliders. The wheel side-cylinder trap in DEVNOTES applies here too.

## Suggested order

1. **Physics first, on the existing flat dyno pad**: independent suspension, RWD with an LSD, tyre retune for asphalt, a car params preset (start with an S13-ish 1200 kg, 50:50, 200 hp). Proof: `yawtest` holds a steady drift with counter-steer and throttle.
2. **Drift input**: keyboard and gamepad steering assist for counter-steer (the existing `steerAssist` is grip-limit based; drifting needs a countersteer assist mode). A clutch-kick binding.
3. **Touge road**: spline road generator and course. Strip the offroad map features.
4. **Car model and lights** (pop-ups).
5. **Audio**: engines, turbo, tyre squeal. **Effects**: smoke and skid marks.
6. **Gameplay**: drift scoring and combos, time attack (uphill and downhill), ghost replay, later a tandem AI lead car.
