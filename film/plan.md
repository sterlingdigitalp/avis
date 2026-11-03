# Avis film plan

A 78 s film of the Larus GL-2, 1920x1080 at 30 fps, built entirely from the design file, the catalogue entries
it names, the gate files and the flight logs. Nothing is keyframed by hand. Every shot can be rendered again with one
command after the design freezes.

Claim on screen in every shot, bottom left: **simulated flight · real parts**. No physical bird exists.

## One command

```
node film/render.mjs                  # final cut: film/edit.json -> out/film/avis.mp4
node film/render.mjs --edit preview   # the short preview: film/edit.preview.json -> out/film/preview.mp4
node film/render.mjs --dry            # plan and GPU estimate only
node film/render.mjs --proxy          # review proxy: 640x360 plates at 8 samples, every third frame held for three
node film/render.mjs --stills DIR     # one Blender still per flight shot at its log start and mid-shot, film settings
                                      # (add --threads 4 to cap Blender's CPU threads while other work runs)
```

It flies the corridor log again when the design, the flight model or the corridor changed (`film/flight_log.mjs`,
below), regenerates the bundled data from `data/designs/<design>.json` (`film/gen_data.mjs`), rebuilds the pages,
re-exports the GLB when the design or the bird code changed, bakes the pose of every flight shot from its log,
renders the Blender plates in holds under 10 minutes through `tools/run-guarded.py --gpu`, captures the film page in
20 s chunks (also guarded, also GPU), concatenates, synthesises the score on the same timeline and muxes.
Resumable: Blender frames and capture chunks on disk are kept unless their inputs changed (`out/film/state.json`).
If `env/trees.py` changed since the last render, `env/prebuild_trees.py` runs first on the CPU, as the environment
workstream asked. Blender occasionally aborts while Metal loads its kernels (crash reports show it inside
`MTLLibraryBuilder`, not in our scripts); a failed hold is re-run from the first missing frame, up to six times.

## The edit (`film/edit.json`)

| # | id | kind | source | s | what it shows |
|---|---|---|---|---|---|
| 1 | title | studio | `title` | 4.0 | The bird flapping at its real cruise frequency, slow push in. "Avis", the design name, the claim. |
| 2 | wide | Blender | `wide_valley`, corridor log from 20 s | 5.0 | The wall of Rundle, the bird a speck on its corridor: 3-4 px at 1.24 km, so a ring marks it with its span and distance (`film/bird_px.py`, the shot's own rig and camera). |
| 3 | track | Blender | `track_side`, corridor log from 50 s | 4.0 | Cruise beside the bird, sun over the shoulder. HUD: airspeed, power, battery from the log. |
| 4 | assembly | studio | `assembly` | 8.0 | The bird goes together step by step in an order a person could follow with these parts (below), close on the gearbox, then out to the whole bird for the packed parts, the pod and the wings; the mass counts up from the BOM to the total, margin last. |
| 5 | drive | studio | `drive` | 8.0 | Cutaway of the gearbox from behind and below at ¼ speed: crank, couplers, rockers, gears at the true tooth ratios; with a stroke servo it then walks the amplitude through the drive's range (a fixed-stroke drive, v14 on, holds it and points at the crank angle encoder its governor reads). Panel: ratio, motor rpm, crank Hz, live stroke angle, transmission angle. |
| 6 | twist | studio | `twist` | 6.5 | Along the right wing at ¼ speed: ribs turning nose down on the downstroke, nose up on the upstroke, the elbow servo and torque rod. |
| 7 | power | studio | `power` | 5.5 | Battery, ESC and wiring. Panel: cruise power, endurance and the M2 verdict from `out/verification/M2-*.json`. |
| 8 | lead | Blender | `lead_front`, from 66 s | 4.0 | Head on. |
| 9 | under | Blender | `under_wing_full`, from 100 s (v23's log; 98 s left the bird at 398 px), 2x slow motion | 4.0 | Below and behind the sun-side wing on a 20 mm lens, the whole bird high in the frame and Rundle's skyline low in it; labelled "2× slow motion". |
| 10 | treetop | Blender | `treetop_pass`, from 112 s | 4.0 | Hovering over the forest beside the path; the bird flies past below the camera with the canopy behind it. |
| 11 | topdown | Blender | `top_down`, from 36 s | 3.5 | From above over the bench: planform and the wing's stroke. |
| 12 | ridge | Blender | `ridge_rise`, from 104 s, 200 mm | 5.0 | The bank near the end of the corridor, from a stand on the slope solved from the terrain for a 135 mm lens, filmed on 200 mm so the bird spans 250-340 px: the bird against forest. |
| 13 | river | Blender | `river_pass`, from 24 s, 80 mm | 4.0 | Hovering beside the bird where it crosses the Bow, the river behind it; 80 mm on the rig's stand (200-400 px). |
| 14 | gates | studio | `gates` | 6.0 | Every gate as written in `out/verification`, failures in their own words and numbers. A mission gate whose run at the published η_s 0.6 (`out/verification/eta0.6/`) differs is NOT FINAL, with both runs' numbers, never a pass (`numbers.js` gateVerdict: `tools/verify.mjs`'s rule, pinned gate by gate against it). |
| 15 | credits | studio | `credits` | 7.0 | Data and software credits from `data/references/credits.md`, the claim, the parts statement. |

Changes of segment cut, or dip to black for at most 0.45 s (0.2 s out, 0.25 s in; `film/test/edit.test.mjs` holds every
dip under 0.5 s). A contact sheet sampled on whole seconds lands on those dips at 4 s and 13 s, so those tiles are black.

Flight shots come only from flight logs: the camera is env/shots.py's function of the log; the bird's pose is
baked from the same log by `film/bake_pose.mjs` (phase, amplitude, twist and tail columns through
`src/physics/kinematics.js`). The corridor log is `src/physics/gates/corridor.mjs`'s scenario (12 m/s, mean wind
3.2 m/s, Dryden u_rms 1 m/s, seed 7) flown by `film/flight_log.mjs` through `runCorridor`, without that script's climb
runs or its evidence file; the log records the sha256 of the design, the flight model's sources and the corridor it was
flown from, and render.mjs flies it again (6 s) when any of them changed. Start times are seconds into the log (v21's
is 122.6 s); render.mjs refuses a window that runs past the end of its log. `env/test_shots.py` pins what each shot promises on today's edit and log (the bird inside the frame, its
size in pixels, land rather than sky, forest or water behind it), so a new corridor log that breaks a shot fails a test
named after it: move that shot's `start_s` and re-run the tests. The HUD on flight shots reads the log at the same frame.

## Studio sequences (three.js, Teardown structure)

`sims/avis-film/` is the recorded page; `sims/avis-live/` is the interactive page; `sims/avis-shared/` holds the
bird (`bird.js`), its numbers (`model.js`, pure), the stage, the film sequences and the score. The film page is a pure
function of time (`window.FILM.frame(t)`): Blender plates are composited in the page as full-frame images behind
the same HUD type, so studio and flight shots share one look and one capture path (`film/capture.mjs`).

**Drawn from the physics' solids (v22 on).** `src/design/clearance.js` lays every part of the drive and frame out
as prisms along body x (rings, capsules, polygons with holes; the moving ones as functions of the drive pose), and its
clearance check reads exactly those. The film draws the same solids (`sims/avis-shared/solids.js` physicsAssembly, a
film-only module that imports the physics and changes nothing in it): `drive-solids.js` draws the fixed parts as the
solids' own prisms (plates with their posts, pads and bosses, every plate journal, gear shafts, the encoder, shoulder
bushings and brackets, rails and lock flanges, panels, boom, clamps, trays), and `bird.js` details the rest at the
solids' planes (gear teeth to the catalogue's tip circle, the crank, couplers, rockers with the hub relief and collar,
the horn stacks, the motor inside the box). A part the physics adds that no builder names is drawn from its solids
(`drawRest`), so v23's new parts appear with no film work. `film/test/physics-solids.test.mjs` holds the drawing to the
check: every vertex of every drawn part inside the solids it names at every crank angle sampled, every solid drawn to
its own x range and outer section, within 0.01 mm (fifty times finer than the 0.5 mm rule). BOM lines the physics has
no solid for (rail aft seat bosses, the two doublers, M3 screws and nuts, cable ties) and the formers are placed by
the film and listed there; the formers keep out of the drive's x range but cross the trays, which have no cutout.

**The order of assembly** (`solids.js` assemblyPlan): aft plate, crank from aft, the motor (inside the box), then the
gear train inside the box, both from forward, each side's coupler onto the crank pin, shoulder brackets, rocker,
shoulder pin and horn pin, the rails through the brackets and the aft plate, the forward plate, stage 1 (overhung) and
the encoder, panels, boom, clamps, the trays up onto the plates' pads; then the packed parts, the pod and its covering,
the wings. A part the physics adds goes on with the earliest step of a part it is joined to (its fixing, bracket or
carrier: `joinedParts`, from the assembly's own joints), so v23's new parts take their place with no film work. Each part is
drawn once its step begins and slides in along its own line. `film/test/assembly-order.test.mjs` slides every step in
against everything already in place on the physics' solids; the one conflict no order avoids is reported: v22's
one-piece crank cannot pass the aft plate's crank seat or its bushing (the 14 mm web aft, the 15 mm wheel seat inside).

## The bird's covering

The BOM covers the wings, tail and pod with Oracover Air Light (`film-oracover-air-light`, 12 um polyester). Without
a colour in the catalogue entry the film draws it clear: untinted, a thin slab reflecting 2r/(1+r) of one
polyester face (IOR 1.64, about 11% head on, a mirror at grazing), 3% haze, a little roughness. Through it the balsa
ribs and formers show, as they would. Between ribs the film takes the shape a taut membrane must (equal tension,
no air load: Laplace's equation over each bay, `model.membraneDip`), a dip of at most about 2% of the chord near the
nose.

**Livery.** The covering's colours are set in one place, `LIVERIES` in `sims/avis-shared/livery.js`, per surface
(upper wing, lower wing, tip, tail, pod). The catalogue now records the BOM's film as ORACOVER Air Light, light scale
white 331-099, on every covered surface, so the default is the `white` livery (`defaultLivery`: only when
`liveryInBom` finds every colour in the catalogue entry and every covering line of the BOM on that film; a BOM without a
colour falls back to the clear film above, `--livery clear` asks for it). `node film/render.mjs --livery gull` (or
`"livery": "gull"` in an edit) draws the gull preview, its chrome not in the BOM, so every frame carries "preview,
livery not yet in BOM". bird.js writes each covering mesh's surface colour as vertex colours (the geometry and the bake
are unchanged), the GLB is its own file (`<design>-<livery>.glb`) whose meta names the livery, and
`film/blender_bird.py` pigments the film from those colours (Blender reads the white as 0.871, 0.871, 0.839 linear, the
livery's own value to 8-bit precision), the chrome faces as metal. In the sun the white peaks at 245/255 under AgX,
no clipping. `AVIS_FILM_RGB=r,g,b` still paints a clear GLB one colour in Blender.

The maker's range (2026-11-03): ORACOVER Air Light is sold only in colours, all about 18 g/m2 and 12 um: light scale
white 331-099 and the light chromes 331-090 (silver), -093 red, -094 yellow, -096 violet, -097 blue. The clear film at
about 17 g/m2 is a different product, Air Indoor "light transparent" 331-000; the catalogue entry lumps the two. Air
Light has no grey and no black, so the gull livery is white 331-099 under the wings and on the pod and tail, the silver
chrome 331-090 standing in for gull grey on top, and no black tips. `film/edit.livery.json` renders the preview stills.

## Sound

`sims/avis-shared/score.py`, run by Teardown's engine (`vendor/teardown/run_sfx.py`) on the edit's own timeline,
mastered to -16 LUFS. A quiet D-major bed throughout; a low motor under the drive sequence; air under every flight
shot; a soft thud as each studio sequence opens; one chime when the gates card comes up. Every time comes from
the timeline written by render.mjs.

## Frame rate and budget

Measured on this machine (M5 Max, Metal) with the bird in the Bow Valley scene, 1920x1080, `out/film/timing.json`:

| Blender samples | seconds per frame | note |
|---|---|---|
| 24 (preview) | 9.4-10.5 steady, 14.2 mean with first frames | 165 frames rendered for the preview, before environment pass 3 |
| 48 (final) | 22.3 steady (track), 25.5 steady (ridge), 22-31 single frames across all eight shots | polish pass, clear-film bird, environment pass 3 forest; `out/film/review/polish/timing_run.log` |
| 96-128 (environment's stills) | 25-33 | environment's own logs |

The clear covering is a transparent surface, so `film/blender_shot.py` allows 16 transparent bounces; it costs a few
seconds a frame where the bird is large. Scene setup per GPU hold is now about 10-15 s (the environment's tree cache);
render.mjs still sizes holds with 75 s of setup so no hold can run past 10 minutes on a slow start.

The final cut has 33.5 s of flight shots. At **30 fps** that is 1005 Blender frames: about 6.7 h of rendering at 48
samples plus 57 holds of setup, **about 6.9 GPU hours** at the measured setup and 7.9 h if every hold paid 75 s
(`node film/render.mjs --dry` prints the conservative figure). That is under the 8-hour limit but leaves little room
for a re-render; 40 samples would save about a sixth. At 60 fps it would be twice that; 30 fps it is.

## After the design freezes

1. `node film/render.mjs --dry`, check the estimate.
2. Look at one still per Blender shot at the chosen log times (`node film/render.mjs --edit <one-shot edit>`), and
   move start times in `film/edit.json` if a shot frames badly with the final log.
3. `node film/render.mjs` (background; it resumes if stopped).
4. Review: `node vendor/teardown/sheet.mjs --video out/film/avis.mp4 --every 1`, read every tile.
