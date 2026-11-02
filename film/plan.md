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
```

It regenerates the bundled data from `data/designs/<design>.json` (`film/gen_data.mjs`), rebuilds the pages,
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
| 2 | wide | Blender | `wide_valley`, corridor log from 20 s | 5.0 | The wall of Rundle, the bird a speck on its corridor. |
| 3 | track | Blender | `track_side`, corridor log from 50 s | 4.0 | Cruise beside the bird, sun over the shoulder. HUD: airspeed, power, battery from the log. |
| 4 | assembly | studio | `assembly` | 8.0 | Every subsystem flies in from its exploded position; the mass counts up from the BOM to the total, margin last. |
| 5 | drive | studio | `drive` | 8.0 | Cutaway of the gearbox from behind and below at ¼ speed: crank, couplers, rockers, gears at the true tooth ratios; then the stroke servo walks the amplitude through the drive's range. Panel: ratio, motor rpm, crank Hz, live stroke angle, transmission angle. |
| 6 | twist | studio | `twist` | 6.5 | Along the right wing at ¼ speed: ribs turning nose down on the downstroke, nose up on the upstroke, the elbow servo and torque rod. |
| 7 | power | studio | `power` | 5.5 | Battery, ESC and wiring. Panel: cruise power, endurance and the M2 verdict from `out/verification/M2-*.json`. |
| 8 | lead | Blender | `lead_front`, from 66 s | 4.0 | Head on. |
| 9 | under | Blender | `under_wing`, from 98 s, 2x slow motion | 4.0 | Under the wing, close; labelled "2× slow motion". |
| 10 | treetop | Blender | `treetop_pass`, from 36 s | 4.0 | Low over the bench. |
| 11 | topdown | Blender | `top_down`, from 28 s | 3.5 | From above: planform and the wing's stroke. |
| 12 | ridge | Blender | `ridge_rise`, from 104 s | 5.0 | The bank near the end of the corridor. |
| 13 | river | Blender | `river_low`, from 6 s | 4.0 | From the Bow River bank. |
| 14 | gates | studio | `gates` | 6.0 | Every gate as written in `out/verification`, failures in their own words and numbers. |
| 15 | credits | studio | `credits` | 7.0 | Data and software credits from `data/references/credits.md`, the claim, the parts statement. |

Flight shots come only from flight logs: the camera is env/shots.py's function of the log; the bird's pose is
baked from the same log by `film/bake_pose.mjs` (phase, amplitude, twist and tail columns through
`src/physics/kinematics.js`). Start times are seconds into the log (the v9 corridor log is 116.9 s); render.mjs refuses a window that runs past the
end of its log. Re-check them against the final corridor log. The HUD on flight shots reads the log at the same frame.

## Studio sequences (three.js, Teardown structure)

`sims/avis-film/` is the recorded page; `sims/avis-live/` is the interactive page; `sims/avis-shared/` holds the
bird (`bird.js`), its numbers (`model.js`, pure), the stage, the film sequences and the score. The film page is a pure
function of time (`window.FILM.frame(t)`): Blender plates are composited in the page as full-frame images behind
the same HUD type, so studio and flight shots share one look and one capture path (`film/capture.mjs`).

## Sound

`sims/avis-shared/score.py`, run by Teardown's engine (`vendor/teardown/run_sfx.py`) on the edit's own timeline,
mastered to -16 LUFS. A quiet D-major bed throughout; a low motor under the drive sequence; air under every flight
shot; a soft thud as each studio sequence opens; one chime when the gates card comes up. Every time comes from
the timeline written by render.mjs.

## Frame rate and budget

Measured on this machine (M5 Max, Metal) with the bird in the Bow Valley scene, 1920x1080, `out/film/timing.json`:

| Blender samples | seconds per frame | note |
|---|---|---|
| 24 (preview) | 9.4-10.5 steady, 14.2 mean with first frames | 165 frames rendered for the preview |
| 48 (final) | 14.6-17.6 steady, 17.5 mean | 6-frame measurement of the track shot |
| 96-128 (environment's stills) | 25-33 | environment's own logs |

Each GPU hold also spends about 60-75 s on scene setup (terrain, forest scatter, BVH). Studio capture of the
film page costs about 0.3 s per frame and is negligible.

The final cut has 33.5 s of flight shots. At **30 fps** that is 1005 Blender frames: about 4.9 h of rendering
at 48 samples plus about 0.8 h of setup in ~40 holds, **about 5.7 GPU hours**, under the 8-hour limit with room for one
re-render of a shot or the occasional Metal crash retry. At 60 fps the same shots would be 2010 frames, about 11 GPU
hours, over the limit; 30 fps it is. `node film/render.mjs --dry` prints the estimate from the latest measurements.

## After the design freezes

1. `node film/render.mjs --dry`, check the estimate.
2. Look at one still per Blender shot at the chosen log times (`node film/render.mjs --edit <one-shot edit>`), and
   move start times in `film/edit.json` if a shot frames badly with the final log.
3. `node film/render.mjs` (background; it resumes if stopped).
4. Review: `node vendor/teardown/sheet.mjs --video out/film/avis.mp4 --every 1`, read every tile.
