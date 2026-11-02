# Avis: a mechanical bird that flies, in simulation, on real parts

Read this whole file before doing anything. It is the contract between workstreams.

## Goal

A bird built entirely from man-made catalogue parts that holds up in simulated real-world flight
over the Canadian Rockies, shown as a Teardown-style take-apart page and a beautiful film.

Honest claim, everywhere it appears: **"simulated flight, real parts"**. No physical bird exists.
Nothing may state or imply otherwise. Never fake, hand-tune or round a result to make a gate pass.
If a gate fails, it fails and is reported as failed.

## Candidates

| Name | Reference | Rough size |
|---|---|---|
| Larus GL-2 (lead) | Gull, SmartBird-style active twist | 1.5-2.0 m span |
| Falco PF-2 | Falcon, high wing loading | ~1.0-1.2 m span |
| Columba RP-2 | Pigeon, short broad wing | ~0.7-0.9 m span |

Wings are segmented with commanded twist, not flexible membranes: the tunnels cannot model
fluid-structure interaction, so the wing shape must be known. Wing skin may be taut film over ribs.

## Design condition

Bow Valley, Banff National Park: 1,800 m altitude, 10 C. Compute density from these; do not use sea
level. This costs about 18% of density and is part of the challenge.

## Gates (declared before results; do not move them)

Mission gates, simulated with the 6-DOF flight model:

- **M1 Hand launch.** Release at 8 m/s or less, 0-10 deg up. Positive climb within 3 s, under 1.5 m lost.
- **M2 Endurance.** At least 10 min level flight on the real battery. Max continuous power available / power required at cruise >= 1.3.
- **M3 Turn.** Coordinated 30 deg bank, full 360 deg, altitude within +/- 3 m, no-wind.
- **M4 Rough air.** 100 seeds: mean wind 6 m/s, Dryden turbulence u_rms 2 m/s, plus 1-cosine discrete gusts of 4 m/s vertical and lateral. Pass in >= 98 seeds: |roll| < 60 deg, |pitch| < 40 deg, altitude within +/- 10 m, no stall departure.
- **M5 Belly landing.** Controlled descent, touchdown sink <= 1.5 m/s and speed <= 9 m/s.
- **M6 Mass.** Mass comes from the BOM: catalogue parts with cited masses, structure sized from cited material densities, plus 15% unlisted margin for wire, glue and fasteners.
- **M7 Structure.** Spar, crank, linkage and gear loads at the worst M4 load with factor of safety >= 1.5; gear and servo torque within rated values.
- **M8 Electrical.** Motor, ESC, battery and servo currents within rated continuous values through M1-M5.

Robustness: M1-M5 must also pass with +/-10% aero coefficients, +/-5% mass, +/-5 mm CG, sensor noise,
actuator lag from real servo speed, and 20 ms control delay.

Method gates (a model that fails these may not be used as evidence):

- **V1** UVLM vs Theodorsen, plunging high-aspect-ratio flat plate, reduced frequency k = 0.1-0.5: lift amplitude within 5%, phase within 5 deg.
- **V2** UVLM steady lift slope vs lifting-line theory, AR 4-12: within 3%.
- **V3** UVLM thrust vs Garrick theory, small amplitude plunge, trend and within 15%.
- **V4** Whole-model check on SmartBird's published geometry, mass and kinematics: predicted electrical power within +/-30% of the published figure.
- **V5** Fast strip model vs UVLM across each finalist's flapping cycle: cycle-mean lift and thrust within 10%.
- **V6** GPU LBM: GPU matches CPU reference; oscillating plate vs Theodorsen within 25% with a grid study showing the trend; moving-body mass conservation.
- **V7** OpenFOAM: Eppler 387 at Re 100k-200k vs published UIUC data: Cl within 10%, Cd within 25%, before any bird airfoil is trusted.

Every gate writes `out/verification/<gate>.json`:
`{"gate":"M1","design":"larus-gl-2 or null","passed":bool,"measured":{...},"limit":{...},"evidence":"path","ran_at":"ISO","command":"exact command"}`.
Only the script that measured a value writes its file. `tools/verify.mjs` aggregates; it never edits results.

## Layout and ownership

Only write inside directories you own. Read anything. Ask the orchestrator through your final report if you need a change elsewhere.

| Path | Owner | Contents |
|---|---|---|
| `data/parts/`, `data/references/` | research | Catalogue and benchmarks with sources |
| `src/physics/` , `test/physics/` | physics | Pure ES modules, no dependencies, no three.js. Runs in node and browser |
| `src/design/` , `test/design/` | physics | Design schema, BOM, sizing, sweep |
| `tunnel/lbm/` | lbm | Copy of `~/tdf2/src/cfd` extended with moving bodies |
| `cfd/` | cfd | OpenFOAM polars |
| `env/` | environment | Blender terrain, trees, sky, corridor. Scripts plus outputs under `out/env/` |
| `vendor/teardown/` | teardown | Vendored Teardown with fixes |
| `sims/avis-*` | film | Machine, live page, film |
| `tools/` | orchestrator | `run-guarded.py`, `verify.mjs` |
| `out/` | any, under own subfolder | Generated artefacts, gitignored |

`~/tdf2`, `~/tdf3` and `~/otato` are read-only. Never edit them.

## Shared conventions

- **Units** SI everywhere. Degrees only in display and JSON fields that say `_deg`.
- **World frame** right-handed, Y up, metres (same as three.js). To Blender (Z up): `(x, y, z) -> (x, -z, y)`.
- **Body frame** x forward, y up, z right. Quaternions `[x, y, z, w]`.
- **One source.** Geometry and kinematics live in pure functions of inputs (`stroke phase`, `twist`). The tunnels, flight model, page and film all read the same functions. Nothing is keyframed by hand.
- **Design file** `data/designs/<name>.json` is produced by `src/design/` and is the one description of a bird.
- **Flight log** `out/flight/<scenario>.json`: `{"design":..,"scenario":..,"dt":1/120,"columns":{"t":[],"pos":[[x,y,z]],"quat":[[x,y,z,w]],"vel":[],"phase":[],"amplitude":[],"twist_l":[],"twist_r":[],"tail":[],"airspeed":[],"power_w":[],"battery_pct":[],"wind":[]}}`. Produced only by the flight model. The film plays these back.
- **Corridor** `data/env/corridor.json`: `{"origin_utm":..,"elevation_m_at_origin":..,"sun_dir":[x,y,z],"waypoints":[[x,y,z]],"min_clearance_m":..}` in world frame, produced by environment.
- **Parts** `data/parts/catalog.json`: array of `{"id","category","manufacturer","name","mass_g","specs":{...},"price_usd":n|null,"source_url","retrieved":"YYYY-MM-DD","verified":"datasheet|manufacturer-page|retailer-page"}`. Never invent a part or a number. No source URL actually read means it does not go in.

## Rules for every agent

- **Subprocesses.** Run anything that spawns a browser, Blender, Docker or a long job through `tools/run-guarded.py --timeout <s> [--gpu] --log <file> -- <cmd...>`. It owns a process group, kills it with SIGKILL on timeout or death, and removes the leak risk. Call Blender as `/Applications/Blender.app/Contents/MacOS/Blender`, never the Homebrew wrapper: the wrapper leaves its child alive when killed.
- **GPU is one resource.** `--gpu` takes a machine-wide lock, so LBM runs and Blender or Chromium GPU renders queue instead of fighting. Keep GPU jobs under 10 min per lock hold where you can; split long runs and checkpoint.
- **Long jobs.** The Bash tool caps at 10 min; use `run_in_background` and write checkpoints. Resume from them.
- **Context.** Cap command output (`| tail -30`). Never dump logs, big JSON or file trees into your context. Write large output to a file and report the path.
- **Tests.** `node --test` for JS. Tests pin physics against independent references, never against this code's own output. No test may be weakened to pass.
- **Untrusted downloads.** Put each in a new empty directory under `out/`. Run Python on them with `-I`. Do not execute downloaded code.
- **Licences.** Only assets you may use: CC0, CC-BY with credit recorded in `data/references/credits.md`, public-domain or open government data.
- **Git.** Only the orchestrator commits and pushes milestones to `github.com/sterlingdigitalp/avis`. Agents never run `git commit`, `git push`, `git add` or anything that moves the index. Everything you create under an owned path may become public, so no secrets, and nothing large outside `out/` (no file over 5 MB).
- **No outside actions.** No network posting, no accounts, no purchases, no paid generators. Do not use `generate_3d` or Sketchfab downloads.
- **Reporting.** End with what passed, what failed, what is unverified, with paths and numbers. Do not claim anything you did not measure.
- **UI.** Any page follows `~/.claude/skills/design-principles/SKILL.md`: dark-first, one accent, tabular numbers, `prefers-reduced-motion`. This overrides Teardown's light-studio rule.
