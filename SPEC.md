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

### Declared refinements (2026-11-02, before V4, V7 or any M gate had run)

The research pass found places where the gates above were underspecified. These fill gaps; they do not relax a limit.

- **V4 target.** Measured, not design-point: 18 W electrical at 4.7 m/s (Send et al. 2012), band 12.6-23.4 W, using the paper's 2 m, 0.48 kg, 0.5 m², 2 Hz. Report the 23 W at 5 m/s design point alongside it for information. Stroke kinematics not published are an assumption and must be listed as such in the evidence.
- **V7 reference.** UIUC data, as the gate says. Report the NASA comparison and the two tunnels' own disagreement beside the result (up to 27% Cd at Re 100k). That context does not change pass or fail.
- **M4 turbulence.** Dryden low-altitude form at flight altitude with σ_u = σ_v = 2 m/s and σ_w from the standard ratio at that altitude; the 2 m/s stands even though a 6 m/s mean wind would normally imply less. Scale lengths from the low-altitude formula at flight altitude.
- **M4 gusts.** 1-cosine with gust length H drawn per seed from {5, 10, 20, 40} m, in both vertical and lateral, onset at random phase of the flap cycle, at least two gusts per run. The sweep must also report the worst H.
- **M2/M8 continuous ratings.** A published continuous rating wins. Otherwise continuous = the lower of the top of the published efficient band and 50% of the published burst current, and the evidence says the rule was applied. A motor with neither is ineligible.
- **M8 servos.** Only servos with published stall or rated current are eligible. Current estimate: stall current x (torque / stall torque), plus the published idle current if any.
- **M7 materials.** Only published strength counts. Woven tube 650 MPa, 64 GPa; rods at the lower of their listed ranges, 400 MPa, 28 GPa. Roll-wrapped tubes carry no primary load, because no strength is published.
- **Parts.** `specs.mass_basis` states whether `mass_g` is per part, metre, m² or litre. Parts flagged sold out or end-of-life are ineligible for the winning BOM.

### Rulings after round 1 (2026-11-02)

Round 1 results stay on record as they are. These rulings fix how a gate is evaluated where the original wording was wrong or not followed; no limit changes.

- **V2 reference.** Prandtl's lifting-line theory is itself inaccurate at low aspect ratio, so it cannot serve as the reference at AR 4. V2 is judged against Helmbold's low-aspect-ratio formula, `a = 2*pi*AR / (2 + sqrt(AR^2 + 4))`, at AR 4 and 6, and against Prandtl at AR 8-12, all within 3%. Weissinger is not used as the reference, because it was computed by our own code. The round-1 Prandtl failure remains in the record.
- **V4 method.** The gate says "published kinematics", and round 1 trimmed instead. V4 imposes the published kinematics (2 Hz, about 0.3 m plunge, about 9 deg pitch, from Send et al. 2012) at 4.7 m/s on the published geometry and mass. It reports the cycle-mean net streamwise force, which should be near zero in steady flight, and electrical power including motor, gearbox, ESC and avionics. Pass needs power in 12.6-23.4 W. Every unpublished quantity is listed as an assumption.
- **Control envelope.** Limits on stroke frequency and amplitude come from component ratings (gear, servo, motor), not from fixed multipliers.
- **Gear ratings.** A gear without a published torque rating may be rated by the Lewis bending equation with the published yield strength of its material and a factor of safety of 1.5. The evidence shows the calculation.
- **No regeneration credit.** Electrical power is the cycle mean of max(0, battery power), unless the chosen ESC's own documentation states regenerative operation, in which case regeneration counts with that ESC's stated efficiency. A flywheel may smooth the crank only if its mass is in the BOM. This is conservative and is what M2 and V4 use.
- **Leading-edge suction.** The suction efficiency at the flight Reynolds number must come from an independent published source (for example DeLaurier's ornithopter model or measured flapping-plate data) or from our own LBM and OpenFOAM results. It must never be fitted to SmartBird, because V4 would then test nothing.
- **Converged or failed.** A trim that does not converge is reported as a failure, never as a result.
- **Structural loads.** M7 uses loads from M1-M3, M5 and the M4 seeds that pass, with load factor at the CG. Peak loads must be converged in time step: halving the step changes them by less than 10%. Any gate on a time-integrated quantity reports the same check.

### Rulings after catalogue round 2 (2026-11-02)

- **Disclaimed tube strengths.** Manufacturer strength figures labelled "reference only" or "design guide" (Rock West lamination theory, CST 2011 table) count at half value, on top of the 1.5 factor of safety. Their modulus counts in full, because stiffness from lamination theory is reliable. Titanium tube uses its published minimum yield.
- **Servo voltage.** No extrapolation outside a servo's published voltage rows. The design states its servo supply voltage and includes the BEC or regulator that provides it, with its mass, in the BOM.
- **Stall current without a stated voltage** is used as published, at any supply voltage; that is conservative where makers list several.
- **Batteries.** Li-ion packs are eligible on the same terms as LiPo, rated by their published continuous current.

### Rulings after physics round 4 (2026-11-02)

Round 4 passed M1-M8 on v12 but found loads and functions that no gate checked. These apply existing gate language to parts it already covers; no limit changes.

- **Bearings are linkage.** M7 covers linkage loads, and every bearing and bushing is part of the linkage. Each one carries its peak load from the M7 load population with a static safety factor C0/P0 >= 1.5. For ball bearings, P0 follows ISO 76 with the maker's X0/Y0. Plain bushings are judged by the maker's published static surface pressure, with FoS 1.5. A bearing or bushing without a published static rating is ineligible for a loaded joint. The design file states which bearing reacts which force.
- **Control functions need real hardware.** A function the design relies on, such as a current clamp, crank-speed governor or limiter, must run in one of two places. Either it runs in firmware whose published documentation states the feature. Or it runs in our controller on the flight controller, with every sensor it reads in the BOM, at its published rate and resolution, and with that path's latency simulated.
- **Servo voltage covers current too.** The no-extrapolation rule applies to current as well as torque and speed. A servo fed straight from the pack must have every voltage the pack can present, from full charge to cutoff, inside its published rows. Otherwise the design uses a regulator, or limits charge voltage and derates capacity in M2 accordingly.
- **Mixed-material gear meshes.** Contact stress is computed by Hertz with the pair's combined modulus and the real face width. It is judged against each member's published allowable surface stress, or the maker's published rating for the pair. Bending is checked by Lewis for each member in its own material. A steel-on-steel surface figure is never applied to a steel pinion meshing with plastic.
- **V2 stays a FAIL.** Its grid study converges: the AR 4 slope moves 0.2% from 12x48 to 16x64. So the 4.28% gap at AR 8 is not resolution, and the reference is not changed again. The final report states that V5's reference model fails V2 by that margin.

### Rulings after physics round 5 (2026-11-02)

- **The 20 ms control delay applies to every loop closed on the flight controller.** That includes the crank governor and the current clamp, not just attitude control. A function closed inside the ESC's own firmware is exempt from that delay. It must use the behaviour and loop rate that the firmware's documentation states.
- **Every loaded part is linkage.** M7 checks each shaft, pin, plate bearing seat, end fitting and bore adapter in its load path, using published material strength at FoS 1.5. M6 requires each of those parts to be in the BOM with a cited or stated mass.
- **Motor bearings carrying a pinion.** Rate them by the bearing size the motor maker publishes, for example a spare-part listing, using that bearing maker's C0. If no size is published, an outboard support bearing must carry the side load.

### Rulings after physics round 6 (2026-11-02)

- **Metal strength values.**
  - Specified minimum properties from a published material standard (ASTM, EN or AMS) count in full. They must match the product form, temper and section size. Cite the standard, or a maker's or stockist's sheet that quotes its minimums.
  - Values labelled "typical" or "not for design" count at half, as ruled in round 2.
  - Pin-in-hole bearing strength must come from a published source, at the edge distance that source states, under the same full-or-half rule. Cite it in the part's catalogue entry.
- **The half-step rule stands.** A peak load that moves by 10% or more when the step is halved fails M7. A diagnosis may explain the failure but cannot waive it.
  - **Step-dependent input.** Any random draw, filter or controller tick whose realisation changes with the step is a simulation bug. Fix it so every input is the same function of time at both steps, then rerun the check as written.
  - **Chaotic flight, only after the inputs are fixed.** The inputs must be shown identical at both steps first. Then, at a fixed step, perturb each M4 seed's initial state by 1e-9 m. If the median per-seed peak change is at least half the median half-step change, the per-seed comparison is measuring chaos, not step error.
  - **What applies then.** For the M4 seed population only, the mean and the 90th percentile of the per-seed peaks must each change by less than 10%. M7 then uses the larger of the two steps' population maxima.
  - **Disclosure.** This ruling was written after round 6 failed the check, and any gate table that uses it must say so.

### Rulings after physics round 7 (2026-11-02)

- **Values with no basis count at half.** This covers anything a maker labels "reference" or "estimate", or publishes without saying whether it is a minimum. It is the same rule as for typical values.
- **Crushing on a keyway or seat.** Use the yield that the material standard specifies for the stated heat treatment, for example JIS G4051 for S45C or JIS G4053 for SCM415. If the standard gives only a tensile minimum, use half of that.
- **Every part reaches the airframe through a rated path.** The gearbox, battery, tail boom and pod each need fasteners or joints rated as follows:
  - fasteners against ISO 898-1 or ISO 3506 minimum loads, or the fastener maker's rated loads;
  - bonded joints against the adhesive maker's published strength on that substrate.
- **No modifications against maker advice.** A part modified against its maker's stated advice (drilling, re-boring, cutting) is ineligible. The maker's own machined or keyed variants are fine.

### Rulings after physics round 8 (2026-11-02)

- **A rating with a stated calculation basis counts in full, under its stated conditions.** KHK's allowable gear torques are an example: they are computed to JGMA 401/402 with stated safety factors.
  - Check every stated condition against the design: mating material and hardness, lubrication, load direction, speed and cycles.
  - Where a condition does not hold, apply that standard's own correction. For example, JGMA 401 reduces the allowable bending stress to 2/3 for reversing load. If the standard gives no correction, count the value at half.
  - Figures labelled "reference" with no stated basis still count at half. KHK's DSF slipping torque is one.
  - This clarifies the round 7 rule. Its wording, read literally, would also have halved ratings that do state a basis. Gear ratings have been used at full value since round 2, and they are now subject to the condition check.
- **Bearing below the source's edge distance.**
  - A seat with less edge distance than its bearing source states gets the lesser of two values:
    - that source's value under the half rule;
    - the bolt-bearing resistance of EN 1999-1-1 (aluminium) or EN 1993-1-8 (steel): k1·αb·fu·d·t/γM2 with γM2 = 1.25, using specified-minimum fu at the actual end and edge distances.
  - Either edge distance below 1.2·d0 makes the seat unrated.
  - The old fallback to tensile yield could give a higher allowable at a smaller edge distance. That was a bug.
- **Springs.** A catalogue spring may carry load up to its maker's published maximum load, deflection or torque, read like a rated value. Its seats and mounts are linkage at FoS 1.5.
- **RC practice for joints no one can rate (operator's decision, 2026-11-02).**
  - **Scope.**
    - It covers only a shaft-to-hub fixing: a gear, pulley or coupling held to its shaft by set screw, clamp hub, D-flat or bond. The fixing must be standard RC model practice.
    - It applies only where no maker or standard publishes a rating for that fixing; set screws are the example, since ISO 898-5 gives them no shear strength.
    - It never covers structural members, bearings, gear teeth, seats, lugs or fasteners in tension or shear. Those keep every rule above.
  - **Published figures still bind.** If the maker publishes any figure for the fixing, even a reference one such as KHK DSF slipping torque or a set-screw holding-torque table, the fixing must stay within that figure at full value.
  - **Disclosure.** M7 reports each such joint as "unrated, RC practice", with its peak torque. The README lists every one.
- **Launch speed.** M1's "8 m/s or less" caps the release speed. The design names its release speed, which must be 8 m/s or less, and M1 is gated at that speed. Report the lowest passing speed for information.

### Public repository

`github.com/sterlingdigitalp/avis` is public. UIUC-derived tables and airfoil coordinate files stay local and gitignored (`data/references/airfoil-e387.json`, `data/references/airfoils/`), because their licence terms are unclear or reserved. Our own CFD polars are ours to publish. Copernicus terrain, if used, requires the notice in `data/references/credits.md` in the film credits and README.

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
- **Not the operator's browser.** Never use the Claude-in-Chrome tools (`mcp__claude-in-chrome__*`): that is the operator's own Chrome with their sign-ins. Read pages with WebFetch, or with a headless browser under `tools/run-guarded.py`. A page that blocks both stays unread.
- **Reporting.** End with what passed, what failed, what is unverified, with paths and numbers. Do not claim anything you did not measure.
- **UI.** Any page follows `~/.claude/skills/design-principles/SKILL.md`: dark-first, one accent, tabular numbers, `prefers-reduced-motion`. This overrides Teardown's light-studio rule.
