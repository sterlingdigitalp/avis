# cfd: OpenFOAM polars for Avis

2D airfoil polars with the Langtry-Menter transition model, the V7 validation gate on the Eppler 387, and a 3D
frozen-glide check of the lead design. Everything runs through `tools/run-guarded.py` in the `aerolab-openfoam:13`
image. Outputs go under `out/cfd/` (gitignored) except the polar tables in `data/calibration/polars/`.

| File | Role |
|---|---|
| `airfoil.py` | Read Selig coordinates, spline, normalise (closes a blunt TE by shear), clustered stations |
| `cgrid.py` | Structured C-grid by the square-root map, written straight to `constant/polyMesh` |
| `case.py` | One 2D case: fields, schemes, solver, function objects, freestream turbulence with decay allowance |
| `run-case.sh` | Docker wrapper: CPU, memory, time limits as arguments; container removed by trap, sidecar and `timeout` |
| `pipeline.py` | Run a point: steady, then transient fallback; NaN kill; result.json with explicit status; pool of two |
| `v7_campaign.py`, `gates/v7.py` | V7 runs (main, grid, Tu, transient check) and scoring to `out/verification/V7.json` |
| `polars.py`, `sections.py` | Polar tables and the parametric film-rib and thin-plate sections |
| `run_all.py` | The whole 2D programme in priority order, resumable |
| `pose.mjs`, `stl_bird.py`, `case3d.py`, `bird3d.py` | 3D: glide pose from `src/physics/kinematics.js`, STL, half-model case, run |
| `test/test_cfd.py` | `python3 -m unittest discover -s cfd/test` |

Limits: at most two cases at once (`out/cfd/slots` lowers it live so a 3D case can take a slot), 8 CPUs and 16 GB per
case, 3 MPI ranks inside the 8 CPU quota (see `pipeline.py` for the measured reason).

## Docker memory note (restore this)

Docker Desktop's VM memory was raised for this stage, as authorised by the operator via the orchestrator.

- File: `~/Library/Group Containers/group.com.docker/settings-store.json`, key `MemoryMiB`
- Original value: **6144**
- Value during this stage: **49152** (48 GB)
- The orchestrator restores it later. Docker must be quit before editing the file.
- Restored by this workstream at the coordinator's request on 2026-11-02: Docker Desktop quit, `MemoryMiB` set back
  to 6144 and read back as 6144. Docker was left quit, as it was found.

## V7 method, declared before any result

Written before the first solver run. These do not change in response to results.

**Local-only data.** UIUC tables, airfoil coordinates and transcribed NASA tables stay in gitignored paths, per SPEC
"Public repository". `cfd/` only references them by path: `data/references/airfoil-e387.json`,
`data/references/airfoils/`, `out/cfd/ref/`. Meshes and case data live under `out/cfd/`.

**NASA context.** V7 scores against UIUC. Beside it, the gate reports the CFD against NASA TM-4062 and UIUC against
NASA (up to 27% Cd at Re 100k per the SPEC). Neither changes pass or fail. The NASA rows were transcribed by eye from
scans, so read them as context with that caveat.

**Reference data.** UIUC LSATs, *Summary of Low-Speed Airfoil Data* Vol. 5, Appendix B, Fig. 4.113, E387 (E):
run BM05374 (Re 99,933) and BM05376 (Re 199,747). Vol. 5 tabulates Cl and Cd only. No Cm comparison is scored.

**Pass criterion.** The SPEC says "Cl within 10%, Cd within 25%". It is applied in its strictest reading:
relative error at *every* compared point, at Re 100k and 200k, for alpha -2 to 10 deg. The reference is
interpolated linearly to the CFD alpha (the UIUC alphas are not round numbers). A point that did not
converge counts as a failure of that point. Summary statistics (mean, RMS) are reported beside it and do
not rescue a failing point. Cl error near zero lift is large by construction; it is still scored.

**Turbulence intensity.** Tu = 0.10% at the airfoil, from Vol. 5 Fig. 2.3 (LSATs apparatus installed): about
0.10% (Re 100k) and 0.08% (Re 200k) with a 3 Hz high-pass filter, and 0.12% and 0.145% DC coupled. These are
read off a plot. The far-field value is raised to allow for freestream decay, so Tu is as stated at the
airfoil rather than at the boundary, and the delivered value is measured. Sensitivity runs at 0.05% and 0.15%
are reported, never selected from.

**Model.** kOmegaSSTLM (Langtry-Menter), resolved wall, y+ about 1, no wall-function modelling of the layer.

**Freestream turbulence detail.** Eddy-viscosity ratio at the far field is 5, fixed for all Re. Reaching Tu = 0.10%
at the airfoil across a 40 c decay distance is not possible with a much lower ratio. Delivered Tu is measured by
a probe one chord upstream and written with each result.

## Convergence statuses

Added after looking at two probe points (Re 100k and 200k, alpha 4), before any scored run. They define what
"converged" means and do not depend on the comparison with the tunnel data.

| Status | Meaning | Counts as converged |
|---|---|---|
| `steady` | Over the last 2000 iterations, Cl range under 0.5% and Cd range under 1% | yes |
| `steady-mean` | Small limit cycle: consecutive 2000-iteration means agree (Cl 0.5%, Cd 1%), amplitude under 4% Cl, 15% Cd. Amplitude is reported | yes |
| `transient-mean` | Consistent PIMPLE, backward time, dt 0.0025 c/U, consecutive 5 c/U means agree (Cl 1%, Cd 2%), at most 12 c/U run (time budget) | yes |
| `unconverged-steady`, `unconverged-transient`, `diverged`, `failed` | None of the above. The last values are kept and flagged | no |

Sweeps: each Re is one up-sweep in alpha, as the tunnel data were taken; each point starts from the previous point's
converged field (only `steady` or `steady-mean` fields seed), so its minimum run is 2500 iterations (limit-cycle test
from 5000) instead of 4000 (8000) from potential flow. The settling tests themselves are the same. A cold point
(first of a sweep, or after an unconverged one) uses the full minimums.

Why `steady-mean` exists: SIMPLE settles into a small oscillation around the bubble at many points, and the time-accurate
fallback costs about 30 times as much per point. Group `v7-transient` runs alpha 4 and 8 at both Re time-accurately to
check that the steady mean equals the time mean. The result is reported whatever it shows.

## Results (2026-11-02; every number below is measured, see the paths)

The shared host ran at load 100-460 for most of the session, which made the Docker VM 10-100x slower than on a quiet
machine (measured: 23-30 it/s against 0.3-5 it/s on the same case). That, not the method, is why several planned runs
were cut. Everything is resumable (`cfd/run_all.py`, finished points are never rerun).

### V7: FAIL

`out/verification/V7.json`, local report with every point and the reference values `out/cfd/v7/report.json`.
E387, level 1 grid (55k cells, y+ max 1.93), Tu delivered 0.100-0.103% at one chord upstream. 13 of 26 points pass.
CFD minus UIUC, per cent, Cl / Cd (status: s steady, m steady-mean, t transient-mean, U unconverged, counted as failed):

| alpha | -2 | -1 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Re 100k | -11/-13 U | -9/-11 U | -5/-22 t | -3/-23 t | -1/-27 t | +2/-28 m | +3/-17 m | +2/+4 m | +3/+29 m | +2/+40 s | +3/+39 m | 0/+37 U | -5/+51 U |
| Re 200k | -4/+2 U | -4/-4 m | -2/+3 m | -1/+5 m | 0/+6 m | +1/+4 m | +2/+3 m | +1/+6 m | +2/+14 m | +2/+17 m | +3/+23 U | +3/+26 U | +4/+22 U |

- Lift is right: mean |error| 3%, every point within 10% except Re 100k, -2 deg (11%, unconverged).
- Drag at Re 200k is within 25% to 8 deg; at 9 deg it is 26% high and 8-10 deg do not converge.
- Drag at Re 100k is where the method fails: 22-28% low across the measured laminar-bubble drag rise (0-3 deg) and
  29-51% high from 6 deg, where the model separates early (separation moves to 0.13-0.3 c by 8-10 deg).
- Context, not scored: the two tunnels themselves differ by up to 26.7% in Cd (Re 100k, 3 deg) and 11.6% in Cl.

Bubble (upper-surface reversed-flow regions from wall shear, `out/cfd/runs/v7/*/result.json`): Re 100k, 0.49-0.89 c at
0 deg moving forward to 0.34-0.72 c at 6 deg; Re 200k, shorter, 0.49-0.75 c at 0 deg (split into two reversed regions) to
0.37-0.58 c at 6 deg. No measured bubble positions are in the reference file, so these are not scored.

Grid study (`out/cfd/runs/v7-grid/`): levels 0.667 (25k cells), 1 (55k), 1.5 (124k) at alpha 0, 4, 8, 10. Levels 1
and 1.5 (Re 100k, all four alphas) agree within 2% in Cl and 6% in Cd. Level 0.667 is within 4% in Cl of level 1 except
at Re 200k, 10 deg (10%, at stall), and within about 10% in Cd except Re 100k, 8 deg (24%). Level 1.5 at Re 200k was
stopped for time. Several grid points are unconverged (flagged in the files). The deviations from the tunnel are
therefore the transition model's, not the grid's.

Not run (time): the Tu sensitivity (0.05%, 0.15%) and the separate steady-versus-transient cross-check group. Seven V7
points did run time-accurately as the declared fallback.

### Polar tables

`data/calibration/polars/e387.json` holds the 26 V7 points (Re 100k and 200k, even alpha -2 to 10 on the polar grid
of alpha) and marks every other Re and alpha `not-run`; it carries the V7 verdict. The polar sweeps for Re 30k-200k
and the other sections (thin-plate, sd7037, film-rib-4, sd7003, ag24, mh32, s5010) were not run. The film-rib and plate
sections are parametric interpretations (`cfd/sections.py`), not measured shapes.

### Leading-edge suction efficiency eta_s

`data/calibration/suction.json` (every point run, none fitted or selected; histories inside). E387 section, Tu 0.1%,
kOmegaSSTLM, level 0.667 grid unless stated. eta_s (strip.js form) = mean(Cc + Cd_f cos a) / mean(CL sin a) over the
cycles after the first, Cd_f = lowest converged V7 Cd at Re 100k (0.0162). The pure-plunge points also give the
Garrick-based forms (`eta_s`, `eta_s_literal`), defined in the file.

| Point | Motion | Re | k | alpha_eff range (deg) | amplitude (deg) | cycles averaged | eta_s (strip form) | per cycle | Garrick form |
|---|---|---|---|---|---|---|---|---|---|
| `e387-L0.667-Re100000-k0.35-h0.75-a4` | pure plunge (protocol) | 100000 | 0.350 | -23.7 to 31.7 | 27.7 | 2 | 0.136 | - | 0.058 |
| `e387-L0.667-Re100000-k0.5-h1-a4` | pure plunge (protocol) | 100000 | 0.500 | -41.0 to 49.0 | 45.0 | 2 | 0.113 | - | 0.080 |
| `e387-L0.667-larus-v14-z0.4` | design kinematics (cruise, flight log) | 174658 | 0.182 | -6.7 to 12.2 | 9.5 | 2 | 0.533 | 0.61, 0.47 | - |
| `e387-L0.667-larus-v14-z0.75` | design kinematics (cruise, flight log) | 144109 | 0.140 | -8.0 to 10.9 | 9.4 | 2 | 0.445 | 0.79, -0.15 | - |
| `e387-L1-Re100000-k0.35-h0.75-a4` | pure plunge (protocol) | 100000 | 0.350 | -23.7 to 31.7 | 27.7 | 0 (partial) | - | - | - |

Grid sensitivity: `e387-L0.667-Re100000-k0.35-h0.75-a4` against `e387-L1-Re100000-k0.35-h0.75-a4` over their common 0.973 cycles: mean Cc differs by -4.5% (window inside the start-up cycle: compares the start-up transient on both grids, not the cycle-averaged eta_s). The V7 grid study puts level 0.667 within 2-4% of finer grids in Cl.

Reading: in the protocol's pure plunge (effective angle +/-28 and +/-45 deg, dynamic stall) eta_s is 0.11-0.14. At
the motion Larus v14 flies (cruise from `out/flight/larus-gl-2-M2.json`, sections from `src/physics/kinematics.js`,
effective angle about -8 to +12 deg) the two-cycle means are 0.53 (arm wing, 0.4 semispan, heave only) and 0.45 (hand
wing, 0.75 semispan, heave and twist), but neither is periodic: the arm wing's two cycles give 0.61 and 0.47, and the
hand wing's give 0.79 and -0.16, its mean lift dropping from 0.47 to 0.03 between them. These are not settled values;
more cycles are needed, and the window did not allow them (a case cannot be extended once cleaned). The pure-plunge
fine-grid check was stopped on request inside its start-up cycle; over that common window the two grids differ by
4.5% in mean chordwise force. Motion construction: the airfoil turns about its spar axis and translates as a solid
body (`sixDoFMotion`, angles in degrees, sense checked on a short run) so that its relative wind and pitch rate equal
the kinematics at every instant (`cfd/suction_kin.py`, tests in `cfd/test/test_late.py`).

### 3D finalist check

Not done for the current design. The pipeline is built and tested end to end (`pose.mjs` from
`src/physics/kinematics.js`, body and tail from `src/physics/geometry.js`, half model, snappyHexMesh with prism layers:
82% of target layer thickness on the wing and 85% on the tail at 5 mm surface cells, layer stack sized to fit one cell).
The one longer run used a superseded design and stopped unconverged at 618 iterations; it is kept only as a pipeline
record in `out/cfd/larus-gl-2/partial-earlier-design/` and is not a result.

### Moving-mesh feasibility

`out/cfd/moving-mesh-feasibility.json`: no. Estimated 3-6 h on a quiet host for one flapping case, the image has no
overset solver, and 2D transients needed Courant <= 4 to stay stable. Not built.

### Code checks

`python3 -m unittest discover -s cfd/test`: 81 tests, against independent references (Langtry-Menter correlation,
Wilcox decay, Theodorsen table, Garrick limit, closed-form heave, synthetic histories with a known eta). They found and
fixed real bugs: wall faces split one face off at the leading edge, a zeroGradient wall writing no pressure, an empty
saved-field folder, the 3D parser missing `nan` (a diverging 3D run would not have been killed), a 3D stop gate that
waited 2000 iterations, and a C-grid cap triangle degenerate at the trailing edge.
dryer: no duplicates. crapper (coverage from that suite): every function under 30 except the Docker-driving
orchestration (`suction.run`, `pipeline.container`, `bird3d.finish`, the `main`s), which the fake-solver tests
exercise only in part. mutator on `suction.py`, `suction_kin.py`, `case3d.py` (run in a copy): the kinematics
construction and evaluation are pinned (11 and 6 survivors left, mostly boundary tolerances and the loop-closure
correction, which is an equivalent mutant: the mesh velocity is mean-zero by construction, so the drift it removes is
exactly zero); the remaining survivors are in output formatting of `suction.write`.

### Resuming

Docker must be running. `tools/run-guarded.py --timeout 200000 --log out/cfd/logs/run-all.log -- python3
cfd/run_all.py` continues the 2D programme from where it stopped (finished points are never rerun; a point whose
steady stage finished resumes at its transient stage). `out/cfd/slots` caps concurrent 2D cases. The 3D case:
`python3 cfd/bird3d.py larus-gl-2 --tag final --surface-cell 0.004 --first-layer 5e-5 --iterations 3000`, then
`python3 cfd/report3d.py larus-gl-2`.
