# Diagnose: why the v25 work stops the bird trimming

## Summary

**1. Main cause: v25 "does not trim" because of a numerical bug.** The twist-chain compliance in flight.js (:416-418) is applied as a one-step-lagged offset, θ = S - C·T(prev).
- The loop is unstable once C exceeds about 0.11 rad/N m at dt 1/240 s. The threshold falls in proportion to dt.
- v25's C is 0.208, so the cycle diverges: fx -380 N, twist torque 12.5 N m, 430 W.
- No V or dyn converges, and every free flight diverges as well.
- Fix (2.1), which v25 needs in order to trim:
  - frozen cruise V 10.5, dyn 22: 1.69-1.74 Hz, 47-50 W;
  - flight trim 2.338 Hz, 99 W;
  - against v24's own first pass: 2.133 Hz, 78.3 W.
- No design change stops the trim (2.3).

**2. The lead's two observations are artefacts.**
- **"v24 at 1.75 Hz / 41.6 W"** is the frozen trim at dyn 38°; both codes give it to every digit. v24's official 2.001 Hz / 71.1 W is the free-flight row at dyn 30°. Its frozen row is 1.602 Hz / 40.8 W, identical in both codes. Both powers are electrical, battery side. There is no drop.
- **"No trim even with v24's servo or the CG at target"** comes from testing V 10 only. At V 10.5 those cases converge.

**3. v24 under the new code** has identical trims, but the build stops at the envelope.
- The cause is the new member `crank web hub wall at the keyway` (linkage.js:331-334): FoS 2.377 against the 2.5 needed.
- Verdict: correct. A wall of 0.85 mm passes.
- v24 under a corrected model trims as before: 1.602 Hz / 40.8 W frozen, 2.001 Hz / 71.1 W in flight.

**4. Other bugs.**
- Protrusion drag is counted for one wing (round11.mjs:120).
- The rib-on-rod member load is about 8 times too high (twist-drive.js:612/:696).
- Tube product-of-inertia sign (massprops.js:79): latent, with no effect.

**5. The modules.**
- needs: +101 g, CG 7 mm forward.
- pod: its patch does not apply; module (b) reports +27.0 g, CG unchanged, no drag. Complete hardware is about 2001 g, CG 12.2 mm aft. With the fix it trims at V 11/30: 1.80 Hz, 49.8 W frozen; the committed code still cannot trim.
- Neither changes the cause. The chains throw hides nothing.

**6. To close it, inside the rules:**
- the three bug fixes;
- a ruling or a stronger servo for the "held to stall" twist arm: ≥ 1.54 N m at the servo;
- four members at 13.5 m/s raised by 1-3%;
- optionally, CG to intent: 35.9 g m (154 g of ballast at 0.25 m, or 37 g off the tail).
- The envelope is then 2.94 Hz, limited by the MD89MW's torque. v24's 3.27 Hz needs about 0.58 N m at the wing.
- Nothing from the must-not-change list is needed.

**Side note (pre-existing).** Frozen-cycle power is not dt-converged: 40.8 / 37.9 / 35.7 W at dt 1/240, 1/480 and 1/960 on v24's trim point.

## Method

Read-only on ~/avis. All runs are in scratch copies under `scratchpad/diag/` (DIAG below):

- `v24c` is `git archive a558baf`;
- `v25c` and `v25r16` are `git archive 0d9ba86`;
- `v25fix` is 0d9ba86 plus one diagnostic, env-gated change to `flight.js` (see 2.1).

Each copy has the gitignored inputs: the airfoil tables and `out/sweep/round9/v18-base.json`. Scripts are in `DIAG/scripts/`, logs in `DIAG/logs/`.

Scripts:
- `trim.mjs`: trimLevel at given V and dyn.
- `grid.mjs`: pickCruiseTrim's own grid, V 9-11 x dyn 22/30/38, and its pick.
- `point.mjs`: one frozen-body cycle with its power breakdown.
- `scan.mjs`: thrust surplus against frequency, with lift balanced.
- `track.mjs`: how well the twist servo tracks its command.
- `envdiag2.mjs`: one envelope flight test at the build's allowance, with the limiting member.
- `cgdecomp.mjs`: the CG change, lump by lump.
- `cmp.mjs`: the flight-relevant fields of two designs, side by side.
- `mod.mjs`: the one-change-at-a-time design edits, selected by env flags.

"Frozen" below means trim.js trimLevel/cycleResidual (body held, Newton on theta, freq and tail). "Flight trim" means refineTrimByFlight: closed loop, free body. Power is battery-side electrical in both, with no regeneration credit (trim.js batteryPower, flight.js tel.power_w).


## 1. v24 under the new code

### 1.1 Reproduction: the trims are identical, and "41.6 W" is not the quantity behind 71.1 W

`trim.mjs` was run on `data/designs/larus-gl-2.json` (v24, a558baf; identical in the working tree), with each code copy:

| code | V | dyn | freq Hz | theta deg | tail deg | P W (battery) | twist peak N m | residual / weight |
|---|---|---|---|---|---|---|---|---|
| a558baf | 10 | 22 | 1.5048 | 4.161 | 2.166 | 41.98 | 0.152 | 0.0030 |
| a558baf | 10 | 30 | 1.6018 | 4.114 | 2.050 | 40.80 | 0.156 | 0.0012 |
| a558baf | 10 | 38 | 1.7454 | 4.130 | 1.999 | 41.60 | 0.166 | 0.0021 |
| 0d9ba86 | all three | | the same to every printed digit | | | | | |

- **The lead's "1.75 Hz, 41.6 W at 38°" is the frozen-body trimLevel at V 10, dyn 38°.** It gives the same digits on the a558baf code, so the code change did not move it. Here "38°" is the twist amplitude (dyn); the stroke amplitude is 38° in every case.
- **v24's official 2.001 Hz / 71.1 W is a different trim.** It is the flight-refined row, `control.trimFlight[0].flight_power_w`: closed loop, free body, at dyn **30°**.
- **The frozen trim behind that official row** is `control.trimDetail[0]`: 1.603 Hz / 40.8 W, which this run reproduces (1.6018 / 40.80).
- **The two power figures are the same kind of quantity:** battery-side electrical, with no regeneration credit.
- **There is no 40% drop.** The free-flight refinement has always lifted v24 from 1.60 to 2.00 Hz (+25%) and from 40.8 to 71.1 W (+74%). Round 13's note in trim.js keptTrimRows already records free-flight trims running about 30% above frozen ones.

**Full builds.**
- `round16.mjs` on the a558baf code (`DIAG/out/v24-oldcode.json`, 888 s) reproduces the official v24 exactly: 1749.6 g, cruise V 10, 2.001 Hz / 71.13 W, envelope 38:3.27.
- `round16.mjs` on the 0d9ba86 code fails (`DIAG/v25r16/out/sweep/round16/v24.failed.json`, 828 s) with **"no stroke within ratings in flight"**.

The design that build produces differs from v24 in only two ways (`cmp.mjs`):
- mass is -0.34 g, from the nose-tray riser slot (audit item 23, airframe.js riserVolume);
- there are two new members, `crank web hub wall at the keyway` and `nose tray riser at the hole`.

Mass, inertia, CG, CdA, tail, servos and wing are all identical. Every one of the round-17 flight-model and build changes is gated: on p.tailMech, p.twistDrive, p.massFromSolids, the section record, or the servo block's travel_rad / compliance_rad_per_Nm. None of them is present for v24, so none of them reaches v24.

### 1.2 What stops v24 under the new code: a new member (correct model change)

`envdiag2.mjs` flies one flight test at the build's allowance (round12: member 0.6).

**On the new-code v24, at V 10.5 and 2.133 Hz (its first-pass cruise):**
- members ratio 1.052, ok false;
- limiting member: `crank web hub wall at the keyway`, FoS 2.377 at F = 1929 N;
- every other ratio is at or below 0.79.

**On the old-code v24, in the same flight:** members ratio 0.82, and the lowest member is a key at FoS 3.05.

**On the official v24 at its own cruise (V 10, 2.001 Hz):**
- the peak crank torque is 9.24 N m, so F = 1848 N;
- the new member would then sit at FoS 2.48;
- 1.5 / 0.6 = 2.5 is required, so the official design fails it too, by about 1%.

The envelope needs FoS ≥ 2.5 at every member, because 1.5 / 0.6 = 2.5. This one fails at the cruise flight itself, so no stroke passes.

**The member's code** (linkage.js:331-334, round 17, audit item 13; not gated, so it applies to every design):
- the force is the crank torque over the shaft radius: k = 2000 / 10 mm, i.e. 200 /m, so F = T / r;
- the wall is the hub radius minus the shaft radius minus the key's hub-side depth: 7 - 5 - 1.2 = 0.80 mm;
- the shear area is 2 planes x 0.80 mm x 6 mm = 9.6 mm²;
- d_mm is consistent with that area for a double-shear pin check.

| check | result |
|---|---|
| Units | N from N m / m; MPa from N / mm². |
| Magnitude | 1929 N / 9.6 mm² = 201 MPa in annealed Ti-6Al-4V. |

**Verdict: correct.** The check finds a real thin section: a 0.8 mm ligament behind the keyway. v24 never had to meet it. A 0.1 mm thicker wall, hub OD 14.2 mm, gives FoS 2.67 and passes, for well under 1 g. This is sizing to the FoS rule, inside the rules.

### 1.3 The flight-model changes checked one by one

On v24's data all of them are inert, as shown above. On v25's data:

| change | where | check | verdict |
|---|---|---|---|
| **Twist chain series compliance** | flight.js:416-418 (theta = S - C x lastTwistTorque); lastTwistTorque set at :806-807 | Sign right: lastTwistTorque is the torque the servo puts into the wing in +theta (-Q_air + Jtw theta'' + K theta), so the wing lags the servo by C T. Units right: rad/N m x N m. **Implementation numerically unstable:** an explicit, one-step-lagged algebraic loop through the strip's finite-differenced apparent mass (strip.js:406/422, dVn/dt) and other 1/dt terms. See 2.1. | **BUG** |
| Twist end stop | flight.js:411-413 (the command to servoRate clipped at ±travel_rad) | Sign and units right; the command itself is left unchanged. It never engages in the frozen trims: max command = dyn ≤ 0.663 rad < 0.675 rad, and NOT=1 (no stop) changes no digit. Only 0.012 rad is left for sym and diff in free flight. | correct |
| **Full inertia tensor of lumps** | flight.js lumpOwnInertia (:123-127) into Ib (:168-170), H (:1101) and KE (:1138); massprops.js solidProps / accumulate / central | Frame: body axes x forward, y up, z right, the same as lump.pos. Units: kg m², per kg x lump mass. About each lump's own centroid. Accumulator form [Ixx, Iyy, Izz, -Pxy, -Pxz, -Pyz]: correct for prisms. **For tubes (massprops.js:79) the products are stored with the wrong sign:** `-I(i,k)`, where `I(i,k)` is already the tensor element (= -P). A numerical test on a 45° tube in the y-z plane gives stored +6.47e-10 against the correct -6.48e-10. Tubes have a_x = 0, so only Iyz is affected, and left/right pairs in one lump cancel it: every v25 lump has Iyz about 1e-20 or under 1e-4. **No effect on flight.** The doubling of body Iyy/Izz (0.039 → 0.084, 0.042 → 0.086 kg m²) is real: about 61 g of tail hardware at -0.44 to -0.95 m (1.4). Body inertia does not enter the frozen trim at all. | correct (tube-product sign: latent bug, zero effect) |
| Tail root to the covered root rib | build.js tail block (tailRootShift); flight.js:219 tail stations | Flown span (0.5671 m), root chord (0.1906 m), area (0.1838 m²) and arm (x -0.9458 m) are unchanged. Only the root moves out: z 0.005 → 0.0264 m, y 0.062 → 0.077 m. Reverting to v24's tail block (TAIL24) changes the trim by < 0.01 Hz / 0.1 W. | correct |
| 16/13 tail ratio | build.js atSurface (servos.tail stall / k, speed x k) | Applied once: 0.6060 / 1.2308 = 0.4924 N m, and 7.588 x 1.2308 = 9.34 rad/s at the surface. flight.js reads servos.tail as is, with no further ratio. The chain check only compares flown with drawn. Reverting it (TAILSV24) changes nothing (48.1 vs 48.2 W). | correct |
| Wing-protrusion drag | round11.mjs:120 outsideDrag | The section record (twist-drive.js sectionRecord, "every solid of one wing") lists **right-wing parts only**: twist joiner R, servo arm R, frame pins R, the leading and trailing edge rods. Its CdA_m2 = 8.54e-4 m² is **one wing's**, but it is added once while the basis text says "both wings". | **BUG: undercounted by 8.54e-4 m²** |
| Outside fittings | CONTAINMENT17 (tail hardware outside, with drag) | CdA_front_outside 0.00104 → 0.00312 m². This is real hardware. | correct |
| Wing twist stiffness | build.js (twist-drive.js twistStiffness replaces the unsourced 0.05) | 0.0531 N m/rad. STIFF24 = 0.05 changes nothing. | correct |

## 2. Why the v25 probe design does not trim

### 2.1 The cause: the twist-compliance step is numerically unstable (BUG)

**Evidence 1: one frozen cycle at v24's trim point** (V 10, theta 4.13°, 1.7454 Hz, tail 2°, dyn 38°), with `point.mjs` on the lead's `v25.failed.json`:

| case | fx N | fy N | mz N m | P W | crank peak N m | twist peak N m |
|---|---|---|---|---|---|---|
| v24 | -0.03 | -0.02 | 0.00 | 41.6 | 5.31 | 0.166 |
| v25 as committed (C 0.208) | **-379.9** | -1.32 | **37.1** | **429** | **35.2** | **12.52** |
| v25, C = 0 | -0.57 | -1.30 | 0.40 | 39.1 | 4.41 | 0.177 |

A twist torque of 12.5 N m at the wing is 26 times the servo's stall, and 0.208 x 12.5 = 2.6 rad of wind-up. Neither is physical.

**Evidence 2: a sharp threshold that scales with the time step.** The lagged loop θ_k = S - C·T(θ_{k-1}) diverges once C x (per-step torque sensitivity) exceeds about 1. Sweeping C:

| dt | C stable up to | C unstable from |
|---|---|---|
| 1/240 s (cycleResidual's own rule below 4 Hz) | 0.10 | 0.12 |
| 1/480 s | 0.04 | 0.06 |
| 1/960 s | — | 0.04 |

The critical C falls in proportion to dt. So the loop gain is a 1/dt term, about 9 N m/rad at 1/240 s, and the scheme does not converge as dt → 0: any C > 0 diverges at a small enough step.

**Evidence 3: what carries the 1/dt term.**
- The main path is the strip's apparent mass. It differences the section's normal velocity in time (strip.js:406 and :422). A step in θ_wing becomes a huge dVn/dt.
- With apparent mass at 0 (NOAM), C 0.208 is stable at 1/240 s, but it still diverges at 1/960 s or at C 0.4, so other 1/dt paths exist as well.
- The wing angle includes -C·T, but its rate and acceleration (dtheta = servo rate, ddtheta = servo accel) do not. The kinematics are inconsistent too.

**Evidence 4: a stable treatment barely moves the result.** In `v25fix`, the deflection becomes a state δ, relaxed toward C·T with time constant τ (backward Euler), with θ_wing = S - δ:

| case | fx N | fy N | P W |
|---|---|---|---|
| τ 5 ms | -0.51 | -1.60 | 38.8 |
| τ 10 ms | -0.52 | -1.60 | 38.7 |
| τ 20 ms | -0.52 | -1.60 | 38.7 |
| τ 40 ms | -0.53 | -1.60 | 38.6 |
| C = 0 | -0.57 | -1.30 | 39.1 |

- τ 8 and 20 ms stay stable at dt 1/240, 1/480, 1/960 and 1/1920, and agree within 0.01 N at each dt.
- τ 4 ms diverges at dt ≤ 1/960.
- Feeding δ' into dtheta destabilises the loop unless τ ≥ 40 ms, and changes nothing where it is stable (38.7 W). So it is left out.

Correctly modelled, the compliance costs about 0.3 N of lift and changes power by -0.4 W. Its effect on the trim is small.

**Evidence 5: free flight is hit too** (envelope tests, flight trims, missions). This is v24's official design with v25's twist block swapped in (MD89MW, k 1.25, travel, C 0.208), in one strokeFlightTest at V 10 and 2.001 Hz, at the build's allowance (`envdiag3.mjs`):

| case | twist_servo_torque | gear | spars | members | diverged |
|---|---|---|---|---|---|
| committed code | **104.3** | 13.7 | 9.2 / 12.9 | 3.3 | **2** |
| fixed | 0.40 | 0.75 | 0.26 / 0.16 | 0.79 | 0 (ok true) |

**The fix** (flight.js, 0d9ba86; applied only in `DIAG/v25fix`, env-gated `TWTAU`):

| where | change |
|---|---|
| near :339 (`lastTwistTorque`) | declare `const twDefl = [0, 0]` and a time constant `TAU_CHAIN = 0.02` s. |
| :417 / :418 | `theta: S[16] - twDefl[0]` and `theta: S[17] - twDefl[1]`, in place of `S[16] - Ctw * lastTwistTorque[0]` (and [1]). |
| after :807 (the record block that sets lastTwistTorque) | `if (advanceDt > 0) for (w of 0, 1) twDefl[w] += (advanceDt / (TAU_CHAIN + advanceDt)) * (Ctw * lastTwistTorque[w] - twDefl[w]);` with `Ctw = imposeTwist ? 0 : servoT.compliance_rad_per_Nm ?? 0`. |
| setState | reset twDefl to 0 there too. |
| why τ | It must sit above the chain's own wind-up time C·B (about 5-8 ms, from the strip's apparent-mass damping, B ≈ 0.04 N m s/rad; the stability boundary measured above) and well below the stroke period (500 ms). Results do not depend on τ anywhere in 8-40 ms. |
| test | Pin it with: C 0.208, dt 1/240 and 1/960, finite twist peak below stall, and residual within 2% of C = 0. |

A fully consistent model would carry the chain's mode as its own implicit degree of freedom. The quasi-static state above is the minimal correct form.

### 2.2 With the fix, v25 trims

These runs use pickCruiseTrim's own grid (`grid.mjs`). They are first-pass, at η_s 0.6, as the build runs before attaching the η_s table.

| design | code | pick |
|---|---|---|
| v24 (η 0.6) | a558baf | V 10.5 / dyn 30: 1.701 Hz, 46.3 W |
| v24 (η table, second pass) | a558baf | V 10 / dyn 30: 1.602 Hz, 40.8 W (official frozen row) |
| lead's v25.failed.json, 1881.2 g | as committed | **none**: every point "no", residual about 21 x weight, 300-500 W |
| same | compliance 0 | V 10.5 / dyn 30: 1.818 Hz, 48.9 W |
| same | **fixed** | **V 10.5 / dyn 30: 1.815 Hz, 48.2 W** |
| 0d9ba86 probe, 1871.2 g (q3/base.json, body CdA set to the full build's 0.00727) | as committed | none |
| same | **fixed** | **V 10.5 / dyn 30: 1.748 Hz, 45.3 W** |
| **0d9ba86 full build** (round17.mjs in `v25c`, 2424 s → "no converged trim at cruise speed"; its v25.failed.json is 1867.8 g, CdA 0.00727) | as committed | **none**: residual about 21-23 x weight, 400-450 W |
| same | compliance 0 | V 10.5 / dyn 30: 1.795 Hz, 47.5 W |
| same | **fixed** | **V 10.5 / dyn 22: 1.686 Hz, 47.4 W** (dyn 30: 1.81 Hz, 47.7 W) |
| same, protrusions on both wings (CdA 0.00812) | fixed | V 10.5 / dyn 30: 1.819 Hz, 48.3 W |
| same, CG at target | fixed | V 10.5 / dyn 30: 1.828 Hz, 49.1 W |

**The lead's other findings are artefacts of how the trims were run.**
- **"Converges on a spurious 4.9 Hz branch":** the lead's NOC / SWAP / CGFIX runs were at **V 10 only**. At V 10 trimLevel's bracket is ill-conditioned for both birds:
  - v24 itself fails V 9.5/30 and lands on 4-5.6 Hz roots at V 9-9.5;
  - v25 with C = 0 at V 10/38 stops at residual 1.07% against the 1% tolerance.
- **"Even with v24's servo and the CG at target it does not trim":** at V 10.5, which the grid includes, every one of those cases converges.

### 2.3 What each v25 change costs (one revert at a time, fixed compliance, lead's v25 design, grid pick)

| revert | pick | Δ freq | Δ P |
|---|---|---|---|
| none (v25, fixed) | V 10.5/30, 1.815 Hz, 48.2 W | — | — |
| mass block → v24 (mass, CG, inertia, wing mass) | V 10/38, 1.925 Hz, 51.7 W (V 10.5/30: 1.77 Hz, 51.9 W) | -0.05 at 10.5 | +3.7 |
| weight only → v24 (all masses x 0.930, CG kept) | 1.742 Hz, 44.9 W | -0.073 | -3.3 |
| body lumps → v24 | 1.783 Hz, 46.3 W | -0.032 | -1.9 |
| wing mass → v24 | 1.794 Hz, 53.8 W | -0.021 | +5.6 |
| CG to target (battery at x_battery_m) | 1.835 Hz, 49.3 W | +0.02 | +1.1 |
| tail mass → v24 | 1.793 Hz, 47.0 W | -0.022 | -1.2 |
| twist servo torque and speed → HV6160's | 1.815 Hz, 44.8 W | 0 | -3.4 |
| twist travel stop removed | no change | 0 | 0 |
| compliance 0 | 1.818 Hz, 48.9 W | +0.003 | +0.7 |
| tail block → v24 (root shift) | 1.815 Hz, 48.2 W | 0 | 0 |
| tail ratio → 1 | 1.815 Hz, 48.1 W | 0 | -0.1 |
| protrusion drag removed | V 10.5/22 1.669 Hz, 46.6 W (dyn 30: 1.80 Hz, 47.6 W) | -0.015 | -0.6 |
| body CdA → v24's (protrusions and outside tail hardware) | 1.750 Hz, 45.2 W | -0.065 | -3.0 |
| wing twist stiffness → 0.05 | no change | 0 | 0 |

Notes on the rows:
- The wing-mass row gets dearer: v24's wing mass block has no twisting points and a heavier skin.
- The twist-servo row's power change is the servo bus current: the HV6160's draw is modelled differently.
- Under the fixed model nothing in this list stops the trim. It moves by +0.11 Hz and +1.9 W on v24's first pass, in proportion to +7.5% mass and +68% body CdA.

### 2.4 How the trim fails, in the categories asked

| category | finding |
|---|---|
| Frequency past the member envelope, power past what is available | No. The frozen cruise needs 1.75-1.82 Hz and 45-49 W. |
| Pitch balance / tail authority | No. The trim tail is 1.0-1.9°, against 30° of travel. CG 19 mm aft gives static margin 0.046, not 0.10: stable, but outside the design's intent (1.4). |
| Twist saturation | No, in the frozen trim. Servo/command amplitude is 0.98-1.0 up to 2.5 Hz at dyn 38°; the lag is the servo's tau 0.02 s, the same as v24 (`track.mjs`). |
| Solver failure | **Yes: the cycle itself diverges**, through the compliance bug. trimLevel cannot close a residual of about 380 N. |

### 2.5 With the fix, the full v25 build gets past the trim and stops at the envelope

Build: round17.mjs on 0d9ba86 plus the scratch fix (`v25fix`, TWTAU 0.02), 2241 s. It fails with "no stroke within ratings in flight" (`DIAG/in/v25-fixC-failed.json`).

**Trim stages:**

| stage | result |
|---|---|
| Frozen cruise (first pass, η 0.6) | V 10.5, dyn 22: 1.743 Hz, 49.5 W |
| Frozen rows | 11: 1.723 Hz / 50.4 W; 12.5: 1.780 / 56.4; 13.5: 1.880 / 64.1; 14.5: 2.005 / 74.2 |
| **Flight trims (refineTrimByFlight)**, all converged | **10.5: 2.338 Hz, 99.0 W**; 11: 2.354 / 99.6; 11.5: 2.388 / 104.1; 12.5: 2.508 / 115.9; 13.5: 2.683 / 131.9 (the 14.5 row was dropped) |
| Like for like, v24 under the new code, first pass, V 10.5 | 2.133 Hz, 78.3 W |

- v25 needs +9.6% frequency and +26% power in free flight.
- The free-flight step grows: frozen → flight is +34% in frequency for v25, against +25% for v24.
- The extra pitch inertia and the aft CG are the plausible contributors. They were not isolated, because the controller's gains are tuned on the design's own inertia.
- The frozen envelope is freqMax 3.15 Hz at 38°, limited by twist_servo_torque at 14.5 m/s.

**The flight envelope fails on members at the cruise flight itself.** The required FoS is 1.5 / 0.6 = 2.5 (`envdiag2.mjs`):

| flight | members ratio | lowest members (FoS, peak F) | other ratios |
|---|---|---|---|
| V 10.5, 2.338 Hz | 2.10 | wing ribs R on the edge rods **1.19** (14.6 N); twist servo arm R **1.32** (28.8 N); ribs L 1.34; arm L 1.50; key, crank web 2.67 (10.0 N m) | twist_servo_torque 0.65, gear 0.82 |
| V 13.5, 2.683 Hz | 2.77 | ribs R **0.90** (19.3 N); arm R **0.98** (38.5 N); ribs L 1.03; arm L 1.14; key, crank web **2.44** (10.9 N m) | twist_servo_torque 0.87, gear 0.90 |

The crank-web hub wall that stops v24 is not a limit here: v25's crank has a 3.5 mm wall, FoS of about 7.

**(a) `wing ribs R/L on the edge rods`: the load model overstates the load (BUG).**
- twist-drive.js:612 sets `Rrod = max(R_N_per_rad) / kAero`, where `kAero = d.control?.diffLimit?.twist_torque_per_rad ?? stiff.K_Nm_rad`.
- :696 loads it as `T_(Q, fRod)`: the twist_R channel, which is the twist torque in N m, times Rrod.
- So the twist the rods are taken to see is torque / stiffness. When members are built, d.control does not exist yet, so kAero = K_rods = 0.0531 N m/rad. Even the later control figure, diffLimit.twist_torque_per_rad, is only 0.069 N m/rad.
- At the cruise peak of 0.28 N m that is a "twist" of 5.3 rad. The hand wing's twist is set by the servo and cannot exceed travel_rad 0.675 rad, and the flight model already has that angle as a state.
- The member's coefficient is 46.86 N per N m (k on twist_R), so R_N_per_rad = 46.86 x 0.0531 = 2.49 N/rad. The true bound is R x travel = 2.49 x 0.675 = **1.7 N**, against the 13.2 N used: about 8 times too much.
- With the bound replaced: capacity 17.4 N (1.19 x 14.6). The load is at most 1.7 N + the air share 0.076 x root_yz (about 1.4-1.6 N), giving FoS ≥ 5.3 at both speeds. It passes 2.5.
- **Fix:**
  - **Constant bound (simplest):** at :696 replace the `T_(Q, fRod)` term with the twist bound R_max x travel_rad. That is `Math.max(...stiff.rods.flatMap((r) => r.R_N_per_rad.map(Math.abs))) * twistTravel`, where twistTravel = servoTravel(G, P) or d.servos.twist.travel_rad. It is a constant in the section, or carried on a twist-angle channel.
  - **Honest alternative:** add LOAD_CHANNELS `twist_angle_R/L` = |S[16] - δ| and use k = R_max.
  - Either way the stated bound "torque / stiffness" goes. It is not a bound, because nearly all of the twist torque is air and inertia reacted by the servo, not rod bending.

**(b) `twist servo arm R/L`: correct as ruled, but the ruling interacts badly with the allowance.**
- twist-drive.js:676: the maker's R-ML25 arm is "held to the servo's published stall torque" at its 16 mm hole, so rating_N = 0.606 / 0.016 = 37.9 N.
- The load is twist_R x 102.1, i.e. the drawn chain's largest ratio over travel, 1.634, over 16 mm.
- Under M7's FoS 1.5 and the envelope's member allowance 0.6, the arm demands peak twist torque ≤ 0.148 N m at the wing: 40% of the servo's own stall.
- The servo itself is only held to 90% (twist_servo_torque, allowance 0.9).
- The check is internally consistent with the ruling (the tail arm has the same rule; its torques are small). In effect, though, it is a second and stricter servo-torque limit, 2.25 times tighter. **The orchestrator should rule on it**: a rated arm or horn, or holding a "held to stall" member to the servo's own allowance.

**(c) `key, crank web`: correct, marginal.** It is 3 x 1.5 x 4.3 mm and reaches FoS 2.44 at 13.5 m/s. A 4.4 mm key, or similar, clears 2.5.


### 2.6 The flight envelope after each fix (`envfull.mjs`: flightEnvelope at the build's allowance, V 10.5 and 13.5, 38°)

| variant | freqMax | limited by |
|---|---|---|
| fixed compliance, as built | **0** (no stroke) | members at 10.5 m/s (ribs on the rods, then the twist arm) |
| + rib load corrected | 0 | members at 10.5 (the twist servo arm, FoS 1.32 against 2.5) |
| + the arm member set aside (rated, or held to the servo's own allowance) | 0 | members at 13.5 m/s, all just under 2.5: key, crank web 2.44 (10.9 N m); forward plate under the forward boom clamp 2.47; twist link rod ends 2.54; tail boom at the aft clamp 2.59 (the heavier tail on the boom's end). Members ratio 1.025. |
| member check off (to see the next limit) | **2.935 Hz** | **twist_servo_torque at 13.5 m/s** (0.436 N m = 0.9 x the MD89MW's 0.484 at the wing). v24: 3.27 Hz, limited by members. The fastest flight trim is 2.683 Hz. |

## 3. The two unmerged modules

Probed with `$R/probe/quick17.mjs`. Copies, logs and saved designs are in `DIAG/q3/`.

**Patch status:**

| patch | status |
|---|---|
| needs `wiring-current.patch` | 2 hunks fail `git apply --check`, both one-line context drift: the twist-drive import line in build.js already has `chainCompliance`, and round17.mjs already has the retain2 `crank` merge. Merged by hand as two exact line substitutions; the rest applies as is. |
| needs `standin-current.patch` | Clean. |
| The 10 staged catalogue ids | All present in the probe catalogue. |
| pod `wiring.patch` | **Does not apply.** Every file's hunks fail (round17.mjs 5/5 rejected, clearance.js 2/3, airframe.js 1/3, build.js 1/6). It was made against a round17.mjs from before the twist drive, retention, chains, ratings and interfaces work. |
| pod `catalog.patch` | **Does not apply** either: blob mismatch; it assumes an older, shorter catalog.json. |

**Probe results:**

| design | mass g | clashes | records | body-lump CG x mm (target 16.0 whole-bird) | CG vs target | static margin |
|---|---|---|---|---|---|---|
| 0d9ba86 probe | 1871.2 | 0 | all pass but interfaces (encoder set screws) | -3.49 | -19.0 mm | 0.046 |
| + needs (wiring-current + standin) | **1972.3** (+101.1) | 0 | all pass but interfaces (same); buildneeds passes | 4.79 | **-12.0 mm** | **0.066** |
| + pod | not probed (patch does not apply) | | | | | |

- **pod, as module (b) reported** (orchestrator, after my hand-back):
  - +27.0 g with margin (1820.3 → 1847.3 g in its own copy);
  - CG effectively unchanged (body-lump x -6.26 against -6.25 mm);
  - no flight-model drag change, because the phantom tube carried none;
  - the patch fails hunks against 0d9ba86 and was not merged, as instructed.
- **Applied as a mass edit to the needs design** (`DIAG/q3/needs-pod.json`): the 'fuselage shell and formers' lump (5.89 g at x -43.28 mm in the needs design) replaced with the pod's 34.67 g at x -8.53 mm. That is +28.8 g, against the module's +27.0 g; the other -1.8 g of its total sits elsewhere and is not modelled.
- **Complete hardware (0d9ba86 + needs + pod): about 2001 g (+14.4% on v24).** CG 12.15 mm aft of target, static margin 0.065; 24.3 g m forward needed.

**Does it change the picture?** Trims with the fixed compliance. Body CdA is set to the full build's 0.00727 m²; the probe does not run outsideDrag, and the needs air-path drag is not included.

| design | committed code | fixed compliance |
|---|---|---|
| 0d9ba86 probe | none (residual about 21 x weight) | V 10.5/30: 1.748 Hz, 45.3 W |
| + needs | none | V 11/30: 1.793 Hz, 49.0 W (V 10.5/30: 1.83 Hz, 49.7 W) |
| + needs, protrusions counted on both wings (CdA 0.00812) | — | V 11/30: 1.813 Hz, 50.1 W |
| **+ needs + pod (2001 g)** | **none** (residual about 20 x weight) | **V 11/30: 1.804 Hz, 49.8 W**; both-wing protrusions 1.825 Hz, 50.7 W |

- The needs module moves the FC and regulator onto a belly tray, which brings the CG 7 mm forward (-19.0 → -12.0 mm) but adds 101 g. It does not change the cause: the committed code still cannot trim, and the fixed code trims at about 4 W more.
- The pod's mass adds +0.8 W and +0.01 Hz at V 11/30. It does not change the cause either.

**The chains.js "Cannot use 'in' operator ... in 0.20826".**
- The bare number is `compliance_rad_per_Nm: +comp.rad_per_Nm.toFixed(5)` in twist-drive.js twistChains, the figures object (line 976 at 0d9ba86). Every sibling there is a descriptor object.
- chains.js figureCheck (line 29 at 0d9ba86) already guards `if (typeof f !== 'object') return { …, info: true, ok: true }`. On 0d9ba86 with the needs patches, the '{}' run does not throw and chains passes. Agent (c)'s throw came from a chains.js older than that guard.
- **It hides nothing** in this diagnosis. The chains record is not in the flight path, and the compliance is flown from servos.twist.compliance_rad_per_Nm, not from the chain figure.
- Optional clean-up: twist-drive.js:976 → `compliance_rad_per_Nm: { value: +comp.rad_per_Nm.toFixed(5), units: 'rad/N m', info: true }`.
- `{"layout":{"chains":null}}` is not needed on 0d9ba86.

## 4. What would close it

Everything here assumes the compliance fix (2.1). Without it, nothing trims. The numbers are from the 0d9ba86 build design (1867.8 g) unless a row says otherwise.

### 4.1 Inside the rules

| item | what it needs | evidence |
|---|---|---|
| **Fix the compliance step** (bug) | flight.js as in 2.1 | trims: V 10.5 / dyn 22, 1.686-1.743 Hz frozen, 2.338 Hz / 99 W in flight |
| **Fix the rib-on-rod load** (bug) | twist-drive.js:612/:696 as in 2.5(a) | FoS 1.19 → ≥ 5.3 |
| **Count the protrusion drag on both wings** (bug) | round11.mjs:120 `wingOut = 2 * (sec?.CdA_m2 ?? 0)` (the record is one wing's) | +8.54e-4 m²: +0.6 W, +0.015 Hz frozen |
| **Twist servo** | **Torque at the wing**, by the servo rule (allowance 0.9): ≥ T_peak / 0.9. The flight-test peaks are 0.28 N m at cruise (10.5 m/s) and 0.38 N m at 13.5 m/s, so ≥ 0.31 / 0.42 N m. The MD89MW gives 0.484, which passes up to the envelope's top: frozen freqMax 3.15 Hz at 14.5 m/s is set by this very ratio. **Keeping v24's 3.27 Hz** needs about 0.45-0.52 N m at 14 m/s (frozen cycles at 3.0-3.3 Hz), so ≥ 0.58 N m at the wing. **Under the arm ruling** (the maker's arm held to stall, FoS 1.5, member allowance 0.6) the stall at the servo must be ≥ 2.5 x 1.634 x T_wing: **1.15 N m** at cruise, **1.54 N m** at 13.5 m/s, about **2.1 N m** at the envelope top. The MD89MW at 6 V gives 0.606 N m. The HV6160 at 7.2 V (v24's) gives 1.632 N m. Alternatively, a horn or arm with a published rating makes this check real. **Speed:** the commanded peak twist rate is 2π f dyn: 6.0 rad/s at cruise (2.34 Hz, 22°), 10.8 rad/s at 3.27 Hz and 30°. The MD89MW through k 1.25 gives 9.49 rad/s no-load at the wing. Its tracking falls to 0.95 of amplitude at 3.3 Hz / 30° (14 m/s). **Torque x speed product, ratio-free:** the servo needs stall x no-load speed ≥ (T_peak / 0.9) x ω_peak, about 5.0 W at 3.0 Hz and 6.2 W at 3.3 Hz (14 m/s). The MD89MW at 6 V is 0.606 x 7.59 = 4.6 W; the HV6160 at 7.2 V is 11.2 W. | envdiag2, track.mjs, frozen envelope |
| **CG to target** (static margin 0.10; now 0.045) | **First moment forward 35.9 g m.** **Ballast** at x 0.25 m (the pitot sits at 0.246): 154 g; at 0.30 m: 126 g. **Battery to its target place**: x 0.412 m, against 0.194 m drawn, i.e. +218 mm and a longer nose. **Tail hardware lighter**: 37 g off at x -0.946 m does it alone. The tail mechanism added +33.4 g at the tail (-31.6 g m) and +27.6 g of pushrods at -0.44 m (-12.1 g m); the crank and plates' +11 g forward give back +12.6 g m. **With the needs module** (belly tray, +101 g): 23.6 g m, i.e. 101 g of ballast at 0.25 m or 24.5 g off the tail. **Complete hardware (+ pod, 2001 g):** 24.3 g m, i.e. 104 g of ballast at 0.25 m or 25 g off the tail. The CG does not stop the trim: CGFIX costs +1.4 W frozen. | cgdecomp.mjs |
| **Crank-web key** (v25) | FoS 2.44 → 2.5 at 13.5 m/s: key length 4.3 → about 4.4 mm | envdiag2 |
| **Crank-web hub wall** (v24 only, if v24 is rebuilt) | wall 0.80 → ≥ 0.85 mm (hub OD 14.2 mm) | 1.2 |
| Lighter sizing to the FoS rule | The CG lever is the tail mechanism: every gram off at the tail is 0.96 g m. Its members' FoS in flight were not surveyed here; mission M7 peaks are needed for that. | — |

### 4.2 The must-not-change list (none is needed for the trim)

- **Stroke amplitude, frequency law, wing area and span, motor, pack, gears:** with the corrected model, v25 trims and the flight trims converge without touching any of them. They would matter only for the envelope's twist-servo and member limits:
  - the twist torque grows roughly as f² at a fixed speed (0.25 N m at 2.0 Hz, 0.45 at 3.0 Hz, 0.52 at 3.3 Hz at 14 m/s);
  - a lower envelope top (or a lower stroke-frequency law at the fast speeds) would let the MD89MW's 0.436 N m pass. About 2.9-3.0 Hz is where its 0.9 allowance binds (frozen), against v24's 3.27 Hz;
  - wing area or span, motor, pack or gears would change the trim frequency and power, but no limit found here calls for it.
- Recommended: none of these. The closing set is inside the rules.

