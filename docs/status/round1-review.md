# Round 1 independent review (read-only reasoning agent), 2026-11-02

Spot-checked by the orchestrator: claims 1 and 2 below were confirmed in the code (`src/design/trim.js:12-20, 55-58`; `src/physics/strip.js:19, 262`).

## Why SmartBird power came out 7 W against 18 W measured

The 7 W is the frozen-body fallback (`gates/v4.mjs:157`); free flight never ran because `buildControl` returned null.

1. **Signed power averaging with a regeneration credit (largest).** `drivePowerFromLoad` lets P = V·I go negative when aero or inertia drives the crank, and `trim.js:55-58` averages it signed. Mean mechanical crank power was 0.95 W for a 0.48 kg flapper: downstroke positive work and upstroke negative work nearly cancel. A non-regenerating drive must supply the positive peaks and waste the negative. Free flight, which counts only positive duty (`flight.js:531/540`), gave 2-4 times more. Most of the 7 W is motor no-load loss plus avionics, not aerodynamic work.
2. **Full leading-edge suction at Re ~ 10^5 (modelling choice).** `fT = kS·(Lp·sinα) − Dp·cosα` with `kS = 1` (`strip.js:262`). Flapping thrust is mostly leading-edge suction; taking all of the ideal lets the model trim at 2 Hz with 9.5 deg of stroke, against SmartBird's roughly 20 deg, and at the published 0.3 m plunge it needs only 1.04 Hz.
3. **Non-converged trim reported as the answer (bug).** The 2 Hz trim had `converged: false`, residual 0.30 N against a 0.235 N threshold.
4. **Drag inputs possibly low (unverified).** Film drag factor 1.2 is assumed; E387 stands in for NACA 7412; body CdA unchecked.

No unit errors were found in the motor model.

## The 43-53 g load spikes

Mostly artefacts. Failed M4 seeds stay upright (roll 27 deg, pitch 27 deg, no departure) and fail on altitude. The load factor is specific force at the body origin, not the CG (`flight.js:605/618`), unfiltered at one RK2 substep of 1/240 s, with stall flag chatter and a differenced apparent-mass term at gust fronts (`strip.js:282`). No timestep convergence study was run.

## V1 and V3

Real evidence. `references.js` computes Theodorsen and Garrick in closed form and imports no solver. V3's monotonic-trend check uses the solver's own output, but its accuracy check is against Garrick.

## M7 and M8

Formulas are right; the population is wrong. Loads and currents take the worst over every M4 seed, including the ones that failed, so structure is sized by non-flying transients. `couplerForce` floors the transmission angle at sin = 0.2 (`flight.js:633`).
