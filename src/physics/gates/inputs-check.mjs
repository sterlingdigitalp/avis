#!/usr/bin/env node
// Round 7, SPEC ruling after round 6 ("step-dependent input"): is every input of the M4 flights the same function of
// time at the design step and at half of it? Not a gate; evidence for M7's half-step check.
//   node src/physics/gates/inputs-check.mjs data/designs/larus-gl-2.json [--seeds 1,2,3]
// Each input is driven on its own with the same exogenous signals at both steps and compared at the instants the two
// steps share (every step of the coarser one):
//  - wind (Dryden turbulence on its 1/240 s clock, rotary terms, 1-cosine gusts at their drawn onset times);
//  - the flight controller (its tick instants, sensor-noise draws per tick, GPS timer and command delay line), fed the
//    same truth signal, with sensor noise on;
//  - the crank governor and its 20 ms line (tick instants and duty), fed the same crank phase and voltage.
// Writes out/verification/evidence/M7-inputs-<design>.json.
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { writeJson, commandLine } from './common.js';
import { m4Wind } from '../scenarios.js';
import { createWind } from '../wind.js';
import { createController } from '../control.js';
import { createFcGovernor } from '../governor.js';

const TWO_PI = 2 * Math.PI;

/** Largest difference between two step resolutions of a signal at the coarse step's instants. */
function compare(run, dtA, T) {
  const a = run(dtA, T), b = run(dtA / 2, T);
  let max = 0, n = 0;
  for (const [t, va] of a) {
    const vb = b.get(t);
    if (!vb) return { comparable: false, missing_t: t };
    for (let i = 0; i < va.length; i++) max = Math.max(max, Math.abs(va[i] - vb[i]));
    n++;
  }
  return { comparable: true, shared_instants: n, max_abs_difference: max, identical: max === 0 };
}
const key = (k, dt) => +(k * dt).toFixed(9);

export function windAt(design, seed) {
  return (dt, T) => {
    const w = m4Wind(design, seed);
    const wind = createWind(w.spec, seed);
    const out = new Map();
    const N = Math.round(T / dt);
    for (let k = 0; k <= N; k++) {
      // The bird's airspeed varies (here as a made-up signal): with a fixed convection speed it must not matter.
      const r = wind.step(k * dt, dt, 15 + 2 * Math.sin(k * dt * 3.1));
      out.set(key(k, dt), [...r.world, ...(r.rot ?? r.rates ?? [])]);
    }
    return out;
  };
}

export function controllerAt(design, seed) {
  return (dt, T) => {
    const c = createController(design, { seed, delay: 0.02, sensorNoise: true, rate: design.control?.rate_Hz });
    const truth = (t) => ({ t, dt, omega: [0.1 * Math.sin(2 * t), 0.05 * Math.cos(3 * t), 0], sf: [0, 9.80665, 0], up: [0, 1, 0], y: Math.sin(0.5 * t), vel: [15, 0.3 * Math.cos(t), 0], pos: [15 * t, Math.sin(0.5 * t), 0], airspeedX: 15 + Math.sin(t), phase: (2 * TWO_PI * t) % TWO_PI, crank: 2 * TWO_PI, heading: 0, battI: 10 + Math.sin(t), battV: 7.4 });
    c.initAttitude(truth(0));
    c.guidance.speed = 15;
    const out = new Map();
    const N = Math.round(T / dt);
    for (let k = 0; k <= N; k++) {
      const cmd = c.update(truth(k * dt), k * dt);
      out.set(key(k, dt), [cmd.freq, cmd.amp, cmd.sym, cmd.dyn ?? 0, cmd.tailPitch ?? 0, cmd.tailRoll ?? 0, c.sensedAmps]);
    }
    return out;
  };
}

export function governorAt(design) {
  return (dt, T) => {
    const gov = design.drive.governor;
    const g = createFcGovernor(gov, { G: design.drive.gearRatio, kv: design.motor.kv_rad_per_v_s, R: design.motor.R, cells: design.battery.cells, delay_s: 0.02 });
    const ph = (t) => 2.5 * TWO_PI * t + 0.3 * Math.sin(5 * t);
    g.reset({ t: 0, phase: 0, crank: 2.5 * TWO_PI, V: 7.6 });
    const out = new Map();
    const N = Math.round(T / dt);
    for (let k = 0; k <= N; k++) {
      const t = k * dt;
      if (g.st.fresh || g.due(t)) g.tick(t, ph(t), 7.6 - 0.1 * Math.sin(t), () => 2.6 * TWO_PI, false, true);
      out.set(key(k, dt), [g.st.applied, g.st.ticks, g.phaseAt(t)]);
    }
    return out;
  };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const path = resolve(process.argv[2]);
  const design = JSON.parse(readFileSync(path, 'utf8'));
  const arg = (k, d) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : d);
  const seeds = arg('--seeds', '1,15,27').split(',').map(Number);
  const dt = design.control.dt;
  const T = 30;
  const res = {
    dt_s: dt, dt_half_s: dt / 2, duration_s: T, controller_rate_Hz: design.control.rate_Hz ?? 100, governor_rate_Hz: design.drive.governor?.rate_Hz ?? null,
    wind: Object.fromEntries(seeds.map((s) => [`seed ${s}`, compare(windAt(design, s), dt, T)])),
    controller: Object.fromEntries(seeds.map((s) => [`seed ${s}`, compare(controllerAt(design, s), dt, T)])),
    governor: compare(governorAt(design), dt, T),
    gust_onsets: 'drawn per seed as times (wind.js randomGusts: uniform in time slots), not read from the simulated state',
  };
  const all = [...Object.values(res.wind), ...Object.values(res.controller), res.governor];
  res.identical = all.every((r) => r.comparable && r.max_abs_difference <= 1e-12);
  res.ran_at = new Date().toISOString();
  res.command = commandLine();
  const p = writeJson(`out/verification/evidence/M7-inputs-${design.name}.json`, res);
  console.log(JSON.stringify({ identical: res.identical, wind: Object.values(res.wind).map((r) => r.max_abs_difference), controller: Object.values(res.controller).map((r) => r.max_abs_difference), governor: res.governor.max_abs_difference }), '->', p);
}
