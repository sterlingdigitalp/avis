#!/usr/bin/env node
// Round 6, for the orchestrator's coming eta_s ruling (information, not a gate): how the final design depends on the
// leading-edge suction efficiency eta_s (strip.js SUCTION, 0.6 assumed).
//   node src/physics/gates/suction-sensitivity.mjs data/designs/larus-gl-2.json [--seeds 100] [--tol 0.01]
// Writes docs/status/round6-suction-sensitivity.json:
//  1. the effective-angle amplitude in cruise at every strip station (cycle min, max, half range), from the strip
//     model flying the design's own nominal cruise (the M2 start, sensor noise on, no wind);
//  2. for M1, M2 and M4 (nominal; M4 on its 100 seeds, pass at >= 98): the lowest eta_s in [0.1, 0.6] at which the
//     gate still passes, by a grid then bisection. The design is flown as it is: its trim tables, envelope and
//     controller were made at eta_s 0.6, and nothing is re-trimmed, so the closed loop has to absorb the change;
//  3. SmartBird V4 at eta_s 0.12 (information only, from V4's own evidence, out/verification/evidence/V4.json).
import { readFileSync, existsSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { runPool } from './pool.js';
import { writeJson, commandLine, ROOT } from './common.js';
import { setupFlight } from '../scenarios.js';
import { SUCTION } from '../strip.js';

const DEG = Math.PI / 180;

/** Effective angle (strip.js alphaEff: induced angle off, Wagner-lagged) over whole cruise cycles, per strip. */
export function cruiseEffectiveAngles(design, { settle_s = 6, cycles = 8 } = {}) {
  const { sim, tick } = setupFlight(design, { init: { y: 0 } });
  const strip = sim.bird.strip;
  const st = sim.bird.st;
  const n = st.n;
  const lo = new Float64Array(2 * n).fill(Infinity), hi = new Float64Array(2 * n).fill(-Infinity);
  const glo = new Float64Array(2 * n).fill(Infinity), ghi = new Float64Array(2 * n).fill(-Infinity);
  let recording = false;
  const evaluate = strip.evaluate;
  strip.evaluate = (...a) => {
    const r = evaluate(...a);
    if (recording && a[7] > 0) {
      for (let q = 0; q < 2 * n; q++) {
        const v = r.alphaEff[q], g = r.alpha[q];
        if (v < lo[q]) lo[q] = v;
        if (v > hi[q]) hi[q] = v;
        if (g < glo[q]) glo[q] = g;
        if (g > ghi[q]) ghi[q] = g;
      }
    }
    return r;
  };
  const HZ = Math.round(1 / sim.dt);
  for (let k = 0; k < settle_s * HZ; k++) tick();
  // Whole cycles of the crank, counted from its phase wrapping.
  let wraps = 0, last = sim.snapshot().phase, steps = 0, f = 0;
  while (wraps < 1) { tick(); const p = sim.snapshot().phase; if (p < last) wraps++; last = p; }
  recording = true;
  const t0 = sim.t;
  while (wraps < 1 + cycles) { tick(); steps++; const p = sim.snapshot().phase; if (p < last) wraps++; last = p; }
  f = cycles / (sim.t - t0);
  strip.evaluate = evaluate;
  const L = sim.bird.wing.L;
  const rows = [];
  for (let q = 0; q < 2 * n; q++) {
    const i = q % n;
    rows.push({
      wing: q < n ? 'right' : 'left', station: i, s_m: +st.s[i].toFixed(4), frac_semispan: +(st.s[i] / L).toFixed(3),
      alpha_eff_min_deg: +(lo[q] / DEG).toFixed(2), alpha_eff_max_deg: +(hi[q] / DEG).toFixed(2), alpha_eff_amplitude_deg: +((hi[q] - lo[q]) / 2 / DEG).toFixed(2),
      alpha_geo_min_deg: +(glo[q] / DEG).toFixed(2), alpha_geo_max_deg: +(ghi[q] / DEG).toFixed(2),
    });
  }
  return { cruise_airspeed_target_m_s: design.control.Vcruise, flap_hz_measured: +f.toFixed(3), cycles, settle_s, stations: rows, wing_length_m: L };
}

/** Pass or fail of M1, M2 (120 s, as the gate) and M4 (seeds 1-N, >= 98%) on the nominal design at one eta_s. */
async function gatesAt(path, eta, seeds) {
  const perturb = { suctionEta: eta };
  const jobs = [{ task: 'M1', args: { design: path, opts: { perturb } } }, { task: 'M2', args: { design: path, opts: { perturb, tEnd: 120 } } }];
  for (let s = 1; s <= seeds; s++) jobs.push({ task: 'M4', args: { design: path, seed: s, opts: { perturb } } });
  const r = await runPool(jobs);
  const m4 = r.slice(2);
  const m4pass = m4.filter((x) => x && !x.error && x.passed).length;
  const pick = (x, keys) => (x && !x.error ? Object.fromEntries(keys.map((k) => [k, x.measured[k]])) : { error: String(x?.error).slice(0, 200) });
  return {
    eta,
    M1: { passed: !!(r[0] && !r[0].error && r[0].passed), ...pick(r[0], ['height_lost_m', 'climb_rate_2_to_3s_m_s']) },
    M2: { passed: !!(r[1] && !r[1].error && r[1].passed), ...pick(r[1], ['endurance_min', 'power_ratio', 'cruise_power_w']) },
    M4: { passed: m4pass >= Math.ceil(0.98 * seeds), seeds_passed: m4pass, seeds_run: seeds },
  };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const path = resolve(process.argv[2]);
  const design = JSON.parse(readFileSync(path, 'utf8'));
  const arg = (k, d) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : d);
  const seeds = +arg('--seeds', 100);
  const tol = +arg('--tol', 0.01);
  const t0 = Date.now();
  const angles = cruiseEffectiveAngles(design);
  const cache = new Map();
  const at = async (eta) => {
    const key = eta.toFixed(4);
    if (!cache.has(key)) { cache.set(key, await gatesAt(path, eta, seeds)); console.log(key, JSON.stringify(cache.get(key))); }
    return cache.get(key);
  };
  const grid = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
  for (const e of grid) await at(e);
  const thresholds = {};
  for (const g of ['M1', 'M2', 'M4']) {
    const ok = (e) => cache.get(e.toFixed(4))[g].passed;
    const passing = grid.filter(ok);
    const monotonic = grid.every((e, i) => i === 0 || !ok(grid[i - 1]) || ok(e));
    if (!ok(0.6)) { thresholds[g] = { lowest_passing_eta: null, note: 'fails at eta_s 0.6', grid_passing: passing, monotonic_on_grid: monotonic }; continue; }
    if (ok(0.1)) { thresholds[g] = { lowest_passing_eta: 0.1, note: 'passes at the bottom of the range (0.1); the threshold is below it', grid_passing: passing, monotonic_on_grid: monotonic }; continue; }
    // Bracket: the highest failing grid point below the lowest passing point above it.
    let hiE = Math.min(...grid.filter((e) => ok(e) && grid.filter((x) => x > e).every(ok)));
    let loE = Math.max(...grid.filter((e) => e < hiE));
    while (hiE - loE > tol) {
      const mid = +((hiE + loE) / 2).toFixed(4);
      if ((await at(mid))[g].passed) hiE = mid; else loE = mid;
    }
    thresholds[g] = { lowest_passing_eta: hiE, highest_failing_eta: loE, tolerance: tol, grid_passing: passing, monotonic_on_grid: monotonic };
  }
  const v4p = join(ROOT, 'out/verification/evidence/V4.json');
  const v4 = existsSync(v4p) ? JSON.parse(readFileSync(v4p, 'utf8')).sensitivity_information_only?.['suction_eta_0.12'] ?? null : null;
  const out = writeJson('docs/status/round6-suction-sensitivity.json', {
    note: 'Information for the eta_s ruling, not a gate and not a fit. The design is the final one flown unchanged (trim tables, envelope and controller made at eta_s 0.6, not re-trimmed); eta_s is the same constant on every strip (buildBird perturb.suctionEta). Nothing here changes SUCTION.eta.',
    design: design.name, version: design.version ?? null, assumed_eta_s: SUCTION.eta,
    effective_angle_cruise: angles,
    thresholds_lowest_passing_eta_s: thresholds,
    runs: [...cache.values()].sort((a, b) => a.eta - b.eta),
    smartbird_v4_at_eta_0_12_information_only: v4 ? { source: 'out/verification/evidence/V4.json sensitivity_information_only.suction_eta_0.12 (src/physics/gates/v4.mjs)', ...v4 } : 'not available: run src/physics/gates/v4.mjs first',
    minutes: +((Date.now() - t0) / 60000).toFixed(2),
    ran_at: new Date().toISOString(), command: commandLine(),
  });
  console.log('thresholds', JSON.stringify(thresholds), '->', out);
}
