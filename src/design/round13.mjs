#!/usr/bin/env node
// Round 13 Larus: round 12's pipeline (round12.mjs) with the wing-loading lever of the round-13 brief: a lower aspect
// ratio at the 2.0 m span cap, the spar moved forward in the chord so the battery stays in the pod (the CG target moves
// aft with the wing's neutral point), and the free-flight trim refine allowed to carry a row that only missed a
// tolerance into the next pass (src/design/trim.js keptTrimRows, opt.trimFlight).
//   node src/design/round13.mjs --cand c45 [--out path] [--set '<json>'] [--opt '<json>'] [--cal path] [--no-suction]
//   [--no-fit] [--no-perf] [--official]
// Exploration builds go to out/sweep/round13/<cand>.json. data/designs/larus-gl-2.json is written only with
// --official, which is for a candidate that has passed every gate (the round-13 brief's v22 rule).
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound12 } from './round12.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Round 13 constants and candidates, over round 12's v21 parameters (ROUND12.set). sparFrac is the spar's (hinge
 * line's) fraction of the chord behind the leading edge: v21 flies 0.35, and each lower aspect ratio takes the value
 * at which the battery lands inside v21's pod (scratchpad r13/fitprobe.mjs: without it the CG target puts the
 * battery 0.2-0.3 m further forward than v21's, outside the loft, or on a tray too long to rate).
 *  - c45t: c45 on the lightest catalogue train without a mixed-material mesh rated for 20 N m at the crank at 4 Hz
 *    (scratchpad r12/trainsearch12.mjs --Tc 20 --Hz 4 --no-mixed --life 1500: 652 g of gears, ratio 41.15).
 */
export const ROUND13 = {
  opt: { trimFlight: { passes: 3, retry: true } },
  candidates: {
    c55: { AR: 5.5, sparFrac: 0.33 },
    c50: { AR: 5, sparFrac: 0.3 },
    c475: { AR: 4.75, sparFrac: 0.29 },
    c45: { AR: 4.5, sparFrac: 0.28 },
    c45t: { AR: 4.5, sparFrac: 0.28, parts: { gears: ['gear-khk-ds1-28', 'gear-khk-ds1-60', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msga1-70'] } },
  },
};

/** A candidate's --set: the candidate's block, then `set` over it (parts merged; linkage replaced as round 12 does). */
export function round13Set(cand, set = {}) {
  const c = ROUND13.candidates[cand];
  if (!c) throw new Error(`unknown round-13 candidate ${cand} (one of ${Object.keys(ROUND13.candidates).join(', ')})`);
  const parts = { ...(c.parts ?? {}), ...(set.parts ?? {}) };
  return { ...c, ...set, ...(Object.keys(parts).length ? { parts } : {}) };
}

/** One round-13 build: buildRound12 on the candidate's set with ROUND13.opt under `opt`. -> { d, p, failed }. */
export async function buildRound13({ cand, base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const s = round13Set(cand, set);
  const b = await buildRound12({ base, set: s, opt: { ...ROUND13.opt, ...opt }, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round13.mjs', round13_candidate: cand, basis: `${b.d.selection.basis}; round 13: candidate ${cand} (ROUND13.candidates), free-flight trim refine ${JSON.stringify(ROUND13.opt.trimFlight)}` };
  b.d.assumptions = [...b.d.assumptions.filter((x) => !/^Round 13/.test(x)),
    `Round 13: aspect ratio ${b.d.geometry.AR} at the ${b.d.geometry.span_m} m span cap, spar at ${s.sparFrac ?? b.d.params.sparFrac} of the chord; the free-flight trim refine carries a row that missed only a tolerance into the next pass and keeps converged rows only at the last (src/design/trim.js keptTrimRows).`];
  return b;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const cand = arg('--cand') ?? 'c45';
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const official = process.argv.includes('--official');
  const outPath = official ? designPath : resolve(arg('--out') ?? join(ROOT, `out/sweep/round13/${cand}.json`));
  if (!official && outPath === designPath) { console.error('data/designs/ is written only with --official (a candidate that passed every gate)'); process.exit(1); }
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound13({ cand, base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (!official && d) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  mkdirSync(dirname(outPath), { recursive: true });
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round13'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round13/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, cand, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, geometry: d.geometry, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name} ${cand}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, AR ${d.geometry.AR} S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, trims ${d.control.trim.length}, env ${env}, Vapp ${d.control.landing.Vapp_m_s}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
