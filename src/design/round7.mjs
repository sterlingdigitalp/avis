#!/usr/bin/env node
// Round 7 Larus: v16 under the SPEC rulings after physics round 6, written atomically through io.js.
//   node src/design/round7.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
//   [--no-suction] (keep the 0.6 default instead of the measured eta_s table)
// --base defaults to out/sweep/round7/v16-base.json (v16 as committed in 020ce9e).
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, gearRatioOf } from './build.js';
import { trainCrankRating } from './trim.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { calibrationMatches } from './round5.mjs';
import { ROUND6, ROUND6_OPT, round6Params, round6Assumptions } from './round6.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';
import { suctionTableFrom } from '../physics/strip.js';
import { cruiseEffectiveAngles } from '../physics/gates/suction-sensitivity.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

const TI = 'metal-ti-6al-4v-annealed-bar', AL7 = 'metal-al-7075-t6', AL6 = 'metal-al-6061-t6';
const L6 = ROUND6.linkage;
/**
 * Round 7 parameter changes from v16 (docs/status/round7-levers.json). With the aluminium at its specified minimums
 * (6061-T6 240 MPa, 7075-T6 485 MPa, EN 754-2 / 755-2 / 485-2) instead of half the typical values, parts are re-sized
 * against v16's flown loads: the first gear shaft and the shoulder and horn pins go from Ti-6Al-4V to 7075-T6, the
 * coupler body from 5 to 4.5 mm with 13 mm eyes, the shoulder housings to 11 mm, the plate bosses to a 2 mm wall. The
 * rocker's horn arm, now checked in bending (it failed at 8 mm wide, FoS 0.99), widens to 11 mm.
 */
export const ROUND7 = {
  linkage: {
    gearShafts: [{ ...L6.gearShafts[0], material: AL7 }, L6.gearShafts[1], L6.gearShafts[2]],
    shoulderPin: { material: AL7 },
    hornPin: { material: AL7 },
    rocker: { ...L6.rocker, arm_w_mm: 11 },
    coupler: { ...L6.coupler, body_d_mm: 4.5, eye_od_mm: 13 },
    shoulderHousing: { ...L6.shoulderHousing, od_mm: 11 },
    plates: { ...L6.plates, boss_wall_mm: 2 },
  },
};
void TI; void AL6;

/**
 * Round-7 options: round 6's, with the flight controller at 240 Hz. The half-step check needs every input to be the
 * same function of time at both steps (SPEC ruling after round 6): 240 Hz ticks at the same instants at 1/1920 and
 * 1/3840 s (100 Hz does not divide either), and its 20 ms delay is 5 periods (20.8 ms).
 */
export const ROUND7_OPT = { ...ROUND6_OPT, ctlRate_Hz: 240 };

/** Parameters: v16's through round 6's rules, then ROUND7, then --set. */
export function round7Params(base, set, catalog) {
  return round6Params(base, { ...ROUND7, ...set, ...(ROUND7.linkage || set.linkage ? { linkage: { ...ROUND6.linkage, ...(ROUND7.linkage ?? {}), ...(set.linkage ?? {}) } } : {}) }, catalog);
}

/**
 * The measured eta_s table (data/calibration/suction.json, the cfd workstream's, read only): every complete point,
 * none chosen or dropped (a complete point without an effective-angle amplitude stops the build), linear in the
 * amplitude, held at the end values (strip.js suctionEtaAt). The per-strip amplitudes are the design's own cruise
 * effective-angle amplitudes (suction-sensitivity.mjs cruiseEffectiveAngles).
 */
export function suctionModel(json, design, amplitudes) {
  const t = suctionTableFrom(json);
  const unplaced = t.skipped.filter((s) => !/incomplete|unsettled|no settled verdict/.test(s.why));
  if (unplaced.length) throw new Error(`suction points without an effective-angle amplitude: ${unplaced.map((s) => s.name).join(', ')}`);
  if (!t.table.length) throw new Error('no measured eta_s point');
  return { source: 'data/calibration/suction.json', points_used: t.used, points_incomplete: t.skipped.map((s) => s.name), points_skipped: t.skipped, table: t.table, amplitude_rad: amplitudes, rule: 'SPEC ruling after cfd round 3: settled points only; linear in effective-angle amplitude between them; held below the smallest; beyond the largest the last slope continued where it lowers eta_s, held where it would raise it; SPEC eta_s default 0.6 not used' };
}

export function round7Assumptions(base, d) {
  return [...new Set([...round6Assumptions(base, d).filter((x) => !/^Round 7/.test(x)),
    'Round 7: the flight controller runs at 240 Hz and ticks at fixed instants; the M4 air is convected at the rough-air speed (Dryden filters, rotary terms and gust durations), so every input is the same function of time at both steps of the half-step check (out/verification/evidence/M7-inputs-<design>.json).',
    ...(d.suction ? [`Round 7: leading-edge suction eta_s per strip from the measured table (${d.suction.points_used.length} points, ${d.suction.source}) at the design's own cruise effective-angle amplitudes; the 0.6 default is not used.`] : [])])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round7/v16-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND7_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round7Params(base, set, catalog);
  const fit = process.argv.includes('--no-fit') ? null : fitBody(p0, catalog);
  if (fit?.failed) { console.error(`body fit failed: ${fit.failed} ${JSON.stringify(fit.best)}`); process.exit(1); }
  const p = fit ? fit.params : p0;
  const d = buildDesign(p, catalog);
  if (fit) {
    const pk = packDesign(d, catalog);
    const asym = strokeAsymmetry(d, catalog);
    d.packing = { checker: 'sims/avis-shared/model.js prepare/layoutParts/fitReport (film workstream), used as is', body_m: { width: fit.body.W, height: fit.body.H, centre_y: fit.body.Y }, protrusions: pk.issues, residual_cg_shift_m: pk.cgShift_m, slides: pk.model.fit.issues.filter((i) => i.shift).map((i) => i.text), lump_positions: p.lumpPos, crank_axis_y_m: pk.model.fourBar.O2[1], left_right_stroke_difference_deg: Object.fromEntries(Object.entries(asym).map(([k, v]) => [k, +(v * 180 / Math.PI).toFixed(2)])), pin_layout: d.drive.pinLayout ?? 'symmetric (assumed)', placeholder: 'none: model.js (round 6) places no stroke servo without its lump, so the pod holds only real parts (src/design/fit.js packingView)' };
  }
  const calP = arg('--cal') ? resolve(arg('--cal')) : join(ROOT, `data/calibration/${d.name}.json`);
  if (existsSync(calP)) {
    const cal = JSON.parse(readFileSync(calP, 'utf8'));
    const same = calibrationMatches(cal, d);
    d.calibration = { file: calP.slice(ROOT.length + 1), source: cal.source, multipliers: cal.multipliers, fitted_wing: cal.wing ?? null, ...(same ? {} : { provisional: true, provisional_note: 'multipliers fitted on another wing or stroke (span, area, twist weights, stroke kind); V5 must be rerun on this one' }) };
  }
  let r = await completeDesign(d, { opt, sched: arg('--sched') ? JSON.parse(arg('--sched')) : null });
  // Round 7: the measured eta_s table at the design's own cruise effective-angle amplitudes, then the trims again with
  // it in place; the amplitudes are refreshed once more and their change recorded.
  if (r.ok && !process.argv.includes('--no-suction')) {
    const sj = JSON.parse(readFileSync(join(ROOT, 'data/calibration/suction.json'), 'utf8'));
    const amp0 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
    d.suction = suctionModel(sj, d, amp0);
    r = await completeDesign(d, { opt, sched: arg('--sched') ? JSON.parse(arg('--sched')) : null });
    if (r.ok) {
      const amp1 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
      d.suction.amplitude_change_deg_max = +Math.max(...amp1.map((a, i) => Math.abs(a - amp0[i]) / DEG)).toFixed(3);
    }
  }
  if (!r.ok) {
    console.error(`build failed: ${r.failed}`);
    if (!official) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  d.round7_options = opt;
  d.selection = { pipeline: 'src/design/round7.mjs', basis: 'explicit round-7 parameters (ROUND6 and ROUND7, ROUND7_OPT in the scripts and --set/--opt) on Larus v16; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round7Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round7'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round7/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
