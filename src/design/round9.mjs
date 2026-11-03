#!/usr/bin/env node
// Round 9 Larus: v18 under the SPEC rulings after physics round 8, written atomically through io.js.
//   node src/design/round9.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
//   [--no-suction] (keep the 0.6 default instead of the measured eta_s table)
// --base defaults to out/sweep/round9/v18-base.json (v18 as committed in c0c555e). --out other than data/designs/ is
// for exploration: nothing else is written then except that file.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign } from './build.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { calibrationMatches } from './round5.mjs';
import { ROUND6 } from './round6.mjs';
import { suctionModel } from './round7.mjs';
import { ROUND8, ROUND8_OPT, round8Params, round8Assumptions } from './round8.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';
import { cruiseEffectiveAngles } from '../physics/gates/suction-sensitivity.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/**
 * Round 9 parameter changes from v18 (docs/status/round9-levers.json):
 *  - Bearing below the source's edge distance (SPEC ruling after round 8; members.js bearingAllowable): every lug and
 *    pin seat reaches e >= 1.2 d0 (EN 1993-1-8 / EN 1999-1-1 minimum), with the ring sizes rounded up: rocker hub 14.5
 *    and horn boss 19.5 mm, coupler eyes 24.5 mm, shoulder housings 19.5 mm, plate seat rings 9.2 mm beyond each part
 *    and bosses 8.5 mm wall, rail seats 29 mm.
 *  - The shoulder rails in 12/9.6 mm 7075-T6 tube (12/10 before: FoS 1.47 on the right rail in round 8).
 *  - The gear ratings under KHK's stated conditions (gearing.js, physics/gears.js), at a stated rating crank frequency
 *    of 3.0 Hz (the envelope's top is below it; M7 checks the flown peak), reversing load, Shell Omala S2 GX 220 on the
 *    steel stages and Molykote EM-30L grease on the Duracon stages, the gearbox at most 40 C.
 *  - The named hand-release speed, 8 m/s (SPEC ruling after round 8: M1 is gated at it).
 *  - The gear train: v18's (DS1-18/32, DS1-20/64, MSGA1-18/SSAG1-40J10, MSGA1-18/MSGB1-40) rates 7.34 N m at the crank
 *    under those conditions (14.16 from the catalogue figures), and no stroke trims within it. The lightest catalogue
 *    train found that rates 15 N m (S/r9/trainsearch.mjs, every module-1 KHK gear in the catalogue, 4 stages, ratio
 *    24-32): DSF1-20/DS1-30, MSGA1-18/DS1-60, MSGA1-18/MSGB1-45 twice, ratio 31.25, 356 g of gears (262 before).
 */
export const ROUND9 = {
  linkage: {
    plates: { ...ROUND8.linkage.plates, edge_mm: 9.2, boss_wall_mm: 8.5 },
    rocker: { hub_od_mm: 14.5, boss_od_mm: 19.5 },
    coupler: { eye_od_mm: 24.5 },
    shoulderHousing: { od_mm: 19.5 },
    airframe: { plates: { edge_mm: 9.2 }, rail: { seat_D_mm: 29, id_mm: 9.6 } },
  },
  gearRating: {
    crank_Hz: 3.0,
    reversing: true,
    metal_lubricant: { id: 'lubricant-shell-omala-s2-gx-220', cSt_50C: 127, basis: 'ASTM D341 from the published 220 / 19.4 cSt at 40 / 100 C (no ZL credit taken)' },
    plastic_lubricant: { id: 'grease-molykote-em-30l', grease: true },
    max_temperature_C: 40,
  },
  launch: { release_speed_m_s: 8 },
  parts: { gears: ['gear-khk-dsf1-20', 'gear-khk-ds1-30', 'gear-khk-msga1-18', 'gear-khk-ds1-60', 'gear-khk-msga1-18', 'gear-khk-msgb1-45', 'gear-khk-msga1-18', 'gear-khk-msgb1-45'] },
};

export const ROUND9_OPT = { ...ROUND8_OPT };

/** Parameters: v18's, with ROUND9 merged into the linkage blocks it names, then --set. */
const merged = (...xs) => Object.assign({}, ...xs);

export function round9Params(base, set, catalog) {
  const L0 = base.params.linkage ?? {}, S = set.linkage ?? {}, R = ROUND9.linkage;
  const A0 = L0.airframe ?? {}, AS = S.airframe ?? {};
  const block = (k) => merged(L0[k], R[k], S[k]);
  const airframe = merged(A0, R.airframe, AS, { plates: merged(A0.plates, R.airframe.plates, AS.plates), rail: merged(A0.rail, R.airframe.rail, AS.rail) });
  const linkage = merged(L0, S, { plates: block('plates'), rocker: block('rocker'), coupler: block('coupler'), shoulderHousing: block('shoulderHousing'), airframe });
  return round8Params(base, { gearRating: ROUND9.gearRating, ...set, parts: merged(ROUND9.parts, set.parts), linkage }, catalog);
}

export function round9Assumptions(base, d) {
  return [...new Set([...round8Assumptions(base, d).filter((x) => !/^Round 9/.test(x)),
    'Round 9: pin-in-hole bearing below the source\'s edge distance is the lesser of that source at half and EN 1999-1-1 / EN 1993-1-8 bolt bearing (k1 alpha_b fu / 1.25, specified-minimum fu) at the seat\'s own distances, every seat at e >= 1.2 d0; rings are judged with e = D / 2 in every direction. EN 1999-1-1 does not list EN AW-7075: its expression is applied with 7075-T6\'s specified minimum fu.',
    `Round 9: gear ratings are KHK's catalogue values corrected to the design's conditions (structure.gear_stages[].rating_check): mating gear by JGMA 402-01 d01 i/(i+1) and KH-beta (the "weak shaft" column wherever a gear is not at mid-span: KHK states no criterion for "rugged"), JGMA 401-01 Y-epsilon; speed by the KV band and ZV (below 1 m/s the curve's first value, as KHK's own 600 rpm ratings need); the oil at or above 100 cSt (50 C) and over 1e7 cycles with no credit; Duracon by KHK's plastic method (KV, KM 0.75 Duracon on Duracon) and JGMA 401's 2/3 for the reversing load. KHK states the JGMA method for module 1.5 and up; these gears are module 1, as KHK's own ratings are. Rating crank frequency ${d.drive?.gear_rating_conditions?.crank_Hz ?? '-'} Hz.`,
    `Round 9: hand release at ${d.launch?.release_speed_m_s ?? 8} m/s, the design's named speed (SPEC ruling after round 8).`])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND9_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round9Params(base, set, catalog);
  const fit = process.argv.includes('--no-fit') ? null : fitBody(p0, catalog);
  if (fit?.failed) { console.error(`body fit failed: ${fit.failed} ${JSON.stringify(fit.best)}`); process.exit(1); }
  const p = fit ? fit.params : p0;
  const d = buildDesign(p, catalog);
  if (fit) {
    const pk = packDesign(d, catalog);
    const asym = strokeAsymmetry(d, catalog);
    d.packing = { checker: 'sims/avis-shared/model.js prepare/layoutParts/fitReport (film workstream), used as is', body_m: { width: fit.body.W, height: fit.body.H, centre_y: fit.body.Y }, protrusions: pk.issues, residual_cg_shift_m: pk.cgShift_m, slides: pk.model.fit.issues.filter((i) => i.shift).map((i) => i.text), lump_positions: p.lumpPos, crank_axis_y_m: pk.model.fourBar.O2[1], left_right_stroke_difference_deg: Object.fromEntries(Object.entries(asym).map(([k, v]) => [k, +(v * 180 / Math.PI).toFixed(2)])), pin_layout: d.drive.pinLayout ?? 'symmetric (assumed)', placeholder: 'none: model.js (round 6) places no stroke servo without its lump, so the pod holds only real parts (src/design/fit.js packingView)', frame_parts: 'not placed by model.js (round 8 airframe): positions and clearances in structure.airframe' };
  }
  const calP = arg('--cal') ? resolve(arg('--cal')) : join(ROOT, `data/calibration/${d.name}.json`);
  if (existsSync(calP)) {
    const cal = JSON.parse(readFileSync(calP, 'utf8'));
    const same = calibrationMatches(cal, d);
    d.calibration = { file: calP.slice(ROOT.length + 1), source: cal.source, multipliers: cal.multipliers, fitted_wing: cal.wing ?? null, ...(same ? {} : { provisional: true, provisional_note: 'multipliers fitted on another wing or stroke (span, area, twist weights, stroke kind); V5 must be rerun on this one' }) };
  }
  const sched = arg('--sched') ? JSON.parse(arg('--sched')) : null;
  let r = await completeDesign(d, { opt, sched });
  if (r.ok && !process.argv.includes('--no-suction')) {
    const sj = JSON.parse(readFileSync(join(ROOT, 'data/calibration/suction.json'), 'utf8'));
    const amp0 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
    d.suction = suctionModel(sj, d, amp0);
    r = await completeDesign(d, { opt, sched });
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
  d.control.launchClimb_m_s = opt.launchClimb;
  d.launch = { ...(d.launch ?? {}), ...ROUND9.launch, ...(set.launch ?? {}), basis: 'SPEC ruling after physics round 8: the design names its release speed (at most 8 m/s); M1 is gated at it' };
  d.round8_options = opt;
  d.selection = { pipeline: 'src/design/round9.mjs', basis: 'explicit round-9 parameters (ROUND9, ROUND9_OPT and --set/--opt) on Larus v18 through rounds 6-8\'s rules; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round9Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round9'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round9/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
