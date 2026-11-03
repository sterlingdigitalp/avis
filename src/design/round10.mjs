#!/usr/bin/env node
// Round 10 Larus: round 9's pipeline (round9.mjs) under the orchestrator's round-10 rulings and the SPEC ruling "RC
// practice for joints no one can rate", written atomically through io.js.
//   node src/design/round10.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
//   [--no-suction] (keep the 0.6 default instead of the measured eta_s table)
// --base defaults to out/sweep/round9/v18-base.json, round 9's base, so ROUND9 and then ROUND10 apply on top of v18 as
// round 9 did. --out other than data/designs/ is for exploration.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign } from './build.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { calibrationMatches } from './round5.mjs';
import { suctionModel } from './round7.mjs';
import { ROUND9, ROUND9_OPT, round9Params, round9Assumptions } from './round9.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';
import { byId } from './catalog.js';
import { cruiseEffectiveAngles } from '../physics/gates/suction-sensitivity.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/** JIS B 1301 parallel-key shaft keyseat depth t1 (mm) by key width b (mm). */
export const KEY_T1 = { 2: 1.2, 3: 1.8, 4: 2.5, 5: 3.0, 6: 3.5 };

/** The key a maker keyway takes ("keyway 5 x 2.3 mm"): b x b, the shaft depth by JIS B 1301; null without a keyway. */
export function keyFor(part, material, L_mm = 10) {
  const m = /^keyway\s+(\d+(?:\.\d+)?)\s*x/.exec(part?.specs?.fastening ?? '');
  if (!m) return null;
  const b = +m[1];
  if (!Number.isFinite(KEY_T1[b])) throw new Error(`${part.id}: no JIS B 1301 keyseat depth for a ${b} mm key`);
  return { material, b_mm: b, h_mm: b, L_mm, shaft_depth_mm: KEY_T1[b] };
}

/**
 * A plain bushing or bearing for a shaft of d mm. The small shafts nearest the motor turn too fast for an iglidur G
 * bushing's surface speed (a 6 mm shaft at about 3000 rpm is near 1 m/s), so they run in ball bearings.
 */
export const SHAFT_SUPPORT = { 5: 'bearing-skf-619-5-2z', 6: 'bearing-skf-628-6-2z', 8: 'bushing-igus-GFM-0810-06', 10: 'bushing-igus-GFM-1012-10' };

/**
 * Round 10: the gear shafts and the crank wheel seat follow the train (round 9 kept v18's shaft 1, a 10 mm seat and
 * 4 mm key, under v19's MSGB1-45, whose bore is 15 mm with a 5 mm keyway). Shaft j (crank side first) carries the
 * wheel of stage n-1-j and the pinion of stage n-j: checked at the smaller bore, the larger as the wheel seat (or as the
 * mass diameter when the pinion's bore is the larger), each maker keyway keyed, plain bores fixed per gearFixing.
 * gears: [p1, g1, p2, g2, ...] (motor side first).
 */
export function trainShafts(gears, catalog, { shaftMaterial = 'metal-al-7075-t6', keyMaterial = 'metal-ti-6al-4v-annealed-bar' } = {}) {
  const n = Math.floor(gears.length / 2);
  const st = Array.from({ length: n }, (_, k) => ({ pinion: byId(catalog, gears[2 * k]), gear: byId(catalog, gears[2 * k + 1]) }));
  const bore = (p) => +p.specs.bore_mm;
  const shafts = [], supports = [];
  for (let j = 1; j < n; j++) {
    const wheel = st[n - 1 - j].gear, pin = st[n - j].pinion;
    const wb = bore(wheel), pb = bore(pin);
    const d = Math.min(wb, pb);
    const s = { material: shaftMaterial, d_mm: d };
    if (wb > d) s.seat_od_mm = wb;
    if (pb > d) s.mass_d_mm = pb;
    const wk = keyFor(wheel, keyMaterial, Math.min(10, wheel.specs.face_width_mm ?? 10));
    const pk = keyFor(pin, keyMaterial, Math.min(10, pin.specs.face_width_mm ?? 10));
    if (wk) s.wheelKey = wk;
    if (pk) s.pinionKey = pk;
    shafts.push(s);
    if (!SHAFT_SUPPORT[d]) throw new Error(`no shaft support listed for a ${d} mm shaft`);
    supports.push(SHAFT_SUPPORT[d]);
  }
  const crankWheel = st[n - 1].gear;
  return { gearShafts: shafts, jointShafts: supports, crankSeat_mm: bore(crankWheel), crankKey: keyFor(crankWheel, keyMaterial, Math.min(10, crankWheel.specs.face_width_mm ?? 10)), motorPinion: st[0].pinion.id, motorPinionBore_mm: bore(st[0].pinion) };
}

/**
 * Round 10 changes on top of ROUND9 (docs/status/round10-levers.json):
 *  - gear ratings: JGMA's KV at the actual speed on the gear's Table 10.3 row (N5 = old JIS grade 1, N7 = 3, N8 = 4,
 *    KHK's Japanese reference Table 8.4 and its "new = old + 4" rule; no tooth modification stated: unmodified rows),
 *    no longer half for a higher band; KH-beta stays on the "weak shaft" column (KHK's Japanese table gives no
 *    criterion for "軸のこわさ大/小", research/round10-parts.json Q3).
 *  - fixings (SPEC ruling "RC practice for joints no one can rate"): plain-bore acetal gears bonded with LOCTITE 406
 *    on SF 770 primer, within Henkel's published acetal block shear (14 MPa); plain-bore steel with LOCTITE 638 within
 *    its published 29 MPa; KHK F-Loc hubs within KHK's reference slipping torque. All reported as unrated, RC practice.
 *  - 7075-T6 seats: EN 1999-1-1 does not list the alloy, so its bolt-bearing result counts at half (members.js).
 *  - the flight step 1/3840 s (item 5: the airframe channel w2 converges there, 2.9% at half that step).
 */
export const ROUND10 = {
  gearRating: {
    ...ROUND9.gearRating,
    accuracy: { old_grade_of: { N5: 1, N7: 3, N8: 4 }, modified: false, basis: 'KHK Japanese technical reference khk416_1 Table 8.4 groups "N5,N6 (old JIS 1,2)", "N7,N8 (old JIS 3,4)"; khk472: new grade ~ old + 4 (rule of thumb). MSGA/MSGB: "Standard full depth", no tooth modification stated: unmodified rows (scratchpad research/round10-parts.json Q2).' },
  },
  fixings: { bond: { pom: 'adhesive-loctite-406-sf770', steel: 'adhesive-loctite-638' } },
};

export const ROUND10_OPT = { ...ROUND9_OPT, dt: 1 / 3840 };

/** Round 10 parameters: --set's train and stroke over ROUND10, shafts and joints derived from the train, then round 9's. */
export function round10Params(base, set, catalog) {
  const gears = set.parts?.gears ?? ROUND9.parts.gears;
  const tr = trainShafts(gears, catalog);
  const crank = { ...(base.params.linkage?.crank ?? {}), seat_od_mm: tr.crankSeat_mm, ...(tr.crankKey ? { key: tr.crankKey } : {}) };
  const linkage = { ...(set.linkage ?? {}), fixings: ROUND10.fixings, gearShafts: tr.gearShafts, crank: { ...crank, ...(set.linkage?.crank ?? {}) } };
  const joints = { ...(base.params.joints ?? {}), shafts: tr.jointShafts, ...(set.joints ?? {}) };
  return round9Params(base, { ...set, gearRating: { ...ROUND10.gearRating, ...(set.gearRating ?? {}) }, parts: { ...(set.parts ?? {}), gears }, linkage, joints }, catalog);
}

export function round10Assumptions(base, d) {
  const a = round9Assumptions(base, d).filter((x) => !/^Round 10/.test(x)).map((x) => x.replace(/ speed by the KV band and ZV \(below 1 m\/s the curve's first value, as KHK's own 600 rpm ratings need\)/, ' speed by JGMA\'s KV on the gear\'s accuracy row at both speeds (round 10) and ZV (below 1 m/s the curve\'s first value, as KHK\'s own 600 rpm ratings need)'));
  return [...new Set([...a,
    'Round 10: gear accuracy rows for JGMA KV: N5 as old JIS grade 1, N7 as 3, N8 as 4 (KHK Japanese reference Table 8.4), unmodified teeth (no modification stated); the KV ratio is applied at the actual speed and capped at 1 (no credit for running slower than rated).',
    'Round 10: shaft-to-hub fixings without a published rating are standard RC practice (SPEC ruling): plain-bore acetal gears bonded with LOCTITE 406 on SF 770 primer, plain-bore steel with LOCTITE 638, F-Loc hubs at KHK\'s standard screw torque; each within the maker\'s published figure at full value, reported in M7 as unrated, RC practice.',
    'Round 10: 7075-T6 seats take EN 1999-1-1\'s bolt-bearing expression at half (the standard does not list the alloy).',
    `Round 10: flight step ${(1 / (d.control?.dt ?? 1 / 3840)).toFixed(0)} Hz: the airframe channel w2 moved 10.8% at half of 1/1920 s and 2.9% at half of 1/3840 s; the flight model's aerodynamic internal states (the lifting-line circulation, one frozen-Jacobian Newton step per flight step, and the strip lag states) advance once per step, so it converges at first order in the step.`])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND10_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round10Params(base, set, catalog);
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
  let r = await completeDesign(d, { opt, sched: null });
  if (r.ok && !process.argv.includes('--no-suction')) {
    const sj = JSON.parse(readFileSync(join(ROOT, 'data/calibration/suction.json'), 'utf8'));
    const amp0 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
    d.suction = suctionModel(sj, d, amp0);
    r = await completeDesign(d, { opt, sched: null });
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
  d.selection = { pipeline: 'src/design/round10.mjs', basis: 'explicit round-10 parameters (ROUND10, ROUND10_OPT and --set/--opt) over round 9\'s (ROUND9 on Larus v18); screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round10Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round10'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round10/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
