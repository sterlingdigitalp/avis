#!/usr/bin/env node
// Round 6 Larus: v14 closed against the SPEC rulings after physics round 5, written atomically through io.js.
//   node src/design/round6.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
// --base defaults to out/sweep/round6/v14-base.json (v14 as committed in 9fefeda). --out other than data/designs/ is
// for exploration: nothing else is written then except that file.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, gearRatioOf } from './build.js';
import { trainCrankRating } from './trim.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { ROUND5, ROUND5_OPT, calibrationMatches, round5Assumptions } from './round5.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);
const TI = 'metal-ti-6al-4v-annealed-bar', AL7 = 'metal-al-7075-t6', AL6 = 'metal-al-6061-t6';

/**
 * Round 6 changes from v14, each against a ruling (SPEC "Rulings after physics round 5") or a finding:
 *  - The 20 ms control delay on the crank governor and drive clamp too (flight.js / governor.js): no parameter here.
 *    The clamp stays in the flight controller: AM32's current limit (the ESC alternative) documents neither what its
 *    sensor reads (battery or phase current) nor a behaviour beyond its code: a 50-sample running mean at about 1 kHz
 *    and a duty cap that moves at most 0.5% per ms (catalogue entry of the ESC, am32_current_limit).
 *  - Every loaded part of the drive sized from a published material and in the BOM (src/design/linkage.js): a
 *    one-piece Ti-6Al-4V crank (10/7 mm shaft, 14 x 5 mm web, 8 mm pin, 15 mm keyed seat for the MSGB1-40), 7075-T6
 *    and Ti-6Al-4V gear shafts (the first, 8 mm with a 10 mm keyed seat for the SSAG1-40J10, in Ti: in 7075 it
 *    fails at half the typical yield), Ti-6Al-4V shoulder and horn pins, 7075-T6 rockers,
 *    one-piece 6061-T6 coupler links with their eyes (no bonded end fittings: no published bond strength), 6061-T6
 *    gearbox plates (bosses at the crank and first gear shaft's aft seats), 6061-T6 shoulder housings, Ti-6Al-4V keys. Carbon plate publishes no
 *    bearing strength, so no seat is in carbon.
 *  - The crank shaft in 10 mm iglidur G bushings and the coupler eyes in 8 mm ones (an 8 mm crank pin: a 6 mm one
 *    cantilevered with both couplers fails in Ti-6Al-4V).
 *  - An outboard SKF 619/5-2Z on the motor shaft in the forward plate: the motor's bearing sizes are published, their
 *    maker and positions are not.
 *  - The F405-WMO with a PDB whose shunt is not marked 3m0 (Matek notice, Dec.22, 2025).
 */
export const ROUND6 = {
  // Mass levers for the linkage's 139 g (out/sweep/round6/levers): the Li-ion 2S Molicel P30B pack (104 g, 3000 mAh,
  // 30 A continuous) for the 130 g 2200 LiPo, and KHK SSAG1-40J10 (92 g, rated 12.3 / 8.42 N m) for the stage 3
  // MSGB1-40 (100 g): its 10 mm J-series bore needs no 15 mm seat.
  parts: { battery: 'batt-nexus-2s1p-p30b', gears: ['gear-khk-ds1-18', 'gear-khk-ds1-32', 'gear-khk-ds1-20', 'gear-khk-ds1-64', 'gear-khk-msga1-18', 'gear-khk-ssag1-40j10', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'] },
  joints: { ...ROUND5.joints, crankShaft: 'bushing-igus-GFM-1012-10', couplerEyes: 'bushing-igus-GFM-0810-06', motorOutboard: 'bearing-skf-619-5-2z' },
  linkage: {
    crank: { material: TI, shaft_od_mm: 10, shaft_id_mm: 7, web_b_mm: 14, web_t_mm: 5, seat_od_mm: 15, key: { material: TI, b_mm: 5, h_mm: 5, L_mm: 10, shaft_depth_mm: 3.0 } },
    gearShafts: [
      { material: TI, d_mm: 8, seat_od_mm: 10, wheelKey: { material: TI, b_mm: 4, h_mm: 4, L_mm: 10, shaft_depth_mm: 2.5 }, pinionKey: { material: TI, b_mm: 3, h_mm: 3, L_mm: 10, shaft_depth_mm: 1.8 } },
      { material: AL7, d_mm: 8, pinionKey: { material: TI, b_mm: 3, h_mm: 3, L_mm: 10, shaft_depth_mm: 1.8 } },
      { material: AL7, d_mm: 5, mass_d_mm: 6 },
    ],
    shoulderPin: { material: TI },
    hornPin: { material: TI },
    rocker: { material: AL7, hub_od_mm: 11, arm_w_mm: 8, arm_t_mm: 5, boss_od_mm: 13, boss_len_mm: 8, spar_od_mm: 10, socket_wall_mm: 1.5, socket_len_mm: 20 },
    coupler: { material: AL6, body_d_mm: 5, eye_od_mm: 15 },
    shoulderHousing: { material: AL6, od_mm: 12 },
    plates: { material: AL6, t_mm: 2, area_m2: 2.95e-3, boss_wall_mm: 3, edge_mm: 5, bossed: ['crank shaft/aft', 'crank shaft/forward', 'gear shaft 1/aft'] },
  },
  currentSense: { ...ROUND5.currentSense, hardware: 'F405-WMO with a PDB whose shunt is NOT marked "3m0" (Matek notice Dec.22, 2025: a 3m0 PDB reads 13.2 A full scale and suits only < 15 A aircraft; its owner gets a new PDB from the dealer). The 40 A/V scale is the specification\'s 132 A sense.', relied_on_by: ['the battery current limit (control.currentLimit_A, 36 A)'], not_read_by: ['the crank governor and drive clamp (crank encoder and the FC voltage sense only)'] },
};

/**
 * Round-6 options: round 5's, with the flight step halved to 1/1920 s. At 1/960 s the left forward shoulder bushing's
 * peak (its smallest load, about 180 N) moved 13.7% at half the step (M4 seeds 17 and 6: 184 -> 151 N and 167 -> 133 N),
 * so M7's convergence rule failed there; at 1/1920 s those seeds move 6% at half the step (151 -> 160, 133 -> 141 N).
 * The step is the one at which the peaks converge (round 4's rule), not a limit.
 */
export const ROUND6_OPT = { ...ROUND5_OPT, dt: 1 / 1920 };

/** Parameters: v14's, then ROUND6, then --set; joints and linkage merged; gear ratio and crank rating from the train. */
export function round6Params(base, set, catalog) {
  const p = { ...base.params, ...ROUND6, ...set, joints: { ...ROUND6.joints, ...(set.joints ?? {}) }, linkage: set.linkage === null ? undefined : { ...ROUND6.linkage, ...(set.linkage ?? {}) }, parts: { ...base.params.parts, ...ROUND6.parts, ...(set.parts ?? {}) } };
  if (!p.linkage) delete p.linkage;
  p.gearRatio = gearRatioOf(p.parts.gears, catalog);
  // round 17: the gear stages only (p._massPass: with p.massFromSolids the lumps are not drawn yet at this point)
  p.crankRating = trainCrankRating(buildDesign({ ...p, _massPass: true }, catalog).structure.gear_stages);
  return p;
}

/** The design's assumptions for round 6: round 5's, plus round 6's own. */
export function round6Assumptions(base, d) {
  const gone = /^Round 6/;
  const a = round5Assumptions({ assumptions: (base.assumptions ?? []).filter((x) => !gone.test(x)) }, d).map((x) => x.replace(/ The motor's own bearings carry the first pinion and are reported, not checked \(no published rating\)\. Shafts, pins and the plates' bearing seats are not checked\./, ' Round 6: the first pinion on an outboard bearing (structure.joints, motor shaft outboard); shafts, pins, seats, rockers, coupler links and keys in structure.members.'));
  return [...new Set([...a,
    'Round 6: the 20 ms control delay applies to the crank governor and drive clamp as to attitude control: the duty the ESC applies was computed ceil(0.02 x rate) controller periods earlier (20.8 ms at 960 Hz), and the twist schedule takes its crank phase from that same tick (the encoder sample then, advanced at the speed measured then). The stroke-frequency command reaching the governor has already passed the attitude controller\'s 20 ms delay, so that path sees 40 ms (conservative).',
    'Round 6: every loaded part of the drive is sized from a catalogue metal with published strength (structure.members, src/design/linkage.js): forces summed as magnitudes (directions not credited); shaft moments bounded at supports and load planes; pins judged at their roots, bosses by the rigid-pin edge pressure; shoulder pins clamped by the rocker hub between their bushings; seats without a published bearing strength judged at the tensile yield; aluminium typical strengths (AA, NOT FOR DESIGN) at half value; stress concentrations at keyseats and shoulders not modelled (static, ductile).',
    ...(d.structure.joints_unchecked?.[0]?.lever ? [`Round 6: an outboard bearing on the motor shaft in the forward plate carries the first pinion with the motor bearing nearest it as the second support no closer than ${(d.structure.joints_unchecked[0].lever.s_min_m * 1000).toFixed(1)} mm (motor bearing positions not published): rated at F (a + s) / s.`] : []),
    'Round 6: the F405-WMO is specified with a PDB whose shunt is not marked 3m0 (Matek notice, Dec.22, 2025); the battery current limit reads current through it at the specification\'s 40 A/V.'])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round6/v14-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND6_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round6Params(base, set, catalog);
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
  const r = await completeDesign(d, { opt, sched: arg('--sched') ? JSON.parse(arg('--sched')) : null });
  if (!r.ok) {
    console.error(`build failed: ${r.failed}`);
    if (!official) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  d.round6_options = opt;
  d.selection = { pipeline: 'src/design/round6.mjs', basis: 'explicit round-6 parameters (ROUND6, ROUND6_OPT in the script and --set/--opt) on Larus v14; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round6Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round6'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round6/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
