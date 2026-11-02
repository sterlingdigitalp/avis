#!/usr/bin/env node
// Round 8 Larus: v17 under the SPEC rulings after physics round 7, written atomically through io.js.
//   node src/design/round8.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
//   [--no-suction] (keep the 0.6 default instead of the measured eta_s table)
// --base defaults to out/sweep/round8/v17-base.json (v17 as committed in 3416016). --out other than data/designs/ is
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
import { ROUND7, ROUND7_OPT, round7Params, round7Assumptions, suctionModel } from './round7.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';
import { cruiseEffectiveAngles } from '../physics/gates/suction-sensitivity.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/**
 * Round 8 parameter changes from v17 (docs/status/round8-levers.json):
 *  - The airframe load path (SPEC ruling after round 7; src/design/airframe.js): shoulder rails and brackets, the
 *    gearbox box (side and floor panels), the boom clamps, the forward and aft trays, the motor and encoder screws,
 *    cable ties for the carried parts, the pod's formers and the tail; the plates' mass from their solid outlines.
 *  - Aspect ratio 6 (7 before) at the same 2.0 m span: the airframe's mass (about 150 g before margin) took M1 at the
 *    worst corner to 2.2 m lost at AR 7 and 1.6 m at AR 6.5; AR 6 loses 1.19 m (out/sweep/round8, v18b-v18h).
 *  - The gearbox plates 1.6 mm aft and 1.2 mm forward (2 mm before), sized on their seats and sections.
 *  - The keel and tail boom in Rock West T-RND-393 (10 x 8 mm pultruded): its page publishes (estimated) transverse
 *    compressive and in-plane shear strengths, which the boom's clamp bearing and torsion need (EC's woven tube has none;
 *    the 8 x 6 mm T-RND-314 fails in bending at FoS 1.26 on the AR 6 wing's tail loads).
 */
export const ROUND8 = {
  AR: 6,
  parts: { keel: 'tube-rockwest-pultruded-10x8' },
  linkage: { airframe: {}, plates: { ...ROUND7.linkage.plates, t_mm: 1.6, t_fwd_mm: 1.2 } },
};

/**
 * Round-8 options: round 7's, with the TECS pitch loop weighted to height above the floor band (spdWeight 0.5;
 * control.js tecsWeights) and the hand-launch climb command at 3 m/s (control.launchClimb_m_s). Both are flight
 * controller settings: no sensor or rate changes.
 */
export const ROUND8_OPT = { ...ROUND7_OPT, spdWeight: 0.5, launchClimb: 3 };

/** Parameters: v17's through rounds 6 and 7's rules, then ROUND8, then --set (parts and linkage merged). */
export function round8Params(base, set, catalog) {
  const linkage = { ...ROUND8.linkage, ...(set.linkage ?? {}), airframe: { ...ROUND8.linkage.airframe, ...(set.linkage?.airframe ?? {}) } };
  return round7Params(base, { ...ROUND8, ...set, parts: { ...ROUND6.parts, ...ROUND8.parts, ...(set.parts ?? {}) }, linkage: { ...ROUND7.linkage, ...linkage } }, catalog);
}

export function round8Assumptions(base, d) {
  return [...new Set([...round7Assumptions(base, d).filter((x) => !/^Round 8/.test(x)),
    'Round 8: every part reaches the airframe through a rated path (structure.airframe, src/design/airframe.js): ISO 898-1 M3 class 10.9 screws at their minimum proof load (shear by von Mises on it), Araldite 2011 at half its typical lap shear on the substrate, HellermannTyton ties at their minimum loop strength; the frame parts are positioned in the design file but not packed by the film checker, and their clearances are reported.',
    'Round 8: inertial loads on carried parts are bounded by m (|f_origin| + r |wdot| + r |w|^2) from the flight model at every step (load channels sf_origin, wdot, w2); the tail by its aerodynamic force tail_F.',
    `Round 8: TECS pitch weights height-first above the floor band (spdWeight ${d.control?.tecs?.spdWeight ?? 1}); hand-launch climb command ${d.control?.launchClimb_m_s ?? 1} m/s.`])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round8/v17-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND8_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round8Params(base, set, catalog);
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
  d.round8_options = opt;
  d.selection = { pipeline: 'src/design/round8.mjs', basis: 'explicit round-8 parameters (ROUND6, ROUND7 and ROUND8, ROUND8_OPT in the scripts and --set/--opt) on Larus v17; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round8Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round8'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round8/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
