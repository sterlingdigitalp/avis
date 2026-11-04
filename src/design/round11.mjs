#!/usr/bin/env node
// Round 11: round 10's pipeline (round10.mjs) for any candidate, written atomically through io.js.
//   node src/design/round11.mjs --design falco|columba|larus [--out path] [--base path] [--set '<json params>']
//   [--opt '<json options>'] [--no-perf] [--no-fit] [--cal path] [--no-suction] (keep the 0.6 default eta_s)
// Every rule round 10 applies to Larus applies here unchanged: the base (default out/sweep/round9/v18-base.json, round
// 9's) carries the rules' parameters (linkage, joints, governor, current sense, airframe) and ROUND9/ROUND10 apply on
// top as for Larus; the candidate's planform, stroke and parts come from ROUND11.candidates and --set. The model fixes
// of round 11 (the negative-stall refit, polar-fit.js; bushings on their supported length, bearings.js and
// linkage.js jointHousings) live in the physics and design code, so they apply to every build. --out other than
// data/designs/ is for exploration.
import { clearanceRecord, fullAssembly } from './clearance.js';
import { massPropsRecord, drawnLumps } from './massprops.js';
import { drivePose } from '../../sims/avis-shared/model.js';
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, CANDIDATES } from './build.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { calibrationMatches } from './round5.mjs';
import { suctionModel } from './round7.mjs';
import { ROUND9 } from './round9.mjs';
import { ROUND10_OPT, round10Params, round10Assumptions } from './round10.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';
import { cruiseEffectiveAngles } from '../physics/gates/suction-sensitivity.mjs';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Starting points per candidate (the search's own choices go in --set): Falco and Columba open from their round-2
 * planforms; everything else is round 10's rule set as it stands for Larus.
 */
export const ROUND11 = {
  candidates: {
    larus: {},
    falco: { candidate: 'falco', span_m: 1.2, AR: 5.5, taper: 0.5 },
    columba: { candidate: 'columba', span_m: 0.9, AR: 4.5, taper: 0.65 },
  },
};

export const ROUND11_OPT = { ...ROUND10_OPT };

/** A span inside the candidate's SPEC band, or an error naming the band. */
export function checkSpan(cand, span) {
  const c = CANDIDATES[cand];
  if (!c) throw new Error(`unknown candidate ${cand}`);
  const [lo, hi] = c.spanRange;
  if (!(span >= lo - 1e-9 && span <= hi + 1e-9)) throw new Error(`${c.name}: span ${span} m outside the SPEC band ${lo}-${hi} m`);
  return span;
}

/** Round 11 parameters: the candidate's starting point, then --set (parts and linkage merged), through round 10's. */
export function round11Params(base, cand, set, catalog) {
  const c = ROUND11.candidates[cand];
  if (!c) throw new Error(`unknown candidate ${cand}`);
  const s = { ...c, ...set, parts: { ...(c.parts ?? {}), ...(set.parts ?? {}) } };
  checkSpan(cand, s.span_m ?? base.params.span_m);
  return round10Params(base, s, catalog);
}

export function round11Assumptions(base, d) {
  return [...new Set([...round10Assumptions(base, d).filter((x) => !/^Round 11/.test(x)),
    'Round 11: the section model\'s negative stall is kept inside the measured UIUC sweep with its own transition width (E387 stalls at -6.3 deg at Re 1e5 and 2e5, where the tables show the drag doubling; the round-10 fit had put it at -53 to -253 deg).',
    'Round 11: a bushing is judged on the length its housing supports (structure.joints[].supports[].housing_mm): the plate thickness in an unbossed plate seat, the part\'s own length in a boss, shoulder housing or coupler eye.',
    ...((d.servos?.twist?.compliance_rad_per_Nm ?? 0) > 0 ? ['Round 18: the twist chain\'s series compliance is flown as a state relaxed toward C x twist torque with time constant TAU_CHAIN_S = 0.02 s (flight.js), not a one-step-lagged algebraic loop (unstable above about C 0.11 rad/N m at dt 1/240 s, and at any C for a small enough dt). The result does not depend on this time constant over 8-40 ms, nor on dt over 1/240-1/1920 s (test/physics/twist-compliance.test.js).'] : [])])];
}

/** The design's packing record from the film's checker (as round 10 writes it). */
function packingOf(d, fit, p, catalog) {
  const pk = packDesign(d, catalog);
  const asym = strokeAsymmetry(d, catalog);
  return { checker: 'sims/avis-shared/model.js prepare/layoutParts/fitReport (film workstream), used as is', body_m: { width: fit.body.W, height: fit.body.H, centre_y: fit.body.Y }, protrusions: pk.issues, residual_cg_shift_m: pk.cgShift_m, slides: pk.model.fit.issues.filter((i) => i.shift).map((i) => i.text), lump_positions: p.lumpPos, crank_axis_y_m: pk.model.fourBar.O2[1], left_right_stroke_difference_deg: Object.fromEntries(Object.entries(asym).map(([k, v]) => [k, +(v * 180 / Math.PI).toFixed(2)])), pin_layout: d.drive.pinLayout ?? 'symmetric (assumed)', placeholder: 'none: model.js (round 6) places no stroke servo without its lump, so the pod holds only real parts (src/design/fit.js packingView)', frame_parts: 'not placed by model.js (round 8 airframe): positions and clearances in structure.airframe' };
}

/**
 * The design's calibration record from calP (default data/calibration/<name>.json): its own V5 multipliers, or
 * provisional when fitted on another wing; null without a file.
 */
function calibrationOf(calPath, d) {
  const calP = calPath ?? join(ROOT, `data/calibration/${d.name}.json`);
  if (!existsSync(calP)) return null;
  const cal = JSON.parse(readFileSync(calP, 'utf8'));
  const same = calibrationMatches(cal, d);
  return { file: calP.slice(ROOT.length + 1), source: cal.source, multipliers: cal.multipliers, fitted_wing: cal.wing ?? null, ...(same ? {} : { provisional: true, provisional_note: 'multipliers fitted on another wing or stroke (span, area, twist weights, stroke kind); V5 must be rerun on this one' }) };
}

/** Trim, control and envelope; then again on the measured eta_s table unless noSuction. Mutates d. */
async function complete(d, opt, noSuction) {
  let r = await completeDesign(d, { opt, sched: null });
  if (!r.ok || noSuction) return r;
  const sj = JSON.parse(readFileSync(join(ROOT, 'data/calibration/suction.json'), 'utf8'));
  const amp0 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
  d.suction = suctionModel(sj, d, amp0);
  r = await completeDesign(d, { opt, sched: null });
  if (r.ok) {
    const amp1 = cruiseEffectiveAngles(d).stations.map((s) => (s.alpha_eff_amplitude_deg * DEG));
    d.suction.amplitude_change_deg_max = +Math.max(...amp1.map((a, i) => Math.abs(a - amp0[i]) / DEG)).toFixed(3);
  }
  return r;
}

/** SPEC "Assembly clearance" (2026-11-03): every part pair through the stroke, recorded with the design (passed or not). */
export function recordClearance(d, p, catalog) {
  if (p.linkage && p.joints) d.structure.clearance = clearanceRecord(d, catalog);
  outsideDrag(d);
  return d;
}

/**
 * Round 15 (SPEC "Support and containment": a fitting left outside counts in the pod's drag): the frontal area the
 * containment check finds outside the loft, at its Cd, added to the pod's frontal CdA (the pod's own kept beside it).
 */
export function outsideDrag(d) {
  const f = d.structure.clearance?.containment?.frontal;
  if (!f) return d;
  const pod = d.body.CdA_front_pod_m2 ?? d.body.CdA_front_m2;
  // round 17b (SPEC Function completeness, "the real section"): what stands out of the wing's E387 surface (the twist
  // drive's section record: the parts declared outside it, at their Cd) counts in the drag too, added at the body
  // round 18 (diagnosis 1.3/4.1, bug): sectionRecord (twist-drive.js) draws and checks the right wing only
  // (twistWing(ax, 'R', frame)), so its CdA_m2 is one wing's; both wings protrude, so it is counted twice.
  const sec = d.structure.clearance?.section, wingOut = 2 * (sec?.CdA_m2 ?? 0);
  // round 18 (SPEC "What a real build needs": the air path "with its drag counted"): the build-needs record's CdA (the
  // cooling inlet and outlet at Cd 1.0 on their area, an upper bound, and the twist-servo leads' exposed span through the
  // wing-root slots at Cd 1.2) is not in the containment record's frontal area (the holes have no solids; the leads'
  // span is declared drag false there), so it is added here.
  const needs = d.structure.clearance?.buildneeds?.CdA_m2 ?? 0;
  // round 18 (pod open problem 4): the covering's open windows (the wing-root slots, the pushrod exits, the pitot exit)
  // as cavities plus the through-flow the pod's interior allows between them (pod-structure.js openDrag: sourced
  // figures, an upper bound), also outside the containment record's frontal area.
  const cuts = d.structure.clearance?.covering?.open?.CdA_m2 ?? 0;
  d.body = { ...d.body, CdA_front_pod_m2: pod, CdA_front_outside_m2: f.CdA_m2, ...(sec ? { CdA_front_wing_protrusions_m2: +wingOut.toFixed(7), wing_protrusions_basis: 'the section record (twist-drive.js sectionRecord): each declared protrusion\'s frontal area outside the E387 at its Cd, both wings; added to the body\'s frontal CdA (conservative: Cd 1.2 on the projected height outside the section)' } : {}), ...(needs ? { CdA_front_needs_m2: +needs.toFixed(8), needs_basis: d.structure.clearance.buildneeds.CdA_basis ?? 'the build-needs record' } : {}), ...(cuts ? { CdA_front_cuts_m2: +cuts.toFixed(8), cuts_basis: d.structure.clearance.covering.open.basis } : {}), CdA_front_m2: pod + f.CdA_m2 + wingOut + needs + cuts, outside_frontal: { area_mm2: f.area_mm2, Cd: f.Cd, basis: f.basis } };
  return d;
}

/**
 * The body fit (film checker) unless fit is false, then the build with its calibration record (calPath as for
 * calibrationOf). -> { p, d } or { failed }.
 */
export function fittedBuild(p0, catalog, fit, calPath = null) {
  // round 17 (p.massFromSolids): the fit with the lumps where the rules put them (p._massPass), then every lump at its
  // drawn parts' centroid with their inertia (massprops.js), and the design built on those
  const f = fit ? fitBody(p0.massFromSolids ? { ...p0, _massPass: true } : p0, catalog) : null;
  if (f?.failed) return { failed: `body fit failed: ${f.failed} ${JSON.stringify(f.best)}` };
  let p = f ? f.params : p0;
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog);
  if (f) d.packing = packingOf(d, f, p, catalog);
  recordClearance(d, p, catalog);
  const cal = calibrationOf(calPath, d);
  if (cal) d.calibration = cal;
  return { p, d };
}

/**
 * Round 17: the build parameters with every lump at its drawn parts' centroid and their inertia (massprops.js), from
 * the design built with the lumps where the rules put them. -> params (without p._massPass).
 */
export function solidMassParams(p, catalog) {
  const { _massPass, ...q } = p;
  const d0 = buildDesign({ ...q, _massPass: true }, catalog);
  const { asm, lay } = fullAssembly(d0, catalog);
  const q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d0.drive.ampNominal);
  const { lumpPos, lumpInertia } = drawnLumps(massPropsRecord(asm, d0, catalog, q0));
  return { ...q, lumpPos: { ...(q.lumpPos ?? {}), ...lumpPos }, lumpInertia };
}

/** The records a finished round-11 design carries: launch, options, selection and assumptions. Mutates d. */
export function finishDesign(d, { cand, base, set = {}, opt }) {
  d.control.launchClimb_m_s = opt.launchClimb;
  d.launch = { ...(d.launch ?? {}), ...ROUND9.launch, ...(set.launch ?? {}), basis: 'SPEC ruling after physics round 8: the design names its release speed (at most 8 m/s); M1 is gated at it' };
  d.round8_options = opt;
  d.selection = { pipeline: 'src/design/round11.mjs', candidate: cand, basis: `explicit round-11 parameters (ROUND11.candidates.${cand}, ROUND11_OPT and --set/--opt) over round 10's rules (ROUND10 over ROUND9 on the Larus v18 base, which carries the rule parameters)`, set, opt };
  d.assumptions = round11Assumptions(base, d);
  return d;
}

/**
 * One round-11 build: { cand, base, set, opt, catalog, fit = true, calPath, noSuction, perf } -> { d, p, failed }.
 * In-process, so search scripts share it with the command line.
 */
export async function buildRound11({ cand, base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const o = { ...ROUND11_OPT, ...opt };
  const b = fittedBuild(round11Params(base, cand, set, catalog), catalog, fit, calPath);
  if (b.failed) return b;
  const r = await complete(b.d, o, noSuction);
  if (!r.ok) return { ...b, failed: r.failed };
  finishDesign(b.d, { cand, base, set, opt: o });
  if (perf) b.d.performance = measurePerformance(b.d);
  return b;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const cand = arg('--design') ?? 'larus';
  if (!CANDIDATES[cand]) { console.error(`--design must be one of ${Object.keys(CANDIDATES).join(', ')}`); process.exit(1); }
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, `data/designs/${CANDIDATES[cand].name}.json`);
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound11({ cand, base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (!official && d) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round11'), { recursive: true });
    writeFileSync(join(ROOT, `out/sweep/round11/build-${d.name}.json`), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, span ${d.geometry.span_m} S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
