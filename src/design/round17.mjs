#!/usr/bin/env node
// Round 17 Larus: v25 = v24 plus function completeness (SPEC "Design freeze", "Function completeness", 2026-11-03).
// Round 16's pipeline (round16.mjs) with: every lump at its drawn parts' centroid with their inertia
// (src/design/massprops.js), ...
//   node src/design/round17.mjs [--out path] [--set '<json>'] [--opt '<json>'] [--cal path] [--no-suction] [--no-fit]
//   [--no-perf] [--official]
// Exploration builds go to out/sweep/round17/v25.json. data/designs/larus-gl-2.json is written only with --official,
// and only when every round-16 check passes and every round-17 record passes. There is no override.
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound16, round16Set, officialRefusal as refusal16, mergeDeep, SEQUENCE as SEQUENCE16, CONTAINMENT as CONTAINMENT16 } from './round16.mjs';
import { RETENTION, RETENTION_SEQUENCE, retentionContainment } from './round17-retention.mjs';
import { TWIST_SEQUENCE, TWIST_CONTAINMENT, TWIST_SECTION } from './twist-drive.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/** Round 17's corrections over v24 (round16Set), each forced by a function-completeness check. */
export const ROUND17 = {
  // mass properties from the drawn solids: every lump where its parts are drawn, the battery included (its mount no
  // longer placed by its lump: right after the airspeed board, where v24's search put it)
  massFromSolids: true,
  // the nose where v24's pod put it (build.js placed it 50 mm ahead of the CG-placed battery): stated, since the battery
  // is now where it is drawn
  noseX_m: 0.25781312347594765,
  // the battery's place on the nose tray stated (v24 searched for it from its CG-placed lump and found the airspeed
  // board's aft end, x 156.5 mm): with mass from solids its lump is where it is drawn, so it no longer places it
  mounts: { battery: { x: null, x0_m: 0.1565 } },
  // the crank web (audit item 13 and its axial fixing): a 20 mm hub boss round the bore (the wall behind the keyway
  // 3.5 mm), and an end lip 1.7 mm thick over the stub's end holding the web axially by an ISO 10642 M3 x 8 into the
  // stub's tapped end (the shaft solid 2.5 mm on beyond the web for its thread), its countersunk head flush (the coupler eyes run 0.5 mm aft of the web, so no head may stand
  // there; ISO 898-1 at half for the countersunk head); the LOCTITE 638 bond, unrated on titanium, is gone. The key over
  // the rest of the web, 4.3 mm, its keyseat 1.5 mm into the stub so the web's keyway is 1.5 mm deep (machined to these
  // depths; the key rule rates them)
  crank: { web: { hub_od_mm: 20, lip_mm: 1.7, axial: { screw: 'screw-iso10642-m3x8-10.9', countersunk: true, clearance_mm: 0.2, solid_mm: 2.5 }, key: { L_mm: 4.3, shaft_depth_mm: 1.5 } } },
  // the servos (orchestrator, 2026-11-03; audit item 15): the twist servo's horn and spline are unsourced (MKS pages
  // never loaded), so its chain cannot be drawn from catalogue parts: the twist servo is the Hitec MD89MW (its R-ML25
  // arm is sourced); both servos on the Pololu D36V50F6 (6 V +-4%, inside the MD89MW's 4.8-7.4 V rows; the F7's
  // 7.2-7.8 V was above the 7.4 V row)
  parts: { twistServo: 'servo-hitec-md89mw', servoRegulator: 'regulator-pololu-d36v50f6' },
  // the all-moving V-tail (audit items 2, 11): pivots, saddle, horns and pushrods (src/design/tail-mech.js TAIL_MECH);
  // the pushrod's length as drawn (the completeness check holds the BOM to the drawn rod)
  tailMech: { pushrod_L_mm: 893.36 },
  // the boom to the saddle's aft end (the sleeve 8 mm aft of the tail's quarter chord)
  boom: { to_tail_spar_mm: 8 },
  // the access hatch's window in the pod's covering (audit item 6): over the pack, on the top (polar angle as the
  // containment record measures it: 0 deg at +z, 90 at the top). The pod's structure (pod-structure.js) leaves this
  // window open; the hatch (build-needs.js) frames and covers it, and its fixing holds it
  hatch: { x_mm: [150, 238], polar_deg: [40, 140] },
  // the wing twist drive (audit items 1, 3, 10, 12; src/design/twist-drive.js TWIST_DRIVE): the spars coaxial 5.5 mm above
  // the E387's chord line, the joiner, the servo in its frame and its four-bar to the torque rod's lever, the bar and the
  // tip; the parts that stand out of the section declared with their drag (TWIST_SECTION)
  twistDrive: { section: TWIST_SECTION },
};

const X = (s) => [s, 0, 0], Y = (s) => [0, s, 0], Z = (s) => [0, 0, s];
const step = (name, members, dir, dist_m = 0.08, why = undefined) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
/**
 * Round 17's assembly sequence: round 16's, with the new parts' steps inserted after the step named in `after` (and
 * any round-16 step replaced where `replace` names it).
 */
export const SEQUENCE_INSERTS = [
  // round 17b: each horn pin with its new groove and tip (round17-retention.mjs: the pin runs on past its eye for the washer)
  ...['R', 'L'].map((t) => ({ replace: `horn pin ${t}`, steps: [step(`horn pin ${t}`, [`^rocker ${t}/horn pin( groove| tip)?$`], X(-1), 0.04)] })),
  { after: 'crank web with its key', steps: [step('crank web axial screw', ['^crank web axial screw$'], X(-1), 0.03, 'along the crank axis from aft into the stub\'s tapped end, its countersunk head into the web\'s lip')] },
];
/**
 * The all-moving tail's steps (tail-mech.js), in place of round 16's tied tail: the saddle with its four bushings on
 * the boom's end; each surface (its pivot tube, ribs, trailing edge rod, covering and the horn on its pivot pin, bonded
 * first) along its pivot axis from outboard; the pin's washer radially into its groove; after the tail servos, each
 * servo's arm, the pushrod with its plugs and rod ends along its own axis, and the two rod-end screws.
 */
const DIH = 35 * DEG, side = (t) => (t === 'R' ? 1 : -1);
export const TAIL_SEQUENCE = [
  { replace: 'tail tie 1', steps: [step('tail saddle with its bushings', ['^tail saddle$', '^tail bushing [RL] (inner|outer)$'], X(-1), 0.05, 'the four bushings pressed in the saddle\'s bosses first; the saddle slid on the boom\'s end from aft and bonded')] },
  { replace: 'tail tie 2', steps: [] },
  ...['R', 'L'].map((t) => ({ replace: `tail surface ${t}`, steps: [
    step(`tail surface ${t}`, [`^tail (pivot tube|pivot pin|horn|trailing edge rod) ${t}$`, `^tail rib ${t} `], [0, Math.sin(DIH), side(t) * Math.cos(DIH)], 0.1, 'its pivot tube, ribs, trailing edge rod and covering, with the horn bonded on its pivot pin and the pin\'s spigot bonded in the tube, one piece; the pin along its own axis from outboard through the outer bushing, the boss and the inner bushing'),
    step(`tail pin washer ${t}`, [`^tail pin washer ${t}$`], [0, Math.cos(DIH), -side(t) * Math.sin(DIH)], 0.03, 'radially into the groove at the pin\'s inboard end, against the inner bushing\'s flange'),
  ] })),
  { after: 'gps ties', steps: ['R', 'L'].flatMap((t) => [
    step(`tail servo arm ${t}`, [`^tail servo arm ${t}$`], Y(1), 0.03, 'onto the servo\'s output spline from above, with its centre screw'),
    step(`tail pushrod ${t} with its rod ends`, [`^tail pushrod ${t}$`, `^tail pushrod plug ${t} `, `^tail rod end ${t} `], { slide_of: `tail pushrod ${t}`, sign: 1 }, 0.02, 'the pushrod with its bonded end plugs and its two rod ends threaded on, one piece, fed in through its exit in the tail cone along its length; the last 20 mm checked: aft and down onto the two arms (the horn\'s rod end along the horn arm\'s face, the servo\'s onto its arm from above)'),
    step(`tail horn screw ${t}`, [`^tail horn screw ${t}$`], [0, -Math.sin(DIH), -side(t) * Math.cos(DIH)], 0.02, 'along the pivot axis from inboard through the rod end\'s ball into the horn\'s tapped hole'),
    step(`tail servo arm screw ${t}`, [`^tail servo arm screw ${t}$`], Y(1), 0.02, 'from above through the rod end\'s ball into the arm\'s hole'),
  ]) },
];

/**
 * Round 17's containment declarations: round 16's, with the all-moving tail's hardware outside on the boom's end (its
 * drag counted: the saddle, pins, bushings, washers, horns, rod ends and screws stand bare at the tail's root), the
 * pivot tubes with the tail's own structure (their drag the tail's, in the flight model), and the pushrods' exits.
 */
export const CONTAINMENT17 = {
  ...CONTAINMENT16,
  openings: [...CONTAINMENT16.openings, { name: 'tail pushrod exits', solids: ['tail pushrod R', 'tail pushrod L', 'tail pushrod plug R horn', 'tail pushrod plug L horn'], why: 'each pushrod leaves the pod through its own exit in the tail cone\'s skin and runs bare to its horn: the length outside the loft counts in the drag' }],
  outside: [
    ...CONTAINMENT16.outside.filter((o) => o.name !== 'the tail'),
    { name: 'the tail', re: '^tail (spar rod|trailing edge rod|rib|tie|pivot tube) ', drag: false, why: 'the V-tail\'s structure on its pivots; its drag is the tail\'s own, in the flight model' },
    { name: 'the tail\'s pivots and horns', re: '^tail (saddle$|pivot pin |bushing |pin washer |horn |horn screw |rod end [RL] horn$)', why: 'the all-moving tail\'s saddle, pivot pins, bushings, washers, horns and horn rod ends with their screws stand bare at the tail\'s root, past the pod: their frontal area counts in the drag' },
  ],
};

export function sequence17(base = SEQUENCE16, inserts = SEQUENCE_INSERTS) {
  const steps = [];
  for (const st of base.steps) {
    const rep = inserts.find((q) => q.replace === st.name);
    if (rep) steps.push(...rep.steps); else steps.push(st);
    for (const q of inserts.filter((q) => q.after === st.name)) steps.push(...q.steps);
  }
  return { ...base, steps };
}

/** Round 17's set over round 16's. */
export function round17Base(catalog = null) {
  const s = round16Set({}, catalog);
  const mounts = { ...s.layout.mounts, battery: { ...s.layout.mounts.battery, ...ROUND17.mounts.battery } };
  // round 17b (retain2): the crank shaft's bore 4 mm (RETENTION.linkage.crank), so it holds FoS 1.5 with the eye gap's overhang
  const crank = mergeDeep(mergeDeep(s.linkage.crank, ROUND17.crank), RETENTION.linkage.crank ?? {});
  const airframe = mergeDeep(s.linkage.airframe, { boom: ROUND17.boom, tail: { ties: 0 } });
  // the outer spar on CST's spec-sheet section (audit item 17: rated on 7.8 x 6.8 mm, weighed at 18.1 g/m)
  return { ...s, tubeSpecSheet: true, massFromSolids: ROUND17.massFromSolids, noseX_m: ROUND17.noseX_m, tailMech: ROUND17.tailMech, twistDrive: ROUND17.twistDrive, parts: { ...(s.parts ?? {}), ...ROUND17.parts }, layout: { ...s.layout, mounts, sequence: sequence17(SEQUENCE16, [...SEQUENCE_INSERTS, ...RETENTION_SEQUENCE, ...TAIL_SEQUENCE, ...TWIST_SEQUENCE]), ...RETENTION.layout, containment: retentionContainment({ ...CONTAINMENT17, outside: [...CONTAINMENT17.outside, ...TWIST_CONTAINMENT.outside] }), massprops: 'SPEC Function completeness (2026-11-03); src/design/round17.mjs', hatch: ROUND17.hatch, chains: 'SPEC Function completeness: every actuated motion has hardware (src/design/chains.js)', ratings: 'SPEC Function completeness: ratings within their published range (src/design/chains.js)', interfaces: 'SPEC Function completeness: every interface has a load path, axial retention and a rule (src/design/interfaces.js)' }, linkage: { ...s.linkage, crank, airframe, retention: RETENTION.linkage.retention, plates: { ...s.linkage.plates, ...RETENTION.linkage.plates }, rocker: { ...s.linkage.rocker, ...RETENTION.linkage.rocker } } };
}

/** Round 17's --set: round17Base, then `set` over it (layout and linkage merged by block, lumpPos merged). */
export function round17Set(set = {}, catalog = null) {
  const s = round17Base(catalog);
  return { ...s, ...set, layout: { ...s.layout, ...(set.layout ?? {}) }, linkage: { ...s.linkage, ...(set.linkage ?? {}) }, lumpPos: { ...(s.lumpPos ?? {}), ...(set.lumpPos ?? {}) } };
}

export { mergeDeep };

/** One round-17 build: buildRound16 on round17Set. -> { d, p, failed }. */
export async function buildRound17({ base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const s = round17Set(set, catalog);
  const b = await buildRound16({ base, set: s, opt, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round17.mjs', basis: `${b.d.selection.basis}; round 17: v24 plus function completeness (round17Set)` };
  b.d.assumptions = [...b.d.assumptions.filter((x) => !/^Round 17/.test(x)), round17Note(b.d.structure.clearance)];
  return b;
}

/** The design's round-17 assumption line, with the new records' verdicts. */
export function round17Note(c) {
  const v = (x, fail) => (!x ? 'not run' : x.passed ? 'passed' : `FAILED (${fail(x)})`);
  return `Round 17: mass properties from the drawn solids ${v(c?.massprops, (x) => x.failed.join(', '))}; ${ROUND17_RECORDS.map(([k, what, f]) => `${what} ${v(c?.[k], f)}`).join('; ')}.`;
}

/** The round-17 records --official requires: [key in the clearance record, what it is, its failure's summary]. */
export const ROUND17_RECORDS = [
  ['chains', 'actuation chains', (x) => x.failed.slice(0, 6).join('; ')],
  ['ratings', 'ratings', (x) => x.failed.slice(0, 6).join('; ')],
  ['section', 'real-section', (x) => `undeclared ${JSON.stringify(x.undeclared ?? '').slice(0, 300)}`],
  ['interfaces', 'interface ledger', (x) => `${x.failed.length} interfaces unrated${x.seats.length ? `, ${x.seats.length} seats narrower than their parts` : ''}`],
];

/** Why --official refuses a design, or null: round 16's reasons, and any round-17 record missing or failed. */
export function officialRefusal(c) {
  const why = [];
  const r16 = refusal16(c);
  if (r16) why.push(r16);
  if (!c) return why.join('; ');
  if (!c.massprops) why.push('no mass-properties record'); else if (!c.massprops.passed) why.push(`lumps not at their drawn parts: ${c.massprops.failed.join(', ')}`);
  // round 17's function-completeness records: each must be present and pass
  for (const [k, what, fail] of ROUND17_RECORDS) {
    if (!c[k]) why.push(`no ${what} record`);
    else if (!c[k].passed) why.push(`${what}: ${fail(c[k])}`);
  }
  return why.length ? [...new Set(why)].join('; ') : null;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const official = process.argv.includes('--official');
  const outPath = official ? designPath : resolve(arg('--out') ?? join(ROOT, 'out/sweep/round17/v25.json'));
  if (!official && outPath === designPath) { console.error('data/designs/ is written only with --official'); process.exit(1); }
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound17({ base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  mkdirSync(join(ROOT, 'out/sweep/round17'), { recursive: true });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (d) writeFileSync(join(ROOT, 'out/sweep/round17/v25.failed.json'), JSON.stringify(d));
    process.exit(1);
  }
  const c = d.structure.clearance;
  const verdict = `clearance ${c.clashes.length} clashes, pod ${c.pod_shell.clashes.length}; support ${c.support?.passed ? 'passed' : 'FAILED'}; containment ${c.containment?.passed ? 'passed' : 'FAILED'}; completeness ${c.completeness?.passed ? 'passed' : 'FAILED'}; assembly ${c.assembly?.passed ? 'passed' : 'FAILED'}; massprops ${c.massprops?.passed ? 'passed' : 'FAILED'}`;
  const refusal = officialRefusal(c);
  if (official && refusal) {
    console.error(`${verdict}: not written to ${designPath} (${refusal})`);
    writeFileSync(join(ROOT, 'out/sweep/round17/v25-refused.json'), JSON.stringify(d));
    process.exit(2);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  mkdirSync(dirname(outPath), { recursive: true });
  if (official) {
    w = writeDesignAtomic(outPath, d);
    writeFileSync(join(ROOT, 'out/sweep/round17/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, clearance: { passed: c.passed, clashes: c.clashes, pod: c.pod_shell.clashes, support: c.support.passed, containment: c.containment.passed, completeness: c.completeness.passed, assembly: c.assembly.passed, massprops: c.massprops.passed, frontal: c.containment.frontal }, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}, Vapp ${d.control.landing.Vapp_m_s}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}; pod ${d.body.width_m} x ${d.body.height_m} @ ${d.body.centre_y_m}; ${verdict}${refusal ? ` [${refusal}]` : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
