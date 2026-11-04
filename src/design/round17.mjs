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
import { ROUND14 } from './round14.mjs'; // round 18: its part change (the 10 mm encoder actuator) kept under round 17's parts
import { RETENTION, RETENTION_SEQUENCE, retentionContainment } from './round17-retention.mjs';
import { TWIST_SEQUENCE, TWIST_CONTAINMENT, TWIST_SECTION } from './twist-drive.js';
import { NEEDS_SEQUENCE, needsContainment, needsParams, mergeContainment } from './build-needs.js';
import { POD_SEQUENCE, POD_CONTAINMENT } from './pod-structure.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Round 18 (the CG, staticMargin 0.10): the pack this far forward of its round-17 place (mm). With it move what is
 * placed by the pack: its strap, plugs, forward stop, the hatch's forward end (rails, forward arc, keeper, the window's
 * forward edge), the nose tray's tip and the pitot probe; and the pod's nose is stretched (pod-structure.js
 * noseStretch) so the loft's section at the pack's front is the same as before.
 */
export const PACK_FWD_MM = 0;
const F = PACK_FWD_MM;
// a station of the nose (mm) where the stretch takes it: the loft's ogive is stretched about its first full section
// (x 12.27 mm on v24's pod) so that x 231.2 (the pack's front) moves F forward; parts fixed to the nose's structure
// move with the stretch (the hatch's rails and arcs; its keeper too, but never within 0.5 mm of the arming jumper),
// parts fixed to the pack (its strap, plugs and jumper on its top, its forward stop) move F
const fwd = (x) => +(x + (F * (x - 12.27)) / (231.2 - 12.27)).toFixed(3);

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
  mounts: { battery: { x: null, x0_m: +(0.1565 + F / 1000).toFixed(6) } },
  // the crank web (audit item 13 and its axial fixing): a 20 mm hub boss round the bore (the wall behind the keyway
  // 3.5 mm), and an end lip 1.7 mm thick over the stub's end holding the web axially by an ISO 10642 M3 x 8 into the
  // stub's tapped end (the shaft solid 2.5 mm on beyond the web for its thread), its countersunk head flush (the coupler eyes run 0.5 mm aft of the web, so no head may stand
  // there; ISO 898-1 at half for the countersunk head); the LOCTITE 638 bond, unrated on titanium, is gone. The key over
  // the rest of the web, 4.3 mm, its keyseat 1.5 mm into the stub so the web's keyway is 1.5 mm deep (machined to these
  // depths; the key rule rates them). Round 18 (sized to the FoS rule: its bearing at 2.44 against 2.5 at 13.5 m/s): a
  // 4 x 4 mm key (DIN 6885's section for 10-12 mm shafts), 2.0 mm into the stub and 2.0 mm into the web (the web's hub
  // wall behind it 3.0 mm), 3.75 mm long: its inner face 3.0 mm from the axis, level with the axial screw's 6.0 mm
  // countersunk head in the lip, so it stops 0.55 mm short of the lip (the 0.5 mm clearance); bearing area 2.0 x 3.75 =
  // 7.5 mm2 a side, against 1.5 x 4.3 = 6.45
  crank: { web: { hub_od_mm: 20, lip_mm: 1.7, axial: { screw: 'screw-iso10642-m3x8-10.9', countersunk: true, clearance_mm: 0.2, solid_mm: 2.5 }, key: { L_mm: 3.75, b_mm: 4, h_mm: 4, shaft_depth_mm: 2.0 } } },
  // the servos (orchestrator, 2026-11-03; audit item 15): the twist servo's horn and spline are unsourced (MKS pages
  // never loaded), so its chain cannot be drawn from catalogue parts: the twist servo is the Hitec MD89MW (its R-ML25
  // arm is sourced); both servos on the Pololu D36V50F6 (6 V +-4%, inside the MD89MW's 4.8-7.4 V rows; the F7's
  // 7.2-7.8 V was above the 7.4 V row)
  parts: { twistServo: 'servo-hitec-md89mw', servoRegulator: 'regulator-pololu-d36v50f6' },
  // the all-moving V-tail (audit items 2, 11): pivots, saddle, horns and pushrods (src/design/tail-mech.js TAIL_MECH);
  // the pushrod's length as drawn (the completeness check holds the BOM to the drawn rod)
  // (round 18: 899.30 with the tail servos 6 mm forward, cgMounts; 893.36 at round 17's place)
  tailMech: { pushrod_L_mm: 899.30 },
  // the boom to the saddle's aft end (the sleeve 8 mm aft of the tail's quarter chord)
  // round 18 (sized to the FoS rule: the forward plate under the forward boom clamp at 2.16 against 2.5 in the fastest
  // trim's flight): the boom clamps 4 mm long (were 6), so the clamp's prying lever on the plate is 2 + 1 mm, not 3 + 1
  boom: { to_tail_spar_mm: 8, clamp_len_mm: 4 },
  // the access hatch's window in the pod's covering (audit item 6): over the pack, on the top (polar angle as the
  // containment record measures it: 0 deg at +z, 90 at the top). The pod's structure (pod-structure.js) leaves this
  // window open; the hatch (build-needs.js) frames and covers it, and its fixing holds it
  // round 17b: the window runs aft to x 133 (the receiver's bind button and the FC's USB-C extender under it) and
  // forward to x 234 (the pack's front end is at x 231.2: it must lift out inside the window).
  // round 18 (the pod's structure merged): the polar range 20-160 deg, on the pod's longerons at 20 and 160. At 40-140 the
  // window's edge rods, where the nose narrows (x 214-234), stood over the cover's lands (z 19.1-25.5 mm) and in the
  // pack's way out (its corners reach polar 41.2-138.8 deg); at 20 deg the rod at x 222 clears the lands by 2.6 mm. The
  // cover's arcs reach to 3 mm short of the window's polar edges (build-needs.js hatchGeometry), so its film closes it
  hatch: { x_mm: [133, 234 + F], polar_deg: [20, 160] },
  // what a real build needs (src/design/build-needs.js NEEDS): the stated lengths the completeness check holds to the
  // drawn parts (the pressure tubing, the clips), and the forward stop at the drawn pack's front end
  // round 18: the antenna's tie 6 mm aft (x 197.25, was 203.25), where the nose is wider: at 203.25 its bottom stood
  // 0.28 mm outside the film's chord between the pod's 180 and 210 deg longerons
  // (round 18: the pitot's two pressure lines 2F longer, the probe being F forward)
  needs: { tubing: { length_mm: +(133.8 + 2 * F).toFixed(3) }, leads: { clip_mm3: 684 }, stop_fwd: { pack_x1: 231.2 + F }, antenna: { tie_x: 197.25 },
    ...(F ? { strap: { x_mm: 174.3 + F }, plugs: { pack_x0: 182.3 + F, arming_x0: 198.7 + F }, hatch: { rail_x: [134, fwd(222)], arcs_x: [[134, 134.5], [164.75, 165.25], [fwd(199.75), fwd(199.75) + 0.5], [fwd(222) - 0.5, fwd(222)]], keepers_x: [[Math.max(fwd(217.5), 215.1 + F), Math.max(fwd(217.5), 215.1 + F) + 4]] } } : {}) },
  // round 18: the nose tray's tip with the stretch, the pitot probe (its ties) F forward with the pack
  ...(F ? { airframeFwd: { noseTray: { tip_x_mm: fwd(228) }, pitot: { x_aft_mm: 196 + F, tie_x_mm: [200 + F, 207.5 + F] } } } : {}),
  // the FC and the servo regulator on the belly tray under the nose tray (the FC's USB-C extender's 20 cm cable then
  // reaches the hatch, and the ASPD-4525's 20 cm cable reaches the FC; the CG comes forward); the pack on a releasable
  // strap in place of its one-shot ties
  needsMounts: {
    fc: { carrier: 'belly tray', face_y_m: -0.0693, x0_m: 0.1115, z_m: 0, ties: 1, z0_m: null },
    regulator: { carrier: 'belly tray', face_y_m: -0.0693, after: 'fc', gap_m: 0.0005, z_m: 0, ties: 1, x0_m: null, z0_m: null },
    battery: { strap: 'strap-rctimer-hookloop-15x330' },
    // the receiver to the left, the airspeed board to the right: the pressure lines reach the board's ports from the
    // right flange without crossing the receiver, whose bind button stays clear under the hatch
    receiver: { z0_m: -0.01726 }, airspeed: { z0_m: -0.0037 },
  },
  // round 18 (the CG): the tail servos 6 mm forward on the aft tray (their fronts 1.1 mm aft of the rockers' horn bosses, which they pass
  // under as they go in from above), the GPS right behind them (on the tray's centre line, under the film and
  // the boom as before: the same sky view); dx in mm, 0 = round 17's places
  cgMounts: { servo_dx_mm: 6, gps_dx_mm: 38.5 },
  // the wing twist drive (audit items 1, 3, 10, 12; src/design/twist-drive.js TWIST_DRIVE): the spars coaxial 5.5 mm above
  // the E387's chord line, the joiner, the servo in its frame and its four-bar to the torque rod's lever, the bar and the
  // tip; the parts that stand out of the section declared with their drag (TWIST_SECTION)
  twistDrive: { section: TWIST_SECTION },
  // the pod's structure and covering (audit item 4): formers, longerons and end discs under the film, the loft ending
  // where they do, its cuts framed (src/design/pod-structure.js POD: its stations, sets and windows); the nose closes on
  // its end disc where the loft's first segment is this wide
  pod: { nose_end_w_mm: 12, ...(F ? { nose_stretch: { x_mm: 231.2, by_mm: F } } : {}) },
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
  { after: 'gps ties', steps: ['R', 'L'].map((t) => step(`tail servo arm ${t}`, [`^tail servo arm ${t}$`], Y(1), 0.03, 'onto the servo\'s output spline from above, with its centre screw')) },
  // round 18: the pushrods after the pod's formers (pod-structure.js POD_SEQUENCE, in place of round 16's 'former 2') and
  // before its longerons (after 'former 1'): each is fed in through its exit window in the cone, which no ring can then
  // slide past, and the exits' edge rods are bonded on after it
  { after: 'former 2', steps: ['R', 'L'].flatMap((t) => [
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
// Round 18 (bug): round14Set merges a set's `parts` over ROUND14.set's wholesale ({ ...ROUND14.set, ...set }), so round
// 17's parts block dropped round 14's 10 mm encoder actuator (sensor-rls-orbis-actuator-10mm, SPEC Design freeze v22: the
// one part that could not fit) and the full build fell back to the base's 8 mm part. The quick probe, built over v24's
// own params, never showed it. ROUND14.set.parts is now carried under round 17's.
export function round17Base(catalog = null) {
  const s = round16Set({}, catalog);
  const NM = ROUND17.needsMounts, mounts = { ...s.layout.mounts, fc: NM.fc, regulator: NM.regulator, battery: { ...s.layout.mounts.battery, ...ROUND17.mounts.battery, ...NM.battery }, receiver: { ...s.layout.mounts.receiver, ...NM.receiver }, airspeed: { ...s.layout.mounts.airspeed, ...NM.airspeed } };
  const CM = ROUND17.cgMounts, mv = (k, dx) => (dx ? { [k]: { ...mounts[k], x0_m: +(mounts[k].x0_m + dx / 1000).toFixed(6) } } : {});
  Object.assign(mounts, mv('servo_tail_L', CM.servo_dx_mm), mv('servo_tail_R', CM.servo_dx_mm), mv('gps', CM.gps_dx_mm));
  // round 17b (retain2): the crank shaft's bore 4 mm (RETENTION.linkage.crank), so it holds FoS 1.5 with the eye gap's overhang
  const crank = mergeDeep(mergeDeep(s.linkage.crank, ROUND17.crank), RETENTION.linkage.crank ?? {});
  const airframe = mergeDeep(s.linkage.airframe, { boom: ROUND17.boom, tail: { ties: 0 }, ...(ROUND17.airframeFwd ?? {}) });
  // the outer spar on CST's spec-sheet section (audit item 17: rated on 7.8 x 6.8 mm, weighed at 18.1 g/m)
  // the pod's longerons on the skin in place of round 16's formers (pod-structure.js POD_CONTAINMENT)
  const c17 = mergeContainment(retentionContainment({ ...CONTAINMENT17, outside: [...CONTAINMENT17.outside, ...TWIST_CONTAINMENT.outside] }), needsContainment(needsParams({ needs: ROUND17.needs }), ROUND17.hatch));
  const containment = { ...c17, on_skin: [...(c17.on_skin ?? []).filter((o) => !/^\^former /.test(o.re)), ...POD_CONTAINMENT.on_skin] };
  return { ...s, tubeSpecSheet: true, massFromSolids: ROUND17.massFromSolids, noseX_m: ROUND17.noseX_m, tailMech: ROUND17.tailMech, twistDrive: ROUND17.twistDrive, pod: ROUND17.pod, parts: { ...(ROUND14.set.parts ?? {}), ...(s.parts ?? {}), ...ROUND17.parts }, needs: ROUND17.needs, layout: { ...s.layout, mounts, sequence: sequence17(SEQUENCE16, [...SEQUENCE_INSERTS, ...RETENTION_SEQUENCE, ...TAIL_SEQUENCE, ...TWIST_SEQUENCE, ...NEEDS_SEQUENCE, ...POD_SEQUENCE]), ...RETENTION.layout, containment, massprops: 'SPEC Function completeness (2026-11-03); src/design/round17.mjs', covering: 'SPEC Function completeness, the covering (2026-11-03); src/design/pod-structure.js', hatch: ROUND17.hatch, chains: 'SPEC Function completeness: every actuated motion has hardware (src/design/chains.js)', ratings: 'SPEC Function completeness: ratings within their published range (src/design/chains.js)', interfaces: 'SPEC Function completeness: every interface has a load path, axial retention and a rule (src/design/interfaces.js)' }, linkage: { ...s.linkage, crank, airframe, retention: RETENTION.linkage.retention, plates: { ...s.linkage.plates, ...RETENTION.linkage.plates }, rocker: { ...s.linkage.rocker, ...RETENTION.linkage.rocker } } };
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
  ['buildneeds', 'what a real build needs', (x) => x.failed.join('; ')],
  ['covering', 'covering', (x) => x.failed.slice(0, 8).join(', ')],
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
