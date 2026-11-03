#!/usr/bin/env node
// Round 16 Larus: v24 = v23 plus completeness and assembly corrections (SPEC "Design freeze", "Completeness and
// assembly order", 2026-11-03). Round 15's pipeline (round15.mjs) with every physical BOM line in the assembly check
// (src/design/completeness.js: each solid tagged with its BOM line, counts and sizes matched), every screw and nut at
// a stated place (src/design/fixings.js), and an assembly sequence that the build checks (src/design/sequence.js).
//   node src/design/round16.mjs [--out path] [--set '<json>'] [--opt '<json>'] [--cal path] [--no-suction] [--no-fit]
//   [--no-perf] [--official]
// Exploration builds go to out/sweep/round16/v24.json. data/designs/larus-gl-2.json is written only with --official,
// and only when the clearance check passes with every part supported and contained, every physical BOM line has its
// solids (count and size), and the assembly sequence is clear. There is no override.
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound15, round15Set, CONTAINMENT as CONTAINMENT15, officialRefusal as refusal15 } from './round15.mjs';
import { ROUND14 } from './round14.mjs';
import { ROUND12 } from './round12.mjs';
import { wingStructureFor } from './wing-solids.js';
import { solidLoftGaps, xSamples } from './clearance.js';
import { driveLayout } from './joints.js';
import { drivePose, linkageAt } from '../../sims/avis-shared/model.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const SCREWS = ['screw-iso4762-m3x8-10.9', 'screw-iso4762-m3x12-10.9'];

/**
 * Round 16's corrections over v23 (round15Set), each forced by the complete check (the ledgers are in
 * docs/status/final-gates-larus.txt):
 *  - fixings: every M3 screw at its place (fixings.js), the shortest catalogue M3 (x8 or x12) that takes its stack and
 *    a full nut; the forward tray's doubler between its root tab and the plate's outer face (inside the box it met the
 *    crank boss and the floor), 12 mm tall (its screws' 2 d edge distance either side);
 *  - the right rail's lock lands (four of six sat inside the motor can's footprint, 0.5 mm aft of the flange, where
 *    each screw's nut goes): two screws per rail at 40 and 85 deg, clear of the motor can and the side panel (three
 *    on that arc would put their nuts within 5.9 mm of each other, inside a nut's 6.35 mm corner circle);
 *  - the aft tray's root screws 38 mm apart (16 mm put their heads in the crank web's sweep; at 38 mm their nuts
 *    pass the crank wheel and clear the floor's flange);
 */
export const ROUND16 = {
  // the wing's stations (wing-solids.js wingStructureFor on the v24 pod: the build checks they still hold)
  wing: { rib_root_m: 0.052, spar_to_m: 0.9383, rib_t_mm: 2 },
  plates: { t_mm: 2.0 },
  // the crank's web and pin one part, keyed on the shaft's aft stub (a DIN 6885 3 x 3 key for a 10 mm shaft, over the
  // web's 6 mm, its keyseat 1.8 mm deep), held axially with LOCTITE 638 (RC practice: no published strength on titanium)
  crank: { web: { separate: true, key: { material: 'metal-ti-6al-4v-annealed-bar', b_mm: 3, h_mm: 3, L_mm: 6, shaft_depth_mm: 1.8 } } },
  shoulderSpan_mm: 37.4,
  airframe: {
    screws: SCREWS,
    boom: { to_tail_spar_mm: 5, screw_edge_mm: 6 },
    plates: { screw_edge_mm: 6 },
    tray: { doubler_at: 'outer', doubler_h_mm: 12 },
    aftTray: { screw_pitch_mm: 38 },
    // the panels with their end flanges where each plate's inner face is free (the floor 32 mm wide so its flanges take
    // the trays' root screws)
    panels: { list: [
      { name: 'side panel right', kind: 'side', z_mm: 45, y0_mm: -20, h_mm: 29, t_mm: 0.9, flanges: [{ z: [31.2, 45.45], y: [-20, -9.75] }, { z: [38, 45.45], y: [-9.75, 8] }] },
      { name: 'side panel left', kind: 'side', z_mm: -45, y0_mm: -20, h_mm: 29, t_mm: 0.9, flanges: [{ z: [-45.45, -31.2], y: [-20, -9.75] }, { z: [-45.45, -38], y: [-9.75, 8] }] },
      { name: 'top panel', kind: 'top', y_mm: -81, w_mm: 30, t_mm: 1, flanges: [{ z: [-15, 15], y: [-81.5, -71] }] },
    ] },
    // the riser's web holed round the crank axis so the encoder's actuator passes it onto the crank's stub
    noseTray: { riser: { actuator_hole: true, actuator_hole_mm: 0.5 } },
    pod: { formers_at_mm: [-100, -30, 97, 165, 200], former_band_mm: 4, former_tab_mm: 2, former_t_mm: 2 },
    // the rails' torsion locks on the aft plate's aft face (the forward plate's lands sat in the motor can's footprint
    // and under the stage-1 wheel), two screws each 26 mm from the rail below and outboard of it, each where its head,
    // nut, shank and the flange reach nothing (probe: out/sweep/round16/scripts/landsolve.mjs); the plate's ear round
    // them; the forward brackets' sleeves start clear of the side panels' flanges on the plate's front face
    rail: { lock_head_mm: 0.9, lock_at: 'aft outer', lock_screws: 2, lock_r_mm: 26, lock_angles_deg: [-63, -39], lock_edge_mm: 6, aft_boss_mm: 0 },
  },
};

/**
 * The assembly sequence (src/design/sequence.js): each step a part or a sub-assembly put together first away from the
 * bird, the direction it arrives from (a unit vector from its place outward, body frame) and how far out it starts.
 * The drive is held with the crank at phase 0, nominal amplitude. Ties are threaded round their bundles and closed
 * in place; each is drawn sliding on along x (its band's width) from the side that is free.
 */
const X = (s) => [s, 0, 0], Y = (s) => [0, s, 0], Z = (s) => [0, 0, s];
const step = (name, members, dir, dist_m = 0.08, why = undefined) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
const side = (t) => (t === 'R' ? 1 : -1);
// a fixing along its own axis: a screw from its head's side, a nut onto its shank from the tip's side
const screwStep = (re, dir, d = 0.03) => step(`screws ${re}`, [re], dir, d);
export const SEQUENCE = {
  pose: { phase_rad: 0, why: 'the crank held at phase 0 at the nominal amplitude while the bird is put together' },
  steps: [
    step('aft plate with its journals', ['^gearbox plate aft$', ' journal aft \\('], X(-1), 0.06),
    step('crank shaft with its wheel and key', ['^crank$', '^stage 4 wheel ', '^key, crank wheel '], X(1), 0.14, 'from forward into the aft plate\'s bushing, before the forward plate'),
    step('floor', ['^gearbox top panel$'], Y(-1), 0.04, 'up under the gears onto the aft plate\'s inner face (its forward flange meets the forward plate later)'),
    step('aft tray', ['^aft tray$'], Y(-1), 0.04),
    step('aft tray screws', ['^aft tray screw '], X(-1), 0.03),
    step('aft tray nuts', ['^aft tray nut '], X(1), 0.03),
    step('crank web with its key', ['^crank web$', '^key, crank web$'], X(-1), 0.05),
    ...['R', 'L'].flatMap((t) => [
      step(`coupler ${t} with its bushings`, [`^coupler ${t}( |$)`], X(-1), 0.05),
      step(`rocker ${t} on its shoulder pin`, [`^rocker ${t}/(hub|hub relief|hub forward|pin collar|shoulder pin|arm|horn boss|spar socket)$`], Y(1), 0.06, 'the shoulder pin (with its collar) through the hub first, away from the bird'),
      step(`horn pin ${t}`, [`^rocker ${t}/horn pin$`], X(-1), 0.04),
      step(`forward shoulder bracket ${t} with its bushing`, [`^shoulder (bracket|bushing) ${t} forward$`], X(1), 0.1, 'along the shoulder pin from forward, before the motor and the gears'),
    ]),
    step('motor on its bracket', ['^motor$', '^motor bracket$', '^motor bracket screw '], X(1), 0.1, 'the motor screwed to its bracket first, away from the bird'),
    step('rails', ['^shoulder rail (right|left)$'], X(-1), 0.14, 'from aft through the aft plate\'s seats, the forward brackets\' and the motor bracket\'s sleeves; their lock flanges stop on the aft plate\'s aft face'),
    ...['R', 'L'].map((t) => step(`aft shoulder bracket ${t} with its bushing`, [`^shoulder (bracket|bushing) ${t} aft$`], X(-1), 0.06, 'along the shoulder pin and the rail from aft')),
    step('side panel right', ['^gearbox side panel right$'], Z(1), 0.04),
    step('side panel left', ['^gearbox side panel left$'], Z(-1), 0.04),
    step('rail lock screws', ['^rail lock (right|left) screw '], X(1), 0.03),
    step('rail lock nuts', ['^rail lock (right|left) nut '], X(-1), 0.03),
    step('gear shaft 1 with its gears and keys', ['^gear shaft 1$', ' on gear shaft 1$', '^stage 3 wheel ', '^stage 4 pinion '], X(1), 0.1),
    step('gear shaft 2 with its gears and keys', ['^gear shaft 2$', ' on gear shaft 2$', '^stage 2 wheel ', '^stage 3 pinion '], X(1), 0.1),
    step('gear shaft 3 with its pinion and key', ['^gear shaft 3$', ' on gear shaft 3$', '^stage 2 pinion '], X(1), 0.1),
    step('forward plate with its journals', ['^gearbox plate forward$', ' journal forward \\('], X(1), 0.08),
    step('stage 1', ['^stage 1 '], X(1), 0.06),
    step('forward tray doubler', ['^forward tray flange doubler$'], Y(-1), 0.04),
    step('forward tray', ['^forward tray$'], Y(-1), 0.04),
    step('forward tray screws', ['^forward tray screw '], X(1), 0.03),
    step('forward tray nuts', ['^forward tray nut '], X(-1), 0.03),
    step('crank encoder readhead with its screws and spacers', ['^crank encoder readhead$', '^readhead (screw|spacer) '], X(1), 0.08, 'the screws through the readhead and the spacers first, away from the bird'),
    step('nose tray riser', ['^nose tray riser$'], X(1), 0.06),
    step('readhead nuts', ['^readhead nut '], X(1), 0.03),
    step('riser screws', ['^nose tray riser screw '], Y(1), 0.03),
    step('riser nuts', ['^nose tray riser nut '], Y(-1), 0.03),
    step('crank encoder actuator', ['^crank encoder actuator$'], X(1), 0.06, 'through the riser web\'s hole onto the crank\'s stub'),
    step('nose tray', ['^nose tray$'], X(1), 0.15),
    step('nose tray screws', ['^nose tray screw '], Y(1), 0.03),
    step('nose tray nuts', ['^nose tray nut '], Y(-1), 0.03),
    step('boom clamps', ['^boom clamp (aft|forward)$'], Y(1), 0.04),
    step('boom clamp screws aft', ['^boom clamp aft screw '], X(-1), 0.03),
    step('boom clamp screws forward', ['^boom clamp forward screw '], X(1), 0.03),
    step('boom clamp nuts aft', ['^boom clamp aft nut '], X(1), 0.03),
    step('boom clamp nuts forward', ['^boom clamp forward nut '], X(-1), 0.03),
    step('tail boom', ['^tail boom$'], X(-1), 1.2, 'from aft through both clamps and both plates'),
    ...[1, 2].map((k) => step(`tail tie ${k}`, [`^tail tie ${k}$`], X(-1), 0.25, 'along the boom from its aft end, before the tail surfaces')),
    ...['R', 'L'].map((t) => step(`tail surface ${t}`, [`^tail (spar rod|trailing edge rod) ${t}$`, `^tail rib ${t} `], [0, Math.sin(35 * Math.PI / 180), side(t) * Math.cos(35 * Math.PI / 180)], 0.1, 'its rods and ribs one piece, covered, along its own span onto the boom, its root rib through the ties')),
    ...['esc', 'airspeed', 'receiver', 'battery', 'servo_tail_L', 'servo_tail_R', 'fc', 'regulator', 'gps'].flatMap((n) => [
      step(`${n}`, [`^packed ${n}$`], Y(1), 0.06),
      { name: `${n} ties`, members: [`^packed ${n} ties$`], threaded: true, why: 'threaded through the tray\'s slots round the part and closed' },
    ]),
    step('pitot-static probe', ['^pitot-static probe$'], X(1), 0.12),
    { name: 'pitot ties', members: ['^pitot-static probe ties$'], threaded: true, why: 'threaded round the probe and the flange, through the slot at its root' },
    step('former 2', ['^former 2$'], X(-1), 0.3), step('former 1', ['^former 1$'], X(-1), 0.3),
    step('former 3', ['^former 3$'], X(1), 0.3), step('former 4', ['^former 4$'], X(1), 0.3), step('former 5', ['^former 5$'], X(1), 0.3),
    ...[1, 2, 3, 4, 5].map((k) => ({ name: `former ${k} tie`, members: [`^former ${k} tie$`], threaded: true, why: 'threaded round the tray\'s flange and the former\'s tab' })),
    ...['R', 'L'].map((t) => step(`wing ${t}`, [`^(inner spar|outer spar|twist torque rod|twist servo|leading edge rod|trailing edge rod|rib) ${t}( |$)`], Z(side(t)), 1.1, 'the wing\'s structure one piece, its inner spar into the rocker\'s socket along the span')),
  ],
};

/**
 * Declared openings and outside fittings (clearance.js containmentRecord): round 15's, with the solids round 16 adds to
 * them (the horn eyes' bushing flanges and their counterbores in the horn slots; the boom clamps' screws and nuts with
 * the clamps outside).
 */
export const CONTAINMENT = {
  ...CONTAINMENT15,
  on_skin: [{ re: '^former \\d+$', why: 'the formers carry the covering: each band\'s outside on the loft (touching it), at the smaller of the loft\'s sections across its thickness' }],
  openings: CONTAINMENT15.openings.map((o) => (o.name === 'horn slots' ? { ...o, solids: [...o.solids, 'coupler R/horn eye counterbore', 'coupler L/horn eye counterbore', 'coupler R horn bushing/flange', 'coupler L horn bushing/flange'] }
    : /^wing root, (right|left)$/.test(o.name) ? { ...o, solids: [...o.solids, `inner spar ${/right/.test(o.name) ? 'R' : 'L'}/tube`], why: `${o.why}; the inner spar runs out of the socket through the same opening` } : o)),
  outside: [
    ...CONTAINMENT15.outside,
    ...['aft', 'forward'].flatMap((nm) => [1, 2].flatMap((k) => [
      { part: `boom clamp ${nm} screw ${k}`, why: `a screw of the ${nm} boom clamp, on the exposed clamp and its plate's pad above the pod` },
      { part: `boom clamp ${nm} nut ${k}`, why: `a nut of the ${nm} boom clamp, on its plate's pad above the pod` },
    ])),
    { name: 'the wings', re: '^(outer spar|twist torque rod|twist servo|leading edge rod|trailing edge rod|rib) [RL]( |$)', drag: false, why: 'the wings\' structure outboard of the wing roots, through the full stroke; their drag is the wing\'s own, in the flight model' },
    { name: 'the tail', re: '^tail (spar rod|trailing edge rod|rib|tie) ', drag: false, why: 'the V-tail\'s structure on the boom; its drag is the tail\'s own, in the flight model' },
  ],
};

const merge = (a, b) => { if (!b || typeof b !== 'object' || Array.isArray(b)) return b; const o = { ...(a ?? {}) }; for (const [k, v] of Object.entries(b)) o[k] = v && typeof v === 'object' && !Array.isArray(v) ? merge(a?.[k], v) : v; return o; };

/** Round 16's set over round 15's: the airframe's fixings and corrections (deep-merged over round15Set's airframe). */
export function round16Base(catalog = null) {
  const s = round15Set({}, catalog);
  const af = merge(s.linkage.airframe, ROUND16.airframe);
  // the wing's spar from its socket's base (the rocker hub's radius from the shoulder axis, build.js); the tail's roots on the boom
  const wingStructure = { ...ROUND16.wing };
  return { ...s, wingStructure, tailLines: 'split', tailY_m: (af.boom?.y_mm ?? 62) / 1000, joints: { ...ROUND14.set.joints, shoulderSpan_mm: ROUND16.shoulderSpan_mm }, layout: { ...s.layout, containment: CONTAINMENT, sequence: SEQUENCE, completeness: 'SPEC Completeness and assembly order (2026-11-03); src/design/round16.mjs' }, linkage: { ...s.linkage, plates: { ...s.linkage.plates, ...ROUND16.plates }, crank: { ...ROUND12.set.linkage.crank, ...ROUND16.crank }, airframe: af } };
}

/** Round 16's --set: round16Base, then `set` over it (layout and linkage merged by block, lumpPos merged). */
export function round16Set(set = {}, catalog = null) {
  const s = round16Base(catalog);
  return { ...s, ...set, layout: { ...s.layout, ...(set.layout ?? {}) }, linkage: { ...s.linkage, ...(set.linkage ?? {}) }, lumpPos: { ...(s.lumpPos ?? {}), ...(set.lumpPos ?? {}) } };
}

export { merge as mergeDeep };

/**
 * One round-16 build: buildRound15 on round16Set, then the wing's stations checked on the built pod (wing-solids.js
 * wingStructureFor: they must be the ones stated, or the build fails naming the right ones). -> { d, p, failed }.
 */
export async function buildRound16({ base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const s = round16Set(set, catalog);
  const b = await buildRound15({ base, set: s, opt, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  // the flown mass must be the BOM's (M6 checks it too): each line in exactly one lump
  if (Math.abs(b.d.bom.total_g - b.d.mass.total_g) >= 0.5) return { ...b, failed: `the mass model's ${b.d.mass.total_g.toFixed(1)} g is not the BOM's ${b.d.bom.total_g.toFixed(1)} g` };
  const ws = wingStations(b.d, catalog), W = b.d.params.wingStructure;
  if (Math.abs(ws.rib_root_m - W.rib_root_m) > 1e-9 || Math.abs(ws.spar_to_m - W.spar_to_m) > 1e-9) return { ...b, failed: `the wing's stations on this pod are ${JSON.stringify(ws)}, the build states ${JSON.stringify(W)}` };
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round16.mjs', basis: `${b.d.selection.basis}; round 16: v23 plus completeness and assembly corrections (round16Set: every BOM line's solids, the fixings, the crank's web, the assembly sequence)` };
  b.d.assumptions = [...b.d.assumptions.filter((x) => !/^Round 16/.test(x)), round16Note(b.d.structure.clearance)];
  return b;
}

/** The wing's stations from a built design (wing-solids.js wingStructureFor with the check's own helpers). */
export function wingStations(d, catalog) {
  return wingStructureFor(d, catalog, driveLayout(d, catalog), {}, { drivePose, linkageAt, solidLoftGaps, xSamplesOf: xSamples });
}

/** The design's round-16 assumption line, with the checks' verdicts. */
export function round16Note(c) {
  const k = c?.completeness, a = c?.assembly;
  const v = (x, fail) => (!x ? 'not run' : x.passed ? 'passed' : `FAILED (${fail(x)})`);
  return `Round 16: every physical BOM line has its solids in the assembly check, tagged with its line, its count and size matched (structure.clearance.completeness; exempt: ${(k?.exemptions ?? []).map((e) => e.line).join('; ')}); every screw and nut at a stated place (structure.airframe.parts.fixings); the crank's web and pin keyed on the shaft's stub; the assembly sequence stated (params.layout.sequence) and checked (structure.clearance.assembly). Completeness ${v(k, (x) => x.failed.join(', '))}; assembly ${v(a, (x) => [...x.failures, ...x.missing.slice(0, 5)].join(', '))}.`;
}

/**
 * Why --official refuses a design, or null when it may be written: round 15's reasons (any clash, any part not
 * supported or not contained), any physical BOM line without its solids or with a count or size that disagrees, any
 * step of the sequence that passes through a placed part or a solid in no step, or a record missing.
 */
export function officialRefusal(c) {
  const why = [];
  const r15 = refusal15(c);
  if (r15) why.push(r15);
  if (!c) return why.join('; ');
  if (!c.completeness) why.push('no completeness record'); else if (!c.completeness.passed) why.push(`incomplete: ${[...c.completeness.failed, ...c.completeness.untagged.slice(0, 5), ...c.completeness.unknown_tags.map((u) => u.tag)].join(', ')}`);
  if (!c.assembly) why.push('no assembly sequence record'); else if (!c.assembly.passed) why.push(`the sequence fails: ${[...c.assembly.failures, ...(c.assembly.missing.length ? [`${c.assembly.missing.length} solids in no step`] : [])].join(', ')}`);
  return why.length ? [...new Set(why)].join('; ') : null;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const official = process.argv.includes('--official');
  const outPath = official ? designPath : resolve(arg('--out') ?? join(ROOT, 'out/sweep/round16/v24.json'));
  if (!official && outPath === designPath) { console.error('data/designs/ is written only with --official'); process.exit(1); }
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound16({ base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  mkdirSync(join(ROOT, 'out/sweep/round16'), { recursive: true });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (d) writeFileSync(join(ROOT, 'out/sweep/round16/v24.failed.json'), JSON.stringify(d));
    process.exit(1);
  }
  const c = d.structure.clearance;
  const verdict = `clearance ${c.clashes.length} clashes, pod ${c.pod_shell.clashes.length}; support ${c.support?.passed ? 'passed' : 'FAILED'}; containment ${c.containment?.passed ? 'passed' : 'FAILED'}; completeness ${c.completeness?.passed ? 'passed' : 'FAILED'}; assembly ${c.assembly?.passed ? 'passed' : 'FAILED'}`;
  const refusal = officialRefusal(c);
  if (official && refusal) {
    console.error(`${verdict}: not written to ${designPath} (${refusal})`);
    writeFileSync(join(ROOT, 'out/sweep/round16/v24-refused.json'), JSON.stringify(d));
    process.exit(2);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  mkdirSync(dirname(outPath), { recursive: true });
  if (official) {
    w = writeDesignAtomic(outPath, d);
    writeFileSync(join(ROOT, 'out/sweep/round16/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, clearance: { passed: c.passed, clashes: c.clashes, pod: c.pod_shell.clashes, support: c.support.passed, containment: c.containment.passed, completeness: c.completeness.passed, assembly: c.assembly.passed, frontal: c.containment.frontal }, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}, Vapp ${d.control.landing.Vapp_m_s}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}; pod ${d.body.width_m} x ${d.body.height_m} @ ${d.body.centre_y_m}; ${verdict}${refusal ? ` [${refusal}]` : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
