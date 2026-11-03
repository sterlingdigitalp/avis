#!/usr/bin/env node
// Round 15 Larus: v23 = v22 plus support and containment corrections (SPEC "Design freeze", "Support and
// containment", 2026-11-03). Round 14's pipeline (round14.mjs ROUND14.set) with every packed part on the structure
// that carries it through a stated fixing (params.layout.mounts, sims/avis-shared/model.js placeMounted), the carriers
// that hold them (structure.airframe carriers, src/design/airframe.js carrierGeometry), and every part inside the pod
// loft except at declared openings, with the fittings left outside counted in the pod's drag (params.layout.containment,
// src/design/clearance.js containmentRecord). Members, ties and fixings follow by the existing sizing rules; nothing
// else changes: same parts, planform, kinematics and controller.
//   node src/design/round15.mjs [--out path] [--set '<json>'] [--opt '<json>'] [--cal path] [--no-suction] [--no-fit]
//   [--no-perf] [--official]
// Exploration builds go to out/sweep/round15/v23.json. data/designs/larus-gl-2.json is written only with --official,
// and only when the design's clearance check passes with every part supported and contained (structure.clearance:
// no clash, support.passed, containment.passed). There is no override.
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound14, ROUND14 } from './round14.mjs';
import { pitotMount } from './airframe.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const AL6 = 'metal-al-6061-t6';

/**
 * The carriers (airframe.js carrierGeometry; mm). The forward tray runs from the forward plate under the crank
 * encoder to the nose tray riser, which stands just forward of the crank stub's end and carries the nose tray at the
 * battery's underside (the battery on the loft's axis, so it reaches furthest into the nose: its CG trim lies further
 * forward than the loft allows, as in v22). The trays sit at y -80 mm, inside both plates' outlines (no post below them).
 * Widths: the nose tray holds the ESC across it with its ties (34.3 + 2 x 1.15 mm), the riser the readhead's three
 * mounting lands, the forward tray the riser's foot, the aft tray two columns (the tail servos side by side, then the
 * flight controller beside the regulator, then the GPS). Sheets and flanges by the round 8 checks (the carriers' own
 * masses among the loads): the new nose tray in 0.6 mm sheet with 5 mm flanges, the motor bracket 2 mm (calm FoS of
 * the first build, 0.8 mm / 7 mm and 2.5 mm: 8.9 and 14.9).
 */
export const CARRIERS = {
  tray: { material: AL6, y_mm: -80, w_mm: 38.4, flange_h_mm: 7, t_mm: 0.8, screws: 2, screw_pitch_mm: 18, screw_lever_mm: 8, flange_mm: 10, doubler_mm: 2, item_height_mm: 30 },
  noseTray: {
    material: AL6, y_mm: -43.1, w_mm: 37.6, flange_h_mm: 5, t_mm: 0.6, tip_x_mm: 228, riser_x_mm: 108.9, screws: 2, screw_pitch_mm: 16, screw_lever_mm: 6, item_height_mm: 15,
    riser: { material: AL6, w_mm: 37.6, flange_mm: 7, t_mm: 0.8, foot_mm: 8, screws: 2, screw_pitch_mm: 16, screw_lever_mm: 6, item_height_mm: 40 },
  },
  aftTray: { material: AL6, y_mm: -81.2, w_mm: 53.2, flange_h_mm: 6, t_mm: 0.6, length_mm: 123, screws: 2, screw_pitch_mm: 16, screw_lever_mm: 8, flange_mm: 10, doubler_mm: 0, item_height_mm: 30 },
  encoder: { screw: 'screw-iso4762-m2x25-8.8', nut: 'nut-iso4032-m2-8', spacer_od_mm: 3.8, spacer_id_mm: 2.2, holes_deg: [30, 150, 270] },
  pitot: { ties: 2, side: 'right', x_aft_mm: 196, tie_x_mm: [200, 207.5] },
  // the motor by its back face (the maker's drawing: 4 x M3 threads 25 mm across on the stator side; its front is the
  // rotating bell) on a 2 mm 6061-T6 plate bonded on both shoulder rails; the bell 1.9 mm from the forward plate, so it
  // clears the rail's 1.4 mm lock flange by the running clearance (as v22 placed it)
  motor: { bracket: { material: AL6, t_mm: 2, circle_d_mm: 25, angles_deg: [0, 90, 180, 270], land_r_mm: 3.5, sleeve_wall_mm: 2, stub_hole_d_mm: 6 } },
};

const face = (T) => (T.y_mm + T.t_mm / 2) / 1000;
const inner = (T) => (T.w_mm - T.t_mm) / 2000; // the flange's inner face (m)
const T40 = 0.00115, T18 = 0.001, GAP = 0.0005;

/**
 * Where each packed part sits (model.js placeMounted; m): on the carrier named, flat, its ties counted (airframe.js
 * picks each tie by its loop). Nose tray, aft to fore: the ESC across the tray clear of the riser tab's screw heads,
 * the airspeed board beside the receiver, then the battery (x from its CG trim, as far forward as the loft holds it).
 * Aft tray, fore to aft from just aft of the shoulder assembly: the tail servos side by side, the flight controller
 * beside the regulator, the GPS.
 */
export function mountsFor(C = CARRIERS) {
  const N = C.noseTray, A = C.aftTray, fN = face(N), fA = face(A), iN = inner(N), iA = inner(A);
  const ntab = (N.riser_x_mm + N.riser.t_mm + N.riser.foot_mm) / 1000; // the riser tab's forward edge
  return {
    esc: { carrier: 'nose tray', face_y_m: fN, x0_m: +(ntab - 0.0007).toFixed(4), z_m: 0, ties: 1 },
    airspeed: { carrier: 'nose tray', face_y_m: fN, after: 'esc', gap_m: GAP, z0_m: +(-iN + T18).toFixed(5), ties: 1 },
    receiver: { carrier: 'nose tray', face_y_m: fN, after: 'esc', gap_m: GAP, z0_m: +(-iN + T18 + 0.02116 + T18 + GAP + T18).toFixed(5), ties: 1 },
    battery: { carrier: 'nose tray', face_y_m: fN, x: 'lump', after: 'airspeed', gap_m: GAP, z_m: 0, ties: 2 },
    servo_tail_L: { carrier: 'aft tray', face_y_m: fA, x0_m: -0.0513, z0_m: -(GAP / 2 + T40 + 0.013), ties: 2 },
    servo_tail_R: { carrier: 'aft tray', face_y_m: fA, x0_m: -0.0513, z0_m: GAP / 2 + T40, ties: 2 },
    fc: { carrier: 'aft tray', face_y_m: fA, x0_m: -0.0838, z0_m: +(-iA + T40).toFixed(5), ties: 1 },
    regulator: { carrier: 'aft tray', face_y_m: fA, x0_m: -0.0772, z0_m: +(-iA + T40 + 0.022 + T40 + GAP + T40).toFixed(5), ties: 1 },
    gps: { carrier: 'aft tray', face_y_m: fA, x0_m: -0.1073, z_m: 0, ties: 1 },
  };
}

/**
 * Declared openings and outside fittings (clearance.js containmentRecord), each decided from the v22 geometry (round
 * 14's podframe report) and recorded with its reason.
 */
export const CONTAINMENT = {
  Cd_out: 1.2,
  openings: [
    { name: 'wing root, right', drag: false, solids: ['rocker R/spar socket'], why: 'the rocker\'s spar socket carries the wing spar out through the pod\'s side at the wing root and sweeps the full stroke; the wing\'s own drag model starts there' },
    { name: 'wing root, left', drag: false, solids: ['rocker L/spar socket'], why: 'the left rocker\'s spar socket, as the right: out through the pod\'s side at the wing root, through the full stroke' },
    { name: 'horn slots', solids: ['coupler R/horn eye', 'coupler L/horn eye', 'coupler R horn bushing/body', 'coupler L horn bushing/body', 'rocker R/horn boss', 'rocker L/horn boss', 'rocker R/horn pin', 'rocker L/horn pin', 'rocker R/arm', 'rocker L/arm'], why: 'each rocker\'s horn (its arm, boss and pin) and the coupler eye on it ride inside the shoulder span, beside the wing root, and rise above the loft at the top of the stroke: a slot in the pod\'s top over the shoulders; their exposed sweep counts in the drag' },
    { name: 'boom posts', solids: ['gearbox plate aft/post 1 (boom clamp)', 'gearbox plate aft/pad 1 (boom clamp)', 'gearbox plate forward/post 1 (boom clamp)', 'gearbox plate forward/pad 1 (boom clamp)'], why: 'the tail boom runs above the pod (over the shoulders it must clear the horn eyes\' sweep), so each gearbox plate\'s post rises through a slot in the pod\'s top to its boom clamp; the exposed post and pad count in the drag' },
    { name: 'pitot', solids: ['pitot-static probe/tube'], why: 'the probe reads the free stream ahead of the nose, so it leaves the nose through a hole of its own diameter; its exposed tip counts in the drag' },
  ],
  outside: [
    { part: 'tail boom', why: 'it runs above the pod from the forward clamp to the tail (it never enters the loft)' },
    { part: 'boom clamp aft', why: 'on the exposed boom, screwed to the aft plate\'s post' },
    { part: 'boom clamp forward', why: 'on the exposed boom, screwed to the forward plate\'s post' },
  ],
};

/**
 * The pod grid the fit rule searches (fit.js fitBody: the smallest frontal area that holds every part, then the
 * highest centre line). It starts at v22's pod, which no smaller pod held, and now steps the centre line by 1 mm,
 * because the enclosed parts (the horn eyes' sweep at the top, the trays at the bottom) and the outside fittings (the
 * boom clamps above the skin) leave a narrow band of centre lines.
 */
export const FIT_GRID = { widths: [0.11, 0.12], heights: [0.14, 0.15], ys: Array.from({ length: 26 }, (_, k) => +(-0.015 - 0.001 * k).toFixed(3)) };

const R14 = ROUND14.set, RL = R14.linkage, RA = RL.airframe;
/** Round 15's set over round 14's: the layout (mounts, openings, encoder and motor faces) and the carriers. */
export function round15Base(C = CARRIERS, catalog = null) {
  const set = {
    fitGrid: FIT_GRID,
    layout: { ...R14.layout, encoder_gap_mm: 2.5, motor_gap_mm: 1.9, plate_faces: 'actual', mounts: mountsFor(C), containment: CONTAINMENT, support: 'SPEC Support and containment (2026-11-03); src/design/round15.mjs' },
    linkage: { ...RL, airframe: { ...RA, tray: C.tray, aftTray: C.aftTray, noseTray: C.noseTray, encoder: { ...(RA.encoder ?? {}), ...C.encoder }, pitot: C.pitot, motor: { ...(RA.motor ?? {}), ...C.motor } } },
  };
  if (catalog) {
    // the probe's lump where its mount puts it (its axis, the middle of its length)
    const pm = pitotMount({ noseTray: C.noseTray, pitot: C.pitot }, {}, catalog);
    set.lumpPos = { 'pitot-static probe': [+((pm.x[0] + pm.x[1]) / 2).toFixed(5), +pm.c[1].toFixed(5), +pm.c[0].toFixed(5)] };
  }
  return set;
}

/** Round 15's --set: round15Base, then `set` over it (layout and linkage merged by block, lumpPos merged). */
export function round15Set(set = {}, catalog = null) {
  const s = round15Base(CARRIERS, catalog);
  return { ...s, ...set, layout: { ...s.layout, ...(set.layout ?? {}) }, linkage: { ...s.linkage, ...(set.linkage ?? {}) }, lumpPos: { ...(s.lumpPos ?? {}), ...(set.lumpPos ?? {}) } };
}

/** One round-15 build: buildRound14 on round15Set. -> { d, p, failed }. */
export async function buildRound15({ base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const s = round15Set(set, catalog);
  s.lumpPos = { ...(base.params.lumpPos ?? {}), ...s.lumpPos }; // the base's lumps as the fit's start, the probe's where it is mounted
  const b = await buildRound14({ base, set: s, opt, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round15.mjs', basis: `${b.d.selection.basis}; round 15: v22 plus support and containment corrections (round15Set: params.layout.mounts, containment, the carriers)` };
  b.d.assumptions = [...b.d.assumptions.filter((x) => !/^Round 15/.test(x)), round15Note(b.d.structure.clearance)];
  return b;
}

/** The design's round-15 assumption line, with the checks' verdicts. */
export function round15Note(c) {
  const sup = c?.support, con = c?.containment;
  const v = (x, fail) => (!x ? 'not run' : x.passed ? 'passed' : `FAILED (${fail(x)})`);
  return `Round 15: every packed part rests on a carrier through a stated fixing (params.layout.mounts): the ESC, airspeed board, receiver and battery on the nose tray, which stands on a riser on the forward tray; the tail servos, flight controller, regulator and GPS on the aft tray; the motor on the forward plate; the crank encoder readhead on the riser by its maker's 3 x M2 and spacers; the pitot along the nose tray's flange. Support check ${v(sup, (x) => x.unsupported.join(', '))}; containment check ${v(con, (x) => [...x.violations.map((q) => q.solid), ...x.outside_violations].join(', '))}; the fittings left outside add ${con ? `${con.frontal.area_mm2} mm2 at Cd ${con.frontal.Cd}` : 'nothing'} to the pod's frontal drag.`;
}

/**
 * Why --official refuses a design, or null when it may be written: any clash, any part not supported, any part not
 * contained, or a record missing (there is no override).
 */
export function officialRefusal(c) {
  if (!c) return 'no clearance record';
  const why = [];
  if (c.clashes?.length || c.pod_shell?.clashes?.length) why.push(`${c.clashes?.length ?? 0} clashing part pairs, ${c.pod_shell?.clashes?.length ?? 0} against the pod`);
  if (!c.support) why.push('no support record'); else if (!c.support.passed) why.push(`unsupported: ${c.support.unsupported.join(', ')}`);
  if (!c.containment) why.push('no containment record'); else if (!c.containment.passed) why.push(`not contained: ${[...c.containment.violations.map((q) => q.solid), ...c.containment.outside_violations].join(', ')}`);
  if (!c.passed && !why.length) why.push('the clearance check failed');
  return why.length ? why.join('; ') : null;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const official = process.argv.includes('--official');
  const outPath = official ? designPath : resolve(arg('--out') ?? join(ROOT, 'out/sweep/round15/v23.json'));
  if (!official && outPath === designPath) { console.error('data/designs/ is written only with --official'); process.exit(1); }
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound15({ base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  mkdirSync(join(ROOT, 'out/sweep/round15'), { recursive: true });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (d) writeFileSync(join(ROOT, 'out/sweep/round15/v23.failed.json'), JSON.stringify(d));
    process.exit(1);
  }
  const c = d.structure.clearance;
  const verdict = `clearance ${c.clashes.length} clashes, pod ${c.pod_shell.clashes.length}; support ${c.support?.passed ? 'passed' : `FAILED (${c.support?.unsupported.join(', ') ?? 'no record'})`}; containment ${c.containment?.passed ? 'passed' : `FAILED (${c.containment ? [...c.containment.violations.map((q) => `${q.solid} ${q.gap_mm}`), ...c.containment.outside_violations].join(', ') : 'no record'})`}`;
  const refusal = officialRefusal(c);
  if (official && refusal) {
    console.error(`${verdict}: not written to ${designPath} (${refusal})`);
    writeFileSync(join(ROOT, 'out/sweep/round15/v23-refused.json'), JSON.stringify(d));
    process.exit(2);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  mkdirSync(dirname(outPath), { recursive: true });
  if (official) {
    w = writeDesignAtomic(outPath, d);
    writeFileSync(join(ROOT, 'out/sweep/round15/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, clearance: { passed: c.passed, clashes: c.clashes, pod: c.pod_shell.clashes, support: c.support.passed, containment: c.containment.passed, frontal: c.containment.frontal }, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}, Vapp ${d.control.landing.Vapp_m_s}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}; pod ${d.body.width_m} x ${d.body.height_m} @ ${d.body.centre_y_m}; ${verdict} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
