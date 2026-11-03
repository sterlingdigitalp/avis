#!/usr/bin/env node
// Round 14 Larus: v22 = v21 plus assembly clearance corrections (SPEC "Design freeze", "Assembly clearance",
// 2026-11-03). Round 12's pipeline on v21's parameters (round12.mjs ROUND12.set) with the drive planes laid out under
// the clearance rule (sims/avis-shared/model.js clearancePlanes, params.layout) and the plate seats' bushing flanges on
// the plate faces (params.linkage.plates.flange 'face', src/design/linkage.js plateSeatLength). Members, pins, seats and
// bearings follow from the new planes by the existing sizing rules; nothing else is changed: same parts, planform,
// kinematics and controller.
//   node src/design/round14.mjs [--out path] [--set '<json>'] [--opt '<json>'] [--cal path] [--no-suction] [--no-fit]
//   [--no-perf] [--official [--allow-clashes]]
// Exploration builds go to out/sweep/round14/v22.json. data/designs/larus-gl-2.json is written only with --official,
// and only when the design's clearance check (structure.clearance, src/design/clearance.js) passes, unless
// --allow-clashes is given, in which case the record in the design file states every clash as a failure.
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound12, ROUND12 } from './round12.mjs';

// The stated gear fold (model.js gearTrain: link angles from the crank, degrees) and the gearbox panels.
const LINKS_DEG = [157.8, 48.7, 1.3, 110.5];
const PANELS = [
  { name: 'side panel right', kind: 'side', z_mm: 45, y0_mm: -20, h_mm: 29, t_mm: 0.9 },
  { name: 'side panel left', kind: 'side', z_mm: -45, y0_mm: -20, h_mm: 29, t_mm: 0.9 },
  { name: 'top panel', kind: 'top', y_mm: -81, w_mm: 24, t_mm: 1 }, // the horizontal ('top' kind) panel, now under the gears
];

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Round 14 corrections over v21 (ROUND12.set), each forced by a clash the clearance check measures (src/design/
 * clearance.js; the clash ledger is in docs/status/final-gates-larus.txt). Layout only, by the SPEC's layout freedom;
 * every member keeps its own sizing rule, and the parts are v21's except the encoder actuator:
 *  - layout 'clearance', 0.5 mm running clearance (model.js clearancePlanes): the coupler eyes side by side on the crank
 *    pin, 0.5 mm apart and 0.5 mm aft of the crank web; the aft plate 0.5 mm forward of the web past the crank bushing's
 *    flange, so the crank overhang is the clearance agent's v22 candidate's; plate bushing flanges on the plate faces.
 *  - horn stacking: the two horns pass within 11 mm of each other, so the stacks sit at different x: the right boss
 *    forward of its eye, the left boss aft of its eye (horns), each 0.5 mm from its eye, the horn pins flush with the
 *    eyes; the horn bosses 7 mm long (v21 8 mm), so the right boss ends 0.5 mm aft of the aft plate.
 *  - shoulder spacing 36.6 mm (v21 24 mm): both stacks between the housings (an eye or boss overlaps a housing radially
 *    by up to 2.4 mm, so it may not share its x), the forward housings 0.5 mm forward of the aft plate; the crank web
 *    8.7 mm forward of the hinge (crank_x_mm); the rocker hub turned down to 13.6 mm where the coupler eyes pass (they
 *    clear it by 0.5 mm; elsewhere it keeps v21's 14.5 mm and its edge distance) and ended 0.5 mm aft of the aft plate;
 *    a 9 mm collar on the shoulder pin from there to 0.5 mm short of the forward bushing keeps the forward axial stop.
 *  - the rails 20.75 mm below and 4 mm outboard of the hinges (v21 11.5 mm above, in the spar sockets' sweep), 13/8 mm,
 *    seats 31.5 mm (1.2 d0 edge distance), bracket arms 11 mm wide; the aft plate's hull then clears the rocker hubs; the lock flange (1.4 mm)
 *    at the forward plate's aft face, its six screws on a 15 mm arc up and over the rail (lock_arc_deg).
 *  - stage 1 overhung forward of the forward plate and the motor inside the box on the forward plate's aft face
 *    (stage1 'overhung'): gear shaft 2 sits 29 mm from gear shaft 3, inside the DS1-60's 31 mm tip circle, so no shaft
 *    that spans the plates may pass stage 1. The fold is stated (links_deg), chosen against every part.
 *  - the encoder: the catalogue's 8 mm actuator cannot go on the 10 mm crank shaft, so the maker's 10 mm actuator
 *    (BA100), on a stub of the crank shaft forward of stage 1 (encoder 'front'; inside the box it lengthened the box,
 *    and the gears' moved CG pushed the battery and the pod's nose aft until no pod in the fit grid held the gears).
 *  - frame routing: side panels at z +-45 mm, y -20 to 9 mm, the horizontal panel (named 'top panel', its kind) a floor
 *    at y -81 mm (v21's at 56.5 mm touched the boom), trays below the drive (forward -66.5 mm, aft -84 mm), plate posts 16 mm wide (the boom posts
 *    are longer). The pod follows by the fit rule, which now keeps every packed part, tip circle and the motor can the
 *    running clearance inside the loft (model.js layoutParts, fit.js protrusions).
 */
const R12 = ROUND12.set, RL = R12.linkage, RA = RL.airframe;
export const ROUND14 = {
  set: {
    layout: { rule: 'clearance', running_mm: 0.5, basis: 'SPEC Design freeze, Assembly clearance (2026-11-03); sims/avis-shared/model.js clearancePlanes', crank_x_mm: 8.7, eye_gap_mm: 0.5, horns: { R: 'fwd', L: 'aft' }, horn_gap_mm: 0.5, horn_pin_past_mm: 0, encoder: 'front', stage1: 'overhung', links_deg: LINKS_DEG },
    joints: { ...R12.joints, shoulderSpan_mm: 36.6 },
    parts: { encoderMagnet: 'sensor-rls-orbis-actuator-10mm' },
    linkage: {
      plates: { ...RL.plates, flange: 'face' },
      rocker: { ...RL.rocker, boss_len_mm: 7, hub_relief_od_mm: 13.6, hub_clear_plate: true, hub_collar_od_mm: 9 },
      airframe: {
        ...RA,
        plates: { ...RA.plates, post_w_mm: 16 },
        rail: { ...RA.rail, od_mm: 13, dz_mm: 4, dy_mm: -20.75, seat_D_mm: 31.5, arm_b_mm: 11, lock_at: 'forward', lock_screws: 6, lock_r_mm: 15, lock_arc_deg: [40, 172], lock_t_mm: 1.4 },
        tray: { y_mm: -84 },
        aftTray: { y_mm: -84 },
        panels: { ...RA.panels, list: PANELS },
      },
    },
  },
};

/** Round 14's --set: ROUND14.set, then `set` over it (layout and linkage merged by block). */
export function round14Set(set = {}) {
  const s = ROUND14.set;
  return { ...s, ...set, layout: { ...s.layout, ...(set.layout ?? {}) }, linkage: { ...s.linkage, ...(set.linkage ?? {}) } };
}

/** One round-14 build: buildRound12 on round14Set. -> { d, p, failed }. */
export async function buildRound14({ base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const b = await buildRound12({ base, set: round14Set(set), opt, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round14.mjs', basis: `${b.d.selection.basis}; round 14: v21 plus assembly clearance corrections (ROUND14.set: params.layout, plates.flange)` };
  b.d.assumptions = [...b.d.assumptions.filter((x) => !/^Round 14/.test(x)), round14Note(b.d.params.layout, b.d.structure.clearance)];
  return b;
}

/** The design's round-14 assumption line, with the clearance check's verdict. */
export function round14Note(layout, c) {
  const verdict = !c ? 'not run' : c.passed ? 'passed' : `FAILED: ${c.clashes.length} part pairs under ${c.required_mm} mm${c.pod_shell?.clashes.length ? `, ${c.pod_shell.clashes.length} against the pod` : ''}`;
  const lay = layout.stage1 === 'overhung' ? ` Layout (SPEC layout freedom, ROUND14.set): horn stacks ${Object.entries(layout.horns ?? {}).map(([t, w]) => `${t} ${w}`).join(', ')} of their eyes between shoulder housings spread to the stacks' length; the aft plate just forward of the crank web; rails below the hinges; stage 1 overhung forward of the forward plate with the motor inside the box; the encoder (10 mm actuator) on a crank stub forward of stage 1; the stated gear fold ${JSON.stringify(layout.links_deg)} deg.` : '';
  return `Round 14: the drive planes keep ${layout.running_mm} mm running clearance along the crank pin and at the plate-adjacent stages (model.js clearancePlanes); plate bushing flanges on the plate faces; rocker hubs that far clear of the shoulder bushings;${lay} Assembly clearance check ${verdict} (structure.clearance).`;
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const official = process.argv.includes('--official');
  const outPath = official ? designPath : resolve(arg('--out') ?? join(ROOT, 'out/sweep/round14/v22.json'));
  if (!official && outPath === designPath) { console.error('data/designs/ is written only with --official'); process.exit(1); }
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound14({ base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
  if (failed) {
    console.error(`build failed: ${failed}`);
    if (!official && d) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  const c = d.structure.clearance;
  const clash = `clearance ${c.passed ? 'passed' : `FAILED (${c.clashes.length} part pairs; worst ${c.clashes[0]?.gap_mm} mm ${c.clashes[0]?.a} / ${c.clashes[0]?.b}${c.pod_shell.clashes.length ? `; pod: ${c.pod_shell.clashes.join(', ')}` : ''})`}`;
  if (official && !c.passed && !process.argv.includes('--allow-clashes')) {
    console.error(`${clash}: not written to ${designPath} (add --allow-clashes to write it with the failures recorded)`);
    mkdirSync(join(ROOT, 'out/sweep/round14'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round14/v22-refused.json'), JSON.stringify(d));
    process.exit(2);
  }
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  mkdirSync(dirname(outPath), { recursive: true });
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round14'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round14/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, clearance: { passed: c.passed, clashes: c.clashes, pod: c.pod_shell.clashes }, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}, Vapp ${d.control.landing.Vapp_m_s}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}; ${clash} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
