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

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Round 14 corrections over v21 (ROUND12.set), each forced by a clash the clearance check measures on v21:
 *  - layout 'clearance' with 0.5 mm running clearance: the coupler eyes side by side on the crank pin, 0.5 mm apart
 *    and 0.5 mm aft of the crank web (v21: 1 mm into each other); the aft plate moved forward so the crank bushing's
 *    1 mm flange sits on its aft face 0.5 mm from the web (v21: the web 0.2 mm from the plate); the crank stage moved
 *    forward of the aft-plate bosses and journals on its two shafts, and the forward plate forward of stage 1's
 *    journals, by 0.5 mm (v21: the bosses 7 mm into the crank wheel and 3.2 mm into the stage-4 pinion).
 *  - plates.flange 'face': the plate bushings' flanges on the plate faces, so a boss is as long as the bushing body
 *    and the seat holds the body (b1 - b2), as the SPEC reads clash 3.
 */
export const ROUND14 = {
  set: {
    layout: { rule: 'clearance', running_mm: 0.5, basis: 'SPEC Design freeze, Assembly clearance (2026-11-03); sims/avis-shared/model.js clearancePlanes' },
    linkage: { plates: { ...ROUND12.set.linkage.plates, flange: 'face' } },
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
  return `Round 14: the drive planes keep ${layout.running_mm} mm running clearance along the crank pin and at the plate-adjacent stages (model.js clearancePlanes); plate bushing flanges on the plate faces; rocker hubs that far clear of the shoulder bushings; assembly clearance check ${verdict} (structure.clearance).`;
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
