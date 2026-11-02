#!/usr/bin/env node
// Round 4 Larus: the round-3 bird with the round-4 levers, trimmed and flight-tested at the converged time step,
// written atomically through io.js.
//   node src/design/round4.mjs [--out path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
// --out other than data/designs/ is for exploration: nothing else is written then except that file.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, gearRatioOf } from './build.js';
import { trainCrankRating } from './trim.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/**
 * Round 4 changes from the round-3 Larus (v9), each against a measured failure (out/sweep/round4/, report in
 * out/sweep/round4/final/):
 *  - HV servos straight on a 2S pack (servoSupply 'battery'): at the 5 V BEC the HV6160 through 1.6:1 gave the hand
 *    wing 7.5 rad/s no-load, the twist lagged the schedule above 2.8 Hz (29.8 of 38 deg at 3.4 Hz), the outer wing ran
 *    at the wrong incidence and the climb fell from 1.25 to 0.39 m/s while crank torque, root bending and motor current
 *    rose. At 6.62 V (20% charge, sagging at the pack's rating) the same servo gives 10.1 rad/s and 0.96 N m at the
 *    wing. The 2S pack is 18 g heavier than the 3S 800 and the 2S-rated ESC 13 g lighter.
 *  - Braking at the ESC's rating (its leaflet states active freewheeling): the model's unsourced half-rating floor let
 *    the wing back-drive the crank to twice its speed; root bending fell from 41-44 to 28-30 N m, twist torque from
 *    0.89 to 0.61 N m.
 *  - Cobra C-2820/14 (840 rpm/V, 36 A continuous, +33 g): 25% more torque per amp and braking torque; motor rms from
 *    38 to 28 A at the same stroke.
 *  - 2.0 m span (round 3's 2.0 m variant failed through a numerical blow-up in the frozen-body cycles, fixed in
 *    flight.js): 17% less wing loading; M1 height lost at the worst corner 2.27 -> 0.82 m.
 *  - Four-stage train (crank rating 9.28 -> 11.60 N m, ratio 19.75 -> 24.69, +62 g) and a drive-side current clamp at
 *    34 A: the gust peaks on the crank (9.9-13.8 N m) do not fit the three-stage train at any envelope that keeps M4.
 *  - Titanium torque tube for the twist (the 3 mm carbon rod publishes no shear strength, so M7 could not check it).
 *  - Rigid elbow (elbowGain 0) and a 48 deg stroke limit: nothing in the BOM made the 1.5x hand-wing stroke.
 *  - One crank pin drives both wings: the left wing's own stroke is flown (pinLayout 'shared').
 *  - The pod sized by the film workstream's packing check (fit.js), lumps where the parts fit; derived parts for the
 *    stroke-servo linkage and the pitot-static probe; bearings placed by the drive.
 */
export const ROUND4 = {
  span_m: 2.0,
  elbowGain: 0,
  ampMax_deg: 48,
  escCurrentClamp_A: 34,
  servoSupply: 'battery',
  pinLayout: 'shared',
  derivedParts: true,
  bearingLayout: 'auto',
  parts: {
    battery: 'batt-gensace-1300-2s-45c',
    esc: 'esc-hobbywing-skywalker-30a-v2-mini',
    motor: 'motor-cobra-c-2820-14',
    twistRod: 'mtube-tigertitanium-ti3al2.5v-6.35x5.54',
    gears: ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-30', 'gear-khk-msga1-18', 'gear-khk-ps1-60', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'],
  },
  // Read from the ESC's catalogue source (the SkyWalker V2 leaflet, page 1 feature row); the catalogue entry does not
  // carry it yet. With it the drive brakes at the ESC's rating instead of the model's unsourced half rating.
  escActiveFreewheeling: {
    esc: 'esc-hobbywing-skywalker-30a-v2-mini',
    family: 'esc-hobbywing-skywalker-', // the leaflet covers the whole SkyWalker V2 line (15 A to 100 A)
    quote: '"DEO Technology/ Active Freewheeling" (feature row, page 1)',
    source: 'https://cdn.shopify.com/s/files/1/0109/9702/files/SKYWALKER_V2_mini_Leaflet_EN.pdf',
  },
};

/**
 * Round-4 controller and envelope options. dt: the converged flight step (M4 worst root bending changes 1.4%, twist
 * torque 2.2% and worst roll 8% from 1/960 s to 1/1920 s; 1/480 s overestimated root bending by 12%). Vrough 15:
 * kinetic energy for sinking air (M4 95 -> 99 of 100 from 13 to 15 m/s). Gear allowance 0.75: the calm-air stroke may
 * use 75% of the train's rating, the rest is left for gusts. motorCurrentLimit and gla were measured and are off
 * (see the report): neither moved a gate.
 */
export const ROUND4_OPT = { Vrough: 15, dt: 1 / 960, allowance: { gear: 0.75, spar: 0.9, coupler: 0.9, servo: 0.9 }, motorCurrentLimit: null, gla: null };

/** Trim, control table, free-flight trim, controller options and the flight-test envelope: round 3's pipeline with the
 * round-4 options (dt, motorCurrentLimit, gla, roughMaxBank_deg) it now takes. Mutates d. */
export const completeDesign4 = (d, args) => completeDesign(d, args);

/** Parameters: the v9 design's, then ROUND4, then --set; gear ratio and crank rating from the train. */
export function round4Params(base, set, catalog) {
  const p = { ...base.params, ...ROUND4, ...set, parts: { ...base.params.parts, ...ROUND4.parts, ...(set.parts ?? {}) } };
  p.gearRatio = gearRatioOf(p.parts.gears, catalog);
  p.crankRating = trainCrankRating(buildDesign(p, catalog).structure.gear_stages);
  return p;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? designPath, 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND4_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round4Params(base, set, catalog);
  // The pod sized to hold the parts (the film workstream's packing check), lumps where the parts fit.
  const fit = process.argv.includes('--no-fit') ? null : fitBody(p0, catalog);
  if (fit?.failed) { console.error(`body fit failed: ${fit.failed} ${JSON.stringify(fit.best)}`); process.exit(1); }
  const p = fit ? fit.params : p0;
  const d = buildDesign(p, catalog);
  if (fit) {
    const pk = packDesign(d, catalog);
    const asym = strokeAsymmetry(d, catalog);
    d.packing = { checker: 'sims/avis-shared/model.js prepare/layoutParts/fitReport (film workstream), used as is', body_m: { width: fit.body.W, height: fit.body.H, centre_y: fit.body.Y }, protrusions: pk.issues, residual_cg_shift_m: pk.cgShift_m, slides: pk.model.fit.issues.filter((i) => i.shift).map((i) => i.text), lump_positions: p.lumpPos, crank_axis_y_m: pk.model.fourBar.O2[1], left_right_stroke_difference_deg: Object.fromEntries(Object.entries(asym).map(([k, v]) => [k, +(v * 180 / Math.PI).toFixed(2)])), pin_layout: d.drive.pinLayout ?? 'symmetric (assumed)' };
  }
  const calP = join(ROOT, `data/calibration/${d.name}.json`);
  if (existsSync(calP)) {
    const cal = JSON.parse(readFileSync(calP, 'utf8'));
    // The multipliers belong to the wing V5 fitted them on (V5 records it from round 4); any other wing is provisional.
    const same = cal.wing && Math.abs(cal.wing.span_m - d.geometry.span_m) < 1e-6 && Math.abs(cal.wing.area_m2 - d.geometry.area_m2) < 1e-6;
    d.calibration = { file: `data/calibration/${d.name}.json`, source: cal.source, multipliers: cal.multipliers, fitted_wing: cal.wing ?? null, ...(same ? {} : { provisional: true, provisional_note: 'multipliers fitted on another wing (or not recorded); V5 must be rerun on this one' }) };
  }
  const r = await completeDesign4(d, { opt, sched: arg('--sched') ? JSON.parse(arg('--sched')) : null });
  if (!r.ok) {
    console.error(`build failed: ${r.failed}`);
    if (!official) writeFileSync(`${outPath}.failed.json`, JSON.stringify(d));
    process.exit(1);
  }
  d.round4_options = opt;
  d.selection = { pipeline: 'src/design/round4.mjs', basis: 'explicit round-4 parameters (ROUND4, ROUND4_OPT in the script and --set/--opt) on the round-3 Larus v9; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = [...new Set([...(base.assumptions ?? []).filter((a) => !/^Twist servo at the elbow|^Twist horn linkage|^Stroke envelope measured|^Servos run straight off|^Crank speed governor|^Drive-side current clamp|^One crank pin|^Rigid elbow|^Derived parts|^Bearings placed|^Motor-current limiter/.test(a)), ...(d.assumptions ?? []),
    d.structure.twist_rod?.kind === 'metal'
      ? `Twist servo at the elbow drives the hand wing through a ${d.structure.twist_rod.id} torque tube; M7 checks its torsion against the published minimum yield through von Mises (tau = sigma_y / sqrt 3).`
      : 'Twist servo at the elbow drives the hand wing through a 3 mm carbon torque rod; the rod\'s torsional strength is not published (stress reported, not checked).',
    'Twist horn linkage ratio is geometry: the servo\'s published torque and speed are scaled by it; linkage backlash and compliance not modelled.',
    ...(d.electrical.servo_source === 'battery' ? [`Servos run straight off the ${d.battery.cells}S pack (HV servos; no regulator): torque and speed from the published rows at ${d.electrical.servo_supply.performance_V} V, stall current at the full-charge ${d.electrical.servo_supply.current_V} V (above the top published row it is scaled by the voltage ratio).`] : []),
    'Crank speed governor: a drive-firmware function (ESC or flight controller with the crank encoder); the catalogue ESC publishes no governor mode.',
    ...(Number.isFinite(d.drive.escCurrentClamp_A) ? [`Drive-side current clamp at ${d.drive.escCurrentClamp_A} A in the governor (duty <= (E + I R)/V from commutation speed and input voltage): a firmware function the catalogue ESC does not publish.`] : []),
    ...(d.control.motorCurrentLimit ? ['Motor-current limiter on the flight controller reads a power module (battery current and voltage); no power module is in the catalogue, so its mass is not in the BOM.'] : []),
    ...(d.drive.pinLayout === 'shared' ? ['One crank pin drives both couplers; the left wing flies its own mirrored linkage (left-right stroke difference in design.packing).'] : []),
    ...((d.params.elbowGain ?? 0.5) === 0 ? ['Rigid elbow: the outer spar is bonded into the inner one (spigot); the arm wing\'s own share of the twist (twist weights 0-0.3 inboard of the elbow) has no mechanism in the BOM and is a kinematic assumption.'] : []),
    ...(d.params.derivedParts ? ['Derived parts: stroke-servo pushrods and yoke, pitot-static probe (bases in the structure list); pitot pressure lines counted in the 15% margin with the wiring.'] : []),
    ...(d.drive.bearings ? ['Bearings placed by the drive (drive.bearings); their load ratings are not checked (shoulder pivots react the coupler force).'] : []),
    'Stroke envelope measured by flight test in calm air at a fraction of the gear, spar, coupler and servo ratings (control.envelopeAllowance: a rough-air allowance, a design choice); M7 judges full ratings on the flown loads.'])];
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round4'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round4/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(0)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
