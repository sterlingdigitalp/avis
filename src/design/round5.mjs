#!/usr/bin/env node
// Round 5 Larus: v12 made real under the SPEC rulings after physics round 4, written atomically through io.js.
//   node src/design/round5.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
// --base defaults to out/sweep/round5/v12-base.json (v12 as committed in 33c598c). --out other than data/designs/ is
// for exploration: nothing else is written then except that file.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, gearRatioOf } from './build.js';
import { trainCrankRating } from './trim.js';
import { validateDesign } from './schema.js';
import { completeDesign, measurePerformance } from './round3.mjs';
import { ROUND4_OPT } from './round4.mjs';
import { fitBody, packDesign, strokeAsymmetry } from './fit.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/**
 * Round 5 changes from v12, each against a ruling or a measured failure (out/sweep/round5/, docs/status/round5-levers.json):
 *  - Fixed stroke at 38 deg (a pinned rocker, no slot, no stroke servo): on v12's slotted rocker the coupler force's
 *    component along the slots reached 1849 N on the yoke, 17.6 N m at the stroke servo's most favourable horn against
 *    1.53 N m (out/sweep/round5/v12-stroke-servo-hold.json). No servo of this class holds it.
 *  - Bearings by joint (joints): iglidur G bushings on the shoulders, crank, slow gear shafts and coupler eyes, an SKF
 *    619/5-2Z pair on the fast shaft; each with its published static rating and the load path in structure.joints.
 *  - The crank governor and the drive-current clamp in our controller on the flight controller (governor), at 960 Hz
 *    from the RLS Orbis crank encoder and the F405-WMO's voltage sense, all in the BOM.
 *  - Servo supply: the pack charged to 4.10 V per cell so every pack voltage lies in the HV6160's published rows
 *    (4.8-8.2 V); M2 counts the derated capacity. The 2200 mAh pack carries the derate at the corners.
 *  - The arm wing does not twist (elbowTwist 0): nothing in the BOM twists it.
 *  - A complete flight controller (Matek F405-WMO, 9 g, with its voltage and current sense and 5 V BEC) in place of the
 *    ARK V6X module, and an AM32 ESC (Holybro Tekko32 F4 45A, 5.8 g) whose firmware documents complementary PWM.
 *  - An all-one-material-per-mesh train with module-1 Duracon first stages (ratio 28.09, crank rating 14.16 N m), and
 *    the energy controller's kTE 1.5 (rough-air altitude).
 */
export const ROUND5 = {
  fixedStroke: true,
  amp_deg: 38, // the fixed half-swing: the stroke speed v12 reached with its slot (41 deg x 3.16 Hz) at a frequency the motor holds
  elbowTwist: 0,
  escCurrentClamp_A: null, // the clamp is the flight controller's now (governor.clamp_A)
  becFrom: 'fc',
  avionicsExtra_A: 0.065,
  avionicsExtraNote: 'RLS Orbis BR10 encoder, typical 65 mA (BRD01 datasheet)',
  chargeLimit: { V_per_cell: 4.1, published_fraction: 0.861, source: 'Battery University BU-808 Table 4 read linearly between its 4.06 V (81%) and 4.13 V (90%) rows (our arithmetic); BU-409 Table 2 gives 90% at 4.10 V; the lower is used' },
  // F405-WMO current sense: 40 A/V (ArduPilot BATT_AMP_PERVLT 40) on a 0-3.3 V input, read by the F405's 12-bit ADC.
  currentSense: { part: 'fc-matek-f405-wmo', lsb_A: +((40 * 3.3) / 4096).toFixed(5), basis: '40 A/V over 3.3 V at 12 bits (STM32F405 ADC, 12-bit; the ST page was not readable this round)' },
  joints: { shoulder: 'bushing-igus-GFM-0608-06', shoulderSpan_mm: 24, crankShaft: 'bushing-igus-GFM-0810-10', shafts: ['bushing-igus-GFM-0810-06', 'bushing-igus-GFM-0810-06', 'bearing-skf-619-5-2z'], couplerEyes: 'bushing-igus-GFM-0608-06' },
  governor: {
    location: 'flight controller', rate_Hz: 960, clamp_A: 34,
    encoder: { id: 'sensor-rls-orbis-br10', bits: 14, window: 2, update_Hz: 50000 },
    // F405-WMO battery sense: 1k:20k divider (x21) on the 3.3 V, 12-bit ADC; a 2 ms first-order filter in our code.
    voltage: { id: 'fc-matek-f405-wmo', lsb_V: +((21 * 3.3) / 4096).toFixed(5), tau_s: 0.002 },
    path: 'encoder (SPI, 50 kHz updates) and battery sense read at each 960 Hz tick; duty to the ESC by DShot600 (26.7 us frame); applied at the next tick (one 1.04 ms period of latency, held between ticks)',
  },
  escActiveFreewheeling: { esc: 'esc-holybro-tekko32-f4-45a-am32', quote: "AM32 'Complementary PWM: Applies PWM to the complementary output during the PWM off time. This acts as a brake slowing the motor when going from a higher duty cycle to lower duty cycle'; default on (EEPROM byte 20 = 1)", source: 'https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/ESC-Settings-Explained' },
  parts: {
    strokeServo: null,
    fc: 'fc-matek-f405-wmo',
    esc: 'esc-holybro-tekko32-f4-45a-am32',
    crankEncoder: 'sensor-rls-orbis-br10',
    encoderMagnet: 'sensor-rls-orbis-actuator-8mm',
    // Stage 2 all Duracon (DS1-18 / DS1-60): the steel MSGA1-18 on MC901 PS1-60 was a mixed mesh whose plastic member
    // publishes no allowable surface stress (KHK rates its plastic gears in bending only), so it could not be judged.
    // Round 5 train (all four stages one material each, ratings by KHK): stage 1 DS1-18 / DS1-32 and stage 2 DS1-20 /
    // DS1-64 (Duracon), then the two steel MSGA1-18 / MSGB1-40 stages; ratio 28.09 (v12 24.69) and crank rating 14.16
    // N m (v12 11.60). The higher ratio buys hand-launch height and endurance at the corners; the module-1 Duracon
    // stages replace the DS0.8 pair and the steel-on-MC901 stage, whose plastic member publishes no surface allowable.
    gears: ['gear-khk-ds1-18', 'gear-khk-ds1-32', 'gear-khk-ds1-20', 'gear-khk-ds1-64', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'],
    // The 2200 mAh 2S pack (+40 g): with the charge limit the 1300 pack's derated capacity left M2 at 9.5-9.9 min at
    // the heavy corners.
    battery: 'batt-gensace-2200-2s-60c',
  },
};

/** Round-5 options: round 4's, with a bearing allowance for the calm-air envelope like the other linkage ratings. */
export const ROUND5_OPT = { ...ROUND4_OPT, allowance: { ...ROUND4_OPT.allowance, bearing: 0.9 }, kTE: 1.5 };

/** Parameters: v12's, then ROUND5, then --set (parts merged); gear ratio and crank rating from the train. */
export function round5Params(base, set, catalog) {
  const p = { ...base.params, ...ROUND5, ...set, parts: { ...base.params.parts, ...ROUND5.parts, ...(set.parts ?? {}) } };
  p.gearRatio = gearRatioOf(p.parts.gears, catalog);
  p.crankRating = trainCrankRating(buildDesign(p, catalog).structure.gear_stages);
  return p;
}

/** Whether a calibration file was fitted on this wing: span, area, twist weights and stroke kind (V5 records them). */
export function calibrationMatches(cal, d) {
  const w = cal.wing;
  if (!w || Math.abs(w.span_m - d.geometry.span_m) > 1e-6 || Math.abs(w.area_m2 - d.geometry.area_m2) > 1e-6) return false;
  const tw = d.wing.segments.map((g) => [g.twist_in, g.twist_out]);
  if (!w.twist_weights || JSON.stringify(w.twist_weights) !== JSON.stringify(tw)) return false;
  return !!w.fixed_stroke === !!d.drive.fixedStroke;
}

/** The design's stated assumptions for round 5: v12's, less what round 5 replaced, plus round 5's own. */
export function round5Assumptions(base, d) {
  const gone = /^Twist servo at the elbow|^Twist horn linkage|^Stroke envelope measured|^Servos run straight off|^Crank speed governor|^Drive-side current clamp|^One crank pin|^Rigid elbow|^Derived parts|^Bearings placed|^Motor-current limiter|^Round 5/;
  const gov = d.drive.governor;
  return [...new Set([...(base.assumptions ?? []).filter((a) => !gone.test(a)), ...(d.assumptions ?? []),
    `Twist servo at the elbow drives the hand wing through a ${d.structure.twist_rod?.id ?? 'torque'} tube; M7 checks its torsion against the published minimum yield through von Mises (tau = sigma_y / sqrt 3).`,
    'Twist horn linkage ratio is geometry: the servo\'s published torque and speed are scaled by it; linkage backlash and compliance not modelled.',
    ...(d.electrical.servo_source === 'battery' ? [`Round 5: servos run straight off the ${d.battery.cells}S pack, charged to ${d.battery.charge_limit_V_per_cell ?? 4.2} V per cell, so the pack presents at most ${d.electrical.servo_supply.current_V} V: torque and speed from the published rows at ${d.electrical.servo_supply.performance_V} V, stall current at ${d.electrical.servo_supply.current_V} V, all inside the published rows (no scaling). ${d.battery.charge_basis ?? ''}`] : []),
    ...(gov ? [`Round 5: the crank-speed governor and the ${gov.clamp_A} A drive-current clamp run in our controller on the flight controller at ${gov.rate_Hz} Hz, reading the crank angle from the ${gov.encoder.id} (${gov.encoder.bits}-bit; speed from ${gov.encoder.window}-sample differences) and the battery voltage from the FC's sense input (${gov.voltage.lsb_V} V resolution); the duty computed at one tick is applied at the next (one period of latency, held in between). The ESC is assumed to apply the commanded duty linearly; the clamp is a duty limit from measured speed and voltage with the motor's published Kv and resistance (its resistance at temperature is not modelled).`] : []),
    'Round 5: the twist schedule takes the crank phase from the same encoder (last sample advanced at the measured speed); the servo PWM refresh is not modelled.',
    ...(d.drive.pinLayout === 'shared' ? ['One crank pin drives both couplers; the left wing flies its own mirrored linkage (left-right stroke difference in design.packing).'] : []),
    ...(d.drive.fixedStroke ? [`Round 5: fixed stroke, half-swing ${(d.drive.ampMax / DEG).toFixed(1)} deg: the coupler pins are pinned in the rockers (no slot, no stroke servo); thrust and lift are set by stroke frequency and twist.`] : []),
    'Rigid elbow: the outer spar is bonded into the inner one (spigot); round 5 removes the arm wing\'s own twist (twist weight 0 at the elbow), so the arm wing flies at its fixed incidence and only the hand wing twists.',
    ...(d.params.derivedParts ? ['Derived parts: pitot-static probe (bases in the structure list); pitot pressure lines counted in the 15% margin with the wiring.'] : []),
    ...(d.structure.joints ? ['Round 5: every drive joint has its own rated bearing, bushing or rod end (structure.joints states which force each reacts); loads combine at every step in the flight model, as magnitudes (directions not credited). The motor\'s own bearings carry the first pinion and are reported, not checked (no published rating). Shafts, pins and the plates\' bearing seats are not checked.'] : []),
    'Stroke envelope measured by flight test in calm air at a fraction of the gear, spar, coupler, servo and bearing ratings (control.envelopeAllowance: a rough-air allowance, a design choice); M7 judges full ratings on the flown loads.'])];
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round5/v12-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = { ...ROUND5_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  const t0 = Date.now();
  const p0 = round5Params(base, set, catalog);
  const fit = process.argv.includes('--no-fit') ? null : fitBody(p0, catalog);
  if (fit?.failed) { console.error(`body fit failed: ${fit.failed} ${JSON.stringify(fit.best)}`); process.exit(1); }
  const p = fit ? fit.params : p0;
  const d = buildDesign(p, catalog);
  if (fit) {
    const pk = packDesign(d, catalog);
    const asym = strokeAsymmetry(d, catalog);
    d.packing = { checker: 'sims/avis-shared/model.js prepare/layoutParts/fitReport (film workstream), used as is', body_m: { width: fit.body.W, height: fit.body.H, centre_y: fit.body.Y }, protrusions: pk.issues, residual_cg_shift_m: pk.cgShift_m, slides: pk.model.fit.issues.filter((i) => i.shift).map((i) => i.text), lump_positions: p.lumpPos, crank_axis_y_m: pk.model.fourBar.O2[1], left_right_stroke_difference_deg: Object.fromEntries(Object.entries(asym).map(([k, v]) => [k, +(v * 180 / Math.PI).toFixed(2)])), pin_layout: d.drive.pinLayout ?? 'symmetric (assumed)', ...(d.drive.fixedStroke ? { placeholder: 'the film checker places a stroke servo unconditionally: a massless placeholder keeps a servo-sized box free that nothing fills (src/design/fit.js packingView)' } : {}) };
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
  d.round5_options = opt;
  d.selection = { pipeline: 'src/design/round5.mjs', basis: 'explicit round-5 parameters (ROUND5, ROUND5_OPT in the script and --set/--opt) on Larus v12; screen not rerun', set, opt: arg('--opt') ? JSON.parse(arg('--opt')) : {} };
  d.assumptions = round5Assumptions(base, d);
  if (!process.argv.includes('--no-perf')) d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  let w = null;
  if (official) {
    w = writeDesignAtomic(outPath, d);
    mkdirSync(join(ROOT, 'out/sweep/round5'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round5/build.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, calibration: d.calibration, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, S ${d.geometry.area_m2.toFixed(3)} m2, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''}${d.calibration?.provisional ? ' (calibration provisional)' : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
