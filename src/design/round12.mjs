#!/usr/bin/env node
// Round 12 Larus: round 11's pipeline (round11.mjs) plus the round-11 rulings (SPEC "Rulings after physics round 11"):
// the controller's landing law as design (control.landing, src/physics/control.js landingLaw), and gear life factors
// at a declared 200-flight design life (src/physics/gears.js lifeFactors, src/design/gearing.js rateStages).
//   node src/design/round12.mjs [--out path] [--base path] [--set '<json params>'] [--opt '<json options>'] [--no-perf]
//   [--no-fit] [--cal path] [--no-suction] (keep the 0.6 default eta_s)
// --out other than data/designs/ is for exploration.
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { validateDesign } from './schema.js';
import { buildRound11, round11Assumptions } from './round11.mjs';
import { trimGlide } from './trim.js';
import { buildBird } from '../physics/flight.js';
import { robustnessCorners } from '../physics/scenarios.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;

/**
 * Round 12 constants, declared before the gates were flown on them.
 *  - landing: the approach speed is VREF = 1.23 x the reference stall speed (14 CFR 25.125(b)(2)(i): "not less than
 *    1.23 VSR0"), with the stall speed taken as the slowest converged steady parked-wing glide over every robustness
 *    corner (conservative: the true 1-g stall lies below it); the stall guard sits at 1.1 x that speed. Gains of the law
 *    (control.js landingLaw): speed hold kV 0.03 rad per m/s (about twice the glide table's own d theta / dV), kIV
 *    0.02; exponential flare with tau 1.5 s from tau x the approach sink, touchdown sink command 0.3 m/s, kS 0.3 rad per
 *    m/s of climb-rate error, kIS 0.1.
 */
export const ROUND12 = {
  landing: { vrefFactor: 1.23, vrefBasis: '14 CFR 25.125(b)(2)(i): VREF not less than 1.23 VSR0', floorFactor: 1.1, kV: 0.03, kIV: 0.02, iMax_rad: 0.1, tau_s: 1.5, sinkTd_m_s: 0.3, kS: 0.3, kIS: 0.1, kFloor: 0.05, dThetaMin_rad: -0.2, dThetaMax_rad: 0.25, resolution_m_s: 0.05 },
  /**
   * v21's parameters (docs/status/round12-levers.json), over round 11's (and so round 10's rules on the v18 base):
   *  - a 3S pack (Gens ace 2200 mAh 3S 35C) behind a Pololu D36V50F7 7.5 V regulator for all four servos (SPEC "Servo
   *    voltage": the BEC in the BOM with its mass), so no charge limit; tail servos Hitec MD89MW (stall current published
   *    at 4.8, 6 and 7.4 V); the twist servos stay MKS HV6160.
   *  - motor Scorpion SII-3008-1090 (95 g, 26 A continuous, 5 mm shaft) on a ratio-41.67 train: DS1-20/DS1-60,
   *    MSGA1-18/SSAG1-40J10, MSGA1-18/MSGB1-45 twice (scratchpad r12/trainsearch12.mjs --no-mixed: the lightest
   *    module-1 KHK train for 15.9 N m at 4 Hz with no steel-on-acetal mesh, 432.5 g of gears; the lighter trains all
   *    mesh steel with Duracon, which the SPEC's mixed-mesh ruling cannot judge without published contact allowables).
   *  - gear ratings at crank_Hz 4 (the flown peak crank rate must stay within it) and a 200-flight design life of
   *    1500 s flights (at or above every M2 endurance measured), the SPEC ruling after round 11.
   *  - the crank governor's drive clamp at 26 A (flight-controller code: it caps the train's peak torque in gusts).
   *  - linkage and airframe members resized to their own loads at the 3S stroke speeds (each failing or thin member of
   *    the round-12 member probes scaled up; scratchpad r12/b/sc40cl26-members.txt, sc40m3-members.txt).
   */
  set: {
    parts: { motor: 'motor-scorpion-sii-3008-1090', battery: 'batt-gensace-2200-3s-35c', tailServo: 'servo-hitec-md89mw', servoRegulator: 'regulator-pololu-d36v50f7', gears: ['gear-khk-ds1-20', 'gear-khk-ds1-60', 'gear-khk-msga1-18', 'gear-khk-ssag1-40j10', 'gear-khk-msga1-18', 'gear-khk-msgb1-45', 'gear-khk-msga1-18', 'gear-khk-msgb1-45'] },
    // The fastest intermediate shaft (8 mm, about 3300 rpm at the rating crank frequency) runs in SKF 628/8-2Z ball
    // bearings: an iglidur G bushing's surface speed would be exceeded there.
    joints: { shafts: ['bushing-igus-GFM-0810-06', 'bushing-igus-GFM-0810-06', 'bearing-skf-628-8-2z'] },
    servoSupply: 'regulator',
    chargeLimit: null,
    gearRating: { crank_Hz: 4, life: { flights: 200, flight_s: 1500 } },
    linkage: {
      // Plate edge distance 11.3 mm: the 628/8's 16 mm seat needs e >= 1.2 d0 (SPEC ruling after round 8).
      plates: { edge_mm: 11.3 },
      crank: { shaft_id_mm: 5, web_t_mm: 6 },
      rocker: { arm_t_mm: 6, boss_od_mm: 23 },
      coupler: { body_d_mm: 5 },
      airframe: {
        plates: { edge_mm: 11.3 },
        rail: { od_mm: 12, id_mm: 8, lock_screws: 3, arm_t_mm: 8, sleeve_len_mm: 13 },
        panels: { material: 'metal-al-6061-t6', flange_mm: 24, list: [{ name: 'side panel right', kind: 'side', z_mm: 31, y0_mm: 3, h_mm: 21.5, t_mm: 1.2 }, { name: 'side panel left', kind: 'side', z_mm: -31, y0_mm: 12.8, h_mm: 11.7, t_mm: 2.2 }, { name: 'top panel', kind: 'top', y_mm: 56.5, w_mm: 20, t_mm: 1 }] },
      },
    },
  },
  governorClamp_A: 26,
  /**
   * The calm-air stroke envelope keeps 0.6 of every rated linkage and airframe member's rating (members ratio, opt-in,
   * src/design/trim.js membersRatio): the round-12 probes measured rough-air member peaks up to 1.58 x the calm edge.
   */
  opt: { allowance: { gear: 0.75, spar: 0.9, coupler: 0.9, servo: 0.9, bearing: 0.9, member: 0.6 } },
  /** The hand-launch stroke frequency (control.launchLaw): the lowest worst-corner height loss of a 2.4-3.0 Hz sweep. */
  launchLaw: { pitch: 'climb', freq_Hz: 2.6, basis: 'scratchpad r12/b/sc40m4-m1.log: launch stroke 2.4 / 2.6 / 2.8 / 3.0 Hz against the envelope top; flapping faster at the 8 m/s release stalls the outer wing' },
};

/** Round 12's --set over v21's parameters: parts and linkage blocks merged, everything else replaced. */
export function round12Set(base, set = {}) {
  const s = ROUND12.set;
  return { ...s, governor: { ...base.params.governor, clamp_A: ROUND12.governorClamp_A }, ...set, parts: { ...s.parts, ...(set.parts ?? {}) }, linkage: { ...s.linkage, ...(set.linkage ?? {}) } };
}

/** The corner list the design must land at: nominal and the eight robustness corners. */
const cornersOf = () => [{ label: 'nominal' }, ...robustnessCorners()];

/**
 * The slowest converged steady glide (wings parked, the glide table's trim) at one corner: walks down from vHi in
 * steps of `step` until the trim fails, then bisects to `res`. Returns { V, row } (the slowest converged speed).
 */
export function slowestGlide(d, perturb, { vHi = 9.5, step = 0.25, res = 0.05, amp = d.control.trim[0].amp, trim = null } = {}) {
  const tr = trim ?? (() => { const bird = buildBird(d, { perturb }); return (V, start) => trimGlide(d, bird, { V, amp, start }); })();
  let ok = null, start = null, v = vHi;
  for (; v > 3; v -= step) {
    const g = tr(v, start);
    if (!(g && g.converged)) break;
    ok = g; start = g;
  }
  if (!ok) return { V: null, row: null };
  let lo = v, hi = ok.V; // lo fails, hi converged
  while (hi - lo > res + 1e-9) {
    const m = 0.5 * (lo + hi);
    const g = tr(m, ok);
    if (g && g.converged) { hi = m; ok = g; } else lo = m;
  }
  return { V: hi, row: ok };
}

/** The design's landing law (control.landing) from its glide stall speeds at every corner (slowest: slowestGlide, injectable for tests). */
export function landingLawFor(d, P = ROUND12.landing, slowest = slowestGlide) {
  const per = cornersOf().map((c) => ({ corner: c.label, ...slowest(d, c.aero ? { aero: c.aero, mass: c.mass, cgx: c.cgx } : {}, { res: P.resolution_m_s }) }));
  if (per.some((p) => p.V === null)) throw new Error(`no converged glide at ${per.filter((p) => p.V === null).map((p) => p.corner).join(', ')}`);
  const worst = per.reduce((a, b) => (b.V > a.V ? b : a));
  const Vs = worst.V;
  const Vapp = +(P.vrefFactor * Vs).toFixed(3);
  const gl = d.control.glide;
  const sinkAt = (v) => { let a = gl[0]; for (const r of gl) if (r.V <= v) a = r; return a.sink; };
  const flareH = +(P.tau_s * sinkAt(Vapp)).toFixed(3);
  return {
    Vapp_m_s: Vapp, kV: P.kV, kIV: P.kIV, iMax_rad: P.iMax_rad, flareH_m: flareH, tau_s: P.tau_s, sinkTd_m_s: P.sinkTd_m_s, kS: P.kS, kIS: P.kIS,
    Vfloor_m_s: +(P.floorFactor * Vs).toFixed(3), kFloor: P.kFloor, dThetaMin_rad: P.dThetaMin_rad, dThetaMax_rad: P.dThetaMax_rad,
    basis: { stall_reference_m_s: Vs, stall_reference_corner: worst.corner, stall_reference_rule: `slowest converged steady parked-wing glide trim (src/design/trim.js trimGlide) over nominal and the 8 robustness corners, to ${P.resolution_m_s} m/s; the 1-g stall lies below it`, vref: `${P.vrefFactor} x ${Vs} m/s (${P.vrefBasis})`, floor: `${P.floorFactor} x ${Vs} m/s`, flare_height: `tau x the nominal glide table's sink at Vapp (${sinkAt(Vapp).toFixed(3)} m/s)`, per_corner_m_s: Object.fromEntries(per.map((p) => [p.corner, p.V])), sensors: 'pitot airspeed, baro and IMU height and climb rate (BOM sensors), at the controller rate, through its 20 ms delay line', law: 'src/physics/control.js landingLaw' },
  };
}

export function round12Assumptions(base, d) {
  return [...new Set([...round11Assumptions(base, d).filter((x) => !/^Round 12/.test(x)),
    `Round 12: the belly landing is flown by the controller's own approach-and-flare law (control.landing): approach at ${d.control.landing?.Vapp_m_s} m/s, 1.23 x the slowest steady glide over the robustness corners (VREF per 14 CFR 25.125), feed-forward from that speed's glide row, exponential flare; field elevation known to the controller (the launch point's baro reference).`,
    'Round 12: gear life factors at the declared design life (200 flights of 1500 s, every load cycle at the M7 peak, each gear\'s own revolutions at the rating crank frequency): JGMA 401-01 K_L and 402-01 K_HL as KHK prints them, a count reading the table row at or above it (no source states a rule between rows); Duracon from KHK Fig. 11.3 read off the printed curve less 0.05 kgf/mm2.',
    'Round 12: the servos run behind a Pololu step-down regulator: torque and speed at its published voltage less its 4% accuracy (or the lowest pack voltage less the dropout read off its curve), stall current at the voltage plus 4%, battery draw through the lower end of its published typical efficiency.',
    'Round 12: the calm-air stroke envelope keeps 0.6 of every rated member\'s rating (rough-air member peaks measured up to 1.58 x the calm edge); the crank governor\'s drive clamp is 26 A; the hand launch flaps at 2.6 Hz (control.launchLaw).'])];
}

const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/** One round-12 build: buildRound11, then the landing law. -> { d, p, failed }. */
export async function buildRound12({ base, set = {}, opt = {}, catalog, fit = true, calPath = null, noSuction = false, perf = true }) {
  const b = await buildRound11({ cand: 'larus', base, set: round12Set(base, set), opt: { ...ROUND12.opt, ...opt }, catalog, fit, calPath, noSuction, perf });
  if (b.failed) return b;
  b.d.control.landing = landingLawFor(b.d);
  b.d.control.launchLaw = { ...ROUND12.launchLaw };
  b.d.selection = { ...b.d.selection, pipeline: 'src/design/round12.mjs', basis: `${b.d.selection.basis}; round 12: control.landing (ROUND12.landing), gear life factors (set.gearRating.life)` };
  b.d.assumptions = round12Assumptions(base, b.d);
  return b;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const designPath = join(ROOT, 'data/designs/larus-gl-2.json');
  const outPath = resolve(arg('--out') ?? designPath);
  const official = outPath === designPath;
  const base = JSON.parse(readFileSync(arg('--base') ?? join(ROOT, 'out/sweep/round9/v18-base.json'), 'utf8'));
  const set = arg('--set') ? JSON.parse(arg('--set')) : {};
  const opt = arg('--opt') ? JSON.parse(arg('--opt')) : {};
  const t0 = Date.now();
  const { d, p, failed } = await buildRound12({ base, set, opt, catalog, fit: !process.argv.includes('--no-fit'), calPath: arg('--cal') ? resolve(arg('--cal')) : null, noSuction: process.argv.includes('--no-suction'), perf: !process.argv.includes('--no-perf') });
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
    mkdirSync(join(ROOT, 'out/sweep/round12'), { recursive: true });
    writeFileSync(join(ROOT, 'out/sweep/round12/build-larus-gl-2.json'), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, landing: d.control.landing, performance: d.performance, calibration: d.calibration, suction: d.suction ?? null, schema_errors: errs }, null, 1));
  } else writeFileSync(outPath, JSON.stringify(d));
  const env = d.control.envelope.map((e) => `${(e.amp / DEG).toFixed(0)}:${e.freqMax.toFixed(2)}${e.limitedBy[0]}`).join(' ');
  console.log(`${d.name}${w ? ` v${w.version}` : ''} -> ${outPath.split('/').pop()}: ${d.mass.total_g.toFixed(1)} g, WL ${d.geometry.wing_loading_N_m2.toFixed(1)}, G ${p.gearRatio.toFixed(2)}, train ${p.crankRating.toFixed(2)} N m, cruise ${d.control.Vcruise}, env ${env}, Vapp ${d.control.landing.Vapp_m_s} (Vs ${d.control.landing.basis.stall_reference_m_s} ${d.control.landing.basis.stall_reference_corner})${d.performance ? `, climb ${d.performance.best_sustained_climb_m_s}` : ''} (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
}
