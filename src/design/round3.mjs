#!/usr/bin/env node
// Round 3 Larus: an explicit design (no screen), trimmed, with the round-3 controller options and a stroke envelope
// measured by flight test, written atomically through io.js.
//   node src/design/round3.mjs [--out path] [--set '<json params>']      build from the round-3 parameters
//   node src/design/round3.mjs --finalize data/designs/larus-gl-2.json   re-trim on data/calibration/<name>.json
// The parameter choices are argued in the comments of ROUND3 below and measured in out/sweep/round3-*.json.
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildDesign, gearRatioOf } from './build.js';
import { buildControl, refineTrimByFlight, trimLevel, flightEnvelope, trainCrankRating, limiterConfig } from './trim.js';
import { validateDesign } from './schema.js';
import { buildBird } from '../physics/flight.js';
import { runClimb } from '../physics/scenarios.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const arg = (k) => (process.argv.includes(k) ? process.argv[process.argv.indexOf(k) + 1] : null);

/**
 * Round 3 changes from the round-2 Larus (v7), each against a measured round-2 failure:
 *  - Twist servo moved from the hand wing to the elbow (torque rod to the tip): the wing's moving mass sat at 80% of
 *    the semispan, the body recoiled against it and free-flight cruise power was 2.7x the frozen-body trim's.
 *  - Twist servo through a 1.6:1 horn linkage: the stroke could not trim at the twist thrust needs, because the servo's
 *    published speed capped the twist rate; the published torque has room (M7 round 2: 0.36 of 1.19 N m).
 *  - Three-stage train with a steel middle stage: the acetal fast stage was over its rating (M7) and capped the stroke.
 *  - Tube coupler (round 2 rod buckled, FoS 0.49) and a woven 12x11 inner spar (8x7 pultruded at half strength, 0.83).
 *  - Lighter motor at a higher ratio: the round-2 motor ran at a quarter of its current and weighed 140 g.
 */
export const ROUND3 = {
  // Larger stroke at lower frequency: at the same thrust the wing's inertia load (A f^2) and the twist rate fall.
  amp_deg: 34,
  ampMax_deg: 42,
  twistServoAt: 'elbow',
  // 1.6:1, not 2:1: rough-air twist torque reached 0.69-0.78 N m at the wing, over the 2:1 and 1.8:1 stall torques.
  twistLinkageRatio: 1.6,
  bearingCount: 10,
  gearboxPlate_g: 11,
  parts: {
    twistRod: 'rod-ec-3mm',
    coupler: 'tube-ec-woven-7x5',
    // Rockwest pultruded 10x8 at half its disclaimed strength: rough-air root bending reached 31 N m.
    sparInner: 'tube-rockwest-pultruded-10x8',
    motor: 'motor-cobra-c-2814-16',
    // 19.75:1, crank rating 9.28 N m (DS0.8-20 acetal pinion governs), 253 g.
    gears: ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'],
  },
};

/** Round-3 defaults for the controller and envelope; measured in out/sweep/round3-*.json and the report. */
export const ROUND3_OPT = { Vrough: 13, allowance: { gear: 0.8, spar: 0.9, coupler: 0.9, servo: 0.9 } };

/** Controller options of round 3 (src/physics/control.js): twist follows the stroke, total energy control. */
export function round3ControlOptions(d, opt = {}) {
  const semi = d.geometry.span_m / 2;
  return {
    dynSchedule: { rRef_m: 0.75 * semi, gain: opt.dynGain ?? 1, min: 10 * DEG, max: (opt.dynMax_deg ?? 38) * DEG },
    tecs: { Vfloor: opt.Vfloor ?? 9.5, Vmax: opt.Vmax ?? 15, kV: opt.kV ?? 0.4, band: opt.band ?? 1.5, kTE: opt.kTE ?? 1 },
  };
}

/**
 * Round 4 options on a fresh control table: the converged flight step, the motor-current limiter and gust load
 * alleviation, in place before the flight trim and the envelope tests so both fly the bird as it will be flown.
 */
export function applyRound4Options(d, opt = {}) {
  if (opt.dt) {
    d.control.dt = opt.dt;
    d.control.dtRule = 'round 4: the step at which M4 peaks converge (half step changes them < 10%), not the 60-steps-per-cycle rule';
  }
  // Round 7: the flight controller's loop rate (default 100 Hz), set before the trim and envelope flights.
  if (opt.ctlRate_Hz) d.control.rate_Hz = opt.ctlRate_Hz;
  if (opt.motorCurrentLimit) d.control.motorCurrentLimit = limiterConfig(d, opt.motorCurrentLimit);
  if (opt.gla) d.control.gla = opt.gla;
  return d.control;
}

/** Cruise schedule: the given one, or the speed and twist of least frozen-body trim power over a small grid. */
export function pickCruiseTrim(d, bird, amp, sched, opt) {
  if (sched) return { dyn: sched.dyn / DEG, t: { V: sched.V } };
  let best = null;
  for (const V of opt.Vtry ?? [9, 9.5, 10, 10.5, 11]) for (const dyn of opt.dynTry ?? [22, 30, 38]) {
    const t = trimLevel(d, bird, { V, amp, dyn: dyn * DEG });
    if (t && t.converged && (!best || t.power < best.t.power)) best = { dyn, t };
  }
  return best;
}

/** Envelope by flight test at the slowest and fastest flight trims, with rough-air allowances on the ratings. */
async function applyFlightEnvelope(d, opt) {
  const allowance = opt.allowance ?? { gear: 0.8, spar: 0.8, coupler: 0.8, servo: 0.8 };
  // Round 4 (opt.envSpeeds): the envelope at each of several airspeeds, interpolated in flight on the sensed airspeed.
  const envSpeeds = opt.envSpeeds ?? [d.control.trim[0].V, d.control.trim[d.control.trim.length - 1].V];
  const env = await flightEnvelope(d, { speeds: envSpeeds, allowance, perSpeed: !!opt.envSpeeds });
  const usable = env.filter((e) => e.freqMax > 0);
  if (!usable.length) return false;
  d.control.envelopeFrozen = d.control.envelope;
  d.control.envelope = env;
  d.control.envelopeAllowance = { ...allowance, note: 'fraction of each rating the calm-air envelope may use, leaving the rest for rough-air load increments; M7 judges the full rating against the loads of the flights themselves' };
  d.control.envelopeBest = usable.reduce((p, q) => (q.amp * q.freqMax > p.amp * p.freqMax ? q : p));
  d.control.freqMax = Math.max(...usable.map((e) => e.freqMax));
  // Round 4 passes opt.dt, the step at which the gates' peaks converge; rounds 1-3 used 60 steps per fastest cycle.
  d.control.dt = opt.dt ?? 1 / (240 * Math.max(1, Math.ceil((60 * d.control.freqMax) / 240)));
  return true;
}

/** Trim, control table, free-flight trim, round-3 options and the flight-test envelope. Mutates d. */
export async function completeDesign(d, { sched = null, opt = {} } = {}) {
  const bird = buildBird(d);
  const amp = d.params.amp_deg * DEG;
  const best = pickCruiseTrim(d, bird, amp, sched, opt);
  if (!best) return { failed: 'no converged trim at cruise speed' };
  const Vc = best.t.V;
  const speeds = opt.speeds ?? [Vc, Vc + 0.5, Vc + 1, Vc + 2, Vc + 3, Vc + 4];
  const ctl = buildControl(d, { amp, sym: 0, dyn: best.dyn * DEG, lead: 0, V: Vc }, { speeds, bird });
  if (!ctl) return { failed: 'no trim table' };
  d.control = ctl;
  applyRound4Options(d, opt);
  // The flight trim is a property of the bird: flown with the round-2 law, then the round-3 options go on top (at a
  // trim row the twist schedule returns the row's own twist, so the table holds for both).
  await refineTrimByFlight(d);
  Object.assign(d.control, round3ControlOptions(d, opt));
  if (!d.control.trimConverged) return { failed: `no free-flight trim converged: ${JSON.stringify((d.control.trimFlight ?? []).map((r) => [r.V, r.reason]))}` };
  const tf = (d.control.trimFlight ?? []).filter((r) => !r.unflyable);
  d.control.Vcruise = tf.reduce((a, b) => (b.flight_power_w < a.flight_power_w ? b : a)).V;
  // Rough air fast (13 m/s, measured best of 12 and 13 over 30 M4 seeds): speed is the energy the bird trades for
  // height in sinking air, and a gust of length H passes in H / V.
  d.control.Vrough = opt.Vrough ?? 13;
  if (opt.roughMaxBank_deg) d.control.roughMaxBank = opt.roughMaxBank_deg * DEG;
  d.control.Vmin = Math.min(d.control.Vmin, d.control.tecs.Vfloor);
  d.cruise = { V: d.control.Vcruise, amp, dyn: best.dyn * DEG, sym: 0, lead: 0 };
  if (!(await applyFlightEnvelope(d, opt))) return { failed: 'no stroke within ratings in flight' };
  return { ok: true };
}

/** Sustained climb at the corridor's elevation (40 m below its origin, as round 2), best over commanded rates. */
export function measurePerformance(d) {
  const corridor = JSON.parse(readFileSync(join(ROOT, 'data/env/corridor.json'), 'utf8'));
  const alt = corridor.elevation_m_at_origin - 40;
  const rows = [];
  for (const V of [d.control.Vcruise, 10, 12].filter((v, i, a) => a.indexOf(v) === i)) {
    let best = null;
    for (const c of [0.5, 1.0, 1.5, 2.0, 3.0]) {
      const r = runClimb(d, { V, altitude_m: alt, climbCmd: c, seconds: 45 });
      r.climb_cmd_m_s = c;
      if (!best || r.climb_m_s > best.climb_m_s) best = r;
    }
    rows.push({ V_target_m_s: V, sustained_climb_m_s: +best.climb_m_s.toFixed(3), mean_airspeed_m_s: +best.mean_airspeed_m_s.toFixed(2), climb_cmd_m_s: best.climb_cmd_m_s, mean_batt_A: +best.mean_batt_A.toFixed(2), motor_rms_2s_max_A: +best.motor_rms_2s_max_A.toFixed(2), altitude_m: alt });
  }
  const bestRow = rows.reduce((a, b) => (b.sustained_climb_m_s > a.sustained_climb_m_s ? b : a));
  return {
    sustained_climb: rows,
    best_sustained_climb_m_s: bestRow.sustained_climb_m_s,
    best_climb_speed_m_s: bestRow.mean_airspeed_m_s,
    corridor_elevation_m: corridor.elevation_m_at_origin,
    method: 'src/physics/scenarios.js runClimb: closed loop with the flight controller, its stroke envelope and current limit; climb from 15 s to 45 s of steady climb, no wind, battery full; best over commanded climb rates 0.5-3 m/s',
    for_environment: 'the corridor can be re-solved to this climb with a margin: a gradient at speed V is climb / V',
    ran_at: new Date().toISOString(),
  };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
  const finalizePath = arg('--finalize');
  const opt = { ...ROUND3_OPT, ...(arg('--opt') ? JSON.parse(arg('--opt')) : {}) };
  let d, outPath;
  const t0 = Date.now();
  if (finalizePath) {
    outPath = resolve(finalizePath);
    const old = JSON.parse(readFileSync(outPath, 'utf8'));
    d = buildDesign(old.params, catalog);
    const cal = JSON.parse(readFileSync(join(ROOT, `data/calibration/${d.name}.json`), 'utf8'));
    d.calibration = { file: `data/calibration/${d.name}.json`, source: cal.source, multipliers: cal.multipliers };
    const r = await completeDesign(d, { sched: { V: old.cruise.V, dyn: old.cruise.dyn }, opt: { ...old.round3_options, ...opt } });
    if (!r.ok) { console.error(`finalize failed: ${r.failed}`); process.exit(1); }
    d.round3_options = { ...old.round3_options, ...opt };
    d.selection = old.selection;
  } else {
    outPath = resolve(arg('--out') ?? join(ROOT, 'data/designs/larus-gl-2.json'));
    const base = JSON.parse(readFileSync(join(ROOT, 'data/designs/larus-gl-2.json'), 'utf8'));
    const set = arg('--set') ? JSON.parse(arg('--set')) : {};
    const p = { ...base.params, ...ROUND3, ...set, parts: { ...base.params.parts, ...ROUND3.parts, ...(set.parts ?? {}) } };
    p.gearRatio = gearRatioOf(p.parts.gears, catalog);
    p.crankRating = trainCrankRating(buildDesign(p, catalog).structure.gear_stages);
    d = buildDesign(p, catalog);
    // Provisional calibration: the round-2 V5 multipliers of the same wing family; V5 is rerun on this design.
    const calP = join(ROOT, `data/calibration/${d.name}.json`);
    if (existsSync(calP)) {
      const cal = JSON.parse(readFileSync(calP, 'utf8'));
      d.calibration = { file: `data/calibration/${d.name}.json`, source: cal.source, multipliers: cal.multipliers, provisional: true };
    }
    const r = await completeDesign(d, { opt });
    if (!r.ok) { console.error(`build failed: ${r.failed}`); process.exit(1); }
    d.round3_options = opt;
    d.selection = { pipeline: 'src/design/round3.mjs', basis: 'explicit round-3 parameters (ROUND3 in the script) on the round-2 Larus; screen not rerun', set };
  }
  d.assumptions = [...new Set([...(d.assumptions ?? []),
    'Twist servo at the elbow drives the hand wing through a 3 mm carbon torque rod; the rod\'s torsional strength is not published (stress reported, not checked).',
    'Twist horn linkage ratio is geometry: the servo\'s published torque and speed are scaled by it; linkage backlash and compliance not modelled.',
    ...(d.bom.parts.filter((b) => b.role === 'gear').length > 4 ? ['Three-stage train: an acetal KHK DS0.8 pair, then two steel KHK MSGA1-18/MSGB1-40 pairs; each gear held to its own published (lower of bending and surface durability) rating; shafts and bearings beyond the BOM count not modelled.'] : []),
    'Stroke envelope measured by flight test in calm air at a fraction of the gear, spar, coupler and servo ratings (control.envelopeAllowance: a rough-air allowance, a design choice); M7 judges full ratings on the flown loads.'])];
  d.performance = measurePerformance(d);
  const errs = validateDesign(d);
  if (errs.length) console.error('schema:', errs.join('; '));
  mkdirSync(join(ROOT, 'out/sweep'), { recursive: true });
  const w = writeDesignAtomic(outPath, d);
  writeFileSync(join(ROOT, `out/sweep/round3-${finalizePath ? 'finalize' : 'build'}.json`), JSON.stringify({ path: outPath, version: w.version, sha256: w.sha256, minutes: (Date.now() - t0) / 60000, mass_g: d.mass.total_g, Vcruise: d.control.Vcruise, Vrough: d.control.Vrough, trimFlight: d.control.trimFlight, envelope: d.control.envelope, performance: d.performance, schema_errors: errs }, null, 1));
  console.log(`${d.name} v${w.version}: ${d.mass.total_g.toFixed(0)} g, cruise ${d.control.Vcruise} m/s, best climb ${d.performance.best_sustained_climb_m_s} m/s; ${outPath}`);
}
