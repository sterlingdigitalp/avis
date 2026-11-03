// Real numbers for panels and cards, every one read from the design file, the catalogue entries or a gate file.
// Nothing here is typed in: if the design changes, these change.
import { twistCommand } from '../../src/physics/kinematics.js';

const DEG = 180 / Math.PI;

export function numbers(model, data) {
  const D = model.design;
  const tr = model.train;
  const Vc = D.control.Vcruise;
  const row = D.control.trim.reduce((b, r) => (Math.abs(r.V - Vc) < Math.abs(b.V - Vc) ? r : b));
  const m2 = data.gates.find((x) => x.gate === 'M2')?.measured;
  const bat = D.battery;
  const sched = { sym: row.sym, dyn: row.dyn, lead: D.cruise.lead };
  return {
    name: D.label,
    reference: D.reference,
    span_m: model.span,
    area_m2: model.area,
    AR: D.geometry.AR,
    mass_g: D.mass.total_g,
    parts_g: D.bom.parts_g,
    structure_g: D.bom.structure_g,
    margin_g: D.bom.margin_g,
    wingLoading: D.geometry.wing_loading_N_m2,
    bomLines: D.bom.parts.length + D.bom.structure.length,
    partCount: D.bom.parts.reduce((p, b) => p + b.count, 0),
    cruiseV: Vc,
    flapHz: row.freq,
    ampDeg: row.amp * DEG,
    twistDynDeg: row.dyn * DEG,
    ratio: tr.ratio,
    stages: tr.stages.map((s) => `${s.wheel.z}/${s.pinion.z}`),
    crankRpm: row.freq * 60,
    motorRpm: row.freq * 60 * tr.ratio,
    kv: model.parts[D.motor.id]?.specs.kv_rpm_per_v,
    motorName: model.parts[D.motor.id]?.name,
    batteryName: model.parts[bat.id]?.name,
    batteryWh: bat.V_nom * bat.capacity_Ah,
    batteryCells: bat.cells,
    batteryMah: bat.capacity_Ah * 1000,
    cruisePowerW: m2 ? m2.cruise_power_w : null,
    enduranceMin: m2 ? m2.endurance_min : null,
    powerRatio: m2 ? m2.power_ratio : null,
    sched,
    twistAt: (phase, side = 1) => twistCommand(phase, sched, side),
    gates: data.gates,
    passCount: data.gates.filter((g) => gateVerdict(g) === 'pass').length,
    fit: data.fit,
  };
}

/** One line per gate for cards and panels, in the gate's own words and numbers, each against its limit. */
export const MISSION = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8'];

/**
 * A gate's verdict by tools/verify.mjs's rule (SPEC "Final verdicts"): a mission gate is final only if it is the same
 * flown with eta_s at the published 0.6 everywhere (out/verification/eta0.6/, carried here as g.at06, the newest such
 * file, counted only when no older than the official run). 'NOT FINAL' when the two differ (a pass that is not final is
 * not a pass), 'pass?' when a mission gate has no comparison on record, else 'pass' or 'FAIL'. A pass that rests on
 * something no rating covers (notCounted) is 'NOT COUNTED': it is not a pass either.
 */
export function gateVerdict(g) {
  if (!g) return 'MISSING';
  const alt = g.at06 && String(g.at06.ran_at) >= String(g.ran_at) ? g.at06 : null;
  if (MISSION.includes(g.gate) && !alt) return g.passed ? (notCounted(g).length ? 'NOT COUNTED' : 'pass?') : 'FAIL';
  if (alt && alt.passed !== g.passed) return 'NOT FINAL';
  if (g.passed && notCounted(g).length) return 'NOT COUNTED';
  return g.passed ? 'pass' : 'FAIL';
}

/**
 * Why a gate's pass is not counted, in its record's own words (SPEC "Completeness and assembly order", v24 result: a
 * pass that rests on a fixing no rating covers is not counted until a rated fixing replaces it):
 *  - the record's own ruling, when it states one: counted false, with not_counted_because (a string or a list);
 *    counted true clears it;
 *  - else every RC-practice fixing the record lists (measured.members.rc_practice) with no published figure: its pass
 *    rests on that fixing, rated by no one. The SPEC's RC-practice ruling may still cover one (a gear, pulley or
 *    coupling held to its shaft); only the record's counted flag can say so, so until it does, the card does not
 *    count it.
 * -> [reasons], empty when counted
 */
export function notCounted(g) {
  if (!g || g.counted === true) return [];
  if (g.counted === false) return [].concat(g.not_counted_because ?? 'not counted (the record says so)');
  if (Array.isArray(g.not_counted)) return g.not_counted; // carried by pageGate, which drops the record's lists
  const rc = g.measured?.members?.rc_practice;
  return (Array.isArray(rc) ? rc : []).filter((x) => x && x.published_Nm == null).map((x) => `the ${x.name} (${x.status ?? 'unrated'}: no published rating)`);
}

export function gateLine(g) {
  if (g.line) return g.line;
  const m = g.measured ?? {}, L = g.limit ?? {};
  const f = (v, d = 1) => (typeof v === 'number' ? v.toFixed(d) : '—');
  const lim = (v, d, unit = '') => (typeof v === 'number' ? ` (limit ${f(v, d)}${unit})` : '');
  const line = GATE_LINES[g.gate];
  return line ? line(m, L, f, lim) : g.gate;
}

const GATE_LINES = {
  M1: (m, L, f, lim) => `Hand launch${typeof m.release_speed_m_s === 'number' ? ` at ${f(m.release_speed_m_s, 0)} m/s, ${f(m.release_pitch_deg, 0)}°` : ''}: climbing from ${f(m.first_1s_window_climbing_s, 2)} s${lim(L.climb_within_s, 0, ' s')}, ${f(m.height_lost_m, 2)} m lost${lim(L.height_lost_m, 1, ' m')}${m.robustness_passed === false ? ', fails' : ''}`,
  M2: (m, L, f, lim) => `Endurance ${f(m.endurance_min)} min at ${f(m.cruise_power_w)} W${lim(L.endurance_min, 0, ' min')}${m.robustness_passed === false ? '; fails' : ''}`,
  M3: (m, L, f, lim) => `${f(L.bank_deg ?? 30, 0)}° turn, height held to ${f(m.altitude_dev_m, 1)} m${lim(L.altitude_band_m, 0, ' m')}`,
  M4: (m, L) => `Rough air: ${m.seeds_passed ?? '—'} of ${m.seeds_run ?? 100} seeds (needs ${L.pass_seeds ?? 98})`,
  M5: (m, L, f, lim) => `Belly landing at ${f(m.touchdown_sink_m_s, 2)} m/s sink${lim(L.sink_m_s, 1, ' m/s')}`,
  M6: (m, L, f) => `Mass ${f(m.total_g, 0)} g from the BOM${typeof m.margin_frac === 'number' ? `, ${f(100 * m.margin_frac, 0)}% margin` : ''}`,
  M7: (m, L, f) => {
    const need = ` (need ${f(L.factor_of_safety ?? 1.5, 1)})`;
    const lowest = typeof m.fos_min === 'number' ? `lowest FoS ${f(m.fos_min, 2)}${need}` : '';
    const named = m.spar_inner || m.coupler ? `inner spar FoS ${f(m.spar_inner?.fos, 2)}, coupler buckling ${f(m.coupler?.fos_buckling, 2)}` : '';
    return `Structure: ${[lowest, named].filter(Boolean).join('; ') || '—'}${lowest ? '' : need}${m.gears_within_rating === false ? ', gears over rating' : ''}`;
  },
  M8: (m) => {
    const f = (v, d) => v.toFixed(d);
    const pairs = [['motor', m.motor_rms_2s_max_A, m.motor_continuous_A], ['ESC', m.esc_rms_2s_max_A, m.esc_continuous_A], ['battery', m.battery_rms_2s_max_A, m.battery_continuous_A]].filter((p) => typeof p[1] === 'number' && p[2]);
    const w = pairs.sort((x, y) => y[1] / y[2] - x[1] / x[2])[0];
    return w ? `Currents: ${w[0]} ${f(w[1], 1)} A against ${f(w[2], 0)} A continuous` : 'Currents against ratings';
  },
  V1: (m, L, f) => `UVLM vs Theodorsen: amplitude ${f(m.max_amp_error_pct)}%, phase ${f(m.max_phase_error_deg)}° off (limits ${f(L.amp_error_pct, 0)}%, ${f(L.phase_error_deg, 0)}°)`,
  V2: (m, L, f, lim) => `UVLM lift slope: ${f(m.max_error_pct, 1)}% off${typeof m.worst_AR === 'number' ? ` at AR ${f(m.worst_AR, 0)}` : ''}${lim(L.max_error_pct ?? 3, 0, '%')}`,
  V3: (m, L, f, lim) => `UVLM thrust vs Garrick: ${f(m.max_ct_error_pct)}% off${lim(L.ct_error_pct, 0, '%')}`,
  V4: (m, L, f) => `SmartBird power: ${f(m.predicted_w)} W against ${f(L.published_w, 0)} W published${Array.isArray(L.band_w) ? ` (band ${f(L.band_w[0])}–${f(L.band_w[1])} W)` : ''}`,
  V5: (m, L, f) => `Strip model vs UVLM: lift ${f(m.worst_lift_err_pct)}%, thrust ${f(m.worst_thrust_err_pct)}% off (limit ${f(L.lift_err_pct, 0)}%)`,
  V6: (m) => `GPU lattice Boltzmann checks${Array.isArray(m.failures) ? `: ${m.failures.length} failures` : ''}`,
  V7: (m, L, f) => `OpenFOAM vs wind tunnel: ${m.points_passed ?? '—'} of ${m.points ?? '—'} points${typeof m.points_unconverged === 'number' ? `, ${m.points_unconverged} unconverged` : ''} (cl ${f(100 * (L.cl_relative ?? NaN), 0)}%, cd ${f(100 * (L.cd_relative ?? NaN), 0)}%)`,
};

/**
 * What a failed gate failed, in the gate file's own words: the nominal case, each robustness corner the file marks
 * false (by its own key), each other check it marks false, and the measured value beside a numeric limit it exceeds.
 * Empty for a pass.
 */
export function gateFailures(g) {
  if (g.passed !== false) return [];
  const m = g.measured ?? {};
  const out = [...(m.nominal_passed === false ? ['nominal case fails'] : []), ...robustnessFailures(m), ...falseChecks(m), ...limitFailures(m, g.limit), ...listedFailures(m)];
  return out.length ? out : ['fails (the file names no single check)'];
}

/** The robustness corners a gate file marks false, by their own keys. */
function robustnessFailures(m) {
  const corners = m.robustness && typeof m.robustness === 'object' ? Object.entries(m.robustness) : [];
  const bad = corners.filter(([, v]) => v === false || v?.passed === false).map(([k]) => k);
  if (bad.length) return [`fails at ${bad.length} of ${corners.length} robustness corners: ${bad.join(', ')}`];
  return m.robustness_passed === false ? ['fails its robustness check'] : [];
}

/** Every other check the file marks false. */
const falseChecks = (m) => Object.entries(m).filter(([k, v]) => v === false && !/^(nominal|robustness)_passed$/.test(k)).map(([k]) => `${k.replace(/_/g, ' ')}: no`);

/** An error or loss over its numeric limit, the measured key beside the limit's. */
function limitFailures(m, limit) {
  const out = [];
  for (const [k, v] of Object.entries(limit ?? {})) {
    const mk = [k, `max_${k}`].find((x) => typeof m[x] === 'number');
    if (typeof v === 'number' && mk && /error|err|lost/.test(k) && m[mk] > v) out.push(limitSentence(mk, m[mk], v, m));
  }
  return out;
}

/** The units a measured key ends in, as shown. */
const UNITS = [[/_pct$/, '%', 1], [/_deg$/, '°', 1], [/_m_s$/, ' m/s', 2], [/_m$/, ' m', 2], [/_s$/, ' s', 2], [/_w$/, ' W', 1]];

/**
 * A measured value over its limit as a sentence, from the key's own words: 'max_error_pct' 4.281 against 3 with
 * worst_AR 8 -> "the worst error is 4.3% at AR 8, over its 3% limit".
 */
export function limitSentence(key, value, limit, m = {}) {
  const [re, unit, d] = UNITS.find(([r]) => r.test(key)) ?? [/$/, '', 2];
  const words = key.replace(re, '').replace(/^max_/, 'worst_').replace(/_/g, ' ');
  const where = Object.entries(m).filter(([k, x]) => /^worst_/.test(k) && (typeof x === 'number' || typeof x === 'string') && !UNITS.some(([r]) => r.test(k))).map(([k, x]) => `${k.replace(/^worst_/, '').replace(/_/g, ' ')} ${x}`);
  return `the ${words} is ${value.toFixed(d)}${unit}${where.length ? ` at ${where.join(', ')}` : ''}, over its ${limit}${unit} limit`;
}

/** Failures the file lists itself, and points that did not pass. */
function listedFailures(m) {
  const out = Array.isArray(m.failures) ? m.failures.map((x) => (typeof x === 'string' ? x : JSON.stringify(x))) : [];
  if (typeof m.points === 'number' && typeof m.points_passed === 'number' && m.points_passed < m.points) out.push(`${m.points - m.points_passed} of ${m.points} points fail${typeof m.points_unconverged === 'number' ? `, ${m.points_unconverged} of them unconverged` : ''}`);
  return out;
}

/**
 * A gate record as the page carries it (film/gen_data.mjs): its verdict's inputs, one line in its own numbers, what a
 * failure failed, why a pass is not counted, and the top-level scalar measurements; the uniform eta_s 0.6 comparison
 * the same way. gateVerdict reads it as it reads the full record.
 */
export function pageGate(g) {
  const scalars = (o) => Object.fromEntries(Object.entries(o ?? {}).filter(([, v]) => typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string'));
  const one = (r) => ({ gate: r.gate, design: r.design, passed: r.passed, line: gateLine(r), failures: gateFailures(r), not_counted: notCounted(r), ...(typeof r.counted === 'boolean' ? { counted: r.counted } : {}), measured: scalars(r.measured), ran_at: r.ran_at, file: r.file });
  return { ...one(g), ...(g.at06 ? { at06: { ...one(g.at06), file: `eta0.6/${g.at06.file}` } } : {}) };
}
