// Round 17 records (SPEC "Design freeze", "Function completeness", 2026-11-03): the actuation chains and the ratings.
//  - Actuation chains: every motion the flight model flies (each wing's twist, each tail surface, the flapping drive)
//    has a drawn chain from its actuator's output to the surface it moves; every element is a drawn, BOM-tagged part and
//    each element is joined to the next; the flight model's figures for the chain (pivot, ratio, travel, the
//    actuator's row) are read from the drawn chain or checked against it within a stated tolerance. A chain that cannot
//    deliver what the controller commands is a finding; the command is never changed to hide it. The loads each link
//    carries are rated by M7 (its members are named here), from the flown peaks, at the gates.
//  - Ratings: no part runs outside its maker's published range, and nothing is extrapolated beyond a published row.
// Both records read the assembly and the design only; they never edit either.
import { byId, servoVoltageRange } from './catalog.js';

const DEG = Math.PI / 180;
const r4 = (v) => (Number.isFinite(v) ? +v.toFixed(4) : v);

/** Whether parts a and b share a row of asm.joins (either way round; a row may name a part's solid, 'part/solid'). */
export function joinedParts(asm, a, b) {
  const pa = (s) => s.split('/')[0];
  return asm.joins.some(([x, y]) => (pa(x) === a && pa(y) === b) || (pa(x) === b && pa(y) === a));
}

/** One figure's check: a number within its tolerance (relative when rel), a delivered >= required, or a law error. */
function figureRow(name, f, flownLimits = []) {
  const row = figureCheck(name, f);
  // a limit the flight model itself flies (the servo's end stop, its torque-speed saturation) is reported, not refused
  return !row.ok && flownLimits.includes(name) ? { ...row, ok: true, flown_limit: true, finding: row.why } : row;
}
function figureCheck(name, f) {
  if (f == null) return { figure: name, ok: false, why: 'not stated' };
  if (typeof f !== 'object') return { figure: name, value: f, info: true, ok: true };
  if ('err' in f && 'tol' in f) return { figure: name, ...f, ok: f.err <= f.tol + 1e-12, ...(f.err <= f.tol + 1e-12 ? {} : { why: `error ${r4(f.err)} against tolerance ${f.tol}` }) };
  if ('required_rad' in f) return { figure: name, ...f, ok: f.delivered_rad >= f.required_rad - 1e-9, ...(f.delivered_rad >= f.required_rad - 1e-9 ? {} : { why: `delivers ${(f.delivered_rad / DEG).toFixed(2)} deg of the ${(f.required_rad / DEG).toFixed(2)} deg commanded` }) };
  if ('required_Nm' in f) return { figure: name, ...f, ok: !(f.available_Nm < f.required_Nm), ...(f.available_Nm < f.required_Nm ? { why: `${r4(f.available_Nm)} N m available against ${r4(f.required_Nm)} N m needed` } : {}) };
  if ('required_rad_s' in f) return { figure: name, ...f, ok: !(f.available_rad_s < f.required_rad_s), ...(f.available_rad_s < f.required_rad_s ? { why: `${r4(f.available_rad_s)} rad/s available against ${r4(f.required_rad_s)} rad/s commanded` } : {}) };
  if ('required' in f) return { figure: name, ...f, ok: f.delivered >= f.required - 1e-12, ...(f.delivered >= f.required - 1e-12 ? {} : { why: `${r4(f.delivered)} ${f.units ?? ''} against ${r4(f.required)} required` }) };
  if ('max_err' in f) return { figure: name, ...f, ok: f.max_err <= f.tol + 1e-12, ...(f.max_err <= f.tol + 1e-12 ? {} : { why: `the drawn law departs from the flown one by ${r4(f.max_err)} (tolerance ${f.tol})` }) };
  if ('members' in f) return { figure: name, ...f, ok: f.missing.length === 0, ...(f.missing.length ? { why: `no M7 member ${f.missing.join(', ')}` } : {}) };
  // no tolerance stated: the flown figure is the drawn one (to 1 part in 10^6)
  const rel = f.rel || f.tol == null, tol = f.tol ?? 1e-6;
  const err = rel ? Math.abs(f.drawn - f.flown) / Math.abs(f.flown) : Math.abs(f.drawn - f.flown);
  const ok = Number.isFinite(err) && err <= tol + 1e-12;
  return { figure: name, ...f, err: r4(err), ok, ...(ok ? {} : { why: `drawn ${r4(f.drawn)} against flown ${r4(f.flown)} (${f.rel ? 'relative ' : ''}tolerance ${f.tol})` }) };
}

/** One chain's row: its elements drawn and tagged, each joined to the next, its figures, its findings. */
export function chainRow(asm, c) {
  const byIdP = new Map(asm.parts.map((p) => [p.id, p]));
  const elements = c.path.map((id) => { const p = byIdP.get(id); return { part: id, drawn: !!p, bom: p ? p.bom ?? [] : [] }; });
  const notDrawn = elements.filter((e) => !e.drawn || !e.bom.length).map((e) => e.part);
  const links = c.path.slice(1).map((b, i) => ({ a: c.path[i], b, joined: joinedParts(asm, c.path[i], b) }));
  const unjoined = links.filter((l) => !l.joined).map((l) => `${l.a} | ${l.b}`);
  const figures = Object.entries(c.figures ?? {}).map(([k, f]) => figureRow(k, f, c.flown_limits ?? []));
  const badFig = figures.filter((f) => !f.ok);
  // a module's finding is a string (it refuses the chain) or { text, flown: true }: a limit or effect the flight model
  // flies (reported with its numbers, never hidden; the gates judge its consequence)
  const blocking = (c.findings ?? []).filter((x) => typeof x === 'string');
  const flown = [...(c.findings ?? []).filter((x) => typeof x !== 'string').map((x) => x.text), ...figures.filter((f) => f.flown_limit).map((f) => `${f.figure}: ${f.finding} (flown: the flight model holds the chain to it)`)];
  const findings = [...blocking, ...badFig.map((f) => `${f.figure}: ${f.why}`)];
  return { name: c.name, actuator: c.actuator, elements: elements.length, path: c.path, not_drawn_or_untagged: notDrawn, unjoined, figures, assumptions: c.assumptions ?? [], findings, flown_findings: flown, passed: notDrawn.length === 0 && unjoined.length === 0 && badFig.length === 0 && !blocking.length };
}

/**
 * The actuation-chains record: every chain the design states (twist R/L, tail R/L, the flapping drive), and every
 * motion the flight model flies has one. -> { rule, chains, missing, failed, passed }.
 */
export function chainRecord(asm, d, chains) {
  const rows = chains.map((c) => chainRow(asm, c));
  const need = ['twist R', 'twist L', 'tail R', 'tail L', 'flap drive R', 'flap drive L'];
  const missing = need.filter((n) => !rows.some((r) => r.name === n));
  const failed = rows.filter((r) => !r.passed).map((r) => r.name);
  return {
    rule: 'SPEC Design freeze, Function completeness (2026-11-03): every actuated motion the flight model flies has a drawn chain from its actuator\'s output to the surface, every element a drawn BOM part joined to the next; the flown figures (pivot, ratio, travel, the actuator\'s row) read from the drawn chain or checked against it within a stated tolerance; a chain that cannot deliver the command is a finding, never hidden by changing the command: where the flight model itself flies the limit (the servo\'s end stop at the drawn travel, its torque-speed saturation, the chain\'s compliance) the finding is reported (flown_findings) and the gates judge it; otherwise it refuses the build; the loads on each link are rated by the M7 members named, from the flown peaks',
    chains: rows, missing, failed: [...missing.map((m) => `${m}: no chain stated`), ...failed],
    passed: !missing.length && !failed.length,
  };
}

/**
 * The flapping drive's chain, each side: the motor to the inner spar, found along the drive's own joins (motor, gear
 * stages, shafts, keys, crank, coupler, rocker, spar), with the gear ratio drawn (the meshes on the path) against the
 * flown one. -> [chain R, chain L].
 */
export function flapChains(asm, d, catalog) {
  // the drive's own parts: everything but the structure that carries it and the fixings (so the path cannot run
  // through a plate or a screw from one shaft to another)
  const notDrive = /plate|panel|tray|riser|rail|bracket|screw|nut|spacer|washer|circlip|journal|readhead|encoder|former|boom|clamp|^packed|pitot|fixing|doubler|lock|sleeve|housing|^tail |^wing|outer spar|leading|trailing|^rib|twist/;
  const drive = { test: (id) => !notDrive.test(id) };
  const adj = new Map();
  for (const [x, y, why] of asm.joins) {
    const a = x.split('/')[0], b = y.split('/')[0];
    if (a === b || !drive.test(a) || !drive.test(b)) continue;
    for (const [p, q] of [[a, b], [b, a]]) { if (!adj.has(p)) adj.set(p, new Map()); adj.get(p).set(q, why); }
  }
  const pathTo = (goal) => {
    const prev = new Map([['motor', null]]), queue = ['motor'];
    while (queue.length) { const p = queue.shift(); if (p === goal) break; for (const q of adj.get(p)?.keys() ?? []) if (!prev.has(q)) { prev.set(q, p); queue.push(q); } }
    if (!prev.has(goal)) return null;
    const out = []; for (let p = goal; p; p = prev.get(p)) out.unshift(p); return out;
  };
  const teeth = (id) => byId(catalog, id).specs.teeth;
  return ['R', 'L'].map((t) => {
    const path = pathTo(`inner spar ${t}`) ?? ['motor', `inner spar ${t}`];
    // the meshes on the path: each wheel's teeth over its pinion's (the pinion first, from the motor)
    let G = 1;
    for (let i = 1; i < path.length; i++) {
      const why = adj.get(path[i - 1])?.get(path[i]);
      if (why !== 'gears in mesh') continue;
      const g0 = / (gear-[\w-]+)$/.exec(path[i - 1])?.[1], g1 = / (gear-[\w-]+)$/.exec(path[i])?.[1];
      if (g0 && g1) G *= teeth(g1) / teeth(g0);
    }
    const members = (d.structure?.members ?? []).map((m) => m.name);
    const sw = t === 'R' ? 'right' : 'left', want = ['crank shaft', 'crank pin', 'crank web', `coupler eye ${sw}`, `rocker arm ${sw}`, `rocker hub ${sw}`];
    return {
      name: `flap drive ${t}`, actuator: { part: 'motor', bom: d.params?.parts?.motor ?? null, output: 'its shaft' }, path,
      figures: {
        ratio: { flown: d.drive.gearRatio, drawn: r4(G), tol: 1e-6, rel: true, units: '-', basis: 'the gear ratio the flight model flies (design.drive.gearRatio) against the teeth of the meshes on the drawn path' },
        loads: { members: want.filter((w) => members.some((n) => n.startsWith(w))), missing: want.filter((w) => !members.some((n) => n.startsWith(w))), basis: 'the M7 members on the drive\'s links (the gear stages are rated by Lewis/Hertz in M7, the shafts, keys, crank and coupler by their members)' },
      },
      assumptions: ['the stroke law is the drawn four-bar itself (joints.js driveLayout is what both the flight model and the clearance check pose)'],
    };
  });
}

/**
 * The ratings record: each servo's supply inside its published rows (torque and speed at the lowest supply voltage,
 * never beyond a row) and its published voltage range (the stall current at the highest), and (round 19) its highest
 * supply voltage inside the published rows too, torque, speed and stall current alike; the regulator's input range
 * holding the pack's whole range; the ESC rated for the pack's cells. -> { rule, rows, failed, passed }.
 */
export function ratingsRecord(d, catalog) {
  const sup = d.electrical?.servo_supply ?? null, rows = [];
  const lo = sup?.performance_V ?? d.electrical?.servo_supply_V, hi = sup?.current_V ?? lo;
  for (const [role, sv] of Object.entries(d.servos ?? {})) rows.push(...servoSupplyRows(role, sv, lo, hi, catalog));
  if (sup?.part) {
    const reg = byId(catalog, sup.part).specs, [a, b] = sup.input_V ?? [NaN, NaN];
    const ok = a >= reg.min_input_v - 1e-9 && b <= reg.max_input_v + 1e-9;
    rows.push({ what: `servo regulator ${sup.part}: input ${a}-${b} V (the pack's lowest terminal voltage to full charge)`, published: `${reg.min_input_v}-${reg.max_input_v} V input`, ok, ...(ok ? {} : { why: 'the pack\'s range is outside the regulator\'s published input range' }) });
  }
  const esc = d.esc ?? null, cells = d.battery?.cells ?? null;
  if (esc?.cells_text && cells) {
    const m = /(\d)\s*[-~]\s*(\d)\s*S/i.exec(esc.cells_text), ok = m ? cells >= +m[1] && cells <= +m[2] : false;
    rows.push({ what: `ESC ${esc.id} on a ${cells}S pack`, published: esc.cells_text, ok, ...(ok ? {} : { why: 'the pack\'s cell count is outside the ESC\'s published range' }) });
  }
  // round 18 (SPEC ruling "Maker's part"): the motor's own bearings and threads are valid only within every figure its
  // maker publishes; the two the build can decide without a flight (cells, and the power its continuous current allows)
  rows.push(...motorRows(d, catalog));
  const failed = rows.filter((r) => !r.ok).map((r) => r.what);
  return {
    rule: 'SPEC Design freeze, Function completeness, "Ratings within their published range" (2026-11-03): no part runs outside its maker\'s published range, and nothing is extrapolated beyond a published row',
    rows, failed, passed: rows.length > 0 && failed.length === 0,
  };
}

/** One servo's ratings rows at a supply lo-hi V: lo inside its published rows (torque and speed), lo-hi inside its
 * published range, and (round 19) hi inside its published rows too, torque, speed and stall current alike. */
function servoSupplyRows(role, sv, lo, hi, catalog) {
  const rows = [];
  const part = byId(catalog, sv.id), perf = (part.specs.performance ?? []).map((r) => r.voltage_v).filter(Number.isFinite);
  const rowLo = Math.min(...perf), rowHi = Math.max(...perf), range = servoVoltageRange(part);
  const okRow = lo >= rowLo - 1e-9 && lo <= rowHi + 1e-9, okRange = !!range && hi <= range[1] + 1e-9 && lo >= range[0] - 1e-9;
  rows.push({ what: `${role} servo ${sv.id}: torque and speed at ${lo} V`, published: `rows ${perf.join(', ')} V`, ok: okRow, ...(okRow ? {} : { why: `${lo} V is outside the published rows ${rowLo}-${rowHi} V (no extrapolation)` }) });
  rows.push({ what: `${role} servo ${sv.id}: supply ${lo}-${hi} V`, published: range ? `${range[0]}-${range[1]} V (${part.specs.voltage_range_text})` : 'no published range', ok: okRange, ...(okRange ? {} : { why: `the supply ${lo}-${hi} V is outside the published range` }) });
  // round 19 (SPEC "Servo voltage covers current too"): every voltage the supply presents inside the published rows,
  // its highest too (the stall current there is read from a row, never scaled beyond the top one)
  const curV = (part.specs.stall_current ?? []).map((q) => q.voltage_v).filter(Number.isFinite), curHi = curV.length ? Math.max(...curV) : Infinity;
  const okHi = hi <= rowHi + 1e-9 && hi <= curHi + 1e-9;
  rows.push({ what: `${role} servo ${sv.id}: highest supply ${hi} V inside its published rows`, published: `torque and speed rows ${perf.join(', ')} V; stall current ${curV.length ? `rows ${curV.join(', ')} V` : 'published without a voltage'}`, ok: okHi, ...(okHi ? {} : { why: `${hi} V is above the published top row ${Math.min(rowHi, curHi)} V (no extrapolation, current included)` }) });
  return rows;
}

// ---- round 18: the maker's-part ruling's conditions (SPEC "Maker's part", operator's decision 2026-11-03) -------------

/** The pack at full charge (4.20 V a cell, or its charge limit), as build.js batteryServoSupply takes it. */
const packFullV = (bt) => +(bt.cells * (Number.isFinite(bt.charge_limit_V_per_cell) ? bt.charge_limit_V_per_cell : 4.2 + (bt.cell_shift_V ?? 0))).toFixed(3);

/**
 * Every figure the motor's maker publishes, and where the build holds the motor to it: { figure, published, units,
 * within (true/false decided here; null: checked by a flight gate, `checked_by`, or nothing published), basis, row (a
 * ratings-record row, for the figures decided without a flight) }. Read from the motor's catalogue entry and, where the
 * catalogue has it, its thermal reference entry ('thermal-ref-' + the motor id without 'motor-').
 */
export function motorFigures(d, catalog) {
  const m = d.motor, bt = d.battery;
  const part = m?.id ? catalog.find((p) => p.id === m.id) : null;
  if (!part || !bt) return [];
  const s = part.specs, th = catalog.find((p) => p.id === `thermal-ref-${m.id.replace(/^motor-/, '')}`)?.specs ?? null;
  const out = [];
  const I = s.continuous_current_a, P = s.continuous_power_w, Vf = packFullV(bt);
  if (Number.isFinite(I)) out.push({ figure: 'current', published: I, units: 'A', within: null, checked_by: "M8: the motor's RMS current over 2 s windows within its continuous rating (missions.mjs); the flight envelope's motor_current ratio", basis: `${s.continuous_rating_note ?? `continuous ${I} A`}${th?.rating_conditions_text ? `; stated for (the maker's manual): ${th.rating_conditions_text} (the 'continuous rating conditions' row holds the motor to it derated)` : ''}` });
  if (Number.isFinite(P) && Number.isFinite(I)) {
    const ok = Vf * I <= P + 1e-9;
    out.push({ figure: 'power', published: P, units: 'W', within: ok, basis: `a window's mean electrical input is at most the pack's full-charge ${Vf} V times its RMS current, so with the current within ${I} A (M8) at most ${(Vf * I).toFixed(1)} W`, row: { what: `motor ${m.id}: power at its continuous current, at most ${Vf} V x ${I} A = ${(Vf * I).toFixed(1)} W`, published: `${P} W continuous`, ok, ...(ok ? {} : { why: 'the pack at full charge can drive more than the published continuous power at the continuous current' }) } });
  }
  const mc = /(\d+)\s*s\b/i.exec(s.cells_text ?? '');
  if (mc) {
    const ok = bt.cells <= +mc[1];
    out.push({ figure: 'voltage (cells)', published: +mc[1], units: 'cells', within: ok, basis: `${s.cells_text}; the pack is ${bt.cells}S`, row: { what: `motor ${m.id} on a ${bt.cells}S pack`, published: s.cells_text, ok, ...(ok ? {} : { why: "the pack's cell count is above the motor's published maximum" }) } });
  }
  out.push({ figure: 'speed', published: null, units: 'rpm', within: null, basis: `none published (Kv ${s.kv_rpm_per_v} rpm/V only)` });
  out.push({ figure: 'torque', published: null, units: 'N m', within: null, basis: 'none published' });
  out.push(motorTemperature(th));
  const rc = th ? ratingConditions(d, s, th) : null;
  if (rc) out.push(rc);
  return out;
}

// ---- round 19: the motor's temperature and its rating's stated conditions (decision 5, "Motor temperature") ----------

/**
 * The motor's temperature figure. Round 19: Scorpion publishes a limit (its operating manual: 100 C at any measured
 * point) but nothing that bounds the motor's temperature from its load (no thermal resistance, no loss or temperature
 * curve; the manual tells the user to measure), and the build has no sourced thermal path: the temperature cannot be
 * bounded, so the maker's-part interfaces resting on it are not counted (counted: false, never a silent pass).
 */
function motorTemperature(th) {
  if (!th) return { figure: 'temperature', published: null, units: 'C', within: null, counted: false, not_counted_because: 'no thermal entry for this motor in the catalogue: whether its maker publishes a temperature limit was not read', checked_by: 'nothing: not read', basis: 'no temperature figure read' };
  const lim = th.max_measured_temperature_c;
  const basis = [th.manual_temperature_text, th.manual_margin_text, th.manual_winding_text, th.manual_magnet_text, th.wire_rating_text, th.magnet_rating_text].filter(Boolean).join(' ');
  // no limit published (the wire's and magnets' ratings are the materials', not a limit on the motor): disclosed
  if (!Number.isFinite(lim)) return { figure: 'temperature', published: `wire ${th.wire_rating_c} C, magnets ${th.magnet_rating_c} C`, units: 'C', within: null, checked_by: "not checked: the maker publishes no temperature limit for the motor (its wire's and magnets' ratings only); its continuous current and power are the figures checked", basis };
  return {
    figure: 'temperature', published: lim, units: 'C', within: null, counted: false,
    not_counted_because: `the maker publishes a limit (${lim} C at any measured point) but no figure that bounds the motor's temperature from its load (no thermal resistance, loss or temperature curve, or cooling figure for this motor; its manual tells the user to measure), and the build has no sourced thermal path, so the temperature cannot be bounded`,
    checked_by: 'nothing: unbounded (round 19)', basis,
  };
}

const rowAtOrAbove = (rows, x) => rows.filter(([k]) => k !== null && k >= x - 1e-9).sort((a, b) => a[0] - b[0])[0] ?? rows.find(([k]) => k === null) ?? null;
const rowAtOrBelow = (rows, x) => rows.filter(([k]) => k !== null && k <= x + 1e-9).sort((a, b) => b[0] - a[0])[0] ?? null;

/**
 * A continuous rating ({ current_A, power_W }) derated for a condition by its maker's published tables (`tables`: %
 * power ratio by ambient temperature, cooling airflow and duty), each read at its published row on the conservative side
 * of the condition (the ambient and the duty at or above it, the airflow at or below it; no interpolation), the three
 * ratios multiplied (the manual states each alone). The tables are labelled "for reference only": SPEC (round 7) counts
 * such a figure at half, so `spec_A` / `spec_W` are the derated values at half. -> { rows, ratio, current_A, power_W,
 * spec_A, spec_W } or null when a table has no row for the condition (an ambient above the table's last column).
 */
export function deratedRating(rating, tables, { airflow_m_s, ambient_C, duty_min }) {
  if (!tables || !Number.isFinite(airflow_m_s) || !Number.isFinite(ambient_C)) return null;
  const a = rowAtOrAbove(tables.ambient_C.filter(([k]) => k !== null), ambient_C), v = rowAtOrBelow(tables.airflow_m_s, airflow_m_s);
  const t = Number.isFinite(duty_min) ? rowAtOrAbove(tables.duty_min, duty_min) : tables.duty_min.find(([k]) => k === null);
  if (!a || !v || !t) return null;
  const ratio = (a[1] / 100) * (v[1] / 100) * (t[1] / 100);
  const I = rating.current_A * ratio, P = rating.power_W * ratio;
  return { rows: { ambient_C: a, airflow_m_s: v, duty_min: t }, ratio: r4(ratio), current_A: r4(I), power_W: r4(P), spec_A: r4(I / 2), spec_W: r4(P / 2) };
}

/** The pack's run time at the cruise trim's power (min): its full capacity at its nominal voltage; null without a trim. */
export function cruiseEndurance(d) {
  const bt = d.battery, V = d.control?.Vcruise;
  const row = (d.control?.trimFlight ?? []).find((r) => r.V === V);
  const ok = row && Number.isFinite(row.flight_power_w) && bt?.capacity_Ah && bt?.V_nom;
  return ok ? (bt.capacity_Ah * bt.V_nom * 60) / row.flight_power_w : null;
}

/**
 * The derated readings at a design's conditions: the air at the motor at most the flight speed (the most the bird can
 * give it, a cooling path from the nose assumed) and none (no cooling path); the ambient the design's atmosphere (the
 * air in the pod is warmer: not quantified); the duty `duty_min` (the pack's run), or the tables' longest row.
 */
function deratedReadings(rating, tables, d, duty_min) {
  const ambient = d.atmosphere?.temperature_C, V = d.control?.Vcruise;
  const at = (airflow) => deratedRating(rating, tables, { airflow_m_s: airflow, ambient_C: ambient, duty_min });
  return { conditions: { duty_min: Number.isFinite(duty_min) ? +duty_min.toFixed(2) : null, airflow_m_s_at_most: V ?? null, ambient_C: ambient ?? null }, derated: { flight_speed_air: Number.isFinite(V) ? at(V) : null, no_cooling_path: at(0) } };
}

/**
 * The continuous rating's stated conditions (the maker's manual: 60 s at 100% throttle, forced airflow of at least
 * 30 m/s, ~30 C ambient) against the design's, with the rating derated by the maker's tables (deratedReadings; at the
 * build, before the trim, the duty is the tables' longest row). within is decided where a flight gives the sustained
 * current (flownMotorCondition): the endurance cruise's RMS over the whole flight against the derated current at half
 * (SPEC round 7) with no cooling path, the design stating none (a sourced, drawn cooling path would earn its airflow's
 * row); the flight-speed reading is reported beside it.
 */
function ratingConditions(d, s, th) {
  if (!th.derating || !Number.isFinite(th.rating_airflow_m_s)) return null;
  const rating = { current_A: s.continuous_current_a, power_W: s.continuous_power_w, duty_s: th.rating_duty_s, airflow_m_s: th.rating_airflow_m_s, ambient_C: th.rating_ambient_c };
  const { conditions, derated } = deratedReadings(rating, th.derating, d, cruiseEndurance(d));
  return {
    figure: 'continuous rating conditions', published: s.continuous_current_a, units: 'A', within: null, rating, tables: th.derating,
    conditions: { rating: { duty_s: rating.duty_s, airflow_m_s: rating.airflow_m_s, ambient_C: rating.ambient_C }, design: conditions }, derated,
    checked_by: "M7 (missions.mjs flownMotorCondition): the endurance cruise's (M2's) motor RMS over the whole flight against the derated current at half (SPEC round 7: the tables are 'for reference only') with no cooling path (the design states none); the flight-speed reading beside it",
    basis: `${th.rating_conditions_text} ${th.rating_reduce_text} ${th.derating_label} (${th.derating_rows_note})`,
  };
}

/**
 * Round 19: whether the maker's rating conditions (60 s at 100% throttle with >= 30 m/s forced air, ~30 C) and its
 * derating tables ("Approximate % Power Ratio for reference only") bind the motor is the operator's ruling, asked on
 * 2026-11-04 and not yet made. Until it is, the row's verdict is held: within null and not counted, its reading beside
 * it (would_be_within, at the SPEC reading: the tables at half, no cooling path). 'binds' applies the reading.
 */
export const RATING_CONDITIONS_RULING = 'pending';

/** The rating-conditions row at the flown design's conditions and its sustained current. */
function flownRating(c, { sustainedRms, endurance_min, design, sustainedFrom, ruling = RATING_CONDITIONS_RULING }) {
  const duty = Number.isFinite(endurance_min) ? endurance_min : design ? cruiseEndurance(design) : null;
  const { conditions, derated } = design ? deratedReadings(c.rating, c.tables, design, duty) : { conditions: c.conditions?.design, derated: c.derated };
  const held = derated?.no_cooling_path;
  const exceeds = Object.fromEntries(Object.entries(derated ?? {}).filter(([, r]) => r).map(([k, r]) => [k, { full: sustainedRms > r.current_A, spec_half: sustainedRms > r.spec_A }]));
  const reading = held ? { within: sustainedRms <= held.spec_A, published_derated: held.spec_A } : { within: null, counted: false, not_counted_because: "the maker's derating tables have no row for the design's condition" };
  const verdict = !held || ruling === 'binds' ? reading : { within: null, counted: false, would_be_within: reading.within, published_derated: reading.published_derated, not_counted_because: `held pending the operator's ruling (asked 2026-11-04) on whether the maker's rating conditions and its reference-only derating tables bind: the sustained ${sustainedRms.toFixed(2)} A against ${held.spec_A} A derated at half with no cooling path (${held.current_A} A at full value) would ${reading.within ? 'pass' : 'fail'}` };
  return { ...c, conditions: { ...c.conditions, design: conditions }, derated, value: sustainedRms, value_basis: `${sustainedFrom}'s motor RMS over the whole flight (the sustained cruise); duty from ${Number.isFinite(endurance_min) ? `its endurance, ${endurance_min.toFixed(2)} min` : "the cruise trim's pack run"}`, exceeds, ...verdict };
}

/**
 * A motor maker's-part condition with what the flights measured: the current row against the population's 2 s RMS
 * (as M8); the rating-conditions row, derated at the flown design's conditions (its atmosphere, cruise speed and the
 * endurance flight's run), against the endurance cruise's sustained RMS (within at the SPEC reading). -> the
 * condition, with `not_counted` the reasons a pass resting on it is not counted.
 */
export function flownMotorCondition(condition, { rms2sMax, sustainedRms, endurance_min = null, design = null, sustainedFrom = 'M2', ruling = RATING_CONDITIONS_RULING }) {
  const out = condition.map((c) => {
    if (c.within === null && c.figure === 'current' && Number.isFinite(c.published) && Number.isFinite(rms2sMax)) return { ...c, value: rms2sMax, within: rms2sMax <= c.published, checked_by: "this population's motor RMS over 2 s windows (as M8)" };
    if (c.figure === 'continuous rating conditions') return Number.isFinite(sustainedRms) ? flownRating(c, { sustainedRms, endurance_min, design, sustainedFrom, ruling }) : { ...c, counted: false, not_counted_because: 'no endurance flight gave the sustained current' };
    return c;
  });
  return Object.assign(out, { not_counted: out.filter((c) => c.counted === false).map((c) => `${c.figure}: ${c.not_counted_because}`) });
}

/**
 * M7's maker's parts (the members checked "maker's part") with their conditions flown (flownMotorCondition): each
 * with its peak loads, its condition and ok (its member's check, and no figure over); a part resting on a figure no
 * check can hold is not counted. -> { parts, ok, notCounted: the reasons, each naming its part }.
 */
export function flownMakersParts(members, flight) {
  const parts = members.filter((r) => r.check === "maker's part").map((r) => {
    const condition = flownMotorCondition(r.condition ?? [], flight);
    return { name: r.name, status: r.status, part: r.part, of: r.of, covers: r.covers, peaks: r.peaks, condition: [...condition], ok: r.ok && condition.every((c) => c.within !== false), ...(condition.not_counted.length ? { counted: false, not_counted_because: condition.not_counted } : {}) };
  });
  return { parts, ok: parts.every((q) => q.ok), notCounted: parts.flatMap((q) => (q.not_counted_because ?? []).map((w) => `${q.name} (unrated, maker's part): ${w}`)) };
}

/** The ratings-record rows for the motor's figures the build decides without a flight (cells; power at its current). */
const motorRows = (d, catalog) => motorFigures(d, catalog).filter((q) => q.row).map((q) => q.row);

/**
 * The condition a motor's own interface (its bearings carrying its shaft and pinion, its mounting threads) is held to: the
 * motor within every figure its maker publishes (motorFigures). `report`: the member's sections reported as its peak load.
 */
export function motorMaker(d, catalog, covers, report) {
  return { part: d.motor.id, of: d.motor.id, covers, report, condition: motorFigures(d, catalog).map(({ row, ...c }) => c) };
}

/**
 * The condition a servo's own arm (the maker's arm on its own servo's spline, with its centre screw and its tapped
 * holes) is held to: that servo's own rule at the published row for the real supply voltage. Section T of the arm's
 * member is the servo's torque at its spline (the surface's torque times the linkage ratio, the quantity M7 holds within
 * the stall at the surface), held within the stall at the supply's lowest voltage; and that supply inside the servo's
 * published rows and range (this record's rows for the servo).
 */
export function servoArmMaker(d, catalog, role, armId) {
  const sv = d.servos?.[role];
  const part = byId(catalog, sv.id), stall = sv.servo_stall_Nm ?? sv.stall_Nm;
  const rowsV = (part.specs.performance ?? []).map((r) => r.voltage_v).filter(Number.isFinite);
  const rr = ratingsRecord(d, catalog).rows.filter((r) => r.what.startsWith(`${role} servo ${sv.id}:`));
  return {
    part: armId, of: sv.id, covers: `the maker's arm ${armId}, its centre screw and its tapped holes, on its own servo's (${sv.id}) spline`, report: ['F', 'T'],
    condition: [
      { figure: `${role} servo torque at its spline`, of: 'T', published: +stall.toFixed(5), units: 'N m', basis: `${sv.id}: ${stall.toFixed(4)} N m stall at ${sv.volts} V, the supply's lowest voltage, between its published rows (${rowsV.join(', ')} V; no extrapolation): the servo's own rule, as M7 holds it (the flight envelope at its servo allowance)` },
      { figure: `${role} servo supply within its published rows and range`, within: rr.length > 0 && rr.every((r) => r.ok), basis: rr.map((r) => `${r.what}: ${r.published}${r.ok ? '' : ` (${r.why})`}`).join('; ') || 'no ratings rows for this servo' },
    ],
  };
}
