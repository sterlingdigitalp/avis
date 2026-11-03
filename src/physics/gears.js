// Round 9: KHK's catalogue gear ratings under their stated calculation conditions (SPEC ruling after physics round 8).
// A rating with a stated basis counts in full under its stated conditions; where a condition does not hold, that
// standard's own correction applies; where the standard gives none, the value counts at half.
//  - Metal gears (MSGA, MSGB, SSAG, SS ...): KHK computes them to JGMA 401-01 (bending) and 402-01 (surface) with a
//    mating gear of the same number of teeth, 600 rpm, over 1e7 cycles, uniform load, bidirectional load (2/3 of
//    sigma_Flim), 100 cSt (50 C) oil and symmetric support (KHK 2025 spur gear catalogue, "Calculation of Bending
//    Strength of Gears"). The corrections below are the standard's own factors as KHK's technical reference prints
//    them (scratchpad research/round9-gears.json): the surface formula's d01 i/(i+1) and KH-beta, the bending formula's
//    load sharing factor 1/eps_alpha, the dynamic factor's speed band and the lubrication speed factor ZV.
//  - Duracon gears (DS): the Lewis formula at sigma_b 3.5 kgf/mm2 for m 1 (40 C, grease), 100 rpm; KHK's plastic gear
//    method gives the speed factor KV and the material factor KM (0.75 Duracon on Duracon) and no reversing rule, so
//    JGMA 401's 2/3 is applied for a reversing load.
// Pure functions of the gear geometry and the stated conditions.

const RAD = Math.PI / 180;

/**
 * Transverse contact ratio of a standard (unshifted, full-depth) external spur pair, module-free:
 * eps = [sqrt(ra1^2 - rb1^2) + sqrt(ra2^2 - rb2^2) - a sin(alpha)] / (pi m cos(alpha)).
 */
export function contactRatio(z1, z2, alphaDeg = 20, ha = 1) {
  const al = alphaDeg * RAD;
  const r1 = z1 / 2, r2 = z2 / 2;
  const ra1 = r1 + ha, ra2 = r2 + ha, rb1 = r1 * Math.cos(al), rb2 = r2 * Math.cos(al);
  return (Math.sqrt(ra1 * ra1 - rb1 * rb1) + Math.sqrt(ra2 * ra2 - rb2 * rb2) - (r1 + r2) * Math.sin(al)) / (Math.PI * Math.cos(al));
}

/**
 * Tooth-trace load distribution factor KH-beta when tooth contact is not predictable, JGMA 402-01 as KHK Table 10.11
 * prints it, against b / d01, for four supports. Linear between rows (KHK's own worked example takes 1.025 at 0.5);
 * below 0.2 the 0.2 row; beyond the last row of a column, null (no value: unrated).
 */
export const KH_BETA = {
  bd: [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0],
  equidistant: [1, 1, 1.05, 1.1, 1.2, 1.3, 1.4, 1.5, 1.8, 2.1],
  rugged: [1, 1.1, 1.2, 1.3, 1.45, 1.6, 1.8, 2.05, null, null],
  weak: [1.1, 1.3, 1.5, 1.7, 1.85, 2, 2.1, 2.2, null, null],
  overhung: [1.2, 1.45, 1.65, 1.85, 2, 2.15, null, null, null, null],
};
export function khBeta(bOverD, support) {
  const col = KH_BETA[support];
  if (!col) return null;
  const x = KH_BETA.bd;
  if (bOverD <= x[0]) return col[0];
  for (let i = 1; i < x.length; i++) {
    if (bOverD <= x[i]) {
      if (col[i] === null || col[i - 1] === null) return null;
      return col[i - 1] + ((bOverD - x[i - 1]) / (x[i] - x[i - 1])) * (col[i] - col[i - 1]);
    }
  }
  return null;
}

/** JGMA dynamic load factor KV's speed band (KHK Table 10.3): 0 for 1 m/s or under, then 1-3, 3-5, 5-8, 8-12, 12-18, 18-25 m/s. */
export function kvBand(v) {
  const edges = [1, 3, 5, 8, 12, 18, 25];
  const i = edges.findIndex((e) => v <= e);
  return i < 0 ? edges.length : i;
}

/**
 * Round 10: JGMA's dynamic load factor KV (KHK Table 10.3), every row and band as printed (null where the table has
 * a dash). Rows are by the old JIS B 1702 accuracy grade: row g for an unmodified grade-g gear, row g - 1 for a
 * tooth-modified one ('modified 1' is row 0, 'unmodified 1 / modified 2' row 1, ... 'unmodified 6' row 6).
 */
export const KV_TABLE = [
  [null, null, 1.0, 1.0, 1.1, 1.2, 1.3],
  [null, 1.0, 1.05, 1.1, 1.2, 1.3, 1.5],
  [1.0, 1.1, 1.15, 1.2, 1.3, 1.5, null],
  [1.0, 1.2, 1.3, 1.4, 1.5, null, null],
  [1.0, 1.3, 1.4, 1.5, null, null, null],
  [1.1, 1.4, 1.5, null, null, null, null],
  [1.2, 1.5, null, null, null, null, null],
];

/** KHK Table 10.3 row for an old JIS grade g (unmodified: row g; modified: g - 1); finer than the table: its first row; coarser: none. */
export function kvRow(oldGrade, modified = false) {
  if (!Number.isFinite(oldGrade)) return null;
  const r = modified ? oldGrade - 1 : oldGrade;
  return r > KV_TABLE.length - 1 ? null : Math.max(0, r);
}

/**
 * KV at pitch-line speed v on row r. A dash below a row's first printed value (fine grades at low speed) reads as
 * that row's floor 1.0: KV never falls below 1 and does not fall with speed. A dash above a row's last value (the
 * grade is not tabulated for that speed) gives null: no factor.
 */
export function kvFactor(v, row) {
  const r = KV_TABLE[row];
  if (!r) return null;
  const b = kvBand(v);
  if (b >= r.length) return null;
  if (r[b] !== null) return r[b];
  return b < r.findIndex((x) => x !== null) ? 1.0 : null;
}

/**
 * JGMA 402-01 lubrication speed factor ZV (KHK Fig. 10.5, read from the printed curves: approximate), surface
 * hardened or thermal refined. The curves start at 1 m/s and the standard gives no rule below; KHK's own catalogue
 * ratings (600 rpm: 0.57 m/s on an 18-tooth pinion) sit there too, so below 1 m/s the curve's first value is read.
 */
export function zvFactor(v, hardened = true) {
  const pts = hardened ? [[1, 0.955], [3, 0.965], [10, 1]] : [[1, 0.9], [3, 0.92], [10, 1]];
  if (v <= pts[0][0]) return pts[0][1];
  for (let i = 1; i < pts.length; i++) if (v <= pts[i][0]) return pts[i - 1][1] + ((v - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0])) * (pts[i][1] - pts[i - 1][1]);
  return 1;
}

/** KHK's Duracon speed factor KV against pitch-line speed (chart reading, approximate): 1.40 at 0, 1.32 at 5, 1.17 at 10, 0.62 at 20 m/s. */
export function duraconKv(v) {
  const pts = [[0, 1.4], [5, 1.32], [10, 1.17], [20, 0.62]];
  if (v >= 20) return null;
  for (let i = 1; i < pts.length; i++) if (v <= pts[i][0]) return pts[i - 1][1] + ((v - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0])) * (pts[i][1] - pts[i - 1][1]);
  return null;
}

/**
 * Round 12 (SPEC ruling after physics round 11, design life): JGMA 401-01 life factor K_L and JGMA 402-01 K_HL as
 * KHK's technical reference prints them (Tables 10.2 and 10.10; KHK's Japanese PDF gives the same K_L as text; the
 * SDP/SI-numbered copy, Tables 17-2 and 17-10, agrees; scratchpad research/round12-parts.json Q1). Rows by the number
 * of loaded meshing cycles in the life; K_L columns HB 120-220, over HB 220, carburized or nitrided (induction
 * hardened: the core hardness). The tables cap at 1.5 and give 1.0 at 1e7 and over. No source states a rule between
 * rows, so a count takes the row of the smallest tabulated count at or above it (conservative): 1e4, 1e5, 1e6, else
 * the 1e7 row.
 */
export const LIFE_ROWS = [1e4, 1e5, 1e6];
export const KL_TABLE = { hb120_220: [1.4, 1.2, 1.1], hb_over_220: [1.5, 1.4, 1.1], carburized_nitrided: [1.5, 1.5, 1.1] };
export const KHL_TABLE = [1.5, 1.3, 1.15];

/** Index of the life row a cycle count reads (smallest tabulated count >= n), or -1 for the 1e7 row (factor 1). */
export function lifeRow(n) {
  if (!(n >= 0)) return -1;
  return LIFE_ROWS.findIndex((r) => n <= r);
}

/** K_L and K_HL at n cycles for a K_L column; 1 and 1 where the count is unknown (KHK: "assumed to be 1.0"). */
export function jgmaLifeFactors(n, column = 'hb120_220') {
  const i = lifeRow(n);
  const col = KL_TABLE[column] ?? KL_TABLE.hb120_220;
  return { row: i < 0 ? '1e7 and over' : LIFE_ROWS[i], KL: i < 0 ? 1 : col[i], KHL: i < 0 ? 1 : KHL_TABLE[i] };
}

/**
 * KHK's plastic gear method (Fig. 11.3, Duracon allowable bending stress against cycles, 40 C, grease): read off the
 * printed curves (no printed numbers; reading +-0.05 kgf/mm2, so 0.05 is taken off), m 0.8 / 1 / 2 at 1e5 and 1e6
 * cycles (kgf/mm2). KHK's catalogue figure for m 1 is 3.5, which the m 1 curve reaches at about 1.3e7 cycles.
 */
export const DURACON_LIFE = { rows: [1e5, 1e6], m: { 0.8: [5.03, 4.45], 1: [4.54, 3.97], 2: [2.07, 1.53] }, catalogue: { 0.8: 4.0, 1: 3.5 }, reading_allowance: 0.05 };

/** The Duracon life factor at n cycles for module m: (chart stress at the row - allowance) / catalogue stress; else 1. */
export function duraconLifeFactor(n, m) {
  const curve = DURACON_LIFE.m[m], cat = DURACON_LIFE.catalogue[m];
  if (!curve || !cat || !(n >= 0)) return { row: null, factor: 1 };
  const i = DURACON_LIFE.rows.findIndex((r) => n <= r);
  if (i < 0) return { row: '1e7 level (catalogue)', factor: 1 };
  return { row: DURACON_LIFE.rows[i], factor: Math.max(1, (curve[i] - DURACON_LIFE.reading_allowance) / cat) };
}

/** Pitch-line speed (m/s) of a gear of z teeth, module m (mm), at n rpm. */
export const pitchLineSpeed = (m, z, rpm) => (Math.PI * (m / 1000) * z * rpm) / 60;

/**
 * The speed condition of a metal gear's KHK rating: KV (KHK Table 10.3) and ZV (Fig. 10.5) at the rated pitch-line
 * speed vR and the actual one. With an accuracy row (mesh.kv_row, round 10): KV on that row at both speeds, the ratio
 * capped at 1 (no credit for running slower), half only where the row prints nothing at the actual speed. Without one:
 * bands compared, a higher actual band has no correction (half). Returns { condition, bending, surface, half }.
 */
export function speedCorrection(mesh, vR, rpm = 600) {
  const bA = kvBand(mesh.v_m_s), bR = kvBand(vR);
  const zvS = (zvFactor(mesh.v_m_s, mesh.hardened !== false) / zvFactor(vR, mesh.hardened !== false)) ** 2;
  const speed = { condition: 'speed', rated: `${rpm} rpm (${vR.toFixed(3)} m/s on this gear)`, actual: `${mesh.v_m_s.toFixed(3)} m/s`, holds: Math.abs(mesh.v_m_s - vR) < 1e-9 };
  if (Number.isInteger(mesh.kv_row)) {
    const kvA = kvFactor(mesh.v_m_s, mesh.kv_row), kvR = kvFactor(vR, mesh.kv_row);
    const fk = kvA !== null && kvR !== null ? Math.min(1, kvR / kvA) : null;
    if (fk === null) return { condition: { ...speed, kv_row: mesh.kv_row, kv: { rated: kvR, actual: kvA }, correction: `KV: row ${mesh.kv_row} prints no value at ${mesh.v_m_s.toFixed(2)} m/s: no correction, half`, bending_factor: null, surface_factor: null }, half: true };
    return { condition: { ...speed, kv_row: mesh.kv_row, kv: { rated: kvR, actual: kvA }, zv_surface_factor: zvS, correction: `JGMA KV (KHK Table 10.3, row ${mesh.kv_row}) ${kvR} rated / ${kvA} actual on both strengths; ZV (Fig. 10.5) squared on surface`, bending_factor: fk, surface_factor: fk * zvS }, bending: fk, surface: fk * zvS, half: false };
  }
  if (bA > bR) return { condition: { ...speed, correction: 'KV: a higher speed band than rated and no accuracy-grade row stated: no correction, half', bending_factor: null, surface_factor: null }, half: true };
  return { condition: { ...speed, correction: `KV: the actual speed band (${bA}) is not above the rated one (${bR}), so KV is not larger: no credit taken; ZV (Fig. 10.5) squared`, bending_factor: 1, surface_factor: zvS }, bending: 1, surface: zvS, half: false };
}

/**
 * One metal gear's KHK ratings (bending, surface; N m on that gear) corrected from KHK's stated conditions to the
 * design's: mesh { z (this gear), zMate, m, b_mm, v_m_s (the mesh's pitch-line speed), support ('equidistant' |
 * 'rugged' | 'weak' | 'overhung'), lubricant_cSt_50C, reversing, hardened }, rated { zMate (KHK: same teeth), rpm,
 * lubricant_cSt_50C, support, bidirectional }. Returns the corrected values and every condition with its factor.
 */
export function jgmaGearRating(cat, mesh, rated = {}) {
  const r = { zMate: mesh.z, rpm: 600, lubricant_cSt_50C: 100, support: 'equidistant', bidirectional: true, ...rated };
  const conds = [];
  let fB = 1, fS = 1, half = false;
  // Mating gear: KHK rates against a gear of the same number of teeth.
  const zp = Math.min(mesh.z, mesh.zMate), zw = Math.max(mesh.z, mesh.zMate);
  const zpR = Math.min(mesh.z, r.zMate), zwR = Math.max(mesh.z, r.zMate);
  const iA = zw / zp, iR = zwR / zpR;
  const dA = mesh.m * zp, dR = mesh.m * zpR;
  const khA = khBeta(mesh.b_mm / dA, mesh.support), khR = khBeta(mesh.b_mm / dR, r.support);
  const epsA = contactRatio(mesh.z, mesh.zMate), epsR = contactRatio(mesh.z, r.zMate);
  const mateS = (dA * (iA / (iA + 1))) / (dR * (iR / (iR + 1)));
  const supS = khA !== null && khR !== null ? khR / khA : null;
  conds.push({ condition: 'mating gear', rated: `${r.zMate} teeth`, actual: `${mesh.zMate} teeth`, holds: mesh.zMate === r.zMate, correction: 'JGMA 402-01 eq. 10.17: d01 i / (i + 1) (d01 the pinion, i = z2 / z1); JGMA 401-01: Y-epsilon = 1 / eps_alpha', surface_factor: mateS, bending_factor: epsA / epsR, eps_alpha: { rated: epsR, actual: epsA } });
  fS *= mateS;
  fB *= epsA / epsR;
  conds.push({ condition: 'gear support', rated: `${r.support}, b/d01 ${(mesh.b_mm / dR).toFixed(3)}, KH-beta ${khR?.toFixed(3)}`, actual: `${mesh.support}, b/d01 ${(mesh.b_mm / dA).toFixed(3)}, KH-beta ${khA === null ? 'none' : khA.toFixed(3)}`, holds: mesh.support === r.support && khA === khR, correction: 'JGMA 402-01 KH-beta (KHK Table 10.11, contact not predictable)', surface_factor: supS });
  if (supS === null) half = true; else fS *= supS;
  // Speed: KV and ZV, JGMA's own factors, at the rated and the actual pitch-line speed.
  const sp = speedCorrection(mesh, pitchLineSpeed(mesh.m, mesh.z, r.rpm), r.rpm);
  conds.push(sp.condition);
  if (sp.half) half = true; else { fB *= sp.bending; fS *= sp.surface; }
  // Lubricant: ZL rises with viscosity; at or above the rated 100 cSt (50 C) no credit is taken.
  const lubOk = Number.isFinite(mesh.lubricant_cSt_50C) && mesh.lubricant_cSt_50C >= r.lubricant_cSt_50C;
  conds.push({ condition: 'lubricant', rated: `${r.lubricant_cSt_50C} cSt at 50 C`, actual: Number.isFinite(mesh.lubricant_cSt_50C) ? `${mesh.lubricant_cSt_50C} cSt at 50 C` : 'not stated', holds: lubOk, correction: lubOk ? 'ZL at or above the rated value: no credit' : 'ZL for a lower or unstated viscosity: no correction applied, half', surface_factor: lubOk ? 1 : null });
  if (!lubOk) half = true;
  // Load direction: KHK already takes 2/3 of sigma_Flim (bidirectional); surface has no reversing reduction (JGMA 402).
  const dirOk = r.bidirectional || !mesh.reversing;
  conds.push({ condition: 'load direction', rated: r.bidirectional ? 'bidirectional (2/3 of sigma_Flim)' : 'unidirectional', actual: mesh.reversing ? 'reversing (the wing back-drives the train)' : 'one direction', holds: dirOk, correction: dirOk ? 'none needed' : 'JGMA 401-01: 2/3 of sigma_Flim', bending_factor: dirOk ? 1 : 2 / 3 });
  if (!dirOk) fB *= 2 / 3;
  // Cycles and impact: rated over 1e7 cycles (KL = KHL = 1) and uniform load (KO = 1); the rating is applied to the
  // simulated instantaneous peak torque, gusts included. Round 12: with a stated design life (mesh.cycles, this gear's
  // own revolutions over it, every one at the peak), JGMA's own life factors: K_L on bending, K_HL squared on surface
  // (torque goes as the square of the Hertz stress).
  if (Number.isFinite(mesh.cycles)) {
    const lf = jgmaLifeFactors(mesh.cycles, mesh.lifeColumn);
    conds.push({ condition: 'cycles', rated: 'over 1e7 (KL = KHL = 1)', actual: `${mesh.cycles.toExponential(3)} cycles (design life, every cycle at the peak)`, holds: lf.KL === 1 && lf.KHL === 1, correction: `JGMA 401-01 K_L ${lf.KL} (${mesh.lifeColumn ?? 'hb120_220'}), JGMA 402-01 K_HL ${lf.KHL} (row ${lf.row}; a count takes the row at or above it)`, bending_factor: lf.KL, surface_factor: lf.KHL * lf.KHL, life: lf });
    fB *= lf.KL;
    fS *= lf.KHL * lf.KHL;
  } else conds.push({ condition: 'cycles', rated: 'over 1e7 (KL = KHL = 1)', actual: 'not limited; no life credit', holds: true, correction: 'none' });
  conds.push({ condition: 'impact', rated: 'uniform load (KO = 1)', actual: 'applied to the time-resolved peak torque of the M7 population', holds: true, correction: 'none' });
  const k = half ? 0.5 : 1;
  const bending = Number.isFinite(cat.bending_Nm) ? cat.bending_Nm * fB * k : null;
  const surface = Number.isFinite(cat.surface_Nm) ? cat.surface_Nm * fS * k : null;
  return { method: 'JGMA 401-01 / 402-01 (KHK)', catalogue: { bending_Nm: cat.bending_Nm ?? null, surface_Nm: cat.surface_Nm ?? null }, bending_factor: fB * k, surface_factor: fS * k, half_rule: half, bending_Nm: bending, surface_Nm: surface, allowable_Nm: Math.min(...[bending, surface].filter(Number.isFinite)), conditions: conds };
}

/**
 * One Duracon gear's KHK Lewis rating corrected to the design's conditions (KHK's plastic gear method): speed factor
 * KV (the rated 100 rpm against the actual pitch-line speed), material factor KM 0.75 when the mate is Duracon too (the
 * catalogue states no mate, so it is taken as rated against metal), JGMA 401's 2/3 for a reversing load (the method
 * has no reversing rule), grease and 40 C as stated (half if the design states no grease or a hotter gearbox).
 */
export function lewisDuraconRating(cat, mesh, rated = {}) {
  const r = { rpm: 100, temperature_C: 40, grease: true, ...rated };
  const conds = [];
  let f = 1, half = false;
  const vR = pitchLineSpeed(mesh.m, mesh.z, r.rpm);
  const kvA = duraconKv(mesh.v_m_s), kvR = duraconKv(vR);
  const fv = kvA !== null ? kvA / kvR : null;
  conds.push({ condition: 'speed', rated: `${r.rpm} rpm (${vR.toFixed(3)} m/s)`, actual: `${mesh.v_m_s.toFixed(3)} m/s`, holds: false, correction: 'KHK plastic gear speed factor KV (chart reading)', factor: fv });
  if (fv === null) half = true; else f *= fv;
  const km = mesh.mateDuracon ? 0.75 : 1;
  conds.push({ condition: 'mating material', rated: 'not stated (taken as metal, KM 1)', actual: mesh.mateDuracon ? 'Duracon' : 'metal', holds: !mesh.mateDuracon, correction: 'KHK plastic gear material factor KM', factor: km });
  f *= km;
  const fd = mesh.reversing ? 2 / 3 : 1;
  conds.push({ condition: 'load direction', rated: 'not stated', actual: mesh.reversing ? 'reversing' : 'one direction', holds: !mesh.reversing, correction: 'JGMA 401-01 2/3 (the plastic gear method has no reversing rule)', factor: fd });
  f *= fd;
  const lubOk = mesh.grease === true;
  conds.push({ condition: 'lubrication', rated: 'grease', actual: lubOk ? 'grease' : 'not stated', holds: lubOk, correction: lubOk ? 'none' : 'no correction applied: half', factor: lubOk ? 1 : null });
  if (!lubOk) half = true;
  const tOk = Number.isFinite(mesh.temperature_C) && mesh.temperature_C <= r.temperature_C;
  conds.push({ condition: 'temperature', rated: `${r.temperature_C} C`, actual: Number.isFinite(mesh.temperature_C) ? `${mesh.temperature_C} C (stated design limit)` : 'not stated', holds: tOk, correction: tOk ? 'KT not below the rated: no credit' : 'no correction applied: half', factor: tOk ? 1 : null });
  if (!tOk) half = true;
  if (Number.isFinite(mesh.cycles)) {
    const lf = duraconLifeFactor(mesh.cycles, mesh.m);
    conds.push({ condition: 'cycles', rated: 'the catalogue stress (the m 1 curve of KHK Fig. 11.3 reaches 3.5 kgf/mm2 near 1.3e7 cycles)', actual: `${mesh.cycles.toExponential(3)} cycles (design life, every cycle at the peak)`, holds: lf.factor === 1, correction: `KHK Fig. 11.3 read at the row at or above the count (${lf.row}), less 0.05 kgf/mm2 for the reading`, factor: lf.factor, life: lf });
    f *= lf.factor;
  } else conds.push({ condition: 'cycles', rated: 'sigma_b 3.5 kgf/mm2 (the 1e7-cycle level of KHK\'s chart)', actual: 'no life credit', holds: true, correction: 'none' });
  const k = half ? 0.5 : 1;
  const bending = Number.isFinite(cat.bending_Nm) ? cat.bending_Nm * f * k : null;
  return { method: 'Lewis (KHK plastic gear method)', catalogue: { bending_Nm: cat.bending_Nm ?? null, surface_Nm: null }, bending_factor: f * k, half_rule: half, bending_Nm: bending, surface_Nm: null, allowable_Nm: bending, conditions: conds };
}
