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

/** Pitch-line speed (m/s) of a gear of z teeth, module m (mm), at n rpm. */
export const pitchLineSpeed = (m, z, rpm) => (Math.PI * (m / 1000) * z * rpm) / 60;

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
  // Speed: KV by band (grade-independent comparison: KV does not fall with speed), ZV by the printed curve.
  const vR = pitchLineSpeed(mesh.m, mesh.z, r.rpm);
  const bA = kvBand(mesh.v_m_s), bR = kvBand(vR);
  const kvOk = bA <= bR;
  const zvS = (zvFactor(mesh.v_m_s, mesh.hardened !== false) / zvFactor(vR, mesh.hardened !== false)) ** 2;
  conds.push({ condition: 'speed', rated: `${r.rpm} rpm (${vR.toFixed(3)} m/s on this gear)`, actual: `${mesh.v_m_s.toFixed(3)} m/s`, holds: Math.abs(mesh.v_m_s - vR) < 1e-9, correction: kvOk ? `KV: the actual speed band (${bA}) is not above the rated one (${bR}), so KV is not larger: no credit taken; ZV (Fig. 10.5) squared` : 'KV: a higher speed band than rated and the KHK grade-to-JGMA row is not resolved: no correction, half', bending_factor: kvOk ? 1 : null, surface_factor: kvOk ? zvS : null });
  if (!kvOk) half = true; else fS *= zvS;
  // Lubricant: ZL rises with viscosity; at or above the rated 100 cSt (50 C) no credit is taken.
  const lubOk = Number.isFinite(mesh.lubricant_cSt_50C) && mesh.lubricant_cSt_50C >= r.lubricant_cSt_50C;
  conds.push({ condition: 'lubricant', rated: `${r.lubricant_cSt_50C} cSt at 50 C`, actual: Number.isFinite(mesh.lubricant_cSt_50C) ? `${mesh.lubricant_cSt_50C} cSt at 50 C` : 'not stated', holds: lubOk, correction: lubOk ? 'ZL at or above the rated value: no credit' : 'ZL for a lower or unstated viscosity: no correction applied, half', surface_factor: lubOk ? 1 : null });
  if (!lubOk) half = true;
  // Load direction: KHK already takes 2/3 of sigma_Flim (bidirectional); surface has no reversing reduction (JGMA 402).
  const dirOk = r.bidirectional || !mesh.reversing;
  conds.push({ condition: 'load direction', rated: r.bidirectional ? 'bidirectional (2/3 of sigma_Flim)' : 'unidirectional', actual: mesh.reversing ? 'reversing (the wing back-drives the train)' : 'one direction', holds: dirOk, correction: dirOk ? 'none needed' : 'JGMA 401-01: 2/3 of sigma_Flim', bending_factor: dirOk ? 1 : 2 / 3 });
  if (!dirOk) fB *= 2 / 3;
  // Cycles and impact: rated over 1e7 cycles (KL = KHL = 1) and uniform load (KO = 1); no life credit is taken, and
  // the rating is applied to the simulated instantaneous peak torque, gusts included.
  conds.push({ condition: 'cycles', rated: 'over 1e7 (KL = KHL = 1)', actual: 'not limited; no life credit', holds: true, correction: 'none' });
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
  conds.push({ condition: 'cycles', rated: 'sigma_b 3.5 kgf/mm2 (the 1e7-cycle level of KHK\'s chart)', actual: 'no life credit', holds: true, correction: 'none' });
  const k = half ? 0.5 : 1;
  const bending = Number.isFinite(cat.bending_Nm) ? cat.bending_Nm * f * k : null;
  return { method: 'Lewis (KHK plastic gear method)', catalogue: { bending_Nm: cat.bending_Nm ?? null, surface_Nm: null }, bending_factor: f * k, half_rule: half, bending_Nm: bending, surface_Nm: null, allowable_Nm: bending, conditions: conds };
}
