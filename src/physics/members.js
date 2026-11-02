// Round 6: every loaded part of the drive is linkage (SPEC ruling after physics round 5). Shafts, pins, plate bearing
// seats, coupler end fittings, gear bore adapters, keys and bonds, each checked at its peak load from the M7 load
// population against published material strength with a factor of safety of 1.5. Pure functions of section,
// material and load. The loads are the flight model's load channels (flight.js LOAD_CHANNELS), combined at every step
// with non-negative coefficients exactly as the bearing supports are (a member's "sections"), so a section's peak is
// the peak of a bound that holds whatever the directions of the forces it sums.
import { FOS_STATIC } from './bearings.js';

export const FOS = FOS_STATIC;

/** Round section, solid (di 0) or hollow, diameters in m: area, second moment, polar moment, section moduli. */
export function roundSection(d, di = 0) {
  const A = (Math.PI / 4) * (d * d - di * di);
  const I = (Math.PI / 64) * (d ** 4 - di ** 4);
  return { A, I, J: 2 * I, Z: I / (d / 2), Zp: (2 * I) / (d / 2) };
}

/**
 * Bending moment at x of a beam on simple supports at xA, xB from a unit transverse load at xi (loads may overhang).
 * Statics: the moment of every force to one side of x about x.
 */
export function unitMoment(xA, xB, xi, x) {
  const L = xB - xA;
  const RA = (xB - xi) / L, RB = (xi - xA) / L;
  let m = 0;
  for (const [f, p] of [[RA, xA], [RB, xB], [-1, xi]]) if (p < x) m += f * (x - p);
  return m;
}

/**
 * Sections of a shaft on two supports for the flight model: at every support and load plane, the bound on the
 * bending moment sum_i |F_i| |m_i(x)| as channel terms. loads: [{ ch, perUnit, x }] (force = perUnit x channel).
 * Returns [{ name, x_m, radial: [{ ch, k }] }] (k in N m per channel unit).
 */
export function shaftMomentSections(xA, xB, loads, name = 'M') {
  const xs = [...new Set([xA, xB, ...loads.map((l) => l.x)].map((v) => +v.toFixed(6)))].sort((a, b) => a - b);
  return xs.map((x) => {
    const terms = new Map();
    for (const l of loads) {
      const k = Math.abs(l.perUnit * unitMoment(xA, xB, l.x, x));
      if (k > 1e-12) terms.set(l.ch, (terms.get(l.ch) ?? 0) + k);
    }
    return { name: `${name} at x ${(x * 1000).toFixed(1)} mm`, x_m: x, radial: [...terms].map(([ch, k]) => ({ ch, k: +k.toFixed(8) })) };
  }).filter((s) => s.radial.length);
}

/**
 * Round shaft or pin under bending M, torque T and transverse shear V: von Mises at the two critical points of the
 * section, the outer fibre (bending with torsion, where transverse shear is zero) and the neutral axis (torsion plus
 * the transverse shear peak, 4/3 V/A solid or 2 V/A for a thin tube, where bending is zero); the larger governs.
 * material: { yield_MPa }.
 */
export function roundMemberCheck({ d_mm, di_mm = 0 }, material, { M = 0, T = 0, V = 0 }) {
  const s = roundSection(d_mm / 1000, di_mm / 1000);
  const sigma = M / s.Z / 1e6;
  const tauT = T / s.Zp / 1e6;
  const tauV = ((di_mm > 0 ? 2 : 4 / 3) * V) / s.A / 1e6;
  const vmOuter = Math.sqrt(sigma * sigma + 3 * tauT * tauT);
  const vmAxis = Math.sqrt(3) * (tauT + tauV);
  const vm = Math.max(vmOuter, vmAxis);
  const fos = vm > 0 ? material.yield_MPa / vm : Infinity;
  return { sigma_MPa: sigma, tau_torsion_MPa: tauT, tau_shear_MPa: tauV, von_mises_MPa: vm, governs: vmOuter >= vmAxis ? 'outer fibre' : 'neutral axis', yield_MPa: material.yield_MPa, fos, ok: fos >= FOS };
}

/** Pin in single or double shear (planes) of a solid round: tau = F / (planes A) against the shear yield. */
export function pinShearCheck({ d_mm }, material, F, planes = 1) {
  const A = (Math.PI / 4) * (d_mm / 1000) ** 2;
  const tau = F / (planes * A) / 1e6;
  const allow = material.shear_yield_MPa ?? material.yield_MPa / Math.sqrt(3);
  const fos = tau > 0 ? allow / tau : Infinity;
  return { tau_MPa: tau, shear_allowable_MPa: allow, fos, ok: fos >= FOS };
}

/**
 * Bearing allowable of a seat: the published bearing yield where the seat's edge distance / hole diameter reaches the
 * value it is quoted at; otherwise the material's tensile yield, a lower bound (bearing yield exceeds tensile yield in
 * every published metal table), and null where neither is published.
 */
export function bearingAllowable(material, eOverD = null) {
  if (Number.isFinite(material.bearing_yield_MPa) && Number.isFinite(eOverD) && eOverD >= (material.bearing_e_over_d_min ?? Infinity)) return { MPa: material.bearing_yield_MPa, basis: `published bearing yield at e/D >= ${material.bearing_e_over_d_min}` };
  if (Number.isFinite(material.yield_MPa)) return { MPa: material.yield_MPa, basis: 'tensile yield as a lower bound on bearing yield' };
  return { MPa: null, basis: 'no published strength' };
}

/**
 * A round part of diameter d held in a seat of length t (a pin pressed in a boss, a shaft in a hub): projected
 * pressure F / (d t) plus, when the part is cantilevered from the seat with moment M, the linear edge pressure
 * 6 M / (d t^2) of a rigid pin in an elastic seat. Against bearingAllowable(material, e/D).
 */
export function seatCheck({ d_mm, t_mm, e_over_d = null }, material, F, M = 0) {
  const d = d_mm / 1000, t = t_mm / 1000;
  const p = (F / (d * t) + (6 * M) / (d * t * t)) / 1e6;
  const a = bearingAllowable(material, e_over_d);
  if (!Number.isFinite(a.MPa)) return { p_MPa: p, bearing_allowable_MPa: null, fos: null, ok: false, why: a.basis };
  const fos = p > 0 ? a.MPa / p : Infinity;
  return { p_MPa: p, bearing_allowable_MPa: a.MPa, allowable_basis: a.basis, fos, ok: fos >= FOS };
}

/**
 * A ring lug (an eye or a housing of outside diameter D round a hole d, thickness t) carrying F in any direction:
 * bearing at the hole (seatCheck), net-section tension F / ((D - d) t) and shear-out F / (2 t (D - d) / 2) against
 * the material's tensile and shear yield. The smallest factor governs.
 */
export function lugCheck({ d_mm, D_mm, t_mm }, material, F) {
  const t = t_mm / 1000, w = (D_mm - d_mm) / 1000;
  const brg = seatCheck({ d_mm, t_mm, e_over_d: D_mm / 2 / d_mm }, material, F);
  const sTen = F / (w * t) / 1e6;
  const sShear = F / (2 * t * (w / 2)) / 1e6;
  const tauY = material.shear_yield_MPa ?? material.yield_MPa / Math.sqrt(3);
  const fosT = sTen > 0 ? material.yield_MPa / sTen : Infinity;
  const fosS = sShear > 0 ? tauY / sShear : Infinity;
  const fos = Math.min(brg.fos ?? 0, fosT, fosS);
  return { bearing_MPa: brg.p_MPa, bearing_allowable_MPa: brg.bearing_allowable_MPa, net_tension_MPa: sTen, shear_out_MPa: sShear, fos_bearing: brg.fos, fos_tension: fosT, fos_shear_out: fosS, fos, ok: fos >= FOS };
}

/**
 * A rectangular web b x h (h along the shaft) bent out of its plane by M (about the b axis) and in its plane by F at
 * lever (about the h axis): the corner stress is the sum, against the tensile yield.
 */
export function webCheck({ b_mm, h_mm, lever_mm }, material, { M = 0, F = 0 }) {
  const b = b_mm / 1000, h = h_mm / 1000;
  const sigma = ((6 * M) / (b * h * h) + (6 * F * (lever_mm / 1000)) / (h * b * b)) / 1e6;
  const fos = sigma > 0 ? material.yield_MPa / sigma : Infinity;
  return { sigma_MPa: sigma, yield_MPa: material.yield_MPa, fos, ok: fos >= FOS };
}

/** Cylindrical bonded joint (retaining compound or adhesive) of diameter d and length L: torque and axial capacity. */
export function bondCheck({ d_mm, L_mm }, adhesive, { T = 0, F = 0 }) {
  const A = Math.PI * (d_mm / 1000) * (L_mm / 1000);
  const tau = Math.hypot((2 * T) / (d_mm / 1000), F) / A / 1e6;
  const allow = adhesive.shear_MPa * (adhesive.factor ?? 1);
  const fos = tau > 0 ? allow / tau : Infinity;
  return { tau_MPa: tau, shear_allowable_MPa: allow, fos, ok: fos >= FOS };
}

/**
 * Parallel key transmitting torque T on a shaft of diameter d: shear over its width b and length L, and bearing on the
 * engaged depth h of the side checked (the shaft's keyseat), against the key's and that side's yield (seat material
 * without a published bearing yield: its tensile yield). The other side (the gear's own keyway) is the gear maker's.
 */
export function keyCheck({ d_mm, b_mm, h_mm, L_mm }, keyMat, sideMat, T) {
  const F = T / (d_mm / 2000);
  const tau = F / ((b_mm / 1000) * (L_mm / 1000)) / 1e6;
  const p = F / ((h_mm / 1000) * (L_mm / 1000)) / 1e6;
  const fosShear = (keyMat.shear_yield_MPa ?? keyMat.yield_MPa / Math.sqrt(3)) / tau;
  const brg = Math.min(keyMat.yield_MPa, bearingAllowable(sideMat).MPa ?? 0);
  const fosBearing = brg / p;
  const fos = Math.min(fosShear, fosBearing);
  return { force_N: F, tau_MPa: tau, bearing_MPa: p, fos_shear: fosShear, fos_bearing: fosBearing, fos, ok: fos >= FOS };
}

/** Peak value of every member section from the flight model's support peaks (the members follow the joints). */
export function memberPeaks(members, supPeaks, offset) {
  const out = {};
  let k = offset;
  for (const m of members ?? []) for (const s of m.sections ?? []) out[`${m.name}/${s.name}`] = supPeaks[k++]?.Fr ?? 0;
  return out;
}

/**
 * Every member against its check. member: { name, kind, check, dims, material, sections: [...], uses: { M, T, V, F }
 * naming the sections whose peaks it takes (several names: the largest) }. Members with check 'unrated' fail.
 */
export function checkMembers(members, peaks) {
  const rows = [];
  for (const m of members ?? []) {
    const val = (names) => Math.max(0, ...[].concat(names ?? []).map((n) => peaks[`${m.name}/${n}`] ?? 0));
    const u = m.uses ?? {};
    const load = { M: val(u.M), T: val(u.T), V: val(u.V), F: val(u.F) };
    let r;
    if (m.check === 'round') r = roundMemberCheck(m.dims, m.material, load);
    else if (m.check === 'pin-shear') r = pinShearCheck(m.dims, m.material, load.F, m.dims.planes ?? 1);
    else if (m.check === 'seat') r = seatCheck(m.dims, m.material, load.F, load.M);
    else if (m.check === 'lug') r = lugCheck(m.dims, m.material, load.F);
    else if (m.check === 'web') r = webCheck(m.dims, m.material, { M: load.M, F: load.F });
    else if (m.check === 'bond') r = bondCheck(m.dims, m.material, { T: load.T, F: load.F });
    else if (m.check === 'key') r = keyCheck(m.dims, m.material, m.side_material, load.T);
    else r = { fos: null, ok: false, why: `no check for ${m.check}` };
    rows.push({ name: m.name, kind: m.kind, check: m.check, material: m.material?.id ?? null, dims: m.dims, load, ...r, ...(m.note ? { note: m.note } : {}) });
  }
  const rated = rows.filter((r) => Number.isFinite(r.fos));
  return { ok: rows.every((r) => r.ok), fos_min: rated.length ? Math.min(...rated.map((r) => r.fos)) : null, worst: rated.length ? rated.reduce((a, b) => (b.fos < a.fos ? b : a)).name : null, members: rows };
}
