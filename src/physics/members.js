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

/**
 * Round 8 (airframe load path). A group of n identical fasteners joining two parts, each with a rated tension Pt and a
 * rated shear Ps (N; a fastener's own maker or ISO 898-1 / ISO 3506 minimum, as its catalogue entry states). Loads:
 * in-plane shear V (any direction), in-plane torsion T about the group centroid, out-of-plane moment M (prying) and
 * out-of-plane force F. Per fastener: shear V / n + T r_max / sum r^2; tension F / n + M / (n_far lever), the moment
 * taken by the n_far fasteners of the far row about the near row (lever: the row spacing; a pivot on the near row,
 * not the part's edge, is the conservative choice). Linear interaction s / Ps + t / Pt, so the load factor to failure
 * is its inverse.
 */
export function boltGroupCheck({ n, r_max_mm = 0, sum_r2_mm2 = 0, lever_mm = Infinity, n_far = 1 }, fastener, { V = 0, T = 0, M = 0, F = 0 }) {
  const shear = V / n + (sum_r2_mm2 > 0 ? (T * (r_max_mm / 1000)) / (sum_r2_mm2 * 1e-6) : 0);
  const tension = F / n + (Number.isFinite(lever_mm) && lever_mm > 0 ? M / (n_far * (lever_mm / 1000)) : 0);
  const u = shear / fastener.shear_N + tension / fastener.tension_N;
  const fos = u > 0 ? 1 / u : Infinity;
  return { shear_each_N: shear, tension_each_N: tension, shear_rated_N: fastener.shear_N, tension_rated_N: fastener.tension_N, interaction: u, fos, ok: fos >= FOS };
}

/**
 * A strut (rectangular b x t, or round d / di) of length L with end-fixity factor K under axial force F and a bending
 * moment M about its weak axis. Euler load Pcr = pi^2 E I / (K L)^2; the stress F / A + M / Z / (1 - F / Pcr) (the
 * beam-column amplification) against the yield. fos is the load factor at which either is reached (bisection on the
 * factored load, so the amplification is applied to the factored load, not the working one).
 */
export function strutCheck({ L_mm, K = 1, b_mm, t_mm, d_mm, di_mm = 0 }, material, { F = 0, M = 0 }) {
  const rect = Number.isFinite(b_mm);
  const A = rect ? (b_mm * t_mm) * 1e-6 : roundSection(d_mm / 1000, di_mm / 1000).A;
  const I = rect ? (b_mm / 1000) * (t_mm / 1000) ** 3 / 12 : roundSection(d_mm / 1000, di_mm / 1000).I;
  const c = rect ? t_mm / 2000 : d_mm / 2000;
  const E = material.E_GPa * 1e9;
  const Pcr = (Math.PI ** 2 * E * I) / ((K * L_mm) / 1000) ** 2;
  const stress = (lam) => (lam * F / A + (lam * M * c / I) / Math.max(1e-12, 1 - (lam * F) / Pcr)) / 1e6;
  const fy = material.yield_MPa;
  let fos;
  if (F <= 0 && M <= 0) fos = Infinity;
  else {
    let lo = 0, hi = F > 0 ? Pcr / F : 1e6;
    if (stress(hi * (1 - 1e-9)) <= fy && F > 0) fos = hi; // buckling first
    else {
      for (let i = 0; i < 100; i++) { const mid = 0.5 * (lo + hi); if (stress(mid) > fy) hi = mid; else lo = mid; }
      fos = lo;
    }
  }
  return { stress_MPa: stress(1), euler_N: Pcr, fos_buckling: F > 0 ? Pcr / F : Infinity, yield_MPa: fy, fos, ok: fos >= FOS };
}

/** A flat shear panel h x b (h the loaded edge) of thickness t carrying shear V: tau = V / (h t) against the shear yield, and simply supported shear buckling tau_cr = k pi^2 E / (12 (1 - nu^2)) (t / b)^2 with k = 5.35 + 4 (b / a)^2, a >= b the panel's long side. */
export function panelShearCheck({ h_mm, b_mm, t_mm, nu = 0.33 }, material, V) {
  const a = Math.max(h_mm, b_mm), s = Math.min(h_mm, b_mm);
  const k = 5.35 + 4 * (s / a) ** 2;
  const tauCr = (k * Math.PI ** 2 * material.E_GPa * 1e3) / (12 * (1 - nu * nu)) * (t_mm / s) ** 2; // MPa
  const tau = V / (h_mm * t_mm); // N/mm2 = MPa
  const tauY = material.shear_yield_MPa ?? material.yield_MPa / Math.sqrt(3);
  const fos = tau > 0 ? Math.min(tauY, tauCr) / tau : Infinity;
  return { tau_MPa: tau, shear_yield_MPa: tauY, tau_buckling_MPa: tauCr, fos, ok: fos >= FOS };
}

/** n cable ties (or straps) each with a rated minimum loop tensile strength, holding a force F (plus a fixed extra_N, e.g. a drag bound) between them. */
export function tieCheck({ n, extra_N = 0 }, tie, F) {
  const Ft = F + extra_N;
  const fos = Ft > 0 ? (n * tie.loop_N) / Ft : Infinity;
  return { force_N: Ft, rated_each_N: tie.loop_N, fos, ok: fos >= FOS };
}

/**
 * A rectangular plate section b x t (b in the plate's plane) under axial force F, in-plane bending M (about the
 * thickness axis, section modulus t b^2 / 6) and out-of-plane bending Mo (b t^2 / 6): the corner stress, summed,
 * against the yield. Linear in the loads, so yield / stress is the load factor.
 */
export function plateSectionCheck({ b_mm, t_mm }, material, { F = 0, M = 0, Mo = 0 }) {
  const b = b_mm / 1000, t = t_mm / 1000;
  const sigma = (F / (b * t) + (6 * M) / (t * b * b) + (6 * Mo) / (b * t * t)) / 1e6;
  const fos = sigma > 0 ? material.yield_MPa / sigma : Infinity;
  return { sigma_MPa: sigma, yield_MPa: material.yield_MPa, fos, ok: fos >= FOS };
}

/** A bent-sheet channel (web w, two flanges h, thickness t, mm): area and section moduli for bending that bends the web out of its plane (Zo, flange tips the extreme fibre) and in it (Zi). */
export function channelSection(w, h, t) {
  const Aw = w * t, Af = h * t;
  const A = Aw + 2 * Af;
  const yc = (2 * Af * (h / 2)) / A; // from the web's mid-plane, towards the flanges
  const Io = (w * t ** 3) / 12 + Aw * yc ** 2 + 2 * ((t * h ** 3) / 12 + Af * (h / 2 - yc) ** 2);
  const Ii = (t * w ** 3) / 12 + 2 * ((h * t ** 3) / 12 + Af * (w / 2) ** 2);
  return { A_mm2: A, Zo_mm3: Io / Math.max(yc + t / 2, h - yc), Zi_mm3: Ii / (w / 2 + t / 2) };
}

/** A beam section given by its area and section moduli (mm^2, mm^3): F / A + M / Zi + Mo / Zo against the yield. */
export function beamSectionCheck({ A_mm2, Zi_mm3, Zo_mm3 }, material, { F = 0, M = 0, Mo = 0 }) {
  const sigma = F / A_mm2 + (M * 1000) / Zi_mm3 + (Mo * 1000) / Zo_mm3;
  const fos = sigma > 0 ? material.yield_MPa / sigma : Infinity;
  return { sigma_MPa: sigma, yield_MPa: material.yield_MPa, fos, ok: fos >= FOS };
}

/**
 * A unidirectional carbon tube (d / di) in bending M, torsion T and transverse shear V: fibre stress M / Z against the
 * axial allowable, shear T / Zp + 2 V / A against the in-plane shear allowable, linear interaction (the matrix carries
 * the shear, so no von Mises credit). material: { axial_MPa, shear_MPa }.
 */
export function cfrpTubeCheck({ d_mm, di_mm }, material, { M = 0, T = 0, V = 0 }) {
  const s = roundSection(d_mm / 1000, di_mm / 1000);
  const sigma = M / s.Z / 1e6;
  const tau = (T / s.Zp + (2 * V) / s.A) / 1e6;
  const u = sigma / material.axial_MPa + tau / material.shear_MPa;
  const fos = u > 0 ? 1 / u : Infinity;
  return { sigma_MPa: sigma, tau_MPa: tau, axial_allowable_MPa: material.axial_MPa, shear_allowable_MPa: material.shear_MPa, fos, ok: fos >= FOS };
}

/** A flat bonded lap w x L carrying shear force F (and peel not credited): tau = F / (w L) against the adhesive's allowable. */
export function lapCheck({ w_mm, L_mm }, adhesive, F) {
  const tau = F / (w_mm * L_mm);
  const allow = adhesive.shear_MPa;
  const fos = tau > 0 ? allow / tau : Infinity;
  return { tau_MPa: tau, shear_allowable_MPa: allow, fos, ok: fos >= FOS };
}

/** Peak value of every member section from the flight model's support peaks (the members follow the joints). */
export function memberPeaks(members, supPeaks, offset) {
  const out = {};
  let k = offset;
  for (const m of members ?? []) for (const s of m.sections ?? []) out[`${m.name}/${s.name}`] = supPeaks[k++]?.Fr ?? 0;
  return out;
}

/** Each member check kind: (member, its section peaks, the peak of named sections, its uses) -> result. */
const MEMBER_CHECKS = {
  round: (m, l) => roundMemberCheck(m.dims, m.material, l),
  'pin-shear': (m, l) => pinShearCheck(m.dims, m.material, l.F, m.dims.planes ?? 1),
  seat: (m, l) => seatCheck(m.dims, m.material, l.F, l.M),
  lug: (m, l) => lugCheck(m.dims, m.material, l.F),
  web: (m, l) => webCheck(m.dims, m.material, { M: l.M, F: l.F }),
  bond: (m, l) => bondCheck(m.dims, m.material, { T: l.T, F: l.F }),
  key: (m, l) => keyCheck(m.dims, m.material, m.side_material, l.T),
  bolts: (m, l) => boltGroupCheck(m.dims, m.fastener, { V: l.V, T: l.T, M: l.M, F: l.F }),
  strut: (m, l) => strutCheck(m.dims, m.material, { F: l.F, M: l.M }),
  panel: (m, l) => panelShearCheck(m.dims, m.material, l.V),
  tie: (m, l) => tieCheck(m.dims, m.tie, l.F),
  lap: (m, l) => lapCheck(m.dims, m.material, l.F),
  plate: (m, l, val, u) => plateSectionCheck(m.dims, m.material, { F: l.F, M: l.M, Mo: val(u.Mo) }),
  beam: (m, l, val, u) => beamSectionCheck(m.dims, m.material, { F: l.F, M: l.M, Mo: val(u.Mo) }),
  'cfrp tube': (m, l) => cfrpTubeCheck(m.dims, m.material, { M: l.M, T: l.T, V: l.V }),
  unrated: (m) => ({ fos: null, ok: false, why: m.why ?? 'no published rating' }),
};

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
    const f = MEMBER_CHECKS[m.check];
    const r = f ? f(m, load, val, u) : { fos: null, ok: false, why: `no check for ${m.check}` };
    rows.push({ name: m.name, kind: m.kind, check: m.check, material: m.material?.id ?? m.fastener?.id ?? m.tie?.id ?? null, dims: m.dims, load, ...r, ...(m.note ? { note: m.note } : {}) });
  }
  const rated = rows.filter((r) => Number.isFinite(r.fos));
  return { ok: rows.every((r) => r.ok), fos_min: rated.length ? Math.min(...rated.map((r) => r.fos)) : null, worst: rated.length ? rated.reduce((a, b) => (b.fos < a.fos ? b : a)).name : null, members: rows };
}
