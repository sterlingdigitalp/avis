// Bearings and bushings of the drive (round 5, SPEC ruling "Bearings are linkage"): static checks of every loaded joint
// against its published static rating, and the reactions of a shaft on two supports. Pure functions.
//
// Rolling bearings: ISO 76 static equivalent load P0 = X0 Fr + Y0 Fa, and P0 = Fr when that is larger, with the maker's
// X0 and Y0; static safety factor C0 / P0 >= 1.5. Plain bushings: projected surface pressure Fr / (d1 b) on the length b
// the housing actually supports (round 11; b1 when the housing is as long as the bushing) against the maker's
// published static surface pressure, factor of safety 1.5; a flange takes the axial load on its own ring area.
// A joint part without a published static rating is ineligible for a loaded joint (fos null: the check cannot pass).

export const FOS_STATIC = 1.5;

/**
 * Static factors of a deep groove ball bearing as SKF publishes them: P0 = 0.6 Fr + 0.5 Fa, and P0 = Fr if that is
 * smaller than Fr. The catalogue's SKF deep groove entries carry no X0/Y0 of their own; this is their maker's rule.
 */
export const DEEP_GROOVE = { X0: 0.6, Y0: 0.5, source: 'SKF Rolling bearings catalogue, single row deep groove ball bearings, "Equivalent static bearing load: P0 = 0,6 Fr + 0,5 Fa; P0 < Fr -> P0 = Fr" (single bearings); https://www.famcocorp.com/uploads/product/Bearing/SKF/Ball-Bearing/Deep-Groove/SKF-Deep-Groove-Ball-Bearings-Catalog.pdf' };

/** ISO 76 static equivalent load of a radial rolling bearing (N). X0, Y0 from the maker; P0 is never below Fr. */
export function staticEquivalentLoad(Fr, Fa, { X0, Y0 }) {
  return Math.max(Fr, X0 * Fr + Y0 * Fa);
}

/**
 * Shares of a point load at x carried by two supports at xA and xB (a simply supported or overhung shaft), as
 * magnitudes: [|xB - x| / L, |x - xA| / L]. Their sum is 1 between the supports and grows past them (overhang). Adding
 * magnitudes of several loads bounds the reaction whatever their directions.
 */
export function leverShares(xA, xB, x) {
  const L = Math.abs(xB - xA);
  if (!(L > 0)) throw new Error('supports must be apart');
  return [Math.abs(xB - x) / L, Math.abs(x - xA) / L];
}

/** What kind of joint part a catalogue entry is, from its category and specs. */
export function jointKind(part) {
  if (!part) return null;
  const s = part.specs ?? {};
  if (part.category === 'bushing') return 'bushing';
  if (part.category === 'rod_end') return 'rod-end';
  if (part.category === 'bearing') {
    if (/needle/i.test(s.kind ?? part.name ?? '')) return 'needle';
    if (/deep groove/i.test(part.name ?? '') || /deep groove/i.test(s.kind ?? '')) return 'deep-groove';
    return 'rolling';
  }
  return null;
}

/**
 * Static check of one support (one bearing, bushing or rod end) under its peak loads. loads: { Fr, Fa, P0?, v_m_s? }
 * in N and m/s; P0, when given, is the peak of the ISO 76 equivalent over time (it can exceed the form applied to the
 * two separate peaks only if they did not coincide: it never does, but the larger is used). Returns
 * { kind, fos, ok, ... } with fos null when the part publishes no static rating.
 */
export function supportCheck(part, loads, { housing_mm = null } = {}) {
  const kind = jointKind(part);
  const s = part?.specs ?? {};
  const L = { Fr: Math.max(0, loads.Fr ?? 0), Fa: Math.max(0, loads.Fa ?? 0), P0: loads.P0 ?? 0, v_m_s: loads.v_m_s };
  if (kind === 'deep-groove' || kind === 'rolling' || kind === 'needle') return rollingCheck(kind, s, L);
  if (kind === 'bushing') return bushingCheck(s, L, housing_mm);
  if (kind === 'rod-end') return rodEndCheck(s, L);
  return { kind: null, fos: null, ok: false, basis: 'no bearing or bushing part: a loaded joint without a rated part is ineligible' };
}

/** ISO 76 static factors of a rolling bearing: needle 1/0; the part's own X0/Y0; SKF's for deep groove; else null. */
function staticFactors(kind, s) {
  if (kind === 'needle') return { X0: 1, Y0: 0 };
  if (Number.isFinite(s.X0) && Number.isFinite(s.Y0)) return { X0: s.X0, Y0: s.Y0 };
  return kind === 'deep-groove' ? DEEP_GROOVE : null;
}

/** Rolling bearing: C0 / P0 >= 1.5 with P0 by ISO 76. A radial needle bearing carries no axial load. */
function rollingCheck(kind, s, { Fr, Fa, P0: P0peak }) {
  const C0 = Number.isFinite(s.static_load_rating_kn) ? s.static_load_rating_kn * 1000 : null;
  const f = staticFactors(kind, s);
  if (!C0 || !f) return { kind, fos: null, ok: false, basis: !C0 ? 'no published static load rating: ineligible for a loaded joint' : 'no published static factors X0/Y0' };
  const P0 = Math.max(staticEquivalentLoad(Fr, Fa, f), P0peak);
  const axialOk = kind !== 'needle' || Fa <= 0;
  const fos = P0 > 0 ? C0 / P0 : Infinity;
  return { kind, Fr_N: Fr, Fa_N: Fa, P0_N: P0, C0_N: C0, X0: f.X0, Y0: f.Y0, fos, ok: fos >= FOS_STATIC && axialOk, ...(axialOk ? {} : { note: 'radial needle bearing under axial load' }), basis: `ISO 76: P0 = max(Fr, ${f.X0} Fr + ${f.Y0} Fa); C0/P0 >= ${FOS_STATIC}` };
}

/** Axial load on a bushing's flange ring at the published pressure; a sleeve without a flange takes none. */
function flangeCheck(s, Fa, p0) {
  if (!(Fa > 0)) return null;
  const ring = Number.isFinite(s.d3_mm) && Number.isFinite(s.d2_mm) ? (Math.PI / 4) * (s.d3_mm ** 2 - s.d2_mm ** 2) : 0;
  return ring > 0 ? { area_mm2: ring, p_MPa: Fa / ring, fos: p0 / (Fa / ring) } : { area_mm2: 0, p_MPa: Infinity, fos: 0, note: 'sleeve bushing under axial load (no flange)' };
}

/**
 * Supported length of a bushing (mm): its b1, or the housing's length where the housing holds it over less (round 11:
 * a 6 mm bushing in a 1.2 mm plate bears on 1.2 mm, not 6). A housing length that is not a positive number leaves b1.
 */
export function supportedLength(b1, housing_mm) {
  return Number.isFinite(housing_mm) && housing_mm > 0 ? Math.min(b1, housing_mm) : b1;
}

/**
 * Plain bushing: projected pressure Fr / (d1 b) on its supported length b against the published static surface
 * pressure, and its speed limit.
 */
function bushingCheck(s, { Fr, Fa, v_m_s }, housing_mm = null) {
  const p0 = s.static_surface_pressure_mpa;
  const d1 = s.d1_mm, b1 = s.b1_mm;
  if (!Number.isFinite(p0) || !(d1 > 0) || !(b1 > 0)) return { kind: 'bushing', fos: null, ok: false, basis: 'no published static surface pressure or size: ineligible for a loaded joint' };
  const b = supportedLength(b1, housing_mm);
  const p = Fr / (d1 * b); // N/mm^2 = MPa on the projected area
  const fos = p > 0 ? p0 / p : Infinity;
  const flange = flangeCheck(s, Fa, p0);
  const vMax = s.max_surface_speed_rotating_m_s;
  const speed = Number.isFinite(v_m_s) ? { v_m_s, max_m_s: vMax ?? null, ok: Number.isFinite(vMax) ? v_m_s <= vMax : false } : null;
  const fosAll = Math.min(fos, flange ? flange.fos : Infinity);
  return { kind: 'bushing', Fr_N: Fr, Fa_N: Fa, p_MPa: p, allowable_MPa: p0, b1_mm: b1, supported_mm: b, fos: fosAll, radial_fos: fos, flange, speed, ok: fosAll >= FOS_STATIC && (speed ? speed.ok : true), basis: `projected pressure Fr/(d1 b) on the supported length b = ${b} mm (b1 ${b1} mm) against the published static surface pressure ${p0} MPa; FoS >= ${FOS_STATIC}` };
}

/** Rod end: its published static load along the shank (the coupler's own force); no axial (pin-direction) load. */
function rodEndCheck(s, { Fr, Fa }) {
  const Nr = Number.isFinite(s.static_radial_load_n) ? s.static_radial_load_n : Number.isFinite(s.static_radial_load_kn) ? s.static_radial_load_kn * 1000 : null;
  if (!Nr) return { kind: 'rod-end', fos: null, ok: false, basis: 'no published static radial load: ineligible for a loaded joint' };
  const fos = Fr > 0 ? Nr / Fr : Infinity;
  return { kind: 'rod-end', Fr_N: Fr, Fa_N: Fa, rated_N: Nr, fos, ok: fos >= FOS_STATIC && Fa <= 0, basis: `published static radial load ${Nr} N; FoS >= ${FOS_STATIC}` };
}

/**
 * M7 check of every joint of design.structure.joints against the peak support loads of a flight population
 * (supPeaks: [{ Fr, Fa, P0 }] in the joints' support order, as the flight model records them) and the peak crank rate
 * (rad/s) for the bushings' surface speed. Returns { joints, fos_min, ok, worst }.
 */
export function checkJoints(joints, supPeaks, { crankRate = 0 } = {}) {
  let k = 0;
  let fosMin = Infinity, worst = null, ok = true;
  const out = [];
  for (const j of joints ?? []) {
    const sups = [];
    for (const s of j.supports) {
      const p = supPeaks?.[k++] ?? { Fr: 0, Fa: 0, P0: 0 };
      const v = Number.isFinite(s.surface_m_s_per_crank_rad_s) ? s.surface_m_s_per_crank_rad_s * crankRate : undefined;
      const c = supportCheck(s.part, { Fr: p.Fr, Fa: p.Fa, P0: p.P0, v_m_s: v }, { housing_mm: s.housing_mm ?? null });
      sups.push({ name: s.name, ...c });
      const f = c.fos ?? 0;
      if (f < fosMin) { fosMin = f; worst = `${j.name}/${s.name}`; }
      if (!c.ok) ok = false;
    }
    out.push({ name: j.name, part: j.part_id, count: j.count, reacts: j.reacts, fos: Math.min(...sups.map((s) => s.fos ?? 0)), ok: sups.every((s) => s.ok), supports: sups });
  }
  return { joints: out, fos_min: out.length ? fosMin : null, ok: out.length > 0 && ok, worst };
}

/**
 * The worst use of any support's rating in a checkJoints result, as a ratio (<= 1 passes): FOS_STATIC / fos, or the
 * surface speed over its published limit, whichever is larger. An ineligible part gives Infinity.
 */
export function jointsRatio(check) {
  let r = 0;
  for (const j of check?.joints ?? []) for (const s of j.supports) {
    r = Math.max(r, s.fos ? FOS_STATIC / s.fos : Infinity);
    if (s.speed) r = Math.max(r, Number.isFinite(s.speed.max_m_s) ? s.speed.v_m_s / s.speed.max_m_s : Infinity);
    if (s.kind === 'needle' && s.Fa_N > 0) r = Infinity;
    if (s.flange && s.flange.area_mm2 === 0) r = Infinity;
  }
  return r;
}

/**
 * Loads on one support at an instant from the drive's load channels: Fr = sum |k| ch over its radial terms, Fa over
 * its axial terms. support: { radial: [{ ch, k }], axial: [{ ch, k }] }; ch: { name: value >= 0 }.
 */
export function supportLoads(support, ch) {
  let Fr = 0, Fa = 0;
  for (const t of support.radial ?? []) Fr += Math.abs(t.k) * Math.abs(ch[t.ch] ?? 0);
  for (const t of support.axial ?? []) Fa += Math.abs(t.k) * Math.abs(ch[t.ch] ?? 0);
  return { Fr, Fa };
}
