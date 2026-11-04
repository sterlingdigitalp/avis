// Round 17 (SPEC "Function completeness" -> "The real section"): the wing's clearance and containment checks against
// the Eppler 387's own published coordinates (NASA TM-4062 Table I, catalogue id airfoil-e387-nasa-tm4062), in place
// of wing-solids.js's NACA-form thickness on a parabolic camber line. audit-v24.md D4: the old model puts the spar's
// axis on the chord line, but the real lower surface near the spar (x/c 0.34-0.39) sits at only -0.008..-0.006 c, so
// a spar centred there stands out of the covering by about 2 mm; this module gives the real surfaces and the
// containment arithmetic to fix that.
//
// Sign conventions (figure in words). Looking at one wing laid flat (zero pitch) from behind the bird, span to the
// right: x is the body chordwise coordinate (SPEC's body x), running from the trailing edge (low x) to the leading
// edge (high x); n is normal to the wing, running from the lower surface (negative n) up through the chord line
// (n = 0) to the upper surface (positive n), the two surfaces meeting (n = 0) only at the leading and trailing
// edges. A section is turned about the spar's own axis (X = 0, at body x = wing.spar_x) by its local pitch (nose up
// positive, wing.incidence plus washout toward the tip): n = X sin(pitch) + Y cos(pitch), X the chordwise distance
// from the spar axis (positive toward the leading edge) and Y = z/c * c the untilted height. A positive pitch
// therefore raises points ahead of the spar (X > 0) and lowers points behind it (X < 0). sparN, where given, is the
// spar axis's own height above the chord line (n_section at X = 0, Y = 0 would be the chord line; the spar axis
// itself sits sparN above it), so subtracting it re-origins n to be measured from the spar axis rather than the
// chord line: n_from_spar = n_section - sparN.
import { chordAt } from './wing-solids.js';
import { byId } from './catalog.js';

const totalSpan = (wing) => wing.segments.reduce((a, g) => a + g.length, 0);
const clamp01 = (x) => Math.min(Math.max(x, 0), 1);

/**
 * Linear interpolation in x/c of a published [x/c, z/c] table (ascending x/c, as printed from the leading edge to
 * the trailing edge). Clamped at the ends (SPEC: nothing is extrapolated beyond a published row), so x/c outside
 * [0, 1] holds the leading- or trailing-edge ordinate. The returned function carries its table's x/c breakpoints as
 * `.xs`, so a caller needing the exact extremum of the (piecewise-linear) function over a sub-range of x/c can find
 * it from the range's own ends and any breakpoint strictly inside them, without resampling.
 */
function makeInterp(table) {
  const xs = table.map((r) => r[0]), zs = table.map((r) => r[1]);
  const fn = (xc) => {
    const x = Math.min(Math.max(xc, xs[0]), xs[xs.length - 1]);
    let i = 0;
    while (i < xs.length - 2 && xs[i + 1] < x) i++;
    const x0 = xs[i], x1 = xs[i + 1], t = x1 === x0 ? 0 : (x - x0) / (x1 - x0);
    return zs[i] + t * (zs[i + 1] - zs[i]);
  };
  fn.xs = xs;
  return fn;
}

/**
 * The Eppler 387's published section (NASA TM-4062 Table I, catalogue id airfoil-e387-nasa-tm4062), as z/c functions
 * of x/c by linear interpolation on each surface's own table. Uses the design columns (specs.upper / specs.lower,
 * the z_des/c rows), not the measured-model columns (specs.measured_model): the design ordinates are the Eppler 387
 * section itself (with the wind-tunnel model's documented thickened trailing edge blended in from x/c 0.95), while
 * the measured columns are that one 6 in-chord model's as-built deviation from its own design, not the section's
 * definition. camber(xc) is the mean line (upper + lower) / 2 and thickness(xc) is the full thickness (upper -
 * lower), both piecewise-linear with breakpoints at the union of the two surfaces' x/c stations.
 */
export function e387Surfaces(catalog) {
  const part = byId(catalog, 'airfoil-e387-nasa-tm4062');
  const s = part.specs;
  const upper = makeInterp(s.upper), lower = makeInterp(s.lower);
  const unionXs = [...new Set([...upper.xs, ...lower.xs])].sort((a, b) => a - b);
  const camber = (xc) => (upper(xc) + lower(xc)) / 2;
  const thickness = (xc) => upper(xc) - lower(xc);
  camber.xs = unionXs; thickness.xs = unionXs;
  return {
    upper, lower, camber, thickness,
    source: { id: part.id, citation: s.citation, table: s.table, coordinate_set_used: s.coordinate_set_used, citation_page_url: s.citation_page_url },
  };
}

/** n (m) of one surface (surf.lower or surf.upper, a z/c(xc) function from e387Surfaces) at body x (m), span s (m). */
function nOf(wing, s, x, zcFn, sparN) {
  const { c, xle } = chordAt(wing, s), L = totalSpan(wing);
  const pitch = wing.incidence + wing.washout * (s / L);
  const xi = clamp01((xle - x) / c);
  const X = x - wing.spar_x;
  return X * Math.sin(pitch) + zcFn(xi) * c * Math.cos(pitch) - sparN;
}

/**
 * The section's lower and upper surface as the wing-normal coordinate n (m), at body chordwise x (m) and span s (m
 * from the shoulder axis), for the design's wing (segments with chord/xle per segment, incidence, washout linear
 * with s / L, spar_x), turned about the spar axis by its local pitch exactly as wing-solids.js's ribEnvelope does.
 * surf is e387Surfaces(catalog)'s return. sparN (m, default 0), if given, re-origins n to the spar axis (see the
 * file header's sign conventions): n_from_spar = n_section - sparN.
 */
export function sectionNAt(wing, s, x, surf, { sparN = 0 } = {}) {
  return { lo: nOf(wing, s, x, surf.lower, sparN), hi: nOf(wing, s, x, surf.upper, sparN) };
}

/** The exact max (mode 'max') or min (mode 'min') of n(x) for one surface over [a, b] (any order), using the fact
 * that n(x) is piecewise-linear in body x for fixed s (chordAt and the z/c table compose two affine maps), so its
 * extremum over a sub-range is at the range's own ends or at a table breakpoint mapped into body x that falls
 * strictly inside it -- never approximated by sampling. */
function sliceExtreme(wing, s, zcFn, a, b, mode, sparN) {
  const { c, xle } = chordAt(wing, s), lo = Math.min(a, b), hi = Math.max(a, b);
  const inner = (zcFn.xs ?? []).map((xi) => xle - xi * c).filter((x) => x > lo + 1e-12 && x < hi - 1e-12);
  const vals = [lo, hi, ...inner].map((x) => nOf(wing, s, x, zcFn, sparN));
  return mode === 'max' ? Math.max(...vals) : Math.min(...vals);
}

/**
 * The rib's outline between x0 and x1 (m, body x, either order) at span s, cut into `slices` equal x-slices: each
 * slice's n-range is the innermost of the two surfaces over that slice (the highest of the lower surface and the
 * lowest of the upper surface within it), so a rib drawn as these rectangles never stands outside the real section
 * anywhere in [x0, x1] -- not even between the slice's own x samples, because sliceExtreme finds the true extremum.
 * sparN (m, default 0) re-origins n to the spar axis, as sectionNAt.
 */
export function ribSlices(wing, s, [x0, x1], surf, { slices = 12, sparN = 0 } = {}) {
  const out = [];
  for (let k = 0; k < slices; k++) {
    const xa = x0 + (k / slices) * (x1 - x0), xb = x0 + ((k + 1) / slices) * (x1 - x0);
    const lo = sliceExtreme(wing, s, surf.lower, xa, xb, 'max', sparN);
    const hi = sliceExtreme(wing, s, surf.upper, xa, xb, 'min', sparN);
    out.push({ x: [xa, xb], n: [lo, hi] });
  }
  return out;
}

/**
 * How far a box-shaped part -- spanning s0..s1 (m) along the span, with chordwise extent xRange (m, body x, either
 * order) and constant normal extent nRange ([lo, hi], m, in the spar-axis frame: sparN already applied) -- stands
 * outside the section anywhere in that box, by a grid of `samples` x `samples` stations in s and x: at each, the
 * protrusion is max(nRange.hi - upper, lower - nRange.lo, 0) (the amount it pokes past the upper surface, or past
 * the lower surface, whichever is worse, zero if the box is inside there). out_mm is the worst of the grid and `at`
 * is where. frontal_mm2 is that worst-case height's own further worst case per span station, integrated over s (m)
 * by the trapezoid rule -- the frontal area the protrusion presents to the airflow, as audit-v24.md D4 estimates it.
 * For a round part (e.g. a spar), pass a single chordwise point (xRange = [spar_x, spar_x]) with nRange spanning its
 * full diameter: the round part's own worst point against a smoothly-curving surface is at its centreline, where
 * its height already equals the diameter, so this reproduces its clearance without modelling its taper explicitly.
 */
export function protrusion(wing, s0, s1, xRange, nRange, surf, { sparN = 0, samples = 25 } = {}) {
  const lin = (a, b, n) => (n <= 1 ? [a] : Array.from({ length: n }, (_, i) => a + (i / (n - 1)) * (b - a)));
  const sVals = lin(s0, s1, samples), xVals = lin(xRange[0], xRange[1], samples);
  let out = 0, at = { s: sVals[0], x: xVals[0] }, frontal = 0, prevWorst = null, prevS = null;
  for (const s of sVals) {
    let worstAtS = 0;
    for (const x of xVals) {
      const { lo, hi } = sectionNAt(wing, s, x, surf, { sparN });
      const p = Math.max(nRange[1] - hi, lo - nRange[0], 0);
      if (p > worstAtS) worstAtS = p;
      if (p > out) { out = p; at = { s, x }; }
    }
    if (prevWorst !== null) frontal += 0.5 * (prevWorst + worstAtS) * (s - prevS);
    prevWorst = worstAtS; prevS = s;
  }
  return { out_mm: out * 1000, at, frontal_mm2: frontal * 1e6 };
}

/**
 * The band of spar-axis height above the chord line (sparN, m) at which a round spar of OD sparOdByS(s) (m, a
 * function of span s; return a non-finite value where the spar does not run) centred chordwise at x = wing.spar_x
 * stays inside the section with clearance_m everywhere on its circular cross-section, at each sampled station (every
 * step_m of span) and overall (the intersection of every station's band, so the caller can choose one sparN that
 * works everywhere the spar runs). Builds its own e387Surfaces(catalog); wing is the design's wing.
 */
export function sparHeightFor(wing, catalog, sparOdByS, { clearance_m = 0.0005, step_m = 0.005, chordSamples = 41 } = {}) {
  const surf = e387Surfaces(catalog), L = totalSpan(wing);
  const stations = [];
  for (let s = 0; s <= L + 1e-9; s += step_m) {
    const sc = Math.min(s, L), od = sparOdByS(sc);
    if (!Number.isFinite(od) || od <= 0) continue;
    const r = od / 2;
    let hi = Infinity, lo = -Infinity;
    for (let k = 0; k < chordSamples; k++) {
      const dx = -r + (k / (chordSamples - 1)) * (2 * r), x = wing.spar_x + dx;
      const half = Math.sqrt(Math.max(0, r * r - dx * dx));
      const { lo: lowerN, hi: upperN } = sectionNAt(wing, sc, x, surf);
      hi = Math.min(hi, upperN - clearance_m - half);
      lo = Math.max(lo, lowerN + clearance_m + half);
    }
    stations.push({ s: sc, lo, hi, feasible: lo <= hi });
  }
  const feasibleStations = stations.filter((t) => t.feasible);
  const lo = stations.length ? Math.max(...stations.map((t) => t.lo)) : null;
  const hi = stations.length ? Math.min(...stations.map((t) => t.hi)) : null;
  return { stations, overall: { lo, hi, feasible: stations.length > 0 && lo <= hi && feasibleStations.length === stations.length } };
}
