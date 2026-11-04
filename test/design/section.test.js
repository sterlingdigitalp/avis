// Round 17 (SPEC "Function completeness" -> "The real section"): the Eppler 387's own published section
// (src/design/section.js, NASA TM-4062 Table I, catalogue id airfoil-e387-nasa-tm4062), in place of wing-solids.js's
// NACA-form approximation (audit-v24.md D4). Pinned against rows quoted verbatim in SOURCES-e387.md and independent
// hand calculations worked from the same rows, written out below -- never against the module's own output.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { e387Surfaces, sectionNAt, ribSlices, protrusion, sparHeightFor } from '../../src/design/section.js';
import { chordAt } from '../../src/design/wing-solids.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const design = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);
const surf = e387Surfaces(catalog);

// ---- e387Surfaces: pinned against rows quoted verbatim in SOURCES-e387.md --------------------------------------

test('e387Surfaces: upper and lower interpolate exactly through rows quoted verbatim in SOURCES-e387.md', () => {
  // "The lower surface at the spar station reads '.33968 -.00803' and '.39252 -.00605' (design)."
  near(surf.lower(0.33968), -0.00803);
  near(surf.lower(0.39252), -0.00605);
  // Table I's quoted rows, upper surface.
  near(surf.upper(0.31078), 0.08157);
  near(surf.upper(0.35505), 0.08247);
  // the leading edge (0, 0) is printed on both surfaces, and the quoted trailing-edge row (thickened to +-0.00083)
  near(surf.upper(0), 0); near(surf.lower(0), 0);
  near(surf.upper(1), 0.00083); near(surf.lower(1), -0.00083);
  // camber (mean line) and thickness are plain compositions of the same two pinned rows, at a shared x/c
  near(surf.camber(0.31078), (0.08157 + surf.lower(0.31078)) / 2);
  near(surf.thickness(0.33968), surf.upper(0.33968) - -0.00803);
});

test('e387Surfaces: camber.xs and thickness.xs are the sorted union of both surfaces\' own x/c breakpoints', () => {
  const union = [...new Set([...surf.upper.xs, ...surf.lower.xs])].sort((a, b) => a - b);
  assert.deepEqual(surf.camber.xs, union);
  assert.deepEqual(surf.thickness.xs, union);
  for (let i = 1; i < union.length; i++) assert.ok(union[i] > union[i - 1], 'breakpoints must be strictly ascending');
});

test('e387Surfaces: held at the published row, not extrapolated beyond it (SPEC: no extrapolation)', () => {
  near(surf.upper(-0.2), surf.upper(0));
  near(surf.lower(1.4), surf.lower(1));
});

test("e387Surfaces: max thickness is ~9.06% near x/c 0.31, NASA TM-4062's published figure (E387_THICKNESS)", () => {
  let best = { t: -Infinity, x: 0 };
  for (let i = 0; i <= 2000; i++) { const x = i / 2000, t = surf.thickness(x); if (t > best.t) best = { t, x }; }
  assert.ok(Math.abs(best.x - 0.31) < 0.02, `max thickness landed at x/c ${best.x}, not near 0.31`);
  assert.ok(Math.abs(best.t - 0.0906) < 0.001, `max thickness ${best.t}, not within 0.001 of the published 0.0906`);
});

// ---- sectionNAt / ribSlices: the turned section, and slices that never stand outside it -------------------------

test('ribSlices: every slice lies inside the real (turned) section, sampled far finer than the slices themselves', () => {
  const s = 0.15; // mid root segment, clear of the rib-root search's own edge cases
  const { c, xle } = chordAt(design.wing, s);
  const x0 = xle - 0.05 * c, x1 = xle - 0.9 * c; // a chordwise range clear of the LE/TE pinch points
  const slices = ribSlices(design.wing, s, [x0, x1], surf, { slices: 12 });
  assert.equal(slices.length, 12);
  for (const sl of slices) {
    const [a, b] = sl.x;
    for (let k = 0; k <= 40; k++) {
      const x = a + (k / 40) * (b - a);
      const { lo, hi } = sectionNAt(design.wing, s, x, surf);
      assert.ok(sl.n[0] >= lo - 1e-9, `slice lower ${sl.n[0]} sits below the true surface ${lo} at x=${x}`);
      assert.ok(sl.n[1] <= hi + 1e-9, `slice upper ${sl.n[1]} sits above the true surface ${hi} at x=${x}`);
      assert.ok(sl.n[0] <= sl.n[1], 'a slice must have a non-negative height');
    }
  }
});

test('ribSlices: the x boundaries are evenly spaced from x0 to x1 (12 equal slices, shared edges)', () => {
  const s = 0.15;
  const { c, xle } = chordAt(design.wing, s);
  const x0 = xle - 0.05 * c, x1 = xle - 0.9 * c;
  const slices = ribSlices(design.wing, s, [x0, x1], surf, { slices: 12 });
  near(slices[0].x[0], x0, 1e-12);
  near(slices[11].x[1], x1, 1e-12);
  for (let k = 0; k < 12; k++) near(slices[k].x[1] - slices[k].x[0], (x1 - x0) / 12, 1e-9);
  for (let k = 0; k < 11; k++) near(slices[k].x[1], slices[k + 1].x[0], 1e-12);
});

test('sectionNAt: sparN re-origins n to the spar axis (n_from_spar = n_section - sparN)', () => {
  const s = 0.2, x = chordAt(design.wing, s).xle - 0.3 * chordAt(design.wing, s).c;
  const atChord = sectionNAt(design.wing, s, x, surf);
  const atSpar = sectionNAt(design.wing, s, x, surf, { sparN: 0.01 });
  near(atSpar.lo, atChord.lo - 0.01);
  near(atSpar.hi, atChord.hi - 0.01);
});

test('sectionNAt: pitch = incidence + washout * (s / L), L the sum of the segment lengths (hand-checked away from s = 0)', () => {
  // L and s are picked so s / L is a clean, independently-known fraction: the elbow is exactly segments[0].length,
  // and L (independently summed here, not by calling the module) is the full two-segment span.
  const L = design.wing.segments[0].length + design.wing.segments[1].length; // 0.975 m
  const s = design.wing.segments[0].length; // the elbow, s / L = 0.5 exactly
  near(s / L, 0.5, 1e-12);
  const pitch = design.wing.incidence + design.wing.washout * (s / L);
  // At the leading edge (xi = 0) z/c = 0 on both surfaces (table row [0, 0]), so n = X * sin(pitch) exactly, with
  // nothing from the z/c term to obscure the pitch formula itself.
  const xle = chordAt(design.wing, s).xle, X = xle - design.wing.spar_x;
  const { hi } = sectionNAt(design.wing, s, xle, surf);
  near(hi, X * Math.sin(pitch), 1e-9);
});

test('sectionNAt: X = x - spar_x (sign isolated with a synthetic wing, since the v24 root happens to have spar_x = 0)', () => {
  const wing = { segments: [{ length: 1, chord_in: 1, chord_out: 1, xle_in: 0, xle_out: 0 }], incidence: 0.1, washout: 0, spar_x: 0.1 };
  // x = 0 = xle, so xi = (xle - x) / c = 0, where the published table gives z/c = 0 on both surfaces: n is then
  // exactly X * sin(pitch), with pitch = incidence = 0.1 (washout = 0) and X = x - spar_x = 0 - 0.1 = -0.1.
  const { hi } = sectionNAt(wing, 0, 0, e387Surfaces(catalog));
  near(hi, -0.1 * Math.sin(0.1), 1e-12);
});

// ---- protrusion: a 10 mm round spar at the v24 root, hand-calculated from the same pinned rows -------------------

test('protrusion: a 10 mm round centred on the chord line at the v24 root protrudes about 2 mm; on the camber line it does not', () => {
  // v24 root (data/designs/larus-gl-2.json, wing.segments[0]): chord 0.394 m, spar at x/c ~0.35.
  const seg0 = design.wing.segments[0];
  const c = seg0.chord_in, xle = seg0.xle_in, sparX = design.wing.spar_x;
  near(c, 0.394, 0.001);
  const xiSpar = (xle - sparX) / c;
  assert.ok(Math.abs(xiSpar - 0.35) < 0.001, `spar station x/c ${xiSpar}, not ~0.35`);

  // Independent hand interpolation, using only the design rows quoted verbatim in SOURCES-e387.md that straddle
  // xi_spar (the same two pairs pinned above), not section.js's own tables:
  const loA = [0.33968, -0.00803], loB = [0.39252, -0.00605];
  const upA = [0.31078, 0.08157], upB = [0.35505, 0.08247];
  assert.ok(xiSpar > loA[0] && xiSpar < loB[0] && xiSpar > upA[0] && xiSpar < upB[0], 'xi_spar must fall between the quoted rows used below');
  const zLo = loA[1] + ((xiSpar - loA[0]) / (loB[0] - loA[0])) * (loB[1] - loA[1]);
  const zUp = upA[1] + ((xiSpar - upA[0]) / (upB[0] - upA[0])) * (upB[1] - upA[1]);
  // At the spar's own x (X = 0), the pitch rotation's X sin(pitch) term vanishes, so n = z/c * c * cos(pitch); at
  // the root (s = 0) washout * (0 / L) = 0, so pitch is just wing.incidence.
  const pitch = design.wing.incidence;
  const nLower = zLo * c * Math.cos(pitch), nUpper = zUp * c * Math.cos(pitch);
  const r = 0.005; // 10 mm round, radius

  // Centred on the chord line (sparN = 0): the round spar's lowest point (n = -r) against the true lower surface.
  const handChordline_m = nLower - -r;
  assert.ok(handChordline_m > 0.0015 && handChordline_m < 0.0025, `hand calc ${handChordline_m * 1000} mm, not ~2 mm`);

  // Cross-check against the module: a round part's worst point is at its own centreline (file header), so a
  // single chordwise point at the spar's x, with nRange spanning the full 10 mm diameter, is the right call.
  const gotChordline = protrusion(design.wing, 0, 0, [sparX, sparX], [-r, r], surf, { samples: 1 });
  near(gotChordline.out_mm, handChordline_m * 1000, 1e-6);
  near(gotChordline.at.x, sparX);

  // Centred on the camber line instead (sparN = the camber height at the spar's own x/c): clear on both sides.
  const nCamber = ((zLo + zUp) / 2) * c * Math.cos(pitch);
  const gotCamber = protrusion(design.wing, 0, 0, [sparX, sparX], [nCamber - r, nCamber + r], surf, { samples: 1 });
  assert.equal(gotCamber.out_mm, 0);
});

test('protrusion: a box spanning a stretch of span, checked on its full s x x grid (samples > 1)', () => {
  const s0 = 0.4875, s1 = 0.5025; // the elbow and a little outboard of it, where wing-solids.js places the twist servo
  const { c, xle } = chordAt(design.wing, s0), x1 = xle - 0.5 * c;
  const xRange = [x1 - 0.01, x1]; // a 10 mm chordwise box, aft of the leading edge by half the chord
  // A box small enough to sit inside the section there is reported clear; stretching its top far past the
  // upper surface makes it stick out by (nRange.hi - upper), the amount the formula is built to find.
  const inside = protrusion(design.wing, s0, s1, xRange, [-0.002, 0.002], surf, { samples: 15 });
  const tooTall = protrusion(design.wing, s0, s1, xRange, [-0.002, 0.05], surf, { samples: 15 });
  assert.equal(inside.out_mm, 0, 'a small box well inside the section is reported clear');
  assert.ok(tooTall.out_mm > 20, `a box stretched 50 mm up should stick out well past the upper surface, got ${tooTall.out_mm} mm`);
  // the worst station found must actually lie within the given s and x ranges
  assert.ok(tooTall.at.s >= s0 - 1e-12 && tooTall.at.s <= s1 + 1e-12);
  assert.ok(tooTall.at.x >= Math.min(...xRange) - 1e-12 && tooTall.at.x <= Math.max(...xRange) + 1e-12);
  // frontal_mm2 is the worst-per-station height integrated over the span (m), so a protruding box has some
  assert.ok(tooTall.frontal_mm2 > 0);
});

test('protrusion: with samples = 2, the grid is exactly the four corners (s0/s1 x xRange[0]/xRange[1])', () => {
  const s0 = 0.1, s1 = 0.2;
  const { c, xle } = chordAt(design.wing, s0);
  const xA = xle - 0.2 * c, xB = xle - 0.6 * c; // two distinct chordwise points
  const nRange = [-0.01, 0.01];
  const got = protrusion(design.wing, s0, s1, [xA, xB], nRange, surf, { samples: 2 });
  let expected = 0;
  for (const s of [s0, s1]) for (const x of [xA, xB]) {
    const { lo, hi } = sectionNAt(design.wing, s, x, surf);
    expected = Math.max(expected, nRange[1] - hi, lo - nRange[0], 0);
  }
  near(got.out_mm, expected * 1000, 1e-9);
});

test('protrusion: xRange[0] and xRange[1] are both really sampled, not one standing in for the other', () => {
  // At s = 0.1: xA (20% aft of the LE) has upper surface ~30.9 mm; xB (60% aft) has upper surface ~19.8 mm. With
  // nRange.hi = 25 mm, only xB protrudes (25 - 19.8 > 0); xA does not (25 - 30.9 < 0). If xVals ever collapsed to
  // just one of the two corners, this would come out either 0 (missing xB) or a different value (using xA alone).
  const s = 0.1;
  const { c, xle } = chordAt(design.wing, s);
  const xA = xle - 0.2 * c, xB = xle - 0.6 * c;
  const { hi: hiA } = sectionNAt(design.wing, s, xA, surf), { hi: hiB } = sectionNAt(design.wing, s, xB, surf);
  const nRangeHi = 0.025;
  assert.ok(nRangeHi < hiA && nRangeHi > hiB, 'xA must be clear and xB must protrude for this check to work');
  const got = protrusion(design.wing, s, s, [xA, xB], [-0.001, nRangeHi], surf, { samples: 2 });
  near(got.out_mm, (nRangeHi - hiB) * 1000, 1e-9);
  near(got.at.x, xB, 1e-12);
});

test('protrusion: frontal_mm2 is the trapezoid-rule integral over s of the worst height per station', () => {
  const s0 = 0.1, s1 = 0.3, x = chordAt(design.wing, s0).xle; // a fixed chordwise point, the leading edge (xi = 0)
  const nRange = [0.05, 0.06]; // well above the upper surface there, so both ends protrude by a known amount
  const got = protrusion(design.wing, s0, s1, [x, x], nRange, surf, { samples: 2 });
  const worst = (s) => Math.max(nRange[1] - sectionNAt(design.wing, s, x, surf).hi, 0);
  const w0 = worst(s0), w1 = worst(s1);
  assert.ok(w0 > 0 && w1 > 0, 'both stations must protrude for this check to be meaningful');
  near(got.frontal_mm2, 0.5 * (w0 + w1) * (s1 - s0) * 1e6, 1e-6);
  near(got.out_mm, Math.max(w0, w1) * 1000, 1e-6);
});

// ---- sparHeightFor: the feasible sparN band -----------------------------------------------------------------------

test('sparHeightFor: a modest round spar has a feasible band at the root; a far oversized one does not', () => {
  const small = sparHeightFor(design.wing, catalog, (s) => (s <= 0.01 ? 0.01 : NaN), { clearance_m: 0.0005, step_m: 0.005 });
  assert.ok(small.stations.length > 0);
  assert.ok(small.overall.feasible);
  assert.ok(small.overall.lo <= small.overall.hi);
  // sanity: every reported per-station band is itself non-empty where feasible
  for (const st of small.stations) if (st.feasible) assert.ok(st.lo <= st.hi);

  const tooBig = sparHeightFor(design.wing, catalog, (s) => (s <= 0.01 ? 0.3 : NaN), { clearance_m: 0.0005, step_m: 0.005 });
  assert.equal(tooBig.overall.feasible, false);
});

test('sparHeightFor: the station loop reaches the full span, including the tip', () => {
  const L = design.wing.segments[0].length + design.wing.segments[1].length;
  const band = sparHeightFor(design.wing, catalog, () => 0.008, { clearance_m: 0.0005, step_m: 0.01 });
  const lastS = band.stations.at(-1).s;
  assert.ok(Math.abs(lastS - L) < 0.01, `last station s=${lastS}, should reach close to L=${L}`);
});

test('sparHeightFor: a station where the OD is exactly 0 is skipped, not treated as a zero-size spar', () => {
  const band = sparHeightFor(design.wing, catalog, (s) => (s < 0.0005 ? 0 : s <= 0.01 ? 0.01 : NaN), { clearance_m: 0.0005, step_m: 0.001 });
  assert.ok(band.stations.every((t) => t.s >= 0.001 - 1e-9), 'the od = 0 station (s = 0) must not appear');
});

test('sparHeightFor: with chordSamples = 2, the band is exactly the hand-computed value from the two true edges', () => {
  const s = 0, od = 0.02, r = od / 2; // the root, a 20 mm spar: big enough that the two edges differ measurably
  const band = sparHeightFor(design.wing, catalog, () => od, { clearance_m: 0.0005, step_m: 1, chordSamples: 2 });
  const st = band.stations.find((t) => Math.abs(t.s - s) < 1e-9);
  // Hand-check (via the already-pinned sectionNAt) using exactly the two true edges (dx = -r and dx = +r), where
  // the circle's local half-height is 0 (it just touches the surface, nothing more): an exact match, not merely a
  // bound, so a sample that is missing, duplicated or displaced from either true edge is caught.
  let expectedHi = Infinity, expectedLo = -Infinity;
  for (const dx of [-r, r]) {
    const { lo, hi } = sectionNAt(design.wing, s, design.wing.spar_x + dx, surf);
    expectedHi = Math.min(expectedHi, hi - 0.0005);
    expectedLo = Math.max(expectedLo, lo + 0.0005);
  }
  near(st.hi, expectedHi, 1e-9);
  near(st.lo, expectedLo, 1e-9);
});

test('sparHeightFor: the loop includes the exact final station when L is a whole multiple of step_m', () => {
  const L = design.wing.segments[0].length + design.wing.segments[1].length;
  const step_m = L / 10;
  const band = sparHeightFor(design.wing, catalog, () => 0.008, { clearance_m: 0.0005, step_m });
  assert.equal(band.stations.length, 11); // s = 0, step, 2*step, ..., 10*step = L
  near(band.stations.at(-1).s, L, 1e-9);
});

test('sparHeightFor: no valid station anywhere means not feasible (not vacuously true)', () => {
  const band = sparHeightFor(design.wing, catalog, () => NaN, { clearance_m: 0.0005, step_m: 0.1 });
  assert.equal(band.stations.length, 0);
  assert.equal(band.overall.feasible, false);
});

test('sparHeightFor: exactly one valid station is still feasible on its own (not just "more than one")', () => {
  const band = sparHeightFor(design.wing, catalog, (s) => (s < 1e-9 ? 0.008 : NaN), { clearance_m: 0.0005, step_m: 1 });
  assert.equal(band.stations.length, 1);
  assert.equal(band.overall.feasible, true);
});

test('sparHeightFor: a one-piece spar across both segments is infeasible even though each segment alone is feasible (non-overlapping per-station bands)', () => {
  // The real v24 inner spar (OD 9.982 mm, s 0.007-0.4875) and outer spar (OD 8.0 mm, s 0.4875-0.9383), each
  // individually feasible (see the v24 report), do not share a common sparN: run as one constant-height spar
  // across both segments, the combination must be reported infeasible overall.
  const combined = sparHeightFor(design.wing, catalog, (s) => (s >= 0.007 && s <= 0.4875 ? 0.009982 : s > 0.4875 && s <= 0.9383 ? 0.008 : NaN), { clearance_m: 0.0005, step_m: 0.002 });
  assert.ok(combined.stations.every((t) => t.feasible), 'every individual station must be feasible on its own');
  assert.equal(combined.overall.feasible, false, 'but the combined band is empty: the two segments share no common sparN');
});
