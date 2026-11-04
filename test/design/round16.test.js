// Round 16 (SPEC "Completeness and assembly order"): the completeness check (src/design/completeness.js), the
// assembly-order check (src/design/sequence.js), the fixings' layout (src/design/fixings.js), the round-16 geometry
// helpers and the official build's refusal (src/design/round16.mjs). Expected values are worked by hand from the
// catalogue's numbers and toy assemblies, never read back from the code under test.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { bomLines, resolveTag, completenessRecord, fastenerGeometry, shapeArea, solidVolume, EXEMPT, VOLUME_TOL } from '../../src/design/completeness.js';
import { slideGap, resolveSteps, sequenceRecord, STEP_MAX_M } from '../../src/design/sequence.js';
import { pickScrew } from '../../src/design/fixings.js';
import { ring, cap, poly, segSegDist, screwX, screwY, capsulePoly, circlePoly, gap2, addPart, EXEMPT_JOINTS, KEYWAY, THROUGH, MESH, GROOVE, xSlabs } from '../../src/design/clearance.js';
import { frameRect, chordAt } from '../../src/design/wing-solids.js';
import { officialRefusal, SEQUENCE, ROUND16 } from '../../src/design/round16.mjs';
import { insideConvex, clipConvex } from '../../src/design/airframe.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
const S = (name, x, shape, extra = {}) => ({ name, x, shape, ...extra });

// ---- completeness -------------------------------------------------------------------------------------------------

/** A toy design: two bought parts of one id on two lines, a tube by the metre, a metal part, a fastener, a covering. */
const toyDesign = () => ({
  bom: {
    parts: [
      { id: 'gear-khk-msga1-18', role: 'gear', count: 1, mass_g: 20 },
      { id: 'gear-khk-msga1-18', role: 'gear', count: 1, mass_g: 20 },
    ],
    structure: [
      // 43 g/m x 0.5 m: a 500 mm length of the 9.982/7.976 mm tube
      { name: 'keel and tail boom', material_id: 'tube-rockwest-pultruded-10x8', mass_g: 21.5, basis: '43 g/m x 0.5 m' },
      // 6061-T6 at 2.7 g/cm3: 2.7 g is 1000 mm3
      { name: 'motor bracket', material_id: 'metal-al-6061-t6', mass_g: 2.7, basis: '1000 mm3' },
      { name: 'M3 screws, 2 off', material_id: 'screw-iso4762-m3x8-10.9', mass_g: 2, basis: '2 x 1 g' },
      { name: 'wing film, both faces', material_id: 'film-oracover-air-light', mass_g: 20, basis: 'film' },
    ],
  },
});
// the KHK MSGA1-18 (catalogue): 20 mm outside, 8 mm bore, 10 mm face
const gear = (id) => ({ id, solids: [S('teeth', [0, 0.01], ring([0, 0], 0.010, 0.004))], bom: ['gear-khk-msga1-18'] });
const m3 = fastenerGeometry(catalog.find((p) => p.id === 'screw-iso4762-m3x8-10.9').specs, 'screw-iso4762-m3x8-10.9');
const toyAsm = () => {
  const parts = [];
  addPart(parts, 'gear a', gear('a').solids, { bom: 'gear-khk-msga1-18' });
  addPart(parts, 'gear b', gear('b').solids, { bom: 'gear-khk-msga1-18' });
  addPart(parts, 'boom', [S('tube', [0, 0.5], ring([0, 0.06], 0.009982 / 2, 0.007976 / 2))], { bom: 'keel and tail boom' });
  // a 10 x 10 x 10 mm block: 1000 mm3
  addPart(parts, 'bracket', [S('plate', [0, 0.01], poly([[0, 0], [0.01, 0], [0.01, 0.01], [0, 0.01]]))], { bom: 'motor bracket' });
  for (const k of [1, 2]) addPart(parts, `screw ${k}`, screwX([0.02 * k, 0], 0, 1, m3).screw, { bom: 'screw-iso4762-m3x8-10.9' });
  return { parts, joins: [] };
};

test('BOM lines: bought parts by catalogue id (counts summed), structure items by name, covering film exempt', () => {
  const L = bomLines(toyDesign());
  const g = L.find((l) => l.key === 'gear-khk-msga1-18');
  assert.equal(g.count, 2); near(g.mass_g, 40);
  assert.equal(L.find((l) => l.key === 'M3 screws, 2 off').count, 2);
  assert.equal(L.find((l) => l.key === 'motor bracket').count, 1);
  assert.ok(L.find((l) => l.key === 'wing film, both faces').exempt);
  assert.equal(L.filter((l) => l.exempt).length, 1);
  // a tag names its line by id, by name, by name without ", N off", or by a material only one line has
  assert.equal(resolveTag(L, 'gear-khk-msga1-18'), 'gear-khk-msga1-18');
  assert.equal(resolveTag(L, 'M3 screws'), 'M3 screws, 2 off');
  assert.equal(resolveTag(L, 'screw-iso4762-m3x8-10.9'), 'M3 screws, 2 off');
  assert.equal(resolveTag(L, 'motor bracket'), 'motor bracket');
  assert.equal(resolveTag(L, 'no such line'), null);
  assert.equal(resolveTag(L, null), null);
  // two structure lines of one material: the material alone names neither
  const two = bomLines({ bom: { parts: [], structure: [{ name: 'a', material_id: 'metal-al-6061-t6', mass_g: 1 }, { name: 'b', material_id: 'metal-al-6061-t6', mass_g: 1 }] } });
  assert.equal(resolveTag(two, 'metal-al-6061-t6'), null);
  assert.ok(EXEMPT.some((e) => e.re.test('fuselage film shell')) && EXEMPT.some((e) => e.re.test('tail film, both faces')));
});

test('completeness: the toy assembly is complete; a missing item, a lost tag, an unknown tag, a line without solids are each refused', () => {
  const d = toyDesign();
  const ok = completenessRecord(toyAsm(), d, catalog, null);
  assert.ok(ok.passed, JSON.stringify(ok.lines.filter((r) => !r.passed)));
  // the gear's catalogue tip, bore and face; the boom's section and length; the bracket's volume; the screw's head
  const gl = ok.lines.find((r) => r.line === 'gear-khk-msga1-18');
  assert.equal(gl.items, 2);
  assert.ok(gl.size.every((r) => r.ok));
  // one gear fewer than the BOM counts
  const a1 = toyAsm(); a1.parts = a1.parts.filter((p) => p.id !== 'gear b');
  const r1 = completenessRecord(a1, d, catalog, null);
  assert.equal(r1.passed, false); assert.deepEqual(r1.failed, ['gear-khk-msga1-18']);
  assert.match(r1.lines.find((r) => r.line === 'gear-khk-msga1-18').why, /1 items, the BOM counts 2/);
  // a solid with no tag
  const a2 = toyAsm(); delete a2.parts[2].solids[0].bom;
  const r2 = completenessRecord(a2, d, catalog, null);
  assert.equal(r2.passed, false); assert.deepEqual(r2.untagged, ['boom/tube']);
  // a tag naming no line
  const a3 = toyAsm(); a3.parts[3].solids[0].bom = 'phantom bracket';
  const r3 = completenessRecord(a3, d, catalog, null);
  assert.equal(r3.passed, false); assert.deepEqual(r3.unknown_tags.map((u) => u.tag), ['phantom bracket']);
  // a physical line with no solids at all
  const d4 = toyDesign(); d4.bom.structure.push({ name: 'forward tray flange doubler', material_id: 'metal-al-7075-t6', mass_g: 1 });
  const r4 = completenessRecord(toyAsm(), d4, catalog, null);
  assert.equal(r4.passed, false); assert.equal(r4.lines.find((r) => r.line === 'forward tray flange doubler').why, 'no solids');
  // the exemption is in the record with its reason, and the margin's contents are listed
  assert.ok(ok.exemptions.some((e) => e.line === 'wing film, both faces' && /covering/.test(e.why)));
  assert.ok(ok.exemptions.some((e) => /glue/.test(e.line) && e.not_a_bom_line));
});

test('completeness sizes: a tube by its section and length, a part by its volume (10%), a screw by its head and shank', () => {
  const d = toyDesign();
  // the boom 1 mm short of the BOM's 500 mm: over the 0.5 mm per item
  const a = toyAsm(); a.parts[2].solids[0].x = [0, 0.499];
  assert.equal(completenessRecord(a, d, catalog, null).failed.includes('keel and tail boom'), true);
  const b = toyAsm(); b.parts[2].solids[0].x = [0, 0.5004];
  assert.equal(completenessRecord(b, d, catalog, null).failed.includes('keel and tail boom'), false);
  // its bore 0.1 mm too small
  const c = toyAsm(); c.parts[2].solids[0].shape = ring([0, 0.06], 0.009982 / 2, 0.007876 / 2);
  assert.equal(completenessRecord(c, d, catalog, null).failed.includes('keel and tail boom'), true);
  // the bracket 9% over its 1000 mm3 passes, 11% over fails (VOLUME_TOL 10%)
  near(VOLUME_TOL, 0.1);
  for (const [k, ok] of [[1.09, true], [1.11, false], [0.91, true], [0.89, false]]) {
    const e = toyAsm(); e.parts[3].solids[0].x = [0, 0.01 * k];
    assert.equal(completenessRecord(e, d, catalog, null).failed.includes('motor bracket'), !ok, `x ${k}`);
  }
  // an envelope (a lightened rib drawn by its outline) needs at least the BOM's volume, and may hold more
  const f = toyAsm(); f.parts[3].solids[0] = { ...f.parts[3].solids[0], x: [0, 0.03], envelope: true };
  assert.equal(completenessRecord(f, d, catalog, null).failed.includes('motor bracket'), false);
  const g = toyAsm(); g.parts[3].solids[0] = { ...g.parts[3].solids[0], x: [0, 0.008], envelope: true };
  assert.equal(completenessRecord(g, d, catalog, null).failed.includes('motor bracket'), true);
  // ISO 4762 M3 x 8 (catalogue mass basis): head 5.5 x 3.0 mm, 3 mm x 8 mm shank
  near(m3.head_d, 0.0055); near(m3.head_k, 0.003); near(m3.d, 0.003); near(m3.L, 0.008);
  const h = toyAsm(); h.parts[4].solids[1] = { ...h.parts[4].solids[1], x: [0, 0.012] };
  assert.equal(completenessRecord(h, d, catalog, null).failed.includes('M3 screws, 2 off'), true);
  // ISO 4032 M3: 5.5 mm across flats x 2.4 mm
  const n = fastenerGeometry(catalog.find((p) => p.id === 'nut-iso4032-m3-10').specs, 'nut-iso4032-m3-10');
  near(n.s, 0.0055); near(n.m, 0.0024);
  // T18R: 100 x 2.5 x 1.0 mm
  const t = fastenerGeometry(catalog.find((p) => p.id === 'tie-hellermanntyton-t18r').specs, 'tie-hellermanntyton-t18r');
  near(t.L, 0.1); near(t.b, 0.0025); near(t.t, 0.001);
});

test('section areas and volumes by hand: an annulus, a stadium, a polygon less its holes, a tube along the span, an ear\'s own area', () => {
  near(shapeArea(ring([0, 0], 0.005, 0.003)), Math.PI * (25e-6 - 9e-6), 1e-15);
  near(shapeArea(cap([0, 0], [0.01, 0], 0.001)), Math.PI * 1e-6 + 2 * 0.001 * 0.01, 1e-15);
  near(shapeArea(poly([[0, 0], [0.01, 0], [0.01, 0.01], [0, 0.01]], [{ c: [0.005, 0.005], r: 0.001 }])), 1e-4 - Math.PI * 1e-6, 1e-15);
  near(solidVolume({ x: [0, 0.002], tube: { ro: 0.005, ri: 0.004, L: 0.3 } }), Math.PI * (25e-6 - 16e-6) * 0.3, 1e-15);
  near(solidVolume({ x: [0, 0.002], shape: ring([0, 0], 0.01), area_m2: 2e-5 }), 4e-8, 1e-18);
  near(solidVolume({ x: [0, 0.002], at: () => ring([0, 0], 0.01) }, {}), Math.PI * 1e-4 * 0.002, 1e-15);
});

// ---- geometry helpers --------------------------------------------------------------------------------------------

test('segSegDist: two segments on one line read their gap, not a crossing (the spar and its torque rod)', () => {
  // on the line y = 0.3 x: [0, 0.4] and [0.4005, 0.9] (by x) are 0.5 mm apart along it
  const u = [1 / Math.hypot(1, 0.3), 0.3 / Math.hypot(1, 0.3)], P = (s) => [s * u[0], s * u[1]];
  near(segSegDist(P(0), P(0.4), P(0.4005), P(0.9)), 0.0005, 1e-12);
  near(segSegDist(P(0), P(0.4), P(0.3), P(0.9)), 0, 1e-12); // overlapping along the line
  near(segSegDist([0, 0], [1, 1], [0, 1], [1, 0]), 0, 1e-15); // a true crossing
});

test('round 16 sections: a screw along x and along y, a capsule with a bore, a circle with holes, a frame rectangle', () => {
  const s = screwX([0, 0], 0.01, -1, m3, 0.004, { s: 0.0055, m: 0.0024 });
  assert.deepEqual(s.screw.map((q) => q.x.map((v) => +v.toFixed(6))), [[0.01, 0.013], [0.002, 0.01]]); // head behind the face, shank 8 mm on
  assert.deepEqual(s.nut[0].x.map((v) => +v.toFixed(6)), [0.0016, 0.004]); // the nut from its face along the shank
  // round 17b (retain2): the nut's true section, its hexagon (corners at 5.5 / sqrt 3) bored to the screw (no minor
  // diameter given here) or to its catalogue minor diameter
  assert.equal(s.nut[0].shape.kind, 'poly'); assert.equal(s.nut[0].shape.pts.length, 6);
  near(Math.max(...s.nut[0].shape.pts.map((p) => Math.hypot(...p))), 0.0055 / Math.sqrt(3)); near(s.nut[0].shape.holes[0].r, 0.0015);
  near(screwX([0, 0], 0.01, -1, m3, 0.004, { s: 0.0055, m: 0.0024, bore: 0.0025 }).nut[0].shape.holes[0].r, 0.00125);
  const y = screwY([0.1, 0.02], -0.05, -1, m3, -0.054, { s: 0.0055, m: 0.0024 });
  // round 18.2 (the film's request): the shank its true section, N slabs across x (xSlabs), not a square prism
  const shankSlabs = y.screw.filter((s) => /^shank/.test(s.name)), headSlabs = y.screw.filter((s) => /^head/.test(s.name));
  assert.equal(shankSlabs.length, 8); assert.equal(headSlabs.length, 8);
  near(shankSlabs[0].tube.L, 0.008); near(headSlabs[0].tube.L, 0.003); near(y.nut[0].tube.L, 0.0024);
  for (const sl of shankSlabs) assert.equal(sl.shape.kind, 'poly'); // a true-section slab, not a capsule or one flat box
  const yOf = (slabs) => { const ys = slabs.flatMap((s) => s.shape.pts.map((p) => p[1])); return [Math.min(...ys), Math.max(...ys)]; };
  const [sMin, sMax] = yOf(shankSlabs); near(sMin, -0.058); near(sMax, -0.05); // the full catalogue length, exactly
  // the group circumscribes the true circle: no slab's half-width exceeds the radius, and the centre slab's equals it
  const zHalf = (sl) => (Math.max(...sl.shape.pts.map((p) => p[0])) - Math.min(...sl.shape.pts.map((p) => p[0]))) / 2;
  for (const sl of shankSlabs) assert.ok(zHalf(sl) <= m3.d / 2 + 1e-9);
  near(Math.max(...shankSlabs.map(zHalf)), m3.d / 2);
  // the capsule's polygon contains its round ends and has the bore as its hole
  const c = capsulePoly([0, 0], [0.012, 0], 0.007, [{ c: [0, 0], r: 0.005 }]);
  for (let k = 0; k < 64; k++) { const a = (k / 64) * 2 * Math.PI; for (const e of [[0, 0], [0.012, 0]]) assert.ok(insideConvex(c.pts, [e[0] + 0.007 * Math.cos(a), e[1] + 0.007 * Math.sin(a)])); }
  near(gap2(c, ring([0, 0], 0.005)), 0, 1e-12); // a 10 mm shaft in its bore: touching
  const r = circlePoly([0, 0], 0.0225, [{ c: [0, 0], r: 0.0079 }]);
  near(Math.max(...r.pts.map((p) => p[0])), 0.0225, 1e-12); near(Math.max(...r.pts.map((p) => p[1])), 0.0225, 1e-12);
  const f = frameRect([0, 0], [0, 1], [-1, 0], [0.1, 0.102], [-0.01, 0.02]);
  assert.deepEqual(f.pts.map((p) => p.map((v) => +v.toFixed(6))), [[0.01, 0.1], [0.01, 0.102], [-0.02, 0.102], [-0.02, 0.1]]);
});

/**
 * Round 18.2 (the film's request): xSlabs, the true section of a fastener lying across x. Its N slabs lie inside
 * the circumscribed square the old box drew (never wider than the radius), cover the true circle at every x (each
 * slab's half-width is at least the circle's true half-width anywhere within it, since the circle narrows toward
 * the edges and the slab uses its nearest-to-centre, widest point), and their x-z outline differs from the square
 * it replaces (most slabs are strictly narrower than the radius; only the centre reaches it). This would fail on
 * the old square drawing, which had no xSlabs at all -- every slab here would be the same, full-radius width.
 */
test('xSlabs: the true section across x -- inside the old square, covering the true circle, and not the square', () => {
  const xc = 0.05, r = 0.0015, n = 8;
  const slabs = xSlabs(xc, r, (w) => w, n);
  assert.equal(slabs.length, n);
  // the slabs' own x bounds, independent of the function's internal arithmetic: n even steps over [xc - r, xc + r]
  for (let k = 0; k < n; k++) { near(slabs[k].x[0], xc - r + (k / n) * (2 * r)); near(slabs[k].x[1], xc - r + ((k + 1) / n) * (2 * r)); }
  for (const s of slabs) {
    assert.ok(s.shape <= r + 1e-12); // inside the circumscribed square
    const dNear = Math.max(0, Math.min(Math.abs(s.x[0] - xc), Math.abs(s.x[1] - xc)));
    const dFar = Math.max(Math.abs(s.x[0] - xc), Math.abs(s.x[1] - xc));
    near(s.shape, Math.sqrt(Math.max(0, r * r - dNear * dNear))); // its own width: the circle's true width nearest xc
    assert.ok(s.shape >= Math.sqrt(Math.max(0, r * r - dFar * dFar)) - 1e-12); // covers the (narrower) true width at its far edge too
  }
  assert.ok(slabs.some((s) => s.shape < r - 1e-9)); // not the square: most slabs are narrower than the full radius
  assert.ok(slabs.some((s) => Math.abs(s.shape - r) < 1e-9)); // the centre slab reaches the true circle's widest point
});

test('convex clip and inside test: a square clipped to a half plane, a point on and off it', () => {
  const sq = [[0, 0], [2, 0], [2, 2], [0, 2]], half = [[1, -5], [9, -5], [9, 9], [1, 9]];
  const c = clipConvex(sq, half);
  const A = Math.abs(c.reduce((s, p, i) => s + p[0] * c[(i + 1) % c.length][1] - c[(i + 1) % c.length][0] * p[1], 0)) / 2;
  near(A, 2, 1e-12);
  assert.ok(insideConvex(sq, [1, 1]) && insideConvex(sq, [2, 1]) && !insideConvex(sq, [2.1, 1]));
  assert.ok(insideConvex([...sq].reverse(), [1, 1])); // either winding
});

test('the wing\'s chord and leading edge along its two segments, as the design states them', () => {
  const wing = { segments: [{ length: 0.4875, chord_in: 0.394, chord_out: 0.374, xle_in: 0.1379, xle_out: 0.1379 }, { length: 0.4875, chord_in: 0.374, chord_out: 0.2246, xle_in: 0.1379, xle_out: -0.0011 }] };
  near(chordAt(wing, 0).c, 0.394); near(chordAt(wing, 0.24375).c, 0.384); near(chordAt(wing, 0.4875).xle, 0.1379);
  near(chordAt(wing, 0.73125).xle, (0.1379 - 0.0011) / 2, 1e-12); near(chordAt(wing, 0.975).c, 0.2246); near(chordAt(wing, 2).xle, -0.0011);
});

test('fixings: the shortest catalogue M3 that takes the stack and a full nut, or 1.5 d of thread tapped', () => {
  const S2 = ['screw-iso4762-m3x12-10.9', 'screw-iso4762-m3x8-10.9'];
  // 8 mm takes up to 8 - 2.4 = 5.6 mm with its nut, 12 mm up to 9.6 mm
  assert.equal(pickScrew(catalog, S2, 0.0056, 0.0024).id, 'screw-iso4762-m3x8-10.9');
  assert.equal(pickScrew(catalog, S2, 0.0057, 0.0024).id, 'screw-iso4762-m3x12-10.9');
  assert.equal(pickScrew(catalog, S2, 0.0096, 0.0024).id, 'screw-iso4762-m3x12-10.9');
  assert.equal(pickScrew(catalog, S2, 0.0097, 0.0024), null);
  // tapped: 2 mm of bracket and 1.5 x 3 = 4.5 mm of thread
  assert.equal(pickScrew(catalog, S2, 0.002, 0.0024, true).id, 'screw-iso4762-m3x8-10.9');
  assert.equal(pickScrew(catalog, S2, 0.0036, 0.0024, true).id, 'screw-iso4762-m3x12-10.9');
});

// ---- the assembly order ------------------------------------------------------------------------------------------

const solid = (part, name, x, sec) => ({ part, id: `${part}/${name}`, x, sec });
test('a slide along x is swept exactly: a shaft into its bore touches, into a plate without one passes through', () => {
  const shaft = solid('s', 'shaft', [0.01, 0.1], ring([0, 0], 0.005));
  const holed = solid('p', 'plate', [0, 0.002], poly([[-0.02, -0.02], [0.02, -0.02], [0.02, 0.02], [-0.02, 0.02]], [{ c: [0, 0], r: 0.005 }]));
  const solidPlate = solid('p', 'plate', [0, 0.002], poly([[-0.02, -0.02], [0.02, -0.02], [0.02, 0.02], [-0.02, 0.02]]));
  // the shaft comes from aft (dir -x out) through the plate to its place forward of it
  near(slideGap(shaft, holed, [-1, 0, 0], 0.05).gap, 0, 1e-12);
  assert.ok(slideGap(shaft, solidPlate, [-1, 0, 0], 0.05).gap < -0.004);
  // from forward it never meets the plate (it stops 8 mm forward of it)
  near(slideGap(shaft, solidPlate, [1, 0, 0], 0.05).gap, 0.008, 1e-12);
});

test('a slide across x cannot jump a 0.5 mm wall: steps of at most 0.125 mm, and touching a face it slides along is allowed', () => {
  assert.ok(STEP_MAX_M <= 0.00025);
  // a 0.1 mm thin plate dropping 200 mm onto its place, a 0.5 mm wall 50 mm above its place in its way
  const thin = solid('m', 'sheet', [0, 0.01], poly([[0, 0], [0.01, 0], [0.01, 0.0001], [0, 0.0001]]));
  const wall = solid('w', 'wall', [0, 0.01], poly([[0, 0.05], [0.01, 0.05], [0.01, 0.0505], [0, 0.0505]]));
  const r = slideGap(thin, wall, [0, 1, 0], 0.2);
  assert.ok(r.gap < 0, `${r.gap}`);
  near(r.at, 0.25, 0.002); // met at 50 mm of the 200
  // the same wall beside its path, 0.3 mm off: never met; and a face it slides along (gap 0) is allowed
  const beside = solid('w', 'wall', [0, 0.01], poly([[0.0103, 0.05], [0.02, 0.05], [0.02, 0.0505], [0.0103, 0.0505]]));
  near(slideGap(thin, beside, [0, 1, 0], 0.2).gap, 0.0003, 1e-9);
  const face = solid('w', 'face', [0, 0.01], poly([[0.01, -0.1], [0.02, -0.1], [0.02, 0.3], [0.01, 0.3]]));
  near(slideGap(thin, face, [0, 1, 0], 0.2).gap, 0, 1e-12);
});

/** A toy bird: a plate, a shaft through it, a gear on the shaft forward of it, a tie round the gear's hub. */
function toyBird() {
  const parts = [];
  addPart(parts, 'plate', [S('plate', [0, 0.002], poly([[-0.03, -0.03], [0.03, -0.03], [0.03, 0.03], [-0.03, 0.03]], [{ c: [0, 0], r: 0.004 }]))]);
  addPart(parts, 'shaft', [S('shaft', [-0.001, 0.05], ring([0, 0], 0.004))]);
  addPart(parts, 'gear', [S('teeth', [0.01, 0.02], ring([0, 0], 0.02, 0.004))]);
  addPart(parts, 'key', [S('key', [0.01, 0.02], poly([[-0.0015, 0.0025], [0.0015, 0.0025], [0.0015, 0.0055], [-0.0015, 0.0055]]))]);
  addPart(parts, 'tie', [S('tie', [0.03, 0.0325], ring([0, 0], 0.006, 0.004))]);
  return { parts, joins: [['gear', 'shaft', 'on its shaft'], ['key', 'gear', KEYWAY], ['key', 'shaft', KEYWAY], ['tie', 'shaft', 'round it']] };
}
const seqDesign = (steps) => ({ drive: { ampNominal: 0.5 }, params: { layout: { sequence: { pose: { phase_rad: 0 }, steps } } } });
const lay = { law: null, fb: null };

test('the sequence check: a workable order passes; the gear before its shaft is fine, the plate after the gear on the wrong side passes through', () => {
  const good = [
    { name: 'plate', members: ['^plate$'], dir: [-1, 0, 0], dist_m: 0.05 },
    { name: 'shaft', members: ['^shaft$'], dir: [1, 0, 0], dist_m: 0.1 },
    { name: 'gear and key', members: ['^gear$', '^key$'], dir: [1, 0, 0], dist_m: 0.1 },
    { name: 'tie', members: ['^tie$'], threaded: true },
  ];
  const r = sequenceRecord(toyBird(), seqDesign(good), lay);
  assert.ok(r.passed, JSON.stringify(r.steps));
  assert.deepEqual(r.missing, []);
  // the key in its keyway touches nothing it may not; the tie, threaded, binds the shaft already there
  assert.deepEqual(r.steps.at(-1).binds, ['shaft']);
  // the shaft from aft passes the plate's bore (touching), from forward through the gear placed first it would not
  const bad = [good[0], good[2], { ...good[1], dir: [-1, 0, 0] }, good[3]];
  const rb = sequenceRecord(toyBird(), seqDesign(bad), lay);
  assert.equal(rb.passed, true, 'the shaft through the gear\'s bore and the plate\'s bore touches both');
  // the plate after the gear, coming from forward over it: the plate's solid face through the gear's teeth
  const worse = [good[1], good[2], { ...good[0], dir: [1, 0, 0] }, good[3]];
  const rw = sequenceRecord(toyBird(), seqDesign(worse), lay);
  assert.equal(rw.passed, false);
  assert.deepEqual(rw.failures, ['plate']);
  assert.ok(rw.steps.find((s) => s.name === 'plate').through.some((t) => t.placed === 'gear/teeth'));
  // the tie before the shaft it binds: refused; a solid in no step: refused
  const early = [good[3], ...good.slice(0, 3)];
  const re = sequenceRecord(toyBird(), seqDesign(early), lay);
  // (and the gear, coming on from forward, then passes through the tie round the shaft)
  assert.deepEqual(re.failures, ['tie', 'gear and key']); assert.deepEqual(re.steps[0].binds_not_yet_placed, ['shaft']);
  const missing = sequenceRecord(toyBird(), seqDesign(good.slice(0, 3)), lay);
  assert.equal(missing.passed, false); assert.deepEqual(missing.missing, ['tie/tie']);
});

test('steps resolve each solid once, in order; a step that names nothing is reported', () => {
  const r = resolveSteps(toyBird(), { steps: [{ name: 'all', members: ['.'], dir: [1, 0, 0] }, { name: 'again', members: ['^plate$'], dir: [1, 0, 0] }] });
  assert.equal(r.steps.length, 1); assert.equal(r.steps[0].solids.length, 5); assert.deepEqual(r.empty, ['again']);
  near(Math.hypot(...resolveSteps(toyBird(), { steps: [{ name: 'a', members: ['.'], dir: [3, 0, 4] }] }).steps[0].dir), 1, 1e-12);
});

test('exempt joint classes: mesh, keyway, a shank through its hole, its tapped hole, a rib\'s hole, a tube\'s bore, a washer or circlip\'s groove', () => {
  assert.equal(EXEMPT_JOINTS.length, 7);
  for (const j of [MESH, KEYWAY, THROUGH, GROOVE]) assert.ok(EXEMPT_JOINTS.includes(j));
});

// ---- round 16's stated design and its refusal ----------------------------------------------------------------------

test('round 16 states every step with a unit-able direction and a positive distance, ties threaded', () => {
  for (const s of SEQUENCE.steps) {
    if (s.threaded) { assert.match(s.name, /ties?$/); continue; }
    assert.ok(Math.hypot(...s.dir) > 0 && s.dist_m > 0, s.name);
  }
  assert.ok(ROUND16.crank.web.separate);
  // DIN 6885 for a 10 mm shaft: 3 x 3 mm, keyseat 1.8 mm; over the 6 mm web
  assert.deepEqual([ROUND16.crank.web.key.b_mm, ROUND16.crank.web.key.h_mm, ROUND16.crank.web.key.shaft_depth_mm, ROUND16.crank.web.key.L_mm], [3, 3, 1.8, 6]);
});

test('--official refuses an incomplete design or a failing sequence, and names why', () => {
  const okRec = { passed: true, clashes: [], pod_shell: { clashes: [] }, support: { passed: true, unsupported: [] }, containment: { passed: true, violations: [], outside_violations: [] }, completeness: { passed: true, failed: [], untagged: [], unknown_tags: [] }, assembly: { passed: true, failures: [], missing: [] } };
  assert.equal(officialRefusal(okRec), null);
  assert.match(officialRefusal({ ...okRec, passed: false, completeness: { passed: false, failed: ['M3 nuts, 24 off'], untagged: [], unknown_tags: [] } }), /incomplete: M3 nuts, 24 off/);
  assert.match(officialRefusal({ ...okRec, passed: false, assembly: { passed: false, failures: ['crank'], missing: ['x/y'] } }), /the sequence fails: crank, 1 solids in no step/);
  assert.match(officialRefusal({ ...okRec, completeness: undefined }), /no completeness record/);
  assert.match(officialRefusal({ ...okRec, assembly: undefined }), /no assembly sequence record/);
  assert.match(officialRefusal(null), /no clearance record/);
});

// ---- the check's core (the v23 mutation survivors: inShape, sampleMoving, plateSolids, solidPairs) ---------------

import { inShape, sampleMoving, plateSolids, solidPairs, partPairClashes, podRecord } from '../../src/design/clearance.js';

test('inShape: a ring\'s annulus with both edges, a disc with no bore, a capsule to its radius, a polygon less its holes', () => {
  const r = ring([0.01, 0.02], 0.005, 0.002);
  assert.ok(inShape(r, [0.015, 0.02]) && inShape(r, [0.012, 0.02])); // on the outer and the inner edge
  assert.ok(!inShape(r, [0.0151, 0.02]) && !inShape(r, [0.0119, 0.02]) && !inShape(r, [0.01, 0.02]));
  assert.ok(inShape({ kind: 'ring', c: [0, 0], ro: 0.003 }, [0, 0])); // no bore: the centre is in it
  const c = cap([0, 0], [0.01, 0], 0.002);
  assert.ok(inShape(c, [0.005, 0.002]) && inShape(c, [0.012, 0]) && !inShape(c, [0.005, 0.0021]) && !inShape(c, [-0.0021, 0]));
  const p = poly([[0, 0], [0.01, 0], [0.01, 0.01], [0, 0.01]], [{ c: [0.005, 0.005], r: 0.002 }]);
  assert.ok(inShape(p, [0.001, 0.001]) && inShape(p, [0.01, 0.005]) && !inShape(p, [0.0101, 0.005]));
  assert.ok(!inShape(p, [0.005, 0.005]) && !inShape(p, [0.0069, 0.005]) && inShape(p, [0.007, 0.005])); // in the hole, at its edge
  assert.ok(inShape({ kind: 'poly', pts: p.pts }, [0.005, 0.005])); // no holes listed
});

test('sampleMoving: a pair\'s smallest gap over the phases, refined between samples when under 10 mm, its phase', () => {
  // a 2 mm disc on a 20 mm orbit about the origin (phase q.ph) and a fixed 3 mm disc at (25 mm, 0): closest at phase 0,
  // gap 25 - 20 - 2 - 3 = 0 mm; sampled at 7 phases (0 is a sample), and at phases offset so the minimum falls between
  const mk = (off) => ({ a: { id: 'm/disc', part: 'm', x: [0, 0.01], at: (q) => ring([0.02 * Math.cos(q.ph + off), 0.02 * Math.sin(q.ph + off)], 0.002) }, b: { id: 'f/disc', part: 'f', x: [0, 0.01], shape: ring([0.025, 0], 0.003) }, min: Infinity, at: null });
  const poseAt = (amp, ph) => ({ ph, amp });
  const exact = mk(0);
  sampleMoving([exact], [1], 7, poseAt, poseAt(1, 0));
  near(exact.min, 0, 1e-12); near(exact.phase, 0, 1e-12);
  // the orbit started 0.3 rad on: the samples miss the closest point by 0.3 rad, the refinement finds it (6.0 rad)
  const off = mk(0.3);
  sampleMoving([off], [1], 7, poseAt, poseAt(1, 0));
  near(off.min, 0, 1e-7); near(off.phase, 2 * Math.PI - 0.3, 1e-3);
  // a pair whose sampled gap is over 10 mm is left at its sample (no refinement)
  const far = { a: { ...mk(0.3).a }, b: { id: 'f/disc', part: 'f', x: [0, 0.01], shape: ring([0.040, 0], 0.003) }, min: Infinity, at: null };
  sampleMoving([far], [1], 7, poseAt, poseAt(1, 0));
  const sampled = Math.min(...Array.from({ length: 7 }, (_, i) => Math.hypot(0.04 - 0.02 * Math.cos((i / 7) * 2 * Math.PI + 0.3), 0.02 * Math.sin((i / 7) * 2 * Math.PI + 0.3)) - 0.005));
  near(far.min, sampled, 1e-12);
  assert.ok(far.min > 0.015 + 1e-6, 'the true minimum, 15 mm, is not sought');
  // two amplitudes: the worse one wins (a larger orbit comes closer)
  const two = { a: { id: 'm/disc', part: 'm', x: [0, 0.01], at: (q) => ring([0.02 * q.amp * Math.cos(q.ph), 0], 0.002) }, b: { id: 'f/disc', part: 'f', x: [0, 0.01], shape: ring([0.03, 0], 0.003) }, min: Infinity, at: null };
  sampleMoving([two], [1, 1.1], 4, poseAt, poseAt(1, 0));
  near(two.min, 0.03 - 0.022 - 0.005, 1e-12); assert.equal(two.at.amp, 1.1);
});

test('solidPairs and the clash rows: solids of one part and meshes exempt, joined pairs at zero, the rest at the clearance', () => {
  const parts = [], d0 = (name, x, sh) => ({ name, x, shape: sh });
  addPart(parts, 'a', [d0('one', [0, 0.01], ring([0, 0], 0.005)), d0('two', [0, 0.01], ring([0, 0], 0.004))]);
  addPart(parts, 'b', [d0('one', [0, 0.01], ring([0.0102, 0], 0.005))]);
  addPart(parts, 'c', [d0('one', [0, 0.01], ring([-0.0103, 0], 0.005))]);
  addPart(parts, 'g', [d0('teeth', [0, 0.01], ring([0, 0.014], 0.012))]);
  const asm = { parts, joins: [['a', 'b', 'bonded'], ['a/one', 'c', 'on it'], ['a', 'g', MESH]] };
  const { pairs, nExempt } = solidPairs(asm, 0.0005);
  // a/one-a/two is one part's; a-g a mesh (two solids of a): 3 exempt; the rest checked
  assert.equal(nExempt, 3);
  assert.equal(pairs.length, 10 - 3);
  const pr = (x, y) => pairs.find((p) => (p.a.id === x && p.b.id === y) || (p.a.id === y && p.b.id === x));
  assert.equal(pr('a/one', 'b/one').required, 0); assert.equal(pr('a/one', 'b/one').join, 'bonded');
  assert.equal(pr('a/one', 'c/one').required, 0); assert.equal(pr('a/two', 'c/one').join, undefined); // the join names a/one only
  assert.equal(pr('b/one', 'c/one').required, 0.0005);
  for (const p of pairs) { p.min = gap2(p.a.shape, p.b.shape); p.class = 'fixed / fixed'; }
  // a-b 0.2 mm apart (joined: fine), a/one-c 0.3 mm (joined: fine), a/two-c 1.3 mm (fine); b-g 0.30 mm and c-g 0.38 mm
  // apart, under the 0.5 mm
  const rows = partPairClashes(pairs);
  assert.deepEqual(rows.map((r) => `${r.a}|${r.b}`).sort(), ['b/one|g/teeth', 'c/one|g/teeth'].sort());
  const pod = podRecord([{ part: 'x', gap_m: 0.0004 }, { part: 'y', gap_m: 0.0006 }, { part: 'z', gap_m: -0.001, protrusion_only: true }], 0.0005);
  assert.deepEqual(pod.clashes, ['x']); assert.equal(pod.rows[2].note.includes('protrusion'), true);
});

test('plateSolids: the outline with its holes, each post and pad (the boom\'s hole in them), each lock ear by its own area, the bosses', () => {
  const af = { rails: { right: [0.03, 0], left: [-0.03, 0] }, plates: { aft: { outline_m: [[-0.04, -0.05], [0.04, -0.05], [0.04, 0.01], [-0.04, 0.01]], posts: [{ to: 'boom clamp', post_m: [-0.008, 0.008, 0.01, 0.05], pad_m: [-0.01, 0.01, 0.05, 0.07] }, { to: 'aft tray', post_m: null }], ears: [{ rail: 'right', outline_m: [[0.03, 0], [0.05, 0], [0.05, 0.02]], area_m2: 1e-4 }] } } };
  const ax = { P: { plateAft: 0.014, plateFwd: 0.075 }, tA: 0.002, tF: 0.0012, af, plateIds: { aft: 'gearbox plate aft', fwd: 'gearbox plate forward' }, assumptions: [] };
  const holes = [{ c: [0, -0.03], r: 0.006 }], boomHole = [{ c: [0, 0.062], r: 0.0055 }], bosses = [{ name: 'boss', x: [0.015, 0.02], shape: ring([0, -0.03], 0.01, 0.006) }];
  const s = plateSolids(ax, 'aft', holes, boomHole, bosses);
  assert.deepEqual(s.map((q) => q.name), ['plate', 'post 1 (boom clamp)', 'pad 1 (boom clamp)', 'ear 1 (rail lock right)', 'boss']);
  near(s[0].x[0], 0.013, 1e-12); near(s[0].x[1], 0.015, 1e-12); assert.deepEqual(s[0].shape.holes, [...holes, ...boomHole]);
  assert.deepEqual(s[1].shape.holes, boomHole); assert.deepEqual(s[2].shape.pts.map((p) => p[1]), [0.05, 0.05, 0.07, 0.07]);
  assert.equal(s[3].area_m2, 1e-4); assert.deepEqual(s[3].shape.holes, holes);
  // the forward plate has no outline here: it is checked by its bosses alone, and says so
  const f = plateSolids(ax, 'fwd', [], [], []);
  assert.deepEqual(f, []); assert.ok(ax.assumptions.some((a) => /no outline/.test(a)));
});

import { holeRecord, THREAD } from '../../src/design/clearance.js';
import { clampFlange } from '../../src/design/airframe.js';

test('holeRecord: a shank through a part must lie in its material; its nut is not asked to hold it', () => {
  const parts = [];
  addPart(parts, 'plate', [S('plate', [0, 0.002], poly([[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]]))]);
  const inside = screwX([0.005, 0], 0, 1, m3), edge = screwX([0.0091, 0], 0, 1, m3);
  addPart(parts, 'screw a', inside.screw); addPart(parts, 'screw b', edge.screw);
  addPart(parts, 'nut a', [S('nut', [0.004, 0.0064], ring([0.005, 0], 0.0032, 0.0015))]);
  const joins = [['screw a/shank', 'plate', THROUGH], ['screw b/shank', 'plate', THROUGH], ['screw a/shank', 'nut a', THREAD]];
  const r = holeRecord({ parts, joins });
  // a's 3 mm shank 5 mm from the plate's 10 mm edge: in it; b's at 9.1 mm: 0.6 mm of its hole past the edge
  assert.equal(r.holes, 2); assert.equal(r.passed, false); assert.deepEqual(r.failures, [{ screw: 'screw b/shank', part: 'plate' }]);
});

test('clampFlange: a row of two screws 14 mm apart, 8.5 mm below the boom, each with 6 mm of metal round it', () => {
  const f = clampFlange({ screw_edge_mm: 6, screw_pitch_mm: 14, screws: 2, screw_offset_mm: 8.5 }, 0.013, 0.062);
  near(f.z[0], -0.013); near(f.z[1], 0.013); near(f.y[0], 0.062 - 0.0085 - 0.006); near(f.y[1], 0.062 - 0.0065);
  assert.equal(clampFlange({ screw_pitch_mm: 14 }, 0.013, 0.062), null);
});

// ---- a v24-like build: the v21 fixture's params with round 16's set (no fit, the fixture's pod) --------------------

import { buildDesign } from '../../src/design/build.js';
import { round16Set, mergeDeep } from '../../src/design/round16.mjs';
import { fullAssembly as fullAsm } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';
let V24 = null;
function v24Like() {
  if (V24) return V24;
  const v21 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
  const d = buildDesign(mergeDeep(v21.params, round16Set({}, catalog)), catalog);
  V24 = { d, ...fullAsm(d, catalog) };
  return V24;
}

test('v24-like: every screw at its place, by joint and by length; the BOM counts them; each hole in its parts\' metal', () => {
  const { d, asm } = v24Like();
  const fx = d.structure.airframe.parts.fixings, by = (j) => fx.filter((b) => b.joint === j);
  // two lock screws a rail, two a boom clamp, two a tray root, two each for the riser's foot and the nose tray, four motor
  assert.deepEqual(['rail lock right', 'rail lock left', 'boom clamp aft', 'boom clamp forward', 'forward tray', 'aft tray', 'nose tray riser', 'nose tray', 'motor bracket'].map((j) => by(j).length), [2, 2, 2, 2, 2, 2, 2, 2, 4]);
  // a boom clamp's stack: its 6 mm clamp and a plate (2.0 aft, 1.2 forward) and a 2.4 mm nut: over 8 mm, so M3 x 12
  for (const b of [...by('boom clamp aft'), ...by('boom clamp forward')]) assert.equal(b.screw, 'screw-iso4762-m3x12-10.9');
  near(by('boom clamp aft')[0].stack_m, 0.006 + 0.002, 1e-9); near(by('boom clamp forward')[0].stack_m, 0.006 + 0.0012, 1e-9);
  // the rail lock: the aft plate's 2.0 mm and the 1.4 mm flange, its head inside the box on the plate's front face
  for (const b of [...by('rail lock right'), ...by('rail lock left')]) { near(b.stack_m, 0.0034, 1e-9); assert.equal(b.screw, 'screw-iso4762-m3x8-10.9'); assert.equal(b.dir, -1); }
  // the motor's four into its own threads, no nuts
  assert.ok(by('motor bracket').every((b) => b.tapped === 'motor' && b.nut === null));
  const lines = d.bom.structure.filter((s) => /^M3 |M3 x 12/.test(s.name)).map((s) => s.name).sort();
  assert.deepEqual(lines, ['ISO 4762 M3 x 12 socket head cap screw, class 10.9, 4 off', 'M3 nuts, 16 off', 'M3 screws, 16 off']);
  const h = holeRecord(asm);
  assert.ok(h.passed, JSON.stringify(h.failures)); assert.ok(h.holes >= 40);
});

test('v24-like: the BOM\'s lines match their solids, the wings and tail drawn, the formers on the loft', () => {
  const { d, asm } = v24Like();
  const { lay } = v24Like(), q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal);
  const k = completenessRecord(asm, d, catalog, q0);
  for (const line of ['inner spar, 2 off', 'outer spar, 2 off', 'leading edge rod, 2 off', 'ribs, 20 off', 'tail rods, 4 off', 'tail ribs, 8 off', 'twist torque rods, 2 off', 'fuselage formers', 'keel and tail boom', 'gearbox panels, 3 off', 'tail boom clamp blocks, 2 off', 'crank web and pin', 'key, crank web']) {
    const r = k.lines.find((x) => x.line === line);
    assert.ok(r?.passed, `${line}: ${JSON.stringify(r)}`);
  }
  // the inner spar from the socket's base (the hub's 14.5 mm, half of it) to the elbow
  near(d.bom.structure.find((s) => s.name === 'inner spar, 2 off').mass_g, 43 * 2 * (0.4875 - 0.00725), 1e-6);
  assert.equal(asm.parts.filter((p) => /^rib [RL] \d+$/.test(p.id)).length, 20);
  assert.equal(asm.parts.filter((p) => /^former \d+$/.test(p.id)).length, 5);
  assert.equal(d.tail.y_m, d.structure.airframe.parts.boom_y_m, 'the tail\'s roots at the boom\'s height');
  // the keel and the formers come with the airframe's items and have lumps of their own: counted once, so the flown
  // mass is the BOM's (M6)
  near(d.mass.total_g, d.bom.total_g, 0.5);
  const lumpG = (n) => d.mass.lumps.find((l) => l.name === n).mass_kg * 1000 / 1.15;
  near(lumpG('keel and boom'), d.bom.structure.find((s) => s.name === 'keel and tail boom').mass_g, 1e-6);
  near(lumpG('fuselage shell and formers') - d.bom.structure.find((s) => s.name === 'fuselage formers').mass_g, d.bom.structure.find((s) => s.name === 'fuselage film shell').mass_g, 1e-6);
});

test('v24-like: the v23 one-piece crank (web and pin on the shaft) cannot go in: its web meets the aft bushing and plate', () => {
  const v21 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
  const p = mergeDeep(v21.params, round16Set({}, catalog));
  p.linkage.crank.web.separate = false;
  const d = buildDesign(p, catalog), { asm, lay } = fullAsm(d, catalog);
  assert.deepEqual(asm.parts.find((x) => x.id === 'crank').solids.map((s) => s.name).sort(), ['pin', 'shaft', 'web', 'wheel seat']);
  const r = sequenceRecord(asm, d, lay), st = r.steps.find((s) => /^crank shaft/.test(s.name));
  assert.equal(st.failed, true);
  assert.ok(st.through.some((t) => t.moving === 'crank/web' && /journal aft|gearbox plate aft/.test(t.placed)), JSON.stringify(st.through));
  assert.ok(r.empty.includes('crank web with its key'));
  // split (the v24 crank): the shaft's step is clear
  const { asm: a2, d: d2, lay: l2 } = v24Like();
  assert.ok(!sequenceRecord(a2, d2, l2).steps.find((s) => /^crank shaft/.test(s.name)).failed);
});

// ---- the records a design states, and the rail locks' three layouts ------------------------------------------------

import { designRecords } from '../../src/design/clearance.js';
import { round16Note } from '../../src/design/round16.mjs';

test('designRecords: each record only where the design states it, the packed ones only with the packed parts', () => {
  const { d, asm, lay } = v24Like(), af = d.structure.airframe, q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal);
  const pk = { pod: [], model: null };
  const withLayout = (Ly) => ({ ...d, params: { ...d.params, layout: Ly } });
  // nothing stated, no fixings: no records
  assert.deepEqual(designRecords(asm, withLayout({}), catalog, lay, { ...af, parts: { ...af.parts, fixings: undefined } }, pk, q0, { required: 5e-4, pack: true }), {});
  // the fixings drawn: the hole record alone
  assert.deepEqual(Object.keys(designRecords(asm, withLayout({}), catalog, lay, af, pk, q0, { required: 5e-4, pack: true })), ['holes']);
  // completeness and the sequence need the packed parts; mounts need the packing model
  const Ly = { completeness: true, sequence: d.params.layout.sequence, mounts: d.params.layout.mounts };
  assert.deepEqual(Object.keys(designRecords(asm, withLayout(Ly), catalog, lay, af, pk, q0, { required: 5e-4, pack: false })), ['holes']);
  const r = designRecords(asm, withLayout(Ly), catalog, lay, af, pk, q0, { required: 5e-4, pack: true });
  assert.deepEqual(Object.keys(r), ['completeness', 'assembly', 'holes']);
  // the v21 fixture's 8 mm encoder actuator: its catalogue entry gives no outside diameter or height, so its size
  // cannot be matched and the line fails (v24's 10 mm actuator states both)
  assert.deepEqual(r.completeness.failed, ['sensor-rls-orbis-actuator-8mm']);
  assert.deepEqual(r.completeness.lines.find((l) => l.line === 'sensor-rls-orbis-actuator-8mm').size.filter((x) => !x.ok).map((x) => x.bom_mm), [null, null]);
  assert.equal(r.holes.passed, true);
});

test('round16Note: each check passed, failed (naming what failed) or not run', () => {
  assert.match(round16Note(null), /Completeness not run; assembly not run\.$/);
  const ok = { completeness: { passed: true, exemptions: [{ line: 'wing film' }, { line: 'glue' }] }, assembly: { passed: true } };
  assert.match(round16Note(ok), /exempt: wing film; glue\)/);
  assert.match(round16Note(ok), /Completeness passed; assembly passed\.$/);
  const bad = { completeness: { passed: false, failed: ['M3 nuts, 16 off', 'phantom'], exemptions: [] }, assembly: { passed: false, failures: ['rails'], missing: ['a/x', 'b/y'] } };
  assert.match(round16Note(bad), /Completeness FAILED \(M3 nuts, 16 off, phantom\); assembly FAILED \(rails, a\/x, b\/y\)\.$/);
});

test('rail lock screws: forward (through the forward plate from its outer face), aft (v21: from the aft plate\'s aft face) and aft outer (round 16: from inside the box, through the aft plate and any boss)', () => {
  const v21 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
  // the v21 rail states no lock arc: no lock screws are laid out
  assert.equal(buildDesign(v21.params, catalog).structure.airframe.parts.fixings?.filter((b) => /^rail lock/.test(b.joint)).length ?? 0, 0);
  const p16 = mergeDeep(v21.params, round16Set({}, catalog));
  const locks = (mut) => { const p = structuredClone(p16); mut(p.linkage.airframe.rail); return buildDesign(p, catalog).structure.airframe.parts.fixings.filter((b) => b.joint === 'rail lock right'); };
  const [fw] = locks((r) => { r.lock_at = 'forward'; }), [af] = locks((r) => { r.lock_at = 'aft'; }), [ao] = locks((r) => { r.lock_at = 'aft outer'; });
  // stacks: the forward plate 1.2 + the 1.4 mm flange; the aft plate 2.0 + 1.4
  near(fw.stack_m, 0.0026, 1e-9); near(af.stack_m, 0.0034, 1e-9); near(ao.stack_m, 0.0034, 1e-9);
  assert.deepEqual([fw.dir, af.dir, ao.dir], [-1, 1, -1]);
  assert.deepEqual(fw.through, ['gearbox plate forward', 'shoulder rail right']);
  assert.deepEqual(af.through, ['gearbox plate aft', 'shoulder rail right']);
  // aft outer: the head on the plate's front face, a plate's thickness forward of the v21 head on its aft face
  near(ao.face - af.face, 0.002, 1e-9);
  for (const b of [fw, af, ao]) { near(b.nut_face, b.face + b.dir * b.stack_m, 1e-12); assert.equal(b.screw, 'screw-iso4762-m3x8-10.9'); }
  // a 3 mm boss on the aft face under the flange joins the stack: 6.4 mm and a 2.4 mm nut need the M3 x 12
  const [bo] = locks((r) => { r.lock_at = 'aft outer'; r.aft_boss_mm = 3; r.aft_boss_at = 'aft face'; });
  near(bo.stack_m, 0.0064, 1e-9);
  assert.deepEqual(bo.through, ['gearbox plate aft', 'shoulder rail right aft seat boss', 'shoulder rail right']);
  assert.equal(bo.screw, 'screw-iso4762-m3x12-10.9');
});

import { carrierGeometry } from '../../src/design/airframe.js';

test('carrierGeometry: the trays from the plates\' own faces; round 16\'s outer doubler and the riser\'s actuator hole', () => {
  const tray = { y_mm: -80, w_mm: 38.4, flange_h_mm: 7, t_mm: 0.8, flange_mm: 8, doubler_mm: 2, material: 'al' };
  const A = (trayX, riserX = {}) => ({
    tray: { ...tray, ...trayX },
    noseTray: { y_mm: -43, w_mm: 37.6, flange_h_mm: 5, t_mm: 0.6, flange_mm: 8, riser_x_mm: 100, tip_x_mm: 228, riser: { t_mm: 0.8, w_mm: 30, flange_mm: 7, foot_mm: 8, ...riserX } },
    aftTray: { y_mm: -81.2, w_mm: 53.2, flange_h_mm: 6, t_mm: 0.6, flange_mm: 8, length_mm: 123 },
  });
  const planes = { plateFwd: 0.0762, plateAft: 0.0142, encoder: { od: { actuator: 0.022 } } };
  // the forward plate's outer face 76.2 + 0.6 = 76.8 mm, the aft plate's aft face 14.2 - 1.0 = 13.2 mm
  const g = carrierGeometry(A({ doubler_at: 'outer', doubler_h_mm: 12 }, { actuator_hole: true }), planes, 0.0012, 0.002, [0, -0.02]);
  near(g.forward.x[0], 0.0788, 1e-12); near(g.forward.x[1], 0.1008, 1e-12);
  assert.deepEqual(g.nose.x, [0.1, 0.228]);
  near(g.aft.x[0], 0.0132 - 0.123, 1e-12); near(g.aft.x[1], 0.0132, 1e-12);
  // the doubler: the plate's face to the tray's root, 12 mm high centred on the root tab (y -80 + 0.4 + 4 = -75.6 mm)
  near(g.doubler.x[0], 0.0768, 1e-12); near(g.doubler.x[1], 0.0788, 1e-12);
  near(g.doubler.y[0], -0.0816, 1e-12); near(g.doubler.y[1], -0.0696, 1e-12); near(g.doubler.w, 0.0384, 1e-12);
  // the riser from the forward tray's top to the nose tray's underside, the tray's material; the hole 11 + 0.5 mm
  near(g.riser.y[0], -0.0796, 1e-12); near(g.riser.y[1], -0.0433, 1e-12); assert.equal(g.riser.material, 'al');
  assert.deepEqual(g.riser.hole.c, [0, -0.02]); near(g.riser.hole.r, 0.0115, 1e-12);
  // the default doubler height is 24 mm
  near(carrierGeometry(A({ doubler_at: 'outer' }), planes, 0.0012, 0.002).doubler.y[1] - (-0.0756), 0.012, 1e-12);
  // v23: the doubler inside the box (not between tray and plate), no hole without the crank's axis
  const v23 = carrierGeometry(A({}, { actuator_hole: true, material: 'cf' }), planes, 0.0012, 0.002);
  near(v23.forward.x[0], 0.0768, 1e-12);
  assert.equal(v23.doubler, undefined); assert.equal(v23.riser.hole, undefined); assert.equal(v23.riser.material, 'cf');
});

// ---- the formers and the fixings as the check draws them (round 16's surviving mutants: formerParts 1/32, fixingParts 1/8)

import { loftHalf } from '../../sims/avis-shared/model.js';
import { RIB_HOLE } from '../../src/design/clearance.js';

test('v24-like formers: five 2 mm bands, 4 mm deep inside the loft\'s smaller section, each with its tab tied to the tray above', () => {
  const { d, asm } = v24Like(), part = (id) => asm.parts.find((p) => p.id === id);
  const joined = (a, b, w) => asm.joins.some(([x, y, v]) => ((x === a && y === b) || (x === b && y === a)) && (w == null || v === w));
  const c = d.structure.airframe.parts.carriers, key = { 'aft tray': 'aft', 'forward tray': 'forward', 'nose tray': 'nose' };
  // each former's tray: the one whose x range holds it (aft tray -109.8..13.2 mm, forward 78.8.., nose from the riser)
  const trays = [-100, -30, 97, 165, 200].map((xmm) => ['nose tray', 'aft tray', 'forward tray'].find((t) => xmm / 1000 >= c[key[t]].x[0] && xmm / 1000 <= c[key[t]].x[1]));
  assert.deepEqual(trays, ['aft tray', 'aft tray', 'forward tray', 'nose tray', 'nose tray']);
  [-100, -30, 97, 165, 200].forEach((xmm, k) => {
    const f = part(`former ${k + 1}`), x = xmm / 1000, T = c[key[trays[k]]];
    assert.equal(f.solids.length, 49);
    for (const s of f.solids) { assert.equal(s.bom, 'fuselage formers'); near(s.x[0], x - 0.001, 1e-12); near(s.x[1], x + 0.001, 1e-12); }
    // the smaller of the loft's sections at the former's two faces; every band's outer corners on it, inner 4 mm in
    const [a, b] = [loftHalf(d.body.loft, x - 0.001), loftHalf(d.body.loft, x + 0.001)], h = a.hw * a.hh <= b.hw * b.hh ? a : b;
    for (const s of f.solids.filter((q) => /^band /.test(q.name))) {
      const [p0, p1, q1, q0] = s.shape.pts;
      for (const q of [p0, p1]) near((q[0] / h.hw) ** 2 + ((q[1] - h.y) / h.hh) ** 2, 1, 1e-9);
      for (const q of [q0, q1]) near((q[0] / (h.hw - 0.004)) ** 2 + ((q[1] - h.y) / (h.hh - 0.004)) ** 2, 1, 1e-9);
    }
    // the tab: 2 mm wide just outboard of the tray's -z flange, from the flange's top to the band
    const tab = f.solids.find((q) => q.name === 'tab').shape, z = tab.pts.map((q) => q[0]), y = tab.pts.map((q) => q[1]);
    near(Math.max(...z), -(T.w / 2 + T.t / 2), 1e-12); near(Math.max(...z) - Math.min(...z), 0.002, 1e-12);
    const top = T.y + T.h + T.t / 2;
    assert.ok(Math.abs(Math.max(...y) - top) < 1e-12 || Math.abs(Math.min(...y) - top) < 1e-12, `former ${k + 1}: tab ${y} to the flange top ${top}`);
    assert.ok(joined(`former ${k + 1}`, trays[k]));
    assert.ok(joined(`former ${k + 1} tie`, `former ${k + 1}`) && joined(`former ${k + 1} tie`, trays[k]) && joined(`former ${k + 1} tie/tie bottom`, `former ${k + 1}`, RIB_HOLE));
  });
});

test('v24-like fixings: each screw\'s head on its face, its shank through what it clamps, its nut on the far face (ISO 4762 / 4032 M3)', () => {
  const { d, asm } = v24Like(), fx = d.structure.airframe.parts.fixings, part = (id) => asm.parts.find((p) => p.id === id);
  const joinOf = (a, b) => asm.joins.find(([x, y]) => (x === a && y === b) || (x === b && y === a))?.[2];
  const n = new Map();
  for (const b of fx) {
    const k = (n.get(b.joint) ?? 0) + 1; n.set(b.joint, k);
    const sc = part(`${b.joint} screw ${k}`);
    // M3: head 5.5 mm across and 3 mm high, shank 3 mm, as long as the screw (8 or 12 mm); along x or vertical (y)
    const L = b.screw === 'screw-iso4762-m3x12-10.9' ? 0.012 : 0.008;
    if (b.axis === 'x') {
      const head = sc.solids.find((s) => s.name === 'head'), shank = sc.solids.find((s) => s.name === 'shank');
      near(head.shape.ro, 0.00275, 1e-12); near(shank.shape.ro, 0.0015, 1e-12);
      near(b.dir > 0 ? head.x[1] : head.x[0], b.face, 1e-12); near(head.x[1] - head.x[0], 0.003, 1e-12);
      near(b.dir > 0 ? shank.x[0] : shank.x[1], b.face, 1e-12); near(shank.x[1] - shank.x[0], L, 1e-12);
      assert.deepEqual(head.shape.c, b.c);
      for (const p of b.through) { assert.equal(joinOf(`${sc.id}/shank`, p), THROUGH); assert.match(joinOf(sc.id, p), /fastened/); }
      const nut = part(`${b.joint} nut ${k}`);
      if (b.tapped) { assert.equal(nut, undefined); assert.equal(joinOf(`${sc.id}/shank`, b.tapped), THREAD); continue; }
      assert.equal(joinOf(`${sc.id}/shank`, nut.id), THREAD);
      assert.equal(joinOf(nut.id, b.through.at(-1)), 'on its face');
      // the nut: 5.5 mm across flats (round 17b: its hexagon, corners at 5.5 / sqrt 3), 2.4 mm high, from the far
      // face out, bored to the catalogue's 2.5 mm minor diameter
      const ns = nut.solids[0];
      near(Math.max(...ns.shape.pts.map((p) => Math.hypot(p[0] - b.c[0], p[1] - b.c[1]))), 0.0055 / Math.sqrt(3), 1e-12); near(ns.shape.holes[0].r, 0.00125, 1e-12);
      near(b.dir > 0 ? ns.x[0] : ns.x[1], b.nut_face, 1e-12); near(ns.x[1] - ns.x[0], 0.0024, 1e-12);
    } else {
      // round 18.2 (the film's request): the shank its true section, N slabs across x (xSlabs, screwY), not a
      // square prism; the head stays the box (an open problem: see fasteners2.patch notes)
      const head = sc.solids.filter((s) => /^head/.test(s.name)), shank = sc.solids.filter((s) => /^shank/.test(s.name));
      assert.equal(shank.length, 8); assert.equal(head.length, 8);
      near(head[0].tube.axis === 'y' ? head[0].tube.ro : NaN, 0.00275, 1e-12); near(head[0].tube.L, 0.003, 1e-12); near(shank[0].tube.ro, 0.0015, 1e-12); near(shank[0].tube.L, L, 1e-12);
      const hx = [Math.min(...head.map((s) => s.x[0])), Math.max(...head.map((s) => s.x[1]))];
      near(hx[0], b.c[0] - 0.00275, 1e-12); near(hx[1], b.c[0] + 0.00275, 1e-12);
      for (const sl of shank) assert.equal(sl.shape.kind, 'poly');
      const ys = (slabs) => { const v = slabs.flatMap((q) => q.shape.pts.map((p) => p[1])); return [Math.min(...v), Math.max(...v)]; };
      const [h0, h1] = ys(head), [s0, s1] = ys(shank);
      near(b.dir > 0 ? h1 : h0, b.face, 1e-12); near(b.dir > 0 ? s0 : s1, b.face, 1e-12); near(s1 - s0, L, 1e-12);
      for (const p of b.through) { for (const sl of shank) assert.equal(joinOf(`${sc.id}/${sl.name}`, p), THROUGH); assert.match(joinOf(sc.id, p), /fastened/); }
      const nut = part(`${b.joint} nut ${k}`);
      if (b.tapped) { assert.equal(nut, undefined); for (const sl of shank) assert.equal(joinOf(`${sc.id}/${sl.name}`, b.tapped), THREAD); continue; }
      for (const sl of shank) assert.equal(joinOf(`${sc.id}/${sl.name}`, nut.id), THREAD);
      assert.equal(joinOf(nut.id, b.through.at(-1)), 'on its face');
    }
  }
  assert.equal(n.size, 9);
});

// ---- the sequence check's sweep (round 16's surviving mutants: slideRow 16/79, bbox 5/24, slideGap 19/39) ------------

/** A one-step sequence: `obstacle` placed, then `mover` along dir from dist out. -> that step's row. */
function oneStep(mover, obstacle, dir, dist = 0.1, pose = undefined) {
  const parts = [];
  addPart(parts, 'obstacle', [S('block', obstacle.x, obstacle.shape)]);
  addPart(parts, 'mover', [S('body', mover.x, mover.shape)]);
  const steps = [{ name: 'obstacle', members: ['^obstacle$'], dir: [0, 1, 0], dist_m: 0.05 }, { name: 'mover', members: ['^mover$'], dir, dist_m: dist }];
  const d = { drive: { ampNominal: 0.5 }, params: { layout: { sequence: { ...(pose ? { pose } : {}), steps } } } };
  return sequenceRecord({ parts, joins: [] }, d, lay);
}
const box = (z0, z1, y0, y1) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);

test('a step across x meets what lies midway along its path, whichever axis and sign, and whatever the moving solid\'s shape', () => {
  const movers = { ring: ring([0, 0], 0.002), cap: cap([-0.002, 0], [0.002, 0], 0.001), poly: box(-0.002, 0.002, -0.001, 0.001) };
  // the block 40 to 45 mm out along the step's axis, 10 mm across it: the mover starts 100 mm out, clear of it, and its
  // place is clear of it; only the path meets it
  const blocks = { '0,1,0': box(-0.005, 0.005, 0.04, 0.045), '0,-1,0': box(-0.005, 0.005, -0.045, -0.04), '0,0,1': box(0.04, 0.045, -0.005, 0.005), '0,0,-1': box(-0.045, -0.04, -0.005, 0.005) };
  for (const [kind, sh] of Object.entries(movers)) for (const [k, b] of Object.entries(blocks)) {
    const dir = k.split(',').map(Number), r = oneStep({ x: [0, 0.01], shape: sh }, { x: [0, 0.01], shape: b }, dir);
    assert.equal(r.steps[1].failed, true, `${kind} along ${k}`);
    const t = r.steps[1].through[0];
    assert.deepEqual([t.moving, t.placed, r.steps[1].n_through], ['mover/body', 'obstacle/block', 1]);
    // met when its leading edge reaches the block: 40 mm less its half-extent along the axis, of the 100 mm
    const half = { ring: 0.002, cap: Math.abs(dir[2]) ? 0.003 : 0.001, poly: Math.abs(dir[2]) ? 0.002 : 0.001 }[kind];
    near(t.at_frac, (0.04 - half) / 0.1, 0.0015);
    assert.ok(t.by_mm > 0 && t.by_mm <= 0.126, `${t.by_mm}`);
  }
});

test('the sweep\'s box: a block 0.8 mm beside the path is measured (the box\'s 1 mm margin), one 2 mm beside it is not', () => {
  const m = { x: [0, 0.01], shape: ring([0, 0], 0.002) };
  // beside a step along y, off in z; beside a step along z, off in y; beside a step along y, off in x
  const near8 = [[[0, 1, 0], { x: [0, 0.01], shape: box(0.0028, 0.01, 0.04, 0.045) }], [[0, -1, 0], { x: [0, 0.01], shape: box(-0.01, -0.0028, -0.045, -0.04) }],
    [[0, 0, 1], { x: [0, 0.01], shape: box(0.04, 0.045, 0.0028, 0.01) }], [[0, 0, -1], { x: [0, 0.01], shape: box(-0.045, -0.04, -0.01, -0.0028) }],
    [[0, 1, 0], { x: [0.0108, 0.02], shape: box(-0.005, 0.005, 0.04, 0.045) }], [[0, 1, 0], { x: [-0.02, -0.0008], shape: box(-0.005, 0.005, 0.04, 0.045) }]];
  for (const [dir, ob] of near8) {
    const r = oneStep(m, ob, dir).steps[1];
    assert.equal(r.failed, undefined);
    near(r.worst_gap_mm, 0.8, 1e-6);
  }
  for (const [dir, ob] of [[[0, 1, 0], { x: [0, 0.01], shape: box(0.004, 0.01, 0.04, 0.045) }], [[0, 0, 1], { x: [0, 0.01], shape: box(0.04, 0.045, 0.004, 0.01) }], [[0, 1, 0], { x: [0.012, 0.02], shape: box(-0.005, 0.005, 0.04, 0.045) }], [[0, 1, 0], { x: [0, 0.01], shape: box(-0.005, 0.005, 0.105, 0.11) }], [[0, -1, 0], { x: [0, 0.01], shape: box(-0.005, 0.005, -0.11, -0.105) }]]) {
    const r = oneStep(m, ob, dir).steps[1];
    assert.equal(r.worst_gap_mm, null, JSON.stringify(dir));
  }
});

test('a step along x: met on the way (no fraction: the sweep is exact) or already at its place (fraction 0); the row\'s fields', () => {
  const m = { x: [0, 0.01], shape: ring([0, 0], 0.002) }, wall = (x0) => ({ x: [x0, x0 + 0.002], shape: box(-0.005, 0.005, -0.005, 0.005) });
  const r = oneStep(m, wall(0.05), [1, 0, 0]).steps[1];
  // (the overlap reported is the smaller penetration: its section 7 mm into the wall's, the swept x range 52 mm)
  assert.equal(r.failed, true); assert.equal(r.through[0].at_frac, undefined); near(r.through[0].by_mm, 7, 1e-9);
  const back = oneStep(m, wall(-0.05), [-1, 0, 0]).steps[1];
  assert.equal(back.failed, true);
  // a wall 0.8 mm beyond its start, or 0.8 mm beyond its place on the far side: never met, the gap measured
  near(oneStep(m, wall(0.1108), [1, 0, 0]).steps[1].worst_gap_mm, 0.8, 1e-9);
  near(oneStep(m, wall(0.0108), [-1, 0, 0]).steps[1].worst_gap_mm, 0.8, 1e-9);
  near(oneStep(m, wall(-0.0028), [1, 0, 0]).steps[1].worst_gap_mm, 0.8, 1e-9);
  // (the wall over its whole place: the overlap there is as deep as anywhere on the way)
  const home = oneStep(m, { x: [0, 0.01], shape: box(-0.005, 0.005, -0.005, 0.005) }, [1, 0, 0]).steps[1];
  assert.equal(home.through[0].at_frac, 0);
  // the row: its direction rounded to 4 places, its distance in mm, its parts and solids
  const o = oneStep(m, wall(0.12), [1, 1e-13, 0], 0.075).steps[1];
  assert.deepEqual([o.dir, o.dist_mm, o.parts, o.solids], [[1, 0, 0], 75, ['mover'], 1]);
});

test('the sequence\'s pose: stated, or the crank at phase 0 at the nominal amplitude', () => {
  const m = { x: [0, 0.01], shape: ring([0, 0], 0.002) }, ob = { x: [0.2, 0.21], shape: box(-0.005, 0.005, -0.005, 0.005) };
  assert.deepEqual(oneStep(m, ob, [1, 0, 0]).pose, { phase_rad: 0, amp_rad: 0.5 });
  assert.deepEqual(oneStep(m, ob, [1, 0, 0], 0.1, { phase_rad: 0.3, amp: 0.4, why: 'held' }).pose, { phase_rad: 0.3, amp_rad: 0.4, why: 'held' });
  assert.deepEqual(oneStep(m, ob, [1, 0, 0], 0.1, { phase_rad: 0.3 }).pose, { phase_rad: 0.3, amp_rad: 0.5 });
});

// ---- the hole check on every kind of part (round 16's surviving mutants: holds 3/14, shankIn 28/40) -------------------

test('holeRecord: a shank\'s whole circle in a plate, a ring between its bore and rim, or a capsule; a vertical screw\'s square in its sheet', () => {
  const g = { d: 0.003, L: 0.008, head_d: 0.0055, head_k: 0.003 };
  const plate = poly([[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]]);
  const one = (shape, c, x = [0, 0.002]) => {
    const parts = [];
    addPart(parts, 'part', [S('body', x, shape)]);
    addPart(parts, 'screw', screwX(c, -0.001, 1, g).screw);
    return holeRecord({ parts, joins: [['screw/shank', 'part', THROUGH]] }).passed;
  };
  // a plate: in until its 1.5 mm radius reaches the 10 mm edge (8.5 mm), not past
  assert.deepEqual([one(plate, [0, 0]), one(plate, [0.0085, 0]), one(plate, [0.0086, 0]), one(plate, [0, -0.0086])], [true, true, false, false]);
  // a ring 4 to 10 mm: in between 5.5 and 8.5 mm from its centre, out in its bore or past its rim
  const rg = ring([0, 0], 0.01, 0.004);
  assert.deepEqual([one(rg, [0.007, 0]), one(rg, [0.0055, 0]), one(rg, [0.0085, 0]), one(rg, [0.005, 0]), one(rg, [0, 0.009])], [true, true, true, false, false]);
  // a capsule of radius 4 mm: in within 2.5 mm of its spine, round its ends too
  const cp = cap([-0.01, 0], [0.01, 0], 0.004);
  assert.deepEqual([one(cp, [0.005, 0.0025]), one(cp, [0.005, 0.0026]), one(cp, [0.012, 0]), one(cp, [0.0126, 0])], [true, false, true, false]);
  // the part must span the shank along x: a plate wholly forward of the shank's 7 mm holds nothing
  assert.deepEqual([one(plate, [0, 0], [0.006, 0.008]), one(plate, [0, 0], [0.008, 0.01])], [true, false]);
  // a vertical screw (screwY: c = [x, z]) through a 1 mm sheet at y 8-9 mm: its 3 mm square across x and z in the
  // sheet's x range and section
  const vert = (sheetX, sheet) => {
    const parts = [];
    addPart(parts, 'sheet', [S('sheet', sheetX, sheet)]);
    addPart(parts, 'screw', screwY([0.001, 0], 0.012, -1, g).screw);
    return holeRecord({ parts, joins: [['screw/shank', 'sheet', THROUGH]] }).passed;
  };
  const sheet = (z0, z1, y0 = 0.008, y1 = 0.009) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);
  assert.deepEqual([vert([-0.002, 0.004], sheet(-0.01, 0.01)), vert([0, 0.004], sheet(-0.01, 0.01)), vert([-0.002, 0.0024], sheet(-0.01, 0.01)), vert([-0.002, 0.004], sheet(-0.001, 0.01)), vert([-0.002, 0.004], sheet(-0.01, 0.0014))],
    [true, false, false, false, false]);
  // a sheet the shank never reaches (it ends 4 mm below its 12 mm face)
  assert.equal(vert([-0.002, 0.004], sheet(-0.01, 0.01, 0.0005, 0.0015)), false);
  // the records: a missing part fails, a part made only of a nut is not asked (its bore is the shank), other joins are not holes
  const parts = []; addPart(parts, 'screw', screwX([0, 0], -0.001, 1, g).screw); addPart(parts, 'n', [S('nut', [0.004, 0.006], ring([0, 0], 0.003, 0.0015))]);
  const r = holeRecord({ parts, joins: [['screw/shank', 'n', THREAD], ['screw/shank', 'ghost', THROUGH], ['screw', 'n', 'threaded on the screw'], ['screw/head', 'n', THROUGH]] });
  assert.equal(r.holes, 0); assert.deepEqual(r.failures, [{ screw: 'screw/shank', part: 'ghost', why: 'no such part' }]);
});
