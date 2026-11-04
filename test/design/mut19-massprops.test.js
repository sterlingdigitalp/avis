// Round 19 mutation follow-up (massprops.js linePieces / addLines / lumpRow): the cases the round-19 tests left open.
// Every expected figure is hand-computed from the solids' own geometry: a solid cylinder (ring, ri 0) of radius r and
// length L has, per kg, Ixx = r^2 / 2 about its axis and Iyy = Izz = r^2 / 4 + L^2 / 12 across it; a thin cylindrical
// shell of radius R and length L (no ends) Ixx = R^2, Iyy = Izz = R^2 / 2 + L^2 / 12; a structure line on several solids
// splits by volume (pi r^2 L each).
import test from 'node:test';
import assert from 'node:assert/strict';
import { massPropsRecord } from '../../src/design/massprops.js';

const ringPart = (id, bom, x0, L, y, z, ro) => ({ id, solids: [{ name: 's', x: [x0, x0 + L], shape: { kind: 'ring', c: [z, y], ro, ri: 0 }, bom }] });
const near = (a, b, rel = 1e-9) => Math.abs(a - b) <= rel * Math.max(Math.abs(a), Math.abs(b), 1e-30);
const lump = (name, lines, pos, mass_g) => ({ name, mass_kg: mass_g / 1000, pos, lines });
const rec = (parts, bom, lumps, extra = {}) => massPropsRecord({ parts, joins: [] }, { bom, mass: { margin_frac: 0, lumps }, body: {}, tail: null, ...extra }, [], null);

test('massprops mut19: a lump of one solid cylinder has the textbook inertia per kg about its own centroid', () => {
  const r = 0.005, L = 0.01;
  const row = rec([ringPart('block', 'block a', 0, L, 0.02, 0.03, r)], { parts: [], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] }, [lump('b', ['block a'], [L / 2, 0.02, 0.03], 6)]).rows[0];
  assert.equal(row.ok, true);
  const I = row.inertia_per_kg_m2, want = [r * r / 2, r * r / 4 + L * L / 12, r * r / 4 + L * L / 12, 0, 0, 0];
  want.forEach((w, i) => assert.ok(w === 0 ? Math.abs(I[i]) < 1e-15 : near(I[i], w, 1e-9), `I[${i}] ${I[i]} against ${w}`));
});

test('massprops mut19: the pod film line on a cylindrical loft has the thin shell\'s inertia per kg', () => {
  const R = 0.02, L = 0.1, loft = { sections: [{ x_m: 0, y_m: 0, width_m: 2 * R, height_m: 2 * R }, { x_m: L, y_m: 0, width_m: 2 * R, height_m: 2 * R }] };
  const r = massPropsRecord({ parts: [], joins: [] }, { bom: { parts: [], structure: [{ name: 'fuselage film shell', material_id: 'film', mass_g: 4 }] }, mass: { margin_frac: 0, lumps: [lump('film', ['fuselage film shell'], [L / 2, 0, 0], 4)] }, body: { loft }, tail: null }, [], null).rows[0];
  assert.equal(r.ok, true, JSON.stringify(r));
  const I = r.inertia_per_kg_m2, want = [R * R, R * R / 2 + L * L / 12, R * R / 2 + L * L / 12];
  want.forEach((w, i) => assert.ok(near(I[i], w, 1e-9), `I[${i}] ${I[i]} against ${w}`));
});

test('massprops mut19: a structure line split over two lumps by a part filter puts its volume share on each', () => {
  // 'block a' (6 g) drawn on two parts: r 3 mm and r 4 mm, 10 mm long: shares 6 x 9/25 and 6 x 16/25
  const parts = [ringPart('tail block', 'block a', 0, 0.01, 0, 0, 0.003), ringPart('twist block', 'block a', 0.5, 0.01, 0, 0, 0.004)];
  const bom = { parts: [], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] };
  const [t, w] = rec(parts, bom, [lump('t', ['block a@^tail '], [0.005, 0, 0], 6 * 9 / 25), lump('w', ['block a@^twist '], [0.505, 0, 0], 6 * 16 / 25)]).rows;
  assert.ok(near(t.drawn_mass_g, 2.16, 1e-6) && near(w.drawn_mass_g, 3.84, 1e-6), `${t.drawn_mass_g} g, ${w.drawn_mass_g} g`);
  assert.equal(t.ok && w.ok, true);
});

test('massprops mut19: a bought part drawn as more items than its count is spread by volume, never weighed per item', () => {
  // one BOM row of 'part-x', count 1, 2 g, drawn on two parts: the line still weighs 2 g (completeness refuses the count)
  const parts = [ringPart('block', 'block a', 0, 0.01, 0, 0, 0.005), ringPart('rod end 1', 'part-x', 0.1, 0.01, 0, 0, 0.002), ringPart('rod end 2', 'part-x', 0.2, 0.01, 0, 0, 0.002)];
  const bom = { parts: [{ id: 'part-x', role: 'tail rod ends', count: 1, mass_g: 2 }], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] };
  const row = rec(parts, bom, [lump('t', ['block a', 'part-x'], [0, 0, 0], 8)]).rows[0];
  assert.ok(near(row.drawn_mass_g, 8, 1e-9), `drawn ${row.drawn_mass_g} g`);
});

test('massprops mut19: a bought part with a drawn item of no volume is spread by volume (no NaN)', () => {
  // count 2, two items, one a zero-length solid: by volume, all 2 g on the other
  const parts = [ringPart('rod end 1', 'part-x', 0.1, 0.01, 0, 0, 0.002), ringPart('rod end 2', 'part-x', 0.2, 0, 0, 0, 0.002)];
  const bom = { parts: [{ id: 'part-x', role: 'tail rod ends', count: 2, mass_g: 2 }], structure: [] };
  const row = rec(parts, bom, [lump('t', ['part-x'], [0.105, 0, 0], 2)]).rows[0];
  assert.ok(near(row.drawn_mass_g, 2, 1e-9), `drawn ${row.drawn_mass_g} g`);
  assert.equal(row.ok, true);
});

test('massprops mut19: a line whose solids have no volume is reported missing, not weighed', () => {
  const row = rec([ringPart('flat', 'block a', 0, 0, 0, 0, 0.005)], { parts: [], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] }, [lump('b', ['block a'], [0, 0, 0], 6)]).rows[0];
  assert.deepEqual(row.missing, ['block a: no volume']);
  assert.equal(row.ok, false);
});

test('massprops mut19: a mass mismatch on single-role lines does not blame a shared part id', () => {
  const parts = [ringPart('block', 'block a', 0, 0.01, 0, 0, 0.005), ringPart('rod end', 'part-x', 0.1, 0.01, 0, 0, 0.002)];
  const bom = { parts: [{ id: 'part-x', role: 'tail rod ends', count: 1, mass_g: 2 }], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] };
  const row = rec(parts, bom, [lump('t', ['block a', 'part-x'], [0.03, 0, 0], 9)]).rows[0];
  assert.equal(row.ok, false);
  assert.match(row.mass_mismatch, /8\.000 g .* 9\.000 g$/);
  assert.doesNotMatch(row.mass_mismatch, /several roles/);
});

test('massprops mut19: no margin stated is no margin; the record states its tolerances (1 mm, 0.01 g)', () => {
  const r = massPropsRecord({ parts: [ringPart('block', 'block a', 0, 0.01, 0, 0, 0.005)], joins: [] }, { bom: { parts: [], structure: [{ name: 'block a', material_id: 'm', mass_g: 6 }] }, mass: { lumps: [lump('b', ['block a'], [0.005, 0, 0], 6)] }, body: {}, tail: null }, [], null);
  assert.equal(r.rows[0].drawn_mass_g, 6);
  assert.equal(r.passed, true);
  assert.equal(r.tol_mm, 1);
  assert.equal(r.mass_tol_g, 0.01);
});
