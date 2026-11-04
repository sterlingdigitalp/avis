// Round 19 (decision 5, "Massprops shared part id"): when two BOM rows share a catalogue id (the KBRM-02 rod ends and the
// M2 x 6 screws of both the tail and the twist drive), each row's mass must sit at its own drawn pieces. Before round 19
// a lump naming the id bare took every piece of the id (the twist drive's on the wing too) into its centroid with no
// change in its mass, and a lump naming its own pieces ('id@<part regex>') took the line's mass by volume, so two
// drawings of one part (a cut-down servo arm) put the wrong mass on each. The hand-built cases below have centroids
// computed by hand from the solids' own geometry (rings along x: centroid at mid x, at the ring's centre).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as MP from '../../src/design/massprops.js';
import { bomLines } from '../../src/design/completeness.js';
import { buildDesign } from '../../src/design/build.js';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { fullAssembly } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const { massPropsRecord } = MP, MASS_TOL_G = MP.MASS_TOL_G ?? 0.01; // (?? only so the pre-round-19 code runs each test)
const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));

/** A part of one ring solid along x: x range [x0, x0 + L], centre (y, z), outer radius ro (m). */
const ringPart = (id, bom, x0, L, y, z, ro) => ({ id, solids: [{ name: 's', x: [x0, x0 + L], shape: { kind: 'ring', c: [z, y], ro, ri: 0 }, bom }] });

// Two lumps, each a 6 g block plus one piece of the shared part 'part-x' (2 g each: two BOM rows of one piece, in two
// roles). The tail's piece is drawn smaller than the twist's (radius 2 mm against 3 mm), as the tail's servo arm is.
const asm = { parts: [
  ringPart('tail block', 'block a', 0, 0.01, 0, 0, 0.005), // centroid (0.005, 0, 0)
  ringPart('tail rod end', 'part-x', 0.1, 0.01, 0, 0, 0.002), // centroid (0.105, 0, 0), V = pi 4e-6 0.01
  ringPart('twist block', 'block b', 0.5, 0.01, 0.2, 0.3, 0.005), // centroid (0.505, 0.2, 0.3)
  ringPart('twist rod end', 'part-x', 0.6, 0.01, 0.2, 0.3, 0.003), // centroid (0.605, 0.2, 0.3), V = pi 9e-6 0.01
], joins: [] };
const bom = { parts: [{ id: 'part-x', role: 'tail rod ends', count: 1, mass_g: 2 }, { id: 'part-x', role: 'twist rod ends', count: 1, mass_g: 2 }], structure: [{ name: 'block a', material_id: 'm-a', mass_g: 6 }, { name: 'block b', material_id: 'm-b', mass_g: 6 }] };
// by hand: the tail lump 6 g at x 0.005 and 2 g at x 0.105 -> x = (0.030 + 0.210) / 8 = 0.030; the twist lump 6 g at
// (0.505, 0.2, 0.3) and 2 g at (0.605, 0.2, 0.3) -> x = (3.030 + 1.210) / 8 = 0.530, y 0.2, z 0.3
const TAIL_C = [0.030, 0, 0], TWIST_C = [0.530, 0.2, 0.3];
const design = (lumps) => ({ bom, mass: { margin_frac: 0, lumps }, body: {}, tail: null });
const lump = (name, lines, pos) => ({ name, mass_kg: 0.008, pos, lines });
const near = (a, b, tol = 1e-9) => a.every((v, i) => Math.abs(v - b[i]) <= tol);

test('massprops: two lumps naming their own pieces of a shared part id sit at the hand-computed centroids, whatever each piece\'s drawn size', () => {
  const rec = massPropsRecord(asm, design([lump('tail', ['block a', 'part-x@^tail '], TAIL_C), lump('twist', ['block b', 'part-x@^twist '], TWIST_C)]), catalog, null);
  const [t, w] = rec.rows;
  // before round 19 the tail took the line's 4 g (both rows) by volume, 4 x 4/13 = 1.23 g for its 2 g piece: x 0.0220
  assert.ok(near(t.centroid_m, TAIL_C), `the tail lump at its hand-computed centroid ${TAIL_C}, drawn ${t.centroid_m}`);
  assert.ok(near(w.centroid_m, TWIST_C), `the twist lump at its hand-computed centroid ${TWIST_C}, drawn ${w.centroid_m}`);
  assert.equal(t.drawn_mass_g, 8);
  assert.equal(w.drawn_mass_g, 8);
  assert.equal(rec.passed, true);
});

test('massprops: a lump naming a part id shared by BOM rows of two roles without a part filter refuses, naming the id and its rows', () => {
  // as the build sets it: the lump at whatever its lines' solids give (solidMassParams), so only the mass check can see it
  const first = massPropsRecord(asm, design([lump('tail', ['block a', 'part-x'], [0, 0, 0])]), catalog, null).rows[0];
  assert.ok(!near(first.centroid_m, TAIL_C, 1e-3), 'the bare id drags the tail lump toward the twist drive\'s piece');
  const row = massPropsRecord(asm, design([lump('tail', ['block a', 'part-x'], first.centroid_m)]), catalog, null).rows[0];
  assert.equal(row.err_mm, 0, 'at its own drawn centroid the position check alone passes it (the round-18 finding)');
  assert.equal(row.ok, false, 'its lines weigh 10 g on their solids, the lump 8 g');
  assert.match(row.mass_mismatch, /10\.000 g .* 8\.000 g/);
  assert.match(row.mass_mismatch, /part-x \(BOM rows: tail rod ends; twist rod ends\)/);
});

test('massprops: the margin is applied to the drawn mass as the build applies it to the lump, and a mismatch beyond rounding fails', () => {
  const d = (kg) => ({ ...design([{ name: 'tail', mass_kg: kg, pos: TAIL_C, lines: ['block a', 'part-x@^tail '] }]), mass: { margin_frac: 0.15, lumps: [{ name: 'tail', mass_kg: kg, pos: TAIL_C, lines: ['block a', 'part-x@^tail '] }] } });
  assert.equal(massPropsRecord(asm, d(0.008 * 1.15), catalog, null).rows[0].ok, true);
  assert.equal(massPropsRecord(asm, d(0.008 * 1.15 + (2 * MASS_TOL_G) / 1000), catalog, null).rows[0].ok, false);
});

test('massprops on the design: every lump\'s lines weigh its own mass, and no lump names a part id of several BOM roles bare', () => {
  const v24 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
  const set = round17Set({}, catalog);
  const p = mergeDeep(v24.params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  const d = buildDesign({ ...p, _massPass: true }, catalog), { asm: A, lay } = fullAssembly(d, catalog);
  const rec = massPropsRecord(A, d, catalog, drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal));
  const shared = new Set(bomLines(d).filter((l) => new Set(l.roles ?? []).size > 1).map((l) => l.key));
  assert.ok(shared.has('rodend-igus-KBRM-02') && shared.has('screw-iso4762-m2x6-8.8'), 'the tail and the twist drive share the KBRM-02 and the M2 x 6');
  for (const l of d.mass.lumps) for (const t of l.lines) assert.ok(!shared.has(t), `${l.name} names ${t} without a part filter`);
  for (const r of rec.rows) assert.ok(Math.abs(r.drawn_mass_g - r.mass_g) <= 0.01, `${r.lump}: lines ${r.drawn_mass_g} g, lump ${r.mass_g} g`);
});
