// Round 16 (SPEC "Completeness and assembly order"): the official v24 design file. Its completeness and assembly
// records pass, recomputed from its own parameters; a deliberately incomplete BOM (a line's solids dropped, a phantom
// line) and deliberately impossible orders (the forward plate before the gear shafts, the v23 one-piece crank) are
// refused. Kept apart from round16.test.js: it reads the design on disk, which the calibration loop rewrites.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { completenessRecord } from '../../src/design/completeness.js';
import { sequenceRecord } from '../../src/design/sequence.js';
import { officialRefusal } from '../../src/design/round16.mjs';
import { fullAssembly } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const official = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const pose0 = (lay, d) => drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal);

test('v24: every physical BOM line has its solids, counts and sizes; dropping a line\'s solids or adding a phantom line is refused', () => {
  const d = official();
  assert.ok(d.version >= 24);
  const c = d.structure.clearance;
  assert.ok(c.completeness.passed && c.assembly.passed && c.passed);
  assert.equal(officialRefusal(c), null);
  const { asm, lay } = fullAssembly(d, catalog), q0 = pose0(lay, d);
  // the same record from the design file's own parameters
  const k = completenessRecord(asm, d, catalog, q0);
  assert.ok(k.passed, JSON.stringify(k.failed));
  // every M3 nut's solids gone: the line fails, and the build would refuse
  const a2 = { ...asm, parts: asm.parts.filter((p) => !/ nut \d+$/.test(p.id) || /readhead/.test(p.id)) };
  const k2 = completenessRecord(a2, d, catalog, q0);
  assert.ok(k2.failed.some((l) => /^M3 nuts/.test(l)), JSON.stringify(k2.failed));
  assert.match(officialRefusal({ ...c, completeness: k2 }), /M3 nuts/);
  // a BOM line no solid stands for
  const d3 = structuredClone(d); d3.bom.structure.push({ name: 'phantom bracket', material_id: 'metal-al-7075-t6', mass_g: 3 });
  assert.deepEqual(completenessRecord(asm, d3, catalog, q0).failed, ['phantom bracket']);
});

test('v24: the stated sequence is clear; the forward plate moved ahead of the gear shafts is refused', () => {
  const d = official();
  const { asm, lay } = fullAssembly(d, catalog);
  assert.ok(sequenceRecord(asm, d, lay).passed);
  // the forward plate moved ahead of the gear shafts: the shafts' gears come on from forward through it
  const st = d.params.layout.sequence.steps, fp = st.findIndex((s) => /^forward plate/.test(s.name)), g1 = st.findIndex((s) => /^gear shaft 1/.test(s.name));
  assert.ok(fp > g1 && g1 >= 0, `steps: ${st.map((s) => s.name).join(' / ')}`);
  const re = [...st.slice(0, g1), st[fp], ...st.slice(g1, fp), ...st.slice(fp + 1)];
  const r2 = sequenceRecord(asm, { ...d, params: { ...d.params, layout: { ...d.params.layout, sequence: { ...d.params.layout.sequence, steps: re } } } }, lay);
  assert.equal(r2.passed, false);
  assert.ok(r2.failures.some((n) => /^gear shaft/.test(n)), JSON.stringify(r2.failures));
});

test('v24: the wing\'s stated stations are the ones its own pod gives (the root rib clear of the loft through the stroke)', async () => {
  const { wingStations } = await import('../../src/design/round16.mjs');
  const d = official();
  assert.deepEqual(wingStations(d, catalog), d.params.wingStructure);
  assert.deepEqual(d.params.wingStructure, { rib_root_m: 0.052, spar_to_m: 0.9383, rib_t_mm: 2 });
});
