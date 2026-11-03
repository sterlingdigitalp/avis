// Round 12: the design-file fixture is v20 (test/fixtures/larus-gl-2-v20.json, the committed HEAD design), the design these hand calculations were written for; v21 has its own tests (round12).
// Round 11: the candidate span bands and the housing length each joint support is judged on.
import test from 'node:test';
import assert from 'node:assert/strict';
import { checkSpan, round11Params, finishDesign, fittedBuild, ROUND11_OPT } from '../../src/design/round11.mjs';
import { readFileSync } from 'node:fs';
import { plateSeatLength, jointHousings } from '../../src/design/linkage.js';
import { buildDesign } from '../../src/design/build.js';

test('spans are held to the SPEC bands: Larus 1.5-2.0, Falco 1.0-1.2, Columba 0.7-0.9 m', () => {
  assert.equal(checkSpan('falco', 1.2), 1.2);
  assert.equal(checkSpan('falco', 1.0), 1.0);
  assert.equal(checkSpan('columba', 0.7), 0.7);
  assert.equal(checkSpan('larus', 2.0), 2.0);
  assert.throws(() => checkSpan('falco', 1.21), /outside the SPEC band 1-1.2/);
  assert.throws(() => checkSpan('falco', 0.99), /outside the SPEC band/);
  assert.throws(() => checkSpan('columba', 0.95), /outside the SPEC band 0.7-0.9/);
  assert.throws(() => checkSpan('larus', 1.4), /outside the SPEC band/);
  assert.throws(() => checkSpan('aquila', 1), /unknown candidate/);
  // round11Params refuses a span outside the band before any build.
  assert.throws(() => round11Params({ params: { span_m: 2 } }, 'columba', { span_m: 1.0 }, []), /outside the SPEC band/);
});

test('plate seats hold a part over the plate thickness, or its own length where bossed', () => {
  const plates = { t_mm: 1.6, t_fwd_mm: 1.2, bossed: ['crank shaft/aft', 'gear shaft 1/forward'] };
  assert.deepEqual(plateSeatLength(plates, 'crank shaft', 'aft', 10), { plate_mm: 1.6, bossed: true, length_mm: 10 });
  assert.deepEqual(plateSeatLength(plates, 'crank shaft', 'forward', 10), { plate_mm: 1.2, bossed: false, length_mm: 1.2 });
  assert.deepEqual(plateSeatLength(plates, 'gear shaft 1', 'forward', 6), { plate_mm: 1.2, bossed: true, length_mm: 6 });
  assert.deepEqual(plateSeatLength(plates, 'gear shaft 2', 'aft', 6), { plate_mm: 1.6, bossed: false, length_mm: 1.6 });
  // A boss never holds less than the plate it rises from; one plate thickness when t_fwd_mm is absent.
  assert.equal(plateSeatLength({ t_mm: 2, bossed: ['gear shaft 1/aft'] }, 'gear shaft 1', 'aft', 1.5).length_mm, 2);
  assert.equal(plateSeatLength({ t_mm: 2 }, 'gear shaft 1', 'forward', 6).length_mm, 2);
});

test('jointHousings: plate joints get their seat length, rings their part length, and the input is not changed', () => {
  const catalog = [
    { id: 'bush-8x6', category: 'bushing', specs: { d1_mm: 8, d2_mm: 10, b1_mm: 6 } },
    { id: 'brg-6x13x5', category: 'bearing', specs: { dimensions_mm_d_D_B: '6x13x5' } },
  ];
  const joints = [
    { name: 'gear shaft 1', part_id: 'bush-8x6', supports: [{ name: 'aft' }, { name: 'forward' }] },
    { name: 'gear shaft 3', part_id: 'brg-6x13x5', supports: [{ name: 'aft' }] },
    { name: 'shoulder pivot right', part_id: 'bush-8x6', supports: [{ name: 'aft' }] },
    { name: 'coupler eye left, crank pin', part_id: 'bush-8x6', supports: [{ name: 'eye' }] },
    { name: 'motor shaft outboard', part_id: 'brg-6x13x5', supports: [{ name: 'plate' }] },
  ];
  const L = { plates: { t_mm: 1.6, t_fwd_mm: 1.2, bossed: ['gear shaft 1/aft'] } };
  const out = jointHousings(joints, L, catalog);
  assert.equal(out[0].supports[0].housing_mm, 6); // bossed: the boss is as long as the 6 mm bushing
  assert.equal(out[0].supports[1].housing_mm, 1.2); // forward plate alone
  assert.equal(out[1].supports[0].housing_mm, 1.6); // a 5 mm bearing in the 1.6 mm aft plate
  assert.equal(out[2].supports[0].housing_mm, 6);
  assert.equal(out[3].supports[0].housing_mm, 6);
  assert.equal(out[4].supports[0].housing_mm, 1.2); // the forward plate holds the outboard bearing
  assert.equal(joints[0].supports[0].housing_mm, undefined);
});

test('jointHousings: thin parts, parts without dimensions, and plate joints without plates', () => {
  const catalog = [
    { id: 'bush-thin', category: 'bushing', specs: { d1_mm: 4, d2_mm: 5, b1_mm: 0.8 } },
    { id: 'no-dims', category: 'bushing', specs: {} },
  ];
  const plates = { t_mm: 1.6, t_fwd_mm: 1.2 };
  const out = jointHousings([
    { name: 'gear shaft 2', part_id: 'bush-thin', supports: [{ name: 'forward' }] },
    { name: 'gear shaft 2', part_id: 'no-dims', supports: [{ name: 'forward' }] },
  ], { plates }, catalog);
  assert.equal(out[0].supports[0].housing_mm, 0.8); // shorter than the 1.2 mm plate: supported over its own length
  assert.equal(out[1].supports[0].housing_mm, undefined);
  const none = jointHousings([{ name: 'crank shaft', part_id: 'bush-thin', supports: [{ name: 'aft' }] }, { name: 'coupler eye right, crank pin', part_id: 'bush-thin', supports: [{ name: 'eye' }] }], undefined, catalog);
  assert.equal(none[0].supports[0].housing_mm, undefined);
  assert.equal(none[1].supports[0].housing_mm, 0.8);
  assert.deepEqual(jointHousings(undefined, { plates }, catalog), []);
});

test('a built Larus v20: bossed seats as long as the igus GFM-1012-10 (b1 10 mm), unbossed ones the plate', () => {
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const v20 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v20.json', import.meta.url), 'utf8'));
  const d = buildDesign(v20.params, catalog);
  const seat = (n) => d.structure.members.find((m) => m.name === n);
  const sup = (j, s) => d.structure.joints.find((q) => q.name === j).supports.find((q) => q.name === s);
  // igus publishes GFM-1012-10 as 10 mm long and GFM-0810-06 as 6 mm (catalogue b1); v20's plates are 1.6 / 1.2 mm.
  assert.equal(seat('plate seat crank shaft aft').dims.t_mm, 10);
  assert.equal(sup('crank shaft', 'aft').housing_mm, 10);
  assert.equal(seat('plate seat gear shaft 2 forward').dims.t_mm, 1.2);
  assert.equal(sup('gear shaft 2', 'forward').housing_mm, 1.2);
  assert.equal(sup('gear shaft 2', 'aft').housing_mm, 1.6);
  assert.equal(sup('shoulder pivot right', 'aft').housing_mm, 6);
});

test('finishDesign: the named 8 m/s release, the round-11 selection and both round-11 assumptions', () => {
  const v20 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v20.json', import.meta.url), 'utf8'));
  const d = finishDesign(structuredClone(v20), { cand: 'larus', base: v20, set: { launch: { release_speed_m_s: 7.5 } }, opt: ROUND11_OPT });
  assert.equal(d.launch.release_speed_m_s, 7.5);
  assert.equal(finishDesign(structuredClone(v20), { cand: 'larus', base: v20, opt: ROUND11_OPT }).launch.release_speed_m_s, 8);
  assert.equal(d.selection.pipeline, 'src/design/round11.mjs');
  assert.equal(d.selection.candidate, 'larus');
  assert.equal(d.control.launchClimb_m_s, ROUND11_OPT.launchClimb);
  assert.equal(d.round8_options.dt, 1 / 3840);
  assert.equal(d.assumptions.filter((a) => /^Round 11/.test(a)).length, 2);
});

test('fittedBuild without the body fit: the v20 parameters rebuild to v20\'s BOM, with its own calibration record', () => {
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const v20 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v20.json', import.meta.url), 'utf8'));
  const { p, d } = fittedBuild(v20.params, catalog, false);
  assert.equal(p, v20.params);
  assert.equal(d.name, 'larus-gl-2');
  assert.ok(Math.abs(d.mass.total_g - v20.mass.total_g) < 1e-6, `${d.mass.total_g} vs ${v20.mass.total_g}`);
  assert.equal(d.calibration.file, 'data/calibration/larus-gl-2.json');
  assert.equal(d.packing, undefined);
  assert.equal(fittedBuild(v20.params, catalog, false, '/nonexistent/calibration.json').d.calibration, undefined);
});
