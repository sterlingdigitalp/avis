// Round 10 design wiring: shafts and keys from the train, and RC-practice fixings from the catalogue, by hand.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { keyFor, trainShafts, KEY_T1 } from '../../src/design/round10.mjs';
import { gearFixing } from '../../src/design/linkage.js';
import { kvRowOf, rateStages } from '../../src/design/gearing.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const byId = (id) => catalog.find((p) => p.id === id);
const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('keys from maker keyways (JIS B 1301 depths) and the shafts of the v20 train', () => {
  assert.deepEqual(keyFor(byId('gear-khk-msgb1-45'), 'ti'), { material: 'ti', b_mm: 5, h_mm: 5, L_mm: 10, shaft_depth_mm: 3.0 });
  assert.equal(keyFor(byId('gear-khk-msga1-18'), 'ti').shaft_depth_mm, 1.8);
  assert.equal(keyFor(byId('gear-khk-ds1-36'), 'ti'), null);
  assert.equal(keyFor(byId('gear-khk-ssy1-15h'), 'ti'), null); // "plain bore, no keyway ..." is not a keyway
  assert.equal(KEY_T1[4], 2.5);
  const t = trainShafts(['gear-khk-dsf1-24', 'gear-khk-ds1-36', 'gear-khk-msga1-20', 'gear-khk-ds1-64', 'gear-khk-msga1-18', 'gear-khk-msgb1-45', 'gear-khk-msga1-18', 'gear-khk-msgb1-45'], catalog);
  // Shaft 1: MSGB1-45 (bore 15, 5 mm key) and MSGA1-18 (bore 8, 3 mm key); shaft 2: DS1-64 (8, bonded) and MSGA1-18;
  // shaft 3: DS1-36 (6, bonded) and MSGA1-20 (8): checked at 6 mm, its mass at 8 mm.
  assert.equal(t.gearShafts.length, 3);
  assert.equal(t.gearShafts[0].d_mm, 8);
  assert.equal(t.gearShafts[0].seat_od_mm, 15);
  assert.equal(t.gearShafts[0].wheelKey.b_mm, 5);
  assert.equal(t.gearShafts[0].pinionKey.b_mm, 3);
  assert.equal(t.gearShafts[1].wheelKey, undefined);
  assert.equal(t.gearShafts[2].d_mm, 6);
  assert.equal(t.gearShafts[2].mass_d_mm, 8);
  assert.deepEqual(t.jointShafts, ['bushing-igus-GFM-0810-06', 'bushing-igus-GFM-0810-06', 'bearing-skf-628-6-2z']);
  assert.equal(t.crankSeat_mm, 15);
  assert.equal(t.motorPinion, 'gear-khk-dsf1-24');
});

test('RC-practice fixings: F-Loc within KHK slip torque; bonds within the published shear on the gear material', () => {
  const terms = [{ ch: 'crank_torque', k: 0.03 }];
  const fl = gearFixing('f', catalog, 'gear-khk-dsf1-24', {}, 5, terms);
  assert.equal(fl.check, 'rc practice');
  assert.equal(fl.rc.published_Nm, 0.62);
  // DS1-36, 6 mm bore, 6 mm face, LOCTITE 406 on SF 770, acetal 14 MPa: 14e6 x pi x 0.006 x 0.006 x 0.003 = 4.750 N m.
  const fixings = { bond: { pom: 'adhesive-loctite-406-sf770', steel: 'adhesive-loctite-638' } };
  const b = gearFixing('b', catalog, 'gear-khk-ds1-36', fixings, 6, terms);
  assert.equal(b.check, 'rc practice');
  near(b.rc.published_Nm, 14e6 * Math.PI * 0.006 * 0.006 * 0.003, 1e-9);
  near(b.rc.published_Nm, 4.750, 1e-3);
  // A plain-bore steel gear: LOCTITE 638 at 29 MPa over its face (SSY1-15H, 6 mm bore, 6 mm face).
  const s = gearFixing('s', catalog, 'gear-khk-ssy1-15h', fixings, 6, terms);
  near(s.rc.published_Nm, 29e6 * Math.PI * 0.006 * 0.006 * 0.003, 1e-9);
  // No adhesive stated: unrated (fails M7).
  assert.equal(gearFixing('u', catalog, 'gear-khk-ds1-36', {}, 6, terms).check, 'unrated');
});

test('accuracy row of a catalogue gear from its N grade', () => {
  const acc = { old_grade_of: { N5: 1, N7: 3 }, modified: false };
  assert.equal(kvRowOf(byId('gear-khk-msga1-18'), acc), 1); // "JIS grade N5": old 1, unmodified: row 1
  assert.equal(kvRowOf(byId('gear-khk-msga1-18'), { ...acc, modified: true }), 0);
  assert.equal(kvRowOf(byId('gear-khk-msga1-18'), { old_grade_of: { N7: 3 } }), null);
  assert.equal(kvRowOf({ specs: {} }, acc), null);
});

test('a maker-tapped set screw is RC practice with no published figure; "no ... set screw" text is not one', () => {
  const terms = [{ ch: 'crank_torque', k: 0.03 }];
  const f = gearFixing('t', catalog, 'gear-khk-bss1-16b', {}, 5, terms);
  assert.equal(f.check, 'rc practice');
  assert.match(f.rc.method, /set screw/);
  assert.equal(f.rc.published_Nm, null);
  assert.equal(gearFixing('p', catalog, 'gear-khk-ssy1-15h', {}, 6, terms).check, 'unrated');
});

test('fixing descriptions and the bond class of a gear material', async () => {
  const { bondClass } = await import('../../src/design/linkage.js');
  assert.equal(bondClass('Duracon (R) acetal M90-44'), 'pom');
  assert.equal(bondClass('Polyacetal (Hub: SUS303)'), 'pom');
  assert.equal(bondClass('S45C'), 'steel');
  assert.equal(bondClass('MC901 nylon'), null);
  assert.equal(bondClass(undefined), null);
  const terms = [{ ch: 'crank_torque', k: 0.03 }];
  const fl = gearFixing('f', catalog, 'gear-khk-dsf1-24', {}, 5, terms);
  assert.match(fl.rc.method, /M2\.5 at KHK's standard tightening torque 0\.45 N m/);
  assert.match(fl.rc.published_basis, /reference slipping torque 0\.62 N m/);
  const fixings = { bond: { pom: 'adhesive-loctite-406-sf770', steel: 'adhesive-loctite-638' } };
  const b = gearFixing('b', catalog, 'gear-khk-ds1-36', fixings, 6, terms);
  assert.match(b.rc.method, /on acetal/);
  assert.match(b.rc.published_basis, /^14 MPa x pi x 6 x 6\.0 mm x 6 \/ 2/);
  assert.equal(b.dims.L_mm, 6);
  assert.match(gearFixing('s', catalog, 'gear-khk-ssy1-15h', fixings, 6, terms).rc.method, /on steel/);
  // An adhesive for the other material only: unrated.
  assert.equal(gearFixing('x', catalog, 'gear-khk-ds1-36', { bond: { steel: 'adhesive-loctite-638' } }, 6, terms).check, 'unrated');
  // A set screw with a cited holding figure binds it.
  const ss = gearFixing('t', catalog, 'gear-khk-bss1-16b', { set_screw: { holding_Nm: 1.2, basis: 'x' } }, 5, terms);
  assert.equal(ss.rc.published_Nm, 1.2);
  assert.equal(ss.rc.published_basis, 'x');
});

test('isDuracon covers acetal: DSF gears take the plastic method', () => {
  const st = [{ pinion: 'gear-khk-dsf1-24', gear: 'gear-khk-ds1-36', ratio: 1.5, pinion_allowable_Nm: null, gear_allowable_Nm: null }];
  const r = rateStages(st, catalog, { planes_m: { plateAft: 0, plateFwd: 1, stage: [0.2] } }, { crank_Hz: 3, reversing: true, metal_lubricant: { cSt_50C: 127 }, plastic_lubricant: { grease: true }, max_temperature_C: 40 });
  assert.equal(r[0].rating_check.pinion.method, 'Lewis (KHK plastic gear method)');
  assert.equal(r[0].rating_check.gear.method, 'Lewis (KHK plastic gear method)');
});
