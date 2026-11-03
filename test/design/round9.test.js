// Round 12: the design-file fixture is v20 (test/fixtures/larus-gl-2-v20.json, the committed HEAD design), the design these hand calculations were written for; v21 has its own tests (round12).
// Round 9 design wiring: the gear train re-rated under the design's conditions (src/design/gearing.js), by hand.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { supportClass, stageSpeeds, rateStages } from '../../src/design/gearing.js';
import { ROUND9, round9Params, round9Assumptions } from '../../src/design/round9.mjs';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('support class from the gear plane between the plates, and stage speeds from the crank', () => {
  assert.equal(supportClass(0.5, 0, 1), 'equidistant');
  assert.equal(supportClass(0.54, 0, 1), 'equidistant');
  assert.equal(supportClass(0.2, 0, 1), 'weak');
  assert.equal(supportClass(0.0125, 0.0055, 0.0705), 'weak');
  assert.equal(supportClass(-0.01, 0, 1), 'overhung');
  // Ratios 2, 3 from the motor to the crank at 60 rpm: stage 2 wheel 60, pinion 180; stage 1 wheel 180, pinion 360.
  const sp = stageSpeeds([{ ratio: 2 }, { ratio: 3 }], 60);
  near(sp[1].wheel_rpm, 60, 0); near(sp[1].pinion_rpm, 180, 1e-12);
  near(sp[0].wheel_rpm, 180, 1e-12); near(sp[0].pinion_rpm, 360, 1e-12);
});

test('rateStages: a steel stage and a Duracon stage re-rated at the stated crank rate, by hand', () => {
  // Test articles (round numbers, not catalogue parts).
  const cat = [
    { id: 'p18', specs: { module_mm: 1, teeth: 18, face_width_mm: 10, material: 'SCM415', heat_treatment: 'Carburized', allowable_torque_bending_nm: 12, allowable_torque_surface_durability_nm: 6 } },
    { id: 'g36', specs: { module_mm: 1, teeth: 36, face_width_mm: 10, material: 'SCM415', heat_treatment: 'Carburized', allowable_torque_bending_nm: 30, allowable_torque_surface_durability_nm: 24 } },
    { id: 'd20', specs: { module_mm: 1, teeth: 20, face_width_mm: 6, material: 'Duracon (R) acetal M90-44', allowable_torque_bending_nm: 1 } },
    { id: 'd40', specs: { module_mm: 1, teeth: 40, face_width_mm: 6, material: 'Duracon (R) acetal M90-44', allowable_torque_bending_nm: 2 } },
  ];
  const stages = [{ pinion: 'd20', gear: 'd40', ratio: 2, pinion_allowable_Nm: 1, gear_allowable_Nm: 2 }, { pinion: 'p18', gear: 'g36', ratio: 2, pinion_allowable_Nm: 6, gear_allowable_Nm: 24 }];
  const layout = { planes_m: { plateAft: 0, plateFwd: 0.1, stage: [0.05, 0.02] } };
  const gr = { crank_Hz: 1, reversing: true, metal_lubricant: { cSt_50C: 150 }, plastic_lubricant: { grease: true }, max_temperature_C: 40 };
  const r = rateStages(stages, cat, layout, gr);
  // Steel stage at the crank: pinion at 120 rpm, 0.1131 m/s; support weak (plane at 20% of the span).
  assert.equal(r[1].rating_check.support, 'weak');
  near(r[1].rating_check.pinion_rpm, 120, 1e-12);
  const kh = (bd, a, b) => a + ((bd - 0.4) / 0.2) * (b - a);
  // Pinion surface: 6 x (2/3 / 0.5) x KH-beta equidistant(0.5556) / weak(0.5556); ZV both on the first value.
  near(r[1].pinion_allowable_Nm, 6 * ((2 / 3) / 0.5) * (kh(10 / 18, 1, 1.05) / kh(10 / 18, 1.3, 1.5)), 1e-9);
  // Wheel surface: 24 x (18 x 2/3) / (36 x 0.5) x 1.0 / weak(0.5556) x ZV(0.1131 m/s) / ZV(rated 600 rpm on 36 teeth, 1.131 m/s), squared.
  const zvR = 0.955 + ((Math.PI * 0.036 * 10 - 1) / 2) * 0.01;
  near(r[1].gear_allowable_Nm, 24 * ((18 * (2 / 3)) / 18) * (1 / kh(10 / 18, 1.3, 1.5)) * (0.955 / zvR) ** 2, 1e-9);
  // Duracon stage at mid-span: wheel at 120 rpm, pinion at 240 rpm (0.2513 m/s), Duracon on Duracon, reversing.
  assert.equal(r[0].rating_check.support, 'equidistant');
  const v = Math.PI * 0.02 * 240 / 60;
  near(r[0].pinion_allowable_Nm, 1 * ((1.4 - 0.016 * v) / (1.4 - 0.016 * (Math.PI * 0.02 * 100 / 60))) * 0.75 * (2 / 3), 1e-9);
  near(r[0].gear_allowable_Nm, 2 * ((1.4 - 0.016 * v) / (1.4 - 0.016 * (Math.PI * 0.04 * 100 / 60))) * 0.75 * (2 / 3), 1e-9);
  // The catalogue values are kept beside the corrected ones.
  assert.equal(r[1].catalogue_pinion_allowable_Nm, 6);
  assert.equal(r[0].catalogue_gear_allowable_Nm, 2);
});

test('rateStages edges: the plate ends, no layout, unequal faces, no lubricant stated', () => {
  assert.equal(supportClass(0, 0, 1), 'weak');
  assert.equal(supportClass(1, 0, 1), 'weak');
  assert.equal(supportClass(0.549, 0, 1), 'equidistant');
  assert.equal(supportClass(0.5501, 0, 1), 'weak');
  assert.equal(supportClass(0.5, 1, 0), 'equidistant');
  assert.equal(supportClass(1.01, 0, 1), 'overhung');
  const cat = [
    { id: 'p', specs: { module_mm: 1, teeth: 20, face_width_mm: 8, material: 'S45C', heat_treatment: 'Gear teeth induction hardened', allowable_torque_bending_nm: 10, allowable_torque_surface_durability_nm: 4 } },
    { id: 'g', specs: { module_mm: 1, teeth: 40, face_width_mm: 10, allowable_torque_bending_nm: 20, allowable_torque_surface_durability_nm: 16 } },
  ];
  const gr = { crank_Hz: 1, reversing: false, metal_lubricant: { cSt_50C: 100 }, plastic_lubricant: { grease: true }, max_temperature_C: 40 };
  const r = rateStages([{ pinion: 'p', gear: 'g', ratio: 2, pinion_allowable_Nm: 4, gear_allowable_Nm: 16 }], cat, null, gr);
  // No layout: the weak-shaft column; the narrower face (8 mm) sets b/d01 = 0.4: weak 1.3 against equidistant 1.0 rated.
  assert.equal(r[0].rating_check.support, 'weak');
  assert.equal(r[0].rating_check.plane_x_m, null);
  near(r[0].pinion_allowable_Nm, 4 * ((2 / 3) / 0.5) * (1 / 1.3), 1e-9);
  // A gear without a stated material is metal (JGMA), and no lubricant stated halves it.
  const nl = rateStages([{ pinion: 'p', gear: 'g', ratio: 2 }], cat, null, { ...gr, metal_lubricant: null });
  assert.equal(nl[0].rating_check.gear.method, 'JGMA 401-01 / 402-01 (KHK)');
  assert.equal(nl[0].rating_check.pinion.half_rule, true);
  // No grease named for a Duracon gear: half.
  const dc = [{ id: 'dp', specs: { module_mm: 1, teeth: 20, face_width_mm: 6, material: 'Duracon (R) M90-44', allowable_torque_bending_nm: 1 } }, { id: 'dg', specs: { module_mm: 1, teeth: 40, face_width_mm: 6, material: 'Duracon (R) M90-44', allowable_torque_bending_nm: 2 } }];
  const ng = rateStages([{ pinion: 'dp', gear: 'dg', ratio: 2 }], dc, null, { ...gr, plastic_lubricant: null });
  assert.equal(ng[0].rating_check.pinion.half_rule, true);
});

test('round-9 parameters: v18 with ROUND9 merged into the linkage blocks it names, then --set wins', () => {
  const ROOT = new URL('../../', import.meta.url).pathname;
  const catalog = JSON.parse(readFileSync(`${ROOT}data/parts/catalog.json`, 'utf8'));
  const base = JSON.parse(readFileSync(`${ROOT}out/sweep/round9/v18-base.json`, 'utf8'));
  const p = round9Params(base, {}, catalog);
  // Every ring at e >= 1.2 d0: 14.5 / 12, 19.5 / 16, 24.5 / 20 (D / 2 d0).
  assert.equal(p.linkage.rocker.hub_od_mm, 14.5);
  assert.ok(p.linkage.rocker.hub_od_mm / 2 / 6 >= 1.2 && p.linkage.rocker.boss_od_mm / 2 / 8 >= 1.2 && p.linkage.coupler.eye_od_mm / 2 / 10 >= 1.2 && p.linkage.shoulderHousing.od_mm / 2 / 8 >= 1.2);
  assert.equal(p.linkage.rocker.material, base.params.linkage.rocker.material, 'the rest of a block is kept');
  assert.equal(p.linkage.airframe.rail.id_mm, 9.6);
  assert.equal(p.linkage.airframe.rail.seat_D_mm, 29); // 29 / 2 / 12 = 1.208 d0 (od 12 from airframe.js's defaults)
  assert.equal(p.linkage.plates.edge_mm, 9.2);
  assert.deepEqual(p.parts.gears, ROUND9.parts.gears);
  assert.deepEqual(p.gearRating, ROUND9.gearRating);
  // The train's ratio from the gears: 30/20 x 60/18 x (45/18)^2 = 31.25.
  near(p.gearRatio, 1.5 * (60 / 18) * 2.5 * 2.5, 1e-9);
  // --set wins over ROUND9 in the blocks it names.
  const q = round9Params(base, { linkage: { rocker: { hub_od_mm: 16 } }, parts: { gears: base.params.parts.gears } }, catalog);
  assert.equal(q.linkage.rocker.hub_od_mm, 16);
  assert.equal(q.linkage.rocker.boss_od_mm, 19.5);
  assert.deepEqual(q.parts.gears, base.params.parts.gears);
  const d = JSON.parse(readFileSync(`${ROOT}test/fixtures/larus-gl-2-v20.json`, 'utf8'));
  const a = round9Assumptions(base, { ...d, drive: { ...d.drive, gear_rating_conditions: { crank_Hz: 3 } }, launch: { release_speed_m_s: 8 } });
  assert.ok(a.some((x) => /^Round 9: hand release at 8 m\/s/.test(x)));
  assert.ok(a.some((x) => /Rating crank frequency 3 Hz/.test(x)));
});
