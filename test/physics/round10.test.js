// Round 10: the formulas added for the orchestrator's round-10 rulings (JGMA's own speed factors at the actual speed,
// the RC-practice fixing rule), each pinned against a hand calculation or the printed table (never this code's output).
import test from 'node:test';
import assert from 'node:assert/strict';
import { KV_TABLE, kvRow, kvFactor, contactRatio, jgmaGearRating } from '../../src/physics/gears.js';
import { rcPracticeCheck, checkMembers } from '../../src/physics/members.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('KHK Table 10.3 (JGMA KV) as printed, row by old JIS grade', () => {
  // Rows as KHK prints them: 'unmodified 2 / modified 3' is 1.0 1.1 1.15 1.2 1.3 1.5 -.
  assert.deepEqual(KV_TABLE[2], [1.0, 1.1, 1.15, 1.2, 1.3, 1.5, null]);
  assert.deepEqual(KV_TABLE[5], [1.1, 1.4, 1.5, null, null, null, null]);
  // Unmodified grade g on row g; modified on row g - 1; finer than the table on its first row; coarser than 6: none.
  assert.equal(kvRow(1), 1);
  assert.equal(kvRow(1, true), 0);
  assert.equal(kvRow(0), 0);
  assert.equal(kvRow(0, true), 0);
  assert.equal(kvRow(6), 6);
  assert.equal(kvRow(7), null);
  assert.equal(kvRow(7, true), 6);
  assert.equal(kvRow(NaN), null);
  // Values at speeds: 0.5 m/s (1 or under), 2 m/s (1-3), 4 m/s (3-5).
  assert.equal(kvFactor(0.5, 2), 1.0);
  assert.equal(kvFactor(2, 2), 1.1);
  assert.equal(kvFactor(4, 2), 1.15);
  assert.equal(kvFactor(4, 5), 1.5);
  // A dash below a row's first value reads as 1.0 (row 1 at 0.5 m/s, row 0 at 2 m/s); above its last: none.
  assert.equal(kvFactor(0.5, 1), 1.0);
  assert.equal(kvFactor(2, 0), 1.0);
  assert.equal(kvFactor(20, 2), null);
  assert.equal(kvFactor(4, 6), null);
  assert.equal(kvFactor(30, 1), null);
  assert.equal(kvFactor(2, 9), null);
});

test('speed correction on the KV row at the actual speed, not half', () => {
  // MSGA1-18 (12.1 / 6.37 N m) on a 40-tooth wheel, b 10 mm, weak shaft, oil 127 cSt, reversing, row 2, at 1.5 m/s.
  const mesh = { m: 1, z: 18, zMate: 40, b_mm: 10, v_m_s: 1.5, support: 'weak', lubricant_cSt_50C: 127, reversing: true, hardened: true, kv_row: 2 };
  const r = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, mesh);
  assert.equal(r.half_rule, false);
  // Rated 600 rpm on 18 teeth: 0.5655 m/s, KV 1.0; actual 1.5 m/s: KV 1.1. ZV (surface hardened) 0.955 at or below
  // 1 m/s, 0.955 + 0.25 x 0.01 = 0.9575 at 1.5 m/s.
  const mate = ((40 / 18) / (40 / 18 + 1)) / 0.5;
  const kh = (1 + ((10 / 18 - 0.4) / 0.2) * 0.05) / (1.3 + ((10 / 18 - 0.4) / 0.2) * 0.2);
  near(r.surface_Nm, 6.37 * mate * kh * (1 / 1.1) * (0.9575 / 0.955) ** 2, 1e-9);
  near(r.bending_Nm, 12.1 * (contactRatio(18, 40) / contactRatio(18, 18)) / 1.1, 1e-9);
  // Row 2 prints nothing at 20 m/s: no factor, half.
  const off = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, v_m_s: 20 });
  assert.equal(off.half_rule, true);
  // A wheel rated faster than it runs takes no credit: 40 teeth at 600 rpm is 1.2566 m/s (KV 1.1 on row 2); at 0.38
  // m/s KV is 1.0, ratio capped at 1 on bending (the mating and contact-ratio factors alone remain).
  const w = jgmaGearRating({ bending_Nm: 36.5, surface_Nm: 34.6 }, { ...mesh, z: 40, zMate: 18, v_m_s: 0.38 });
  near(w.bending_Nm, 36.5 * contactRatio(40, 18) / contactRatio(40, 40), 1e-9);
  // Without a stated row the band-only rule of round 9 holds (a higher band: half).
  assert.equal(jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, kv_row: undefined }).half_rule, true);
});

test('RC-practice fixing: unrated, reported with its peak torque, bound by any published figure at full value', () => {
  const dsf = { rc: { method: 'KHK F-Loc clamp hub', published_Nm: 0.62, published_basis: 'KHK reference slipping torque' } };
  const a = rcPracticeCheck(dsf, 0.5);
  assert.equal(a.ok, true);
  assert.equal(a.fos, null);
  assert.equal(a.status, 'unrated, RC practice');
  near(a.published_ratio, 0.62 / 0.5, 1e-12);
  assert.equal(rcPracticeCheck(dsf, 0.62).ok, true);
  assert.equal(rcPracticeCheck(dsf, 0.6201).ok, false);
  const none = rcPracticeCheck({ rc: { method: 'set screw' } }, 3);
  assert.equal(none.ok, true);
  assert.equal(none.published_Nm, null);
  assert.equal(none.within_published, null);
  // In checkMembers: it never sets the minimum FoS, and a peak over the published figure fails the set.
  const m = (name, check, extra) => ({ name, check, sections: [{ name: 'T' }], uses: { T: 'T' }, ...extra });
  const res = checkMembers([m('hub', 'rc practice', dsf)], { 'hub/T': 0.7 });
  assert.equal(res.ok, false);
  assert.equal(res.fos_min, null);
  assert.equal(checkMembers([m('hub', 'rc practice', dsf)], { 'hub/T': 0.4 }).ok, true);
});

test('EN bolt bearing on an alloy the standard does not list (7075): at half, by hand', async () => {
  const { bearingAllowable } = await import('../../src/physics/members.js');
  const m7075 = { yield_MPa: 485, bearing_yield_MPa: null, ultimate_MPa: 540, bearing_standard: 'EN 1999-1-1', bearing_standard_lists_alloy: false };
  // Rocker hub ring, e/d0 = 14.5 / (2 x 6) = 1.2083: k1 = 2.8 x 1.2083 - 1.7 = 1.6833, alpha_b = 1.2083 / 3 = 0.40278.
  const e = 14.5 / 12;
  const full = (2.8 * e - 1.7) * (e / 3) * 540 / 1.25;
  near(full, 292.9, 0.1);
  near(bearingAllowable(m7075, e).MPa, full / 2, 1e-9);
  near(bearingAllowable({ ...m7075, bearing_standard_lists_alloy: undefined }, e).MPa, full, 1e-9);
  assert.equal(bearingAllowable(m7075, 1.19).MPa, null);
});

test('speed condition record: KV row, factors and the no-row band rule, by hand', async () => {
  const { speedCorrection } = await import('../../src/physics/gears.js');
  const vR = Math.PI * 0.018 * 10; // 18 teeth at 600 rpm: 0.5655 m/s
  const mesh = { m: 1, z: 18, v_m_s: 1.5, hardened: true, kv_row: 2 };
  const a = speedCorrection(mesh, vR);
  assert.equal(a.half, false);
  assert.deepEqual(a.condition.kv, { rated: 1.0, actual: 1.1 });
  near(a.bending, 1 / 1.1, 1e-12);
  near(a.surface, (1 / 1.1) * (0.9575 / 0.955) ** 2, 1e-12);
  near(a.condition.surface_factor, a.surface, 0);
  assert.equal(a.condition.holds, false);
  assert.equal(speedCorrection({ ...mesh, v_m_s: vR }, vR).condition.holds, true);
  // Same band, no row: no KV credit, ZV only; a higher band without a row: half; a row with no value: half.
  const same = speedCorrection({ ...mesh, kv_row: undefined, v_m_s: 0.9 }, vR);
  assert.equal(same.bending, 1);
  assert.equal(same.condition.bending_factor, 1);
  near(same.surface, 1, 1e-12);
  assert.equal(speedCorrection({ ...mesh, kv_row: undefined }, vR).half, true);
  const none = speedCorrection({ ...mesh, v_m_s: 20 }, vR);
  assert.equal(none.half, true);
  assert.equal(none.condition.bending_factor, null);
  // Soft (thermal refined) gears read the lower ZV curve: 0.90 at 1 m/s and below, 0.91 at 2 m/s.
  near(speedCorrection({ ...mesh, v_m_s: 2, hardened: false }, vR).surface, (1 / 1.1) * (0.91 / 0.9) ** 2, 1e-12);
});

test('RC-practice record fields: method, published basis, ratio at zero torque', () => {
  const c = rcPracticeCheck({ rc: { method: 'm', published_Nm: 1, published_basis: 'b' } }, 0);
  assert.equal(c.published_ratio, Infinity);
  assert.equal(c.method, 'm');
  assert.equal(c.published_basis, 'b');
  assert.match(c.why, /against the published 1 N m/);
  const n = rcPracticeCheck({}, 1);
  assert.equal(n.method, null);
  assert.equal(n.published_basis, null);
  assert.equal(n.published_ratio, null);
  assert.match(n.why, /no published figure/);
});
