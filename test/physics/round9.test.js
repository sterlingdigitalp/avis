// Round 9: the formulas added for the SPEC rulings after physics round 8, each pinned against a hand calculation or a
// published table (never against this code's own output).
import test from 'node:test';
import assert from 'node:assert/strict';
import { enBoltBearing, bearingAllowable, lugCheck, seatCheck, crushingAllowable, edgeDistances, GAMMA_M2, EDGE_MIN_OVER_D0 } from '../../src/physics/members.js';
import { contactRatio, khBeta, kvBand, zvFactor, duraconKv, pitchLineSpeed, jgmaGearRating, lewisDuraconRating } from '../../src/physics/gears.js';
import { createFlight, buildBird, quatFromEuler } from '../../src/physics/flight.js';
import { strokeAngle } from '../../src/physics/kinematics.js';
import { releaseSpeedOf, m1Passed } from '../../src/physics/scenarios.js';
import { checkMembers } from '../../src/physics/members.js';
import { testDesign } from './fixtures.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('EN bolt bearing (EN 1993-1-8 Table 3.4 / EN 1999-1-1 Table 8.5) by hand, and its 1.2 d0 floor', () => {
  assert.equal(GAMMA_M2, 1.25);
  assert.equal(EDGE_MIN_OVER_D0, 1.2);
  // e1 = e2 = 1.3 d0, fu 260: alpha_d 1.3/3 = 0.43333, k1 2.8 x 1.3 - 1.7 = 1.94: 1.94 x 0.43333 x 260 / 1.25 = 174.859 MPa.
  near(enBoltBearing({ e1_over_d0: 1.3, e2_over_d0: 1.3 }, 260).MPa, 174.8587, 1e-3);
  // 1.5 d0: k1 caps at 2.5 (2.8 x 1.5 - 1.7 = 2.5), alpha_d 0.5: 2.5 x 0.5 x 260 / 1.25 = 260.
  near(enBoltBearing({ e1_over_d0: 1.5, e2_over_d0: 1.5 }, 260).MPa, 260, 1e-9);
  // e1 2.0, e2 1.25: alpha_d 0.6667, k1 1.8: 1.8 x 0.66667 x 260 / 1.25 = 249.6.
  near(enBoltBearing({ e1_over_d0: 2, e2_over_d0: 1.25 }, 260).MPa, 249.6, 1e-9);
  // Far from the edges alpha_b is capped at 1, or at fub / fu for a weaker pin: 520 and 260.
  near(enBoltBearing({ e1_over_d0: 3.5, e2_over_d0: 3.5 }, 260, 1000).MPa, 520, 1e-9);
  near(enBoltBearing({ e1_over_d0: 3.5, e2_over_d0: 3.5 }, 260, 130).MPa, 260, 1e-9);
  assert.equal(enBoltBearing({ e1_over_d0: 2, e2_over_d0: 1.19 }, 260).MPa, null);
  assert.equal(enBoltBearing({ e1_over_d0: 1.19, e2_over_d0: 2 }, 260).MPa, null);
  assert.deepEqual(edgeDistances({ e_over_d: 1.4 }), { e1_over_d0: 1.4, e2_over_d0: 1.4 });
  assert.deepEqual(edgeDistances({ e1_over_d0: 2, e2_over_d0: 1.3 }), { e1_over_d0: 2, e2_over_d0: 1.3 });
});

test('bearing below the source edge distance: the lesser of the source at half and EN; unrated below 1.2 d0 or without EN', () => {
  // 6061-T6 as catalogued: ASM 386 MPa at e/D 2, already counted at half (193), fu 260, EN 1999-1-1.
  const al = { yield_MPa: 240, bearing_yield_MPa: 193, bearing_e_over_d_min: 2, bearing_counted_at_half: true, ultimate_MPa: 260, bearing_standard: 'EN 1999-1-1' };
  near(bearingAllowable(al, 2).MPa, 193, 1e-12);
  // e 1.9: EN 2.5 x 0.6333 x 260 / 1.25 = 329.3 > 193: the source at half governs.
  near(bearingAllowable(al, 1.9).MPa, 193, 1e-12);
  // e 1.3: EN 174.86 < 193.
  near(bearingAllowable(al, 1.3).MPa, 174.8587, 1e-3);
  assert.equal(bearingAllowable(al, 1.1).MPa, null);
  assert.equal(bearingAllowable(al, {}).MPa, null);
  // A source value counted in full is halved below its e/D: 386 -> 193.
  near(bearingAllowable({ ...al, bearing_yield_MPa: 386, bearing_counted_at_half: false }, 1.9).MPa, 193, 1e-12);
  // No bearing source (7075-T6, fu 540): EN alone, 2.5 x 0.5 x 540 / 1.25 = 540 at 1.5 d0.
  near(bearingAllowable({ yield_MPa: 485, ultimate_MPa: 540, bearing_standard: 'EN 1999-1-1' }, 1.5).MPa, 540, 1e-9);
  // Titanium: no EN standard, so below its source's e/D it is unrated; at it, the source value.
  const ti = { yield_MPa: 827, bearing_yield_MPa: 740, bearing_e_over_d_min: 2, bearing_counted_at_half: true, bearing_standard: null };
  assert.equal(bearingAllowable(ti, 1.5).MPa, null);
  near(bearingAllowable(ti, 2).MPa, 740, 1e-12);
  // The old fallback is gone: a smaller edge distance can never rate higher than a larger one.
  const es = [1.2, 1.3, 1.5, 1.7, 1.9, 2, 3];
  const v = es.map((e) => bearingAllowable(al, e).MPa);
  for (let i = 1; i < v.length; i++) assert.ok(v[i] >= v[i - 1] - 1e-12, `${es[i]}: ${v[i]} < ${v[i - 1]}`);
  // A ring lug 24.5 / 10 mm (e = 1.225 d0), 6 mm thick, 1000 N: alpha_d 0.40833, k1 1.73: 146.93 MPa; p = 16.667 MPa.
  const lg = lugCheck({ d_mm: 10, D_mm: 24.5, t_mm: 6 }, { ...al, shear_yield_MPa: 138.6 }, 1000);
  near(lg.bearing_allowable_MPa, (1.73 * (1.225 / 3) * 260) / 1.25, 1e-9);
  near(lg.fos_bearing, lg.bearing_allowable_MPa / (1000 / 60), 1e-9);
  // Crushing (keys, a tube in its socket): the yield, whatever the edge.
  near(crushingAllowable(al).MPa, 240, 0);
  near(seatCheck({ d_mm: 10, t_mm: 20, crush: true }, { yield_MPa: 133.1 }, 1000).fos, 133.1 / 5, 1e-9);
});

test('contact ratio of standard spur pairs against KHK Table 10.1', () => {
  // KHK Gear Technical Reference, Table 10.1 (alpha 20 deg, standard full depth), three decimals.
  for (const [z1, z2, e] of [[20, 20, 1.557], [20, 40, 1.635], [17, 30, 1.584], [40, 40, 1.714], [20, 65, 1.677], [17, 17, 1.514], [30, 30, 1.654]]) near(contactRatio(z1, z2), e, 1e-3); // printed to three decimals (17 x 17 reads 1.514 for 1.5148)
});

test('KH-beta table (JGMA 402-01 as KHK Table 10.11) and the speed factors by hand', () => {
  near(khBeta(0.5, 'equidistant'), 1.025, 1e-12); // KHK's own worked example (b 20, d01 40)
  near(khBeta(0.1, 'equidistant'), 1, 0);
  near(khBeta(10 / 18, 'weak'), 1.3 + ((10 / 18 - 0.4) / 0.2) * 0.2, 1e-12);
  near(khBeta(0.7, 'rugged'), 1.25, 1e-12);
  assert.equal(khBeta(1.9, 'rugged'), null);
  assert.equal(khBeta(0.5, 'nowhere'), null);
  // KV speed bands: 1 m/s or under, then 1-3, 3-5 ...
  assert.deepEqual([0.3, 1, 1.01, 3, 4, 26].map(kvBand), [0, 0, 1, 1, 2, 7]);
  // ZV (Fig. 10.5 readings): the first value below 1 m/s, linear between 1, 3 and 10 m/s.
  near(zvFactor(0.3), 0.955, 0);
  near(zvFactor(2), 0.96, 1e-12);
  near(zvFactor(2, false), 0.91, 1e-12);
  near(zvFactor(12), 1, 0);
  // Duracon KV readings: 1.40 at 0, 1.32 at 5, 1.17 at 10 m/s; null at 20 m/s and over.
  near(duraconKv(2.5), 1.36, 1e-12);
  near(duraconKv(7.5), 1.245, 1e-12);
  assert.equal(duraconKv(20), null);
  near(pitchLineSpeed(1, 18, 600), Math.PI * 0.018 * 10, 1e-12);
});

test('JGMA corrections of KHK ratings to the actual mesh, by hand', () => {
  // MSGA1-18 (12.1 / 6.37 N m) driving a 40-tooth wheel, b 10 mm, 0.38 m/s, on a weak shaft, oil 127 cSt, reversing.
  const mesh = { m: 1, z: 18, zMate: 40, b_mm: 10, v_m_s: 0.38, support: 'weak', lubricant_cSt_50C: 127, reversing: true, hardened: true };
  const r = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, mesh);
  // Mate: i 40/18; (i / (i + 1)) / 0.5 = 1.37931. KH-beta: equidistant 1.03889 rated, weak 1.45556 actual. ZV unchanged.
  near(r.surface_Nm, 6.37 * ((40 / 18) / (40 / 18 + 1) / 0.5) * ((1 + (10 / 18 - 0.4) / 0.2 * 0.05) / (1.3 + (10 / 18 - 0.4) / 0.2 * 0.2)), 1e-9);
  near(r.surface_Nm, 6.2711, 1e-3);
  near(r.bending_Nm, 12.1 * contactRatio(18, 40) / contactRatio(18, 18), 1e-12);
  assert.equal(r.half_rule, false);
  near(r.allowable_Nm, r.surface_Nm, 0);
  // The 40-tooth wheel (34.6 N m surface) against the 18-tooth pinion: d01 18 i/(i+1) over 40 x 0.5 = 0.62069; KH-beta
  // 1.0 (b/d 0.25) rated against 1.45556; ZV (0.955 / 0.95628)^2 (rated 600 rpm is 1.257 m/s on the 40-tooth gear).
  const w = jgmaGearRating({ bending_Nm: 36.5, surface_Nm: 34.6 }, { ...mesh, z: 40, zMate: 18 });
  near(w.surface_Nm, 34.6 * 0.6206897 * (1 / 1.4555556) * (0.955 / (0.955 + ((Math.PI * 0.04 * 10 - 1) / 2) * 0.01)) ** 2, 1e-4);
  near(w.surface_Nm, 14.714, 2e-3);
  // A condition with no correction counts at half: no lubricant stated; a higher KV band than rated.
  assert.equal(jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, lubricant_cSt_50C: null }).half_rule, true);
  const fast = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, v_m_s: 1.5 });
  assert.equal(fast.half_rule, true);
  near(fast.bending_Nm, 0.5 * 12.1 * contactRatio(18, 40) / contactRatio(18, 18), 1e-12);
  // Unidirectional rating with a reversing load: JGMA 401's 2/3 on bending.
  near(jgmaGearRating({ bending_Nm: 12.1 }, mesh, { bidirectional: false }).bending_Nm, (2 / 3) * 12.1 * contactRatio(18, 40) / contactRatio(18, 18), 1e-12);
});

test('Duracon Lewis rating corrected by KHK plastic gear factors, by hand', () => {
  // DS1-64 (4.07 N m) at 2.98 m/s on a Duracon mate, reversing, greased, 40 C: KV (1.4 - 0.016 v) at 2.98 over at
  // 100 rpm (0.3351 m/s): 1.35232 / 1.39464; KM 0.75; 2/3. 4.07 x 0.969657 x 0.75 x 2/3 = 1.97325.
  const r = lewisDuraconRating({ bending_Nm: 4.07 }, { m: 1, z: 64, v_m_s: 2.98, mateDuracon: true, reversing: true, grease: true, temperature_C: 40 });
  near(r.bending_Nm, 4.07 * ((1.4 - 0.016 * 2.98) / (1.4 - 0.016 * (Math.PI * 0.064 * 100 / 60))) * 0.75 * (2 / 3), 1e-9);
  near(r.bending_Nm, 1.9733, 1e-3);
  // On a metal mate, one direction: speed alone.
  const m = lewisDuraconRating({ bending_Nm: 4.07 }, { m: 1, z: 64, v_m_s: 2.98, mateDuracon: false, reversing: false, grease: true, temperature_C: 30 });
  near(m.bending_Nm, 4.07 * 0.969657, 1e-5);
  // No grease, or hotter than 40 C: half.
  assert.equal(lewisDuraconRating({ bending_Nm: 4.07 }, { m: 1, z: 64, v_m_s: 2.98, grease: false, temperature_C: 30 }).half_rule, true);
  assert.equal(lewisDuraconRating({ bending_Nm: 4.07 }, { m: 1, z: 64, v_m_s: 2.98, grease: true, temperature_C: 50 }).half_rule, true);
});

test('rocker springs: the crank sees -dV/dtheta of the spring energy, and the split adds up', () => {
  const d = testDesign();
  d.drive.rockerSpring = { k_Nm_rad: 3, phi0_rad: -0.4 };
  const bird = buildBird(d);
  const sim = createFlight(d, { bird, rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 0, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  const th = 0.8; // mid-stroke: |dphi/dtheta| is large, so the spring's crank torque is far from zero
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0, 0), vel: [3, 0, 0], omega: [0, 0, 0], phase: th, crank: 0, amp: 0.26 });
  sim.step();
  const t = sim.tel;
  // Independent: the stroke angle's derivative by finite difference of the kinematics at the recorded phase.
  const h = 1e-5;
  const dphi = (strokeAngle(t.phase + h, 0.26, bird.law).phi - strokeAngle(t.phase - h, 0.26, bird.law).phi) / (2 * h);
  const want = 2 * 3 * (t.phi - -0.4) * dphi; // two wings, one stroke law
  assert.ok(Math.abs(want) > 0.05, `spring torque ${want} too small to test`);
  near(t.crankTorqueSpring, want, 1e-4 * Math.max(1, Math.abs(want)));
  near(t.crankTorqueAero + t.crankTorqueGravity + t.crankTorqueSpring + t.crankTorqueInertia, t.crankTorque, 1e-12);
  // Its energy: 1/2 k (phi - phi0)^2 per wing.
  const e = sim.springEnergy();
  const phiNow = strokeAngle(sim.x[13], 0.26, bird.law).phi;
  near(e, 3 * (phiNow + 0.4) ** 2, 1e-9);
  // No spring: no spring torque or energy.
  const s0 = createFlight(testDesign(), { bird: buildBird(testDesign()), rho: 1e-9, gravity: 0, dt: 1 / 240 });
  assert.equal(s0.springEnergy(), 0);
  Object.assign(s0.cmd, { freq: 0, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  s0.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0, 0), vel: [3, 0, 0], omega: [0, 0, 0], phase: th, crank: 0, amp: 0.26 });
  s0.step();
  assert.equal(s0.tel.crankTorqueSpring, 0);
  // With gravity on, the split still adds up and gravity has its own share.
  const g = createFlight(d, { bird, rho: 1e-9, dt: 1 / 240 });
  Object.assign(g.cmd, { freq: 0, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  g.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0, 0), vel: [3, 0, 0], omega: [0, 0, 0], phase: th, crank: 0, amp: 0.26 });
  g.step();
  assert.ok(Math.abs(g.tel.crankTorqueGravity) > 1e-3);
  near(g.tel.crankTorqueAero + g.tel.crankTorqueGravity + g.tel.crankTorqueSpring + g.tel.crankTorqueInertia, g.tel.crankTorque, 1e-12);
  // A zero or missing rate is no spring.
  const z = testDesign(); z.drive.rockerSpring = { k_Nm_rad: 0, phi0_rad: 0 };
  assert.equal(createFlight(z, { bird: buildBird(z), rho: 1e-9, gravity: 0, dt: 1 / 240 }).springEnergy(), 0);
});

test('boundaries of the new checks, by hand', () => {
  // EN: exactly 1.2 d0 is rated: k1 2.8 x 1.2 - 1.7 = 1.66, alpha_d 0.4: 1.66 x 0.4 x 260 / 1.25 = 138.112 MPa.
  near(enBoltBearing({ e1_over_d0: 1.2, e2_over_d0: 1.2 }, 260).MPa, 138.112, 1e-9);
  const al = { yield_MPa: 240, shear_yield_MPa: 138.6, bearing_yield_MPa: 193, bearing_e_over_d_min: 2, bearing_counted_at_half: true, ultimate_MPa: 260, bearing_standard: 'EN 1999-1-1' };
  near(bearingAllowable(al, 1.2).MPa, 138.112, 1e-9);
  // An fu without an EN standard named is not enough; a standard without fu neither.
  assert.equal(bearingAllowable({ ...al, bearing_standard: null }, 1.5).MPa, null);
  assert.equal(bearingAllowable({ ...al, ultimate_MPa: null }, 1.5).MPa, null);
  // Seat: no load is an infinite factor; exactly 1.5 is ok.
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 2 }, al, 0).fos, Infinity);
  const F15 = (193 / 1.5) * 10 * 2; // p = F / (d t) in MPa with mm
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 2 }, al, F15).ok, true);
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 2 }, al, F15 * 1.001).ok, false);
  // Lug: a small load still gives finite section factors; a rated lug is ok above 1.5 and not below.
  const small = lugCheck({ d_mm: 10, D_mm: 24.5, t_mm: 6 }, al, 10);
  near(small.fos_tension, 240 / (10 / (14.5 * 6)), 1e-9);
  near(small.fos_shear_out, 138.6 / (10 / (2 * 6 * 7.25)), 1e-9);
  assert.equal(small.ok, true);
  const big = lugCheck({ d_mm: 10, D_mm: 24.5, t_mm: 6 }, al, 20000);
  assert.ok(big.fos < 1.5 && big.ok === false);
  assert.equal(lugCheck({ d_mm: 10, D_mm: 16, t_mm: 6 }, al, 10).ok, false);
  // M1's pass rule: climbing, under 1.5 m, at most 8 m/s.
  assert.equal(m1Passed({ climb3: 0.1, lost: 1.49, speed: 8 }), true);
  assert.equal(m1Passed({ climb3: 0.1, lost: 1.49, speed: 8.01 }), false);
  assert.equal(m1Passed({ climb3: 0.1, lost: 1.5, speed: 8 }), false);
  assert.equal(m1Passed({ climb3: 0, lost: 1, speed: 8 }), false);
  assert.equal(m1Passed({ climb3: 0.5, lost: 1, speed: 7 }), true);
  // KH-beta at the rows themselves and at the ends of the columns.
  near(khBeta(0.2, 'weak'), 1.1, 0); near(khBeta(0.4, 'weak'), 1.3, 0); near(khBeta(1.6, 'rugged'), 2.05, 1e-12);
  near(khBeta(0.3, 'overhung'), 1.325, 1e-12);
  assert.equal(khBeta(1.7, 'rugged'), null);
  assert.equal(khBeta(2.5, 'equidistant'), null);
  near(zvFactor(1), 0.955, 0); near(zvFactor(3), 0.965, 0); near(zvFactor(10), 1, 0); near(zvFactor(0.5, false), 0.9, 0); near(zvFactor(6.5), 0.9825, 1e-12);
  near(duraconKv(0), 1.4, 0); near(duraconKv(5), 1.32, 1e-12); near(duraconKv(15), 0.895, 1e-12);
});

test('condition records: which hold, and the half rule where no correction exists', () => {
  const mesh = { m: 1, z: 18, zMate: 18, b_mm: 10, v_m_s: 0.5, support: 'equidistant', lubricant_cSt_50C: 100, reversing: false, hardened: true };
  const same = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, mesh);
  // KHK's own conditions: everything holds except the speed (0.5 vs 0.565 m/s), whose factors are 1 below 1 m/s.
  const by = Object.fromEntries(same.conditions.map((c) => [c.condition, c]));
  assert.equal(by['mating gear'].holds, true);
  assert.equal(by['gear support'].holds, true);
  assert.equal(by.lubricant.holds, true);
  assert.equal(by['load direction'].holds, true);
  assert.equal(by.cycles.holds, true); assert.equal(by.impact.holds, true);
  assert.equal(by.speed.holds, false);
  near(same.surface_Nm, 6.37, 1e-12); near(same.bending_Nm, 12.1, 1e-12);
  // Exactly at the rated viscosity holds; below it, half.
  assert.equal(jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, lubricant_cSt_50C: 99 }).half_rule, true);
  // A support beyond the table (overhung at b/d 1.4): no KH-beta, half.
  const oh = jgmaGearRating({ bending_Nm: 12.1, surface_Nm: 6.37 }, { ...mesh, b_mm: 25.2, support: 'overhung' });
  assert.equal(oh.half_rule, true);
  // A missing catalogue figure stays missing; the allowable is the one that exists.
  const nb = jgmaGearRating({ surface_Nm: 6.37 }, mesh);
  assert.equal(nb.bending_Nm, null); near(nb.allowable_Nm, 6.37, 1e-12);
  // Duracon: KHK's conditions (metal mate, one direction, grease, 40 C) give the speed factor alone; beyond 20 m/s, half.
  const d = lewisDuraconRating({ bending_Nm: 1 }, { m: 1, z: 20, v_m_s: 0.10472, mateDuracon: false, reversing: false, grease: true, temperature_C: 40 });
  near(d.bending_Nm, 1, 1e-8);
  const dby = Object.fromEntries(d.conditions.map((c) => [c.condition, c]));
  assert.equal(dby['mating material'].holds, true); assert.equal(dby['load direction'].holds, true);
  assert.equal(dby.lubrication.holds, true); assert.equal(dby.temperature.holds, true); assert.equal(dby.cycles.holds, true);
  const fast = lewisDuraconRating({ bending_Nm: 1 }, { m: 1, z: 20, v_m_s: 25, grease: true, temperature_C: 40 });
  assert.equal(fast.half_rule, true);
  assert.equal(lewisDuraconRating({}, { m: 1, z: 20, v_m_s: 1, grease: true, temperature_C: 40 }).bending_Nm, null);
});

test('hand-launch release speed: the design names it, capped by the gate at 8 m/s', () => {
  assert.equal(releaseSpeedOf({ launch: { release_speed_m_s: 7.5 } }), 7.5);
  assert.equal(releaseSpeedOf({}), 8);
});
