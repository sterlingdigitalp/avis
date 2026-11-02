import test from 'node:test';
import assert from 'node:assert/strict';
import { staticEquivalentLoad, leverShares, supportCheck, checkJoints, supportLoads, DEEP_GROOVE, FOS_STATIC, jointKind } from '../../src/physics/bearings.js';
import { strokeServoHold } from '../../src/physics/structure.js';
import { createFcGovernor } from '../../src/physics/governor.js';
import { makeFlapDrive, strokeAngle, rockerForAmplitude } from '../../src/physics/kinematics.js';
import { createFlight, quatFromEuler, compileSupports, LOAD_CHANNELS } from '../../src/physics/flight.js';
import { G0 } from '../../src/physics/atmosphere.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

test('ISO 76 static equivalent load, SKF deep groove form, worked by hand', () => {
  // SKF: P0 = 0.6 Fr + 0.5 Fa, and P0 = Fr when that is smaller than Fr.
  assert.equal(DEEP_GROOVE.X0, 0.6);
  assert.equal(DEEP_GROOVE.Y0, 0.5);
  // Fr 1000, Fa 500: 600 + 250 = 850 < 1000, so P0 = Fr.
  near(staticEquivalentLoad(1000, 500, DEEP_GROOVE), 1000, 1e-9);
  // Fr 1000, Fa 1500: 600 + 750 = 1350.
  near(staticEquivalentLoad(1000, 1500, DEEP_GROOVE), 1350, 1e-9);
  // Pure axial: 0.5 Fa.
  near(staticEquivalentLoad(0, 800, DEEP_GROOVE), 400, 1e-9);
});

test('rolling bearing check: C0 / P0 against 1.5, ineligible without a published C0', () => {
  const brg = { id: 'b', category: 'bearing', name: '608-2Z deep groove ball bearing', specs: { static_load_rating_kn: 1.37 } };
  assert.equal(jointKind(brg), 'deep-groove');
  // Fr 800, Fa 200: P0 = max(800, 480 + 100) = 800; 1370 / 800 = 1.7125.
  const c = supportCheck(brg, { Fr: 800, Fa: 200 });
  near(c.P0_N, 800, 1e-9);
  near(c.fos, 1.7125, 1e-9);
  assert.equal(c.ok, true);
  // Fr 1000: 1.37 < 1.5.
  assert.equal(supportCheck(brg, { Fr: 1000, Fa: 0 }).ok, false);
  const bare = { id: 'x', category: 'bearing', name: 'deep groove ball bearing, no rating', specs: {} };
  const u = supportCheck(bare, { Fr: 10, Fa: 0 });
  assert.equal(u.fos, null);
  assert.equal(u.ok, false);
  // A radial needle bearing: P0 = Fr, and any axial load fails the layout.
  const nd = { id: 'n', category: 'bearing', name: 'HK0608 drawn cup needle roller bearing', specs: { kind: 'drawn cup needle roller', static_load_rating_kn: 2.0 } };
  near(supportCheck(nd, { Fr: 1000, Fa: 0 }).fos, 2.0, 1e-9);
  assert.equal(supportCheck(nd, { Fr: 1000, Fa: 5 }).ok, false);
});

test('plain bushing: projected pressure, flange ring for axial load, published surface speed', () => {
  const bu = { id: 'g', category: 'bushing', name: 'flange bushing', specs: { d1_mm: 8, d2_mm: 10, b1_mm: 8, d3_mm: 15, static_surface_pressure_mpa: 80, max_surface_speed_rotating_m_s: 1 } };
  // 3000 N on 8 x 8 mm = 46.875 MPa; 80 / 46.875 = 1.70667.
  const c = supportCheck(bu, { Fr: 3000, Fa: 0, v_m_s: 0.2 });
  near(c.p_MPa, 46.875, 1e-9);
  near(c.fos, 80 / 46.875, 1e-9);
  assert.equal(c.ok, true);
  // Axial 3000 N on the flange ring pi/4 (15^2 - 10^2) = 98.175 mm2: 30.558 MPa, FoS 2.618; the radial governs.
  const f = supportCheck(bu, { Fr: 3000, Fa: 3000 });
  near(f.flange.area_mm2, (Math.PI / 4) * 125, 1e-9);
  near(f.flange.fos, 80 / (3000 / ((Math.PI / 4) * 125)), 1e-9);
  near(f.fos, 80 / 46.875, 1e-9);
  // Too fast for the material's published rotating surface speed.
  assert.equal(supportCheck(bu, { Fr: 100, v_m_s: 1.4 }).ok, false);
  // A sleeve (no flange) cannot take axial load.
  const sl = { ...bu, specs: { ...bu.specs, d3_mm: undefined } };
  assert.equal(supportCheck(sl, { Fr: 100, Fa: 50 }).ok, false);
  // A rod end against its published static radial load.
  const re = { id: 'r', category: 'rod_end', name: 'rod end', specs: { static_radial_load_n: 2000 } };
  near(supportCheck(re, { Fr: 1000 }).fos, 2, 1e-12);
  assert.equal(FOS_STATIC, 1.5);
});

test('lever shares of a shaft on two supports, between and overhung', () => {
  // Supports at 0 and 50 mm. A load at 12.5 mm: 3/4 on the near support, 1/4 on the far one.
  const [a, b] = leverShares(0, 0.05, 0.0125);
  near(a, 0.75, 1e-12);
  near(b, 0.25, 1e-12);
  // Overhung 10 mm behind the first support: R1 = F (L + e) / L = 1.2 F, R2 = F e / L = 0.2 F (opposite sense).
  const [c, d] = leverShares(0, 0.05, -0.01);
  near(c, 1.2, 1e-12);
  near(d, 0.2, 1e-12);
  // supportLoads sums the terms' magnitudes.
  const L = supportLoads({ radial: [{ ch: 'a', k: 1.2 }, { ch: 'b', k: 0.5 }], axial: [{ ch: 'c', k: 1 }] }, { a: 100, b: -40, c: 7 });
  near(L.Fr, 140, 1e-12);
  near(L.Fa, 7, 1e-12);
});

test('checkJoints finds the worst support and fails on any ineligible part', () => {
  const brg = { id: 'b', category: 'bearing', name: 'deep groove ball bearing', specs: { static_load_rating_kn: 1.0 } };
  const bu = { id: 'g', category: 'bushing', name: 'bushing', specs: { d1_mm: 6, b1_mm: 6, static_surface_pressure_mpa: 80, max_surface_speed_rotating_m_s: 1 } };
  const joints = [
    { name: 'shaft', part_id: 'b', count: 2, supports: [{ name: 'aft', part: brg }, { name: 'forward', part: brg }] },
    { name: 'eye', part_id: 'g', count: 1, supports: [{ name: 'eye', part: bu, surface_m_s_per_crank_rad_s: 0.003 }] },
  ];
  // 500 N on 1 kN: FoS 2; 400 N on 36 mm2 = 11.1 MPa: FoS 7.2; 600 N with Fa 300: P0 = max(600, 510) = 600: FoS 1.667.
  const r = checkJoints(joints, [{ Fr: 500, Fa: 0, P0: 500 }, { Fr: 600, Fa: 300, P0: 600 }, { Fr: 400, Fa: 0, P0: 400 }], { crankRate: 20 });
  assert.equal(r.ok, true);
  near(r.fos_min, 1000 / 600, 1e-9);
  assert.equal(r.worst, 'shaft/forward');
  near(r.joints[1].supports[0].speed.v_m_s, 0.06, 1e-12);
  const bad = checkJoints([{ name: 'x', supports: [{ name: 's', part: { id: 'n', category: 'bearing', name: 'bearing', specs: {} } }] }], [{ Fr: 1, Fa: 0, P0: 1 }]);
  assert.equal(bad.ok, false);
});

test('stroke servo holding load: pair force times the most favourable horn, worked by hand', () => {
  const d = { drive: { ampMin: 0.3, ampMax: 0.8 }, servos: { stroke: { stall_Nm: 1.5 } } };
  // 1000 N through the yoke, slot travel 20 mm: horn 10 mm, 10 N m against 1.5.
  const h = strokeServoHold(d, 1000, [0.016, 0.036]);
  near(h.horn_radius_m, 0.01, 1e-12);
  near(h.torque_Nm, 10, 1e-9);
  assert.equal(h.ok, false);
  // A fixed stroke has no slot to hold.
  assert.equal(strokeServoHold({ drive: { ampMin: 0.5, ampMax: 0.5, fixedStroke: true }, servos: {} }, 1000).applicable, false);
});

test('a fixed stroke drive is one rocker: the half-swing equals the amplitude, c from the closed-form limits', () => {
  const amp = 34 * DEG;
  const law = makeFlapDrive({ a: 0.018, b: 0.08, d: 0.08, ampMin: amp, ampMax: amp, ampNominal: amp, phiMean: 5 * DEG });
  const c = rockerForAmplitude({ a: 0.018, b: 0.08, d: 0.08 }, amp);
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < 1440; i++) {
    const s = strokeAngle((i / 1440) * TWO_PI, amp, law);
    near(s.c, c, 1e-12, 'one rocker radius');
    lo = Math.min(lo, s.phi);
    hi = Math.max(hi, s.phi);
  }
  near((hi - lo) / 2, amp, 2e-4, 'half swing');
  // Any amplitude asked of it gives the same rocker: the slot is gone.
  near(strokeAngle(0.3, 20 * DEG, law).c, c, 1e-12);
});

test('FC governor: encoder speed within its quantisation, one period of latency, clamp from measured speed and voltage', () => {
  const gov = { rate_Hz: 480, encoder: { bits: 12, window: 2 }, voltage: { lsb_V: 0.01, tau_s: 0 }, clamp_A: 30 };
  const kv = 90, R = 0.05, G = 25;
  const g = createFcGovernor(gov, { G, kv, R, cells: 2 });
  const T = 1 / 480, q = TWO_PI / 4096;
  const w = 20; // crank rad/s
  g.reset({ t: 0, phase: 0, crank: w, V: 8 });
  const want = () => 30; // far above the crank speed: the clamp must act
  let duty0 = null;
  for (let k = 0; k <= 10; k++) {
    const t = k * T;
    g.tick(t, w * t, 8, want, false, true);
    // Speed from two encoder intervals: error at most two quantisation steps over the window.
    assert.ok(Math.abs(g.st.rate - w) <= (2 * q) / (2 * T) + 1e-9, `rate ${g.st.rate}`);
    if (k === 0) duty0 = g.st.applied;
    else {
      // What reaches the ESC now was computed one tick ago.
      assert.equal(g.st.applied, g.st.prevPending);
    }
    g.st.prevPending = g.st.pending;
    // Clamp: duty <= (E + I R) / V with the measured speed: E = 20 * 25 / 90 = 5.556 V, I R = 1.5 V, V = 8 V.
    const dMax = ((g.st.rate * G) / kv + 30 * R) / 8;
    assert.ok(g.st.pending <= dMax + 1e-12);
    assert.equal(g.st.clampedPending, true);
  }
  near(duty0, (((20 * 25) / 90) + 1.5) / 8, 0.01, 'first duty at the clamp');
  // Extrapolated phase between ticks.
  near(g.phaseAt(10 * T + 0.001), w * (10 * T + 0.001), 2 * q + 0.001 * Math.abs(g.st.rate - w) + 1e-9);
});

test('root load channels: no internal load in free fall; wing inertia loads scale with the square of the stroke rate', () => {
  // Equivalence principle: a parked bird falling freely in vacuum carries no load at its shoulders.
  const d = testDesign();
  d.structure = { joints: [{ name: 'shoulder pivot right', supports: [{ name: 'aft', radial: [{ ch: 'root_yz_R', k: 1 }, { ch: 'root_mperp_R', k: 10 }], axial: [{ ch: 'root_x_R', k: 1 }] }] }] };
  const run = (freq, gravity) => {
    const sim = createFlight(d, { dt: 1 / 960, rho: 0, gravity });
    const fl = freq > 0;
    sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0, 0), vel: [9, 0, 0], phase: fl ? 0 : Math.PI / 2, crank: freq * TWO_PI, amp: 18 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
    Object.assign(sim.cmd, fl ? { freq, amp: 18 * DEG, dyn: 0, sym: 0, tailPitch: 0 } : { freq: 0, amp: 18 * DEG, dyn: 0, sym: 0, tailPitch: 0, throttleOff: true, parkPhase: Math.PI / 2 });
    // One second to settle the governor, then one second of peaks.
    for (let k = 0; k < 960; k++) sim.step();
    sim.resetPeaks();
    for (let k = 0; k < 960; k++) sim.step();
    return sim.peaks;
  };
  const still = run(0, G0);
  assert.ok(still.channels.root_yz_R < 1e-6 && still.channels.root_x_R < 1e-6 && still.channels.root_mperp_R < 1e-6, JSON.stringify(still.channels));
  // In vacuum without gravity every load is inertial, so doubling the stroke rate at the same stroke quadruples it.
  const p1 = run(1, 0), p2 = run(2, 0);
  for (const k of ['root_yz_R', 'root_mperp_R', 'coupler_R']) near(p2.channels[k] / p1.channels[k], 4, 0.4, k);
  // The support sums its terms as the load path says.
  assert.ok(p2.supports[0].Fr >= p2.channels.root_yz_R && p2.supports[0].Fr <= p2.channels.root_yz_R + 10 * p2.channels.root_mperp_R + 1e-9);
  assert.equal(LOAD_CHANNELS.length, compileSupports([{ name: 'j', supports: [{ name: 's', radial: LOAD_CHANNELS.map((ch) => ({ ch, k: 1 })) }] }])[0].rc.length);
});

test('mixed-material mesh: Hertz at the pitch point, worked by hand; unjudged without published surface allowables', async () => {
  const { hertzSpur, mixedMeshCheck, gearMaterialClass } = await import('../../src/physics/structure.js');
  // Steel 18T m1 pinion on a 60T nylon wheel, face 10 mm, 0.68 N m: Ft 75.56 N, Fn 80.40 N, rho 3.078 and 10.26 mm,
  // E 205.9 GPa (nu 0.3) and 3.43 GPa (nu 0.4): sigma_H = 65.84 MPa.
  const s = hertzSpur({ T_pinion_Nm: 0.68, r1_m: 0.009, r2_m: 0.03, b_m: 0.01, alpha_rad: 20 * DEG, E1_Pa: 205.9e9, nu1: 0.3, E2_Pa: 3.43e9, nu2: 0.4 });
  near(s, 65.84, 0.02);
  assert.equal(gearMaterialClass('SCM415'), 'steel');
  assert.equal(gearMaterialClass('Duracon (R) acetal M90-44'), 'acetal');
  assert.equal(gearMaterialClass('MC901 nylon'), 'nylon');
  const stage = { pinion_material: 'SCM415', gear_material: 'MC901 nylon', pinion_rp_m: 0.009, gear_rp_m: 0.03, pinion_face_m: 0.01, gear_face_m: 0.01, alpha_rad: 20 * DEG };
  // KHK publishes no surface allowable for MC901: the mesh cannot be judged.
  assert.equal(mixedMeshCheck(stage, 0.68).ok, false);
  const judged = mixedMeshCheck({ ...stage, pinion_contact: { allowable_MPa: 1628, E_Pa: 205.9e9, nu: 0.3 }, gear_contact: { allowable_MPa: 60, E_Pa: 3.43e9, nu: 0.4 } }, 0.68);
  assert.equal(judged.ok, false); // 65.8 MPa over a 60 MPa member
  assert.equal(mixedMeshCheck({ ...stage, gear_material: 'SCM415' }, 0.68).mixed, false);
});

test('jointsRatio: the worst use of any rating, speed limits and layout faults included', async () => {
  const { jointsRatio } = await import('../../src/physics/bearings.js');
  const j = (sup) => ({ joints: [{ supports: sup }] });
  // FoS 3 uses half the 1.5 margin: 0.5; a bushing at 0.9 of its surface speed: 0.9.
  near(jointsRatio(j([{ fos: 3 }, { fos: 6, speed: { v_m_s: 0.9, max_m_s: 1 } }])), 0.9, 1e-12);
  near(jointsRatio(j([{ fos: 1.2 }])), 1.25, 1e-12);
  assert.equal(jointsRatio(j([{ fos: null }])), Infinity, 'ineligible part');
  assert.equal(jointsRatio(j([{ fos: 5, kind: 'needle', Fa_N: 3 }])), Infinity, 'needle under axial load');
  assert.equal(jointsRatio(j([{ fos: 5, flange: { area_mm2: 0 } }])), Infinity, 'sleeve under axial load');
  assert.equal(jointsRatio(j([{ fos: 5, speed: { v_m_s: 0.1, max_m_s: null } }])), Infinity, 'no published speed limit');
  assert.equal(jointsRatio(null), 0);
});

test('FC governor: integrator held against the clamp, reset when off, PI tracks a step in the target', () => {
  const kv = 90, R = 0.05, G = 25;
  const gov = { rate_Hz: 960, encoder: { bits: 14, window: 2 }, voltage: { lsb_V: 0, tau_s: 0 }, clamp_A: 1e9 };
  const g = createFcGovernor(gov, { G, kv, R, cells: 2, kp: 0.5, ki: 1.0 });
  const T = 1 / 960;
  g.reset({ t: 0, phase: 0, crank: 20, V: 8, iInt: 0.2 });
  // Crank held at 20 rad/s, target 21: the error integrates at Ki = ki * kp * G / kv per second per rad/s.
  for (let k = 0; k <= 96; k++) g.tick(k * T, 20 * k * T, 8, () => 21, false, true);
  const Ki = 1.0 * 0.5 * (G / kv);
  near(g.st.iInt, 0.2 + 97 * T * Ki * 1, 0.02, 'integrator');
  // Duty: (target G / kv + Kp err + I) / V.
  near(g.st.pending, ((21 * G) / kv + 0.5 * (G / kv) * 1 + g.st.iInt) / 8, 0.01, 'duty law');
  // Off: duty 0 and the integrator cleared; not integrating (stopped, no park): cleared too.
  g.tick(98 * T, 20 * 98 * T, 8, () => 21, true, true);
  assert.equal(g.st.pending, 0);
  assert.equal(g.st.iInt, 0);
  g.tick(99 * T, 20 * 99 * T, 8, () => 21, false, false);
  assert.equal(g.st.iInt, 0);
  // Against the clamp the integrator does not wind up.
  const c = createFcGovernor({ ...gov, clamp_A: 1 }, { G, kv, R, cells: 2 });
  c.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  for (let k = 0; k <= 50; k++) c.tick(k * T, 20 * k * T, 8, () => 40, false, true);
  assert.equal(c.st.clampedPending, true);
  assert.equal(c.st.iInt, 0);
  assert.ok(c.due(51 * T) && !c.due(50.5 * T));
});

test('flight channels: symmetric flight loads both shoulders alike; the slot share is cos(mu) of the coupler force; peaks reset in place', async () => {
  const { strokeAngle, makeFlapDrive } = await import('../../src/physics/kinematics.js');
  const d = testDesign();
  d.structure = { joints: [{ name: 'j', supports: [{ name: 's', radial: [{ ch: 'root_yz_L', k: 1 }], axial: [{ ch: 'root_x_L', k: 2 }], X0: 0.6, Y0: 0.5 }] }] };
  const sim = createFlight(d, { dt: 1 / 960 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.05, 0), vel: [9, 0, 0], phase: 0, crank: TWO_PI * 2, amp: 18 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
  Object.assign(sim.cmd, { freq: 2, amp: 18 * DEG, dyn: 0.3, sym: 0, tailPitch: 0 });
  const law = makeFlapDrive(d.drive);
  let worstSlot = 0, worstSym = 0;
  for (let k = 0; k < 960; k++) {
    sim.step();
    const p = sim.peaks.channels;
    worstSym = Math.max(worstSym, Math.abs(p.root_yz_L - p.root_yz_R) / Math.max(p.root_yz_R, 1e-9), Math.abs(p.root_mperp_L - p.root_mperp_R) / Math.max(p.root_mperp_R, 1e-9), Math.abs(p.coupler_L - p.coupler_R) / Math.max(p.coupler_R, 1e-9));
  }
  assert.ok(worstSym < 1e-6, `left and right differ by ${worstSym}`);
  // The slot share can never exceed the coupler force, and at the coupler's peak it is |cos mu| of it.
  const p = sim.peaks.channels;
  assert.ok(p.slot_R <= p.coupler_R + 1e-9 && p.slot_R > 0);
  let cosMax = 0;
  for (let i = 0; i < 720; i++) cosMax = Math.max(cosMax, Math.abs(Math.cos(strokeAngle((i / 720) * TWO_PI, 18 * DEG, law).mu)));
  assert.ok(p.slot_R <= cosMax * p.coupler_R + 1e-9, `${p.slot_R} > ${cosMax} x ${p.coupler_R}`);
  // The support sees the left root force, its axial term doubled, and tracks the ISO 76 peak.
  const s = sim.peaks.supports[0];
  near(s.Fr, p.root_yz_L, 1e-9);
  near(s.Fa, 2 * p.root_x_L, 1e-9);
  assert.ok(s.P0 >= s.Fr - 1e-12);
  assert.ok(sim.peaks.motorIDrive > 0 && sim.peaks.crankRate > 10);
  sim.resetPeaks();
  assert.equal(sim.peaks.channels.coupler_R, 0);
  assert.equal(sim.peaks.supports[0].P0, 0);
  assert.equal(sim.peaks.motorIBrake, 0);
});

test('governor in the flight controller holds the commanded crank speed as the ESC-side law does', () => {
  const run = (gov) => {
    const d = testDesign();
    if (gov) d.drive.governor = { location: 'flight controller', rate_Hz: 960, encoder: { bits: 14, window: 2 }, voltage: { lsb_V: 0.0169, tau_s: 0.002 }, clamp_A: 1e9 };
    const sim = createFlight(d, { dt: 1 / 960 });
    sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.05, 0), vel: [9, 0, 0], phase: 0, crank: TWO_PI * 2, amp: 15 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
    Object.assign(sim.cmd, { freq: 2.2, amp: 15 * DEG, dyn: 0.2, sym: 0, tailPitch: 0 });
    let sum = 0, n = 0;
    for (let k = 0; k < 4800; k++) { sim.step(); if (k >= 3840) { sum += sim.x[14]; n++; } }
    return sum / n / TWO_PI;
  };
  // Mean crank frequency over the fifth second within 2% of 2.2 Hz either way, and the two laws within 0.5% of each
  // other (the PI's slow integral leaves both about 1% short at 5 s).
  const fc = run(true), esc = run(false);
  near(fc, 2.2, 0.044, 'FC');
  near(esc, 2.2, 0.044, 'ESC');
  near(fc, esc, 0.011, 'FC vs ESC');
});
