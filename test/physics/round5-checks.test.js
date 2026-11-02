// Round 5: the bearing, governor and mesh checks case by case, each against numbers worked by hand.
import test from 'node:test';
import assert from 'node:assert/strict';
import { leverShares, supportCheck, checkJoints, supportLoads, jointKind } from '../../src/physics/bearings.js';
import { createFcGovernor } from '../../src/physics/governor.js';
import { strokeServoHold, mixedMeshCheck } from '../../src/physics/structure.js';
import { rockerForAmplitude } from '../../src/physics/kinematics.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

test('joint kinds from category and name; supports without a rated part are ineligible', () => {
  assert.equal(jointKind({ category: 'bearing', name: 'angular contact bearing', specs: {} }), 'rolling');
  assert.equal(jointKind({ category: 'bearing', name: 'x', specs: { kind: 'deep groove ball' } }), 'deep-groove');
  assert.equal(jointKind({ category: 'bearing', name: 'HK0608', specs: { kind: 'drawn cup needle roller' } }), 'needle');
  assert.equal(jointKind({ category: 'bearing', name: 'HK needle roller', specs: {} }), 'needle');
  assert.equal(jointKind({ category: 'bushing', name: 'b' }), 'bushing');
  assert.equal(jointKind({ category: 'rod_end', name: 'r' }), 'rod-end');
  assert.equal(jointKind({ category: 'gear', name: 'g' }), null);
  assert.equal(jointKind(null), null);
  const none = supportCheck({ category: 'gear', name: 'g', specs: {} }, { Fr: 1 });
  assert.equal(none.ok, false);
  assert.equal(none.fos, null);
  assert.throws(() => leverShares(0.01, 0.01, 0), /apart/);
  // Shares are magnitudes whichever way round the supports are given.
  const [a, b] = leverShares(0.05, 0, 0.0125);
  near(a, 0.25, 1e-12);
  near(b, 0.75, 1e-12);
});

test('rolling bearing: the part\'s own X0/Y0 win, the peak P0 over time counts, negative loads are none', () => {
  const own = { category: 'bearing', name: 'angular contact', specs: { static_load_rating_kn: 2, X0: 0.5, Y0: 0.9 } };
  // P0 = max(Fr, 0.5 Fr + 0.9 Fa) = max(1000, 500 + 900) = 1400; FoS 2000 / 1400.
  const c = supportCheck(own, { Fr: 1000, Fa: 1000 });
  near(c.P0_N, 1400, 1e-9);
  near(c.fos, 2000 / 1400, 1e-12);
  assert.equal(c.X0, 0.5);
  assert.equal(c.Y0, 0.9);
  // A larger peak of the equivalent over time is used: 1600 N.
  near(supportCheck(own, { Fr: 1000, Fa: 1000, P0: 1600 }).P0_N, 1600, 1e-9);
  // Negative inputs read as zero load: infinite FoS, ok.
  const z = supportCheck(own, { Fr: -5, Fa: -5 });
  assert.equal(z.Fr_N, 0);
  assert.equal(z.fos, Infinity);
  assert.equal(z.ok, true);
  // A generic rolling bearing without published X0/Y0 cannot be judged under axial load.
  assert.equal(supportCheck({ category: 'bearing', name: 'angular contact', specs: { static_load_rating_kn: 2 } }, { Fr: 10, Fa: 10 }).fos, null);
  // FoS exactly 1.5 passes; just under fails.
  const dg = { category: 'bearing', name: 'deep groove ball bearing', specs: { static_load_rating_kn: 1.5 } };
  assert.equal(supportCheck(dg, { Fr: 1000 }).ok, true);
  assert.equal(supportCheck(dg, { Fr: 1000.1 }).ok, false);
});

test('bushing: ineligible without size or pressure, no flange term without axial load, speed rules', () => {
  const s = { d1_mm: 6, d2_mm: 8, b1_mm: 6, d3_mm: 12, static_surface_pressure_mpa: 80, max_surface_speed_rotating_m_s: 1 };
  const bu = (x = {}) => ({ category: 'bushing', name: 'b', specs: { ...s, ...x } });
  assert.equal(supportCheck(bu({ d1_mm: undefined }), { Fr: 10 }).fos, null);
  assert.equal(supportCheck(bu({ b1_mm: 0 }), { Fr: 10 }).fos, null);
  assert.equal(supportCheck(bu({ static_surface_pressure_mpa: undefined }), { Fr: 10 }).ok, false);
  // 1920 N on 36 mm2 = 53.33 MPa: FoS 1.5 exactly passes; no axial load, no flange entry, no speed given, no speed entry.
  const c = supportCheck(bu(), { Fr: 1920 });
  near(c.p_MPa, 1920 / 36, 1e-9);
  assert.equal(c.flange, null);
  assert.equal(c.speed, null);
  assert.equal(c.ok, true);
  assert.equal(supportCheck(bu(), { Fr: 1921 }).ok, false);
  // A speed exactly at the limit passes; a part without a published speed limit fails when a speed is asked.
  assert.equal(supportCheck(bu(), { Fr: 10, v_m_s: 1 }).speed.ok, true);
  const noV = supportCheck(bu({ max_surface_speed_rotating_m_s: undefined }), { Fr: 10, v_m_s: 0.01 });
  assert.equal(noV.speed.ok, false);
  assert.equal(noV.ok, false);
  // The flange governs when its ring is the weaker: 1000 N axial on pi/4 (144 - 64) = 62.83 mm2 = 15.92 MPa: FoS 5.03.
  const f = supportCheck(bu(), { Fr: 10, Fa: 1000 });
  near(f.flange.p_MPa, 1000 / ((Math.PI / 4) * 80), 1e-9);
  near(f.fos, 80 / (1000 / ((Math.PI / 4) * 80)), 1e-9);
  near(f.radial_fos, 80 / (10 / 36), 1e-9);
});

test('rod end: rating in kN or N along the shank; a load along the pin fails', () => {
  const kn = { category: 'rod_end', name: 'r', specs: { static_radial_load_kn: 1.5 } };
  near(supportCheck(kn, { Fr: 500 }).fos, 3, 1e-12);
  assert.equal(supportCheck(kn, { Fr: 1000 }).ok, true);
  assert.equal(supportCheck(kn, { Fr: 1001 }).ok, false);
  assert.equal(supportCheck(kn, { Fr: 10, Fa: 1 }).ok, false);
  assert.equal(supportCheck(kn, { Fr: 0 }).fos, Infinity);
  assert.equal(supportCheck({ category: 'rod_end', name: 'r', specs: {} }, { Fr: 1 }).fos, null);
});

test('support loads and joint checks: missing channels are zero, coefficients as magnitudes, empty is not a pass', () => {
  const L = supportLoads({ radial: [{ ch: 'a', k: -2 }, { ch: 'missing', k: 5 }], axial: [{ ch: 'b', k: -0.5 }] }, { a: 3, b: -8 });
  near(L.Fr, 6, 1e-12);
  near(L.Fa, 4, 1e-12);
  assert.deepEqual(supportLoads({}, {}), { Fr: 0, Fa: 0 });
  const empty = checkJoints([], []);
  assert.equal(empty.ok, false);
  assert.equal(empty.fos_min, null);
  const brg = { category: 'bearing', name: 'deep groove ball bearing', specs: { static_load_rating_kn: 3 } };
  const r = checkJoints([{ name: 'j', part_id: 'p', count: 2, reacts: 'x', supports: [{ name: 'a', part: brg }, { name: 'b', part: brg }] }], [{ Fr: 1000, Fa: 0, P0: 1000 }, { Fr: 500, Fa: 0, P0: 500 }]);
  assert.equal(r.joints[0].part, 'p');
  assert.equal(r.joints[0].count, 2);
  near(r.joints[0].fos, 3, 1e-12);
  assert.equal(r.worst, 'j/a');
  // One failing support fails the joint and the check; missing peaks read as no load.
  const bad = checkJoints([{ name: 'j', supports: [{ name: 'a', part: brg }, { name: 'b', part: brg }] }], [{ Fr: 2500, Fa: 0, P0: 2500 }]);
  assert.equal(bad.ok, false);
  assert.equal(bad.joints[0].ok, false);
  assert.equal(bad.joints[0].supports[1].fos, Infinity);
});

test('FC governor: encoder wrap backwards, speed over the sample window, voltage resolution and its filter', () => {
  const T = 1 / 960;
  const mk = (extra = {}) => createFcGovernor({ rate_Hz: 960, encoder: { bits: 14 }, clamp_A: Infinity, ...extra }, { G: 25, kv: 90, R: 0.05, cells: 2 });
  // A crank turning backwards through zero: the encoder wraps, the speed stays -10 rad/s.
  const g = mk();
  g.reset({ t: 0, phase: 0.005, crank: -10, V: 8 });
  for (let k = 0; k < 10; k++) g.tick(k * T, 0.005 - 10 * k * T, 8, () => 0, false, true);
  near(g.st.rate, -10, 0.2, 'backwards');
  // Constant acceleration: the default window is two intervals, so the estimate is the speed one interval back.
  const a = mk();
  a.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  const th = (t) => 20 * t + 0.5 * 300 * t * t;
  for (let k = 1; k <= 20; k++) a.tick(k * T, th(k * T), 8, () => 0, false, true);
  near(a.st.rate, 20 + 300 * 19 * T, 0.05, 'window 2');
  // Voltage: quantised to its resolution, then a first-order filter of time constant tau at the tick rate.
  const v = mk({ voltage: { lsb_V: 0.01, tau_s: 0.002 } });
  v.reset({ t: 0, phase: 0, crank: 20, V: 8.0049 });
  near(v.st.vF, 8.0, 1e-9, 'quantised');
  v.tick(0, 0, 7.0, () => 20, false, true);
  near(v.st.vF, 8 + (T / (0.002 + T)) * (7 - 8), 1e-9, 'filter step');
  // Without a filter the reading follows the input at once.
  const n = mk({ voltage: { lsb_V: 0, tau_s: 0 } });
  n.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  n.tick(0, 0, 7.3333, () => 20, false, true);
  near(n.st.vF, 7.3333, 1e-12);
  // Due exactly on its grid, not before.
  assert.equal(n.due(T - 1e-6), false);
  assert.equal(n.due(T), true);
});

test('FC governor: duty law, clamp and integrator limit, worked by hand', () => {
  const T = 1 / 960, G = 25, kv = 90, R = 0.05;
  const g = createFcGovernor({ rate_Hz: 960, encoder: { bits: 16, window: 1 }, clamp_A: 30 }, { G, kv, R, cells: 2, kp: 1, ki: 0 });
  g.reset({ t: 0, phase: 0, crank: 10, V: 8, iInt: 0 });
  g.tick(0, 0, 8, () => 12, false, true);
  // Below the clamp: d = (12 G / kv + Kp (12 - 10) + 0) / V with Kp = 1 x G / kv: (3.333 + 0.5556) / 8 = 0.48611.
  near(g.st.pending, ((12 * G) / kv + (G / kv) * 2) / 8, 2e-3);
  assert.equal(g.st.clampedPending, false);
  assert.equal(g.st.applied, g.st.pending, 'the first tick applies at once');
  // Clamp: (E + I R) / V = (10 x 25 / 90 + 1.5) / 8 = 0.53472 against a demand of (100 G / kv + ...) / 8 > 1.
  g.tick(T, 10 * T, 8, () => 100, false, true);
  near(g.st.pending, ((10 * G) / kv + 30 * R) / 8, 2e-3);
  assert.equal(g.st.clampedPending, true);
  // The integrator is bounded at half the full-charge pack voltage: 0.5 x 2 x 4.2 = 4.2 V.
  const w = createFcGovernor({ rate_Hz: 960, encoder: { bits: 14, window: 1 }, clamp_A: Infinity }, { G, kv, R, cells: 2, kp: 0.5, ki: 1000 });
  w.reset({ t: 0, phase: 0, crank: 10, V: 8 });
  for (let k = 0; k < 200; k++) w.tick(k * T, 10 * k * T, 8, () => 30, false, true);
  near(w.st.iInt, 4.2, 1e-9);
  for (let k = 200; k < 400; k++) w.tick(k * T, 10 * k * T, 8, () => -30, false, true);
  near(w.st.iInt, -4.2, 1e-9);
  // Duty never leaves [0, 1].
  assert.ok(w.st.pending >= 0 && w.st.pending <= 1);
  // Phase as known between ticks: last sample advanced at the measured speed.
  near(w.phaseAt(399 * T + 0.0005), (10 * 399 * T) % TWO_PI + 10 * 0.0005, 1e-3);
});

test('stroke servo hold from the drive\'s own rocker table; mixed meshes judged member by member', () => {
  const drive = { kind: 'crank-rocker', a: 0.012, b: 0.08, d: 0.08, ampMin: 20 * DEG, ampMax: 48 * DEG, ampNominal: 34 * DEG, phiMean: 5 * DEG, gearRatio: 25 };
  // The slot runs from the rocker of the largest amplitude to that of the smallest (closed-form limits).
  const cHi = rockerForAmplitude(drive, 20 * DEG), cLo = rockerForAmplitude(drive, 48 * DEG);
  const h = strokeServoHold({ drive, servos: { stroke: { stall_Nm: 1.5 } } }, 100);
  near(h.slot_travel_m, cHi - cLo, 1e-6);
  near(h.torque_Nm, (100 * (cHi - cLo)) / 2, 1e-6);
  assert.equal(h.ok, true); // 100 N x 9.5 mm = 0.95 N m < 1.5
  assert.equal(strokeServoHold({ drive, servos: { stroke: { stall_Nm: 0.5 } } }, 100).ok, false);
  assert.equal(strokeServoHold({ drive, servos: {} }, 100).applicable, false);
  // Mixed mesh: both allowables published, each member judged, the narrower face used.
  const st = { pinion_material: 'SCM415', gear_material: 'Duracon (R) acetal M90-44', pinion_rp_m: 0.009, gear_rp_m: 0.03, pinion_face_m: 0.01, gear_face_m: 0.006, alpha_rad: 20 * DEG, pinion_contact: { allowable_MPa: 1628, E_Pa: 205.9e9, nu: 0.3 }, gear_contact: { allowable_MPa: 200, E_Pa: 2.7e9, nu: 0.35 } };
  const m = mixedMeshCheck(st, 0.68);
  // Hand: Fn 80.40 N, k 422.33 /m, c = 0.91/205.9e9 + 0.8775/2.7e9 = 3.2942e-10; b 6 mm: sigma = 82.57 MPa.
  near(m.sigma_H_MPa, Math.sqrt((80.4041 * 422.334) / (Math.PI * 0.006 * (0.91 / 205.9e9 + 0.8775 / 2.7e9))) / 1e6, 0.05);
  assert.equal(m.ok, true);
  assert.equal(mixedMeshCheck({ ...st, pinion_contact: { ...st.pinion_contact, allowable_MPa: 50 } }, 0.68).ok, false);
  assert.equal(mixedMeshCheck({ ...st, gear_contact: { ...st.gear_contact, E_Pa: undefined } }, 0.68).ok, false);
  assert.equal(mixedMeshCheck({ ...st, gear_material: null }, 0.68).mixed, false);
});

test('edge cases the gate relies on: small loads, factor fields, single joints, speed ratios, needle axial', async () => {
  const { jointsRatio } = await import('../../src/physics/bearings.js');
  const { gearMaterialClass } = await import('../../src/physics/structure.js');
  const dg = { category: 'bearing', name: 'deep groove ball bearing', specs: { static_load_rating_kn: 1 } };
  // Sub-newton loads are loads: 0.5 N gives FoS 2000.
  near(supportCheck(dg, { Fr: 0.5 }).fos, 2000, 1e-9);
  assert.equal(supportCheck(null, { Fr: 1 }).ok, false);
  // A part with only X0 published falls back to SKF's pair; a needle reports 1 / 0 and passes without axial load.
  const half = supportCheck({ category: 'bearing', name: 'deep groove ball bearing', specs: { static_load_rating_kn: 1, X0: 0.3 } }, { Fr: 100, Fa: 1000 });
  assert.deepEqual([half.X0, half.Y0], [0.6, 0.5]);
  const nd = supportCheck({ category: 'bearing', name: 'n', specs: { kind: 'drawn cup needle roller', static_load_rating_kn: 2 } }, { Fr: 1000, Fa: 0 });
  assert.deepEqual([nd.X0, nd.Y0, nd.ok], [1, 0, true]);
  // A sleeve takes no axial load at all, however small; a flange needs both diameters.
  const sl = { category: 'bushing', name: 's', specs: { d1_mm: 6, d2_mm: 8, b1_mm: 6, static_surface_pressure_mpa: 80, max_surface_speed_rotating_m_s: 1 } };
  assert.equal(supportCheck(sl, { Fr: 10, Fa: 0.5 }).ok, false);
  assert.equal(supportCheck({ ...sl, specs: { ...sl.specs, d3_mm: 12, d2_mm: undefined } }, { Fr: 10, Fa: 5 }).ok, false);
  assert.equal(supportCheck({ category: 'rod_end', name: 'r', specs: {} }, { Fr: 1 }).ok, false);
  // One joint alone can pass; no crank rate given means no surface speed.
  const one = checkJoints([{ name: 'e', supports: [{ name: 'eye', part: sl, surface_m_s_per_crank_rad_s: 2 }] }], [{ Fr: 100, Fa: 0, P0: 100 }]);
  assert.equal(one.ok, true);
  assert.equal(one.joints[0].supports[0].speed.v_m_s, 0);
  // Speed ratio is v / v_max; only a needle with axial load is a layout fault.
  near(jointsRatio({ joints: [{ supports: [{ fos: 10, speed: { v_m_s: 0.5, max_m_s: 2 } }] }] }), 0.25, 1e-12);
  near(jointsRatio({ joints: [{ supports: [{ fos: 3, kind: 'bushing', Fa_N: 5 }] }] }), 0.5, 1e-12);
  near(jointsRatio({ joints: [{ supports: [{ fos: 3, kind: 'needle', Fa_N: 0 }] }] }), 0.5, 1e-12);
  assert.equal(gearMaterialClass('brass'), 'brass');
  assert.equal(gearMaterialClass(''), null);
});

test('governor and hold edge cases: no voltage spec means exact voltage, off is not clamped, no wind-up near the clamp', () => {
  const T = 1 / 960;
  const g = createFcGovernor({ rate_Hz: 960, encoder: { bits: 14, window: 2 }, clamp_A: Infinity }, { G: 25, kv: 90, R: 0.05, cells: 2 });
  g.reset({ t: 0, phase: 0, crank: 20, V: 7.3333 });
  near(g.st.vF, 7.3333, 1e-12, 'no resolution, no filter');
  // A 2 V step through a 2 ms filter at 960 Hz moves the reading by 2 T / (tau + T).
  const f = createFcGovernor({ rate_Hz: 960, encoder: { bits: 14, window: 2 }, voltage: { lsb_V: 0, tau_s: 0.002 }, clamp_A: Infinity }, { G: 25, kv: 90, R: 0.05, cells: 2 });
  f.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  f.tick(0, 0, 6, () => 20, false, true);
  near(f.st.vF, 8 - (2 * T) / (0.002 + T), 1e-12);
  f.tick(T, 20 * T, 8, () => 20, true, true);
  assert.equal(f.st.clampedPending, false);
  // Clamped with a small positive speed error: the integrator holds.
  const c = createFcGovernor({ rate_Hz: 960, encoder: { bits: 16, window: 1 }, clamp_A: 0.01 }, { G: 25, kv: 90, R: 0.05, cells: 2 });
  c.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  for (let k = 0; k < 20; k++) c.tick(k * T, 20 * k * T, 8, () => 20.5, false, true);
  assert.equal(c.st.clampedPending, true);
  assert.equal(c.st.iInt, 0);
  // Stroke servo hold: equal amplitudes without the flag are a fixed stroke too; a slotted drive is applicable.
  const fx = strokeServoHold({ drive: { ampMin: 0.6, ampMax: 0.6 }, servos: { stroke: { stall_Nm: 1 } } }, 1e4);
  assert.deepEqual([fx.applicable, fx.ok], [false, true]);
  const sl = strokeServoHold({ drive: { ampMin: 0.3, ampMax: 0.8 }, servos: { stroke: { stall_Nm: 1 } } }, 10, [0.01, 0.03]);
  assert.deepEqual([sl.applicable, sl.ok], [true, true]);
  // Mixed meshes: one allowable missing is unjudged (and still mixed); one material is fine and judged by its maker.
  const st = { pinion_material: 'SCM415', gear_material: 'MC901 nylon', pinion_rp_m: 0.009, gear_rp_m: 0.03, pinion_face_m: 0.01, gear_face_m: 0.01, alpha_rad: 20 * DEG, pinion_contact: { allowable_MPa: 1628, E_Pa: 205.9e9, nu: 0.3 }, gear_contact: { E_Pa: 3.43e9, nu: 0.4 } };
  const u = mixedMeshCheck(st, 0.68);
  assert.deepEqual([u.mixed, u.ok], [true, false]);
  const same = mixedMeshCheck({ ...st, gear_material: 'SCM415' }, 0.68);
  assert.deepEqual([same.mixed, same.ok], [false, true]);
  const j = mixedMeshCheck({ ...st, gear_contact: { allowable_MPa: 100, E_Pa: 3.43e9, nu: 0.4 } }, 0.68);
  assert.deepEqual([j.mixed, j.ok], [true, true]);
});

test('strip root moment: its hinge-axis part is the generalised stroke force; with the hinge offsets it rebuilds the total moment', async () => {
  const { makeWing } = await import('../../src/physics/kinematics.js');
  const { createStrip } = await import('../../src/physics/strip.js');
  const hinge = [0, 0.02, 0.025];
  const w = makeWing({ hinge, segments: [{ length: 0.5, chord_in: 0.3, chord_out: 0.2, xle_in: 0.1, xle_out: 0.05, gain: 0, twist_in: 0, twist_out: 0 }], spar_x: 0, camber: 0.03, camber_pos: 0.4 });
  for (const inviscid of [true, false]) {
    const m = createStrip(w, { n: 8, inviscid });
    const J = { phi: 0, dphi: 3, ddphi: 40, theta: 0, dtheta: 0, ddtheta: 0 };
    let r;
    for (let k = 0; k < 400; k++) r = m.evaluate([9, -0.5, 0], [0, 0, 0], [0, 0, 0], J, J, 1, 1.5e-5, 0.002);
    // Rotation about body x through the hinge: the moment about x is the stroke's generalised force (partial velocities,
    // a separate derivation), right wing -Qphi and left wing +Qphi by the mirror.
    near(-r.rootMoment[0], r.Qphi[0], 1e-9 + 1e-9 * Math.abs(r.Qphi[0]), 'right');
    near(r.rootMoment[3], r.Qphi[1], 1e-9 + 1e-9 * Math.abs(r.Qphi[1]), 'left');
    // Total aero moment about the body origin = sum of root moments + hinge x wing force, per wing.
    const n = r.fx.length / 2;
    const F = [0, 1].map((wi) => [0, 1, 2].map((c) => { let s = 0; for (let i = 0; i < n; i++) s += [r.fx, r.fy, r.fz][c][i + wi * n]; return s; }));
    const H = [[hinge[0], hinge[1], hinge[2]], [hinge[0], hinge[1], -hinge[2]]];
    const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    for (let c = 0; c < 3; c++) {
      const rebuilt = r.rootMoment[c] + r.rootMoment[3 + c] + cross(H[0], F[0])[c] + cross(H[1], F[1])[c];
      near(rebuilt, r.M[c], 1e-9 + 1e-6 * Math.abs(r.M[c]), `M[${c}]`);
    }
  }
});
