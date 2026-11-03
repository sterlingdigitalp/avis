import test from 'node:test';
import assert from 'node:assert/strict';
import { createFcGovernor, latencyTicks } from '../../src/physics/governor.js';
import { makeWing } from '../../src/physics/kinematics.js';
import { createStrip, suctionEtaAt, suctionTableFrom, SUCTION } from '../../src/physics/strip.js';
import { createFlight, quatFromEuler, crossY, crossZ } from '../../src/physics/flight.js';
import { createController } from '../../src/physics/control.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

test('FC governor with the 20 ms control delay: the ESC applies the duty computed ceil(delay x rate) ticks earlier', () => {
  // 20 ms at 960 Hz is 19.2 periods: rounded up, never under the delay; at least one period with no delay stated.
  assert.equal(latencyTicks(960, 0.02), 20);
  assert.equal(latencyTicks(500, 0.02), 10);
  assert.equal(latencyTicks(960, 0), 1);
  assert.equal(latencyTicks(960), 1);
  const gov = { rate_Hz: 960, encoder: { bits: 14, window: 2 }, voltage: { lsb_V: 0, tau_s: 0 }, clamp_A: 1e9 };
  const kv = 90, R = 0.05, G = 25, T = 1 / 960;
  const g = createFcGovernor(gov, { G, kv, R, cells: 2, delay_s: 0.02 });
  assert.equal(g.lag, 20);
  // An accelerating crank, so that a stale encoder sample extrapolates differently from a fresh one.
  const ph = (t) => 20 * t + 300 * t * t;
  g.reset({ t: 0, phase: 0, crank: 20, V: 8 });
  const pend = [], samp = [];
  for (let k = 0; k <= 60; k++) {
    const t = k * T;
    g.tick(t, ph(t), 8, () => 22 + 0.05 * k, false, true);
    pend.push(g.st.pending);
    samp.push({ a: g.st.angle, r: g.st.rate, t: g.st.tAngle });
    // A fresh start holds the first duty for the whole latency (the same command sent before the flight began).
    const src = Math.max(0, k - 20);
    assert.equal(g.st.applied, pend[src], `tick ${k}`);
    // The twist schedule's phase comes from the tick whose duty is in effect: its sample advanced at its speed.
    const s = samp[src];
    near(g.phaseAt(t + 0.0004), s.a + s.r * (t + 0.0004 - s.t), 1e-12, `phase at tick ${k}`);
  }
  // With an accelerating crank the delayed estimate lags the latest sample's: the delay is really there.
  assert.ok(Math.abs(g.phaseAt(60 * T) - (samp[60].a)) > 1e-3);
});

test('strip root moment: moment transfer from the body origin to each shoulder holds with the hinge off every axis', () => {
  // Statics, independent of the force model: M_O = sum over wings of (M_hinge + H x F), H the shoulder point
  // (x, y, +z right, -z left). The hinge is put off the origin in x as well, which no design so far has done.
  const spec = { ...testDesign().wing, hinge: [0.031, 0.022, 0.027], spar_x: 0.012 };
  const m = createStrip(makeWing(spec), { n: 8 });
  let r;
  for (let k = 0; k < 40; k++) {
    const t = k * 0.002;
    const jR = { phi: 0.3 * Math.sin(12 * t), dphi: 3.6 * Math.cos(12 * t), ddphi: -43 * Math.sin(12 * t), theta: 0.1, dtheta: 0.5, ddtheta: 0 };
    const jL = { phi: 0.25 * Math.sin(12 * t + 0.3), dphi: 3 * Math.cos(12 * t + 0.3), ddphi: -36 * Math.sin(12 * t + 0.3), theta: -0.05, dtheta: 0.2, ddtheta: 0 };
    r = m.evaluate([9, -0.8, 0.9], [0.2, -0.15, 0.1], [0, 0, 0], jR, jL, 1, 1.8e-5, 0.002);
  }
  const N = r.fx.length, n = N / 2;
  const F = [[0, 0, 0], [0, 0, 0]];
  for (let q = 0; q < N; q++) {
    const w = q < n ? 0 : 1;
    F[w][0] += r.fx[q]; F[w][1] += r.fy[q]; F[w][2] += r.fz[q];
  }
  // The apparent-mass and circulatory parts are both present, so every arm in the sum is exercised.
  assert.ok(Math.abs(r.fAm[2]) > 1e-3 && Math.abs(r.fCirc[2]) > 1e-3);
  const H = [[spec.hinge[0], spec.hinge[1], spec.hinge[2]], [spec.hinge[0], spec.hinge[1], -spec.hinge[2]]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const hR = cross(H[0], F[0]), hL = cross(H[1], F[1]);
  const scale = Math.hypot(...r.M) + 1;
  for (let a = 0; a < 3; a++) near(r.M[a], r.rootMoment[a] + r.rootMoment[3 + a] + hR[a] + hL[a], 1e-9 * scale, `axis ${a}`);
  // The hinge's x enters: about the origin the two moments differ by H x F, and H x F has an x-arm part.
  assert.ok(Math.abs(spec.hinge[0] * F[0][2]) > 1e-4);
});

test('shoulder root loads do not depend on where the shoulder sits along the body: a hinge moved in x carries the same loads', () => {
  // A wing translated with its shoulder, on a body too heavy to move, in uniform air: same flow, same motion, same
  // loads about its own shoulder (air and inertia). Pins the hinge origin of the root moment in x (flight.js armX).
  const run = (hx) => {
    const d = testDesign();
    d.wing.hinge = [hx, d.wing.hinge[1], d.wing.hinge[2]];
    // A lump off the spar axis on a twisting hand wing gives the inertial loads an x part, so every arm counts.
    d.mass.wing.point = [{ s_m: 0.6, mass_kg: 0.05, d_m: 0.05 }];
    d.mass.lumps.push({ name: 'test ballast', mass_kg: 1e5, pos: [0, 0, 0], gyration_m: 1 });
    d.structure = { joints: [{ name: 'shoulder', supports: [{ name: 's', radial: [{ ch: 'root_yz_R', k: 1 }], axial: [] }] }] };
    const sim = createFlight(d, { dt: 1 / 960, gravity: 0 });
    sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.04, 0), vel: [9, 0, 0], phase: 0, crank: 2 * TWO_PI, amp: 18 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
    Object.assign(sim.cmd, { freq: 2, amp: 18 * DEG, dyn: 0.4, sym: 0, tailPitch: 0 });
    for (let k = 0; k < 480; k++) sim.step();
    sim.resetPeaks();
    for (let k = 0; k < 960; k++) sim.step();
    return sim.peaks.channels;
  };
  const a = run(0), b = run(0.05);
  for (const k of ['root_yz_R', 'root_x_R', 'root_mperp_R', 'root_yz_L', 'root_x_L', 'root_mperp_L']) {
    assert.ok(a[k] > 1e-3, `${k} loaded`);
    near(b[k] / a[k], 1, 2e-3, `${k} hinge x`);
  }
});

const ctlCfg = (lsb) => {
  const tr = { V: 12, theta: 0.02, freq: 2.2, tail: 0, amp: 0.5, sym: 0, dyn: 0.4 };
  return {
    trim: [tr, { ...tr, V: 14, freq: 2.5 }], Vcruise: 12, Vmin: 9.5, kH: 0.5, kGamma: 1, kIGamma: 0.15, kTheta: 0.2, kQ: 0, kFc: 0.36, kFcFf: 0.5, kIF: 0.08, kFv: 0.3,
    kPhi: 0.3, kP: 0, kR: 0.02, kChi: 1, maxBank: 0.6, diffMax: 0.3, climbMax: 2, sinkMax: 2.5, freqMin: 1, ampMax: 0.7, ampNominal: 0.5, kStallPitch: 0.08,
    tecs: { Vfloor: 9.5, Vmax: 15, kV: 0.4, band: 1.5 }, currentLimit_A: 36, ...(lsb === undefined ? {} : { currentSenseLsb_A: lsb }),
  };
};
const senseRun = (lsb, battI, { noise = false, steps = 1, seed = 3 } = {}) => {
  const c = createController({ control: ctlCfg(lsb) }, { sensorNoise: noise, delay: 0, seed });
  const truth = { t: 0, dt: 0.01, omega: [0, 0, 0], sf: [0, 9.80665, 0], up: [0, 1, 0], y: 0, vel: [12, 0, 0], pos: [0, 0, 0], airspeedX: 12, phase: 0, crank: 2.2 * TWO_PI, heading: 0, battI };
  c.initAttitude(truth);
  const out = [];
  for (let k = 0; k < steps; k++) { c.update({ ...truth, t: k * 0.01 }, k * 0.01); out.push(c.sensedAmps); }
  return out;
};

test('flight controller current sense: readings at the ADC resolution, raw without one, noise of 0.1 A rms', () => {
  // 40 A/V over 3.3 V at 12 bits is 0.03223 A per count; 10.017 A is 310.8 counts, read as 311.
  near(senseRun(0.03223, 10.017)[0], 311 * 0.03223, 1e-12, 'quantised');
  near(senseRun(1.0, 10.4)[0], 10, 1e-12, 'one-amp steps');
  // No resolution stated (or zero): the reading is the true current.
  assert.equal(senseRun(0, 10.017)[0], 10.017);
  assert.equal(senseRun(undefined, 10.017)[0], 10.017);
  // With sensor noise on: zero-mean noise of 0.1 A rms before the converter (fine steps here).
  const r = senseRun(0.0001, 10, { noise: true, steps: 4000 });
  const mean = r.reduce((s, v) => s + v, 0) / r.length;
  const sd = Math.sqrt(r.reduce((s, v) => s + (v - mean) ** 2, 0) / r.length);
  near(mean, 10, 0.01, 'mean');
  near(sd, 0.1, 0.01, 'rms');
  // Coarse steps quantise the noisy reading too: every reading is a whole number of counts.
  for (const v of senseRun(0.5, 10.1, { noise: true, steps: 50 })) near(v / 0.5, Math.round(v / 0.5), 1e-9, 'counts');
});

// SPEC ruling after cfd round 3: beyond the largest amplitude a lowering last slope continues (floored at 0), a raising
// one is held; below the smallest, held as before.
test('eta_s table: linear in the effective-angle amplitude between points, held below, extrapolated downward only', () => {
  const t = [[0.4, 0.1], [0.2, 0.3]]; // any order
  near(suctionEtaAt(t, 0.3), 0.2, 1e-12);
  near(suctionEtaAt(t, 0.25), 0.25, 1e-12);
  assert.equal(suctionEtaAt(t, 0.1), 0.3);
  near(suctionEtaAt(t, 0.45), 0.05, 1e-12); // slope -1 per rad continued
  assert.equal(suctionEtaAt(t, 0.9), 0); // floored at 0
  assert.equal(suctionEtaAt([[0.2, 0.1], [0.4, 0.3]], 0.9), 0.3); // a raising last segment is held
  assert.equal(suctionEtaAt([[0.3, 0.5]], 0.1), 0.5);
  assert.throws(() => suctionEtaAt([], 0.2));
});

// SPEC ruling after cfd round 3 replaced this test's rule (every complete point, ties averaged): only settled: true
// points enter, no verdict means out, and two settled points at one amplitude are an error.
test('eta_s table from the cfd file: settled points only, pure plunge from atan(2 k h/c), no verdict out, ties an error', () => {
  const json = {
    points: [
      { name: 'plunge', complete: true, settled: true, k: 0.35, h_over_c: 0.75, eta_s_strip: 0.136 },
      { name: 'stated', complete: true, settled: true, k: 0.3, h_over_c: 0.5, twist_amplitude_deg: 10, alpha_eff_amplitude_deg: 15, eta_s_strip: 0.3 },
      { name: 'no-verdict', complete: true, alpha_eff_amplitude_deg: 12, eta_s_strip: 0.5 },
      { name: 'twist-no-amplitude', complete: true, settled: true, k: 0.3, h_over_c: 0.5, twist_amplitude_deg: 10, eta_s_strip: 0.2 },
      { name: 'unfinished', complete: false, settled: true, k: 0.3, h_over_c: 0.5, eta_s_strip: 0.9 },
    ],
  };
  const r = suctionTableFrom(json);
  assert.deepEqual(r.skipped.map((s) => s.name), ['no-verdict', 'twist-no-amplitude', 'unfinished']);
  assert.equal(r.table.length, 2);
  near(r.table[0][0], 15 * DEG, 1e-12);
  near(r.table[0][1], 0.3, 1e-12);
  // hdot_max / U = h0 omega / U = 2 k h0 / c with k = omega c / (2 U): 0.525, i.e. 27.7 deg.
  near(r.table[1][0], Math.atan(0.525), 1e-12);
  near(r.table[1][0] / DEG, 27.7, 0.05);
  near(r.table[1][1], 0.136, 1e-12);
  assert.throws(() => suctionTableFrom({ points: [json.points[1], { ...json.points[1], name: 'stated-2' }] }), /two settled eta_s points/);
});

test('strip model with an eta_s table: each strip takes the table at its amplitude; a flat table equals the constant', () => {
  const wing = makeWing(testDesign().wing);
  const dflt = createStrip(wing, { n: 6 });
  assert.equal(dflt.suctionEtaOf(0), SUCTION.eta);
  const table = [[10 * DEG, 0.5], [30 * DEG, 0.1]];
  const amp = [5, 10, 15, 20, 30, 40].map((a) => a * DEG);
  const m = createStrip(wing, { n: 6, suction: { table, amplitude: amp } });
  // 40 deg lies beyond the last point: its lowering slope (-0.4 per 20 deg) continues, to -0.1, floored at 0 (SPEC
  // ruling after cfd round 3; before it, the last value was held).
  [0.5, 0.5, 0.4, 0.3, 0.1, 0].forEach((e, i) => { near(m.suctionEtaOf(i), e, 1e-12, `right ${i}`); near(m.suctionEtaOf(i + 6), e, 1e-12, `left ${i}`); });
  assert.throws(() => createStrip(wing, { n: 6, suction: { table, amplitude: [0.1, 0.2] } }));
  // Inviscid keeps full suction whatever is passed.
  assert.equal(createStrip(wing, { n: 6, inviscid: true, suction: { table, amplitude: amp } }).suctionEtaOf(3), 1);
  // A flat table at eta is the constant eta: identical forces on an identical flapping history.
  const fly = (s) => {
    let r;
    for (let k = 0; k < 60; k++) {
      const t = k * 0.002;
      const j = { phi: 0.4 * Math.sin(14 * t), dphi: 5.6 * Math.cos(14 * t), ddphi: -78 * Math.sin(14 * t), theta: 0.2 * Math.cos(14 * t), dtheta: -2.8 * Math.sin(14 * t), ddtheta: 0 };
      r = s.evaluate([8, -0.5, 0], [0, 0, 0], [0, 0, 0], j, j, 1, 1.8e-5, 0.002);
    }
    return [...r.F, ...r.M];
  };
  const a = fly(createStrip(wing, { n: 6, suctionEta: 0.3 }));
  const b = fly(createStrip(wing, { n: 6, suction: { table: [[0.1, 0.3], [0.6, 0.3]], amplitude: amp } }));
  a.forEach((v, i) => assert.equal(b[i], v));
  // A lower eta_s takes thrust away (or leaves it) on the same history, never adds it.
  const lo = fly(createStrip(wing, { n: 6, suctionEta: 0.1 }));
  assert.ok(lo[0] <= a[0] + 1e-12, `${lo[0]} vs ${a[0]}`);
});

test('members: beam statics by hand, overhung and between supports; the moment bound sums magnitudes', async () => {
  const { unitMoment, shaftMomentSections } = await import('../../src/physics/members.js');
  // Supports at 0 and 0.1 m. A unit load at mid-span: 0.025 N m at the centre; at a support, zero.
  near(Math.abs(unitMoment(0, 0.1, 0.05, 0.05)), 0.025, 1e-15);
  near(unitMoment(0, 0.1, 0.05, 0), 0, 1e-15);
  near(unitMoment(0, 0.1, 0.05, 0.1), 0, 1e-15);
  // Overhung 0.02 m behind the first support: the moment at that support is the load times the overhang.
  near(Math.abs(unitMoment(0, 0.1, -0.02, 0)), 0.02, 1e-15);
  near(Math.abs(unitMoment(0, 0.1, -0.02, 0.05)), 0.01, 1e-15);
  // Two channels at their planes: the section at the first support takes 0.02 x 3 (overhung) only.
  const s = shaftMomentSections(0, 0.1, [{ ch: 'coupler_R', perUnit: 3, x: -0.02 }, { ch: 'crank_torque', perUnit: 10, x: 0.05 }]);
  const at0 = s.find((q) => q.x_m === 0);
  assert.deepEqual(at0.radial, [{ ch: 'coupler_R', k: 0.06 }]);
  const mid = s.find((q) => q.x_m === 0.05);
  near(mid.radial.find((t) => t.ch === 'crank_torque').k, 0.25, 1e-12);
  near(mid.radial.find((t) => t.ch === 'coupler_R').k, 0.03, 1e-12);
});

test('members: round shaft von Mises, pin shear, seat bearing, bond and key worked by hand', async () => {
  const { roundMemberCheck, pinShearCheck, seatCheck, lugCheck, webCheck, bondCheck, keyCheck } = await import('../../src/physics/members.js');
  // 8 mm solid: Z = pi d^3 / 32 = 50.27 mm3, Zp = 100.53 mm3, A = 50.27 mm2.
  const r = roundMemberCheck({ d_mm: 8 }, { yield_MPa: 1000 }, { M: 10, T: 5, V: 0 });
  near(r.sigma_MPa, 10 / ((Math.PI * 0.008 ** 3) / 32) / 1e6, 1e-9);
  near(r.tau_torsion_MPa, 5 / ((Math.PI * 0.008 ** 3) / 16) / 1e6, 1e-9);
  near(r.von_mises_MPa, Math.sqrt(198.944 ** 2 + 3 * 49.736 ** 2), 0.01);
  assert.equal(r.governs, 'outer fibre');
  near(r.fos, 1000 / r.von_mises_MPa, 1e-12);
  // Transverse shear: 4/3 V / A solid; 2 V / A hollow; at the neutral axis it adds to the torsion, without bending.
  const sv = roundMemberCheck({ d_mm: 8 }, { yield_MPa: 1 }, { V: 100 });
  near(sv.tau_shear_MPa, (4 / 3) * 100 / 50.265, 1e-3);
  near(sv.von_mises_MPa, Math.sqrt(3) * sv.tau_shear_MPa, 1e-9);
  assert.equal(sv.governs, 'neutral axis');
  near(roundMemberCheck({ d_mm: 8, di_mm: 6 }, { yield_MPa: 1 }, { V: 100 }).tau_shear_MPa, 2 * 100 / ((Math.PI / 4) * 28), 1e-3);
  const mix = roundMemberCheck({ d_mm: 8 }, { yield_MPa: 1 }, { M: 1, T: 2, V: 3000 });
  near(mix.von_mises_MPa, Math.max(Math.sqrt(mix.sigma_MPa ** 2 + 3 * mix.tau_torsion_MPa ** 2), Math.sqrt(3) * (mix.tau_torsion_MPa + mix.tau_shear_MPa)), 1e-9);
  // Pin: 6 mm in double shear, 1000 N: 17.68 MPa; shear yield from von Mises when none is published.
  const p = pinShearCheck({ d_mm: 6 }, { yield_MPa: 866 }, 1000, 2);
  near(p.tau_MPa, 1000 / (2 * 28.274), 1e-3);
  near(p.shear_allowable_MPa, 866 / Math.sqrt(3), 1e-9);
  // Seat: 1000 N on a 10 mm bushing in 2 mm of plate: 50 MPa; the published bearing yield only at its e/D. Round 9
  // (ruling after round 8): below it, unrated here (no EN standard named on this material; the EN case is in
  // round9.test.js);
  // no stated edge distance, or nothing published, no rating; a crushed round part takes the yield.
  const al = { yield_MPa: 138, bearing_yield_MPa: 386, bearing_e_over_d_min: 2 };
  near(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 2 }, al, 1000).fos, 386 / 50, 1e-9);
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 1.9 }, al, 1000).fos, null);
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2 }, al, 1000).fos, null);
  near(seatCheck({ d_mm: 10, t_mm: 2, crush: true }, al, 1000).fos, 138 / 50, 1e-9);
  assert.equal(seatCheck({ d_mm: 10, t_mm: 2, e_over_d: 2 }, {}, 1000).ok, false);
  // A pin cantilevered from its seat adds the edge pressure 6 M / (d t^2): 1 N m on 8 mm in 8 mm is 11.72 MPa.
  near(seatCheck({ d_mm: 8, t_mm: 8 }, al, 0, 1).p_MPa, 6 / (0.008 * 0.008 ** 2) / 1e6, 1e-9);
  // Ring lug 10 mm hole, 16 mm outside, 6 mm thick, 1000 N: bearing 16.7 MPa, net tension 27.8 MPa, shear-out 27.8 MPa.
  const lug = lugCheck({ d_mm: 10, D_mm: 16, t_mm: 6 }, { yield_MPa: 138, shear_yield_MPa: 103.5, bearing_yield_MPa: 386, bearing_e_over_d_min: 2 }, 1000);
  near(lug.bearing_MPa, 1000 / 60, 1e-9);
  assert.equal(lug.bearing_allowable_MPa, null); // round 9: e/D = 0.8 is below 1.2 d0, so the lug is unrated
  assert.equal(lug.fos, null);
  assert.equal(lug.ok, false);
  near(lug.net_tension_MPa, 1000 / 36, 1e-9);
  near(lug.fos_shear_out, 103.5 / (1000 / 36), 1e-9);
  // Web 14 x 5 mm: 10 N m out of plane (6M / (b h^2) = 171.4 MPa) plus 1000 N at 12 mm in plane (73.5 MPa).
  near(webCheck({ b_mm: 14, h_mm: 5, lever_mm: 12 }, { yield_MPa: 827 }, { M: 10, F: 1000 }).sigma_MPa, 6 * 10 / (0.014 * 0.005 ** 2) / 1e6 + 6 * 12 / (0.005 * 0.014 ** 2) / 1e6, 1e-9);
  // Bond: 1 N m on 8 mm x 10 mm: F = 250 N over 251.3 mm2.
  near(bondCheck({ d_mm: 8, L_mm: 10 }, { shear_MPa: 10 }, { T: 1 }).tau_MPa, 250 / 251.327, 1e-4);
  // Key 3 x 3 on an 8 mm shaft, 10 mm long, 1.4 mm in the hub: 5 N m is 1250 N; 41.7 MPa shear, 89.3 MPa bearing.
  const k = keyCheck({ d_mm: 8, b_mm: 3, h_mm: 1.4, L_mm: 10 }, { yield_MPa: 600 }, { yield_MPa: 400 }, 5);
  near(k.tau_MPa, 1250 / 30, 1e-6);
  near(k.bearing_MPa, 1250 / 14, 1e-6);
  near(k.fos_bearing, 400 / (1250 / 14), 1e-9);
});

test('inertial root moment: the y and z components of r x F, by hand', () => {
  // r = (2, 3, 5), F = (7, 11, 13): r x F = (3*13 - 5*11, 5*7 - 2*13, 2*11 - 3*7) = (-16, 9, 1).
  assert.equal(crossY(5, 2, 7, 13), 9);
  assert.equal(crossZ(2, 3, 7, 11), 1);
});

test('members: sections, checks and dispatch by hand, every branch', async () => {
  const M = await import('../../src/physics/members.js');
  // Hollow 10/6 mm: A = pi/4 (100 - 36) mm2, I = pi/64 (10^4 - 6^4) mm4, J = 2I, Z = I / 5 mm, Zp = 2I / 5 mm.
  const h = M.roundSection(0.01, 0.006);
  near(h.A, (Math.PI / 4) * 64e-6, 1e-15);
  near(h.I, (Math.PI / 64) * (1e-8 - 1.296e-9), 1e-20);
  near(h.J, 2 * h.I, 1e-20);
  near(h.Zp, (2 * h.I) / 0.005, 1e-18);
  near(M.roundSection(0.008).A, (Math.PI / 4) * 64e-6, 1e-15);
  // A beam on supports at 0.02 and 0.12 m (not at the origin): unit load at 0.07 m gives 0.025 N m there.
  near(Math.abs(M.unitMoment(0.02, 0.12, 0.07, 0.07)), 0.025, 1e-15);
  near(Math.abs(M.unitMoment(0.02, 0.12, 0.1, 0.07)), 0.2 * 0.05, 1e-15);
  // Sections come in x order whatever the load order, named in mm to 0.1.
  // (The free end of an overhang and a support with nothing beyond it carry no moment and drop out.)
  const ss = M.shaftMomentSections(0.02, 0.12, [{ ch: 'coupler_R', perUnit: 1, x: 0.1 }, { ch: 'crank_torque', perUnit: 1, x: 0.05 }, { ch: 'coupler_L', perUnit: 1, x: -0.01 }]);
  assert.deepEqual(ss.map((q) => q.x_m), [0.02, 0.05, 0.1]);
  assert.equal(ss[0].name, 'M at x 20.0 mm');
  // Only bending given: no shear; a small load still gives a finite factor; a thin-wall tube under 1 mm bore takes 2 V / A.
  const only = M.roundMemberCheck({ d_mm: 8 }, { yield_MPa: 100 }, { M: 0.001 });
  assert.equal(only.tau_shear_MPa, 0);
  assert.equal(only.tau_torsion_MPa, 0);
  near(only.fos, 100 / only.von_mises_MPa, 1e-9);
  assert.ok(Number.isFinite(only.fos) && only.von_mises_MPa < 1);
  near(M.roundMemberCheck({ d_mm: 8, di_mm: 0.5 }, { yield_MPa: 1 }, { V: 10 }).tau_shear_MPa, 2 * 10 / ((Math.PI / 4) * (64 - 0.25)), 1e-9);
  assert.equal(M.roundMemberCheck({ d_mm: 8 }, { yield_MPa: 1 }, {}).fos, Infinity);
  // Pin: single shear by default; shear yield from von Mises when none is published; factor by hand.
  const ps = M.pinShearCheck({ d_mm: 6 }, { yield_MPa: 866 }, 1000);
  near(ps.tau_MPa, 1000 / 28.274334, 1e-4);
  near(ps.fos, (866 / Math.sqrt(3)) / ps.tau_MPa, 1e-9);
  assert.equal(M.pinShearCheck({ d_mm: 6 }, { yield_MPa: 866 }, 0).fos, Infinity);
  // Lug without a published shear yield: von Mises shear; each factor by hand; no strength at all: no rating.
  const al = { yield_MPa: 138, bearing_yield_MPa: 386, bearing_e_over_d_min: 2 };
  const lg = M.lugCheck({ d_mm: 10, D_mm: 16, t_mm: 6 }, al, 1000);
  near(lg.fos_tension, 138 / (1000 / 36), 1e-9);
  near(lg.fos_shear_out, (138 / Math.sqrt(3)) / (1000 / 36), 1e-9);
  assert.equal(lg.fos_bearing, null); // round 9: e/D 0.8 < 1.2, unrated
  assert.equal(lg.fos, null);
  // e/D 1.5 (a 30 mm ring) with no EN standard on this material: unrated below the source's e/D 2.
  assert.equal(M.lugCheck({ d_mm: 10, D_mm: 30, t_mm: 6 }, al, 1000).fos, null);
  assert.equal(M.lugCheck({ d_mm: 10, D_mm: 40, t_mm: 6 }, al, 0).fos, Infinity);
  // A big ring reaches e/D 2: the published bearing yield applies.
  near(M.lugCheck({ d_mm: 10, D_mm: 40, t_mm: 6 }, al, 1000).fos_bearing, 386 / (1000 / 60), 1e-9);
  // Web: only M given; only F given; factor by hand.
  const wb = M.webCheck({ b_mm: 14, h_mm: 5, lever_mm: 12 }, { yield_MPa: 827 }, { M: 10 });
  near(wb.fos, 827 / wb.sigma_MPa, 1e-9);
  near(M.webCheck({ b_mm: 14, h_mm: 5, lever_mm: 12 }, { yield_MPa: 827 }, { F: 1000 }).sigma_MPa, 6 * 1000 * 0.012 / (0.005 * 0.014 ** 2) / 1e6, 1e-9);
  assert.equal(M.webCheck({ b_mm: 14, h_mm: 5, lever_mm: 12 }, { yield_MPa: 827 }, {}).fos, Infinity);
  // Bond: 2 N m and 100 N on 8 x 10 mm: hypot(500, 100) N over 251.3 mm2; a published factor scales the allowable.
  const bd = M.bondCheck({ d_mm: 8, L_mm: 10 }, { shear_MPa: 10, factor: 0.5 }, { T: 2, F: 100 });
  near(bd.tau_MPa, Math.hypot(500, 100) / 251.327412, 1e-6);
  near(bd.shear_allowable_MPa, 5, 1e-12);
  near(bd.fos, 5 / bd.tau_MPa, 1e-9);
  near(M.bondCheck({ d_mm: 8, L_mm: 10 }, { shear_MPa: 10 }, { T: 2 }).shear_allowable_MPa, 10, 1e-12);
  // Key: von Mises shear yield; a side with no published strength gives no bearing capacity.
  const ky = M.keyCheck({ d_mm: 8, b_mm: 3, h_mm: 1.8, L_mm: 10 }, { yield_MPa: 827 }, { yield_MPa: 251.5 }, 5);
  near(ky.fos_shear, (827 / Math.sqrt(3)) / (1250 / 30), 1e-9);
  near(ky.fos_bearing, 251.5 / (1250 / 18), 1e-9);
  assert.equal(M.keyCheck({ d_mm: 8, b_mm: 3, h_mm: 1.8, L_mm: 10 }, { yield_MPa: 827 }, {}, 5).fos_bearing, 0);
  // memberPeaks: the members' sections follow the joints' supports in order; a missing peak reads 0.
  const mem = [{ name: 'a', sections: [{ name: 'M' }, { name: 'V' }] }, { name: 'b', sections: [{ name: 'F' }] }];
  assert.deepEqual(M.memberPeaks(mem, [{ Fr: 9 }, { Fr: 1 }, { Fr: 2 }, { Fr: 3 }], 1), { 'a/M': 1, 'a/V': 2, 'b/F': 3 });
  assert.deepEqual(M.memberPeaks(mem, [{ Fr: 1 }], 0), { 'a/M': 1, 'a/V': 0, 'b/F': 0 });
  assert.deepEqual(M.memberPeaks(null, [], 0), {});
  // checkMembers: each check named by its member, its loads the largest of the sections it uses.
  const ti = { id: 'ti', yield_MPa: 827 };
  const rows = M.checkMembers([
    { name: 'r', check: 'round', material: ti, dims: { d_mm: 8 }, uses: { M: ['m1', 'm2'], T: 't' } },
    { name: 'p', check: 'pin-shear', material: ti, dims: { d_mm: 6, planes: 2 }, uses: { F: 'f' } },
    { name: 'p1', check: 'pin-shear', material: ti, dims: { d_mm: 6 }, uses: { F: 'f' } },
    { name: 's', check: 'seat', material: al, dims: { d_mm: 8, t_mm: 8, e_over_d: 2 }, uses: { F: 'f', M: 'm' } },
    { name: 'l', check: 'lug', material: al, dims: { d_mm: 10, D_mm: 40, t_mm: 6 }, uses: { F: 'f' } },
    { name: 'w', check: 'web', material: ti, dims: { b_mm: 14, h_mm: 5, lever_mm: 12 }, uses: { M: 'm', F: 'f' } },
    { name: 'b', check: 'bond', material: { shear_MPa: 10 }, dims: { d_mm: 8, L_mm: 10 }, uses: { T: 't', F: 'f' } },
    { name: 'k', check: 'key', material: ti, side_material: al, dims: { d_mm: 8, b_mm: 3, h_mm: 1.8, L_mm: 10 }, uses: { T: 't' }, note: 'n' },
  ], { 'r/m1': 3, 'r/m2': 5, 'r/t': 2, 'p/f': 1000, 'p1/f': 1000, 's/f': 1000, 's/m': 1, 'l/f': 1000, 'w/m': 10, 'w/f': 1000, 'b/t': 2, 'b/f': 100, 'k/t': 5 });
  const by = Object.fromEntries(rows.members.map((r) => [r.name, r]));
  assert.deepEqual(by.r.load, { M: 5, T: 2, V: 0, F: 0 });
  near(by.r.fos, M.roundMemberCheck({ d_mm: 8 }, ti, { M: 5, T: 2 }).fos, 1e-12);
  near(by.p.fos, M.pinShearCheck({ d_mm: 6 }, ti, 1000, 2).fos, 1e-12);
  near(by.p1.fos, M.pinShearCheck({ d_mm: 6 }, ti, 1000, 1).fos, 1e-12);
  near(by.s.fos, M.seatCheck({ d_mm: 8, t_mm: 8, e_over_d: 2 }, al, 1000, 1).fos, 1e-12);
  near(by.l.fos, M.lugCheck({ d_mm: 10, D_mm: 40, t_mm: 6 }, al, 1000).fos, 1e-12);
  near(by.w.fos, M.webCheck({ b_mm: 14, h_mm: 5, lever_mm: 12 }, ti, { M: 10, F: 1000 }).fos, 1e-12);
  near(by.b.fos, M.bondCheck({ d_mm: 8, L_mm: 10 }, { shear_MPa: 10 }, { T: 2, F: 100 }).fos, 1e-12);
  near(by.k.fos, M.keyCheck({ d_mm: 8, b_mm: 3, h_mm: 1.8, L_mm: 10 }, ti, al, 5).fos, 1e-12);
  assert.equal(by.k.note, 'n');
  assert.equal(by.r.material, 'ti');
  const worst = rows.members.reduce((a, b) => (b.fos < a.fos ? b : a));
  assert.equal(rows.worst, worst.name);
  near(rows.fos_min, worst.fos, 1e-12);
  assert.equal(rows.ok, rows.members.every((r) => r.ok));
  // An unknown check is no rating; nothing rated gives no minimum.
  const bad = M.checkMembers([{ name: 'x', check: 'glue', material: null, dims: {} }], {});
  assert.equal(bad.ok, false);
  assert.equal(bad.fos_min, null);
  assert.equal(bad.worst, null);
  assert.equal(bad.members[0].material, null);
  assert.equal(M.checkMembers(null, {}).ok, true);
});

test('beam statics with a forward overhang: the far support reaction counts beyond it', async () => {
  const { unitMoment } = await import('../../src/physics/members.js');
  // Supports at 0.02 and 0.12 m, unit load at 0.15 m: at 0.13 m the moment is the load times its 0.02 m arm.
  near(Math.abs(unitMoment(0.02, 0.12, 0.15, 0.13)), 0.02, 1e-15);
  near(Math.abs(unitMoment(0.02, 0.12, 0.15, 0.12)), 0.03, 1e-15);
});

test('eta_s reaches the flight model: a design table per strip, a perturbation constant over it; a binding eta matters', async () => {
  const { buildBird } = await import('../../src/physics/flight.js');
  const d = testDesign();
  assert.equal(buildBird(d).strip.suctionEtaOf(0), SUCTION.eta);
  const n = 8;
  d.suction = { table: [[0.05, 0.2], [0.2, 0.4]], amplitude_rad: Array.from({ length: n }, (_, i) => 0.05 + (0.15 * i) / (n - 1)) };
  const b = buildBird(d);
  near(b.strip.suctionEtaOf(0), 0.2, 1e-12);
  near(b.strip.suctionEtaOf(n - 1), 0.4, 1e-12);
  near(b.strip.suctionEtaOf(2 * n - 1), 0.4, 1e-12);
  assert.equal(buildBird(d, { perturb: { suctionEta: 0.15 } }).strip.suctionEtaOf(3), 0.15);
  assert.equal(buildBird(d, { suctionEta: 0.25 }).strip.suctionEtaOf(3), 0.25);
  // On a hard flapping history a low eta_s binds: a flat table at 0.05 is the constant 0.05, and both differ from 1.
  const wing = makeWing(testDesign().wing);
  const amp = Array.from({ length: 6 }, () => 0.2);
  const fly = (s) => {
    let r;
    for (let k = 0; k < 80; k++) {
      const t = k * 0.002;
      const j = { phi: 0.5 * Math.sin(20 * t), dphi: 10 * Math.cos(20 * t), ddphi: -200 * Math.sin(20 * t), theta: 0, dtheta: 0, ddtheta: 0 };
      r = s.evaluate([6, -0.3, 0], [0, 0, 0], [0, 0, 0], j, j, 1, 1.8e-5, 0.002);
    }
    return r.F[0];
  };
  const c05 = fly(createStrip(wing, { n: 6, suctionEta: 0.05 }));
  const t05 = fly(createStrip(wing, { n: 6, suction: { table: [[0.1, 0.05], [0.3, 0.05]], amplitude: amp } }));
  const c1 = fly(createStrip(wing, { n: 6, suctionEta: 1 }));
  assert.equal(t05, c05);
  assert.ok(Math.abs(c1 - c05) > 1e-3 * Math.abs(c1), `${c1} vs ${c05}`);
});
