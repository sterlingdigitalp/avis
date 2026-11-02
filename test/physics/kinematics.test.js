import test from 'node:test';
import assert from 'node:assert/strict';
import { crankRockerSolve, crankRockerLimits, rockerForAmplitude, isCrankRocker, makeFlapDrive, strokeAngle, linkageCycleStats, makeWing, makeStations, makeSectionState, sectionStates, jointState, wingGrid, makeLatticeSpec, wingArea, mac } from '../../src/physics/kinematics.js';

const link = { a: 0.01, b: 0.07, c: 0.03, d: 0.07 };

test('crank-rocker satisfies loop closure at every crank angle', () => {
  assert.ok(isCrankRocker(link));
  for (let k = 0; k < 36; k++) {
    const th = (k / 36) * 2 * Math.PI;
    const r = crankRockerSolve(link, th);
    const A = [link.a * Math.cos(th), link.a * Math.sin(th)];
    const B = [link.d + link.c * Math.cos(r.psi), link.c * Math.sin(r.psi)];
    assert.ok(Math.abs(Math.hypot(B[0] - A[0], B[1] - A[1]) - link.b) < 1e-12);
  }
});

test('velocity ratio and its derivative match central differences of position', () => {
  const h = 1e-6;
  for (const th of [0.3, 1.7, 3.0, 4.4, 5.9]) {
    const r = crankRockerSolve(link, th);
    const p = crankRockerSolve(link, th + h), m = crankRockerSolve(link, th - h);
    assert.ok(Math.abs(r.dpsi - (p.psi - m.psi) / (2 * h)) < 1e-6);
    assert.ok(Math.abs(r.ddpsi - (p.dpsi - m.dpsi) / (2 * h)) < 1e-5);
  }
});

test('rocker extremes from the collinear triangles agree with a dense sweep', () => {
  const lim = crankRockerLimits(link);
  let lo = Infinity, hi = -Infinity;
  for (let k = 0; k < 20000; k++) {
    const r = crankRockerSolve(link, (k / 20000) * 2 * Math.PI);
    lo = Math.min(lo, r.psi);
    hi = Math.max(hi, r.psi);
  }
  assert.ok(Math.abs(lim.psiMin - lo) < 1e-6 && Math.abs(lim.psiMax - hi) < 1e-6);
  // Virtual work: ideal rocker torque / crank torque = 1 / psi'. Check energy over a small rotation.
  const r = crankRockerSolve(link, 1.0);
  const tauOut = 2.0, tauCrank = tauOut * r.dpsi;
  assert.ok(Math.abs(tauCrank * 1e-4 - tauOut * (crankRockerSolve(link, 1.0 + 0.5e-4).psi - crankRockerSolve(link, 1.0 - 0.5e-4).psi)) < 1e-9);
});

test('amplitude servo: the solved rocker gives the asked half swing, and phase 0 is the top', () => {
  const amp = 0.3;
  const c = rockerForAmplitude({ a: 0.008, b: 0.07, d: 0.07 }, amp);
  assert.ok(Math.abs(crankRockerLimits({ a: 0.008, b: 0.07, c, d: 0.07 }).swing / 2 - amp) < 1e-9);
  const drive = makeFlapDrive({ a: 0.008, b: 0.07, d: 0.07, ampMin: 0.15, ampMax: 0.45, ampNominal: amp, phiMean: 0.05 });
  const st = linkageCycleStats(drive, amp);
  assert.ok(Math.abs(st.halfSwing - amp) < 2e-3, `half swing ${st.halfSwing}`);
  const top = strokeAngle(0, amp, drive);
  assert.ok(Math.abs(top.phi - st.phiMax) < 1e-6 && top.dphi <= 1e-9);
  // Downstroke follows: phi decreasing just after phase 0
  assert.ok(strokeAngle(0.1, amp, drive).phi < top.phi);
});

const wingSpec = {
  hinge: [0, 0.02, 0.03], spar_x: 0, incidence: 0.05, washout: -0.03, camber: 0.03, camber_pos: 0.4,
  segments: [
    { length: 0.4, chord_in: 0.25, chord_out: 0.24, xle_in: 0.075, xle_out: 0.075, gain: 0, twist_in: 0, twist_out: 0.3 },
    { length: 0.4, chord_in: 0.24, chord_out: 0.13, xle_in: 0.075, xle_out: 0.03, gain: 0.5, twist_in: 0.3, twist_out: 1 },
  ],
};

test('section velocities and accelerations are the time derivatives of section positions', () => {
  const wing = makeWing(wingSpec);
  const st = makeStations(wing, 8);
  const w = 2 * Math.PI * 3;
  const law = { kind: 'sine', phiMean: 0.05 };
  const sched = { sym: 0.02, dyn: 0.2, lead: 0.3 };
  const at = (t) => {
    const s = makeSectionState(8);
    sectionStates(wing, st, jointState(w * t, w, 0, 0.3, law, sched, -1), -1, s);
    return s;
  };
  const t = 0.071, h = 1e-5;
  const s0 = at(t), sp = at(t + h), sm = at(t - h);
  for (let i = 0; i < 24; i++) {
    assert.ok(Math.abs(s0.Ad[i] - (sp.A[i] - sm.A[i]) / (2 * h)) < 1e-5);
    assert.ok(Math.abs(s0.Add[i] - (sp.A[i] - 2 * s0.A[i] + sm.A[i]) / (h * h)) < 2e-2);
    assert.ok(Math.abs(s0.td[i] - (sp.t[i] - sm.t[i]) / (2 * h)) < 1e-5);
    assert.ok(Math.abs(s0.tdd[i] - (sp.t[i] - 2 * s0.t[i] + sm.t[i]) / (h * h)) < 2e-2);
  }
  // Frames stay orthonormal, and the left wing is the mirror of the right
  const sr = makeSectionState(8);
  const sl = makeSectionState(8);
  sectionStates(wing, st, jointState(1, w, 0, 0.3, law, { dyn: 0.2 }, 1), 1, sr);
  sectionStates(wing, st, jointState(1, w, 0, 0.3, law, { dyn: 0.2 }, -1), -1, sl);
  for (let i = 0; i < 8; i++) {
    const t3 = [sr.t[3 * i], sr.t[3 * i + 1], sr.t[3 * i + 2]], b3 = [sr.b[3 * i], sr.b[3 * i + 1], sr.b[3 * i + 2]];
    assert.ok(Math.abs(Math.hypot(...t3) - 1) < 1e-12 && Math.abs(t3[0] * b3[0] + t3[1] * b3[1] + t3[2] * b3[2]) < 1e-12);
    assert.ok(Math.abs(sr.A[3 * i + 2] + sl.A[3 * i + 2]) < 1e-12 && Math.abs(sr.A[3 * i + 1] - sl.A[3 * i + 1]) < 1e-12);
  }
});

test('sign conventions: raising phi lifts the tip, positive twist puts the leading edge up', () => {
  const wing = makeWing({ ...wingSpec, incidence: 0, washout: 0 });
  const ls = makeLatticeSpec(wing, 2, 4);
  const up = wingGrid(wing, ls, { phi: 0.3, theta: 0 }, 1);
  const flat = wingGrid(wing, ls, { phi: 0, theta: 0 }, 1);
  const tip = 3 * 4; // i = 0, j = ns
  assert.ok(up[tip + 1] > flat[tip + 1]);
  const tw = wingGrid(wing, ls, { phi: 0, theta: 0.2 }, 1);
  const le = 3 * 4, te = 3 * (2 * 5 + 4);
  assert.ok(tw[le + 1] - tw[te + 1] > flat[le + 1] - flat[te + 1]);
  // Planform integrals by hand: trapezoids
  const A = 0.5 * (0.25 + 0.24) * 0.4 + 0.5 * (0.24 + 0.13) * 0.4;
  assert.ok(Math.abs(wingArea(wing) - A) < 1e-12);
  assert.ok(mac(wing) > 0.13 && mac(wing) < 0.25);
});
