import test from 'node:test';
import assert from 'node:assert/strict';
import { makeWing, jointState } from '../../src/physics/kinematics.js';
import { createStrip, clampCalibration, CALIBRATION_BOUNDS } from '../../src/physics/strip.js';
import { weissingerSlope } from '../../src/physics/gates/references.js';
import { theodorsen } from '../../src/physics/gates/references.js';

const plate = (AR) => makeWing({ hinge: [0, 0, 0], segments: [{ length: AR / 2, chord_in: 1, chord_out: 1, xle_in: 0.25, xle_out: 0.25, gain: 0, twist_in: 0, twist_out: 0 }], spar_x: 0 });
const J0 = { phi: 0, dphi: 0, ddphi: 0, theta: 0, dtheta: 0, ddtheta: 0 };

function steady(m, U, alpha, steps = 3000) {
  let r;
  for (let k = 0; k < steps; k++) r = m.evaluate([U * Math.cos(alpha), -U * Math.sin(alpha), 0], [0, 0, 0], [0, 0, 0], J0, J0, 1, 1e-5, 0.002);
  return r;
}

test('strip steady lift slope is within 5% of Weissinger lifting line, with induced drag e near 1', () => {
  for (const AR of [6, 10]) {
    const m = createStrip(plate(AR), { n: 16, inviscid: true });
    const a = 0.05, U = 10;
    const r = steady(m, U, a);
    const L = r.F[1] * Math.cos(a) + r.F[0] * Math.sin(a);
    const D = -(r.F[0] * Math.cos(a) - r.F[1] * Math.sin(a));
    const q = 0.5 * U * U * AR;
    const CLa = L / q / a;
    assert.ok(Math.abs(CLa / weissingerSlope({ AR }) - 1) < 0.05, `AR ${AR}: ${CLa}`);
    const e = (L / q) ** 2 / (Math.PI * AR * (D / q));
    assert.ok(e > 0.85 && e < 1.1, `e ${e}`);
  }
});

test('a wing plunging down gets lift up and a forward force (flapping thrust from Kutta-Joukowski)', () => {
  const m = createStrip(plate(8), { n: 8, inviscid: true });
  // Body moving down at 1 m/s through still air at 10 m/s forward
  let r;
  for (let k = 0; k < 2000; k++) r = m.evaluate([10, -1, 0], [0, 0, 0], [0, 0, 0], J0, J0, 1, 1e-5, 0.002);
  assert.ok(r.F[1] > 0, 'lift up');
  assert.ok(r.F[0] > 0, 'force towards the nose');
});

test('an updraft raises lift and a downdraft lowers it, as a gust should', () => {
  const run = (w) => {
    const m = createStrip(plate(8), { n: 8, inviscid: true });
    let r;
    for (let k = 0; k < 2000; k++) r = m.evaluate([10, 0, 0], [0, 0, 0], [0, w, 0], J0, J0, 1, 1e-5, 0.002);
    return r.F[1];
  };
  assert.ok(run(1) > run(0) && run(0) > run(-1));
});

test('Wagner lag: after a step in incidence the lift starts near half its final value (2D Jones form)', () => {
  const m = createStrip(plate(400), { n: 4, inviscid: true });
  const U = 10, dt = 0.001;
  const lift = [];
  for (let k = 0; k < 4000; k++) {
    const r = m.evaluate([U, -0.5, 0], [0, 0, 0], [0, 0, 0], J0, J0, 1, 1e-5, dt);
    lift.push(r.F[1]);
  }
  // Exclude the step's apparent-mass spike: compare a few steps after the start with the end.
  const ratio = lift[5] / lift[lift.length - 1];
  assert.ok(ratio > 0.45 && ratio < 0.65, `initial/final ${ratio}`);
  // Theodorsen's own F(k) at high k tends to 1/2: same physics the Jones form encodes
  assert.ok(Math.abs(theodorsen(5).F - 0.5) < 0.02);
});

test('calibration multipliers are clamped to their physical bounds', () => {
  const c = clampCalibration({ lift: 3, suction: 0.1, induced: 1.1 });
  assert.equal(c.lift, CALIBRATION_BOUNDS.lift[1]);
  assert.equal(c.suction, CALIBRATION_BOUNDS.suction[0]);
  assert.equal(c.induced, 1.1);
});

test('thin-airfoil mean line: NACA 2412 gives alpha_L0 = -2.077 deg and cm_c/4 = -0.053 (Anderson, Fundamentals of Aerodynamics, Ex. 4.6); parabolic gives -2f and -pi f', async () => {
  const { thinAirfoilCamber } = await import('../../src/physics/polars.js');
  const n = thinAirfoilCamber(0.02, 0.4);
  assert.ok(Math.abs((n.alphaL0 * 180) / Math.PI + 2.077) < 0.01, `alpha_L0 ${(n.alphaL0 * 180) / Math.PI}`);
  assert.ok(Math.abs(n.cmC4 + 0.053) < 0.001, `cm ${n.cmC4}`);
  const q = thinAirfoilCamber(0.03, 0.5);
  assert.ok(Math.abs(q.alphaL0 + 0.06) < 1e-5 && Math.abs(q.cmC4 + Math.PI * 0.03) < 1e-5);
});
