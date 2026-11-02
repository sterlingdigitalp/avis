import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { createFlight, buildBird, quatFromEuler, eulerFromQuat } from '../../src/physics/flight.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;

function flapInVacuum(design, { seconds = 1.5, crank = 2.5 * TWO_PI } = {}) {
  const bird = buildBird(design);
  // No air and no gravity: the wings beat against the body alone.
  const sim = createFlight(design, { bird, rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: crank / TWO_PI, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0.3, 0.1, 0.05), vel: [3, 0.5, -0.2], omega: [0.1, -0.2, 0.3], phase: 0, crank, amp: 0.26 });
  const m0 = sim.momentum();
  let worstP = 0, worstH = 0;
  for (let k = 0; k < seconds * 240; k++) {
    sim.step();
    const m = sim.momentum();
    worstP = Math.max(worstP, Math.hypot(m.P[0] - m0.P[0], m.P[1] - m0.P[1], m.P[2] - m0.P[2]));
    worstH = Math.max(worstH, Math.hypot(m.H[0] - m0.H[0], m.H[1] - m0.H[1], m.H[2] - m0.H[2]));
  }
  return { worstP, worstH, P0: Math.hypot(...m0.P), H0: Math.hypot(...m0.H), sim };
}

test('flapping in vacuum conserves linear and angular momentum of the body and wings', () => {
  const d = testDesign();
  const r = flapInVacuum(d);
  // Wings swing at metres per second with tens of grams: momentum scale of the wing motion ~ 0.1 kg m/s.
  assert.ok(r.worstP < 2e-3, `linear momentum drift ${r.worstP} (initial ${r.P0})`);
  assert.ok(r.worstH < 2e-3, `angular momentum drift ${r.worstH} (initial ${r.H0})`);
});

test('quaternion and Euler helpers agree with the SPEC frames', () => {
  const q = quatFromEuler(0.4, 0.2, -0.3);
  const e = eulerFromQuat(q);
  assert.ok(Math.abs(e.chi - 0.4) < 1e-9 && Math.abs(e.theta - 0.2) < 1e-9 && Math.abs(e.phi + 0.3) < 1e-9);
  // Nose up: body x gets a +Y component. Right wing down for positive bank: body z gets -Y.
  const up = quatFromEuler(0, 0.3, 0);
  const R10 = 2 * (up[0] * up[1] + up[3] * up[2]);
  assert.ok(R10 > 0);
  const bank = quatFromEuler(0, 0, 0.3);
  const R12 = 2 * (bank[1] * bank[2] - bank[3] * bank[0]);
  assert.ok(R12 < 0);
});

test('regression: fast flapping with a small roll inertia stays bounded (apparent mass is not differenced through body motion)', () => {
  const d = testDesign();
  for (const l of d.mass.lumps) l.gyration_m = 0;
  const bird = buildBird(d);
  const sim = createFlight(d, { bird, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 6, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.0, 0.1), vel: [13, 0, 0], omega: [0.5, 0, 0], phase: 0, crank: 6 * TWO_PI, amp: 0.26 });
  let worst = 0;
  for (let k = 0; k < 240; k++) {
    sim.step();
    worst = Math.max(worst, Math.abs(sim.x[10]));
  }
  // Physical roll rates here are a few rad/s at most over one second; the old explicit scheme ran away.
  assert.ok(Number.isFinite(worst) && worst < 8, `peak roll rate ${worst}`);
});
