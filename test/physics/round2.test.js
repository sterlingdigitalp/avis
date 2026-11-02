// Round 2 model fixes, each against a hand calculation or an identity independent of the code under test.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createFlight, buildBird, quatFromEuler, escBatteryCurrent, servoSlewRate } from '../../src/physics/flight.js';
import { makeWing, crankRockerSolve, makeFlapDrive, strokeAngle } from '../../src/physics/kinematics.js';
import { createStrip, SUCTION } from '../../src/physics/strip.js';
import { POLARS, polarEval } from '../../src/physics/polars.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;

test('no regeneration credit: a braking motor draws nothing from the battery unless the ESC documents regeneration', () => {
  assert.equal(escBatteryCurrent(-3, { eta: 0.95 }), 0);
  assert.ok(Math.abs(escBatteryCurrent(2, { eta: 0.8 }) - 2.5) < 1e-12);
  assert.ok(Math.abs(escBatteryCurrent(-3, { eta: 0.95, regenerative: true, regen_eta: 0.7 }) + 2.1) < 1e-12);
  assert.equal(escBatteryCurrent(-3, { eta: 0.95, regen_eta: 0.7 }), 0, 'an efficiency alone is not a documented regeneration');
  // Hand calculation: P = a + b sin(wt) with b > a. Mean of max(0, P) = [a (pi - 2 t1) + 2 b cos t1] / (2 pi), t1 = asin(-a/b).
  const a = 1.0, b = 3.0, n = 20000;
  let m = 0;
  for (let k = 0; k < n; k++) m += escBatteryCurrent(a + b * Math.sin((TWO_PI * k) / n), { eta: 1 });
  m /= n;
  const t1 = Math.asin(-a / b);
  const hand = (a * (Math.PI - 2 * t1) + 2 * b * Math.cos(t1)) / TWO_PI;
  assert.ok(Math.abs(m - hand) < 1e-3, `${m} vs ${hand}`);
  assert.ok(hand > a * 1.3, 'clipping raises the mean well above the signed mean');
});

test('no regeneration credit in flight: an overspeeding crank that the motor brakes leaves only the 5 V bus on the battery', () => {
  const d = testDesign();
  const sim = createFlight(d, { bird: buildBird(d), rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 1, amp: 0.26, dyn: 0, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  // Crank spinning at 4 Hz against a 1 Hz command: the governor drives the duty to zero and back-EMF brakes.
  sim.setState({ pos: [0, 0, 0], quat: [0, 0, 0, 1], vel: [10, 0, 0], phase: 0, crank: 4 * TWO_PI, amp: 0.26 });
  let worst = 0;
  for (let k = 0; k < 24; k++) {
    sim.step();
    const bus = (5 * sim.tel.servo_I) / (0.85 * sim.tel.batt_V);
    worst = Math.max(worst, Math.abs(sim.tel.batt_I - bus) * (sim.tel.motor_I < 0 ? 1 : 0));
    assert.ok(sim.tel.batt_I >= 0);
  }
  assert.ok(worst < 1e-9, `battery current beyond the 5 V bus while braking: ${worst} A`);
});

test('load factor is at the mass centre: zero in free fall with the wings flapping in vacuum, while the body origin is not', () => {
  const d = testDesign();
  const sim = createFlight(d, { bird: buildBird(d), rho: 1e-12, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 3, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.1, 0), vel: [5, 0, 0], phase: 0, crank: 3 * TWO_PI, amp: 0.26 });
  let nCg = 0, nO = 0;
  for (let k = 0; k < 240; k++) {
    sim.step();
    nCg = Math.max(nCg, sim.tel.loadFactor);
    nO = Math.max(nO, sim.tel.loadFactorOrigin);
  }
  // Newton for the whole system: M a_cg = F_aero + M g, and F_aero = 0 here.
  assert.ok(nCg < 1e-6, `load factor at the CG ${nCg}`);
  assert.ok(nO > 0.05, `body origin specific force ${nO} g should show the wing recoil`);
});

test('coupler force by the rocker moment equals the crank-side value by virtual work (no transmission-angle floor)', () => {
  const d = testDesign();
  const law = makeFlapDrive(d.drive);
  const sim = createFlight(d, { bird: buildBird(d), rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 3, amp: law.ampNominal, dyn: 0, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: [0, 0, 0, 1], vel: [0, 0, 0], phase: 0.3, crank: 3 * TWO_PI, amp: law.ampNominal });
  let checked = 0;
  for (let k = 0; k < 120; k++) {
    sim.step();
    // The telemetry is taken at the mid-point of the step: use its own phase and amplitude.
    const s = { phase: sim.tel.phase, amp: sim.tel.amp };
    const sa = strokeAngle(s.phase, s.amp, law);
    // Symmetric wings: each coupler carries half the crank torque. Crank side: tau/2 = F a |sin(theta - beta)|.
    const tab = law.table;
    const x = ((s.amp - law.ampMin) / (law.ampMax - law.ampMin)) * (tab.length - 1);
    const i = Math.max(0, Math.min(tab.length - 2, Math.floor(x)));
    const f = x - i;
    const thetaTop = tab[i].thetaTop + f * (tab[i + 1].thetaTop - tab[i].thetaTop);
    const r = crankRockerSolve({ a: law.a, b: law.b, c: sa.c, d: law.d }, thetaTop + s.phase);
    const Fcrank = Math.abs(sim.tel.crankTorque) / 2 / (law.a * Math.abs(Math.sin(thetaTop + s.phase - r.beta)));
    if (Math.abs(sim.tel.crankTorque) > 1e-3 && Math.abs(Math.sin(thetaTop + s.phase - r.beta)) > 0.2) {
      assert.ok(Math.abs(sim.tel.couplerForce / Fcrank - 1) < 0.02, `rocker side ${sim.tel.couplerForce} N, crank side ${Fcrank} N`);
      checked++;
    }
  }
  assert.ok(checked > 40);
});

test('servo speed follows the published torque-speed line: half stall load, half speed; at stall it stops', () => {
  const sv = { stall_Nm: 0.2, speed_rad_s: 6, tau_s: 0.02 };
  const big = 1; // a command far away: the slew limit governs
  assert.ok(Math.abs(servoSlewRate(big, 0, sv, 0) - 6) < 1e-12);
  assert.ok(Math.abs(servoSlewRate(big, 0, sv, 0.1) - 3) < 1e-12, 'half stall torque against the motion: half speed');
  assert.equal(servoSlewRate(big, 0, sv, 0.25), 0, 'beyond stall: no motion that way');
  assert.ok(Math.abs(servoSlewRate(big, 0, sv, -0.1) - 6) < 1e-12, 'an aiding load does not slow it');
  assert.ok(Math.abs(servoSlewRate(-big, 0, sv, -0.1) + 3) < 1e-12, 'moving negative against a negative load');
  assert.ok(Math.abs(servoSlewRate(-big, 0, sv, 0.1, 1, false) + 3) < 1e-12, 'unsigned loads always oppose');
  assert.ok(Math.abs(servoSlewRate(0.001, 0, sv, 0) - 0.05) < 1e-12, 'small errors close with the servo lag');
  assert.ok(Math.abs(servoSlewRate(0.015, 0, sv, 0.19) - 0.3) < 1e-12, 'a slow move against a near-stall load is speed limited');
  assert.ok(Math.abs(servoSlewRate(big, 0, sv, 0, 0.5) - 3) < 1e-12, 'robustness speed scale');
});

test('partial suction: a 2D section in steady flow realises min(polar form, eta_s ideal suction - friction)', () => {
  const plate = makeWing({ hinge: [0, 0, 0], segments: [{ length: 200, chord_in: 0.2, chord_out: 0.2, xle_in: 0.05, xle_out: 0.05, gain: 0, twist_in: 0, twist_out: 0 }], spar_x: 0, camber: 0.03 });
  const J0 = { phi: 0, dphi: 0, ddphi: 0, theta: 0, dtheta: 0, ddtheta: 0 };
  const U = 10, rho = 1, mu = 1.8e-5, c = 0.2;
  const run = (eta, alpha) => {
    const m = createStrip(plate, { n: 4, polar: 'e387-film', suctionEta: eta });
    let r;
    for (let k = 0; k < 3000; k++) r = m.evaluate([U * Math.cos(alpha), -U * Math.sin(alpha), 0], [0, 0, 0], [0, 0, 0], J0, J0, rho, mu, 0.002);
    // chordwise force per span towards the LE, body x is along the chord here
    const span = 2 * 200;
    return r.F[0] / span;
  };
  for (const alpha of [0.06, 0.12]) {
    const pol = new Float64Array(5);
    polarEval(POLARS['e387-film'], alpha, (U * c * rho) / mu, 1, 1, pol);
    const q = 0.5 * rho * U * U;
    const meas = q * c * (pol[0] * Math.sin(alpha) - pol[1] * Math.cos(alpha));
    const eta = q * c * (0.5 * pol[0] * Math.sin(alpha) - pol[4] * Math.cos(alpha));
    const full = run(1, alpha);
    const half = run(0.5, alpha);
    const none = run(0, alpha);
    assert.ok(Math.abs(none - Math.min(meas, -q * c * pol[4] * Math.cos(alpha))) < 0.03 * q * c * pol[0] * Math.sin(alpha), `eta 0: ${none}`);
    assert.ok(Math.abs(full - meas) < 0.03 * q * c * pol[0] * Math.sin(alpha), `eta 1: ${full} vs polar ${meas}`);
    assert.ok(Math.abs(half - Math.min(meas, eta)) < 0.03 * q * c * pol[0] * Math.sin(alpha), `eta 0.5: ${half} vs ${Math.min(meas, eta)}`);
  }
  assert.ok(SUCTION.eta > 0 && SUCTION.eta <= 1 && SUCTION.source.length > 20);
});

test('load factor at the mass centre is 1 in a steady parked glide: the air force carries the weight exactly', async () => {
  const { trimGlide } = await import('../../src/design/trim.js');
  const d = testDesign();
  d.geometry = { chord_root_m: 0.25 };
  const bird = buildBird(d);
  const g = trimGlide(d, bird, { V: 11, amp: 0.26 });
  assert.ok(g.converged, `glide residual ${g.residual_norm_frac_weight}`);
  const sim = createFlight(d, { bird, dt: 1 / 240, frozenBody: true });
  Object.assign(sim.cmd, { freq: 0, amp: 0.26, dyn: 0, sym: 0, diff: 0, lead: 0, tailPitch: g.tail, tailYaw: 0, throttleOff: true, parkPhase: Math.PI / 2 });
  const q = quatFromEuler(0, g.theta, 0);
  const vx = 11 * Math.cos(g.gamma), vy = 11 * Math.sin(g.gamma);
  sim.setState({ pos: [0, 0, 0], quat: q, vel: [vx, vy, 0], phase: Math.PI / 2, crank: 0, amp: 0.26, tailR: -g.tail, tailL: -g.tail });
  let n = 0, k = 0;
  for (; k < 240; k++) {
    sim.step();
    sim.x.set([0, 0, 0, ...q, vx, vy, 0, 0, 0, 0], 0);
    if (k >= 120) n += sim.tel.loadFactor;
  }
  n /= 120;
  assert.ok(Math.abs(n - 1) < 0.02, `load factor ${n}`);
});

test('a sine stroke law has no linkage, so no coupler force is reported', () => {
  const d = testDesign();
  d.drive = { kind: 'sine', phiMean: 0, ampMin: 0.1, ampMax: 0.5, gearRatio: 30, gearEfficiency: 0.85 };
  const sim = createFlight(d, { bird: buildBird(d), rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 3, amp: 0.26, dyn: 0, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: [0, 0, 0, 1], vel: [0, 0, 0], phase: 0, crank: 3 * TWO_PI, amp: 0.26 });
  for (let k = 0; k < 20; k++) sim.step();
  assert.equal(sim.tel.couplerForce, 0);
  assert.ok(sim.tel.shoulderTorque > 0);
});
