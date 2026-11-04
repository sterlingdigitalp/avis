// Round 18 (diagnosis 2.1): the twist chain's series compliance was a one-step-lagged algebraic loop
// (theta = S - C*T(prev)), unstable above about C 0.11 rad/N m at dt 1/240 s, and at any C > 0 for a small enough dt.
// Fixed as a relaxed deflection state per wing (flight.js, near lastTwistTorque and TAU_CHAIN_S). These tests pin the
// fix against: the relaxation update's own closed-form recursion and a first-order lag's known frequency response
// (independent of this design or any aircraft physics); and the v24 design with v25's drawn twist chain swapped in,
// at v24's own trim point, against the diagnosis' own account of what the unfixed code does there.
// The behavioural tests live here and import nothing the fix added, so on the unfixed flight.js they run and fail for
// the physical reason (the cycle diverges); the relaxation unit test is in twist-compliance.test.js.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildBird, createFlight, quatFromEuler } from '../../src/physics/flight.js';
import { cycleResidual, settleCycles } from '../../src/design/trim.js';

const DEG = Math.PI / 180;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''}: ${a} vs ${b} (tol ${tol})`);

// v24's drawn twist servo (MKS HV6160) has no series compliance; v25's drawn chain (titanium torque rod in torsion
// plus the bar in bending, twist-drive.js chainCompliance) does. These are the real numbers from the round-18
// diagnosis' v25 probe build (docs/status/diagnose-trim-v25.md 2.1 and 2.5; the build's own servos.twist block,
// read from the MD89MW's catalogue row through the drawn chain), not invented: MD89MW 0.484 N m / 9.49 rad/s at the
// wing, ratio 1.251, travel_rad 0.675, compliance_rad_per_Nm 0.208.
const V25_TWIST_SERVO = {
  id: 'servo-hitec-md89mw', stall_Nm: 0.48434382500620055, speed_rad_s: 9.495045580617575, stall_A: 2.285714285714286,
  tau_s: 0.02, idle_A: 0.03, volts: 5.76, current_volts: 6.24, mass_kg: 0.0215, linkage_ratio: 1.251259887523582,
  servo_stall_Nm: 0.60604, servo_speed_rad_s: 7.5883880521492575,
  travel_rad: 0.6750850251041338, compliance_rad_per_Nm: 0.208256,
};

/** v24's design (frozen copy test/fixtures/larus-gl-2-v24.json, so the test survives the official v25 write) with v25's twist servo block swapped in, optionally at a different C. */
function fixture(compliance_rad_per_Nm = V25_TWIST_SERVO.compliance_rad_per_Nm) {
  const d = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
  d.servos.twist = { ...V25_TWIST_SERVO, compliance_rad_per_Nm };
  return d;
}

// v24's own trim point at dyn 38 deg (diagnosis 1.1: a558baf and 0d9ba86 agree to every printed digit): V 10,
// theta 4.130 deg, 1.7454 Hz, tail 1.999 deg.
const POINT = { V: 10, theta: 4.130 * DEG, freq: 1.7454, tail: 1.999 * DEG, dyn: 38 * DEG };
const point = (d, dt) => cycleResidual(d, buildBird(d), { V: POINT.V, theta: POINT.theta, freq: POINT.freq, tail: POINT.tail, amp: d.params.amp_deg * DEG, sym: 0, dyn: POINT.dyn, lead: 0, dt });

/** A frozen cycle through createFlight directly (not trim.js's cycleResidual, which does not forward chainTau_s):
 * same point, same loop as cycleResidual but able to override the chain's relaxation time constant for the test. */
function frozenCycleAtTau(d, dt, chainTau_s) {
  const bird = buildBird(d);
  const sim = createFlight(d, { bird, dt, frozenBody: true, chainTau_s });
  const amp = d.params.amp_deg * DEG;
  const qv = quatFromEuler(0, POINT.theta, 0);
  const stepsPerCycle = Math.round(1 / (dt * POINT.freq));
  const nCycles = settleCycles(d, POINT.V, POINT.freq);
  const cmd = sim.cmd;
  cmd.freq = POINT.freq; cmd.amp = amp; cmd.sym = 0; cmd.dyn = POINT.dyn; cmd.lead = 0; cmd.diff = 0;
  cmd.tailPitch = POINT.tail; cmd.tailYaw = 0;
  const crank = POINT.freq * 2 * Math.PI;
  sim.setState({ pos: [0, 0, 0], quat: qv, vel: [POINT.V, 0, 0], phase: 0, crank, amp, twistR: 0, twistL: 0, tailR: -POINT.tail, tailL: -POINT.tail, iInt: 0, epsLag: 0.5 });
  const res = new Float64Array(6);
  let n = 0;
  for (let k = 0; k < nCycles * stepsPerCycle; k++) {
    sim.step();
    const x = sim.x;
    x[0] = x[1] = x[2] = 0; x[3] = qv[0]; x[4] = qv[1]; x[5] = qv[2]; x[6] = qv[3];
    x[7] = POINT.V; x[8] = 0; x[9] = 0; x[10] = 0; x[11] = 0; x[12] = 0; x[14] = crank;
    if (k >= (nCycles - 1) * stepsPerCycle) { for (let j = 0; j < 6; j++) res[j] += sim.tel.residual[j]; n++; }
  }
  for (let j = 0; j < 6; j++) res[j] /= n;
  return { fy: res[1], twistPeak: sim.peaks.twistTorque };
}


test('frozen cycle at v24\'s trim point, v25\'s compliance: finite and small, twist under stall (unfixed: fx -380 N, twist 12.5 N m)', () => {
  const q = point(fixture(), 1 / 240);
  assert.ok(Number.isFinite(q.res[0]) && Number.isFinite(q.res[1]) && Number.isFinite(q.res[5]));
  // The unstable loop gives fx -380 N, mz 37 N m, twist peak 12.9 N m (26x the servo's stall) at this point.
  near(q.res[0], 0, 5, 'fx');
  near(q.res[5], 0, 5, 'mz');
  assert.ok(q.twistPeak < V25_TWIST_SERVO.stall_Nm, `twist peak ${q.twistPeak} N m should be under the servo's stall ${V25_TWIST_SERVO.stall_Nm} N m`);
});

test('fixed compliance (C 0.208) residual stays close to the C = 0 result, at the same (unretrimmed) point', () => {
  // Correctly modelled, the diagnosis finds the compliance costs "about 0.3 N of lift" at a fixed point (2.1,
  // Evidence 4): not zero, since the wing really does wind off the servo's command by a small, physical amount.
  // This pins that it stays of that order (well under 1 N, and nowhere near the 2% x weight = 0.34 N band's
  // two-order-of-magnitude-larger neighbour: the unfixed loop's -380 N).
  const d = fixture(), d0 = fixture(0);
  const q = point(d, 1 / 240), q0 = point(d0, 1 / 240);
  const bird = buildBird(d), weight = bird.mTotal * 9.80665;
  const norm = (r) => Math.hypot(r[0], r[1], 3 * r[5]); // trim.js's own convergence norm (streamwise, vertical, 3x pitch/m)
  near(norm(q.res), norm(q0.res), 0.03 * weight, 'residual norm vs C=0, within 3% of weight (diagnosis: about 0.3 N)');
});

test('insensitive to the relaxation time constant over 8-40 ms, at dt 1/240 s', () => {
  const d = fixture();
  const base = frozenCycleAtTau(d, 1 / 240, 0.02);
  for (const tau of [0.008, 0.04]) {
    const q = frozenCycleAtTau(d, 1 / 240, tau);
    // The diagnosis found results agree within 0.01 N across tau 8-40 ms at a given dt; 0.02 N leaves headroom for a
    // different fixture while still catching a reintroduced instability (which moves fy by hundreds of N).
    near(q.fy, base.fy, 0.02, `fy at tau ${tau} vs tau 0.02`);
    assert.ok(q.twistPeak < V25_TWIST_SERVO.stall_Nm, `twist peak at tau ${tau} should stay under stall`);
  }
});

test('insensitive to dt over 1/240-1/1920 s: the compliance fix\'s own contribution (vs C=0 at the same dt) barely moves', () => {
  // The frozen cycle's power (and, to a lesser extent, its force residuals) is not itself dt-converged: a
  // pre-existing, unrelated effect of the strip's finite-differenced apparent mass (diagnosis side note), present
  // whether or not the twist chain has any compliance. Isolating the compliance fix's own effect -- the difference
  // between the fixed-C run and the C=0 run at the SAME dt -- removes that pre-existing drift and tests only what
  // this fix is responsible for.
  const d = fixture(), d0 = fixture(0);
  const base = { fy: point(d, 1 / 240).res[1] - point(d0, 1 / 240).res[1], P: point(d, 1 / 240).power - point(d0, 1 / 240).power };
  for (const dt of [1 / 480, 1 / 960, 1 / 1920]) {
    const dFy = point(d, dt).res[1] - point(d0, dt).res[1];
    const dP = point(d, dt).power - point(d0, dt).power;
    near(dFy, base.fy, 0.05, `compliance's own effect on fy at dt ${Math.round(1 / dt)}`);
    near(dP, base.P, 1.0, `compliance's own effect on power at dt ${Math.round(1 / dt)}`);
  }
});
