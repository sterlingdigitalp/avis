import test from 'node:test';
import assert from 'node:assert/strict';
import { couplerCheck, sparCheck, ROD_ALLOWABLE_MPA } from '../../src/physics/structure.js';
import { twistSchedule } from '../../src/physics/control.js';
import { createFlight, buildBird, quatFromEuler } from '../../src/physics/flight.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;

test('coupler: Euler buckling of a pinned rod and of a tube, against the closed form worked by hand', () => {
  // Rod 4 mm, E 28 GPa, 80 mm: I = pi d^4 / 64 = 12.566 mm^4, P = pi^2 E I / L^2 = 542.6 N (round 2's M7 figure).
  const rod = couplerCheck({ od_m: 0.004, length_m: 0.08, E_GPa: 28 }, 1000);
  assert.ok(Math.abs(rod.buckling_N - 542.6) < 0.1, `rod ${rod.buckling_N}`);
  assert.equal(rod.allowable_MPa, ROD_ALLOWABLE_MPA);
  assert.equal(rod.section.kind, 'rod');
  assert.ok(Math.abs(rod.stress_MPa - 1000 / (Math.PI * 4) ) < 1e-9);
  // Tube 7 x 5 mm, E 64 GPa, 80 mm: I = pi (7^4 - 5^4) / 64 = 87.18 mm^4, P = 8604 N; area pi (49 - 25) / 4 = 18.85 mm^2.
  const tube = couplerCheck({ od_m: 0.007, id_m: 0.005, length_m: 0.08, E_GPa: 64, allowable_MPa: 650 }, 2000);
  assert.ok(Math.abs(tube.buckling_N - 8604) < 2, `tube ${tube.buckling_N}`);
  assert.ok(Math.abs(tube.stress_MPa - 2000 / 18.8496) < 0.01);
  assert.ok(Math.abs(tube.fos_stress - 650 / tube.stress_MPa) < 1e-12);
  assert.ok(Math.abs(tube.fos_buckling - 8604 / 2000) < 1e-3);
  assert.equal(tube.section.kind, 'tube');
});

test('spar: bending stress M c / I of a tube', () => {
  // 10 x 8 mm tube: I = pi (10^4 - 8^4) / 64 = 289.8 mm^4, c = 5 mm; 30 N m gives 517.6 MPa.
  const I = (Math.PI * (0.01 ** 4 - 0.008 ** 4)) / 64;
  const r = sparCheck({ I, c: 0.005, allowable_MPa: 841 }, 30);
  assert.ok(Math.abs(r.stress_MPa - 517.6) < 0.2, `${r.stress_MPa}`);
  assert.ok(Math.abs(r.fos - 841 / r.stress_MPa) < 1e-12);
});

test('twist schedule: the trim twist at the trim stroke, plus the change of plunge angle atan(2 pi f A r / V)', () => {
  const tr = { V: 10, freq: 2, amp: 0.5, dyn: 0.4 };
  assert.equal(twistSchedule({}, tr, 3, 0.6, 8), 0.4, 'no schedule: the trim twist');
  const cfg = { dynSchedule: { rRef_m: 0.6, gain: 1, min: 0.1, max: 0.9 } };
  assert.ok(Math.abs(twistSchedule(cfg, tr, 2, 0.5, 10) - 0.4) < 1e-12);
  const want = 0.4 + Math.atan((TWO_PI * 3 * 0.5 * 0.6) / 10) - Math.atan((TWO_PI * 2 * 0.5 * 0.6) / 10);
  assert.ok(Math.abs(twistSchedule(cfg, tr, 3, 0.5, 10) - want) < 1e-12);
  // Slower air at the same stroke: a steeper plunge angle, so more twist; clamped to the schedule's bounds.
  assert.ok(twistSchedule(cfg, tr, 2, 0.5, 6) > 0.4);
  assert.equal(twistSchedule(cfg, tr, 20, 0.8, 4), 0.9);
  assert.equal(twistSchedule(cfg, tr, 0, 0.5, 10), Math.max(0.1, 0.4 - Math.atan((TWO_PI * 2 * 0.5 * 0.6) / 10)));
});

function brakeRun(design, dt) {
  const bird = buildBird(design);
  const sim = createFlight(design, { bird, dt });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.0, 0), vel: [13, 0, 0], phase: 0, crank: 2.5 * TWO_PI, amp: 0.6 });
  Object.assign(sim.cmd, { freq: 2.5, amp: 0.6, dyn: 0.25, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  let peak = 0;
  const N = Math.round(3 / dt);
  for (let k = 0; k < N; k++) {
    // Cut the stroke to a crawl after half a second: the governor brakes the crank against the wings.
    if (k === Math.round(0.5 / dt)) sim.cmd.freq = 0.6;
    sim.step();
    // Hold the body in level flight so only the crank and wings respond.
    sim.x[3] = 0; sim.x[4] = 0; sim.x[5] = 0; sim.x[6] = 1;
    sim.x[7] = 13; sim.x[8] = 0; sim.x[9] = 0; sim.x[10] = 0; sim.x[11] = 0; sim.x[12] = 0;
    if (k > Math.round(0.5 / dt)) peak = Math.max(peak, Math.abs(sim.tel.crankTorque));
  }
  return peak;
}

test('regression: a stroke braked to a crawl against a light wing stays bounded and converges in time step (round 3)', () => {
  const d = testDesign();
  // A large stroke on a light wing (30% of the article's): the apparent mass on the crank then rivals its inertia.
  // Before the fix this case reached 3e3 N m at 1/480 s and NaN at 1/960 s.
  d.drive.ampMax = 0.75;
  d.mass.wing.spar_kg_per_m *= 0.3;
  d.mass.wing.skin_kg_per_m2 *= 0.3;
  const a = brakeRun(d, 1 / 480);
  const b = brakeRun(d, 1 / 960);
  // A few N m at most for this article; the explicit apparent-mass scheme grew by orders of magnitude here.
  assert.ok(Number.isFinite(a) && a < 5, `peak crank torque ${a}`);
  assert.ok(Math.abs(b / a - 1) < 0.1, `time-step change ${a} -> ${b}`);
});

test('energy control: below the speed floor pitch serves speed alone (nose down), above it height (nose up)', async () => {
  const { createController } = await import('../../src/physics/control.js');
  const tr = { V: 12, theta: 0.02, freq: 2.2, tail: 0, amp: 0.5, sym: 0, dyn: 0.4 };
  const cfg = {
    trim: [tr, { ...tr, V: 14, freq: 2.5 }], Vcruise: 12, Vmin: 9.5, kH: 0.5, kGamma: 1, kIGamma: 0.15, kTheta: 0.2, kQ: 0, kFc: 0.36, kFcFf: 0.5, kIF: 0.08, kFv: 0.3,
    kPhi: 0.3, kP: 0, kR: 0.02, kChi: 1, maxBank: 0.6, diffMax: 0.3, climbMax: 2, sinkMax: 2.5, freqMin: 1, ampMax: 0.7, ampNominal: 0.5, kStallPitch: 0.08,
    tecs: { Vfloor: 9.5, Vmax: 15, kV: 0.4, band: 1.5 },
  };
  const fly = (V) => {
    const c = createController({ control: cfg }, { sensorNoise: false, delay: 0 });
    const truth = { t: 0, dt: 0.01, omega: [0, 0, 0], sf: [0, 9.80665, 0], up: [Math.sin(0.02), Math.cos(0.02), 0], y: 0, vel: [V, 0, 0], pos: [0, 0, 0], airspeedX: V, phase: 0, crank: 2.2 * TWO_PI, heading: 0, battI: 3 };
    c.initAttitude(truth);
    c.guidance.speed = 13;
    c.guidance.altitude = 5; // 5 m below the target: height wanted
    let cmd;
    for (let k = 0; k < 150; k++) cmd = c.update({ ...truth, t: k * 0.01 }, k * 0.01);
    return cmd;
  };
  const slow = fly(8.5), fast = fly(12);
  assert.ok(slow.tailPitch < 0, `below the floor: nose down, tail ${slow.tailPitch}`);
  assert.ok(fast.tailPitch > 0, `above the floor: nose up for height, tail ${fast.tailPitch}`);
  // Both short of energy: the stroke goes above the trim's.
  assert.ok(slow.freq > tr.freq && fast.freq > tr.freq);
});
