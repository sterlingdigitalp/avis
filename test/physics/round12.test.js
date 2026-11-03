// Round 12: the belly-landing approach-and-flare law and the hand-launch pitch law (flight-controller code, SPEC ruling
// after physics round 11), and M5's pass rule, each pinned against hand calculations.
import test from 'node:test';
import assert from 'node:assert/strict';
import { landingLaw, launchPitch } from '../../src/physics/control.js';
import { m5Passed } from '../../src/physics/scenarios.js';

const L = { Vapp_m_s: 9, kV: 0.03, kIV: 0.02, iMax_rad: 0.1, flareH_m: 1.7, tau_s: 1.5, sinkTd_m_s: 0.3, kS: 0.3, kIS: 0.1, Vfloor_m_s: 8, kFloor: 0.05, dThetaMin_rad: -0.2, dThetaMax_rad: 0.25 };
const row = { theta: -0.05 };
const dt = 1 / 240;

test('approach: feed-forward from the approach row plus PI on the airspeed error (fast -> nose up)', () => {
  const s = { flare: false };
  // V 10, Vapp 9: error +1. Integral after one tick: 0.02 * 1 / 240. theta = -0.05 + 0.03 + 8.333e-5.
  const th = landingLaw(L, row, s, { V: 10, vy: -1.1 }, 5, dt);
  assert.ok(Math.abs(th - (-0.05 + 0.03 + 0.02 / 240)) < 1e-12);
  assert.equal(s.flare, false);
  // The integral saturates at iMax: after 1e5 ticks at +1 m/s it holds 0.1.
  for (let k = 0; k < 1e5; k++) landingLaw(L, row, s, { V: 10, vy: -1.1 }, 5, dt);
  assert.ok(Math.abs(landingLaw(L, row, s, { V: 10, vy: -1.1 }, 5, dt) - (-0.05 + 0.03 + 0.1)) < 1e-12);
});

test('flare: latched below flareH, exponential climb-rate command, from the pitch at flare entry', () => {
  const s = { flare: false };
  const th0 = landingLaw(L, row, s, { V: 9, vy: -1.1 }, 2, dt); // on speed: theta = row.theta exactly
  assert.ok(Math.abs(th0 - -0.05) < 1e-12);
  // h 1.2 m: vy_cmd = -max(0.3, 1.2 / 1.5) = -0.8; vy -1.1 -> error +0.3. theta = th0 + 0.3*0.3 + 0.1*0.3/240.
  const th = landingLaw(L, row, s, { V: 9, vy: -1.1 }, 1.2, dt);
  assert.equal(s.flare, true);
  assert.ok(Math.abs(th - (-0.05 + 0.09 + 0.03 / 240)) < 1e-12);
  // Latched: back above flareH it stays in the flare. h 0.3: vy_cmd = -max(0.3, 0.2) = -0.3.
  landingLaw(L, row, s, { V: 9, vy: -1.1 }, 3, dt);
  assert.equal(s.flare, true);
  const s2 = { flare: true, th0: 0, iS: 0 };
  assert.ok(Math.abs(landingLaw(L, row, s2, { V: 9, vy: -0.5 }, 0.3, dt) - (0.3 * 0.2 + 0.1 * 0.2 / 240)) < 1e-12);
});

test('stall guard and pitch bounds', () => {
  // V 7.5 below Vfloor 8: on top of the approach law, -0.05 * 0.5. Approach error -1.5: -0.045, integral -0.03/240.
  const s = { flare: false };
  const th = landingLaw(L, row, s, { V: 7.5, vy: -1 }, 5, dt);
  assert.ok(Math.abs(th - (-0.05 - 0.045 - 0.03 / 240 - 0.025)) < 1e-12);
  // Bounds: a huge sink error saturates at row.theta + dThetaMax.
  const s2 = { flare: true, th0: 0.2, iS: 0 };
  assert.equal(landingLaw(L, row, s2, { V: 9, vy: -5 }, 0.5, dt), -0.05 + 0.25);
  const s3 = { flare: false };
  assert.equal(landingLaw(L, row, s3, { V: 30, vy: 0 }, 5, dt), -0.05 + 0.25);
  assert.equal(landingLaw(L, row, { flare: false }, { V: 2, vy: 0 }, 5, dt), -0.05 - 0.2);
});

test('launch pitch: PI on the climb-out airspeed about the trim row, bounded', () => {
  const P = { V_m_s: 9.5, kV: 0.05, kIV: 0.05, iMax_rad: 0.15, dThetaMin_rad: -0.3, dThetaMax_rad: 0.3 };
  const s = {};
  // V 8: error -1.5. theta = 0.07 - 0.075 - 0.075/240.
  assert.ok(Math.abs(launchPitch(P, { theta: 0.07 }, s, { V: 8 }, dt) - (0.07 - 0.075 - 0.075 / 240)) < 1e-12);
  assert.equal(launchPitch(P, { theta: 0.07 }, {}, { V: 30 }, dt), 0.07 + 0.3);
});

test('M5 pass rule: sink 1.5 and speed 9 inclusive, no stall departure', () => {
  assert.equal(m5Passed({ sink: 1.5, speed: 9 }), true);
  assert.equal(m5Passed({ sink: 1.5001, speed: 8 }), false);
  assert.equal(m5Passed({ sink: 0.5, speed: 9.0001 }), false);
  assert.equal(m5Passed({ sink: 0.5, speed: 8 }, true), false);
  assert.equal(m5Passed(null), false);
});

test('flare latches only below flareH, and takes the last pitch even when it is exactly zero', () => {
  const s = { flare: false };
  landingLaw(L, row, s, { V: 9, vy: -1.1 }, L.flareH_m, dt); // at flareH exactly: still the approach
  assert.equal(s.flare, false);
  const s2 = { flare: false, last: 0 };
  // Entering the flare with last = 0: th0 is 0, not row.theta. h 1.2: es = -0.8 + 1.1 = 0.3.
  assert.ok(Math.abs(landingLaw(L, row, s2, { V: 9, vy: -1.1 }, 1.2, dt) - (0.3 * 0.3 + 0.1 * 0.3 / 240)) < 1e-12);
});

test('M5 helpers: the legacy glide law and the descent record by hand', async () => {
  const { legacyGlidePitch, descentTracker } = await import('../../src/physics/scenarios.js');
  // thApp -0.05, Vapp 9.5, V 10, kV 0.03: -0.035 above the flare; at est 0.75 of flareHeight 1.5: + 0.5 x 10 deg.
  assert.ok(Math.abs(legacyGlidePitch({}, -0.05, 9.5, 5, 10) - (-0.035)) < 1e-12);
  assert.ok(Math.abs(legacyGlidePitch({}, -0.05, 9.5, 0.75, 10) - (-0.035 + 0.5 * 10 * Math.PI / 180)) < 1e-12);
  assert.ok(Math.abs(legacyGlidePitch({ kV: 0.1, flareHeight: 2, flareTheta: 0.2 }, 0, 9, -1, 9) - 0.2) < 1e-12);
  const tr = descentTracker();
  tr.add(5, 9, 0.6, 0.5); tr.add(5.5, 8, 0.7, 0.5); tr.add(6, 8.5, 0.2, 0.5); tr.add(6.5, 8.6, 0.9, 0.6);
  const s = tr.summary();
  assert.equal(s.min_airspeed_m_s, 8);
  assert.equal(s.min_airspeed_t_s, 5.5);
  assert.equal(s.max_stalled_fraction, 0.9);
  assert.equal(s.longest_stall_s, 1.0); // two 0.5 s steps above half stalled, then a reset
  assert.equal(tr.departure, false); // a departure needs more than a whole second
  tr.add(7, 8.6, 0.9, 0.5);
  assert.equal(tr.departure, true); // 0.6 + 0.5 s
});

test('M5 flies the controller landing law when the design carries one (coarse step, wiring check)', async () => {
  const { readFileSync } = await import('node:fs');
  const { runM5 } = await import('../../src/physics/scenarios.js');
  const d = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
  const r = runM5(d, { dt: 1 / 480, sensorNoise: false });
  assert.equal(r.measured.law, 'control.landing (control.js landingLaw)');
  assert.ok(r.measured.flare_entry, 'the flare latched');
  // The flare latches on the belly height estimate below flareH; the CG height then is flareH - bottom within the
  // estimator's error (sensor noise off: the baro-inertial filter only).
  assert.ok(Math.abs(r.measured.flare_entry.h - (d.control.landing.flareH_m - d.body.bottom_y_m)) < 0.3);
  assert.equal(r.measured.approach_cmd_m_s, d.control.landing.Vapp_m_s);
  const legacy = runM5({ ...d, control: { ...d.control, landing: undefined } }, { dt: 1 / 480, sensorNoise: false });
  assert.equal(legacy.measured.law, 'scenario-side glide law (before round 12)');
});

test('eta_s table against data/calibration/suction.json (SPEC ruling after cfd round 3)', async () => {
  const { readFileSync } = await import('node:fs');
  const { suctionTableFrom, suctionEtaAt } = await import('../../src/physics/strip.js');
  const DEG = Math.PI / 180;
  const json = JSON.parse(readFileSync(new URL('../../data/calibration/suction.json', import.meta.url), 'utf8'));
  const t = suctionTableFrom(json);
  // The three settled round-3 points (coordinator's ruling table): 5.908 deg 0.732, 6.237 deg 0.699, 6.875 deg 0.659.
  assert.deepEqual([...t.used].sort(), ['e387-L0.667-larus-v17-strip-s0.4-r3', 'e387-L0.667-larus-v17-strip-s0.6-r3', 'e387-L0.667-larus-v17-strip-s0.75-r3']);
  assert.equal(t.table.length, 3);
  [[5.908, 0.732], [6.237, 0.699], [6.875, 0.659]].forEach(([a, e], i) => { assert.ok(Math.abs(t.table[i][0] / DEG - a) < 5e-4, `amp ${i}`); assert.ok(Math.abs(t.table[i][1] - e) < 5e-4, `eta ${i}`); });
  // The round-2 runs of the same cases (settled false) are out, never averaged with their extensions; the v14 and
  // pure-plunge points carry no verdict and are out.
  const why = Object.fromEntries(t.skipped.map((s) => [s.name, s.why]));
  for (const n of ['e387-L0.667-larus-v17-strip-s0.4', 'e387-L0.667-larus-v17-strip-s0.6', 'e387-L0.667-larus-v17-strip-s0.75']) assert.match(why[n], /unsettled/);
  for (const n of ['e387-L0.667-larus-v14-z0.4', 'e387-L0.667-larus-v14-z0.75', 'e387-L0.667-Re100000-k0.35-h0.75-a4', 'e387-L0.667-Re100000-k0.5-h1-a4']) assert.match(why[n], /no settled verdict/);
  // Below the smallest amplitude: hold 0.732. Between: linear. Beyond the largest: the last segment's slope continued
  // (it lowers eta_s): (0.659 - 0.699) / (6.875 - 6.237) per degree, about -0.063.
  const [a1, e1] = t.table[1], [a2, e2] = t.table[2];
  const slope = (e2 - e1) / ((a2 - a1) / DEG);
  assert.ok(Math.abs(slope - -0.063) < 0.001);
  assert.equal(suctionEtaAt(t.table, 4 * DEG), t.table[0][1]);
  assert.ok(Math.abs(suctionEtaAt(t.table, a2 + 1 * DEG) - (e2 + slope)) < 1e-12);
  assert.ok(suctionEtaAt(t.table, a2 + 1 * DEG) < e2);
  // A raising last segment is held, not continued: eta_s never exceeds a measured value. Far beyond: never below 0.
  assert.equal(suctionEtaAt([[5 * DEG, 0.5], [6 * DEG, 0.7]], 8 * DEG), 0.7);
  assert.equal(suctionEtaAt(t.table, 30 * DEG), 0);
});
