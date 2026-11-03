// Closed loop on the test article: controller table from buildControl (envelope, servo limits), then steady level
// flight, where Newton says the cycle-mean vertical air force equals the weight and the mean CG load factor is ~1.
import test from 'node:test';
import assert from 'node:assert/strict';
import { buildBird, eulerFromQuat } from '../../src/physics/flight.js';
import { buildControl, FLIGHT_TRIM_TOL } from '../../src/design/trim.js';
import { setupFlight, runM1 } from '../../src/physics/scenarios.js';
import { testDesign } from '../physics/fixtures.js';

const DEG = Math.PI / 180;

test('closed loop: level cruise carries the weight on average, inside the rated stroke envelope', () => {
  const d = testDesign();
  d.geometry = { chord_root_m: 0.25, area_m2: 0.4, AR: 4 };
  d.atmosphere.rho_kg_m3 = 1.0;
  d.structure = { gear_stages: [{ ratio: 5, gear_allowable_Nm: 3, pinion_allowable_Nm: 0.8 }, { ratio: 6, gear_allowable_Nm: 30, pinion_allowable_Nm: 6 }], spar_inner: { I: (Math.PI * (0.016 ** 4 - 0.014 ** 4)) / 64, c: 0.008, allowable_MPa: 650 } };
  // Battery forward: at the fixture's x = 0.08 m the CG (x = -0.059 m) is 2.8 cm behind the neutral point (-0.031 m),
  // statically unstable in pitch, and the table's pitch gains (which assume a stable airframe) let it tumble. At
  // x = 0.30 m the CG is at +0.002 m, a static margin of 12.6% of the root chord.
  d.mass.lumps.find((l) => l.name === 'battery').pos = [0.3, -0.01, 0];
  const bird = buildBird(d);
  const ctl = buildControl(d, { amp: 15 * DEG, sym: 0, dyn: 0.15, lead: 0, V: 10 }, { speeds: [10, 11], bird });
  assert.ok(ctl, 'trim table');
  d.control = ctl;
  assert.ok(ctl.envelope.length && ctl.envelope.every((e) => e.freqMax >= 0));
  const { sim, tick } = setupFlight(d, { sensorNoise: false, init: { V: 10, y: 0 } });
  let fy = 0, n = 0, lf = 0, V = 0, y0 = null, thMin = Infinity, thMax = -Infinity;
  const HZ = Math.round(1 / sim.dt);
  // The free-flight trim needs ~1.7x the frozen-body trim's stroke speed (the heavy wings recoil the body); the speed
  // loop's integrator takes ~40 s to find it. Average over 40-50 s, once the flight has settled.
  for (let k = 0; k < 50 * HZ; k++) {
    tick();
    if (k >= 40 * HZ) {
      // world-vertical air force: body force rotated by the attitude (only the y row is needed)
      const s = sim.snapshot();
      const [qx, qy, qz, qw] = s.quat;
      const R10 = 2 * (qx * qy + qw * qz), R11 = 1 - 2 * (qx * qx + qz * qz), R12 = 2 * (qy * qz - qw * qx);
      const W = bird.mTotal * 9.80665;
      fy += (R10 * sim.tel.Fx + R11 * sim.tel.Fy + R12 * sim.tel.Fz + W) / W; // tel.F includes gravity
      lf += sim.tel.loadFactor;
      V += sim.tel.airspeed;
      y0 ??= s.pos[1];
      const th = eulerFromQuat(s.quat).theta;
      thMin = Math.min(thMin, th); thMax = Math.max(thMax, th);
      n++;
    }
  }
  // The premise: steady level flight over the window (the free-flight trim's own tolerances), so Newton applies.
  const climb = (sim.snapshot().pos[1] - y0) / (n * sim.dt);
  assert.ok(Math.abs(climb) <= FLIGHT_TRIM_TOL.climb_m_s && Math.abs(V / n - 10) <= FLIGHT_TRIM_TOL.speed_m_s && thMax - thMin < 10 * DEG,
    `not steady level flight: climb ${climb} m/s, airspeed ${V / n} m/s, pitch ${thMin / DEG} to ${thMax / DEG} deg`);
  const envF = Math.max(...ctl.envelope.map((e) => e.freqMax));
  assert.ok(sim.cmd.freq <= envF + 1e-9, 'stroke frequency inside the envelope');
  assert.ok(Math.abs(fy / n - 1) < 0.05, `mean vertical air force / weight ${fy / n}`);
  // Mean of the force magnitude is at least the magnitude of the mean (flapping only adds to it).
  assert.ok(lf / n >= fy / n - 0.01, `mean load factor ${lf / n} below mean vertical force ${fy / n}`);
  const m1 = runM1(d, { sensorNoise: false });
  assert.ok(Number.isFinite(m1.measured.height_lost_m) && Number.isFinite(m1.measured.climb_rate_2_to_3s_m_s));
});
