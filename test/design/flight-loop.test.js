// Closed loop on the test article: controller table from buildControl (envelope, servo limits), then steady level
// flight, where Newton says the cycle-mean vertical air force equals the weight and the mean CG load factor is ~1.
import test from 'node:test';
import assert from 'node:assert/strict';
import { buildBird } from '../../src/physics/flight.js';
import { buildControl } from '../../src/design/trim.js';
import { setupFlight, runM1 } from '../../src/physics/scenarios.js';
import { testDesign } from '../physics/fixtures.js';

const DEG = Math.PI / 180;

test('closed loop: level cruise carries the weight on average, inside the rated stroke envelope', () => {
  const d = testDesign();
  d.geometry = { chord_root_m: 0.25, area_m2: 0.4, AR: 4 };
  d.atmosphere.rho_kg_m3 = 1.0;
  d.structure = { gear_stages: [{ ratio: 5, gear_allowable_Nm: 3, pinion_allowable_Nm: 0.8 }, { ratio: 6, gear_allowable_Nm: 30, pinion_allowable_Nm: 6 }], spar_inner: { I: (Math.PI * (0.016 ** 4 - 0.014 ** 4)) / 64, c: 0.008, allowable_MPa: 650 } };
  const bird = buildBird(d);
  const ctl = buildControl(d, { amp: 15 * DEG, sym: 0, dyn: 0.15, lead: 0, V: 10 }, { speeds: [10, 11], bird });
  assert.ok(ctl, 'trim table');
  d.control = ctl;
  assert.ok(ctl.envelope.length && ctl.envelope.every((e) => e.freqMax >= 0));
  const { sim, tick } = setupFlight(d, { sensorNoise: false, init: { V: 10, y: 0 } });
  let fy = 0, n = 0, lf = 0;
  const HZ = Math.round(1 / sim.dt);
  for (let k = 0; k < 12 * HZ; k++) {
    tick();
    if (k >= 4 * HZ) {
      // world-vertical air force: body force rotated by the attitude (only the y row is needed)
      const [qx, qy, qz, qw] = sim.snapshot().quat;
      const R10 = 2 * (qx * qy + qw * qz), R11 = 1 - 2 * (qx * qx + qz * qz), R12 = 2 * (qy * qz - qw * qx);
      const W = bird.mTotal * 9.80665;
      fy += (R10 * sim.tel.Fx + R11 * sim.tel.Fy + R12 * sim.tel.Fz + W) / W; // tel.F includes gravity
      lf += sim.tel.loadFactor;
      n++;
    }
  }
  const envF = Math.max(...ctl.envelope.map((e) => e.freqMax));
  assert.ok(sim.cmd.freq <= envF + 1e-9, 'stroke frequency inside the envelope');
  assert.ok(Math.abs(fy / n - 1) < 0.05, `mean vertical air force / weight ${fy / n}`);
  // Mean of the force magnitude is at least the magnitude of the mean (flapping only adds to it).
  assert.ok(lf / n >= fy / n - 0.01, `mean load factor ${lf / n} below mean vertical force ${fy / n}`);
  const m1 = runM1(d, { sensorNoise: false });
  assert.ok(Number.isFinite(m1.measured.height_lost_m) && Number.isFinite(m1.measured.climb_rate_2_to_3s_m_s));
});
