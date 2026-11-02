// Trim convergence (SPEC: a trim that does not converge is a failure) and the stroke envelope from ratings.
import test from 'node:test';
import assert from 'node:assert/strict';
import { buildBird } from '../../src/physics/flight.js';
import { trimLevel, TRIM_TOL, trainCrankRating, strokeLimits, strokeEnvelope, batteryPower } from '../../src/design/trim.js';
import { batteryModel } from '../../src/design/catalog.js';
import { testDesign } from '../physics/fixtures.js';

const DEG = Math.PI / 180;
function article() {
  const d = testDesign();
  d.geometry = { chord_root_m: 0.25 };
  d.structure = {
    gear_stages: [{ ratio: 5, gear_allowable_Nm: 2, pinion_allowable_Nm: 0.5 }, { ratio: 6, gear_allowable_Nm: 30, pinion_allowable_Nm: 6 }],
    spar_inner: { I: (Math.PI * (0.016 ** 4 - 0.014 ** 4)) / 64, c: 0.008, allowable_MPa: 650 },
  };
  return d;
}

test('gear train rating: the weakest of each gear and pinion, reflected to the crank (hand calculation)', () => {
  const st = [{ ratio: 80 / 12, gear_allowable_Nm: 2.55, pinion_allowable_Nm: 0.22 }, { ratio: 40 / 18, gear_allowable_Nm: 34.6, pinion_allowable_Nm: 6.37 }];
  const hand = Math.min(2.55 * (40 / 18), 0.22 * (80 / 12) * (40 / 18), 34.6, 6.37 * (40 / 18));
  assert.ok(Math.abs(trainCrankRating(st) - hand) < 1e-12);
  assert.ok(Math.abs(hand - 3.259) < 0.001);
});

test('a converged trim is below the declared tolerance; an impossible one reports converged = false', () => {
  const d = article();
  const bird = buildBird(d);
  const t = trimLevel(d, bird, { V: 10, amp: 15 * DEG, dyn: 0.15, maxFreq: 8 });
  assert.equal(t.converged, true, JSON.stringify(t.residuals));
  assert.ok(t.residual_norm_frac_weight <= TRIM_TOL);
  const heavy = article();
  for (const l of heavy.mass.lumps) l.mass_kg *= 12;
  const tb = trimLevel(heavy, buildBird(heavy), { V: 10, amp: 15 * DEG, dyn: 0.15, maxFreq: 4 });
  assert.equal(tb.converged, false);
  assert.ok(tb.residual_norm_frac_weight > TRIM_TOL);
});

test('stroke envelope: every rating holds at the envelope frequency and one breaks just above it', () => {
  const d = article();
  const bird = buildBird(d);
  const row = { V: 10, theta: 0.06, tail: 0, freq: 3, sym: 0, dyn: 0.15, lead: 0 };
  const env = strokeEnvelope(d, bird, [row], { nAmp: 2 });
  for (const e of env) {
    if (!(e.freqMax > 0) || e.limitedBy === 'search cap') continue;
    assert.ok(strokeLimits(d, bird, row, e.freqMax, e.amp).ok, `at ${e.freqMax} Hz`);
    const above = strokeLimits(d, bird, row, e.freqMax * 1.1, e.amp);
    assert.equal(above.ok, false, `10% above (${e.freqMax * 1.1} Hz) should break ${e.limitedBy}`);
  }
});

test('battery model: resistance from the claimed C rate, nominal-voltage shift for Li-ion (hand calculation)', () => {
  const b = batteryModel({ id: 'x', mass_g: 150, specs: { capacity_mah: 3000, c_rating_continuous_claimed: 10, cells_series: 3, nominal_voltage_v: 10.8, chemistry: 'Li-ion' } });
  assert.ok(Math.abs(b.R_int - (0.1 * 3 * 3.7) / 30) < 1e-12);
  assert.equal(b.I_cont, 30);
  assert.ok(Math.abs(b.cell_shift_V + 0.1) < 1e-12);
  assert.equal(batteryPower(-5, { eta: 0.9 }), 0);
  assert.ok(Math.abs(batteryPower(9, { eta: 0.9 }) - 10) < 1e-12);
});
