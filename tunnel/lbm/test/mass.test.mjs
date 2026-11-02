// Closed-box mass summary against series with a known drift and a known budget.
import test from 'node:test';
import assert from 'node:assert/strict';
import { massSeries } from '../src/mass.mjs';

test('drift per cycle is the slope of a known linear trend under an oscillation orthogonal to it', () => {
  const cells = 1000, b = -2.5e-6, pts = [];
  // An oscillation even about the middle cycle is orthogonal to the trend, so the slope is exact.
  const osc = (i) => 3e-6 * Math.cos((2 * Math.PI * (i - 24.5)) / 7);
  for (let i = 0; i < 50; i++) pts.push({ deltaMass: cells * (1e-5 + b * i + osc(i)), cells, budget: 7 * i + 3, fixApplied: -i - 2 });
  const s = massSeries(pts);
  assert.ok(Math.abs(s.slopePerCycle - b) < 1e-15, `slope ${s.slopePerCycle}`);
  assert.ok(Math.abs(s.meanDrhoEnd - (1e-5 + 49 * b + osc(49))) < 1e-15);
  assert.ok(Math.abs(s.meanDrhoFirst - (1e-5 + osc(0))) < 1e-15);
  assert.deepEqual(s.budgetCheck, { massChange: pts[49].deltaMass - pts[0].deltaMass, budget: 343, fixApplied: -49 });
  assert.deepEqual(s.series, [0, 5, 10, 15, 20, 25, 30, 35, 40, 45].map((i) => pts[i].deltaMass / cells));
  assert.equal(massSeries([pts[0], pts[0]]).slopePerCycle, 0);
  assert.equal(massSeries([pts[0]]).budgetCheck, null);
});
