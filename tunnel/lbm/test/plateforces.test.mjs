// Lift extraction against a synthetic signal of known amplitude and phase, sampled the way a plate
// run samples it, at the sampling parameters of every grid level of the V6 study.
import test from 'node:test';
import assert from 'node:assert/strict';
import { liftHarmonic, sampleTimes } from '../src/plateforces.mjs';

test('sample times are window centres, the last window shorter', () => {
  assert.deepEqual(sampleTimes(10, 5), [2.5, 7.5]);
  assert.deepEqual(sampleTimes(11, 5), [2.5, 7.5, 10.5]);
});

// Without harmonics the tolerance is below the window attenuation (0.018% at 96 samples a cycle),
// so a missing correction fails. With them, the window is not a whole number of periods (by up to
// 0.26%), and the leak of 10% and 5% harmonics sets the tolerance.
for (const [label, h, ampTol, phaseTol] of [['a pure harmonic and drift', 0, 5e-5, 0.005], ['second and third harmonics and drift', 1, 3e-4, 0.03]]) test(`lift amplitude and phase come back from window-averaged samples: ${label}`, () => {
  const k = 0.5, U = 0.05, A = 0.22, phi = 0.45;
  for (const c of [40, 80, 113, 160, 226]) {
    const omega = (2 * k * U) / c, period = (2 * Math.PI) / omega;
    const M = Math.max(1, Math.round(period / 96)), total = Math.round(6 * period);
    // Half amplitude in the first two cycles, a start-up transient that no fitted window may see.
    const sig = (s) => (s < 2 * period ? -0.5 * A * Math.sin(omega * s + phi) : 0) + 0.003 - (1e-4 * s) / total + A * Math.sin(omega * s + phi) + h * (0.1 * A * Math.sin(2 * omega * s + 0.3) + 0.05 * A * Math.sin(3 * omega * s + 1));
    const t = sampleTimes(total, M), cl = [], cd = [];
    for (let n = 0; n < total; n += M) {
      const steps = Math.min(M, total - n);
      let sum = 0;
      for (let j = 0; j < steps; j++) sum += sig(n + j + 0.5);
      cl.push(sum / steps);
      cd.push(0.04);
    }
    const r = liftHarmonic({ t, cl, cd }, { omega, period, M });
    assert.ok(Math.abs(r.cl.amp / A - 1) < ampTol, `c${c}: amplitude ${r.cl.amp} vs ${A}`);
    assert.ok(Math.abs(r.cl.phaseDeg - (phi * 180) / Math.PI) < phaseTol, `c${c}: phase ${r.cl.phaseDeg} vs ${(phi * 180) / Math.PI}`);
    assert.ok(Math.abs(r.cl.prevAmp / A - 1) < ampTol, `c${c}: previous-cycles amplitude ${r.cl.prevAmp}`);
    assert.ok(Math.abs(r.cl.prevPhaseDeg - (phi * 180) / Math.PI) < phaseTol, `c${c}: previous-cycles phase ${r.cl.prevPhaseDeg}`);
    assert.ok(Math.abs(r.meanCd - 0.04) < 1e-12);
  }
});
