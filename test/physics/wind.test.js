import test from 'node:test';
import assert from 'node:assert/strict';
import { createWind, drydenLowAltitude } from '../../src/physics/wind.js';

test('Dryden filters reproduce the requested rms (frozen field, long run)', () => {
  const w = createWind({ mean: [0, 0, 0], uRms: 2, vRms: 2, wRms: 1.2, Lu: 150, Lv: 150, Lw: 30, wingspan: 1.5 }, 3);
  const dt = 1 / 120, V = 10;
  let su = 0, sv = 0, sw = 0, n = 0;
  for (let k = 0; k < 120 * 3000; k++) {
    const o = w.step(k * dt, dt, V).world;
    su += o[0] ** 2; sw += o[1] ** 2; sv += o[2] ** 2; n++;
  }
  // Time constants of 15 s and 3 s over 3000 s: sampling error a few percent.
  assert.ok(Math.abs(Math.sqrt(su / n) / 2 - 1) < 0.12, `u rms ${Math.sqrt(su / n)}`);
  assert.ok(Math.abs(Math.sqrt(sv / n) / 2 - 1) < 0.12, `v rms ${Math.sqrt(sv / n)}`);
  assert.ok(Math.abs(Math.sqrt(sw / n) / 1.2 - 1) < 0.1, `w rms ${Math.sqrt(sw / n)}`);
});

test('MIL-F-8785C low altitude: the worked values in flight-standards.md (30 m: Lw 30 m, Lu 152.5 m, ratio 1.719)', () => {
  const r = drydenLowAltitude(30);
  assert.ok(Math.abs(r.Lw_m - 30) < 0.1);
  assert.ok(Math.abs(r.Lu_m - 152.5) < 0.5);
  assert.ok(Math.abs(1 / r.ratioW - 1.719) < 0.005);
});

test('a 1-cosine gust peaks at its amplitude half way through its length and returns to zero', () => {
  const w = createWind({ mean: [0, 0, 0], uRms: 0, gusts: [{ t: 1, H: 20, amp: [0, 4, 0] }] }, 1);
  const V = 10, dt = 0.001;
  let peak = 0, tPeak = 0, end = null;
  for (let t = 0; t < 4; t += dt) {
    const y = w.step(t, dt, V).world[1];
    if (y > peak) { peak = y; tPeak = t; }
    if (t > 3.1) end = y;
  }
  assert.ok(Math.abs(peak - 4) < 1e-3 && Math.abs(tPeak - 2) < 2 * dt && Math.abs(end) < 1e-9);
});
