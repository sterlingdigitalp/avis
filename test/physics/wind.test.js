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

test('turbulence runs on its own clock: halving the flight step samples the same air at shared instants', () => {
  const spec = { mean: [6, 0, 0], uRms: 2, vRms: 2, wRms: 1.2, Lu: 150, Lv: 150, Lw: 30, wingspan: 1.5 };
  const a = createWind(spec, 7), b = createWind(spec, 7);
  const V = 10;
  let worst = 0;
  const dtA = 1 / 240, dtB = 1 / 480;
  for (let k = 0; k < 240 * 20; k++) {
    const oa = a.step(k * dtA, dtA, V).world.slice();
    b.step((2 * k) * dtB, dtB, V);
    const ob = b.step((2 * k) * dtB, dtB, V).world; // a repeat at the same instant changes nothing
    worst = Math.max(worst, Math.abs(oa[0] - ob[0]), Math.abs(oa[1] - ob[1]), Math.abs(oa[2] - ob[2]));
    b.step((2 * k + 1) * dtB, dtB, V);
  }
  assert.ok(worst < 1e-12, `largest difference ${worst} m/s`);
});

test('between its own clock ticks the turbulence is the straight-line blend of the two neighbouring samples', () => {
  const spec = { mean: [0, 0, 0], uRms: 2, vRms: 2, wRms: 1.2, Lu: 150, Lv: 150, Lw: 30, wingspan: 1.5, gridDt: 0.01 };
  const w = createWind(spec, 11);
  const V = 10;
  let worst = 0;
  for (let k = 0; k < 200; k++) {
    const a = w.step(0.01 * k, 0.0025, V).world.slice();
    const q = w.step(0.01 * k + 0.0025, 0.0025, V).world.slice();
    const h = w.step(0.01 * k + 0.005, 0.0025, V).world.slice();
    const b = w.step(0.01 * (k + 1), 0.0025, V).world.slice();
    for (let j = 0; j < 3; j++) worst = Math.max(worst, Math.abs(h[j] - 0.5 * (a[j] + b[j])), Math.abs(q[j] - (0.75 * a[j] + 0.25 * b[j])));
  }
  assert.ok(worst < 1e-12, `interpolation error ${worst}`);
});

test("Dryden lateral and vertical correlation (MIL-F-8785C): R(tau) = sigma^2 (1 - tau/(2T)) exp(-tau/T), zero at tau = 2T", () => {
  const T = 2; // L/V
  const w = createWind({ mean: [0, 0, 0], uRms: 0.0001, vRms: 1, wRms: 1, Lu: 20, Lv: 20, Lw: 20, wingspan: 1.5, gridDt: 0.02 }, 5);
  const V = 10, dt = 0.02, n = 150000;
  const vs = new Float64Array(n);
  for (let k = 0; k < n; k++) vs[k] = w.step(k * dt, dt, V).world[1];
  const R = (lag) => {
    let s = 0, s0 = 0;
    for (let k = 0; k + lag < n; k++) { s += vs[k] * vs[k + lag]; s0 += vs[k] * vs[k]; }
    return s / s0;
  };
  const r1 = R(Math.round(T / dt)), r2 = R(Math.round((2 * T) / dt)), rHalf = R(Math.round(T / (2 * dt)));
  assert.ok(Math.abs(r1 - 0.5 * Math.exp(-1)) < 0.05, `R(T) ${r1} vs ${0.5 * Math.exp(-1)}`);
  assert.ok(Math.abs(r2) < 0.05, `R(2T) ${r2} vs 0`);
  assert.ok(Math.abs(rHalf - 0.75 * Math.exp(-0.5)) < 0.05, `R(T/2) ${rHalf} vs ${0.75 * Math.exp(-0.5)}`);
});
