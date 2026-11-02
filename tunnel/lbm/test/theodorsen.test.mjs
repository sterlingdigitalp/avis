// Theodorsen's function against values tabulated in the literature and against limits, none of
// which come from this code.
import test from 'node:test';
import assert from 'node:assert/strict';
import { theodorsenC, theodorsenLift, besselJ, besselY, fitHarmonic } from '../src/theodorsen.mjs';

test('Bessel functions match Abramowitz & Stegun table 9.1', () => {
  assert.ok(Math.abs(besselJ(0, 1) - 0.7651976866) < 1e-8);
  assert.ok(Math.abs(besselJ(1, 1) - 0.4400505857) < 1e-8);
  assert.ok(Math.abs(besselY(0, 1) - 0.0882569642) < 1e-7);
  assert.ok(Math.abs(besselY(1, 1) + 0.7812128213) < 1e-7);
});

test('C(k) matches the classic table and its limits', () => {
  // F, G from Theodorsen 1935 as tabulated in Bisplinghoff, Ashley & Halfman, table 5-2.
  for (const [k, F, G] of [[0.1, 0.8319, -0.1723], [0.2, 0.7276, -0.1886], [0.5, 0.5979, -0.1507], [1.0, 0.5394, -0.1003]]) {
    const c = theodorsenC(k);
    assert.ok(Math.abs(c[0] - F) < 1e-3, `F(${k})=${c[0]} vs ${F}`);
    assert.ok(Math.abs(c[1] - G) < 1e-3, `G(${k})=${c[1]} vs ${G}`);
  }
  const lo = theodorsenC(1e-3), hi = theodorsenC(50);
  assert.ok(Math.abs(lo[0] - 1) < 0.02 && Math.abs(lo[1]) < 0.02);
  assert.ok(Math.abs(hi[0] - 0.5) < 0.01 && Math.abs(hi[1]) < 0.01);
});

test('C(k) agrees with the independent R.T. Jones rational approximation', () => {
  for (const k of [0.1, 0.25, 0.5, 1]) {
    const c = theodorsenC(k);
    // C = 1 - 0.165/(1 - 0.0455 i/k) - 0.335/(1 - 0.3 i/k), a fit known good to a few percent only.
    const term = (A, B) => { const d = 1 + (B / k) ** 2; return [A / d, (A * (B / k)) / d]; };
    const t1 = term(0.165, 0.0455), t2 = term(0.335, 0.3);
    const jones = [1 - t1[0] - t2[0], -t1[1] - t2[1]];
    assert.ok(Math.hypot(c[0] - jones[0], c[1] - jones[1]) < 0.03, `k=${k}`);
  }
});

test('Quasi-steady limit of the lift response', () => {
  // k -> 0: alpha mode gives CL = 2 pi alpha; plunge gives CL = -2 pi hdot/U.
  const a = theodorsenLift({ k: 1e-3, alpha0: 0.01 });
  assert.ok(Math.abs(a.amp / (2 * Math.PI * 0.01) - 1) < 0.02);
  const k = 1e-3, h0 = 0.1, w = 2 * k;
  const p = theodorsenLift({ k, h0 });
  assert.ok(Math.abs(p.amp / (2 * Math.PI * h0 * w) - 1) < 0.02);
  // Plunging up (h ~ sin) makes lift ~ -cos: phasor angle -90 deg.
  assert.ok(Math.abs(p.phaseRad + Math.PI / 2) < 0.05);
});

test('High-frequency plunge is the added-mass limit', () => {
  // CL -> (pi b^2 w^2 h0)/(0.5 U^2 c) in phase with h, with circulation only a correction.
  const k = 20, b = 0.5, w = k / b, h0 = 0.001;
  const r = theodorsenLift({ k, h0 });
  const am = (Math.PI * b * b * w * w * h0) / 0.5;
  assert.ok(Math.abs(r.Z[0] / am - 1) < 0.03);
});

test('fitHarmonic recovers a known signal', () => {
  const w = 0.3, ts = [], ys = [];
  for (let i = 0; i < 500; i++) { ts.push(i * 0.1); ys.push(1.2 + 2 * Math.sin(w * ts[i]) - 0.7 * Math.cos(w * ts[i])); }
  const f = fitHarmonic(ts, ys, w);
  assert.ok(Math.abs(f.Z[0] - 2) < 1e-9 && Math.abs(f.Z[1] + 0.7) < 1e-9 && Math.abs(f.mean - 1.2) < 1e-9);
  assert.ok(Math.abs(f.amp - Math.hypot(2, 0.7)) < 1e-9 && Math.abs(f.phaseRad - Math.atan2(-0.7, 2)) < 1e-9);
  // A short, unevenly sampled record with a large mean makes the normal equations far from diagonal.
  const t2 = [0.1, 0.4, 0.9, 1.7, 2.2, 3.1, 3.3], y2 = t2.map((v) => 50 - 3 * Math.sin(2 * v) + 4 * Math.cos(2 * v));
  const g = fitHarmonic(t2, y2, 2);
  assert.ok(Math.abs(g.mean - 50) < 1e-8 && Math.abs(g.Z[0] + 3) < 1e-8 && Math.abs(g.Z[1] - 4) < 1e-8);
});

test('Lift response at k = 0.5, worked by hand from tabulated C(0.5) = 0.5979 - 0.1507 i', () => {
  // With U = 1, c = 1, b = 1/2, w = 1. Non-circulatory (pi/2)(w^2 h + i w alpha + a b w^2 alpha) and
  // circulatory 2 pi C (-i w h + alpha + i w b (1/2 - a) alpha), each per unit motion:
  //   plunge                 0.624 - 3.757 i
  //   pitch about mid-chord  3.994 + 1.563 i
  //   pitch about c/4        3.838 + 2.503 i
  const near = (z, re, im, what) => assert.ok(Math.abs(z[0] - re) < 0.004 && Math.abs(z[1] - im) < 0.004, `${what}: ${z} vs ${re} ${im}`);
  const per = (o, s) => theodorsenLift({ k: 0.5, ...o }).Z.map((v) => v / s);
  near(per({ h0: 0.05 }, 0.05), 0.624, -3.757, 'plunge');
  near(per({ alpha0: 0.05 }, 0.05), 3.994, 1.563, 'pitch mid-chord');
  near(per({ alpha0: 0.05, axisC: -0.25 }, 0.05), 3.838, 2.503, 'pitch quarter-chord');
  // Pitch leading plunge by 90 degrees: the alpha phasor is multiplied by i.
  const z = per({ alpha0: 0.05, phi: Math.PI / 2 }, 0.05);
  near(z, -1.563, 3.994, 'pitch with phase');
  // Superposition, and dimensional scaling: lift coefficient depends only on k and h/c.
  const both = theodorsenLift({ k: 0.5, h0: 0.05, alpha0: 0.05 }).Z;
  near([both[0] / 0.05, both[1] / 0.05], 0.624 + 3.994, -3.757 + 1.563, 'superposition');
  const big = theodorsenLift({ k: 0.5, chord: 2, U: 3, h0: 0.1 }).Z;
  near([big[0] / 0.05, big[1] / 0.05], 0.624, -3.757, 'scaled');
});
