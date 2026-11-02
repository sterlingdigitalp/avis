import test from 'node:test';
import assert from 'node:assert/strict';
import { segmentsVelocity, createUvlm } from '../../src/physics/uvlm.js';
import { plateSurface, runPlunge } from '../../src/physics/gates/plate.js';
import { theodorsen, besselJ, besselY, theodorsenPlungeLift, garrickPlungeCt, liftingLineSlope, ellipticLiftSlope, weissingerSlope } from '../../src/physics/gates/references.js';

test('Biot-Savart of a long straight filament equals Gamma / (2 pi r)', () => {
  const S = Float64Array.from([-1e4, 0, 0, 1e4, 0, 0, 1]);
  const u = new Float64Array(3);
  segmentsVelocity(0, 0, 2, S, 1, 0, u);
  // Filament along +x, point at +z: right-hand rule gives velocity along x cross z = -y.
  assert.ok(Math.abs(u[1] + 1 / (2 * Math.PI * 2)) < 1e-8, `uy ${u[1]}`);
  assert.ok(Math.abs(u[0]) < 1e-12 && Math.abs(u[2]) < 1e-12);
});

test('reference functions match published values', () => {
  // Abramowitz & Stegun table 9.1 (x = 1)
  assert.ok(Math.abs(besselJ(0, 1) - 0.7651976866) < 1e-8);
  assert.ok(Math.abs(besselJ(1, 1) - 0.4400505857) < 1e-8);
  assert.ok(Math.abs(besselY(0, 1) - 0.0882569642) < 1e-7);
  assert.ok(Math.abs(besselY(1, 1) + 0.7812128213) < 1e-7);
  // Theodorsen's own table (NACA 496)
  for (const [k, F, G] of [[0.1, 0.8319, -0.1723], [0.2, 0.7276, -0.1886], [0.5, 0.5979, -0.1507], [1.0, 0.5394, -0.1003]]) {
    const c = theodorsen(k);
    assert.ok(Math.abs(c.F - F) < 5e-4, `F(${k})=${c.F}`);
    assert.ok(Math.abs(c.G - G) < 5e-4, `G(${k})=${c.G}`);
  }
  // High frequency limit of Garrick: Ct -> pi (k h / b ... ) (F^2+G^2) with F -> 1/2
  assert.ok(Math.abs(garrickPlungeCt({ k: 50, h0: 0.01, c: 1 }) / (Math.PI * (2 * 50 * 0.01) ** 2 * 0.25) - 1) < 0.02);
  // Lifting line: elliptic planform recovers a0/(1+a0/(pi AR)) exactly and has e = 1
  const AR = 6, b = 6, cr = (4 * (b / AR)) / Math.PI;
  const ll = liftingLineSlope({ AR, b, chordFn: (y) => cr * Math.sqrt(Math.max(1e-14, 1 - (2 * y / b) ** 2)) });
  assert.ok(Math.abs(ll.CLa - ellipticLiftSlope(AR)) < 1e-6);
  assert.ok(Math.abs(ll.e - 1) < 1e-6);
  // Rectangular Prandtl slope: Glauert tau small and positive
  const r = liftingLineSlope({ AR: 6, b: 6, chordFn: () => 1 });
  assert.ok(r.CLa < ellipticLiftSlope(6) && r.e < 1 && r.e > 0.9);
  // Weissinger tends to Prandtl as AR grows and sits below at low AR
  assert.ok(weissingerSlope({ AR: 4 }) < liftingLineSlope({ AR: 4, b: 4, chordFn: () => 1 }).CLa);
});

test('sign: a wing moving down makes lift up, and lift is in phase with plunge rate', () => {
  // Plate moves down at constant speed w: steady state must be lift up, like incidence atan(w/U).
  const U = 10, w = 0.5, chord = 1, rho = 1.2, AR = 12;
  const { surface } = plateSurface({ chord, AR, nc: 4, ns: 12, plunge: (t) => -w * t });
  const sim = createUvlm([surface], { Vinf: [-U, 0, 0], rho, dt: chord / (2 * U), symmetric: true, maxWakeRows: 80 });
  let f;
  for (let n = 0; n < 80; n++) f = sim.advance(n === 79);
  assert.ok(f.force[1] > 0, `lift ${f.force[1]}`);
  // Same as the plate at the same effective incidence, within a small margin
  const { surface: s2 } = plateSurface({ chord, AR, nc: 4, ns: 12 });
  const a = Math.atan(w / U);
  const sim2 = createUvlm([s2], { Vinf: [-U * Math.cos(a), U * Math.sin(a), 0], rho, dt: chord / (2 * U), symmetric: true, maxWakeRows: 80 });
  let g;
  for (let n = 0; n < 80; n++) g = sim2.advance(n === 79);
  assert.ok(Math.abs(f.force[1] / g.force[1] - 1) < 0.01, `${f.force[1]} vs ${g.force[1]}`);
});

test('sign: an updraft gust raises lift, a downdraft lowers it', () => {
  const U = 10, chord = 1, rho = 1.2, AR = 8;
  const run = (wg) => {
    const { surface } = plateSurface({ chord, AR, nc: 4, ns: 8, alpha: 0.05 });
    const sim = createUvlm([surface], { Vinf: [-U, wg, 0], rho, dt: chord / (2 * U), symmetric: true, maxWakeRows: 60 });
    let f;
    for (let n = 0; n < 60; n++) f = sim.advance(n === 59);
    return f.force[1];
  };
  const base = run(0);
  assert.ok(run(+1) > base);
  assert.ok(run(-1) < base);
});

test('induced drag of a steady wing is close to CL^2/(pi AR e), and points aft', () => {
  const U = 10, chord = 1, rho = 1, AR = 8, alpha = 0.06;
  const { surface } = plateSurface({ chord, AR, nc: 4, ns: 24 });
  const Vinf = [-U * Math.cos(alpha), U * Math.sin(alpha), 0];
  const sim = createUvlm([surface], { Vinf, rho, dt: chord / (2 * U), symmetric: true, maxWakeRows: 120 });
  let f;
  for (let n = 0; n < 120; n++) f = sim.advance(n === 119);
  const q = 0.5 * rho * U * U, S = (AR * chord * chord) / 2;
  // Drag is along the air's travel direction. Air moves along Vinf, so drag = F . Vinf/U.
  const D = (f.force[0] * Vinf[0] + f.force[1] * Vinf[1]) / U;
  const L = f.force[1] * Math.cos(alpha) + f.force[0] * Math.sin(alpha);
  const CL = L / (q * S), CD = D / (q * S);
  assert.ok(CD > 0, `CD ${CD}`);
  const eff = (CL * CL) / (Math.PI * AR * CD);
  // A rectangular wing has e about 0.9-1; the prescribed wake and small tilt keep it near there.
  assert.ok(eff > 0.8 && eff < 1.05, `e = ${eff}`);
});

test('symmetric half model agrees with the full two-wing model', () => {
  const U = 10, chord = 0.5, rho = 1;
  const run = (full) => {
    const a = plateSurface({ chord, AR: 6, nc: 3, ns: 6, cosine: false, alpha: 0.1 });
    const surfaces = [a.surface];
    if (full) {
      const n = a.surface.nc, m = a.surface.ns;
      surfaces.push({ nc: n, ns: m, grid(t, out) {
        // Mirror the right wing, with j reversed so that j still runs in +z.
        const tmp = new Float64Array(out.length);
        a.surface.grid(t, tmp);
        for (let i = 0; i <= n; i++) for (let j = 0; j <= m; j++) {
          const p = 3 * (i * (m + 1) + j), q = 3 * (i * (m + 1) + (m - j));
          out[q] = tmp[p]; out[q + 1] = tmp[p + 1]; out[q + 2] = -tmp[p + 2];
        }
      } });
    }
    const sim = createUvlm(surfaces, { Vinf: [-U, 0, 0], rho, dt: chord / (2 * U), symmetric: !full, maxWakeRows: 30 });
    let f;
    for (let n = 0; n < 30; n++) f = sim.advance(n === 29);
    return f.force[1] * (full ? 1 : 2);
  };
  const a = run(false), b = run(true);
  assert.ok(Math.abs(a / b - 1) < 1e-6, `${a} vs ${b}`);
});

test('V1 condition at one k: amplitude and phase track Theodorsen', () => {
  const U = 10, chord = 1, rho = 1, h0 = 0.02;
  const r = runPlunge({ U, chord, k: 0.3, h0, AR: 20, nc: 6, ns: 6, rho, periods: 2.5, wakePeriods: 2 });
  const th = theodorsenPlungeLift({ rho, U, b: 0.5, h0, omega: r.omega });
  assert.ok(Math.abs(r.L.amp / th.amp - 1) < 0.05);
  const dphi = ((((Math.atan2(-r.L.b, r.L.a) - th.phase) * 180) / Math.PI + 540) % 360) - 180;
  assert.ok(Math.abs(dphi) < 5, `phase error ${dphi}`);
});
