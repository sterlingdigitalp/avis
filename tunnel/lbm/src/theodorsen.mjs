// Theodorsen (1935) unsteady thin-aerofoil lift, computed from first principles and shared with
// nothing else in the project. Bessel functions come from their integral representations
// (Abramowitz & Stegun 9.1.21, 9.1.22) by Simpson quadrature, not from a fitted approximation.

function simpson(fn, a, b, n) {
  const h = (b - a) / n;
  let s = fn(a) + fn(b);
  for (let i = 1; i < n; i++) s += fn(a + i * h) * (i % 2 ? 4 : 2);
  return (s * h) / 3;
}

export function besselJ(nu, x) {
  return simpson((t) => Math.cos(nu * t - x * Math.sin(t)), 0, Math.PI, 4000) / Math.PI;
}

export function besselY(nu, x) {
  const a = simpson((t) => Math.sin(x * Math.sin(t) - nu * t), 0, Math.PI, 4000) / Math.PI;
  // Integrand decays double-exponentially; 12/x is safely past it for the x used here.
  const tMax = Math.asinh(60 / x) + 1;
  const b = simpson((t) => (Math.exp(nu * t) + (nu % 2 ? -1 : 1) * Math.exp(-nu * t)) * Math.exp(-x * Math.sinh(t)), 0, tMax, 40000) / Math.PI;
  return a - b;
}

const cMul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
const cDiv = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
const cAdd = (a, b) => [a[0] + b[0], a[1] + b[1]];

/** Theodorsen's function C(k) = H1(k) / (H1(k) + i H0(k)), with Hankel functions of the second kind. */
export function theodorsenC(k) {
  const H1 = [besselJ(1, k), -besselY(1, k)];
  const H0 = [besselJ(0, k), -besselY(0, k)];
  return cDiv(H1, cAdd(H1, cMul([0, 1], H0)));
}

/**
 * Complex lift-coefficient response of a 2D flat plate. Motion is h(t) = h0 sin(wt) positive up and
 * alpha(t) = alpha0 sin(wt + phi) positive nose-up, about a point axisC chords aft of mid-chord.
 * Response: CL(t) = Re(Z) sin(wt) + Im(Z) cos(wt).
 *
 * L/rho = pi b^2 (-hdd + U alphad - a b alphadd) + 2 pi U b C(k) (-hd + U alpha + b (1/2 - a) alphad)
 * which is Bisplinghoff, Ashley & Halfman eq. 5-308 with their downward h changed to upward.
 */
export function theodorsenLift({ k, chord = 1, U = 1, h0 = 0, alpha0 = 0, phi = 0, axisC = 0 }) {
  const b = chord / 2, w = (k * U) / b, a = axisC / 0.5;
  const C = theodorsenC(k);
  // Phasors with h = Im(h0 e^{iwt}): d/dt multiplies by iw.
  const iw = [0, w];
  const h = [h0, 0];
  const al = [alpha0 * Math.cos(phi), alpha0 * Math.sin(phi)];
  const hd = cMul(iw, h), hdd = cMul(iw, hd), ald = cMul(iw, al), aldd = cMul(iw, ald);
  const nonCirc = [
    -hdd[0] + U * ald[0] - a * b * aldd[0],
    -hdd[1] + U * ald[1] - a * b * aldd[1],
  ].map((v) => Math.PI * b * b * v);
  const circIn = [
    -hd[0] + U * al[0] + b * (0.5 - a) * ald[0],
    -hd[1] + U * al[1] + b * (0.5 - a) * ald[1],
  ];
  const circ = cMul(C, circIn).map((v) => 2 * Math.PI * U * b * v);
  const L = cAdd(nonCirc, circ);
  const q = 0.5 * U * U * chord;
  const Z = [L[0] / q, L[1] / q];
  return { Z, amp: Math.hypot(Z[0], Z[1]), phaseRad: Math.atan2(Z[1], Z[0]), C };
}

/** Fit a + A sin(wt) + B cos(wt) by least squares. Returns Z = A + iB, the same phasor as above. */
export function fitHarmonic(ts, ys, w) {
  const m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], r = [0, 0, 0];
  for (let i = 0; i < ts.length; i++) {
    const v = [1, Math.sin(w * ts[i]), Math.cos(w * ts[i])];
    for (let p = 0; p < 3; p++) { r[p] += v[p] * ys[i]; for (let q = 0; q < 3; q++) m[p][q] += v[p] * v[q]; }
  }
  for (let p = 0; p < 3; p++) {
    let best = p;
    for (let q = p + 1; q < 3; q++) if (Math.abs(m[q][p]) > Math.abs(m[best][p])) best = q;
    [m[p], m[best]] = [m[best], m[p]]; [r[p], r[best]] = [r[best], r[p]];
    for (let q = p + 1; q < 3; q++) {
      const f = m[q][p] / m[p][p];
      for (let c = p; c < 3; c++) m[q][c] -= f * m[p][c];
      r[q] -= f * r[p];
    }
  }
  const x = [0, 0, 0];
  for (let p = 2; p >= 0; p--) {
    let s = r[p];
    for (let q = p + 1; q < 3; q++) s -= m[p][q] * x[q];
    x[p] = s / m[p][p];
  }
  return { mean: x[0], Z: [x[1], x[2]], amp: Math.hypot(x[1], x[2]), phaseRad: Math.atan2(x[2], x[1]) };
}
