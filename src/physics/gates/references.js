// Analytic references for the method gates. Written independently of the solvers and importing none
// of them: Bessel functions by their integral representations, Theodorsen's function, Garrick's
// thrust, and Prandtl's lifting-line equation solved by Fourier collocation.

function simpson(f, a, b, n) {
  const N = n % 2 === 0 ? n : n + 1;
  const h = (b - a) / N;
  let s = f(a) + f(b);
  for (let i = 1; i < N; i++) s += f(a + i * h) * (i % 2 ? 4 : 2);
  return (s * h) / 3;
}

export function besselJ(n, x) {
  return simpson((t) => Math.cos(n * t - x * Math.sin(t)), 0, Math.PI, 2000) / Math.PI;
}

export function besselY(n, x) {
  const a = simpson((t) => Math.sin(x * Math.sin(t) - n * t), 0, Math.PI, 2000) / Math.PI;
  const sgn = n % 2 === 0 ? 1 : -1;
  // Decays as exp(-x sinh t); the cut-off below is where it has fallen below 1e-14.
  const tMax = Math.asinh(40 / x) + 1;
  const b = simpson((t) => (Math.exp(n * t) + sgn * Math.exp(-n * t)) * Math.exp(-x * Math.sinh(t)), 0, tMax, 20000) / Math.PI;
  return a - b;
}

/** Theodorsen's C(k) = F + iG with k = omega b / U. */
export function theodorsen(k) {
  const J0 = besselJ(0, k), J1 = besselJ(1, k), Y0 = besselY(0, k), Y1 = besselY(1, k);
  // C = H1^(2) / (H1^(2) + i H0^(2)); H^(2) = J - iY
  const h1r = J1, h1i = -Y1, h0r = J0, h0i = -Y0;
  const dr = h1r - h0i; // i*H0 = i(h0r + i h0i) = -h0i + i h0r
  const di = h1i + h0r;
  const den = dr * dr + di * di;
  return { F: (h1r * dr + h1i * di) / den, G: (h1i * dr - h1r * di) / den };
}

/**
 * Theodorsen lift per unit span of a plate in pure plunge, y = h0 sin(w t), y positive up.
 * With h positive down Theodorsen writes L = pi rho b^2 hdd + 2 pi rho U b C hd. Here h = -y. Returns
 * the complex amplitude L(t) = Re[ Lhat e^{i w t} ] for y = Re[ -i h0 e^{i w t} ].
 */
export function theodorsenPlungeLift({ rho, U, b, h0, omega }) {
  const k = (omega * b) / U;
  const { F, G } = theodorsen(k);
  // y = Re[Y e^{iwt}], Y = -i h0 ; ydot -> i w Y = w h0 ; ydd -> -w^2 Y = i w^2 h0
  const ydr = omega * h0, ydi = 0;
  const yddr = 0, yddi = omega * omega * h0;
  const nc = Math.PI * rho * b * b;
  const ci = 2 * Math.PI * rho * U * b;
  // L = -nc ydd - ci C yd
  const lr = -nc * yddr - ci * (F * ydr - G * ydi);
  const li = -nc * yddi - ci * (F * ydi + G * ydr);
  return { re: lr, im: li, amp: Math.hypot(lr, li), phase: Math.atan2(li, lr), k, F, G };
}

/** Garrick (1936) mean thrust coefficient for pure plunge, including leading edge suction. c = 2b. */
export function garrickPlungeCt({ k, h0, c }) {
  const { F, G } = theodorsen(k);
  const kh = (2 * k * h0) / c; // omega h0 / U
  return Math.PI * kh * kh * (F * F + G * G);
}

/**
 * Prandtl lifting-line for an untwisted wing of arbitrary symmetric chord distribution, by Fourier
 * collocation (Glauert). chord(y) with y in -b/2..b/2. Returns the lift slope per radian.
 */
export function liftingLineSlope({ AR, chordFn, b, a0 = 2 * Math.PI, nTerms = 60 }) {
  const n = nTerms;
  const M = new Array(n);
  const rhs = new Array(n);
  for (let i = 0; i < n; i++) {
    const th = (Math.PI * (i + 1)) / (2 * n + 1) * 1; // collocation in (0, pi/2] for the symmetric half
    const theta = th * 1;
    const y = -(b / 2) * Math.cos(theta);
    const c = chordFn(y);
    M[i] = new Array(n);
    for (let j = 0; j < n; j++) {
      const m = 2 * j + 1;
      M[i][j] = ((4 * b) / (a0 * c)) * Math.sin(m * theta) + (m * Math.sin(m * theta)) / Math.sin(theta);
    }
    rhs[i] = 1; // per unit alpha
  }
  // Gauss elimination with pivoting
  const A = M.map((r, i) => [...r, rhs[i]]);
  for (let k = 0; k < n; k++) {
    let p = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(A[i][k]) > Math.abs(A[p][k])) p = i;
    [A[k], A[p]] = [A[p], A[k]];
    for (let i = k + 1; i < n; i++) {
      const f = A[i][k] / A[k][k];
      for (let j = k; j <= n; j++) A[i][j] -= f * A[k][j];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let v = A[i][n];
    for (let j = i + 1; j < n; j++) v -= A[i][j] * x[j];
    x[i] = v / A[i][i];
  }
  const A1 = x[0];
  // CL = pi AR A1 ; e = 1 / (sum n A_n^2 / A1^2)
  let sum = 0;
  for (let j = 0; j < n; j++) sum += (2 * j + 1) * (x[j] / A1) ** 2;
  return { CLa: Math.PI * AR * A1, e: 1 / sum, coefficients: x };
}

export function ellipticLiftSlope(AR, a0 = 2 * Math.PI) {
  return a0 / (1 + a0 / (Math.PI * AR));
}

/**
 * Weissinger's extended lifting line (NACA TM 1120, 1947): one bound vortex per station at the quarter
 * chord, trailing legs straight back, boundary condition at the three-quarter chord. Unlike Prandtl's
 * it accounts for chord, which is what fails at low aspect ratio. Written here with plain horseshoes
 * and importing nothing from the lattice solver. Rectangular wing, chord 1, half span AR/2.
 */
export function weissingerSlope({ AR, n = 120 }) {
  const c = 1;
  const semi = AR / 2;
  // Cosine edges on the half span; symmetric images handled explicitly.
  const edge = [];
  for (let i = 0; i <= n; i++) edge.push(semi * Math.sin(((i / n) * Math.PI) / 2));
  const mid = edge.slice(0, n).map((e, i) => 0.5 * (e + edge[i + 1]));
  // Normal velocity at (x = c/2 behind the bound line, z) from a horseshoe spanning z1..z2, per unit gamma.
  const horseshoe = (x, z, z1, z2) => {
    const bound = (1 / (4 * Math.PI * x)) * ((z - z1) / Math.hypot(x, z - z1) - (z - z2) / Math.hypot(x, z - z2));
    const leg = (zv) => (1 / (4 * Math.PI * (z - zv))) * (1 + x / Math.hypot(x, z - zv));
    return -(bound + leg(z1) - leg(z2));
  };
  const A = [];
  for (let i = 0; i < n; i++) {
    A.push([]);
    for (let j = 0; j < n; j++) {
      A[i].push(horseshoe(c / 2, mid[i], edge[j], edge[j + 1]) + horseshoe(c / 2, mid[i], -edge[j + 1], -edge[j]));
    }
    A[i].push(1); // U sin(alpha) per unit, sign below
  }
  for (let k = 0; k < n; k++) {
    let p = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(A[i][k]) > Math.abs(A[p][k])) p = i;
    [A[k], A[p]] = [A[p], A[k]];
    for (let i = k + 1; i < n; i++) {
      const f = A[i][k] / A[k][k];
      for (let j = k; j <= n; j++) A[i][j] -= f * A[k][j];
    }
  }
  const g = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let v = A[i][n];
    for (let j = i + 1; j < n; j++) v -= A[i][j] * g[j];
    g[i] = v / A[i][i];
  }
  // Gamma per (U alpha); CL = 2 * integral(2 Gamma dz) / (U alpha * S) over the half span
  let sum = 0;
  for (let j = 0; j < n; j++) sum += g[j] * (edge[j + 1] - edge[j]);
  return Math.abs((4 * sum) / (AR * c * c));
}

/**
 * Published converged lifting-surface slopes for flat rectangular wings. Independent of any code here.
 * AR 1 and AR 2: Wagner 1969; Medan 1974; Hauptman & Miloh 1986, per rad.
 */
export const LIFTING_SURFACE_EXACT = [
  { AR: 1, CLa: 1.4578, source: 'Medan 1974 / Hauptman & Miloh 1986 (rectangular, AR 1), value recalled, check against the paper' },
  { AR: 2, CLa: 2.4744, source: 'Wagner 1969 / Medan 1974 (rectangular, AR 2), value recalled, check against the paper' },
];
