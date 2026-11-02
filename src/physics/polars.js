// Section polars. Published parameters of our own low-order model (polar-model.js), fitted to measured UIUC tables
// held locally by src/design/fit-polars.mjs. Interpolated in log Re, flat-plate past stall. Below the lowest
// fitted Re, drag grows as Re^-0.5 and lift stays at that fit. Our own CFD polars are to replace these.
// No published polar of a film-over-rib section was found, see low-re-airfoils.md: the drag factor is assumed.

import { POLAR_FITS } from './polar-fit.js';
import { sectionModel } from './polar-model.js';

function fitPolar(f, extra = {}) {
  const fits = f.fits;
  return {
    kind: 'fit', name: f.name, fits, nRe: fits.length, logRe: Float64Array.from(fits, (e) => Math.log10(e.re)),
    a: fits.reduce((p, e) => p + e.a, 0) / fits.length,
    camber: f.camber, camberPos: f.camberPos, cm0: -0.5 * Math.PI * f.camber, cdFactor: 1, ...extra,
  };
}

function analyticPolar(p) {
  return { kind: 'analytic', ...p };
}

const FILM = { cdFactor: 1.2, assumed: 'cd factor 1.2 for film over ribs: an assumption with no published source' };
export const POLARS = {
  e387: fitPolar(POLAR_FITS.e387),
  ag24: fitPolar(POLAR_FITS.ag24),
  'e387-film': fitPolar(POLAR_FITS.e387, FILM),
  'ag24-film': fitPolar(POLAR_FITS.ag24, FILM),
  // Thin symmetric tail plate. There is no table in the repository for this; values are assumed.
  'thin-plate': analyticPolar({ name: 'Thin flat plate (assumed)', a: 5.2, alphaStall: 10 * (Math.PI / 180), cd0: 0.022, reRef: 1e5, reExp: 0.5, kDrag: 0.03, camber: 0, cm0: 0, assumed: 'thin plate slope, stall and Cd0 assumed' }),
};

const FP_W = 2.5 * (Math.PI / 180);
const lo = new Float64Array(4);
const hi = new Float64Array(4);

/** Writes [cl, cd, cm, attachedFraction] for section angle alpha (rad) at Reynolds number Re. */
export function polarEval(P, alpha, Re, kCl, kCd, out) {
  let a = alpha;
  if (a > Math.PI) a -= 2 * Math.PI;
  else if (a < -Math.PI) a += 2 * Math.PI;
  if (P.kind === 'fit') {
    const lr = Math.log10(Math.max(Re, 100));
    const L = P.logRe;
    let i = 0;
    let f = 0;
    if (lr >= L[P.nRe - 1]) {
      i = P.nRe - 2;
      f = 1;
    } else if (lr > L[0]) {
      while (lr > L[i + 1]) i++;
      f = (lr - L[i]) / (L[i + 1] - L[i]);
    }
    sectionModel(P.fits[i], a, lo);
    sectionModel(P.fits[i + 1], a, hi);
    const cl = lo[0] + f * (hi[0] - lo[0]);
    let cd = lo[1] + f * (hi[1] - lo[1]);
    const sigma = lo[3] + f * (hi[3] - lo[3]);
    if (lr < L[0]) cd *= Math.pow(10, 0.5 * (L[0] - lr));
    out[0] = cl * kCl;
    out[1] = cd * kCd * P.cdFactor;
    out[2] = sigma * P.cm0 + (1 - sigma) * (-0.25 * (cl * Math.cos(a) + cd * Math.sin(a)));
    out[3] = sigma;
    // Friction part: the fit's minimum drag at this Re, scaled as cd is (used by the suction-efficiency limit).
    let cdf = P.fits[i].cdMin + f * (P.fits[i + 1].cdMin - P.fits[i].cdMin);
    if (lr < L[0]) cdf *= Math.pow(10, 0.5 * (L[0] - lr));
    if (out.length > 4) out[4] = cdf * kCd * P.cdFactor;
    return out;
  }
  const s = Math.sin(a);
  const c = Math.cos(a);
  const sigma = 1 / ((1 + Math.exp((a - P.alphaStall) / FP_W)) * (1 + Math.exp((-P.alphaStall - a) / FP_W)));
  const cdm = P.cd0 * Math.pow(Math.max(Re, 1e3) / P.reRef, -P.reExp);
  const clLin = P.a * a;
  const clFp = 1.9 * s * c;
  out[0] = (sigma * clLin + (1 - sigma) * clFp) * kCl;
  out[1] = (sigma * (cdm + P.kDrag * clLin * clLin) + (1 - sigma) * (cdm + 1.9 * s * s)) * kCd;
  out[2] = sigma * P.cm0 + (1 - sigma) * (-0.25 * clFp * c);
  out[3] = sigma;
  if (out.length > 4) out[4] = cdm * kCd;
  return out;
}

/**
 * Thin-airfoil properties of the NACA 4-digit mean line (max camber f at chord fraction p) by Glauert's integrals:
 *   alpha_L0 = -(1/pi) int_0^pi (dz/dx)(cos t - 1) dt,  cm_c/4 = (pi/4)(A2 - A1),  A_n = (2/pi) int_0^pi (dz/dx) cos(n t) dt,
 * with x = (1 - cos t)/2. For p = 0.5 (parabolic) these reduce to -2f and -pi f / 2.
 */
export function thinAirfoilCamber(f, p = 0.4, n = 4000) {
  if (!f) return { alphaL0: 0, cmC4: 0 };
  let i0 = 0, a1 = 0, a2 = 0;
  for (let k = 0; k < n; k++) {
    const t = ((k + 0.5) * Math.PI) / n;
    const x = 0.5 * (1 - Math.cos(t));
    const dz = x < p ? (2 * f * (p - x)) / (p * p) : (2 * f * (p - x)) / ((1 - p) * (1 - p));
    const w = Math.PI / n;
    i0 += dz * (Math.cos(t) - 1) * w;
    a1 += dz * Math.cos(t) * w;
    a2 += dz * Math.cos(2 * t) * w;
  }
  return { alphaL0: -i0 / Math.PI, cmC4: (Math.PI / 4) * ((2 / Math.PI) * a2 - (2 / Math.PI) * a1) };
}

/**
 * Inviscid thin-airfoil: 2 pi slope, zero-lift angle and moment from the mean line, no drag, no stall. `camber` is
 * either the max camber (parabolic mean line, round 1) or { alphaL0, cmC4 } from thinAirfoilCamber.
 */
export function inviscidEval(camber, alpha, kCl, out) {
  const tp = typeof camber === 'object' ? camber : { alphaL0: -2 * camber, cmC4: -0.5 * Math.PI * camber };
  const al = alpha - tp.alphaL0;
  out[0] = 2 * Math.PI * kCl * al;
  out[1] = 0;
  out[2] = tp.cmC4;
  out[3] = 1;
  return out;
}
