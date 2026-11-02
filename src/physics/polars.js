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
  return out;
}

/** Inviscid thin-airfoil: 2 pi slope, zero-lift angle from the mean line, no drag, no stall. */
export function inviscidEval(camber, alpha, kCl, out) {
  const al = alpha + 2 * camber;
  out[0] = 2 * Math.PI * kCl * al;
  out[1] = 0;
  out[2] = -0.5 * Math.PI * camber;
  out[3] = 1;
  return out;
}
