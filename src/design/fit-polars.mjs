#!/usr/bin/env node
// Fits our low-order section model to the measured UIUC tables held locally (gitignored, licence unclear)
// and writes only the fitted parameters to src/physics/polar-fit.js. No table row leaves the local files.
// Model, per Reynolds number: linear lift to a stall angle, then a flat-plate branch; a quadratic drag bucket
// with separate curvature above and below the lift of minimum drag; round 11: the negative stall kept inside the
// measured sweep, with its own transition width.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { sectionModel } from '../physics/polar-model.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const fE = join(ROOT, 'data/references/airfoil-e387.json');
const fA = join(ROOT, 'data/references/airfoils/uiuc-vol5-ag-polars.json');
if (!existsSync(fE)) {
  console.error('local UIUC tables absent: nothing to fit');
  process.exit(1);
}

function upSweep(rows) {
  const out = [];
  for (const r of rows) {
    if (out.length && r[0] <= out[out.length - 1][0]) break;
    out.push(r);
  }
  return out;
}

function nelderMead(f, x0, steps, iters = 3000) {
  const n = x0.length;
  let sim = [x0.slice(), ...x0.map((_, i) => x0.map((v, j) => (j === i ? v + steps[i] : v)))];
  let val = sim.map(f);
  for (let it = 0; it < iters; it++) {
    const ord = val.map((v, i) => [v, i]).sort((a, b) => a[0] - b[0]).map((p) => p[1]);
    sim = ord.map((i) => sim[i]);
    val = ord.map((i) => val[i]);
    const c = Array.from({ length: n }, (_, j) => sim.slice(0, n).reduce((a, p) => a + p[j], 0) / n);
    const refl = c.map((v, j) => 2 * v - sim[n][j]);
    const fr = f(refl);
    if (fr < val[0]) {
      const e = c.map((v, j) => 3 * v - 2 * sim[n][j]);
      const fe = f(e);
      if (fe < fr) { sim[n] = e; val[n] = fe; } else { sim[n] = refl; val[n] = fr; }
    } else if (fr < val[n - 1]) { sim[n] = refl; val[n] = fr; } else {
      const k = c.map((v, j) => 0.5 * (v + sim[n][j]));
      const fk = f(k);
      if (fk < val[n]) { sim[n] = k; val[n] = fk; } else {
        sim = sim.map((p) => p.map((v, j) => sim[0][j] + 0.5 * (v - sim[0][j])));
        val = sim.map(f);
      }
    }
  }
  return sim[0];
}

const DEG = Math.PI / 180;
const out = [0, 0, 0, 0];
const params = (x) => ({ a: x[0], aL0: x[1], aStall: x[2], aStallNeg: x[3], cdMin: Math.abs(x[4]), clD: x[5], kUp: Math.abs(x[6]), kLo: Math.abs(x[7]), wNeg: Math.abs(x[8]) });
/** Quadratic penalty outside [lo, hi]. */
const outside = (v, lo, hi) => (v < lo ? 1e6 * (v - lo) ** 2 : v > hi ? 1e6 * (v - hi) ** 2 : 0);

/**
 * Least squares on the ascending sweep. A stall angle beyond the measured sweep is unidentified, so both stall angles
 * are kept where the data can speak: the positive one at most 13.5 deg, the negative one (round 11) no lower than the
 * sweep's lowest angle. Without that guard the E387 fit put the negative stall at -53 to -253 deg (-0.93 to -4.42 rad)
 * and the section stayed attached through the upstroke, while the tables show the drag doubling between -5.2 and -6.3 deg
 * and the cfd workstream's sections stall near alpha_E -6 deg. The negative transition half-width wNeg is fitted
 * within 0.5-3 deg (the tables' 1 deg spacing resolves no less).
 */
function fit(rows) {
  const up = upSweep(rows);
  const aMin = up[0][0] * DEG;
  const obj = (x) => {
    const p = params(x);
    let s = x[2] > 13.5 * DEG ? 1e6 * (x[2] - 13.5 * DEG) ** 2 : 0;
    s += outside(x[3], aMin, 0) + outside(p.wNeg, 0.5 * DEG, 3 * DEG);
    for (const [al, cl, cd] of up) {
      sectionModel(p, al * DEG, out);
      s += ((out[0] - cl) / 0.05) ** 2 + ((out[1] - cd) / (0.15 * cd + 0.002)) ** 2;
    }
    return s;
  };
  const steps = [0.5, 0.02, 0.03, 0.03, 0.003, 0.1, 0.01, 0.02, 0.005];
  // Two passes: the restart from the first optimum re-spans the simplex, which a 9-parameter fit needs to settle.
  const x1 = nelderMead(obj, [5.5, -3.5 * DEG, 11 * DEG, -7 * DEG, 0.015, 0.5, 0.02, 0.05, 2 * DEG], steps, 6000);
  const x = nelderMead(obj, x1, steps.map((v) => v / 3), 6000);
  const p = params(x);
  // Fit quality, reported as summary numbers only: -3..10 deg as before, and the negative angles below -3 deg.
  let eCl = 0, eCd = 0, eClN = 0, eCdN = 0;
  for (const [al, cl, cd] of up) {
    if (al > 10) continue;
    sectionModel(p, al * DEG, out);
    const dl = Math.abs(out[0] - cl), dd = Math.abs(out[1] / cd - 1);
    if (al >= -3) { eCl = Math.max(eCl, dl); eCd = Math.max(eCd, dd); } else { eClN = Math.max(eClN, dl); eCdN = Math.max(eCdN, dd); }
  }
  return { p, eCl, eCd, eClN, eCdN, aMin };
}

const round = (v) => +v.toPrecision(4);
const rec = (name, camber, camberPos, sets) => {
  const entries = sets.map((s) => {
    const { p, eCl, eCd, eClN, eCdN, aMin } = fit(s.rows);
    console.log(`${name} Re ${Math.round(s.re)}: a ${p.a.toFixed(2)} aL0 ${(p.aL0 / DEG).toFixed(2)} stall ${(p.aStall / DEG).toFixed(1)} / ${(p.aStallNeg / DEG).toFixed(2)} (w ${(p.wNeg / DEG).toFixed(2)}, sweep from ${(aMin / DEG).toFixed(2)}) cdMin ${p.cdMin.toFixed(4)} | worst in -3..10 deg: dCl ${eCl.toFixed(3)} dCd ${(eCd * 100).toFixed(0)}% | below -3 deg: dCl ${eClN.toFixed(3)} dCd ${(eCdN * 100).toFixed(0)}%`);
    const o = {};
    for (const k of Object.keys(p)) o[k] = round(p[k]);
    return { re: Math.round(s.re / 1000) * 1000, ...o, worst_dCl: round(eCl), worst_dCd_frac: round(eCd), worst_dCl_below_m3deg: round(eClN), worst_dCd_frac_below_m3deg: round(eCdN) };
  });
  return { name, camber, camberPos, fits: entries };
};

const e = JSON.parse(readFileSync(fE, 'utf8')).uiuc_lsat_vol5;
const a = JSON.parse(readFileSync(fA, 'utf8')).polars.AG24;
const pick = (re) => a.filter((s) => Math.abs(s.re_measured / re - 1) < 0.1)[0];
const data = {
  e387: rec('E387', 0.038, 0.4, [e.re_60000, e.re_100000, e.re_200000].map((s) => ({ re: s.re_measured, rows: s.rows }))),
  ag24: rec('AG24', 0.0222, 0.4, [pick(60000), pick(100000), pick(200000)].map((s) => ({ re: s.re_measured, rows: s.rows }))),
};
writeFileSync(join(ROOT, 'src/physics/polar-fit.js'), `// GENERATED by src/design/fit-polars.mjs: parameters of our own low-order section model, fitted to the local
// UIUC Low-Speed Airfoil Data vol. 5 tables. No measured table is held here; those stay local and gitignored.
export const POLAR_FITS = ${JSON.stringify(data, null, 1)};
`);
