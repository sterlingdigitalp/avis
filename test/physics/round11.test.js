// Round 11: the section model's negative stall against the measured E387 and AG24 tables, and a bushing judged on
// the length its housing supports.
import test from 'node:test';
import assert from 'node:assert/strict';
import { POLAR_FITS } from '../../src/physics/polar-fit.js';
import { sectionModel } from '../../src/physics/polar-model.js';
import { POLARS, polarEval } from '../../src/physics/polars.js';
import { supportCheck, supportedLength } from '../../src/physics/bearings.js';

const DEG = Math.PI / 180;
const out = [0, 0, 0, 0];
const fitAt = (name, re) => POLAR_FITS[name].fits.find((f) => f.re === re);

/** Angle (deg) where the model's attached fraction falls through 0.5 below zero incidence, by bisection. */
function negativeStall(p) {
  let lo = -40 * DEG, hi = -1 * DEG; // attached fraction < 0.5 at lo, > 0.5 at hi
  for (let k = 0; k < 60; k++) {
    const m = 0.5 * (lo + hi);
    if (sectionModel(p, m, out)[3] < 0.5) lo = m; else hi = m;
  }
  return (0.5 * (lo + hi)) / DEG;
}

// Reference values read from the local, gitignored UIUC table (data/references/airfoil-e387.json): Selig, Guglielmo,
// Broeren and Giguere, Summary of Low-Speed Airfoil Data vol. 5 (UIUC LSAT, 1995), E387 ascending sweeps. The lowest
// measured angle of each sweep, and the row above it: [alpha_deg, cl, cd].
const E387 = {
  100000: { low: [-6.27, -0.359, 0.0756], next: [-5.24, -0.299, 0.0577] },
  200000: { low: [-6.28, -0.385, 0.0771], next: [-5.23, -0.184, 0.0396] },
  60000: { low: [-5.11, -0.238, 0.0489] },
};
// Same volume, AG24 (data/references/airfoils/uiuc-vol5-ag-polars.json), lowest measured angle of each sweep.
const AG24_LOW = { 60000: -5.9, 100000: -5.88, 200000: -5.84 };

test('E387 negative stall sits at the measured drag rise near -6.3 deg (UIUC vol. 5), not tens of degrees beyond it', () => {
  for (const re of [100000, 200000]) {
    const p = fitAt('e387', re);
    const [aLow, clLow, cdLow] = E387[re].low;
    const aNext = E387[re].next[0];
    const st = negativeStall(p);
    // The tables show the drag doubling between the two lowest rows: the stall lies there or at most half a degree
    // past the sweep, where the data cannot speak.
    assert.ok(st <= aNext && st >= aLow - 0.5, `Re ${re}: negative stall ${st.toFixed(2)} deg outside [${aLow - 0.5}, ${aNext}]`);
    sectionModel(p, aLow * DEG, out);
    assert.ok(Math.abs(out[1] / cdLow - 1) < 0.15, `Re ${re}: cd ${out[1].toFixed(4)} at ${aLow} deg vs ${cdLow}`);
    assert.ok(Math.abs(out[0] - clLow) < 0.15, `Re ${re}: cl ${out[0].toFixed(3)} at ${aLow} deg vs ${clLow}`);
    // The row above the drag rise is reproduced too, and the section is mostly attached a degree above it and
    // separated two degrees below the sweep.
    const [, , cdNext] = E387[re].next;
    sectionModel(p, aNext * DEG, out);
    assert.ok(Math.abs(out[1] / cdNext - 1) < 0.15, `Re ${re}: cd ${out[1].toFixed(4)} at ${aNext} deg vs ${cdNext}`);
    assert.ok(sectionModel(p, (aNext + 1) * DEG, out)[3] > 0.85, `Re ${re}: attached at ${aNext + 1} deg`);
    assert.ok(sectionModel(p, (aLow - 2) * DEG, out)[3] < 0.15, `Re ${re}: mostly separated at ${aLow - 2} deg`);
  }
  // Re 6e4: the sweep stops at -5.11 deg; the stall may not be placed more than a degree below it.
  assert.ok(negativeStall(fitAt('e387', 60000)) >= E387[60000].low[0] - 1);
});

test('AG24 negative stall stays within a degree of its measured sweep (UIUC vol. 5)', () => {
  for (const [re, aLow] of Object.entries(AG24_LOW)) assert.ok(negativeStall(fitAt('ag24', +re)) >= aLow - 1, `AG24 Re ${re}`);
});

test('flight polar e387-film at Re 1.5e5: stalled on the upstroke past -8 deg, attached at -4 deg', () => {
  const o = new Float64Array(5);
  polarEval(POLARS['e387-film'], -8 * DEG, 1.5e5, 1, 1, o);
  assert.ok(o[3] < 0.1, `attached fraction ${o[3]} at -8 deg`);
  polarEval(POLARS['e387-film'], -4 * DEG, 1.5e5, 1, 1, o);
  assert.ok(o[3] > 0.9, `attached fraction ${o[3]} at -4 deg`);
});

test('a bushing is judged on its supported length: igus GFM-0810-06 in a 1.2 mm plate', () => {
  // Hand calculation: Fr 300 N on d1 8 mm. Full length b1 6 mm: p = 300 / (8 x 6) = 6.25 MPa, FoS 80 / 6.25 = 12.8.
  // In a 1.2 mm plate: p = 300 / (8 x 1.2) = 31.25 MPa, FoS 80 / 31.25 = 2.56.
  const part = { category: 'bushing', specs: { d1_mm: 8, d2_mm: 10, b1_mm: 6, static_surface_pressure_mpa: 80 } };
  const full = supportCheck(part, { Fr: 300 });
  assert.ok(Math.abs(full.p_MPa - 6.25) < 1e-12 && Math.abs(full.fos - 12.8) < 1e-12);
  const plate = supportCheck(part, { Fr: 300 }, { housing_mm: 1.2 });
  assert.ok(Math.abs(plate.p_MPa - 31.25) < 1e-12 && Math.abs(plate.fos - 2.56) < 1e-12);
  assert.equal(plate.supported_mm, 1.2);
  assert.equal(plate.b1_mm, 6);
  // A boss longer than the bushing supports only b1; no housing length leaves b1.
  assert.equal(supportedLength(6, 9), 6);
  assert.equal(supportedLength(6, null), 6);
  assert.equal(supportedLength(6, 0), 6);
  assert.equal(supportCheck(part, { Fr: 300 }, { housing_mm: 6 }).fos, full.fos);
});

test('section model: attached fraction stays within 0..1 at every angle, and each stall is a transition through 0.5', () => {
  for (const name of Object.keys(POLAR_FITS)) for (const p of POLAR_FITS[name].fits) {
    for (let a = -40; a <= 40; a += 0.25) {
      const s = sectionModel(p, a * DEG, out)[3];
      assert.ok(s >= 0 && s <= 1, `${name} Re ${p.re}: attached fraction ${s} at ${a} deg`);
    }
    assert.ok(Math.abs(sectionModel(p, p.aStall, out)[3] - 0.5) < 0.02, `${name} Re ${p.re}: half attached at the positive stall`);
    assert.ok(sectionModel(p, p.aStall + 6 * DEG, out)[3] < 0.1, `${name} Re ${p.re}: separated 6 deg past the positive stall`);
  }
});

test('bushing check at small sizes and without dimensions (hand calculation)', () => {
  // d1 0.8 mm, b1 0.8 mm, 10 N: p = 10 / 0.64 = 15.625 MPa, FoS 80 / 15.625 = 5.12; half supported: 31.25 MPa, 2.56.
  const part = { category: 'bushing', specs: { d1_mm: 0.8, d2_mm: 1.2, b1_mm: 0.8, static_surface_pressure_mpa: 80 } };
  assert.ok(Math.abs(supportCheck(part, { Fr: 10 }).fos - 5.12) < 1e-12);
  assert.ok(Math.abs(supportCheck(part, { Fr: 10 }, { housing_mm: 0.4 }).fos - 2.56) < 1e-12);
  assert.equal(supportedLength(6, 0.5), 0.5);
  for (const specs of [{ d1_mm: 0, b1_mm: 6, static_surface_pressure_mpa: 80 }, { d1_mm: 8, b1_mm: 0, static_surface_pressure_mpa: 80 }, { d1_mm: 8, b1_mm: 6 }]) {
    const r = supportCheck({ category: 'bushing', specs }, { Fr: 10 });
    assert.equal(r.fos, null);
    assert.equal(r.ok, false);
  }
});
