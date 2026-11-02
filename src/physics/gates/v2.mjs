#!/usr/bin/env node
// V2: UVLM steady lift slope vs lifting-line theory, AR 4-12, within 3%.
// Ruling after round 1 (SPEC): judged against Helmbold's low-aspect-ratio formula a = 2 pi AR / (2 + sqrt(AR^2 + 4))
// at AR 4 and 6, and against Prandtl's lifting line at AR 8-12. Both closed forms are for the elliptic planform, so
// the gate's UVLM wing is a flat elliptic plate; the round-1 rectangular comparison is kept beside it for the record.
// Grid: chosen before results as the finest of a study (nc x ns = 12 x 48); the study is in the evidence.
//   node src/physics/gates/v2.mjs [--quick]
import { plateSurface, ellipticPlateSurface } from './plate.js';
import { createUvlm } from '../uvlm.js';
import { liftingLineSlope, ellipticLiftSlope, weissingerSlope, LIFTING_SURFACE_EXACT } from './references.js';
import { writeGate, writeJson } from './common.js';

const LIMIT_PCT = 3;
const chord = 1;
const U = 10;
const rho = 1;
const alpha = 2 * (Math.PI / 180);

export function steadySlope({ AR, nc = 6, ns = 24, chordsOfWake = 60, stepsPerChord = 2 }) {
  const { surface } = plateSurface({ chord, AR, nc, ns });
  const dt = chord / (stepsPerChord * U);
  const steps = Math.round(chordsOfWake * stepsPerChord);
  const sim = createUvlm([surface], {
    Vinf: [-U * Math.cos(alpha), U * Math.sin(alpha), 0],
    rho, dt, symmetric: true, maxWakeRows: steps + 2,
  });
  let f = null;
  const S = (AR * chord * chord) / 2;
  const hist = [];
  for (let n = 0; n < steps; n++) {
    f = sim.advance(n >= steps - 5);
    if (f) {
      // Lift is normal to the free stream.
      const L = f.force[1] * Math.cos(alpha) + f.force[0] * Math.sin(alpha);
      hist.push(L / (0.5 * rho * U * U * S * Math.sin(alpha)));
    }
  }
  return { CLa: hist[hist.length - 1], drift: Math.abs(hist[hist.length - 1] - hist[0]) / hist[0] };
}

/** Steady lift slope of a flat elliptic plate by the UVLM, per rad. */
export function ellipticSlope({ AR, nc = 12, ns = 48, chordsOfWake = 60, stepsPerChord = 2 }) {
  const { surface, area } = ellipticPlateSurface({ c0: chord, AR, nc, ns });
  const dt = chord / (stepsPerChord * U);
  const steps = Math.round(chordsOfWake * stepsPerChord);
  const sim = createUvlm([surface], { Vinf: [-U * Math.cos(alpha), U * Math.sin(alpha), 0], rho, dt, symmetric: true, maxWakeRows: steps + 2 });
  const hist = [];
  for (let n = 0; n < steps; n++) {
    const f = sim.advance(n >= steps - 5);
    if (f) hist.push((f.force[1] * Math.cos(alpha) + f.force[0] * Math.sin(alpha)) / (0.5 * rho * U * U * (area / 2) * Math.sin(alpha)));
  }
  return { CLa: hist[hist.length - 1], drift: Math.abs(hist[hist.length - 1] - hist[0]) / hist[0] };
}

export const helmboldSlope = (AR) => (2 * Math.PI * AR) / (2 + Math.sqrt(AR * AR + 4));
export const prandtlEllipticSlope = (AR) => (2 * Math.PI * AR) / (AR + 2);

if (import.meta.url === `file://${process.argv[1]}`) {
  const quick = process.argv.includes('--quick');
  const G = quick ? { nc: 6, ns: 24 } : { nc: 12, ns: 48 };
  const rows = [];
  for (const AR of [4, 5, 6, 8, 10, 12]) {
    const t0 = Date.now();
    const uv = ellipticSlope({ AR, ...G });
    const helm = helmboldSlope(AR), pr = prandtlEllipticSlope(AR);
    const ref = AR <= 6 ? 'helmbold' : 'prandtl';
    const refV = ref === 'helmbold' ? helm : pr;
    const judged = AR !== 5; // the ruling names AR 4 and 6, and 8-12; AR 5 is shown for information
    rows.push({ AR, reference: ref, judged, uvlm_per_rad: uv.CLa, helmbold_per_rad: helm, prandtl_per_rad: pr, error_pct: (uv.CLa / refV - 1) * 100, error_vs_helmbold_pct: (uv.CLa / helm - 1) * 100, error_vs_prandtl_pct: (uv.CLa / pr - 1) * 100, settle_drift_frac: uv.drift, seconds: (Date.now() - t0) / 1000 });
    console.log(`elliptic AR ${AR}: UVLM ${uv.CLa.toFixed(4)} vs ${ref} ${refV.toFixed(4)} (${rows.at(-1).error_pct.toFixed(2)}%)`);
  }
  // Grid study at the two ends of the range.
  const grids = quick ? [[4, 6], [6, 24]] : [[6, 24], [8, 32], [12, 48], [16, 64]];
  const gridStudy = [];
  for (const AR of [4, 12]) for (const [nc, ns] of grids) {
    const r = AR === 4 || AR === 12 ? rows.find((q) => q.AR === AR && nc === G.nc && ns === G.ns) : null;
    const CLa = r ? r.uvlm_per_rad : ellipticSlope({ AR, nc, ns }).CLa;
    gridStudy.push({ AR, nc, ns, CLa });
    console.log(`grid AR ${AR} ${nc}x${ns}: ${CLa.toFixed(4)}`);
  }
  // Round-1 rectangular comparison, for the record (not the gate).
  const rect = [];
  for (const AR of [4, 6, 8, 10, 12]) {
    const ll = liftingLineSlope({ AR, chordFn: () => chord, b: AR * chord });
    const uv = steadySlope({ AR });
    rect.push({ AR, uvlm_per_rad: uv.CLa, prandtl_rect_per_rad: ll.CLa, weissinger_per_rad: weissingerSlope({ AR }), helmbold_elliptic_per_rad: helmboldSlope(AR), error_vs_prandtl_rect_pct: (uv.CLa / ll.CLa - 1) * 100 });
  }
  writeJson('out/verification/evidence/V2_table.json', { rows, grid: G, gridStudy, rectangular_round1_method: rect, note: 'Gate: flat elliptic plate, straight quarter-chord line, 2 deg incidence, 60 chords of wake. Helmbold at AR 4 and 6, Prandtl elliptic a0 AR/(AR+2) at AR 8-12, a0 = 2 pi. Rectangular rows repeat the round-1 method and are not judged.' });
  const judged = rows.filter((r) => r.judged);
  const maxErr = Math.max(...judged.map((r) => Math.abs(r.error_pct)));
  const worst = judged.reduce((p, q) => (Math.abs(q.error_pct) > Math.abs(p.error_pct) ? q : p));
  writeGate({
    gate: 'V2',
    passed: maxErr <= LIMIT_PCT && !quick,
    measured: {
      reference: 'Helmbold at AR 4 and 6, Prandtl lifting line at AR 8-12 (SPEC ruling after round 1), elliptic planform',
      grid: G,
      max_error_pct: +maxErr.toFixed(3),
      worst_AR: worst.AR,
      table: rows.map((r) => [r.AR, r.reference, r.judged, +r.uvlm_per_rad.toFixed(4), +(r.reference === 'helmbold' ? r.helmbold_per_rad : r.prandtl_per_rad).toFixed(4), +r.error_pct.toFixed(2)]),
      table_columns: ['AR', 'reference', 'judged', 'UVLM per rad', 'reference per rad', 'error %'],
      grid_study: gridStudy.map((r) => [r.AR, `${r.nc}x${r.ns}`, +r.CLa.toFixed(4)]),
      round1_prandtl_rectangular_failure: 'stays on record (9.38% at AR 4); rectangular rows repeated in the evidence',
    },
    limit: { max_error_pct: LIMIT_PCT, AR_range: [4, 12] },
    evidence: 'out/verification/evidence/V2_table.json',
  });
  console.log(`V2 max error ${maxErr.toFixed(2)}% at AR ${worst.AR}`, maxErr <= LIMIT_PCT ? 'PASS' : 'FAIL');
}
