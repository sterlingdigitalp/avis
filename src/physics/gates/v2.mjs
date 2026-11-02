#!/usr/bin/env node
// V2: UVLM steady lift slope vs Prandtl lifting-line theory, AR 4-12, within 3%.
// Rectangular flat wing at small incidence, marched until the lift has settled.
import { plateSurface } from './plate.js';
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

if (import.meta.url === `file://${process.argv[1]}`) {
  // The gate's reference is classical (Prandtl) lifting-line theory, as declared before results.
  // --reference weissinger exists only so the orchestrator can run a ruling; it is never the default.
  const useW = process.argv.includes('--reference') && process.argv[process.argv.indexOf('--reference') + 1] === 'weissinger';
  const rows = [];
  for (const AR of [4, 5, 6, 8, 10, 12]) {
    const ll = liftingLineSlope({ AR, chordFn: () => chord, b: AR * chord });
    const w = weissingerSlope({ AR });
    const uv = steadySlope({ AR });
    const err = (uv.CLa / ll.CLa - 1) * 100;
    rows.push({ AR, uvlm_per_rad: uv.CLa, prandtl_ll_per_rad: ll.CLa, weissinger_per_rad: w, helmbold_per_rad: (2 * Math.PI * AR) / (2 + Math.sqrt(4 + AR * AR)), error_vs_prandtl_pct: err, error_vs_weissinger_pct: (uv.CLa / w - 1) * 100, settle_drift_frac: uv.drift });
    console.log(`AR ${AR} UVLM ${uv.CLa.toFixed(4)} Prandtl ${ll.CLa.toFixed(4)} err ${err.toFixed(2)}% Weissinger ${w.toFixed(4)} err ${(uv.CLa / w - 1) * 100}`);
  }
  // Is the solver or the reference responsible? Asymptote towards high AR, and published surface values.
  const asymptote = [20, 40].map((AR) => {
    const u = steadySlope({ AR, ns: 40, chordsOfWake: 140 }).CLa;
    const ll = liftingLineSlope({ AR, chordFn: () => chord, b: AR * chord }).CLa;
    return { AR, uvlm_per_rad: u, prandtl_ll_per_rad: ll, error_pct: (u / ll - 1) * 100 };
  });
  const surface = LIFTING_SURFACE_EXACT.map((r) => {
    const fine = steadySlope({ AR: r.AR, nc: 12, ns: 48 }).CLa;
    return { ...r, uvlm_12x48_per_rad: fine, error_pct: (fine / r.CLa - 1) * 100 };
  });
  const grid = [[4, 12], [6, 24], [8, 24], [8, 48]].map(([nc, ns]) => ({ AR: 4, nc, ns, CLa: steadySlope({ AR: 4, nc, ns }).CLa }));
  writeJson('out/verification/evidence/V2_table.json', { rows, asymptote, surface, grid, note: 'Rectangular flat wing. Reference is Prandtl by Glauert Fourier collocation with a0 = 2 pi. Weissinger and lifting-surface values are secondary checks on whether the solver or the reference is off.' });
  const key = useW ? 'error_vs_weissinger_pct' : 'error_vs_prandtl_pct';
  const maxErr = Math.max(...rows.map((r) => Math.abs(r[key])));
  const worst = rows.reduce((p, q) => (Math.abs(q[key]) > Math.abs(p[key]) ? q : p));
  writeGate({
    gate: 'V2',
    passed: maxErr <= LIMIT_PCT,
    measured: {
      reference: useW ? 'weissinger-extended-lifting-line' : 'prandtl-lifting-line',
      max_error_pct: +maxErr.toFixed(3),
      worst_AR: worst.AR,
      table: rows.map((r) => [r.AR, +r.uvlm_per_rad.toFixed(4), +r.prandtl_ll_per_rad.toFixed(4), +r.error_vs_prandtl_pct.toFixed(2), +r.error_vs_weissinger_pct.toFixed(2)]),
      table_columns: ['AR', 'UVLM', 'Prandtl LL', 'err vs Prandtl %', 'err vs Weissinger %'],
      asymptote: asymptote.map((r) => [r.AR, +r.error_pct.toFixed(2)]),
      surface_check: surface.map((r) => [r.AR, +r.uvlm_12x48_per_rad.toFixed(4), r.CLa]),
    },
    limit: { max_error_pct: LIMIT_PCT, AR_range: [4, 12] },
    evidence: 'out/verification/evidence/V2_table.json',
  });
  console.log(`V2 reference ${useW ? 'Weissinger' : 'Prandtl'} max error ${maxErr.toFixed(2)}%`, maxErr <= LIMIT_PCT ? 'PASS' : 'FAIL');
}
