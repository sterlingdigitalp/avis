#!/usr/bin/env node
// V1: UVLM vs Theodorsen, plunging high-aspect-ratio flat plate, k = 0.1-0.5: lift amplitude within 5%,
//     phase within 5 deg.
// V3: UVLM thrust vs Garrick, small-amplitude plunge: trend correct and within 15%.
// Both run the same simulation. Compared at mid-span of AR 20, where the flow is closest to the 2D
// section the theories describe; the whole-wing values are reported alongside.
import { runPlunge } from './plate.js';
import { theodorsenPlungeLift, garrickPlungeCt } from './references.js';
import { writeGate, writeJson } from './common.js';

const U = 10, chord = 1, rho = 1, AR = 20;
const KS = [0.1, 0.2, 0.3, 0.4, 0.5];
const h0 = 0.02 * chord; // small amplitude: h0/c = 0.02

function compare({ k, nc, ns, per }) {
  const r = runPlunge({ U, chord, k, h0, AR, nc, ns, rho, periods: 2.5, wakePeriods: 2, stepsPerChord: per });
  const th = theodorsenPlungeLift({ rho, U, b: chord / 2, h0, omega: r.omega });
  const phase = Math.atan2(-r.L.b, r.L.a);
  const dphi = ((((phase - th.phase) * 180) / Math.PI + 540) % 360) - 180;
  const qc = 0.5 * rho * U * U * chord;
  const ct = r.thrustMid / qc;
  const cg = garrickPlungeCt({ k, h0, c: chord });
  return { k, nc, ns, steps_per_chord: per, lift_amp_uvlm_N_per_m: r.L.amp, lift_amp_theodorsen_N_per_m: th.amp, amp_error_pct: (r.L.amp / th.amp - 1) * 100, phase_error_deg: dphi, whole_wing_amp_error_pct: (r.LT.amp / th.amp - 1) * 100, ct_uvlm: ct, ct_garrick: cg, ct_error_pct: (ct / cg - 1) * 100, whole_wing_ct_error_pct: (r.thrustTotal / qc / cg - 1) * 100, steps: r.steps };
}

const rows = KS.map((k) => compare({ k, nc: 8, ns: 8, per: 8 }));
for (const r of rows) console.log(`k ${r.k} amp ${r.amp_error_pct.toFixed(2)}% phase ${r.phase_error_deg.toFixed(2)}deg ct ${r.ct_error_pct.toFixed(2)}%`);
const grid = [[4, 4], [8, 8], [16, 16]].map(([nc, per]) => compare({ k: 0.5, nc, ns: 6, per }));
writeJson('out/verification/evidence/V1V3_table.json', { rows, grid, AR, h0_over_c: h0 / chord });

const maxAmp = Math.max(...rows.map((r) => Math.abs(r.amp_error_pct)));
const maxPhase = Math.max(...rows.map((r) => Math.abs(r.phase_error_deg)));
writeGate({
  gate: 'V1',
  passed: maxAmp <= 5 && maxPhase <= 5,
  measured: { max_amp_error_pct: +maxAmp.toFixed(3), max_phase_error_deg: +maxPhase.toFixed(3), k: KS, amp_error_pct: rows.map((r) => +r.amp_error_pct.toFixed(2)), phase_error_deg: rows.map((r) => +r.phase_error_deg.toFixed(2)), grid_at_k05: grid.map((r) => [r.nc, +r.amp_error_pct.toFixed(2), +r.phase_error_deg.toFixed(2)]) },
  limit: { amp_error_pct: 5, phase_error_deg: 5, k_range: [0.1, 0.5] },
  evidence: 'out/verification/evidence/V1V3_table.json',
});

const maxCt = Math.max(...rows.map((r) => Math.abs(r.ct_error_pct)));
const ratioU = rows.map((r) => r.ct_uvlm);
const monotonic = ratioU.every((v, i) => i === 0 || v > ratioU[i - 1]);
writeGate({
  gate: 'V3',
  passed: maxCt <= 15 && monotonic,
  measured: { max_ct_error_pct: +maxCt.toFixed(3), trend_monotonic: monotonic, k: KS, ct_uvlm: rows.map((r) => +r.ct_uvlm.toExponential(4)), ct_garrick: rows.map((r) => +r.ct_garrick.toExponential(4)), ct_error_pct: rows.map((r) => +r.ct_error_pct.toFixed(2)) },
  limit: { ct_error_pct: 15, trend: 'thrust rises with k as Garrick predicts' },
  evidence: 'out/verification/evidence/V1V3_table.json',
});
console.log(`V1 max amp ${maxAmp.toFixed(2)}% phase ${maxPhase.toFixed(2)}deg; V3 max ct ${maxCt.toFixed(2)}% monotonic ${monotonic}`);
