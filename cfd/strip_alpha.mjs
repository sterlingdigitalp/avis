#!/usr/bin/env node
// The strip model's own effective angles over whole cruise cycles, per right-wing strip, phase-averaged.
//   node cfd/strip_alpha.mjs <design.json> [cycles=8] [settle_s=6] [bins=240] > strip-alpha.json
// Read-only use of src/physics: the same setupFlight run as gates/suction-sensitivity.mjs cruiseEffectiveAngles, with
// strip.evaluate wrapped (not edited). Outputs per strip and phase bin:
//   alpha_geo  strip.js alphaGeo, the 3/4-chord kinematic angle
//   alpha_e    alphaGeo minus the induced angle, before the Wagner lag (reconstructed exactly by running strip.js's
//              Jones two-state recursion backwards from its lagged output: alphaE = (alphaLag - x1 - x2)/(1 - A1 - A2))
//   alpha_lag  strip.js alphaEff, the Wagner-lagged angle that cruiseEffectiveAngles reports
import { readFileSync } from 'node:fs';
import { setupFlight } from '../src/physics/scenarios.js';

const A1 = 0.165, B1 = 0.0455, A2 = 0.335, B2 = 0.3; // as in strip.js (Jones)
const design = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const cycles = Number(process.argv[3] ?? 8);
const settle = Number(process.argv[4] ?? 6);
const bins = Number(process.argv[5] ?? 240);
const { sim, tick } = setupFlight(design, { init: { y: 0 } });
const strip = sim.bird.strip;
const st = sim.bird.st;
const n = st.n;
const x1 = new Float64Array(2 * n), x2 = new Float64Array(2 * n);
let primed = true; // the run starts primed (strip.prime before the first advancing step)
const prime = strip.prime;
strip.prime = (...a) => { primed = true; return prime(...a); };
let record = null;
const evaluate = strip.evaluate;
strip.evaluate = (...a) => {
  const r = evaluate(...a);
  const dt = a[7];
  if (dt > 0) {
    const lag = strip.calibration.lag ?? 1;
    const vals = [];
    for (let q = 0; q < 2 * n; q++) {
      const aLag = r.alphaEff[q];
      if (primed) { x1[q] = A1 * aLag; x2[q] = A2 * aLag; }
      const aE = (aLag - x1[q] - x2[q]) / (1 - A1 - A2);
      const rate = (2 * r.speed[q] * lag) / st.chord[q % n];
      const e1 = Math.exp(-B1 * rate * dt), e2 = Math.exp(-B2 * rate * dt);
      x1[q] = x1[q] * e1 + A1 * aE * (1 - e1);
      x2[q] = x2[q] * e2 + A2 * aE * (1 - e2);
      vals.push([r.alpha[q], aE, aLag, r.speed[q]]);
    }
    primed = false;
    if (record) record.push(vals);
  }
  return r;
};
const HZ = Math.round(1 / sim.dt);
for (let k = 0; k < settle * HZ; k++) tick();
let wraps = 0, last = sim.snapshot().phase;
while (wraps < 1) { tick(); const p = sim.snapshot().phase; if (p < last) wraps++; last = p; }
const sums = Array.from({ length: n }, () => Array.from({ length: bins }, () => [0, 0, 0, 0, 0]));
const lo = new Float64Array(n).fill(Infinity), hi = new Float64Array(n).fill(-Infinity);
const t0 = sim.t;
while (wraps < 1 + cycles) {
  record = [];
  tick();
  const p = sim.snapshot().phase;
  const b = Math.min(bins - 1, Math.floor((p / (2 * Math.PI)) * bins));
  for (const vals of record) {
    for (let i = 0; i < n; i++) {
      const v = vals[i];
      const s = sums[i][b];
      s[0] += v[0]; s[1] += v[1]; s[2] += v[2]; s[3] += v[3]; s[4] += 1;
      if (v[2] < lo[i]) lo[i] = v[2];
      if (v[2] > hi[i]) hi[i] = v[2];
    }
  }
  if (p < last) wraps++;
  last = p;
}
const f = cycles / (sim.t - t0);
const L = sim.bird.wing.L;
const strips = [];
for (let i = 0; i < n; i++) {
  const m = (k) => sums[i].map((s) => (s[4] ? s[k] / s[4] : null));
  strips.push({ station: i, s_m: st.s[i], frac_semispan: st.s[i] / L, chord_m: st.chord[i],
    alpha_lag_amplitude_deg: ((hi[i] - lo[i]) / 2) * 180 / Math.PI,
    alpha_geo: m(0), alpha_e: m(1), alpha_lag: m(2), speed: m(3) });
}
console.log(JSON.stringify({ design: design.name, version: design.version ?? null, flap_hz: f, cycles, settle_s: settle, bins,
  wing_length_m: L, hinge_z_m: sim.bird.wing.hinge[2], strips }));
