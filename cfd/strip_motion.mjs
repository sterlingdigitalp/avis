#!/usr/bin/env node
// Section motion for one station taken from the strip model's own cruise run, with a pitch offset so that the 2D
// section's quasi-steady 3/4-chord angle equals the strip model's effective angle before its Wagner lag (alphaE).
//   node cfd/strip_motion.mjs <design.json> <s/L> [cycles=8] [settle_s=6] [bins=240] > motion.json
// Read-only use of src/physics. Every strip.evaluate call of the setupFlight cruise (as gates/suction-sensitivity.mjs)
// is observed: its body velocity, rates and right-wing joint state give the station's kinematics through
// sectionStates exactly as strip.js forms them; its outputs give alphaGeo and, by running the Jones lag recursion
// backwards, alphaE per strip, so the induced angle alphaI = alphaGeo - alphaE is known per strip and interpolated in s.
import { readFileSync } from 'node:fs';
import { setupFlight } from '../src/physics/scenarios.js';
import { sectionStates, makeSectionState, chordAt, xleAt, twistWeightAt, baseTwistAt } from '../src/physics/kinematics.js';

const A1 = 0.165, B1 = 0.0455, A2 = 0.335, B2 = 0.3; // as in strip.js (Jones)
const design = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const frac = Number(process.argv[3]);
const cycles = Number(process.argv[4] ?? 8);
const settle = Number(process.argv[5] ?? 6);
const bins = Number(process.argv[6] ?? 240);
const { sim, tick } = setupFlight(design, { init: { y: 0 } });
const strip = sim.bird.strip;
const st = sim.bird.st;
const wing = sim.bird.wing;
const n = st.n;
const s = frac * wing.L;
const seg = wing.segments.findIndex((g) => s >= g.s0 && s <= g.s0 + g.length);
const one = { n: 1, s: [s], ds: [0], chord: [chordAt(wing, s)], xle: [xleAt(wing, s)], weight: [twistWeightAt(wing, s)],
  base: [baseTwistAt(wing, s)], seg: [seg] };
const c = one.chord[0];
const dLE = one.xle[0] - wing.spar_x;
const d34 = dLE - 0.75 * c;
const W = makeSectionState(1);
// interpolation weights in s between the strips bracketing the station
let k0 = 0;
while (k0 < n - 2 && st.s[k0 + 1] < s) k0++;
const fw = Math.min(1, Math.max(0, (s - st.s[k0]) / (st.s[k0 + 1] - st.s[k0])));

const x1 = new Float64Array(2 * n), x2 = new Float64Array(2 * n);
let primed = true;
const prime = strip.prime;
strip.prime = (...a) => { primed = true; return prime(...a); };
let pending = null;
const evaluate = strip.evaluate;
strip.evaluate = (...a) => {
  const r = evaluate(...a);
  const [vb, om, wind, jR, , , , dt] = a;
  if (!(dt > 0)) return r;
  const lag = strip.calibration.lag ?? 1;
  const aI = [];
  for (let q = 0; q < 2 * n; q++) {
    const aLag = r.alphaEff[q];
    if (primed) { x1[q] = A1 * aLag; x2[q] = A2 * aLag; }
    const aE = (aLag - x1[q] - x2[q]) / (1 - A1 - A2);
    const rate = (2 * r.speed[q] * lag) / st.chord[q % n];
    const e1 = Math.exp(-B1 * rate * dt), e2 = Math.exp(-B2 * rate * dt);
    x1[q] = x1[q] * e1 + A1 * aE * (1 - e1);
    x2[q] = x2[q] * e2 + A2 * aE * (1 - e2);
    if (q < n) aI.push([r.alpha[q] - aE, r.alphaEff[q]]);
  }
  primed = false;
  // station kinematics, as strip.js forms them
  sectionStates(wing, one, jR, +1, W);
  const rel = (d) => {
    const px = W.A[0] + d * W.t[0], py = W.A[1] + d * W.t[1], pz = W.A[2] + d * W.t[2];
    const vx = wind[0] - vb[0] - (om[1] * pz - om[2] * py) - (W.Ad[0] + d * W.td[0]);
    const vy = wind[1] - vb[1] - (om[2] * px - om[0] * pz) - (W.Ad[1] + d * W.td[1]);
    const vz = wind[2] - vb[2] - (om[0] * py - om[1] * px) - (W.Ad[2] + d * W.td[2]);
    return [-(vx * W.t[0] + vy * W.t[1] + vz * W.t[2]), vx * W.b[0] + vy * W.b[1] + vz * W.b[2]];
  };
  const [u, w] = rel(0);
  const [u34, w34] = rel(d34);
  pending = { u, w, theta: W.theta[0], dtheta: W.dtheta[0], ageo: Math.atan2(w34, u34),
    aI: aI[k0][0] + fw * (aI[k0 + 1][0] - aI[k0][0]), alag: aI[k0][1] + fw * (aI[k0 + 1][1] - aI[k0][1]) };
  return r;
};
const HZ = Math.round(1 / sim.dt);
for (let k = 0; k < settle * HZ; k++) tick();
let wraps = 0, last = sim.snapshot().phase;
while (wraps < 1) { tick(); const p = sim.snapshot().phase; if (p < last) wraps++; last = p; }
const keys = ['u', 'w', 'theta', 'dtheta', 'ageo', 'aI', 'alag'];
const acc = Array.from({ length: bins }, () => ({ n: 0, ...Object.fromEntries(keys.map((k) => [k, 0])) }));
let lagLo = Infinity, lagHi = -Infinity;
const t0 = sim.t;
while (wraps < 1 + cycles) {
  pending = null;
  tick();
  const p = sim.snapshot().phase;
  if (pending) {
    const b = acc[Math.min(bins - 1, Math.floor((p / (2 * Math.PI)) * bins))];
    b.n++;
    for (const k of keys) b[k] += pending[k];
    lagLo = Math.min(lagLo, pending.alag);
    lagHi = Math.max(lagHi, pending.alag);
  }
  if (p < last) wraps++;
  last = p;
}
const f = cycles / (sim.t - t0);
const T = 1 / f;
const m = acc.map((b) => Object.fromEntries(keys.map((k) => [k, b[k] / b.n])));
const aE = m.map((x) => x.ageo - x.aI);
// Pitch offset: rotate the section nose up by delta (relative wind rotated into the new frame), so that
// atan2(|q| sin(a_spar + delta) + (theta' rate) * (-d34), |q| cos(a_spar + delta)) = alphaE. Fixed point on delta.
const dt = T / bins;
// Linearised, the condition reads delta - tau * d(delta)/dt = residual with tau = d34 / |q|, a periodic linear ODE: solve
// it per Fourier harmonic, then iterate on the full nonlinear residual (Newton with that operator).
const qMean = m.reduce((a2, x) => a2 + Math.hypot(x.u, x.w), 0) / bins;
const tau = d34 / qMean;
const omega0 = (2 * Math.PI) / T;
const solveLinear = (r) => {
  const N = r.length;
  const out = new Array(N).fill(0);
  for (let k = 0; k < N; k++) {
    if (k > N / 2) continue;
    let re = 0, im = 0;
    for (let j = 0; j < N; j++) { const a = (-2 * Math.PI * k * j) / N; re += r[j] * Math.cos(a); im += r[j] * Math.sin(a); }
    // divide by (1 - i k omega tau)
    const dr = 1, di = -k * omega0 * tau;
    const den = dr * dr + di * di;
    const xr = (re * dr + im * di) / den, xi = (im * dr - re * di) / den;
    const wgt = k === 0 || (N % 2 === 0 && k === N / 2) ? 1 / N : 2 / N;
    for (let j = 0; j < N; j++) { const a = (2 * Math.PI * k * j) / N; out[j] += wgt * (xr * Math.cos(a) - xi * Math.sin(a)); }
  }
  return out;
};
let delta = new Array(bins).fill(0);
const a34 = (dl) => m.map((x, i) => {
  const q = Math.hypot(x.u, x.w);
  const a = Math.atan2(x.w, x.u) + dl[i];
  const rate = x.dtheta + (dl[(i + 1) % bins] - dl[(i - 1 + bins) % bins]) / (2 * dt);
  return Math.atan2(q * Math.sin(a) - rate * d34, q * Math.cos(a));
});
for (let it = 0; it < 12; it++) {
  const cur = a34(delta);
  const corr = solveLinear(aE.map((x, i) => x - cur[i]));
  delta = delta.map((d, i) => d + corr[i]);
}
const fin = a34(delta);
const rms = Math.sqrt(fin.reduce((acc2, x, i) => acc2 + (x - aE[i]) ** 2, 0) / bins);
const rmsKin = Math.sqrt(m.reduce((acc2, x, i) => acc2 + (Math.atan2(x.w - x.dtheta * d34, x.u) - x.ageo) ** 2, 0) / bins);
const samples = m.map((x, i) => {
  const q = Math.hypot(x.u, x.w);
  const a = Math.atan2(x.w, x.u) + delta[i];
  return { t: i * dt, phase: (2 * Math.PI * i) / bins, theta: x.theta + delta[i],
    dtheta: x.dtheta + (delta[(i + 1) % bins] - delta[(i - 1 + bins) % bins]) / (2 * dt),
    u: q * Math.cos(a), w: q * Math.sin(a), alpha_eff: a, alpha34_2d: fin[i], alpha_e_strip: aE[i],
    alpha_geo_strip: x.ageo, alpha_induced_strip: x.aI, alpha_lag_strip: x.alag, pitch_offset: delta[i] };
});
const speed = samples.reduce((acc2, p) => acc2 + Math.hypot(p.u, p.w), 0) / bins;
const atm = design.atmosphere;
const D = 180 / Math.PI;
const half = (arr) => ((Math.max(...arr) - Math.min(...arr)) / 2) * D;
console.log(JSON.stringify({
  design: design.name, version: design.version ?? null, log: 'setupFlight cruise (strip model run)', source: 'strip_motion.mjs',
  frac_semispan_L: frac, z_over_semispan: (wing.hinge[2] + s) / (wing.hinge[2] + wing.L), s_m: s, segment: seg, chord_m: c,
  pitch_axis_x_over_c: dLE / c, twist_weight: one.weight[0], frequency_hz: f, period_s: T, amplitude_rad: null,
  mean_inplane_speed_m_s: speed, re_chord: (atm.rho_kg_m3 * speed * c) / atm.mu_Pa_s, k: (Math.PI * f * c) / speed,
  strip_alpha_lag_amplitude_deg: ((lagHi - lagLo) / 2) * D, alpha_e_amplitude_deg: half(aE),
  alpha34_2d_amplitude_deg: half(fin), rms_mismatch_deg: rms * D, rms_kinematics_2d_vs_strip_geo_deg: rmsKin * D,
  pitch_offset_range_deg: [Math.min(...delta) * D, Math.max(...delta) * D], samples }));
