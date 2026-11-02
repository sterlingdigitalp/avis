#!/usr/bin/env node
// One wing section's in-plane motion over one cruise flap cycle, from the physics code (SPEC: one source).
//   node cfd/section_motion.mjs <design.json> <flight-log.json> <z/semispan> [samples]
// Uses makeWing, makeFlapDrive, strokeAngle and sectionStates from src/physics/kinematics.js, the cruise phase and
// commanded twist (twist_r) from the flight log, and the body-frame velocity from the log's vel and quat.
// Output per sample: relative wind at the spar axis in the section frame, u along the chord from LE to TE and w along
// the section up-normal; theta, the section's geometric incidence; alpha_eff = atan2(w, u).
import { readFileSync } from 'node:fs';
import { makeWing, makeFlapDrive, strokeAngle, sectionStates, makeSectionState, chordAt, xleAt, twistWeightAt,
  baseTwistAt } from '../src/physics/kinematics.js';

const design = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const log = JSON.parse(readFileSync(process.argv[3], 'utf8'));
const zFrac = Number(process.argv[4]);
const N = Number(process.argv[5] ?? 240);
const wing = makeWing(design.wing);
const law = design.drive.kind === 'crank-rocker' ? makeFlapDrive(design.drive) : { kind: 'sine', phiMean: design.drive.phiMean ?? 0 };
const semispan = wing.hinge[2] + wing.L;
const s = zFrac * semispan - wing.hinge[2];
const seg = wing.segments.findIndex((g) => s >= g.s0 && s <= g.s0 + g.length);
const st = { n: 1, s: [s], ds: [0], chord: [chordAt(wing, s)], xle: [xleAt(wing, s)], weight: [twistWeightAt(wing, s)],
  base: [baseTwistAt(wing, s)], seg: [seg] };

// Pick one whole flap cycle from the middle half of the log (steady cruise): phase wraps 2pi -> 0 at cycle starts.
const c = log.columns;
const n = c.t.length;
const starts = [];
for (let i = Math.floor(n / 4); i < Math.floor((3 * n) / 4); i++) if (c.phase[i] < c.phase[i - 1]) starts.push(i);
if (starts.length < 2) throw new Error('no whole cycle in the middle of the log');
const i0 = starts[0];
const i1 = starts[1];
const T = c.t[i1] - c.t[i0];
const f = 1 / T;
const interp = (col, t) => {
  let k = i0;
  while (k < i1 && c.t[k + 1] < t) k++;
  const a = (t - c.t[k]) / (c.t[k + 1] - c.t[k]);
  return col[k] + a * (col[k + 1] - col[k]);
};
const unwrap = (t) => {
  // phase over the cycle, made monotonic from 0 to 2pi
  let k = i0;
  while (k < i1 && c.t[k + 1] < t) k++;
  const p0 = c.phase[k], p1 = c.phase[k + 1] < p0 ? c.phase[k + 1] + 2 * Math.PI : c.phase[k + 1];
  return p0 + ((t - c.t[k]) / (c.t[k + 1] - c.t[k])) * (p1 - p0);
};
const rotInv = (q, v) => {
  // body = q^-1 * world * q for a unit quaternion [x, y, z, w]
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  // v' = v - w t + q x t  (inverse rotation)
  return [v[0] - w * tx + (y * tz - z * ty), v[1] - w * ty + (z * tx - x * tz), v[2] - w * tz + (x * ty - y * tx)];
};
const amp = c.amplitude[i0];
const out = makeSectionState(1);
const samples = [];
const dt = T / N;
for (let m = 0; m < N; m++) {
  const t = c.t[i0] + m * dt;
  const phase = unwrap(t) % (2 * Math.PI);
  const dphase = (unwrap(Math.min(t + 1e-3, c.t[i1])) - unwrap(Math.max(t - 1e-3, c.t[i0]))) /
    (Math.min(t + 1e-3, c.t[i1]) - Math.max(t - 1e-3, c.t[i0]));
  const sa = strokeAngle(phase, amp, law);
  const th = interp(c.twist_r, t);
  const thp = interp(c.twist_r, Math.min(t + dt, c.t[i1]));
  const thm = interp(c.twist_r, Math.max(t - dt, c.t[i0]));
  const joint = { phi: sa.phi, dphi: sa.dphi * dphase, ddphi: 0, theta: th, dtheta: (thp - thm) / (2 * dt), ddtheta: 0 };
  sectionStates(wing, st, joint, +1, out);
  const q = [0, 1, 2, 3].map((k) => interp(c.quat.map((r) => r[k]), t));
  const qn = Math.hypot(...q);
  const vw = [0, 1, 2].map((k) => interp(c.vel.map((r) => r[k]), t));
  const vb = rotInv(q.map((x) => x / qn), vw);
  const W = [0, 1, 2].map((k) => -(vb[k] + out.Ad[k])); // air relative to the spar point, body frame
  const tv = [out.t[0], out.t[1], out.t[2]];
  const bv = [out.b[0], out.b[1], out.b[2]];
  const u = -(W[0] * tv[0] + W[1] * tv[1] + W[2] * tv[2]); // along the chord, LE to TE
  const w = W[0] * bv[0] + W[1] * bv[1] + W[2] * bv[2];
  const span = W[0] * out.e[0] + W[1] * out.e[1] + W[2] * out.e[2];
  samples.push({ t: m * dt, phase, phi: sa.phi, theta: out.theta[0], dtheta: out.dtheta[0], u, w, span,
    alpha_eff: Math.atan2(w, u), heave_velocity: out.Ad[1] * bv[1] + out.Ad[2] * bv[2] });
}
const speed = samples.reduce((a, p) => a + Math.hypot(p.u, p.w), 0) / N;
const atm = design.atmosphere;
console.log(JSON.stringify({
  design: design.name, version: design.version ?? null, log: log.scenario, z_over_semispan: zFrac, s_m: s,
  segment: seg, chord_m: st.chord[0], pitch_axis_x_over_c: (st.xle[0] - wing.spar_x) / st.chord[0],
  twist_weight: st.weight[0], frequency_hz: f, period_s: T, amplitude_rad: amp, cycle_t0_s: c.t[i0],
  mean_inplane_speed_m_s: speed, re_chord: atm.rho_kg_m3 * speed * st.chord[0] / atm.mu_Pa_s,
  k: Math.PI * f * st.chord[0] / speed, samples }));
