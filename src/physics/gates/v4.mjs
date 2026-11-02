#!/usr/bin/env node
// V4: the whole model on SmartBird (Send et al., ICAS 2012) as declared in SPEC: 18 W electrical in horizontal free
// flight at 4.7 m/s, band 12.6-23.4 W, with the paper's 2 m span, 0.48 kg, 0.5 m2 and 2 Hz. The 23 W design point
// at 5 m/s is reported beside it. What the paper does not publish is assumed and listed in the evidence.
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { buildDesign, gearRatioOf } from '../../design/build.js';
import { buildControl, refineTrimByFlight, trimLevel, cycleResidual } from '../../design/trim.js';
import { buildBird } from '../flight.js';
import { setupFlight } from '../scenarios.js';
import { writeGate, writeJson, ROOT } from './common.js';

const DEG = Math.PI / 180;
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
const bench = JSON.parse(readFileSync(join(ROOT, 'data/references/benchmarks.json'), 'utf8')).smartbird;
const span = bench.geometry_and_mass.span_m.values[0].value; // 2 m, primary paper
const mass = bench.geometry_and_mass.mass_kg.values[0].value; // 0.48 kg
const area = bench.geometry_and_mass.wing_area_m2.value; // 0.5 m2
const freq = bench.kinematics.flap_frequency_hz.values[0].value; // 2 Hz
const target = bench.v4_power_targets.measured_free_flight; // 18 W at 4.7 m/s
const design5 = bench.v4_power_targets.design_point; // 23 W at 5 m/s
const plunge = bench.kinematics.stroke_2d_section.plunge_amplitude_m; // 0.3 m, read from a figure
const pitch = bench.kinematics.torsion.pitch_amplitude_deg_typical; // 9 deg, read from a figure
const G = parseFloat(String(bench.kinematics.transmission.gear_reduction).split(':')[1]); // 45

const assumptions = [
  'The stroke amplitude is solved so the model flies level at 4.7 m/s at the published 2 Hz; at the published 0.3 m plunge the model needs less than 2 Hz (both in the evidence).',
  'Planform: two segments, elbow at half span, outer wing gain 0.5 (the paper gives a 1:3 hinge ratio, not the outer stroke).',
  'Stroke: amplitude set so the 3/4-span section plunges 0.3 m, the paper\'s typical 2D section value (read from a figure).',
  'Twist: 9 deg tip amplitude, sinusoidal (the paper describes a partially linear schedule).',
  'Section: our E387 fit with the film drag factor stands in for the NACA 7412 the paper names; no coordinates or polars of either SmartBird section are published.',
  'Structure: wing spars, ribs and film sized by our builder; body lumps scaled so the total is 0.48 kg.',
  'Drive: E-flite Park 370 1080 class motor, the paper\'s 1:45 reduction as a two-stage train ratio, 3S 1000 mAh; SmartBird\'s own motor and battery are not published.',
  'Air density 1.2 kg/m3 as the paper used.',
  'Stroke law: sinusoidal; SmartBird\'s crank and hinge linkage geometry is not published.',
];

// Gear train with a ratio close to 45 from the catalogue (only the ratio matters for V4).
const params = {
  candidate: 'larus', span_m: span, AR: (span * span) / area, taper: 0.6, elbowFrac: 0.5, elbowGain: 0.5, amp_deg: 20, ampMax_deg: 34, phiMean_deg: 0,
  gearRatio: 45, staticMargin: 0.1, sparFrac: 0.35, crank_mm: 8, coupler_mm: 70,
  parts: {
    motor: 'motor-eflite-park370-1080', esc: 'esc-hobbywing-skywalker-20a-v2-mini', battery: 'batt-gensace-1000-3s-25c',
    twistServo: 'servo-kst-x06-v6', tailServo: 'servo-kst-x06-v6', strokeServo: 'servo-kst-x06-v6',
    fc: 'fc-ark-v6x', gps: 'gps-holybro-m10', airspeed: 'aspd-matek-4525', receiver: 'rx-matek-elrs-r24-p6',
    sparInner: 'tube-ec-woven-16x14', sparOuter: 'tube-ec-woven-8x6', leRod: 'rod-ec-2mm', teRod: 'rod-ec-1-5mm',
    ribMaterial: 'balsa-wood-database', film: 'film-oracover-air-light', keel: 'tube-ec-woven-8x6',
    gears: ['gear-khk-ds0.8-12', 'gear-khk-ds0.8-80', 'gear-khk-ds0.8-12', 'gear-khk-ds0.8-80'], bearing: 'bearing-skf-624-2z',
  },
};
const d = buildDesign(params, catalog);
d.name = 'smartbird-v4';
d.rho_override = 1.2;
d.atmosphere.rho_kg_m3 = 1.2;
d.drive.gearRatio = G;
// Scale body lumps so the total matches the published mass; wing mass stays as sized.
const wingKg = 2 * d.mass.wing.one_wing_kg;
const bodyKg = d.mass.lumps.reduce((a, l) => a + l.mass_kg, 0);
const k = (mass - wingKg) / bodyKg;
for (const l of d.mass.lumps) l.mass_kg *= k;
d.mass.total_kg = mass;
// Stroke amplitude for a 0.3 m plunge at 3/4 span: y = L1 sin A + (s - L1) sin(1.5 A)
const L1 = d.wing.segments[0].length;
const s34 = 0.75 * (L1 + d.wing.segments[1].length);
let A = 0.2;
for (let i = 0; i < 60; i++) {
  const y = L1 * Math.sin(A) + (s34 - L1) * Math.sin(1.5 * A);
  A += (plunge - y) / (L1 * Math.cos(A) + 1.5 * (s34 - L1) * Math.cos(1.5 * A));
}
const ampPlunge = A;
// SmartBird's own linkage is not ours: a sinusoidal stroke stands in for it (listed in the assumptions).
d.drive = { ...d.drive, kind: 'sine', phiMean: 0, ampNominal: ampPlunge, ampMin: ampPlunge * 0.3, ampMax: Math.min(ampPlunge * 1.5, 50 * DEG) };
let bird = buildBird(d);
// The paper's 2 Hz is an input. Our model at the 0.3 m plunge needs less than 2 Hz to fly level, so hold 2 Hz and
// find the stroke amplitude that trims; report both.
const tPlunge = trimLevel(d, bird, { V: target.speed_m_s, amp: ampPlunge, dyn: pitch * DEG, maxFreq: 4 });
console.log('frozen trim at 4.7 m/s with 0.3 m plunge', JSON.stringify(tPlunge));
// Newton on (theta, stroke amplitude, tail) at the published 2 Hz, frozen body.
function trimAtFreq(V, f0, start) {
  let x = start.slice();
  const ev = (u) => cycleResidual(d, bird, { V, theta: u[0], freq: f0, tail: u[2], amp: u[1], sym: 0, dyn: pitch * DEG, lead: 0 });
  let r = ev(x);
  const nrm = (q) => Math.hypot(q.res[0], q.res[1], 3 * q.res[5]);
  for (let it = 0; it < 14 && nrm(r) > 0.01 * mass * 9.81; it++) {
    const g = [r.res[0], r.res[1], r.res[5]];
    const h = [0.01, 0.01, 0.01];
    const J = [[], [], []];
    for (let c = 0; c < 3; c++) {
      const y = x.slice();
      y[c] += h[c];
      const q = ev(y);
      J[0][c] = (q.res[0] - g[0]) / h[c]; J[1][c] = (q.res[1] - g[1]) / h[c]; J[2][c] = (q.res[5] - g[2]) / h[c];
    }
    const dx = solve3(J, g.map((v) => -v));
    if (!dx) break;
    let step = 1;
    for (let c = 0; c < 3; c++) step = Math.min(step, 0.08 / Math.max(Math.abs(dx[c]), 1e-9));
    let moved = false;
    for (let a2 = step; a2 >= step / 16; a2 /= 2) {
      const y = [x[0] + a2 * dx[0], Math.max(0.05, x[1] + a2 * dx[1]), x[2] + a2 * dx[2]];
      const q = ev(y);
      if (nrm(q) < nrm(r)) { x = y; r = q; moved = true; break; }
    }
    if (!moved) break;
  }
  return { theta: x[0], amp: x[1], tail: x[2], power: r.power, stalled: r.stalled, residual: nrm(r), converged: nrm(r) < 0.05 * mass * 9.81 };
}
function solve3(A, b) {
  const m = A.map((r, i) => [...r, b[i]]);
  for (let k = 0; k < 3; k++) {
    let p = k;
    for (let i = k + 1; i < 3; i++) if (Math.abs(m[i][k]) > Math.abs(m[p][k])) p = i;
    [m[k], m[p]] = [m[p], m[k]];
    if (Math.abs(m[k][k]) < 1e-12) return null;
    for (let i = k + 1; i < 3; i++) { const f = m[i][k] / m[k][k]; for (let j = k; j < 4; j++) m[i][j] -= f * m[k][j]; }
  }
  const x = [0, 0, 0];
  for (let i = 2; i >= 0; i--) { let v = m[i][3]; for (let j = i + 1; j < 3; j++) v -= m[i][j] * x[j]; x[i] = v / m[i][i]; }
  return x;
}
const t = trimAtFreq(target.speed_m_s, freq, [0.1, 12 * DEG, 0.02]);
console.log('trim at 2 Hz', JSON.stringify(t));
const amp = t.amp;
console.log('amplitude for 2 Hz', amp / DEG, JSON.stringify(t));
d.drive = { ...d.drive, kind: 'sine', ampNominal: amp, ampMin: amp * 0.6, ampMax: Math.min(amp * 1.6, 50 * DEG) };
bird = buildBird(d);
const ctl = buildControl(d, { amp, sym: 0, dyn: pitch * DEG * (amp / ampPlunge), lead: 0, V: target.speed_m_s }, { speeds: [4.7, 5.0], bird });
let r47, r5;
if (ctl) {
  d.control = ctl;
  await refineTrimByFlight(d, { passes: 1 });
}
function fly(V) {
  const { sim, ctl: c, tick } = setupFlight(d, { sensorNoise: false, init: { V, y: 0 } });
  c.guidance.speed = V;
  let P = 0, f = 0, Vm = 0, n = 0, Pmech = 0;
  for (let k2 = 0; k2 < 240 * 40; k2++) {
    tick();
    if (k2 > 240 * 20) {
      P += sim.tel.power_w;
      f += sim.snapshot().crank / (2 * Math.PI);
      Vm += sim.tel.airspeed;
      Pmech += sim.tel.crankTorque * sim.snapshot().crank;
      n++;
    }
  }
  return n ? { power_w: P / n, freq_hz: f / n, airspeed_m_s: Vm / n, crank_mech_w: Pmech / n } : { power_w: NaN };
}
if (ctl) {
  r47 = fly(target.speed_m_s);
  r5 = fly(design5.speed_m_s);
} else {
  r47 = { power_w: NaN, note: 'no trim table: free flight not run' };
  r5 = { power_w: NaN };
}
// Frozen-body trim at 2 Hz: the drive's electrical power from the crank load, plus servos and avionics at 5 V.
const frozenElectric = t.power + 5 * (d.electrical.avionics_A_5V ?? 0.35);
const lo = 12.6, hi = 23.4; // SPEC declared band
const ev = writeJson('out/verification/evidence/V4.json', { assumptions, published: { target, design5, span, mass, area, freq, plunge, pitch_deg: pitch, gear: G }, frozen_trim_4_7_at_published_plunge: tPlunge, frozen_trim_4_7_at_2Hz: t, plunge_amplitude_deg: ampPlunge / DEG, flight_4_7: r47, flight_5_0: r5, amplitude_deg: amp / DEG, trimFlight: d.control?.trimFlight ?? null });
const usedP = Number.isFinite(r47.power_w) ? r47.power_w : frozenElectric;
const method = Number.isFinite(r47.power_w) ? 'free flight, closed loop' : 'frozen-body trim at 2 Hz (no free-flight trim table could be built; free flight not run)';
writeGate({ gate: 'V4', passed: usedP >= lo && usedP <= hi, measured: { method, predicted_used_w: usedP, error_used_pct: (usedP / target.electrical_power_w - 1) * 100, frozen_body_trim_2hz_w: frozenElectric, frozen_trim: t, predicted_w: r47.power_w, at_speed_m_s: r47.airspeed_m_s, flap_freq_hz: r47.freq_hz, error_pct: (r47.power_w / target.electrical_power_w - 1) * 100, crank_mech_w: r47.crank_mech_w, design_point_5ms_predicted_w: r5.power_w, design_point_5ms_published_w: design5.electrical_power_w, design_point_error_pct: (r5.power_w / design5.electrical_power_w - 1) * 100, assumptions_count: assumptions.length }, limit: { published_w: target.electrical_power_w, band_w: [lo, hi] }, evidence: 'out/verification/evidence/V4.json' });
console.log(`V4: used ${usedP.toFixed(1)} W (${method}); free flight ${r47.power_w} W (published 18 W at 4.7 m/s; 23 W at 5 m/s)`);
