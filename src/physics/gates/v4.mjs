#!/usr/bin/env node
// V4: the whole model on SmartBird (Send et al., ICAS 2012). SPEC ruling after round 1: impose the published
// kinematics (2 Hz, about 0.3 m plunge, about 9 deg pitch) at 4.7 m/s on the published geometry and mass; report
// the cycle-mean net streamwise force (near zero in steady flight) and the electrical power including motor,
// gearbox, ESC and avionics, with no regeneration credit. Pass: power in 12.6-23.4 W. The 23 W design point at
// 5 m/s is reported beside it. Every unpublished quantity is an assumption listed in the evidence.
//   node src/physics/gates/v4.mjs
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { cycleResidual, TRIM_TOL } from '../../design/trim.js';
import { buildBird } from '../flight.js';
import { SUCTION } from '../strip.js';
import { smartbirdDesign, SMARTBIRD_ASSUMPTIONS, SECTION_FRAC } from './smartbird.js';
import { writeGate, writeJson, ROOT } from './common.js';

const DEG = Math.PI / 180;
const G0 = 9.80665;
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
const bench = JSON.parse(readFileSync(join(ROOT, 'data/references/benchmarks.json'), 'utf8')).smartbird;

/**
 * Body held level at speed V with the kinematics imposed; Newton on (pitch, tail) for vertical force and pitching
 * moment. Returns the converged state or { converged: false } — a non-converged balance is a failure.
 */
export function imposedBalance(d, bird, kin, V, { dyn = kin.dyn_rad, lead = 0, freq = kin.freq_hz } = {}) {
  const W = bird.mTotal * G0;
  const ev = (u) => cycleResidual(d, bird, { V, theta: u[0], freq, tail: u[1], amp: kin.amp_rad, sym: 0, dyn, lead, imposeTwist: true });
  const nrm = (q) => Math.hypot(q.res[1], 3 * q.res[5]);
  let x = [0.08, 0.02];
  let r = ev(x);
  const path = [];
  for (let it = 0; it < 20 && nrm(r) > TRIM_TOL * W; it++) {
    const g = [r.res[1], r.res[5]];
    const h = [0.01, 0.01];
    const J = [[0, 0], [0, 0]];
    for (let c = 0; c < 2; c++) {
      const y = x.slice();
      y[c] += h[c];
      const q = ev(y);
      J[0][c] = (q.res[1] - g[0]) / h[c];
      J[1][c] = (q.res[5] - g[1]) / h[c];
    }
    const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (Math.abs(det) < 1e-12) break;
    const dx = [(-g[0] * J[1][1] + g[1] * J[0][1]) / det, (-g[1] * J[0][0] + g[0] * J[1][0]) / det];
    let step = Math.min(1, 0.1 / Math.max(Math.abs(dx[0]), 1e-9), 0.1 / Math.max(Math.abs(dx[1]), 1e-9));
    let moved = false;
    for (let a = step; a >= step / 32; a /= 2) {
      const y = [x[0] + a * dx[0], x[1] + a * dx[1]];
      const q = ev(y);
      if (nrm(q) < nrm(r)) { x = y; r = q; moved = true; break; }
    }
    path.push({ theta_deg: x[0] / DEG, tail_deg: x[1] / DEG, residual_frac_weight: nrm(r) / W });
    if (!moved) break;
  }
  const converged = nrm(r) <= TRIM_TOL * W;
  return {
    converged, V, freq_hz: freq, theta_deg: x[0] / DEG, tail_deg: x[1] / DEG,
    residual_frac_weight: nrm(r) / W, tolerance_frac_weight: TRIM_TOL,
    net_streamwise_force_N: r.res[0], net_streamwise_over_weight: r.res[0] / W,
    vertical_residual_N: r.res[1], pitch_moment_residual_Nm: r.res[5],
    power: { drive_battery_w: r.power_drive, bus_5v_model_w: r.power_5v, total_model_w: r.power, total_with_regen_credit_w: r.power_regen_credit },
    crank_mech_mean_w: r.mechP, crank_mech_positive_w: r.mechP_positive, crank_torque_peak_Nm: r.crankPeak, twist_torque_peak_Nm: r.twistPeak,
    stalled_fraction: r.stalled, cycles: r.cycles, path,
  };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const { design: d, published: P, kinematics: kin } = smartbirdDesign(catalog, bench);
  const bird = buildBird(d);
  // Our 5 V bus model with the stand-in servos and avionics; the paper's own measured servo bias as a second sum.
  const avionicsW = (5 * (d.electrical.avionics_A_5V ?? 0.35)) / 0.85;
  const idleW = (5 * (2 * (d.servos.twist.idle_A ?? 0) + 2 * (d.servos.tail.idle_A ?? 0) + (d.servos.stroke.idle_A ?? 0))) / 0.85;
  const totals = (b) => {
    const servoActive = Math.max(0, b.power.bus_5v_model_w - avionicsW - idleW);
    return {
      model_total_w: b.power.total_model_w,
      drive_w: b.power.drive_battery_w,
      servo_active_model_w: servoActive,
      avionics_model_w: avionicsW,
      servo_idle_model_w: idleW,
      with_published_servo_bias_w: b.power.drive_battery_w + servoActive + P.servoBias,
      net_flapping_w: b.power.drive_battery_w + servoActive,
    };
  };
  const at47 = imposedBalance(d, bird, kin, P.target.speed_m_s);
  const at5 = imposedBalance(d, bird, kin, P.design5.speed_m_s);
  // Sensitivity to the assumptions, for information (never used to pass).
  const sens = {};
  {
    const tipOnly = { ...kin, dyn_rad: P.pitchDeg * DEG };
    sens.pitch_9deg_at_tip = imposedBalance(d, bird, tipOnly, P.target.speed_m_s);
    sens.phase_kappa_100deg = imposedBalance(d, bird, kin, P.target.speed_m_s, { lead: 10 * DEG });
    // Round 6: 0.12, near the cfd workstream's first plunge measurements (0.11-0.14), at the orchestrator's request.
    for (const eta of [1.0, 0.98, 0.18, 0.12]) sens[`suction_eta_${eta}`] = imposedBalance(d, buildBird(d, { suctionEta: eta }), kin, P.target.speed_m_s);
  }
  const T47 = at47.converged ? totals(at47) : null;
  const T5 = at5.converged ? totals(at5) : null;
  const lo = 12.6, hi = 23.4; // SPEC declared band on 18 W
  const used = T47 ? T47.model_total_w : NaN;
  const passed = at47.converged && used >= lo && used <= hi;
  const evidence = writeJson('out/verification/evidence/V4.json', {
    assumptions: SMARTBIRD_ASSUMPTIONS,
    published: P,
    imposed_kinematics: kin,
    section_station_frac_of_semispan: SECTION_FRAC,
    leading_edge_suction: SUCTION,
    at_4_7: { ...at47, totals: T47 },
    at_5_0: { ...at5, totals: T5 },
    sensitivity_information_only: Object.fromEntries(Object.entries(sens).map(([k, v]) => [k, { converged: v.converged, net_streamwise_force_N: v.net_streamwise_force_N, totals: v.converged ? totals(v) : null, theta_deg: v.theta_deg, stalled_fraction: v.stalled_fraction }])),
    regeneration: 'SPEC ruling: none credited; the ESC (Hobbywing Skywalker 20A V2 mini) documentation read for the catalogue states no regenerative operation.',
  });
  writeGate({
    gate: 'V4',
    passed,
    measured: {
      method: 'published kinematics imposed (2 Hz, 0.3 m plunge, 9 deg pitch at the 3/4-semispan section), body held level at 4.7 m/s, pitch and tail balanced for lift and pitching moment',
      balance_converged: at47.converged,
      balance_residual_frac_weight: at47.residual_frac_weight,
      predicted_w: used,
      error_pct: (used / P.target.electrical_power_w - 1) * 100,
      net_streamwise_force_N: at47.net_streamwise_force_N,
      net_streamwise_over_weight: at47.net_streamwise_over_weight,
      breakdown_w: T47,
      with_published_servo_bias_w: T47?.with_published_servo_bias_w ?? null,
      net_flapping_w_vs_published_13: T47?.net_flapping_w ?? null,
      body_pitch_deg: at47.theta_deg,
      stalled_fraction: at47.stalled_fraction,
      design_point_5ms: { converged: at5.converged, predicted_w: T5?.model_total_w ?? null, published_w: P.design5.electrical_power_w, net_streamwise_force_N: at5.net_streamwise_force_N },
      leading_edge_suction_efficiency: SUCTION.eta,
      assumptions_count: SMARTBIRD_ASSUMPTIONS.length,
    },
    limit: { published_w: P.target.electrical_power_w, band_w: [lo, hi] },
    evidence: evidence.slice(ROOT.length + 1),
  });
  console.log(`V4 ${passed ? 'pass' : 'FAIL'}: ${used.toFixed(2)} W vs 18 W (${(((used / 18) - 1) * 100).toFixed(1)}%), net Fx ${at47.net_streamwise_force_N.toFixed(3)} N, converged ${at47.converged}`);
}
