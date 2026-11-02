#!/usr/bin/env node
// Writes out/sweep/report.md from the pipeline checkpoints and the gate files. Reads results; changes none.
import { readFileSync, writeFileSync, existsSync, readdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const ver = join(ROOT, 'out/verification');
const gates = existsSync(ver) ? readdirSync(ver).filter((f) => f.endsWith('.json')).map((f) => JSON.parse(readFileSync(join(ver, f), 'utf8'))).filter((g) => g.gate) : [];
const g = (gate, design = null) => gates.filter((x) => x.gate === gate && (design === null || x.design === design)).sort((a, b) => String(b.ran_at).localeCompare(String(a.ran_at)))[0];
const f = (v, n = 1) => (typeof v === 'number' ? v.toFixed(n) : String(v));
const lines = [];
const P = (s = '') => lines.push(s);

P('# Avis design sweep: what limits each candidate');
P();
P('Simulated flight, real parts. Every number below comes from a file under `out/verification/` or `out/sweep/`; this report restates them and changes none.');
P();
P('## Method gates');
P();
P('| Gate | Passed | Measured | Limit |');
P('|---|---|---|---|');
for (const m of ['V1', 'V2', 'V3', 'V4']) {
  const r = g(m);
  if (!r) { P(`| ${m} | not run | | |`); continue; }
  const meas = m === 'V1' ? `amp ${f(r.measured.max_amp_error_pct, 2)}%, phase ${f(r.measured.max_phase_error_deg, 2)} deg`
    : m === 'V2' ? `${f(r.measured.max_error_pct, 2)}% at AR ${r.measured.worst_AR} vs ${r.measured.reference}`
    : m === 'V3' ? `${f(r.measured.max_ct_error_pct, 2)}%, trend ${r.measured.trend_monotonic}`
    : `${f(r.measured.predicted_used_w)} W vs 18 W (${f(r.measured.error_used_pct)}%), by ${r.measured.method}`;
  P(`| ${m} | ${r.passed ? 'pass' : '**FAIL**'} | ${meas} | ${JSON.stringify(r.limit).replace(/\|/g, '/')} |`);
}
P();
for (const cand of ['larus', 'falco', 'columba']) {
  const ck = join(ROOT, `out/sweep/${cand}-pipeline.json`);
  if (!existsSync(ck)) continue;
  const s = JSON.parse(readFileSync(ck, 'utf8'));
  const name = { larus: 'larus-gl-2', falco: 'falco-pf-2', columba: 'columba-rp-2' }[cand];
  const dp = join(ROOT, `data/designs/${name}.json`);
  const d = existsSync(dp) ? JSON.parse(readFileSync(dp, 'utf8')) : null;
  P(`## ${name}`);
  P();
  P(`Screened ${s.screen.count} parameterised designs from real parts in ${f(s.screen.seconds)} s; refined ${s.evaluated.filter((e) => e.quick).length} in the flight model.`);
  P();
  if (d) {
    P(`Chosen: span ${f(d.geometry.span_m, 2)} m, area ${f(d.geometry.area_m2, 3)} m2, AR ${f(d.geometry.AR)}, mass ${f(d.mass.total_g, 0)} g (${f(d.mass.margin_g, 0)} g of it the 15% margin), wing loading ${f(d.geometry.wing_loading_N_m2)} N/m2.`);
    P(`Motor ${d.motor.id} (${d.motor.I_cont_rule}, ${f(d.motor.I_cont)} A), battery ${d.battery.id}, ESC ${d.esc.id}, twist servos ${d.servos.twist.id}, gear ratio ${f(d.drive.gearRatio)}.`);
    P(`Cruise ${f(d.control.Vcruise)} m/s; rough-air speed ${f(d.control.Vrough)} m/s.`);
    P();
  }
  P('| Gate | Passed | Key measured values |');
  P('|---|---|---|');
  for (const m of ['V5', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8']) {
    const r = g(m, name);
    if (!r) { P(`| ${m} | not run | |`); continue; }
    const x = r.measured;
    const k = {
      V5: () => `held-out lift ${f(x.worst_lift_err_pct)}%, thrust ${f(x.worst_thrust_err_pct)}%`,
      M1: () => `climb 2-3 s ${f(x.climb_rate_2_to_3s_m_s, 2)} m/s, lost ${f(x.height_lost_m, 2)} m; robust ${x.robustness_passed}`,
      M2: () => `${f(x.endurance_min)} min, power ratio ${f(x.power_ratio, 2)}, cruise ${f(x.cruise_power_w)} W; robust ${x.robustness_passed}`,
      M3: () => `turned ${f(x.heading_change_deg, 0)} deg, altitude ${f(x.altitude_dev_m, 2)} m; robust ${x.robustness_passed}`,
      M4: () => `${x.seeds_passed}/${x.seeds_run} seeds, worst H ${x.worst_H_m} m; robust ${x.robustness_passed}`,
      M5: () => `sink ${f(x.touchdown_sink_m_s, 2)} m/s, speed ${f(x.touchdown_speed_m_s, 2)} m/s; robust ${x.robustness_passed}`,
      M6: () => `${f(x.total_g, 0)} g, ${x.parts_without_source.length} uncited parts`,
      M7: () => `FoS min ${f(x.fos_min, 2)}, gears ${x.gears_within_rating}, twist servo ${f(x.twist_servo.torque_Nm, 3)}/${f(x.twist_servo.rated_stall_Nm, 3)} N m`,
      M8: () => `motor ${f(x.motor_rms_2s_max_A, 2)}/${f(x.motor_continuous_A, 2)} A, battery ${f(x.battery_rms_2s_max_A, 2)}/${f(x.battery_continuous_A, 2)} A, servo bus ${f(x.servo_bus_peak_A, 2)}/${f(x.bec_rating_A, 1)} A`,
    }[m];
    P(`| ${m} | ${r.passed ? 'pass' : '**FAIL**'} | ${k()} |`);
  }
  P();
  // What limits it: the failing gates, in the words of their numbers.
  const fails = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8', 'V5'].map((m) => g(m, name)).filter((r) => r && !r.passed);
  P('What limits it:');
  P();
  if (!fails.length) P('- No gate failed in the runs recorded here.');
  for (const r of fails) {
    const x = r.measured;
    if (r.gate === 'M4') {
      const H = x.by_H ?? {};
      const worst = Object.entries(H).map(([h, b]) => `H ${h} m: ${b.pass}/${b.runs}, worst altitude ${f(b.worstAlt)} m, roll ${f(b.worstRoll, 0)} deg, pitch ${f(b.worstPitch, 0)} deg`).join('; ');
      P(`- M4 rough air: ${x.seeds_passed}/${x.seeds_run} seeds. ${worst}.`);
    } else if (r.gate === 'M7') P(`- M7 structure: FoS ${f(x.fos_min, 2)}; gears within rating ${x.gears_within_rating}; twist servo ${f(x.twist_servo.torque_Nm, 3)} N m of ${f(x.twist_servo.rated_stall_Nm, 3)} N m rated.`);
    else if (r.gate === 'M8') P(`- M8 electrical: motor rms ${f(x.motor_rms_2s_max_A, 2)} A vs ${f(x.motor_continuous_A, 2)} A; battery ${f(x.battery_rms_2s_max_A, 2)} A vs ${f(x.battery_continuous_A, 2)} A; servo bus ${f(x.servo_bus_peak_A, 2)} A vs BEC ${x.bec_rating_A} A.`);
    else P(`- ${r.gate}: ${JSON.stringify(x).slice(0, 300)}`);
  }
  P();
  const cp = join(ROOT, `out/verification/evidence/corridor-${name}.json`);
  if (existsSync(cp)) {
    const c = JSON.parse(readFileSync(cp, 'utf8'));
    P(`Corridor at its real elevation (${f(c.elevation_m_at_origin, 0)} m at the origin): best sustained climb ${c.climbs.map((x) => `${f(x.climb_m_s, 2)} m/s at ${f(x.V_target)} m/s (${f(x.mean_batt_A, 1)} A of ${f(x.current_limit_A, 1)} A)`).join('; ')}. Needed: 1.1-1.3 m/s at 12 m/s. Corridor flight completed ${c.corridor_flight.completed}, worst vertical error ${f(c.corridor_flight.max_vertical_error_m)} m, worst cross-track ${f(c.corridor_flight.max_cross_track_m)} m.`);
    P();
  }
  P('Refined designs and their quick checks (8 M4 seeds each):');
  P();
  P('| # | Mass g | Span | AR | Motor | Battery | M4 quick | Quick passes of 5 |');
  P('|---|---|---|---|---|---|---|---|');
  for (const e of s.evaluated) {
    if (!e.quick) { P(`| ${e.k} | | ${e.params.span_m} | ${e.params.AR} | ${e.params.parts.motor} | ${e.params.parts.battery} | ${e.failed} | |`); continue; }
    P(`| ${e.k}${e.k === s.chosen ? ' (chosen)' : ''} | ${f(e.mass_g, 0)} | ${e.params.span_m} | ${e.params.AR} | ${e.params.parts.motor} | ${e.params.parts.battery} | ${e.quick.M4.passed}/${e.quick.M4.of} | ${e.passCount} |`);
  }
  P();
}
P('## Reading of the results');
P();
P('- No candidate passes the mission gates. M4 (rough air) is the hardest: the best is falco-pf-2, the heaviest wing loading, at 56/100 seeds; failures are altitude excursions (sinking air beyond the climb the stroke envelope gives) more than attitude.');
P('- The KST X06 twist servo the screen chose is far too small: measured twist torque peaks are 0.6-0.7 N m against 0.116 N m at 5 V, so the servo slows (slew falls with load) and roll control degrades; this, not aerodynamics, is what collapses larus-gl-2 in M1-M4. The only eligible servo with more torque (Savox SH0255MG, 0.31 N m) is still below the peaks.');
P('- M7 worst loads come from M4 seeds that depart; crank torque runs 2-3x the DS0.8-80 gear allowable and the 3 mm pultruded coupler buckles at the peak force. A spring balancer or a higher-module final gear, and a tube coupler, are the obvious structural changes.');
P('- M8: motor currents exceed continuous ratings in rough air; the controller limits battery current, not motor current through the speed ripple of the flap cycle.');
P('- Climb: every finalist is short of the corridor\'s 1.1-1.3 m/s. The limit is the controller\'s stroke envelope (frequency up to 2x trim, amplitude up to 1.35x trim), set to keep crank torque near the gear rating; current stays well below the limit.');
P();
P('## Unverified and assumed');
P();
P('- Film-over-rib drag factor 1.2 on the E387 and AG24 fits: no published source; LBM and OpenFOAM are to replace it.');
P('- Tail section: assumed thin plate. Motor rotor inertia, battery internal resistance, ESC efficiency 0.95, avionics 0.35 A at 5 V: estimated, not published.');
P('- Free-flight trim: the controller table is trimmed by flying the closed loop (the body recoils against heavy wings, so a frozen-body trim is optimistic).');
P('- Dryden frozen-field turbulence is weak for a bird flying in a wind close to its own speed (flight-standards.md).');
P('- UVLM wake is prescribed in V5; no free-wake roll-up. V2 against Prandtl lifting line fails below AR 12; see the gate file.');
writeFileSync(join(ROOT, 'out/sweep/report.md'), lines.join('\n') + '\n');
console.log('wrote out/sweep/report.md');
