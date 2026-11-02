#!/usr/bin/env node
// Mission gates M1-M8 and the robustness sweep for one design, on the 6-DOF model with the flight controller.
//   node src/physics/gates/missions.mjs data/designs/larus-gl-2.json [--seeds 100] [--corners]
// Writes out/verification/M1-<design>.json ... M8, out/verification/robust-<design>.json, out/flight/<design>-*.json.
import { readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { runPool } from './pool.js';
import { writeGate, writeJson, ROOT } from './common.js';
import { LIMITS, robustnessCorners, runM1, runM2, runM3, runM4Seed, runM5, runCorridor } from '../scenarios.js';
import { createWind } from '../wind.js';

const path = resolve(process.argv[2]);
const design = JSON.parse(readFileSync(path, 'utf8'));
const name = design.name;
const seeds = +(process.argv.includes('--seeds') ? process.argv[process.argv.indexOf('--seeds') + 1] : 100);
const doCorners = process.argv.includes('--corners');
const cornerSeeds = +(process.argv.includes('--corner-seeds') ? process.argv[process.argv.indexOf('--corner-seeds') + 1] : seeds);
const rel = (p) => p.slice(ROOT.length + 1);

// ---- Nominal runs with logs (M1, M2, M3, M5) and the M4 seeds through the pool --------------------------------
const logOut = (scen, r) => {
  if (r.log) {
    r.log.scenario = `${name}-${scen}`;
    writeJson(`out/flight/${name}-${scen}.json`, r.log);
    delete r.log;
  }
};
const nominal = {};
for (const [m, fn] of [['M1', runM1], ['M2', runM2], ['M3', runM3], ['M5', runM5]]) {
  const r = fn(design, { log: true, ...(m === 'M2' ? { tEnd: 120 } : {}) });
  logOut(m, r);
  nominal[m] = r;
  console.log(m, r.passed ? 'pass' : 'FAIL', JSON.stringify(r.measured).slice(0, 200));
}
const m4log = runM4Seed(design, 1, { log: true });
logOut('M4-seed1', m4log);
const jobs = [];
for (let s = 1; s <= seeds; s++) jobs.push({ task: 'M4', args: { design: path, seed: s, opts: {} } });
const m4 = await runPool(jobs);
const m4ok = m4.filter((r) => r && !r.error && r.passed).length;
const byH = {};
for (const r of m4) {
  if (!r || r.error) continue;
  byH[r.H] ??= { runs: 0, pass: 0, worstAlt: 0, worstRoll: 0, worstPitch: 0 };
  const b = byH[r.H];
  b.runs++;
  if (r.passed) b.pass++;
  b.worstAlt = Math.max(b.worstAlt, r.measured.altitude_dev_m);
  b.worstRoll = Math.max(b.worstRoll, r.measured.max_roll_deg);
  b.worstPitch = Math.max(b.worstPitch, r.measured.max_pitch_deg);
}
const worstH = Object.entries(byH).sort((a, b) => a[1].pass / a[1].runs - b[1].pass / b[1].runs)[0]?.[0] ?? null;
const ev4 = writeJson(`out/verification/evidence/M4-${name}.json`, { seeds: m4.map((r, i) => (r && !r.error ? { seed: i + 1, passed: r.passed, H: r.H, ...r.measured } : { seed: i + 1, error: r?.error })), byH, note: 'Dryden low-altitude at 30 m AGL, sigma_u = sigma_v = 2 m/s, sigma_w by the standard ratio; 1-cosine gusts of 4 m/s, H per seed from {5,10,20,40} m, vertical and lateral, 4 per run. Frozen-field turbulence at airspeed: weak for a bird in a wind near its own speed.' });

function writeAll(robust) {
// ---- Gate files ------------------------------------------------------------------------------------------------
const robustNote = (m) => (robust ? Object.fromEntries(Object.entries(robust).map(([k, v]) => [k, m === 'M4' ? `${v.M4pass}/${v.M4runs}` : v[m]])) : 'not run');
const robustOk = (m) => (robust ? Object.values(robust).every((v) => (m === 'M4' ? v.M4pass >= 0.98 * v.M4runs : v[m])) : null);
for (const m of ['M1', 'M2', 'M3', 'M5']) {
  const r = nominal[m];
  const rOk = robustOk(m);
  writeGate({ gate: m, design: name, passed: r.passed && rOk !== false, measured: { ...r.measured, nominal_passed: r.passed, robustness_passed: rOk, robustness: robustNote(m) }, limit: LIMITS[m], evidence: `out/flight/${name}-${m}.json` });
}
writeGate({ gate: 'M4', design: name, passed: m4ok >= 98 * (seeds / 100) && robustOk('M4') !== false && seeds >= 100, measured: { seeds_passed: m4ok, seeds_run: seeds, worst_H_m: worstH ? +worstH : null, by_H: byH, robustness_passed: robustOk('M4'), robustness: robustNote('M4') }, limit: LIMITS.M4, evidence: rel(ev4) });

// ---- M6 mass: BOM completeness and margin ---------------------------------------------------------------------
const bom = design.bom;
const missing = bom.parts.filter((p) => !p.source_url || !p.id);
const m6ok = missing.length === 0 && Math.abs(bom.margin_g - 0.15 * (bom.parts_g + bom.structure_g)) < 1e-6 && Math.abs(bom.total_g - design.mass.total_g) < 0.5;
writeGate({ gate: 'M6', design: name, passed: m6ok, measured: { total_g: design.mass.total_g, parts_g: bom.parts_g, structure_g: bom.structure_g, margin_g: bom.margin_g, margin_frac: 0.15, parts_without_source: missing.map((p) => p.id), structure_items: bom.structure.length }, limit: { margin_frac: 0.15, every_part_cited: true }, evidence: rel(path) });

// ---- M7 structure at the worst load seen in M1-M5 and all M4 seeds ---------------------------------------------
const allPeaks = [nominal.M1, nominal.M2, nominal.M3, nominal.M5, ...m4.filter((r) => r && !r.error)].map((r) => r.peaks);
const worst = (k) => Math.max(...allPeaks.map((p) => p[k] ?? 0));
const st = design.structure;
const sIn = st.spar_inner, sOut = st.spar_outer;
const sigIn = (worst('rootBend') * sIn.c) / sIn.I / 1e6;
const sigOut = (worst('elbowBend') * sOut.c) / sOut.I / 1e6;
const fosIn = sIn.allowable_MPa / sigIn;
const fosOut = sOut.allowable_MPa / sigOut;
const gears = st.gear_stages;
const crankT = worst('crankTorque');
const G2 = gears[gears.length - 1];
const gearOk = gears.every((g, i) => {
  const below = gears.slice(i + 1).reduce((r, q) => r * q.ratio, 1);
  const tGear = crankT / below;
  return tGear <= g.gear_allowable_Nm && tGear / g.ratio <= g.pinion_allowable_Nm;
});
const gearUse = gears.map((g, i) => {
  const below = gears.slice(i + 1).reduce((r, q) => r * q.ratio, 1);
  return { gear: g.gear, torque_Nm: crankT / below, allowable_Nm: g.gear_allowable_Nm, pinion: g.pinion, pinion_torque_Nm: crankT / below / g.ratio, pinion_allowable_Nm: g.pinion_allowable_Nm };
});
const cp = st.coupler;
const Fc = worst('couplerForce');
const Irod = (Math.PI * cp.od_m ** 4) / 64;
const Pcr = (Math.PI ** 2 * cp.E_GPa * 1e9 * Irod) / cp.length_m ** 2;
const sigRod = Fc / ((Math.PI * cp.od_m ** 2) / 4) / 1e6;
const fosRodBuckle = Pcr / Fc;
const fosRodStress = 400 / sigRod;
const twistT = worst('twistTorque');
const tailT = worst('tailTorque');
const servoOk = twistT <= design.servos.twist.stall_Nm && tailT <= design.servos.tail.stall_Nm;
const fosMin = Math.min(fosIn, fosOut, fosRodBuckle, fosRodStress);
writeGate({
  gate: 'M7', design: name, passed: fosMin >= 1.5 && gearOk && servoOk && seeds >= 100,
  measured: {
    spar_inner: { id: sIn.id, moment_Nm: worst('rootBend'), stress_MPa: sigIn, allowable_MPa: sIn.allowable_MPa, fos: fosIn },
    spar_outer: { id: sOut.id, moment_Nm: worst('elbowBend'), stress_MPa: sigOut, allowable_MPa: sOut.allowable_MPa, fos: fosOut },
    coupler: { id: cp.id, force_N: Fc, buckling_N: Pcr, fos_buckling: fosRodBuckle, stress_MPa: sigRod, fos_stress: fosRodStress, E_GPa: cp.E_GPa },
    crank_torque_Nm: crankT, gears: gearUse, gears_within_rating: gearOk,
    twist_servo: { torque_Nm: twistT, rated_stall_Nm: design.servos.twist.stall_Nm }, tail_servo: { torque_Nm: tailT, rated_stall_Nm: design.servos.tail.stall_Nm },
    peak_load_factor: worst('loadFactor'), fos_min: fosMin,
    note: 'Worst load over M1, M2, M3, M5 and every M4 seed. Gear and servo torques against their published ratings; spars and coupler against published strength with FoS 1.5.',
  },
  limit: { factor_of_safety: 1.5, gear_torque: 'within catalogue allowable', servo_torque: 'within published stall-style torque' },
  evidence: rel(ev4),
});

// ---- M8 electrical through M1-M5 -----------------------------------------------------------------------------
const all = [nominal.M1, nominal.M2, nominal.M3, nominal.M5, ...m4.filter((r) => r && !r.error)];
const motorRms = Math.max(...all.map((r) => r.rms.motorRmsMax));
const battRms = Math.max(...all.map((r) => r.rms.battRmsMax));
const servo5 = Math.max(...all.map((r) => r.peaks.servoI));
const m8 = {
  motor_rms_2s_max_A: motorRms, motor_continuous_A: design.motor.I_cont, motor_rule: design.motor.I_cont_rule, motor_peak_A: Math.max(...all.map((r) => r.peaks.motorI)),
  esc_rms_2s_max_A: motorRms, esc_continuous_A: design.esc.I_cont,
  battery_rms_2s_max_A: battRms, battery_continuous_A: design.battery.I_cont, battery_peak_A: Math.max(...all.map((r) => r.peaks.battI)),
  servo_bus_peak_A: servo5, bec_rating_A: design.esc.bec_A, servo_model: 'stall current x torque/stall torque + published idle',
};
const m8ok = motorRms <= design.motor.I_cont && motorRms <= design.esc.I_cont && battRms <= (design.battery.I_cont ?? 0) && servo5 <= (design.esc.bec_A ?? 0);
writeGate({ gate: 'M8', design: name, passed: m8ok && seeds >= 100, measured: m8, limit: { continuous: 'rms over 2 s windows within published continuous ratings (SPEC rule where none published)' }, evidence: rel(ev4) });

}
let robust = null;
writeAll(null);
console.log('gate files written (robustness not yet run)');
// ---- Robustness corners ----------------------------------------------------------------------------------------
if (doCorners) {
  const corners = robustnessCorners();
  const cj = [];
  for (const c of corners) {
    const perturb = { aero: c.aero, mass: c.mass, cgx: c.cgx };
    for (const t of ['M1', 'M2', 'M3', 'M5']) cj.push({ task: t, args: { design: path, opts: { perturb, ...(t === 'M2' ? { tEnd: 60 } : {}) } }, corner: c.label });
    for (let s = 1; s <= Math.min(cornerSeeds, 100); s++) cj.push({ task: 'M4', args: { design: path, seed: s, opts: { perturb } }, corner: c.label });
  }
  const res = await runPool(cj);
  robust = {};
  res.forEach((r, i) => {
    const c = cj[i].corner;
    const t = cj[i].task;
    robust[c] ??= { M1: null, M2: null, M3: null, M5: null, M4pass: 0, M4runs: 0 };
    if (t === 'M4') {
      robust[c].M4runs++;
      if (r && !r.error && r.passed) robust[c].M4pass++;
    } else robust[c][t] = r && !r.error ? r.passed : false;
  });
  const allOk = Object.values(robust).every((c) => c.M1 && c.M2 && c.M3 && c.M5 && c.M4pass >= 0.98 * c.M4runs);
  writeJson(`out/verification/robust-${name}.json`, { design: name, passed: allOk, corners: robust, m4_seeds_per_corner: Math.min(cornerSeeds, 100), includes: 'sensor noise and bias, servo slew from published speed and lag, 20 ms command delay, in every run', ran_at: new Date().toISOString() });
  console.log('robustness', allOk ? 'pass' : 'FAIL');
}

if (robust) writeAll(robust);
// The corridor is flown by src/physics/gates/corridor.mjs at its real elevation.
console.log(`M4 ${m4ok}/${seeds}`);
