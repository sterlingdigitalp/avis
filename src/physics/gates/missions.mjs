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
import { couplerCheck, torsionCheck, strokeServoHold, mixedMeshCheck } from '../structure.js';
import { checkJoints } from '../bearings.js';
import { checkMembers, memberPeaks } from '../members.js';
import { flownMakersParts } from '../../design/chains.js';

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
// ---- Time-step convergence (SPEC ruling): the same flights at half the step. Peak loads must change < 10%. -----
const DT = design.control?.dt ?? 1 / 240;
const half = { dt: DT / 2 };
const nominalHalf = {};
for (const [m, fn] of [['M1', runM1], ['M2', runM2], ['M3', runM3], ['M5', runM5]]) nominalHalf[m] = fn(design, { ...half, ...(m === 'M2' ? { tEnd: 120 } : {}) });
const passingSeeds = m4.map((r, i) => (r && !r.error && r.passed ? i + 1 : null)).filter((x) => x !== null);
const halfSeeds = passingSeeds.length ? passingSeeds : m4.map((r, i) => i + 1).slice(0, 20);
const m4half = await runPool(halfSeeds.map((s) => ({ task: 'M4', args: { design: path, seed: s, opts: half } })));
const m4ok = m4.filter((r) => r && !r.error && r.passed).length;
// Flight logs of two passing seeds for the film and page (round 3), rerun in this process with the log on.
const loggedPassing = passingSeeds.slice(0, 2);
for (const s of loggedPassing) logOut(`M4-seed${s}`, runM4Seed(design, s, { log: true }));
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
  const h = nominalHalf[m];
  const change = Object.fromEntries(Object.entries(r.measured).filter(([k, v]) => typeof v === 'number' && typeof h.measured[k] === 'number').map(([k, v]) => [k, v ? +((h.measured[k] / v - 1) * 100).toFixed(2) : null]));
  writeGate({ notCountedBecause: [], gate: m, design: name, passed: r.passed && rOk !== false, measured: { ...r.measured, nominal_passed: r.passed, robustness_passed: rOk, robustness: robustNote(m), timestep_check: { dt_half_s: DT / 2, passed_at_half_step: h.passed, measured_at_half_step: h.measured, change_pct: change } }, limit: LIMITS[m], evidence: `out/flight/${name}-${m}.json` });
}
const m4dt = (() => {
  const full = halfSeeds.map((s) => m4[s - 1]).filter((r) => r && !r.error);
  const hf = m4half.filter((r) => r && !r.error);
  const w = (arr, k) => Math.max(0, ...arr.map((r) => r.measured[k]));
  return { dt_half_s: DT / 2, seeds_rerun: halfSeeds.length, seeds_rerun_rule: passingSeeds.length ? 'the seeds that passed at the full step' : 'seeds 1-20 (none passed)', passed_at_full: full.filter((r) => r.passed).length, passed_at_half: hf.filter((r) => r.passed).length, worst: Object.fromEntries(['max_roll_deg', 'max_pitch_deg', 'altitude_dev_m'].map((k) => [k, { dt: w(full, k), dt_half: w(hf, k), change_pct: w(full, k) ? (w(hf, k) / w(full, k) - 1) * 100 : null }])) };
})();
writeGate({ notCountedBecause: [], gate: 'M4', design: name, passed: m4ok >= 98 * (seeds / 100) && robustOk('M4') !== false && seeds >= 100, measured: { seeds_passed: m4ok, seeds_run: seeds, worst_H_m: worstH ? +worstH : null, by_H: byH, robustness_passed: robustOk('M4'), robustness: robustNote('M4'), timestep_check: m4dt }, limit: LIMITS.M4, evidence: rel(ev4) });

// ---- M6 mass: BOM completeness and margin ---------------------------------------------------------------------
// round 17b (audit-v24.md item 24): a part whose maker states a mass tolerance, disclosed beside M6 (a note only)
const MASS_TOLERANCE_NOTES = [{ id: 'batt-gensace-2200-3s-35c', text: 'Net Weight(±20g): 143', tolerance_g: 20, source: 'genstattu.com, SKU GEA223S35DGT', read: '2026-11-03', note: 'the pack is flown at 143 g; the maker allows ±20 g, inside the ±5% mass corners' }];
const bom = design.bom;
const missing = bom.parts.filter((p) => !p.source_url || !p.id);
const m6ok = missing.length === 0 && Math.abs(bom.margin_g - 0.15 * (bom.parts_g + bom.structure_g)) < 1e-6 && Math.abs(bom.total_g - design.mass.total_g) < 0.5;
writeGate({ notCountedBecause: [], gate: 'M6', design: name, passed: m6ok, measured: { total_g: design.mass.total_g, parts_g: bom.parts_g, structure_g: bom.structure_g, margin_g: bom.margin_g, margin_frac: 0.15, parts_without_source: missing.map((p) => p.id), structure_items: bom.structure.length, mass_tolerance_notes: MASS_TOLERANCE_NOTES.filter((n) => bom.parts.some((q) => q.id === n.id)) }, limit: { margin_frac: 0.15, every_part_cited: true }, evidence: rel(path) });

// Round 6: the motor bearing nearest the first pinion, with the outboard bearing as the other support: information only
// (its maker is not named, so the lowest C0 read for its published sizes is shown, not credited).
function motorBearingInfo(d, crankT) {
  const u = (d.structure.joints_unchecked ?? []).find((q) => q.name === 'motor shaft' && Number.isFinite(q.motor_bearing_force_per_crank_Nm));
  if (!u) return null;
  const P = u.motor_bearing_force_per_crank_Nm * crankT;
  return { load_N: P, lowest_same_size_C0_N: 276, C0_over_P0: 276 / P, basis: 'NMB L-1150ZZ (5 x 11 x 5 mm, the smaller of the two published Cobra sizes) Cor 276 N; not the motor maker\'s rating' };
}
// ---- M7 structure at the worst load seen in M1-M3, M5 and the M4 seeds that pass (SPEC ruling) ----------------
const population = (nom, m4runs) => [nom.M1, nom.M2, nom.M3, nom.M5, ...m4runs.filter((r) => r && !r.error && r.passed)];
const PEAK_KEYS = ['rootBend', 'elbowBend', 'crankTorque', 'crankDrive', 'crankBack', 'couplerForce', 'twistTorque', 'tailTorque', 'loadFactor', 'shoulderTorque', 'motorI', 'battI', 'servoI', 'becI', 'regI', 'crankRate', 'slotPair', 'motorIDrive', 'motorIBrake'];
const worstOf = (runs) => Object.fromEntries(PEAK_KEYS.map((k) => [k, Math.max(0, ...runs.map((r) => r.peaks[k] ?? 0))]));
// Round 5: worst load on every support of design.structure.joints over a population (radial, axial, ISO 76 P0).
const worstSupports = (runs) => {
  const n = Math.max(0, ...runs.map((r) => r.peaks.supports?.length ?? 0));
  return Array.from({ length: n }, (_, k) => Object.fromEntries(['Fr', 'Fa', 'P0'].map((q) => [q, Math.max(0, ...runs.map((r) => r.peaks.supports?.[k]?.[q] ?? 0))])));
};
const m4passRuns = m4.filter((r) => r && !r.error && r.passed);
const popRuns = population(nominal, m4);
// Half-step population: the same scenarios and the same seeds (those that passed at the full step).
const m4halfBySeed = new Map(halfSeeds.map((s, i) => [s, m4half[i]]));
const popHalf = [nominalHalf.M1, nominalHalf.M2, nominalHalf.M3, nominalHalf.M5, ...(passingSeeds.length ? passingSeeds.map((s) => m4halfBySeed.get(s)).filter((r) => r && !r.error) : [])];
const W1 = worstOf(popRuns);
const W2 = worstOf(popHalf);
const dtCheck = Object.fromEntries(PEAK_KEYS.map((k) => [k, { dt: W1[k], dt_half: W2[k], change_pct: W1[k] ? (W2[k] / W1[k] - 1) * 100 : 0 }]));
const M7_KEYS = ['rootBend', 'elbowBend', 'crankTorque', 'couplerForce', 'twistTorque', 'tailTorque', 'loadFactor'];
// Bearings (round 5): every support's peak P0 (rolling) or radial load (bushing, rod end) at half the step, too.
const S1 = worstSupports(popRuns), S2 = worstSupports(popHalf);
// Round 6: the loaded members' sections follow the joints' supports (flight.js), so they share the step check.
const supKeys = [...(design.structure.joints ?? []).flatMap((j) => j.supports.map((s) => `${j.name}/${s.name}`)), ...(design.structure.members ?? []).flatMap((m) => m.sections.map((s) => `${m.name}/${s.name}`))];
const supDt = supKeys.map((key, k) => ({ key, P0_dt: S1[k]?.P0 ?? 0, P0_dt_half: S2[k]?.P0 ?? 0, change_pct: S1[k]?.P0 ? ((S2[k]?.P0 ?? 0) / S1[k].P0 - 1) * 100 : 0 }));
const loadsConverged = M7_KEYS.every((k) => Math.abs(dtCheck[k].change_pct) < 10) && supDt.every((q) => Math.abs(q.change_pct) < 10);
const worst = (k) => W1[k];
const st = design.structure;
const sIn = st.spar_inner, sOut = st.spar_outer;
const sigIn = (worst('rootBend') * sIn.c) / sIn.I / 1e6;
const sigOut = (worst('elbowBend') * sOut.c) / sOut.I / 1e6;
const fosIn = sIn.allowable_MPa / sigIn;
const fosOut = sOut.allowable_MPa / sigOut;
const gears = st.gear_stages;
const crankT = worst('crankTorque');
const G2 = gears[gears.length - 1];
// Round 9 (SPEC ruling after physics round 8): the ratings are KHK's under the design's stated conditions
// (structure.gear_stages[].rating_check, src/design/gearing.js). Those conditions must hold in flight: the flown peak
// crank rate within the rating crank frequency, and a reversing load declared if the wing back-drives the train.
const grc = design.drive?.gear_rating_conditions ?? null;
const ratingConditions = grc ? { crank_Hz: grc.crank_Hz, peak_crank_rate_rad_s: worst('crankRate'), speed_within: worst('crankRate') <= 2 * Math.PI * grc.crank_Hz, reversing_declared: grc.reversing === true, back_driven_Nm: worst('crankBack'), direction_consistent: grc.reversing === true || worst('crankBack') === 0 } : null;
const ratingConditionsOk = !ratingConditions || (ratingConditions.speed_within && ratingConditions.direction_consistent);
const gearOk = ratingConditionsOk && gears.every((g, i) => {
  const below = gears.slice(i + 1).reduce((r, q) => r * q.ratio, 1);
  const tGear = crankT / below;
  return tGear <= g.gear_allowable_Nm && tGear / g.ratio <= g.pinion_allowable_Nm;
});
const gearUse = gears.map((g, i) => {
  const below = gears.slice(i + 1).reduce((r, q) => r * q.ratio, 1);
  return { gear: g.gear, torque_Nm: crankT / below, allowable_Nm: g.gear_allowable_Nm, pinion: g.pinion, pinion_torque_Nm: crankT / below / g.ratio, pinion_allowable_Nm: g.pinion_allowable_Nm, mesh_materials: mixedMeshCheck(g, crankT / below / g.ratio) };
});
// Round 5: mixed-material meshes (SPEC ruling) must be judged by Hertz on each member's published surface allowable.
const meshesOk = gearUse.every((g) => g.mesh_materials.ok);
const cp = st.coupler;
const Fc = worst('couplerForce');
const cpc = couplerCheck(cp, Fc);
const Pcr = cpc.buckling_N;
const sigRod = cpc.stress_MPa;
const fosRodBuckle = cpc.fos_buckling;
const fosRodStress = cpc.fos_stress;
const twistT = worst('twistTorque');
const tailT = worst('tailTorque');
// Round 5: the stroke servo holds the coupler pins in their slots (a slotted rocker): the coupler force's component along
// each rocker, both through one yoke, at the servo horn. A fixed stroke has no slot and no stroke servo.
const hold = strokeServoHold(design, worst('slotPair'));
const servoOk = twistT <= design.servos.twist.stall_Nm && tailT <= design.servos.tail.stall_Nm && hold.ok;
// Round 5: every bearing, bushing and rod end against its published static rating (SPEC ruling "Bearings are linkage").
const hasBearings = design.bom.parts.some((b) => /^(bearing|bushing|rod end)/.test(b.role));
const nJointSup = (design.structure.joints ?? []).reduce((a, j) => a + j.supports.length, 0);
const bj = design.structure.joints ? checkJoints(design.structure.joints, S1.slice(0, nJointSup), { crankRate: worst('crankRate') }) : null;
const bearingsOk = bj ? bj.ok : !hasBearings;
// Round 6 (SPEC ruling after round 5): every shaft, pin, seat, end fitting, adapter and key against published strength.
const mb = design.structure.members ? checkMembers(design.structure.members, memberPeaks(design.structure.members, S1, nJointSup)) : null;
const membersOk = mb ? mb.ok : null;
// Round 4: the twist torque rod (elbow servo to the hand wing) carries the hand wing's twist torque in torsion.
const tr = st.twist_rod ? { id: st.twist_rod.id, torque_Nm: twistT, ...torsionCheck(st.twist_rod, twistT) } : null;
const fosMin = Math.min(fosIn, fosOut, fosRodBuckle, fosRodStress, ...(tr ? [tr.fos ?? 0] : []), ...(bj ? [bj.fos_min ?? 0] : []), ...(mb ? [mb.fos_min ?? 0] : []));
// Round 17: a pass resting on an RC-practice joint outside the ruling's scope (only a gear, pulley or coupling held to
// its shaft) is not counted; v24's crank-web LOCTITE 638 bond was the case
const m7NotCounted = (mb?.members ?? []).filter((r) => r.check === 'rc practice' && r.kind !== 'gear fixing').map((r) => `${r.name}: RC practice outside the ruling's scope (a gear, pulley or coupling on its shaft only)`);
// Round 18 (SPEC ruling "Maker's part", operator's decision 2026-11-03): each maker's part with its peak load and its
// condition (a servo arm: its servo's rule at the supply's row, checked in its member; the motor's bearings and threads:
// Scorpion's figures, the current from this population's 2 s RMS, as M8 holds it). Valid only while every one holds.
const motorRmsPop = Math.max(0, ...popRuns.map((r) => r.rms?.motorRmsMax ?? 0));
// Round 19 (decision 5): the motor's continuous rating is stated for 60 s at 100% throttle with forced cooling air; the
// sustained figure it is held to, derated, is the endurance cruise's (M2's) RMS over its whole flight. A condition no
// check can hold (the motor's temperature: unbounded) leaves the pass not counted.
const m2 = nominal.M2;
const { parts: makersParts, ok: makersOk, notCounted: makersNotCounted } = flownMakersParts(mb?.members ?? [], { rms2sMax: motorRmsPop, sustainedRms: m2?.rms?.n ? Math.sqrt(m2.rms.motorI2 / m2.rms.n) : NaN, endurance_min: m2?.measured?.endurance_min ?? null, design });
writeGate({
  notCountedBecause: [...m7NotCounted, ...makersNotCounted],
  gate: 'M7', design: name, passed: fosMin >= 1.5 && gearOk && meshesOk && servoOk && bearingsOk && membersOk !== false && makersOk && seeds >= 100 && loadsConverged && m4passRuns.length > 0,
  measured: {
    spar_inner: { id: sIn.id, moment_Nm: worst('rootBend'), stress_MPa: sigIn, allowable_MPa: sIn.allowable_MPa, fos: fosIn },
    spar_outer: { id: sOut.id, moment_Nm: worst('elbowBend'), stress_MPa: sigOut, allowable_MPa: sOut.allowable_MPa, fos: fosOut },
    coupler: { id: cp.id, kind: cp.kind ?? null, force_N: Fc, buckling_N: Pcr, fos_buckling: fosRodBuckle, stress_MPa: sigRod, fos_stress: fosRodStress, allowable_MPa: cpc.allowable_MPa, allowable_basis: cp.allowable_basis ?? null, E_GPa: cp.E_GPa, section: cpc.section },
    crank_torque_Nm: crankT, crank_torque_split: { drive_pushing_Nm: worst('crankDrive'), back_driven_Nm: worst('crankBack'), note: 'drive pushing: load torque along the crank rotation; back-driven: air and wing inertia driving the crank and motor' }, gears: gearUse, gears_within_rating: gearOk, gear_rating_conditions: ratingConditions, gear_rating_checks: gears.map((g) => g.rating_check ?? null), mixed_material_meshes_ok: meshesOk,
    twist_torque_rod: tr,
    twist_servo: { torque_Nm: twistT, rated_stall_Nm: design.servos.twist.stall_Nm }, tail_servo: { torque_Nm: tailT, rated_stall_Nm: design.servos.tail.stall_Nm },
    stroke_servo_hold: hold,
    bearings: bj ? { passed: bj.ok, fos_min: bj.fos_min, worst: bj.worst, peak_crank_rate_rad_s: worst('crankRate'), joints: bj.joints, unchecked: design.structure.joints_unchecked ?? [], motor_bearing_information: motorBearingInfo(design, crankT), layout: design.structure.joints_layout ?? null, rule: 'SPEC ruling after round 4: ISO 76 P0 with the maker X0/Y0 and C0/P0 >= 1.5; bushings by projected pressure against the published static surface pressure with FoS 1.5 (flange ring for axial load) and their published surface speed; loads are the worst over the M7 population at each support, combined in the flight model at every step' } : { passed: bearingsOk, note: hasBearings ? 'the design file has no load path (structure.joints): its bearings cannot be checked' : 'no bearings in the BOM' },
    // round 17 (SPEC Function completeness): what is rated only by being held to another part's published figure (the
    // maker's servo arm; round 18: a maker's part, listed with its peak below), and the chains' stated assumptions, disclosed
    disclosed_unrated: (design.structure.members ?? []).filter((m) => /no (load )?rating[^.]*published|held to the servo/i.test(`${m.dims?.rating_basis ?? ''} ${m.note ?? ''}`)).map((m) => ({ name: m.name, basis: m.dims?.rating_basis ?? m.note })),
    // round 18: "unrated, maker's part", beside the RC-practice joints (members.rc_practice), each with its peak load
    makers_parts: makersParts, makers_parts_passed: makersOk, makers_part_rule: 'SPEC ruling "Maker\'s part" (operator\'s decision 2026-11-03): an interface inside a bought part, or between a bought part and its maker\'s own fittings, that its maker publishes no figure for; reported as unrated, maker\'s part, with its peak load; valid only while the bought part stays within every figure its maker publishes (a maker\'s arm under its own servo\'s rule at the published row for its real supply voltage)',
    chain_assumptions: [...new Set((design.structure.clearance?.chains?.chains ?? []).flatMap((c) => c.assumptions ?? []))],
    members: mb ? { passed: mb.ok, fos_min: mb.fos_min, worst: mb.worst, rc_practice: mb.members.filter((r) => r.check === 'rc practice').map((r) => ({ name: r.name, status: r.status, method: r.method, peak_T_Nm: r.peak_T_Nm, published_Nm: r.published_Nm, within_published: r.within_published })), rc_practice_rule: 'SPEC ruling "RC practice for joints no one can rate": shaft-to-hub fixings no maker or standard rates are reported as unrated, RC practice, with their peak torque; any published figure binds at full value', members: mb.members, unchecked: design.structure.members_unchecked ?? [], rule: 'SPEC ruling after round 5: every shaft, pin, plate bearing seat, end fitting and bore adapter against published material strength with FoS 1.5 (src/physics/members.js); loads combined in the flight model at every step, worst over the M7 population' } : { passed: null, note: 'the design file states no loaded members (structure.members)' },
    peak_load_factor_at_cg: worst('loadFactor'), fos_min: fosMin,
    population: { scenarios: ['M1', 'M2', 'M3', 'M5'], m4_seeds_passing: m4passRuns.length, m4_seeds_run: seeds },
    timestep_check: { dt_s: DT, dt_half_s: DT / 2, peaks: dtCheck, supports_P0: supDt, converged_within_10pct: loadsConverged },
    note: 'Worst load over M1, M2, M3, M5 and the M4 seeds that pass (SPEC ruling), load factor at the system mass centre. Gear and servo torques against their published or Lewis ratings; spars and coupler against published strength with FoS 1.5. Peaks rerun at half the time step on the same flights and seeds.',
  },
  limit: { factor_of_safety: 1.5, gear_torque: 'within catalogue allowable', servo_torque: 'within published stall-style torque', bearings: 'static C0/P0 >= 1.5 (ISO 76), bushings published static surface pressure / 1.5' },
  evidence: rel(ev4),
});

// ---- M8 electrical through M1-M5 -----------------------------------------------------------------------------
const all = popRuns; // M1-M3, M5 and the M4 seeds that pass, as for M7
const rmsHalf = { motor: Math.max(0, ...popHalf.map((r) => r.rms.motorRmsMax)), batt: Math.max(0, ...popHalf.map((r) => r.rms.battRmsMax)) };
const motorRms = Math.max(...all.map((r) => r.rms.motorRmsMax));
const battRms = Math.max(...all.map((r) => r.rms.battRmsMax));
const servosOnBattery = design.electrical?.servo_source === 'battery';
// Round 12: servos behind a step-down regulator: their peak current against its published continuous rating, and the
// FC's BEC carries the avionics only.
const servoReg = design.electrical?.servo_source === 'regulator' ? design.electrical.servo_supply : null;
const regPeak = servoReg ? Math.max(...all.map((r) => r.peaks.regI ?? 0)) : null;
// The BEC's load: servos and avionics (rounds 1-3), or the avionics only when the HV servos run off the pack (round 4).
const servo5 = Math.max(...all.map((r) => (servosOnBattery || servoReg ? r.peaks.becI ?? 0 : r.peaks.servoI)));
const m8 = {
  motor_rms_2s_max_A: motorRms, motor_continuous_A: design.motor.I_cont, motor_rule: design.motor.I_cont_rule, motor_peak_A: Math.max(...all.map((r) => r.peaks.motorI)),
  // Round 5: what the drive clamp and the braking limit did, by sign (the clamp is the governor's, gate-independent).
  motor_peak_driving_A: worst('motorIDrive'), motor_peak_braking_A: worst('motorIBrake'), drive_clamp_A: design.drive.governor?.clamp_A ?? design.drive.escCurrentClamp_A ?? null, clamp_location: design.drive.governor?.location ?? (Number.isFinite(design.drive.escCurrentClamp_A) ? 'ESC (round 4 model)' : null), braking_limit_A: design.motor.regenLimit_A ?? null,
  esc_rms_2s_max_A: motorRms, esc_continuous_A: design.esc.I_cont,
  battery_rms_2s_max_A: battRms, battery_continuous_A: design.battery.I_cont, battery_peak_A: Math.max(...all.map((r) => r.peaks.battI)),
  servo_bus_peak_A: servo5, bec_rating_A: design.esc.bec_A, bec_carries: servosOnBattery ? 'avionics only (servos on the pack, in the battery current)' : servoReg ? 'avionics only (servos on their regulator)' : 'servos and avionics',
  ...(servoReg ? { servo_regulator: { part: servoReg.part, servo_peak_A: regPeak, continuous_A: servoReg.I_cont_A, within: regPeak <= servoReg.I_cont_A, rule: 'peak servo current (stall current x torque / stall torque + idle) against the published continuous output current' } } : {}), servo_and_avionics_peak_A: Math.max(...all.map((r) => r.peaks.servoI)), servo_model: 'stall current x torque/stall torque + published idle',
  population: { scenarios: ['M1', 'M2', 'M3', 'M5'], m4_seeds_passing: m4passRuns.length },
  timestep_check: { motor_rms_dt_half_A: rmsHalf.motor, battery_rms_dt_half_A: rmsHalf.batt, motor_change_pct: (rmsHalf.motor / motorRms - 1) * 100, battery_change_pct: (rmsHalf.batt / battRms - 1) * 100, servo_peak_change_pct: dtCheck.servoI.change_pct },
};
const m8ok = motorRms <= design.motor.I_cont && motorRms <= design.esc.I_cont && battRms <= (design.battery.I_cont ?? 0) && servo5 <= (design.esc.bec_A ?? 0) && (!servoReg || regPeak <= servoReg.I_cont_A) && m4passRuns.length > 0;
writeGate({ notCountedBecause: [], gate: 'M8', design: name, passed: m8ok && seeds >= 100, measured: m8, limit: { continuous: 'rms over 2 s windows within published continuous ratings (SPEC rule where none published)' }, evidence: rel(ev4) });

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
