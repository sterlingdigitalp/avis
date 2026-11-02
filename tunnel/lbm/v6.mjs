#!/usr/bin/env node
// Gate V6: GPU LBM. Runs every section under tools/run-guarded.py --gpu in holds under 10 minutes,
// resuming from checkpoints, then evaluates against limits fixed here and writes
// out/verification/V6.json. This is the only script that writes that file.
//
//   node tunnel/lbm/v6.mjs                   run whatever is not finished, then evaluate
//   node tunnel/lbm/v6.mjs --force           discard section results and start again
//   node tunnel/lbm/v6.mjs --only agree,mass run those sections, then evaluate what exists
//   node tunnel/lbm/v6.mjs --evaluate        no GPU, just evaluate existing section results
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { theodorsenLift } from './src/theodorsen.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '../..');
const SECT = path.join(ROOT, 'out/lbm/v6');
const argv = process.argv.slice(2);
const only = argv.includes('--only') ? argv[argv.indexOf('--only') + 1].split(',') : null;

const ALPHA3 = (3 * Math.PI) / 180;
const plate = (key, o) => ({ job: 'runPlate', key, args: { name: `v6-${key}`, ...o } });
const SECTIONS = {
  agree: [
    { job: 'agree', key: 'thin-bouzidi', args: { variant: 'thin', bouzidi: true, steps: 600, every: 200 }, needsSeconds: 300 },
    { job: 'agree', key: 'thin-bounceback', args: { variant: 'thin', bouzidi: false, steps: 600, every: 200 }, needsSeconds: 300 },
    { job: 'agree', key: 'slab-ladd', args: { variant: 'slab', bouzidi: false, steps: 600, every: 200 }, needsSeconds: 300 },
    { job: 'agree', key: 'half-model', args: { variant: 'half', bouzidi: true, steps: 600, every: 200 }, needsSeconds: 300 },
  ],
  restart: [
    { job: 'restartCheck', key: 'thin', args: { kind: 'thin', name: 'v6-restart-thin' } },
    { job: 'restartCheck', key: 'slab', args: { kind: 'slab', name: 'v6-restart-slab' } },
  ],
  mass: [
    { job: 'massBox', key: 'control-no-body', args: { withBody: false, name: 'v6-m0' } },
    { job: 'massBox', key: 'control-still-plate', args: { kind: 'thin', bouzidi: false, h0: 0, alphaMean: 0.3, name: 'v6-m0b' } },
    { job: 'massBox', key: 'thin-bouzidi-raw', args: { kind: 'thin', bouzidi: true, name: 'v6-m1' } },
    { job: 'massBox', key: 'thin-bounceback-raw', args: { kind: 'thin', bouzidi: false, name: 'v6-m2' } },
    { job: 'massBox', key: 'slab-ladd-raw', args: { kind: 'slab', bouzidi: false, name: 'v6-m3' } },
    { job: 'massBox', key: 'thin-bouzidi-corrected', args: { kind: 'thin', bouzidi: true, massFix: true, name: 'v6-m4' } },
    plate('open-tunnel-30-cycles', { c: 40, Re: 1000, k: 0.5, h0c: 0.05, cycles: 30, massEveryCycle: true }),
  ],
  theodorsen: [
    ...[20, 40, 80, 160].map((c) => plate(`plunge-Re1000-c${c}`, { c, Re: 1000, k: 0.5, h0c: 0.05 })),
    ...[20, 40, 80, 160].map((c) => plate(`pitch-Re1000-c${c}`, { c, Re: 1000, k: 0.5, alpha0: ALPHA3, axis: -0.25 })),
    ...[40, 80, 113, 160].map((c) => plate(`plunge-Re5000-c${c}`, { c, Re: 5000, k: 0.5, h0c: 0.05 })),
    ...[40, 80, 113, 160].map((c) => plate(`pitch-Re5000-c${c}`, { c, Re: 5000, k: 0.5, alpha0: ALPHA3, axis: -0.25 })),
    plate('plunge-Re1000-c80-k0.25', { c: 80, Re: 1000, k: 0.25, h0c: 0.1, cycles: 5 }),
    plate('pitch-Re1000-c80-k0.25', { c: 80, Re: 1000, k: 0.25, alpha0: ALPHA3, axis: -0.25, cycles: 5 }),
    plate('plunge-Re1000-c80-k1', { c: 80, Re: 1000, k: 1, h0c: 0.025, cycles: 8 }),
    plate('plunge-Re1000-c40-wide', { c: 40, Re: 1000, k: 0.5, h0c: 0.05, nxc: 40, nyc: 32, xPlate: 28 }),
  ],
  represent: [
    ...[20, 40, 80].flatMap((c) => [
      plate(`bounceback-c${c}`, { c, Re: 1000, k: 0.5, h0c: 0.05, bouzidi: false }),
      plate(`slab2-c${c}`, { c, Re: 1000, k: 0.5, h0c: 0.05, kind: 'slab', thickness: 2, bouzidi: false }),
      plate(`slab4-c${c}`, { c, Re: 1000, k: 0.5, h0c: 0.05, kind: 'slab', thickness: 4, bouzidi: false }),
    ]),
    plate('relink2-c80', { c: 80, Re: 1000, k: 0.5, h0c: 0.05, relink: 2 }),
    plate('relink4-c80', { c: 80, Re: 1000, k: 0.5, h0c: 0.05, relink: 4 }),
    plate('relink8-c80', { c: 80, Re: 1000, k: 0.5, h0c: 0.05, relink: 8 }),
  ],
  // Franck & Breuer (2017), arXiv:1703.10906v3, table 3. Re = 40000, St = 0.08, UT/U = 0.12, span 0.2c.
  franck: [40, 80].flatMap((c) => [
    { job: 'runInclined', key: `static-a5-c${c}`, args: { c, alphaDeg: 5, name: `v6-fb-s5-${c}` } },
    { job: 'runInclined', key: `heave-a5-c${c}`, args: { c, alphaDeg: 5, St: 0.08, UT: 0.12, name: `v6-fb-h5-${c}` } },
    { job: 'runInclined', key: `static-a13-c${c}`, args: { c, alphaDeg: 13, name: `v6-fb-s13-${c}` } },
    { job: 'runInclined', key: `heave-a13-c${c}`, args: { c, alphaDeg: 13, St: 0.08, UT: 0.12, name: `v6-fb-h13-${c}` } },
  ]),
};

function runSection(name) {
  const out = path.join(SECT, `${name}.json`);
  for (let hold = 0; hold < 60; hold++) {
    const r = spawnSync('python3', [path.join(ROOT, 'tools/run-guarded.py'), '--timeout', '590', '--gpu', '--log', path.join(SECT, `${name}.log`), '--',
      'node', path.join(HERE, 'tools/job.mjs'), out, JSON.stringify(SECTIONS[name])], { env: { ...process.env, JOB_SECONDS: '520' }, stdio: 'inherit' });
    console.log(`section ${name}: hold ${hold + 1} exit ${r.status}`);
    if (r.status === 0) return true;
    if (r.status !== 75) return false;
    // Let anyone else queued on the GPU lock in: run-guarded polls it every 2 s.
    spawnSync('sleep', ['5']);
  }
  return false;
}

if (!argv.includes('--evaluate')) {
  fs.mkdirSync(SECT, { recursive: true });
  for (const name of Object.keys(SECTIONS)) {
    if (only && !only.includes(name)) continue;
    const out = path.join(SECT, `${name}.json`);
    if (argv.includes('--force')) fs.rmSync(out, { force: true });
    if (!runSection(name)) console.error(`section ${name} did not finish`);
  }
}

// ---- evaluation ------------------------------------------------------------------------------
const read = (n) => { try { return JSON.parse(fs.readFileSync(path.join(SECT, `${n}.json`), 'utf8')); } catch { return {}; } };
const R = Object.fromEntries(Object.keys(SECTIONS).map((n) => [n, read(n)]));

// Limits, fixed before the final runs. Round-off here means single precision on populations whose
// magnitude is at most about 0.05, for which one unit in the last place is 3.7e-9.
const LIMIT = {
  gpu_vs_cpu_max_population_difference: 1e-6,
  gpu_vs_cpu_force_relative: 1e-4,
  restart_max_population_difference: 0,
  mass_corrected_closed_box_abs_mean_drho_per_cycle: 1e-7,
  mass_raw_thin_closed_box_abs_mean_drho_per_cycle: 1e-4,
  mass_open_tunnel_abs_mean_drho: 1e-3,
  mass_budget_identity_relative: 1e-3,
  theodorsen_amplitude_pct: 25,
  theodorsen_phase_pct_of_theory: 25,
  grid_min_resolutions: 3,
};

const measured = {}, failures = [], notes = [];
// 1. GPU vs CPU
const ag = Object.entries(R.agree).map(([k, v]) => ({ case: k, max_population_diff: v.last?.maxAbs, mean_population_diff: v.last?.meanAbs, force_rel: v.last?.forceRelErr, moment_rel: v.last?.momentRelErr, cpu_f32_vs_f64: v.last?.cpuF32vsF64, non_finite: v.last?.nonFinite, swept_events: v.sweptEvents, mask_edits: v.maskEdits, links: v.links, steps: v.steps, grid: [v.nx, v.ny, v.nz], webgpu_errors: v.errors }));
measured.gpu_vs_cpu = ag;
measured.adapter = Object.values(R.agree)[0]?.adapterInfo;
if (ag.length < SECTIONS.agree.length) failures.push('gpu_vs_cpu: section incomplete');
for (const a of ag) {
  if (!(a.max_population_diff <= LIMIT.gpu_vs_cpu_max_population_difference)) failures.push(`gpu_vs_cpu ${a.case}: max diff ${a.max_population_diff}`);
  if (!(a.force_rel <= LIMIT.gpu_vs_cpu_force_relative)) failures.push(`gpu_vs_cpu ${a.case}: force ${a.force_rel}`);
  if (a.non_finite) failures.push(`gpu_vs_cpu ${a.case}: non-finite`);
  if (a.webgpu_errors?.length) failures.push(`gpu_vs_cpu ${a.case}: WebGPU errors`);
}
// Restart
measured.restart = Object.entries(R.restart).map(([k, v]) => ({ case: k, max_population_diff: v.maxAbs, differing_slots: v.differing, total_slots: v.total }));
if (measured.restart.length < 2) failures.push('restart: section incomplete');
for (const r of measured.restart) if (!(r.max_population_diff <= LIMIT.restart_max_population_difference)) failures.push(`restart ${r.case}: ${r.max_population_diff}`);
// 2. Mass
const ms = {};
for (const [k, v] of Object.entries(R.mass)) {
  if (v.meanDrhoByCycle) {
    const a = v.meanDrhoByCycle;
    ms[k] = { cycles: a.length, mean_drho_at_cycle_5: a[4], mean_drho_end: a[a.length - 1], max_abs_mean_drho: Math.max(...a.map(Math.abs)) };
  } else {
    const b = v.budgetCheck;
    ms[k] = { cycles: v.cycles, mean_drho_end: v.meanDrhoEnd, per_cycle_slope: v.slopePerCycle, peak_body_u: v.peakBodyU, blockage: v.blockage, mean_u: v.U, every_5th_cycle: v.series,
      budget_identity: b ? { mass_change: b.massChange, link_and_refill_budget: b.budget, correction_applied: b.fixApplied, residual_relative: Math.abs(b.massChange - b.budget - b.fixApplied) / Math.max(1e-12, Math.abs(b.budget)) } : null };
  }
}
measured.mass = ms;
const pc = (k) => Math.abs(ms[k]?.per_cycle_slope ?? NaN);
if (!(pc('thin-bouzidi-corrected') <= LIMIT.mass_corrected_closed_box_abs_mean_drho_per_cycle)) failures.push(`mass corrected ${pc('thin-bouzidi-corrected')}`);
if (!(pc('thin-bouzidi-raw') <= LIMIT.mass_raw_thin_closed_box_abs_mean_drho_per_cycle)) failures.push(`mass raw thin ${pc('thin-bouzidi-raw')}`);
for (const k of ['thin-bouzidi-raw', 'thin-bounceback-raw', 'slab-ladd-raw']) {
  const r = ms[k]?.budget_identity?.residual_relative;
  if (!(r <= LIMIT.mass_budget_identity_relative)) failures.push(`mass budget identity ${k}: ${r}`);
}
if (!(ms['open-tunnel-30-cycles']?.max_abs_mean_drho <= LIMIT.mass_open_tunnel_abs_mean_drho)) failures.push(`mass open tunnel ${ms['open-tunnel-30-cycles']?.max_abs_mean_drho}`);

// 3. Theodorsen and 4. grid study
const phasePct = (r) => (100 * Math.abs(r.phaseErrDeg)) / Math.abs(r.theo.phaseDeg);
const row = (key, r) => {
  const th = [r.theo.amp * Math.cos((r.theo.phaseDeg * Math.PI) / 180), r.theo.amp * Math.sin((r.theo.phaseDeg * Math.PI) / 180)];
  const lb = [r.cl.amp * Math.cos((r.cl.phaseDeg * Math.PI) / 180), r.cl.amp * Math.sin((r.cl.phaseDeg * Math.PI) / 180)];
  return { case: key, c_cells: r.c, Re: r.Re, k: r.k, tau0: +r.tau0.toFixed(5), cells: r.cells, steps: r.steps, seconds: +r.seconds.toFixed(1), mlups: Math.round(r.mlups),
    amp: +r.cl.amp.toFixed(5), amp_previous_cycles: +r.cl.prevAmp.toFixed(5), theory_amp: +r.theo.amp.toFixed(5), amp_err_pct: +r.ampErrPct.toFixed(2),
    phase_deg: +r.cl.phaseDeg.toFixed(2), theory_phase_deg: +r.theo.phaseDeg.toFixed(2), phase_err_deg: +r.phaseErrDeg.toFixed(2), phase_err_pct_of_theory: +phasePct(r).toFixed(1),
    complex_err_pct: +((100 * Math.hypot(lb[0] - th[0], lb[1] - th[1])) / r.theo.amp).toFixed(2), mean_cd: +r.meanCd.toFixed(4), links: r.links };
};
const T = R.theodorsen;
// A run has diverged if its lift is not finite, its drag went negative, or its lift amplitude is
// more than three times the inviscid value. Diverged runs are reported, never used in a study.
const diverged = (r) => !Number.isFinite(r.cl.amp) || !(r.meanCd > 0) || r.cl.amp > 3 * r.theo.amp;
measured.theodorsen = Object.entries(T).filter(([, r]) => r.cl).map(([k, r]) => ({ ...row(k, r), diverged: diverged(r) }));
const order = (vals, ratio) => {
  // Three grids refined by a constant ratio r: p = ln(d1/d2)/ln(r) from the successive differences.
  const d = [];
  for (let i = 1; i < vals.length; i++) d.push(vals[i] - vals[i - 1]);
  const n = d.length;
  const q = n >= 2 ? d[n - 2] / d[n - 1] : NaN;
  const p = q > 0 ? Math.log(q) / Math.log(ratio) : NaN;
  const extrap = q > 0 && Number.isFinite(p) ? vals[vals.length - 1] + d[n - 1] / (ratio ** p - 1) : NaN;
  return { values: vals, differences: d, difference_ratio: q, refinement_ratio: ratio, observed_order: p, richardson_limit: extrap, monotone: d.every((x) => Math.sign(x) === Math.sign(d[0])) };
};
const study = (prefix, cs) => {
  const all = cs.map((c) => ({ c, r: T[`${prefix}-c${c}`] }));
  const missing = all.filter((x) => !x.r?.cl).map((x) => x.c);
  const div = all.filter((x) => x.r?.cl && diverged(x.r)).map((x) => x.c);
  const ok = all.filter((x) => x.r?.cl && !diverged(x.r));
  const use = ok.slice(-3);
  const out = { c_cells: cs, missing, diverged: div, used_for_order: use.map((x) => x.c), complete: missing.length === 0 && use.length >= 3 };
  if (use.length < 3) return out;
  const ratio = Math.pow(use[2].c / use[0].c, 1 / 2);
  const rows = use.map((x) => x.r);
  return {
    ...out,
    amp: order(rows.map((r) => r.cl.amp), ratio), phase_deg: order(rows.map((r) => r.cl.phaseDeg), ratio), mean_cd: order(rows.map((r) => r.meanCd), ratio),
    all_amp: ok.map((x) => [x.c, +x.r.cl.amp.toFixed(5)]),
    amp_err_pct_vs_theodorsen: ok.map((x) => [x.c, +x.r.ampErrPct.toFixed(2)]), phase_err_deg_vs_theodorsen: ok.map((x) => [x.c, +x.r.phaseErrDeg.toFixed(2)]),
  };
};
measured.grid_study = {
  'plunge-Re1000': study('plunge-Re1000', [20, 40, 80, 160]),
  'pitch-Re1000': study('pitch-Re1000', [20, 40, 80, 160]),
  'plunge-Re5000': study('plunge-Re5000', [40, 80, 113, 160]),
  'pitch-Re5000': study('pitch-Re5000', [40, 80, 113, 160]),
};
for (const [k, s] of Object.entries(measured.grid_study)) {
  if (s.diverged.length) notes.push(`grid study ${k}: diverged at chord ${s.diverged.join(', ')} cells (reported, not used)`);
  if (!s.complete) { failures.push(`grid study ${k}: fewer than ${LIMIT.grid_min_resolutions} stable resolutions, or runs missing (${s.missing.join(', ')})`); continue; }
  if (!(s.amp.difference_ratio > 1)) failures.push(`grid study ${k}: lift amplitude differences do not shrink (ratio ${s.amp.difference_ratio})`);
}
for (const key of ['plunge-Re1000-c160', 'pitch-Re1000-c160', 'plunge-Re5000-c160', 'pitch-Re5000-c160']) {
  const r = T[key];
  if (!r?.cl) { failures.push(`theodorsen ${key}: missing`); continue; }
  if (diverged(r)) { failures.push(`theodorsen ${key}: diverged`); continue; }
  if (!(Math.abs(r.ampErrPct) <= LIMIT.theodorsen_amplitude_pct)) failures.push(`theodorsen ${key}: amplitude ${r.ampErrPct}%`);
  if (!(phasePct(r) <= LIMIT.theodorsen_phase_pct_of_theory)) failures.push(`theodorsen ${key}: phase ${r.phaseErrDeg} deg`);
  if (Math.abs(r.cl.amp - r.cl.prevAmp) / r.cl.amp > 0.03) notes.push(`${key}: amplitude still moving ${(100 * Math.abs(r.cl.amp - r.cl.prevAmp) / r.cl.amp).toFixed(1)}% between the last two pairs of cycles`);
}
// Wing representation (informational)
measured.wing_representation = Object.entries(R.represent).filter(([, r]) => r.cl).map(([k, r]) => row(k, r));
// Published case: comparison is reported, with a stated limit only of "reported"; LES is itself a model.
const FB = { static: { 5: 0.468, 13: 0.887 }, heave: { 5: 0.494, 13: 1.04 } };
measured.franck_breuer_2017 = Object.entries(R.franck).filter(([, r]) => r.clMean !== undefined).map(([k, r]) => {
  const ref = (k.startsWith('static') ? FB.static : FB.heave)[r.alphaDeg];
  // Table 1 (mesh 2, the mesh used for every case) gives static Cd at 5 degrees only.
  const refCd = k.startsWith('static') && r.alphaDeg === 5 ? 0.046 : null;
  return { case: k, c_cells: r.c, tau0: +r.tau0.toFixed(5), cells: r.cells, steps: r.steps, seconds: +r.seconds.toFixed(1), cl: +r.clMean.toFixed(4), reference_les_cl: ref, cl_err_pct: +((100 * (r.clMean - ref)) / ref).toFixed(1), cd: +r.cdMean.toFixed(4), reference_les_cd: refCd, cd_err_pct: refCd ? +((100 * (r.cdMean - refCd)) / refCd).toFixed(1) : null, cl_max: +r.clMax.toFixed(3), per_cycle_cl: r.perCycleCl?.map((v) => +v.toFixed(4)) };
});
// Independent re-evaluation of theory at the headline case, from src/theodorsen.mjs directly.
measured.theory_check = theodorsenLift({ k: 0.5, h0: 0.05 });

const passed = failures.length === 0;
const doc = {
  gate: 'V6', design: null, passed, measured,
  limit: LIMIT,
  failures, notes,
  fairness: 'Theodorsen is inviscid, thin, attached and small-amplitude. Comparison range: flat plate, k 0.25-1, effective angle under 3 degrees (h0/c 0.05 at k 0.5, 3 degree pitch), Re 1000 (boundary layer resolved) and 5000, Mach about 0.09. The Re 1000 runs are viscous and should sit below the inviscid curve; the grid study tracks convergence to that viscous answer, not to Theodorsen.',
  evidence: 'out/lbm/v6/',
  ran_at: new Date().toISOString(),
  // Sections run in resumable GPU holds and may span several invocations; this command reruns all of them.
  command: 'node tunnel/lbm/v6.mjs --force',
  invocation: `node tunnel/lbm/v6.mjs${argv.length ? ` ${argv.join(' ')}` : ''}`,
  published_case: 'Franck & Breuer, Unsteady high-lift mechanisms from heaving flat plate simulations, arXiv:1703.10906v3 (2017), table 3 and table 1: LES of a 2% thin ellipse, Re 40000, St 0.08, UT/U 0.12, span 0.2c periodic. Reference is itself LES, so this is a cross-code check at the Reynolds number of interest, not truth. Here: zero-thickness sheet, Smagorinsky 0.1 (as the reference), domain 20c x 12c instead of 30c x 15c.',
};
fs.mkdirSync(path.join(ROOT, 'out/verification'), { recursive: true });
fs.writeFileSync(path.join(ROOT, 'out/verification/V6.json'), JSON.stringify(doc, null, 1));
console.log(JSON.stringify({ passed, failures, notes }, null, 1));
