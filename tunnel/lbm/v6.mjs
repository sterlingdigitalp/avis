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
import { spawn, spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { theodorsenLift } from './src/theodorsen.mjs';
import { gridStudy, observedOrder } from './src/gridstudy.mjs';

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
    // Round 2: edges at the same sub-cell position on every level (edgeFrac). For an even chord this
    // is the round-1 placement exactly; c113 is new (round 1's ran with its edges at 0.77, kept as
    // pitch-Re5000-c113-round1-edge0.77), and c226 adds a finer level.
    // keepHistory only adds the lift and drag series to the output, so other fit windows can be checked.
    ...[40, 80, 113, 160, 226].map((c) => plate(`pitch-Re5000-c${c}`, { c, Re: 5000, k: 0.5, alpha0: ALPHA3, axis: -0.25, edgeFrac: 0.27, keepHistory: true })),
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
  // Round 2, informational. Same chord, edges moved half a cell: how much the edges' sub-cell
  // position alone moves the answer. Each pairs with the theodorsen run of the same chord.
  placement: [
    plate('pitch-Re5000-c80-edge0.77', { c: 80, Re: 5000, k: 0.5, alpha0: ALPHA3, axis: -0.25, edgeFrac: 0.77 }),
    plate('plunge-Re5000-c113-edge0.27', { c: 113, Re: 5000, k: 0.5, h0c: 0.05, edgeFrac: 0.27 }),
    plate('pitch-Re5000-c160-edge0.77', { c: 160, Re: 5000, k: 0.5, alpha0: ALPHA3, axis: -0.25, edgeFrac: 0.77 }),
  ],
  // Round 2, informational. The per-step mass correction in the open tunnel, against the same case
  // without it: open-tunnel-30-cycles in mass, the other two in theodorsen.
  massfix: [
    plate('open-tunnel-30-cycles-massfix', { c: 40, Re: 1000, k: 0.5, h0c: 0.05, cycles: 30, massEveryCycle: true, massFix: true }),
    plate('plunge-Re1000-c80-massfix', { c: 80, Re: 1000, k: 0.5, h0c: 0.05, massFix: true }),
    plate('pitch-Re5000-c80-massfix', { c: 80, Re: 5000, k: 0.5, alpha0: ALPHA3, axis: -0.25, edgeFrac: 0.27, massFix: true }),
  ],
  // Franck & Breuer (2017), arXiv:1703.10906v3, table 3. Re = 40000, St = 0.08, UT/U = 0.12, span 0.2c.
  franck: [40, 80].flatMap((c) => [
    { job: 'runInclined', key: `static-a5-c${c}`, args: { c, alphaDeg: 5, name: `v6-fb-s5-${c}` } },
    { job: 'runInclined', key: `heave-a5-c${c}`, args: { c, alphaDeg: 5, St: 0.08, UT: 0.12, name: `v6-fb-h5-${c}` } },
    { job: 'runInclined', key: `static-a13-c${c}`, args: { c, alphaDeg: 13, name: `v6-fb-s13-${c}` } },
    { job: 'runInclined', key: `heave-a13-c${c}`, args: { c, alphaDeg: 13, St: 0.08, UT: 0.12, name: `v6-fb-h13-${c}` } },
  ]),
};

// CPU sections need no GPU, so they run outside the lock and in parallel. Round 2: the mass budget
// identity on the CPU reference (same algorithm and geometry frames as the GPU) in f64 with
// compensated sums, and the same cases in f32 to show what the f32 residual is made of. Half size
// keeps the dimensionless case (tools/mass-audit.mjs); full size is the GPU case itself.
const AUDIT = path.join(SECT, 'audit');
const audit = (key, kind, bouzidi, precision, scale, timeout) => ({ key, args: { kind, bouzidi, precision, scale }, timeout });
const CPU_SECTIONS = {
  'mass-audit': [
    ...[['thin-bouzidi', 'thin', true], ['thin-bounceback', 'thin', false], ['slab-ladd', 'slab', false]].flatMap(([k, kind, bz]) =>
      ['f64', 'f32'].map((p) => audit(`${k}-half-${p}`, kind, bz, p, 0.5, 3000))),
    ...['f64', 'f32'].map((p) => audit(`thin-bouzidi-full-${p}`, 'thin', true, p, 1, 14000)),
  ],
};

// Each child is a run-guarded wrapper that owns its own tree; make sure none outlives this script.
const children = new Set();
const stopChildren = () => { for (const p of children) try { p.kill('SIGTERM'); } catch { /* gone */ } };
process.on('exit', stopChildren);
for (const sig of ['SIGTERM', 'SIGINT', 'SIGHUP']) process.on(sig, () => { stopChildren(); process.exit(130); });

async function runCpuSection(name) {
  fs.mkdirSync(AUDIT, { recursive: true });
  const todo = CPU_SECTIONS[name].filter((j) => !fs.existsSync(path.join(AUDIT, `${j.key}.json`)));
  const codes = await Promise.all(todo.map((j) => new Promise((resolve) => {
    const p = spawn('python3', [path.join(ROOT, 'tools/run-guarded.py'), '--timeout', String(j.timeout), '--log', path.join(AUDIT, `${j.key}.log`), '--',
      'node', path.join(HERE, 'tools/mass-audit.mjs'), path.join(AUDIT, `${j.key}.json`), JSON.stringify(j.args)], { stdio: 'ignore' });
    children.add(p);
    p.on('exit', (code) => { children.delete(p); resolve(code); });
  })));
  return codes.every((c) => c === 0);
}

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
  for (const name of Object.keys(CPU_SECTIONS)) {
    if (only && !only.includes(name)) continue;
    if (argv.includes('--force')) for (const j of CPU_SECTIONS[name]) fs.rmSync(path.join(AUDIT, `${j.key}.json`), { force: true });
    if (!(await runCpuSection(name))) console.error(`section ${name} did not finish`);
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
// Budget identity (round 2). The GPU residuals in measured.mass are f32 round-off of the collision
// itself, which no budget sees: on the CPU mirror in f32 the residual equals the collision term,
// and in f64 it vanishes. The identity is a claim about bookkeeping, so it is judged in f64 with
// compensated sums, on the CPU reference that the agree check ties to the GPU population by population.
const readAudit = (k) => { try { return JSON.parse(fs.readFileSync(path.join(AUDIT, `${k}.json`), 'utf8')); } catch { return null; } };
measured.mass_budget_identity = {};
for (const k of ['thin-bouzidi', 'thin-bounceback', 'slab-ladd']) {
  const row = {};
  for (const v of ['half-f64', 'half-f32', 'full-f64', 'full-f32']) {
    const a = readAudit(`${k}-${v}`);
    if (!a) continue;
    row[v] = { grid: [a.case.nx, a.case.ny, a.case.nz], chord_cells: a.case.c, cycles: a.case.cycles, mass_change: a.massChange, budget_exact: a.budgetExact, budget_fixed_point: a.budgetFixedPoint,
      collision_term: a.collisionTerm, residual_relative: a.residualExactRelative, residual_fixed_point_relative: a.residualFixedPointRelative, residual_after_collision_term_relative: a.residualAfterCollisionRelative, cpu_seconds: a.seconds };
    if (v.endsWith('f64') && !(a.residualExactRelative <= LIMIT.mass_budget_identity_relative)) failures.push(`mass budget identity ${k} ${v}: ${a.residualExactRelative}`);
  }
  row.gpu_f32 = ms[`${k}-raw`]?.budget_identity ?? null;
  measured.mass_budget_identity[k] = row;
  if (!row['half-f64']) failures.push(`mass budget identity ${k}: f64 audit missing`);
}
{
  const rows = Object.entries(measured.mass_budget_identity);
  const worst = (vs, f) => Math.max(...rows.flatMap(([, r]) => vs.map((v) => r[v]?.[f] ?? NaN)).filter(Number.isFinite));
  const gpu = rows.map(([k, r]) => `${k} ${r.gpu_f32?.residual_relative?.toExponential(2)}`).join(', ');
  const f64 = ['half-f64', 'full-f64'], f32 = ['half-f32', 'full-f32'];
  const full = measured.mass_budget_identity['thin-bouzidi']['full-f32'];
  notes.push(`mass budget identity is judged in f64 on the CPU reference (out/lbm/v6/audit/): worst residual ${worst(f64, 'residual_relative').toExponential(2)} with the exact budget, ${worst(f64, 'residual_fixed_point_relative').toExponential(2)} with the GPU's fixed-point one. The GPU f32 residuals (${gpu}) are the collision's own f32 round-off, which no budget sees: on the CPU in f32 the residual (up to ${worst(f32, 'residual_relative').toExponential(2)}) equals the measured collision term to ${worst(f32, 'residual_after_collision_term_relative').toExponential(2)}.${full ? ` At the GPU's own size the CPU in f32 gives mass change ${full.mass_change.toFixed(3)} against budget ${full.budget_exact.toFixed(3)} (GPU: ${measured.mass['thin-bouzidi-raw'].budget_identity.mass_change.toFixed(3)} against ${measured.mass['thin-bouzidi-raw'].budget_identity.link_and_refill_budget.toFixed(3)}), residual ${full.residual_relative.toExponential(2)}, collision term ${full.collision_term.toFixed(3)}.` : ''}`);
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
// Order from the finest three stable levels (src/gridstudy.mjs); every consecutive triple is reported too.
const study = (prefix, cs) => gridStudy(cs.map((c) => ({ c, r: T[`${prefix}-c${c}`] })), diverged);
measured.grid_study = {
  'plunge-Re1000': study('plunge-Re1000', [20, 40, 80, 160]),
  'pitch-Re1000': study('pitch-Re1000', [20, 40, 80, 160]),
  'plunge-Re5000': study('plunge-Re5000', [40, 80, 113, 160]),
  'pitch-Re5000': study('pitch-Re5000', [40, 80, 113, 160, 226]),
};
// Round 2, informational: sub-cell placement of the edges at fixed chord, and the per-step mass
// correction in the open tunnel. Each pair is the same case with one thing changed.
const P = R.placement, MF = R.massfix;
const pct = (a, b) => (100 * (b - a)) / a;
const placementPairs = [
  ['pitch-Re5000-c80', T['pitch-Re5000-c80'], P['pitch-Re5000-c80-edge0.77']],
  ['pitch-Re5000-c113', T['pitch-Re5000-c113'], T['pitch-Re5000-c113-round1-edge0.77']],
  ['pitch-Re5000-c160', T['pitch-Re5000-c160'], P['pitch-Re5000-c160-edge0.77']],
  ['plunge-Re5000-c113', P['plunge-Re5000-c113-edge0.27'], T['plunge-Re5000-c113']],
];
measured.edge_placement = placementPairs.filter(([, a, b]) => a?.cl && b?.cl).map(([k, a, b]) => ({
  case: k, c_cells: a.c, amp_edges_at_0_27: +a.cl.amp.toFixed(5), amp_edges_at_0_77: +b.cl.amp.toFixed(5), amp_change_pct: +pct(a.cl.amp, b.cl.amp).toFixed(3),
  phase_change_deg: +(b.cl.phaseDeg - a.cl.phaseDeg).toFixed(3), mean_cd_change_pct: +pct(a.meanCd, b.meanCd).toFixed(3),
}));
{
  const s = measured.grid_study['pitch-Re5000'], r1 = T['pitch-Re5000-c113-round1-edge0.77'], r2 = T['pitch-Re5000-c113'];
  if (s.triples && r1?.cl && r2?.cl) {
    const shift = Math.max(...measured.edge_placement.map((e) => Math.abs(e.amp_change_pct)));
    const tri = s.triples.map((t) => `${t.c_cells.join('/')}: amplitude ${t.amp.monotone ? `monotone, order ${t.amp.observed_order.toFixed(2)}` : 'not monotone'}, phase ${t.phase_deg.monotone ? `monotone, order ${t.phase_deg.observed_order.toFixed(2)}` : 'not monotone'}`).join('; ');
    notes.push(`grid study pitch-Re5000: round 1's c113 had its edges 0.77 cells past a node and the other levels 0.27, so its levels were not geometrically similar. With 0.27 on every level c113 reads ${r2.cl.amp.toFixed(5)}, not ${r1.cl.amp.toFixed(5)}. Amplitudes ${s.all_amp.map(([c, a]) => `${c}: ${a}`).join(', ')}. At a fixed chord, moving the edges half a cell moves the amplitude by up to ${shift.toFixed(2)}% (measured.edge_placement), the size of the level-to-level differences, so any observed order here sits inside that scatter. ${tri}.`);
  }
}
{
  // The plunge study passed in round 1 and is carried forward; its c113 has the edges at 0.77.
  const e = P['plunge-Re5000-c113-edge0.27'], a = T['plunge-Re5000-c80'], b = T['plunge-Re5000-c160'];
  if (e?.cl && a?.cl && b?.cl) {
    const o = observedOrder([a.cl.amp, e.cl.amp, b.cl.amp], Math.SQRT2);
    notes.push(`grid study plunge-Re5000 is carried forward from round 1, whose c113 has its edges at 0.77 cells. With the edges at 0.27 (placement section) c113 reads ${e.cl.amp.toFixed(5)} instead of ${T['plunge-Re5000-c113'].cl.amp.toFixed(5)}; the study 80/113/160 would then be ${o.monotone ? `monotone, difference ratio ${o.difference_ratio.toFixed(2)}, order ${o.observed_order.toFixed(2)}` : 'not monotone'}.`);
  }
}
const drhoMax = (r) => (r.meanDrhoByCycle ? Math.max(...r.meanDrhoByCycle.map(Math.abs)) : null);
const fixPairs = [
  ['open-tunnel-30-cycles', R.mass['open-tunnel-30-cycles'], MF['open-tunnel-30-cycles-massfix']],
  ['plunge-Re1000-c80', T['plunge-Re1000-c80'], MF['plunge-Re1000-c80-massfix']],
  ['pitch-Re5000-c80', T['pitch-Re5000-c80'], MF['pitch-Re5000-c80-massfix']],
];
measured.mass_correction_effect = fixPairs.filter(([, a, b]) => a?.cl && b?.cl).map(([k, a, b]) => ({
  case: k, c_cells: a.c, amp_raw: +a.cl.amp.toFixed(6), amp_corrected: +b.cl.amp.toFixed(6), amp_change_pct: +pct(a.cl.amp, b.cl.amp).toFixed(4),
  phase_change_deg: +(b.cl.phaseDeg - a.cl.phaseDeg).toFixed(4), mean_cl_raw: +a.cl.mean.toFixed(6), mean_cl_corrected: +b.cl.mean.toFixed(6),
  mean_cd_raw: +a.meanCd.toFixed(6), mean_cd_corrected: +b.meanCd.toFixed(6), mean_cd_change_pct: +pct(a.meanCd, b.meanCd).toFixed(4),
  correction_total_mass: b.massFixApplied, max_abs_mean_drho_raw: drhoMax(a), max_abs_mean_drho_corrected: drhoMax(b), mlups_raw: Math.round(a.mlups), mlups_corrected: Math.round(b.mlups),
}));
if (measured.mass_correction_effect.length) {
  const e = measured.mass_correction_effect, big = (f) => Math.max(...e.map((x) => Math.abs(x[f])));
  notes.push(`per-step mass correction in the open tunnel (measured.mass_correction_effect): lift amplitude moves by up to ${big('amp_change_pct').toFixed(3)}%, phase by up to ${big('phase_change_deg').toFixed(3)} deg, mean drag by up to ${big('mean_cd_change_pct').toFixed(4)}%; the largest mean density deviation over 30 cycles is ${e[0].max_abs_mean_drho_raw?.toExponential(2)} without it and ${e[0].max_abs_mean_drho_corrected?.toExponential(2)} with it. Bird runs (runDesign) leave it off.`);
}
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
  // Sections are measured once and carried forward; this says when each section file last changed.
  section_files_written_at: Object.fromEntries([...Object.keys(SECTIONS).map((n) => [n, path.join(SECT, `${n}.json`)]),
    ...CPU_SECTIONS['mass-audit'].map((j) => [`mass-audit/${j.key}`, path.join(AUDIT, `${j.key}.json`)])]
    .map(([n, f]) => [n, fs.existsSync(f) ? fs.statSync(f).mtime.toISOString() : null])),
  // Sections run in resumable GPU holds and may span several invocations; this command reruns all of them.
  command: 'node tunnel/lbm/v6.mjs --force',
  invocation: `node tunnel/lbm/v6.mjs${argv.length ? ` ${argv.join(' ')}` : ''}`,
  published_case: 'Franck & Breuer, Unsteady high-lift mechanisms from heaving flat plate simulations, arXiv:1703.10906v3 (2017), table 3 and table 1: LES of a 2% thin ellipse, Re 40000, St 0.08, UT/U 0.12, span 0.2c periodic. Reference is itself LES, so this is a cross-code check at the Reynolds number of interest, not truth. Here: zero-thickness sheet, Smagorinsky 0.1 (as the reference), domain 20c x 12c instead of 30c x 15c.',
};
fs.mkdirSync(path.join(ROOT, 'out/verification'), { recursive: true });
fs.writeFileSync(path.join(ROOT, 'out/verification/V6.json'), JSON.stringify(doc, null, 1));
console.log(JSON.stringify({ passed, failures, notes }, null, 1));
