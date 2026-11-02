// The pure halves of the GPU page jobs: run options, and results from a synthetic series of known content.
import test from 'node:test';
import assert from 'node:assert/strict';
import { plateRun, plateSummary, massBoxRun } from '../src/jobs.mjs';
import { sampleTimes } from '../src/plateforces.mjs';
import { theodorsenLift } from '../src/theodorsen.mjs';

test('plate run options: sampling, length and name follow the case', () => {
  const run = plateRun({ c: 20, Re: 1000, k: 0.5, h0c: 0.05 });
  const period = (2 * Math.PI) / ((2 * 0.5 * 0.05) / 20);
  assert.equal(run.total, Math.round(6 * period));
  assert.equal(run.M, Math.round(period / 96));
  assert.deepEqual([run.series.name, run.series.totalSteps, run.series.sampleEvery, run.series.massEvery, run.series.massFix, run.series.resume, run.series.maxSeconds],
    ['plate-20-1000-0.5-0.05-0-thin-true-1', run.total, run.M, 0, false, true, 1e9]);
  const r2 = plateRun({ c: 20, Re: 1000, k: 0.5, alpha0: 0.05, cycles: 3, samplesPerCycle: 48, massEveryCycle: true, massFix: true, resume: false, maxSeconds: 9, name: 'x' });
  assert.equal(plateRun({ c: 20, Re: 1000, k: 0.5, alpha0: 0.05 }).series.name, 'plate-20-1000-0.5-0-0.05-thin-true-1');
  assert.deepEqual(run.series.parts, [0]);
  assert.equal(plateRun({ c: 20, Re: 1000, k: 0.5, samplesPerCycle: 1e6 }).M, 1, 'at least one step a sample');
  assert.deepEqual([r2.total, r2.M, r2.series.massEvery, r2.series.massFix, r2.series.resume, r2.series.maxSeconds, r2.series.name], [Math.round(3 * period), Math.round(period / 48), Math.round(period), true, false, 9, 'x']);
});

test('plate summary: coefficients from forces, Theodorsen errors with the phase wrapped, optional outputs', () => {
  const o = { c: 20, Re: 1000, k: 0.5, h0c: 0.05, keepHistory: true, massEveryCycle: true, nz: 3 };
  const run = plateRun(o);
  const { pc, total, M } = run;
  const q = 0.5 * pc.U * pc.U * pc.c * pc.nz, A = 0.2, phase = (170 * Math.PI) / 180;
  const t = sampleTimes(total, M);
  // Window averages of A sin(w s + phase), in closed form.
  const F = [], Mo = [];
  for (let n = 0; n < total; n += M) {
    const a = n, b = n + Math.min(M, total - n), w = pc.omega;
    const cl = (A * (Math.cos(w * a + phase) - Math.cos(w * b + phase))) / (w * (b - a));
    F.push([-0.03 * q, cl * q, 0]); Mo.push([0, 0, 0.01 * q * pc.c]);
  }
  const r = { cells: 1234, mlups: 5, series: { t, F: [F], M: [Mo], mass: [{ deltaMass: 2, cells: 4 }], ms: 2000, frameMs: 10, segments: 1, maxLinks: 7 } };
  const s = plateSummary(run, r);
  const th = theodorsenLift({ k: 0.5, h0: 0.05 });
  assert.ok(Math.abs(s.cl.amp / A - 1) < 5e-5, `amplitude ${s.cl.amp}`);
  assert.ok(Math.abs(s.cl.phaseDeg - 170) < 0.005, `phase ${s.cl.phaseDeg}`);
  assert.ok(Math.abs(s.meanCd - 0.03) < 1e-12 && Math.abs(s.history.cm[3] - 0.01) < 1e-12);
  assert.ok(Math.abs(s.ampErrPct - (100 * (s.cl.amp - th.amp)) / th.amp) < 1e-12);
  const raw = s.cl.phaseDeg - (th.phaseRad * 180) / Math.PI;
  assert.ok(raw > 180 && Math.abs(s.phaseErrDeg - (raw - 360)) < 1e-9, `wrapped ${s.phaseErrDeg} from ${raw}`);
  assert.deepEqual([s.steps, s.cells, s.seconds, s.links, s.meanDrhoByCycle, s.massFix, s.massFixApplied, s.thickness, s.kind, s.edgeFrac], [total, 1234, 2, 7, [0.5], false, 0, 0, 'thin', 0.27]);
  assert.deepEqual([s.done, s.c, s.Re, s.k, s.h0c, s.alpha0, s.axis, s.bouzidi, s.relink, s.mlups, s.segments, s.tau0], [true, 20, 1000, 0.5, 0.05, 0, 0, true, 1, 5, 1, pc.tau0]);
  assert.ok(Math.abs(s.frameMsPerStep - 10 / total) < 1e-15);
  const bare = plateSummary(plateRun({ c: 20, Re: 1000, k: 0.5, kind: 'slab', thickness: 3, nz: 3 }), r);
  assert.ok(!('history' in bare) && !('meanDrhoByCycle' in bare));
  assert.equal(bare.thickness, 3);
  assert.equal(plateSummary(plateRun({ c: 20, Re: 1000, k: 0.5, kind: 'slab', nz: 3 }), r).thickness, 2);
  // A pitch case is compared with Theodorsen for pitch about its own axis.
  const ps = plateSummary(plateRun({ c: 20, Re: 1000, k: 0.5, alpha0: 0.05, axis: -0.25, nz: 3 }), r);
  const tp = theodorsenLift({ k: 0.5, alpha0: 0.05, axisC: -0.25 });
  assert.ok(Math.abs(ps.ampErrPct - (100 * (ps.cl.amp - tp.amp)) / tp.amp) < 1e-12);
  assert.deepEqual([ps.h0c, ps.alpha0, ps.axis], [0, 0.05, -0.25]);
  const pm = plateSummary(plateRun({ c: 20, Re: 1000, k: 0.5, alpha0: 0.05, nz: 3 }), r), tm = theodorsenLift({ k: 0.5, alpha0: 0.05 });
  assert.ok(Math.abs(pm.ampErrPct - (100 * (pm.cl.amp - tm.amp)) / tm.amp) < 1e-12);
});

test('closed-box run options and description', () => {
  const { meta, series } = massBoxRun({});
  assert.deepEqual([series.name, series.totalSteps, series.sampleEvery, series.massEvery, series.relink, series.maxSeconds], ['mass-thin-true-false-true', 50000, 10, 1000, 1, 1e9]);
  assert.ok(Math.abs(meta.peakBodyU - (8 * 2 * Math.PI) / 1000) < 1e-15);
  assert.deepEqual([meta.blockage, meta.tau0, meta.U, meta.cycles], [0.25, 0.53, 0.03, 50]);
  const fix = massBoxRun({ massFix: true, kind: 'slab', bouzidi: false, cycles: 2, period: 100, name: 'm', maxSeconds: 5 });
  assert.deepEqual([fix.series.name, fix.series.sampleEvery, fix.series.totalSteps, fix.series.maxSeconds, fix.meta.kind], ['m', 1, 200, 5, 'slab']);
  assert.equal(fix.series.surfaces[0].kind, 'slab');
  assert.deepEqual(series.parts, [0]);
});

// Pose of a plate at time t: chord ends, centre height and pitch angle, from its mesh vertices.
const poseOf = (surf, t) => {
  const p = surf.pose(t).pos;
  let iLe = 0, iTe = 0;
  for (let v = 0; v < p.length; v += 3) { if (p[v] > p[iLe]) iLe = v; if (p[v] < p[iTe]) iTe = v; }
  return { le: [p[iLe], p[iLe + 1]], te: [p[iTe], p[iTe + 1]], zMin: Math.min(...Array.from({ length: p.length / 3 }, (_, i) => p[i * 3 + 2])), zMax: Math.max(...Array.from({ length: p.length / 3 }, (_, i) => p[i * 3 + 2])), alpha: Math.atan2(p[iLe + 1] - p[iTe + 1], p[iLe] - p[iTe]) };
};
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

test('plate case means what its parameters say: Reynolds number, reduced frequency, domain, plate and motion', async () => {
  const { plateCase } = await import('../src/jobs.mjs');
  const { AXIS } = await import('../src/lattice.mjs');
  for (const [c, Re, k, U] of [[40, 1000, 0.5, 0.05], [113, 5000, 0.25, 0.04]]) {
    const pc = plateCase({ c, Re, k, U, h0c: 0.05, edgeFrac: 0.27 });
    near(U * c / ((pc.tau0 - 0.5) / 3), Re, 1e-9 * Re, 'Re = U c / nu with nu = (tau - 1/2)/3');
    near(pc.omega * c / (2 * U), k, 1e-15, 'k = omega c / 2U');
    near(pc.period * pc.omega, 2 * Math.PI, 1e-12, 'period');
    assert.deepEqual([pc.nx, pc.ny, pc.nz, pc.grid.xMode, pc.grid.yLowMode, pc.grid.yHighMode, pc.grid.zLowMode, pc.grid.zHighMode], [24 * c, 16 * c, 1, AXIS.inflow, AXIS.freeslip, AXIS.freeslip, AXIS.periodic, AXIS.periodic]);
    assert.deepEqual([pc.cfg.inletU, pc.cfg.u0, pc.cfg.sponge, pc.cfg.csSmag, pc.cfg.giTerm, pc.cfg.localRho, pc.cfg.tau0], [[-U, 0, 0], [-U, 0, 0], { strength: 0.05, xLow: Math.round(2 * c) }, 0, true, true, pc.tau0]);
    // At rest before the ramp: level, centred 0.13 below mid-height, leading edge 16c + c/2 from the outlet.
    const p0 = poseOf(pc.surf, 0);
    near(p0.le[0] - p0.te[0], c, 1e-9, 'chord');
    near(p0.le[0], 16 * c + Math.floor(c / 2) + 0.27, 1e-9, 'leading edge x');
    near(p0.le[1], pc.ny / 2 - 0.13, 1e-12, 'height');
    assert.ok(p0.zMin < 0 && p0.zMax > pc.nz, 'spans the periodic width');
    // A quarter period after the ramp the heave is at its crest, h0/c = 0.05.
    const tc = pc.period + pc.period / 4;
    near(poseOf(pc.surf, tc).le[1] - p0.le[1], 0.05 * c, 1e-6 * c, 'heave amplitude');
    assert.equal(pc.surf.tris.length / 6, Math.max(4, Math.round(c / 8)));
  }
  // Pitch about the quarter chord: the point 0.25c behind the leading edge does not move.
  const pp = plateCase({ c: 80, Re: 1000, k: 0.5, alpha0: 0.05, axis: -0.25 });
  const tc = pp.period + pp.period / 4, a = poseOf(pp.surf, tc), b = poseOf(pp.surf, 0);
  near(a.alpha, 0.05, 1e-9, 'pitch amplitude');
  near(a.le[1] - 0.25 * 80 * Math.sin(a.alpha), b.le[1], 1e-9, 'quarter-chord height fixed');
  // Pitch about the mid-chord by default: the mid-chord stays at its height.
  const pm = plateCase({ c: 80, Re: 1000, k: 0.5, alpha0: 0.05 });
  const am = poseOf(pm.surf, tc), bm = poseOf(pm.surf, 0);
  near((am.le[1] + am.te[1]) / 2, (bm.le[1] + bm.te[1]) / 2, 1e-9, 'mid-chord height fixed');
  assert.equal(pm.surf.part, 0);
  assert.deepEqual([plateCase({ c: 113, Re: 1000, k: 0.5 }).edgeFrac, plateCase({ c: 113, Re: 1000, k: 0.5, edgeFrac: 0.27 }).edgeFrac], [0.77, 0.27]);
  const slab = plateCase({ c: 20, Re: 1000, k: 0.5, kind: 'slab', thickness: 3, sponge: { strength: 0.1, xLow: 5 }, giTerm: false, localRho: false, nz: 2 });
  assert.deepEqual([slab.surf.kind, slab.surf.thickness, slab.cfg.sponge, slab.cfg.giTerm, slab.cfg.localRho, slab.nz], ['slab', 3, { strength: 0.1, xLow: 5 }, false, false, 2]);
});

test('closed-box case: periodic box, blockage, heave amplitude and body speed as stated', async () => {
  const { massBoxCase } = await import('../src/jobs.mjs');
  const { AXIS } = await import('../src/lattice.mjs');
  const { grid, cfg, surfaces } = massBoxCase({ kind: 'slab' });
  assert.deepEqual([grid.nx, grid.ny, grid.nz, ...['xMode', 'yLowMode', 'yHighMode', 'zLowMode', 'zHighMode'].map((k) => grid[k])], [160, 160, 4, ...Array(5).fill(AXIS.periodic)]);
  assert.deepEqual([cfg.tau0, cfg.csSmag, cfg.u0], [0.53, 0, [-0.03, 0, 0]]);
  const s = surfaces[0];
  assert.deepEqual([s.kind, s.thickness, s.part], ['slab', 2, 0]);
  const p0 = poseOf(s, 0);
  near(p0.le[0] - p0.te[0], 40, 1e-9, 'chord, blockage 40/160');
  near(p0.le[0], 80 + 20 + 0.27, 1e-9, 'leading edge');
  near(p0.le[1], 80.37, 1e-12, 'height');
  assert.ok(p0.zMin < 0 && p0.zMax > 4);
  near(poseOf(s, 1250).le[1] - p0.le[1], 8, 1e-9, 'heave amplitude at the crest after the ramp');
  near(s.pose(1000).vel[1], 8 * (2 * Math.PI) / 1000, 1e-12, 'peak body speed');
  near(poseOf(massBoxCase({ alphaMean: 0.1 }).surfaces[0], 0).alpha, 0.1, 1e-12, 'mean pitch');
  assert.equal(massBoxCase({ withBody: false }).surfaces.length, 0);
  const small = massBoxCase({ nx: 80, ny: 60, nz: 2, c: 20, h0: 4, period: 500, tau0: 0.6, U: 0.02 });
  assert.deepEqual([small.grid.nx, small.grid.ny, small.grid.nz, small.cfg.tau0, small.cfg.u0[0]], [80, 60, 2, 0.6, -0.02]);
  near(poseOf(small.surfaces[0], 625).le[1] - poseOf(small.surfaces[0], 0).le[1], 4, 1e-9, 'scaled heave');
});
