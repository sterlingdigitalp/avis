// Page-side jobs driven by page.evaluate. Each is a plain async function returning JSON.
import { CpuLbm } from './cpu.mjs';
import { GpuLbm, requestDevice } from './gpu.mjs';
import { BoundaryEngine } from './geometry.mjs';
import { AXIS, Q } from './lattice.mjs';
import { rigidPlateSurface, harmonic } from './motion.mjs';

let ctx = null;
export async function gpu() {
  if (!ctx) ctx = await requestDevice();
  return ctx;
}

export async function adapter() {
  const { adapterInfo, limits } = await gpu();
  return { adapterInfo, limits };
}

/** MLUPS at one size with a realistic link load, no readback except timing fences. */
export async function bench({ nx, ny, nz, steps = 40, tau0 = 0.5002, csSmag = 0.16, nb, layout, wg, sponge, withPlate = true }) {
  const { device, errors } = await gpu();
  const cfg = { nx, ny, nz, tau0, csSmag, nb, layout, wg, inletU: [-0.06, 0, 0], u0: [-0.06, 0, 0], sponge };
  const surfaces = withPlate ? [rigidPlateSurface({ chord: nx * 0.2, zMin: nz * 0.1, zMax: nz * 0.9, x0: nx * 0.5, y0: ny * 0.5 + 0.3, nChord: 16, nSpan: 16, m: harmonic({ h0: 2, omega: 0.002 }) })] : [];
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
  const eng = surfaces.length ? new BoundaryEngine(grid, surfaces, { bouzidi: true }) : null;
  const s = new GpuLbm(device, cfg);
  if (eng) s.setFrame(eng.init(0.5));
  await s.finish();
  const warm = 4;
  s.step(warm);
  await s.finish();
  const t0 = performance.now();
  let frameMs = 0;
  for (let n = 0; n < steps; n++) {
    if (eng) { const a = performance.now(); s.setFrame(eng.frame(n + warm + 0.5)); frameMs += performance.now() - a; }
    s.step(1);
    if (n % 4 === 3) await s.finish();
  }
  await s.finish();
  const dtMs = performance.now() - t0;
  const res = { nx, ny, nz, cells: s.n, nb: s.nb, layout: s.layout, wg: s.wg, steps, ms: dtMs, mlups: (s.n * steps) / (dtMs * 1000), msPerStep: dtMs / steps, frameMsPerStep: frameMs / steps, links: eng?.stats.links ?? 0, errors: errors.splice(0) };
  s.destroy();
  return res;
}

function buildAgreeSurfaces(variant, nx, ny, nz, omega) {
  const m = harmonic({ h0: 2.3, alpha0: 0.15, omega });
  const plate = (o) => rigidPlateSurface({ chord: 14, zMin: 3.2, zMax: nz - 5.7, x0: nx * 0.5 + 0.21, y0: ny * 0.5 + 0.37, nChord: 5, nSpan: 3, axis: -0.25, m, ...o });
  if (variant === 'thin') return [plate({ part: 2, kind: 'thin' })];
  if (variant === 'slab') return [plate({ part: 2, kind: 'slab', thickness: 2.4 })];
  if (variant === 'half') return [plate({ part: 2, kind: 'thin', zMin: -0.5, zMax: nz - 5.7, mirrorZ: true })];
  throw new Error(variant);
}

/**
 * GPU versus CPU on the same setup and the same geometry frames: every population of every fluid
 * cell and the force and moment accumulators, after a moving-body run with swept or covered cells.
 * Also runs the CPU in f64 so the f32 round-off level is measured, not assumed.
 */
export async function agree({ variant = 'thin', bouzidi = true, nx = 48, ny = 32, nz = 24, steps = 400, every = 100, tau0 = 0.56, csSmag = 0.12, U = -0.05, omega = 0.03, sponge = { strength: 0.1, xLow: 6 } }) {
  const { device, errors, adapterInfo } = await gpu();
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
  const eng = new BoundaryEngine(grid, buildAgreeSurfaces(variant, nx, ny, nz, omega), { bouzidi });
  const init = eng.init(0.5);
  const base = { ...grid, tau0, csSmag, inletU: [U, 0, 0], u0: [U, 0, 0], sponge, mask: eng.mask ?? undefined };
  const cpu = new CpuLbm({ ...base, precision: 'f32' });
  const ref = new CpuLbm({ ...base, precision: 'f64' });
  const g = new GpuLbm(device, base);
  for (const x of [cpu, ref, g]) x.setFrame(init);
  const part = 2;
  const trace = [];
  let swept = 0, edits = 0;
  let Fg = [0, 0, 0], Fc = [0, 0, 0], Mg = [0, 0, 0], Mc = [0, 0, 0];
  for (let n = 0; n < steps; n++) {
    const fr = eng.frame(n + 0.5);
    swept += eng.stats.swept;
    edits += fr.maskEdits.length;
    for (const x of [cpu, ref, g]) { x.setFrame(fr); x.step(1); }
    if ((n + 1) % every === 0) {
      await g.finish();
      const a = await g.readAll();
      let maxAbs = 0, sumAbs = 0, count = 0, bad = 0, maxPop = 0, maxF64 = 0;
      for (let c = 0; c < g.n; c++) {
        if (eng.mask && eng.mask[c] !== 255) continue;
        for (let i = 0; i < Q; i++) {
          const v = a[c * Q + i], w = cpu.src[c * Q + i];
          const d = Math.abs(v - w);
          if (!Number.isFinite(d)) { bad++; continue; }
          sumAbs += d; count++;
          if (d > maxAbs) maxAbs = d;
          if (Math.abs(w) > maxPop) maxPop = Math.abs(w);
          const e = Math.abs(w - ref.src[c * Q + i]);
          if (e > maxF64) maxF64 = e;
        }
      }
      const fg = (await g.takeForces())[part], fc = cpu.takeForces()[part];
      ref.takeForces();
      for (let k = 0; k < 3; k++) { Fg[k] += fg.F[k]; Fc[k] += fc.F[k]; Mg[k] += fg.M[k]; Mc[k] += fc.M[k]; }
      const norm = Math.hypot(...Fc), normM = Math.hypot(...Mc);
      trace.push({ step: n + 1, maxAbs, meanAbs: sumAbs / count, maxPop, nonFinite: bad, cpuF32vsF64: maxF64,
        forceRelErr: Math.hypot(Fg[0] - Fc[0], Fg[1] - Fc[1], Fg[2] - Fc[2]) / norm, momentRelErr: Math.hypot(Mg[0] - Mc[0], Mg[1] - Mc[1], Mg[2] - Mc[2]) / normM });
    }
  }
  const out = { variant, bouzidi, nb: g.nb, nx, ny, nz, steps, sweptEvents: swept, maskEdits: edits, links: eng.stats.links, trace, last: trace[trace.length - 1], Fcpu: Fc, Fgpu: Fg, adapterInfo, errors: errors.splice(0) };
  g.destroy();
  return out;
}

/** Closed periodic box with a body oscillating through it: the setup shared by the GPU job and the CPU audit. */
export function massBoxCase({ kind = 'thin', withBody = true, nx = 160, ny = 160, nz = 4, c = 40, h0 = 8, alphaMean = 0, period = 1000, tau0 = 0.53, U = 0.03 }) {
  const grid = { nx, ny, nz, xMode: AXIS.periodic, yLowMode: AXIS.periodic, yHighMode: AXIS.periodic, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic };
  const omega = (2 * Math.PI) / period;
  const m = harmonic({ h0, omega, alphaMean, rampSteps: period / 4 });
  const surfaces = withBody ? [rigidPlateSurface({ part: 0, kind, thickness: 2, chord: c, zMin: -3, zMax: nz + 2, x0: nx / 2 + 0.27, y0: ny / 2 + 0.37, nChord: 4, nSpan: 1, m })] : [];
  return { grid, surfaces, cfg: { ...grid, tau0, csSmag: 0, u0: [-U, 0, 0] } };
}

/** runSeries options and the description of one closed-box run. Pure. */
export function massBoxRun(o) {
  const { kind = 'thin', bouzidi = true, massFix = false, withBody = true, ny = 160, c = 40, h0 = 8, period = 1000, cycles = 50, tau0 = 0.53, U = 0.03, name, maxSeconds } = o;
  const { grid, cfg, surfaces } = massBoxCase(o);
  const series = { name: name ?? `mass-${kind}-${bouzidi}-${massFix}-${withBody}`, grid, cfg, surfaces, bouzidi, relink: 1, totalSteps: cycles * period, sampleEvery: massFix ? 1 : 10, parts: [0], massEvery: period, massFix, maxSeconds: maxSeconds ?? 1e9 };
  return { meta: { kind, bouzidi, massFix, withBody, cycles, peakBodyU: h0 * ((2 * Math.PI) / period), U, blockage: c / ny, tau0 }, series };
}

/** Closed periodic box with a body oscillating through it for many cycles, tracking total mass. */
export async function massBox(o) {
  const { device, errors } = await gpu();
  const run = massBoxRun(o);
  const r = await runSeries(device, run.series);
  if (!r.done) return { done: false, n: r.n, errors: errors.splice(0) };
  const pts = r.series.mass;
  return { done: true, ...run.meta, cells: r.cells, ...massSeries(pts), fluidCellsEnd: pts[0]?.cells ?? r.cells, errors: errors.splice(0) };
}

/** Checkpoint and restart against an uninterrupted run. Must match bit for bit. */
export async function restartCheck({ nx = 64, ny = 48, nz = 32, steps = 240, kind = 'thin', name = 'restart-check' }) {
  const { device, errors } = await gpu();
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
  const surf = () => [rigidPlateSurface({ part: 0, kind, thickness: 2.4, chord: 16, zMin: 4.2, zMax: nz - 6.1, x0: nx / 2 + 0.3, y0: ny / 2 + 0.2, nChord: 4, nSpan: 2, axis: -0.25, m: harmonic({ h0: 3, alpha0: 0.1, omega: 0.04 }) })];
  const mk = (eng) => new GpuLbm(device, { ...grid, tau0: 0.54, csSmag: 0.1, inletU: [-0.05, 0, 0], u0: [-0.05, 0, 0], mask: eng.mask ?? undefined });
  const run = async (s, eng, a, b) => { for (let n = a; n < b; n++) { s.setFrame(eng.frame(n + 0.5)); s.step(1); } await s.finish(); };
  const engA = new BoundaryEngine(grid, surf(), { bouzidi: true });
  const A = mk(engA); A.setFrame(engA.init(0.5));
  await run(A, engA, 0, steps);
  const fa = await A.readAll();
  const engB = new BoundaryEngine(grid, surf(), { bouzidi: true });
  const B = mk(engB); B.setFrame(engB.init(0.5));
  await run(B, engB, 0, steps / 2);
  await B.saveCheckpoint(`out/lbm/ckpt/${name}`, { name });
  B.destroy();
  const engC = new BoundaryEngine(grid, surf(), { bouzidi: true });
  const C = mk(engC);
  await C.loadCheckpoint(`out/lbm/ckpt/${name}`);
  C.setFrame(engC.init(steps / 2 - 1 + 0.5));
  await run(C, engC, steps / 2, steps);
  const fc = await C.readAll();
  let maxAbs = 0, differing = 0;
  for (let k = 0; k < fa.length; k++) {
    if (engA.mask && engA.mask[Math.floor(k / Q)] !== 255) continue;
    const d = Math.abs(fa[k] - fc[k]);
    if (d > 0) differing++;
    if (d > maxAbs) maxAbs = d;
  }
  A.destroy(); C.destroy();
  for (const suffix of ['', '.json']) await fetch(`/out/lbm/ckpt/${name}${suffix}`, { method: 'DELETE' });
  return { kind, steps, maxAbs, differing, total: fa.length, errors: errors.splice(0) };
}

/** Static or heaving inclined plate at high Re with LES. See Franck & Breuer (2017), table 3. */
export async function runInclined(o) {
  const { device, errors, adapterInfo } = await gpu();
  const { c, alphaDeg, St = 0, UT = 0, Re = 40000, U = 0.06, nxc = 20, nyc = 12, xPlate = 13, spanC = 0.2, csSmag = 0.1, cycles = 5, staticSteps } = o;
  const nx = Math.round(nxc * c), ny = Math.round(nyc * c), nz = Math.max(1, Math.round(spanC * c));
  const nu = (U * c) / Re, tau0 = 3 * nu + 0.5;
  const alpha = (alphaDeg * Math.PI) / 180;
  const heaving = St > 0;
  const f = (St * U) / c, omega = 2 * Math.PI * f, period = heaving ? 1 / f : 0;
  // Reference: v_y = -UT sin(2 pi f t), so h = (UT/(2 pi f)) cos(2 pi f t): plate starts at the top.
  const h0 = heaving ? (UT * U) / omega : 0;
  const m = heaving
    ? { h: (t) => h0 * Math.cos(omega * t), hd: (t) => -h0 * omega * Math.sin(omega * t), alpha: () => alpha, alphad: () => 0 }
    : { h: () => 0, hd: () => 0, alpha: () => alpha, alphad: () => 0 };
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic };
  const surf = rigidPlateSurface({ part: 0, kind: 'thin', chord: c, zMin: -3, zMax: nz + 2, x0: xPlate * c + 0.27, y0: ny / 2 - 0.5 + 0.37, axis: 0, nChord: Math.max(4, Math.round(c / 8)), nSpan: Math.max(1, Math.round(nz / 16)), m });
  surf.moving = heaving;
  const total = heaving ? Math.round(cycles * period) : (staticSteps ?? Math.round((25 * c) / U));
  const M = Math.max(4, Math.round(heaving ? period / 96 : total / 400));
  const name = o.name ?? `inclined-${c}-${alphaDeg}-${St}`;
  const r = await runSeries(device, { name, grid, cfg: { ...grid, tau0, csSmag, inletU: [-U, 0, 0], u0: [-U, 0, 0], sponge: { strength: 0.05, xLow: 2 * c }, inletRampSteps: 0 }, surfaces: [surf], bouzidi: true, relink: heaving ? (o.relink ?? 1) : total + 1, totalSteps: total, sampleEvery: M, parts: [0], maxSeconds: o.maxSeconds ?? 1e9 });
  if (!r.done) return { done: false, n: r.n, total, errors: errors.splice(0) };
  const q = 0.5 * U * U * c * nz;
  const se = r.series;
  const cl = se.F[0].map((v) => v[1] / q), cd = se.F[0].map((v) => -v[0] / q);
  // Average over whole cycles after the first two, or the last half of a static run.
  const startT = heaving ? 2 * period : total / 2;
  let i0 = se.t.findIndex((t) => t >= startT);
  if (heaving) { const nC = Math.floor((total - startT) / period); i0 = se.t.findIndex((t) => t >= total - nC * period); }
  const mean = (a) => a.slice(i0).reduce((p, v) => p + v, 0) / (a.length - i0);
  const clMean = mean(cl), cdMean = mean(cd);
  const clMax = Math.max(...cl.slice(i0));
  let lastCycles = null;
  if (heaving) {
    const perCycle = [];
    for (let j = 0; j < cycles; j++) {
      const idx = se.t.map((t, k) => (t >= j * period && t < (j + 1) * period ? k : -1)).filter((k) => k >= 0);
      perCycle.push(idx.reduce((p, k) => p + cl[k], 0) / idx.length);
    }
    lastCycles = perCycle;
  }
  return { done: true, c, alphaDeg, St, UT, Re, tau0, cells: r.cells, steps: total, seconds: se.ms / 1000, segments: se.segments, mlups: r.mlups, frameMsPerStep: se.frameMs / total, links: se.maxLinks, clMean, cdMean, clMax, perCycleCl: lastCycles, h0c: h0 / c, adapterInfo, errors: errors.splice(0) };
}

import { theodorsenLift } from './theodorsen.mjs';
import { liftHarmonic } from './plateforces.mjs';
import { massSeries } from './mass.mjs';
import { runSeries } from './run.mjs';

/** 2D oscillating plate: one cell periodic span, free-slip walls, inlet at +x, outlet at -x. */
export function plateCase(o) {
  const { c, Re, U = 0.05, k, h0c = 0, alpha0 = 0, alphaMean = 0, axis = 0, kind = 'thin', thickness = 2, bouzidi = true, nxc = 24, nyc = 16, xPlate = 16, nz = 1, relink = 1, csSmag = 0, precisionNote } = o;
  void precisionNote;
  const nx = Math.round(nxc * c), ny = Math.round(nyc * c);
  const nu = (U * c) / Re, tau0 = 3 * nu + 0.5;
  const omega = (2 * k * U) / c;
  const period = (2 * Math.PI) / omega;
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic };
  // Fractional offsets keep the plate off lattice planes. By default they fix the mid-chord, which
  // puts the edges 0.27 cells past a node when c/2 is a whole number and 0.77 when it is not, so an
  // odd chord is not geometrically similar to an even one. edgeFrac fixes the edges' sub-cell
  // position instead, the same at every chord: use it for grid studies that mix odd and even chords.
  const midFrac = o.edgeFrac === undefined ? 0.27 : o.edgeFrac - (c / 2 - Math.floor(c / 2));
  const x0 = xPlate * c + midFrac, y0 = ny / 2 - 0.5 + 0.37;
  const m = harmonic({ h0: h0c * c, alpha0, alphaMean, omega, rampSteps: Math.round(period / 4) });
  const surf = rigidPlateSurface({ part: 0, kind, thickness, chord: c, zMin: -3, zMax: nz + 2, x0, y0, axis, nChord: Math.max(4, Math.round(c / 8)), nSpan: 1, m });
  const sponge = o.sponge ?? { strength: 0.05, xLow: Math.round(2 * c) };
  return { grid, surf, omega, period, tau0, cfg: { ...grid, tau0, csSmag, inletU: [-U, 0, 0], u0: [-U, 0, 0], sponge, giTerm: o.giTerm ?? true, localRho: o.localRho ?? true }, bouzidi, relink, nx, ny, nz, U, c,
    edgeFrac: +((x0 + c / 2) % 1).toFixed(6) };
}

/** runSeries options and sampling of one plate run. Pure. */
export function plateRun(o) {
  const pc = plateCase(o);
  const { period, relink, c } = pc;
  const total = Math.round((o.cycles ?? 6) * period);
  const M = Math.max(1, Math.round(period / (o.samplesPerCycle ?? 96)));
  const series = {
    name: o.name ?? `plate-${c}-${o.Re}-${o.k}-${o.h0c ?? 0}-${o.alpha0 ?? 0}-${o.kind ?? 'thin'}-${pc.bouzidi}-${relink}`,
    grid: pc.grid, cfg: pc.cfg, surfaces: [pc.surf], bouzidi: pc.bouzidi, relink, totalSteps: total, sampleEvery: M, parts: [0],
    maxSeconds: o.maxSeconds ?? 1e9, resume: o.resume ?? true, massEvery: o.massEveryCycle ? Math.round(period) : 0, massFix: o.massFix ?? false,
  };
  return { o, pc, total, M, series };
}

/** Result of a finished plate run from its force series, against Theodorsen. Pure. */
export function plateSummary({ o, pc, total, M }, r) {
  const { period, omega, relink, U, c } = pc;
  const se = r.series, t = se.t;
  const q = 0.5 * U * U * c * pc.nz;
  const cl = se.F[0].map((v) => v[1] / q), cd = se.F[0].map((v) => -v[0] / q), cm = se.M[0].map((v) => v[2] / (q * c));
  const lift = liftHarmonic({ t, cl, cd }, { omega, period, M });
  const th = theodorsenLift({ k: o.k, chord: 1, U: 1, h0: o.h0c ?? 0, alpha0: o.alpha0 ?? 0, axisC: o.axis ?? 0 });
  const res = {
    done: true, c, Re: o.Re, k: o.k, h0c: o.h0c ?? 0, alpha0: o.alpha0 ?? 0, axis: o.axis ?? 0, kind: o.kind ?? 'thin', thickness: o.kind === 'slab' ? o.thickness ?? 2 : 0, bouzidi: pc.bouzidi, relink,
    edgeFrac: pc.edgeFrac, massFix: o.massFix ?? false, massFixApplied: se.fixApplied ?? 0,
    tau0: pc.cfg.tau0, cells: r.cells, steps: total, seconds: se.ms / 1000, segments: se.segments, mlups: r.mlups, frameMsPerStep: se.frameMs / total, links: se.maxLinks,
    cl: lift.cl, theo: { amp: th.amp, phaseDeg: (th.phaseRad * 180) / Math.PI }, meanCd: lift.meanCd,
  };
  res.ampErrPct = (100 * (res.cl.amp - th.amp)) / th.amp;
  res.phaseErrDeg = ((res.cl.phaseDeg - res.theo.phaseDeg + 540) % 360) - 180;
  if (o.massEveryCycle) res.meanDrhoByCycle = se.mass.map((p) => p.deltaMass / p.cells);
  if (o.keepHistory) res.history = { t, cl, cd, cm };
  return res;
}

/** Oscillating plate case, resumable. Returns done:false if its time budget ran out. */
export async function runPlate(o) {
  const { device, errors, adapterInfo } = await gpu();
  const run = plateRun(o);
  const r = await runSeries(device, run.series);
  if (!r.done) return { done: false, n: r.n, total: run.total, errors: errors.splice(0) };
  return { ...plateSummary(run, r), adapterInfo, errors: errors.splice(0) };
}


/** A whole bird in the tunnel for some wingbeats. Half model by default. Resumable. */
export async function runDesign(o) {
  const { device, errors, adapterInfo } = await gpu();
  const kin = await import('/src/physics/kinematics.js');
  const { designAtmosphere } = await import('/src/physics/atmosphere.js');
  const B = await import('./bird.mjs');
  const design = B.readDesign(o.designFile, kin, o.speed);
  const air = designAtmosphere();
  const speed = o.speed ?? design.speed, freq = o.freq ?? design.freq, amp = o.amp ?? design.amp;
  if (!(speed > 0 && freq > 0 && amp > 0)) throw new Error(`design lacks speed/freq/amp: ${speed} ${freq} ${amp}`);
  const dx = o.dx ?? 0.005, half = o.half ?? true, uLatMax = o.uLat ?? 0.12, csSmag = o.csSmag ?? 0.16, relink = o.relink ?? 2, wingbeats = o.wingbeats ?? 3;
  const wing = kin.makeWing(design.wingSpec);
  const span = kin.wingSpan(wing), cMean = kin.meanChord(wing), mac = kin.mac(wing), area = 2 * kin.wingArea(wing);
  const vWall = B.peakWallSpeed(kin, design, freq, amp);
  const dt = (uLatMax * dx) / (speed + vWall);
  const nuLat = (air.nu * dt) / (dx * dx);
  const tau0 = 0.5 + 3 * nuLat;
  const stepsPerBeat = Math.round(1 / (freq * dt));
  const dtExact = 1 / (freq * stepsPerBeat);
  // Default domain, in spans. Half model: symmetry plane at z = 0.
  const L = o.domainSpans ?? { up: 0.8, down: 1.4, y: 1.4, z: half ? 0.9 : 1.8 };
  const nx = Math.round((L.up + L.down) * span / dx), ny = Math.round(L.y * span / dx), nz = Math.round(L.z * span / dx);
  const origin = [-L.down * span, -0.5 * L.y * span, half ? 0 : -0.5 * L.z * span];
  const uLat = (speed * dtExact) / dx;
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
  const { surfaces, nc, ns } = B.birdSurfaces(kin, { design, dx, dt: dtExact, origin, half, freq, amp, panel: o.panel ?? 4 });
  const parts = [...new Set(surfaces.map((s) => s.part))];
  const total = Math.round((o.wingbeats ?? 3) * stepsPerBeat);
  const sampleEvery = o.sampleEvery ?? Math.max(1, Math.round(stepsPerBeat / 64));
  const cfg = { ...grid, tau0, csSmag, inletU: [-uLat, 0, 0], u0: [-uLat, 0, 0], inletRampSteps: 0, sponge: { strength: 0.05, xLow: Math.round(0.25 * L.down * span / dx) } };
  const r = await runSeries(device, { name: o.name, grid, cfg, surfaces, bouzidi: o.bouzidi ?? true, relink, totalSteps: total, sampleEvery, parts, maxSeconds: o.maxSeconds ?? 1e9 });
  if (!r.done) return { done: false, n: r.n, total, mlups: r.mlups, errors: errors.splice(0) };
  const se = r.series;
  const fConv = (air.rho * dx ** 4) / dtExact ** 2, mConv = (air.rho * dx ** 5) / dtExact ** 2;
  const NAMES = Object.fromEntries(Object.entries(B.PART).map(([k, v]) => [v, k]));
  const ca = Math.cos(design.bodyAlpha), sa = Math.sin(design.bodyAlpha);
  const partsOut = {};
  const add = (name, Fs, Ms, flipZ) => {
    partsOut[name] = {
      lift_N: Fs.map((v) => v[1] * fConv), thrust_N: Fs.map((v) => v[0] * fConv), side_N: Fs.map((v) => (flipZ ? -1 : 1) * v[2] * fConv),
      Mx_Nm: Ms.map((v) => (flipZ ? -1 : 1) * v[0] * mConv), My_Nm: Ms.map((v) => (flipZ ? -1 : 1) * v[1] * mConv), Mz_Nm: Ms.map((v) => v[2] * mConv),
      Fx_body_N: Fs.map((v) => (ca * v[0] + sa * v[1]) * fConv), Fy_body_N: Fs.map((v) => (-sa * v[0] + ca * v[1]) * fConv),
    };
  };
  parts.forEach((p, k) => add(NAMES[p], se.F[k], se.M[k], false));
  if (half) { const w = parts.indexOf(B.PART.right_wing); add('left_wing', se.F[w], se.M[w], true); }
  // Totals. Half model: side force, roll and yaw are zero by symmetry. The left wing is the right
  // mirrored above; body and tail see only their in-domain half, so they are doubled here.
  const names = Object.keys(partsOut);
  const nS = se.t.length;
  const sumOf = (field) => { const a = new Array(nS).fill(0); for (const n of names) for (let i = 0; i < nS; i++) a[i] += partsOut[n][field][i] * (half && (n === 'body' || n === 'tail') ? 2 : 1); return a; };
  const total_ = { lift_N: sumOf('lift_N'), thrust_N: sumOf('thrust_N'), Mz_Nm: sumOf('Mz_Nm') };
  const tSec = se.t.map((t) => t * dtExact);
  const perBeat = [];
  for (let b = 0; b < Math.floor(total / stepsPerBeat); b++) {
    const idx = se.t.map((t, i) => (t >= b * stepsPerBeat && t < (b + 1) * stepsPerBeat ? i : -1)).filter((i) => i >= 0);
    const m = (a) => idx.reduce((p, i) => p + a[i], 0) / idx.length;
    perBeat.push({ beat: b, lift_N: m(total_.lift_N), thrust_N: m(total_.thrust_N), Mz_Nm: m(total_.Mz_Nm), wing_lift_N: m(partsOut.right_wing.lift_N) * 2, wing_thrust_N: m(partsOut.right_wing.thrust_N) * 2 });
  }
  const Re = (speed * mac) / air.nu;
  const cf = 0.058 * Re ** -0.2;
  const uTau = speed * Math.sqrt(cf / 2);
  return {
    done: true, design: design.name, run: o.name, wingbeats, half,
    condition: { altitude_m: 1800, T_C: 10, rho: air.rho, nu: air.nu },
    params: { dx, dt: dtExact, speed, freq, amp, bodyAlpha: design.bodyAlpha, tailDeflection: design.tailDeflection, sched: design.sched, law: design.law.kind ?? 'crank-rocker', trimSource: design.trimSource, csSmag, relink, tau0, uLat, peak_wall_speed: vWall, peak_lattice_speed: ((speed + vWall) * dtExact) / dx, span, area, mean_chord: cMean, mac, Re_mac: Re, chord_cells: mac / dx, panels: [nc, ns], estimated_y_plus_first_cell: ((dx / 2) * uTau) / air.nu },
    grid: { nx, ny, nz, cells: r.cells, nb: r.nb, domain_m: [nx * dx, ny * dx, nz * dx], origin_m: origin },
    cost: { steps_per_wingbeat: stepsPerBeat, mlups: r.mlups, seconds: se.ms / 1000, seconds_per_wingbeat: (se.ms / 1000 / total) * stepsPerBeat, frame_ms_per_step: se.frameMs / total, max_links: se.maxLinks, lock_holds: se.segments },
    adapter: adapterInfo, t_s: tSec, phase_rad: tSec.map((t) => 2 * Math.PI * freq * t), parts: partsOut, total: total_, per_wingbeat: perBeat, errors: errors.splice(0),
    limits: 'Supported: comparing designs run at the same dx and settings; cycle-resolved lift and the phase of lift and thrust; wake and tip-vortex structure at scales of several cells. Not supported: absolute drag or thrust (no wall model, first-cell y+ given above, laminar and transitional boundary layers and separation onset unresolved), stall onset, Reynolds-number trends. Evidence: out/verification/V6.json.',
  };
}
