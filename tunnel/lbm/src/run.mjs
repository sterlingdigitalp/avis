// Time-series driver shared by every long run: geometry frames, force sampling, time budget,
// checkpoint and resume. Runs in the page. A run that hits its budget checkpoints and reports
// done:false, and the caller starts it again under a fresh GPU lock hold.
import { GpuLbm } from './gpu.mjs';
import { BoundaryEngine } from './geometry.mjs';

async function loadSeries(path) {
  const r = await fetch(`/${path}`, { cache: 'no-store' });
  return r.ok ? r.json() : null;
}

/**
 * @param o {name, grid, cfg, surfaces, bouzidi, relink, totalSteps, sampleEvery, parts, maxSeconds,
 *           massFix, massEvery}
 * Times are lattice steps. A sample's time is the centre of its averaging window.
 */
export async function runSeries(device, o) {
  const { name, grid, cfg, surfaces, bouzidi = true, relink = 1, totalSteps, sampleEvery, parts, maxSeconds = 1e9, massFix = false, massEvery = 0, resume = true } = o;
  const ckpt = `out/lbm/ckpt/${name}`;
  const eng = new BoundaryEngine(grid, surfaces, { bouzidi });
  const s = new GpuLbm(device, { ...cfg, mask: eng.mask ?? undefined });
  let series = { t: [], F: parts.map(() => []), M: parts.map(() => []), mass: [], ms: 0, frameMs: 0, gpuSteps: 0, maxLinks: 0, maxCells: 0, segments: 0 };
  let n = 0;
  const meta = resume ? await s.loadCheckpoint(ckpt) : null;
  if (meta) {
    n = meta.steps;
    series = (await loadSeries(`${ckpt}.series.json`)) ?? series;
    if (n > 0) {
      // Rebuild the geometry of the last frame so the next frame diffs against the right state.
      const last = n - 1 - ((n - 1) % relink);
      s.setFrame(eng.init(last + relink / 2));
    }
    await s.finish();
  } else {
    s.setFrame(eng.init(relink / 2));
    await s.finish();
  }
  series.segments++;
  // Leave time inside the hold to write the checkpoint: about 75 MB/s measured through the harness.
  const budget = maxSeconds - (s.n * 19 * 4) / 60e6 - 5;
  const t0 = performance.now();
  let frameMs = 0;
  let done = n >= totalSteps;
  let budgetHit = false;
  let pendingFix = 0;
  while (!done) {
    let steps = 0;
    const startN = n;
    for (let j = 0; j < sampleEvery && n < totalSteps; j++, n++, steps++) {
      if (n % relink === 0) {
        const a = performance.now();
        const fr = eng.frame(n + relink / 2);
        frameMs += performance.now() - a;
        s.setFrame(fr);
        if (fr.linkCount > series.maxLinks) series.maxLinks = fr.linkCount;
        if (fr.cellCount > series.maxCells) series.maxCells = fr.cellCount;
      }
      s.step(1);
      // Keep the queue short so a large grid cannot pile up seconds of unfinished work.
      if (s.n > 2e7 && steps % 8 === 7) await s.finish();
    }
    const F = await s.takeForces();
    series.t.push(startN + steps / 2);
    parts.forEach((p, k) => {
      series.F[k].push([F[p].F[0] / steps, F[p].F[1] / steps, F[p].F[2] / steps]);
      series.M[k].push([F[p].M[0] / steps, F[p].M[1] / steps, F[p].M[2] / steps]);
    });
    for (const p of F) series.budget = (series.budget ?? 0) + p.dMass;
    if (massEvery > 0 && Math.floor(n / massEvery) !== Math.floor(startN / massEvery)) {
      await s.finish();
      const m = await s.massMomentum();
      series.mass.push({ n, deltaMass: m.deltaMass, cells: m.cells, budget: series.budget, fixApplied: series.fixApplied ?? 0 });
    }
    if (massFix) {
      // Apply the budget only once it is large next to the f32 spacing of drho (about 1e-10 at
      // drho ~ 1e-3): a correction near that spacing is partly rounded away, always the same way.
      let budget = 0;
      for (const p of F) budget += p.dMass;
      const cells = eng.mask ? eng.mask.reduce((a, v) => a + (v === 255 ? 1 : 0), 0) : s.n;
      pendingFix -= budget / cells;
      if (Math.abs(pendingFix) > 2e-8) { s.drhoFix = pendingFix; series.fixApplied = (series.fixApplied ?? 0) + pendingFix * cells; pendingFix = 0; }
    }
    done = n >= totalSteps;
    if (!done && (performance.now() - t0) / 1000 > budget) { budgetHit = true; break; }
  }
  await s.finish();
  const segMs = performance.now() - t0;
  series.ms += segMs;
  series.frameMs += frameMs;
  series.gpuSteps = n;
  const out = { done, n, totalSteps, cells: s.n, nb: s.nb, stats: { ...eng.stats }, segMs };
  if (budgetHit) {
    await s.saveCheckpoint(ckpt, { name });
    await fetch(`/${ckpt}.series.json`, { method: 'PUT', body: JSON.stringify(series) });
  } else if (done && meta) {
    // Nothing further needs a many-gigabyte file on disk.
    for (const suffix of ['', '.json', '.series.json']) await fetch(`/${ckpt}${suffix}`, { method: 'DELETE' });
  }
  out.series = series;
  out.mlups = (s.n * (series.gpuSteps)) / (series.ms * 1000);
  s.destroy();
  return out;
}
