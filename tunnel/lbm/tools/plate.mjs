// Run a list of plate cases and print one line of results each. For exploration and for V6.
import { openPage, cleanup, watchdog } from '../src/harness.mjs';
watchdog(+(process.env.PLATE_MS ?? 570000));
const cases = JSON.parse(process.argv[2]);
const { page } = await openPage();
await page.evaluate(async () => { window.jobs = await import('/tunnel/lbm/src/jobs.mjs'); });
for (const c of cases) {
  const r = await page.evaluate((c) => window.jobs.runPlate(c), c);
  console.log(JSON.stringify({ c: r.c, Re: r.Re, k: r.k, h: r.h0c, al: r.alpha0, kind: r.kind, bz: r.bouzidi, tau: +r.tau0.toFixed(4), cells: r.cells, steps: r.steps, s: +r.seconds.toFixed(1), mlups: Math.round(r.mlups), frameMs: +r.frameMsPerStep.toFixed(2), amp: +r.cl.amp.toFixed(4), prev: +r.cl.prevAmp.toFixed(4), theo: +r.theo.amp.toFixed(4), ampErr: +r.ampErrPct.toFixed(1), ph: +r.cl.phaseDeg.toFixed(1), thPh: +r.theo.phaseDeg.toFixed(1), phErr: +r.phaseErrDeg.toFixed(1), mean: +r.cl.mean.toFixed(4), cd: +r.meanCd.toFixed(4), err: r.errors }));
}
cleanup();
setTimeout(() => process.exit(0), 1000);
