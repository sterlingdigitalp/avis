import { openPage, cleanup, watchdog } from '../src/harness.mjs';
watchdog(280000);
const cases = JSON.parse(process.argv[2]);
const { page } = await openPage();
await page.evaluate(async () => { window.jobs = await import('/tunnel/lbm/src/jobs.mjs'); });
for (const c of cases) {
  const r = await page.evaluate((c) => window.jobs.agree(c), c);
  const t = r.trace[r.trace.length - 1];
  console.log(JSON.stringify({ ...r, trace: r.trace.map((x) => [x.step, +x.maxAbs.toExponential(2), +x.meanAbs.toExponential(2), x.nonFinite]), lastF: { g: t.Fgpu, c: t.Fcpu }, lastM: { g: t.Mgpu, c: t.Mcpu } }));
}
cleanup();
setTimeout(() => process.exit(0), 1000);
