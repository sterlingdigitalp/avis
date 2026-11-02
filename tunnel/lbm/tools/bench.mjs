// MLUPS measurements. Usage: node tools/bench.mjs '<json array of bench args>' [--features=...]
import { openPage, cleanup, watchdog } from '../src/harness.mjs';
watchdog(+(process.env.BENCH_MS ?? 530000));
const cases = JSON.parse(process.argv[2]);
const { page } = await openPage();
await page.evaluate(async () => { window.jobs = await import('/tunnel/lbm/src/jobs.mjs'); });
console.log(JSON.stringify(await page.evaluate(() => window.jobs.adapter())));
for (const c of cases) {
  const r = await page.evaluate((c) => window.jobs.bench(c), c);
  console.log(JSON.stringify(r));
}
cleanup();
setTimeout(() => process.exit(0), 1000);
