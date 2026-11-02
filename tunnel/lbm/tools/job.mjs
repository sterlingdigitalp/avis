// Run page jobs from JSON and write each result to a file. Exit 75 if any job ran out of its budget,
// so the caller can start it again under a fresh GPU lock.
//   node tools/job.mjs <out.json> '[{"job":"runPlate","args":{...}}, ...]'
import fs from 'node:fs';
import path from 'node:path';
import { openPage, cleanup, watchdog } from '../src/harness.mjs';
const budget = +(process.env.JOB_SECONDS ?? 520);
watchdog((budget + 60) * 1000);
const started = Date.now();
const [outPath, list] = process.argv.slice(2);
const jobs = JSON.parse(list);
const results = fs.existsSync(outPath) ? JSON.parse(fs.readFileSync(outPath, 'utf8')) : {};
const { page } = await openPage();
await page.evaluate(async () => { window.jobs = await import('/tunnel/lbm/src/jobs.mjs'); });
let incomplete = false;
for (const { job, key, args, needsSeconds = 15 } of jobs) {
  const k = key ?? JSON.stringify([job, args]);
  if (results[k]?.done !== false && results[k]) continue;
  const left = budget - (Date.now() - started) / 1000;
  // Jobs that cannot pause (they compare against the CPU) say how long they need up front.
  if (left < needsSeconds) { incomplete = true; break; }
  const r = await page.evaluate(({ job, args, left }) => window.jobs[job]({ ...args, maxSeconds: left }), { job, args, left });
  results[k] = r;
  fs.mkdirSync(path.dirname(outPath), { recursive: true });
  fs.writeFileSync(outPath, JSON.stringify(results, null, 0));
  console.log(k.slice(0, 120), r.done === false ? `paused at ${r.n}/${r.total ?? '?'}` : 'done');
  if (r.done === false) { incomplete = true; break; }
}
cleanup();
setTimeout(() => process.exit(incomplete ? 75 : 0), 1000);
