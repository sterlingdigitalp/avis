// A small worker_threads pool for sweeps. At most 6 workers: other jobs share the machine.
// Each job is { task, args }; a worker imports ./worker.js and runs TASKS[task](args).
import { Worker } from 'node:worker_threads';
import { cpus } from 'node:os';

export const MAX_WORKERS = 6;

export async function runPool(jobs, { workers = +(process.env.AVIS_WORKERS ?? MAX_WORKERS), onResult = null } = {}) {
  const n = Math.max(1, Math.min(workers, MAX_WORKERS, cpus().length - 1, jobs.length));
  const results = new Array(jobs.length);
  let next = 0;
  let done = 0;
  await new Promise((resolve, reject) => {
    const pool = [];
    const feed = (w) => {
      if (next >= jobs.length) {
        w.terminate();
        return;
      }
      const i = next++;
      w.busy = i;
      w.postMessage({ i, job: jobs[i] });
    };
    for (let k = 0; k < n; k++) {
      const w = new Worker(new URL('./worker.js', import.meta.url));
      w.on('message', ({ i, result, error }) => {
        if (error) results[i] = { error };
        else results[i] = result;
        done++;
        if (onResult) onResult(i, results[i], done, jobs.length);
        if (done === jobs.length) {
          for (const p of pool) p.terminate();
          resolve();
        } else feed(w);
      });
      w.on('error', reject);
      pool.push(w);
      feed(w);
    }
  });
  return results;
}
