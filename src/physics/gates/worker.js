import { parentPort } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import * as sc from '../scenarios.js';

const cache = new Map();
function design(path) {
  if (!cache.has(path)) cache.set(path, JSON.parse(readFileSync(path, 'utf8')));
  return cache.get(path);
}

const TASKS = {
  M1: (a) => sc.runM1(design(a.design), a.opts),
  M2: (a) => sc.runM2(design(a.design), a.opts),
  M3: (a) => sc.runM3(design(a.design), a.opts),
  M4: (a) => sc.runM4Seed(design(a.design), a.seed, a.opts),
  M5: (a) => sc.runM5(design(a.design), a.opts),
};

parentPort.on('message', ({ i, job }) => {
  try {
    const r = TASKS[job.task](job.args);
    if (!job.keepLog) delete r.log;
    parentPort.postMessage({ i, result: r });
  } catch (e) {
    parentPort.postMessage({ i, error: String(e && e.stack ? e.stack : e) });
  }
});
