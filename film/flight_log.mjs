#!/usr/bin/env node
// The corridor flight log the Blender shots fly, from the current design and the current flight model.
//
//   node film/flight_log.mjs [design-name] [--check]
//
// Writes out/flight/<design>-corridor.json exactly as src/physics/gates/corridor.mjs does (the same scenario: corridor
// at its real elevation, 12 m/s, mean wind [3, 0, 1] m/s with Dryden u_rms 1 m/s scaled at 30 m, seed 7), but without
// that script's climb runs or its evidence file in out/verification, which the film does not own. The log carries
// `inputs`: the sha256 of the design file, of the flight model's sources and of the corridor, so render.mjs can tell a
// stale log from a current one and a new design is a re-run, not a rework. --check prints whether it is current and
// exits 0 (current) or 3 (stale), writing nothing.
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');

/** The scenario as corridor.mjs flies it; film/test/flight-log.test.mjs checks these against that script's text. */
export const CORRIDOR_SCENARIO = { seed: 7, speed: 12, wind: { mean: [3, 0, 1], uRms: 1, agl: 30 } };

const sha = (buf) => createHash('sha256').update(buf).digest('hex');

/** The flight model's sources: every module under src/physics except the gate scripts (which only run it). */
export function physicsSources(root = ROOT) {
  const dir = path.join(root, 'src', 'physics');
  return fs.readdirSync(dir).filter((f) => f.endsWith('.js')).sort().map((f) => path.join(dir, f));
}

/** What the log is a function of. */
export function corridorInputs(design, root = ROOT) {
  const h = createHash('sha256');
  for (const f of physicsSources(root)) h.update(`${path.basename(f)}\0`).update(fs.readFileSync(f)).update('\0');
  return {
    design_sha256: sha(fs.readFileSync(path.join(root, 'data', 'designs', `${design}.json`))),
    physics_sha256: h.digest('hex'),
    corridor_sha256: sha(fs.readFileSync(path.join(root, 'data', 'env', 'corridor.json'))),
    scenario: CORRIDOR_SCENARIO,
    by: 'film/flight_log.mjs',
  };
}

export const corridorLogPath = (design, root = ROOT) => path.join(root, 'out', 'flight', `${design}-corridor.json`);

/** True when the log on disk was flown from exactly these inputs. Reads only the head of the file. */
export function isCurrent(file, inputs) {
  if (!fs.existsSync(file)) return false;
  const m = readInputs(file);
  return !!m && ['design_sha256', 'physics_sha256', 'corridor_sha256'].every((k) => m[k] === inputs[k]) && JSON.stringify(m.scenario) === JSON.stringify(inputs.scenario);
}

/** The `inputs` block of a log; this module writes it before the columns, so the first 4 kB hold it. */
export function readInputs(file) {
  const fd = fs.openSync(file, 'r');
  try {
    const buf = Buffer.alloc(4096);
    const n = fs.readSync(fd, buf, 0, buf.length, 0);
    const head = buf.subarray(0, n).toString('utf8');
    const i = head.indexOf('"inputs"');
    if (i < 0) return null;
    const start = head.indexOf('{', i);
    let depth = 0;
    for (let k = start; k < head.length; k++) {
      if (head[k] === '{') depth++;
      else if (head[k] === '}' && --depth === 0) return JSON.parse(head.slice(start, k + 1));
    }
    return null;
  } finally {
    fs.closeSync(fd);
  }
}

/** Flies the corridor and returns the log, `inputs` first. */
export async function flyCorridor(design, root = ROOT) {
  const imp = (rel) => import(pathToFileURL(path.join(root, rel)).href);
  const { runCorridor } = await imp('src/physics/scenarios.js');
  const { createWind } = await imp('src/physics/wind.js');
  const d = JSON.parse(fs.readFileSync(path.join(root, 'data', 'designs', `${design}.json`), 'utf8'));
  const corridor = JSON.parse(fs.readFileSync(path.join(root, 'data', 'env', 'corridor.json'), 'utf8'));
  const S = CORRIDOR_SCENARIO;
  const wind = createWind({ ...S.wind, wingspan: d.geometry.span_m }, S.seed);
  const r = runCorridor(d, corridor, { seed: S.seed, wind, speed: S.speed });
  r.log.scenario = `${d.name}-corridor`;
  return { log: { inputs: corridorInputs(design, root), ...r.log }, completed: r.completed, measured: r.measured };
}

/**
 * Makes the design's corridor log current: 'current' when it already is, 'stale' when only checking, 'written' after
 * flying it again (to a temporary file renamed over the log, so a killed run never leaves half a log for the film).
 */
export async function ensureCorridorLog(design, { check = false, force = false, root = ROOT } = {}) {
  const file = corridorLogPath(design, root);
  const inputs = corridorInputs(design, root);
  if (!force && isCurrent(file, inputs)) return { status: 'current', file, inputs };
  if (check) return { status: 'stale', file, inputs };
  const t0 = Date.now();
  const { log, completed, measured } = await flyCorridor(design, root);
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(`${file}.tmp`, JSON.stringify(log, null, 1));
  fs.renameSync(`${file}.tmp`, file);
  const t = log.columns.t;
  return { status: 'written', file, inputs, span_s: t.at(-1) - t[0], completed, measured, fly_s: (Date.now() - t0) / 1000 };
}

async function main() {
  const args = process.argv.slice(2);
  const design = args.find((x) => !x.startsWith('--')) ?? 'larus-gl-2';
  const r = await ensureCorridorLog(design, { check: args.includes('--check'), force: args.includes('--force') });
  const rel = path.relative(ROOT, r.file);
  if (r.status === 'written') console.log(`${rel}: ${r.span_s.toFixed(2)} s, completed ${r.completed}, ${r.fly_s.toFixed(0)} s to fly; ${JSON.stringify(r.measured)}`);
  else console.log(`${rel}: ${r.status} (design ${r.inputs.design_sha256.slice(0, 12)}, physics ${r.inputs.physics_sha256.slice(0, 12)})`);
  if (r.status === 'stale') process.exit(3);
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) await main();
