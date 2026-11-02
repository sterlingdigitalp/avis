#!/usr/bin/env node
// Applies data/calibration/<design>.json to a design and rebuilds its controller table on the calibrated model:
// trims, effectiveness, gains, and the free-flight trim. Run after V5 and before the mission gates.
//   node src/design/finalize.mjs data/designs/larus-gl-2.json
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join, resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { writeDesignAtomic } from './io.js';
import { buildControl, refineTrimByFlight } from './trim.js';
import { buildBird } from '../physics/flight.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const path = resolve(process.argv[2]);
const d = JSON.parse(readFileSync(path, 'utf8'));
const calPath = join(ROOT, `data/calibration/${d.name}.json`);
if (existsSync(calPath)) {
  const cal = JSON.parse(readFileSync(calPath, 'utf8'));
  d.calibration = { file: `data/calibration/${d.name}.json`, source: cal.source, multipliers: cal.multipliers };
}
const speeds = d.control.trim.map((t) => t.V);
const old = d.control;
const sched = { amp: d.cruise.amp, sym: d.cruise.sym ?? 0, dyn: d.cruise.dyn, lead: d.cruise.lead ?? 0, V: old.Vcruise };
const ctl = buildControl(d, sched, { speeds, bird: buildBird(d) });
if (!ctl) {
  console.error('calibrated model has no trim table');
  process.exit(1);
}
d.control = ctl;
await refineTrimByFlight(d);
if (!d.control.trimConverged) {
  console.error(`${d.name}: no free-flight trim converged on the calibrated model: design fails`);
  process.exit(1);
}
const tf = (d.control.trimFlight ?? []).filter((r) => !r.unflyable);
if (tf.length) {
  const minP = tf.reduce((a, b) => (b.flight_power_w < a.flight_power_w ? b : a));
  d.control.Vcruise = minP.V;
  d.control.Vrough = Math.min(tf[tf.length - 1].V, minP.V + 3);
}
d.cruise.V = d.control.Vcruise;
writeDesignAtomic(path, d);
console.log(`${d.name}: calibrated ${JSON.stringify(d.calibration?.multipliers)}; cruise ${d.control.Vcruise} m/s, rough ${d.control.Vrough} m/s`);
