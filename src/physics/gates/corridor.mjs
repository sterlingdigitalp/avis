#!/usr/bin/env node
// Corridor scenario at the corridor's real elevation, plus the sustained climb each design can hold there.
//   node src/physics/gates/corridor.mjs data/designs/<name>.json
// Not a SPEC gate. Writes out/flight/<name>-corridor.json and out/verification/evidence/corridor-<name>.json.
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { runCorridor, runClimb } from '../scenarios.js';
import { createWind } from '../wind.js';
import { writeJson, ROOT } from './common.js';

const d = JSON.parse(readFileSync(resolve(process.argv[2]), 'utf8'));
const corridor = JSON.parse(readFileSync(join(ROOT, 'data/env/corridor.json'), 'utf8'));
const elev = corridor.elevation_m_at_origin;
const climbs = [];
// The best sustained climb over a few commanded climb rates: asking for too much drives the stroke into stall.
for (const V of [d.control.Vcruise, 12].filter((v, i, a) => a.indexOf(v) === i)) {
  let best = null;
  for (const c of [0.5, 1.0, 1.5, 2.5]) {
    const r = runClimb(d, { V, altitude_m: elev - 40, climbCmd: c });
    r.climb_cmd_m_s = c;
    if (!best || r.climb_m_s > best.climb_m_s) best = r;
  }
  climbs.push(best);
  console.log(`climb at ${V} m/s: ${best.climb_m_s.toFixed(2)} m/s (asked ${best.climb_cmd_m_s}), airspeed ${best.mean_airspeed_m_s.toFixed(1)}, ${best.mean_batt_A.toFixed(2)} A of ${best.current_limit_A}`);
}
const need = { climb_m_s: [1.1, 1.3], gradient: corridor.max_climb_gradient, at_speed_m_s: 12 };
const at12 = climbs.find((c) => c.V_target === 12);
// Turbulence scales at 30 m above ground, as in M4; at the 11 m minimum clearance Dryden's L_w = h gives very short scales.
const w = createWind({ mean: [3, 0, 1], uRms: 1, agl: 30, wingspan: d.geometry.span_m }, 7);
const r = runCorridor(d, corridor, { seed: 7, wind: w, speed: 12 });
r.log.scenario = `${d.name}-corridor`;
writeJson(`out/flight/${d.name}-corridor.json`, r.log);
writeJson(`out/verification/evidence/corridor-${d.name}.json`, {
  design: d.name, elevation_m_at_origin: elev, density_note: 'density at the corridor elevation with the design-condition temperature offset (10 C at 1800 m)',
  climbs, need, holds_climb_with_margin_at_12: at12 ? at12.climb_m_s >= 1.3 * 1.2 && at12.mean_airspeed_m_s > 11 : null,
  corridor_flight: { completed: r.completed, ...r.measured }, wind: 'mean 3.2 m/s, Dryden u_rms 1 m/s, seed 7',
  ran_at: new Date().toISOString(), note: 'not a SPEC gate',
});
console.log('corridor', r.completed, JSON.stringify(r.measured));
