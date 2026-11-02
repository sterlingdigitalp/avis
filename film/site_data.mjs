#!/usr/bin/env node
// Data files beside the pages in out/film/site: the Bow Valley backdrop from out/env/web (copied as is), and every
// flight log of the design from out/flight decimated to 30 Hz for playback (phase unwrapped, nothing else changed).
//   node film/site_data.mjs [design]
import fs from 'node:fs';
import path from 'node:path';
import { ROOT } from '../sims/avis-shared/node-load.js';

const design = process.argv[2] ?? 'larus-gl-2';
const SITE = path.join(ROOT, 'out', 'film', 'site');
const envSrc = path.join(ROOT, 'out', 'env', 'web');
fs.mkdirSync(path.join(SITE, 'env'), { recursive: true });
fs.mkdirSync(path.join(SITE, 'data'), { recursive: true });
if (fs.existsSync(envSrc)) for (const f of fs.readdirSync(envSrc)) fs.copyFileSync(path.join(envSrc, f), path.join(SITE, 'env', f));

const flights = [];
const dir = path.join(ROOT, 'out', 'flight');
const HZ = 30;
for (const f of fs.existsSync(dir) ? fs.readdirSync(dir).filter((x) => x.startsWith(`${design}-`) && x.endsWith('.json')) : []) {
  const L = JSON.parse(fs.readFileSync(path.join(dir, f), 'utf8'));
  const c = L.columns;
  const n = c.t.length;
  const out = { design: L.design, scenario: L.scenario, claim: L.claim, hz: HZ, t: [], pos: [], quat: [], phase: [], amp: [], twistL: [], twistR: [], tailL: [], tailR: [], airspeed: [], power: [], battery: [] };
  let unw = c.phase[0], prev = c.phase[0];
  let next = c.t[0];
  for (let k = 0; k < n; k++) {
    let d = c.phase[k] - prev;
    d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
    unw += k ? d : 0;
    prev = c.phase[k];
    if (c.t[k] + 1e-9 < next) continue;
    next += 1 / HZ;
    const r = (v, p = 4) => +v.toFixed(p);
    out.t.push(r(c.t[k], 3));
    out.pos.push(c.pos[k].map((v) => r(v, 3)));
    out.quat.push(c.quat[k].map((v) => r(v, 5)));
    out.phase.push(r(unw));
    out.amp.push(r(c.amplitude[k]));
    out.twistL.push(r(c.twist_l[k]));
    out.twistR.push(r(c.twist_r[k]));
    out.tailL.push(r(c.tail[k][0]));
    out.tailR.push(r(c.tail[k][1]));
    out.airspeed.push(r(c.airspeed[k], 2));
    out.power.push(r(c.power_w[k], 1));
    out.battery.push(r(c.battery_pct[k], 1));
  }
  const name = f.replace(/\.json$/, '');
  fs.writeFileSync(path.join(SITE, 'data', `flight-${name}.json`), JSON.stringify(out));
  flights.push({ name, scenario: L.scenario, seconds: +(c.t[n - 1] - c.t[0]).toFixed(1), file: `data/flight-${name}.json` });
}
fs.writeFileSync(path.join(SITE, 'data', 'flights.json'), JSON.stringify(flights));
console.log(`site data: ${flights.length} logs, env ${fs.existsSync(envSrc) ? 'copied' : 'missing'}`);
