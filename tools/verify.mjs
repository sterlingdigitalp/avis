#!/usr/bin/env node
// Aggregates out/verification/*.json into a table. Reads only; never edits a result.
// Exit 0 only if every gate has a file and passed, for the method gates and the named design.
//   node tools/verify.mjs [design-name]
import {readdirSync, readFileSync, existsSync} from 'node:fs';
import {join, dirname} from 'node:path';
import {fileURLToPath} from 'node:url';

const root = dirname(dirname(fileURLToPath(import.meta.url)));
const dir = join(root, 'out', 'verification');
const design = process.argv[2] ?? null;
const METHOD = ['V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7'];
const MISSION = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8'];

const readDir = (d) => existsSync(d)
  ? readdirSync(d).filter((f) => f.endsWith('.json')).map((f) => JSON.parse(readFileSync(join(d, f), 'utf8')))
  : [];
const results = readDir(dir);
// The same mission gates flown with eta_s at the 0.6 default everywhere (SPEC "Below the smallest
// measured amplitude"): a verdict is final only if it is the same there. Copied unchanged from the
// scratch run that measured them; see out/verification/eta0.6/PROVENANCE.txt.
const at06 = readDir(join(dir, 'eta0.6'));

// V5 and the mission gates are per design; a method gate with design null applies to all.
function find(gate, pool = results) {
  const hits = pool.filter((r) => r.gate === gate && (r.design == null || design == null || r.design === design));
  return hits.sort((a, b) => String(b.ran_at).localeCompare(String(a.ran_at)))[0] ?? null;
}

// A mission verdict that differs at 0.6 is not final, and a pass that is not final is not a pass.
function verdict(gate, r) {
  if (!r) return 'MISSING';
  // Gate files carry no design version, so a comparison older than the official run is stale.
  const found = MISSION.includes(gate) ? find(gate, at06) : null;
  const alt = found && String(found.ran_at) >= String(r.ran_at) ? found : null;
  if (MISSION.includes(gate) && !alt) return r.passed ? 'pass?' : 'FAIL';
  if (alt && alt.passed !== r.passed) return 'NOT FINAL';
  return r.passed ? 'pass' : 'FAIL';
}

let ok = true;
console.log(`design: ${design ?? '(any)'}`);
console.log('pass? = no uniform-0.6 comparison on record; NOT FINAL = the verdict differs at uniform eta_s 0.6');
for (const gate of [...METHOD, ...MISSION]) {
  const r = find(gate);
  const state = verdict(gate, r);
  if (state !== 'pass') ok = false;
  console.log(`${gate.padEnd(3)} ${state.padEnd(10)} ${r?.design ?? ''} ${r ? JSON.stringify(r.measured).slice(0, 110) : ''}`);
}
console.log(ok ? 'ALL GATES PASS' : 'NOT ALL GATES PASS');
process.exit(ok ? 0 : 1);
