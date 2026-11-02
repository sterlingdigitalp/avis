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

const results = existsSync(dir)
  ? readdirSync(dir).filter((f) => f.endsWith('.json')).map((f) => JSON.parse(readFileSync(join(dir, f), 'utf8')))
  : [];

// V5 and the mission gates are per design; a method gate with design null applies to all.
function find(gate) {
  const hits = results.filter((r) => r.gate === gate && (r.design == null || design == null || r.design === design));
  return hits.sort((a, b) => String(b.ran_at).localeCompare(String(a.ran_at)))[0] ?? null;
}

let ok = true;
console.log(`design: ${design ?? '(any)'}`);
for (const gate of [...METHOD, ...MISSION]) {
  const r = find(gate);
  if (!r) ok = false;
  else if (!r.passed) ok = false;
  const state = !r ? 'MISSING' : r.passed ? 'pass' : 'FAIL';
  console.log(`${gate.padEnd(3)} ${state.padEnd(8)} ${r?.design ?? ''} ${r ? JSON.stringify(r.measured).slice(0, 110) : ''}`);
}
console.log(ok ? 'ALL GATES PASS' : 'NOT ALL GATES PASS');
process.exit(ok ? 0 : 1);
