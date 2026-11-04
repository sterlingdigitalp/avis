// Gate result writer. Only the script that measured a value writes its file.
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

export const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..', '..');

export function commandLine() {
  return process.argv.map((a, i) => (i === 0 ? 'node' : a.startsWith(ROOT) ? a.slice(ROOT.length + 1) : a)).join(' ');
}

export function writeJson(rel, data) {
  const p = join(ROOT, rel);
  mkdirSync(dirname(p), { recursive: true });
  writeFileSync(p, JSON.stringify(data, null, 1));
  return p;
}

/**
 * Round 17 (orchestrator, after the v24 film round): every gate record says whether its pass is counted. A pass that
 * rests on anything SPEC does not count (a joint outside the RC-practice ruling's scope, say) carries counted: false
 * and the reason; a record whose writer gives no verdict on it is counted: false too ("not stated"), so a missing check
 * can never read as counted.
 */
export function countedFields(notCountedBecause) {
  if (notCountedBecause === undefined) return { counted: false, not_counted_because: 'the gate writer did not state whether its pass is counted' };
  const why = [].concat(notCountedBecause ?? []).filter(Boolean);
  return why.length ? { counted: false, not_counted_because: why.join('; ') } : { counted: true, not_counted_because: null };
}

export function writeGate({ gate, design = null, passed, measured, limit, evidence, file = null, extra = {}, notCountedBecause = undefined }) {
  const rec = { gate, design, passed: !!passed, ...countedFields(notCountedBecause), measured, limit, evidence, ran_at: new Date().toISOString(), command: commandLine(), ...extra };
  const name = file ?? gate + (design ? `-${design}` : '');
  writeJson(`out/verification/${name}.json`, rec);
  return rec;
}
