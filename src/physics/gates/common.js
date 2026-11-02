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

export function writeGate({ gate, design = null, passed, measured, limit, evidence, file = null, extra = {} }) {
  const rec = { gate, design, passed: !!passed, measured, limit, evidence, ran_at: new Date().toISOString(), command: commandLine(), ...extra };
  const name = file ?? gate + (design ? `-${design}` : '');
  writeJson(`out/verification/${name}.json`, rec);
  return rec;
}
