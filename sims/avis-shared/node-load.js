// Node-only loading of the design and the catalogue entries it names. The design is read as the bytes on disk,
// with the sha256 from src/design/io.js, so a render can say exactly which design file it drew.
import { readFileSync, existsSync, readdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { fileSha256 } from '../../src/design/io.js';
import { partsIndex, prepare } from './model.js';

export const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');

export function loadDesign(name = 'larus-gl-2') {
  const path = join(ROOT, 'data', 'designs', `${name}.json`);
  const design = JSON.parse(readFileSync(path, 'utf8'));
  return { design, path, sha256: fileSha256(path) };
}

export function loadParts(design) {
  const catalog = JSON.parse(readFileSync(join(ROOT, 'data', 'parts', 'catalog.json'), 'utf8'));
  return partsIndex(design, catalog);
}

export function loadModel(name = 'larus-gl-2') {
  const { design, path, sha256 } = loadDesign(name);
  const parts = loadParts(design);
  return { model: prepare(design, parts), design, parts, path, sha256 };
}

/** The gate files as they are, newest per gate, for this design and the method gates. Never edited. */
export function loadVerification(designName) {
  const dir = join(ROOT, 'out', 'verification');
  if (!existsSync(dir)) return [];
  const all = readdirSync(dir).filter((f) => f.endsWith('.json')).map((f) => {
    try {
      return { file: f, ...JSON.parse(readFileSync(join(dir, f), 'utf8')) };
    } catch {
      return null;
    }
  }).filter(Boolean);
  const keep = new Map();
  for (const r of all) {
    if (!r.gate || (r.design != null && r.design !== designName)) continue;
    const prev = keep.get(r.gate);
    if (!prev || String(r.ran_at) > String(prev.ran_at)) keep.set(r.gate, r);
  }
  return [...keep.values()];
}
