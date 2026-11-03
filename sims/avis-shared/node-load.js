// Node-only loading of the design and the catalogue entries it names. The design is read as the bytes on disk,
// with the sha256 from src/design/io.js, so a render can say exactly which design file it drew.
import { readFileSync, existsSync, readdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { fileSha256 } from '../../src/design/io.js';
import { partsIndex, prepareFilm } from './model.js';

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

/** The catalogue entries as the page bundles them (film/gen_data.mjs): long notes stay in the catalogue, the numbers
 * travel. A part's printed size always travels, however long, so the page draws it as the GLB does. */
export function bundleParts(parts) {
  const keep = (k, v) => /^dimension/.test(k) || !((typeof v === 'string' && v.length > 90) || /note|basis_text|rating_basis|warning/i.test(k));
  const terse = (o) => (Array.isArray(o) ? o.map(terse) : o && typeof o === 'object' ? Object.fromEntries(Object.entries(o).filter(([k, v]) => keep(k, v)).map(([k, v]) => [k, terse(v)])) : o);
  return terse(parts);
}

export function loadModel(name = 'larus-gl-2') {
  const { design, path, sha256 } = loadDesign(name);
  const parts = loadParts(design);
  return { model: prepareFilm(design, parts), design, parts, path, sha256 };
}

/** The gate files as they are, newest per gate, for this design and the method gates, each mission gate with its
 * uniform eta_s 0.6 comparison (out/verification/eta0.6/, newest per gate) as `at06`. Never edited. */
export function loadVerification(designName) {
  const dir = join(ROOT, 'out', 'verification');
  const official = newestGates(dir, designName);
  const at06 = new Map(newestGates(join(dir, 'eta0.6'), designName).map((g) => [g.gate, g]));
  return official.map((g) => (at06.has(g.gate) ? { ...g, at06: at06.get(g.gate) } : g));
}

function newestGates(dir, designName) {
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
