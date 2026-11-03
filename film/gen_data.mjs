#!/usr/bin/env node
// Writes out/film/gen/model-data.json: the current design, the catalogue entries it names, the gate files as they
// are, and the fit report, for the page and the film to bundle. Re-run whenever the design or a gate changes.
//   node --import ./film/lib/three-hooks.mjs film/gen_data.mjs [design-name]
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { loadModel, loadVerification, bundleParts, ROOT } from '../sims/avis-shared/node-load.js';
import { leftRightAsymmetry } from '../sims/avis-shared/model.js';
import { gateLine, gateFailures } from '../sims/avis-shared/numbers.js';

const name = process.argv[2] ?? 'larus-gl-2';
const { model, design, parts, sha256 } = loadModel(name);
// The page needs the verdict, one line in the gate's own numbers, what a failed gate failed (in the file's own words),
// and the top-level scalar measurements.
const scalars = (o) => Object.fromEntries(Object.entries(o ?? {}).filter(([, v]) => typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string'));
const gate = (g) => ({ gate: g.gate, design: g.design, passed: g.passed, line: gateLine(g), failures: gateFailures(g), measured: scalars(g.measured), ran_at: g.ran_at, file: g.file });
// each mission gate with its uniform eta_s 0.6 comparison, for the verdict (numbers.js gateVerdict, tools/verify.mjs's rule)
const gates = loadVerification(name).map((g) => ({ ...gate(g), ...(g.at06 ? { at06: { ...gate(g.at06), file: `eta0.6/${g.at06.file}` } } : {}) }));
const asym = [model.law.ampMin, model.law.ampNominal, model.law.ampMax].map((a) => ({ amp: a, max_left_minus_right_rad: leftRightAsymmetry(model, a) }));
const out = {
  design_name: name,
  design_sha256: sha256,
  design_version: design.version,
  generated_from: `data/designs/${name}.json`,
  design,
  parts: bundleParts(parts),
  gates,
  fit: model.fit,
  layout_assumptions: model.layout.assumptions,
  asymmetry: asym,
};
const dir = join(ROOT, 'out', 'film', 'gen');
mkdirSync(dir, { recursive: true });
writeFileSync(join(dir, 'model-data.json'), JSON.stringify(out));
console.log(`model-data.json: ${name} v${design.version} ${sha256.slice(0, 12)}, ${gates.length} gates, ${model.fit.issues.length} fit issues`);
