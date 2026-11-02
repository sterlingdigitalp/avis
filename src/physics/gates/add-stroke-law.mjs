#!/usr/bin/env node
// Adds the stroke-law angle table to flight logs written before logs carried it. Flight columns are untouched.
import { readFileSync, writeFileSync, readdirSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { strokeLawTable } from '../scenarios.js';
import { ROOT } from './common.js';

const dir = join(ROOT, 'out/flight');
for (const f of readdirSync(dir).filter((x) => x.endsWith('.json'))) {
  const p = join(dir, f);
  const log = JSON.parse(readFileSync(p, 'utf8'));
  if (log.stroke_law || !log.design) continue;
  const dp = join(ROOT, `data/designs/${log.design}.json`);
  if (!existsSync(dp)) continue;
  log.stroke_law = strokeLawTable(JSON.parse(readFileSync(dp, 'utf8')));
  writeFileSync(p, JSON.stringify(log));
  console.log('added stroke_law to', f);
}
