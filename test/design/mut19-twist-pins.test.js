// Round 19 mutation follow-up (twist-drive.js drawServo, the frame pins): the four pins hold the twist servo's case in
// its frame, so each touches the case (zero gap, no overlap) and none comes within the running 0.5 mm of the servo's
// arm; the top pin's crown stands 1.5 mm above the case's lower face (the round-19 rule), so it overlaps the case's
// height by 1.5 mm. Measured on the drawn solids at the drive pose of phase 0 (clearance.js gap2/gap3, the clearance
// check's own distance).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { solidMassParams } from '../../src/design/round11.mjs';
import { fullAssembly, gap2, gap3 } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));

test('twist mut19: each frame pin touches the servo\'s case, clear of its arm by 0.5 mm', () => {
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24.params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm, lay } = fullAssembly(d, catalog);
  const q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal), sh = (s) => (s.at ? s.at(q0) : s.shape);
  const gap = (a, b) => gap3(a.x, b.x, gap2(sh(a), sh(b)));
  for (const t of ['R', 'L']) {
    const part = (id) => { const q = asm.parts.find((r) => r.id === id); assert.ok(q, id); return q; };
    const pins = part(`twist frame pins ${t}`).solids, cs = part(`twist servo ${t}`).solids.find((s) => s.name === 'case'), arm = part(`twist servo arm ${t}`).solids;
    assert.equal(pins.length, 4);
    for (const pin of pins) {
      const g = gap(pin, cs);
      assert.ok(Math.abs(g) <= 1e-7, `${t} ${pin.name}: ${(g * 1000).toFixed(4)} mm from the case (touching expected)`);
      for (const a of arm) assert.ok(gap(pin, a) >= 0.0005 - 1e-9, `${t} ${pin.name} ~ arm ${a.name}: ${(gap(pin, a) * 1000).toFixed(3)} mm`);
    }
  }
});
