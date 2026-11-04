// Round 19 mutation follow-up (pod-structure.js sealRows / openDrag / openingIn, tail-mech.js guideParts): each of the
// exit seal's conditions refuses on its own, on the real pod with one stated figure broken at a time (deliberately
// broken inputs: a guide stated with no leak, a rod excursion past its clearance, a lug bond rated over more area than
// the drawn overlap, a lug off its former); and the through-flow term's smaller side counts the leaks.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { solidMassParams } from '../../src/design/round11.mjs';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { fullAssembly } from '../../src/design/clearance.js';
import * as PS from '../../src/design/pod-structure.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
function build() {
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos', 'tailMech', 'pod']) if (k in set) p[k] = set[k];
  p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm, lay } = fullAssembly(d, catalog);
  return { d, asm, lay };
}
let cached = null;
const base = () => (cached ??= build());
const record = (asm, d, lay) => PS.coveringRecord(asm, d, catalog, { lay, phases: 6 });
/** The assembly with guide R's seal changed by f. */
const withSeal = (asm, f) => ({ ...asm, parts: asm.parts.map((p) => (p.id === 'tail pushrod guide R' ? { ...p, seal: f(p.seal) } : p)) });
const sealRow = (r, t) => r.rows.find((q) => q.kind === 'exit seal' && q.name === `tail pushrod guide ${t}`);

test('seal mut19: each of the exit seal\'s conditions refuses on its own', () => {
  const { d, asm, lay } = base();
  const r0 = record(asm, d, lay), s0 = sealRow(r0, 'R');
  assert.equal(s0.ok, true);
  // a seal stated with no leak (an annulus is always there: the bore is larger than the rod)
  assert.equal(sealRow(record(withSeal(asm, (s) => ({ ...s, leak_m2: 0 })), d, lay), 'R').ok, false, 'no leak stated');
  // the rod's excursion at (and past) its clearance: it would touch the guide's bore
  assert.equal(sealRow(record(withSeal(asm, (s) => ({ ...s, excursion_m: s.clearance_m })), d, lay), 'R').ok, false, 'excursion = clearance');
  assert.equal(sealRow(record(withSeal(asm, (s) => ({ ...s, excursion_m: s.clearance_m * 0.999 })), d, lay), 'R').ok, true, 'excursion just inside');
  // the lug's bond rated over more than the drawn overlap
  const ov = s0.lug_overlap_mm2;
  assert.equal(sealRow(record(withSeal(asm, (s) => ({ ...s, lug_bond_mm2: ov * 1.01 })), d, lay), 'R').ok, false, 'rated over the drawn overlap');
  // the lug named on the wrong former: no face on it, no overlap
  const off = sealRow(record(withSeal(asm, (s) => ({ ...s, former: 'pod former tail kink' })), d, lay), 'R');
  assert.equal(off.lug_overlap_mm2, 0);
  assert.equal(off.ok, false);
  // guide L untouched by all of it
  assert.equal(sealRow(record(withSeal(asm, (s) => ({ ...s, leak_m2: 0 })), d, lay), 'L').ok, true);
});

test('seal mut19: the seal row reports its stated figures in mm and mm2', () => {
  const { d, asm, lay } = base(), s = sealRow(record(asm, d, lay), 'R');
  const p = asm.parts.find((q) => q.id === 'tail pushrod guide R').seal;
  near(s.leak_mm2, p.leak_m2 * 1e6, 0.005, 'leak');
  near(s.clearance_mm, p.clearance_m * 1000, 0.0005, 'clearance');
  near(s.excursion_mm, p.excursion_m * 1000, 0.0005, 'excursion');
});

test('seal mut19: through-flow on the smaller side counts the leaks with the windows (one big window, small paths)', () => {
  // two windows and a leak on a cylinder: the smaller side is everything but the largest path
  const R = 0.03, src = { cavity: 0.108, through: 0.85 };
  const loft = { kind: 'elliptic-loft', sections: [{ x_m: 0.3, y_m: 0, width_m: 2 * R, height_m: 2 * R }, { x_m: -0.3, y_m: 0, width_m: 2 * R, height_m: 2 * R }] };
  const big = { name: 'big', x: [-0.05, 0.05], polar: [0, 180], covered_by: null }, small = { name: 'small', x: [-0.25, -0.2], polar: [330, 10], covered_by: null };
  const aBig = R * Math.PI * 0.1, aSmall = R * (40 * Math.PI / 180) * 0.05, leak = 1e-5;
  const o = PS.openDrag({ loft, windows: [big, small] }, src, [{ name: 'annulus', area_m2: leak }]);
  near(o.through_CdA_m2, 0.85 * (aSmall + leak), 2e-6, 'the small window and the leak against the big one');
  near(o.cavity_CdA_m2, 0.108 * (aBig + aSmall), 2e-6, 'cavities: the windows only');
  near(o.CdA_m2, o.cavity_CdA_m2 + o.through_CdA_m2, 1e-9, 'the sum');
  // a leak larger than every window is the largest path itself
  const o2 = PS.openDrag({ loft, windows: [small] }, src, [{ name: 'annulus', area_m2: 0.01 }]);
  near(o2.through_CdA_m2, 0.85 * aSmall, 2e-6, 'the window against the larger leak');
  // without the source's through-flow figure, no through term
  assert.equal(PS.openDrag({ loft, windows: [big, small] }, { cavity: 0.108 }, [{ name: 'annulus', area_m2: leak }]).through_CdA_m2, 0);
});

test('seal mut19: unionCrossing is the x range and the shortest polar arc holding every crossing (wrap at 0 deg too)', () => {
  const near3 = (a, b, msg) => { near(a[0], b[0], 1e-9, `${msg} [0]`); near(a[1], b[1], 1e-9, `${msg} [1]`); };
  assert.equal(PS.unionCrossing([]), null);
  const one = PS.unionCrossing([{ x: [-0.2, -0.1], polar: [10, 50] }]);
  near3(one.x, [-0.2, -0.1], 'one x'); near3(one.polar, [10, 50], 'one polar');
  // across 0 deg: 350-10 and 5-20 -> 350-20
  const wrap = PS.unionCrossing([{ x: [0, 1], polar: [350, 10] }, { x: [0.5, 2], polar: [5, 20] }]);
  near3(wrap.x, [0, 2], 'wrap x'); near3(wrap.polar, [350, 20], 'wrap polar');
  // two apart: 0-10 and 100-110 -> the shorter arc holding both, 0-110 (not 100-10 the long way round)
  const two = PS.unionCrossing([{ x: [3, 4], polar: [100, 110] }, { x: [1, 2], polar: [0, 10] }]);
  near3(two.x, [1, 4], 'two x'); near3(two.polar, [0, 110], 'two polar');
});

test('seal mut19: unionCrossing finds the largest gap wherever it falls, the first one too', () => {
  // 340-0 and 200-210: sorted 0, 200 ... 210, 340 ... 357.5; the largest gap (0 -> 200) is the first, so the union is 200-0
  const u = PS.unionCrossing([{ x: [0, 1], polar: [340, 0] }, { x: [0, 1], polar: [200, 210] }]);
  near(u.polar[0], 200, 1e-9, 'from'); near(u.polar[1], 0, 1e-9, 'to');
});

// (round 19 lead: the drafted 'a crossing must lie wholly inside its window in x' test is dropped: after seal2 the pitot
// is held by its probe and its crossing is one sample at F 97, so no window test could be built on it; pod-structure.js
// openingIn's window x-end comparisons (mutants 772:36 / 772:63) stay unkilled: a stated gap.)

test('seal mut19: only the declared holder holds a rod; a part merely crossing the skin round it does not', () => {
  const { d, asm, lay } = base();
  // guide R renamed (not '^tail pushrod guide [RL]$'): it still crosses the skin round the rod, but is not the holder
  const renamed = { ...asm, parts: asm.parts.map((p) => (p.id === 'tail pushrod guide R' ? { ...p, id: 'tail pushrod sleeve R' } : p)) };
  const r = record(renamed, d, lay), rods = r.rows.filter((q) => q.kind === 'opening' && /^tail pushrod R\//.test(q.solid));
  assert.ok(rods.length > 0 && rods.some((q) => !q.ok && q.in === null), JSON.stringify(rods.slice(0, 2)));
});

test('seal mut19: a seal whose lug is not drawn refuses (no face, no overlap), and a seal stating no leak adds no leak path', () => {
  const { d, asm, lay } = base();
  const r = record(withSeal(asm, (s) => ({ ...s, lug: 'no such lug' })), d, lay), s = sealRow(r, 'R');
  assert.equal(s.lug_overlap_mm2, 0);
  assert.equal(s.ok, false);
  assert.equal(record(withSeal(asm, (s2) => ({ ...s2, leak_m2: 0 })), d, lay).open.leaks.length, 1, 'only guide L\'s leak');
});
