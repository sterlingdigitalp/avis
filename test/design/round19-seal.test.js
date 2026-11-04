// Round 19, decision 2 ("Seal the pod's through-flow paths; don't change the drag figure"): the tail pushrods leave the
// cone through exit guides (a catalogue tube coaxial with each rod, bonded in a lug on the 'pushrod exits forward'
// former, the film closed round it), the pitot's window closed (the film's cut round the probe; the nose stretched a
// little more than the pack moves, round17.mjs NOSE_EXTRA_MM, so the pack's corner clears its chord), and the open-window drag
// (pod-structure.js openDrag) counts the through-flow from the paths the geometry has: the open windows and the seals'
// leaks (the annulus round each pushrod). These tests pin: the through-flow term on a cylinder whose window areas are
// known in closed form (an open exit brings it back, a sealed one leaves only its leak); the leak's annulus from the
// makers' published figures; the real round-17 pod sealed (and refused when an exit is cut open again or a guide is
// missing); the pushrod's excursion against its small-angle limit; the pushrod's pieces on the guide's breaks.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { solidMassParams } from '../../src/design/round11.mjs';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { fullAssembly } from '../../src/design/clearance.js';
import * as PS from '../../src/design/pod-structure.js';
import { pushrodPieces, pushrodExcursion } from '../../src/design/tail-mech.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

/** Round 17's set (with its all-moving tail and the exit guides) over v24's parameters, mass from solids. */
function build(over = {}) {
  const set = round17Set(over, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos', 'tailMech', 'pod']) if (k in set) p[k] = set[k];
  p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm, lay } = fullAssembly(d, catalog);
  return { d, asm, lay };
}
let cached = null;
const base = () => (cached ??= build());
const record = (asm, d, lay) => PS.coveringRecord(asm, d, catalog, { lay, phases: 6 });

// the leak: the guide's largest bore (Easy Composites 10 x 9 mm woven tube, ID 9 mm, its FAQ's "+/- 0.3mm on the OD and
// ID") round the pushrod's smallest section (CST CS-509, 4.95 mm on its spec sheet): pi/4 (9.3^2 - 4.95^2) mm2
const LEAK_MM2 = (Math.PI / 4) * (9.3 ** 2 - 4.95 ** 2);

test('through-flow on a cylinder: an open exit brings the term back, a sealed one leaves only its leak', () => {
  const R = 0.03, src = { cavity: 0.108, through: 0.85 };
  const loft = { kind: 'elliptic-loft', sections: [{ x_m: 0.3, y_m: 0, width_m: 2 * R, height_m: 2 * R }, { x_m: -0.3, y_m: 0, width_m: 2 * R, height_m: 2 * R }] };
  // a strip of a cylinder: R x its angle x its length (closed form, independent of loftArea)
  const slot = { name: 'slot', x: [-0.05, 0.05], polar: [0, 180], covered_by: null }, exit = { name: 'exit', x: [-0.25, -0.2], polar: [330, 10], covered_by: null };
  const aSlot = R * Math.PI * 0.1, aExit = R * (40 * Math.PI / 180) * 0.05;
  const open = PS.openDrag({ loft, windows: [slot, exit] }, src);
  near(open.windows[0].area_m2, aSlot, 2e-6, 'slot area'); near(open.windows[1].area_m2, aExit, 2e-6, 'exit area');
  near(open.through_CdA_m2, 0.85 * aExit, 2e-6, 'open: C_Di on the smaller side');
  near(open.cavity_CdA_m2, 0.108 * (aSlot + aExit), 2e-6, 'open: both windows cavities');
  const leak = [{ name: 'annulus', area_m2: LEAK_MM2 * 1e-6 }];
  const sealed = PS.openDrag({ loft, windows: [slot] }, src, leak);
  near(sealed.through_CdA_m2, 0.85 * LEAK_MM2 * 1e-6, 2e-6, 'sealed: only the annulus');
  near(sealed.cavity_CdA_m2, 0.108 * aSlot, 2e-6, 'sealed: the leak is no cavity');
  assert.deepEqual(sealed.leaks, leak);
  assert.equal(PS.openDrag({ loft, windows: [slot] }, src).through_CdA_m2, 0, 'sealed with no leak: no path, no term');
  // the leak alone opposite nothing: the interior has one path out, nothing flows through
  assert.equal(PS.openDrag({ loft, windows: [] }, src, leak).through_CdA_m2, 0);
  // the cavity and C_Di figures are the catalogue's, unchanged by round 19
  assert.deepEqual(PS.openDragSources(catalog), { cavity: 0.108, through: 0.85 });
});

test('the real pod sealed: no exit window, each pushrod through its guide, the annuli the only exit leaks', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay);
  assert.equal(r.passed, true, r.failed.join('; '));
  // the wing-root slots the only open window (round 19's second pass closed the pitot exit too): the through-flow is
  // the two annuli's alone, the cavity the slots'
  assert.deepEqual(r.open.windows.map((w) => w.name), ['wing roots and horn slots']);
  assert.equal(r.open.leaks.length, 2);
  for (const l of r.open.leaks) near(l.area_m2 * 1e6, LEAK_MM2, 0.01, l.name);
  near(r.open.through_CdA_m2, 0.85 * 2 * LEAK_MM2 * 1e-6, 2e-6, 'through-flow: the two annuli');
  near(r.open.cavity_CdA_m2, 0.108 * r.open.windows[0].area_m2, 2e-6, 'cavity: the slots');
  // every pushrod solid meeting the skin does so inside its own guide's crossing; the guides hold the film's edge
  const rods = r.rows.filter((q) => q.kind === 'opening' && q.opening === 'tail pushrod exits');
  assert.ok(rods.length >= 8 && rods.every((q) => q.ok && q.in.startsWith(`through tail pushrod guide ${q.solid.match(/^tail pushrod ([RL])/)[1]} `)), JSON.stringify(rods.filter((q) => !q.ok)));
  assert.ok(r.rows.filter((q) => q.kind === 'opening' && q.opening === 'tail pushrod guides').every((q) => q.ok && /held by the part/.test(q.in)));
  // the seal rows: the clearance from the published figures, (9 - 0.3 - 5.0) / 2 mm, above the rod's excursion
  const seals = r.rows.filter((q) => q.kind === 'exit seal');
  assert.equal(seals.length, 2);
  for (const s of seals) {
    near(s.clearance_mm, (9 - 0.3 - 5.0) / 2, 1e-6, `${s.name} clearance`);
    assert.ok(s.excursion_mm > 0.5 && s.excursion_mm < s.clearance_mm && s.lug_overlap_mm2 >= s.lug_bond_rated_mm2 && !s.window_open, JSON.stringify(s));
  }
  // the pitot leaves the nose through the film's cut round it; the pack's front right corner, under the film between
  // the nose's 314 and 345 deg rods, at least the containment's 0.5 mm below its chord at every x the record samples
  const pit = r.rows.filter((q) => q.kind === 'opening' && q.opening === 'pitot');
  assert.ok(pit.length > 0 && pit.every((q) => q.ok && /^held by the part/.test(q.in)), JSON.stringify(pit));
  const G = PS.podGeometry(d, catalog), bat = asm.parts.find((p) => p.id === 'packed battery').solids[0], bays = PS.podBays(G);
  let worst = Infinity;
  for (let k = 0; k <= 20; k++) {
    const x = bat.x[1] - (k / 20) * 0.01, b = bays.find((q) => x >= q.x[0] && x <= q.x[1]), pn = b?.panels.find((q) => q.arc[0] === 314 && q.arc[1] === 345);
    if (!pn) continue;
    for (const pt of bat.shape.pts) if (PS.inArc(PS.polarOf(PS.sectionAt(G.loft, x), pt), [314, 345])) worst = Math.min(worst, PS.chordDepth(PS.linePoint(pn.a.line, x), PS.linePoint(pn.b.line, x), pt));
  }
  assert.ok(worst >= 5e-4 && worst < 0.01, `the corner under the closed panel: ${worst}`);
});

test('an exit cut open again brings its through-flow back and fails its seal; a missing guide leaves the rod unheld', () => {
  const { d, asm, lay } = base(), r0 = record(asm, d, lay);
  const exitR = { name: 'tail pushrod exit R', between: ['pushrod exits forward', 'tail kink'], polar_deg: [-36, 5], openings: [], why: 'the round-18 cut, as a test' };
  const dd = structuredClone(d);
  dd.params.pod = { ...dd.params.pod, windows: [...PS.POD.windows, exitR] };
  const { asm: a2 } = fullAssembly(dd, catalog), r = record(a2, dd, lay);
  const aR = r.open.windows.find((q) => q.name === exitR.name)?.area_m2;
  assert.ok(aR > 0.001, `the exit is open: ${aR}`);
  near(r.open.through_CdA_m2 - r0.open.through_CdA_m2, 0.85 * aR, 3e-6, 'the exit\'s own through-flow');
  near(r.open.cavity_CdA_m2 - r0.open.cavity_CdA_m2, 0.108 * aR, 3e-6, 'and its cavity');
  assert.ok(r.failed.includes('exit seal: tail pushrod guide R') && !r.failed.includes('exit seal: tail pushrod guide L'), r.failed.join('; '));
  // without its guide the pushrod crosses closed film: not in a window, not held, through nothing
  const bare = { ...asm, parts: asm.parts.filter((p) => !/^tail pushrod guide( lug)? R$/.test(p.id)) }, rb = record(bare, d, lay);
  const unheld = rb.rows.filter((q) => q.kind === 'opening' && /^tail pushrod R\//.test(q.solid) && !q.ok);
  assert.ok(unheld.length > 0 && unheld.every((q) => q.in === null), JSON.stringify(unheld.slice(0, 2)));
  assert.equal(rb.open.leaks.length, 1, 'only the remaining guide leaks');
});

test('the pushrod\'s excursion in its guide: the small-angle limit r_arm theta sin(alpha), bounded by its ends\' arcs', () => {
  const { d, asm } = base(), { rows, G } = asm.tailMech, row = rows.find((q) => q.tag === 'R');
  const ra = Math.abs(row.zH - row.zs), R = Math.abs(row.nE), k = ra / R, xs = row.guide ? [row.guide.x[1], row.guide.x[0]] : null;
  assert.ok(xs, 'the round-17 design draws its guides');
  // the balls at zero (the servo's on its arm at x xs, the horn's at the pivot's x): the rod's angle to x
  const dz = row.B.p[0] - row.A.p[0], dy = row.B.p[1] - row.A.p[1], dx = G.xq - row.xs, sinA = Math.hypot(dz, dy) / Math.hypot(dx, dz, dy);
  const dl = 0.002, th = Math.asin(Math.sin(dl) / k);
  near(pushrodExcursion(row, G, dl, xs), ra * th * sinA, 0.02 * ra * th * sinA, 'a small travel: the line translated along x');
  const full = pushrodExcursion(row, G, d.tail.deflection_max_rad, xs), thM = Math.asin(Math.sin(d.tail.deflection_max_rad) / k);
  // the line's offset between its ends is at most the larger of its ends' own displacements (chords of their arcs);
  // the horn's angle within 0.1 rad of the drawn law's at full travel
  const ends = Math.max(2 * ra * Math.sin(thM / 2), 2 * R * Math.sin((d.tail.deflection_max_rad + 0.1) / 2));
  assert.ok(full > ra * thM * sinA * 0.9 && full < ends, `${full} vs ${ra * thM * sinA}, ${ends}`);
});

test('pushrodPieces: unchanged without fine breaks; with them each is a piece boundary, on the fine line between them', () => {
  const A = { x: -0.04, p: [0.024, -0.044] }, B = { x: -0.93, p: [0.013, 0.083] }, ro = 0.0025, ri = 0.00195;
  const plain = pushrodPieces(A, B, ro, ri, 0.005), e = 0.005 / 0.89, n = Math.ceil((0.89 * (1 - 2 * e)) / 0.02);
  assert.equal(plain.solids.length, n + 2, 'the two end pieces and n at most 20 mm');
  const fine = Array.from({ length: 11 }, (_, k) => -0.2 - 0.002 * k), Ft = { A: { x: -0.03, p: [0.024, -0.042] }, B: { x: -0.945, p: [0.013, 0.083] } };
  const pr = pushrodPieces(A, B, ro, ri, 0.005, 0.02, fine, Ft);
  const edges = new Set(pr.solids.flatMap((s) => s.x.map((x) => x.toFixed(9))));
  for (const x of fine) assert.ok(edges.has(x.toFixed(9)), `a break at ${x}`);
  const inZone = pr.solids.filter((s) => s.x[0] >= -0.22 - 1e-9 && s.x[1] <= -0.2 + 1e-9);
  assert.equal(inZone.length, 10);
  const onF = (x) => { const u = (x - Ft.A.x) / (Ft.B.x - Ft.A.x); return [Ft.A.p[0] + u * (Ft.B.p[0] - Ft.A.p[0]), Ft.A.p[1] + u * (Ft.B.p[1] - Ft.A.p[1])]; };
  for (const s of inZone) { const c = onF((s.x[0] + s.x[1]) / 2); near(Math.hypot(s.shape.c[0] - c[0], s.shape.c[1] - c[1]), 0, 1e-12, 'on the fine line'); }
  // the rod's true length is the sum of its pieces' (each on its own line)
  near(pr.solids.reduce((a, s) => a + s.tube.L, 0), plain.L, 3e-4, 'the length');
});

test('the guides and lugs in the BOM at their stated length and volume, their bonds rated, the guide after its pushrod', () => {
  const { d } = base(), lines = d.bom?.structure ?? d.bom?.lines ?? [];
  const P = d.params.tailMech.guide, find = (re) => (Array.isArray(lines) ? lines : []).find((l) => re.test(l.name));
  const g = find(/^tail pushrod guides, 2 off$/), l = find(/^tail pushrod guide lugs, 2 off$/);
  assert.ok(g && l, 'both lines');
  near(g.mass_g, 2 * 24.7 * P.L_mm / 1000, 1e-9, 'guides: 24.7 g/m (the maker\'s) x 2 lengths');
  near(l.mass_g, 2 * P.lug_mm3 * 1e-3 * 1.33, 1e-9, 'lugs: the plate at 1.33 g/cm3');
  for (const t of ['R', 'L']) for (const m of [`tail pushrod guide ${t} bond in its lug`, `tail pushrod guide lug ${t} bond on its former`]) assert.ok(d.structure.members.some((q) => q.name === m), m);
  const names = d.params.layout.sequence.steps.map((s) => s.name);
  for (const t of ['R', 'L']) assert.ok(names.indexOf(`tail pushrod guide ${t} with its lug`) > names.indexOf(`tail pushrod ${t} with its rod ends`), t);
});
