// Round 19 mutation follow-up (tail-mech.js exitGuide, pushrodPieces/pieceBreaks, pushrodExcursion, guideMembers): the
// exit guide as drawn on the sealed round-17 design, against figures by hand: its pieces on the rod's true axis (ball to
// ball), each ring the guide's drawn section (the maker's +-0.3 mm on both diameters: outside 10.3 mm, bore 8.7 mm) grown
// by half its piece's offset, its true length the pieces' x run times |H - S| / |dx|; its forward face on the former's
// aft face, its first piece the lug's 1 mm; the lug's hole on the axis at mid-lug; the bonds' load the guide, lug and
// pushrod at their catalogue masses plus the guide's drag (0.5 rho V^2 x 1.2 x pi/4 x 10.3 mm^2); the excursion from an
// independent solve of the stated geometry (each arc, the rod's length held).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { solidMassParams } from '../../src/design/round11.mjs';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { fullAssembly } from '../../src/design/clearance.js';
import { completenessRecord, solidVolume } from '../../src/design/completeness.js';
import { drivePose } from '../../sims/avis-shared/model.js';
import { pushrodPieces, pushrodExcursion, exitGuide } from '../../src/design/tail-mech.js';
import { podGeometry } from '../../src/design/pod-structure.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
let cached = null;
function base() {
  if (cached) return cached;
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos', 'tailMech', 'pod']) if (k in set) p[k] = set[k];
  p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm, lay } = fullAssembly(d, catalog);
  return (cached = { d, asm, lay });
}
const part = (asm, id) => { const p = asm.parts.find((q) => q.id === id); assert.ok(p, id); return p; };
const axisOf = (row, G) => ({ S: { x: row.xs, p: row.A.p }, H: { x: G.xq, p: row.B.p } });
const onAxis = ({ S, H }, x) => { const t = (x - S.x) / (H.x - S.x); return [S.p[0] + t * (H.p[0] - S.p[0]), S.p[1] + t * (H.p[1] - S.p[1])]; };

test('guide mut19: the guide and lug lines are complete, and the stated length and lug volume are the drawn ones', () => {
  const { d, asm, lay } = base(), q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal);
  const rec = completenessRecord(asm, d, catalog, q0);
  for (const name of ['tail pushrod guides, 2 off', 'tail pushrod guide lugs, 2 off']) {
    const l = rec.lines.find((q) => q.line === name);
    assert.ok(l && l.passed && l.count_ok, `${name}: ${JSON.stringify(l?.size)}`);
  }
  const P = d.params.tailMech.guide;
  for (const row of asm.tailMech.rows) {
    near(row.guide.L * 1000, P.L_mm, 0.01, `${row.tag} true length against the stated ${P.L_mm} mm`);
    // the lug's volume as exitGuide reports it (hull less the hole, times the plate) against the drawn solid's own
    // (completeness.js solidVolume); and the stated figure within 2% (round 19 states 76.0 mm3, drawn 75.15: see report)
    const lugSolid = part(asm, `tail pushrod guide lug ${row.tag}`).solids[0];
    near(row.guide.lug_mm3, solidVolume(lugSolid, q0) * 1e9, 1e-6, `${row.tag} lug volume against its drawn solid`);
    assert.ok(Math.abs(row.guide.lug_mm3 / P.lug_mm3 - 1) < 0.02, `${row.tag} lug ${row.guide.lug_mm3} mm3, stated ${P.lug_mm3}`);
    const g = part(asm, `tail pushrod guide ${row.tag}`).solids;
    near(row.guide.x[0], Math.min(...g.map((s) => s.x[0])), 1e-12, `${row.tag} aft end`);
    near(row.guide.x[1], Math.max(...g.map((s) => s.x[1])), 1e-12, `${row.tag} forward end`);
  }
});

test('guide mut19: each guide piece on the rod\'s true axis, its ring the drawn section grown by half its offset, its true length', () => {
  const { d, asm } = base(), { rows, G } = asm.tailMech, P = d.params.tailMech.guide, F = podGeometry(d, catalog).formers.find((f) => f.name === P.former);
  for (const row of rows) {
    const ax = axisOf(row, G), dx = ax.H.x - ax.S.x, dp = Math.hypot(ax.H.p[0] - ax.S.p[0], ax.H.p[1] - ax.S.p[1]), slope = dp / Math.abs(dx), kL = Math.hypot(dx, dp) / Math.abs(dx);
    const g = [...part(asm, `tail pushrod guide ${row.tag}`).solids].sort((a, b) => b.x[1] - a.x[1]);
    near(g[0].x[1], F.x[0], 1e-12, 'forward face on the former\'s aft face');
    near(g[0].x[1] - g[0].x[0], 0.001, 1e-12, 'first piece: the lug\'s 1 mm');
    for (let k = 1; k < g.length; k++) near(g[k].x[1], g[k - 1].x[0], 1e-12, 'pieces contiguous');
    for (const s of g.slice(1)) near(s.x[1] - s.x[0], P.piece_mm / 1000, 1e-12, 'piece length');
    for (const s of g) {
      const half = (slope * (s.x[1] - s.x[0])) / 2, c = onAxis(ax, (s.x[0] + s.x[1]) / 2);
      near(Math.hypot(s.shape.c[0] - c[0], s.shape.c[1] - c[1]), 0, 1e-12, `${s.name} on the axis`);
      near(s.shape.ro, 10.3e-3 / 2 + half, 1e-12, `${s.name} outside`);
      near(s.shape.ri, 8.7e-3 / 2 - half, 1e-12, `${s.name} bore`);
      near(s.tube.ro, 0.005, 1e-12, 'tube ro'); near(s.tube.ri, 0.0045, 1e-12, 'tube ri');
      near(s.tube.L, kL * (s.x[1] - s.x[0]), 1e-12, `${s.name} true length`);
    }
    near(row.guide.L, kL * (g[0].x[1] - g.at(-1).x[0]), 1e-12, 'the guide\'s true length');
    // its assembly slide: aft along the rod's true axis, [x, y, z] (p is [z, y])
    const L3 = Math.hypot(dx, dp), slide = part(asm, `tail pushrod guide ${row.tag}`).slide;
    [dx / L3, (ax.H.p[1] - ax.S.p[1]) / L3, (ax.H.p[0] - ax.S.p[0]) / L3].forEach((v, i) => near(slide[i], v, 1e-8, `slide ${i}`));
    // the lug: the plate's 1 mm on the former's aft face, its hole on the axis at mid-lug, the guide's outside + the bond gap
    const lug = part(asm, `tail pushrod guide lug ${row.tag}`).solids[0], hole = lug.shape.holes[0], c = onAxis(ax, F.x[0] - 0.0005);
    near(lug.x[0], F.x[0] - 0.001, 1e-12, 'lug aft face'); near(lug.x[1], F.x[0], 1e-12, 'lug forward face');
    near(Math.hypot(hole.c[0] - c[0], hole.c[1] - c[1]), 0, 1e-12, 'lug hole on the axis');
    near(hole.r, 10.3e-3 / 2 + (slope * 0.001) / 2 + P.bond_gap_mm / 1000, 1e-12, 'lug hole radius');
  }
});

test('guide mut19: the pushrod\'s own breaks meet the guide\'s, two pieces forward of it and the sweep aft, concentric there', () => {
  const { d, asm } = base(), { rows } = asm.tailMech, P = d.params.tailMech.guide, step = P.piece_mm / 1000;
  for (const row of rows) {
    const g = part(asm, `tail pushrod guide ${row.tag}`).solids, rod = part(asm, `tail pushrod ${row.tag}`).solids;
    const xf = Math.max(...g.map((s) => s.x[1])), xa = Math.min(...g.map((s) => s.x[0]));
    const edges = rod.flatMap((s) => s.x), has = (x) => edges.some((e) => Math.abs(e - x) < 1e-9);
    for (const x of [xf + step, xf + 2 * step, ...g.flatMap((s) => s.x)]) assert.ok(has(x), `a rod break at ${x}`);
    const nAft = Math.ceil(P.sweep_mm / P.piece_mm) + 2;
    for (let k = 1; k <= nAft; k++) assert.ok(has(xa - k * step), `a rod break ${k} steps aft of the guide`);
    for (const s of g) {
      const r = rod.find((q) => Math.abs(q.x[0] - s.x[0]) < 1e-9 && Math.abs(q.x[1] - s.x[1]) < 1e-9);
      assert.ok(r, `a rod piece on ${s.name}`);
      near(Math.hypot(r.shape.c[0] - s.shape.c[0], r.shape.c[1] - s.shape.c[1]), 0, 1e-12, `${s.name} concentric`);
    }
  }
});

test('guide mut19: a guide whose rod is not out_mm inside the loft at its former is refused', () => {
  const { d, asm } = base(), { rows, G } = asm.tailMech, row = rows[0], ax = axisOf(row, G);
  const P = { ...asm.tailMech.P, guide: { ...asm.tailMech.P.guide, out_mm: 50 } };
  assert.doesNotThrow(() => exitGuide(d, catalog, ax.S, ax.H, asm.tailMech.P), 'the design\'s own guide is drawn');
  assert.throws(() => exitGuide(d, catalog, ax.S, ax.H, P), /not 50 mm inside the loft/);
});

test('guide mut19: pushrodPieces tiles the rod in order, its rings grown by half the offset, fine breaks in any order', () => {
  const A = { x: -0.04, p: [0.024, -0.044] }, B = { x: -0.93, p: [0.013, 0.083] }, ro = 0.0025, ri = 0.00195;
  const fine = Array.from({ length: 11 }, (_, k) => -0.2 - 0.002 * k), Ft = { A: { x: -0.03, p: [0.024, -0.042] }, B: { x: -0.945, p: [0.013, 0.083] } };
  const pr = pushrodPieces(A, B, ro, ri, 0.005, 0.02, fine, Ft);
  // shuffled, with breaks inside the end pieces and beyond the rod (ignored)
  const shuffled = [fine[5], fine[0], fine[10], -0.042, -0.929, -1.5, 0.3, ...fine.slice(1, 5), ...fine.slice(6, 10)];
  const ps = pushrodPieces(A, B, ro, ri, 0.005, 0.02, shuffled, Ft);
  assert.deepEqual(ps.solids.map((s) => s.x), pr.solids.map((s) => s.x));
  // the pieces run from A to B without gap or overlap, each shorter than 20 mm
  const xs = pr.solids.map((s) => s.x).sort((a, b) => b[1] - a[1]);
  near(xs[0][1], A.x, 1e-12, 'from A'); near(xs.at(-1)[0], B.x, 1e-12, 'to B');
  for (let k = 1; k < xs.length; k++) near(xs[k][1], xs[k - 1][0], 1e-12, 'contiguous');
  assert.ok(xs.every(([a, b]) => b - a > 0 && b - a <= 0.02 + 1e-12));
  const end = 0.005 * (Math.abs(B.x - A.x) / 0.89); // the end pieces: endLen (A to B runs 0.89 m in x)
  near(xs[0][1] - xs[0][0], end, 1e-12, 'servo end piece'); near(xs.at(-1)[1] - xs.at(-1)[0], end, 1e-12, 'horn end piece');
  // outside the fine zone each ring on the line A-B, grown by half the piece's offset; inside, the fine line's length
  const kA = Math.hypot(B.p[0] - A.p[0], B.p[1] - A.p[1]) / Math.abs(B.x - A.x), kF = Math.hypot(Ft.B.x - Ft.A.x, Ft.B.p[0] - Ft.A.p[0], Ft.B.p[1] - Ft.A.p[1]) / Math.abs(Ft.B.x - Ft.A.x);
  for (const s of pr.solids) {
    const dxs = s.x[1] - s.x[0], inZ = s.x[0] >= -0.22 - 1e-9 && s.x[1] <= -0.2 + 1e-9;
    near(s.shape.ro, ro + (kA * dxs) / 2 + (inZ ? ((Math.hypot(Ft.B.p[0] - Ft.A.p[0], Ft.B.p[1] - Ft.A.p[1]) / Math.abs(Ft.B.x - Ft.A.x) - kA) * dxs) / 2 : 0), 1e-12, `${s.name} outside`);
    if (inZ) near(s.tube.L, kF * dxs, 1e-12, `${s.name} on the fine line`);
  }
});

/** The excursion solved independently from the stated geometry (tail-mech.js pushrodExcursion's header), by Newton. */
function excursionRef(row, G, dMax, xs, n = 40) {
  const ra = Math.abs(row.zH - row.zs), sg = Math.sign(row.zH - row.zs) || 1, R = Math.abs(row.nE), yb = row.A.p[1];
  const S = (th) => [row.xs - ra * Math.sin(th), row.zs + sg * ra * Math.cos(th), yb];
  const P0 = [G.xq, row.o[0] + row.u[0] * row.sE, row.o[1] + row.u[1] * row.sE];
  const H = (ph) => [P0[0] - R * Math.sin(ph), P0[1] + row.v[0] * row.nE * Math.cos(ph), P0[2] + row.v[1] * row.nE * Math.cos(ph)];
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const D0 = dist(H(0), S(0)), s0 = S(0), h0 = H(0), u0 = [0, 1, 2].map((i) => (h0[i] - s0[i]) / D0);
  let worst = 0;
  for (let i = -n; i <= n; i++) {
    const dl = (i / n) * dMax, th = Math.asin((Math.sin(dl) * R) / ra), Sx = S(th);
    let ph = dl;
    for (let it = 0; it < 50; it++) { const f = dist(H(ph), Sx) - D0, df = (dist(H(ph + 1e-7), Sx) - dist(H(ph - 1e-7), Sx)) / 2e-7; ph -= f / df; }
    const Hx = H(ph), L = dist(Hx, Sx), u = [0, 1, 2].map((k) => (Hx[k] - Sx[k]) / L);
    for (const x of xs) {
      const gp = [0, 1, 2].map((k) => s0[k] + (u0[k] * (x - s0[0])) / u0[0]);
      const t = ((gp[0] - Sx[0]) * u0[0] + (gp[1] - Sx[1]) * u0[1] + (gp[2] - Sx[2]) * u0[2]) / (u[0] * u0[0] + u[1] * u0[1] + u[2] * u0[2]);
      worst = Math.max(worst, dist([0, 1, 2].map((k) => Sx[k] + u[k] * t), gp));
    }
  }
  return worst;
}

test('guide mut19: the pushrod\'s excursion matches an independent solve of the stated geometry, both ways', () => {
  const { d, asm } = base(), { rows, G } = asm.tailMech, dm = d.tail.deflection_max_rad;
  for (const row of rows) {
    const xs = part(asm, `tail pushrod guide ${row.tag}`).solids.flatMap((s) => s.x);
    const ref = excursionRef(row, G, dm, xs), up = excursionRef(row, G, dm, xs, 1), down = excursionRef(row, G, -dm, xs, 1);
    near(pushrodExcursion(row, G, dm, xs), ref, 1e-8, `${row.tag} over the travel`);
    near(pushrodExcursion(row, G, dm, xs, 1), Math.max(up, down), 1e-8, `${row.tag} at the two ends`);
    near(part(asm, `tail pushrod guide ${row.tag}`).seal.excursion_m, pushrodExcursion(row, G, dm, part(asm, `tail pushrod guide ${row.tag}`).solids.map((s) => s.x[1]).concat(Math.min(...xs))), 1e-9, 'the seal\'s figure');
  }
});

test('guide mut19: the guide\'s bonds carry the guide, lug and whole pushrod at their catalogue masses, and the guide\'s drag', () => {
  const { d } = base(), P = d.params.tailMech, gd = P.guide;
  // 24.7 g/m guide (Easy Composites), 11.1 g/m pushrod (CST CS-509), the 1 mm plate at 1.33 g/cm3
  const m = (24.7 * gd.L_mm + 11.1 * P.pushrod_L_mm) / 1e6 + gd.lug_mm3 * 1e-9 * 1330;
  const rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0, D = 0.5 * rho * 25 ** 2 * 1.2 * (Math.PI / 4) * (10.3e-3) ** 2;
  for (const t of ['R', 'L']) for (const name of [`tail pushrod guide ${t} bond in its lug`, `tail pushrod guide lug ${t} bond on its former`]) {
    const mem = d.structure.members.find((q) => q.name === name), F = mem.sections.find((s) => s.name === 'F').radial;
    near(F.filter((q) => q.ch === 'sf_origin').reduce((a, q) => a + q.k, 0), m + D / 9.80665, 2e-8, `${name}: mass and drag`);
    if (/in its lug/.test(name)) { near(mem.dims.d_mm, 9.7, 1e-9, 'bond on the guide\'s smallest outside'); near(mem.dims.L_mm, 1, 1e-12, 'over the lug'); }
  }
});

test('guide mut19: a tail mechanism stating no guide draws none (the exits are then the covering record\'s to refuse)', () => {
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos', 'tailMech', 'pod']) if (k in set) p[k] = set[k];
  p = { ...p, tailMech: { ...p.tailMech } };
  delete p.tailMech.guide;
  p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm } = fullAssembly(d, catalog);
  assert.ok(d.params.pod, 'the pod is drawn');
  assert.equal(asm.parts.filter((q) => /^tail pushrod guide/.test(q.id)).length, 0);
  assert.ok(asm.tailMech.rows.every((r) => !r.guide));
});
