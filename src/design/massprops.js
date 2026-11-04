// Mass properties from the drawn solids (SPEC "Design freeze", "Function completeness", 2026-11-03: each lump's CG lies
// within 1 mm of the centroid of its drawn parts, its inertia follows them, and a hard-coded position is a failure).
// A lump's BOM lines (build.js lump.lines) name the solids it stands for (each solid tagged with its line, round 16);
// a bought part's line puts its unit mass on each drawn piece (round 19), a structure line is spread over its solids
// by volume, a piece's mass over its own solids by volume, and each solid is the prism the assembly check reads (its
// section in the body's (z, y) plane over its x range), or for a tube along another axis (solid.tube) that tube
// itself. The covering film, which has no solid, is spread over the loft's surface. Round 19: a lump's lines must weigh
// the lump's own mass (MASS_TOL_G), so a part id on BOM rows of several roles cannot put another row's pieces into it.
//
// Inertia is about the lump's own centroid, in body axes, in flight.js's accumulator form
// [Ixx, Iyy, Izz, -Ixy, -Ixz, -Iyz] (the products stored with flight.js's sign), per kg of the lump: the build
// multiplies it by the lump's mass (margin included, spread like the parts).
import { bomLines, resolveTag, solidVolume } from './completeness.js';

/** The largest distance (m) between a lump's position and its drawn parts' centroid. */
export const CG_TOL_M = 0.001;

const sub = (p, q) => [p[0] - q[0], p[1] - q[1]];
const hyp = (p) => Math.hypot(p[0], p[1]);

/**
 * Second moments of a section about its own centroid in the (z, y) plane: { A, c: [z, y], Szz: integral z'^2 dA,
 * Syy: integral y'^2 dA, Szy: integral z'y' dA }. Rings, capsules (stadia) and polygons less their round holes.
 */
export function sectionProps(sh) {
  if (sh.kind === 'ring') {
    const ri = sh.ri ?? 0, A = Math.PI * (sh.ro ** 2 - ri ** 2), S = (Math.PI / 4) * (sh.ro ** 4 - ri ** 4);
    return { A, c: [...sh.c], Szz: S, Syy: S, Szy: 0 };
  }
  if (sh.kind === 'cap') {
    const r = sh.r, d = sub(sh.b, sh.a), L = hyp(d), e = L > 0 ? [d[0] / L, d[1] / L] : [1, 0];
    const A = Math.PI * r * r + 2 * r * L;
    // in the capsule's frame (u along a -> b, v across it): the rectangle 2r x L and the two half discs at its ends
    const Suu = (2 * r * L ** 3) / 12 + (Math.PI * r * r * L * L) / 4 + (4 / 3) * r ** 3 * L + (Math.PI * r ** 4) / 4;
    const Svv = (L * (2 * r) ** 3) / 12 + (Math.PI * r ** 4) / 4;
    return { A, c: [(sh.a[0] + sh.b[0]) / 2, (sh.a[1] + sh.b[1]) / 2], ...rotate(Suu, Svv, e) };
  }
  return polyProps(sh.pts, sh.holes ?? []);
}

/** Moments Suu (along e), Svv (across) of a section symmetric about its axes, turned into the (z, y) frame. */
function rotate(Suu, Svv, e) {
  const [cz, sy] = e; // u = (cz, sy), v = (-sy, cz)
  return { Szz: Suu * cz * cz + Svv * sy * sy, Syy: Suu * sy * sy + Svv * cz * cz, Szy: (Suu - Svv) * cz * sy };
}

/** A polygon (any winding) less round holes: area, centroid and central second moments. */
function polyProps(pts, holes) {
  let A = 0, Cz = 0, Cy = 0, Izz = 0, Iyy = 0, Izy = 0; // Izz = integral z^2, Iyy = integral y^2, Izy = integral z y (about the origin)
  for (let i = 0; i < pts.length; i++) {
    const [z0, y0] = pts[i], [z1, y1] = pts[(i + 1) % pts.length], k = z0 * y1 - z1 * y0;
    A += k; Cz += (z0 + z1) * k; Cy += (y0 + y1) * k;
    Izz += (z0 * z0 + z0 * z1 + z1 * z1) * k; Iyy += (y0 * y0 + y0 * y1 + y1 * y1) * k;
    Izy += (z0 * y1 + 2 * z0 * y0 + 2 * z1 * y1 + z1 * y0) * k;
  }
  const s = Math.sign(A) || 1;
  A = (A / 2) * s; Cz = (Cz / 6) * s; Cy = (Cy / 6) * s; Izz = (Izz / 12) * s; Iyy = (Iyy / 12) * s; Izy = (Izy / 24) * s;
  for (const h of holes) {
    const a = Math.PI * h.r * h.r, j = (Math.PI / 4) * h.r ** 4;
    A -= a; Cz -= a * h.c[0]; Cy -= a * h.c[1];
    Izz -= j + a * h.c[0] ** 2; Iyy -= j + a * h.c[1] ** 2; Izy -= a * h.c[0] * h.c[1];
  }
  const c = [Cz / A, Cy / A];
  return { A, c, Szz: Izz - A * c[0] ** 2, Syy: Iyy - A * c[1] ** 2, Szy: Izy - A * c[0] * c[1] };
}

/**
 * One solid's volume, centroid [x, y, z] (body axes) and inertia per unit density about that centroid
 * ([Ixx, Iyy, Izz, -Ixy, -Ixz, -Iyz]), at the drive pose q0. A tube along the span (solid.tube) is that tube, along its
 * capsule's axis in the (z, y) plane, at the solid's mid x.
 */
export function solidProps(s, q0) {
  const sh = s.at ? s.at(q0) : s.shape, x0 = s.x[0], x1 = s.x[1], xm = (x0 + x1) / 2;
  if (s.tube && sh.kind === 'cap') {
    const { ro, ri = 0, L } = s.tube, V = Math.PI * (ro * ro - ri * ri) * L;
    const d = sub(sh.b, sh.a), n = hyp(d), e = n > 0 ? [d[0] / n, d[1] / n] : [1, 0];
    const jAx = (ro * ro + ri * ri) / 2, jPerp = (ro * ro + ri * ri) / 4 + (L * L) / 12; // per unit volume
    // about its own axis Iu, about any axis across it Ip; its axis in body axes a = (0, y, z) from the capsule's (z, y)
    const Iu = jAx * V, Ip = jPerp * V, [cz, sy] = e;
    const a = [0, sy, cz], I = (i, k) => (i === k ? Ip : 0) + (Iu - Ip) * a[i] * a[k];
    // Round 18 (diagnosis 1.3, bug): for i != k, I(i, k) is already the accumulator's stored sign (-Ixy/-Ixz/-Iyz: the
    // standard inertia-tensor off-diagonal element, itself the negative of the raw product integral x_i x_k dm), so
    // storing -I(i, k) double-negated it. A 45-degree tube in the y-z plane (test/design/round18-bugs.test.js) pinned
    // it against the independent, closed-form rotated-tube tensor.
    return { V, c: [xm, (sh.a[1] + sh.b[1]) / 2, (sh.a[0] + sh.b[0]) / 2], J: [I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2)] };
  }
  const P = sectionProps(sh), Lx = x1 - x0;
  const A = Number.isFinite(s.area_m2) ? s.area_m2 : P.A, k = A / P.A; // a stated added area (a plate's ear) scales the section
  const V = A * Lx;
  // Szz = integral z'^2 dA, Syy = integral y'^2 dA (section frame [z, y])
  const Ixx = Lx * (P.Szz + P.Syy) * k, Iyy = (A * Lx ** 3) / 12 + Lx * P.Szz * k, Izz = (A * Lx ** 3) / 12 + Lx * P.Syy * k;
  return { V, c: [xm, P.c[1], P.c[0]], J: [Ixx, Iyy, Izz, 0, 0, -Lx * P.Szy * k] };
}

/** Add a point-and-own-inertia body (mass m, centroid c, own inertia I) to an accumulator about the origin. */
function accumulate(acc, m, c, I) {
  acc.m += m;
  for (let i = 0; i < 3; i++) acc.s[i] += m * c[i];
  const [x, y, z] = c;
  acc.I[0] += I[0] + m * (y * y + z * z); acc.I[1] += I[1] + m * (x * x + z * z); acc.I[2] += I[2] + m * (x * x + y * y);
  acc.I[3] += I[3] - m * x * y; acc.I[4] += I[4] - m * x * z; acc.I[5] += I[5] - m * y * z;
}

/** An accumulator's centroid and inertia about it. */
function central(acc) {
  const c = acc.s.map((v) => v / acc.m), [x, y, z] = c, m = acc.m, I = acc.I;
  return { m, c, I: [I[0] - m * (y * y + z * z), I[1] - m * (x * x + z * z), I[2] - m * (x * x + y * y), I[3] + m * x * y, I[4] + m * x * z, I[5] + m * y * z] };
}

/**
 * The covering film over a loft (body.loft.sections, each { x_m, y_m, width_m, height_m }): its area, centroid and
 * inertia per m2 of film, from the elliptic sections' perimeters (Ramanujan) between neighbouring sections.
 */
export function loftSurfaceProps(loft) {
  const S = (loft?.sections ?? []).map((q) => ({ x: q.x_m, y: q.y_m ?? 0, w: q.width_m, h: q.height_m })).sort((a, b) => a.x - b.x);
  const acc = { m: 0, s: [0, 0, 0], I: [0, 0, 0, 0, 0, 0] };
  const per = (a, b) => { if (!(a + b > 0)) return 0; const h = ((a - b) / (a + b)) ** 2; return Math.PI * (a + b) * (1 + (3 * h) / (10 + Math.sqrt(4 - 3 * h))); };
  for (let i = 0; i + 1 < S.length; i++) {
    const q0 = S[i], q1 = S[i + 1], dx = q1.x - q0.x;
    if (!(dx > 0)) continue;
    const a = (q0.w + q1.w) / 4, b = (q0.h + q1.h) / 4, y = (q0.y + q1.y) / 2, area = per(a, b) * dx;
    if (!(area > 0)) continue;
    // a thin elliptic band: its own moments about its centre taken as a ring's in each axis, and its length along x
    const ry2 = (b * b) / 2, rz2 = (a * a) / 2;
    accumulate(acc, area, [(q0.x + q1.x) / 2, y, 0], [area * (ry2 + rz2), area * (rz2 + (dx * dx) / 12), area * (ry2 + (dx * dx) / 12), 0, 0, 0]);
  }
  if (!acc.m) return null;
  const c = central(acc);
  return { area_m2: c.m, c: c.c, I_per_m2: c.I.map((v) => v / c.m) };
}

/**
 * The tail's covering (both faces of both V surfaces, design.tail): area, centroid and inertia per m2, from its
 * planform (each surface from its root on the boom along its dihedral, quarter chord at tail.x_m, chord tapering to
 * the tip), in 40 strips per surface. The boom's height is tail.y_m.
 */
export function tailSurfaceProps(T) {
  if (!T?.span_each_m) return null;
  const acc = { m: 0, s: [0, 0, 0], I: [0, 0, 0, 0, 0, 0] }, N = 40, L = T.span_each_m;
  for (const sg of [1, -1]) for (let i = 0; i < N; i++) {
    const s = ((i + 0.5) / N) * L, c = T.chord_root_m * (1 - (1 - T.taper) * (s / L)), dA = (2 * c * L) / N; // both faces
    const pos = [T.x_m - 0.25 * c, (T.y_m ?? 0) + s * Math.sin(T.dihedral_rad), sg * ((T.z_m ?? 0) + s * Math.cos(T.dihedral_rad))];
    accumulate(acc, dA, pos, [0, (dA * c * c) / 12, (dA * c * c) / 12, 0, 0, 0]);
  }
  const c = central(acc);
  return { area_m2: c.m, c: c.c, I_per_m2: c.I.map((v) => v / c.m) };
}

/**
 * Each lump's drawn mass properties: the solids of its BOM lines (lump.lines), each line's mass spread over its solids
 * by volume; a covering line (exempt, no solids) spread over its surface: the pod's loft, the tail's planform. -> { rows, passed, ... }.
 * rows: { lump, mass_g, pos_mm, drawn_mm, err_mm, lines, solids, inertia_per_kg, missing }.
 */
export function massPropsRecord(asm, d, catalog, q0, { tol = CG_TOL_M } = {}) {
  const lines = bomLines(d), byKey = new Map(lines.map((l) => [l.key, l]));
  const sols = new Map(); // line key -> solids
  for (const p of asm.parts) for (const s of p.solids) {
    const k = resolveTag(lines, s.bom);
    if (!k) continue;
    if (!sols.has(k)) sols.set(k, []);
    sols.get(k).push({ ...s, part: p.id });
  }
  const films = { pod: loftSurfaceProps(d.body?.loft), tail: tailSurfaceProps(d.tail) };
  const margin = d.mass.margin_frac ?? 0;
  const rows = d.mass.lumps.map((l) => lumpRow(l, byKey, sols, films, lines, q0, tol, margin));
  const failed = rows.filter((r) => !r.ok).map((r) => r.lump);
  return {
    rule: 'SPEC Design freeze, Function completeness (2026-11-03): each lump\'s CG within 1 mm of the centroid of its drawn parts, its inertia from them; a hard-coded position is a failure',
    method: 'src/design/massprops.js: each BOM line\'s mass (margin included) on its tagged solids: a bought part\'s unit mass on each of its drawn pieces (round 19), a structure line by volume, each piece\'s mass over its solids by volume; the solids as the assembly check reads them at the drive pose of phase 0 (a tube along the span as that tube); the covering film over the loft\'s surface; a lump whose lines\' drawn mass is not its own mass (a part id shared by BOM rows of several roles, named without a part filter) fails',
    tol_mm: tol * 1000, mass_tol_g: MASS_TOL_G,
    rows, failed, passed: failed.length === 0,
  };
}

/**
 * A lump's line reference: a BOM line's key or tag, or (round 17) 'tag@<part regex>', the line's solids on the parts
 * whose ids match (a line shared by two lumps: the MD89MW drives the tail and the twist), with those pieces' own mass
 * (linePieces). -> { key, re }.
 */
function lineRef(t, byKey, lines) {
  const at = t.indexOf('@'), tag = at < 0 ? t : t.slice(0, at), re = at < 0 ? null : new RegExp(t.slice(at + 1));
  return { key: byKey.has(tag) ? tag : resolveTag(lines, tag), re };
}

/**
 * The masses a line puts on the solids `ss` (of its solids `all`): [{ m (g), s (the solid) }], or null with no volume.
 * Round 19: a bought part's line (bomLines kind 'part': identical catalogue pieces) puts its unit mass (line mass over
 * its count) on each drawn item (solid.item, else its part, as completeness.js counts them), spread over that item's
 * solids by volume, so a part filter takes exactly its own items' mass however each item is drawn; a line sized from a
 * material (structure) is spread by volume, its mass being its volume's. Each piece of a bought part weighs its unit
 * mass wherever it is drawn: before round 19 a filtered share was the line's mass by volume, so two drawings of one
 * part (the R-ML25 arm cut down on the tail servo) put the wrong mass on each.
 */
function linePieces(line, all, ss, q0) {
  const pr = new Map(ss.map((s) => [s, solidProps(s, q0)]));
  const V = [...pr.values()].reduce((a, q) => a + q.V, 0);
  if (!(V > 0)) return null;
  const itemOf = (s) => s.item ?? s.part, items = new Map();
  for (const s of all) items.set(itemOf(s), (items.get(itemOf(s)) ?? 0) + (pr.get(s) ?? solidProps(s, q0)).V);
  if (line.kind === 'part' && items.size === line.count && [...items.values()].every((v) => v > 0)) {
    const unit = line.mass_g / line.count;
    return ss.map((s) => ({ s, q: pr.get(s), m: (unit * pr.get(s).V) / items.get(itemOf(s)) }));
  }
  // by volume (a structure line, or a part line whose drawn items do not match its count: completeness fails it)
  const Vall = ss.length === all.length ? V : all.reduce((a, s) => a + (pr.get(s) ?? solidProps(s, q0)).V, 0);
  return ss.map((s) => ({ s, q: pr.get(s), m: (line.mass_g * pr.get(s).V) / Vall }));
}

/** The largest difference (g) between a lump's mass and its lines' drawn mass with the margin: rounding only. */
export const MASS_TOL_G = 0.01;

/** A covering line with no solids: its surface (the pod's loft, the tail's planform), or null. */
const filmOf = (line, films) => (!line.exempt ? null : /film shell/.test(line.name) ? films.pod : /^tail film/.test(line.name) ? films.tail : null);

/**
 * A lump's lines (refs) on their solids, added to acc. -> { missing, used, bareShared }: what has no solids, the lines
 * used, and the part ids on BOM rows of several roles named without a part filter.
 */
function addLines(refs, byKey, sols, films, q0, acc) {
  const missing = [], used = [], bareShared = [];
  for (const { key: k, re } of refs) {
    const line = k ? byKey.get(k) : null;
    if (!line) { missing.push('a line the lump names is not in the BOM'); continue; }
    const all = sols.get(k) ?? [], ss = re ? all.filter((s) => re.test(s.part)) : all;
    const film = ss.length ? null : filmOf(line, films);
    if (film) { accumulate(acc, line.mass_g, film.c, film.I_per_m2.map((v) => v * line.mass_g)); used.push(`${k} (over its surface)`); continue; }
    const pieces = ss.length ? linePieces(line, all, ss, q0) : null;
    if (!pieces) { missing.push(`${k}: ${ss.length ? 'no volume' : 'no solids'}`); continue; }
    for (const { q, m } of pieces) accumulate(acc, m, q.c, q.J.map((v) => (v * m) / q.V));
    if (!re && new Set(line.roles ?? []).size > 1) bareShared.push(`${k} (BOM rows: ${line.roles.join('; ')})`);
    used.push(re ? `${k} (on ${re.source})` : k);
  }
  return { missing, used, bareShared };
}

function lumpRow(l, byKey, sols, films, lines, q0, tol, margin) {
  const refs = (l.lines ?? []).map((t) => lineRef(t, byKey, lines)), acc = { m: 0, s: [0, 0, 0], I: [0, 0, 0, 0, 0, 0] };
  const { missing, used, bareShared } = addLines(refs, byKey, sols, films, q0, acc);
  if (!l.lines?.length) missing.push('the lump names no BOM lines');
  const g = acc.m > 0 ? central(acc) : null;
  const err = g ? Math.hypot(...l.pos.map((v, i) => v - g.c[i])) : Infinity;
  // Round 19: the lump's drawn mass (its lines' masses on their solids, with the margin the build adds) must be the
  // lump's own mass. A part id on BOM rows of several roles (the KBRM-02 rod ends and M2 x 6 screws of both the tail
  // and the twist drive), named by a lump without a part filter, put every row's pieces into that lump's centroid; it
  // now refuses, naming the line ('id@<part regex>' names only this lump's pieces).
  const drawnG = acc.m * (1 + margin), massErr = drawnG - l.mass_kg * 1000;
  const massOk = !g || Math.abs(massErr) <= MASS_TOL_G;
  const ok = !missing.length && massOk && err <= tol + 1e-12;
  return {
    lump: l.name, mass_g: +(l.mass_kg * 1000).toFixed(2), lines: used, missing,
    drawn_mass_g: +drawnG.toFixed(3),
    ...(massOk ? {} : { mass_mismatch: `its lines weigh ${drawnG.toFixed(3)} g on their solids (margin ${margin} included), the lump ${(l.mass_kg * 1000).toFixed(3)} g${bareShared.length ? `: named without a part filter, a part id on BOM rows of several roles puts every row's pieces here: ${bareShared.join(', ')}` : ''}` }),
    pos_mm: l.pos.map((v) => +(v * 1000).toFixed(2)), drawn_mm: g ? g.c.map((v) => +(v * 1000).toFixed(2)) : null,
    err_mm: Number.isFinite(err) ? +(err * 1000).toFixed(3) : null, ok,
    // per kg (g.m in grams: I in g m2 -> per kg divide by g.m)
    ...(g ? { centroid_m: g.c, inertia_per_kg_m2: g.I.map((v) => v / g.m) } : {}),
  };
}

/**
 * The drawn positions and inertias of a built design's lumps (massPropsRecord), as build parameters:
 * { lumpPos: { name: [x, y, z] }, lumpInertia: { name: [6] per kg } }. Lumps with missing solids are left out.
 */
export function drawnLumps(rec) {
  const lumpPos = {}, lumpInertia = {};
  for (const r of rec.rows) {
    if (r.missing.length || !r.centroid_m) continue;
    lumpPos[r.lump] = r.centroid_m.map((v) => +v.toFixed(6));
    lumpInertia[r.lump] = r.inertia_per_kg_m2.map((v) => +v.toExponential(6));
  }
  return { lumpPos, lumpInertia };
}

export const _test = { polyProps, accumulate, central, rotate };
