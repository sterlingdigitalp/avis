// Round 16 (SPEC "Completeness and assembly order", 2026-11-03): the wings' and the tail's structure as solids of the
// assembly check (src/design/clearance.js), each tagged with its BOM line (src/design/completeness.js).
//  - A wing's parts move with its rocker: in the wing's frame (s along the spar from the shoulder axis, X chordwise =
//    body x, n normal to the wing in the stroke plane) they are fixed, and the frame turns with the spar socket. A part
//    across x (a rib, a bay of a swept rod) is a prism along x whose (z, y) section is a rectangle in the (s, n) frame;
//    a tube along the span is the capsule round its axis, with its own tube measured for the BOM (solid.tube).
//  - The outer panel twists about the spar in flight; its parts are drawn at zero twist. Only the wing's own parts are
//    near them, and those are joined to each other (the ribs on the spar and rods, the torque rod in the spar).
//  - The tail's V surfaces are fixed: each from its root on the boom's side, along its dihedral.
// The spar's or a rod's hole in a rib, and a tube in another's bore, are not drawn: such pairs are exempt (RIB_HOLE,
// BORE in clearance.js) and every other pair is checked.
import { poly, cap, S, M, tagged, bandSlabs, RIB_HOLE, BORE } from './clearance.js';
import { byId, tubeSpec } from './catalog.js';
import { E387_THICKNESS } from '../../sims/avis-shared/model.js';
import { twistParams, twistWing } from './twist-drive.js';

const mm = (v) => v / 1000;
const add2 = (p, q, k = 1) => [p[0] + k * q[0], p[1] + k * q[1]];

/** The rectangle s in [s0, s1], n in [n0, n1] of the frame (origin o, span u, normal v) as a (z, y) polygon. */
export function frameRect(o, u, v, [s0, s1], [n0, n1]) {
  const P = (s, n) => [o[0] + s * u[0] + n * v[0], o[1] + s * u[1] + n * v[1]];
  return poly([P(s0, n0), P(s1, n0), P(s1, n1), P(s0, n1)]);
}

/** The wing's chord and leading edge x at span s (the design's segments, model.js's own interpolation). */
export function chordAt(wing, s) {
  let s0 = 0;
  for (const g of wing.segments) {
    if (s <= s0 + g.length + 1e-12) { const t = Math.min(1, Math.max(0, (s - s0) / g.length)); return { c: g.chord_in + t * (g.chord_out - g.chord_in), xle: g.xle_in + t * (g.xle_out - g.xle_in) }; }
    s0 += g.length;
  }
  const g = wing.segments.at(-1);
  return { c: g.chord_out, xle: g.xle_out };
}

/** The rib stations of one wing (round 16): the BOM's count per wing, evenly from the root rib to the tip. */
export function wingRibStations(design) {
  const W = design.params.wingStructure, L = design.wing.segments.reduce((a, g) => a + g.length, 0);
  const total = +(design.bom.structure.find((s) => /^ribs/.test(s.name))?.name.match(/(\d+)\s*off/)?.[1] ?? 20), per = Math.max(2, Math.round(total / 2));
  const t = mm(W.rib_t_mm ?? 2), s0 = W.rib_root_m, s1 = L - t / 2;
  return Array.from({ length: per }, (_, k) => s0 + (k / (per - 1)) * (s1 - s0));
}

/**
 * The rib's section envelope at span s (m): its chordwise x range between the edge rods and its height across the
 * wing (n), from the E387 thickness and the camber line at the section's incidence and washout, sampled along the chord.
 */
function ribEnvelope(wing, s, rLe, rTe) {
  const { c, xle } = chordAt(wing, s), L = wing.segments.reduce((a, g) => a + g.length, 0);
  const pitch = wing.incidence + wing.washout * (s / L);
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i <= 40; i++) {
    const xi = i / 40, cam = camber(xi, wing.camber, wing.camber_pos), h = half(xi, E387_THICKNESS);
    for (const z of [cam - h, cam + h]) {
      const X = xle - xi * c - wing.spar_x, Y = z * c;
      // the section turned by its pitch about the spar (nose up positive)
      const n = X * Math.sin(pitch) + Y * Math.cos(pitch);
      lo = Math.min(lo, n); hi = Math.max(hi, n);
    }
  }
  return { x: [xle - c + 2 * rTe, xle - 2 * rLe], n: [lo, hi], c, xle };
}
const camber = (x, m, p) => (x < p ? (m / (p * p)) * (2 * p * x - x * x) : (m / ((1 - p) ** 2)) * (1 - 2 * p + 2 * p * x - x * x));
const half = (x, t) => 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);

/**
 * One wing's structure as moving solids (round 16), in the frame of its rocker's spar: the inner spar from the
 * socket's base to the elbow, the outer spar on to the tip with the twist torque rod in its bore, the twist servo at
 * the elbow just aft of the spar, the edge rods from the root rib to the tip (a bay between ribs as one prism over the
 * rod's x there), and the ribs. frame(q) -> { o, u, v } (origin on the shoulder axis, span and normal unit vectors).
 */
export function wingParts(ax, tag, frame) {
  const { d, catalog, add, join, assumptions } = ax;
  const W = d.params.wingStructure;
  if (!W) return;
  // round 17b: the twist drive's wing (the real E387, ribs as chordwise pieces, the drive itself): src/design/twist-drive.js
  if (twistParams(d.params)) return twistWing(ax, tag, frame);
  const P = d.params.parts, side = tag === 'R' ? 'right' : 'left', wing = d.wing, L = wing.segments.reduce((a, g) => a + g.length, 0), elbow = wing.segments[0].length;
  // round 17: drawn on the envelope of a tube's published sections (catalog.js tubeSpec)
  const tube = (id) => { const t = tubeSpec(byId(catalog, id), !!d.params.tubeSpecSheet); return { ro: mm(t.od_draw) / 2, ri: mm(t.id_draw) / 2 }; };
  const si = tube(P.sparInner), so = tube(P.sparOuter), le = tube(P.leRod), te = tube(P.teRod);
  const xs = wing.spar_x;
  // a tube along the span from s0 to s1 (its capsule ends reach its radius past the segment's ends)
  const spanTube = (name, t, s0, s1) => ({ ...M(name, [xs - t.ro, xs + t.ro], (q) => { const f = frame(q); return cap(add2(f.o, f.u, s0 + t.ro), add2(f.o, f.u, s1 - t.ro), t.ro); }), tube: { ...t, L: s1 - s0, axis: 'span' } });
  const sp0 = W.spar_from_m ?? d.params.linkage.rocker.hub_od_mm / 2000; // the socket's base
  const inner = add(`inner spar ${tag}`, [spanTube('tube', si, sp0, elbow)], { bom: 'inner spar' });
  // the outer spar to where the swept leading edge's rod still clears it (W.spar_to_m), else the tip
  const sEnd = W.spar_to_m ?? L;
  const outer = add(`outer spar ${tag}`, [spanTube('tube', so, elbow, sEnd)], { bom: 'outer spar' });
  join(inner, `rocker ${tag}/spar socket`, BORE);
  join(inner, `rocker ${tag}`, 'bonded in the spar socket');
  join(outer, inner, 'butted to the inner spar at the elbow, bonded');
  let rodId = null;
  if (P.twistRod && d.bom.structure.some((s) => /^twist torque rods/.test(s.name))) {
    // the torque rod in the outer spar's bore, from the running clearance past the inner spar's end to the spar's end
    const tr = tube(P.twistRod), run = mm(d.params.layout?.running_mm ?? 0.5);
    rodId = add(`twist torque rod ${tag}`, [spanTube('tube', tr, elbow + run, sEnd)], { bom: 'twist torque rods' });
    join(rodId, outer, BORE);
  }
  // the twist servo at the elbow (its case aft of the inner spar by the running clearance, across the wing's plane)
  const sv = d.bom.parts.find((p) => /twist servo/.test(p.role));
  const ribs = wingRibStations(d), t = mm(W.rib_t_mm ?? 2);
  if (sv) {
    const dims = (byId(catalog, sv.id).specs.dimensions_text_mm ?? '').match(/[\d.]+/g).map((v) => mm(+v)), run = mm(d.params.layout?.running_mm ?? 0.5);
    const x1 = xs - si.ro - run, sc = W.servo_s_m ?? elbow;
    const id = add(`twist servo ${tag}`, [{ ...M('case', [x1 - dims[0], x1], (q) => { const f = frame(q); return frameRect(f.o, f.u, f.v, [sc - dims[2] / 2, sc + dims[2] / 2], [-dims[1] / 2, dims[1] / 2]); }), box: [dims[0], dims[2], dims[1]] }], { bom: sv.id });
    join(id, inner, 'on the spar at the elbow');
  }
  // the edge rods: each bay between neighbouring ribs in pieces of at most ROD_PIECE_M of span, each a prism over the
  // rod's x along that piece (a swept rod's prism is then within a few mm of the rod)
  const edge = (name, r, at, bom) => {
    const sol = ribs.slice(0, -1).flatMap((s0, k) => rodPieces(s0 + t / 2, ribs[k + 1] - t / 2).map(([a, b], j, all) => {
      const x0 = at(a), x1 = at(b), La = j === 0 ? a - s0 : 0, Lb = j === all.length - 1 ? ribs[k + 1] - b : 0;
      return { ...M(`bay ${k + 1}${all.length > 1 ? `.${j + 1}` : ''}`, [Math.min(x0, x1) - r, Math.max(x0, x1) + r], (q) => { const f = frame(q); return cap(add2(f.o, f.u, a), add2(f.o, f.u, b), r); }), tube: { ro: r, ri: 0, L: b - a + La + Lb, axis: 'span' } };
    }));
    // the rod's ends inside the root and tip ribs
    // (its length the BOM's, root rib to tip: the bays and half the root rib)
    sol.push({ ...M('in the ribs', [at(ribs[0]) - r, at(ribs[0]) + r], (q) => { const f = frame(q); return cap(add2(f.o, f.u, ribs[0] - t / 2 + r), add2(f.o, f.u, ribs[0] + t / 2 - r), r); }), tube: { ro: r, ri: 0, L: t / 2, axis: 'span' } });
    return add(name, sol, { bom });
  };
  const leId = edge(`leading edge rod ${tag}`, le.ro, (s) => chordAt(wing, s).xle - le.ro, 'leading edge rod');
  const teId = edge(`trailing edge rod ${tag}`, te.ro, (s) => chordAt(wing, s).xle - chordAt(wing, s).c + te.ro, 'trailing edge rod');
  ribs.forEach((s, k) => {
    const e = ribEnvelope(wing, s, le.ro, te.ro);
    const id = add(`rib ${tag} ${k + 1}`, [{ ...M('rib', e.x, (q) => { const f = frame(q); return frameRect(f.o, f.u, f.v, [s - t / 2, s + t / 2], e.n); }), envelope: true, rib_t: t }], { bom: 'ribs' });
    join(id, k === 0 ? inner : s < elbow ? inner : outer, 'on the spar');
    join(id, s <= elbow ? inner : outer, RIB_HOLE);
    if (s >= elbow - 1e-9) join(id, inner, RIB_HOLE);
    join(id, leId, RIB_HOLE); join(id, teId, RIB_HOLE);
    if (rodId && s >= elbow && s <= sEnd) join(id, rodId, RIB_HOLE);
  });
  assumptions.push(`wing ${side} (round 16, params.wingStructure): the spar from the socket's base (${(sp0 * 1000).toFixed(1)} mm from the shoulder axis) to the elbow and on to the tip, the torque rod in the outer spar's bore, the twist servo at the elbow aft of the spar, the edge rods and ${ribs.length} ribs from the root rib (${(W.rib_root_m * 1000).toFixed(1)} mm) to the tip, at zero twist; a rib by the envelope of its E387 section at its incidence and washout between the edge rods; the ribs' holes for the spar and rods not drawn (RIB_HOLE)`);
}


/** The pieces [a, b] of at most ROD_PIECE_M that cover [s0, s1]. */
export const ROD_PIECE_M = 0.01;
function rodPieces(s0, s1) {
  const n = Math.max(1, Math.ceil((s1 - s0) / ROD_PIECE_M - 1e-9));
  return Array.from({ length: n }, (_, k) => [s0 + (k / n) * (s1 - s0), s0 + ((k + 1) / n) * (s1 - s0)]);
}

/**
 * The wing structure's stations (round 16; m from the shoulder axis), from a built design: the outer spar ends where
 * the swept leading edge's rod pieces (ROD_PIECE_M) still clear it by the running clearance; the root rib is the first
 * station (1 mm steps) at which the rib keeps the running clearance outside the pod's loft through the whole stroke
 * (every amplitude, `phases` crank angles) and lies outboard of the rocker's spar socket.
 */
export function wingStructureFor(d, catalog, lay, { phases = 72, required = 0.0005, rib_t_mm = 2 } = {}, { drivePose, linkageAt, solidLoftGaps, xSamplesOf }) {
  const wing = d.wing, P = d.params.parts, L = wing.segments.reduce((a, g) => a + g.length, 0), elbow = wing.segments[0].length;
  const r = (id) => byId(catalog, id).specs.od_mm / 2000, rLe = r(P.leRod), rTe = r(P.teRod), rSo = r(P.sparOuter), run = (d.params.layout?.running_mm ?? 0.5) / 1000;
  // the spar's end: a rod piece short of the first s at which the leading edge rod stops clearing the spar's x by `run`
  let sEnd = L;
  for (let s = elbow; s <= L; s += 1e-4) if (chordAt(wing, s).xle - 2 * rLe < wing.spar_x + rSo + run) { sEnd = s - ROD_PIECE_M; break; }
  const t = rib_t_mm / 1000, R = d.params.linkage.rocker, O4 = lay.fb.O4, delta = lay.fb.alpha + linkageAt(lay.law, d.drive.ampNominal).psiMid - lay.law.phiMean;
  const amps = [...new Set([d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite))];
  const poses = amps.flatMap((amp) => Array.from({ length: phases }, (_, i) => drivePose({ law: lay.law, fourBar: lay.fb }, (i / phases) * 2 * Math.PI, amp)));
  const sock = (R.hub_od_mm / 2 + R.socket_len_mm) / 1000;
  for (let s = Math.ceil((sock + run + t / 2) * 1000) / 1000; s < elbow; s += 0.001) {
    const e = ribEnvelope(wing, s, rLe, rTe);
    const shapes = poses.map((q) => { const u = [Math.cos(q.right.rocker - delta), Math.sin(q.right.rocker - delta)]; return frameRect(O4, u, [-u[1], u[0]], [s - t / 2, s + t / 2], e.n); });
    const g = solidLoftGaps(d.body.loft, shapes, xSamplesOf(d.body.loft, e.x), required);
    if (g.clearOut >= required) return { rib_root_m: +s.toFixed(3), spar_to_m: +sEnd.toFixed(4), rib_t_mm };
  }
  throw new Error('wingStructureFor: no root rib station outside the loft inboard of the elbow');
}

/**
 * The tail's V surfaces (round 16): each from its root on the boom's side along its dihedral, a spar rod on the
 * quarter chord, a trailing edge rod (a bay between ribs as one prism over its x there), four ribs from the root rib,
 * which lies against the boom, to the tip; one tie per surface round the boom and through a slot in its root rib.
 */
export function tailParts(ax) {
  const { d, catalog, af, add, join, assumptions } = ax;
  // round 17: the all-moving tail with its linkage is placed after the packed parts (tail-mech.js, clearance.js afterPacked)
  if (d.params.tailLines !== 'split' || !af || d.params.tailMech) return;
  const T = d.tail, yB = af.boom_y_m, rod = byId(catalog, d.params.parts.teRod).specs, r = mm(rod.od_mm) / 2;
  const rb = mm(byId(catalog, d.params.parts.keel).specs.od_mm) / 2, L = T.span_each_m, t = mm(d.params.wingStructure?.rib_t_mm ?? 2);
  const tie = d.params.linkage?.airframe?.tieSmall ?? 'tie-hellermanntyton-t18r';
  const band = String(byId(catalog, tie).specs.dimensions).match(/x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/).slice(1).map((v) => mm(+v));
  const cAt = (s) => T.chord_root_m * (1 - (1 - T.taper) * (s / L)), xq = T.x_m;
  const rootRibs = [];
  for (const [tag, sg] of [['R', 1], ['L', -1]]) {
    const o = [sg * T.z_m, yB], u = [sg * Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)], v = [-sg * Math.sin(T.dihedral_rad), Math.cos(T.dihedral_rad)];
    const h = (s) => E387_THICKNESS * cAt(s);
    // the root rib against the boom: the first s at which its rectangle clears the boom's circle
    const boomC = [0, yB], clearOf = (s, hh) => { const p = frameRect(o, u, v, [s - t / 2, s + t / 2], [-hh / 2, hh / 2]).pts; return Math.min(...p.map((q) => Math.hypot(q[0] - boomC[0], q[1] - boomC[1]))) >= rb && !insideRect(p, boomC); };
    let s0 = t / 2;
    while (!clearOf(s0, h(s0)) || !edgesClear(frameRect(o, u, v, [s0 - t / 2, s0 + t / 2], [-h(s0) / 2, h(s0) / 2]).pts, boomC, rb)) s0 += 1e-4;
    const ribs = Array.from({ length: 4 }, (_, k) => s0 + (k / 3) * (L - t / 2 - s0));
    const sp = add(`tail spar rod ${tag}`, [{ ...S('rod', [xq - r, xq + r], cap(add2(o, u, s0 - t / 2 + r), add2(o, u, s0 - t / 2 + L - r), r)), tube: { ro: r, ri: 0, L, axis: 'span' } }], { bom: 'tail rods' });
    const te = add(`tail trailing edge rod ${tag}`, ribs.slice(0, -1).map((a, k) => {
      const b = ribs[k + 1], x0 = xq - 0.75 * cAt(a) + r, x1 = xq - 0.75 * cAt(b) + r;
      // each bay's share of the rod's length (the BOM's span, its ends in the root and tip ribs)
      return { ...S(`bay ${k + 1}`, [Math.min(x0, x1) - r, Math.max(x0, x1) + r], cap(add2(o, u, a + t / 2), add2(o, u, b - t / 2), r)), tube: { ro: r, ri: 0, L: ((b - a) * L) / (ribs[3] - ribs[0]), axis: 'span' } };
    }), { bom: 'tail rods' });
    ribs.forEach((s, k) => {
      const id = add(`tail rib ${tag} ${k + 1}`, [{ ...S('rib', [xq - 0.75 * cAt(s) + 2 * r, xq + 0.25 * cAt(s)], frameRect(o, u, v, [s - t / 2, s + t / 2], [-h(s) / 2, h(s) / 2])), envelope: true, rib_t: t }], { bom: 'tail ribs' });
      join(id, sp, RIB_HOLE); join(id, te, RIB_HOLE);
    });
    join(sp, 'tail boom', 'its root against the boom, through the root rib');
    rootRibs.push({ tag, pts: frameRect(o, u, v, [s0 - t / 2, s0 + t / 2], [-h(s0) / 2, h(s0) / 2]).pts, sparX: xq, teX: xq - 0.75 * cAt(s0) + r, c: cAt(s0) });
  }
  // the ties: two loops round the boom through slots in both root ribs, in the (z, y) plane, at two stations between the
  // root's leading edge and the spar rod (clear of it, on the boom), out to just past where the root ribs cross the
  // boom's height band
  const zt = Math.max(...rootRibs.flatMap((q) => q.pts.filter((p) => p[1] <= yB + rb + 1e-9).map((p) => Math.abs(p[0])))) + 0.5e-3;
  // (forward of the spar rod, between it and the root's leading edge: the boom ends just aft of the spar)
  const x0 = rootRibs[0].sparX + 0.25 * rootRibs[0].c, x1 = rootRibs[0].sparX + r + 0.5e-3, tieX = [x0 - (x0 - x1) / 3, x0 - (2 * (x0 - x1)) / 3];
  tieX.forEach((x, k) => {
    const tid = add(`tail tie ${k + 1}`, tagged(bandSlabs('tie', x, band[0], band[1], [-zt, zt], [yB - rb, yB + rb]), tie));
    join(tid, 'tail boom', 'round the boom');
    for (const q of rootRibs) { join(tid, `tail rib ${q.tag} 1`, RIB_HOLE); join(`tail rib ${q.tag} 1`, 'tail boom', 'against the boom, tied'); }
  });
  assumptions.push('tail (round 16): each V surface from its root on the boom\'s side (tail.z_m) at the boom\'s height along its dihedral: a spar rod on the unswept quarter chord, a trailing edge rod (a bay between ribs as one prism over its x), four ribs of the E387 thickness at the local chord from the root rib (against the boom) to the tip, and one T18R round the boom through a slot in the root rib; the covering is the surface the flight model flies');
}

const insideRect = (pts, c) => { let s = 0; for (let i = 0; i < 4; i++) { const a = pts[i], b = pts[(i + 1) % 4]; const cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]); if (cr > 0) s++; else if (cr < 0) s--; } return Math.abs(s) === 4; };
function edgesClear(pts, c, r) {
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length], ab = [b[0] - a[0], b[1] - a[1]], L2 = ab[0] ** 2 + ab[1] ** 2;
    const tt = Math.max(0, Math.min(1, ((c[0] - a[0]) * ab[0] + (c[1] - a[1]) * ab[1]) / L2));
    if (Math.hypot(c[0] - a[0] - tt * ab[0], c[1] - a[1] - tt * ab[1]) < r - 1e-9) return false;
  }
  return true;
}
