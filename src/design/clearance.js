// Assembly clearance (SPEC "Design freeze", "Assembly clearance", 2026-11-03): a design whose parts occupy the same
// space is not built from real parts. Every pair of parts that does not share a joint keeps at least 0.5 mm through
// the full stroke, and parts stacked on one pin keep 0.5 mm running clearance between them (the catalogue lists no
// thrust washer, so none is credited).
//
// Every part is a prism along body x: a cross-section in the body's (z, y) plane over an x interval. The
// cross-sections are rings (a disc is a ring with no bore), capsules (a round-ended bar: crank web, rocker arm, spar
// socket, coupler body, panels, tray sheet) and convex polygons with round holes (gearbox plates, packed boxes).
// A capsule contains the box or cylinder it stands for, so a gap measured to it never overstates the real one.
// The gap between two prisms is exact for prisms: sqrt(gx^2 + g2^2) when both the x gap gx and the in-plane gap g2
// are positive, the positive one when one is, and the smaller penetration (a negative number) when they overlap.
//
// Positions are the design's own numbers: the drive layout (joints.js driveLayout: model.js placeFourBar, gearTrain,
// gearPlanes), the linkage and plate parameters (params.linkage), the joint parts (params.joints, catalogue
// dimensions) and the airframe's stated positions (structure.airframe); where the design states no position for a
// part, the placement used is listed in the record's assumptions. Exempt from the rule: solids of one part, and two
// parts joined directly (a journal on its shaft, a part pressed, keyed, bonded or screwed to another, two gears in
// mesh). Two parts of one rigid assembly that are not joined to each other are checked like any other pair.
import { drivePose, gearAngles, linkageAt, journalDims, journalBodyLength, journalEdges, partsIndex, prepare, boxProtrusion, parseDims, packRegulator, loftHalf } from '../../sims/avis-shared/model.js';
import { byId } from './catalog.js';
import { AIRFRAME_DEFAULTS, mergeDefaults, lockFlangeOD, hullOf, lockFlangeCircles, panelFlanges, formerPieces, formerGeometry, clampFlange, seatRingBoss, lockPadCircles } from './airframe.js';
import { driveLayout as driveLayoutOf } from './joints.js';
import { rockerHub, pinWasher, crankPinEnd, hornStack, hornSide, shaftEndRetainer, actuatorBond } from './linkage.js';
import { fastenerGeometry, completenessRecord } from './completeness.js';
import { sequenceRecord } from './sequence.js';
import { wingParts, tailParts } from './wing-solids.js';
import { massPropsRecord } from './massprops.js';
import { tailMechParts, tailChains, TAIL_INTERFACE_RULES } from './tail-mech.js';
import { chainRecord, flapChains, ratingsRecord } from './chains.js';
import { twistSupportRows, sectionRecord, twistChains, TWIST_INTERFACE_RULES } from './twist-drive.js';
import { MESH, KEYWAY, THROUGH, THREAD, RIB_HOLE, BORE, GROOVE, EXEMPT_JOINTS } from './join-reasons.js';
import { interfaceLedger, RETAIN_INTERFACE_RULES, ACTUATOR_BOND, ACTUATOR_BONDED } from './interfaces.js';
import { needsParts, harnessTies, buildNeedsRecord, NEEDS_INTERFACE_RULES } from './build-needs.js';
import { podParts, coveringRecord, POD_INTERFACE_RULES } from './pod-structure.js';

const TWO_PI = 2 * Math.PI;
export const CLEARANCE_M = 0.0005;

// ---- plane geometry ---------------------------------------------------------------------------------------------
const sub = (p, q) => [p[0] - q[0], p[1] - q[1]];
const dot = (p, q) => p[0] * q[0] + p[1] * q[1];
const len = (p) => Math.hypot(p[0], p[1]);
const lerp = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];

/** Distance from point p to segment ab. */
export function segPointDist(p, a, b) {
  const ab = sub(b, a), L2 = dot(ab, ab);
  const t = L2 > 0 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / L2)) : 0;
  return len(sub(p, lerp(a, b, t)));
}

/**
 * Distance between segments ab and cd (0 when they cross). Round 16: a cross product within rounding of zero is not a
 * side (two segments on one line, a wing's spar and its torque rod, would otherwise read as crossing); the endpoint
 * distances then give the exact answer, 0 where they overlap.
 */
export function segSegDist(a, b, c, d) {
  const cr = (o, p, q) => (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
  const eps = 1e-12 * len(sub(b, a)) * len(sub(d, c));
  const d1 = cr(c, d, a), d2 = cr(c, d, b), d3 = cr(a, b, c), d4 = cr(a, b, d);
  const opp = (u, v) => (u > eps && v < -eps) || (u < -eps && v > eps);
  if (opp(d1, d2) && opp(d3, d4)) return 0;
  return Math.min(segPointDist(a, c, d), segPointDist(b, c, d), segPointDist(c, a, b), segPointDist(d, a, b));
}

/** Signed distance from p to a convex polygon (any winding): positive outside, negative inside. */
export function polySd(pts, p) {
  let inside = true, worst = -Infinity, out = Infinity;
  const n = pts.length;
  const sgn = Math.sign(area2(pts)) || 1;
  for (let i = 0; i < n; i++) {
    const a = pts[i], b = pts[(i + 1) % n], e = sub(b, a), L = len(e);
    if (L === 0) continue;
    // outward normal for a counter-clockwise polygon is (e_y, -e_x)
    const s = sgn * ((p[0] - a[0]) * e[1] - (p[1] - a[1]) * e[0]) / L;
    if (s > 0) inside = false;
    worst = Math.max(worst, s);
    out = Math.min(out, segPointDist(p, a, b));
  }
  return inside ? worst : out;
}
function area2(pts) {
  let A = 0;
  for (let i = 0; i < pts.length; i++) { const p = pts[i], q = pts[(i + 1) % pts.length]; A += p[0] * q[1] - q[0] * p[1]; }
  return A;
}

/** Golden-section search for the smallest f on [lo, hi]: -> [f, x] (unimodal f assumed). */
function goldenMin(f, lo, hi, iters) {
  const g = (Math.sqrt(5) - 1) / 2;
  let x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo), f1 = f(x1), f2 = f(x2);
  for (let k = 0; k < iters; k++) {
    if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - g * (hi - lo); f1 = f(x1); } else { lo = x1; x1 = x2; f1 = f2; x2 = lo + g * (hi - lo); f2 = f(x2); }
  }
  return f1 < f2 ? [f1, x1] : [f2, x2];
}

/** Smallest signed polygon distance along segment ab (convex in t, so a golden-section search finds it). */
function segPolySd(pts, a, b) {
  const f = (t) => polySd(pts, lerp(a, b, t));
  return Math.min(goldenMin(f, 0, 1, 40)[0], f(0), f(1));
}

/** Signed gap of two convex polygons: separation along the best axis (SAT) when they overlap, exact distance when apart. */
function polyPolyGap(P, Q) {
  let sep = -Infinity;
  for (const [A, B] of [[P, Q], [Q, P]]) {
    for (let i = 0; i < A.length; i++) {
      const e = sub(A[(i + 1) % A.length], A[i]), L = len(e);
      if (L === 0) continue;
      const nrm = [e[1] / L, -e[0] / L];
      const pa = A.map((p) => dot(p, nrm)), pb = B.map((p) => dot(p, nrm));
      sep = Math.max(sep, Math.min(...pb) - Math.max(...pa), Math.min(...pa) - Math.max(...pb));
    }
  }
  if (sep <= 0) return sep;
  return Math.min(...P.map((p) => polySd(Q, p)), ...Q.map((q) => polySd(P, q)));
}

const holesGap = (holes, far, r) => Math.max(-Infinity, ...(holes ?? []).map((h) => h.r - far(h.c) - r));

/**
 * Signed in-plane gap of two cross-sections: the distance between them when positive, minus the smaller overlap
 * when negative. ring { c, ro, ri }, cap { a, b, r }, poly { pts, holes: [{ c, r }] }.
 */
export function gap2(A, B) {
  if (A.kind > B.kind) [A, B] = [B, A]; // order: cap < poly < ring
  if (A.kind === 'ring' && B.kind === 'ring') {
    const dd = len(sub(A.c, B.c));
    return Math.max(dd - A.ro - B.ro, (A.ri ?? 0) - dd - B.ro, (B.ri ?? 0) - dd - A.ro);
  }
  if (A.kind === 'cap' && B.kind === 'ring') {
    const outer = segPointDist(B.c, A.a, A.b) - A.r - B.ro;
    const inHole = (B.ri ?? 0) - Math.max(len(sub(A.a, B.c)), len(sub(A.b, B.c))) - A.r;
    return Math.max(outer, inHole);
  }
  if (A.kind === 'cap' && B.kind === 'cap') return segSegDist(A.a, A.b, B.a, B.b) - A.r - B.r;
  if (A.kind === 'poly' && B.kind === 'ring') {
    const inHole = (B.ri ?? 0) - Math.max(...A.pts.map((p) => len(sub(p, B.c))));
    return Math.max(polySd(A.pts, B.c) - B.ro, inHole, holesGap(A.holes, (c) => len(sub(B.c, c)), B.ro));
  }
  if (A.kind === 'cap' && B.kind === 'poly') {
    return Math.max(segPolySd(B.pts, A.a, A.b) - A.r, holesGap(B.holes, (c) => Math.max(len(sub(A.a, c)), len(sub(A.b, c))), A.r));
  }
  if (A.kind === 'poly' && B.kind === 'poly') return polyPolyGap(A.pts, B.pts);
  throw new Error(`gap2: ${A.kind} / ${B.kind}`);
}

/** Signed gap of two prisms ([x0, x1] and a cross-section each). */
export function gap3(xa, xb, g2) {
  const gx = Math.max(xb[0] - xa[1], xa[0] - xb[1]);
  if (gx > 0 && g2 > 0) return Math.hypot(gx, g2);
  if (gx > 0) return gx;
  if (g2 > 0) return g2;
  return Math.max(gx, g2);
}

export const ring = (c, ro, ri = 0) => ({ kind: 'ring', c, ro, ri });
export const cap = (a, b, r) => ({ kind: 'cap', a, b, r });
export const poly = (pts, holes = []) => ({ kind: 'poly', pts, holes });
const boxYZ = (z0, z1, y0, y1) => { const p = poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]); p.withHoles = (h) => ({ ...p, holes: h }); return p; };

/**
 * Round 18.2 (the film's request): a fastener lying across x (its own axis along y, z, span or n) has its true
 * round section in the plane that contains x -- here, a circle of radius r about xc -- which this model (a prism
 * along x, one section per solid) cannot draw as one solid. N slabs along x approximate it, each the shape at the
 * circle's true half-width at the slab's edge nearest xc (so the group circumscribes the circle: never a smaller
 * section than the true one, so the clash check never misses what the true cylinder would find), narrowing toward
 * the real section instead of staying one flat box. shapeAt(w) -> the slab's (z, y) shape at half-width w <= r.
 * n (6-8 is enough: see each caller's stated error) -> [{x: [xi, xi1], shape}, ...] over [xc - r, xc + r].
 */
export function xSlabs(xc, r, shapeAt, n = 8) {
  const x0 = xc - r, x1 = xc + r;
  return Array.from({ length: n }, (_, k) => {
    const xi = x0 + (k / n) * (x1 - x0), xi1 = x0 + ((k + 1) / n) * (x1 - x0);
    const dNear = Math.max(0, Math.min(Math.abs(xi - xc), Math.abs(xi1 - xc)));
    return { x: [xi, xi1], shape: shapeAt(Math.sqrt(Math.max(0, r * r - dNear * dNear))) };
  });
}
const mirror = (p) => [-p[0], p[1]];
const dir = (a) => [Math.cos(a), Math.sin(a)];
const add2 = (p, q, k = 1) => [p[0] + k * q[0], p[1] + k * q[1]];
const mm = (v) => v / 1000;
/** A flat-ended bar of half-width w along a -> b, from ra past a to rb short of b, as a convex polygon. */
function bar(a, b, ra, rb, w) {
  const L = len(sub(b, a)), u = L > 0 ? [(b[0] - a[0]) / L, (b[1] - a[1]) / L] : [1, 0], v = [-u[1], u[0]];
  const p0 = add2(a, u, ra), p1 = add2(b, u, -rb);
  return poly([add2(p0, v, w), add2(p1, v, w), add2(p1, v, -w), add2(p0, v, -w)]);
}

/** A capsule of radius r along a -> b, starting ra from a and ending rb short of b (a bar between two round ends). */
function between(a, b, ra, rb, r) {
  const L = len(sub(b, a)), u = L > 0 ? [(b[0] - a[0]) / L, (b[1] - a[1]) / L] : [0, 0];
  return cap(add2(a, u, ra), add2(b, u, -rb), r);
}

// ---- the assembly ---------------------------------------------------------------------------------------------

/**
 * Where a journal sits in a gearbox plate, by the design's own convention (the same as linkage.js plateSeatLength):
 * with plates.flange 'face' (v22 on) a flanged bushing's flange sits on the plate's outer face and its body
 * (b1 - b2) runs through the plate inward; otherwise (v21) the whole part runs inward from the outer face, its flange
 * counterbored into the plate at that face. A bearing sits flush with the outer face, inward. -> { body, flange|null } (m).
 */
export function plateJournalX(plates, side, xPlate, tp, dims, outward = false) {
  // inward is +x from the aft plate, -x from the forward plate; `outward` (round 14: the motor's outboard bearing when the
  // motor is inside the box, its pinion forward of the plate) seats the part from the inner face outward instead
  const sgn = (side === 'aft' ? 1 : -1) * (outward ? -1 : 1);
  const outer = xPlate - sgn * tp / 2;
  const iv = (a, b) => [Math.min(a, b), Math.max(a, b)];
  const body = journalBodyLength(plates, dims);
  if (!(dims.flangeB > 0)) return { body: iv(outer, outer + sgn * body), flange: null };
  if (body < dims.B) return { body: iv(outer, outer + sgn * body), flange: iv(outer - sgn * dims.flangeB, outer) };
  return { body: iv(outer, outer + sgn * body), flange: iv(outer, outer + sgn * dims.flangeB) };
}

/** Length (m) of a plate boss beyond the plate: as long as the journal's body in the seat, less the plate. */
export function bossBeyond(plates, tp, dims) {
  return Math.max(0, journalBodyLength(plates, dims) - tp);
}

/**
 * Round 17b (audit item 16): a plate boss's x span: the part of the journal's own body (plateJournalX) that lies
 * beyond the plate [xp - tp/2, xp + tp/2], on whichever side the body runs (inward for every seat but the motor's
 * outboard bearing, seated from the plate's inner face outward: before round 17b the boss always grew inward, into
 * the motor can). bossBeyond long. -> [x0, x1] (m) or null when the body does not leave the plate.
 */
export function bossSpan(body, xp, tp) {
  const p0 = xp - tp / 2, p1 = xp + tp / 2;
  if (body[1] > p1 + 1e-12) return [p1, body[1]];
  if (body[0] < p0 - 1e-12) return [body[0], p0];
  return null;
}

/**
 * Round 17b: whether a plate seat is bossed (plates.bossed lists 'joint/support'; a joint with one plate support, the
 * motor's outboard bearing, is listed under linkage.js's support name 'plate'), and the boss's wall (m): the seat's
 * own plates.boss_walls_mm entry, else plates.boss_wall_mm.
 */
export function plateBoss(PL, jname, support) {
  const keys = [`${jname}/${support}`, ...(jname === 'motor shaft outboard' ? [`${jname}/plate`] : [])];
  const key = keys.find((k) => (PL.bossed ?? []).includes(k));
  return key ? { key, wall: mm(PL.boss_walls_mm?.[key] ?? PL.boss_wall_mm) } : null;
}

/** What every part builder needs: the layout, the design's numbers and the lists it fills. */
function assemblyContext(d, catalog, L, lay, airframe) {
  const parts = [], joins = [], assumptions = [];
  const P = lay.planes, n = lay.n, PL = L.plates;
  const jd = (id) => journalDims(byId(catalog, id).specs);
  const J = d.params.joints;
  const A = mergeDefaults(AIRFRAME_DEFAULTS, L.airframe ?? {});
  return {
    d, catalog, L, lay, P, n, PL, J, A, parts, joins, assumptions, Ly: d.params.layout ?? {},
    x0: lay.x0, fb: lay.fb, sh: lay.train.shafts, plateT: P.plateT ?? 0.002,
    tA: mm(PL.t_mm), tF: mm(PL.t_fwd_mm ?? PL.t_mm), jd, eye: jd(J.couplerEyes), shJ: jd(J.shoulder),
    af: airframe?.parts, railR: mm(A.rail.od_mm) / 2, span: mm(J.shoulderSpan_mm ?? 24),
    plateIds: { aft: 'gearbox plate aft', fwd: 'gearbox plate forward' },
    add: (id, solids, extra = {}) => addPart(parts, id, solids, extra),
    join: (a, b, why) => joins.push([a, b, why]),
    shaftPart: (j) => (j === 0 ? 'crank' : j === n ? 'motor' : `gear shaft ${j}`),
  };
}
/** A fixed solid (its section) and a moving one (its section as a function of the drive pose). */
export const S = (name, x, shape) => ({ name, x, shape });
export const M = (name, x, at) => ({ name, x, at });
/** Solids tagged with their BOM line (completeness.js resolves the tag) and the item they belong to. */
export const tagged = (solids, bom, item) => solids.map((s) => ({ ...s, bom: s.bom ?? bom, ...(item != null || s.item != null ? { item: s.item ?? item } : {}) }));

/**
 * Round 16 (SPEC "Completeness and assembly order"): a part with its solids, each tagged with the BOM line it is
 * (extra.bom; a solid's own tag wins) and its item of that line (extra.item, else the part); part.bom lists its lines.
 */
export function addPart(parts, id, solids, extra = {}) {
  const { bom, item, ...rest } = extra;
  const sol = bom != null || item != null ? tagged(solids, bom, item) : solids;
  parts.push({ id, solids: sol, bom: [...new Set(sol.map((s) => s.bom).filter(Boolean))], ...rest });
  return id;
}

/** The solids and joints of a design's assembly; moving solids are functions of the drive pose. */
export function assembly(d, catalog, L, lay, airframe) {
  const ax = assemblyContext(d, catalog, L, lay, airframe);
  crankParts(ax);
  encoderParts(ax);
  gearParts(ax);
  plateParts(ax);
  const delta = ax.fb.alpha + linkageAt(lay.law, d.drive.ampNominal).psiMid - lay.law.phiMean;
  for (const [tag, sg] of [['R', 1], ['L', -1]]) sideParts(ax, tag, sg, delta);
  ax.assumptions.push(`rocker arm centred on the horn boss (the physics states only its section); horn boss ${ax.Ly.horn_gap_mm ?? 1} mm from the coupler eye (forward of it${ax.Ly.horns ? `; ${Object.entries(ax.Ly.horns).map(([t, w]) => `${t} ${w}`).join(', ')}` : ''}), horn pin from the boss\'s far face through the eye to ${ax.Ly.horn_pin_past_mm ?? 1} mm past it (linkage.js horn pin); spar socket along the spar from the hub`);
  // Frame parts the airframe states (round 8): rails, panels, boom and clamps, trays.
  if (ax.af) frameParts(ax);
  if (ax.af?.carriers) carrierParts(ax);
  if (ax.af?.support?.pitot) pitotParts(ax);
  if (ax.af?.fixings) fixingParts(ax);
  // round 17: the pod's formers, longerons and end discs (pod-structure.js) in place of round 16's balsa rings
  if (ax.af && d.params.pod) podParts(ax);
  else if (ax.af?.formers) formerParts(ax);
  tailParts(ax);
  return { parts: ax.parts, joins: ax.joins, assumptions: ax.assumptions };
}

/**
 * A capsule a-b of radius r as the convex polygon round it (n points per round end, circumscribed so it contains the
 * capsule), with holes: a part with a round end that has a bore (round 16: the crank web on its shaft).
 */
export function capsulePoly(a, b, r, holes = [], n = 16) {
  const R = r / Math.cos(Math.PI / (2 * n)), u = sub(b, a), L = len(u), e = L > 0 ? [u[0] / L, u[1] / L] : [1, 0], th = Math.atan2(e[1], e[0]);
  const end = (c, a0) => Array.from({ length: n + 1 }, (_, k) => { const t = a0 + (k / n) * Math.PI; return [c[0] + R * Math.cos(t), c[1] + R * Math.sin(t)]; });
  return poly([...end(b, th - Math.PI / 2), ...end(a, th + Math.PI / 2)], holes);
}

/**
 * Round 17: the convex hull of two circles (c1, r1) and (c2, r2), as a polygon that contains both (n points per circle,
 * circumscribed), with round holes: the crank web with its hub boss larger than its throw end.
 */
export function hullPoly(c1, r1, c2, r2, holes = [], n = 32) {
  const pts = [];
  for (const [c, r] of [[c1, r1], [c2, r2]]) { const R = r / Math.cos(Math.PI / n); for (let k = 0; k < n; k++) { const t = (k / n) * TWO_PI; pts.push([c[0] + R * Math.cos(t), c[1] + R * Math.sin(t)]); } }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = (list) => { const h = []; for (const p of list) { while (h.length >= 2 && cross(h[h.length - 2], h[h.length - 1], p) <= 0) h.pop(); h.push(p); } h.pop(); return h; };
  return poly([...half(pts), ...half([...pts].reverse())], holes);
}

/**
 * Crank: web, pin, shaft and wheel seat in one piece; round 16 (crank.web.separate): the web and its pin one part, keyed
 * on the shaft's aft stub (its key in its keyseat and keyway, KEYWAY), the shaft with its stub and wheel seat another,
 * so the shaft goes into the aft plate's bushing from forward and the web onto its stub from aft.
 */
function crankParts(ax) {
  const { L, P, n, fb, lay, eye, plateT, add, join, catalog } = ax;
  const C = L.crank, tw = mm(C.web_t_mm), bw = mm(C.web_b_mm);
  const xw = P.crankArm - tw / 2;
  const { pinSolids, washer } = crankPin(ax, xw);
  const wheelN = lay.train.stages[n - 1].wheel;
  if (C.web?.separate) {
    const W = C.web, rs = mm(C.shaft_od_mm) / 2, web = [P.crankArm - tw / 2, P.crankArm + tw / 2];
    // round 17 (audit items 13 and the axial fixing): a hub boss round the bore (W.hub_od_mm) and, at the web's aft face,
    // an end lip (W.lip_mm) over the stub's end with the axial screw's clearance hole; the stub (and the key) under the
    // rest of the web, the screw's head on the lip's aft face, its shank into the stub's tapped end
    const lip = mm(W.lip_mm ?? 0), rh = mm(W.hub_od_mm ?? C.web_b_mm) / 2, bore = [web[0] + lip, web[1]];
    const ag = W.axial ? fastenerGeometry(byId(catalog, W.axial.screw).specs ?? {}, W.axial.screw) : null;
    const webShape = (q, holes) => (rh > bw / 2 + 1e-9 ? hullPoly(q.O2, rh, q.A, bw / 2, holes) : capsulePoly(q.O2, q.A, bw / 2, holes));
    add('crank web', [
      M('web', bore, (q) => webShape(q, [{ c: q.O2, r: rs }])),
      ...(lip > 0 ? [M('web end lip', [web[0], bore[0]], (q) => webShape(q, ag ? [{ c: q.O2, r: (W.axial.countersunk ? ag.head_d : ag.d) / 2 + mm(W.axial.clearance_mm ?? 0.2) }] : []))] : []),
      ...pinSolids,
    ], { bom: 'crank web and pin' });
    washer('crank web');
    // round 17b (retain2): a DIN 471 circlip at the forward bushing's flange (retention.crankCirclip) stops the shaft
    // aft; the web on the aft flange stops it forward. Its groove splits the shaft: 'shaft' up to the flange's outer face,
    // the groove at the ring's published diameter, 'shaft forward' beyond.
    const cc = crankCirclip(ax), ri = mm(C.shaft_id_mm ?? 0) / 2, x1 = P.crankEnd ?? P.plateFwd + plateT / 2 + 0.001;
    const x0 = web[1] + (ag && W.axial.solid_mm ? mm(W.axial.solid_mm) : 0);
    const land = actuatorLand(L, P, x0, x1); // round 18: the encoder actuator's bond land (linkage.js actuatorBond)
    add('crank', [
      S('stub', bore, ring(fb.O2, rs)), // solid under the web (its keyseat cut to the key)
      // round 17: solid for the axial screw's thread beyond the web (W.axial.solid_mm), then bored
      ...(ag && W.axial.solid_mm ? [S('shaft solid end', [web[1], web[1] + mm(W.axial.solid_mm)], ring(fb.O2, rs))] : []),
      ...(cc ? [S('shaft', [x0, cc.face], ring(fb.O2, rs, ri)), S('shaft forward groove', [cc.face, cc.face + cc.m], ring(fb.O2, cc.ri, ri)), ...crankForwardSolids('shaft forward', [cc.face + cc.m, x1], land, fb.O2, rs, ri)] : crankForwardSolids('shaft', [x0, x1], land, fb.O2, rs, ri)),
      ...(C.seat_od_mm ? [S('wheel seat', [P.stage[n - 1] - wheelN.face / 2, P.stage[n - 1] + wheelN.face / 2], ring(fb.O2, mm(C.seat_od_mm) / 2, rs))] : []),
    ], { bom: 'crank shaft' });
    if (cc) {
      const cid = add('crank shaft circlip forward', [S('circlip', [cc.face, cc.face + cc.t], ring(fb.O2, cc.ro, cc.ri))], { bom: cc.id });
      join(cid, 'crank', GROOVE); join(cid, `crank shaft journal forward (${ax.J.crankShaft})`, 'against its flange');
    }
    if (ag) {
      // the axial screw along the crank axis from aft: its head on the lip's aft face, its shank through the lip into the stub
      // a countersunk head lies in the lip's countersink, flush with the web's aft face (nothing stands aft of the web,
      // where the coupler eyes run 0.5 mm off it); its length includes the head
      const screw = W.axial.countersunk ? [S('head', [web[0], web[0] + ag.head_k], ring(fb.O2, ag.head_d / 2)), S('shank', [web[0] + ag.head_k, web[0] + ag.L], ring(fb.O2, ag.d / 2))] : screwX(fb.O2, web[0], 1, ag).screw;
      const sid = add('crank web axial screw', screw, { bom: W.axial.screw });
      join('crank web', 'crank', 'keyed on the shaft\'s aft stub; held axially by its screw (crank web axial screw)');
      join(sid, 'crank web', W.axial.countersunk ? 'fastened (its countersunk head in the lip\'s countersink)' : 'fastened (its head on the face)'); join(`${sid}/shank`, 'crank web', THROUGH);
      join(`${sid}/shank`, 'crank', THREAD); join(sid, 'crank', 'screwed into the stub\'s tapped end');
    } else join('crank web', 'crank', 'keyed on the shaft\'s aft stub, held there with LOCTITE 638');
    // the web's key: b x h over the web's bore, its bottom shaft_depth_mm into the stub; round 17b, turning with the crank
    const K = C.web.key;
    const kid = add('key, crank web', [turningKey(lay.train, 0, fb.O2, rs, K, [bore[1] - mm(K.L_mm), bore[1]], keyRefAngles(ax)[0])], { bom: 'key, crank web' });
    join(kid, 'crank web', KEYWAY); join(kid, 'crank', KEYWAY);
    return;
  }
  add('crank', [
    M('web', [P.crankArm - tw / 2, P.crankArm + tw / 2], (q) => cap(q.O2, q.A, bw / 2)),
    ...pinSolids,
    S('shaft', [P.crankArm + tw / 2, P.crankEnd ?? P.plateFwd + plateT / 2 + 0.001], ring(fb.O2, mm(C.shaft_od_mm) / 2, mm(C.shaft_id_mm ?? 0) / 2)),
    ...(C.seat_od_mm ? [S('wheel seat', [P.stage[n - 1] - wheelN.face / 2, P.stage[n - 1] + wheelN.face / 2], ring(fb.O2, mm(C.seat_od_mm) / 2, mm(C.shaft_od_mm) / 2))] : []),
  ], { bom: 'crank: shaft, web and pin in one piece' });
  washer('crank');
}

/**
 * Round 17b (audit item 7): the crank pin's solids from the web's aft face xw: with a retaining washer
 * (retention.crankPinWasher) it runs on past the left eye's free face by the washer's groove and land, the groove cut
 * to the washer's published diameter; else to 1 mm past the eye. washer(pinPart) adds the washer on that part.
 */
function crankPin(ax, xw) {
  const { L, P, eye, catalog } = ax;
  const pw = pinWasher(catalog, L.retention, 'crank'), pinEnd = crankPinEnd(P, eye.B, pw);
  const pg = pw ? pinEndGroove(P.couplerL - eye.B / 2, -1, pw) : null;
  const pinSolids = pg
    ? [M('pin', [pg.groove[1], xw], (q) => ring(q.A, eye.d / 2)), M('pin groove', pg.groove, (q) => ring(q.A, pw.ri)), M('pin tip', pg.tip, (q) => ring(q.A, eye.d / 2))]
    : [M('pin', [pinEnd, xw], (q) => ring(q.A, eye.d / 2))];
  return { pinSolids, washer: (pinPart) => (pg ? pinWasherPart(ax, 'crank pin washer', pw, pg, (q) => q.A, pinPart, 'coupler L', 'coupler L crank bushing') : null) };
}


/** Round 18: the actuator's bond land on the crank, [x0, x1] (m) and its radius, or null (no encoder or no bond). */
function actuatorLand(L, P, x0, x1) {
  const ab = P.encoder ? actuatorBond(L) : null;
  if (!ab) return null;
  const x = [Math.max(P.encoder.actuator[0], x0), Math.min(P.encoder.actuator[1], x1)];
  return x[1] > x[0] ? { x, r: mm(ab.land_d_mm) / 2 } : null;
}
/** Round 18: the crank's last piece over [xa, x1], split at the land when one lies inside it (then 'shaft land', 'shaft end'). */
function crankForwardSolids(name, [xa, x1], land, c, rs, ri) {
  if (!land || !(xa < land.x[0])) return [S(name, [xa, x1], ring(c, rs, ri))];
  const end = land.x[1] < x1 ? [S('shaft end', [land.x[1], x1], ring(c, rs, ri))] : [];
  return [S(name, [xa, land.x[0]], ring(c, rs, ri)), S('shaft land', land.x, ring(c, land.r, ri)), ...end];
}

/** Round 14: the crank encoder where the planes put it (model.js clearancePlanes, encoder 'front'): readhead and actuator rings. */
function encoderParts(ax) {
  const { d, catalog, P, fb, af, add, join, assumptions } = ax;
  const e = P.encoder;
  if (!e) return;
  const fx0 = af?.support?.readhead;
  // round 16: with its fixing stated, the readhead as a polygon round its outside diameter with its bore and its three
  // 4 mm mounting holes (the catalogue's), so the screws pass through it
  const rhHoles = fx0 ? [{ c: fb.O2, r: e.id.readhead / 2 }, ...fx0.holes_m.map((c) => ({ c, r: (fx0.hole_d_mm ?? 4) / 2000 }))] : null;
  const rh = add('crank encoder readhead', [S('ring', e.readhead, rhHoles ? circlePoly(fb.O2, e.od.readhead / 2, rhHoles) : ring(fb.O2, e.od.readhead / 2, e.id.readhead / 2))], { bom: d.params.parts?.crankEncoder ?? 'sensor-rls-orbis-br10' });
  const ac = add('crank encoder actuator', [S('ring', e.actuator, ring(fb.O2, e.od.actuator / 2, e.id.actuator / 2))], { bom: d.params.parts?.encoderMagnet ?? 'sensor-rls-orbis-actuator-10mm' });
  // round 18: bonded on its land when the design bonds it (linkage.js actuatorBond), else on RLS's set screws alone
  const bonded = !!actuatorBond(d.params.linkage);
  for (const sol of shaftSolidsOver(ax, 0, e.actuator)) join(ac, `crank/${sol}`, bonded ? ACTUATOR_BONDED : 'on the crank shaft by its three set screws (maker\'s fixing)');
  const front = e.readhead[0] > P.plateFwd;
  const fx = af?.support?.readhead;
  if (fx) readheadFixing({ add, join, assumptions, catalog }, rh, e, fx, af.carriers.riser);
  else join(rh, front ? 'gearbox plate forward' : 'gearbox plate aft', `on its bracket to ${front ? 'the forward plate\'s outer face' : 'the aft plate\'s crank boss'}`);
  assumptions.push('crank encoder (round 14): readhead as its ring envelope (outer diameter, through-hole, thickness with connector), the actuator at the ride height from the readhead\'s face, coaxial with the crank (model.js clearancePlanes)');
}

/**
 * A circle of radius r about c as a convex polygon that contains it, n sides, a flat at +z, -z, +y and -y (so its
 * extents are the circle's), with round holes: for a part drawn round a circle that has holes besides its bore.
 */
export function circlePoly(c, r, holes = [], n = 48) {
  const R = r / Math.cos(Math.PI / n);
  return poly(Array.from({ length: n }, (_, k) => { const a = ((k + 0.5) / n) * TWO_PI; return [c[0] + R * Math.cos(a), c[1] + R * Math.sin(a)]; }), holes);
}

/**
 * A screw along x (round 16): its head on the face at xFace (the head on the side away from `dir`), its shank `dir`
 * (+1 or -1) from that face for its catalogue length; and its nut, if any, on the face at xNut (the nut toward `dir`).
 * g: fastenerGeometry of the screw, ng of the nut (completeness.js). A nut is drawn by its corner circle.
 */
export function screwX(c, xFace, dir, g, xNut = null, ng = null) {
  const iv = (a, b) => [Math.min(a, b), Math.max(a, b)];
  const screw = [S('head', iv(xFace, xFace - dir * g.head_k), ring(c, g.head_d / 2)), S('shank', iv(xFace, xFace + dir * g.L), ring(c, g.d / 2))];
  const nut = xNut != null && ng ? [S('nut', iv(xNut, xNut + dir * ng.m), nutSection(c, ng, g))] : [];
  return { screw, nut };
}

/**
 * Round 17b (retain2, the film's request): a nut's true section on an axis along x: the hexagon across its flats s
 * (corners on the z axis), bored to its catalogue minor diameter (the nut entry's 'less a ... mm minor-diameter bore',
 * fastenerGeometry bore), else the screw's own diameter. Its threads overlap the shank, so a nut is always joined to
 * its screw's shank as THREAD. -> poly.
 */
export function nutSection(c, ng, g) {
  const R = ng.s / Math.sqrt(3);
  return poly(Array.from({ length: 6 }, (_, k) => [c[0] + R * Math.cos((k * Math.PI) / 3), c[1] + R * Math.sin((k * Math.PI) / 3)]), [{ c, r: (ng.bore ?? g.d) / 2 }]);
}

/**
 * Round 15, as separate parts in round 16: the readhead's own fixing (BRD01: 3 x M2 DIN 912 with spacers): each screw's
 * head on the readhead's back face and its shank through the readhead's 4 mm hole, its spacer, the riser's web and its
 * nut on the web's forward face.
 */
function readheadFixing({ add, join, assumptions, catalog }, rh, e, fx, riser) {
  const mm1 = (v) => v / 1000;
  const g = fastenerGeometry(byId(catalog, fx.screw).specs ?? {}, fx.screw), ng = fastenerGeometry(byId(catalog, fx.nut).specs ?? {}, fx.nut);
  fx.holes_m.forEach((c, i) => {
    const { screw, nut } = screwX(c, e.readhead[0], 1, g, riser.x[1], ng);
    const sid = add(`readhead screw ${i + 1}`, screw, { bom: fx.screw });
    const pid = add(`readhead spacer ${i + 1}`, [S('spacer', [e.readhead[1], e.readhead[1] + mm1(fx.spacer_mm)], ring(c, mm1(fx.spacer_od_mm) / 2, mm1(fx.spacer_id_mm) / 2))], { bom: 'crank encoder spacers' });
    const nid = add(`readhead nut ${i + 1}`, nut, { bom: fx.nut });
    join(sid, rh, 'through its mounting hole, the head on its back face');
    join(sid, pid, 'through the spacer');
    join(sid, 'nose tray riser', 'through its hole in the riser\'s web');
    join(sid, nid, 'threaded');
    join(`${sid}/shank`, nid, THREAD); // round 17b: the nut's minor-diameter bore on the shank's thread
    join(pid, rh, 'on the readhead\'s front face');
    join(pid, 'nose tray riser', 'on the riser\'s web');
    join(nid, 'nose tray riser', 'on the riser\'s forward face');
  });
  assumptions.push('crank encoder readhead (round 15; round 16 every screw, spacer and nut its own part): on the nose tray riser by the maker\'s fixing, 3 x M2 DIN 912 through its 4 mm mounting holes on the 34 mm circle, spacers of under 4 mm diameter to the riser\'s web (BRD01), nuts on the web\'s forward face; a nut its hexagon bored to its minor diameter (round 17b); the readhead as a polygon round its outside diameter with its bore and mounting holes');
}

const airframeOf = (ax) => ax.af;

/** Round 17: the washer/circlip's groove width (m), minimum land (n) and bore radius (m, metres), from its catalogue entry. */
const retGroove = (catalog, id) => { const s = byId(catalog, id).specs; return { m: mm(s.groove_width_m_mm), n: mm(s.min_groove_to_shaft_end_n_mm), ri: mm(s.groove_diameter_d2_mm) / 2 }; };
const retLand = (catalog, id) => { const g = retGroove(catalog, id); return g.m + g.n; };

/**
 * Round 17b (retain2): the crank shaft's forward circlip (retention.crankCirclip, a DIN 471 ring) against the forward
 * bushing's flange: its face (the flange's outer face, m), groove width m, land n, thickness t and its swept (d4) and
 * groove radii. -> { id, face, m, n, t, ro, ri } or null.
 */
function crankCirclip({ catalog, L, PL, P, tF, jd, J }) {
  const id = L.retention?.crankCirclip;
  if (!id) return null;
  const g = retGroove(catalog, id), f = fastenerGeometry(byId(catalog, id).specs ?? {}, id);
  return { id, face: journalEdges(PL, 'fwd', P.plateFwd, tF, jd(J.crankShaft)).outer, m: g.m, n: g.n, t: f.t, ro: f.od / 2, ri: g.ri };
}

/**
 * Round 17 (audit items 7, 8, F4/F5): a shaft's end solids where a washer or circlip sits in a groove against a
 * face at x = `face` (its journal's outer face): the shaft at full diameter from `start` (the end furthest from the
 * face) in to the groove's land, cut down to the washer/circlip's own bore over the groove's width, resuming full
 * diameter only at `face` itself (so a washer drawn at the groove's published bore never overlaps the shaft it sits
 * on). dir: +1 if `start` is aft of `face` (the aft end), -1 if forward of it. -> [S(...), ...].
 */
function shaftEndSegments(name, bom, sh_j, dShaft, start, face, dir, groove) {
  if (!groove) return [S(name, dir > 0 ? [start, face] : [face, start], ring(sh_j, dShaft / 2))].map((s) => ({ ...s, bom }));
  const landEnd = face - dir * groove.m, grooveX = [landEnd, face].sort((a, b) => a - b), tipX = [start, landEnd].sort((a, b) => a - b);
  return [S(name, tipX, ring(sh_j, dShaft / 2)), S(`${name} groove`, grooveX, ring(sh_j, groove.ri))].map((s) => ({ ...s, bom }));
}

/**
 * Round 17b (retain2): the solids of shaft part k (its shaft pieces, grooves, stub, wheel seat) that a part spanning xr
 * along x sits on or touches at an end, so a journal, gear, key or actuator is joined to the pieces under it (a groove cut in the
 * shaft splits it). -> names; ['shaft'] when the shaft part is not drawn yet.
 */
function shaftSolidsOver(ax, k, xr) {
  const p = ax.parts.find((q) => q.id === ax.shaftPart(k));
  if (!p) return ['shaft'];
  return p.solids.filter((s) => /^(shaft|wheel seat|stub)( |$)/.test(s.name) && Math.min(s.x[1], xr[1]) - Math.max(s.x[0], xr[0]) > -1e-9).map((s) => s.name);
}

/** Gear shafts, the motor and every gear: link j joins shaft j (wheel of stage n-1-j) to shaft j+1 (its pinion). */
function gearParts(ax0) {
  const { d, catalog, n } = ax0;
  const ms = byId(catalog, d.motor?.id ?? d.params.parts.motor).specs;
  const motorShaft = mm(ms.shaft_mm ?? 5);
  for (let j = 1; j < n; j++) gearShaftPart(ax0, j);
  motorParts(ax0, ms, motorShaft);
  stageGearParts(ax0, motorShaft);
}

/** A shaft runs plate to plate, 1 mm proud of each; round 14, to the far face of a gear it carries outside the plates. */
const gearFarEnd = (P, si, g) => P.stage[si] + g.face / 2 + (g.hubW ?? 0);

/**
 * Gear shaft j (round 17b, split out of gearParts): its shaft with its wheel seat and, at each end that takes a
 * retaining ring, the groove and land past the journal's outer face (round 17, audit items 7, 8, F4: read off the
 * catalogue, not the old 1 mm guess). Each end's ring is linkage.js shaftEndRetainer's: gear shaft 2's forward end, whose
 * washer (groove and land 1.94 mm) came within 0.36 mm of the DS1-60 overhung on gear shaft 3, takes a DIN 471-8 circlip
 * (0.9 + 0.6 mm), 0.80 mm clear of it.
 */
function gearShaftPart(ax0, j) {
  const { catalog, L, lay, P, n, PL, J, tA, tF, jd, sh, plateT, add } = ax0;
  const g = L.gearShafts[j - 1], Ret = L.retention ?? null;
  const edges = (side, bushId) => journalEdges(PL, side, side === 'aft' ? P.plateAft : P.plateFwd, side === 'aft' ? tA : tF, jd(bushId));
  const bushId = J.shafts[j - 1] ?? J.shafts.at(-1);
  const isBearing = byId(catalog, bushId).category === 'bearing';
  const kw = n - 1 - j, kp = n - j, wheel = lay.train.stages[kw].wheel, pinion = lay.train.stages[kp].pinion;
  const farGears = Math.max(gearFarEnd(P, kw, wheel), gearFarEnd(P, kp, pinion));
  const aftId = shaftEndRetainer(Ret, j, 'aft', isBearing), fwdId = shaftEndRetainer(Ret, j, 'forward', isBearing);
  const aftGroove = aftId ? retGroove(catalog, aftId) : null, fwdGroove = fwdId ? retGroove(catalog, fwdId) : null;
  const aftStart = aftGroove ? edges('aft', bushId).outer - retLand(catalog, aftId) : P.plateAft - plateT / 2 - 0.001;
  const xEnd = Math.max(fwdGroove ? edges('fwd', bushId).outer + retLand(catalog, fwdId) : P.plateFwd + plateT / 2 + 0.001, farGears);
  const dShaft = mm(g.d_mm), aftFace = aftGroove ? edges('aft', bushId).outer : aftStart, fwdFace = fwdGroove ? edges('fwd', bushId).outer : xEnd;
  add(`gear shaft ${j}`, [
    ...(aftGroove ? shaftEndSegments('shaft aft tip', `gear shaft ${j}`, sh[j], dShaft, aftStart, aftFace, 1, aftGroove) : []),
    S('shaft', [aftFace, fwdFace], ring(sh[j], dShaft / 2)),
    ...(fwdGroove ? shaftEndSegments('shaft forward tip', `gear shaft ${j}`, sh[j], dShaft, xEnd, fwdFace, -1, fwdGroove) : []),
    ...(g.seat_od_mm ? [S('wheel seat', [P.stage[kw] - wheel.face / 2, P.stage[kw] + wheel.face / 2], ring(sh[j], mm(g.seat_od_mm) / 2, dShaft / 2))] : []),
  ], { bom: `gear shaft ${j}` });
  if (Ret) gearShaftRetention(ax0, j, bushId, isBearing, [['aft', aftId], ['forward', fwdId]], edges);
}

/** The motor (its can and its shaft to the first pinion) and, round 15, its back-face bracket bonded on both rails. */
function motorParts(ax0, ms, motorShaft) {
  const { d, lay, P, n, sh, add, join, assumptions } = ax0;
  const md = parseDims(ms.dimensions_text_mm)?.values ?? [0.035, 0.04];
  const can = P.motorX ?? [P.motorAft, P.motorAft + md[1]], p1 = lay.train.stages[0].pinion;
  add('motor', [
    S('can', can, ring(sh[n], md[0] / 2)),
    // its shaft from its face to the first pinion: aft (the motor forward of the box) or, round 14, forward through the plate
    S('shaft', P.stage1 === 'overhung' ? [P.motorFace, gearFarEnd(P, 0, p1)] : [P.stage[0] - p1.face / 2, P.motorAft], ring(sh[n], motorShaft / 2)),
  ], { bom: d.motor?.id ?? d.params.parts.motor });
  // round 15: the motor on its back-face bracket (structure.airframe.parts.motor_bracket), bonded on both rails
  const mb = airframeOf(ax0)?.motor_bracket;
  if (mb) {
    add('motor bracket', [S('plate', mb.x, poly(mb.outline, [...mb.sleeves.map((q) => ({ c: q.c, r: q.ro })), { c: mb.stub.c, r: mb.stub.r }])), ...mb.sleeves.map((q) => S(`sleeve ${q.side}`, mb.x, ring(q.c, q.ro, q.ri)))], { bom: 'motor bracket' });
    join('motor bracket', 'motor', 'screwed into its back-face threads');
    for (const q of mb.sleeves) join('motor bracket', `shoulder rail ${q.side}`, 'its sleeve bonded on the rail');
    assumptions.push('motor bracket (round 15): the motor hangs by its back face (the maker\'s 4 x M3 threads, 25 mm across) on a plate bonded on both shoulder rails; its bell clears the forward plate and the rail\'s lock flange');
  } else join('motor', 'gearbox plate forward', 'screwed to the forward plate');
}

/** Every stage's wheel and pinion on its shaft (round 17b: joined to the shaft pieces under its bore), with its key. */
function stageGearParts(ax0, motorShaft) {
  const { L, lay, P, n, sh, add, join, shaftPart, assumptions } = ax0;
  const boreOn = (k) => (k === n ? motorShaft : mm(k === 0 ? L.crank.seat_od_mm ?? L.crank.shaft_od_mm : L.gearShafts[k - 1].d_mm));
  for (let si = 0; si < n; si++) {
    const st = lay.train.stages[si], j = n - 1 - si, x = P.stage[si];
    for (const [g, k, tag] of [[st.wheel, j, 'wheel'], [st.pinion, j + 1, 'pinion']]) {
      const bore = g.bore || boreOn(k), hubW = g.hub && g.hubW > 0 ? g.hubW : 0;
      const id = add(`stage ${si + 1} ${tag} ${g.id}`, [
        S('teeth', [x - g.face / 2, x + g.face / 2], ring(sh[k], g.ro, bore / 2)),
        ...(hubW ? [S('hub', [x + g.face / 2, x + g.face / 2 + hubW], ring(sh[k], g.hub / 2, bore / 2))] : []),
      ], { gear: { stage: si }, bom: g.id });
      for (const sol of shaftSolidsOver(ax0, k, [x - g.face / 2, x + g.face / 2 + hubW])) join(id, `${shaftPart(k)}/${sol}`, 'gear on its shaft (bore on the shaft or seat)');
      keyParts(ax0, g, k, tag, si, id, x);
    }
    join(`stage ${si + 1} wheel ${st.wheel.id}`, `stage ${si + 1} pinion ${st.pinion.id}`, MESH);
  }
  assumptions.push('gear hubs on the forward side of the face (model.js stagePlanes allots each stage its hub width there)');
}

/**
 * Round 17 (audit item 8, F4), round 17b (retain2): a gear shaft's own axial retention, outside its torque path
 * (linkage.js gearShaftRetentionMembers rates each part against its catalogue load).
 *  - A bushing shaft (1, 2): a retaining ring at each end (shaftEndRetainer: a DIN 6799 washer, or a DIN 471 circlip
 *    where the washer's land does not fit), outside the bushing's flange, against its outer face: the shaft is held
 *    between the two flanges, each on its plate's outer face.
 *  - The bearing shaft (3): nothing drawn. Both 628/8s are bonded, inner rings on the shaft and outer rings in their
 *    seats (LOCTITE 638, retention.bearingBonds), so the shaft is held at both plates.
 *    Round 17b dropped the round-17 DIN 471-8 circlip on the aft inner ring: its swept diameter d4 14.7 mm passes SKF's
 *    largest shaft abutment da max 10.0 mm for the 628/8 (it would ride the shield).
 * Round 17b dropped the round-17 spacer tubes: each gear is bonded on its shaft (linkage.js gearBond), and the tubes
 * did not hold what they were joined to: their 12 mm ends met the wheel seats, not the 15 mm wheel bores on gear shaft
 * 1, and turned against the stationary plate bosses at the bushings.
 * ends: [['aft' | 'forward', catalogue id | null], ...].
 */
function gearShaftRetention(ax0, j, bushId, isBearing, ends, edges) {
  const { catalog, sh, add, join, shaftPart } = ax0;
  for (const [end, rid] of ends) {
    if (!rid) continue;
    const g = fastenerGeometry(byId(catalog, rid).specs ?? {}, rid), side = end === 'aft' ? 'aft' : 'fwd', e = edges(side, bushId);
    const kind = /^circlip/.test(rid) ? 'circlip' : 'washer', jId = `${shaftPart(j)} journal ${end} (${bushId})`;
    const xs = end === 'aft' ? [e.outer - g.t, e.outer] : [e.outer, e.outer + g.t];
    const id = add(`gear shaft ${j} ${kind} ${end}`, [S(kind, xs, ring(sh[j], g.od / 2, g.id / 2))], { bom: rid });
    join(id, shaftPart(j), GROOVE);
    join(id, jId, isBearing ? `against the inner ring's ${end} face` : 'against its flange');
  }
}

/**
 * Round 16: a gear's parallel key (params.linkage crank.key, gearShafts[].wheelKey / pinionKey; linkage.js keyMember
 * sizes it and names its BOM line): a b x h box over the gear's face, its bottom `shaft_depth_mm` into the seat or
 * shaft, at the top of the shaft (the design states no angle). The keyseat and the gear's keyway are cut to it and not
 * drawn, so the key is exempt against its shaft and its gear (KEYWAY) and checked against everything else.
 */
function keyParts(ax, g, k, tag, si, gearId, x) {
  const { L, lay, n, sh, add, join, shaftPart } = ax;
  if (k === n) return; // the motor's pinion: bonded, no key
  const st = lay.train.stages[si], wheel = tag === 'wheel';
  const spec = k === 0 ? (wheel ? L.crank.key : null) : (wheel ? L.gearShafts[k - 1].wheelKey : L.gearShafts[k - 1].pinionKey);
  if (!spec) return;
  const rSeat = mm(k === 0 ? L.crank.seat_od_mm ?? L.crank.shaft_od_mm : (wheel ? L.gearShafts[k - 1].seat_od_mm ?? L.gearShafts[k - 1].d_mm : L.gearShafts[k - 1].d_mm)) / 2;
  const name = k === 0 ? `key, crank wheel ${st.wheel.id}` : wheel ? `key, wheel ${st.wheel.id} on gear shaft ${k}` : `key, pinion ${st.pinion.id} on gear shaft ${k}`;
  const xs = [x - mm(spec.L_mm) / 2, x + mm(spec.L_mm) / 2];
  const id = add(name, [turningKey(lay.train, k, sh[k], rSeat, spec, xs, keyRefAngles(ax)[k])], { bom: name });
  join(id, gearId, KEYWAY);
  for (const sol of shaftSolidsOver(ax, k, xs)) join(id, `${shaftPart(k)}/${sol}`, KEYWAY);
}

/** The crank angle of a drive pose as the film turns its crank and gears (bird.js: from +z toward +y, O2 to the pin). */
const crankAngleOf = (q) => Math.atan2(q.A[1] - q.O2[1], q.A[0] - q.O2[0]);
/** Round 17b: each shaft's angle (gearAngles) at the design's reference pose, phase 0 at the nominal amplitude. */
const keyRefAngles = ({ d, lay }) => gearAngles(lay.train, crankAngleOf(drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal)));

/**
 * Round 17b (retain2, the film's request): a parallel key as a moving solid that turns with its shaft k: its b x h box
 * over the seat (rSeat), its bottom shaft_depth_mm into it, at +y at the reference pose (shaft k at angle a0, as the
 * round-16 key was drawn) and turned with the shaft from there (model.js gearAngles from the crank angle, the film's own
 * convention: from +z toward +y, about the shaft's centre c). `sweep` is the static ring every pose of it lies in (its
 * bottom's and outer corners' radii): the clearance check skips a far pair by it, and the swept ring lies inside the
 * gear's and the shaft's own envelope, both exempt as KEYWAY, so sampling cannot miss a clash with any other part.
 * -> the solid.
 */
export function turningKey(train, k, c, rSeat, spec, xs, a0 = 0) {
  const hw = mm(spec.b_mm) / 2, y0 = rSeat - mm(spec.shaft_depth_mm), y1 = y0 + mm(spec.h_mm);
  const corners = [[-hw, y0], [hw, y0], [hw, y1], [-hw, y1]];
  const at = (q) => {
    const a = gearAngles(train, crankAngleOf(q))[k] - a0, ca = Math.cos(a), sa = Math.sin(a);
    return poly(corners.map(([z, y]) => [c[0] + z * ca - y * sa, c[1] + z * sa + y * ca]));
  };
  return { ...M('key', xs, at), sweep: ring(c, Math.hypot(hw, y1), Math.max(0, y0)) };
}

/** One journal in a plate seat, with the seat's boss where the design bosses it. Fills holes/bosses of its plate. */
function plateJournal(ax, jname, k, id, side, holes, bosses) {
  const { P, PL, sh, tA, tF, jd, add, join, shaftPart, plateIds } = ax;
  const dims = jd(id), xp = side === 'aft' ? P.plateAft : P.plateFwd, tp = side === 'aft' ? tA : tF;
  const support = side === 'aft' ? 'aft' : 'forward';
  const xs = plateJournalX(PL, side, xp, tp, dims, jname === 'motor shaft outboard' && P.stage1 === 'overhung');
  const pid = add(`${jname} journal ${support} (${id})`, [
    S('body', xs.body, ring(sh[k], dims.D / 2, dims.d / 2)),
    ...(xs.flange ? [S('flange', xs.flange, ring(sh[k], dims.flangeD / 2, dims.d / 2))] : []),
  ], { bom: id });
  // round 17: on the shaft's solids (the crank's solid end for its web's axial screw too); round 17b, those under it
  for (const sol of shaftSolidsOver(ax, k, [Math.min(xs.body[0], xs.flange?.[0] ?? Infinity), Math.max(xs.body[1], xs.flange?.[1] ?? -Infinity)])) join(pid, `${shaftPart(k)}/${sol}`, 'journal on its shaft');
  join(pid, plateIds[side], 'pressed in its plate seat');
  holes[side].push({ c: sh[k], r: dims.D / 2 });
  const pb = plateBoss(PL, jname, support), bx = pb ? bossSpan(xs.body, xp, tp) : null;
  if (!bx) return;
  bosses[side].push({ ...S(`boss ${jname}/${support}`, bx, ring(sh[k], dims.D / 2 + pb.wall, dims.D / 2)), bom: 'gearbox plate bosses', item: `boss ${jname}/${support}` });
}

/** Gearbox plates (the airframe's outline, holes at every seat and rail) with their bosses; the journals in them. */
function plateParts(ax) {
  const { PL, J, n, af, railR, add, plateIds, assumptions } = ax;
  const holes = { aft: [], fwd: [] }, bosses = { aft: [], fwd: [] };
  const plateJoints = [['crank shaft', 0, J.crankShaft], ...Array.from({ length: n - 1 }, (_, i) => [`gear shaft ${i + 1}`, i + 1, J.shafts[i] ?? J.shafts.at(-1)])];
  for (const [jname, k, id] of plateJoints) for (const side of ['aft', 'fwd']) plateJournal(ax, jname, k, id, side, holes, bosses);
  if (J.motorOutboard) plateJournal(ax, 'motor shaft outboard', n, J.motorOutboard, 'fwd', holes, bosses);
  assumptions.push(PL.flange === 'face' ? 'plate journals: a flanged bushing\'s flange on the plate\'s outer face, its body through the plate inward; a bearing flush with the outer face, inward' : 'plate journals (v21 convention, the seat holding the whole part): each from the plate\'s outer face inward, a bushing flange counterbored into the plate at that face; a bearing flush with the outer face, inward');
  const rails = af?.rails ? Object.values(af.rails).map((c) => ({ c, r: railR })) : [];
  // Round 14: the boom passes through both plates (its clamps are on their outer faces), so each plate and post has a
  // clearance hole for it, the running clearance round the tube.
  const boomR = boomRadius(ax), run = (ax.Ly.running_mm ?? 0.5) / 1000;
  const boomHole = boomR && af?.boom_y_m != null ? [{ c: [0, af.boom_y_m], r: boomR + run }] : [];
  for (const side of ['aft', 'fwd']) add(plateIds[side], plateSolids(ax, side, [...holes[side], ...rails], boomHole, bosses[side]), { bom: 'gearbox plates' });
  if (boomHole.length) assumptions.push('gearbox plates and their posts: a clearance hole for the tail boom, the running clearance round it; the posts and pads to the boom clamps and trays as the airframe states them (airframe.js platePosts)');
}

/** One gearbox plate's solids: its outline with its holes, the posts and pads the airframe states, its bosses. */
export function plateSolids({ P, tA, tF, af, plateIds, assumptions }, side, holes, boomHole, bosses) {
  const xp = side === 'aft' ? P.plateAft : P.plateFwd, tp = side === 'aft' ? tA : tF, xs = [xp - tp / 2, xp + tp / 2];
  const pl = af?.plates?.[side], outline = pl?.outline_m;
  if (!outline) assumptions.push(`${plateIds[side]}: no outline in the design (no airframe), checked by its bosses and journals only`);
  const posts = (pl?.posts ?? []).filter((q) => q.post_m).flatMap((q, i) => [S(`post ${i + 1} (${q.to})`, xs, boxYZ(...q.post_m).withHoles(boomHole)), S(`pad ${i + 1} (${q.to})`, xs, boxYZ(...q.pad_m).withHoles(boomHole))]);
  // round 16: a lock land's ear (airframe.js lockEars), its rail's hole in it, measured by its area outside the hull
  const ears = (pl?.ears ?? []).map((e, k) => ({ ...S(`ear ${k + 1} (rail lock ${e.rail})`, xs, poly(e.outline_m, holes)), area_m2: e.area_m2 }));
  return [...(outline ? [S('plate', xs, poly(outline, [...holes, ...boomHole]))] : []), ...posts, ...ears, ...bosses];
}

/** The tail boom tube's outer radius (m), from its catalogue entry. */
function boomRadius({ d, catalog }) {
  const id = d.params?.parts?.keel;
  return id ? mm(byId(catalog, id).specs.od_mm) / 2 : 0;
}

/**
 * Round 17b: a pin's end beyond an eye's free face at x = face, running `out` (+1 forward, -1 aft) for the washer's
 * groove (m, against the face, so the washer bears on the eye and its bushing's end) and its land (n) to the tip.
 * -> { groove: [x0, x1], tip: [x0, x1], washer: [x0, x1], end } (m, sorted; end the pin's tip).
 */
export function pinEndGroove(face, out, pw) {
  const iv = (a, b) => [Math.min(a, b), Math.max(a, b)];
  return { groove: iv(face, face + out * pw.m), tip: iv(face + out * pw.m, face + out * pw.past), washer: iv(face, face + out * pw.s), end: face + out * pw.past };
}

/** Round 17b: a horn pin's free (eye) face and the direction its tip runs from it, by the hornStack convention. */
const hornFreeFace = (H, xc, B) => (H.side === 'fwd' ? { face: xc - B / 2, out: -1 } : { face: xc + B / 2, out: 1 });

function sideParts(ax, tag, sg, delta) {
  const { d, catalog, L, P, x0, fb, eye, shJ, span, add, join } = ax;
  const O4 = sg > 0 ? fb.O4 : mirror(fb.O4);
  const xc = sg > 0 ? P.couplerR : P.couplerL;
  const pose = (q) => (sg > 0 ? q.right : q.left);
  const sparDir = (q) => dir(sg > 0 ? q.right.rocker - delta : q.left.rocker + delta);
  const R = L.rocker, hub = rockerHub(ax.d, R, span, shJ.B, P, eye.B);
  // round 17b: with a retaining washer the pin runs on past the eye by the washer's groove and land, and the boss and
  // arm keep within the hub (linkage.js hornSide, shared with the members)
  const hs = hornSide(ax.Ly, catalog, L, hub, xc, eye.B, tag), { H, arm, pw } = hs, [bossA, bossF] = H.boss;
  const so = mm(R.spar_od_mm ?? 10) / 2 + mm(R.socket_wall_mm), r0 = mm(R.hub_od_mm) / 2;
  const sideName = sg > 0 ? 'right' : 'left', pinBom = (s) => ({ ...s, bom: `shoulder pin ${sideName}` });
  // round 17b: with a retaining washer, the pin runs on past the eye's free face: the groove cut to the washer's own
  // published groove diameter against that face, then the land at full diameter to the tip
  const pinHornBom = (s) => ({ ...s, bom: `horn pin ${sideName}` });
  const ff = hornFreeFace(H, xc, eye.B), pg = pw ? pinEndGroove(ff.face, ff.out, pw) : null;
  const pinSolids = pg
    ? [
        pinHornBom(M('horn pin', H.side === 'fwd' ? [ff.face, H.pin[1]] : [H.pin[0], ff.face], (q) => ring(pose(q).B, eye.d / 2))),
        pinHornBom(M('horn pin groove', pg.groove, (q) => ring(pose(q).B, pw.ri))),
        pinHornBom(M('horn pin tip', pg.tip, (q) => ring(pose(q).B, eye.d / 2))),
      ]
    : [pinHornBom(M('horn pin', H.pin, (q) => ring(pose(q).B, eye.d / 2)))];
  const rk = add(`rocker ${tag}`, [
    ...(hub.relief ? [S('hub', [hub.x[0], hub.relief[0]], ring(O4, r0, shJ.d / 2)), S('hub relief', hub.relief, ring(O4, mm(R.hub_relief_od_mm) / 2, shJ.d / 2)), S('hub forward', [hub.relief[1], hub.x[1]], ring(O4, r0, shJ.d / 2))] : [S('hub', hub.x, ring(O4, r0, shJ.d / 2))]),
    ...(hub.collar ? [pinBom(S('pin collar', hub.collar, ring(O4, mm(R.hub_collar_od_mm) / 2, shJ.d / 2)))] : []),
    pinBom(S('shoulder pin', [x0 - (span + shJ.B) / 2, x0 + (span + shJ.B) / 2], ring(O4, shJ.d / 2))),
    M('arm', arm.x, (q) => cap(O4, pose(q).B, mm(R.arm_w_mm) / 2)),
    M('horn boss', [bossA, bossF], (q) => ring(pose(q).B, mm(hs.boss_od_mm) / 2, eye.d / 2)),
    ...pinSolids,
    { ...M('spar socket', [x0 - so, x0 + so], (q) => { const u = sparDir(q); return cap(add2(O4, u, r0), add2(O4, u, r0 + mm(R.socket_len_mm)), so); }), tube: { ro: so, ri: mm(R.spar_od_mm ?? 10) / 2, L: mm(R.socket_len_mm), axis: 'spar' } },
  ], { bom: `rocker ${sideName}` });
  for (const [nm, xs, fl] of [['aft', x0 - span / 2, 1], ['forward', x0 + span / 2, -1]]) shoulderParts(ax, tag, sg, O4, nm, xs, fl, rk);
  // round 16: the wing on its spar socket (params.wingStructure), in the socket's frame
  wingParts(ax, tag, (q) => { const u = sparDir(q); return { o: O4, u, v: sg > 0 ? [-u[1], u[0]] : [u[1], -u[0]] }; });
  couplerParts(ax, tag, xc, pose, rk);
  if (pg) pinWasherPart(ax, `horn pin washer ${sideName}`, pw, pg, (q) => pose(q).B, rk, `coupler ${tag}`, `coupler ${tag} horn bushing`);
}

/**
 * Round 17 (audit item 7, F5), round 17b for the crank pin too: a DIN 6799 washer in its pin's groove against an eye's
 * free face (params.linkage.retention.washer8mm, one BOM line with the gear shafts' end washers), moving with the pin
 * (at(q) its centre). In its groove on the pin's part (GROOVE); against the eye's face and its bushing's end.
 */
function pinWasherPart(ax, id, pw, pg, at, pinPart, eyePart, bushPart) {
  const wk = ax.add(id, [M('washer', pg.washer, (q) => ring(at(q), pw.ro, pw.ri))], { bom: pw.id });
  ax.join(wk, pinPart, GROOVE);
  ax.join(wk, eyePart, 'against the eye\'s free face');
  ax.join(wk, bushPart, 'against its bushing\'s end');
  return wk;
}

// Round 17b: hornStack (and the arm's span) live in linkage.js, shared with the members that rate them
export { hornStack };

/** A shoulder bushing (flange inboard against the hub, counterbored in the housing: round 11) and its bracket. */
function shoulderParts(ax, tag, sg, O4, nm, xs, fl, rk) {
  const { L, A, af, shJ, railR, add, join } = ax;
  const side = sg > 0 ? 'right' : 'left';
  const bx = [xs - shJ.B / 2, xs + shJ.B / 2];
  const fx = fl > 0 ? [bx[1] - shJ.flangeB, bx[1]] : [bx[0], bx[0] + shJ.flangeB];
  const bid = add(`shoulder bushing ${tag} ${nm}`, [S('body', bx, ring(O4, shJ.D / 2, shJ.d / 2)), ...(shJ.flangeB ? [S('flange', fx, ring(O4, shJ.flangeD / 2, shJ.d / 2))] : [])], { bom: ax.d.params.joints.shoulder });
  const hid = `shoulder bracket ${tag} ${nm}`, ho = mm(L.shoulderHousing.od_mm) / 2;
  // the housing counterbored for the bushing's flange (round 11: the flange inboard, in the housing): the bore D over the
  // body, the flange diameter over the flange (round 14: joined parts may touch but not interpenetrate)
  const hs = shJ.flangeB ? [S('housing', fl > 0 ? [bx[0], fx[0]] : [fx[1], bx[1]], ring(O4, ho, shJ.D / 2)), S('housing counterbore', fx, ring(O4, ho, shJ.flangeD / 2))] : [S('housing', bx, ring(O4, ho, shJ.D / 2))];
  const rail = af?.rails?.[side];
  if (rail) {
    const at = mm(A.rail.arm_t_mm), sl = mm(A.rail.sleeve_len_mm), sw = mm(A.rail.sleeve_wall_mm);
    // the arm flush with the housing's face toward the hub, its extra thickness away from it; the sleeve centred on the housing (the rail's length assumes it)
    // the arm a bar of its width from the bushing's bore to the rail's surface (round 14: it may not reach into either)
    hs.push(S('arm', fl > 0 ? [bx[1] - at, bx[1]] : [bx[0], bx[0] + at], bar(O4, rail, (shJ.flangeB ? shJ.flangeD : shJ.D) / 2, railR, mm(A.rail.arm_b_mm) / 2)));
    hs.push(S('sleeve', sleeveX(ax, xs, sl), ring(rail, railR + sw, railR)));
    join(hid, `shoulder rail ${side}`, 'bonded on the rail');
  }
  // the housing rings are their own BOM line (shoulder housings), the arm and sleeve the bracket's; each bracket an item
  add(hid, hs.map((q) => ({ ...q, bom: /^housing/.test(q.name) ? `shoulder housings ${side}` : `shoulder bracket ${side}` })), { item: hid });
  join(bid, hid, 'pressed in the housing');
  join(bid, `${rk}/shoulder pin`, 'journal on the shoulder pin');
}

/**
 * A bracket's sleeve along the rail: centred on its housing (v21), unless that puts it in a gearbox plate, where the rail
 * is seated: then (round 14, a housing forward of the aft plate) it starts the running clearance clear of that plate.
 */
export function sleeveX({ P, tA, tF, Ly, A }, xs, sl) {
  const iv = [xs - sl / 2, xs + sl / 2], run = (Ly?.running_mm ?? 0.5) / 1000;
  // round 16: a rail's lock flange on the aft plate's front face (rail.lock_at 'aft'), or the lock screws' heads there
  // (lock_at 'aft outer': an ISO 4762 M3 head, 3 mm), thickens that plate for the sleeve
  const lk = !A?.rail?.lock_arc_deg ? 0 : A.rail.lock_at === 'aft' ? mm(A.rail.lock_t_mm ?? 2) : A.rail.lock_at === 'aft outer' ? mm(A.rail.lock_head_mm ?? 3) : 0;
  for (const [xp, tp] of [[P.plateAft + lk / 2, tA + lk], [P.plateFwd, tF]]) {
    if (iv[0] < xp + tp / 2 + run && iv[1] > xp - tp / 2 - run) return xs > xp ? [xp + tp / 2 + run, xp + tp / 2 + run + sl] : [xp - tp / 2 - run - sl, xp - tp / 2 - run];
  }
  return iv;
}

/** A coupler: one piece, an eye round a bushing at each end; the crank-end eye on the crank pin, the other on the horn pin. */
const ax0L = (L) => !!L.crank?.web?.separate;
function couplerParts({ L, J, eye, add, join }, tag, xc, pose, rk) {
  const cp = L.coupler, ck = `coupler ${tag}`, eo = mm(cp.eye_od_mm) / 2, rb = mm(cp.body_d_mm) / 2;
  const xe = [xc - eye.B / 2, xc + eye.B / 2];
  // round 16: each eye bushing's flange (the catalogue's d3 x b2) counterbored into its eye, at the eye's face away from
  // the other side's coupler (+x for the right coupler, -x for the left): the eye's bore D over the body, d3 over the flange
  const fb = eye.flangeB > 0 ? eye.flangeB : 0, sg = tag === 'R' ? 1 : -1;
  const xf = fb ? (sg > 0 ? [xe[1] - fb, xe[1]] : [xe[0], xe[0] + fb]) : null, xb = fb ? (sg > 0 ? [xe[0], xe[1] - fb] : [xe[0] + fb, xe[1]]) : xe;
  const eyeSolids = (nm, at) => [M(nm, xb, (q) => ring(at(q), eo, eye.D / 2)), ...(fb ? [M(`${nm} counterbore`, xf, (q) => ring(at(q), eo, eye.flangeD / 2))] : [])];
  add(ck, [
    ...eyeSolids('crank eye', (q) => q.A),
    ...eyeSolids('horn eye', (q) => pose(q).B),
    // the round body between the eyes (it starts 1 mm inside each eye, as the film draws it)
    M('body', [xc - rb, xc + rb], (q) => between(q.A, pose(q).B, eo - 0.001, eo - 0.001, rb)),
  ], { bom: 'coupler links', item: ck });
  for (const [end, at] of [['crank', (q) => q.A], ['horn', (q) => pose(q).B]]) {
    const bid = add(`coupler ${tag} ${end} bushing`, [M('body', xb, (q) => ring(at(q), eye.D / 2, eye.d / 2)), ...(fb ? [M('flange', xf, (q) => ring(at(q), eye.flangeD / 2, eye.d / 2))] : [])], { bom: J.couplerEyes });
    join(bid, ck, 'pressed in the eye');
    join(bid, end === 'crank' ? (ax0L(L) ? 'crank web/pin' : 'crank/pin') : `${rk}/horn pin`, 'journal on its pin');
  }
}

/**
 * The rail's torsion lock flange in the plane: a ring round the rail (v21), or round 14 (rail.lock_arc_deg [a0, a1],
 * angles from +z counter-clockwise for the right rail, mirrored for the left) the hull of the rail and its screws'
 * lands on that arc at lock_r_mm (an M3 head and its edge, 3.5 mm in radius), with the rail's bore.
 */
export function lockFlangeShape(rail, c, sg, railR) {
  if (!rail.lock_arc_deg) return ring(c, mm(lockFlangeOD(rail)) / 2, railR);
  return poly(hullOf(lockFlangeCircles(rail, c, sg)).poly, [{ c, r: railR }]);
}

function frameParts({ A, af, P, plateT, tA, tF, x0, span, railR, d, catalog, plateIds, assumptions, add, join, parts }) {
  const xA = P.plateAft, xF = P.plateFwd, gap = xF - xA, xa = x0 - span / 2;
  for (const [side, c] of Object.entries(af.rails ?? {})) {
    const railLen = gap + plateT + (xA - xa) + mm(A.rail.sleeve_len_mm) / 2 + 0.002;
    // its torsion lock flange, 2 mm, on the face of the plate it is screwed to (airframe.js: rail.lock_at)
    const lt = mm(A.rail.lock_t_mm ?? 2);
    // round 16: the aft seat boss on the aft plate's aft face (rail.aft_boss_at 'aft face'), and lock_at 'aft outer', the
    // flange aft of the plate (and of that boss)
    const bossT = A.rail.aft_boss_at === 'aft face' ? mm(A.rail.aft_boss_mm ?? 0) : 0;
    const fx = A.rail.lock_at === 'forward' ? [xF - tF / 2 - lt, xF - tF / 2] : A.rail.lock_at === 'aft outer' ? [xA - tA / 2 - bossT - lt, xA - tA / 2 - bossT] : [xA + tA / 2, xA + tA / 2 + lt];
    if (bossT && !seatRingBoss(A.rail)) {
      // round 19 (members3, rail.aft_boss_shape 'lock flange'): a pad under the lock flange (airframe.js lockPadCircles),
      // machined with the aft plate, so it is a solid of the plate (no joint), weighed on its own BOM line (airframe.js)
      const pp = parts.find((q) => q.id === plateIds.aft), bom = `shoulder rail ${side} aft seat boss`;
      pp.solids.push({ ...S(`lock pad (rail ${side})`, [xA - tA / 2 - bossT, xA - tA / 2], poly(hullOf(lockPadCircles(A.rail, c, side === 'right' ? 1 : -1)).poly, [{ c, r: railR }])), bom });
      pp.bom = [...new Set([...pp.bom, bom])];
    } else if (bossT) {
      const bid = add(`shoulder rail ${side} aft seat boss`, [S('ring', [xA - tA / 2 - bossT, xA - tA / 2], ring(c, mm(A.rail.seat_D_mm) / 2, railR))], { bom: `shoulder rail ${side} aft seat boss` });
      join(bid, plateIds.aft, 'on the plate\'s aft face (machined with it)');
      join(bid, `shoulder rail ${side}`, 'the rail through its bore, its lock flange on its face');
    }
    const id = add(`shoulder rail ${side}`, [S('tube', [xF + plateT / 2 - railLen, xF + plateT / 2], ring(c, railR, mm(A.rail.id_mm) / 2)), S('lock flange', fx, lockFlangeShape(A.rail, c, side === 'right' ? 1 : -1, railR))], { bom: `shoulder rail ${side}` });
    join(id, plateIds.aft, 'seated in the plate'); join(id, plateIds.fwd, 'seated in the plate');
  }
  for (const q of A.panels.list) {
    const t = mm(q.t_mm) / 2;
    const sh = q.kind === 'side' ? cap([mm(q.z_mm), mm(q.y0_mm)], [mm(q.z_mm), mm(q.y0_mm + q.h_mm)], t) : cap([-mm(q.w_mm) / 2, mm(q.y_mm)], [mm(q.w_mm) / 2, mm(q.y_mm)], t);
    // round 16: its end flanges on the plates' inner faces (airframe.js panelFlanges), each rectangle a solid
    const fl = panelFlanges(A, P, tA, tF).filter((f) => f.part === `gearbox ${q.name}`).flatMap((f) => f.rects.map((r, k) => S(`flange ${f.end} ${k + 1}`, f.x, boxYZ(r.z[0], r.z[1], r.y[0], r.y[1]))));
    const id = add(`gearbox ${q.name}`, [S('sheet', [xA + tA / 2, xF - tF / 2], sh), ...fl], { bom: 'gearbox panels', item: `gearbox ${q.name}` }); // between the plates' inner faces
    join(id, plateIds.aft, 'flange bonded to the plate'); join(id, plateIds.fwd, 'flange bonded to the plate');
  }
  const bo = A.boom, tube = byId(catalog, d.params.parts.keel).specs, yB = af.boom_y_m;
  const cl = mm(bo.clamp_len_mm), blk = mm(tube.od_mm + 2 * bo.clamp_wall_mm);
  // round 15 (params.layout.plate_faces 'actual'): the clamps on the plates' own faces
  const [hA, hF] = d.params.layout?.plate_faces === 'actual' ? [tA / 2, tF / 2] : [plateT / 2, plateT / 2];
  const cA = [xA - hA - cl, xA - hA], cF = [xF + hF, xF + hF + cl];
  // round 16 (boom.to_tail_spar_mm): to that far aft of the tail's spar, so the spar's root and the ties lie on it
  const xb0 = bo.to_tail_spar_mm != null ? d.tail.x_m - bo.to_tail_spar_mm / 1000 : d.tail.x_m;
  const boom = add('tail boom', [S('tube', [xb0, cF[1]], ring([0, yB], mm(tube.od_mm) / 2, mm(tube.id_mm) / 2))], { bom: 'keel and tail boom' });
  for (const [nm, xs, pl] of [['aft', cA, plateIds.aft], ['forward', cF, plateIds.fwd]]) {
    // round 16 (boom.screw_edge_mm): a square collar bored for the boom and a screw flange below it round its screws
    const cf = clampFlange(bo, blk, yB);
    const solids = cf ? [S('block', xs, boxYZ(-blk / 2, blk / 2, yB - blk / 2, yB + blk / 2).withHoles([{ c: [0, yB], r: mm(tube.od_mm) / 2 }])), S('flange', xs, boxYZ(cf.z[0], cf.z[1], cf.y[0], cf.y[1]))]
      : [S('block', xs, boxYZ(-blk / 2, blk / 2, yB - blk / 2 - 0.006, yB + blk / 2).withHoles([{ c: [0, yB], r: mm(tube.od_mm) / 2 }]))];
    const id = add(`boom clamp ${nm}`, solids, { bom: 'tail boom clamp blocks', item: `boom clamp ${nm}` }); // bored for the boom
    join(id, boom, 'bonded on the boom'); join(id, pl, 'screwed to the plate');
  }
  for (const [nm, T, xs, pl] of [['forward tray', A.tray, (Lt) => [xF + plateT / 2, xF + plateT / 2 + Lt], plateIds.fwd], ['aft tray', A.aftTray, (Lt) => [xA - plateT / 2 - Lt, xA - plateT / 2], plateIds.aft]]) {
    const r = af.trays?.[nm === 'forward tray' ? 'forward' : 'aft'];
    if (!r || af.carriers) continue; // round 15: the carriers are their own parts (carrierParts)
    const y = r.y_m, w = mm(T.w_mm) / 2, t = mm(T.t_mm) / 2, h = mm(T.flange_h_mm);
    const id = add(nm, [S('base', xs(r.length_m), cap([-w, y], [w, y], t)), S('flange right', xs(r.length_m), cap([w, y], [w, y + h], t)), S('flange left', xs(r.length_m), cap([-w, y], [-w, y + h], t))], { bom: nm });
    join(id, pl, 'screwed to the plate');
  }
  assumptions.push('frame (structure.airframe): rails from the aft housing\'s sleeve to the forward plate\'s outer face; panels between the plate planes; boom clamps on the plates\' outer faces with the screw flange below the boom; trays as channels, flanges up, from the plate face for their stated length');
}

/** A channel's solids (structure.airframe carriers, airframe.js carrierGeometry): base and two flanges over x. */
export function channelSolids(c, x) {
  return [S('base', x, cap([-c.w / 2, c.y], [c.w / 2, c.y], c.t / 2)), S('flange right', x, cap([c.w / 2, c.y], [c.w / 2, c.y + c.h], c.t / 2)), S('flange left', x, cap([-c.w / 2, c.y], [-c.w / 2, c.y + c.h], c.t / 2))];
}

/** A channel's root tab: a sheet up the plate face from the base's top, its width the flanges' spacing. */
const rootTab = (c, x) => S('root tab', x, boxYZ(-c.w / 2, c.w / 2, c.y + c.t / 2, c.y + c.t / 2 + c.tab));

/**
 * The riser's solids (round 15): its web across the pod, a flange forward along each vertical edge, its foot aft on
 * the forward tray's base and its tab forward under the nose tray. holes: small round pass-throughs (readhead
 * screws), left as holes in the web. slot (round 17, audit item 23): the actuator opening, which came within 1.09 mm
 * of the tab's bend; instead of a plain round hole, the web is split either side of a slot of the opening's own
 * width that runs open to the top edge, so there is no thin captive strip there to distort when the tab is formed.
 * Each piece stays a plain box (never a concave polygon), which keeps the SAT clash check, poly-poly, exact.
 */
export function riserSolids(r, holes = [], slot = null) {
  const [x0, x1] = r.x, [y0, y1] = r.y, hw = r.w / 2;
  const web = slot ? [
    S('web below the opening', [x0, x1], boxYZ(-hw, hw, y0, slot.c[1] - slot.r).withHoles(holes)),
    S('web left of the opening', [x0, x1], boxYZ(-hw, slot.c[0] - slot.r, slot.c[1] - slot.r, y1).withHoles(holes)),
    S('web right of the opening', [x0, x1], boxYZ(slot.c[0] + slot.r, hw, slot.c[1] - slot.r, y1).withHoles(holes)),
  ] : [S('web', [x0, x1], boxYZ(-hw, hw, y0, y1).withHoles(holes))];
  return [
    ...web,
    S('flange right', [x1, x1 + r.f], boxYZ(hw - r.t, hw, y0, y1 - r.t)),
    S('flange left', [x1, x1 + r.f], boxYZ(-hw, -hw + r.t, y0, y1 - r.t)),
    S('foot', [x0 - r.foot, x0], boxYZ(-hw, hw, y0, y0 + r.t)),
    S('tab', [x1, x1 + r.foot], boxYZ(-hw, hw, y1 - r.t, y1)),
  ];
}

/** Round 15 carriers (structure.airframe.parts.carriers): forward tray, riser, nose tray, aft tray, each screwed to its parent. */
function carrierParts({ af, add, join, plateIds, assumptions }) {
  const G = af.carriers;
  const f = add('forward tray', [...channelSolids(G.forward, G.forward.x), rootTab(G.forward, [G.forward.x[0], G.forward.x[0] + G.forward.t])], { bom: 'forward tray' });
  join(f, plateIds.fwd, 'its root tab screwed to the plate');
  // round 16: the readhead screws' clearance holes in the web (ISO 273 medium, an M2's 2.4 mm)
  const rhx = af.support?.readhead, holes = rhx ? rhx.holes_m.map((c) => ({ c, r: 0.0012 })) : [];
  // round 17 (audit item 23): the actuator opening is a slot open to the tab's bend, not a plain hole (riserSolids)
  const r = add('nose tray riser', riserSolids(G.riser, holes, G.riser.hole ?? null), { bom: 'nose tray riser' });
  join(r, f, 'its foot screwed to the tray\'s base');
  const n = add('nose tray', channelSolids(G.nose, G.nose.x), { bom: 'nose tray' });
  join(n, r, 'screwed to the riser\'s tab');
  const a = add('aft tray', [...channelSolids(G.aft, G.aft.x), rootTab(G.aft, [G.aft.x[1] - G.aft.t, G.aft.x[1]])], { bom: 'aft tray' });
  if (G.doubler) {
    // round 16: the forward tray's doubler between its root tab and the plate's outer face, under the same screws
    const D = G.doubler, db = add('forward tray flange doubler', [S('plate', D.x, boxYZ(-D.w / 2, D.w / 2, D.y[0], D.y[1]))], { bom: 'forward tray flange doubler' });
    join(db, plateIds.fwd, 'on the plate\'s outer face, under the tray\'s screws');
    join(db, f, 'under the tray\'s root tab');
  }
  join(a, plateIds.aft, 'its root tab screwed to the plate');
  assumptions.push('carriers (round 15, structure.airframe.parts.carriers): channels with their flanges up and root tabs up the plate faces; the nose tray riser\'s web, flanges, foot and tab as sheets; screws and their heads are not drawn (no part sits on a screw: the parts start clear of the riser tab\'s screws)');
}

/**
 * A cable tie as four slabs round a bundle box [z0, z1] x [yb, yt] at x (round 15): over the top, down each side from
 * that side's floor (the carrier's face where the tie passes through its slot), under the bottom; band width b along x,
 * thickness tt. The slots themselves are not drawn.
 */
export function bandSlabs(tag, x, b, tt, [z0, z1], [yb, yt], floors = [yb, yb]) {
  const xs = [x - b / 2, x + b / 2];
  return [
    S(`${tag} top`, xs, boxYZ(z0 - tt, z1 + tt, yt, yt + tt)),
    S(`${tag} side -z`, xs, boxYZ(z0 - tt, z0, floors[0], yt)),
    S(`${tag} side +z`, xs, boxYZ(z1, z1 + tt, floors[1], yt)),
    S(`${tag} bottom`, xs, boxYZ(z0 - tt, z1 + tt, yb - tt, yb)),
  ];
}

/** The x of each of n ties spread evenly over [x0, x1] (each at the middle of its share). */
export const tieXs = (x0, x1, n) => Array.from({ length: n }, (_, k) => x0 + ((k + 0.5) / n) * (x1 - x0));

/** The carrier a mount names, as structure.airframe.parts.carriers draws it. */
const carrierOf = (af, name) => ({ 'forward tray': af?.carriers?.forward, 'nose tray': af?.carriers?.nose, 'aft tray': af?.carriers?.aft, 'belly tray': af?.carriers?.belly })[name];

/** A mounted part's ties (round 15): through slots in its carrier either side of it, round its box and the carrier's sheet. */
function partTies(add, join, id, box, c, tr) {
  const [b, tt] = tr.band_mm.map((v) => v / 1000), face = c.y + c.t / 2;
  // round 17b: a strap stated at its own x (tr.x_mm, clear of the ties beside it), else spread evenly
  const sol = (tr.x_mm ? tr.x_mm.map((v) => v / 1000) : tieXs(box.x0, box.x1, tr.n)).flatMap((x, k) => tagged(bandSlabs(`tie ${k + 1}`, x, b, tt, [box.z0, box.z1], [face - c.t, box.y1], [face, face]), tr.tie, `${id} tie ${k + 1}`));
  const tid = add(`${id} ties`, sol);
  join(tid, id, 'round the part');
  join(tid, tr.carrier, 'through its slots');
  return tid;
}

/**
 * Packed boxes (model.js layoutParts, the body fit) as parts on their carriers, and their clearance to the pod shell.
 * Round 15: a mounted part rests on the carrier its mount names, tied through it (its ties drawn); a part without a
 * mount keeps round 8's notional ties to a tray. -> { pod, model }.
 */
function packedParts(add, join, d, catalog, af) {
  const pIdx = partsIndex(d, catalog);
  const model = prepare(d, pIdx);
  packRegulator(model, d, pIdx); // round 14: the servo regulator where the film packs it (last, no other part moves)
  const onFwd = /battery|esc|receiver|airspeed/, onAft = /servo|fc|gps/;
  const pod = [];
  for (const p of model.layout.parts) {
    if (p.name === 'motor') { pod.push(p.shell_gap_m != null ? { part: 'motor (its can)', gap_m: p.shell_gap_m } : { part: 'motor (its box)', gap_m: -boxProtrusion(d.body.loft, p.box).out }); continue; }
    const b = p.box;
    const id = add(`packed ${p.name}`, [S('box', [b.x0, b.x1], boxYZ(b.z0, b.z1, b.y0, b.y1))], { bom: p.id });
    const c = p.mount && carrierOf(af, p.mount.carrier);
    if (c) {
      join(id, p.mount.carrier, `resting on the ${p.mount.carrier}`);
      const tr = af.support?.ties?.find((t) => t.part === p.name);
      if (tr) partTies(add, join, id, b, c, tr);
    } else {
      if (onFwd.test(p.name)) join(id, 'forward tray', 'tied to the tray');
      if (onAft.test(p.name)) join(id, 'aft tray', 'tied to the tray');
    }
    pod.push({ part: id, gap_m: -boxProtrusion(d.body.loft, b).out });
  }
  // round 14: the tip circles' signed gap to the loft (model.js gearFit gap_m), checked like the packed parts
  for (const g of model.layout.gearFit) pod.push(g.gap_m != null ? { part: `${g.name} (${g.id}) tip circle`, gap_m: g.gap_m } : { part: `${g.name} (${g.id}) tip circle`, gap_m: -g.protrusion, protrusion_only: true });
  return { pod, model };
}

/**
 * Round 15: the pitot-static probe (structure.airframe.parts.support.pitot) along the nose tray's flange, and its ties
 * round the probe and the flange (through a slot at the flange's root).
 */
function pitotParts({ af, add, join, assumptions }) {
  const pm = af.support.pitot, N = af.carriers.nose, s = Math.sign(pm.c_m[0]) || 1;
  const id = add('pitot-static probe', [S('tube', pm.x_m, ring(pm.c_m, pm.r_m))], { bom: 'pitot-static probe' });
  join(id, 'nose tray', `against the ${s > 0 ? 'right' : 'left'} flange's outer face, tied`);
  const zin = s * (N.w / 2 - N.t / 2), zout = pm.c_m[0] + s * pm.r_m, yb = N.y - N.t / 2, yt = Math.max(N.y + N.h + N.t / 2, pm.c_m[1] + pm.r_m);
  const [b, tt] = pm.band_mm.map((v) => v / 1000), top = N.y + N.t / 2;
  const zs = s > 0 ? [zin, zout] : [zout, zin], floors = s > 0 ? [top, yb] : [yb, top];
  const tid = add('pitot-static probe ties', pm.tie_x_mm.flatMap((x, k) => tagged(bandSlabs(`tie ${k + 1}`, x / 1000, b, tt, zs, [yb, yt], floors), pm.tie, `pitot tie ${k + 1}`)));
  join(tid, id, 'round the probe');
  join(tid, 'nose tray', 'round the flange, through a slot at its root');
  assumptions.push('pitot-static probe (round 15): the titanium tube as a solid rod along the nose tray\'s flange, its bottom flush with the tray\'s base; its ties round the probe and the flange');
}

/**
 * Round 16: every screw and nut of the airframe's bolt layout (structure.airframe.parts.fixings, src/design/fixings.js)
 * as its own part: the head on its face, the shank through the parts it clamps (THROUGH, the holes not drawn) or into
 * its tapped hole (THREAD), the nut on the last part's face. A screw along y is drawn as the boxes round its circles.
 */
function fixingParts({ af, catalog, add, join, assumptions }) {
  const n = new Map();
  for (const b of af.fixings) {
    const k = (n.get(b.joint) ?? 0) + 1;
    n.set(b.joint, k);
    const g = fastenerGeometry(byId(catalog, b.screw).specs ?? {}, b.screw), ng = b.nut ? fastenerGeometry(byId(catalog, b.nut).specs ?? {}, b.nut) : null;
    const sol = b.axis === 'x' ? screwX(b.c, b.face, b.dir, g, b.nut_face, ng) : screwY(b.c, b.face, b.dir, g, b.nut_face, ng);
    const sid = add(`${b.joint} screw ${k}`, sol.screw, { bom: b.screw });
    // round 18.2: an x-axis shank is one solid named 'shank'; a y-axis shank is now N true-section slabs named
    // 'shank 1'..'shank N' (xSlabs, screwY); either way every slab needs its own THROUGH/THREAD join, since those
    // override the part-level descriptive join (not itself exempt) for exactly the pairs that overlap
    const shankSlabs = sol.screw.filter((s) => /^shank/.test(s.name));
    for (const p of b.through) { join(sid, p, 'fastened (its head or its nut on the face)'); for (const sl of shankSlabs) join(`${sid}/${sl.name}`, p, THROUGH); }
    if (b.tapped) { for (const sl of shankSlabs) join(`${sid}/${sl.name}`, b.tapped, THREAD); join(sid, b.tapped, 'screwed into its threads'); }
    if (sol.nut.length) {
      const nid = add(`${b.joint} nut ${k}`, sol.nut, { bom: b.nut });
      join(nid, sid, 'threaded on the screw');
      for (const sl of shankSlabs) join(`${sid}/${sl.name}`, nid, THREAD);
      join(nid, b.through.at(-1), 'on its face');
    }
  }
  assumptions.push('fixings (round 16, structure.airframe.parts.fixings): every M3 screw at its stated place with its head on its face and its shank through the parts it clamps (the holes are cut to it and not drawn: exempt as THROUGH), each nut on the last part\'s face, drawn by its corner circle; a screw along y its true section, N slabs across x (round 18.2); the motor bracket\'s screws into the motor\'s back-face threads (not drawn: THREAD)');
}

/**
 * Round 16: the pod's formers (structure.airframe.parts.formers, airframe.js formerGeometry): each band as convex pieces
 * on the covering (touching it: the containment check holds them at zero), its tab against the left flange of its
 * tray, and its tie round the flange and the tab through a slot at the flange's root (the slot and the tab's hole for
 * the tie not drawn: the tie's bottom through the tab is exempt as RIB_HOLE).
 */
function formerParts({ af, catalog, d, add, join, assumptions }) {
  const tie = d.params.linkage?.airframe?.tieSmall ?? 'tie-hellermanntyton-t18r';
  const F = af.formers, formers = formerGeometry(d.body.loft, { formers_at_mm: F.formers_at_mm, former_band_mm: F.band_mm, former_tab_mm: F.tab_mm, former_t_mm: F.t_mm }, af.carriers, loftHalf);
  const [b, tt] = String(byId(catalog, tie).specs.dimensions).match(/x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/).slice(1).map((v) => +v / 1000);
  formers.forEach((f, k) => {
    const sol = [...formerPieces(f).map((p, i) => S(`band ${i + 1}`, f.x_m, poly(p))), S('tab', f.x_m, boxYZ(f.tab.z[0], f.tab.z[1], f.tab.y[0], f.tab.y[1]))];
    const id = add(`former ${k + 1}`, sol, { bom: 'fuselage formers' });
    join(id, f.tray, 'its tab against the tray\'s flange, tied');
    const c = carrierOf(af, f.tray), x = (f.x_m[0] + f.x_m[1]) / 2, top = c.y + c.t / 2;
    // the loop: over the flange's top, down its inside to the base (a slot at its root), under the base, up outside the tab
    const zs = [f.tab.z[0], -(c.w / 2 - c.t / 2)], floors = [c.y - c.t / 2, top];
    const tid = add(`former ${k + 1} tie`, tagged(bandSlabs('tie', x, b, tt, zs, [c.y - c.t / 2, c.y + c.h + c.t / 2], floors), tie));
    join(tid, id, 'round the tab');
    join(`${tid}/tie bottom`, id, RIB_HOLE);
    join(tid, f.tray, 'round the flange, through a slot at its root');
  });
  assumptions.push(`formers (round 16, structure.airframe.parts.formers): ${formers.length} balsa bands on the covering at x ${F.formers_at_mm.join(', ')} mm (each band's outside on the design's loft at the smaller section across it), each tied by its tab to the left flange of the tray above it`);
}

/** Whether a cross-section, holes aside, holds point p with `edge` to spare (m). */
function holds(sh, p, edge) {
  // a ring's material is its annulus; round 17: a solid ring (no bore) holds a point anywhere inside it, its centre too
  if (sh.kind === 'ring') { const r = len(sub(p, sh.c)); return r <= sh.ro - edge + 1e-12 && ((sh.ri ?? 0) > 0 ? r >= sh.ri + edge - 1e-12 : true); }
  if (sh.kind === 'cap') return segPointDist(p, sh.a, sh.b) <= sh.r - edge + 1e-12;
  return polySd(sh.pts, p) <= -edge + 1e-12;
}

/**
 * Round 16: every screw's hole in the material of each part it clamps or threads into. The THROUGH and THREAD
 * exemptions let a shank meet those parts; this check makes sure the hole is in them: the shank's circle (an x screw)
 * or its square across the sheet (a y screw) inside a solid of the part, holes aside, where their ranges overlap.
 * -> { rule, holes, failures: [{ screw, part }], passed }.
 */
export function holeRecord(asm, q0 = null) {
  const byPart = new Map(asm.parts.map((p) => [p.id, p]));
  const rows = [], failures = [], seen = new Set();
  for (const [a, b, w] of asm.joins) {
    if (!(w === THROUGH || w === THREAD)) continue;
    // round 18.2: a shank is one solid (an x-axis screw's exact ring, join "part/shank") or a group of
    // true-section slabs (xSlabs, a y/span-axis screw's: one join per slab, "part/shank 1".."part/shank N",
    // every one checked but the (screw, part) pair counted once
    const m = /^(.*)\/shank(?:\s\d+)?$/.exec(a);
    if (!m) continue;
    const key = `${m[1]}|${b}`;
    if (seen.has(key)) continue;
    seen.add(key);
    const screw = `${m[1]}/shank`, sp = byPart.get(m[1]), part = byPart.get(b.split('/')[0]);
    // round 17b: a screw on a moving part (the twist drive's) at the reference pose
    const group = (sp?.solids ?? []).filter((s) => /^shank(\s\d+)?$/.test(s.name)).map((s0) => (!s0.shape && s0.at && q0 ? { ...s0, shape: s0.at(q0) } : s0));
    if (!group.length || !part) { failures.push({ screw, part: b, why: 'no such part' }); continue; }
    if (part.solids.every((s) => s.name === 'nut')) continue; // its nut, threaded on (its bore is the shank)
    // round 17: a moving part's solids at the reference pose (the crank web's axial screw lies on the crank axis)
    const ok = part.solids.some((s) => { const shp = s.shape ?? (q0 && s.at ? s.at(q0) : null); return shp && group.every((sh) => shankIn(sh, { ...s, shape: shp })); });
    rows.push({ screw, part: b, in_material: ok });
    if (!ok) failures.push({ screw, part: b });
  }
  return { rule: 'round 16: each screw\'s hole lies in the material of every part it clamps or threads into (its shank inside a solid of the part, holes aside); src/design/clearance.js holeRecord', holes: rows.length, failures, passed: failures.length === 0 };
}

/** Whether a shank (or one of its true-section slabs) lies in a fixed solid's material where their ranges overlap. */
function shankIn(sh, s) {
  if (sh.tube?.axis === 'span') {
    // round 17: a screw across x (round 18.2: a long, unsliced round tube, a cap, or one of N true-section slabs,
    // a poly box -- edge-midpoint to edge-midpoint still gives its centre line either way): its axis passes
    // through the solid, the true radius (sh.tube.ro, not the slab's own, narrower width) inside it, at an x the
    // solid spans
    const sp = sh.shape, ends = sp.kind === 'cap' ? [sp.a, sp.b] : [lerp(sp.pts[0], sp.pts[3], 0.5), lerp(sp.pts[1], sp.pts[2], 0.5)];
    const xm = (sh.x[0] + sh.x[1]) / 2;
    if (!(xm >= s.x[0] - 1e-12 && xm <= s.x[1] + 1e-12)) return false;
    return Array.from({ length: 11 }, (_, k) => lerp(ends[0], ends[1], k / 10)).some((p) => holds(s.shape, p, sh.tube.ro));
  }
  if (sh.shape.kind === 'poly') {
    // a vertical screw (c = [x, z]), round 18.2: one of N true-section slabs: its true radius (sh.tube.ro, not
    // the slab's own, narrower width) across x and z inside the solid at a height both share
    const pts = sh.shape.pts, r = sh.tube.ro, z = (Math.min(...pts.map((p) => p[0])) + Math.max(...pts.map((p) => p[0]))) / 2;
    const yr = [Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[1]))];
    const sy = s.shape.kind === 'cap' ? [Math.min(s.shape.a[1], s.shape.b[1]) - s.shape.r, Math.max(s.shape.a[1], s.shape.b[1]) + s.shape.r] : s.shape.kind === 'poly' ? [Math.min(...s.shape.pts.map((p) => p[1])), Math.max(...s.shape.pts.map((p) => p[1]))] : [s.shape.c[1] - s.shape.ro, s.shape.c[1] + s.shape.ro];
    const y = (Math.max(yr[0], sy[0]) + Math.min(yr[1], sy[1])) / 2;
    return sh.x[0] >= s.x[0] - 1e-9 && sh.x[1] <= s.x[1] + 1e-9 && yr[0] <= sy[1] && yr[1] >= sy[0] && holds(s.shape, [z - r, y], 0) && holds(s.shape, [z + r, y], 0);
  }
  // an x-axis screw's exact ring, unsliced: still a single solid, never grouped
  if (!(sh.x[0] < s.x[1] && sh.x[1] > s.x[0])) return false;
  return holds(s.shape, sh.shape.c, sh.shape.ro);
}

/**
 * A screw along y (round 16; round 18.2, the film's request: its true section, not a square prism -- a fastener
 * lying across x has its real round section in the plane that contains x (here, x-z), which this model (a prism
 * along x) cannot draw as one solid. xSlabs slices the head and the shank each into N prisms along x, every one
 * the exact rectangle at that slice's true half-width, holding the exact nominal y length throughout (no capsule,
 * no overshoot past either end: the true length and the true width are independent here, unlike a round-in-y
 * shape would force). n=8: about 11% more cross-section than the true circle (6: ~14%), always a superset of it,
 * so the clash and hole checks stay conservative. As screwX, its nut the box round its corner circle (c: [x, z]).
 */
export function screwY(c, yFace, dir, g, yNut = null, ng = null) {
  const iv = (a, b) => [Math.min(a, b), Math.max(a, b)];
  const box = (r, ys) => [[c[0] - r, c[0] + r], boxYZ(c[1] - r, c[1] + r, ...ys)];
  const sol = (name, r, ys, ri = 0) => { const [x, sh] = box(r, ys); return { ...S(name, x, sh), tube: { ro: r, ri, L: ys[1] - ys[0], axis: 'y' } }; };
  // the head and the shank, each N true-section slabs (xSlabs): one shared tube record per fastener (not per
  // slab), so completeness.js's size checks (which read it, never the drawn polygon) still match the BOM line
  const slabs = (name, r, ys) => xSlabs(c[0], r, (w) => boxYZ(c[1] - w, c[1] + w, ...ys)).map((s, i) => ({ ...S(`${name} ${i + 1}`, s.x, s.shape), tube: { ro: r, ri: 0, L: ys[1] - ys[0], axis: 'y' } }));
  const screw = [...slabs('head', g.head_d / 2, iv(yFace, yFace - dir * g.head_k)), ...slabs('shank', g.d / 2, iv(yFace, yFace + dir * g.L))];
  const nut = yNut != null && ng ? [sol('nut', ng.s / Math.sqrt(3), iv(yNut, yNut + dir * ng.m), g.d / 2)] : [];
  return { screw, nut };
}

/** Points on a cross-section's outline (every point the loft gap can be worst at: a convex section's vertices, n on each circle). */
function outlinePoints(sh, n = 96) {
  const circ = (c, r) => Array.from({ length: n }, (_, k) => [c[0] + r * Math.cos((k / n) * TWO_PI), c[1] + r * Math.sin((k / n) * TWO_PI)]);
  if (sh.kind === 'ring') return circ(sh.c, sh.ro);
  if (sh.kind === 'cap') return [...circ(sh.a, sh.r), ...circ(sh.b, sh.r)];
  return sh.pts;
}

/** Whether point p lies in a cross-section. */
export function inShape(sh, p) {
  if (sh.kind === 'ring') { const r = len(sub(p, sh.c)); return r <= sh.ro && r >= (sh.ri ?? 0); }
  if (sh.kind === 'cap') return segPointDist(p, sh.a, sh.b) <= sh.r;
  return polySd(sh.pts, p) <= 0 && !(sh.holes ?? []).some((h) => len(sub(p, h.c)) < h.r);
}

/**
 * The loft's section at x as model.js reads it, and a point's signed gap to it (m, positive inside): model.js's own
 * measure (1 - r) min(hh, hw), r the point's elliptic radius. It understates the true distance inside, so a gap it
 * passes is at least that large.
 */
const loftAt = loftHalf;
const gapIn = (h, p) => (h ? (1 - Math.hypot((p[1] - h.y) / h.hh, p[0] / h.hw)) * Math.min(h.hh, h.hw) : -Infinity);
const angleIn = (h, p) => (Math.atan2((p[1] - h.y) / h.hh, p[0] / h.hw) * 180) / Math.PI;

/** x samples of a prism against the loft: its ends, the loft's section planes between them, and eight between. */
export function xSamples(loft, [x0, x1]) {
  const xs = [x0, x1, ...Array.from({ length: 7 }, (_, k) => x0 + ((k + 1) / 8) * (x1 - x0)), ...loft.sections.map((q) => q.x_m).filter((x) => x > x0 && x < x1)];
  return [...new Set(xs)];
}

/**
 * Round 15 (SPEC "Support and containment"): every solid against the pod loft. An enclosed solid keeps `required`
 * inside it (model.js's measure); a solid of a declared opening may cross it, and the window of skin it crosses is
 * recorded (x and the elliptic polar angle, 0 deg at +z, 90 at the top); an outside fitting keeps `required` clear of
 * the skin outward. The material outside the loft (outside fittings and openings that count in drag), projected on
 * the (z, y) plane at `cell` resolution, is the frontal area added to the pod's drag at Cd_out.
 */
export function containmentRecord(asm, d, lay, decl, { required = CLEARANCE_M, phases = 72, cell = 0.00025 } = {}) {
  const loft = d.body.loft;
  const amps = [...new Set([d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite))];
  const poses = amps.flatMap((amp) => Array.from({ length: phases }, (_, i) => drivePose({ law: lay.law, fourBar: lay.fb }, (i / phases) * TWO_PI, amp)));
  const rows = [], openings = new Map((decl.openings ?? []).map((o) => [o.name, newOpening(o)]));
  // round 16: an outside entry names a part, or (re) every part whose id matches, as one group; drag false for parts whose
  // own drag the flight model carries (the wings, the tail)
  const outside = new Map((decl.outside ?? []).map((o) => [o.part ?? o.name, { part: o.part ?? o.name, why: o.why, min_clear_m: Infinity, drag: o.drag !== false, ...(o.re ? { re: new RegExp(o.re), parts: 0 } : {}) }]));
  const outsideOf = (id) => outside.get(id) ?? [...outside.values()].find((o) => o.re?.test(id));
  // round 16: parts that carry the covering (the formers) touch the skin from inside: held at zero, not 0.5 mm
  const skin = (decl.on_skin ?? []).map((o) => ({ ...o, rx: new RegExp(o.re) }));
  const exposed = new Set();
  for (const part of asm.parts) for (const sol of part.solids) {
    const shapes = sol.at ? poses.map((q) => sol.at(q)) : [sol.shape];
    const xs = xSamples(loft, sol.x);
    const op = declaredOpening(decl.openings, part.id, sol.name), out = !op && outsideOf(part.id);
    const g = solidLoftGaps(loft, shapes, xs, required, op ? openings.get(op.name) : null);
    const id = `${part.id}/${sol.name}`;
    if (op) openings.get(op.name).solids.push(id);
    else if (out) out.min_clear_m = Math.min(out.min_clear_m, g.clearOut);
    else { rows.push({ solid: id, gap_m: g.worst, ...(skin.some((o) => o.rx.test(part.id)) ? { on_skin: true } : {}) }); continue; }
    if ((out ? out.drag : op.drag !== false) && g.worst < 0) for (const sh of shapes) markExposed(exposed, loft, sh, xs, cell);
  }
  return containmentSummary(rows, openings, outside, exposed, { required, phases, amps, cell, Cd: decl.Cd_out ?? 1.2 });
}

/** The declared opening a solid belongs to (by its part, or by part/solid), if any. */
export const declaredOpening = (list, part, sol) => (list ?? []).find((o) => (o.solids ?? [o.part]).some((k) => k === part || k === `${part}/${sol}`));

/** An opening's running record: the solids in it and the window of skin they cross. */
const newOpening = (o) => ({ name: o.name, why: o.why, drag: o.drag !== false, solids: [], x: [Infinity, -Infinity], angle: [Infinity, -Infinity], max_out_m: -Infinity, past_loft: false });

/**
 * One solid's gaps to the loft over its shapes (a moving solid's poses) and x samples: the worst (smallest) signed gap,
 * the smallest clearance outward (for an outside fitting), and, for a solid of an opening, the window of skin where it
 * comes within `required` of or crosses the skin (x, polar angle, the most beyond it; past the loft's ends noted).
 */
export function solidLoftGaps(loft, shapes, xs, required, opening = null) {
  const hs = xs.map((x) => [x, loftAt(loft, x)]);
  let worst = Infinity, clearOut = Infinity;
  for (const sh of shapes) for (const p of outlinePoints(sh)) for (const [x, h] of hs) {
    const g = gapIn(h, p);
    worst = Math.min(worst, g);
    clearOut = Math.min(clearOut, -g);
    if (opening && g < required) widenWindow(opening, x, h, p, g);
  }
  return { worst, clearOut };
}

/** Widen an opening's window by one crossing point. */
function widenWindow(o, x, h, p, g) {
  o.x = [Math.min(o.x[0], x), Math.max(o.x[1], x)];
  if (!h) { o.past_loft = true; return; }
  const a = angleIn(h, p);
  o.angle = [Math.min(o.angle[0], a), Math.max(o.angle[1], a)];
  o.max_out_m = Math.max(o.max_out_m, -g);
}

/** Mark, on the (z, y) raster of `cell`, every cell of a cross-section that lies outside the loft at any of its x. */
export function markExposed(exposed, loft, sh, xs, cell) {
  const pts = outlinePoints(sh, 24), zs = pts.map((q) => q[0]), ys = pts.map((q) => q[1]);
  const hs = xs.map((x) => loftAt(loft, x));
  for (let iz = Math.floor(Math.min(...zs) / cell); iz <= Math.ceil(Math.max(...zs) / cell); iz++) for (let iy = Math.floor(Math.min(...ys) / cell); iy <= Math.ceil(Math.max(...ys) / cell); iy++) {
    const q = [(iz + 0.5) * cell, (iy + 0.5) * cell];
    if (inShape(sh, q) && hs.some((h) => gapIn(h, q) < 0)) exposed.add(`${iz},${iy}`);
  }
}

/** The containment record from its accumulated rows, openings, outside fittings and exposed cells. */
function containmentSummary(rows, openings, outside, exposed, { required, phases, amps, cell, Cd }) {
  rows.sort((a, b) => a.gap_m - b.gap_m);
  const enclosed = rows.filter((r) => r.gap_m < (r.on_skin ? -1e-9 : required - 1e-9));
  const outs = [...outside.values()], outBad = outs.filter((o) => !(o.min_clear_m >= required - 1e-9));
  const area = exposed.size * cell * cell;
  const r3 = (v) => +(v * 1000).toFixed(3);
  const crossing = (o) => (Number.isFinite(o.x[0]) ? { x_mm: o.x.map(r3), polar_angle_deg: o.angle.map((a) => +a.toFixed(1)), max_beyond_skin_mm: r3(o.max_out_m), ...(o.past_loft ? { reaches_past_the_loft: true } : {}) } : null);
  return {
    rule: 'SPEC Support and containment (2026-11-03): every part meant to be inside the pod keeps 0.5 mm inside the loft (model.js\'s elliptic measure, which understates the true distance); a part may cross the skin only at a declared opening; a fitting left outside keeps 0.5 mm clear of the skin and its frontal area counts in the pod\'s drag',
    required_mm: required * 1000,
    sampling: { crank_phases_per_amplitude: phases, amplitudes_rad: amps, points: 'section outlines (96 per circle), x at the ends, 7 between and every loft section plane' },
    solids_enclosed: rows.length,
    ...(rows.some((r) => r.on_skin) ? { on_skin: { rule: 'parts that carry the covering touch it from inside: at least zero', solids: rows.filter((r) => r.on_skin).length, closest_mm: r3(Math.min(...rows.filter((r) => r.on_skin).map((r) => r.gap_m))) } } : {}),
    closest_enclosed: rows.filter((r) => !r.on_skin).slice(0, 12).map((r) => ({ solid: r.solid, gap_mm: r3(r.gap_m) })),
    violations: enclosed.map((r) => ({ solid: r.solid, gap_mm: r3(r.gap_m) })),
    openings: [...openings.values()].map((o) => ({ name: o.name, why: o.why, counts_in_drag: o.drag, solids: o.solids, crossing: crossing(o) })),
    outside: outs.map((o) => ({ part: o.part, why: o.why, clear_of_skin_mm: r3(o.min_clear_m), ...(o.drag ? {} : { counts_in_drag: false }) })),
    outside_violations: outBad.map((o) => o.part),
    frontal: { area_mm2: +(area * 1e6).toFixed(1), Cd, CdA_m2: +(Cd * area).toFixed(7), cell_mm: cell * 1000, basis: `the outside fittings' and counted openings' material outside the loft, projected on the (z, y) plane (a shielded part counts once); Cd ${Cd}, a bluff body, as the pitot tip's load already takes it` },
    passed: enclosed.length === 0 && outBad.length === 0,
  };
}

/**
 * Round 15 (SPEC "Support and containment"): every packed part on the structure that carries it. Each row names the
 * carrier, the contact face and the fixing, and measures the gap from the part to that face (zero within `tol`:
 * touching; the assembly check separately proves they never interpenetrate). A part resting on a tray must also
 * have its centre of mass over the contact patch and every tie on the carrier.
 */
export function supportRecord(model, d, af, lay, { tol = 1e-6 } = {}) {
  const P = lay.planes, tF = mm(d.params.linkage.plates.t_fwd_mm ?? d.params.linkage.plates.t_mm), ok = (g) => Math.abs(g) <= tol;
  const rows = model.layout.parts.map((p) => (p.name === 'motor' ? motorSupport(p.box, af, P, tF, ok) : traySupport(p, af, ok)));
  if (P.encoder) rows.push(...encoderSupport(P.encoder, d, af, ok));
  if (af?.support?.pitot) rows.push(pitotSupport(af.support.pitot, af.carriers.nose, ok));
  return { rule: 'SPEC Support and containment (2026-11-03): every packed part rests on the structure that carries it through a stated fixing, touching (gap zero within the tolerance) and never interpenetrating (the assembly check, joined pairs at zero gap)', tolerance_mm: tol * 1000, rows, unsupported: rows.filter((r) => !r.passed).map((r) => r.part), passed: rows.every((r) => r.passed) };
}

const r4 = (v) => +(v * 1000).toFixed(4);
const unsupported = (part) => ({ part, carrier: null, face: null, fixing: null, gap_mm: null, passed: false, note: 'no stated mount: not supported' });

/** The motor on its back-face bracket (round 15), else (v22) on the forward plate's face. */
function motorSupport(b, af, P, tF, ok) {
  const mb = af?.motor_bracket;
  const gap = mb ? b.x0 - mb.x[1] : P.stage1 === 'overhung' ? P.plateFwd - tF / 2 - b.x1 : b.x0 - (P.plateFwd + tF / 2);
  return { part: 'motor', carrier: mb ? 'motor bracket' : 'gearbox plate forward', face: mb ? 'the bracket\'s forward face, on the motor\'s back face' : 'plate face', fixing: mb ? `${mb.holes.length} M3 into the motor's back-face threads (the maker's drawing)` : 'M3 into its own threads (airframe motor mount screws)', gap_mm: r4(gap), passed: ok(gap) && !!mb };
}

/** A packed part on a tray's base: touching, its centre of mass over the contact patch, its ties on the tray. */
function traySupport(p, af, ok) {
  const c = p.mount && carrierOf(af, p.mount.carrier);
  if (!c) return unsupported(p.name);
  const b = p.box, gap = b.y0 - (c.y + c.t / 2), inner = c.w / 2 - c.t / 2;
  const cx = [Math.max(b.x0, c.x[0]), Math.min(b.x1, c.x[1])], cz = [Math.max(b.z0, -inner), Math.min(b.z1, inner)];
  const tr = af.support?.ties?.find((t) => t.part === p.name);
  const tx = tr ? (tr.x_mm ? tr.x_mm.map((v) => v / 1000) : tieXs(b.x0, b.x1, tr.n)) : [];
  const cg = b.cx >= cx[0] && b.cx <= cx[1] && b.cz >= cz[0] && b.cz <= cz[1];
  const tiesOn = tx.every((x) => x >= c.x[0] && x <= c.x[1]);
  return { part: p.name, carrier: p.mount.carrier, face: 'base, top face', fixing: tr ? `${tr.n} x ${tr.tie} (${tr.loop_mm} mm loop)` : null, gap_mm: r4(gap), contact_x_mm: cx.map(r4), contact_z_mm: cz.map(r4), cg_over_contact: cg, ties_x_mm: tx.map(r4), passed: ok(gap) && cx[1] > cx[0] && cz[1] > cz[0] && cg && !!tr && tiesOn };
}

/** The crank encoder: the actuator in its bore on the crank stub, the readhead on the riser through its spacers. */
function encoderSupport(e, d, af, ok) {
  // round 18: bonded on its land, the gap is the bond line: within the adhesive maker's (the bond member's figures)
  const bm = (d.structure?.members ?? []).find((m) => m.name === ACTUATOR_BOND && m.check === 'gap bond'), bd = bm?.dims;
  const ag = e.id.actuator / 2 - mm(bd ? bd.land_d_mm : d.params.linkage.crank.shaft_od_mm) / 2, fx = af?.support?.readhead;
  const inLine = (g) => !!bd && g * 1000 >= bd.gap_limits_mm[0] - 1e-9 && g * 1000 <= bd.gap_limits_mm[1] + 1e-9;
  const rows = [bd
    ? { part: 'crank encoder actuator', carrier: 'crank', face: `the shaft's ${bd.land_d_mm} mm land (its bore, through the bond line)`, fixing: `${bm.material.id} bond (its 3 x M3 DIN 913 set screws centre it, not credited)`, gap_mm: r4(ag), gap_toleranced_mm: bd.gap_mm, bond_line_mm: bd.gap_limits_mm, passed: inLine(ag) && bd.gap_mm.every((g) => inLine(g / 1000)) }
    : { part: 'crank encoder actuator', carrier: 'crank', face: 'shaft stub (its bore)', fixing: '3 x M3 DIN 913 set screws (the maker\'s, included)', gap_mm: r4(ag), passed: ok(ag) }];
  if (!fx) return [...rows, unsupported('crank encoder readhead')];
  const R = af.carriers.riser, land = mm(fx.spacer_od_mm) / 2, gap = R.x[0] - (e.readhead[1] + mm(fx.spacer_mm));
  const onWeb = fx.holes_m.every(([z, y]) => Math.abs(z) + land <= R.w / 2 && y - land >= R.y[0] && y + land <= R.y[1]);
  return [...rows, { part: 'crank encoder readhead', carrier: 'nose tray riser', face: 'web, aft face (through its spacers)', fixing: `${fx.n} x ${fx.screw}, ${fx.spacer_mm} mm spacers, ${fx.nut}`, gap_mm: r4(gap), lands_on_web: onWeb, passed: ok(gap) && onWeb }];
}

/** The pitot against the nose tray's flange, both its ties on the tray and on the probe. */
function pitotSupport(pm, N, ok) {
  const gap = Math.abs(pm.c_m[0]) - pm.r_m - (N.w / 2 + N.t / 2);
  const tiesOn = pm.tie_x_mm.every((x) => x / 1000 >= Math.max(N.x[0], pm.x_m[0]) && x / 1000 <= Math.min(N.x[1], pm.x_m[1]));
  return { part: 'pitot-static probe', carrier: 'nose tray', face: pm.face, fixing: `${pm.n} x ${pm.tie} (${pm.loop_mm} mm loop)`, gap_mm: r4(gap), ties_x_mm: pm.tie_x_mm, ties_on_carrier: tiesOn, note: 'held by its ties along the flange (the probe overhangs forward)', passed: ok(gap) && tiesOn };
}

/**
 * Round 15: the containment check alone (the assembly and the packed parts against the loft, no pair check), for the
 * body fit (src/design/fit.js), which must hold every enclosed part. -> containmentRecord, or null without declarations.
 */
export function containmentOf(d, catalog, opts = {}) {
  const decl = d.params.layout?.containment;
  if (!decl || !d.params.linkage || !d.structure?.airframe) return null;
  const lay = driveLayoutOf(d, catalog);
  const asm = assembly(d, catalog, d.params.linkage, lay, d.structure.airframe);
  const pk = packedParts((id, solids, extra) => addPart(asm.parts, id, solids, extra), (a, b, w) => asm.joins.push([a, b, w]), d, catalog, d.structure.airframe.parts);
  afterPacked(asm, d, catalog, d.structure.airframe, pk);
  return containmentRecord(asm, d, lay, decl, opts);
}

/**
 * Round 17: the parts placed from the packed parts (they need the packing's boxes): the all-moving tail's linkage,
 * from its servos on the aft tray (tail-mech.js), when the design states it (params.tailMech).
 */
export function afterPacked(asm, d, catalog, airframe, pk) {
  if (!airframe?.parts || !pk?.model) return;
  const ax = { d, catalog, af: airframe.parts, add: (id, solids, extra) => addPart(asm.parts, id, solids, extra), join: (a, b, w) => asm.joins.push([a, b, w]), assumptions: asm.assumptions };
  if (d.params.tailMech) asm.tailMech = tailMechParts(ax, (tag) => pk.model.layout.parts.find((p) => p.name === `servo_tail_${tag}`).box);
  // round 17b (build-needs.js): the hatch, connectors, leads, tubing and antenna, which need the packed parts' boxes
  if (d.params.layout?.hatch) {
    const lay = driveLayoutOf(d, catalog), amps = [...new Set([d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite))];
    const poses = amps.flatMap((amp) => Array.from({ length: 36 }, (_, i) => drivePose({ law: lay.law, fourBar: lay.fb }, (i / 36) * TWO_PI, amp)));
    const nx = { ...ax, asm, q0: drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampNominal), poses };
    asm.needs = needsParts(nx, pk);
    if (asm.needs) asm.needs.harnessTies = harnessTies(nx, asm.needs.leads, asm.needs.P);
  }
}

/**
 * Round 16: every solid the checks read, of a built design: the assembly (assembly()) and the packed parts with their
 * ties (packedParts). -> { asm, lay, pod, model }.
 */
export function fullAssembly(d, catalog, L = d.params.linkage, lay = driveLayoutOf(d, catalog), airframe = d.structure.airframe) {
  const asm = assembly(d, catalog, L, lay, airframe);
  const pk = packedParts((id, solids, extra) => addPart(asm.parts, id, solids, extra), (a, b, w) => asm.joins.push([a, b, w]), d, catalog, airframe?.parts);
  afterPacked(asm, d, catalog, airframe, pk);
  return { asm, lay, pod: pk.pod, model: pk.model };
}

/** The check on a built design (its own layout, linkage and airframe), as the build records it. */
export function clearanceRecord(d, catalog, opts) {
  return checkClearance(d, catalog, d.params.linkage, driveLayoutOf(d, catalog), d.structure.airframe, opts);
}

// ---- the check --------------------------------------------------------------------------------------------------

const keyOf = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`);

/** Pair class for the report: a pin stack shares a pin axis (both solids are rings round one centre), else by motion. */
function pairClass(a, b, sa, sb) {
  if (sa.shape?.kind === 'ring' && sb.shape?.kind === 'ring' && len(sub(sa.shape.c, sb.shape.c)) < 1e-9) return 'shaft or pin stack';
  if (a.at || b.at) return (a.at && b.at) ? 'moving / moving' : 'moving / fixed';
  return 'fixed / fixed';
}

// round 17: the join reasons live in join-reasons.js (interfaces.js reads them without importing this module)
export { MESH, KEYWAY, THROUGH, THREAD, RIB_HOLE, BORE, GROOVE, EXEMPT_JOINTS } from './join-reasons.js';

/**
 * Every pair of solids of two different parts. Round 14 (the brief's check of the check: fastened faces may touch, at
 * zero gap, but must never interpenetrate): a pair whose parts are joined is checked too, against zero instead of the
 * running clearance; only solids of one part and gears in mesh (their teeth interleave) are exempt.
 * -> { solids, pairs, nExempt }; a pair carries `required` 0 and its `join` when its parts are joined.
 */
export function solidPairs(asm, required) {
  const joined = new Map(asm.joins.map(([a, b, why]) => [keyOf(a, b), why]));
  const solids = asm.parts.flatMap((p) => p.solids.map((s) => ({ ...s, part: p.id, id: `${p.id}/${s.name}` })));
  // the most specific joint first (round 16: a screw's shank through its hole, its head on the part's face)
  const joinOf = (a, b) => joined.get(keyOf(a.id, b.id)) ?? joined.get(keyOf(a.id, b.part)) ?? joined.get(keyOf(a.part, b.id)) ?? joined.get(keyOf(a.part, b.part));
  const pairs = [];
  let nExempt = 0;
  for (let i = 0; i < solids.length; i++) for (let k = i + 1; k < solids.length; k++) {
    const a = solids[i], b = solids[k], j = a.part === b.part ? MESH : joinOf(a, b);
    if (EXEMPT_JOINTS.includes(j)) nExempt++;
    else pairs.push({ a, b, min: Infinity, at: null, required: j ? 0 : required, ...(j ? { join: j } : {}) });
  }
  return { solids, pairs, nExempt };
}

/** A pair under its requirement (the running clearance, or zero for joined parts, 0.1 um of rounding allowed). */
const under = (p) => p.min < p.required - (p.join ? 1e-7 : 1e-9);

const shapeAt = (s, q) => (s.at ? s.at(q) : s.shape);
const evalPair = (p, q) => gap3(p.a.x, p.b.x, gap2(shapeAt(p.a, q), shapeAt(p.b, q)));

/** Each moving pair's smallest gap over the sampled phases, then refined between its neighbouring samples. */
/** Round 17b: how far apart (m) a swept envelope must be for its pair to skip sampling (the refine threshold). */
export const SWEEP_SKIP = 0.01;

export function sampleMoving(moving, amps, phases, poseAt, q0) {
  // round 17b (retain2): a moving solid that states its swept envelope (a key turning in its gear: `sweep`, a static
  // shape every pose of it lies in) against a fixed one, or another such, more than SWEEP_SKIP apart: that envelope gap
  // is a lower bound of the pair's gap at every pose, recorded as its minimum without sampling (swept: true)
  const sampled = [];
  for (const p of moving) {
    const A = p.a.at ? p.a.sweep : p.a.shape, B = p.b.at ? p.b.sweep : p.b.shape;
    const g = A && B ? gap3(p.a.x, p.b.x, gap2(A, B)) : -Infinity;
    if (g >= SWEEP_SKIP) Object.assign(p, { min: g, at: { amp: amps[0], i: 0 }, swept: true });
    else sampled.push(p);
  }
  for (const amp of amps) {
    for (let i = 0; i < phases; i++) {
      const q = poseAt(amp, (i / phases) * TWO_PI);
      for (const p of sampled) {
        const g = evalPair(p, q);
        if (g < p.min) { p.min = g; p.at = { amp, i }; }
      }
    }
  }
  const h = TWO_PI / phases;
  for (const p of moving) {
    p.class = pairClass(p.a, p.b, { shape: shapeAt(p.a, q0) }, { shape: shapeAt(p.b, q0) });
    p.phase = (p.at.i / phases) * TWO_PI;
    if (!(p.min < 0.01) || p.swept) continue;
    const best = goldenMin((ph) => evalPair(p, poseAt(p.at.amp, ph)), (p.at.i - 1) * h, (p.at.i + 1) * h, 30);
    if (best[0] < p.min) { p.min = best[0]; p.phase = ((best[1] % TWO_PI) + TWO_PI) % TWO_PI; }
  }
}

/** A pair as the record prints it. */
const pairRow = (p) => ({ a: p.a.id, b: p.b.id, gap_mm: +(p.min * 1000).toFixed(3), class: p.class, ...(p.join ? { joined: p.join, required_mm: 0 } : {}), ...(p.a.at || p.b.at ? { phase_deg: +((p.phase * 180) / Math.PI).toFixed(1), amp_rad: p.at?.amp ?? null } : {}) });

/** One row per pair of parts under the required gap (its closest solids), closest first. */
export function partPairClashes(pairs) {
  const worst = new Map();
  for (const p of pairs) {
    if (!under(p)) continue;
    const k = keyOf(p.a.part, p.b.part);
    if (!worst.has(k) || p.min < worst.get(k).min) worst.set(k, p);
  }
  return [...worst.values()].sort((u, v) => u.min - v.min).map(pairRow);
}

/** The closest pair of each class. */
function minByClass(pairs) {
  const byClass = {};
  for (const p of pairs) { if (p.join) continue; const c = byClass[p.class]; if (!c || p.min < c.min) byClass[p.class] = p; }
  return Object.fromEntries(Object.entries(byClass).map(([k, p]) => [k, pairRow(p)]));
}

/** The pod-shell rows: each packed part's gap to the loft (the gear rows carry only model.js's protrusion). */
export function podRecord(pod, required) {
  const rows = pod.map((r) => ({ part: r.part, gap_mm: +(r.gap_m * 1000).toFixed(3), ...(r.protrusion_only ? { note: 'model.js reports only the protrusion beyond the loft (0 inside)' } : {}) }));
  const clashes = pod.filter((r) => !r.protrusion_only && r.gap_m < required - 1e-9).map((r) => r.part);
  return { rule: 'packed parts against the pod loft (model.js boxProtrusion), the same 0.5 mm', rows, clashes };
}

/**
 * The clearance check: every pair of solids of parts not joined, at `phases` crank angles per stroke amplitude, the
 * smallest gap refined by a golden-section search between the neighbouring samples. Returns the design record.
 */
export function checkClearance(d, catalog, L, lay, airframe, { phases = 360, required = CLEARANCE_M, pack = true, all = false } = {}) {
  const asm = assembly(d, catalog, L, lay, airframe);
  const add = (id, solids, extra) => addPart(asm.parts, id, solids, extra);
  const pk = pack ? packedParts(add, (a, b, w) => asm.joins.push([a, b, w]), d, catalog, airframe?.parts) : { pod: [], model: null };
  if (pack) afterPacked(asm, d, catalog, airframe, pk);
  const { solids, pairs, nExempt } = solidPairs(asm, required);
  const amps = [...new Set([d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite))];
  const poseAt = (amp, ph) => drivePose({ law: lay.law, fourBar: lay.fb }, ph, amp);
  const q0 = poseAt(amps[0], 0);
  for (const p of pairs) if (!p.a.at && !p.b.at) { p.min = evalPair(p, q0); p.class = pairClass(p.a, p.b, p.a, p.b); }
  sampleMoving(pairs.filter((p) => p.a.at || p.b.at), amps, phases, poseAt, q0);
  const clashes = partPairClashes(pairs);
  const podR = podRecord(pk.pod, required);
  const recs = designRecords(asm, d, catalog, lay, airframe, pk, q0, { required, pack });
  return {
    rule: 'SPEC Design freeze, Assembly clearance (2026-11-03): at least 0.5 mm between every pair of parts that does not share a joint, through the full stroke; at least 0.5 mm running clearance between parts stacked on one pin (no catalogue thrust washer is credited); parts joined directly may touch but never interpenetrate (round 14), gears in mesh excepted',
    required_mm: required * 1000,
    method: 'parts as prisms along body x (rings, capsules, convex polygons with holes in the (z, y) plane); gap sqrt(gx^2 + g2^2), overlap as the smaller penetration; src/design/clearance.js',
    sampling: { crank_phases_per_amplitude: phases, amplitudes_rad: amps, refine: 'golden-section search between the neighbouring samples for every moving pair under 10 mm', swept: `a moving solid with a stated swept envelope (the keys, turning with their shafts) more than ${SWEEP_SKIP * 1000} mm from the other solid by that envelope is recorded at the envelope's gap (a lower bound at every pose), not sampled; nearer, it is sampled like any other (a key turns several times per crank turn, but its envelope lies inside its gear's and shaft's own, both exempt as KEYWAY)` },
    parts: asm.parts.length, solids: solids.length, pairs_checked: pairs.length, pairs_joined_checked_at_zero: pairs.filter((p) => p.join).length, pairs_exempt: nExempt, solid_pairs_clashing: pairs.filter(under).length,
    joints: asm.joins.map(([a, b, why]) => ({ a, b, why })),
    min_by_class: minByClass(pairs),
    clashes, // per pair of parts, the closest solids
    pod_shell: podR,
    ...recs,
    assumptions: asm.assumptions,
    passed: clashes.length === 0 && podR.clashes.length === 0 && Object.values(recs).every((r) => r.passed),
    // every solid pair (for tests and probes; not written to the design file)
    ...(all ? { all_pairs: Object.fromEntries(pairs.map((p) => [`${p.a.id} | ${p.b.id}`, pairRow(p)])) } : {}),
  };
}

/**
 * The records a design asks for beside the pair check: round 15's support and containment (a design that states its
 * mounts and openings), round 16's completeness, assembly sequence and holes (a design that states them).
 */
export function designRecords(asm, d, catalog, lay, airframe, pk, q0, { required, pack }) {
  const Ly = d.params.layout ?? {}, out = {};
  if (Ly.mounts && pk.model) out.support = supportRecord(pk.model, d, airframe?.parts, lay);
  // round 17b: the twist servos in their frames (twist-drive.js), and every wing part inside the real E387
  if (out.support && d.params.twistDrive) { const tw = twistSupportRows(d, catalog); out.support.rows.push(...tw); out.support.unsupported.push(...tw.filter((r) => !r.passed).map((r) => r.part)); out.support.passed = out.support.passed && tw.every((r) => r.passed); }
  if (d.params.twistDrive) out.section = sectionRecord(d, catalog, d.params.twistDrive.section ?? {});
  if (Ly.containment) out.containment = containmentRecord(asm, d, lay, Ly.containment, { required });
  if (Ly.completeness && pack) out.completeness = completenessRecord(asm, d, catalog, q0);
  if (Ly.sequence && pack) out.assembly = sequenceRecord(asm, d, lay);
  if (airframe?.parts?.fixings) out.holes = holeRecord(asm, q0);
  // round 17 (SPEC "Function completeness"): the lumps at their drawn parts' centroids
  if (Ly.massprops && pack) out.massprops = massPropsRecord(asm, d, catalog, q0);
  // round 17: the actuation chains (each module's), the ratings, and the interface ledger with each module's rule rows
  if (Ly.chains && pack) out.chains = chainRecord(asm, d, chainSources().flatMap((f) => f(asm, d, catalog)));
  if (Ly.ratings) out.ratings = ratingsRecord(d, catalog);
  if (Ly.interfaces && pack) out.interfaces = interfaceLedger(asm, d, [...(Ly.hatch ? NEEDS_INTERFACE_RULES : []), ...(d.params.pod ? POD_INTERFACE_RULES : []), ...moduleRules()]);
  // round 17b (SPEC "What a real build needs"): connectors, the hatch and its fixing, access, the air path
  if (Ly.hatch && pack) out.buildneeds = buildNeedsRecord(asm, d, catalog);
  // round 17 (SPEC "The covering spans only what is supported"): the pod's film spans and its cuts, the wing's and tail's rib panels
  if (Ly.covering && pack && d.params.pod) out.covering = coveringRecord(asm, d, catalog, { required, lay });
  return out;
}

/** Round 17: the modules' chain sources and interface rule rows (functions, so a module cycle never reads them early). */
const chainSources = () => [tailChains, flapChains, twistChains];
const moduleRules = () => [...TAIL_INTERFACE_RULES, ...TWIST_INTERFACE_RULES, ...RETAIN_INTERFACE_RULES];
