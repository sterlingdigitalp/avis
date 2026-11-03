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
import { drivePose, linkageAt, journalDims, journalBodyLength, partsIndex, prepare, boxProtrusion, parseDims, packRegulator, loftHalf } from '../../sims/avis-shared/model.js';
import { byId } from './catalog.js';
import { AIRFRAME_DEFAULTS, mergeDefaults, lockFlangeOD, hullOf, lockFlangeCircles } from './airframe.js';
import { driveLayout as driveLayoutOf } from './joints.js';
import { rockerHub } from './linkage.js';

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

/** Distance between segments ab and cd (0 when they cross). */
export function segSegDist(a, b, c, d) {
  const cr = (o, p, q) => (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
  const d1 = cr(c, d, a), d2 = cr(c, d, b), d3 = cr(a, b, c), d4 = cr(a, b, d);
  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return 0;
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
    add: (id, solids, extra = {}) => { parts.push({ id, solids, ...extra }); return id; },
    join: (a, b, why) => joins.push([a, b, why]),
    shaftPart: (j) => (j === 0 ? 'crank' : j === n ? 'motor' : `gear shaft ${j}`),
  };
}
const S = (name, x, shape) => ({ name, x, shape });
const M = (name, x, at) => ({ name, x, at });

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
  return { parts: ax.parts, joins: ax.joins, assumptions: ax.assumptions };
}

/** Crank: web, pin, shaft and wheel seat in one piece. */
function crankParts({ L, P, n, fb, lay, eye, plateT, add }) {
  const C = L.crank, tw = mm(C.web_t_mm), bw = mm(C.web_b_mm);
  const xw = P.crankArm - tw / 2;
  const pinEnd = P.couplerL - eye.B / 2 - 0.001;
  const wheelN = lay.train.stages[n - 1].wheel;
  add('crank', [
    M('web', [P.crankArm - tw / 2, P.crankArm + tw / 2], (q) => cap(q.O2, q.A, bw / 2)),
    M('pin', [pinEnd, xw], (q) => ring(q.A, eye.d / 2)),
    S('shaft', [P.crankArm + tw / 2, P.crankEnd ?? P.plateFwd + plateT / 2 + 0.001], ring(fb.O2, mm(C.shaft_od_mm) / 2, mm(C.shaft_id_mm ?? 0) / 2)),
    ...(C.seat_od_mm ? [S('wheel seat', [P.stage[n - 1] - wheelN.face / 2, P.stage[n - 1] + wheelN.face / 2], ring(fb.O2, mm(C.seat_od_mm) / 2, mm(C.shaft_od_mm) / 2))] : []),
  ]);
}

/** Round 14: the crank encoder where the planes put it (model.js clearancePlanes, encoder 'front'): readhead and actuator rings. */
function encoderParts({ P, fb, af, add, join, assumptions }) {
  const e = P.encoder;
  if (!e) return;
  const rh = add('crank encoder readhead', [S('ring', e.readhead, ring(fb.O2, e.od.readhead / 2, e.id.readhead / 2))]);
  const ac = add('crank encoder actuator', [S('ring', e.actuator, ring(fb.O2, e.od.actuator / 2, e.id.actuator / 2))]);
  join(ac, 'crank/shaft', 'on the crank shaft by its three set screws (maker\'s fixing)');
  const front = e.readhead[0] > P.plateFwd;
  const fx = af?.support?.readhead;
  if (fx) readheadFixing({ add, join, assumptions }, rh, e, fx, af.carriers.riser);
  else join(rh, front ? 'gearbox plate forward' : 'gearbox plate aft', `on its bracket to ${front ? 'the forward plate\'s outer face' : 'the aft plate\'s crank boss'}`);
  assumptions.push('crank encoder (round 14): readhead as its ring envelope (outer diameter, through-hole, thickness with connector), the actuator at the ride height from the readhead\'s face, coaxial with the crank (model.js clearancePlanes)');
}

/**
 * Round 15: the readhead's own fixing (BRD01: 3 x M2 DIN 912 with spacers) as one part: each screw's head on the
 * readhead's back face, its spacer from the readhead's front face to the riser's web, its nut on the web's forward
 * face. The shanks run inside the holes and are not drawn.
 */
function readheadFixing({ add, join, assumptions }, rh, e, fx, riser) {
  const mm1 = (v) => v / 1000, hr = mm1(fx.screw_head.d_mm) / 2, k = mm1(fx.screw_head.k_mm), nr = mm1(fx.nut_s_mm) / Math.sqrt(3);
  const sol = fx.holes_m.flatMap((c, i) => [
    S(`head ${i + 1}`, [e.readhead[0] - k, e.readhead[0]], ring(c, hr)),
    S(`spacer ${i + 1}`, [e.readhead[1], e.readhead[1] + mm1(fx.spacer_mm)], ring(c, mm1(fx.spacer_od_mm) / 2, mm1(fx.spacer_id_mm) / 2)),
    S(`nut ${i + 1}`, [riser.x[1], riser.x[1] + mm1(fx.nut_m_mm)], ring(c, nr, mm1(fx.spacer_id_mm) / 2)),
  ]);
  const id = add('crank encoder readhead fixing', sol);
  join(id, rh, 'its screws through the readhead\'s holes, heads on its back');
  join(id, 'nose tray riser', 'spacers on the riser\'s web, nuts on its forward face');
  assumptions.push('crank encoder readhead (round 15): on the nose tray riser by the maker\'s fixing, 3 x M2 DIN 912 through its mounting holes on the 34 mm circle with spacers of under 4 mm diameter to the riser\'s web (BRD01); heads, spacers and nuts drawn as rings (a nut by its corner circle), the shanks inside the holes not drawn');
}

const airframeOf = (ax) => ax.af;

/** Gear shafts, the motor and every gear: link j joins shaft j (wheel of stage n-1-j) to shaft j+1 (its pinion). */
function gearParts(ax0) {
  const { d, catalog, L, lay, P, n, sh, plateT, add, join, shaftPart, assumptions } = ax0;
  const ms = byId(catalog, d.motor?.id ?? d.params.parts.motor).specs;
  const md = parseDims(ms.dimensions_text_mm)?.values ?? [0.035, 0.04];
  const motorShaft = mm(ms.shaft_mm ?? 5);
  // A shaft runs plate to plate, 1 mm proud of each; round 14, to the far face of a gear it carries outside the plates.
  const farEnd = (si, g) => P.stage[si] + g.face / 2 + (g.hubW ?? 0);
  for (let j = 1; j < n; j++) {
    const g = L.gearShafts[j - 1];
    const kw = n - 1 - j, wheel = lay.train.stages[kw].wheel;
    const xEnd = Math.max(P.plateFwd + plateT / 2 + 0.001, farEnd(kw, wheel), farEnd(n - j, lay.train.stages[n - j].pinion));
    add(`gear shaft ${j}`, [
      S('shaft', [P.plateAft - plateT / 2 - 0.001, xEnd], ring(sh[j], mm(g.d_mm) / 2)),
      ...(g.seat_od_mm ? [S('wheel seat', [P.stage[kw] - wheel.face / 2, P.stage[kw] + wheel.face / 2], ring(sh[j], mm(g.seat_od_mm) / 2, mm(g.d_mm) / 2))] : []),
    ]);
  }
  const can = P.motorX ?? [P.motorAft, P.motorAft + md[1]], p1 = lay.train.stages[0].pinion;
  add('motor', [
    S('can', can, ring(sh[n], md[0] / 2)),
    // its shaft from its face to the first pinion: aft (the motor forward of the box) or, round 14, forward through the plate
    S('shaft', P.stage1 === 'overhung' ? [P.motorFace, farEnd(0, p1)] : [P.stage[0] - p1.face / 2, P.motorAft], ring(sh[n], motorShaft / 2)),
  ]);
  // round 15: the motor on its back-face bracket (structure.airframe.parts.motor_bracket), bonded on both rails
  const mb = airframeOf(ax0)?.motor_bracket;
  if (mb) {
    add('motor bracket', [S('plate', mb.x, poly(mb.outline, [...mb.sleeves.map((q) => ({ c: q.c, r: q.ri })), { c: mb.stub.c, r: mb.stub.r }])), ...mb.sleeves.map((q) => S(`sleeve ${q.side}`, mb.x, ring(q.c, q.ro, q.ri)))]);
    join('motor bracket', 'motor', 'screwed into its back-face threads');
    for (const q of mb.sleeves) join('motor bracket', `shoulder rail ${q.side}`, 'its sleeve bonded on the rail');
    assumptions.push('motor bracket (round 15): the motor hangs by its back face (the maker\'s 4 x M3 threads, 25 mm across) on a plate bonded on both shoulder rails; its bell clears the forward plate and the rail\'s lock flange');
  } else join('motor', 'gearbox plate forward', 'screwed to the forward plate');
  const boreOn = (k) => (k === n ? motorShaft : mm(k === 0 ? L.crank.seat_od_mm ?? L.crank.shaft_od_mm : L.gearShafts[k - 1].d_mm));
  for (let si = 0; si < n; si++) {
    const st = lay.train.stages[si], j = n - 1 - si, x = P.stage[si];
    for (const [g, k, tag] of [[st.wheel, j, 'wheel'], [st.pinion, j + 1, 'pinion']]) {
      const bore = g.bore || boreOn(k);
      const id = add(`stage ${si + 1} ${tag} ${g.id}`, [
        S('teeth', [x - g.face / 2, x + g.face / 2], ring(sh[k], g.ro, bore / 2)),
        ...(g.hub && g.hubW > 0 ? [S('hub', [x + g.face / 2, x + g.face / 2 + g.hubW], ring(sh[k], g.hub / 2, bore / 2))] : []),
      ], { gear: { stage: si } });
      for (const sol of ['shaft', 'wheel seat']) join(id, `${shaftPart(k)}/${sol}`, 'gear on its shaft (bore on the shaft or seat)');
    }
    join(`stage ${si + 1} wheel ${st.wheel.id}`, `stage ${si + 1} pinion ${st.pinion.id}`, MESH);
  }
  assumptions.push('gear hubs on the forward side of the face (model.js stagePlanes allots each stage its hub width there)');
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
  ]);
  join(pid, `${shaftPart(k)}/shaft`, 'journal on its shaft');
  join(pid, plateIds[side], 'pressed in its plate seat');
  holes[side].push({ c: sh[k], r: dims.D / 2 });
  if (!(PL.bossed ?? []).includes(`${jname}/${support}`)) return;
  const b = bossBeyond(PL, tp, dims), inner = xp + (side === 'aft' ? 1 : -1) * tp / 2;
  bosses[side].push(S(`boss ${jname}/${support}`, side === 'aft' ? [inner, inner + b] : [inner - b, inner], ring(sh[k], dims.D / 2 + mm(PL.boss_wall_mm), dims.D / 2)));
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
  for (const side of ['aft', 'fwd']) add(plateIds[side], plateSolids(ax, side, [...holes[side], ...rails], boomHole, bosses[side]));
  if (boomHole.length) assumptions.push('gearbox plates and their posts: a clearance hole for the tail boom, the running clearance round it; the posts and pads to the boom clamps and trays as the airframe states them (airframe.js platePosts)');
}

/** One gearbox plate's solids: its outline with its holes, the posts and pads the airframe states, its bosses. */
function plateSolids({ P, tA, tF, af, plateIds, assumptions }, side, holes, boomHole, bosses) {
  const xp = side === 'aft' ? P.plateAft : P.plateFwd, tp = side === 'aft' ? tA : tF, xs = [xp - tp / 2, xp + tp / 2];
  const pl = af?.plates?.[side], outline = pl?.outline_m;
  if (!outline) assumptions.push(`${plateIds[side]}: no outline in the design (no airframe), checked by its bosses and journals only`);
  const posts = (pl?.posts ?? []).filter((q) => q.post_m).flatMap((q, i) => [S(`post ${i + 1} (${q.to})`, xs, boxYZ(...q.post_m).withHoles(boomHole)), S(`pad ${i + 1} (${q.to})`, xs, boxYZ(...q.pad_m).withHoles(boomHole))]);
  return [...(outline ? [S('plate', xs, poly(outline, [...holes, ...boomHole]))] : []), ...posts, ...bosses];
}

/** The tail boom tube's outer radius (m), from its catalogue entry. */
function boomRadius({ d, catalog }) {
  const id = d.params?.parts?.keel;
  return id ? mm(byId(catalog, id).specs.od_mm) / 2 : 0;
}

/** One side's rocker (hub, shoulder pin, arm, horn boss and pin, spar socket), shoulder joint and coupler. */
function sideParts(ax, tag, sg, delta) {
  const { L, P, x0, fb, eye, shJ, span, add } = ax;
  const O4 = sg > 0 ? fb.O4 : mirror(fb.O4);
  const xc = sg > 0 ? P.couplerR : P.couplerL;
  const pose = (q) => (sg > 0 ? q.right : q.left);
  const sparDir = (q) => dir(sg > 0 ? q.right.rocker - delta : q.left.rocker + delta);
  const R = L.rocker, hub = rockerHub(ax.d, R, span, shJ.B, P, eye.B);
  const H = hornStack(ax.Ly, tag, xc, eye.B, mm(R.boss_len_mm)), [bossA, bossF] = H.boss, armX = (bossA + bossF) / 2;
  const so = mm(R.spar_od_mm ?? 10) / 2 + mm(R.socket_wall_mm), r0 = mm(R.hub_od_mm) / 2;
  const rk = add(`rocker ${tag}`, [
    ...(hub.relief ? [S('hub', [hub.x[0], hub.relief[0]], ring(O4, r0, shJ.d / 2)), S('hub relief', hub.relief, ring(O4, mm(R.hub_relief_od_mm) / 2, shJ.d / 2)), S('hub forward', [hub.relief[1], hub.x[1]], ring(O4, r0, shJ.d / 2))] : [S('hub', hub.x, ring(O4, r0, shJ.d / 2))]),
    ...(hub.collar ? [S('pin collar', hub.collar, ring(O4, mm(R.hub_collar_od_mm) / 2, shJ.d / 2))] : []),
    S('shoulder pin', [x0 - (span + shJ.B) / 2, x0 + (span + shJ.B) / 2], ring(O4, shJ.d / 2)),
    M('arm', [armX - mm(R.arm_t_mm) / 2, armX + mm(R.arm_t_mm) / 2], (q) => cap(O4, pose(q).B, mm(R.arm_w_mm) / 2)),
    M('horn boss', [bossA, bossF], (q) => ring(pose(q).B, mm(R.boss_od_mm) / 2, eye.d / 2)),
    M('horn pin', H.pin, (q) => ring(pose(q).B, eye.d / 2)),
    M('spar socket', [x0 - so, x0 + so], (q) => { const u = sparDir(q); return cap(add2(O4, u, r0), add2(O4, u, r0 + mm(R.socket_len_mm)), so); }),
  ]);
  for (const [nm, xs, fl] of [['aft', x0 - span / 2, 1], ['forward', x0 + span / 2, -1]]) shoulderParts(ax, tag, sg, O4, nm, xs, fl, rk);
  couplerParts(ax, tag, xc, pose, rk);
}

/**
 * Where a rocker's horn boss and horn pin sit along x beside its coupler eye (centre xc, length B): the boss
 * params.layout.horn_gap_mm (1 mm, v21) forward of the eye, or round 14 aft of it (params.layout.horns[tag] 'aft'), the
 * pin from the boss's far face through the eye to params.layout.horn_pin_past_mm (1 mm, v21) beyond it. -> { boss, pin } (m).
 */
export function hornStack(Ly, tag, xc, B, bossLen) {
  const gap = (Ly?.horn_gap_mm ?? 1) / 1000, past = (Ly?.horn_pin_past_mm ?? 1) / 1000;
  if ((Ly?.horns?.[tag] ?? 'fwd') === 'aft') {
    const b1 = xc - B / 2 - gap;
    return { boss: [b1 - bossLen, b1], pin: [b1 - bossLen, xc + B / 2 + past], side: 'aft' };
  }
  const b0 = xc + B / 2 + gap;
  return { boss: [b0, b0 + bossLen], pin: [xc - B / 2 - past, b0 + bossLen], side: 'fwd' };
}

/** A shoulder bushing (flange inboard against the hub, counterbored in the housing: round 11) and its bracket. */
function shoulderParts(ax, tag, sg, O4, nm, xs, fl, rk) {
  const { L, A, af, shJ, railR, add, join } = ax;
  const side = sg > 0 ? 'right' : 'left';
  const bx = [xs - shJ.B / 2, xs + shJ.B / 2];
  const fx = fl > 0 ? [bx[1] - shJ.flangeB, bx[1]] : [bx[0], bx[0] + shJ.flangeB];
  const bid = add(`shoulder bushing ${tag} ${nm}`, [S('body', bx, ring(O4, shJ.D / 2, shJ.d / 2)), ...(shJ.flangeB ? [S('flange', fx, ring(O4, shJ.flangeD / 2, shJ.d / 2))] : [])]);
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
  add(hid, hs);
  join(bid, hid, 'pressed in the housing');
  join(bid, `${rk}/shoulder pin`, 'journal on the shoulder pin');
}

/**
 * A bracket's sleeve along the rail: centred on its housing (v21), unless that puts it in a gearbox plate, where the rail
 * is seated: then (round 14, a housing forward of the aft plate) it starts the running clearance clear of that plate.
 */
export function sleeveX({ P, tA, tF, Ly }, xs, sl) {
  const iv = [xs - sl / 2, xs + sl / 2], run = (Ly?.running_mm ?? 0.5) / 1000;
  for (const [xp, tp] of [[P.plateAft, tA], [P.plateFwd, tF]]) {
    if (iv[0] < xp + tp / 2 + run && iv[1] > xp - tp / 2 - run) return xs > xp ? [xp + tp / 2 + run, xp + tp / 2 + run + sl] : [xp - tp / 2 - run - sl, xp - tp / 2 - run];
  }
  return iv;
}

/** A coupler: one piece, an eye round a bushing at each end; the crank-end eye on the crank pin, the other on the horn pin. */
function couplerParts({ L, eye, add, join }, tag, xc, pose, rk) {
  const cp = L.coupler, ck = `coupler ${tag}`, eo = mm(cp.eye_od_mm) / 2, rb = mm(cp.body_d_mm) / 2;
  const xe = [xc - eye.B / 2, xc + eye.B / 2];
  add(ck, [
    M('crank eye', xe, (q) => ring(q.A, eo, eye.D / 2)),
    M('horn eye', xe, (q) => ring(pose(q).B, eo, eye.D / 2)),
    // the round body between the eyes (it starts 1 mm inside each eye, as the film draws it)
    M('body', [xc - rb, xc + rb], (q) => between(q.A, pose(q).B, eo - 0.001, eo - 0.001, rb)),
  ]);
  for (const [end, at] of [['crank', (q) => q.A], ['horn', (q) => pose(q).B]]) {
    const bid = add(`coupler ${tag} ${end} bushing`, [M('body', xe, (q) => ring(at(q), eye.D / 2, eye.d / 2))]);
    join(bid, ck, 'pressed in the eye');
    join(bid, end === 'crank' ? 'crank/pin' : `${rk}/horn pin`, 'journal on its pin');
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

function frameParts({ A, af, P, plateT, tA, tF, x0, span, railR, d, catalog, plateIds, assumptions, add, join }) {
  const xA = P.plateAft, xF = P.plateFwd, gap = xF - xA, xa = x0 - span / 2;
  for (const [side, c] of Object.entries(af.rails ?? {})) {
    const railLen = gap + plateT + (xA - xa) + mm(A.rail.sleeve_len_mm) / 2 + 0.002;
    // its torsion lock flange, 2 mm, on the face of the plate it is screwed to (airframe.js: rail.lock_at)
    const lt = mm(A.rail.lock_t_mm ?? 2);
    const fx = A.rail.lock_at === 'forward' ? [xF - tF / 2 - lt, xF - tF / 2] : [xA + tA / 2, xA + tA / 2 + lt];
    const id = add(`shoulder rail ${side}`, [S('tube', [xF + plateT / 2 - railLen, xF + plateT / 2], ring(c, railR, mm(A.rail.id_mm) / 2)), S('lock flange', fx, lockFlangeShape(A.rail, c, side === 'right' ? 1 : -1, railR))]);
    join(id, plateIds.aft, 'seated in the plate'); join(id, plateIds.fwd, 'seated in the plate');
  }
  for (const q of A.panels.list) {
    const t = mm(q.t_mm) / 2;
    const sh = q.kind === 'side' ? cap([mm(q.z_mm), mm(q.y0_mm)], [mm(q.z_mm), mm(q.y0_mm + q.h_mm)], t) : cap([-mm(q.w_mm) / 2, mm(q.y_mm)], [mm(q.w_mm) / 2, mm(q.y_mm)], t);
    const id = add(`gearbox ${q.name}`, [S('sheet', [xA + tA / 2, xF - tF / 2], sh)]); // between the plates' inner faces
    join(id, plateIds.aft, 'flange bonded to the plate'); join(id, plateIds.fwd, 'flange bonded to the plate');
  }
  const bo = A.boom, tube = byId(catalog, d.params.parts.keel).specs, yB = af.boom_y_m;
  const cl = mm(bo.clamp_len_mm), blk = mm(tube.od_mm + 2 * bo.clamp_wall_mm);
  // round 15 (params.layout.plate_faces 'actual'): the clamps on the plates' own faces
  const [hA, hF] = d.params.layout?.plate_faces === 'actual' ? [tA / 2, tF / 2] : [plateT / 2, plateT / 2];
  const cA = [xA - hA - cl, xA - hA], cF = [xF + hF, xF + hF + cl];
  const boom = add('tail boom', [S('tube', [d.tail.x_m, cF[1]], ring([0, yB], mm(tube.od_mm) / 2, mm(tube.id_mm) / 2))]);
  for (const [nm, xs, pl] of [['aft', cA, plateIds.aft], ['forward', cF, plateIds.fwd]]) {
    const id = add(`boom clamp ${nm}`, [S('block', xs, boxYZ(-blk / 2, blk / 2, yB - blk / 2 - 0.006, yB + blk / 2).withHoles([{ c: [0, yB], r: mm(tube.od_mm) / 2 }]))]); // bored for the boom
    join(id, boom, 'bonded on the boom'); join(id, pl, 'screwed to the plate');
  }
  for (const [nm, T, xs, pl] of [['forward tray', A.tray, (Lt) => [xF + plateT / 2, xF + plateT / 2 + Lt], plateIds.fwd], ['aft tray', A.aftTray, (Lt) => [xA - plateT / 2 - Lt, xA - plateT / 2], plateIds.aft]]) {
    const r = af.trays?.[nm === 'forward tray' ? 'forward' : 'aft'];
    if (!r || af.carriers) continue; // round 15: the carriers are their own parts (carrierParts)
    const y = r.y_m, w = mm(T.w_mm) / 2, t = mm(T.t_mm) / 2, h = mm(T.flange_h_mm);
    const id = add(nm, [S('base', xs(r.length_m), cap([-w, y], [w, y], t)), S('flange right', xs(r.length_m), cap([w, y], [w, y + h], t)), S('flange left', xs(r.length_m), cap([-w, y], [-w, y + h], t))]);
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
 * the forward tray's base and its tab forward under the nose tray.
 */
export function riserSolids(r) {
  const [x0, x1] = r.x, [y0, y1] = r.y, hw = r.w / 2;
  return [
    S('web', [x0, x1], boxYZ(-hw, hw, y0, y1)),
    S('flange right', [x1, x1 + r.f], boxYZ(hw - r.t, hw, y0, y1 - r.t)),
    S('flange left', [x1, x1 + r.f], boxYZ(-hw, -hw + r.t, y0, y1 - r.t)),
    S('foot', [x0 - r.foot, x0], boxYZ(-hw, hw, y0, y0 + r.t)),
    S('tab', [x1, x1 + r.foot], boxYZ(-hw, hw, y1 - r.t, y1)),
  ];
}

/** Round 15 carriers (structure.airframe.parts.carriers): forward tray, riser, nose tray, aft tray, each screwed to its parent. */
function carrierParts({ af, add, join, plateIds, assumptions }) {
  const G = af.carriers;
  const f = add('forward tray', [...channelSolids(G.forward, G.forward.x), rootTab(G.forward, [G.forward.x[0], G.forward.x[0] + G.forward.t])]);
  join(f, plateIds.fwd, 'its root tab screwed to the plate');
  const r = add('nose tray riser', riserSolids(G.riser));
  join(r, f, 'its foot screwed to the tray\'s base');
  const n = add('nose tray', channelSolids(G.nose, G.nose.x));
  join(n, r, 'screwed to the riser\'s tab');
  const a = add('aft tray', [...channelSolids(G.aft, G.aft.x), rootTab(G.aft, [G.aft.x[1] - G.aft.t, G.aft.x[1]])]);
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
const carrierOf = (af, name) => ({ 'forward tray': af?.carriers?.forward, 'nose tray': af?.carriers?.nose, 'aft tray': af?.carriers?.aft })[name];

/** A mounted part's ties (round 15): through slots in its carrier either side of it, round its box and the carrier's sheet. */
function partTies(add, join, id, box, c, tr) {
  const [b, tt] = tr.band_mm.map((v) => v / 1000), face = c.y + c.t / 2;
  const sol = tieXs(box.x0, box.x1, tr.n).flatMap((x, k) => bandSlabs(`tie ${k + 1}`, x, b, tt, [box.z0, box.z1], [face - c.t, box.y1], [face, face]));
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
    const id = add(`packed ${p.name}`, [S('box', [b.x0, b.x1], boxYZ(b.z0, b.z1, b.y0, b.y1))]);
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
  const id = add('pitot-static probe', [S('tube', pm.x_m, ring(pm.c_m, pm.r_m))]);
  join(id, 'nose tray', `against the ${s > 0 ? 'right' : 'left'} flange's outer face, tied`);
  const zin = s * (N.w / 2 - N.t / 2), zout = pm.c_m[0] + s * pm.r_m, yb = N.y - N.t / 2, yt = Math.max(N.y + N.h + N.t / 2, pm.c_m[1] + pm.r_m);
  const [b, tt] = pm.band_mm.map((v) => v / 1000), top = N.y + N.t / 2;
  const zs = s > 0 ? [zin, zout] : [zout, zin], floors = s > 0 ? [top, yb] : [yb, top];
  const tid = add('pitot-static probe ties', pm.tie_x_mm.flatMap((x, k) => bandSlabs(`tie ${k + 1}`, x / 1000, b, tt, zs, [yb, yt], floors)));
  join(tid, id, 'round the probe');
  join(tid, 'nose tray', 'round the flange, through a slot at its root');
  assumptions.push('pitot-static probe (round 15): the titanium tube as a solid rod along the nose tray\'s flange, its bottom flush with the tray\'s base; its ties round the probe and the flange');
}

/** Points on a cross-section's outline (every point the loft gap can be worst at: a convex section's vertices, n on each circle). */
function outlinePoints(sh, n = 96) {
  const circ = (c, r) => Array.from({ length: n }, (_, k) => [c[0] + r * Math.cos((k / n) * TWO_PI), c[1] + r * Math.sin((k / n) * TWO_PI)]);
  if (sh.kind === 'ring') return circ(sh.c, sh.ro);
  if (sh.kind === 'cap') return [...circ(sh.a, sh.r), ...circ(sh.b, sh.r)];
  return sh.pts;
}

/** Whether point p lies in a cross-section. */
function inShape(sh, p) {
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
function xSamples(loft, [x0, x1]) {
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
  const outside = new Map((decl.outside ?? []).map((o) => [o.part, { part: o.part, why: o.why, min_clear_m: Infinity }]));
  const exposed = new Set();
  for (const part of asm.parts) for (const sol of part.solids) {
    const shapes = sol.at ? poses.map((q) => sol.at(q)) : [sol.shape];
    const xs = xSamples(loft, sol.x);
    const op = declaredOpening(decl.openings, part.id, sol.name), out = !op && outside.get(part.id);
    const g = solidLoftGaps(loft, shapes, xs, required, op ? openings.get(op.name) : null);
    const id = `${part.id}/${sol.name}`;
    if (op) openings.get(op.name).solids.push(id);
    else if (out) out.min_clear_m = Math.min(out.min_clear_m, g.clearOut);
    else { rows.push({ solid: id, gap_m: g.worst }); continue; }
    if ((out || op.drag !== false) && g.worst < 0) for (const sh of shapes) markExposed(exposed, loft, sh, xs, cell);
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
  const enclosed = rows.filter((r) => r.gap_m < required - 1e-9);
  const outs = [...outside.values()], outBad = outs.filter((o) => !(o.min_clear_m >= required - 1e-9));
  const area = exposed.size * cell * cell;
  const r3 = (v) => +(v * 1000).toFixed(3);
  const crossing = (o) => (Number.isFinite(o.x[0]) ? { x_mm: o.x.map(r3), polar_angle_deg: o.angle.map((a) => +a.toFixed(1)), max_beyond_skin_mm: r3(o.max_out_m), ...(o.past_loft ? { reaches_past_the_loft: true } : {}) } : null);
  return {
    rule: 'SPEC Support and containment (2026-11-03): every part meant to be inside the pod keeps 0.5 mm inside the loft (model.js\'s elliptic measure, which understates the true distance); a part may cross the skin only at a declared opening; a fitting left outside keeps 0.5 mm clear of the skin and its frontal area counts in the pod\'s drag',
    required_mm: required * 1000,
    sampling: { crank_phases_per_amplitude: phases, amplitudes_rad: amps, points: 'section outlines (96 per circle), x at the ends, 7 between and every loft section plane' },
    solids_enclosed: rows.length,
    closest_enclosed: rows.slice(0, 12).map((r) => ({ solid: r.solid, gap_mm: r3(r.gap_m) })),
    violations: enclosed.map((r) => ({ solid: r.solid, gap_mm: r3(r.gap_m) })),
    openings: [...openings.values()].map((o) => ({ name: o.name, why: o.why, counts_in_drag: o.drag, solids: o.solids, crossing: crossing(o) })),
    outside: outs.map((o) => ({ part: o.part, why: o.why, clear_of_skin_mm: r3(o.min_clear_m) })),
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
  const tx = tr ? tieXs(b.x0, b.x1, tr.n) : [];
  const cg = b.cx >= cx[0] && b.cx <= cx[1] && b.cz >= cz[0] && b.cz <= cz[1];
  const tiesOn = tx.every((x) => x >= c.x[0] && x <= c.x[1]);
  return { part: p.name, carrier: p.mount.carrier, face: 'base, top face', fixing: tr ? `${tr.n} x ${tr.tie} (${tr.loop_mm} mm loop)` : null, gap_mm: r4(gap), contact_x_mm: cx.map(r4), contact_z_mm: cz.map(r4), cg_over_contact: cg, ties_x_mm: tx.map(r4), passed: ok(gap) && cx[1] > cx[0] && cz[1] > cz[0] && cg && !!tr && tiesOn };
}

/** The crank encoder: the actuator in its bore on the crank stub, the readhead on the riser through its spacers. */
function encoderSupport(e, d, af, ok) {
  const ag = e.id.actuator / 2 - mm(d.params.linkage.crank.shaft_od_mm) / 2, fx = af?.support?.readhead;
  const rows = [{ part: 'crank encoder actuator', carrier: 'crank', face: 'shaft stub (its bore)', fixing: '3 x M3 DIN 913 set screws (the maker\'s, included)', gap_mm: r4(ag), passed: ok(ag) }];
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
  packedParts((id, solids) => { asm.parts.push({ id, solids }); return id; }, (a, b, w) => asm.joins.push([a, b, w]), d, catalog, d.structure.airframe.parts);
  return containmentRecord(asm, d, lay, decl, opts);
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

export const MESH = 'gears in mesh';

/**
 * Every pair of solids of two different parts. Round 14 (the brief's check of the check: fastened faces may touch, at
 * zero gap, but must never interpenetrate): a pair whose parts are joined is checked too, against zero instead of the
 * running clearance; only solids of one part and gears in mesh (their teeth interleave) are exempt.
 * -> { solids, pairs, nExempt }; a pair carries `required` 0 and its `join` when its parts are joined.
 */
function solidPairs(asm, required) {
  const joined = new Map(asm.joins.map(([a, b, why]) => [keyOf(a, b), why]));
  const solids = asm.parts.flatMap((p) => p.solids.map((s) => ({ ...s, part: p.id, id: `${p.id}/${s.name}` })));
  const joinOf = (a, b) => joined.get(keyOf(a.part, b.part)) ?? joined.get(keyOf(a.id, b.part)) ?? joined.get(keyOf(a.part, b.id)) ?? joined.get(keyOf(a.id, b.id));
  const pairs = [];
  let nExempt = 0;
  for (let i = 0; i < solids.length; i++) for (let k = i + 1; k < solids.length; k++) {
    const a = solids[i], b = solids[k], j = a.part === b.part ? MESH : joinOf(a, b);
    if (j === MESH) nExempt++;
    else pairs.push({ a, b, min: Infinity, at: null, required: j ? 0 : required, ...(j ? { join: j } : {}) });
  }
  return { solids, pairs, nExempt };
}

/** A pair under its requirement (the running clearance, or zero for joined parts, 0.1 um of rounding allowed). */
const under = (p) => p.min < p.required - (p.join ? 1e-7 : 1e-9);

const shapeAt = (s, q) => (s.at ? s.at(q) : s.shape);
const evalPair = (p, q) => gap3(p.a.x, p.b.x, gap2(shapeAt(p.a, q), shapeAt(p.b, q)));

/** Each moving pair's smallest gap over the sampled phases, then refined between its neighbouring samples. */
function sampleMoving(moving, amps, phases, poseAt, q0) {
  for (const amp of amps) {
    for (let i = 0; i < phases; i++) {
      const q = poseAt(amp, (i / phases) * TWO_PI);
      for (const p of moving) {
        const g = evalPair(p, q);
        if (g < p.min) { p.min = g; p.at = { amp, i }; }
      }
    }
  }
  const h = TWO_PI / phases;
  for (const p of moving) {
    p.class = pairClass(p.a, p.b, { shape: shapeAt(p.a, q0) }, { shape: shapeAt(p.b, q0) });
    p.phase = (p.at.i / phases) * TWO_PI;
    if (!(p.min < 0.01)) continue;
    const best = goldenMin((ph) => evalPair(p, poseAt(p.at.amp, ph)), (p.at.i - 1) * h, (p.at.i + 1) * h, 30);
    if (best[0] < p.min) { p.min = best[0]; p.phase = ((best[1] % TWO_PI) + TWO_PI) % TWO_PI; }
  }
}

/** A pair as the record prints it. */
const pairRow = (p) => ({ a: p.a.id, b: p.b.id, gap_mm: +(p.min * 1000).toFixed(3), class: p.class, ...(p.join ? { joined: p.join, required_mm: 0 } : {}), ...(p.a.at || p.b.at ? { phase_deg: +((p.phase * 180) / Math.PI).toFixed(1), amp_rad: p.at?.amp ?? null } : {}) });

/** One row per pair of parts under the required gap (its closest solids), closest first. */
function partPairClashes(pairs) {
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
function podRecord(pod, required) {
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
  const add = (id, solids) => { asm.parts.push({ id, solids }); return id; };
  const pk = pack ? packedParts(add, (a, b, w) => asm.joins.push([a, b, w]), d, catalog, airframe?.parts) : { pod: [], model: null };
  const pod = pk.pod;
  const { solids, pairs, nExempt } = solidPairs(asm, required);
  const amps = [...new Set([d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite))];
  const poseAt = (amp, ph) => drivePose({ law: lay.law, fourBar: lay.fb }, ph, amp);
  const q0 = poseAt(amps[0], 0);
  for (const p of pairs) if (!p.a.at && !p.b.at) { p.min = evalPair(p, q0); p.class = pairClass(p.a, p.b, p.a, p.b); }
  sampleMoving(pairs.filter((p) => p.a.at || p.b.at), amps, phases, poseAt, q0);
  const clashes = partPairClashes(pairs);
  const podR = podRecord(pod, required);
  // round 15 (SPEC "Support and containment"): for a design that states its mounts and openings
  const decl = d.params.layout?.containment;
  const containment = decl ? containmentRecord(asm, d, lay, decl, { required }) : null;
  const support = d.params.layout?.mounts && pk.model ? supportRecord(pk.model, d, airframe?.parts, lay) : null;
  return {
    rule: 'SPEC Design freeze, Assembly clearance (2026-11-03): at least 0.5 mm between every pair of parts that does not share a joint, through the full stroke; at least 0.5 mm running clearance between parts stacked on one pin (no catalogue thrust washer is credited); parts joined directly may touch but never interpenetrate (round 14), gears in mesh excepted',
    required_mm: required * 1000,
    method: 'parts as prisms along body x (rings, capsules, convex polygons with holes in the (z, y) plane); gap sqrt(gx^2 + g2^2), overlap as the smaller penetration; src/design/clearance.js',
    sampling: { crank_phases_per_amplitude: phases, amplitudes_rad: amps, refine: 'golden-section search between the neighbouring samples for every moving pair under 10 mm' },
    parts: asm.parts.length, solids: solids.length, pairs_checked: pairs.length, pairs_joined_checked_at_zero: pairs.filter((p) => p.join).length, pairs_exempt: nExempt, solid_pairs_clashing: pairs.filter(under).length,
    joints: asm.joins.map(([a, b, why]) => ({ a, b, why })),
    min_by_class: minByClass(pairs),
    clashes, // per pair of parts, the closest solids
    pod_shell: podR,
    ...(support ? { support } : {}),
    ...(containment ? { containment } : {}),
    assumptions: asm.assumptions,
    passed: clashes.length === 0 && podR.clashes.length === 0 && (support?.passed ?? true) && (containment?.passed ?? true),
    // every solid pair (for tests and probes; not written to the design file)
    ...(all ? { all_pairs: Object.fromEntries(pairs.map((p) => [`${p.a.id} | ${p.b.id}`, pairRow(p)])) } : {}),
  };
}
