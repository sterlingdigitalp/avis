// The hand wing's twist drive (round 17b, SPEC "Function completeness"; audit-v24.md items 1, 3, 10, 12 and interfaces
// 14-20). The flight model twists each hand wing about the spar axis by w(s) theta, w rising linearly from 0 at the
// elbow to 1 at the tip (kinematics.js twistWeightAt). The drive that delivers it:
//  - The two spars are coaxial, raised sparN above the chord line so that they and every round part on them sit inside
//    the real E387 (section.js). A machined 7075 joiner bonds the inner spar to the outer spar; between them, in an
//    open-topped bay, a 7075 lever is cross-pinned on the inboard end of the titanium torque rod, which runs in a GFI-0405
//    bushing in the joiner's wall, through the outer spar's bore, and out of its tip end through a second GFI-0405 in a
//    7075 end cap bonded on the spar. The MD89MW hangs under a strip machined on the joiner, tied with two T18R; its
//    R-ML25 drives the lever through two KBRM-02 rod ends and a 6061 link (a parallel four-bar, ratio r_servo / r_lever).
//  - On the rod's tip end a 7075 hub (cross-pinned) carries a Rock West 6 x 5 arm out to a 7075 tip plate beside rib 10;
//    rib 10 turns with them. A straight Rock West 6 x 5 bar runs from a round hole in a plate machined on the joiner (at
//    the elbow, fixed) to a round hole in the tip plate, e behind the spar axis. Ribs 6 to 9 turn on GFM-0810-03
//    bushings on the outer spar, held spanwise by bonded rings, and ride on the bar in chordwise slots, so each turns by
//    phi = atan(xi sin th / (1 - xi (1 - cos th))) (the bar's polar angle at xi of its length) where the flight model
//    has xi th: exact at xi 0, 1/2 and 1. The thin plates' holes let the bar tilt and roll; it slides through the
//    joiner's plate as it lengthens and is held spanwise at the tip plate.
//  - Frame: the wing's (s along the spar socket's axis from the shoulder axis, X = body x, n normal to the wing from the
//    spar axis toward the upper surface), drawn at zero twist; a solid is a prism along x with an (s, n) section.
import { poly, cap, M, RIB_HOLE, BORE, THREAD, THROUGH, xSlabs } from './clearance.js';
import { byId, tubeSpec, tubeAllowable } from './catalog.js';
import { e387Surfaces, sectionNAt, ribSlices } from './section.js';
import { servoArmMaker } from './chains.js'; // round 18: the maker's-part condition
import { MAKERS_PART_RULE } from './interfaces.js'; // round 18
import { tappedThreadMember } from './tail-mech.js'; // round 18: the lever's tapped thread, rated as the horn's

const mm = (v) => v / 1000;
const r6 = (v) => +v.toFixed(6);
const span = (wing) => wing.segments.reduce((a, g) => a + g.length, 0);

/** Default drive parameters (mm unless stated); params.twistDrive overrides them. */
export const TWIST_DRIVE = {
  sparN_mm: 5.5, material: 'metal-al-7075-t6', linkMat: 'metal-al-6061-t6',
  rodBushing: 'bushing-igus-GFI-0405-04', ribBushing: 'bushing-igus-GFM-0810-03', bar: 'tube-rockwest-pultruded-6x5', arm: 'tube-rockwest-pultruded-6x5',
  rodEnd: 'rodend-igus-KBRM-02', horn: 'horn-hitec-r-ml25', screw: 'screw-iso4762-m2x6-8.8', tie: 'tie-hellermanntyton-t18r', pinWasher: 'washer-din6799-1-5',
  e_mm: 120, clear_mm: 0.5,
  joiner: { od_mm: 12.4, outer_od_mm: 10.4, wall_od_mm: 14, inner_mm: 15, outer_mm: 20, u_t_mm: 0.8 },
  lever: { hub_od_mm: 9, hub_mm: 6, arm_t_mm: 2.5, arm_w_mm: 4, r_mm: 12.8, pin_d_mm: 2 },
  servo: { spline_x_mm: -30, spline_n_mm: 2, hole_mm: 16, travel_deg: 30 },
  hanger: { t_mm: 1.2, lip_mm: 1.0, width_mm: 6, pin_d_mm: 2, gap_mm: 0 },
  plate: { t_mm: 1.2, depth_mm: 5, hole_clear_mm: 0.4 },
  cap: { sleeve_mm: 6, od_mm: 10 },
  tip: { hub_od_mm: 9, hub_mm: 6, socket_mm: 8, socket_od_mm: 8, plate_t_mm: 1.0 },
  ring: { t_mm: 1.0, wall_mm: 0.75 },
  link_d_mm: 4,
};
export const twistParams = (params) => (params?.twistDrive ? mergeP(TWIST_DRIVE, params.twistDrive === true ? {} : params.twistDrive) : null);
function mergeP(a, b) { const o = { ...a }; for (const [k, v] of Object.entries(b ?? {})) o[k] = v && typeof v === 'object' && !Array.isArray(v) ? { ...(a[k] ?? {}), ...v } : v; return o; }

// ---- the section ----------------------------------------------------------------------------------------------------

/** A point of the section's mean line at body x (m), span s, in the spar-axis frame (n from the spar axis). */
export function meanLineN(wing, s, x, surf, sparN) {
  const q = sectionNAt(wing, s, x, surf, { sparN });
  return (q.lo + q.hi) / 2;
}

/** The edge rods' centres at span s: the leading edge rod inside the nose, the trailing edge rod at the trailing edge. */
export function edgeRodPoints(wing, s, rLe, rTe, surf, sparN) {
  const { c, xle } = chordAtW(wing, s);
  const xL = xle - rLe, xT = xle - c + rTe;
  return { le: { x: xL, n: meanLineN(wing, s, xL, surf, sparN) }, te: { x: xT, n: meanLineN(wing, s, xT, surf, sparN) } };
}
function chordAtW(wing, s) {
  let s0 = 0;
  for (const g of wing.segments) {
    if (s <= s0 + g.length + 1e-12) { const t = Math.min(1, Math.max(0, (s - s0) / g.length)); return { c: g.chord_in + t * (g.chord_out - g.chord_in), xle: g.xle_in + t * (g.xle_out - g.xle_in) }; }
    s0 += g.length;
  }
  const g = wing.segments.at(-1);
  return { c: g.chord_out, xle: g.xle_out };
}

/**
 * A rib as chordwise pieces following the E387 (the film request: like the rod bays, not an envelope rectangle): a
 * built-up rib, its caps `cap_mm` deep along both surfaces, full depth where the section is thinner than two caps, at
 * the nose and tail blocks round the edge rods, round the spar (`hubs`: x ranges that must be full depth) and in webs
 * every `web_pitch_mm`. Each piece { x: [a, b], n: [lo, hi] } is inside the real section (ribSlices takes the innermost
 * surfaces over it). -> { pieces, x: [x0, x1], area_m2 }.
 */
export function ribPieces(wing, s, rLe, rTe, surf, sparN, { hubs = [], holes = [], cap_mm = 2.5, web_mm = 4, web_pitch_mm = 60, block_mm = 6, slice_mm = 30, t_mm = 2 } = {}) {
  // the rib's two faces (s +- t/2): the piece inside the section at both, the x range the smaller of the two
  const tf = mm(t_mm) / 2, A = chordAtW(wing, s - tf), B = chordAtW(wing, s + tf);
  const x0 = Math.max(A.xle - A.c, B.xle - B.c) + 2 * rTe, x1 = Math.min(A.xle, B.xle) - 2 * rLe, capH = mm(cap_mm);
  // the full-depth intervals: the tail and nose blocks, every hub, and webs at the pitch between them
  const full = [[x0, x0 + mm(block_mm)], [x1 - mm(block_mm), x1], ...hubs.map(([a, b]) => [Math.max(x0, a), Math.min(x1, b)])];
  for (let x = x0 + mm(web_pitch_mm); x < x1 - mm(web_pitch_mm) / 2; x += mm(web_pitch_mm)) full.push([x - mm(web_mm) / 2, x + mm(web_mm) / 2]);
  const merged = mergeIntervals(full.filter(([a, b]) => b > a));
  const cuts = [...new Set([x0, x1, ...merged.flat()])].filter((x) => x >= x0 && x <= x1).sort((a, b) => a - b);
  const pieces = [];
  for (let i = 0; i + 1 < cuts.length; i++) {
    const a = cuts[i], b = cuts[i + 1];
    if (b - a < 1e-6) continue;
    const isFull = merged.some(([p, q]) => a >= p - 1e-9 && b <= q + 1e-9);
    const n = Math.max(1, Math.ceil((b - a) / mm(slice_mm)));
    const sA = ribSlices(wing, s - tf, [a, b], surf, { slices: n, sparN }), sB = ribSlices(wing, s + tf, [a, b], surf, { slices: n, sparN });
    for (const [i, sl] of sA.entries()) {
      const lo = Math.max(sl.n[0], sB[i].n[0]), hi = Math.min(sl.n[1], sB[i].n[1]);
      if (hi - lo <= 1e-5) continue;
      if (isFull || hi - lo <= 2 * capH) pieces.push({ x: sl.x, n: [lo, hi], kind: 'full' });
      else pieces.push({ x: sl.x, n: [lo, lo + capH], kind: 'lower cap' }, { x: sl.x, n: [hi - capH, hi], kind: 'upper cap' });
    }
  }
  const cut = holes.reduce((list, h) => list.flatMap((p) => cutHole(p, h)), pieces);
  const area_m2 = cut.reduce((A, p) => A + Math.abs(p.x[1] - p.x[0]) * (p.n[1] - p.n[0]), 0);
  return { pieces: cut, x: [x0, x1], area_m2 };
}
/** A piece less a rectangular hole { x, n } (the spar's, a bushing's or the bar's way through the rib). */
function cutHole(p, h) {
  const [a, b] = p.x, [hx0, hx1] = h.x;
  if (hx1 <= a || hx0 >= b || h.n[1] <= p.n[0] || h.n[0] >= p.n[1]) return [p];
  const out = [];
  if (hx0 > a) out.push({ ...p, x: [a, hx0] });
  if (hx1 < b) out.push({ ...p, x: [hx1, b] });
  const xm = [Math.max(a, hx0), Math.min(b, hx1)];
  if (h.n[0] > p.n[0] + 1e-6) out.push({ ...p, x: xm, n: [p.n[0], h.n[0]] });
  if (h.n[1] < p.n[1] - 1e-6) out.push({ ...p, x: xm, n: [h.n[1], p.n[1]] });
  return out;
}
function mergeIntervals(list) {
  const q = [...list].sort((a, b) => a[0] - b[0]), out = [];
  for (const [a, b] of q) { if (out.length && a <= out.at(-1)[1] + 1e-9) out.at(-1)[1] = Math.max(out.at(-1)[1], b); else out.push([a, b]); }
  return out;
}

// ---- the geometry ---------------------------------------------------------------------------------------------------

const tubeOf = (catalog, id, specSheet) => { const t = tubeSpec(byId(catalog, id), specSheet); return { ro: mm(t.od_draw) / 2, ri: mm(t.id_draw) / 2, od_rate: t.od_rate, id_rate: t.id_rate, g_per_m: t.g_per_m }; };

/**
 * The drive's geometry for one wing (m, the wing frame), shared by the solids, the BOM, the members and the chain
 * record. wing: design.wing; parts: params.parts; ribs: the rib stations; P: twistParams(params).
 */
export function twistGeometry(wing, parts, ribs, P, catalog, { rib_t_mm = 2, specSheet = true } = {}) {
  const L = span(wing), elbow = wing.segments[0].length, clr = mm(P.clear_mm), tR = mm(rib_t_mm);
  const si = tubeOf(catalog, parts.sparInner, specSheet), so = tubeOf(catalog, parts.sparOuter, specSheet), rod = tubeOf(catalog, parts.twistRod, specSheet);
  const bar = tubeOf(catalog, P.bar, specSheet), arm = tubeOf(catalog, P.arm, specSheet);
  const gfi = byId(catalog, P.rodBushing).specs, gfm = byId(catalog, P.ribBushing).specs, ke = byId(catalog, P.rodEnd).specs, hn = byId(catalog, P.horn).specs;
  const J = P.joiner, Lv = P.lever, SV = P.servo, T = P.tip;
  const k5 = ribs.findIndex((s) => s > elbow) - 1, s5 = ribs[k5];
  // the joiner from rib 5's face: the inner sleeve, the bay (the lever and its rod end), the wall with the elbow bushing
  // (its flange outboard, in a counterbore, against the outer spar's end), the outer sleeve
  const jA = s5 + tR / 2, jB0 = jA + mm(J.inner_mm); // the inner sleeve's face on rib 5 (its outboard stop)
  // in the bay: the lever's rod end (beside the inner sleeve's end), its arm at the hub's inboard end, the hub's outboard
  // face on the elbow bushing's body (the rod's outboard stop)
  const eyeS = [jB0 + clr, jB0 + clr + mm(ke.B_mm)], sL = (eyeS[0] + eyeS[1]) / 2, h0 = eyeS[1], h1 = h0 + mm(Lv.hub_mm), armS = [h0, h0 + mm(Lv.arm_t_mm)];
  const pinS = (h0 + mm(Lv.arm_t_mm) + h1) / 2;
  const jW0 = h1, gL = mm(gfi.b1_mm - gfi.b2_mm), jW1 = jW0 + mm(gfi.b1_mm), jO1 = jW1 + mm(J.outer_mm);
  // the lever and the servo's arm: a parallel four-bar (both arms along u, S.u = -(r_s - r_l)), the link between them
  const rl = mm(Lv.r_mm), rs = mm(SV.hole_mm), S = [mm(SV.spline_x_mm), mm(SV.spline_n_mm)];
  const alpha = solveAlpha(S, rs - rl), u = [Math.cos(alpha), Math.sin(alpha)];
  const Lp = [rl * u[0], rl * u[1]], Hp = [S[0] + rs * u[0], S[1] + rs * u[1]];
  const sCase = sL + mm(ke.B_mm) / 2 + mm(hn.hub_overall_height_mm), caseDims = byId(catalog, parts.twistServo).specs.dimensions_mm.map(mm);
  // the elbow plate (the bar's fixed hole) machined on the bay's aft bar at the inner sleeve's end, under the link
  const e = mm(P.e_mm), sE = jB0 + mm(P.plate.t_mm) / 2;
  // the outer spar's tip end: rib 9 on its bushing between two rings, the end cap, the tip bushing, the tip hub
  const k9 = ribs.length - 2, s9 = ribs[k9], s10 = ribs[k9 + 1], ringT = mm(P.ring.t_mm);
  const sOe = s9 + tR / 2 + mm(gfm.b2_mm) + ringT + clr + mm(P.cap.sleeve_mm), capW1 = sOe + gL, capF1 = sOe + mm(gfi.b1_mm);
  const sH0 = capF1, sH1 = sH0 + mm(T.hub_mm), rodEnd = sH1;
  // the tip plate at the tip, rib 10 against its inboard face (moved in from its round-16 station by the plate), the
  // arm from the hub through rib 10 to the plate
  const sTp1 = L, sTp0 = sTp1 - mm(T.plate_t_mm), sT = (sTp0 + sTp1) / 2, s10t = sTp0 - tR / 2;
  const armA = { s: sH1 + mm(5.5), x: -mm(6.5), n: 0 }, armB = { s: sTp0, x: -e + bar.ro + clr + arm.ro + mm(1), n: 0 };
  // the bar: from inboard of the elbow plate (room for its slide through it) into rib 10 (its end inside the rib)
  const barS0 = sE - mm(10), barS1 = sTp1 - clr;
  return {
    L, elbow, si, so, rod, bar, arm, gfi, gfm, ke, hn, clr, tR, ribs, k5, k9, s5, s9, s10, alpha, u, Lp, Hp, S, rl, rs,
    jA, jB0, h0, h1, armS, eyeS, sL, pinS, jW0, jW1, jWb: jW0 + gL, jO1, sCase, caseDims, sE, e, nBar: 0, sOe, capW1, capF1, sH0, sH1, rodEnd, sTp0, sTp1, sT, s10t, armA, armB, barS0, barS1,
    rodS0: h0, outerS0: jW1, innerS1: jB0, sparN: mm(P.sparN_mm),
  };
}

/** The four-bar's arm direction: S.u = -d for u = (cos a, sin a), the root nearest straight up. */
function solveAlpha(S, d) {
  let best = null;
  for (let a = 0.2; a < Math.PI - 0.2; a += 1e-4) { const f = S[0] * Math.cos(a) + S[1] * Math.sin(a) + d; if (best === null || Math.abs(f) < Math.abs(best.f)) best = { a, f }; }
  return best.a;
}

/**
 * The delivered twist law and four-bar: phi(xi, th) at a rib xi of the bar's length for the rod's angle th, and the
 * rod's angle for a servo angle (the four-bar solved exactly). -> { law(xi, th), rodAt(ths), ratio(ths) }.
 */
export function twistKinematics(G) {
  const law = (xi, th) => Math.atan2(xi * Math.sin(th), 1 - xi * (1 - Math.cos(th)));
  const ground = Math.hypot(G.S[0], G.S[1]), link = Math.hypot(G.Hp[0] - G.Lp[0], G.Hp[1] - G.Lp[1]);
  // servo arm angle alpha + ths about S; the lever's tip where the circle round the rod's axis (rl) meets the circle
  // round the servo arm's hole (the link), on the open four-bar's branch (the root nearer the servo's own angle)
  const rodAt = (ths) => {
    const as = G.alpha + ths, H = [G.S[0] + G.rs * Math.cos(as), G.S[1] + G.rs * Math.sin(as)], d = Math.hypot(H[0], H[1]);
    const a = (G.rl * G.rl - link * link + d * d) / (2 * d), h = Math.sqrt(Math.max(0, G.rl * G.rl - a * a));
    const roots = [1, -1].map((sg) => Math.atan2((a * H[1]) / d + (sg * h * H[0]) / d, (a * H[0]) / d - (sg * h * H[1]) / d));
    const near = (r) => Math.abs(Math.atan2(Math.sin(r - as), Math.cos(r - as)));
    return (near(roots[0]) < near(roots[1]) ? roots[0] : roots[1]) - G.alpha;
  };
  const ratio = (ths, h = 1e-3) => (rodAt(ths + h) - rodAt(ths - h)) / (2 * h);
  return { law, rodAt, ratio, ground, link };
}

// ---- the solids -----------------------------------------------------------------------------------------------------

const at = (f, s, n) => [f.o[0] + s * f.u[0] + n * f.v[0], f.o[1] + s * f.u[1] + n * f.v[1]];
/** The rectangle s in [s0, s1], n in [n0, n1] of a wing frame as a (z, y) polygon. */
export const boxSN = (f, [s0, s1], [n0, n1]) => poly([at(f, s0, n0), at(f, s1, n0), at(f, s1, n1), at(f, s0, n1)]);
const mbox = (frame, name, xr, sr, nr, extra = {}) => ({ ...M(name, [Math.min(...xr), Math.max(...xr)], (q) => boxSN(frame(q), sr, nr)), ...extra });
/** A short tube along the span at (x0, n0): the box round it (contains it), its own tube for the volume. */
const mtube = (frame, name, x0, n0, sr, ro, ri = 0) => mbox(frame, name, [x0 - ro, x0 + ro], sr, [n0 - ro, n0 + ro], { tube: { ro, ri, L: sr[1] - sr[0], axis: 'span' }, cl: { s: sr, x: [x0, x0], n: [n0, n0], r: ro } });
/** A long tube along the span: its capsule. */
const mcap = (frame, name, x0, n0, [s0, s1], ro, ri = 0) => ({ ...M(name, [x0 - ro, x0 + ro], (q) => { const f = frame(q); return cap(at(f, s0 + ro, n0), at(f, s1 - ro, n0), ro); }), tube: { ro, ri, L: s1 - s0, axis: 'span' }, cl: { s: [s0, s1], x: [x0, x0], n: [n0, n0], r: ro } });
/** A round pin along x at (s, n): a ring over its x range. */
const mpinX = (frame, name, xr, s, n, r) => M(name, xr, (q) => ({ kind: 'ring', c: at(frame(q), s, n), ro: r, ri: 0 }));
/**
 * Round 18.2 (the film's request): a fastener crossing x (a screw head/shank on this axis), its true section: N
 * slabs across x (xSlabs), each a moving box narrowing in n, holding the full span sr throughout (so the true
 * length along the screw's own axis is never overshot). One shared tube record per fastener, not per slab.
 */
const mSlabsS = (frame, name, x0, n0, sr, ro, ri = 0) => xSlabs(x0, ro, (w) => (q) => boxSN(frame(q), sr, [n0 - w, n0 + w])).map((sl, i) => ({ ...M(`${name} ${i + 1}`, sl.x, sl.shape), tube: { ro, ri, L: Math.abs(sr[1] - sr[0]), axis: 'span' } }));
/** The same, for a cross pin whose length runs along n: xSlabs narrowing in s instead, holding the full nr throughout. */
const mSlabsN = (frame, name, x0, s0, nr, ro, ri = 0) => xSlabs(x0, ro, (w) => (q) => boxSN(frame(q), [s0 - w, s0 + w], nr)).map((sl, i) => ({ ...M(`${name} ${i + 1}`, sl.x, sl.shape), tube: { ro, ri, L: Math.abs(nr[1] - nr[0]), axis: 'n' } }));
const lerp3 = (A, B, t) => ({ s: A.s + t * (B.s - A.s), x: A.x + t * (B.x - A.x), n: A.n + t * (B.n - A.n) });

/**
 * A straight tube from A to B ({ s, x, n }) not along the span, as x-pieces of at most `piece`: each the capsule in
 * (s, n) between its ends' axis points with the radius of the tube's section across x (ro / |cos| of its angle to x),
 * the end pieces grown in x by the end faces' reach; each piece's own tube (its length along the axis) for the volume.
 */
export function diagTube(frame, name, A, B, ro, ri = 0, piece = 0.01) {
  const D = [B.x - A.x, B.s - A.s, B.n - A.n], Ln = Math.hypot(...D), cx = Math.abs(D[0]) / Ln;
  const k = Math.max(1, Math.ceil(Math.abs(D[0]) / piece)), rr = ro / Math.max(cx, 0.2), ext = ro * Math.sqrt(Math.max(0, 1 - cx * cx));
  return Array.from({ length: k }, (_, i) => {
    const P0 = lerp3(A, B, i / k), P1 = lerp3(A, B, (i + 1) / k), g0 = i === 0 ? ext : 0, g1 = i === k - 1 ? ext : 0;
    const lo = P0.x <= P1.x ? [P0.x - g0, P1.x + g1] : [P1.x - g1, P0.x + g0];
    return { ...M(k > 1 ? `${name} ${i + 1}` : name, lo, (q) => { const f = frame(q); return cap(at(f, P0.s, P0.n), at(f, P1.s, P1.n), rr); }), tube: { ro, ri, L: Ln / k, axis: 'diag' } };
  });
}

/** A rib's pieces as moving solids at span s (its thickness t), tagged with the ribs' line. */
export function ribSolids(frame, rp, s, t) {
  return rp.pieces.map((p, i) => mbox(frame, `${p.kind} ${i + 1}`, p.x, [s - t / 2, s + t / 2], p.n));
}

/** A rib's full-depth zones (round the spar or its bushing's flange, round the bar's slot) and its holes (the spar or
 * its bushing, the bar's slot or hole), x and n in the spar-axis frame. k: the rib's index. */
export function ribZones(G, k, travel_rad) {
  const onInner = k <= G.k5, onOuter = k > G.k5 && k <= G.k9, tip = k === G.ribs.length - 1, sl = G.bar.ro + mm(0.2);
  const rHole = onInner ? G.si.ro : onOuter ? mm(G.gfm.d2_mm) / 2 : 0, rHub = (onInner ? G.si.ro : mm(G.gfm.d3_mm) / 2) + mm(3);
  const hubs = tip ? [] : [[-rHub, rHub]], holes = rHole ? [{ x: [-rHole, rHole], n: [-rHole, rHole] }] : [];
  if (onOuter) {
    const rMin = G.e * Math.cos(travel_rad / 2), slot = [-G.e - sl, -rMin + sl];
    hubs.push([slot[0] - mm(3), slot[1] + mm(3)]); holes.push({ x: slot, n: [G.nBar - sl, G.nBar + sl] });
  }
  if (tip) { hubs.push([-G.e - sl - mm(3), -G.e + sl + mm(3)]); holes.push({ x: [-G.e - sl, -G.e + sl], n: [G.nBar - sl, G.nBar + sl] }); }
  return { hubs, holes };
}

/** The joins' reasons (the interface rules match them). */
export const TW = {
  innerSleeve: 'its inner sleeve bonded on the inner spar', outerSleeve: 'its outer sleeve bonded on the outer spar',
  seat: 'pressed in its seat', turns: 'the torque rod turns in it', flangeHeld: 'its flange held by the outer spar\'s end',
  pinned: 'cross-pinned on the torque rod (the pin bonded in the hub)', pinIn: 'its cross pin, through the torque rod', thrust: 'its face on the bushing (the torque rod\'s axial stop)',
  ball: 'its ball on the arm, on its screw', throughBall: 'through the ball', tapped: 'screwed into the arm\'s tapped hole', stud: 'its stud in the rod end\'s thread',
  spline: 'on the twist servo\'s output spline (the maker\'s arm and its centre screw)', frame: 'in its frame on the joiner (lips, strip and pins)', framePin: 'bonded in the frame\'s lips', pinFace: 'against its face',
  barHole: 'the bar through its hole (it tilts and rolls there)', barSlide: 'the bar through its hole (it tilts, rolls and slides there)', ringBar: 'bonded on the bar', stop: 'against its face (spanwise stop)',
  cap: 'bonded on the outer spar\'s tip end', socket: 'its end bonded in the socket', ribTip: 'against the tip plate, on the bar and the arm',
  onSpar: 'on the inner spar between its rings', inRib: 'pressed in the rib', slot: 'rides on the bar in its chordwise slot', spEnd: 'the spar\'s end against it',
};

/**
 * One wing's structure with the twist drive (params.twistDrive): the spars, edge rods and ribs in the real E387 (the
 * spars sparN above the chord line), and the drive. ax as wing-solids.js wingParts's; frame(q) -> { o, u, v }.
 */
export function twistWing(ax, tag, frame) {
  const { d, catalog } = ax, P = twistParams(d.params), W = d.params.wingStructure, parts = d.params.parts;
  const ctx = twistContext(d, catalog, P);
  const G = ctx.G, t = G.tR;
  const ids = drawSpars(ax, tag, frame, G, P, W);
  drawJoiner(ax, tag, frame, G, P, ids);
  drawServo(ax, tag, frame, G, P, ids, parts);
  drawBarAndTip(ax, tag, frame, G, P, ids);
  drawRibs(ax, tag, frame, G, P, ctx, ids);
  ax.assumptions.push(`wing ${tag === 'R' ? 'right' : 'left'} (round 17b, params.twistDrive, src/design/twist-drive.js): the spars coaxial ${P.sparN_mm} mm above the E387's chord line, the ribs as chordwise pieces of a built-up rib (caps, webs, full depth round the spar, the bar's slot and the edge rods) inside the real section, the edge rods on its leading and trailing edges; the hand wing's ribs 6-9 on ${P.ribBushing} on the outer spar between bonded rings, slotted on a ${P.bar} bar ${P.e_mm} mm behind the spar axis from a hole in the joiner's plate to the tip plate; rib 10 on the tip plate, moved in from its round-16 station by the plate (${(t * 1000).toFixed(0)} mm rib, ${P.tip.plate_t_mm} mm plate); drawn at zero twist; the MD89MW's spline position on its case is not published (drawn on its top face's centre), its travel is not published (the speed is published over 60 deg, so at least +-30 deg is used)`);
  return { G, ids };
}

/** The design's rib stations with rib 10 against the tip plate, the drive's geometry and the section. */
export function twistContext(d, catalog, P = twistParams(d.params)) {
  const W = d.params.wingStructure, L = span(d.wing), t = mm(W?.rib_t_mm ?? 2);
  const ribs = ribStationsOf(d, L, t, P);
  const G = twistGeometry(d.wing, d.params.parts, ribs, P, catalog, { rib_t_mm: W?.rib_t_mm ?? 2, specSheet: !!d.params.tubeSpecSheet });
  return { G, ribs, surf: e387Surfaces(catalog), travel: servoTravel(G, P) };
}
function ribStationsOf(d, L, t, P) {
  const W = d.params.wingStructure, total = +(d.bom.structure.find((s) => /^ribs/.test(s.name))?.name.match(/(\d+)\s*off/)?.[1] ?? 20), per = Math.max(2, Math.round(total / 2));
  const s0 = W.rib_root_m, s1 = L - t / 2;
  const ribs = Array.from({ length: per }, (_, k) => s0 + (k / (per - 1)) * (s1 - s0));
  ribs[per - 1] = L - mm(P.tip.plate_t_mm) - t / 2;
  return ribs;
}
/** The rod's travel either way at the servo's published travel (the four-bar solved), rad. */
export function servoTravel(G, P) {
  const K = twistKinematics(G), th = (P.servo.travel_deg * Math.PI) / 180;
  return Math.min(Math.abs(K.rodAt(th)), Math.abs(K.rodAt(-th)));
}

function drawSpars(ax, tag, frame, G, P, W) {
  const { d, add, join } = ax, sp0 = W.spar_from_m ?? d.params.linkage.rocker.hub_od_mm / 2000;
  const inner = add(`inner spar ${tag}`, [mcap(frame, 'tube', 0, 0, [sp0, G.innerS1], G.si.ro, G.si.ri)], { bom: 'inner spar' });
  join(inner, `rocker ${tag}/spar socket`, BORE);
  join(inner, `rocker ${tag}`, 'bonded in the spar socket');
  const outer = add(`outer spar ${tag}`, [mcap(frame, 'tube', 0, 0, [G.outerS0, G.sOe], G.so.ro, G.so.ri)], { bom: 'outer spar' });
  const rod = add(`twist torque rod ${tag}`, [mcap(frame, 'tube', 0, 0, [G.rodS0, G.rodEnd], G.rod.ro, G.rod.ri)], { bom: 'twist torque rods' });
  join(rod, outer, BORE);
  return { inner, outer, rod };
}

function drawJoiner(ax, tag, frame, G, P, ids) {
  const { add, join } = ax, J = P.joiner, ro = mm(J.od_mm) / 2, rw = mm(J.wall_od_mm) / 2, roO = mm(J.outer_od_mm) / 2, ut = mm(J.u_t_mm);
  const bay = [G.jB0, G.jW0], pl = P.plate, hg = P.hanger, cs = G.caseDims, Sx = G.S[0], Sn = G.S[1];
  const caseX = [Sx - cs[0] / 2, Sx + cs[0] / 2], caseN = [Sn - cs[1] / 2, Sn + cs[1] / 2], caseS = [G.sCase, G.sCase + cs[2]];
  const lip = mm(hg.lip_mm), rq = mm(hg.pin_d_mm) / 2, lipS = [caseS[0] - 2 * rq - mm(0.5), caseS[1] + 2 * rq + mm(0.5)], lipN = [caseN[0] - 2 * rq - mm(0.25), caseN[1] + mm(hg.t_mm)];
  // the strip over the case's middle and over the outer sleeve (its post)
  const sm = Math.max(Math.min((caseS[0] + caseS[1]) / 2, G.jO1 - mm(hg.width_mm) / 2 - mm(0.5)), G.jW1 + mm(hg.width_mm) / 2), stripS = [sm - mm(hg.width_mm) / 2, sm + mm(hg.width_mm) / 2];
  const sol = [
    mtube(frame, 'inner sleeve', 0, 0, [G.jA, G.jB0], ro, G.si.ro),
    mbox(frame, 'bay bottom', [-(ro - ut), ro - ut], bay, [-ro, -ro + ut]),
    mbox(frame, 'bay aft', [-ro, -ro + ut], bay, [-ro, mm(2.5)]),
    mbox(frame, 'bay forward', [ro - ut, ro], bay, [-ro, mm(2)]),
    mtube(frame, 'wall', 0, 0, [G.jW0 + G.clr, G.jWb], rw, mm(G.gfi.d2_mm) / 2),
    mtube(frame, 'counterbore', 0, 0, [G.jWb, G.jW1], rw, mm(G.gfi.d3_mm) / 2),
    mtube(frame, 'outer sleeve', 0, 0, [G.jW1, G.jO1], roO, G.so.ro),
    mbox(frame, 'plate', [-G.e - mm(pl.depth_mm) * 0.75, -ro], [G.sE - mm(pl.t_mm) / 2, G.sE + mm(pl.t_mm) / 2], [-mm(pl.depth_mm) / 2, mm(pl.depth_mm) / 2]),
    mbox(frame, 'hanger post', [-mm(3), mm(3)], stripS, [roO, caseN[1] + mm(hg.gap_mm)]),
    mbox(frame, 'hanger strip', [caseX[0] - lip, mm(3)], stripS, [caseN[1] + mm(hg.gap_mm), caseN[1] + mm(hg.gap_mm) + mm(hg.t_mm)]),
    mbox(frame, 'hanger lip forward', [caseX[1], caseX[1] + lip], lipS, lipN),
    mbox(frame, 'hanger lip aft', [caseX[0] - lip, caseX[0]], lipS, lipN),
  ];
  const j = add(`twist joiner ${tag}`, sol, { bom: 'twist joiners' });
  join(j, ids.inner, TW.innerSleeve); join(`${j}/inner sleeve`, ids.inner, BORE);
  join(j, ids.outer, TW.outerSleeve); join(`${j}/outer sleeve`, ids.outer, BORE);
  for (const nm of ['wall', 'counterbore', 'outer sleeve']) join(`${j}/${nm}`, ids.rod, BORE);
  // the elbow bushing in the wall, its flange in the counterbore against the outer spar's end
  const b = add(`twist elbow bushing ${tag}`, [mtube(frame, 'body', 0, 0, [G.jW0, G.jWb], mm(G.gfi.d2_mm) / 2, mm(G.gfi.d1_mm) / 2), mtube(frame, 'flange', 0, 0, [G.jWb, G.jW1], mm(G.gfi.d3_mm) / 2, mm(G.gfi.d1_mm) / 2)], { bom: P.rodBushing });
  join(b, j, TW.seat); join(`${b}/body`, `${j}/wall`, BORE); join(`${b}/flange`, `${j}/counterbore`, BORE);
  join(ids.rod, b, TW.turns); join(b, ids.outer, TW.flangeHeld); join(`${ids.rod}/tube`, `${b}/body`, BORE); join(`${ids.rod}/tube`, `${b}/flange`, BORE);
  Object.assign(ids, { joiner: j, elbowBushing: b, caseX, caseN, caseS, lipS, lipN });
}

function drawServo(ax, tag, frame, G, P, ids, parts) {
  const { catalog, add, join } = ax, Lv = P.lever, ke = G.ke, hn = G.hn, u = G.u, pr = [-u[1], u[0]];
  // the lever: its hub cross-pinned on the rod's inboard end, its arm along u at the hub's inboard end
  const armBox = (c0, r0, r1, w) => { const pts = [r0, r1].flatMap((r) => [-1, 1].map((k) => [c0[0] + r * u[0] + k * (w / 2) * pr[0], c0[1] + r * u[1] + k * (w / 2) * pr[1]])); return { x: [Math.min(...pts.map((p) => p[0])), Math.max(...pts.map((p) => p[0]))], n: [Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[1]))], len: r1 - r0 }; };
  const rH = mm(Lv.hub_od_mm) / 2, la = armBox([0, 0], rH, G.rl + mm(3), mm(Lv.arm_w_mm)), aT = G.armS[1] - G.armS[0];
  const lever = add(`twist elbow lever ${tag}`, [mtube(frame, 'hub', 0, 0, [G.h0, G.h1], rH, G.rod.ro), mbox(frame, 'arm', la.x, G.armS, la.n, { area_m2: (la.len * mm(Lv.arm_w_mm) * aT) / (la.x[1] - la.x[0]) })], { bom: 'twist levers' });
  join(`${lever}/hub`, ids.rod, BORE); join(lever, ids.rod, TW.pinned); join(lever, ids.elbowBushing, TW.thrust);
  const rp = mm(Lv.pin_d_mm) / 2;
  // round 18.2 (the film's request): the pin's true section, N slabs across x (mSlabsN) narrowing toward the
  // circle, not mbox's flat box; every slab (not just one) needs its own THROUGH join with the hub, since that
  // overrides the part-level TW.pinIn (not itself exempt) for exactly the pair that overlaps
  const pinSlabs = mSlabsN(frame, 'pin', 0, G.pinS, [-rH, rH], rp);
  const pin = add(`twist elbow pin ${tag}`, pinSlabs, { bom: 'twist cross pins' });
  join(pin, lever, TW.pinIn); for (const sl of pinSlabs) join(`${pin}/${sl.name}`, `${lever}/hub`, THROUGH);
  join(pin, ids.rod, THROUGH);
  // the servo's arm on its spline, the rod ends on the two arms, the link between their shanks
  const sv = byId(catalog, parts.twistServo), cs = G.caseDims;
  const ha = armBox(G.S, mm(hn.hub_od_mm) / 2, mm(hn.hole_radii_mm.at(-1)) + mm(hn.tip_od_mm) / 2, mm(hn.tip_od_mm));
  const hS = [G.sCase - mm(hn.hub_overall_height_mm), G.sCase], hA = [hS[0], hS[0] + mm(hn.arm_thickness_mm)];
  const horn = add(`twist servo arm ${tag}`, [mtube(frame, 'hub', G.S[0], G.S[1], hS, mm(hn.hub_od_mm) / 2), mbox(frame, 'arm', ha.x, hA, ha.n)], { bom: P.horn });
  const servo = add(`twist servo ${tag}`, [{ ...mbox(frame, 'case', ids.caseX, ids.caseS, ids.caseN), box: [cs[0], cs[2], cs[1]] }], { bom: sv.id });
  join(horn, servo, TW.spline); join(servo, ids.joiner, TW.frame);
  const dir = [G.Hp[0] - G.Lp[0], G.Hp[1] - G.Lp[1]], Ld = Math.hypot(...dir), du = [dir[0] / Ld, dir[1] / Ld], h1 = mm(ke.h1_mm);
  const pt = (p, k) => ({ s: G.sL, x: p[0] + k * du[0], n: p[1] + k * du[1] });
  const rodEnd = (nm, c, sg) => add(`twist rod end ${tag} ${nm}`, [mbox(frame, 'eye', [c[0] - mm(ke.d2_mm) / 2, c[0] + mm(ke.d2_mm) / 2], G.eyeS, [c[1] - mm(ke.d2_mm) / 2, c[1] + mm(ke.d2_mm) / 2]), ...diagTube(frame, 'shank', pt(c, sg * mm(ke.d2_mm) / 2), pt(c, sg * h1), mm(ke.d5_mm) / 2, 0, 0.02)], { bom: P.rodEnd });
  const eL = rodEnd('lever', G.Lp, 1), eS = rodEnd('servo', G.Hp, -1);
  const sg = screwGeom(catalog, P.screw);
  // round 18.2 (the film's request): the screw's true section, N slabs across x (mSlabsS) narrowing toward the
  // circle, not mtube's flat box. Every shank slab (not just one) needs its own THROUGH/THREAD join with the ball
  // and the tapped arm, since those override the part-level TW.throughBall/TW.tapped (not themselves exempt) for
  // exactly the pairs that overlap.
  const screw = (nm, c, sHead, sgn, e, arm) => {
    const headSlabs = mSlabsS(frame, 'head', c[0], c[1], sgn > 0 ? [sHead, sHead + sg.k] : [sHead - sg.k, sHead], sg.hd / 2);
    const shankSlabs = mSlabsS(frame, 'shank', c[0], c[1], sgn > 0 ? [sHead - sg.L, sHead] : [sHead, sHead + sg.L], sg.d / 2);
    const sc = add(`twist ${nm} screw ${tag}`, [...headSlabs, ...shankSlabs], { bom: P.screw });
    join(e, arm, TW.ball); join(sc, e, TW.throughBall); join(sc, arm, TW.tapped);
    for (const sl of shankSlabs) { join(`${sc}/${sl.name}`, e, THROUGH); join(`${sc}/${sl.name}`, arm, THREAD); }
    return sc;
  };
  const sL = screw('lever', G.Lp, G.eyeS[0], -1, eL, lever), sS = screw('servo arm', G.Hp, G.eyeS[0], -1, eS, horn);
  const A = pt(G.Lp, h1), B = pt(G.Hp, -h1), stud = mm(Math.min(ke.l1_mm, ke.min_thread_depth_mm ?? ke.l1_mm));
  const g6 = mm(1.0), link = add(`twist link ${tag}`, [...diagTube(frame, 'body', pt(G.Lp, h1 + g6), pt(G.Hp, -h1 - g6), mm(P.link_d_mm) / 2, 0, 0.02), ...diagTube(frame, 'stud lever', pt(G.Lp, h1 - stud), pt(G.Lp, h1 + g6), mm(1), 0, 0.02), ...diagTube(frame, 'stud servo', pt(G.Hp, -h1 - g6), pt(G.Hp, -(h1 - stud)), mm(1), 0, 0.02)], { bom: 'twist links' });
  join(link, eL, TW.stud); join(link, eS, TW.stud); join(`${link}/stud lever`, eL, THREAD); join(`${link}/stud servo`, eS, THREAD);
  // the frame's pins along x through both lips: over the case's top face (below the arm's hub), over its bottom face,
  // and two under the case
  const xr = [ids.caseX[0] - mm(P.hanger.lip_mm), ids.caseX[1] + mm(P.hanger.lip_mm)], rq = mm(P.hanger.pin_d_mm) / 2;
  const pins = add(`twist frame pins ${tag}`, [
    mpinX(frame, 'pin top', xr, ids.caseS[0] - rq, ids.caseN[0] + rq - mm(0.5), rq), mpinX(frame, 'pin bottom', xr, ids.caseS[1] + rq, G.S[1], rq),
    mpinX(frame, 'pin under 1', xr, ids.caseS[0] + mm(7), ids.caseN[0] - rq, rq), mpinX(frame, 'pin under 2', xr, ids.caseS[1] - mm(7), ids.caseN[0] - rq, rq),
  ].map((q, i) => ({ ...q, item: `twist frame pins ${tag} ${i + 1}` })), { bom: 'twist frame pins' });
  for (const nm of ['hanger lip forward', 'hanger lip aft']) join(pins, `${ids.joiner}/${nm}`, THROUGH);
  join(pins, ids.joiner, TW.framePin); join(pins, servo, TW.pinFace);
  Object.assign(ids, { lever, pin, horn, servo, eL, eS, sL, sS, link, pins });
}

function screwGeom(catalog, id) {
  const s = byId(catalog, id).specs ?? {}, t = `${s.mass_basis ?? ''} ${s.dimensions ?? ''}`, h = t.match(/head ([\d.]+) mm x ([\d.]+) mm/);
  return { d: mm(+id.match(/-m(\d+(?:\.\d+)?)x/)[1]), L: mm(s.length_mm ?? +id.match(/x(\d+(?:\.\d+)?)-/)[1]), hd: mm(s.head_d_mm ?? +h?.[1]), k: mm(s.head_k_mm ?? +h?.[2]) };
}

function drawBarAndTip(ax, tag, frame, G, P, ids) {
  const { add, join } = ax, R = P.ring, T = P.tip, clr = G.clr;
  // the bar, its two bonded rings: at the tip inboard of rib 10 (its outboard stop), at the elbow 0.6 mm outboard of the
  // joiner's plate (its inboard stop; the bar slides outboard through the plate as it lengthens with twist)
  const bar = add(`twist bar ${tag}`, [mcap(frame, 'tube', -G.e, G.nBar, [G.barS0, G.barS1], G.bar.ro, G.bar.ri)], { bom: 'twist bars' });
  join(bar, ids.joiner, TW.barSlide); join(bar, `${ids.joiner}/plate`, RIB_HOLE);
  const rrO = G.bar.ro + mm(R.wall_mm), rT = mm(R.t_mm), sR10 = G.s10t - G.tR / 2, sRe = G.sE + mm(P.plate.t_mm) / 2 + mm(0.6);
  const ringTip = add(`twist bar ring ${tag} tip`, [mtube(frame, 'ring', -G.e, G.nBar, [sR10 - rT, sR10], rrO, G.bar.ro)], { bom: 'twist rings' });
  const ringEl = add(`twist bar ring ${tag} elbow`, [mtube(frame, 'ring', -G.e, G.nBar, [sRe, sRe + rT], rrO, G.bar.ro)], { bom: 'twist rings' });
  for (const r of [ringTip, ringEl]) { join(r, bar, TW.ringBar); join(`${r}/ring`, bar, BORE); }
  // the end cap on the outer spar's tip end, the tip bushing in its wall, the tip hub on the rod against the bushing's flange
  const rc = mm(P.cap.od_mm) / 2;
  const capId = add(`twist spar cap ${tag}`, [mtube(frame, 'sleeve', 0, 0, [G.sOe - mm(P.cap.sleeve_mm), G.sOe], rc, G.so.ro), mtube(frame, 'wall', 0, 0, [G.sOe, G.capW1], rc, mm(G.gfi.d2_mm) / 2)], { bom: 'twist spar caps' });
  join(capId, ids.outer, TW.cap); join(`${capId}/sleeve`, ids.outer, BORE); join(`${capId}/sleeve`, ids.rod, BORE); join(`${capId}/wall`, ids.rod, BORE);
  const tb = add(`twist tip bushing ${tag}`, [mtube(frame, 'body', 0, 0, [G.sOe, G.capW1], mm(G.gfi.d2_mm) / 2, mm(G.gfi.d1_mm) / 2), mtube(frame, 'flange', 0, 0, [G.capW1, G.capF1], mm(G.gfi.d3_mm) / 2, mm(G.gfi.d1_mm) / 2)], { bom: P.rodBushing });
  join(tb, capId, TW.seat); join(`${tb}/body`, `${capId}/wall`, BORE); join(ids.rod, tb, TW.turns); join(`${ids.rod}/tube`, `${tb}/body`, BORE); join(`${ids.rod}/tube`, `${tb}/flange`, BORE); join(ids.outer, tb, TW.spEnd);
  const rH = mm(T.hub_od_mm) / 2, Dv = { s: G.armB.s - G.armA.s, x: G.armB.x - G.armA.x }, Ln = Math.hypot(Dv.s, Dv.x), so = mm(T.socket_mm) / Ln;
  const sk = mm(T.socket_od_mm) / 2, A = G.armA, B = G.armB;
  const hub = add(`twist tip hub ${tag}`, [mtube(frame, 'hub', 0, 0, [G.sH0, G.sH1], rH, G.rod.ro), mbox(frame, 'web', [A.x - mm(2), -rH + mm(0.5)], [G.sH0, A.s + mm(1)], [-mm(3.5), mm(3.5)]), ...diagTube(frame, 'socket', A, lerp3(A, B, so), sk, G.arm.ro, 0.02)], { bom: 'twist tip hubs' });
  join(`${hub}/hub`, ids.rod, BORE); join(hub, ids.rod, TW.pinned); join(hub, tb, TW.thrust);
  const rp = mm(P.lever.pin_d_mm) / 2, pS = (G.sH0 + G.sH1) / 2;
  // round 18.2 (the film's request): the pin's true section, N slabs across x (mSlabsN) narrowing toward the
  // circle, not mbox's flat box; every slab needs its own THROUGH join with the hub (see the elbow pin's note)
  const tipPinSlabs = mSlabsN(frame, 'pin', 0, pS, [-rH, rH], rp);
  const pin = add(`twist tip pin ${tag}`, tipPinSlabs, { bom: 'twist cross pins' });
  join(pin, hub, TW.pinIn); for (const sl of tipPinSlabs) join(`${pin}/${sl.name}`, `${hub}/hub`, THROUGH);
  join(pin, ids.rod, THROUGH);
  const arm = add(`twist tip arm ${tag}`, diagTube(frame, 'tube', A, B, G.arm.ro, G.arm.ri, 0.01), { bom: 'twist tip arms' });
  const pSock = diagTube(frame, 'socket', lerp3(A, B, 1 - so), B, sk, G.arm.ro, 0.02);
  const plate = add(`twist tip plate ${tag}`, [mbox(frame, 'plate', [-G.e - mm(4.5), B.x + mm(4.5)], [G.sTp0, G.sTp1], [-mm(4.5), mm(4.5)]), ...pSock], { bom: 'twist tip plates' });
  ids.plateSockets = pSock.map((q) => q.name);
  // the arm's ends bonded in the hub's and the plate's sockets (the tube in their bores); the bar through the plate's hole
  join(arm, hub, BORE); join(arm, plate, BORE); join(bar, plate, RIB_HOLE);
  Object.assign(ids, { bar, ringTip, ringEl, cap: capId, tipBushing: tb, hub, tipPin: pin, arm, plate });
}

function drawRibs(ax, tag, frame, G, P, ctx, ids) {
  const { d, catalog, add, join } = ax, parts = d.params.parts, wing = d.wing, t = G.tR, ribs = ctx.ribs;
  const rr = (id) => mm(byId(catalog, id).specs.od_mm) / 2, rLe = rr(parts.leRod), rTe = rr(parts.teRod), rT = mm(P.ring.t_mm);
  const pts = (s) => edgeRodPoints(wing, s, rLe, rTe, ctx.surf, G.sparN);
  // the edge rods: each bay in pieces of at most 10 mm, from the section's own leading and trailing edge points
  const edge = (name, r, which, bom) => {
    const sol = ribs.slice(0, -1).flatMap((s0, k) => {
      const a0 = s0 + t / 2, b0 = ribs[k + 1] - t / 2, n = Math.max(1, Math.ceil((b0 - a0) / 0.01 - 1e-9));
      return Array.from({ length: n }, (_, j) => {
        const a = a0 + (j / n) * (b0 - a0), b = a0 + ((j + 1) / n) * (b0 - a0), pa = pts(a)[which], pb = pts(b)[which];
        const La = j === 0 ? a - s0 : 0, Lb = j === n - 1 ? (k === ribs.length - 2 ? ribs[k + 1] + t / 2 - G.clr : ribs[k + 1]) - b : 0;
        return { ...M(`bay ${k + 1}${n > 1 ? `.${j + 1}` : ''}`, [Math.min(pa.x, pb.x) - r, Math.max(pa.x, pb.x) + r], (q) => { const f = frame(q); return cap(at(f, a, pa.n), at(f, b, pb.n), r); }), tube: { ro: r, ri: 0, L: b - a + La + Lb, axis: 'span' }, cl: { s: [a, b], x: [pa.x, pb.x], n: [pa.n, pb.n], r } };
      });
    });
    const p0 = pts(ribs[0])[which];
    sol.push({ ...M('in the ribs', [p0.x - r, p0.x + r], (q) => { const f = frame(q); return cap(at(f, ribs[0] - t / 2 + G.clr + r, p0.n), at(f, ribs[0] + t / 2 - r, p0.n), r); }), tube: { ro: r, ri: 0, L: t / 2 - G.clr, axis: 'span' }, cl: { s: [ribs[0] - t / 2 + G.clr, ribs[0] + t / 2], x: [p0.x, p0.x], n: [p0.n, p0.n], r } });
    // its ends in blind holes in the root and tip ribs (they hold it spanwise)
    return add(name, sol, { bom });
  };
  const leId = edge(`leading edge rod ${tag}`, rLe, 'le', 'leading edge rod'), teId = edge(`trailing edge rod ${tag}`, rTe, 'te', 'trailing edge rod');
  ribs.forEach((s, k) => {
    const z = ribZones(G, k, ctx.travel), rp = ribPieces(wing, s, rLe, rTe, ctx.surf, G.sparN, { ...z, t_mm: t * 1000 });
    const id = add(`rib ${tag} ${k + 1}`, ribSolids(frame, rp, s, t), { bom: 'ribs' });
    join(id, leId, RIB_HOLE); join(id, teId, RIB_HOLE);
    if (k <= G.k5) ribOnInner(ax, tag, frame, G, P, ids, id, s, k);
    else if (k <= G.k9) ribOnOuter(ax, tag, frame, G, P, ids, id, s, k);
    else { join(id, ids.plate, TW.ribTip); join(id, ids.arm, RIB_HOLE); join(ids.ringTip, id, TW.stop); join(ids.bar, id, TW.barHole); for (const q of ids.plateSockets ?? []) join(`${ids.plate}/${q}`, id, RIB_HOLE); }
  });
}

/** Ribs 1-5 on the inner spar, between bonded rings (rib 1: its outboard ring; rib 5: its inboard ring and the joiner). */
function ribOnInner(ax, tag, frame, G, P, ids, id, s, k) {
  const { add, join } = ax, t = G.tR, rT = mm(P.ring.t_mm), ro = G.si.ro + mm(P.ring.wall_mm);
  join(id, ids.inner, TW.onSpar);
  const sides = [...(k > 0 ? [['in', [s - t / 2 - rT, s - t / 2]]] : []), ...(k < G.k5 ? [['out', [s + t / 2, s + t / 2 + rT]]] : [])];
  for (const [nm, sr] of sides) { const r = add(`twist rib ring ${tag} ${k + 1} ${nm}`, [mtube(frame, 'ring', 0, 0, sr, ro, G.si.ro)], { bom: 'twist rings' }); join(r, ids.inner, BORE); join(r, id, TW.stop); }
  if (k === G.k5) join(ids.joiner, id, TW.stop);
}

/** Ribs 6-9 on their bushings on the outer spar, between bonded rings, riding on the bar in their slots. */
function ribOnOuter(ax, tag, frame, G, P, ids, id, s, k) {
  const { add, join } = ax, t = G.tR, rT = mm(P.ring.t_mm), ro = mm(G.gfm.d2_mm) / 2 + mm(P.ring.wall_mm), b2 = mm(G.gfm.b2_mm);
  const b = add(`twist rib bushing ${tag} ${k + 1}`, [mtube(frame, 'body', 0, 0, [s - t / 2, s + t / 2], mm(G.gfm.d2_mm) / 2, mm(G.gfm.d1_mm) / 2), mtube(frame, 'flange', 0, 0, [s + t / 2, s + t / 2 + b2], mm(G.gfm.d3_mm) / 2, mm(G.gfm.d1_mm) / 2)], { bom: P.ribBushing });
  join(b, id, TW.inRib); join(b, ids.outer, BORE);
  const rIn = add(`twist rib ring ${tag} ${k + 1} in`, [mtube(frame, 'ring', 0, 0, [s - t / 2 - rT, s - t / 2], ro, G.so.ro)], { bom: 'twist rings' });
  const rOut = add(`twist rib ring ${tag} ${k + 1} out`, [mtube(frame, 'ring', 0, 0, [s + t / 2 + b2, s + t / 2 + b2 + rT], ro, G.so.ro)], { bom: 'twist rings' });
  for (const r of [rIn, rOut]) join(r, ids.outer, BORE);
  join(rIn, id, TW.stop); join(rIn, b, TW.stop); join(rOut, b, TW.stop); join(id, ids.bar, TW.slot);
  for (const r of [rIn, rOut]) join(`${ids.rod}/tube`, `${r}/ring`, BORE);
  join(`${ids.rod}/tube`, `${b}/body`, BORE); join(`${ids.rod}/tube`, `${b}/flange`, BORE);
}

// ---- the BOM ----------------------------------------------------------------------------------------------------------

/** The machined and bought lines of the drive (per wing, drawn once in an unturned frame), by their BOM tag. */
const MACHINED = [['twist joiners', 'material'], ['twist levers', 'material'], ['twist tip hubs', 'material'], ['twist tip plates', 'material'], ['twist spar caps', 'material'], ['twist cross pins', 'material'], ['twist frame pins', 'material'], ['twist rings', 'material'], ['twist links', 'linkMat']];
const STOCK = [['twist bars', 'bar'], ['twist tip arms', 'arm']];

/**
 * One wing drawn in the unturned frame (s along z, n along y), its solids by BOM tag: { tag: { solids, items, V_m3,
 * L_m, c: [x, s, n] } }. pd: a design-like { params, wing, bom: { structure: [{ name: 'ribs, N off' }] } }.
 */
export function drawnLines(pd, catalog) {
  const parts = [], joins = [], frame = () => ({ o: [0, 0], u: [1, 0], v: [0, 1] });
  const ax = { d: pd, catalog, add: (id, solids, extra = {}) => { parts.push({ id, solids: solids.map((s) => ({ ...s, bom: s.bom ?? extra.bom })) }); return id; }, join: (a, b, w) => joins.push([a, b, w]), assumptions: [] };
  const out = twistWing(ax, 'R', frame), by = {}, tubeGroups = new Map();
  // round 18.2: a fastener's true section may be N slabs sharing one tube record (xSlabs); its volume and length
  // belong to the fastener once, not to every slab, so group them by item and base name (the slab suffix
  // stripped) and add that one volume at their average position, instead of summing the tube's full volume once
  // per slab (which would multiply it by N)
  for (const p of parts) for (const s of p.solids) {
    const b = by[s.bom] ?? (by[s.bom] = { solids: 0, items: new Set(), V_m3: 0, L_m: 0, m1: [0, 0, 0] });
    b.solids++; b.items.add(s.item ?? p.id);
    const sh = s.at ? s.at(null) : s.shape, c = shapeCentre(sh), pos = [(s.x[0] + s.x[1]) / 2, c[0], c[1]];
    // diagTube's own numbered pieces (axis 'diag') are genuinely different segments of one long member, each its
    // own volume and length, and must still be summed individually, not grouped as one fastener's slabs
    if (s.tube && s.tube.axis !== 'diag') {
      const key = `${s.bom}|${s.item ?? p.id}|${s.name.replace(/\s\d+$/, '')}`;
      const g = tubeGroups.get(key) ?? { bom: s.bom, tube: s.tube, sum: [0, 0, 0], n: 0 };
      g.sum = g.sum.map((q, i) => q + pos[i]); g.n++;
      tubeGroups.set(key, g);
    } else {
      const v = volumeOf(s, sh);
      b.V_m3 += v; b.L_m += s.tube?.L ?? 0;
      b.m1 = b.m1.map((q, i) => q + v * pos[i]);
    }
  }
  for (const g of tubeGroups.values()) {
    const b = by[g.bom], v = Math.PI * (g.tube.ro ** 2 - (g.tube.ri ?? 0) ** 2) * g.tube.L, pos = g.sum.map((q) => q / g.n);
    b.V_m3 += v; b.L_m += g.tube.L;
    b.m1 = b.m1.map((q, i) => q + v * pos[i]);
  }
  for (const b of Object.values(by)) { b.c = b.m1.map((q) => q / b.V_m3); b.items = b.items.size; }
  return { by, G: out.G, parts, joins };
}
function shapeCentre(sh) {
  if (sh.kind === 'ring') return sh.c;
  if (sh.kind === 'cap') return [(sh.a[0] + sh.b[0]) / 2, (sh.a[1] + sh.b[1]) / 2];
  const n = sh.pts.length; return [sh.pts.reduce((a, p) => a + p[0], 0) / n, sh.pts.reduce((a, p) => a + p[1], 0) / n];
}
function volumeOf(s, sh) {
  if (s.tube) return Math.PI * (s.tube.ro ** 2 - (s.tube.ri ?? 0) ** 2) * s.tube.L;
  const dx = s.x[1] - s.x[0];
  if (Number.isFinite(s.area_m2)) return s.area_m2 * dx;
  if (sh.kind === 'ring') return Math.PI * (sh.ro ** 2 - (sh.ri ?? 0) ** 2) * dx;
  const p = sh.pts; let A = 0; for (let i = 0; i < p.length; i++) { const q = p[(i + 1) % p.length]; A += p[i][0] * q[1] - q[0] * p[i][1]; }
  return (Math.abs(A) / 2) * dx;
}

/**
 * The drive's BOM (both wings) and the wing lines it redraws, from the drawn solids: { structure: [{ name, material,
 * mass_g, basis, count }], parts: [{ id, count, role }], lines: { 'inner spar': L_m, ... } (per wing, for the spar,
 * rod and rib lines build.js keeps), points: the wing's point masses (per wing, kg, at their drawn centroids), G }.
 */
export function twistBom(pd, catalog) {
  const P = twistParams(pd.params), { by, G } = drawnLines(pd, catalog), rho = (id) => byId(catalog, id).specs.density_g_cm3 * 1e6;
  const structure = [];
  for (const [tag, key] of MACHINED) {
    const b = by[tag]; if (!b) continue;
    const m = byId(catalog, P[key]);
    structure.push({ name: `${tag}, ${2 * b.items} off`, material: m, mass_g: 2 * b.V_m3 * rho(m.id), basis: `${(b.V_m3 * 1e9).toFixed(1)} mm3 per wing as drawn (src/design/twist-drive.js), x 2 wings; ${m.specs.density_g_cm3} g/cm3`, count: 2 * b.items });
  }
  for (const [tag, key] of STOCK) {
    const b = by[tag], m = byId(catalog, P[key]), gm = tubeSpec(m).g_per_m;
    structure.push({ name: `${tag}, ${2 * b.items} off`, material: m, mass_g: 2 * gm * b.L_m, basis: `${gm} g/m x ${b.L_m.toFixed(4)} m x 2 (as drawn)`, count: 2 * b.items });
  }
  const cnt = (id) => (by[id]?.items ?? 0) * 2;
  const parts = [
    { id: P.rodBushing, count: cnt(P.rodBushing), role: 'twist torque rod bushings (elbow and tip, two per wing)' },
    { id: P.ribBushing, count: cnt(P.ribBushing), role: 'hand-wing rib bushings (ribs 6-9 on the outer spar)' },
    { id: P.rodEnd, count: cnt(P.rodEnd), role: 'twist link rod ends, two per wing' },
    { id: P.horn, count: cnt(P.horn), role: 'twist servo arms (the maker\'s)' },
    { id: P.screw, count: cnt(P.screw), role: 'twist link screws (rod ends to the lever and the servo arm)' },
  ];
  const lines = { 'inner spar': by['inner spar'].L_m, 'outer spar': by['outer spar'].L_m, 'twist torque rods': by['twist torque rods'].L_m, 'leading edge rod': by['leading edge rod'].L_m, 'trailing edge rod': by['trailing edge rod'].L_m, ribsV_m3: by.ribs.V_m3 };
  return { structure, parts, lines, by, G, P };
}

// ---- the twist stiffness ---------------------------------------------------------------------------------------------

/**
 * The hand wing's twist stiffness about the spar axis (N m per rad of the flight model's twist): the edge rods' bending
 * energy as the ribs turn by w theta (w the flown weight at each rib) and carry each rod, X from the spar axis, X w theta
 * across the wing; each rod a natural cubic spline through its ribs (free at its ends in the root and tip ribs), its
 * energy (EI / 2) integral of y''^2, EI from the rod's published diameter and the top of its published modulus range
 * (the stiffer bound). The covering's own shear stiffness is not counted. -> { K_Nm_rad, rods: [{ rod, EI, K, R_N_per_rad }] }.
 */
export function twistStiffness(pd, catalog) {
  const P = twistParams(pd.params), { G, ribs, surf } = twistContext(pd, catalog, P), wing = pd.wing, L = span(wing), el = wing.segments[0].length;
  const wOf = (s) => (s <= el ? 0 : (s - el) / (L - el));
  const parts = pd.params.parts, rods = [['leading edge rod', parts.leRod, 'le'], ['trailing edge rod', parts.teRod, 'te']];
  const out = rods.map(([name, id, which]) => {
    const sp = byId(catalog, id).specs, r = mm(sp.od_mm) / 2, E = Math.max(...(sp.tensile_modulus_lengthways_gpa_range ?? [sp.tensile_modulus_lengthways_gpa])) * 1e9, EI = (E * Math.PI * (2 * r) ** 4) / 64;
    const ys = ribs.map((s) => { const p = edgeRodPoints(wing, s, mm(byId(catalog, parts.leRod).specs.od_mm) / 2, mm(byId(catalog, parts.teRod).specs.od_mm) / 2, surf, G.sparN)[which]; return p.x * wOf(s); });
    const sp2 = naturalSpline(ribs, ys);
    return { rod: name, id, EI_Nm2: EI, E_basis: `top of the published ${(sp.tensile_modulus_lengthways_gpa_range ?? []).join('-')} GPa range`, K_Nm_rad: EI * sp2.int2, R_N_per_rad: sp2.R.map((v) => v * EI) };
  });
  return { K_Nm_rad: out.reduce((a, q) => a + q.K_Nm_rad, 0), rods: out, basis: 'the edge rods\' bending energy (natural cubic splines through the ribs\' displacements at unit twist); the covering\'s shear stiffness not counted' };
}

/** A natural cubic spline through (x, y): the integral of y''^2 and each knot's reaction (the jump in y''') per unit EI. */
function naturalSpline(x, y) {
  const n = x.length, h = x.slice(1).map((v, i) => v - x[i]), Mv = new Array(n).fill(0);
  // tridiagonal system for the interior second derivatives
  const a = [], b = [], c = [], r = [];
  for (let i = 1; i < n - 1; i++) { a.push(h[i - 1]); b.push(2 * (h[i - 1] + h[i])); c.push(h[i]); r.push(6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])); }
  const m = r.length;
  for (let i = 1; i < m; i++) { const w = a[i] / b[i - 1]; b[i] -= w * c[i - 1]; r[i] -= w * r[i - 1]; }
  const z = new Array(m); for (let i = m - 1; i >= 0; i--) z[i] = (r[i] - (i < m - 1 ? c[i] * z[i + 1] : 0)) / b[i];
  for (let i = 0; i < m; i++) Mv[i + 1] = z[i];
  let int2 = 0; for (let i = 0; i < n - 1; i++) int2 += (h[i] * (Mv[i] ** 2 + Mv[i] * Mv[i + 1] + Mv[i + 1] ** 2)) / 3;
  const shear = h.map((hi, i) => (Mv[i + 1] - Mv[i]) / hi), R = x.map((_, i) => (i < n - 1 ? shear[i] : 0) - (i > 0 ? shear[i - 1] : 0));
  return { int2, M: Mv, R };
}

// ---- the loads --------------------------------------------------------------------------------------------------------

/**
 * The drive's load coefficients for one design (per wing): each a factor on a flight-model channel (twist_R / twist_L
 * the twist torque Q referred to the flown tip twist; root_yz, root_x, root_mperp the wing root's), from the drawn chain.
 *  - Q is carried kinematically: the rod's torque Tr = kq Q (kq = dtheta/dtheta_rod), the servo's Tr / k_rod at the
 *    four-bar's smallest ratio, the link's Tr / (r_l sin gamma) at its worst transmission angle over the travel.
 *  - Q is shared among ribs 6-10 as M_k = a_k Q with a_k proportional to c_k^2 b_k (each rib's chord squared times its
 *    span share, all of one sign: a stated distribution, conservative for the bar's bending and ends) and sum a_k w_k = 1;
 *    the bar carries ribs 6-9's (F_k = M_k / r_min), rib 10's goes to the tip plate; the bar is simply supported at
 *    its two holes.
 *  - The wing's air load is spread over the span as c(s) s (a stated distribution: its elbow-to-root moment ratio, 0.31,
 *    bounds v24's flown elbowBend / rootBend, 5.913 / 23.64 = 0.25, from above; c alone gives 0.24, under it): a rib's
 *    share of root_yz, the elbow moment's share of root_mperp. A part's spanwise inertia is its m s share of the wing's
 *    sum m s times root_x. The edge rods' reactions at the ribs grow with the twist, bounded through the twist torque
 *    at the design's own aerodynamic twist stiffness (control.diffLimit.twist_torque_per_rad; else the rods' own).
 */
export function twistLoads(d, catalog) {
  const P = twistParams(d.params), ctx = twistContext(d, catalog, P), { G, ribs } = ctx, K = twistKinematics(G), wing = d.wing, L = span(wing), el = wing.segments[0].length;
  const xi = (s) => (s - G.sE) / (G.sT - G.sE), kq = xi(L), wf = (s) => Math.max(0, (s - el) / (L - el));
  const th = (P.servo.travel_deg * Math.PI) / 180, samp = Array.from({ length: 61 }, (_, i) => -th + (2 * th * i) / 60);
  const kRod = Math.min(...samp.map((t) => K.ratio(t))), kRod0 = K.ratio(0);
  const sinG = Math.min(...samp.map((t) => { const al = G.alpha + K.rodAt(t), as = G.alpha + t, H = [G.S[0] + G.rs * Math.cos(as), G.S[1] + G.rs * Math.sin(as)], Lp = [G.rl * Math.cos(al), G.rl * Math.sin(al)], dv = [H[0] - Lp[0], H[1] - Lp[1]], n = Math.hypot(...dv); return Math.abs(Math.cos(al) * dv[1] - Math.sin(al) * dv[0]) / n; }));
  const kTr = kq, kTs = kq / kRod, kFL = kq / (G.rl * sinG);
  // ribs 6-10 share Q
  const k6 = G.k5 + 1, hand = ribs.slice(k6), bnd = ribs.map((s, k) => [k === 0 ? s : (s + ribs[k - 1]) / 2, k === ribs.length - 1 ? L : (s + ribs[k + 1]) / 2]);
  const raw = hand.map((s, j) => { const k = k6 + j, c = chordAtW(wing, s).c; return c * c * (bnd[k][1] - Math.max(bnd[k][0], el)); });
  const norm = raw.reduce((a, r, j) => a + r * wf(hand[j]), 0), a = raw.map((r) => r / norm);
  const rMin = G.e * Math.cos(servoTravel(G, P) / 2), Fk = a.slice(0, -1).map((v) => v / rMin);
  // the bar: simply supported at sE and sT, loads Fk at ribs 6-9
  const Lb = G.sT - G.sE, xs = hand.slice(0, -1).map((s) => s - G.sE);
  const Rt = Fk.reduce((acc, f, i) => acc + (f * xs[i]) / Lb, 0), Re = Fk.reduce((acc, f) => acc + f, 0) - Rt;
  const Mbar = Math.max(...xs.map((x0) => Re * x0 - Fk.reduce((acc, f, i) => acc + (xs[i] < x0 ? f * (x0 - xs[i]) : 0), 0)));
  // the wing's air load as c s^2: rib shares of root_yz, the elbow moment's share of root_mperp
  const N = 400, ds = L / N, q = Array.from({ length: N }, (_, i) => { const s = (i + 0.5) * ds; return { s, w: chordAtW(wing, s).c * s * ds }; });
  const W = q.reduce((acc, p) => acc + p.w, 0), Mroot = q.reduce((acc, p) => acc + p.w * p.s, 0);
  const share = (s0, s1) => q.filter((p) => p.s >= s0 && p.s < s1).reduce((acc, p) => acc + p.w, 0) / W;
  const fRib = bnd.map(([s0, s1]) => share(s0, s1)), kElb = q.filter((p) => p.s > G.jA).reduce((acc, p) => acc + p.w * (p.s - G.jA), 0) / Mroot, kV = share(G.jA, L);
  // spanwise inertia shares (m s over the wing's sum m s, from the design's own wing mass distribution)
  const wm = d.mass?.wing, ms = wm ? (wm.spar_kg_per_m * L * L) / 2 + q.reduce((acc, p) => acc + wm.skin_kg_per_m2 * chordAtW(wing, p.s).c * ds * p.s, 0) + (wm.point ?? []).reduce((acc, p) => acc + p.mass_kg * p.s_m, 0) : 1;
  const kx = (m_kg, s) => (m_kg * s) / ms;
  const stiff = twistStiffness(d, catalog);
  // Round 18 (diagnosis 2.5(a), bug): the edge rods' reaction at a rib per rad of the wing's ACTUAL twist angle
  // (R_N_per_rad, from the spline bending energy), not per N m of twist torque divided by an aerodynamic stiffness
  // (0.0531-0.069 N m/rad): that stiffness is far softer than the hardware, so torque/stiffness overstated the twist
  // by about 8x (5.3 rad at cruise, against the hardware's travel_rad of 0.675 rad). The flight model carries the
  // flown twist angle directly (LOAD_CHANNELS twist_angle_R/L, bounded by the servo's travel_rad).
  const Rrod = Math.max(...stiff.rods.flatMap((r) => r.R_N_per_rad.map(Math.abs)));
  return { P, G, K, ctx, kq, kRod, kRod0, sinG, kTr, kTs, kFL, a, Fk, rMin, Re, Rt, Mbar, fRib, kElb, kV, kx, ms, stiff, Rrod, hand };
}

// ---- the M7 members ---------------------------------------------------------------------------------------------------

const sect = (name, radial) => ({ name, radial, axial: [] });
const T_ = (ch, k) => ({ ch, k: r6(k) });

/** The drive's materials: the catalogue's metals, the rod's titanium, iglidur G, balsa (crushing at half), the bar's CFRP. */
function driveMaterials(catalog, P, mat) {
  const ti = byId(catalog, 'mtube-tigertitanium-ti3al2.5v-6.35x5.54').specs, gi = byId(catalog, P.rodBushing).specs, bal = byId(catalog, 'balsa-wood-database').specs;
  const igus = { id: 'iglidur G', yield_MPa: gi.static_surface_pressure_mpa, static_surface_pressure_mpa: gi.static_surface_pressure_mpa, basis: `iglidur G static surface pressure ${gi.static_surface_pressure_mpa} MPa (igus), as a crushing allowable` };
  const balsa = { id: 'balsa-wood-database', yield_MPa: bal.crushing_strength_mpa / 2, static_surface_pressure_mpa: bal.crushing_strength_mpa / 2, basis: `balsa crushing strength ${bal.crushing_strength_mpa} MPa (The Wood Database, species average, not a grade): counted at half` };
  const tr = byId(catalog, 'composite-rockwest-t-rnd-236-transverse').specs.design_values, a = tubeAllowable(byId(catalog, P.bar));
  return {
    al7: mat(P.material), al6: mat(P.linkMat), igus, balsa,
    ti: { id: 'mtube-tigertitanium-ti3al2.5v-6.35x5.54', yield_MPa: ti.yield_strength_mpa, shear_yield_MPa: ti.yield_strength_mpa / Math.sqrt(3), basis: 'published minimum yield; shear by von Mises' },
    rwT: { id: 'composite-rockwest-t-rnd-236-transverse', yield_MPa: tr.yield_MPa, basis: tr.basis },
    bar: { id: P.bar, axial_MPa: a.sigma_MPa, shear_MPa: tr.xy_shear_MPa, shear_basis: tr.xy_shear_basis },
  };
}

/**
 * The drive's M7 members (round 17b; audit items 1, 3, 12 and interfaces 14-20): every element of the chain and of the
 * elbow and the tip on the twist channel (twist_R / twist_L) and the wing's root channels. ax: airframe.js's member
 * context ({ d, catalog, members, mat, glueAl, glueCf }).
 */
export function twistMembers(ax) {
  const { d, catalog, members, mat, glueAl, glueCf } = ax, P = twistParams(d.params);
  if (!P) return;
  const Ld = twistLoads(d, catalog), { G } = Ld, M = driveMaterials(catalog, P, mat), sv = d.servos?.twist;
  // round 18: the arm's maker's-part condition (the servo's own rule at its supply's published row: chains.js)
  const armMk = sv ? servoArmMaker(d, catalog, 'twist', P.horn) : null;
  for (const [t, side] of [['R', 'right'], ['L', 'left']]) for (const m of twistMembersSide(Ld, G, P, M, { glueAl, glueCf, sv: sv && { ...sv, armMk }, catalog }, t, side)) members.push(m);
}

function twistMembersSide(Ld, G, P, M, { glueAl, glueCf, sv, catalog }, t, side) {
  const Q = `twist_${t}`, Qth = `twist_angle_${t}`, yz = `root_yz_${t}`, rx = `root_x_${t}`, ke = G.ke, gi = G.gfi, gm = G.gfm;
  const Tr = [T_(Q, Ld.kTr)], FL = [T_(Q, Ld.kFL)], dRod = mm(gi.d1_mm), rH = mm(P.lever.hub_od_mm) / 2, pinD = P.lever.pin_d_mm;
  const Fpin = [T_(Q, Ld.kTr / dRod)], armLen = Math.hypot(G.armB.s - G.armA.s, G.armB.x - G.armA.x), f10 = Ld.fRib.at(-1);
  const Farm = [T_(Q, Ld.kTr / Math.abs(G.armB.x)), T_(yz, f10)], Marm = Farm.map((q) => T_(q.ch, q.k * armLen));
  const inert = (m_kg, s) => [T_(rx, Ld.kx(m_kg, s))];
  // the elbow moment: the flight model's own (elbow_R / elbow_L, the M7 spar_outer moment)
  const Melb = [T_(`elbow_${t}`, 1)], Re = [T_(Q, Ld.Re)], Rt = [T_(Q, Ld.Rt)], fk = Math.max(...Ld.fRib.slice(G.k5 + 1, -1)), fIn = Math.max(...Ld.fRib.slice(0, G.k5 + 1));
  const caseL = G.caseDims[0], Tcase = [T_(Q, Ld.kTs / caseL)], lipH = mm(15);
  const U = uSection(P), seatF = (name, mat, dmm, tmm, F, note) => ({ name, kind: 'seat', check: 'seat', material: mat, dims: { d_mm: dmm, t_mm: tmm, crush: true }, sections: [sect('F', F)], uses: { F: 'F' }, note });
  const bond = (name, g, dmm, Lmm, T, F, note) => ({ name, kind: 'bond', check: 'bond', material: g, dims: { d_mm: dmm, L_mm: Lmm }, sections: [sect('T', T ?? []), sect('F', F ?? [])], uses: { T: 'T', F: 'F' }, note });
  const flange = (name, mat, D, dd, F, note) => ({ name, kind: 'retainer', check: 'flange', material: mat, dims: { D_mm: D, d_mm: dd }, sections: [sect('F', F)], uses: { F: 'F' }, note });
  const ribIn = Math.max(...Ld.fRib.slice(0, G.k5 + 1)), mRib = 0.0015, mBar = (tubeSpec(byId(catalog, P.bar)).g_per_m * (G.barS1 - G.barS0)) / 1000;
  const fRod = Ld.Rrod;
  const out = [
    { name: `twist torque rod ${t}`, kind: 'shaft', check: 'round', material: M.ti, dims: { d_mm: gi.d1_mm, di_mm: G.rod.ri * 2000 }, sections: [sect('T', Tr)], uses: { T: 'T' }, note: `the twist torque kq Q to the tip hub (kq ${Ld.kq.toFixed(4)})` },
    seatF(`twist elbow bushing ${t}`, M.igus, gi.d1_mm, gi.b1_mm - gi.b2_mm - P.clear_mm, FL, `the link's force at the lever beside it, on its seated length (the joiner's wall ${P.clear_mm} mm short of the body: the lever's thrust face)`),
    flange(`twist elbow bushing ${t} thrust`, M.igus, gi.d2_mm, gi.d1_mm, inert(0.03, G.sL), 'the lever\'s hub on the body\'s end: the rod, its hubs, the arm and the tip plate outward (their m s share of the root\'s spanwise force, 30 g at the elbow: an upper bound)'),
    seatF(`twist tip bushing ${t}`, M.igus, gi.d1_mm, gi.b1_mm - gi.b2_mm, Farm, 'the tip arm\'s force at the hub (the twist torque over the arm\'s reach, and rib 10\'s air load)'),
    flange(`twist tip bushing ${t} thrust`, M.igus, gi.d3_mm, gi.d1_mm, inert(0.03, G.sH0), 'the tip hub on the flange (the rod\'s inboard stop): the same spanwise bound'),
    { name: `twist elbow lever ${t} arm`, kind: 'plate', check: 'plate', material: M.al7, dims: { b_mm: P.lever.arm_w_mm, t_mm: P.lever.arm_t_mm }, sections: [sect('M', FL.map((q) => T_(q.ch, q.k * (G.rl - rH))))], uses: { M: 'M' }, note: 'the link\'s force at its screw, bending the arm in its plane at the hub' },
    ...pinMembers(`twist elbow pin ${t}`, M, glueAl, pinD, rH, G, Fpin, inert(0.0002, G.sL)),
    ...pinMembers(`twist tip pin ${t}`, M, glueAl, pinD, mm(P.tip.hub_od_mm) / 2, G, Fpin, inert(0.0002, G.sH0)),
    { name: `twist link ${t} rod ends`, kind: 'rod end', check: 'rated', dims: { rating_N: ke.max_static_tensile_long_term_n, rating_basis: `${P.rodEnd}: maximum static tensile load, long term (igus)` }, sections: [sect('F', FL)], uses: { F: 'F' }, note: 'the link\'s force through each rod end (along its shank)' },
    { name: `twist link ${t} screws`, kind: 'fasteners', check: 'bolts', fastener: fastenerOfCat(catalog, P.screw), dims: { n: 1 }, sections: [sect('V', FL)], uses: { V: 'V' }, note: 'each rod end\'s M2 screw in shear' },
    // round 18 (agent C's finding): the lever's rod end screw goes into a thread tapped in our 7075 lever, rated as the
    // tail horn's is (the servo arm's thread is the maker's part)
    tappedThreadMember(`twist elbow lever ${t} screw thread in the arm`, M.al7, { screw_len_mm: byId(catalog, P.screw).specs.length_mm, ball_B_mm: ke.B_mm, arm_t_mm: P.lever.arm_t_mm }, FL, 'the 7075 lever\'s arm', "the link's"),
    { name: `twist link ${t}`, kind: 'strut', check: 'strut', material: M.al6, dims: { L_mm: +(Ld.K.link * 1000 - 2 * ke.h1_mm).toFixed(2), d_mm: P.link_d_mm, di_mm: 0 }, sections: [sect('F', FL)], uses: { F: 'F' }, note: 'the 6061 link between the rod ends\' shanks' },
    { name: `twist link ${t} studs`, kind: 'stud', check: 'beam', material: M.al6, dims: { A_mm2: 2.07, Zi_mm3: 1e9, Zo_mm3: 1e9 }, sections: [sect('F', FL)], uses: { F: 'F' }, note: 'the link\'s M2 studs in tension over the M2 stress area 2.07 mm2 (6061 at its yield)' },
    // round 18 (SPEC ruling "Maker's part", operator's decision 2026-11-03): the maker's arm, its centre screw and its
    // tapped holes on its own servo's spline are a maker's part, no longer a member held to the stall at FoS 1.5 (40% of
    // stall under the envelope's member allowance). It reports its force at the hole (F, the link's) and its torque at the
    // spline (T, the servo's: the wing's twist torque x the linkage ratio), and is valid only while the servo is within its
    // own rule at the supply's published row (chains.js servoArmMaker)
    ...(sv ? [{ name: `twist servo arm ${t}`, kind: 'arm', check: "maker's part", maker: sv.armMk, dims: { hole_mm: P.servo.hole_mm }, sections: [sect('F', FL), sect('T', [{ ch: Q, k: sv.linkage_ratio ?? 1 }])], uses: { F: 'F', T: 'T' }, note: `the maker's ${P.horn} on the servo's spline: the link's force at its ${P.servo.hole_mm} mm hole, and the servo's torque at the spline; no rating of its own is published (SPEC ruling "Maker's part")` }] : []),
    { name: `twist servo ${t} frame lips`, kind: 'plate', check: 'plate', material: M.al7, dims: { b_mm: +((G.caseDims[2] + mm(4)) * 1000).toFixed(1), t_mm: P.hanger.lip_mm }, sections: [sect('Mo', [...FL, ...Tcase].map((q) => T_(q.ch, q.k * lipH / 2)))], uses: { Mo: 'Mo' }, note: 'the case\'s chordwise push (the link\'s force and its torque as a couple over its length) on one lip, at half its depth' },
    { name: `twist frame pins ${t}`, kind: 'pin', check: 'round', material: M.al7, dims: { d_mm: P.hanger.pin_d_mm }, sections: [sect('M', [...Tcase, T_(yz, 0.0215 / 0.11)].map((q) => T_(q.ch, (q.k * (caseL + 2 * mm(P.hanger.lip_mm))) / 4)))], uses: { M: 'M' }, note: 'an under pin across the case\'s length: the case\'s torque reaction as a couple over its length, and its own normal load (its mass share of the wing\'s root force, 21.5 g of about 110 g), simply supported' },
    bond(`twist frame pin bonds ${t}`, glueAl, P.hanger.pin_d_mm, 2 * P.hanger.lip_mm, null, inert(0.0215, G.sCase), 'Araldite 2011 on aluminium: each pin in its two lips, the case\'s spanwise inertia'),
    bond(`twist joiner ${t} inner sleeve bond`, glueCf, G.si.ro * 2000, P.joiner.inner_mm, [T_(Q, Ld.kTs + Ld.Re * G.e)], [...Melb.map((q) => T_(q.ch, (4 * q.k) / (G.si.ro * 2))), T_(rx, 1)], 'Araldite 2011 on CFRP (typical, half): the servo\'s reaction and the bar\'s elbow force about the spar as torsion; the elbow moment as the bond\'s axial shear (tau = M / (pi r^2 L), the spar\'s bending stress handed to the sleeve along it), and the hand wing\'s spanwise force'),
    bond(`twist joiner ${t} outer sleeve bond`, glueCf, G.so.ro * 2000, P.joiner.outer_mm, null, [...Melb.map((q) => T_(q.ch, (4 * q.k) / (G.so.ro * 2))), T_(rx, 1)], 'Araldite 2011 on CFRP (typical, half): the outer spar\'s elbow moment as the bond\'s axial shear, and its spanwise force'),
    { name: `twist joiner ${t} bay`, kind: 'beam', check: 'beam', material: M.al7, dims: U, sections: [sect('M', Melb), sect('F', [T_(rx, 1)])], uses: { M: 'M', F: 'F' }, note: 'the open-topped bay (its floor and two low walls) in the elbow moment, about its weaker axis, and the spanwise force' },
    { name: `twist joiner ${t} outer sleeve`, kind: 'tube', check: 'round', material: M.al7, dims: { d_mm: P.joiner.outer_od_mm, di_mm: G.so.ro * 2000 }, sections: [sect('M', Melb)], uses: { M: 'M' }, note: 'the thinnest sleeve in the elbow moment' },
    { name: `twist joiner ${t} plate`, kind: 'plate', check: 'plate', material: M.al7, dims: { b_mm: P.plate.depth_mm, t_mm: P.plate.t_mm }, sections: [sect('M', Re.map((q) => T_(q.ch, q.k * (G.e - mm(P.joiner.od_mm) / 2))))], uses: { M: 'M' }, note: 'the bar\'s elbow force at the plate\'s hole, the plate cantilevered from the bay' },
    { name: `twist bar ${t}`, kind: 'tube', check: 'cfrp tube', material: M.bar, dims: { d_mm: G.bar.od_rate, di_mm: G.bar.id_rate }, sections: [sect('M', [T_(Q, Ld.Mbar)]), sect('V', [T_(Q, Math.max(Ld.Re, Ld.Rt))])], uses: { M: 'M', V: 'V' }, note: `the ribs' twist moments as forces on the bar at r_min ${(Ld.rMin * 1000).toFixed(1)} mm, simply supported at its two holes` },
    seatF(`twist bar ${t} in the joiner's plate`, M.rwT, G.bar.od_rate, P.plate.t_mm, Re, 'the bar bearing in the plate\'s hole (Rock West transverse compressive, CLT, half)'),
    seatF(`twist bar ${t} in the tip plate`, M.rwT, G.bar.od_rate, P.tip.plate_t_mm, Rt, 'the bar bearing in the tip plate\'s hole'),
    seatF(`twist bar ${t} in the ribs`, M.balsa, G.bar.od_rate, G.tR * 1000, [T_(Q, Math.max(...Ld.Fk)), T_(yz, f10)], 'the bar on a rib\'s slot (ribs 6-9: the largest share) or rib 10\'s hole: balsa crushing at half'),
    bond(`twist bar ${t} rings`, glueCf, G.bar.od_rate, P.ring.t_mm, null, inert(mBar + 0.003, G.sT), 'Araldite 2011 on CFRP (typical, half): the bar\'s and rib 10\'s spanwise inertia on its tip ring'),
    seatF(`twist rib bushings ${t}`, M.igus, gm.d1_mm, G.tR * 1000, [T_(Q, Math.max(...Ld.Fk)), T_(yz, fk)], 'a hand-wing rib\'s bar force and its air load (its c s^2 share of the root force) on its bushing'),
    seatF(`twist rib bushings ${t} in the ribs`, M.balsa, gm.d2_mm, G.tR * 1000, [T_(Q, Math.max(...Ld.Fk)), T_(yz, fk)], 'the bushing bearing in its balsa rib (crushing at half)'),
    flange(`twist rib bushings ${t} thrust`, M.igus, gm.d2_mm, gm.d1_mm, inert(mRib, G.s9), 'a rib\'s spanwise inertia through its bushing\'s body end on the ring (a 1.5 g rib at rib 9: the largest m s)'),
    bond(`twist rib rings ${t}`, glueCf, G.so.ro * 2000, P.ring.t_mm, null, inert(mRib, G.s9), 'Araldite 2011 on CFRP (typical, half): a ring holding its rib spanwise (the 8 mm spar, the smaller bond)'),
    flange(`wing ribs ${t} on their rings`, M.balsa, G.si.ro * 2000 + 2 * P.ring.wall_mm, G.si.ro * 2000, inert(mRib, G.s9), 'a rib\'s face on its ring (balsa crushing at half over the ring\'s annulus)'),
    seatF(`wing ribs ${t} on the inner spar`, M.balsa, G.si.ro * 2000, G.tR * 1000, [T_(yz, ribIn)], 'ribs 1-5: the air load (the largest c s^2 share) on the spar through the rib\'s hole'),
    seatF(`wing ribs ${t} on the edge rods`, M.balsa, 1.5, G.tR * 1000, [T_(Qth, fRod), T_(yz, fk / 2)], 'round 18 (diagnosis 2.5(a)): an edge rod\'s largest reaction at a rib (its spline reaction per rad, R_N_per_rad) at the flown wing twist (the hardware\'s own bound, LOAD_CHANNELS twist_angle), not torque over the far softer aerodynamic twist stiffness (about 8x too much) and half the rib\'s air load, on the 1.5 mm rod\'s hole'),
    bond(`twist rod rings ${t}`, glueCf, 1.5, P.ring.t_mm, null, inert(0.0045, G.L / 2), 'Araldite 2011 on CFRP (typical, half): an edge rod\'s spanwise inertia on its rings at rib 2 (the 2 mm rod\'s 4.5 g/m, the 1.5 mm rod\'s smaller bond)'),
    bond(`twist spar cap ${t} bond`, glueCf, G.so.ro * 2000, P.cap.sleeve_mm, null, Farm, 'Araldite 2011 on CFRP (typical, half): the tip bushing\'s radial load on the spar\'s end'),
    { name: `twist tip arm ${t}`, kind: 'tube', check: 'cfrp tube', material: M.bar, dims: { d_mm: G.arm.od_rate, di_mm: G.arm.id_rate }, sections: [sect('M', Marm), sect('V', Farm)], uses: { M: 'M', V: 'V' }, note: `the twist torque and rib 10's air load at the tip plate, the arm cantilevered ${(armLen * 1000).toFixed(0)} mm from the hub` },
    bond(`twist tip arm ${t} socket bonds`, glueCf, G.arm.od_rate, P.tip.socket_mm, null, Marm.map((q) => T_(q.ch, (4 * q.k) / (G.arm.ro * 2))), 'Araldite 2011 on CFRP (typical, half): the arm\'s root moment as the bond\'s axial shear'),
    { name: `twist tip plate ${t}`, kind: 'plate', check: 'plate', material: M.al7, dims: { b_mm: 9, t_mm: P.tip.plate_t_mm }, sections: [sect('M', Rt.map((q) => T_(q.ch, q.k * (Math.abs(G.armB.x) - G.e + mm(4)))))], uses: { M: 'M' }, note: 'the bar\'s tip force from its hole to the arm\'s socket' },
    flange(`wing rib ${t} 10 on the tip plate`, M.balsa, 13.5, 0, inert(0.002, G.L), 'rib 10 pressed on the tip plate by its spanwise inertia (the plate\'s 9 x 16 mm face as a 13.5 mm disc)'),
    bond(`twist spar root bond ${t} torsion`, glueCf, G.si.ro * 2000, 20, [T_(Q, Ld.kTs + Ld.Re * G.e)], null, 'Araldite 2011 on CFRP (typical, half): the twist drive\'s reaction (the servo\'s and the bar\'s elbow force about the spar) reaches the rocker through the inner spar\'s root bond in torsion (audit-v24 interface 14)'),
  ];
  return out;
}

function pinMembers(name, M, glueAl, pinD, rHub, G, F, Fax) {
  const wall = (G.rod.ro - G.rod.ri) * 1000, hubWall = (rHub - G.rod.ro) * 1000;
  return [
    { name, kind: 'pin', check: 'pin-shear', material: M.al7, dims: { d_mm: pinD, planes: 2 }, sections: [sect('F', F)], uses: { F: 'F' }, note: 'the rod\'s torque as a couple across its diameter, the pin in double shear' },
    { name: `${name} in the rod wall`, kind: 'seat', check: 'seat', material: M.ti, dims: { d_mm: pinD, t_mm: +wall.toFixed(3), crush: true }, sections: [sect('F', F)], uses: { F: 'F' }, note: 'the pin bearing on the titanium rod\'s 0.406 mm wall (crushing at the published minimum yield)' },
    { name: `${name} in the hub`, kind: 'seat', check: 'seat', material: M.al7, dims: { d_mm: pinD, t_mm: +hubWall.toFixed(3), crush: true }, sections: [sect('F', F)], uses: { F: 'F' }, note: 'the pin bearing on the hub\'s wall' },
    { name: `${name} bond`, kind: 'bond', check: 'bond', material: glueAl, dims: { d_mm: pinD, L_mm: +(2 * hubWall).toFixed(3) }, sections: [sect('T', []), sect('F', Fax)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on aluminium: the pin held in the hub (its own inertia)' },
  ];
}

/** The bay's open-topped section (its floor and two low walls), mm: area and the smaller section modulus. */
function uSection(P) {
  const ro = P.joiner.od_mm / 2, t = P.joiner.u_t_mm, rects = [[-(ro - t), ro - t, -ro, -ro + t], [-ro, -ro + t, -ro, 2.5], [ro - t, ro, -ro, 2]];
  const A = rects.reduce((a, [x0, x1, n0, n1]) => a + (x1 - x0) * (n1 - n0), 0);
  const cx = rects.reduce((a, [x0, x1, n0, n1]) => a + (x1 - x0) * (n1 - n0) * (x0 + x1) / 2, 0) / A, cn = rects.reduce((a, [x0, x1, n0, n1]) => a + (x1 - x0) * (n1 - n0) * (n0 + n1) / 2, 0) / A;
  const I = (ax) => rects.reduce((a, [x0, x1, n0, n1]) => { const b = x1 - x0, h = n1 - n0; return a + (ax === 'x' ? (b * h ** 3) / 12 + b * h * ((n0 + n1) / 2 - cn) ** 2 : (h * b ** 3) / 12 + b * h * ((x0 + x1) / 2 - cx) ** 2); }, 0);
  const cMax = (ax) => Math.max(...rects.flatMap(([x0, x1, n0, n1]) => (ax === 'x' ? [Math.abs(n0 - cn), Math.abs(n1 - cn)] : [Math.abs(x0 - cx), Math.abs(x1 - cx)])));
  const Z = Math.min(I('x') / cMax('x'), I('n') / cMax('n'));
  return { A_mm2: +A.toFixed(3), Zi_mm3: +Z.toFixed(3), Zo_mm3: +Z.toFixed(3) };
}
function fastenerOfCat(catalog, id) { const f = byId(catalog, id), dv = f.specs.design_values ?? {}; return { id, tension_N: dv.tension_N, shear_N: dv.shear_N }; }

// ---- the interface rules (interfaces.js format; the lead passes them ahead of INTERFACE_RULES) -----------------------

const RULE = {
  bush: "the maker's published static surface pressure at FoS 1.5 (plain bushings; SPEC Rulings after physics round 4, 'Bearings are linkage')",
  bond: "the adhesive maker's published lap-shear strength on that substrate (SPEC Rulings after physics round 7)",
  seat: 'pin-in-hole bearing at the crushing allowable (SPEC Rulings after physics round 7, crushing on a seat)',
  pin: 'the pin in double shear and its bearing on the rod and the hub (the material\'s published yield; von Mises shear)',
  rated: "the part maker's published load rating at FoS 1.5 (members.js 'rated')",
  bolt: 'ISO 898-1 proof load (SPEC Rulings after physics round 7)',
  beam: 'the member\'s own section at the material\'s published yield, FoS 1.5',
  thread: "pin-in-shear on the thread's stripping area at the material's shear yield, FoS 1.5 (as the tail horn's)",
  none: 'SPEC Design freeze, Function completeness: what holds it axially needs a named, rated member',
};
const sideOf = (a, b) => { const m = /(?:^| )([RL])(?= |$|\/)/.exec(a) ?? /(?:^| )([RL])(?= |$|\/)/.exec(b); return m ? m[1] : null; };
const exactRe = (text) => new RegExp(`^${text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}$`);
const BORE_RE = /^in the other tube's bore/, HOLE_RE = /^through its hole in the rib/;
/** A rule row: the join's reason, the part pattern, kind, the load and axial members (names from the side t). */
const row = (why, part, kind, load, axial, byL, byA, ruleL = RULE.seat, ruleA = RULE.bond, note = '') => ({
  why, part, kind, note,
  load: { by: byL, rating: (d, a, b) => { const t = sideOf(a, b); return t ? load(t, t === 'R' ? 'right' : 'left') : []; }, rule: ruleL },
  axial: { by: byA, rating: (d, a, b) => { const t = sideOf(a, b); return t ? axial(t, t === 'R' ? 'right' : 'left') : []; }, rule: ruleA },
});
const rodSupport = (t) => [`twist elbow bushing ${t}`, `twist tip bushing ${t}`], rodStops = (t) => [`twist elbow bushing ${t} thrust`, `twist tip bushing ${t} thrust`];
const ribRingsT = (t) => [`twist rib rings ${t}`];

/** The twist drive's interface rules, most specific first (the part patterns pick the drive's own pairs). */
export const TWIST_INTERFACE_RULES = [
  // the spar root bond now also carries the drive's reaction about the spar (audit interface 14)
  row(exactRe('bonded in the spar socket'), /^inner spar [RL]$/, 'bonded', (t, sd) => [`spar root socket ${sd}`, `twist spar root bond ${t} torsion`], (t, sd) => [`spar root bond ${sd}`], 'the socket seat and the root bond in torsion', 'the spar root bond', RULE.seat, RULE.bond),
  // tubes in bores (the clearance check's exemption) that are the drive's real fits
  row(BORE_RE, /^twist joiner [RL]\/inner sleeve$/, 'bonded', (t) => [`twist joiner ${t} inner sleeve bond`, `twist joiner ${t} bay`], (t) => [`twist joiner ${t} inner sleeve bond`], 'the joiner\'s inner sleeve bond', 'the same bond', RULE.bond),
  row(BORE_RE, /^twist joiner [RL]\/outer sleeve$/, 'bonded', (t) => [`twist joiner ${t} outer sleeve bond`, `twist joiner ${t} outer sleeve`], (t) => [`twist joiner ${t} outer sleeve bond`], 'the joiner\'s outer sleeve bond', 'the same bond', RULE.bond),
  row(BORE_RE, /^twist (elbow|tip) bushing [RL]\/(body|flange)$/, 'press fit', (t) => rodSupport(t), (t) => [`twist joiner ${t} outer sleeve bond`, `twist spar cap ${t} bond`], 'the bushing (static pressure)', 'its flange captured by the spar\'s end (the sleeve and cap bonds hold the spar)', RULE.bush, RULE.bond),
  row(BORE_RE, /^twist (bar|rod) ring|^twist (bar|rod) ring/, 'bonded', (t) => [`twist bar ${t} rings`, `twist rod rings ${t}`], (t) => [`twist bar ${t} rings`, `twist rod rings ${t}`], 'the ring\'s bond', 'the same bond', RULE.bond),
  row(BORE_RE, /^twist rib ring [RL] \d+ (in|out)/, 'bonded', (t) => ribRingsT(t), (t) => ribRingsT(t), 'the ring\'s bond on its spar', 'the same bond', RULE.bond),
  row(BORE_RE, /^twist rib bushing [RL] \d+/, 'journal', (t) => [`twist rib bushings ${t}`], (t) => [`twist rib bushings ${t} thrust`, ...ribRingsT(t)], 'the rib\'s bushing on the outer spar (static pressure)', 'its rings on the spar', RULE.bush, RULE.bond),
  row(BORE_RE, /^twist spar cap [RL]\/sleeve$/, 'bonded', (t) => [`twist spar cap ${t} bond`], (t) => [`twist spar cap ${t} bond`], 'the cap\'s bond on the spar', 'the same bond', RULE.bond),
  row(BORE_RE, /^twist (elbow lever|tip hub) [RL]\/hub$/, 'pinned', (t) => [`twist elbow pin ${t}`, `twist elbow pin ${t} in the rod wall`, `twist tip pin ${t}`, `twist tip pin ${t} in the rod wall`], (t) => [`twist elbow pin ${t} bond`, `twist tip pin ${t} bond`, ...rodStops(t)], 'the hub\'s cross pin', 'the pin\'s bond and the rod\'s thrust faces', RULE.pin, RULE.bond),
  row(BORE_RE, /^twist tip arm [RL]/, 'bonded', (t) => [`twist tip arm ${t}`, `twist tip arm ${t} socket bonds`], (t) => [`twist tip arm ${t} socket bonds`], 'the arm in its sockets', 'the socket bonds', RULE.beam, RULE.bond),
  // the torque rod passing inside the joiner, the spar, the cap, the bushings and rings (no contact but at its bushings)
  row(BORE_RE, /^twist torque rod [RL]/, 'shaft in bearing', (t) => rodSupport(t), (t) => rodStops(t), 'the rod\'s two bushings carry it', 'its two thrust faces', RULE.bush, RULE.bush, 'the rod runs in the outer spar\'s bore with 0.225 mm radial clearance, touching only its two GFI bushings'),
  // holes through ribs and plates
  row(HOLE_RE, /^twist bar [RL]$/, 'through hole', (t) => [`twist bar ${t} in the joiner's plate`, `twist joiner ${t} plate`, `twist bar ${t} in the tip plate`, `twist tip plate ${t}`], (t) => [`twist bar ${t} rings`], 'the bar bearing in the plate\'s hole', 'its two bonded rings', RULE.seat, RULE.bond),
  row(HOLE_RE, /^rib [RL] \d+$/, 'through hole', (t) => [`wing ribs ${t} on the edge rods`, `twist bar ${t} in the ribs`], (t) => [`twist rod rings ${t}`, ...ribRingsT(t), `twist bar ${t} rings`], 'the rod, arm or socket bearing in the rib\'s hole (balsa crushing at half)', 'the rods\' rings at rib 2, the ribs\' rings on the spars, the bar\'s rings', RULE.seat, RULE.bond),
  // the drive's own reasons
  row(exactRe(TW.innerSleeve), null, 'bonded', (t) => [`twist joiner ${t} inner sleeve bond`, `twist joiner ${t} bay`], (t) => [`twist joiner ${t} inner sleeve bond`], 'the inner sleeve bond', 'the same bond', RULE.bond),
  row(exactRe(TW.outerSleeve), null, 'bonded', (t) => [`twist joiner ${t} outer sleeve bond`, `twist joiner ${t} outer sleeve`], (t) => [`twist joiner ${t} outer sleeve bond`], 'the outer sleeve bond', 'the same bond', RULE.bond),
  row(exactRe(TW.seat), null, 'press fit', (t) => rodSupport(t), (t) => [`twist joiner ${t} outer sleeve bond`, `twist spar cap ${t} bond`], 'the bushing in its seat (static pressure)', 'its flange between the seat and the spar\'s end', RULE.bush, RULE.bond),
  row(exactRe(TW.turns), null, 'shaft in bearing', (t) => rodSupport(t), (t) => rodStops(t), 'the rod in its bushings', 'the hubs\' faces on the bushings', RULE.bush, RULE.bush),
  row(exactRe(TW.flangeHeld), null, 'captured', (t) => [`twist joiner ${t} outer sleeve bond`], (t) => [`twist joiner ${t} outer sleeve bond`], 'the spar held in its sleeve', 'the same bond', RULE.bond),
  row(exactRe(TW.spEnd), null, 'captured', (t) => [`twist spar cap ${t} bond`], (t) => [`twist spar cap ${t} bond`], 'the cap on the spar', 'the cap\'s bond', RULE.bond),
  row(exactRe(TW.pinned), null, 'pinned', (t) => [`twist elbow pin ${t}`, `twist elbow pin ${t} in the rod wall`, `twist elbow pin ${t} in the hub`, `twist tip pin ${t}`, `twist tip pin ${t} in the rod wall`, `twist tip pin ${t} in the hub`], (t) => [`twist elbow pin ${t} bond`, `twist tip pin ${t} bond`, ...rodStops(t)], 'the cross pins', 'the pins\' bonds and the thrust faces', RULE.pin, RULE.bond),
  row(exactRe(TW.pinIn), null, 'bonded pin', (t) => [`twist elbow pin ${t}`, `twist elbow pin ${t} in the hub`, `twist tip pin ${t}`, `twist tip pin ${t} in the hub`], (t) => [`twist elbow pin ${t} bond`, `twist tip pin ${t} bond`], 'the pin in its hub', 'its bond', RULE.pin, RULE.bond),
  row(exactRe(TW.thrust), null, 'thrust face', (t) => rodStops(t), (t) => rodStops(t), 'the hub\'s face on the bushing', 'the same faces', RULE.bush, RULE.bush),
  // round 18 (SPEC ruling "Maker's part"): on the servo's arm, the arm's hole and its tapped thread are the maker's part;
  // the KBRM rod end and its M2 screw (a bought part on our linkage) keep their own rules. The lever's rows are ours.
  row(exactRe(TW.ball), /^twist servo arm [RL]$/, 'rod end', (t) => [`twist link ${t} rod ends`, `twist link ${t} screws`, `twist servo arm ${t}`], (t) => [`twist link ${t} screws`], 'the rod end on its screw; the arm\'s hole the maker\'s part', 'the screw', `${RULE.rated}; the arm: ${MAKERS_PART_RULE}`, RULE.bolt),
  row(exactRe(TW.ball), null, 'rod end', (t) => [`twist link ${t} rod ends`, `twist link ${t} screws`, `twist elbow lever ${t} arm`], (t) => [`twist link ${t} screws`], 'the rod end on its screw', 'the screw', RULE.rated, RULE.bolt),
  row(exactRe(TW.throughBall), null, 'fastened', (t) => [`twist link ${t} screws`], (t) => [`twist link ${t} screws`], 'the screw', 'the screw', RULE.bolt, RULE.bolt),
  row(exactRe(TW.tapped), /^twist servo arm [RL]$/, 'fastened', (t) => [`twist link ${t} screws`], (t) => [`twist link ${t} screws`, `twist servo arm ${t}`], 'the screw', 'the screw, in the maker\'s tapped thread in its arm', RULE.bolt, `${RULE.bolt}; the arm's thread: ${MAKERS_PART_RULE}`),
  // round 18: on our lever the thread is ours (7075), rated as the tail horn's: pin-in-shear on its stripping area
  row(exactRe(TW.tapped), null, 'fastened', (t) => [`twist link ${t} screws`, `twist elbow lever ${t} screw thread in the arm`], (t) => [`twist link ${t} screws`, `twist elbow lever ${t} screw thread in the arm`], 'the screw in shear, in the lever\'s tapped thread', 'the screw, in the thread tapped in the 7075 lever', RULE.bolt, `${RULE.bolt}; the lever's thread: ${RULE.thread}`),
  row(exactRe(TW.stud), null, 'threaded', (t) => [`twist link ${t} studs`, `twist link ${t} rod ends`, `twist link ${t}`], (t) => [`twist link ${t} studs`], 'the link\'s studs and rod ends', 'the studs', RULE.beam, RULE.beam),
  // round 18: the maker's arm and its centre screw on its own servo's spline: the maker's part (SPEC ruling)
  row(exactRe(TW.spline), null, 'spline', (t) => [`twist servo arm ${t}`], (t) => [`twist servo arm ${t}`], 'the maker\'s arm on its own servo\'s spline (maker\'s part)', 'the maker\'s centre screw, supplied with the servo (maker\'s part)', MAKERS_PART_RULE, MAKERS_PART_RULE),
  row(exactRe(TW.frame), null, 'framed', (t) => [`twist servo ${t} frame lips`, `twist frame pins ${t}`], (t) => [`twist frame pins ${t}`, `twist frame pin bonds ${t}`], 'the frame\'s lips and pins', 'the pins in their bonded holes', RULE.beam, RULE.bond),
  row(exactRe(TW.framePin), null, 'bonded pin', (t) => [`twist frame pins ${t}`], (t) => [`twist frame pin bonds ${t}`], 'the pins', 'their bonds', RULE.beam, RULE.bond),
  row(exactRe(TW.pinFace), null, 'resting', (t) => [`twist frame pins ${t}`], (t) => [`twist frame pin bonds ${t}`], 'the pins', 'their bonds', RULE.beam, RULE.bond),
  row(exactRe(TW.barSlide), null, 'through hole', (t) => [`twist bar ${t} in the joiner's plate`, `twist joiner ${t} plate`], (t) => [`twist bar ${t} rings`], 'the bar in the plate\'s hole', 'its rings', RULE.seat, RULE.bond),
  row(exactRe(TW.barHole), null, 'through hole', (t) => [`twist bar ${t} in the ribs`, `twist bar ${t} in the tip plate`], (t) => [`twist bar ${t} rings`], 'the bar in its hole', 'its rings', RULE.seat, RULE.bond),
  row(exactRe(TW.ringBar), null, 'bonded', (t) => [`twist bar ${t} rings`], (t) => [`twist bar ${t} rings`], 'the ring\'s bond', 'the same', RULE.bond),
  row(exactRe('bonded on the rod'), null, 'bonded', (t) => [`twist rod rings ${t}`], (t) => [`twist rod rings ${t}`], 'the ring\'s bond', 'the same', RULE.bond),
  row(exactRe(TW.stop), null, 'stop', (t) => [`wing ribs ${t} on their rings`, `twist rib bushings ${t} thrust`, `twist bar ${t} rings`, `twist rod rings ${t}`, ...ribRingsT(t)], (t) => [...ribRingsT(t), `twist bar ${t} rings`, `twist rod rings ${t}`], 'the face on its ring (crushing at half) or the bushing\'s end', 'the rings\' bonds', RULE.seat, RULE.bond),
  row(exactRe(TW.cap), null, 'bonded', (t) => [`twist spar cap ${t} bond`], (t) => [`twist spar cap ${t} bond`], 'the cap\'s bond', 'the same', RULE.bond),
  row(exactRe(TW.ribTip), null, 'resting', (t) => [`wing rib ${t} 10 on the tip plate`, `twist bar ${t} in the ribs`], (t) => [`twist bar ${t} rings`], 'rib 10 on the plate, the bar and the arm', 'the bar\'s tip ring', RULE.seat, RULE.bond),
  row(exactRe(TW.onSpar), null, 'resting', (t) => [`wing ribs ${t} on the inner spar`], (t) => [...ribRingsT(t), `wing ribs ${t} on their rings`], 'the rib on the spar (balsa crushing at half)', 'its rings', RULE.seat, RULE.bond),
  row(exactRe(TW.inRib), null, 'press fit', (t) => [`twist rib bushings ${t} in the ribs`], (t) => [`twist rib bushings ${t} thrust`, ...ribRingsT(t)], 'the bushing in its rib (balsa crushing at half)', 'the rib between its rings', RULE.seat, RULE.bond),
  row(exactRe(TW.slot), null, 'sliding fit', (t) => [`twist bar ${t} in the ribs`], (t) => [...ribRingsT(t), `twist bar ${t} rings`], 'the bar on the slot (balsa crushing at half)', 'the rib\'s rings; the bar\'s rings', RULE.seat, RULE.bond),
];

/** The drawn chain's ratio (the flight model's tip twist per servo turn, at the four-bar's centre) and its travel. */
export function twistChainFigures(pd, catalog) {
  const P = twistParams(pd.params), { G } = twistContext(pd, catalog, P), K = twistKinematics(G), L = span(pd.wing);
  const kq = (L - G.sE) / (G.sT - G.sE), tr = servoTravel(G, P);
  return { ratio: K.ratio(0) * kq, rod_ratio: K.ratio(0), kq, travel_rad: tr * kq, servo_travel_rad: (P.servo.travel_deg * Math.PI) / 180, G, K, P };
}

/** The wing's point masses of the drive (per wing, kg before the margin), each at its drawn centroid; those that turn with the hand wing flagged. */
export function twistPoints(tw, catalog) {
  const P = tw.P, rho = (id) => byId(catalog, id).specs.density_g_cm3 * 1e6, out = [];
  const turns = new Set(['twist bars', 'twist tip arms', 'twist tip plates', 'twist tip hubs', P.ribBushing]);
  for (const [tag, b] of Object.entries(tw.by)) {
    let kg = null;
    const mach = MACHINED.find(([t]) => t === tag), stock = STOCK.find(([t]) => t === tag);
    if (mach) kg = (b.V_m3 * rho(P[mach[1]])) / 1000;
    else if (stock) kg = (tubeSpec(byId(catalog, P[stock[1]])).g_per_m * b.L_m) / 1000;
    else if ([P.rodBushing, P.ribBushing, P.rodEnd, P.horn, P.screw].includes(tag)) kg = (byId(catalog, tag).mass_g * b.items) / 1000;
    if (kg) out.push({ name: tag, s_m: b.c[1], mass_kg: kg, d_m: b.c[0], ...(turns.has(tag) ? { twists: true } : {}) });
  }
  return out;
}

// ---- the section record ---------------------------------------------------------------------------------------------

/** The wing's parts that turn with the hand wing (checked against the section at zero twist only: they turn with it). */
export const TURNING_RE = /^(rib [RL] \d+|twist (bar|rib bushing|tip arm|tip plate|tip hub|tip pin)|leading edge rod|trailing edge rod)( |$)/;
/** The wing's parts on the skin (they carry the covering: touching it allowed). */
export const ON_SKIN_RE = /^(rib [RL] \d+|leading edge rod|trailing edge rod|twist tip plate)/;

/**
 * Round 17b (audit item 10): every solid of one wing (drawn unturned) against the real E387 (section.js), with the
 * spar axis sparN above the chord line. A solid's points (its section's corners or a capsule's extremes, at its x
 * range's ends and middle) must lie inside the section by the running clearance (0 for the ribs, rods and tip plate,
 * which carry the covering); a part fixed in the hand wing (the spar, the rod, the joiner's outboard end, the caps) is
 * checked against the covering turned by its own twist there (the ribs' angles interpolated) at both ends of the drawn
 * travel. A part that stands out must be declared in decl.protrusions ({ re, why, Cd }), its frontal area (the height
 * out times its span) and CdA counted; an undeclared one fails the record. -> { rows, protrusions, undeclared, passed }.
 */
export function sectionRecord(d, catalog, decl = {}, { required = 0.0005 } = {}) {
  const P = twistParams(d.params);
  if (!P) return null;
  const { parts } = drawnLines(d, catalog), ctx = twistContext(d, catalog, P), { G, surf } = ctx, wing = d.wing, K = twistKinematics(G);
  const ribAng = (th) => ctx.ribs.map((s, k) => (k <= G.k5 ? 0 : k === ctx.ribs.length - 1 ? th : K.law((s - G.sE) / (G.sT - G.sE), th)));
  const cov = (s, th) => { const a = ribAng(th), r = ctx.ribs; if (s <= r[0]) return 0; for (let k = 0; k + 1 < r.length; k++) if (s <= r[k + 1]) return a[k] + ((s - r[k]) / (r[k + 1] - r[k])) * (a[k + 1] - a[k]); return a.at(-1); };
  const thMax = servoTravel(G, P), rows = [], protrusions = [], undeclared = [];
  const gapAt = (s, n, x, ph) => { const X = x - wing.spar_x, Xr = X * Math.cos(ph) + n * Math.sin(ph), nr = -X * Math.sin(ph) + n * Math.cos(ph), q = sectionNAt(wing, s, wing.spar_x + Xr, surf, { sparN: G.sparN }); return Math.min(q.hi - nr, nr - q.lo); };
  for (const p of parts) {
    const turning = TURNING_RE.test(p.id), onSkin = ON_SKIN_RE.test(p.id), need = onSkin ? -1e-6 : required;
    const { worst, at, frontal } = partGap(p, turning ? [0] : [0, thMax, -thMax], (s, th) => (turning ? 0 : cov(s, th)), gapAt);
    rows.push({ part: p.id, gap_mm: +(worst * 1000).toFixed(3), required_mm: +(need * 1000).toFixed(3), at, ...(onSkin ? { on_skin: true } : {}), ...(turning ? { turning: true } : {}) });
    if (worst >= need) continue;
    const dd = (decl.protrusions ?? []).find((q) => new RegExp(q.re).test(p.id));
    protrusions.push({ part: p.id, out_mm: +(-worst * 1000).toFixed(3), frontal_mm2: +(frontal * 1e6).toFixed(2), ...(dd ? { declared: true, why: dd.why, Cd: dd.Cd, CdA_m2: dd.Cd * frontal } : { declared: false }) });
    if (!dd) undeclared.push(p.id);
  }
  return {
    rule: 'round 17b (audit-v24 item 10): every part of the wing inside the real E387 (NASA TM-4062, airfoil-e387-nasa-tm4062) with the spar axis sparN above the chord line, by the running clearance (on the skin for the ribs, rods and tip plate); a part fixed in the hand wing at both ends of the drawn twist travel; anything out declared with its frontal area and CdA; src/design/twist-drive.js sectionRecord',
    sparN_mm: P.sparN_mm, travel_rad: +thMax.toFixed(4), rows, protrusions, undeclared, CdA_m2: protrusions.reduce((a, q) => a + (q.CdA_m2 ?? 0), 0), passed: undeclared.length === 0,
  };
}
/** A part's smallest gap to the section over its solids' points and the twists th (the covering turned by ph(s, th)), where, and its frontal area out. */
function partGap(p, twists, ph, gapAt) {
  let worst = Infinity, at = null, frontal = 0;
  for (const sol of p.solids) {
    const sh = sol.at ? sol.at(null) : sol.shape, pts3 = sol.cl ? clPoints(sol.cl) : shapePoints(sh).flatMap(([s, n]) => [sol.x[0], (sol.x[0] + sol.x[1]) / 2, sol.x[1]].map((x) => [s, n, x]));
    let solWorst = Infinity;
    for (const th of twists) for (const [s, n, x] of pts3) {
      const g = gapAt(s, n, x, ph(s, th));
      if (g < solWorst) solWorst = g;
      if (g < worst) { worst = g; at = { solid: sol.name, s_mm: +(s * 1000).toFixed(1), x_mm: +(x * 1000).toFixed(1), n_mm: +(n * 1000).toFixed(2), twist_rad: +th.toFixed(3) }; }
    }
    if (solWorst < 0) frontal += -solWorst * spanOf(pts3);
  }
  return { worst, at, frontal };
}
function shapePoints(sh) {
  if (sh.kind === 'poly') return sh.pts;
  if (sh.kind === 'ring') return [0, 1, 2, 3, 4, 5, 6, 7].map((k) => [sh.c[0] + sh.ro * Math.cos((k * Math.PI) / 4), sh.c[1] + sh.ro * Math.sin((k * Math.PI) / 4)]);
  return [sh.a, sh.b].flatMap((c) => [0, 1, 2, 3, 4, 5, 6, 7].map((k) => [c[0] + sh.r * Math.cos((k * Math.PI) / 4), c[1] + sh.r * Math.sin((k * Math.PI) / 4)]));
}
const spanOf = (pts) => Math.max(...pts.map((p) => p[0])) - Math.min(...pts.map((p) => p[0]));
/** A span tube's surface points [s, n, x]: its circle round the centreline at both ends. */
function clPoints(cl) {
  return [0, 1].flatMap((e) => Array.from({ length: 16 }, (_, k) => { const a = (k * Math.PI) / 8; return [cl.s[e], cl.n[e] + cl.r * Math.sin(a), cl.x[e] + cl.r * Math.cos(a)]; }));
}

// ---- declarations: the sequence, the containment, the section ---------------------------------------------------------

/**
 * Round 17's sequence inserts (round17.mjs sequence17): each wing, its drive put together away from the bird, goes on
 * as one piece along the span (round 16's 'wing R' / 'wing L', now with the drive's parts).
 */
export const TWIST_SEQUENCE = ['R', 'L'].map((t) => ({ replace: `wing ${t}`, steps: [{ name: `wing ${t}`, members: [`^(inner spar|outer spar|leading edge rod|trailing edge rod|rib) ${t}( |$)`, `^twist [a-z ]+ ${t}( |$)`], dir: [0, 0, t === 'R' ? 1 : -1], dist_m: 1.1, why: 'the wing\'s structure and its twist drive one piece (put together away from the bird), its inner spar into the rocker\'s socket along the span' }] }));

/** The pod containment's declaration of the drive (outside the pod, in the wing: its drag is the wing's own; the section record holds it inside the E387). */
export const TWIST_CONTAINMENT = { outside: [{ name: 'the twist drive', re: '^twist [a-z ]+ [RL]( |$)', drag: false, why: 'the wing twist drive, outboard of the wing roots inside the wing\'s own section (the section record, twist-drive.js, holds it inside the E387 or declares it); its drag is the wing\'s' }] };

/** What stands out of the E387 (the section record finds them; each declared here with its reason and drag coefficient). */
export const TWIST_SECTION = {
  Cd_basis: 'the containment record\'s Cd for outside fittings (1.2, round 15): a stated assumption, not a measured figure',
  protrusions: [
    { re: '^twist servo arm [RL]$', Cd: 1.2, why: 'the R-ML25 (the maker\'s arm, 22 mm from the spline) at the top of the wing between ribs 5 and 6, under the covering as it twists there' },
    { re: '^twist rod end [RL] servo$', Cd: 1.2, why: 'the servo arm\'s rod end, beside it' },
    { re: '^twist joiner [RL]$', Cd: 1.2, why: 'the servo frame\'s aft lip at the bottom of the wing at the largest nose-down twist (the covering between ribs 5 and 6 turns onto it)' },
    { re: '^twist frame pins [RL]$', Cd: 1.2, why: 'the frame\'s under pins, beside the lip' },
    { re: '^(leading|trailing) edge rod [RL]$', Cd: 1.2, why: 'the edge rods are the section\'s edges: the 1.5 mm trailing edge rod against the E387 design\'s 0.0017 c trailing edge, the 2 mm leading edge rod in the tip\'s nose' },
  ],
};

/** The twist servo's support rows (clearance.js supportRecord): its case in the frame on the joiner, measured on the drawn solids (the case's side face against the frame's strip). */
export function twistSupportRows(d, catalog, { tol = 1e-6 } = {}) {
  const P = twistParams(d.params);
  if (!P) return [];
  const { parts } = drawnLines(d, catalog), sol = (id, nm) => parts.find((p) => p.id === id)?.solids.find((s) => s.name === nm);
  const cs = sol('twist servo R', 'case'), st = sol('twist joiner R', 'hanger strip'), nOf = (s) => (s ? s.at(null).pts.map((q) => q[1]) : null);
  const gap = cs && st ? Math.min(...nOf(st)) - Math.max(...nOf(cs)) : null;
  return ['R', 'L'].map((t) => ({ part: `twist servo ${t}`, carrier: `twist joiner ${t}`, face: 'the frame\'s strip on the case\'s side face, its lips on the case\'s ends', fixing: `four ${P.hanger.pin_d_mm} mm 7075 pins bonded in the lips (two across its end faces, two under it)`, gap_mm: gap === null ? null : +(gap * 1000).toFixed(4), passed: gap !== null && Math.abs(gap) <= tol }));
}

// ---- the chain record -------------------------------------------------------------------------------------------------

/** The flown twist demands a design records (its controller and envelope), or nulls with the reason. */
function flownDemands(d, opts) {
  const c = d.control ?? null;
  return { torque: opts.twistTorque_Nm ?? flownTorque(c), travel: opts.travel_rad ?? flownTravel(c), rate: opts.twistRate_rad_s ?? flownRate(c), basis: opts.basis ?? demandBasis(c) };
}
/** The largest twist torque the controller's envelope and roll limit record (N m at the wing), or null. */
function flownTorque(c) {
  if (!c) return null;
  const env = (c.envelope ?? []).map((e) => e.edge_peaks?.twistTorque).filter(Number.isFinite);
  const dl = c.diffLimit, diffT = dl && Number.isFinite(c.diffMax) ? dl.twist_torque_trim_Nm + dl.twist_torque_per_rad * c.diffMax : null;
  return env.length || diffT !== null ? Math.max(...env, diffT ?? 0) : null;
}
/** The largest tip twist the controller can command: |sym| + the dynamic schedule's max + diffMax (rad), or null. */
function flownTravel(c) {
  if (!c) return null;
  const trims = c.trim ?? [], sym = Math.max(0, ...trims.map((t) => Math.abs(t.sym ?? 0))), dyn = c.dynSchedule?.max ?? Math.max(0, ...trims.map((t) => t.dyn ?? 0));
  return sym + dyn + (c.diffMax ?? 0);
}
/** The trims' peak twist rate, dyn x 2 pi f (rad/s), or null. */
function flownRate(c) {
  return c?.trim?.length ? Math.max(...c.trim.map((t) => Math.abs(t.dyn ?? 0) * 2 * Math.PI * (t.freq ?? 0))) : null;
}
const demandBasis = (c) => (c ? 'the design\'s own controller and envelope (control.envelope edge peaks, control.diffLimit at diffMax; |sym| + the dynamic schedule\'s max + diffMax; the trims\' dyn x 2 pi f)' : 'no flown record in this design (no control block): demands not stated');

/**
 * The twist chains (one per wing) in the lead's chains.js format: the actuator, the path of joined parts from the servo
 * to the tip rib, and the figures the flight model flies against what the drawn chain delivers. opts may state the
 * flown demands (twistTorque_Nm, twistRate_rad_s, travel_rad, basis) when the design carries none.
 */
export function twistChains(asm, d, catalog, opts = {}) {
  const P = twistParams(d.params);
  if (!P) return [];
  const F = twistChainFigures(d, catalog), { G, K } = F, wing = d.wing, L = span(wing), el = wing.segments[0].length, sv = d.servos?.twist ?? {};
  const dem = flownDemands(d, opts), law = lawError(F, G, K, wing), comp = chainCompliance(d, catalog, F);
  const tipC = chordAtW(wing, L).c, axisErr = G.sparN * Math.sin(F.travel_rad);
  const ratios = Array.from({ length: 61 }, (_, i) => K.ratio(-F.servo_travel_rad + (2 * F.servo_travel_rad * i) / 60));
  const sides = [['R', 'right'], ['L', 'left']];
  return sides.map(([t]) => {
    const path = ['twist servo', 'twist servo arm', 'twist rod end # servo', 'twist link', 'twist rod end # lever', 'twist elbow lever', 'twist torque rod', 'twist tip hub', 'twist tip arm', 'twist tip plate'].map((n) => (n.includes('#') ? n.replace('#', t) : `${n} ${t}`));
    path.push(`rib ${t} 10`);
    const findings = chainFindings(dem, sv, F, law, comp);
    return {
      name: `twist ${t}`, actuator: { part: `twist servo ${t}`, bom: d.params.parts.twistServo, output: 'spline' }, path,
      also: { bar: [`twist tip plate ${t}`, `twist bar ${t}`, ...[6, 7, 8, 9].map((k) => `rib ${t} ${k}`)] },
      figures: {
        axis: { flown: { x_m: wing.spar_x, n_m: 0 }, drawn: { x_m: wing.spar_x, n_m: +G.sparN.toFixed(4) }, err: +axisErr.toFixed(5), tol: +(0.02 * tipC).toFixed(4), units: 'm: err is the section\'s largest displacement from the flown one at the full drawn travel, sparN sin(travel)', basis: `the strip model turns each section about its chord line at spar_x; the drawn pivot (the spar axis, x = spar_x exactly) is sparN above it, which moves a section at most sparN sin(travel) = ${(axisErr * 1000).toFixed(2)} mm (${(100 * axisErr / tipC).toFixed(2)}% of the tip chord): the tolerance is 2% of the tip chord` },
        ratio: { flown: sv.linkage_ratio ?? null, drawn: +F.ratio.toFixed(5), tol: 0.005, units: '-', basis: `the four-bar at its centre (${F.rod_ratio.toFixed(4)}) times the rod-to-tip factor ${F.kq.toFixed(4)}; over the servo's +-${(F.servo_travel_rad * 180 / Math.PI).toFixed(0)} deg it runs ${Math.min(...ratios).toFixed(3)}-${Math.max(...ratios).toFixed(3)} (the flight model holds the centre's: the larger torque at the wing)` },
        travel: { required_rad: dem.travel, delivered_rad: +F.travel_rad.toFixed(4), basis: `the four-bar solved at the servo's +-${(F.servo_travel_rad * 180 / Math.PI).toFixed(0)} deg; required: ${dem.basis}` },
        torque: { required_Nm: dem.torque, available_Nm: sv.stall_Nm ?? null, row: { V: sv.volts ?? null, stall_Nm: sv.servo_stall_Nm ?? null, speed_rad_s: sv.servo_speed_rad_s ?? null, basis: 'the MD89MW\'s published rows at the regulator\'s lowest output (build.js regulatorServoSupply), between the 4.8 and 6.0 V rows' }, basis: `the servo's stall over the drawn ratio (at the wing); required: ${dem.basis}` },
        speed: { required_rad_s: dem.rate, available_rad_s: sv.speed_rad_s ?? null, basis: 'the servo\'s no-load speed at its row times the drawn ratio; required: the trims\' dyn x 2 pi f' },
        law: { flown: 'w(s) th: w linear from 0 at the elbow to 1 at the tip', drawn: 'ribs 6-9: atan(xi sin th_r / (1 - xi (1 - cos th_r))) at xi of the bar from the joiner\'s hole to the tip plate\'s; rib 10: th_r', max_err: +law.max_err.toFixed(5), tol: +law.tol.toFixed(5), basis: `at the ribs over the full drawn travel; the tolerance is 2% of the drawn travel (the bar's own compliance is reported apart)` },
        stiffness: { flown: wing.twist_stiffness_Nm_rad ?? null, drawn: +twistStiffness(d, catalog).K_Nm_rad.toFixed(5), tol: 1e-3, rel: true, units: 'N m/rad', basis: 'the edge rods\' bending energy (twistStiffness)' },
        // round 18: the chain's series compliance as a drawn-against-flown figure like its siblings (was a bare number,
        // reported as info): what flight.js flies (servos.twist.compliance_rad_per_Nm) must be the drawn chain's.
        compliance_rad_per_Nm: { flown: sv.compliance_rad_per_Nm ?? null, drawn: +comp.rad_per_Nm.toFixed(6), tol: 1e-3, rel: true, units: 'rad/N m', basis: `the titanium rod in torsion (${comp.rod.toFixed(4)}) plus the bar in bending (${comp.bar.toFixed(4)}) (chainCompliance); flown as a relaxed deflection state, flight.js TAU_CHAIN_S` },
      },
      findings,
      flown_limits: ['travel', 'torque', 'speed'],
    };
  });
}

/** What the chain cannot deliver against the flown demands, with numbers; and what the flight model does not model. */
function chainFindings(dem, sv, F, law, comp) {
  const out = [];
  if (dem.torque !== null && dem.torque > sv.stall_Nm) out.push(`torque: the flown peak twist torque ${dem.torque.toFixed(3)} N m at the wing exceeds the MD89MW's ${sv.stall_Nm.toFixed(3)} N m (its ${(sv.servo_stall_Nm ?? 0).toFixed(3)} N m stall at ${sv.volts} V through ${F.ratio.toFixed(3)}:1)`);
  if (dem.rate !== null && dem.rate > sv.speed_rad_s) out.push(`speed: the flown schedule's peak twist rate ${dem.rate.toFixed(2)} rad/s exceeds the servo's no-load ${sv.speed_rad_s.toFixed(2)} rad/s at the wing (the twist lags; the flight model simulates it)`);
  if (dem.travel !== null && dem.travel > F.travel_rad) out.push(`travel: the controller can command ${dem.travel.toFixed(3)} rad of tip twist; the chain delivers +-${F.travel_rad.toFixed(3)} rad at the servo's +-${(F.servo_travel_rad * 180 / Math.PI).toFixed(0)} deg (the most its published 60 deg speed figure states; no travel is published); the flight model has no travel limit`);
  if (law.max_err > law.tol) out.push(`law: the ribs' twist differs from the flown w th by up to ${law.max_err.toFixed(4)} rad (tolerance ${law.tol.toFixed(4)})`);
  const q = dem.torque;
  out.push(q !== null ? `compliance: at the flown peak twist torque the chain winds up ${(q * comp.rad_per_Nm).toFixed(3)} rad at the tip (the titanium rod in torsion ${(q * comp.rod).toFixed(3)}, the bar in bending ${(q * comp.bar).toFixed(3)}); the flight model's twist is rigid behind the servo (series compliance ${(1 / comp.rad_per_Nm).toFixed(2)} N m/rad not modelled)` : `compliance: the chain's series compliance is ${comp.rad_per_Nm.toFixed(4)} rad per N m of twist torque at the tip (the rod ${comp.rod.toFixed(4)}, the bar ${comp.bar.toFixed(4)}), flown (flight.js winds the wing off the servo's position by it, servos.twist.compliance_rad_per_Nm)`);
  out.push(`covering: between rib 5 (fixed) and rib 6 the film interpolates the ribs' twist, ${law.cov_err.toFixed(4)} rad from the flown w th at the elbow at full travel`);
  // round 17b (lead): the torque-speed saturation, the travel stop and the compliance are flown by the flight model
  // (flight.js), and the covering's interpolation is a note: reported, not refusing; the law's error refuses
  return out.map((x) => (/^(torque|speed|travel|compliance|covering):/.test(x) ? { text: x, flown: true } : x));
}

/** The delivered law's largest error at the ribs over the travel, and the film's between rib 5 and rib 6 at the elbow. */
function lawError(F, G, K, wing) {
  const L = span(wing), el = wing.segments[0].length, w = (s) => Math.max(0, (s - el) / (L - el)), xi = (s) => (s - G.sE) / (G.sT - G.sE);
  let max_err = 0;
  for (let i = 0; i <= 40; i++) {
    const thr = -F.travel_rad / F.kq + (2 * F.travel_rad / F.kq) * (i / 40), th = thr * F.kq;
    G.ribs.forEach((s, k) => { if (k <= G.k5) return; const ph = k === G.ribs.length - 1 ? thr : K.law(xi(s), thr); max_err = Math.max(max_err, Math.abs(ph - w(s) * th)); });
  }
  const thr = F.travel_rad / F.kq, s6 = G.ribs[G.k5 + 1], s5 = G.ribs[G.k5], ph6 = K.law(xi(s6), thr), cov = ((el - s5) / (s6 - s5)) * ph6;
  return { max_err, tol: 0.02 * F.travel_rad, cov_err: Math.abs(cov) };
}

/**
 * The chain's series compliance at the tip (rad per N m of the flight model's twist torque): the titanium rod in torsion
 * (G from its published E and an assumed Poisson's ratio 0.3: no shear modulus is published), the bar's bending under
 * the ribs' forces (its published modulus) over r_min.
 */
export function chainCompliance(d, catalog, F = twistChainFigures(d, catalog)) {
  const { G } = F, Ld = twistLoads(d, catalog), ti = byId(catalog, d.params.parts.twistRod).specs, E = ti.elastic_modulus_gpa * 1e9, Gm = E / (2 * 1.3);
  const J = (Math.PI * ((2 * G.rod.ro) ** 4 - (2 * G.rod.ri) ** 4)) / 32, Lr = G.sH0 - G.h1, rod = (Ld.kTr * Lr) / (Gm * J) * Ld.kq;
  const bp = tubeSpec(byId(catalog, twistParams(d.params).bar)), EI = tubeAllowable(byId(catalog, twistParams(d.params).bar)).E_GPa * 1e9 * (Math.PI * ((bp.od_rate / 1000) ** 4 - (bp.id_rate / 1000) ** 4)) / 64;
  // the bar's deflection at each rib under the forces Fk (simply supported), over the rib's radius
  const Lb = G.sT - G.sE, xs = Ld.hand.slice(0, -1).map((s) => s - G.sE);
  const defl = (x) => Ld.Fk.reduce((acc, f, i) => { const a = xs[i], b = Lb - a, xx = x <= a ? x : Lb - x, aa = x <= a ? b : a; return acc + (f * aa * xx * (Lb * Lb - aa * aa - xx * xx)) / (6 * EI * Lb); }, 0);
  const bar = Math.max(...xs.map((x) => defl(x) / Ld.rMin));
  return { rod, bar, rad_per_Nm: rod + bar, G_basis: `G = E / 2.6 (Poisson 0.3 assumed) = ${(Gm / 1e9).toFixed(1)} GPa` };
}
