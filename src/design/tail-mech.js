// The all-moving V-tail's actuation (round 17, SPEC "Function completeness"; audit items 2 and 11): the flight model
// flies each V surface all-moving about its quarter chord (flight.js, tail.deflection_max_rad), so each surface turns
// on a pivot tube in two flanged bushings in a saddle bonded on the boom's end, and its servo on the aft tray drives it
// through the maker's arm, a rod end, a pushrod, a rod end and a horn bonded on the pivot tube.
//  - Frame: as wing-solids.js tailParts, each surface from its root on the boom's side o = (+-tail.z_m, boom y) along
//    its dihedral u, n along v (normal to the surface), x the body x; the pivot axis is the surface's unswept quarter
//    chord at x = tail.x_m, n = 0.
//  - Order along the pivot axis from the boom: a DIN 6799 retaining washer in a groove at the pivot pin's inboard end, the
//    inner bushing's flange, the saddle's boss with a bushing pressed in each end, the outer bushing's flange, the horn's
//    hub (bonded on the pin), the pivot tube (bonded on the pin's outboard end) with the ribs on it. The washer and the
//    horn hold the pin axially between the two flanges. The pin is machined from catalogue 7075-T6 (a groove in the
//    pultruded tube would cut its fibres); the washer goes on radially, after the surface is in its bushings.
//  - Drawn at zero deflection; the chain record (chains.js) checks the travel.
import { S, cap, poly, ring, RIB_HOLE, BORE, THREAD, THROUGH, screwY, xSlabs } from './clearance.js';
import { byId, tubeSpec, tubeAllowable } from './catalog.js';
import { fastenerGeometry, shapeArea } from './completeness.js';
import { E387_THICKNESS } from '../../sims/avis-shared/model.js';
import { servoArmMaker } from './chains.js'; // round 18: the maker's-part condition
import { MAKERS_PART_RULE } from './interfaces.js'; // round 18
import { podGeometry, podParams, sectionAt, loftGap, polarOf, insetPoint } from './pod-structure.js'; // round 19: the exit guides

const mm = (v) => v / 1000;
const add2 = (p, q, k = 1) => [p[0] + k * q[0], p[1] + k * q[1]];
const rect = (o, u, v, [s0, s1], [n0, n1]) => { const P = (s, n) => [o[0] + s * u[0] + n * v[0], o[1] + s * u[1] + n * v[1]]; return poly([P(s0, n0), P(s1, n0), P(s1, n1), P(s0, n1)]); };
const boxZY = (z0, z1, y0, y1) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);

/** Default mechanism parameters (mm unless stated); params.tailMech overrides them. */
export const TAIL_MECH = {
  // the pivot tube in torsion needs a published shear strength: Rock West's T-RND-236 (6 x 5 mm) publishes its XY shear
  // and GXY (CLT, reference only), CST's pultrusions publish none
  // round 18 (sized to the FoS rule, the CG's lever: each gram off here is 0.95 g m forward): the pin 5 mm in two
  // GFM-0506-05 (was 6 mm in GFM-0608-06; the DIN 6799 size 4 is the washer for 5 mm shafts), the saddle's sleeve 13 mm
  // (twice the boss's half-width, so the webs land on it) with a 1.0 mm wall, its bosses kept at 11 mm (at 10 mm the horn
  // screw's 20 mm path from inboard meets the other side's inner flange), the horn's hub 7 x 6 mm (was 8 x 4: the longer hub
  // keeps the horn screw's path clear of the other side's flange) and its arm 3 x 5 mm (was 3 x 8: the screw's 2 mm
  // engagement, with the arm 1 mm past the screw's end as the hole rule wants), the pushrod's end plugs 5 mm
  pivot: 'tube-rockwest-pultruded-6x5', pin: 'metal-al-7075-t6', washer: 'washer-din6799-4', bushing: 'bushing-igus-GFM-0506-05', rodEnd: 'rodend-igus-KBRM-02', arm: 'horn-hitec-r-ml25',
  screw: 'screw-iso4762-m2x6-8.8', material: 'metal-al-6061-t6', pushrod: 'tube-cst-pultruded-5x4',
  saddle: { sleeve_len: 13, wall: 1.0, boss_wall: 1.0, web_t: 1.0, boss_len: 11, clear: 0.5 },
  // the horn's arm stands above the pivot (+v, side 1): the pushrod's rod end then comes onto it from above, as at the
  // servo's arm, without passing the horn's hub or the bushing's flange
  horn: { hub_od: 7, hub_len: 6, arm_t: 3, arm_depth: 5, radius: 13, side: 1 },
  arm_hole_mm: 16, spline_from_aft_mm: 14.5, plug_len: 5, rib_t: 2, pin_in_tube: 10, pin_bore: 4,
};

/**
 * The linkage's small-angle ratio (surface turn per servo turn): the servo arm's hole radius over the horn's (the arm
 * and the horn parallel at zero, the pushrod across both). P0: params.tailMech.
 */
export function tailLinkageRatio(P0) {
  const P = { ...TAIL_MECH, ...(P0 ?? {}), horn: { ...TAIL_MECH.horn, ...(P0?.horn ?? {}) } };
  return P.arm_hole_mm / P.horn.radius;
}

/**
 * The mechanism's geometry for one design, shared by the solids, the BOM, the members and the chain record (m).
 * T: design.tail; boom: { y, r }; P: TAIL_MECH merged with params.tailMech.
 */
export function tailMechGeometry(T, boom, P, catalog) {
  const bu = byId(catalog, P.bushing).specs, pv = tubeSpec(byId(catalog, P.pivot)), S1 = P.saddle, H = P.horn;
  const rT = mm(pv.od_draw) / 2, d2 = mm(bu.d2_mm), d3 = mm(bu.d3_mm), bF = mm(bu.b2_mm), bL = mm(bu.b1_mm); // b1 includes the flange
  const rBoss = d2 / 2 + mm(S1.boss_wall), rb = boom.r + mm(S1.wall), run = mm(S1.clear); // clear of the saddle's sleeve
  const u0 = [Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)];
  // the first s along the axis at which a capsule of radius r centred on it lies `g` clear of the boom's circle
  // the pivot axis starts at the boom's side (T.pivot_z_m, round 17: the flown root is the covered root, past the saddle)
  const z0 = T.pivot_z_m ?? T.z_m, dist = (s) => Math.hypot(z0 + s * u0[0], s * u0[1]);
  const clearAt = (r, g) => { let s = 0; while (dist(s) < rb + r + g) s += 1e-5; return s; };
  // the pin's inboard end (n past its groove), the washer, the inner flange, the boss, the outer flange, the horn's hub,
  // the tube on the pin, the root rib: each the first place clear of the boom
  // round 18: the pin runs in the bushings' bore (d1), not at the pivot tube's outside; its spigot (in the tube's bore)
  // no larger, so the pin is one plain turned diameter with a step down to the spigot, and the washer is the size for it
  const W = byId(catalog, P.washer).specs, rW = mm(W.outer_diameter_d3_mm) / 2, sW = mm(W.thickness_s_mm), nW = mm(W.min_groove_to_shaft_end_n_mm), rPin = mm(bu.d1_mm) / 2;
  const rSp = mm(pv.id_draw) / 2 - run / 10, wr = W.shaft_diameter_d1_range_mm;
  if (rSp > rPin + 1e-9) throw new Error(`tail pivot pin: ${P.bushing}'s ${bu.d1_mm} mm bore is smaller than the ${(rSp * 2000).toFixed(2)} mm spigot in the ${P.pivot}'s bore (the pin would need a shoulder the assembly does not draw)`);
  if (wr && !(bu.d1_mm >= wr[0] && bu.d1_mm <= wr[1])) throw new Error(`tail pivot pin: ${P.washer} is for ${wr[0]}-${wr[1]} mm shafts, not the ${bu.d1_mm} mm pin`);
  const sW0 = Math.max(clearAt(rW, run), clearAt(rPin, run) + nW), sP0 = sW0 - nW, sF1 = sW0 + sW;
  const sB0 = Math.max(sF1 + bF, clearAt(d3 / 2, run) + bF), sB1 = sB0 + mm(S1.boss_len), sF2 = sB1 + bF;
  const sH0 = sF2, sH1 = sH0 + mm(H.hub_len), sT0 = sH1, sP1 = sT0 + mm(P.pin_in_tube);
  const tRib = mm(P.rib_t), sRoot = sT0 + tRib / 2 + run;
  // round 17: with T.pivot_z_m stated, the flown surface starts at the root rib's inboard face (s0) and runs its flown
  // span from there; without it, from the pivot's origin (s0 0), as drawn before
  const s0 = T.pivot_z_m != null ? sRoot - tRib / 2 : 0, Ltip = s0 + T.span_each_m;
  return { rT, d2, d3, bF, bL, rBoss, rb, rW, sW, sW0, sP0, sP1, rPin, sB0, sB1, sF1, sF2, sH0, sH1, sT0, sRoot, tRib, run, s0, hornR: mm(H.radius), hornT: mm(H.arm_t), hornD: mm(H.arm_depth), hubR: mm(H.hub_od) / 2, xq: T.x_m, L: Ltip, ri: mm(pv.id_draw) / 2, groove_d: mm(W.groove_diameter_d2_mm), groove_m: mm(W.groove_width_m_mm) };
}

/**
 * Round 17: the flown tail's root where its covering starts (audit item 2's follow-up: the saddle's span is bare, so
 * the flown surface must not include it): the pivot axis keeps its origin at the boom's side (pivot_z_m), and the
 * flown root (z_m, y_m) moves out along the dihedral to the root rib's inboard face, the flown span and chords
 * unchanged from there. T0: the tail block (z_m the boom-side origin, y_m the boom's height). -> { z_m, y_m, pivot_z_m,
 * root_offset_m }.
 */
export function tailRootShift(T0, boomR, P0, catalog) {
  const P = { ...TAIL_MECH, ...(P0 ?? {}) }, T = { ...T0, pivot_z_m: T0.z_m };
  const G = tailMechGeometry(T, { y: T0.y_m, r: boomR }, P, catalog), off = G.s0;
  return { z_m: T0.z_m + off * Math.cos(T0.dihedral_rad), y_m: T0.y_m + off * Math.sin(T0.dihedral_rad), pivot_z_m: T0.z_m, root_offset_m: off };
}

/** The surface's frame: its root on the boom's side o, u along its dihedral, v normal to it (the (z, y) plane). */
export function tailFrame(T, yB, sg) {
  return { o: [sg * (T.pivot_z_m ?? T.z_m), yB], u: [sg * Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)], v: [-sg * Math.sin(T.dihedral_rad), Math.cos(T.dihedral_rad)] };
}

/**
 * The saddle's solids: a sleeve bonded on the boom's end and, per side, a block boss round the two bushings along the
 * pivot axis, wider in x than the washer and flanges on the axis inboard of it, so its two webs (plates fore and aft of
 * them) join it to the sleeve without crossing the axis.
 */
export function saddleSolids(T, yB, boomR, G, P) {
  const xq = G.xq, sleeve = [xq - mm(P.saddle.sleeve_len) / 2, xq + mm(P.saddle.sleeve_len) / 2], rS = boomR + mm(P.saddle.wall);
  const out = [S('sleeve', sleeve, ring([0, yB], rS, boomR))];
  for (const [tag, sg] of [['R', 1], ['L', -1]]) {
    const { o, u, v } = tailFrame(T, yB, sg);
    const xw = Math.max(G.hubR, G.d3 / 2, G.rW) + G.run, xb = xw + mm(P.saddle.web_t ?? P.saddle.wall), Ls = G.sB1 - G.sB0;
    // the boss's own material: its box less the bushings' bore along the axis (not drawn; area_m2 states what it adds)
    out.push({ ...S(`boss ${tag}`, [xq - xb, xq + xb], rect(o, u, v, [G.sB0, G.sB1], [-G.rBoss, G.rBoss])), area_m2: (Ls * 2 * G.rBoss * 2 * xb - Math.PI * (G.d2 / 2) ** 2 * Ls) / (2 * xb) });
    const a0 = Math.atan2(add2(o, u, G.sB0)[1] - yB, add2(o, u, G.sB0)[0]), sp = (k) => [rS * Math.cos(a0 + k), yB + rS * Math.sin(a0 + k)];
    const bp = (sx, n) => add2(add2(o, u, sx), v, n);
    const web = poly(hull2D([sp(-0.5), sp(0.5), bp(G.sB0, -G.rBoss), bp(G.sB0, G.rBoss), bp(G.sB1, G.rBoss), bp(G.sB1, -G.rBoss)]));
    for (const [nm, xr] of [['aft', [xq - xb, xq - xw]], ['fwd', [xq + xw, xq + xb]]]) out.push(S(`web ${tag} ${nm}`, xr, web));
  }
  return out;
}

/** The surface's chord at s along its span (the flown taper). */
const chordOf = (T, s, s0 = 0) => T.chord_root_m * (1 - (1 - T.taper) * ((s - s0) / T.span_each_m));

/**
 * The tail and its actuation as solids of the assembly (in place of round 16's tied tail): ax as wing-solids.js
 * tailParts's, with `servoBox(tag)` the packed tail servo's box (model.js) for its arm and rod end.
 */
export function tailMechParts(ax, servoBox) {
  const { d, catalog, af, add, join, assumptions } = ax;
  const T = d.tail, P = { ...TAIL_MECH, ...(d.params.tailMech ?? {}) }, yB = af.boom_y_m;
  const rod = byId(catalog, d.params.parts.teRod).specs, r = mm(rod.od_mm) / 2;
  const boomR = mm(byId(catalog, d.params.parts.keel).specs.od_mm) / 2;
  const G = tailMechGeometry(T, { y: yB, r: boomR }, P, catalog);
  const ke = byId(catalog, P.rodEnd).specs, arm = byId(catalog, P.arm).specs, sg0 = fastenerGeometry(byId(catalog, P.screw).specs ?? {}, P.screw);
  const xq = G.xq, ribs = (s0) => Array.from({ length: 4 }, (_, k) => s0 + (k / 3) * (G.L - G.tRib / 2 - s0));
  // a tube along the pivot axis: long ones as the capsule round it, short ones (bushings, flanges, hubs) as the box
  // round them in (s, n) over x +-ro (each contains the cylinder it stands for); n0 offsets it from the axis
  const tube = (name, o, u, s0, s1, ro, ri = 0, n0 = 0, v = null) => {
    const oo = v ? add2(o, v, n0) : o, sh = s1 - s0 > 4 * ro ? cap(add2(oo, u, s0 + ro), add2(oo, u, s1 - ro), ro) : rect(oo, u, v ?? [-u[1], u[0]], [s0, s1], [-ro, ro]);
    return { ...S(name, [xq - ro, xq + ro], sh), tube: { ro, ri, L: s1 - s0, axis: 'span' } };
  };
  /**
   * Round 18.2 (the film's request): a screw head/shank's true section, N slabs across x (xSlabs) narrowing in n,
   * holding the full span [s0, s1] throughout (so the true length is never overshot). One shared tube record per
   * fastener, not per slab.
   */
  const slabs = (name, o, u, s0, s1, ro, n0 = 0, v = null) => { const oo = v ? add2(o, v, n0) : o; return xSlabs(xq, ro, (w) => rect(oo, u, v ?? [-u[1], u[0]], [s0, s1], [-w, w])).map((sl, i) => ({ ...S(`${name} ${i + 1}`, sl.x, sl.shape), tube: { ro, ri: 0, L: Math.abs(s1 - s0), axis: 'span' } })); };
  const saddle = saddleSolids(T, yB, boomR, G, P);
  const rows = [];
  for (const [tag, sg] of [['R', 1], ['L', -1]]) {
    const o = [sg * (T.pivot_z_m ?? T.z_m), yB], u = [sg * Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)], v = [-sg * Math.sin(T.dihedral_rad), Math.cos(T.dihedral_rad)];
    // the pivot pin (7075, grooved at its inboard end) through both bushings and the horn's hub into the tube's bore; the
    // pivot tube from the horn's hub to the tip (its ribs and trailing edge rod on it)
    const pinId = add(`tail pivot pin ${tag}`, [tube('pin', o, u, G.sP0, G.sT0, G.rPin), tube('spigot', o, u, G.sT0, G.sP1, G.ri - G.run / 10)], { bom: 'tail pivot pins' });
    const pid = add(`tail pivot tube ${tag}`, [tube('tube', o, u, G.sT0, G.L, G.rT, G.ri)], { bom: 'tail pivot tubes' });
    join(pinId, pid, 'its spigot bonded in the tube\'s bore'); join(`${pinId}/spigot`, `${pid}/tube`, BORE);
    const wid = add(`tail pin washer ${tag}`, [tube('washer', o, u, G.sW0, G.sW0 + G.sW, G.rW, G.groove_d / 2)], { bom: P.washer });
    join(wid, pinId, 'in its groove on the pin'); join(`${wid}/washer`, `${pinId}/pin`, BORE); join(wid, `tail bushing ${tag} inner`, 'against the flange (the pin\'s axial stop)');
    for (const [nm, s0, s1, dir] of [['inner', G.sB0 - G.bF, G.sB0 - G.bF + G.bL, 1], ['outer', G.sB1 + G.bF - G.bL, G.sB1 + G.bF, -1]]) {
      const fl = dir > 0 ? [s0, s0 + G.bF] : [s1 - G.bF, s1], body = dir > 0 ? [s0 + G.bF, s1] : [s0, s1 - G.bF];
      const bid = add(`tail bushing ${tag} ${nm}`, [tube('body', o, u, body[0], body[1], G.d2 / 2, G.rPin), tube('flange', o, u, fl[0], fl[1], G.d3 / 2, G.rPin)], { bom: P.bushing });
      join(bid, 'tail saddle', 'pressed in the housing'); join(bid, pinId, 'journal on its shaft');
      // the bores are not drawn (a tube along the span is drawn filled): the body in the boss's bore, the pin in both
      join(`${bid}/body`, `tail saddle/boss ${tag}`, BORE); join(`${bid}/body`, `${pinId}/pin`, BORE); join(`${bid}/flange`, `${pinId}/pin`, BORE);
    }
    // the horn: its hub bonded on the tube, its arm along -v to the rod end's screw at hornR
    const sHm = (G.sH0 + G.sH1) / 2;
    const hs = P.horn.side ?? -1, armN = hs > 0 ? [G.hubR - 1e-4, G.hornR + 3e-3] : [-(G.hornR + 3e-3), -G.hubR + 1e-4];
    const hid = add(`tail horn ${tag}`, [tube('hub', o, u, G.sH0, G.sH1, G.hubR, G.rPin), S('arm', [xq - G.hornD / 2, xq + G.hornD / 2], rect(o, u, v, [sHm - G.hornT / 2, sHm + G.hornT / 2], armN))], { bom: 'tail horns' });
    join(hid, pinId, 'bonded on the pin'); join(`${hid}/hub`, `${pinId}/pin`, BORE); join(hid, `tail bushing ${tag} outer`, 'its hub\'s face on the flange (the pin\'s axial stop)'); join(hid, pid, 'the tube\'s end on its hub');
    // the horn's rod end: its ball on the arm's inboard face at hornR (outboard, it would meet the root rib), its bore
    // along u; its shank forward along +x
    const sE = sHm - G.hornT / 2 - mm(ke.B_mm) / 2, nE = hs * G.hornR, D2 = mm(ke.d2_mm), Lsh = mm(ke.h1_mm);
    const eH = add(`tail rod end ${tag} horn`, [S('eye', [xq - D2 / 2, xq + D2 / 2], rect(o, u, v, [sE - mm(ke.B_mm) / 2, sE + mm(ke.B_mm) / 2], [nE - D2 / 2, nE + D2 / 2])), S('shank', [xq + D2 / 2, xq + Lsh], ring(add2(add2(o, u, sE), v, nE), mm(ke.d5_mm) / 2))], { bom: P.rodEnd });
    // its screw along u: the head inboard of the ball, the shank through the ball into the arm's tapped hole.
    // Round 18.2 (the film's request): the screw's true section, N slabs across x, not tube's flat box. Every
    // shank slab (not just one) needs its own THROUGH/THREAD join, since those override the part-level
    // descriptive joins (not themselves exempt) for exactly the pairs that overlap.
    const hornShankSlabs = slabs('shank', o, u, sE - mm(ke.B_mm) / 2, sE - mm(ke.B_mm) / 2 + sg0.L, sg0.d / 2, nE, v);
    const sid = add(`tail horn screw ${tag}`, [...slabs('head', o, u, sE - mm(ke.B_mm) / 2 - sg0.head_k, sE - mm(ke.B_mm) / 2, sg0.head_d / 2, nE, v), ...hornShankSlabs], { bom: P.screw });
    join(eH, hid, 'its ball on the horn\'s arm, on its screw'); join(sid, eH, 'through the ball'); join(sid, hid, 'screwed into the arm\'s tapped hole');
    for (const sl of hornShankSlabs) { join(`${sid}/${sl.name}`, hid, THREAD); join(`${sid}/${sl.name}`, eH, THROUGH); }
    // the servo's arm on its spline (assumed on the case's top face, on its centre line, spline_from_aft_mm from its
    // aft end: the maker publishes no spline position), pointing outboard; the servo's rod end on its hole
    const b = servoBox(tag);
    const xs = b.x0 + mm(P.spline_from_aft_mm), zs = (b.z0 + b.z1) / 2, yT = b.y1, armY = [yT + mm(arm.hub_boss_beyond_arm_mm), yT + mm(arm.hub_overall_height_mm)];
    const zH = zs + sg * mm(P.arm_hole_mm), zTip = zs + sg * (mm(arm.hole_radii_mm.at(-1)) + mm(arm.tip_od_mm) / 2);
    const aid = add(`tail servo arm ${tag}`, [S('hub', [xs - mm(arm.hub_od_mm) / 2, xs + mm(arm.hub_od_mm) / 2], boxZY(zs - mm(arm.hub_od_mm) / 2, zs + mm(arm.hub_od_mm) / 2, yT, armY[1])), S('arm', [xs - mm(arm.tip_od_mm) / 2, xs + mm(arm.tip_od_mm) / 2], boxZY(Math.min(zs, zTip), Math.max(zs, zTip), armY[0], armY[1]))], { bom: P.arm });
    join(aid, `packed servo_tail_${tag}`, 'on the servo\'s output spline (the maker\'s arm and its centre screw)');
    const eS = add(`tail rod end ${tag} servo`, [S('eye', [xs - D2 / 2, xs + D2 / 2], boxZY(zH - D2 / 2, zH + D2 / 2, armY[1], armY[1] + mm(ke.B_mm))), S('shank', [xs - Lsh, xs - D2 / 2], ring([zH, armY[1] + mm(ke.B_mm) / 2], mm(ke.d5_mm) / 2))], { bom: P.rodEnd });
    const { screw: ss } = screwY([xs, zH], armY[1] + mm(ke.B_mm), -1, sg0);
    const sid2 = add(`tail servo arm screw ${tag}`, ss, { bom: P.screw });
    // round 18.2: screwY's shank is now N true-section slabs; every one needs its own THROUGH/THREAD join (see
    // the horn screw's note above)
    join(eS, aid, 'its ball on the servo\'s arm, on its screw'); join(sid2, eS, 'through the ball'); join(sid2, aid, 'screwed into the arm\'s tapped hole');
    for (const sl of ss.filter((s) => /^shank/.test(s.name))) { join(`${sid2}/${sl.name}`, eS, THROUGH); join(`${sid2}/${sl.name}`, aid, THREAD); }
    // the pushrod between the two rod ends' shank ends, with an end plug bonded in each end, its stud in the rod end
    const A = { x: xs - Lsh, p: [zH, armY[1] + mm(ke.B_mm) / 2] }, B = { x: xq + Lsh, p: add2(add2(o, u, sE), v, nE) };
    // round 19 (decision 2, the pod's through-flow sealed): its exit guide, when the design states one and has a pod
    // (on the rod's true axis, ball to ball: the rod ends' shanks are drawn along x, so the drawn tube A-B lies up to
    // ~1 mm off it at the exit; the clearance check sees the drawn tube in the guide, the travel row the true one)
    const S0 = { x: xs, p: A.p }, H0 = { x: xq, p: B.p }, eg = exitGuideOf(d, catalog, S0, H0, P);
    const ts = tubeSpec(byId(catalog, P.pushrod)), pr = pushrodPieces(A, B, mm(ts.od_draw) / 2, mm(ts.id_draw) / 2, mm(P.plug_len), 0.02, ...guideZone(eg, S0, H0));
    // the direction its assembly step brings it in along (unit [x, y, z]): from forward and above, half as much along
    // the surface's normal on the horn's side as along x (nothing along the pivot axis), so the horn's rod end comes
    // along the arm's face and its shank from ahead of the arm, and the servo's comes onto its arm from above
    const sl = [1, 0.5 * hs * v[1], 0.5 * hs * v[0]], nsl = Math.hypot(...sl);
    const rodId = add(`tail pushrod ${tag}`, pr.solids, { bom: 'tail pushrods', slide: sl.map((q) => q / nsl) });
    for (const [end, piece, eye] of [['servo', pr.solids[0], eS], ['horn', pr.solids.at(-1), eH]]) {
      // the plug fills its end piece's bore (its centre that piece's), its M2 stud into the rod end's female thread
      const c = piece.shape.c, atA = end === 'servo', xe = atA ? A.x : B.x, dir = atA ? 1 : -1; // dir: from the tube's end toward its rod end
      const plug = add(`tail pushrod plug ${tag} ${end}`, [S('plug', piece.x, ring(c, pr.ri - 1e-5)), S('stud', dir > 0 ? [xe, xe + mm(ke.l1_mm)] : [xe - mm(ke.l1_mm), xe], ring(atA ? A.p : B.p, 1e-3))], { bom: 'tail pushrod end plugs' });
      join(plug, rodId, 'bonded in the tube\'s end'); join(plug, eye, 'its stud in the rod end\'s thread'); join(`${plug}/stud`, eye, THREAD);
      join(rodId, eye, 'its end on the rod end\'s shank');
    }
    if (eg) guideParts(ax, eg, { tag, o, u, v, sE, nE, xs, zs, zH, A }, G, P);
    // ribs on the pivot tube from the root rib (its face on the outer flange) to the tip, the trailing edge rod through them
    const rs = ribs(G.sRoot);
    const te = add(`tail trailing edge rod ${tag}`, rs.slice(0, -1).map((a, k) => {
      const bb = rs[k + 1], x0 = xq - 0.75 * chordOf(T, a, G.s0) + r, x1 = xq - 0.75 * chordOf(T, bb, G.s0) + r;
      return { ...S(`bay ${k + 1}`, [Math.min(x0, x1) - r, Math.max(x0, x1) + r], cap(add2(o, u, a + G.tRib / 2), add2(o, u, bb - G.tRib / 2), r)), tube: { ro: r, ri: 0, L: ((bb - a) * (G.L - G.sRoot + G.tRib / 2)) / (rs[3] - rs[0]), axis: 'span' } };
    }), { bom: 'tail trailing edge rods' });
    rs.forEach((s, k) => {
      const c = chordOf(T, s, G.s0), h = E387_THICKNESS * c;
      const id = add(`tail rib ${tag} ${k + 1}`, [{ ...S('rib', [xq - 0.75 * c + 2 * r, xq + 0.25 * c], rect(o, u, v, [s - G.tRib / 2, s + G.tRib / 2], [-h / 2, h / 2])), envelope: true, rib_t: G.tRib }], { bom: 'tail ribs' });
      join(id, pid, 'bonded on the tube'); join(id, pid, RIB_HOLE); join(id, te, RIB_HOLE); join(id, pinId, RIB_HOLE);
    });
    rows.push({ tag, o, u, v, sE, nE, xs, zs, zH, armY, A, B, pushrod_L: pr.L, ribs: rs, ...guideRow(eg) });
  }
  add('tail saddle', saddle, { bom: 'tail saddle' });
  join('tail saddle', 'tail boom', 'bonded on the boom\'s end');
  for (const t of ['R', 'L']) join('tail saddle', `tail pivot pin ${t}`, BORE);
  assumptions.push(`tail (round 17, params.tailMech, src/design/tail-mech.js): each V surface all-moving on its quarter chord: a 7075 pivot pin in two ${P.bushing} in a 6061 saddle bonded on the boom's end (bosses ${P.saddle.boss_len} mm long), held axially between a ${P.washer} in its groove and the horn's hub on the two flanges, the ${P.pivot} tube bonded on its outboard end; driven by its MD89MW through the maker's ${P.arm} (hole at ${P.arm_hole_mm} mm), two ${P.rodEnd} and a ${P.pushrod} pushrod with bonded 6061 end plugs to a 6061 horn (${P.horn.radius} mm) bonded on the pin; the servo's spline position on its case is not published, drawn on its top face's centre line ${P.spline_from_aft_mm} mm from its aft end; drawn at zero deflection (the travel is checked by the chain record)`);
  return { G, rows, P };
}

/** Round 19: the design's exit guide on this rod's true axis (S0 to H0), when it states one and has a pod; else null. */
const exitGuideOf = (d, catalog, S0, H0, P) => (P.guide && d.params.pod ? exitGuide(d, catalog, S0, H0, P) : null);
/** The pushrod's fine breaks and their line (pushrodPieces' last two arguments) for an exit guide eg, or none. */
const guideZone = (eg, S0, H0) => (eg ? [eg.rodBreaks, { A: S0, B: H0 }] : [[], null]);
/** The mechanism row's guide figures (its x range, true length, lug volume and leak), or nothing. */
const guideRow = (eg) => (eg ? { guide: { x: eg.x, L: eg.L, lug_mm3: eg.lug.volume_m3 * 1e9, leak_m2: eg.leak_m2 } } : {});

/**
 * Round 19: an exit guide's parts (tail-mech.js exitGuide): the guide (its pieces on the pushrod's own breaks, concentric
 * with its true-axis pieces there) and its lug on the former's aft face (the guide passes inside the former's band); the
 * film is closed round the guide where it crosses the skin (pod-structure.js holders). The guide states its seal: the
 * window it replaces, its leak, its clearance and the rod's excursion over the commanded travel (the covering record's
 * exit seal row).
 */
function guideParts({ d, add, join }, eg, row, G, P) {
  const tag = row.tag, exc = pushrodExcursion(row, G, d.tail.deflection_max_rad, eg.stations);
  const seal = { replaces: `tail pushrod exit ${tag}`, leak_m2: eg.leak_m2, leak_basis: eg.leak_basis, clearance_m: eg.clear_m, excursion_m: exc, travel_basis: `the pushrod's true line over the commanded travel (+-${(d.tail.deflection_max_rad * 180 / Math.PI).toFixed(1)} deg at the surface: the servo arm's and the horn's arcs, the rod's length held) against its drawn-zero line, at the guide's stations`, lug: `tail pushrod guide lug ${tag}`, lug_bond_mm2: P.guide.lug_bond.w_mm * P.guide.lug_bond.L_mm, former: eg.former };
  const gid = add(`tail pushrod guide ${tag}`, eg.solids, { bom: 'tail pushrod guides', slide: eg.dir, seal });
  const lid = add(`tail pushrod guide lug ${tag}`, [S('lug', eg.lug.x, eg.lug.shape)], { bom: 'tail pushrod guide lugs' });
  join(gid, lid, 'bonded in its lug\'s hole');
  join(lid, eg.former, 'bonded on the former\'s aft face');
}

/**
 * A straight pushrod from A to B (x and the (z, y) centre at each end) as x-pieces: an end piece `endLen` long at each
 * end (its plug's), the rest at most 20 mm; each a ring round the piece's mid-point that holds the tube's slope over
 * the piece (its outside grown by half the piece's offset; its bore drawn at the tube's own).
 */
export function pushrodPieces(A, B, ro, ri, endLen, piece = 0.02, fine = [], fineLine = null) {
  const dx = B.x - A.x, L = Math.hypot(dx, B.p[0] - A.p[0], B.p[1] - A.p[1]), Lx = Math.abs(dx), e = Math.min(endLen, Lx / 3) / Lx;
  // round 19: `fine` (x stations, m) are breaks of their own (the exit guide's, so its pieces and the rod's are
  // concentric); between the outermost of them every piece is one of theirs, elsewhere at most `piece` long
  const tf = [...new Set(fine.map((x) => +((x - A.x) / dx).toFixed(12)))].filter((t) => t > e + 1e-9 && t < 1 - e - 1e-9).sort((p, q) => p - q);
  const ts = pieceBreaks(e, tf, Lx, piece);
  const at0 = lineOf(A, B);
  // round 19: in the fine zone, on fineLine ({ A, B }: the rod's true axis, where it slides in its exit guide; the rod
  // ends' shanks are drawn along x, which puts the tube A-B elsewhere up to ~1.8 mm off that axis)
  const fl = fineLine && tf.length ? fineLine : null, inZone = (t0, t1) => !!fl && t0 >= tf[0] - 1e-12 && t1 <= tf.at(-1) + 1e-12;
  const atF = fl ? lineOf(fl.A, fl.B) : null, kF = fl ? Math.hypot(fl.B.x - fl.A.x, fl.B.p[0] - fl.A.p[0], fl.B.p[1] - fl.A.p[1]) / Math.abs(fl.B.x - fl.A.x) : 0;
  const solids = ts.slice(0, -1).map((t0, k) => {
    const t1 = ts[k + 1], x0 = A.x + t0 * dx, x1 = A.x + t1 * dx, z = inZone(t0, t1), at = z ? atF : at0;
    const c = at((x0 + x1) / 2), half = Math.hypot(...at(x1).map((v, i) => v - at(x0)[i])) / 2;
    return { ...S(`piece ${k + 1}`, [Math.min(x0, x1), Math.max(x0, x1)], ring(c, ro + half, ri)), tube: { ro, ri, L: z ? kF * Math.abs(x1 - x0) : L * (t1 - t0), axis: 'x' } };
  });
  return { solids, L, ri };
}

/** The point at x (m) on the straight line through a and b ({ x, p: [z, y] }). */
const lineOf = (a, b) => (x) => { const u = (x - a.x) / (b.x - a.x); return [a.p[0] + u * (b.p[0] - a.p[0]), a.p[1] + u * (b.p[1] - a.p[1])]; };

/** A pushrod's breaks (fractions of its x run): its two end pieces (e), the fine breaks tf as they are, the rest at most `piece` long. */
function pieceBreaks(e, tf, Lx, piece) {
  const fixed = [e, ...tf, 1 - e], ts = [0];
  for (let k = 0; k + 1 < fixed.length; k++) {
    const [t0, t1] = [fixed[k], fixed[k + 1]], inFine = tf.length > 0 && t0 >= tf[0] - 1e-12 && t1 <= tf.at(-1) + 1e-12;
    const n = inFine ? 1 : Math.max(1, Math.ceil((Lx * (t1 - t0)) / piece));
    for (let j = 0; j < n; j++) ts.push(t0 + (j / n) * (t1 - t0));
  }
  return [...ts, 1 - e, 1];
}

/**
 * Round 19 (decision 2: seal the pod's through-flow paths): a tail pushrod's exit guide, P.guide on the rod's true axis
 * from its servo ball S to its horn ball H ({ x, p: [z, y] }, m). A catalogue tube (g.tube) coaxial with the rod, the rod
 * sliding in its bore, from the lug's forward face (the support former's aft face, g.former) aft until its whole section
 * is g.out_mm outside the loft, so the film's cut round it closes on the guide; the lug (g.lug plate, one thickness) on
 * the former's aft face round it, bonded to the former's band and the guide bonded in its hole. Drawn at the maker's
 * tolerance (g.tol_mm on both diameters: the outside larger, the bore smaller), each piece g.piece_mm (the lug's
 * thickness first) holding the tube's slope as the pushrod's pieces do. -> { x: [aft, fwd], solids, rodBreaks (the
 * pushrod's own breaks: the guide's, g.piece_mm on for two pieces forward and g.sweep_mm aft, so the guide's assembly
 * slide over the rod meets concentric pieces), dir (unit [x, y, z] aft along the rod: the guide comes from aft), L (its
 * true length), lug: { x, shape, volume_m3 }, leak_m2 (the annulus: the largest bore against the rod's smallest
 * section), clear_m (the smallest radial clearance: the smallest bore against the rod's largest section), stations,
 * former (its id) }.
 */
export function exitGuide(d, catalog, Sb, Hb, P) {
  const g = P.guide, PG = podGeometry(d, catalog), f = PG.formers.find((q) => q.name === g.former);
  if (!f) throw new Error(`tail pushrod guide: no pod former '${g.former}'`);
  const ts = tubeSpec(byId(catalog, g.tube)), pl = byId(catalog, g.lug), tol = mm(g.tol_mm), loft = PG.loft;
  const rs = byId(catalog, P.pushrod).specs, rodMin = mm(Math.min(rs.od_mm, rs.od_mm_spec_sheet ?? Infinity)), rodMax = mm(Math.max(rs.od_mm, rs.od_mm_spec_sheet ?? 0));
  const dx = Hb.x - Sb.x, Lr = Math.hypot(dx, Hb.p[0] - Sb.p[0], Hb.p[1] - Sb.p[1]), kL = Lr / Math.abs(dx);
  const at = (x) => { const t = (x - Sb.x) / dx; return [Sb.p[0] + t * (Hb.p[0] - Sb.p[0]), Sb.p[1] + t * (Hb.p[1] - Sb.p[1])]; };
  const slope = Math.hypot(Hb.p[0] - Sb.p[0], Hb.p[1] - Sb.p[1]) / Math.abs(dx);
  const roT = mm(ts.od_draw) / 2, riT = mm(ts.id_draw) / 2, ro = roT + tol / 2, ri = riT - tol / 2, step = mm(g.piece_mm), tL = mm(pl.specs.thickness_mm_nominal);
  const ringPts = (c, r, n = 48) => Array.from({ length: n }, (_, k) => [c[0] + r * Math.cos((2 * Math.PI * k) / n), c[1] + r * Math.sin((2 * Math.PI * k) / n)]);
  const gaps = (x) => { const h = sectionAt(loft, x); return h ? ringPts(at(x), ro).map((p) => loftGap(h, p)) : null; };
  const xf = f.x[0], g0 = gaps(xf);
  if (!g0 || Math.min(...g0) < mm(g.out_mm)) throw new Error(`tail pushrod guide: not ${g.out_mm} mm inside the loft at the former ${g.former}`);
  const br = [xf, xf - tL];
  for (let gg = gaps(br.at(-1)); gg && Math.max(...gg) > -mm(g.out_mm); gg = gaps(br.at(-1))) {
    br.push(br.at(-1) - step);
    if (br.length > 400) throw new Error('tail pushrod guide: the rod never leaves the loft');
  }
  const solids = br.slice(0, -1).map((x1, k) => {
    const x0 = br[k + 1], half = (slope * (x1 - x0)) / 2;
    return { ...S(`piece ${k + 1}`, [x0, x1], ring(at((x0 + x1) / 2), ro + half, Math.max(ri - half, 0))), tube: { ro: roT, ri: riT, L: kL * (x1 - x0), axis: 'x' } };
  });
  const fwd = [1, 2].map((k) => xf + k * step), aft = Array.from({ length: Math.ceil(mm(g.sweep_mm) / step) + 2 }, (_, k) => br.at(-1) - (k + 1) * step);
  // the lug: the convex hull of a web round the guide's hole and two points on the former's band g.lug_band_deg either
  // side of the guide (g.lug_depth_mm in from the skin: under the film's chord between the cone's rods), on the smaller
  // of the loft's sections across it; its hole the guide's drawn outside over the lug, plus the bond's gap
  const ha = sectionAt(loft, xf - tL + 1e-9), hb = sectionAt(loft, xf - 1e-9), h = ha.hw * ha.hh <= hb.hw * hb.hh ? ha : hb;
  const c = at(xf - tL / 2), rh = ro + (slope * tL) / 2 + mm(g.bond_gap_mm), ac = polarOf(h, c);
  const band = [-1, 1].map((s) => insetPoint(h, (ac + s * g.lug_band_deg) * Math.PI / 180, mm(g.lug_depth_mm)));
  const pts = hull2D([...ringPts(c, rh + mm(g.web_mm), 32), ...band]);
  const area = Math.abs(pts.reduce((a, p, i) => a + p[0] * pts[(i + 1) % pts.length][1] - pts[(i + 1) % pts.length][0] * p[1], 0)) / 2 - Math.PI * rh * rh;
  const idMax = 2 * riT + tol, leak = (Math.PI / 4) * (idMax ** 2 - rodMin ** 2);
  return {
    x: [br.at(-1), xf], solids, rodBreaks: [...fwd, ...br, ...aft], stations: br, slope, former: f.id,
    dir: [dx / Lr, (Hb.p[1] - Sb.p[1]) / Lr, (Hb.p[0] - Sb.p[0]) / Lr].map((q) => +q.toFixed(9)), L: kL * (xf - br.at(-1)),
    lug: { x: [xf - tL, xf], shape: poly(pts, [{ c, r: rh }]), volume_m3: area * tL },
    leak_m2: leak, clear_m: (2 * riT - tol - rodMax) / 2,
    leak_basis: `the annulus between the guide's largest bore (${ts.id_draw} + ${g.tol_mm} mm, ${g.tube} at the maker's +-${g.tol_mm} mm) and the pushrod's smallest section (${(rodMin * 1000).toFixed(2)} mm, ${P.pushrod}): pi/4 (D^2 - d^2) = ${(leak * 1e6).toFixed(2)} mm2`,
  };
}

/**
 * Round 19: how far the pushrod's true axis moves from its drawn-zero place, over the commanded travel, at the stations
 * xs (m) along it: the servo's ball on its arm's arc about the spline (r_arm, in the x-z plane), the horn's ball on the
 * horn's arc about the pivot axis (in the plane of x and the surface's normal), the horn's angle for each servo angle
 * from the rod's fixed length (the root nearest the drawn law r_arm sin(servo) = r_horn sin(surface)), both ways to the
 * commanded deflection dMax. row: the mechanism's row (o, u, v, sE, nE, xs, zs, zH, A); G: tailMechGeometry. -> m.
 */
export function pushrodExcursion(row, G, dMax, xs, n = 40) {
  const sub = (a, b) => a.map((q, i) => q - b[i]), add = (a, b) => a.map((q, i) => q + b[i]), mul = (a, k) => a.map((q) => q * k), dot = (a, b) => a.reduce((s, q, i) => s + q * b[i], 0), nrm = (a) => Math.hypot(...a);
  const ra = Math.abs(row.zH - row.zs), sg = Math.sign(row.zH - row.zs) || 1, R = Math.abs(row.nE), k = ra / R, yb = row.A.p[1];
  // (x, z, y)
  const ballS = (th) => [row.xs - ra * Math.sin(th), row.zs + sg * ra * Math.cos(th), yb];
  const P0 = [G.xq, row.o[0] + row.u[0] * row.sE, row.o[1] + row.u[1] * row.sE], v3 = [0, row.v[0], row.v[1]];
  const ballH = (ph) => add(P0, add(mul(v3, row.nE * Math.cos(ph)), [-R * Math.sin(ph), 0, 0]));
  const D0 = nrm(sub(ballH(0), ballS(0))), u0 = mul(sub(ballH(0), ballS(0)), 1 / D0), p0 = ballS(0);
  let worst = 0;
  for (let i = -n; i <= n; i++) {
    const dl = (i / n) * dMax, th = Math.asin(Math.sin(dl) / k), S0 = ballS(th), f = (ph) => nrm(sub(ballH(ph), S0)) - D0;
    let best = null;
    for (let j = 0; j < 400; j++) {
      let a = dl - 0.4 + (j / 400) * 0.8, b = dl - 0.4 + ((j + 1) / 400) * 0.8;
      if (f(a) * f(b) > 0) continue;
      for (let m = 0; m < 60; m++) { const c = (a + b) / 2; if (f(a) * f(c) <= 0) b = c; else a = c; }
      const rt = (a + b) / 2; if (best == null || Math.abs(rt - dl) < Math.abs(best - dl)) best = rt;
    }
    if (best == null) throw new Error('pushrodExcursion: no horn angle holds the rod\'s length');
    const H = ballH(best), u = mul(sub(H, S0), 1 / nrm(sub(H, S0)));
    for (const x of xs) {
      const gp = add(p0, mul(u0, (x - p0[0]) / u0[0])), t = dot(sub(gp, S0), u0) / dot(u, u0);
      worst = Math.max(worst, nrm(sub(add(S0, mul(u, t)), gp)));
    }
  }
  return worst;
}

/** The convex hull of points in the plane (monotone chain), counter-clockwise. */
function hull2D(pts) {
  const q = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]), cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = (list) => { const h = []; for (const p of list) { while (h.length >= 2 && cr(h[h.length - 2], h[h.length - 1], p) <= 0) h.pop(); h.push(p); } h.pop(); return h; };
  return [...half(q), ...half(q.reverse())];
}

/**
 * The mechanism's BOM (round 17): its structure lines, sized from the same geometry as its solids (the pushrod's length
 * as the design states it, P.pushrod_L_mm, which the completeness check holds to the drawn rod), and its bought parts.
 * T: the design's tail block; yB, boomR: the boom's height and radius. -> { structure: [{ name, material, mass_g,
 * basis, count }], parts: [{ id, count, role }], G }.
 */
export function tailMechBom(T, yB, boomR, P0, catalog) {
  const P = { ...TAIL_MECH, ...(P0 ?? {}) }, G = tailMechGeometry(T, { y: yB, r: boomR }, P, catalog);
  const al7 = byId(catalog, P.pin), al6 = byId(catalog, P.material), rho = (m) => m.specs.density_g_cm3 * 1e6; // g/m3
  const pv = byId(catalog, P.pivot), pr = byId(catalog, P.pushrod), te = byId(catalog, P.teRod ?? 'rod-ec-1-5mm'), ke = byId(catalog, P.rodEnd).specs;
  const pinV = Math.PI * G.rPin ** 2 * (G.sT0 - G.sP0) + Math.PI * (G.ri - G.run / 10) ** 2 * (G.sP1 - G.sT0);
  // the saddle's material as drawn (a boss's own material is its box less the bushings' bore: its area_m2)
  const saddleV = saddleSolids(T, yB, boomR, G, P).reduce((a, q) => a + (q.area_m2 ?? shapeArea(q.shape)) * (q.x[1] - q.x[0]), 0);
  const hornV = Math.PI * (G.hubR ** 2 - G.rPin ** 2) * (G.sH1 - G.sH0) + G.hornT * (G.hornR + 3e-3 - G.hubR + 1e-4) * G.hornD;
  // the end plugs fill the pushrod's bore (not the pivot tube's)
  const rPr = mm(tubeSpec(pr).id_draw) / 2;
  const Lp = mm(P.pushrod_L_mm ?? NaN), plugX = mm(P.plug_len), plugV = Math.PI * (rPr - 1e-5) ** 2 * plugX + Math.PI * 1e-6 * mm(ke.l1_mm);
  const LTE = G.L - G.sRoot + G.tRib / 2;
  const gm = (id) => tubeSpec(byId(catalog, id)).g_per_m;
  const structure = [
    { name: 'tail pivot pins, 2 off', material: al7, mass_g: 2 * pinV * rho(al7), basis: `2 x a ${(G.rPin * 2000).toFixed(2)} mm pin x ${((G.sT0 - G.sP0) * 1000).toFixed(1)} mm (in both bushings and the horn's hub, its inboard end grooved for a ${P.washer}) and a ${((G.ri - G.run / 10) * 2000).toFixed(2)} mm spigot x ${P.pin_in_tube} mm bonded in the pivot tube; ${al7.specs.density_g_cm3} g/cm3`, count: 2 },
    { name: 'tail pivot tubes, 2 off', material: pv, mass_g: 2 * gm(P.pivot) * (G.L - G.sT0), basis: `${gm(P.pivot)} g/m x ${(G.L - G.sT0).toFixed(4)} m x 2 (the horn's hub to the tip, on the quarter chord)`, count: 2 },
    { name: 'tail trailing edge rods, 2 off', material: te, mass_g: 2 * te.mass_g * LTE, basis: `${te.mass_g} g/m x ${LTE.toFixed(4)} m x 2 (root rib to tip rib)`, count: 2 },
    { name: 'tail saddle', material: al6, mass_g: saddleV * rho(al6), basis: `a ${P.saddle.sleeve_len} mm sleeve ${((boomR + mm(P.saddle.wall)) * 2000).toFixed(1)}/${(boomR * 2000).toFixed(2)} mm bonded on the boom's end, and per side a ${P.saddle.boss_len} mm boss round two ${P.bushing} on the pivot axis with its webs to the sleeve (${(saddleV * 1e9).toFixed(0)} mm3, the drawn solids); ${al6.specs.density_g_cm3} g/cm3` },
    { name: 'tail horns, 2 off', material: al6, mass_g: 2 * hornV * rho(al6), basis: `each a ${P.horn.hub_od} mm hub x ${P.horn.hub_len} mm bonded on the pivot pin and a ${P.horn.arm_t} x ${P.horn.arm_depth} mm arm to its rod end's screw at ${P.horn.radius} mm; ${al6.specs.density_g_cm3} g/cm3`, count: 2 },
    { name: 'tail pushrods, 2 off', material: pr, mass_g: 2 * gm(P.pushrod) * Lp, basis: `${gm(P.pushrod)} g/m x ${Lp.toFixed(4)} m x 2 (between the two rod ends' shanks)`, count: 2 },
    { name: 'tail pushrod end plugs, 4 off', material: al6, mass_g: 4 * plugV * rho(al6), basis: `each a ${((rPr - 1e-5) * 2000).toFixed(2)} mm plug x ${P.plug_len} mm bonded in the pushrod's end with an M2 stud x ${ke.l1_mm} mm into the rod end; ${al6.specs.density_g_cm3} g/cm3`, count: 4 },
  ];
  // round 19 (decision 2): the exit guides and their lugs (P.guide), at the length and volume the design states (the
  // completeness check holds them to the drawn guide and lug, as it holds the pushrod's)
  if (P.guide) {
    const gt = byId(catalog, P.guide.tube), lp = byId(catalog, P.guide.lug), Lg = mm(P.guide.L_mm ?? NaN), Vl = (P.guide.lug_mm3 ?? NaN) * 1e-9;
    structure.push({ name: 'tail pushrod guides, 2 off', material: gt, mass_g: 2 * gm(P.guide.tube) * Lg, basis: `${gm(P.guide.tube)} g/m x ${Lg.toFixed(4)} m x 2 (each from its lug on the former ${P.guide.former} aft until its whole section is ${P.guide.out_mm} mm outside the loft)`, count: 2 });
    structure.push({ name: 'tail pushrod guide lugs, 2 off', material: lp, mass_g: 2 * Vl * rho(lp), basis: `2 x ${(Vl * 1e9).toFixed(1)} mm3 of ${lp.id} (the drawn lug less the guide's hole) at ${lp.specs.density_g_cm3} g/cm3`, count: 2 });
  }
  const parts = [
    { id: P.bushing, count: 4, role: 'tail pivot bushings, two per surface' },
    { id: P.washer, count: 2, role: 'tail pivot pin retaining washers' },
    { id: P.rodEnd, count: 4, role: 'tail pushrod rod ends, two per pushrod' },
    { id: P.arm, count: 2, role: 'tail servo arms (the maker\'s)' },
    { id: P.screw, count: 4, role: 'tail linkage screws (rod ends to arm and horn)' },
  ];
  return { structure, parts, G };
}

/**
 * The mechanism's M7 members (round 17; audit items 2 and 11: the tail's root moment and hinge moment now have a
 * rated path): each surface's force (channel tail_F_R / tail_F_L) at its area centroid along the span, its hinge
 * moment (tail_M_R / tail_M_L) through the horn and pushrod. ax: airframe.js's member context.
 */
export function tailMechMembers(ax) {
  const { d, catalog, members, mat, glueAl, glueCf, lump, tieL } = ax;
  const sect = (name, radial) => ({ name, radial, axial: [] });
  const T = d.tail, P = { ...TAIL_MECH, ...(d.params.tailMech ?? {}) }, yB = d.params.tailY_m ?? T.y_m;
  const boomR = byId(catalog, d.params.parts.keel).specs.od_mm / 2000;
  const G = tailMechGeometry(T, { y: yB, r: boomR }, P, catalog);
  // the area centroid along the span, from the pivot's origin (the flown root at G.s0)
  const lam = T.taper, sBar = G.s0 + (T.span_each_m * (1 + 2 * lam)) / (3 * (1 + lam));
  const sIn = (G.sB0 + G.sB0 + G.bL) / 2, sOut = G.sB1, Lb = sOut - sIn, arm = sBar - sOut, armT = sBar - G.sT0;
  const al7 = mat(P.pin), al6 = mat(P.material), ig = byId(catalog, P.bushing).specs, ke = byId(catalog, P.rodEnd).specs;
  const pr = tubeAllowableOf(catalog, P.pushrod), pv = tubeAllowableOf(catalog, P.pivot), W = byId(catalog, P.washer).specs;
  const tl = lump('tail surfaces'), inSurf = inertial(tl.m / 2, tl.r);
  const sv = d.servos?.tail, rArm = mm(P.arm_hole_mm), Rh = G.hornR;
  const igus = { id: P.bushing, yield_MPa: ig.static_surface_pressure_mpa, basis: `iglidur G static surface pressure ${ig.static_surface_pressure_mpa} MPa (igus), as a crushing allowable` };
  for (const t of ['R', 'L']) {
    const F = [{ ch: `tail_F_${t}`, k: 1 }], Mh = [{ ch: `tail_M_${t}`, k: 1 }], Fp = [{ ch: `tail_M_${t}`, k: +(1 / Rh).toFixed(6) }];
    const Mroot = [{ ch: `tail_F_${t}`, k: +arm.toFixed(6) }];
    members.push({ name: `tail pivot pin ${t} at the outer bushing`, kind: 'pin', check: 'round', material: al7, dims: { d_mm: +(G.rPin * 2000).toFixed(3), di_mm: 0 }, sections: [sect('M', Mroot), sect('T', Mh), sect('V', F)], uses: { M: 'M', T: 'T', V: 'V' }, note: `the surface's force at its area centroid (${(sBar * 1000).toFixed(0)} mm out) about the outer bushing (${(arm * 1000).toFixed(0)} mm), its hinge moment, its force in shear` });
    for (const [nm, k] of [['outer', 1 + arm / Lb], ['inner', arm / Lb]]) {
      members.push({ name: `tail bushing ${t} ${nm}`, kind: 'seat', check: 'seat', material: igus, dims: { d_mm: ig.d1_mm, t_mm: ig.b1_mm - ig.b2_mm, crush: true }, sections: [sect('F', [{ ch: `tail_F_${t}`, k: +k.toFixed(6) }])], uses: { F: 'F' }, note: `the root moment as a couple over the bushings ${(Lb * 1000).toFixed(1)} mm apart plus the force: ${k.toFixed(2)} x the surface's force on the ${nm} bushing; its static surface pressure (SPEC round 4: a bushing by the maker's static pressure, FoS 1.5)` });
      members.push({ name: `tail saddle boss ${t} round the ${nm} bushing`, kind: 'seat', check: 'seat', material: al6, dims: { d_mm: ig.d2_mm, t_mm: ig.b1_mm - ig.b2_mm, crush: true }, sections: [sect('F', [{ ch: `tail_F_${t}`, k: +k.toFixed(6) }])], uses: { F: 'F' }, note: 'the bushing\'s load on its bore in the saddle\'s boss (6061 at its design yield)' });
    }
    members.push({ name: `tail saddle webs ${t}`, kind: 'plate', check: 'plate', material: al6, dims: { b_mm: +(2 * G.rBoss * 1000).toFixed(2), t_mm: 2 * (P.saddle.web_t ?? P.saddle.wall) }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +(sBar - G.sB0).toFixed(6) }])], uses: { M: 'M' }, note: 'the root moment at the boss\'s inboard end in the plane of its two webs (each web_t thick as drawn, round 18; as deep as the boss)' });
    members.push({ name: `tail pivot tube ${t} at its root`, kind: 'tube', check: 'cfrp tube', material: pv.composite, dims: { d_mm: pv.od_rate, di_mm: pv.id_rate }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +armT.toFixed(6) }]), sect('T', Mh), sect('V', F)], uses: { M: 'M', T: 'T', V: 'V' }, note: `the surface's force at its area centroid about the tube's root on the pin (${(armT * 1000).toFixed(0)} mm), its hinge moment` });
    members.push({ name: `tail pivot pin ${t} spigot`, kind: 'pin', check: 'round', material: al7, dims: { d_mm: +((G.ri - G.run / 10) * 2000).toFixed(3), di_mm: 0 }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +armT.toFixed(6) }]), sect('T', Mh)], uses: { M: 'M', T: 'T' }, note: 'the spigot in the tube\'s bore at the tube\'s root: the root moment there and the hinge moment' });
    members.push({ name: `tail pivot pin ${t} spigot bond`, kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: +(G.ri * 2000).toFixed(3), L_mm: P.pin_in_tube }, sections: [sect('T', Mh), sect('F', inSurf)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half): the hinge moment and the surface\'s inertia along its span' });
    members.push({ name: `tail horn ${t} hub bond`, kind: 'bond', check: 'bond', material: glueAl, dims: { d_mm: +(G.rPin * 2000).toFixed(3), L_mm: P.horn.hub_len }, sections: [sect('T', Mh)], uses: { T: 'T' }, note: 'Araldite 2011 on aluminium: the hinge moment' });
    members.push({ name: `tail horn ${t} arm`, kind: 'plate', check: 'plate', material: al6, dims: { b_mm: P.horn.arm_depth, t_mm: P.horn.arm_t }, sections: [sect('M', [{ ch: `tail_M_${t}`, k: +((Rh - G.hubR) / Rh).toFixed(6) }])], uses: { M: 'M' }, note: 'the pushrod\'s force at the screw, bending the arm in its plane at the hub' });
    members.push({ name: `tail bushing ${t} flanges, axial thrust`, kind: 'seat', check: 'flange', material: { id: P.bushing, static_surface_pressure_mpa: ig.static_surface_pressure_mpa }, dims: { D_mm: ig.d3_mm, d_mm: ig.d2_mm }, sections: [sect('F', inSurf)], uses: { F: 'F' }, note: `the pin's axial stops (the washer inboard, the horn's hub outboard) thrust the surface's spanwise inertia into a ${P.bushing} flange's ring, at its published static surface pressure; both flanges alike` });
    members.push(tappedThreadMember(`tail horn ${t} screw thread in the arm`, al6, { screw_len_mm: byId(catalog, P.screw).specs.length_mm, ball_B_mm: ke.B_mm, arm_t_mm: P.horn.arm_t }, Fp, 'the 6061 horn arm', "the pushrod's"));
    // round 17b: the ribs (balsa) on the pivot tube and the trailing edge rod: the surface's force and hinge moment shared
    // over its ribs (each taken at a third, the four ribs bounding three bays), the bond on the tube limited by the
    // balsa's published shear (Wood Handbook Table 5-5a, at half) below the adhesive's on CFRP; the rib on the
    // trailing edge rod by crushing at half the species-average crushing strength, at the tip rib's (smallest) lever
    { const bal = byId(catalog, 'reference-balsa-usda-wood-handbook-2021').specs.design_values, crush = byId(catalog, 'balsa-wood-database').specs.crushing_strength_mpa;
      const glueBal = { id: `${glueCf.id} on balsa`, shear_MPa: Math.min(glueCf.shear_MPa, bal.shear_MPa), basis: `the lower of ${glueCf.basis} and the balsa's own shear, ${bal.basis}` };
      const teR = mm(byId(catalog, d.params.parts.teRod).specs.od_mm) / 2, lev = 0.75 * T.chord_root_m * T.taper - teR, sh = 1 / 3;
      members.push({ name: `tail ribs ${t} on the pivot tube, each`, kind: 'bond', check: 'bond', material: glueBal, dims: { d_mm: +(2 * G.rT * 1000).toFixed(3), L_mm: P.rib_t }, sections: [sect('T', [{ ch: `tail_M_${t}`, k: sh }]), sect('F', [{ ch: `tail_F_${t}`, k: sh }])], uses: { T: 'T', F: 'F' }, note: `each rib's bond round the pivot tube over its ${P.rib_t} mm thickness: a third of the surface's hinge moment and force` });
      members.push({ name: `tail ribs ${t} on the trailing edge rod, each`, kind: 'seat', check: 'seat', material: { id: 'balsa-wood-database', yield_MPa: crush / 2, basis: `balsa crushing ${crush} MPa (species average, wood-database) at half` }, dims: { d_mm: +(teR * 2000).toFixed(3), t_mm: P.rib_t, crush: true }, sections: [sect('F', [{ ch: `tail_M_${t}`, k: +(sh / lev).toFixed(6) }, { ch: `tail_F_${t}`, k: sh }])], uses: { F: 'F' }, note: `the rib's hole bearing on the trailing edge rod: a third of the hinge moment over the tip rib's ${(lev * 1000).toFixed(0)} mm lever, and a third of the force` });
      members.push({ name: `tail trailing edge rod ${t} in its ribs`, kind: 'bond', check: 'bond', material: glueBal, dims: { d_mm: +(teR * 2000).toFixed(3), L_mm: 4 * P.rib_t }, sections: [sect('F', inSurf)], uses: { F: 'F' }, note: 'the rod bonded through its four ribs: the surface\'s inertia along the span (all of it, conservative)' }); }
    members.push({ name: `tail pin washer ${t}`, kind: 'retainer', check: 'rated', dims: { rating_N: 1000 * Math.min(W.load_columns.FN_kN, W.load_columns.Fs_kN), rating_basis: `${P.washer}: the lower of FN ${W.load_columns.FN_kN} kN (groove) and Fs ${W.load_columns.Fs_kN} kN (washer) as tabulated (DIN 6799 convention; the groove's figure is for steel shafts and the pin is 7075)` }, sections: [sect('F', inSurf)], uses: { F: 'F' }, note: 'the surface\'s inertia along its span (every axis counted)' });
    members.push({ name: `tail pushrod ${t}`, kind: 'strut', check: 'strut', material: { id: P.pushrod, E_GPa: pr.E_GPa, yield_MPa: pr.sigma_MPa }, dims: { L_mm: +(P.pushrod_L_mm ?? 0).toFixed(1), d_mm: pr.od_rate, di_mm: pr.id_rate }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: `the hinge moment over the horn's ${(Rh * 1000).toFixed(0)} mm, pinned at its rod ends: Euler on the published modulus, the strength at ${pr.basis}` });
    members.push({ name: `tail pushrod ${t} rod ends`, kind: 'rod end', check: 'rated', dims: { rating_N: ke.max_static_tensile_long_term_n, rating_basis: `${P.rodEnd}: maximum static tensile load, long term (igus)` }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: 'the pushrod\'s force through each rod end' });
    members.push({ name: `tail pushrod ${t} screws`, kind: 'fasteners', check: 'bolts', fastener: fastenerOfCat(catalog, P.screw), dims: { n: 1 }, sections: [sect('V', Fp)], uses: { V: 'V' }, note: 'each rod end\'s M2 screw in shear' });
    members.push({ name: `tail pushrod ${t} end plug bonds`, kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: tubeSpec(byId(catalog, P.pushrod)).id_draw, L_mm: P.plug_len }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half): the pushrod\'s force on each end plug' });
    // round 18 (SPEC ruling "Maker's part"): the maker's arm on its spline, valid while its servo is within its rule
    if (sv) members.push(tailServoArmMember(d, catalog, t, sv, P, Fp));
    if (P.guide) members.push(...guideMembers(ax, P, t));
    if (tieL) members.push({ name: `tail servo ${t} ties under its arm's reaction`, kind: 'tie', check: 'tie', tie: { id: tieL.id, loop_N: tieL.loop_N }, dims: { n: 2 }, sections: [sect('F', [...Fp, ...inertial(sv?.mass_kg ?? 0.0215, [-0.04, -0.065, 0])])], uses: { F: 'F' }, note: 'the servo\'s two T40R carry the pushrod\'s force at its arm and its own inertia' });
  }
  members.push({ name: 'tail saddle bond on the boom', kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: boomR * 2000, L_mm: P.saddle.sleeve_len }, sections: [sect('T', [{ ch: 'tail_F_R', k: +arm.toFixed(6) }, { ch: 'tail_F_L', k: +arm.toFixed(6) }]), sect('F', [{ ch: 'tail_F', k: 1 }])], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half) on the boom\'s end: both surfaces\' root moments as torsion (their sum, conservative) and the tail\'s force' });
}

/**
 * Round 19: an exit guide's two bonds (the guide in its lug's hole, the lug's face on the former's band), each carrying
 * the guide, its lug and the whole pushrod at their inertia (were the rod all to rest on the guide: conservative; over the
 * commanded travel it does not touch it, the covering record's exit seal row) and the drag on the guide's whole section
 * at the pod's drag speed (Cd 1.2, the containment record's bluff-body figure: an upper bound on what stands out).
 */
function guideMembers({ d, catalog, glueCf, lump }, P, t) {
  const sect = (name, radial) => ({ name, radial, axial: [] });
  const gd = P.guide, gt = tubeSpec(byId(catalog, gd.tube)), lp = byId(catalog, gd.lug).specs, lugT = lp.thickness_mm_nominal;
  const mG = (gt.g_per_m * (gd.L_mm ?? NaN)) / 1e6, mL = ((gd.lug_mm3 ?? NaN) * 1e-9) * lp.density_g_cm3 * 1000, mR = (tubeSpec(byId(catalog, P.pushrod)).g_per_m * (P.pushrod_L_mm ?? NaN)) / 1e6;
  const rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0, V = podParams(d).V_drag_m_s, od = gt.od_draw + gd.tol_mm;
  const D = 0.5 * rho * V ** 2 * 1.2 * (Math.PI / 4) * mm(od) ** 2;
  const F = [...inertial(mG + mL + mR, lump('tail pushrods').r), { ch: 'sf_origin', k: +(D / 9.80665).toFixed(8) }];
  return [
    { name: `tail pushrod guide ${t} bond in its lug`, kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: +(gt.od_draw - gd.tol_mm).toFixed(3), L_mm: lugT }, sections: [sect('F', F)], uses: { F: 'F' }, note: `Araldite 2011 on CFRP (typical, half): the guide (its smallest outside, ${gd.tube} at -${gd.tol_mm} mm) in its lug's hole over the lug's ${lugT} mm; the guide (${(mG * 1000).toFixed(2)} g), its lug and the whole pushrod (${(mR * 1000).toFixed(1)} g) at their inertia and ${D.toFixed(3)} N of drag on the guide's whole ${od.toFixed(1)} mm section at ${V} m/s` },
    { name: `tail pushrod guide lug ${t} bond on its former`, kind: 'bond', check: 'lap', material: glueCf, dims: { w_mm: gd.lug_bond.w_mm, L_mm: gd.lug_bond.L_mm }, sections: [sect('F', F)], uses: { F: 'F' }, note: `Araldite 2011 on CFRP (typical, half): the lug's face on the ${gd.former} former's band over ${gd.lug_bond.w_mm} x ${gd.lug_bond.L_mm} mm (the covering record holds the drawn overlap to at least this), the same load` },
  ];
}

/**
 * Round 18 (SPEC ruling "Maker's part", operator's decision 2026-11-03): the maker's arm, its centre screw and its tapped
 * holes on its own servo's spline are a maker's part, no longer a member held to the stall at FoS 1.5. Its force at the
 * hole (F, the pushrod's) and its torque at the spline (T, the servo's: the hinge moment x the linkage ratio) are
 * reported; it is valid only while the servo is within its own rule at the supply's published row (chains.js).
 */
function tailServoArmMember(d, catalog, t, sv, P, Fp) {
  const sect = (name, radial) => ({ name, radial, axial: [] });
  return { name: `tail servo arm ${t}`, kind: 'arm', check: "maker's part", maker: servoArmMaker(d, catalog, 'tail', P.arm), dims: { hole_mm: P.arm_hole_mm }, sections: [sect('F', Fp), sect('T', [{ ch: `tail_M_${t}`, k: sv.linkage_ratio ?? 1 }])], uses: { F: 'F', T: 'T' }, note: `the maker's ${P.arm} on the servo's spline: the pushrod's force at its ${P.arm_hole_mm} mm hole, and the servo's torque at the spline; no rating of its own is published (SPEC ruling "Maker's part")` };
}

/**
 * A rod end's M2 screw in the tapped hole of an arm we machine (the tail's horn, round 17; the twist drive's lever, round
 * 18): the internal thread engaged over Le = min(the screw's length less the rod end's ball, the arm's thickness), in
 * shear on its stripping area pi d2 Le / 2 (d2 the M2 pitch diameter, 2 - 0.6495 x 0.4 = 1.74 mm) as a pin of that area
 * at the arm's material's shear yield, under the link's whole force F (conservative: its part along the screw is
 * smaller). where: the arm, as the note names it; whose: whose force F is.
 */
export function tappedThreadMember(name, material, { screw_len_mm, ball_B_mm, arm_t_mm }, F, where, whose) {
  const d2 = 1.74, Le = Math.min(screw_len_mm - ball_B_mm, arm_t_mm), A = (Math.PI * d2 * Le) / 2;
  return { name, kind: 'pin', check: 'pin-shear', material, dims: { d_mm: +Math.sqrt((4 * A) / Math.PI).toFixed(4), planes: 1, Le_mm: +Le.toFixed(2), area_mm2: +A.toFixed(3) }, sections: [{ name: 'F', radial: F, axial: [] }], uses: { F: 'F' }, note: `the M2 internal thread in ${where} over ${Le.toFixed(1)} mm engaged (the screw's length less the rod end's ball): shear area pi d2 Le / 2 = ${A.toFixed(2)} mm2 (d2 the pitch diameter ${d2} mm), as a pin of that area, under ${whose} whole force (conservative: its part along the screw is smaller)` };
}

const inertial = (m, r) => { const R = Math.hypot(...r); return [{ ch: 'sf_origin', k: m }, { ch: 'wdot', k: m * R }, { ch: 'w2', k: m * R }]; };
function fastenerOfCat(catalog, id) { const f = byId(catalog, id), dv = f.specs.design_values ?? {}; return { id, tension_N: dv.tension_N, shear_N: dv.shear_N }; }
function tubeAllowableOf(catalog, id) {
  const p = byId(catalog, id), t = tubeSpec(p), a = tubeAllowable(p), s = p.specs;
  // the shear allowable from the maker's transverse record (Rock West: composite-rockwest-<part>-transverse), else none
  const tr = s.part_number ? catalog.find((q) => q.id === `composite-rockwest-${s.part_number.toLowerCase()}-transverse`) : null;
  return { od_rate: t.od_rate, id_rate: t.id_rate, E_GPa: a.E_GPa, sigma_MPa: a.sigma_MPa, basis: a.basis, composite: { id, axial_MPa: a.sigma_MPa, shear_MPa: tr?.specs.design_values.xy_shear_MPa ?? null, shear_basis: tr?.specs.design_values.xy_shear_basis ?? 'no published shear strength', note: s.strength_note ?? null } };
}

/**
 * The tail's actuation chains (chains.js format), from the drawn mechanism (asm.tailMech, placed by clearance.js
 * afterPacked): each servo's spline to its surface's pivot tube, with the pivot, ratio, travel and law against the
 * flown ones. The servo's travel is not published (its speed is given per 60 deg): P.servo_travel_deg, an assumption
 * stated in the chain. Through the travel the horn's rod end turns about the pivot past the saddle's boss (the only
 * neighbour sharing its span): the smallest gap is computed over the commanded travel.
 */
export function tailChains(asm, d, catalog) {
  const tm = asm.tailMech;
  if (!tm) return [];
  const { G, rows, P } = tm, T = d.tail, sv = d.servos?.tail ?? {}, ke = byId(catalog, P.rodEnd).specs;
  const DEG = Math.PI / 180, dMax = T.deflection_max_rad, thS = (P.servo_travel_deg ?? 30) * DEG;
  const members = (d.structure?.members ?? []).map((m) => m.name);
  return rows.map((r) => {
    const t = r.tag, k = Math.abs(r.zH - r.zs) / Math.abs(r.nE);
    // the drawn law: the pushrod moves its two ends alike along x, r_arm sin(servo) = r_horn sin(surface)
    const delivered = Math.asin(Math.min(1, k * Math.sin(thS)));
    let maxErr = 0;
    for (let i = 1; i <= 60; i++) { const dl = (i / 60) * dMax, th = Math.asin(Math.sin(dl) / k), local = (k * Math.cos(th)) / Math.cos(dl); maxErr = Math.max(maxErr, Math.abs(local / k - 1)); }
    // the horn's rod end (its housing, d2) turning about the pivot axis over +-dMax against the boss's box (x +-xb, n +-rBoss)
    const xb = Math.max(G.hubR, G.d3 / 2, G.rW) + G.run + mm(P.saddle.web_t ?? P.saddle.wall), rE = mm(ke.d2_mm) / 2, R = Math.abs(r.nE);
    let gap = Infinity;
    for (let i = -60; i <= 60; i++) {
      const ph = (i / 60) * dMax, cx = R * Math.sin(ph), cn = Math.sign(r.nE) * R * Math.cos(ph);
      const dx = Math.max(0, Math.abs(cx) - xb), dn = Math.max(0, Math.abs(cn) - G.rBoss);
      gap = Math.min(gap, Math.hypot(dx, dn) - rE);
    }
    const want = [`tail pushrod ${t}`, `tail horn ${t} arm`, `tail horn ${t} hub bond`, `tail pivot pin ${t} at the outer bushing`, `tail pivot tube ${t} at its root`, `tail servo arm ${t}`, `tail pushrod ${t} rod ends`];
    return {
      name: `tail ${t}`, actuator: { part: `packed servo_tail_${t}`, bom: sv.id ?? null, output: 'its spline' },
      path: [`packed servo_tail_${t}`, `tail servo arm ${t}`, `tail rod end ${t} servo`, `tail pushrod plug ${t} servo`, `tail pushrod ${t}`, `tail pushrod plug ${t} horn`, `tail rod end ${t} horn`, `tail horn ${t}`, `tail pivot pin ${t}`, `tail pivot tube ${t}`],
      figures: {
        pivot_x: { flown: T.x_m, drawn: G.xq, tol: 0.0005, units: 'm', basis: 'the flight model turns each surface about its quarter chord at tail.x_m; the drawn pivot pin\'s axis' },
        root_z: { flown: T.z_m, drawn: Math.abs(r.o[0] + r.u[0] * G.s0), tol: 0.0005, units: 'm', basis: 'the flown surface\'s root (tail.z_m) against the drawn root rib\'s inboard face, where the covering starts' },
        root_y: { flown: T.y_m, drawn: r.o[1] + r.u[1] * G.s0, tol: 0.0005, units: 'm', basis: 'the flown root\'s height (tail.y_m) against the drawn root rib\'s inboard face' },
        pivot_dihedral: { flown: T.dihedral_rad, drawn: Math.atan2(r.u[1], Math.abs(r.u[0])), tol: 0.1 * DEG, units: 'rad', basis: 'the pivot along the surface\'s dihedral' },
        ratio: { flown: sv.linkage_ratio ?? 1, drawn: k, tol: 1e-4, rel: true, units: '-', basis: 'the surface\'s turn per servo turn the flight model flies (design.servos.tail.linkage_ratio) against the drawn arm\'s hole radius over the horn\'s' },
        travel: { required_rad: dMax, delivered_rad: delivered, basis: `the commanded deflection (tail.deflection_max_rad) against asin(k sin ${(thS / DEG).toFixed(0)} deg) at the servo's travel (an assumption: not published)` },
        law: { flown: 'surface = k x servo', drawn: 'sin(surface) = k sin(servo)', max_err: maxErr, tol: 0.06, basis: 'the drawn linkage\'s local ratio against the flown constant one over the commanded travel; the flight model\'s torque and speed at the surface carry it within 6 %' },
        travel_clearance: { required: 0.0005, delivered: gap, units: 'm', basis: 'the horn\'s rod end turning about the pivot over the commanded travel: its smallest gap to the saddle\'s boss (the one neighbour in its span), against the 0.5 mm assembly clearance' },
        loads: { members: want.filter((w) => members.includes(w)), missing: want.filter((w) => !members.includes(w)), basis: 'the M7 members carrying the chain\'s loads (tail_F, tail_M channels), from the flown peaks' },
      },
      assumptions: [`the servo's travel ${(thS / DEG).toFixed(0)} deg each way (not published; Hitec gives the speed per 60 deg)`, `the servo's output spline on its case's top face centre line, ${P.spline_from_aft_mm} mm from its aft end (not published)`, `the maker's arm (${P.arm}) has no published load rating: a maker's part (round 18, SPEC ruling), valid while the servo is within its own rule at its supply's published row`],
    };
  });
}

/**
 * The tail's interface-ledger rows (interfaces.js format, passed ahead of INTERFACE_RULES): every join tailMechParts
 * makes, with the M7 members that rate its load and what holds it axially (tailMechMembers). The servo arm is the
 * maker's (no published rating): it and its tapped hole are a maker's part (round 18, SPEC ruling), held to the servo's
 * own rule at its supply's published row.
 */
const side = (a, b) => /(?:^| )([RL])(?: |$|\/)/.exec(`${a} ${b}`)?.[1] ?? null;
const nm = (re) => (d, a, b) => { const t = side(a, b); return t ? re.map((f) => f(t, a, b)) : []; };
const bushOf = (a, b) => /(inner|outer)/.exec(`${a} ${b}`)?.[1];
const TBOND = "the adhesive maker's published lap shear on that substrate, at half the typical figure, and no more than the balsa's published shear at half where a rib is bonded (SPEC round 7)";
const RIB_HOLE_TXT = 'through its hole in the rib \\(not drawn: the rib lies across the span, not across x\\)';
const BORE_TXT = "in the other tube's bore \\(not drawn: a tube along the span is drawn as its capsule across x\\)";
/** The tail's BORE pairs: the real fit each stands for, by its two parts. */
function tailBore(d, a, b) {
  const t = side(a, b), w = `${a} | ${b}`, has = (re) => re.test(w);
  if (!t) return [];
  if (has(/spigot/) && has(/pivot tube/)) return [`tail pivot pin ${t} spigot bond`];
  if (has(/pin washer/)) return [`tail pin washer ${t}`];
  if (has(/horn/)) return [`tail horn ${t} hub bond`];
  const bu = /(inner|outer)/.exec(w)?.[1];
  if (has(/bushing/) && has(/saddle/)) return bu ? [`tail bushing ${t} ${bu}`, `tail saddle boss ${t} round the ${bu} bushing`] : [];
  if (has(/bushing/) && has(/pivot pin/)) return bu ? [`tail bushing ${t} ${bu}`, `tail pivot pin ${t} at the outer bushing`] : [];
  if (has(/saddle/) && has(/pivot pin/)) return [`tail bushing ${t} inner`, `tail bushing ${t} outer`];
  return [];
}
const TSEAT = "the bushing maker's static surface pressure at FoS 1.5 (igus), the housing at its design yield (SPEC round 4)";
const TRATED = 'a published load rating at FoS 1.5 (members.js rated)';
const tr = (why, kind, load, axial, note = '') => ({ why: typeof why === 'string' ? new RegExp(`^${why.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}$`) : why, part: /^tail /, kind, load, axial, note });
export const TAIL_INTERFACE_RULES = [
  tr("its spigot bonded in the tube's bore", 'bonded', { by: 'the spigot bond (Araldite 2011 on CFRP)', rating: nm([(t) => `tail pivot pin ${t} spigot bond`, (t) => `tail pivot pin ${t} spigot`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail pivot pin ${t} spigot bond`]), rule: TBOND }),
  tr('in its groove on the pin', 'retained', { by: 'the DIN 6799 washer in its groove', rating: nm([(t) => `tail pin washer ${t}`]), rule: TRATED }, { by: 'the washer itself', rating: nm([(t) => `tail pin washer ${t}`]), rule: TRATED }),
  tr("against the flange (the pin's axial stop)", 'thrust', { by: "the washer's thrust on the inner bushing's flange", rating: nm([(t) => `tail pin washer ${t}`, (t) => `tail bushing ${t} flanges, axial thrust`]), rule: TRATED }, { by: 'the washer', rating: nm([(t) => `tail pin washer ${t}`]), rule: TRATED }),
  tr('pressed in the housing', 'seat', { by: 'the bushing in the saddle\'s boss', rating: (d, a, b) => { const t = side(a, b), w = bushOf(a, b); return t && w ? [`tail bushing ${t} ${w}`, `tail saddle boss ${t} round the ${w} bushing`] : []; }, rule: TSEAT }, { by: "its flange against the boss's face, held there by the pin's stops (washer inboard, horn outboard)", rating: nm([(t) => `tail bushing ${t} flanges, axial thrust`, (t) => `tail pin washer ${t}`, (t) => `tail horn ${t} hub bond`]), rule: TRATED }),
  tr('journal on its shaft', 'rotating fit', { by: 'the bushing on the pivot pin', rating: (d, a, b) => { const t = side(a, b), w = bushOf(a, b); return t && w ? [`tail bushing ${t} ${w}`, `tail pivot pin ${t} at the outer bushing`] : []; }, rule: TSEAT }, { by: "the pin's stops: the washer and the horn's hub on the two flanges", rating: nm([(t) => `tail pin washer ${t}`, (t) => `tail horn ${t} hub bond`]), rule: TRATED }),
  tr('bonded on the pin', 'bonded', { by: "the horn's hub bond (Araldite 2011 on aluminium)", rating: nm([(t) => `tail horn ${t} hub bond`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail horn ${t} hub bond`]), rule: TBOND }),
  tr("its hub's face on the flange (the pin's axial stop)", 'thrust', { by: "the horn's hub on the outer flange", rating: nm([(t) => `tail horn ${t} hub bond`, (t) => `tail bushing ${t} flanges, axial thrust`]), rule: TBOND }, { by: "the horn's hub bond", rating: nm([(t) => `tail horn ${t} hub bond`]), rule: TBOND }),
  tr("the tube's end on its hub", 'resting', { by: "the tube's end bearing on the hub; the tube is carried by the spigot bond", rating: nm([(t) => `tail pivot pin ${t} spigot bond`]), rule: TBOND }, { by: 'the spigot bond', rating: nm([(t) => `tail pivot pin ${t} spigot bond`]), rule: TBOND }),
  tr(/^its ball on the (horn's arm|servo's arm), on its screw$/, 'pinned', { by: 'the rod end on its screw', rating: nm([(t) => `tail pushrod ${t} rod ends`, (t) => `tail pushrod ${t} screws`]), rule: TRATED }, { by: 'the screw', rating: nm([(t) => `tail pushrod ${t} screws`]), rule: "ISO 898-1 proof load (SPEC round 7)" }),
  tr('through the ball', 'fastened', { by: 'the screw in shear', rating: nm([(t) => `tail pushrod ${t} screws`]), rule: "ISO 898-1 proof load (SPEC round 7)" }, { by: 'the screw', rating: nm([(t) => `tail pushrod ${t} screws`]), rule: "ISO 898-1 proof load (SPEC round 7)" }),
  // round 18 (SPEC ruling "Maker's part"): the servo arm's tapped thread is the maker's part; the horn's is ours (6061)
  tr("screwed into the arm's tapped hole", 'threaded', { by: 'the screw in shear', rating: nm([(t) => `tail pushrod ${t} screws`]), rule: "ISO 898-1 proof load (SPEC round 7)" }, { by: "the thread in the arm: the horn's in 6061, the servo's arm the maker's (maker's part)", rating: (d, a, b) => { const t = side(a, b); return !t ? [] : /servo arm/.test(`${a} ${b}`) ? [`tail servo arm ${t}`] : [`tail horn ${t} screw thread in the arm`]; }, rule: `pin-in-shear on the thread's stripping area at the material's shear yield (the horn); the servo's arm: ${MAKERS_PART_RULE}` }),
  tr("on the servo's output spline (the maker's arm and its centre screw)", 'keyed', { by: "the maker's arm on the servo's spline", rating: nm([(t) => `tail servo arm ${t}`]), rule: MAKERS_PART_RULE }, { by: "the maker's centre screw (supplied with the servo)", rating: nm([(t) => `tail servo arm ${t}`]), rule: MAKERS_PART_RULE }, 'round 18: the R-ML25 arm and its centre screw are the maker\'s, with no published rating: a maker\'s part, valid while the servo is within its own rule at its supply\'s row (SPEC ruling, operator\'s decision 2026-11-03)'),
  tr("bonded in the tube's end", 'bonded', { by: 'the end plug bond (Araldite 2011 on CFRP)', rating: nm([(t) => `tail pushrod ${t} end plug bonds`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail pushrod ${t} end plug bonds`]), rule: TBOND }),
  tr("its stud in the rod end's thread", 'threaded', { by: "the rod end's rating on its thread", rating: nm([(t) => `tail pushrod ${t} rod ends`]), rule: TRATED }, { by: "the rod end's thread", rating: nm([(t) => `tail pushrod ${t} rod ends`]), rule: TRATED }),
  tr("its end on the rod end's shank", 'resting', { by: "the pushrod's force through the plug's stud into the rod end", rating: nm([(t) => `tail pushrod ${t} end plug bonds`, (t) => `tail pushrod ${t} rod ends`]), rule: TRATED }, { by: 'the plug bond and the stud', rating: nm([(t) => `tail pushrod ${t} end plug bonds`]), rule: TBOND }),
  tr('bonded on the tube', 'bonded', { by: "the rib's bond on the pivot tube (balsa-limited)", rating: nm([(t) => `tail ribs ${t} on the pivot tube, each`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail ribs ${t} on the pivot tube, each`]), rule: TBOND }),
  { why: new RegExp(`^${RIB_HOLE_TXT}$`), part: /^tail rib /, kind: 'through hole', load: { by: 'the rib on the tube (its bond) or on the trailing edge rod (crushing at half)', rating: (d, a, b) => { const t = side(a, b), w = `${a} ${b}`; return !t ? [] : /trailing edge rod/.test(w) ? [`tail ribs ${t} on the trailing edge rod, each`] : [`tail ribs ${t} on the pivot tube, each`]; }, rule: TBOND }, axial: { by: "the rib's bond on the tube; the rod's in its ribs", rating: (d, a, b) => { const t = side(a, b), w = `${a} ${b}`; return !t ? [] : /trailing edge rod/.test(w) ? [`tail trailing edge rod ${t} in its ribs`] : [`tail ribs ${t} on the pivot tube, each`]; }, rule: TBOND }, note: 'the pivot pin\'s hole rows: the pin runs inside the tube at the root rib; the rib\'s own interface is its bond on the tube' },
  { why: new RegExp(`^${BORE_TXT}$`), part: /^tail /, kind: 'in a bore', load: { by: 'the part in the bore: the spigot bond, the washer, the bushing in its boss or on its pin, the horn hub\'s bond', rating: tailBore, rule: TSEAT }, axial: { by: 'the same', rating: tailBore, rule: TRATED }, note: 'a tube along the span drawn as its capsule: the pair is the real fit it stands for' },
  tr("bonded on the boom's end", 'bonded', { by: 'the saddle bond on the boom (Araldite 2011 on CFRP)', rating: () => ['tail saddle bond on the boom'], rule: TBOND }, { by: 'the same bond', rating: () => ['tail saddle bond on the boom'], rule: TBOND }),
  // round 19: the pushrods' exit guides (the rod slides in its guide's bore, clear of it over the travel: no joint)
  tr("bonded in its lug's hole", 'bonded', { by: "the guide's bond in its lug (Araldite 2011 on CFRP)", rating: nm([(t) => `tail pushrod guide ${t} bond in its lug`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail pushrod guide ${t} bond in its lug`]), rule: TBOND }),
  tr("bonded on the former's aft face", 'bonded', { by: "the lug's bond on the former's band (Araldite 2011 on CFRP)", rating: nm([(t) => `tail pushrod guide lug ${t} bond on its former`]), rule: TBOND }, { by: 'the same bond', rating: nm([(t) => `tail pushrod guide lug ${t} bond on its former`]), rule: TBOND }),
];
