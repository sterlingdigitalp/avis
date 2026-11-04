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
import { S, cap, poly, ring, RIB_HOLE, BORE, THREAD, THROUGH, screwY } from './clearance.js';
import { byId, tubeSpec, tubeAllowable } from './catalog.js';
import { fastenerGeometry, shapeArea } from './completeness.js';
import { E387_THICKNESS } from '../../sims/avis-shared/model.js';

const mm = (v) => v / 1000;
const add2 = (p, q, k = 1) => [p[0] + k * q[0], p[1] + k * q[1]];
const rect = (o, u, v, [s0, s1], [n0, n1]) => { const P = (s, n) => [o[0] + s * u[0] + n * v[0], o[1] + s * u[1] + n * v[1]]; return poly([P(s0, n0), P(s1, n0), P(s1, n1), P(s0, n1)]); };
const boxZY = (z0, z1, y0, y1) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);

/** Default mechanism parameters (mm unless stated); params.tailMech overrides them. */
export const TAIL_MECH = {
  // the pivot tube in torsion needs a published shear strength: Rock West's T-RND-236 (6 x 5 mm) publishes its XY shear
  // and GXY (CLT, reference only), CST's pultrusions publish none
  pivot: 'tube-rockwest-pultruded-6x5', pin: 'metal-al-7075-t6', washer: 'washer-din6799-4', bushing: 'bushing-igus-GFM-0608-06', rodEnd: 'rodend-igus-KBRM-02', arm: 'horn-hitec-r-ml25',
  screw: 'screw-iso4762-m2x6-8.8', material: 'metal-al-6061-t6', pushrod: 'tube-cst-pultruded-5x4',
  saddle: { sleeve_len: 16, wall: 1.5, boss_wall: 1.0, web_t: 1.0, boss_len: 11, clear: 0.5 },
  horn: { hub_od: 8, hub_len: 4, arm_t: 3, arm_depth: 8, radius: 13 },
  arm_hole_mm: 16, spline_from_aft_mm: 14.5, plug_len: 8, rib_t: 2, pin_in_tube: 10, pin_bore: 4,
};

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
  const dist = (s) => Math.hypot(T.z_m + s * u0[0], s * u0[1]);
  const clearAt = (r, g) => { let s = 0; while (dist(s) < rb + r + g) s += 1e-5; return s; };
  // the pin's inboard end (n past its groove), the washer, the inner flange, the boss, the outer flange, the horn's hub,
  // the tube on the pin, the root rib: each the first place clear of the boom
  const W = byId(catalog, P.washer).specs, rW = mm(W.outer_diameter_d3_mm) / 2, sW = mm(W.thickness_s_mm), nW = mm(W.min_groove_to_shaft_end_n_mm), rPin = mm(pv.od_draw) / 2;
  const sW0 = Math.max(clearAt(rW, run), clearAt(rPin, run) + nW), sP0 = sW0 - nW, sF1 = sW0 + sW;
  const sB0 = Math.max(sF1 + bF, clearAt(d3 / 2, run) + bF), sB1 = sB0 + mm(S1.boss_len), sF2 = sB1 + bF;
  const sH0 = sF2, sH1 = sH0 + mm(H.hub_len), sT0 = sH1, sP1 = sT0 + mm(P.pin_in_tube);
  const tRib = mm(P.rib_t), sRoot = sT0 + tRib / 2 + run;
  return { rT, d2, d3, bF, bL, rBoss, rb, rW, sW, sW0, sP0, sP1, rPin, sB0, sB1, sF1, sF2, sH0, sH1, sT0, sRoot, tRib, run, hornR: mm(H.radius), hornT: mm(H.arm_t), hornD: mm(H.arm_depth), hubR: mm(H.hub_od) / 2, xq: T.x_m, L: T.span_each_m, ri: mm(pv.id_draw) / 2, groove_d: mm(W.groove_diameter_d2_mm), groove_m: mm(W.groove_width_m_mm) };
}

/** The surface's frame: its root on the boom's side o, u along its dihedral, v normal to it (the (z, y) plane). */
export function tailFrame(T, yB, sg) {
  return { o: [sg * T.z_m, yB], u: [sg * Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)], v: [-sg * Math.sin(T.dihedral_rad), Math.cos(T.dihedral_rad)] };
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
const chordOf = (T, s) => T.chord_root_m * (1 - (1 - T.taper) * (s / T.span_each_m));

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
  const saddle = saddleSolids(T, yB, boomR, G, P);
  const rows = [];
  for (const [tag, sg] of [['R', 1], ['L', -1]]) {
    const o = [sg * T.z_m, yB], u = [sg * Math.cos(T.dihedral_rad), Math.sin(T.dihedral_rad)], v = [-sg * Math.sin(T.dihedral_rad), Math.cos(T.dihedral_rad)];
    // the pivot pin (7075, grooved at its inboard end) through both bushings and the horn's hub into the tube's bore; the
    // pivot tube from the horn's hub to the tip (its ribs and trailing edge rod on it)
    const pinId = add(`tail pivot pin ${tag}`, [tube('pin', o, u, G.sP0, G.sT0, G.rPin), tube('spigot', o, u, G.sT0, G.sP1, G.ri - G.run / 10)], { bom: 'tail pivot pins' });
    const pid = add(`tail pivot tube ${tag}`, [tube('tube', o, u, G.sT0, G.L, G.rT, G.ri)], { bom: 'tail pivot tubes' });
    join(pinId, pid, 'its spigot bonded in the tube\'s bore'); join(`${pinId}/spigot`, `${pid}/tube`, BORE);
    const wid = add(`tail pin washer ${tag}`, [tube('washer', o, u, G.sW0, G.sW0 + G.sW, G.rW, G.groove_d / 2)], { bom: P.washer });
    join(wid, pinId, 'in its groove on the pin'); join(`${wid}/washer`, `${pinId}/pin`, BORE); join(wid, `tail bushing ${tag} inner`, 'against the flange (the pin\'s axial stop)');
    for (const [nm, s0, s1, dir] of [['inner', G.sB0 - G.bF, G.sB0 - G.bF + G.bL, 1], ['outer', G.sB1 + G.bF - G.bL, G.sB1 + G.bF, -1]]) {
      const fl = dir > 0 ? [s0, s0 + G.bF] : [s1 - G.bF, s1], body = dir > 0 ? [s0 + G.bF, s1] : [s0, s1 - G.bF];
      const bid = add(`tail bushing ${tag} ${nm}`, [tube('body', o, u, body[0], body[1], G.d2 / 2, G.rT), tube('flange', o, u, fl[0], fl[1], G.d3 / 2, G.rT)], { bom: P.bushing });
      join(bid, 'tail saddle', 'pressed in the housing'); join(bid, pinId, 'journal on its shaft');
      // the bores are not drawn (a tube along the span is drawn filled): the body in the boss's bore, the pin in both
      join(`${bid}/body`, `tail saddle/boss ${tag}`, BORE); join(`${bid}/body`, `${pinId}/pin`, BORE); join(`${bid}/flange`, `${pinId}/pin`, BORE);
    }
    // the horn: its hub bonded on the tube, its arm along -v to the rod end's screw at hornR
    const sHm = (G.sH0 + G.sH1) / 2;
    const hid = add(`tail horn ${tag}`, [tube('hub', o, u, G.sH0, G.sH1, G.hubR, G.rPin), S('arm', [xq - G.hornD / 2, xq + G.hornD / 2], rect(o, u, v, [sHm - G.hornT / 2, sHm + G.hornT / 2], [-(G.hornR + 3e-3), -G.hubR + 1e-4]))], { bom: 'tail horns' });
    join(hid, pinId, 'bonded on the pin'); join(`${hid}/hub`, `${pinId}/pin`, BORE); join(hid, `tail bushing ${tag} outer`, 'its hub\'s face on the flange (the pin\'s axial stop)'); join(hid, pid, 'the tube\'s end on its hub');
    // the horn's rod end: its ball on the arm's inboard face at hornR (outboard, it would meet the root rib), its bore
    // along u; its shank forward along +x
    const sE = sHm - G.hornT / 2 - mm(ke.B_mm) / 2, nE = -G.hornR, D2 = mm(ke.d2_mm), Lsh = mm(ke.h1_mm);
    const eH = add(`tail rod end ${tag} horn`, [S('eye', [xq - D2 / 2, xq + D2 / 2], rect(o, u, v, [sE - mm(ke.B_mm) / 2, sE + mm(ke.B_mm) / 2], [nE - D2 / 2, nE + D2 / 2])), S('shank', [xq + D2 / 2, xq + Lsh], ring(add2(add2(o, u, sE), v, nE), mm(ke.d5_mm) / 2))], { bom: P.rodEnd });
    // its screw along u: the head inboard of the ball, the shank through the ball into the arm's tapped hole
    const sid = add(`tail horn screw ${tag}`, [tube('head', o, u, sE - mm(ke.B_mm) / 2 - sg0.head_k, sE - mm(ke.B_mm) / 2, sg0.head_d / 2, 0, nE, v), tube('shank', o, u, sE - mm(ke.B_mm) / 2, sE - mm(ke.B_mm) / 2 + sg0.L, sg0.d / 2, 0, nE, v)], { bom: P.screw });
    join(eH, hid, 'its ball on the horn\'s arm, on its screw'); join(sid, eH, 'through the ball'); join(`${sid}/shank`, hid, THREAD); join(sid, hid, 'screwed into the arm\'s tapped hole');
    join(`${sid}/shank`, eH, THROUGH);
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
    join(eS, aid, 'its ball on the servo\'s arm, on its screw'); join(sid2, eS, 'through the ball'); join(`${sid2}/shank`, eS, THROUGH); join(`${sid2}/shank`, aid, THREAD); join(sid2, aid, 'screwed into the arm\'s tapped hole');
    // the pushrod between the two rod ends' shank ends, with an end plug bonded in each end, its stud in the rod end
    const A = { x: xs - Lsh, p: [zH, armY[1] + mm(ke.B_mm) / 2] }, B = { x: xq + Lsh, p: add2(add2(o, u, sE), v, nE) };
    const ts = tubeSpec(byId(catalog, P.pushrod)), pr = pushrodPieces(A, B, mm(ts.od_draw) / 2, mm(ts.id_draw) / 2, mm(P.plug_len));
    const rodId = add(`tail pushrod ${tag}`, pr.solids, { bom: 'tail pushrods' });
    for (const [end, piece, eye] of [['servo', pr.solids[0], eS], ['horn', pr.solids.at(-1), eH]]) {
      // the plug fills its end piece's bore (its centre that piece's), its M2 stud into the rod end's female thread
      const c = piece.shape.c, atA = end === 'servo', xe = atA ? A.x : B.x, dir = atA ? 1 : -1; // dir: from the tube's end toward its rod end
      const plug = add(`tail pushrod plug ${tag} ${end}`, [S('plug', piece.x, ring(c, pr.ri - 1e-5)), S('stud', dir > 0 ? [xe, xe + mm(ke.l1_mm)] : [xe - mm(ke.l1_mm), xe], ring(atA ? A.p : B.p, 1e-3))], { bom: 'tail pushrod end plugs' });
      join(plug, rodId, 'bonded in the tube\'s end'); join(plug, eye, 'its stud in the rod end\'s thread'); join(`${plug}/stud`, eye, THREAD);
      join(rodId, eye, 'its end on the rod end\'s shank');
    }
    // ribs on the pivot tube from the root rib (its face on the outer flange) to the tip, the trailing edge rod through them
    const rs = ribs(G.sRoot);
    const te = add(`tail trailing edge rod ${tag}`, rs.slice(0, -1).map((a, k) => {
      const bb = rs[k + 1], x0 = xq - 0.75 * chordOf(T, a) + r, x1 = xq - 0.75 * chordOf(T, bb) + r;
      return { ...S(`bay ${k + 1}`, [Math.min(x0, x1) - r, Math.max(x0, x1) + r], cap(add2(o, u, a + G.tRib / 2), add2(o, u, bb - G.tRib / 2), r)), tube: { ro: r, ri: 0, L: ((bb - a) * (G.L - G.sRoot + G.tRib / 2)) / (rs[3] - rs[0]), axis: 'span' } };
    }), { bom: 'tail trailing edge rods' });
    rs.forEach((s, k) => {
      const c = chordOf(T, s), h = E387_THICKNESS * c;
      const id = add(`tail rib ${tag} ${k + 1}`, [{ ...S('rib', [xq - 0.75 * c + 2 * r, xq + 0.25 * c], rect(o, u, v, [s - G.tRib / 2, s + G.tRib / 2], [-h / 2, h / 2])), envelope: true, rib_t: G.tRib }], { bom: 'tail ribs' });
      join(id, pid, 'bonded on the tube'); join(id, pid, RIB_HOLE); join(id, te, RIB_HOLE); join(id, pinId, RIB_HOLE);
    });
    rows.push({ tag, o, u, v, sE, nE, xs, zs, zH, armY, A, B, pushrod_L: pr.L, ribs: rs });
  }
  add('tail saddle', saddle, { bom: 'tail saddle' });
  join('tail saddle', 'tail boom', 'bonded on the boom\'s end');
  for (const t of ['R', 'L']) join('tail saddle', `tail pivot pin ${t}`, BORE);
  assumptions.push(`tail (round 17, params.tailMech, src/design/tail-mech.js): each V surface all-moving on its quarter chord: a 7075 pivot pin in two ${P.bushing} in a 6061 saddle bonded on the boom's end (bosses ${P.saddle.boss_len} mm long), held axially between a ${P.washer} in its groove and the horn's hub on the two flanges, the ${P.pivot} tube bonded on its outboard end; driven by its MD89MW through the maker's ${P.arm} (hole at ${P.arm_hole_mm} mm), two ${P.rodEnd} and a ${P.pushrod} pushrod with bonded 6061 end plugs to a 6061 horn (${P.horn.radius} mm) bonded on the pin; the servo's spline position on its case is not published, drawn on its top face's centre line ${P.spline_from_aft_mm} mm from its aft end; drawn at zero deflection (the travel is checked by the chain record)`);
  return { G, rows, P };
}

/**
 * A straight pushrod from A to B (x and the (z, y) centre at each end) as x-pieces: an end piece `endLen` long at each
 * end (its plug's), the rest at most 20 mm; each a ring round the piece's mid-point that holds the tube's slope over
 * the piece (its outside grown by half the piece's offset; its bore drawn at the tube's own).
 */
export function pushrodPieces(A, B, ro, ri, endLen, piece = 0.02) {
  const dx = B.x - A.x, L = Math.hypot(dx, B.p[0] - A.p[0], B.p[1] - A.p[1]), Lx = Math.abs(dx), e = Math.min(endLen, Lx / 3) / Lx;
  const n = Math.max(1, Math.ceil((Lx * (1 - 2 * e)) / piece)), ts = [0, ...Array.from({ length: n + 1 }, (_, k) => e + (k / n) * (1 - 2 * e)), 1];
  const at = (t) => [A.p[0] + t * (B.p[0] - A.p[0]), A.p[1] + t * (B.p[1] - A.p[1])];
  const solids = ts.slice(0, -1).map((t0, k) => {
    const t1 = ts[k + 1], c = at((t0 + t1) / 2), half = Math.hypot(...at(t1).map((v, i) => v - at(t0)[i])) / 2;
    const x0 = A.x + t0 * dx, x1 = A.x + t1 * dx;
    return { ...S(`piece ${k + 1}`, [Math.min(x0, x1), Math.max(x0, x1)], ring(c, ro + half, ri)), tube: { ro, ri, L: L * (t1 - t0), axis: 'x' } };
  });
  return { solids, L, ri };
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
  const saddleV = saddleSolids(T, yB, boomR, G, P).reduce((a, q) => a + shapeArea(q.shape) * (q.x[1] - q.x[0]), 0);
  const hornV = Math.PI * (G.hubR ** 2 - G.rPin ** 2) * (G.sH1 - G.sH0) + G.hornT * (G.hornR + 3e-3 - G.hubR + 1e-4) * G.hornD;
  const Lp = mm(P.pushrod_L_mm ?? NaN), plugX = mm(P.plug_len), plugV = Math.PI * (G.ri - 1e-5) ** 2 * plugX + Math.PI * 1e-6 * mm(ke.l1_mm);
  const LTE = G.L - G.sRoot + G.tRib / 2;
  const gm = (id) => tubeSpec(byId(catalog, id)).g_per_m;
  const structure = [
    { name: 'tail pivot pins, 2 off', material: al7, mass_g: 2 * pinV * rho(al7), basis: `2 x a ${(G.rPin * 2000).toFixed(2)} mm pin x ${((G.sT0 - G.sP0) * 1000).toFixed(1)} mm (in both bushings and the horn's hub, its inboard end grooved for a ${P.washer}) and a ${((G.ri - G.run / 10) * 2000).toFixed(2)} mm spigot x ${P.pin_in_tube} mm bonded in the pivot tube; ${al7.specs.density_g_cm3} g/cm3`, count: 2 },
    { name: 'tail pivot tubes, 2 off', material: pv, mass_g: 2 * gm(P.pivot) * (G.L - G.sT0), basis: `${gm(P.pivot)} g/m x ${(G.L - G.sT0).toFixed(4)} m x 2 (the horn's hub to the tip, on the quarter chord)`, count: 2 },
    { name: 'tail trailing edge rods, 2 off', material: te, mass_g: 2 * te.mass_g * LTE, basis: `${te.mass_g} g/m x ${LTE.toFixed(4)} m x 2 (root rib to tip rib)`, count: 2 },
    { name: 'tail saddle', material: al6, mass_g: saddleV * rho(al6), basis: `a ${P.saddle.sleeve_len} mm sleeve ${((boomR + mm(P.saddle.wall)) * 2000).toFixed(1)}/${(boomR * 2000).toFixed(2)} mm bonded on the boom's end, and per side a ${P.saddle.boss_len} mm boss round two ${P.bushing} on the pivot axis with its webs to the sleeve (${(saddleV * 1e9).toFixed(0)} mm3, the drawn solids); ${al6.specs.density_g_cm3} g/cm3` },
    { name: 'tail horns, 2 off', material: al6, mass_g: 2 * hornV * rho(al6), basis: `each a ${P.horn.hub_od} mm hub x ${P.horn.hub_len} mm bonded on the pivot pin and a ${P.horn.arm_t} x ${P.horn.arm_depth} mm arm to its rod end's screw at ${P.horn.radius} mm; ${al6.specs.density_g_cm3} g/cm3`, count: 2 },
    { name: 'tail pushrods, 2 off', material: pr, mass_g: 2 * gm(P.pushrod) * Lp, basis: `${gm(P.pushrod)} g/m x ${Lp.toFixed(4)} m x 2 (between the two rod ends' shanks)`, count: 2 },
    { name: 'tail pushrod end plugs, 4 off', material: al6, mass_g: 4 * plugV * rho(al6), basis: `each a ${((G.ri - 1e-5) * 2000).toFixed(2)} mm plug x ${P.plug_len} mm bonded in the pushrod's end with an M2 stud x ${ke.l1_mm} mm into the rod end; ${al6.specs.density_g_cm3} g/cm3`, count: 4 },
  ];
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
  const lam = T.taper, sBar = (T.span_each_m * (1 + 2 * lam)) / (3 * (1 + lam)); // the area centroid along the span
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
    members.push({ name: `tail saddle webs ${t}`, kind: 'plate', check: 'plate', material: al6, dims: { b_mm: +(2 * G.rBoss * 1000).toFixed(2), t_mm: 2 * P.saddle.wall }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +(sBar - G.sB0).toFixed(6) }])], uses: { M: 'M' }, note: 'the root moment at the boss\'s inboard end in the plane of its two webs (each the saddle\'s wall thick, as deep as the boss)' });
    members.push({ name: `tail pivot tube ${t} at its root`, kind: 'tube', check: 'cfrp tube', material: pv.composite, dims: { d_mm: pv.od_rate, di_mm: pv.id_rate }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +armT.toFixed(6) }]), sect('T', Mh), sect('V', F)], uses: { M: 'M', T: 'T', V: 'V' }, note: `the surface's force at its area centroid about the tube's root on the pin (${(armT * 1000).toFixed(0)} mm), its hinge moment` });
    members.push({ name: `tail pivot pin ${t} spigot`, kind: 'pin', check: 'round', material: al7, dims: { d_mm: +((G.ri - G.run / 10) * 2000).toFixed(3), di_mm: 0 }, sections: [sect('M', [{ ch: `tail_F_${t}`, k: +armT.toFixed(6) }]), sect('T', Mh)], uses: { M: 'M', T: 'T' }, note: 'the spigot in the tube\'s bore at the tube\'s root: the root moment there and the hinge moment' });
    members.push({ name: `tail pivot pin ${t} spigot bond`, kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: +(G.ri * 2000).toFixed(3), L_mm: P.pin_in_tube }, sections: [sect('T', Mh), sect('F', inSurf)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half): the hinge moment and the surface\'s inertia along its span' });
    members.push({ name: `tail horn ${t} hub bond`, kind: 'bond', check: 'bond', material: glueAl, dims: { d_mm: +(G.rPin * 2000).toFixed(3), L_mm: P.horn.hub_len }, sections: [sect('T', Mh)], uses: { T: 'T' }, note: 'Araldite 2011 on aluminium: the hinge moment' });
    members.push({ name: `tail horn ${t} arm`, kind: 'plate', check: 'plate', material: al6, dims: { b_mm: P.horn.arm_depth, t_mm: P.horn.arm_t }, sections: [sect('M', [{ ch: `tail_M_${t}`, k: +((Rh - G.hubR) / Rh).toFixed(6) }])], uses: { M: 'M' }, note: 'the pushrod\'s force at the screw, bending the arm in its plane at the hub' });
    members.push({ name: `tail pin washer ${t}`, kind: 'retainer', check: 'rated', dims: { rating_N: 1000 * Math.min(W.load_columns.FN_kN, W.load_columns.Fs_kN), rating_basis: `${P.washer}: the lower of FN ${W.load_columns.FN_kN} kN (groove) and Fs ${W.load_columns.Fs_kN} kN (washer) as tabulated (DIN 6799 convention; the groove's figure is for steel shafts and the pin is 7075)` }, sections: [sect('F', inSurf)], uses: { F: 'F' }, note: 'the surface\'s inertia along its span (every axis counted)' });
    members.push({ name: `tail pushrod ${t}`, kind: 'strut', check: 'strut', material: { id: P.pushrod, E_GPa: pr.E_GPa, yield_MPa: pr.sigma_MPa }, dims: { L_mm: +(P.pushrod_L_mm ?? 0).toFixed(1), d_mm: pr.od_rate, di_mm: pr.id_rate }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: `the hinge moment over the horn's ${(Rh * 1000).toFixed(0)} mm, pinned at its rod ends: Euler on the published modulus, the strength at ${pr.basis}` });
    members.push({ name: `tail pushrod ${t} rod ends`, kind: 'rod end', check: 'rated', dims: { rating_N: ke.max_static_tensile_long_term_n, rating_basis: `${P.rodEnd}: maximum static tensile load, long term (igus)` }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: 'the pushrod\'s force through each rod end' });
    members.push({ name: `tail pushrod ${t} screws`, kind: 'fasteners', check: 'bolts', fastener: fastenerOfCat(catalog, P.screw), dims: { n: 1 }, sections: [sect('V', Fp)], uses: { V: 'V' }, note: 'each rod end\'s M2 screw in shear' });
    members.push({ name: `tail pushrod ${t} end plug bonds`, kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: +(G.ri * 2000).toFixed(3), L_mm: P.plug_len }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half): the pushrod\'s force on each end plug' });
    if (sv) members.push({ name: `tail servo arm ${t}`, kind: 'arm', check: 'rated', dims: { rating_N: +(sv.servo_stall_Nm ?? sv.stall_Nm * (sv.linkage_ratio ?? 1)) / rArm, rating_basis: `the maker's own arm for this servo (${P.arm}, supplied with the MD89MW): no load rating of its own is published, so it is held to the servo's published stall torque at its ${P.arm_hole_mm} mm hole` }, sections: [sect('F', Fp)], uses: { F: 'F' }, note: 'the pushrod\'s force at the arm\'s hole' });
    if (tieL) members.push({ name: `tail servo ${t} ties under its arm's reaction`, kind: 'tie', check: 'tie', tie: { id: tieL.id, loop_N: tieL.loop_N }, dims: { n: 2 }, sections: [sect('F', [...Fp, ...inertial(sv?.mass_kg ?? 0.0215, [-0.04, -0.065, 0])])], uses: { F: 'F' }, note: 'the servo\'s two T40R carry the pushrod\'s force at its arm and its own inertia' });
  }
  members.push({ name: 'tail saddle bond on the boom', kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: boomR * 2000, L_mm: P.saddle.sleeve_len }, sections: [sect('T', [{ ch: 'tail_F_R', k: +arm.toFixed(6) }, { ch: 'tail_F_L', k: +arm.toFixed(6) }]), sect('F', [{ ch: 'tail_F', k: 1 }])], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half) on the boom\'s end: both surfaces\' root moments as torsion (their sum, conservative) and the tail\'s force' });
}

const inertial = (m, r) => { const R = Math.hypot(...r); return [{ ch: 'sf_origin', k: m }, { ch: 'wdot', k: m * R }, { ch: 'w2', k: m * R }]; };
function fastenerOfCat(catalog, id) { const f = byId(catalog, id), dv = f.specs.design_values ?? {}; return { id, tension_N: dv.tension_N, shear_N: dv.shear_N }; }
function tubeAllowableOf(catalog, id) {
  const p = byId(catalog, id), t = tubeSpec(p), a = tubeAllowable(p), s = p.specs;
  // the shear allowable from the maker's transverse record (Rock West: composite-rockwest-<part>-transverse), else none
  const tr = s.part_number ? catalog.find((q) => q.id === `composite-rockwest-${s.part_number.toLowerCase()}-transverse`) : null;
  return { od_rate: t.od_rate, id_rate: t.id_rate, E_GPa: a.E_GPa, sigma_MPa: a.sigma_MPa, basis: a.basis, composite: { id, axial_MPa: a.sigma_MPa, shear_MPa: tr?.specs.design_values.xy_shear_MPa ?? null, shear_basis: tr?.specs.design_values.xy_shear_basis ?? 'no published shear strength', note: s.strength_note ?? null } };
}
