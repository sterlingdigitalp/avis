// What a real build needs (round 17b, SPEC "Function completeness" -> "What a real build needs"; audit-v24.md items 5,
// 6, 18, 20, 21, 24): connectors for every electrical joint the assembly order makes, a hatch with a rated fixing
// through which the pack comes out to charge and the bird is armed, bound and set up, a releasable pack strap, the
// pitot's pressure tubing, the receiver's antenna, and a declared air path for the parts that make heat.
//  - Frame: x forward, y up, z to the right (m). Every solid is a prism along x with a (z, y) section, as clearance.js
//    draws them; a moving solid's section is a function of the drive pose.
//  - Everything that has a catalogue size is drawn at it; what has none (the AMASS T-plug's and the goBILDA TJC8's
//    housings, the leads' diameters, the strap's thickness, the antenna's length) is drawn at a stated, disclosed size.
//  - The hatch: two 6061 rails on four 6061 posts bonded to the nose tray's flanges (fixed); a cover of two 6061 lands
//    on the rails with the pack's aft stop and two keepers over the pack, and 0.5 mm 6061 arcs on the skin that carry the
//    window's film, held by ISO 4762 M3 10.9 screws into ISO 4032 nuts under the rails (airframe.js counts them in
//    the M3 lines; fixingParts draws them). A fixed 6061 stop under the nose tray's tip holds the pack forward.
//  - The FC and the servo regulator move to a belly tray under the nose tray (lead's CG note, 2026-11-03): the FC's
//    USB-C extender (20 cm JST-SH-6P, Matek page) then reaches the hatch, the ASPD-4525's 20 cm cable reaches the FC,
//    and the CG comes forward. The servos reach the FC through goBILDA extensions drawn along a harness under the trays.
import { S, M, cap, poly, RIB_HOLE, bandSlabs, gap2, gap3, EXEMPT_JOINTS } from './clearance.js';
import { slideGap } from './sequence.js';
import { byId } from './catalog.js';
import { shapeArea } from './completeness.js';
import { loftHalf } from '../../sims/avis-shared/model.js';
import { pickScrew } from './fixings.js';

const mm = (v) => v / 1000;
const boxYZ = (z0, z1, y0, y1) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);
const box3 = (name, [x0, x1], [z0, z1], [y0, y1]) => S(name, [x0, x1], boxYZ(z0, z1, y0, y1));
const r3 = (v) => +(v * 1000).toFixed(3);
/** A section's (z, y) bounding box: { z: [z0, z1], y: [y0, y1] } (ring, capsule or polygon). */
export function bboxOf(sh) {
  if (sh.kind === 'ring') return { z: [sh.c[0] - sh.ro, sh.c[0] + sh.ro], y: [sh.c[1] - sh.ro, sh.c[1] + sh.ro] };
  if (sh.kind === 'cap') return { z: [Math.min(sh.a[0], sh.b[0]) - sh.r, Math.max(sh.a[0], sh.b[0]) + sh.r], y: [Math.min(sh.a[1], sh.b[1]) - sh.r, Math.max(sh.a[1], sh.b[1]) + sh.r] };
  const z = sh.pts.map((p) => p[0]), y = sh.pts.map((p) => p[1]);
  return { z: [Math.min(...z), Math.max(...z)], y: [Math.min(...y), Math.max(...y)] };
}

/** Default build-needs parameters (mm unless stated); params.needs overrides them by block. */
export const NEEDS = {
  material: 'metal-al-6061-t6',
  hatch: {
    rail_t: 1.5, rail_bottom_y: -14, rail_z_out: 25.5, rail_x: [134, 222],
    posts_x: [[154.5, 162.5], [178, 186]], post_t: 1.0,
    land_t: 0.8, screws_x: [145, 195], screw_z: 22.3,
    screws: ['screw-iso4762-m3x8-10.9', 'screw-iso4762-m3x12-10.9'], nut: 'nut-iso4032-m3-10',
    // round 18 (sized to the edge-distance rule, EN 1993-1-8 Table 3.3: at least 1.2 d0 from every edge): the M3's 3.4 mm
    // hole sat 3.2 mm (0.94 d0) from both edges of the 6.4 mm rails, and the rails cannot widen (outward the lands leave
    // the skin and meet the pod's longerons at polar 20/160, inward is the pack's way out). The cover's four screws are
    // M2 (ISO 273 medium hole 2.4 mm: 3.2 mm = 1.33 d0); the belly tray keeps its M3
    cover_screws: ['screw-iso4762-m2x8-8.8'], cover_nut: 'nut-iso4032-m2-8', cover_hole_mm: 2.4,
    arcs_x: [[134, 134.5], [164.75, 165.25], [199.75, 200.25], [221.5, 222]], arc_band: 3, leg_z: [23.5, 25.5], arc_material: 'metal-al-6061-t6',
    // round 18: each arc's band runs on past its legs to this far (along the skin) short of the window's polar edges, so
    // the cover's film reaches the pod's edge longerons all along the window (the pod's 1.5 mm rod and 0.5 mm clear)
    arc_edge_mm: 3,
    stop_aft: { x: [155, 156], z: 18.0, lip_x: 6, lip_z: 8, t: 1.0 },
    keepers_x: [[217.5, 221.5]], keeper_t: 1.5, keeper_z: [17.3, 18.6],
  },
  stop_fwd: { gap: 0.5, t: 1.0, leg_x: 8, z: 10, top_y: -30 },
  strap: { id: 'strap-rctimer-hookloop-15x330', t_assumed: 1.5, x_mm: 174.3 },
  plugs: { male: 'connector-amass-tplug-male', female: 'connector-amass-tplug-female', size_assumed: [15.9, 15.9, 7.9], pack_x0: 182.3, arming_x0: 198.7 },
  tubing: { id: 'tube-saint-gobain-versilon-spt50-094x156', y: [-35, -30.5], z: 24, turn_x: [152, 148], ports_x: [151, 140], port_z: 12, tie_x: 164.25 },
  antenna: { d: 1.13, z: 32.5, y: -35, x_start: 137, x_end: 212, tie_x: 203.25 },
  // the harness's corridor under the trays (lane 0 the twist-servo lead, 1 the tail lead): out and up outside the forward
  // tray between former 3 and the riser's screws, lane 0 bending out past lane 1's riser
  harness: { y_drop: 4.5, y_rise: 2.05, z_lanes: [12, 15.8], z_side: 22.5, x_up: [104.5, 100.7], y_fc: [-63, -59], bend0: [[98.9, 12], [99.9, 14.5], [103.2, 17.5]], y_over: -53.5, z_plug: 4.3 },
  belly: { x: [110.5, 178], y_top: -69.3, t: 0.8, w: 35, h: 5, tab: 9, screws: 2, screw_pitch: 22 },
  cooling: { inlet: { x: 236, polar: -90, d: 8 }, outlet: { x: -105, polar: -90, d: 8 }, Cd_internal: 1.0 },
  servoLead: { length_mm: 240, gauge_awg: 22, source: 'https://hitecrcd.com/md89mw-magnetic-encoder-metal-gear-25t-digital-micro-servo/', quote: 'Connector Wire Length: 240mm (9.449inch)', read: '2026-11-03' },
  leads: { d_assumed: 3.3, housing_assumed: [12, 7.6, 2.6], slack_frac: 0.1, x_gap: 0.8, plug_x: [137, 119], wing: 'cable-gobilda-servo-ext-450', pod: 'cable-gobilda-servo-ext-600', tail: 'cable-gobilda-servo-ext-150' },
  // (the wing side 450 mm: with the twist drive's servo frame (twist-drive.js) the run from the servo's own plug to the
  // wing-root connector is 286 mm drawn, 315 mm with the slack, past the 300 mm lead; the pod side then 600 mm, a
  // length of its own, so each BOM line has one place: the wing's on the wing, the pod's in the build-needs lump)
};

/** The design's build-needs parameters: NEEDS with params.needs merged block by block. */
export function needsParams(p) {
  const o = p?.needs ?? {}, out = {};
  for (const [k, v] of Object.entries(NEEDS)) out[k] = v && typeof v === 'object' && !Array.isArray(v) ? { ...v, ...(o[k] ?? {}) } : o[k] ?? v;
  return out;
}

/** The nose tray as airframe.js carrierGeometry draws it, from its stated parameters (m). */
export function noseTrayOf(A) {
  const N = A.noseTray;
  return { y: mm(N.y_mm), w: mm(N.w_mm), h: mm(N.flange_h_mm), t: mm(N.t_mm), x: [mm(N.riser_x_mm), mm(N.tip_x_mm)] };
}

// ---- the hatch ------------------------------------------------------------------------------------------------

/** One arc's band pieces on the skin at x, from polar angle a0 to a1 (deg), band b deep: convex quads in (z, y). */
export function arcPieces(loft, x, [a0, a1], b, n = 12) {
  const h = loftHalf(loft, x);
  const at = (a, k) => [(h.hw - k) * Math.cos(a), h.y + (h.hh - k) * Math.sin(a)];
  const rad = (d) => (d * Math.PI) / 180;
  return Array.from({ length: n }, (_, i) => {
    const t0 = rad(a0 + ((a1 - a0) * i) / n), t1 = rad(a0 + ((a1 - a0) * (i + 1)) / n);
    return [at(t0, 0), at(t1, 0), at(t1, b), at(t0, b)];
  });
}

/** The polar angle (deg) where the loft at x meets the vertical z = zv above its centre. */
export function polarAtZ(loft, x, zv) {
  const h = loftHalf(loft, x), c = Math.max(-1, Math.min(1, zv / h.hw));
  return (Math.atan2(Math.sqrt(1 - c * c), c) * 180) / Math.PI;
}

/**
 * The hatch's geometry (m) on a loft and the nose tray N: its fixed rails and posts, the cover's lands, aft stop and
 * keepers (6061), its arcs (6061, on the skin), the screws' places, and the window it frames (x, polar range). With the
 * window `win` ({ x_mm, polar_deg }: params.layout.hatch) each arc's band spans the window's polar range less
 * P.hatch.arc_edge_mm at each edge (never short of its own legs); without it, from leg to leg.
 */
export function hatchGeometry(loft, P, N, win = null) {
  const H = P.hatch, zIn = N.w / 2 + N.t / 2, yRb = mm(H.rail_bottom_y), yRt = yRb + mm(H.rail_t), yLt = yRt + mm(H.land_t);
  const rails = ['R', 'L'].map((t, i) => { const s = i ? -1 : 1; return { tag: t, s, x: H.rail_x.map(mm), z: [zIn, mm(H.rail_z_out)].map((v) => s * v).sort((a, b) => a - b), y: [yRb, yRt] }; });
  const posts = rails.flatMap((r) => H.posts_x.map((xs, k) => ({ tag: `${r.tag} ${k ? 'forward' : 'aft'}`, side: r.tag, x: xs.map(mm), z: [zIn, zIn + mm(H.post_t)].map((v) => r.s * v).sort((a, b) => a - b), y: [N.y - N.t / 2, yRb] })));
  const lands = rails.map((r) => ({ ...r, y: [yRt, yLt] }));
  const screws = rails.flatMap((r) => H.screws_x.map((x) => ({ side: r.tag, x: mm(x), z: r.s * mm(H.screw_z) })));
  const sa = H.stop_aft, stop = { x: sa.x.map(mm), z: mm(sa.z), y: [N.y + N.t / 2 + 0.0048, yRt], lip: { x: [mm(sa.x[1]), mm(sa.x[1] + sa.lip_x)], z: mm(sa.lip_z) } };
  const keepers = H.keepers_x.map((xs) => ({ x: xs.map(mm), z: H.keeper_z.map(mm), t: mm(H.keeper_t), yTop: yRt }));
  const legs = H.leg_z.map(mm);
  const arcs = H.arcs_x.map((xs) => {
    const x = xs.map(mm), xm = Math.min(...x.map((v) => loftHalf(loft, v).hw)) === loftHalf(loft, x[0]).hw ? x[0] : x[1];
    const a0 = polarAtZ(loft, xm, legs[1]), h = loftHalf(loft, xm), ySkin = h.y + h.hh * Math.sin((a0 * Math.PI) / 180);
    // (and never down past where its inner edge comes within 0.5 mm of the lands' top, over the rails)
    const aLand = polarAbove(h, mm(H.arc_band), yLt + 0.0005);
    const polar = win ? [Math.min(a0, Math.max(aLand, win.polar_deg[0] + edgeDeg(h, win.polar_deg[0], mm(H.arc_edge_mm ?? 3)))), Math.max(180 - a0, Math.min(180 - aLand, win.polar_deg[1] - edgeDeg(h, win.polar_deg[1], mm(H.arc_edge_mm ?? 3))))] : [a0, 180 - a0];
    return { x, xm, polar, legs: [legs, legs.map((v) => -v).reverse()], leg_y: [yLt, ySkin - mm(H.arc_band) * 0.5] };
  });
  const xs = [mm(H.rail_x[0]), mm(H.arcs_x.at(-1)[1])];
  return { rails, posts, lands, screws, stop, keepers, arcs, band: mm(H.arc_band), zIn, yLt, window_drawn: { x: xs } };
}

/** The smallest polar angle (deg, 0-90) at which the point b (m) in from the section h's skin stands at height y or above. */
export function polarAbove(h, b, y) {
  const s = (y - h.y) / (h.hh - b);
  return s <= -1 ? 0 : s >= 1 ? 90 : (Math.asin(s) * 180) / Math.PI;
}

/** The polar angle (deg) a length s (m) along the skin spans at polar angle a (deg) of the section h. */
export function edgeDeg(h, a, s) {
  const t = (a * Math.PI) / 180;
  return ((s / Math.hypot(h.hw * Math.sin(t), h.hh * Math.cos(t))) * 180) / Math.PI;
}

/** The volume (m3) of a list of solids: each section's area over its x range. */
const volOf = (sols) => sols.reduce((a, s) => a + (Number.isFinite(s.area_m2) ? s.area_m2 : shapeArea(s.shape)) * (s.x[1] - s.x[0]), 0);

/** The hatch's fixed parts' solids: { rails: { R, L }, posts: { tag: [solids] } }. */
export function hatchFrameSolids(G) {
  return {
    rails: Object.fromEntries(G.rails.map((r) => [r.tag, [box3('rail', r.x, r.z, r.y)]])),
    posts: Object.fromEntries(G.posts.map((p) => [p.tag, [box3('post', p.x, p.z, p.y)]])),
  };
}

/** The cover's 6061 solids: its lands, the pack's aft stop with its lip, and the keepers (each a bar on the lands, two drops and a crossbar). */
export function hatchCoverSolids(G) {
  const out = G.lands.map((l) => box3(`land ${l.tag}`, l.x, l.z, l.y));
  const st = G.stop, zo = G.rails[0].z[1];
  out.push(box3('aft stop bar', st.x, [-zo, zo], [G.yLt - (G.lands[0].y[1] - G.lands[0].y[0]), G.yLt]));
  out.push(box3('aft stop', st.x, [-st.z, st.z], [st.y[0], st.y[1]]));
  out.push(box3('aft stop lip', st.lip.x, [-st.lip.z, st.lip.z], [st.y[0] + 0.0211, st.y[0] + 0.0221]));
  G.keepers.forEach((k, i) => {
    const yb = k.yTop - 0.0044;
    out.push(box3(`keeper ${i + 1} bar`, k.x, [-zo, zo], [k.yTop, G.yLt]));
    for (const s of [1, -1]) out.push(box3(`keeper ${i + 1} drop ${s > 0 ? 'R' : 'L'}`, k.x, [s * k.z[0], s * k.z[1]].sort((a, b) => a - b), [yb, k.yTop]));
    out.push(box3(`keeper ${i + 1} crossbar`, k.x, [-k.z[1], k.z[1]], [yb, yb + k.t]));
  });
  return out;
}

/** The cover's arcs (6061 sheet): at each station two legs from the lands up to the skin, and the band on the skin between them. */
export function hatchArcSolids(G, loft) {
  return G.arcs.flatMap((a, i) => [
    box3(`arc ${i + 1} leg R`, a.x, a.legs[0], a.leg_y),
    box3(`arc ${i + 1} leg L`, a.x, a.legs[1], a.leg_y),
    ...arcPieces(loft, a.xm, a.polar, G.band).map((p, k) => S(`arc ${i + 1} band ${k + 1}`, a.x, poly(p))),
  ]);
}

/** The fixed forward stop under the nose tray's tip: a 6061 leg bonded under the base and an upright ahead of the pack. */
export function forwardStopSolids(P, N, packX1) {
  const F = P.stop_fwd, yb = N.y - N.t / 2, x0 = N.x[1] - mm(F.leg_x), xu = packX1 + mm(F.gap), t = mm(F.t), z = mm(F.z);
  return [box3('leg', [x0, xu + t], [-z, z], [yb - t, yb]), box3('upright', [xu, xu + t], [-z, z], [yb, mm(F.top_y)])];
}

// ---- routes (leads, tubing, antenna) -----------------------------------------------------------------------------

/**
 * A round line of radius r along a polyline of [x, z, y] points (m) as solids: each segment one prism over its x range
 * (grown by r) whose section is the capsule between its ends' (z, y), so it holds the line; a segment longer than
 * `piece` along x is split. Each solid states the line it holds (solid.tube: its own length, for the stock and mass
 * checks). -> { solids, L }.
 */
export function routeSolids(name, pts, r, ri = 0, piece = 0.02) {
  const sol = [];
  let L = 0;
  for (let k = 0; k + 1 < pts.length; k++) {
    const [a, b] = [pts[k], pts[k + 1]], len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    // a diagonal segment is split finely enough that each piece's section stays near its own stretch of the line
    const dx = Math.abs(b[0] - a[0]), dzy = Math.hypot(b[1] - a[1], b[2] - a[2]);
    const n = Math.max(1, Math.ceil(dx / piece), dx > 1e-6 ? Math.ceil(dzy / 0.0015) : 1);
    for (let j = 0; j < n; j++) {
      const p = (t) => a.map((v, i) => v + t * (b[i] - v)), u = p(j / n), w = p((j + 1) / n);
      sol.push({ ...S(`${name} ${k + 1}.${j + 1}`, [Math.min(u[0], w[0]) - r, Math.max(u[0], w[0]) + r], cap([u[1], u[2]], [w[1], w[2]], r)), tube: { ro: r, ri, L: len / n, axis: 'route' } });
    }
    L += len;
  }
  return { solids: sol, L };
}

/** A polyline's length (m). */
export const pathLength = (pts) => pts.slice(1).reduce((a, p, k) => a + Math.hypot(...p.map((v, i) => v - pts[k][i])), 0);

// ---- the parts in the assembly (clearance.js afterPacked) --------------------------------------------------------

const boxOf = (pk, name) => pk.model.layout.parts.find((p) => p.name === name)?.box ?? null;

/**
 * Every build-needs solid in the assembly, after the packed parts (it needs their boxes): the hatch, the forward stop,
 * the connectors on the pack, the pressure tubing, the antenna and its tie, the belly tray, the cooling ducts and the
 * servo leads. ax: clearance.js's part context { d, catalog, af, add, join, assumptions, asm }; pk: packedParts' result.
 * -> the geometry it drew (for the record).
 */
export function needsParts(ax, pk) {
  const { d } = ax, P = needsParams(d.params);
  if (!d.params.layout?.hatch || !ax.af?.carriers) return null;
  const N = ax.af.carriers.nose, loft = d.body.loft, G = hatchGeometry(loft, P, N, d.params.layout.hatch);
  const out = { G, P, q0: ax.q0 };
  hatchParts(ax, G, loft);
  out.forwardStop = forwardStopPart(ax, P, N, boxOf(pk, 'battery'));
  out.plugs = plugParts(ax, P, boxOf(pk, 'battery'));
  out.tubing = tubingParts(ax, P, N, boxOf(pk, 'airspeed'));
  out.antenna = antennaParts(ax, P, boxOf(pk, 'receiver'));
  if (ax.af.carriers.belly) out.belly = bellyParts(ax, ax.af.carriers.belly);
  out.leads = leadParts(ax, P, pk);
  ax.assumptions.push(needsAssumption(P, G));
  return out;
}

function hatchParts({ add, join }, G, loft) {
  const fr = hatchFrameSolids(G);
  // the frame is one bonded piece (its rails on its posts): one item of its line
  for (const r of G.rails) add(`hatch rail ${r.tag}`, fr.rails[r.tag], { bom: 'hatch frame', item: 'hatch frame' });
  for (const p of G.posts) {
    const id = add(`hatch post ${p.tag}`, fr.posts[p.tag], { bom: 'hatch frame', item: 'hatch frame' });
    join(id, 'nose tray', 'bonded on the nose tray\'s flange');
    join(id, `hatch rail ${p.side}`, 'the rail bonded on its top');
  }
  const cov = add('hatch cover', hatchCoverSolids(G), { bom: 'hatch cover' });
  for (const r of G.rails) join(cov, `hatch rail ${r.tag}`, 'resting on the rail, screwed');
  const arcs = add('hatch cover arcs', hatchArcSolids(G, loft), { bom: 'hatch cover arcs' });
  join(arcs, cov, 'bonded on the cover\'s lands');
}

function forwardStopPart({ add, join }, P, N, pack) {
  if (!pack) return null;
  const id = add('pack stop forward', forwardStopSolids(P, N, pack.x1), { bom: 'pack stop forward' });
  join(id, 'nose tray', 'bonded under the nose tray\'s tip');
  return { id };
}

/** The pack's mate (a T-plug male on the bird's power lead) and the arming plug (T-plug female in the lead, male jumper), on the pack's top. */
function plugParts({ add, join }, P, pack) {
  if (!pack) return null;
  const [lx, lz, ly] = P.plugs.size_assumed.map(mm), y = [pack.y1, pack.y1 + ly];
  const x1 = mm(P.plugs.pack_x0), x2 = mm(P.plugs.arming_x0);
  const pp = add('pack plug', [box3('housing', [x1, x1 + lx], [-lz / 2, lz / 2], y)], { bom: P.plugs.male });
  const af = add('arming plug female', [box3('housing', [x2, x2 + lx / 2], [-lz / 2, lz / 2], y)], { bom: P.plugs.female });
  const aj = add('arming jumper', [box3('housing', [x2 + lx / 2, x2 + lx], [-lz / 2, lz / 2], y)], { bom: P.plugs.male });
  for (const id of [pp, af, aj]) join(id, 'packed battery', 'lying on the pack\'s top, on its lead');
  join(aj, af, 'plugged in the arming plug');
  join(pp, 'packed battery', 'mated with the pack\'s discharge plug');
  return { pack: pp, arming: [af, aj], x: [x1, x2 + lx] };
}

/**
 * The airspeed board's two pressure lines (Versilon SPT-50, 2.38 x 3.97 mm): from the probe's aft end aft above the
 * nose tray's right flange to beside the board, inboard over it and down onto its ports; tied with the antenna to the
 * flange through a slot at its root.
 */
function tubingParts({ d, catalog, af, add, join }, P, N, aspd) {
  const T = P.tubing, sp = byId(catalog, T.id).specs, ro = mm(sp.od_mm) / 2, ri = mm(sp.id_mm) / 2, pm = af.support?.pitot;
  if (!pm || !aspd) return null;
  const lines = T.y.map((yy, k) => {
    const y = mm(yy), z = mm(T.z), xe = mm(T.ports_x[k]), pz = mm(T.port_z), x0 = pm.x_m[0] - ro - 1e-4, xt = pm.x_m[0] + 0.0002;
    // the total line from the end face's centre aft; the static from a take-off on the probe's top just forward of its
    // end, straight up (the probe's end fitting and take-off are not drawn: audit C6)
    const pts = k ? [[xt, pm.c_m[0], pm.c_m[1] + pm.r_m + ro + 1e-4], [xt, pm.c_m[0], y], [xt - 0.0012, z, y], [x0 - 0.008, z, y], [mm(T.turn_x[k]), z, y], [xe, pz, y], [xe, pz, aspd.y1 + ro]]
      : [[x0, pm.c_m[0], pm.c_m[1]], [x0 - 0.0039, z - 0.0002, pm.c_m[1]], [x0 - 0.0054, z, y], [mm(T.turn_x[k]), z, y], [xe, pz, y], [xe, pz, aspd.y1 + ro]];
    const rt = routeSolids('line', pts, ro, ri);
    const id = add(`pressure tubing ${k ? 'static' : 'total'}`, rt.solids, { bom: 'pressure tubing' });
    join(id, 'pitot-static probe', `on the probe's ${k ? 'static' : 'total-pressure'} take-off`);
    join(id, 'packed airspeed', 'on the sensor\'s barbed port');
    return { id, L: rt.L, pts };
  });
  return { lines, L: lines.reduce((a, l) => a + l.L, 0), N };
}

/** The receiver's antenna on its U.FL socket: up from the board, out over the right flange, forward beside the tubing. */
function antennaParts({ add, join, af, catalog }, P, rx) {
  if (!rx) return null;
  const A = P.antenna, r = mm(A.d) / 2, y = mm(A.y), sg = Math.sign((rx.z0 + rx.z1) / 2) || 1, z = sg * mm(A.z), x0 = mm(A.x_start);
  const pts = [[x0, (rx.z0 + rx.z1) / 2, rx.y1 + r], [x0, (rx.z0 + rx.z1) / 2, y], [x0 + 0.004, z, y], [mm(A.x_end), z, y]];
  const rt = routeSolids('coax', pts, r);
  const id = add('receiver antenna', rt.solids, { bom: 'rx-tbs-crossfire-nano', item: 'packed receiver' });
  join(id, 'packed receiver', 'on its U.FL socket');
  const N = af.carriers.nose, zin = N.w / 2 - N.t / 2, yb = N.y - N.t / 2, top = N.y + N.t / 2;
  // the antenna's tie round its side's flange, through a slot at the flange's root
  const zs = sg > 0 ? [zin, Math.abs(z) + r] : [-(Math.abs(z) + r), -zin], fl = sg > 0 ? [top, yb] : [yb, top];
  const aid = add('receiver antenna tie', bandSlabs('tie', mm(A.tie_x), 0.0025, 0.001, zs, [yb, y + r], fl), { bom: 'tie-hellermanntyton-t18r' });
  join(aid, id, 'round the line, tied to the flange');
  join(aid, 'nose tray', 'round the nose tray\'s flange, through a slot');
  // the pressure lines' tie round the right flange and both lines
  const zt = mm(P.tubing.z) + mm(byId(catalog, P.tubing.id).specs.od_mm) / 2, yt = Math.max(...P.tubing.y.map((v) => mm(v) + 0.002));
  const tid = add('pressure tubing tie', bandSlabs('tie', mm(P.tubing.tie_x), 0.0025, 0.001, [zin, zt], [yb, yt], [top, yb]), { bom: 'tie-hellermanntyton-t18r' });
  for (const t of ['pressure tubing total', 'pressure tubing static']) join(tid, t, 'round the line, tied to the flange');
  join(tid, 'nose tray', 'round the nose tray\'s flange, through a slot');
  return { id, L: rt.L };
}

/** The belly tray under the nose tray (the FC and the servo regulator on it), its root tabs screwed to the riser's web. */
function bellyParts({ add, join }, B) {
  const id = add('belly tray', bellyBaseSolids(B), { bom: 'belly tray' });
  join(id, 'nose tray riser', 'its root tabs screwed to the riser\'s web');
  return { id };
}

// ---- servo leads ----------------------------------------------------------------------------------------------

/** The wing frame of a side from its drawn inner spar: { o, u, v } at a pose, the spar's own capsule ends known. */
function wingFrameOf(asm, tag, sg) {
  const sp = asm.parts.find((p) => p.id === `inner spar ${tag}`)?.solids.find((s) => s.name === 'tube');
  const hub = asm.parts.find((p) => p.id === `rocker ${tag}`)?.solids.find((s) => s.name === 'hub');
  if (!sp?.at || !hub) return null;
  const o = hub.shape.c;
  return (q) => { const c = sp.at(q), d = [c.b[0] - c.a[0], c.b[1] - c.a[1]], n = Math.hypot(...d), u = [d[0] / n, d[1] / n]; return { o, u, v: sg > 0 ? [-u[1], u[0]] : [u[1], -u[0]] }; };
}

/**
 * The servo leads (SPEC: a connector for every electrical joint the assembly order makes; the wings go on last). For
 * each drawn servo: its own lead (240 mm, Hitec, wire), the extension's route as drawn solids, and the reach. Twist
 * servos on a moving wing are routed along their spar and across the shoulder (leadRoutes); a servo whose drawn part
 * does not move needs no wing-root connector.
 */
function leadParts(ax, P, pk) {
  const rows = [];
  const fc = boxOf(pk, 'fc');
  if (!fc) return rows;
  for (const [tag, sg] of [['R', 1], ['L', -1]]) {
    const tail = boxOf(pk, `servo_tail_${tag}`);
    if (tail) rows.push(tailLead(ax, P, tag, sg, tail, fc));
    const sv = ax.asm.parts.find((p) => p.id === `twist servo ${tag}`);
    if (!sv) continue;
    const t = twistLead(ax, P, tag, sg, sv, fc);
    rows.push(t.draw ? drawTwistLead(ax, P, t) : t);
  }
  return rows;
}

/**
 * The harness's common path from the aft region to the FC on the belly tray, for one lane (0 the twist-servo lead, 1 the
 * tail lead) of one side: under the aft tray, under the gearbox and the forward tray, out and up outside the forward
 * tray and the riser, forward past the riser's flanges and in to the FC's side. -> named points [x, z, y] (m).
 */
export function harnessPath(af, fc, sg, lane, P = NEEDS) {
  const AT = af.carriers.aft, hy = mm(P.leads.housing_assumed[2]), Hn = P.harness;
  const yU = AT.y - AT.t / 2 - mm(Hn.y_drop), zU = sg * mm(Hn.z_lanes[lane]), zS = sg * mm(Hn.z_side), yF = AT.y - AT.t / 2 - mm(Hn.y_rise);
  const xUp = mm(Hn.x_up[lane]), yFc = mm(Hn.y_fc[lane]), xk = mm(P.leads.plug_x[lane]), zk = sg * mm(Hn.z_plug), yOver = mm(Hn.y_over);
  const bend = lane ? [[xUp - 0.006, zU, yF]] : Hn.bend0.map(([x, z]) => [mm(x), sg * mm(z), yF]);
  return { yU, zU, zS, under: [AT.x[1] - 0.04, zU, yU], rise: [[0.080, zU, yU], [0.090, zU, yF]], fwd: bend[0], bend: bend.slice(1), side: [xUp, zS, yF], up: [xUp, zS, yFc], past: [xk, zS, yFc], up2: [xk, zS, yOver], over: [xk, zk, yOver], toFc: [xk, zk, fc.y1 + hy + mm(P.leads.d_assumed) / 2], plugY: [fc.y1, fc.y1 + hy] };
}

function tailLead(ax, P, tag, sg, tail, fc) {
  const H = harnessPath(ax.af, fc, sg, 1, P), AT = ax.af.carriers.aft, r = mm(P.leads.d_assumed) / 2;
  const exit = [tail.x1 + r + 0.0005, (tail.z0 + tail.z1) / 2, tail.y0 + 0.009];
  const out = [exit[0] + 0.002, sg * (AT.w / 2 + 0.003), exit[2]];
  const down = [out[0], out[1], H.under[2]];
  const pts = [exit, out, down, [down[0] + 0.004, H.under[1], H.under[2]], ...H.rise, H.fwd, ...H.bend, H.side, H.up, H.past, H.up2, H.over, H.toFc];
  return drawLead(ax, P, { name: `tail lead ${tag}`, ext: P.leads.tail, own: mm(P.servoLead.length_mm), pts, plugY: H.plugY, servo: `packed servo_tail_${tag}`, joint: `tail servo ${tag} to the FC` });
}

/**
 * A twist servo's lead: its own lead along the spar inboard from the servo (wire), the wing-side extension on from its
 * plug along the spar aft of the socket, in through the wing root's slot and across the shoulder in the plane aft of
 * the rocker's socket (its flexing span drawn as the straight line between its two ends at each pose), to the
 * wing-root connector tied on a clip behind the aft shoulder bracket; the pod-side extension from there down the
 * pod's side and along the harness to the FC.
 */
function twistLead(ax, P, tag, sg, sv, fc) {
  const frame = wingFrameOf(ax.asm, tag, sg), case_ = sv.solids[0];
  const base = { joint: `twist servo ${tag} to the FC (wing-root connector)`, servo: `twist servo ${tag}` };
  if (!case_.at) return { ...base, name: `twist-servo lead ${tag}`, moving: false, note: 'the twist servo is drawn in the pod: no wing-root connector; its own lead to the FC', ok: null };
  if (!frame) return { ...base, name: `twist-servo lead ${tag}`, moving: true, ok: false, why: 'no drawn inner spar or rocker hub to route along' };
  const q0 = ax.q0, f0 = frame(q0), pts0 = case_.at(q0).pts ?? [], sOf = (p) => (p[0] - f0.o[0]) * f0.u[0] + (p[1] - f0.o[1]) * f0.u[1];
  const sIn = Math.min(...pts0.map(sOf)), own = mm(P.servoLead.length_mm), r = mm(P.leads.d_assumed) / 2;
  const sock = ax.asm.parts.find((p) => p.id === `rocker ${tag}`).solids.find((s) => s.name === 'spar socket');
  const xL = sock.x[0] - 0.0005 - mm(P.leads.x_gap) - r, sPlug = Math.max(0.03, sIn - own / (1 + P.leads.slack_frac)), sCross = 0.011;
  const brk = ax.asm.parts.find((p) => p.id === `shoulder bracket ${tag} aft`)?.solids.find((s) => s.name === 'sleeve');
  return { ...base, name: `twist-servo lead ${tag}`, moving: true, frame, xL, r, sIn, sPlug, sCross, sleeve: brk, ext: P.leads, fc, sg, tag, draw: true };
}

/** Draw one fixed lead: its route as solids, its two housings, its joins; -> the row for the record. */
function drawLead(ax, P, L) {
  const r = mm(P.leads.d_assumed) / 2, cat = byId(ax.catalog, L.ext), len = mm(cat.specs.length_mm);
  const routeL = pathLength(L.pts), need = Math.max(0, routeL * (1 + P.leads.slack_frac) - L.own);
  // the servo's own lead covers the route's first part (wire, not drawn); the extension the rest, from its socket on
  let acc = 0, k = 0;
  while (k + 1 < L.pts.length && acc + pathLength([L.pts[k], L.pts[k + 1]]) < L.own / (1 + P.leads.slack_frac)) { acc += pathLength([L.pts[k], L.pts[k + 1]]); k++; }
  const a = L.pts[k], b = L.pts[Math.min(k + 1, L.pts.length - 1)], seg = pathLength([a, b]) || 1, t = Math.min(1, (L.own / (1 + P.leads.slack_frac) - acc) / seg);
  const start = a.map((v, i) => v + t * (b[i] - v)), pts = [start, ...L.pts.slice(k + 1)];
  const rt = routeSolids('lead', pts, r);
  const [hx, hz, hy] = P.leads.housing_assumed.map(mm), end = pts.at(-1);
  const id = ax.add(L.name, [...rt.solids, box3('socket', [start[0] - hx / 2, start[0] + hx / 2], [start[1] - hy / 2, start[1] + hy / 2], [start[2] - hz / 2, start[2] + hz / 2]), box3('plug', [end[0] - hx / 2, end[0] + hx / 2], [end[1] - hz / 2, end[1] + hz / 2].sort((a, b) => a - b), L.plugY)], { bom: L.ext });
  ax.join(id, L.servo, 'the servo\'s own lead plugged in its socket');
  ax.join(id, 'packed fc', 'its plug on the FC\'s servo pins');
  return { name: L.name, id, joint: L.joint, ext: L.ext, ext_mm: len * 1000, own_mm: L.own * 1000, route_mm: r3(routeL), drawn_mm: r3(rt.L), need_mm: r3(need), ok: rt.L * (1 + P.leads.slack_frac) <= len + 1e-9 && need <= len + 1e-9, pts };
}

function needsAssumption(P, G) {
  return `build needs (round 17b, src/design/build-needs.js): a hatch over x ${r3(G.window_drawn.x[0])}..${r3(G.window_drawn.x[1])} mm (6061 rails on posts bonded to the nose tray's flanges, a cover of 6061 lands, stops and keepers with 6061 arcs carrying the window's film, ISO 4762 M3 10.9 into ISO 4032 nuts under the rails); the pack on a ${P.strap.id} (drawn ${P.strap.t_assumed} mm thick: unpublished), held fore and aft by the cover's aft stop and a fixed forward stop, up by the cover's keepers; AMASS T-plugs (housings drawn ${P.plugs.size_assumed.join(' x ')} mm: unpublished); Versilon SPT-50 pressure lines; the antenna drawn as ${P.antenna.d} mm coax (its length and mass unpublished); goBILDA TJC8 leads drawn ${P.leads.d_assumed} mm round with ${P.leads.housing_assumed.join(' x ')} mm housings (unpublished)`;
}


/** A rectangle in a frame's (u, v) plane: s along u, n along v, from o. */
const rectUV = (o, u, v, [s0, s1], [n0, n1]) => { const Pt = (s, n) => [o[0] + s * u[0] + n * v[0], o[1] + s * u[1] + n * v[1]]; return poly([Pt(s0, n0), Pt(s1, n0), Pt(s1, n1), Pt(s0, n1)]); };
const along = (f, s) => [f.o[0] + s * f.u[0], f.o[1] + s * f.u[1]];

/** The angle range (rad) of a wing's spar over the drive's poses, and its middle. */
function sparAngles(frame, poses, q0) {
  const u0 = frame(q0).u, base = Math.atan2(u0[1], u0[0]);
  const a = poses.map((q) => { const u = frame(q).u; return base + Math.atan2(u0[0] * u[1] - u0[1] * u[0], u0[0] * u[0] + u0[1] * u[1]); });
  return { min: Math.min(...a), max: Math.max(...a), mid: (Math.min(...a) + Math.max(...a)) / 2 };
}

/**
 * Draw a twist servo's two extensions, the wing-root connector's clip and tie (t: twistLead's geometry). The wing-side
 * extension (moving with the wing): along the spar aft of the socket from its socket (the servo's own plug in it) in
 * to sCross from the shoulder axis, then its flexing span to the fixed point at the stroke's middle angle, then fixed:
 * aft and outboard over the aft shoulder bracket to its plug on the clip. The pod-side extension: its socket on the
 * clip, down the pod's side and along the harness to the FC. -> the row for the record.
 */
function drawTwistLead(ax, P, t) {
  const { frame, xL, r, sCross, sPlug, sleeve, sg, tag, fc } = t, [hx, hz, hy] = P.leads.housing_assumed.map(mm);
  const wingCat = byId(ax.catalog, P.leads.wing).specs, podCat = byId(ax.catalog, P.leads.pod).specs;
  const ang = sparAngles(frame, ax.poses, ax.q0), f0 = frame(ax.q0), o = f0.o;
  const Pf = [o[0] + sCross * Math.cos(ang.mid), o[1] + sCross * Math.sin(ang.mid)];
  const housing = ax.asm.parts.find((p) => p.id === `shoulder bracket ${tag} aft`)?.solids.find((s) => s.name === 'housing');
  const hb = bboxOf(housing.shape), hzMax = Math.max(...hb.z.map((v) => sg * v)), hyTop = hb.y[1];
  const arm = ax.asm.parts.find((p) => p.id === `shoulder bracket ${tag} aft`)?.solids.find((s) => s.name === 'arm');
  const armTop = bboxOf(arm.shape).y[1];
  // the clip: a 6061 strip bonded on the housing's aft face, its top under the connector's housings
  const zc = o[0] + sg * 0.008, clipY = [armTop + 0.0005, armTop + 0.002], xc1 = sleeve.x[0] - 0.0025;
  const clipX = [xc1 - 2 * hx, housing.x[0]], hzC = [zc - hz / 2, zc + hz / 2].sort((a, b) => a - b), yL = clipY[1] + r;
  const clip = ax.add(`twist-servo lead ${tag} clip`, [box3('strip', clipX, hzC, clipY)], { bom: 'twist-servo lead clips' });
  ax.join(clip, `shoulder bracket ${tag} aft`, 'bonded on the bracket\'s aft face');
  const yh = [clipY[1], clipY[1] + hy], yc = (yh[0] + yh[1]) / 2;
  // the wing-side extension
  const zq = o[0] + sg * 0.012, yq = Math.max(hyTop - 0.010, clipY[1] + hy / 2 + r + 0.0005);
  const fixedPts = [[xL, Pf[0], Pf[1]], [xL - 0.003, zq, yq], [sleeve.x[0] + 0.001, zq, yq], [sleeve.x[0] - 0.001, zc, yL], [xc1, zc, yL]];
  const fixed = routeSolids('pod run', fixedPts, r);
  const spanL = sPlug - sCross;
  const flexMax = Math.max(...ax.poses.map((q) => { const p = along(frame(q), sCross); return Math.hypot(p[0] - Pf[0], p[1] - Pf[1]); }));
  const sRoot = (ax.d.params.wingStructure?.rib_root_m ?? 0.052) - mm(ax.d.params.wingStructure?.rib_t_mm ?? 2) / 2;
  const wingSol = [
    { ...M('spar run root', [xL - r, xL + r], (q) => { const f = frame(q); return cap(along(f, sCross), along(f, sRoot), r); }), tube: { ro: r, ri: 0, L: sRoot - sCross, axis: 'span' } },
    { ...M('spar run', [xL - r, xL + r], (q) => { const f = frame(q); return cap(along(f, sRoot), along(f, sPlug), r); }), tube: { ro: r, ri: 0, L: sPlug - sRoot, axis: 'span' } },
    M('socket', [xL - hy / 2, xL + hy / 2], (q) => { const f = frame(q); return rectUV(f.o, f.u, f.v, [sPlug, sPlug + hx], [-hz / 2, hz / 2]); }),
    { ...M('shoulder span', [xL - r, xL + r], (q) => cap(along(frame(q), sCross), Pf, r)), tube: { ro: r, ri: 0, L: flexMax, axis: 'flex' } },
    ...fixed.solids,
    box3('plug', [xc1 - hx, xc1], hzC, yh),
  ];
  const wid = ax.add(`twist-servo lead ${tag} wing`, wingSol, { bom: P.leads.wing });
  ax.join(wid, `twist servo ${tag}`, 'the servo\'s own lead plugged in its socket');
  for (const rib of ax.asm.parts.filter((p) => new RegExp(`^rib ${tag} \\d+$`).test(p.id))) {
    // a rib drawn in pieces (twist-drive.js) is crossed if any of its pieces is at the lead's x and span
    const crossed = rib.solids.some((sr) => { const sh = sr.at ? sr.at(ax.q0) : sr.shape, pts = sh.pts ?? [], ss = pts.map((p) => (p[0] - o[0]) * f0.u[0] + (p[1] - o[1]) * f0.u[1]); return pts.length > 0 && Math.max(...ss) >= sRoot - r && Math.min(...ss) <= sPlug + hx && sr.x[0] <= xL + r && sr.x[1] >= xL - r; });
    if (crossed) for (const sn of ['spar run root', 'spar run', 'socket']) ax.join(`${wid}/${sn}`, rib.id, RIB_HOLE);
  }
  // the pod-side extension: its socket on the clip, mated to the wing side's plug, then down the side and the harness
  const H = harnessPath(ax.af, fc, sg, 0, P), AT = ax.af.carriers.aft, xs = xc1 - 2 * hx;
  const zDown = sg * (AT.w / 2 + AT.t / 2 + 0.0005 + r + 0.0005), yDown = AT.y - AT.t / 2 - 0.0005 - r - 0.0005;
  const podPts = [[xs, zc, yL], [xs - 0.002, zc + sg * 0.003, yL], [xs - 0.002, sg * 0.041, -0.055], [xs - 0.002, zDown, yDown], [xs + 0.002, H.zU, H.yU], ...H.rise, H.fwd, ...H.bend, H.side, H.up, H.past, H.up2, H.over, H.toFc];
  const pod = routeSolids('lead', podPts, r);
  const end = H.toFc;
  const pid = ax.add(`twist-servo lead ${tag} pod`, [box3('socket', [xs, xs + hx], hzC, yh), ...pod.solids, box3('plug', [end[0] - hx / 2, end[0] + hx / 2], [end[1] - hz / 2, end[1] + hz / 2].sort((a, b) => a - b), H.plugY)], { bom: P.leads.pod });
  ax.join(pid, wid, 'plugged in the wing-root connector');
  ax.join(pid, 'packed fc', 'its plug on the FC\'s servo pins');
  for (const id of [wid, pid]) ax.join(id, clip, 'its housing on the clip, tied');
  const tid = ax.add(`twist-servo lead ${tag} tie`, bandSlabs('tie', xc1 - hx, 0.0025, 0.001, hzC, [clipY[0], yh[1]], [clipY[0], clipY[0]]), { bom: 'tie-hellermanntyton-t18r' });
  for (const id of [wid, pid, clip]) ax.join(tid, id, 'round the connector and its clip');
  const wingL = spanL + flexMax + fixed.L, podL = pod.L;
  const slack = P.leads.slack_frac;
  return {
    name: `twist-servo lead ${tag}`, joint: t.joint, moving: true, ids: [wid, pid, clip, tid],
    servo_own_mm: P.servoLead.length_mm, servo_lead_route_mm: r3(t.sIn - sPlug),
    wing: { ext: P.leads.wing, ext_mm: wingCat.length_mm, drawn_mm: r3(wingL), span_on_spar_mm: r3(spanL), root_span_mm: r3(sRoot - sCross), flex_max_mm: r3(flexMax), ok: wingL * (1 + slack) <= mm(wingCat.length_mm) + 1e-9 },
    pod: { ext: P.leads.pod, ext_mm: podCat.length_mm, drawn_mm: r3(podL), ok: podL * (1 + slack) <= mm(podCat.length_mm) + 1e-9 },
    stroke_deg: [ang.min, ang.max].map((a) => +((a * 180) / Math.PI).toFixed(1)),
    ok: wingL * (1 + slack) <= mm(wingCat.length_mm) + 1e-9 && podL * (1 + slack) <= mm(podCat.length_mm) + 1e-9 && t.sIn - sPlug <= mm(P.servoLead.length_mm) / (1 + slack) + 1e-9,
    clip: { x: clipX.map(r3), bond_area_mm2: +(Math.max(0, Math.min(hzC[1], hb.z[1]) - Math.max(hzC[0], hb.z[0])) * (clipY[1] - clipY[0]) * 1e6).toFixed(2) },
  };
}

// ---- the BOM (build.js) -------------------------------------------------------------------------------------------

/**
 * The build needs' BOM: structure lines sized from the drawn geometry (the hatch's frame, cover and arcs, the forward
 * stop, the belly tray, the clips, the ducts, the pressure tubing) and bought parts (connectors, strap, extensions).
 * p: the design's params; loft: its pod loft; A: its airframe parameters (merged). -> { structure, parts, lump, wing }.
 */
export function needsBom(p, catalog, loft, A) {
  const P = needsParams(p), N = noseTrayOf(A), G = hatchGeometry(loft, P, N, p.layout?.hatch), al = byId(catalog, P.material), bal = byId(catalog, P.hatch.arc_material);
  const rho = (m) => (m.specs.density_g_cm3 ?? m.specs.density_kg_m3 / 1000) * 1e6; // g/m3
  const fr = hatchFrameSolids(G), frameV = volOf([...Object.values(fr.rails).flat(), ...Object.values(fr.posts).flat()]);
  const coverV = volOf(hatchCoverSolids(G)), arcV = volOf(hatchArcSolids(G, loft));
  const stopV = volOf(forwardStopSolids(P, N, mm(P.stop_fwd.pack_x1 ?? 231.2)));
  const B = bellyGeometry(P, N, A), bellyV = volOf(bellyBaseSolids(B));
  const tub = byId(catalog, P.tubing.id), tubL = mm(P.tubing.length_mm ?? 0);
  const clipV = mm(P.leads.clip_mm3 ?? 0) / 1e6;
  const structure = [
    { name: 'hatch frame', material: al, mass_g: frameV * rho(al), basis: `two ${P.hatch.rail_t} mm 6061 rails ${P.hatch.rail_x.join('..')} mm on four ${P.hatch.post_t} mm posts bonded on the nose tray's flanges (${(frameV * 1e9).toFixed(0)} mm3, the drawn solids); ${al.specs.density_g_cm3} g/cm3` },
    { name: 'hatch cover', material: al, mass_g: coverV * rho(al), basis: `two ${P.hatch.land_t} mm 6061 lands, the pack's aft stop with its lip and two keepers (${(coverV * 1e9).toFixed(0)} mm3, the drawn solids); ${al.specs.density_g_cm3} g/cm3` },
    { name: 'hatch cover arcs', material: bal, mass_g: arcV * rho(bal), basis: `${G.arcs.length} 6061 arcs on the skin with their legs to the lands (${(arcV * 1e9).toFixed(0)} mm3); ${bal.specs.density_g_cm3} g/cm3` },
    { name: 'pack stop forward', material: al, mass_g: stopV * rho(al), basis: `a ${P.stop_fwd.t} mm 6061 leg bonded under the nose tray's tip and its upright ahead of the pack (${(stopV * 1e9).toFixed(0)} mm3)` },
    { name: 'belly tray', material: al, mass_g: bellyV * rho(al), basis: `channel ${P.belly.w} x ${P.belly.h} x ${P.belly.t} mm 6061 x ${(B.x[1] - B.x[0]) * 1000} mm under the nose tray with two root tabs (${(bellyV * 1e9).toFixed(0)} mm3)` },
    ...(tubL > 0 ? [{ name: 'pressure tubing, 2 off', material: tub, mass_g: tub.mass_g * tubL, basis: `${(tubL * 500).toFixed(2)} mm of ${tub.id} (${tub.mass_g} g/m, computed from the datasheet's section and specific gravity) each on average, ${(tubL * 1000).toFixed(1)} mm in all: the probe's total and static lines to the ASPD-4525`, count: 2 }] : []),
    ...(clipV > 0 ? [{ name: 'twist-servo lead clips, 2 off', material: al, mass_g: clipV * rho(al), basis: `two 6061 strips behind the aft shoulder brackets under the wing-root connectors (${(clipV * 1e9).toFixed(0)} mm3)`, count: 2 }] : []),
  ];
  const parts = [
    { id: P.plugs.male, count: 2, role: 'T-plug male: the pack\'s mate and the arming jumper' },
    { id: P.plugs.female, count: 1, role: 'T-plug female: the arming plug in the power lead' },
    { id: P.strap.id, count: 1, role: 'battery strap (releasable)' },
    { id: P.leads.wing, count: 2, role: 'twist servo extension, wing side' },
    { id: P.leads.pod, count: 2, role: 'twist servo extension, pod side' },
    { id: P.leads.tail, count: 2, role: 'tail servo extension' },
  ];
  const lump = { name: 'build needs', lines: [...structure.filter((s) => !/^twist-servo lead clips/.test(s.name)).map((s) => s.name), P.plugs.male, P.plugs.female, P.strap.id, P.leads.pod, P.leads.tail, ...(clipV > 0 ? [structure.at(-1).name] : [])] };
  return { structure, parts, lump, wing: { id: P.leads.wing, count: 2 }, G, P };
}

/** The belly tray's geometry (m): a channel under the nose tray from the riser's web forward, its two root tabs either side of the readhead's lower nut. */
export function bellyGeometry(P, N, A) {
  const B = P.belly, t = mm(B.t), y = mm(B.y_top) - t / 2, x0 = mm(A.noseTray.riser_x_mm) + mm(A.noseTray.riser.t_mm);
  return { x: [x0, mm(B.x[1])], y, t, w: mm(B.w), h: mm(B.h), tab: mm(B.tab), tabs: [['R', [0.0033, 0.0175]], ['L', [-0.0175, -0.0033]]], material: P.material, screws_z: [0.0145, -0.0145], screw_y: -0.075 };
}
function bellyBaseSolids(B) {
  return [box3('base', B.x, [-B.w / 2, B.w / 2], [B.y - B.t / 2, B.y + B.t / 2]), box3('flange right', B.x, [B.w / 2 - B.t / 2, B.w / 2 + B.t / 2], [B.y, B.y + B.h]), box3('flange left', B.x, [-B.w / 2 - B.t / 2, -B.w / 2 + B.t / 2], [B.y, B.y + B.h]), ...B.tabs.map(([nm, zz]) => box3(`root tab ${nm}`, [B.x[0], B.x[0] + B.t], zz, [B.y - B.tab, B.y - B.t / 2]))];
}

/**
 * The harness's ties: one under the aft tray and one under the forward tray, each round every lead under it and the
 * tray's base (its top over the base, its sides through slots in the base, its bottom under the leads).
 */
export function harnessTies(ax, leadRows, P) {
  const r = mm(P.leads.d_assumed) / 2, ids = leadRows.flatMap((l) => (l.ids ? [l.ids[1]] : l.id ? [l.id] : []));
  if (!ids.length) return [];
  const out = [];
  for (const [nm, c, x] of [['aft', ax.af.carriers.aft, ax.af.carriers.aft.x[1] - 0.025], ['forward', ax.af.carriers.forward, ax.af.carriers.forward.x[0] + 0.004]]) {
    const H = harnessPath(ax.af, { x0: 0, y0: 0, y1: 0, z0: 0, z1: 0 }, 1, 1, P), zMax = Math.abs(H.zU) + r, yb = c.y - c.t / 2, yt = c.y + c.t / 2, b = 0.0025, tt = 0.001, ylo = H.yU - r;
    const xs = [x - b / 2, x + b / 2];
    const sol = [box3('tie top', xs, [-zMax - tt, zMax + tt], [yt, yt + tt]), box3('tie side -z', xs, [-zMax - tt, -zMax], [ylo, yb]), box3('tie side +z', xs, [zMax, zMax + tt], [ylo, yb]), box3('tie bottom', xs, [-zMax - tt, zMax + tt], [ylo - tt, ylo])];
    const id = ax.add(`harness tie ${nm}`, sol, { bom: 'tie-hellermanntyton-t18r' });
    for (const l of ids) ax.join(id, l, 'round the harness under the tray');
    ax.join(id, `${nm} tray`, 'round the tray\'s base, through its slots');
    out.push(id);
  }
  return out;
}

// ---- M7 members (airframe.js) --------------------------------------------------------------------------------

const sect = (name, radial) => ({ name, radial, axial: [] });
const inertial = (m, r) => { const R = Math.hypot(...r); return [{ ch: 'sf_origin', k: m }, { ch: 'wdot', k: m * R }, { ch: 'w2', k: m * R }]; };
const scaleT = (terms, f) => terms.map((t) => ({ ch: t.ch, k: +(t.k * f).toFixed(8) }));

/**
 * The build needs' members on the flight model's load channels (airframe.js context ax, after its carriers): the
 * hatch's screws and nuts' seats, rails, posts and their bonds; the pack's cage (the cover's aft stop and keepers, the
 * fixed forward stop and its bond, the nose tray's flanges laterally); the connectors' published mating pull; the
 * twist-servo leads' ties, clips and rib seats; the harness and tubing ties. Each bolt also enters the fixing layout
 * (ax.needsBolts), so the BOM counts it and the assembly check draws it. -> nothing (pushes to ax.members).
 */
export function needsMembers(ax) {
  const { d, catalog, members, mat, counts, tieS, glueAl, lump } = ax;
  if (!d.params.layout?.hatch || !ax.carriers?.geometry || !d.body?.loft) return;
  const P = needsParams(d.params), N = ax.carriers.geometry.nose, G = hatchGeometry(d.body.loft, P, N, d.params.layout.hatch), al = mat(P.material);
  const bat = lump('battery'), cov = lump('build needs'), pk = bat ? inertial(bat.m, bat.r) : [], cv = cov ? inertial(cov.m * 0.3, cov.r) : [];
  const c = { G, P, N, al, pk, cv, bat };
  hatchMembers(ax, c);
  cageMembers(ax, c);
  connectorMembers(ax, c);
  leadMembers(ax, c);
  needsBolts(ax, G, P, byId(catalog, P.hatch.screws[0]).specs);
}

/** The hatch's fixing and frame: its screws and their seats, rails, posts and their bonds. */
function hatchMembers(ax, { G, P, N, al, pk, cv }) {
  const { d, catalog, members, mat, counts, tieS, glueAl, lump } = ax;
  const H = P.hatch, n = G.screws.length, screw = byId(catalog, H.screws[0]).specs;
  // round 18: the cover's own screws (M2 8.8, coverScrew), rated on their own ISO 898-1 figures
  const cs0 = coverScrew(catalog, H);
  const bolt = (name, V, F, note) => members.push({ name, kind: 'fasteners', check: 'bolts', fastener: { id: cs0.id, tension_N: cs0.tension_N, shear_N: cs0.shear_N }, dims: { n }, sections: [sect('V', V), sect('F', F)], uses: { V: 'V', F: 'F' }, note });
  bolt('hatch screws', [...pk, ...cv], [...pk, ...cv], `${n} x ${cs0.name} with ISO 4032 nuts through the cover's lands and the rails: the pack's inertia every way (the cover's aft stop and keepers take it fore-aft and up) and the cover's own, in shear and as tension at full value (the window's suction, about 0.7 N at 15 m/s, is under 1% of it and not added)`);
  // round 18: the screw's edge distance to every edge of its rail (its outer and inner edges, and its ends: the load is
  // the pack's inertia every way, so the smallest one is taken as both e1 and e2)
  const { eMin, e_over_d0 } = hatchScrewEdge(H, (N.w / 2 + N.t / 2) * 1000, cs0), d0 = cs0.hole_mm;
  members.push({ name: 'hatch screws in the rails', kind: 'seat', check: 'seat', material: al, dims: { d_mm: cs0.d_mm, t_mm: H.rail_t, e1_over_d0: e_over_d0, e2_over_d0: e_over_d0 }, sections: [sect('F', scaleT([...pk, ...cv], 1 / n))], uses: { F: 'F' }, note: `each screw bearing on its 6061 rail, its smallest edge distance ${eMin.toFixed(2)} mm (of the rail's outer and inner edges and its ends) over the ISO 273 medium hole ${d0} mm` });
  const railArm = Math.max(...G.screws.map((s) => Math.min(...G.posts.filter((p) => p.side === s.side).map((p) => Math.min(Math.abs(s.x - p.x[0]), Math.abs(s.x - p.x[1])))))), bw = H.rail_z_out - (N.w / 2 + N.t / 2) * 1000;
  members.push({ name: 'hatch rails', kind: 'plate', check: 'plate', material: al, dims: { b_mm: +bw.toFixed(2), t_mm: H.rail_t }, sections: [sect('Mo', scaleT([...pk, ...cv], railArm / n))], uses: { Mo: 'Mo' }, note: `each screw's share bending its rail out of plane over ${(railArm * 1000).toFixed(1)} mm to the nearest post` });
  const postH = G.posts[0].y[1] - G.posts[0].y[0], nP = G.posts.length;
  members.push({ name: 'hatch posts', kind: 'plate', check: 'plate', material: al, dims: { b_mm: +((G.posts[0].x[1] - G.posts[0].x[0]) * 1000).toFixed(2), t_mm: H.post_t }, sections: [sect('Mo', scaleT([...pk, ...cv], postH / nP)), sect('F', scaleT([...pk, ...cv], 1 / nP))], uses: { Mo: 'Mo', F: 'F' }, note: `each of ${nP} posts: its share of the load laterally at the rail, ${(postH * 1000).toFixed(1)} mm above the flange, and axially` });
  const fl = mm(5.6) * 1000;
  members.push({ name: 'hatch post bonds', kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: +((G.posts[0].x[1] - G.posts[0].x[0]) * 1000).toFixed(2), L_mm: +((N.h + N.t) * 1000).toFixed(2) }, sections: [sect('F', scaleT([...pk, ...cv], 1 / nP))], uses: { F: 'F' }, note: `Araldite 2011 aluminium to aluminium: each post on the flange's outer face over the flange's height (${fl.toFixed(1)} mm drawn)` });
  members.push({ name: 'hatch rail bonds', kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: +((G.posts[0].x[1] - G.posts[0].x[0]) * 1000).toFixed(2), L_mm: H.post_t }, sections: [sect('F', scaleT([...pk, ...cv], 1 / nP))], uses: { F: 'F' }, note: 'Araldite 2011 aluminium to aluminium: each rail on its post\'s top edge (the post\'s thickness by its width)' });
  const st = G.stop, stopH = (st.y[1] - st.y[0]) * 1000;
  members.push({ name: 'hatch aft stop', kind: 'plate', check: 'plate', material: al, dims: { b_mm: +(2 * st.z * 1000).toFixed(2), t_mm: H.stop_aft.t }, sections: [sect('Mo', scaleT(pk, stopH / 2000))], uses: { Mo: 'Mo' }, note: `the pack's inertia (taken every way) on the stop at half its ${stopH.toFixed(1)} mm drop from the lands` });
  const kp = G.keepers[0], span = 2 * kp.z[1];
  members.push({ name: 'hatch keepers', kind: 'plate', check: 'plate', material: al, dims: { b_mm: +((kp.x[1] - kp.x[0]) * 1000).toFixed(2), t_mm: H.keeper_t }, sections: [sect('Mo', scaleT(pk, (0.6 * span) / 4))], uses: { Mo: 'Mo' }, note: `the crossbar over the pack's front: 0.6 of the pack's inertia (its share between the aft stop's lip and this keeper, by the pack's centre) at mid-span, simply supported over ${(span * 1000).toFixed(1)} mm (FL/4)` });
}

/** The pack's cage: the fixed forward stop and its bond, the nose tray's flanges laterally. */
function cageMembers(ax, { P, N, al, pk }) {
  const { d, catalog, members, mat, counts, tieS, glueAl, lump } = ax;
  const F = P.stop_fwd, upH = mm(F.top_y) - (N.y - N.t / 2);
  members.push({ name: 'pack stop forward', kind: 'plate', check: 'plate', material: al, dims: { b_mm: 2 * F.z, t_mm: F.t }, sections: [sect('Mo', scaleT(pk, upH / 2))], uses: { Mo: 'Mo' }, note: `the pack's inertia on the upright at half its ${(upH * 1000).toFixed(1)} mm height` });
  members.push({ name: 'pack stop forward bond', kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: 2 * F.z, L_mm: F.leg_x }, sections: [sect('F', pk)], uses: { F: 'F' }, note: 'Araldite 2011 aluminium to aluminium: the leg under the nose tray\'s tip' });
  members.push({ name: 'nose tray flanges: battery lateral', kind: 'plate', check: 'plate', material: mat(N.material), dims: { b_mm: 74.7, t_mm: N.t * 1000 }, sections: [sect('Mo', scaleT(pk, (N.h + N.t) / 2))], uses: { Mo: 'Mo' }, note: 'the pack\'s inertia (every way) on one flange at its mid-height, the flange bending about its root along the pack\'s length' });
}

/** The T-plugs' published mating pull against the mated pair's own inertia. */
function connectorMembers(ax, { P, bat }) {
  const { d, catalog, members, mat, counts, tieS, glueAl, lump } = ax;
  // the connectors' published mating pull (AMASS: "Pulling Force/500: 25N"), against each plug's own inertia and its lead's pull
  const male = byId(catalog, P.plugs.male), pull = +String(male.specs.pull_force_text ?? '').match(/([\d.]+)\s*N/)?.[1];
  for (const nm of ['pack plug mated', 'arming jumper mated']) members.push({ name: nm, kind: 'connector', check: 'rated', dims: { rating_N: pull, rating_basis: `AMASS T-plug "${male.specs.pull_force_text}" (flyingtech.co.uk), read as the mated pair's holding force` }, sections: [sect('F', inertial(0.004, bat?.r ?? [0.19, -0.03, 0]))], uses: { F: 'F' }, note: 'the mated pair\'s own inertia (2 + 2 g)' });
}

/** The leads' and lines' ties, the wing-root clips' bonds and the wing leads' seats in the ribs. */
function leadMembers(ax, { P }) {
  const { d, catalog, members, mat, counts, tieS, glueAl, lump } = ax;
  // leads, tubing and their ties
  const tie = (name, m, r, nT = 1, note = '') => { members.push({ name, kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: nT }, sections: [sect('F', inertial(m, r))], uses: { F: 'F' }, note }); counts.tieS += nT; };
  const ext = (id) => byId(catalog, id).mass_g / 1000;
  for (const [t, s] of [['R', 1], ['L', -1]]) {
    tie(`twist-servo lead ${t} tie`, ext(P.leads.wing) + ext(P.leads.pod), [-0.047, 0.018, s * 0.035], 1, 'the wing-root connector on its clip: both extensions\' whole masses (conservative) at the connector');
    members.push({ name: `twist-servo lead ${t} clip bond`, kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: 3.8, L_mm: 1.5 }, sections: [sect('F', inertial(ext(P.leads.wing) + ext(P.leads.pod), [-0.047, 0.018, s * 0.035]))], uses: { F: 'F' }, note: 'Araldite 2011 aluminium to aluminium: the clip\'s end on the aft shoulder bracket\'s housing (its overlap with the housing\'s aft face)' });
    const bal = byId(catalog, 'balsa-wood-database').specs, rib = d.params.wingStructure?.rib_t_mm ?? 2;
    members.push({ name: `twist-servo lead ${t} in the ribs`, kind: 'seat', check: 'rated', dims: { rating_N: 0.5 * bal.crushing_strength_mpa * P.leads.d_assumed * rib, rating_basis: `balsa crushing ${bal.crushing_strength_mpa} MPa (species average, Wood Database) at half on the lead's ${P.leads.d_assumed} mm diameter over a ${rib} mm rib` }, sections: [sect('F', inertial(ext(P.leads.wing), [0, 0.1, s * 0.15]))], uses: { F: 'F' }, note: 'the wing-side extension\'s weight and inertia on one rib\'s hole (all of it, conservative)' });
  }
  tie('harness ties', 2 * ext(P.leads.pod) + 2 * ext(P.leads.tail), [0.05, -0.086, 0], 2, 'the four pod-side extensions under the trays: their whole masses on the two ties together');
  const tub = byId(catalog, P.tubing.id).mass_g * mm(P.tubing.length_mm ?? 0) / 1000;
  tie('pressure tubing tie', tub, [0.17, -0.035, 0.026], 1, 'the two pressure lines on the right flange');
  tie('receiver antenna tie', 0.0005, [0.17, -0.035, -0.032], 1, 'the antenna (its mass within the receiver\'s 0.5 g, taken as the whole 0.5 g) on the left flange');
}

/**
 * Round 18: a hatch screw's smallest edge distance in its rail (mm; the rail from zIn_mm to rail_z_out across, rail_x
 * along; the screws at screw_z, screws_x), and over the cover screw's hole d0 (cs: coverScrew).
 */
export function hatchScrewEdge(H, zIn_mm, cs) {
  const eMin = Math.min(H.rail_z_out - H.screw_z, H.screw_z - zIn_mm, ...H.screws_x.flatMap((x) => [x - H.rail_x[0], H.rail_x[1] - x]));
  return { eMin, e_over_d0: +(eMin / cs.hole_mm).toFixed(3) };
}

/**
 * Round 18: the hatch cover's screw (P.hatch.cover_screws, else the belly tray's P.hatch.screws): its id, diameter, its
 * ISO 273 medium clearance hole (P.hatch.cover_hole_mm, 3.4 mm for M3) and its ISO 898-1 ratings from the catalogue.
 */
export function coverScrew(catalog, H) {
  const id = (H.cover_screws ?? H.screws)[0], sp = byId(catalog, id).specs, dv = sp.design_values ?? {};
  const d_mm = +(/M(\d+(?:\.\d+)?)/.exec(sp.thread ?? id.toUpperCase())?.[1] ?? 3);
  return { id, name: byId(catalog, id).name, d_mm, hole_mm: H.cover_hole_mm ?? 3.4, tension_N: dv.tension_N, shear_N: dv.shear_N };
}

/** The hatch's screws and the belly tray's screws as fixing-layout entries (fixings.js format), counted here. */
function needsBolts(ax, G, P, screwSpec) {
  const H = P.hatch, nutG = byId(ax.catalog, H.nut);
  const bolts = [];
  const nutH = (id) => +String(byId(ax.catalog, id).specs.mass_basis ?? '').match(/x ([\d.]+) mm high/)?.[1] / 1000 || 0.0024;
  const pick = (stack) => pickScrew(ax.catalog, H.screws, stack, nutH(H.nut));
  // round 18: the cover's own screws and nuts (M2: the rails' edge distance)
  const coverNut = H.cover_nut ?? H.nut, pickCover = (stack) => pickScrew(ax.catalog, H.cover_screws ?? H.screws, stack, nutH(coverNut));
  for (const s of G.screws) {
    const stack = mm(H.land_t) + mm(H.rail_t), sc = pickCover(stack);
    bolts.push({ joint: 'hatch', screw: sc.id, nut: coverNut, axis: 'y', c: [s.x, s.z], face: G.yLt, dir: -1, stack_m: stack, nut_face: G.yLt - stack, through: ['hatch cover', `hatch rail ${s.side}`] });
  }
  const B = ax.carriers.geometry.belly;
  if (B) {
    const R = ax.carriers.geometry.riser, stack = B.t + R.t, sc = pick(stack);
    for (const z of B.screws_z) bolts.push({ joint: 'belly tray', screw: sc.id, nut: H.nut, axis: 'x', c: [z, B.screw_y], face: B.x[0] + B.t, dir: -1, stack_m: stack, nut_face: R.x[0], through: ['belly tray', 'nose tray riser'] });
  }
  ax.counts.screws += G.screws.length; ax.counts.nuts += G.screws.length;
  ax.needsBolts = bolts;
}

// ---- interface rules (interfaces.js format) -----------------------------------------------------------------------

/** The build needs' interface rules: each join reason they make, with the members that rate its load and retention. */
export function needsInterfaceRules({ exact, constSpec, tieSpec, BOLT_RULE, SEAT_RULE, BOND_RULE, TIE_RULE, BEAM_RULE }) {
  const R = (why, names, rule, by, part = null, axial = names, kind = 'bonded') => ({ why: exact(why), part, kind, load: constSpec(names, rule, by), axial: constSpec(axial, rule, by), note: 'src/design/build-needs.js' });
  const side = (re) => ({ by: 'the side\'s tie', rating: (d, a, b) => { const m = re.exec(a) ?? re.exec(b); return m ? [`twist-servo lead ${m[1]} tie`] : []; }, rule: TIE_RULE });
  const rated = 'a published rating (members.js check \'rated\'), FoS 1.5';
  return [
    R('bonded on the nose tray\'s flange', ['hatch post bonds'], BOND_RULE, 'the post\'s bond'),
    R('the rail bonded on its top', ['hatch rail bonds'], BOND_RULE, 'the rail\'s bond'),
    { why: exact('resting on the rail, screwed'), part: null, kind: 'fastened', load: constSpec(['hatch screws'], BOLT_RULE, 'the hatch screws'), axial: constSpec(['hatch screws in the rails'], SEAT_RULE, 'the screws\' seats in the rails'), note: '' },
    R('bonded on the cover\'s lands', ['hatch screws'], BOLT_RULE, 'the arcs carry only the window\'s film; the cover\'s screws hold the cover'),
    R('bonded under the nose tray\'s tip', ['pack stop forward bond'], BOND_RULE, 'the stop\'s bond'),
    R('lying on the pack\'s top, on its lead', ['nose tray at the root'], BEAM_RULE, 'the pack and its tray carry it', null, ['arming jumper mated'], 'resting'),
    R('mated with the pack\'s discharge plug', ['pack plug mated'], rated, 'the plug\'s published pull', null, ['pack plug mated'], 'connector'),
    R('plugged in the arming plug', ['arming jumper mated'], rated, 'the plug\'s published pull', null, ['arming jumper mated'], 'connector'),
    ...['on the probe\'s total-pressure take-off', 'on the probe\'s static take-off', 'on the sensor\'s barbed port', 'round the line, tied to the flange', 'round the nose tray\'s flange, through a slot', 'on its U.FL socket'].map((w) => { const by = { by: 'the line\'s tie', rating: (d, a, b) => [/antenna/.test(a + b) ? 'receiver antenna tie' : 'pressure tubing tie'], rule: TIE_RULE }; return { why: exact(w), part: null, kind: 'tied', load: by, axial: by, note: '' }; }),
    { why: exact('its root tabs screwed to the riser\'s web'), part: null, kind: 'fastened', load: constSpec(['belly tray screws'], BOLT_RULE, 'the belly tray\'s screws'), axial: constSpec(['belly tray screws in the riser\'s web'], SEAT_RULE, 'their seats'), note: '' },
    { why: exact('resting on the belly tray'), part: null, kind: 'resting', load: constSpec(['belly tray at the root'], BEAM_RULE, 'the tray beam'), axial: tieSpec((a, b) => (/^packed (.+?)(?: ties)?$/.exec(a) ?? /^packed (.+?)(?: ties)?$/.exec(b))?.[1] ?? null), note: '' },
    ...['fastened (its head or its nut on the face)', 'threaded on the screw', 'on its face'].flatMap((w) => [
      { why: exact(w), part: /^hatch (screw|nut) \d+$/, kind: 'fastened', load: constSpec(['hatch screws'], BOLT_RULE, 'the hatch screws'), axial: constSpec(['hatch screws in the rails'], SEAT_RULE, 'their seats'), note: '' },
      { why: exact(w), part: /^belly tray (screw|nut) \d+$/, kind: 'fastened', load: constSpec(['belly tray screws'], BOLT_RULE, 'the belly tray\'s screws'), axial: constSpec(['belly tray screws in the riser\'s web'], SEAT_RULE, 'their seats'), note: '' },
    ]),
    { why: exact(RIB_HOLE), part: /^twist-servo lead [RL] wing/, kind: 'resting', load: { by: 'the rib\'s hole', rating: (d, a, b) => { const m = /twist-servo lead ([RL])/.exec(a) ?? /twist-servo lead ([RL])/.exec(b); return m ? [`twist-servo lead ${m[1]} in the ribs`] : []; }, rule: rated }, axial: side(/twist-servo lead ([RL])/), note: '' },
    ...['its housing on the clip, tied', 'round the connector and its clip', 'plugged in the wing-root connector'].map((w) => ({ why: exact(w), part: /^twist-servo lead [RL]/, kind: 'tied', load: side(/twist-servo lead ([RL])/), axial: side(/twist-servo lead ([RL])/), note: '' })),
    { why: exact('bonded on the bracket\'s aft face'), part: /^twist-servo lead [RL] clip$/, kind: 'bonded', load: { by: 'the clip\'s bond', rating: (d, a) => [`${a} bond`], rule: BOND_RULE }, axial: { by: 'the same bond', rating: (d, a) => [`${a} bond`], rule: BOND_RULE }, note: '' },
    ...['the servo\'s own lead plugged in its socket', 'its plug on the FC\'s servo pins'].flatMap((w) => [
      { why: exact(w), part: /^twist-servo lead [RL]/, kind: 'tied', load: side(/twist-servo lead ([RL])/), axial: side(/twist-servo lead ([RL])/), note: 'the lead is held by its ties; the plug only makes contact' },
      R(w, ['harness ties'], TIE_RULE, 'the harness ties', /^tail lead [RL]$/, ['harness ties'], 'tied'),
    ]),
    ...['round the harness under the tray', 'round the tray\'s base, through its slots'].map((w) => R(w, ['harness ties'], TIE_RULE, 'the harness ties', /^harness tie/, ['harness ties'], 'tied')),
    // the pack: on its tray, held by its cage; the strap only seats it (no published rating)
    ...['resting on the nose tray', 'round the part', 'through its slots'].map((w) => ({ why: exact(w), part: /^packed battery( ties)?$/, kind: 'resting', load: constSpec(['nose tray at the root', 'nose tray flanges: battery lateral'], BEAM_RULE, 'the tray beam and its flanges'), axial: constSpec(['hatch aft stop', 'hatch keepers', 'pack stop forward'], 'the cage\'s plates (members.js plate check)', 'the cage: the cover\'s aft stop and keepers, the fixed forward stop'), note: 'the strap (no published rating) only seats the pack; the cage holds it' })),
  ];
}

/**
 * The rule helpers (interfaces.js RULE_HELPERS' own shapes, restated here: interfaces.js imports clearance.js, which
 * imports this module, so it cannot be imported here without a cycle at load time). The texts name the same rulings.
 */
const esc = (t) => t.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
export const LOCAL_RULE_HELPERS = {
  exact: (t) => new RegExp(`^${esc(t)}$`),
  constSpec: (names, rule, by) => ({ by, rating: () => names, rule }),
  tieSpec: (roleFn) => ({ by: 'the cable tie round it', rating: (d, a, b) => { const role = roleFn(a, b), mt = role ? d.params.layout?.mounts?.[role] : null; return mt ? [`${mt.carrier}: ${role} ties`] : []; }, rule: 'the cable tie maker\'s published loop/tensile strength (SPEC round 15 carriers; src/design/airframe.js tieFor)' }),
  BOLT_RULE: 'ISO 898-1 (or ISO 3506) proof load, or the fastener maker\'s rated load (SPEC Rulings after physics round 7, \'Every part reaches the airframe through a rated path\')',
  SEAT_RULE: 'pin-in-hole / lug bearing strength at the source\'s edge distance, full-or-half rule, with the EN 1999-1-1 / EN 1993-1-8 bolt-bearing fallback below the source\'s edge distance (SPEC Rulings after physics round 6 and round 8)',
  BOND_RULE: 'the adhesive maker\'s published lap-shear strength on that substrate (SPEC Rulings after physics round 7, \'Every part reaches the airframe through a rated path\')',
  TIE_RULE: 'the cable tie maker\'s published loop/tensile strength (SPEC round 15 carriers; src/design/airframe.js tieFor)',
  BEAM_RULE: 'the carrier beam\'s own bending check at the root, both ways at full value (SPEC round 15 carriers)',
};
/**
 * The build needs' interface rule rows (interfaces.js format), for interfaceLedger(asm, d, extraRules): an array
 * filled on first use (its rows name clearance.js's RIB_HOLE, not yet initialised while the module cycle loads).
 */
const rulesCache = [];
export const NEEDS_INTERFACE_RULES = new Proxy(rulesCache, { get(t, k, r) { if (!t.length) t.push(...needsInterfaceRules(LOCAL_RULE_HELPERS)); return Reflect.get(t, k, r); } });

// ---- the assembly sequence (round17.mjs sequence17) --------------------------------------------------------------

const X = (s) => [s, 0, 0], Y = (s) => [0, s, 0], Z = (s) => [0, 0, s];
const step = (name, members, dir, dist_m = 0.08, why = undefined) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
const thread = (name, members, why) => ({ name, members, threaded: true, why });

/** The build needs' steps: inserts after (or in place of) round 16's named steps. */
export const NEEDS_SEQUENCE = [
  { replace: 'nose tray riser', steps: [step('nose tray riser', ['^nose tray riser$', '^belly tray nut '], X(1), 0.06, 'with the belly tray\'s two nuts bonded on its web\'s aft face first')] },
  { after: 'nose tray nuts', steps: [step('belly tray', ['^belly tray$'], X(1), 0.12, 'aft under the nose tray to the riser\'s web'), step('belly tray screws', ['^belly tray screw '], X(1), 0.03), step('pack stop forward', ['^pack stop forward$'], Y(-1), 0.03, 'up under the nose tray\'s tip, bonded')] },
  ...['R', 'L'].map((t) => ({ after: `aft shoulder bracket ${t} with its bushing`, steps: [step(`twist-servo lead ${t} clip`, [`^twist-servo lead ${t} clip$`], X(-1), 0.04, 'from aft onto the bracket\'s housing, bonded')] })),
  { replace: 'fc', steps: [step('fc', ['^packed fc$'], X(1), 0.12, 'aft along the belly tray under the nose tray')] },
  { replace: 'regulator', steps: [step('regulator', ['^packed regulator$'], X(1), 0.12, 'aft along the belly tray under the nose tray')] },
  { after: 'former 5 tie', steps: [thread('pressure lines and antenna, tied', ['^pressure tubing (total|static)$', '^receiver antenna$', '^pressure tubing tie$', '^receiver antenna tie$'], 'flexible lines pushed on the probe\'s take-offs and the sensor\'s ports, the antenna on its U.FL socket, one tie round them and the flange')] },
  { after: 'former 5 tie', steps: [
    step('hatch frame', ['^hatch (rail|post) '], Y(1), 0.06, 'the rails bonded on their posts first; the posts down outside the nose tray\'s flanges'),
    step('pack plug', ['^pack plug$'], Y(1), 0.04, 'onto the pack\'s plug'), step('arming plug female', ['^arming plug female$'], Y(1), 0.04), step('arming jumper', ['^arming jumper$'], Y(1), 0.04),
    step('hatch cover nuts, right', ['^hatch nut [12]$'], Z(1), 0.02, 'captive under the rails, slid in from outboard'), step('hatch cover nuts, left', ['^hatch nut [34]$'], Z(-1), 0.02, 'captive under the rails, slid in from outboard'),
    step('hatch cover', ['^hatch cover( arcs)?$'], Y(1), 0.08, 'down onto the rails'), step('hatch screws', ['^hatch screw '], Y(1), 0.03),
  ] },
];
// the servo leads are flexible: laid once both wings are on (the wing-side extensions ride their wings, threaded through
// the ribs' holes with the wing built), each plugged at both ends and tied
NEEDS_SEQUENCE.push({ after: 'wing L', steps: [thread('servo leads, tied', ['^tail lead [RL]$', '^twist-servo lead [RL] (pod|wing|tie)$', '^harness tie '], 'flexible: each tail servo\'s own lead in its extension\'s socket; each twist servo\'s wing-side extension in through the wing root\'s slot and across the shoulder to the clip, the pod-side extension plugged on it; all laid along the harness to the FC\'s pins; the ties round the connectors and their clips, and round the harness and the trays\' bases')] });

// ---- containment declarations --------------------------------------------------------------------------------

/** The build needs' containment declarations (merged into round 16's): the hatch, the air path, the leads' slots. */
export function needsContainment(P, hatch) {
  const C = P.cooling, hole = (q) => Math.PI * (q.d / 2) ** 2;
  return {
    openings: [
      { name: 'hatch', why: 'the access hatch over the pack (closed in flight, flush with the covering): the pack comes out through it to charge, and the bird is armed, bound and set up through it', drag: false, solids: [], window: { x_mm: hatch.x_mm, polar_deg: hatch.polar_deg } },
      { name: 'cooling inlet', why: `a ${C.inlet.d} mm hole in the covering at x ${C.inlet.x} mm (polar ${C.inlet.polar} deg): air in under the nose, over the belly tray's FC and regulator and the nose tray's ESC`, drag: true, solids: [], window: { x_mm: [C.inlet.x - C.inlet.d / 2, C.inlet.x + C.inlet.d / 2], polar_deg: [C.inlet.polar, C.inlet.polar] }, area_mm2: +hole(C.inlet).toFixed(2), Cd: C.Cd_internal },
      { name: 'cooling outlet', why: `a ${C.outlet.d} mm hole in the covering at x ${C.outlet.x} mm (polar ${C.outlet.polar} deg): air out aft of the gearbox, the motor's and gears' heat with it`, drag: true, solids: [], window: { x_mm: [C.outlet.x - C.outlet.d / 2, C.outlet.x + C.outlet.d / 2], polar_deg: [C.outlet.polar, C.outlet.polar] }, area_mm2: +hole(C.outlet).toFixed(2), Cd: C.Cd_internal },
      ...['R', 'L'].flatMap((t) => [
        { name: `twist-servo lead ${t} through the wing root`, why: 'the wing-side extension runs out of the pod beside the rocker\'s spar socket, through the same slot widened by the lead, to the root rib: exposed between the skin and the root rib, in the spar\'s gap (the containment record would count its whole sweep over the stroke; its own instantaneous frontal area is counted in the buildneeds record)', drag: false, solids: [`twist-servo lead ${t} wing/spar run root`] },
        { name: `twist-servo lead ${t} in the wing`, why: 'the wing-side extension inside the wing\'s section from the root rib out to its socket on the servo\'s plug (its drag is the wing\'s own)', drag: false, solids: [`twist-servo lead ${t} wing/spar run`, `twist-servo lead ${t} wing/socket`] },
      ]),
    ],
    outside: [],
    on_skin: [{ re: '^hatch cover arcs$', why: 'the hatch cover\'s arcs carry the window\'s film: each band\'s outside on the loft' }],
  };
}

/** Round 16's declarations with the build needs' merged in. */
export function mergeContainment(base, add) {
  return { ...base, openings: [...(base.openings ?? []), ...add.openings], outside: [...(base.outside ?? []), ...add.outside], on_skin: [...(base.on_skin ?? []), ...add.on_skin] };
}

// ---- the buildneeds record ------------------------------------------------------------------------------------

/** The electrical joints the assembly order makes and how each is made (SPEC: solder joints are wire; connectors are not). */
export const ELECTRICAL_JOINTS = [
  { joint: 'pack to the power lead', means: 'connector', parts: ['pack plug'], basis: 'the pack ships with a Deans discharge plug (Gens Ace page); its mate is an AMASS T-plug male (Deans-compatible, flyingtech.co.uk)' },
  { joint: 'arming plug in the positive lead', means: 'connector', parts: ['arming plug female', 'arming jumper'], basis: 'an AMASS T-plug female in the lead and a T-plug male jumper (staged electrical.json 4.7)' },
  { joint: 'power lead to the ESC, the FC\'s VBAT pad and the regulator\'s input', means: 'soldered', basis: 'Tekko32 "BATTERY CONNECTOR: Solder/DIY" (ReadyMadeRC); F405-WMO "shipped unsoldered" (Matek); the regulator\'s pads: solder joints are wire (SPEC Completeness: wire exempt; audit C2)' },
  { joint: 'motor to the ESC', means: 'soldered', basis: 'the motor\'s phase leads soldered to the ESC\'s pads (the motor\'s 2.5 mm bullets and the box\'s females unused: their mass and rating are unpublished)' },
  { joint: 'ESC signal to the FC', means: 'soldered', basis: 'the ESC\'s supplied 24 AWG signal wires on the FC\'s motor pads' },
  { joint: 'regulator output to the FC\'s servo rail', means: 'soldered', basis: 'wire from the regulator\'s output pads to the servo rail pads' },
  { joint: 'receiver to the FC', means: 'soldered', basis: 'TBS: "The receiver does not come with any connectors on-board" (quick-start guide, 2018-02-16)' },
  { joint: 'antenna to the receiver', means: 'connector', parts: ['receiver antenna'], basis: 'TBS: "The antenna uses a U.FL connector"; the antenna is in the receiver\'s package' },
  { joint: 'GPS to the FC', means: 'soldered', basis: 'wire on the FC\'s UART pads (the GPS\'s own connector unsourced; not relied on)' },
  { joint: 'crank encoder readhead to the FC', means: 'soldered', basis: 'the readhead\'s own cable on the FC\'s UART pads (wire)' },
];

/** Whether two solids may meet (one part's, or a pair the clearance check exempts). */
function exemptOf(asm) {
  const key = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`), joins = new Map(asm.joins.map(([a, b, w]) => [key(a, b), w]));
  return (m, t) => m.part === t.part || EXEMPT_JOINTS.includes(joins.get(key(m.id, t.id)) ?? joins.get(key(m.id, t.part)) ?? joins.get(key(m.part, t.id)) ?? joins.get(key(m.part, t.part)));
}

/** Every solid of the assembly at pose q as { part, id, x, sec }. */
const solidsAt = (asm, q) => asm.parts.flatMap((p) => p.solids.map((s) => ({ part: p.id, id: `${p.id}/${s.name}`, x: s.x, sec: s.at ? s.at(q) : s.shape })));

/**
 * The pack's removal to charge, as the reverse of its fitting: each step lifts its parts out along its line against
 * everything still in place (sequence.js slideGap; joined exempt pairs may meet); then the pack's sweep is held to the
 * hatch's declared window where it crosses the skin. -> { steps, crossing, window, ok }.
 */
export function removalRecord(asm, d, q0, window, P) {
  const all = solidsAt(asm, q0), exempt = exemptOf(asm), removed = new Set(), rows = [];
  const plan = [['hatch screws', /^hatch screw \d+$/, [0, 1, 0], 0.03], ['hatch cover', /^hatch cover( arcs)?$/, [0, 1, 0], 0.12], ['arming jumper', /^arming jumper$/, [0, 1, 0], 0.12], ['arming plug female', /^arming plug female$/, [0, 1, 0], 0.12], ['pack plug', /^pack plug$/, [0, 1, 0], 0.12], ['strap (released)', /^packed battery ties$/, null, 0], ['pack', /^packed battery$/, [0, 1, 0], 0.12]];
  for (const [name, re, dir, dist] of plan) {
    const moving = all.filter((s) => re.test(s.part));
    moving.forEach((s) => removed.add(s.id));
    if (!dir) { rows.push({ step: name, released: true, solids: moving.length, ok: moving.length > 0 }); continue; }
    const placed = all.filter((s) => !removed.has(s.id));
    let worst = Infinity; const through = [];
    for (const m of moving) for (const t of placed) {
      if (exempt(m, t) || !sweptMeets(m, t, dir, dist)) continue;
      const g = slideGap(m, t, dir, dist);
      if (g.gap < worst) worst = g.gap;
      if (g.gap < -1e-7) through.push({ moving: m.id, placed: t.id, by_mm: r3(-g.gap) });
    }
    rows.push({ step: name, dir, dist_mm: dist * 1000, solids: moving.length, worst_gap_mm: Number.isFinite(worst) ? r3(worst) : null, through: through.slice(0, 8), ok: moving.length > 0 && through.length === 0 });
  }
  const pack = all.filter((s) => s.part === 'packed battery'), cross = sweepCrossing(d.body.loft, pack, [0, 1, 0], 0.12);
  const inWin = cross.n === 0 || (cross.x[0] >= window.x_mm[0] / 1000 - 1e-9 && cross.x[1] <= window.x_mm[1] / 1000 + 1e-9 && cross.polar[0] >= window.polar_deg[0] - 1e-9 && cross.polar[1] <= window.polar_deg[1] + 1e-9);
  return { steps: rows, crossing: { x_mm: cross.x.map(r3), polar_deg: cross.polar.map((a) => +a.toFixed(1)), points: cross.n }, window, crossing_in_window: inWin, ok: rows.every((r) => r.ok) && inWin && cross.n > 0 };
}

/** Whether a moving solid's swept box (along dir over dist) comes within 1 mm of a placed solid's box. */
function sweptMeets(m, t, dir, dist) {
  const a = bboxOf(m.sec), b = bboxOf(t.sec), dz = dir[2] * dist, dy = dir[1] * dist, dx = dir[0] * dist, g = 0.001;
  const z = [Math.min(a.z[0], a.z[0] + dz), Math.max(a.z[1], a.z[1] + dz)], y = [Math.min(a.y[0], a.y[0] + dy), Math.max(a.y[1], a.y[1] + dy)], x = [Math.min(m.x[0], m.x[0] + dx), Math.max(m.x[1], m.x[1] + dx)];
  return !(z[1] < b.z[0] - g || b.z[1] < z[0] - g || y[1] < b.y[0] - g || b.y[1] < y[0] - g || x[1] < t.x[0] - g || t.x[1] < x[0] - g);
}

/** Where a moving box's outline crosses the loft over its sweep: the x and elliptic polar range of every point outside. */
export function sweepCrossing(loft, sols, dir, dist, nT = 24) {
  const xs = [Infinity, -Infinity], pol = [Infinity, -Infinity];
  let n = 0;
  for (const s of sols) {
    const pts = s.sec.pts ?? [];
    for (let k = 0; k <= nT; k++) for (const xx of [s.x[0], (s.x[0] + s.x[1]) / 2, s.x[1]]) {
      const t = (k / nT) * dist, x = xx + dir[0] * t, h = loftHalf(loft, x);
      if (!h) continue;
      for (const p of pts) {
        const z = p[0] + dir[2] * t, y = p[1] + dir[1] * t, e = Math.hypot((y - h.y) / h.hh, z / h.hw);
        if (e < 1) continue;
        n++; xs[0] = Math.min(xs[0], x); xs[1] = Math.max(xs[1], x);
        // where the point's path met the skin: its polar angle there (the path is along y here, so the angle of the crossing point)
        const yc = h.y + Math.sign(y - h.y) * h.hh * Math.sqrt(Math.max(0, 1 - (z / h.hw) ** 2)), a = (Math.atan2((yc - h.y) / h.hh, z / h.hw) * 180) / Math.PI;
        pol[0] = Math.min(pol[0], a); pol[1] = Math.max(pol[1], a);
      }
    }
  }
  return { x: xs, polar: pol, n };
}

/** A column from an item's top straight up to the skin: free of every solid left in place once the hatch is off? */
function reachRow(asm, d, q0, item, partId, P, why, at = null) {
  const all = solidsAt(asm, q0), it = all.filter((s) => s.part === partId);
  if (!it.length) return { item, ok: false, why: `${partId} not drawn` };
  const box = it.map((s) => s.sec.pts).flat(), xs = [Math.min(...it.map((s) => s.x[0])), Math.max(...it.map((s) => s.x[1]))];
  const xm = at?.x != null ? at.x(xs) : (xs[0] + xs[1]) / 2, zm = (Math.min(...box.map((p) => p[0])) + Math.max(...box.map((p) => p[0]))) / 2, yTop = Math.max(...box.map((p) => p[1]));
  const h = loftHalf(d.body.loft, xm), yS = h.y + h.hh * Math.sqrt(Math.max(0, 1 - (zm / h.hw) ** 2)), f = 0.003;
  const col = { part: 'reach', id: 'reach/column', x: [xm - f, xm + f], sec: poly([[zm - f, yTop + 0.0005], [zm + f, yTop + 0.0005], [zm + f, yS], [zm - f, yS]]) };
  const off = /^(hatch (cover|cover arcs|screw \d+))$/, hit = all.filter((s) => s.part !== partId && !off.test(s.part)).filter((t) => gap3(col.x, t.x, gap2(col.sec, t.sec)) < 0).map((t) => t.id);
  const win = d.params.layout.hatch, inX = xm >= win.x_mm[0] / 1000 && xm <= win.x_mm[1] / 1000;
  return { item, part: partId, column_mm: { x: r3(xm), z: r3(zm), from_y: r3(yTop), to_skin_y: r3(yS) }, under_window: inX, blocked_by: hit.slice(0, 6), why, ok: inX && hit.length === 0 };
}

/** The record (SPEC "What a real build needs"). asm: the full assembly with asm.needs (needsParts). -> { rule, rows, failed, passed }. */
export function buildNeedsRecord(asm, d, catalog) {
  const rule = 'SPEC Function completeness, "What a real build needs" (2026-11-03): a connector for every electrical joint the assembly order makes (solder joints are wire, exempt); a hatch with a rated fixing, declared as an opening, through which the pack comes out to charge without cutting the covering and the bird is armed, bound and set up; a declared air path with its drag counted and its cooling assumption stated; src/design/build-needs.js';
  const N = asm.needs;
  if (!N) return { rule, rows: [{ item: 'build needs drawn', ok: false, why: 'no hatch window (params.layout.hatch) or no carriers' }], failed: ['build needs drawn'], passed: false };
  const rows = [...jointRows(asm, N), ...hatchRows(asm, d, N), ...accessRows(asm, d, N), ...airRows(d, N), ...packRows(d, catalog, N)];
  const failed = rows.filter((r) => !r.ok).map((r) => r.item);
  // round 18: the drag the air path's holes and the leads' exposed span add (their rows' CdA), for the body's drag
  const cda = rows.filter((r) => Number.isFinite(r.CdA_m2)).reduce((a, r) => a + r.CdA_m2, 0);
  return { rule, rows, failed, CdA_m2: +cda.toFixed(8), CdA_basis: 'the air path\'s holes at Cd 1.0 on their area (an upper bound) and the twist-servo leads\' exposed span at Cd 1.2 (their rows): not in the containment record\'s frontal area (the holes have no solids; the leads\' span is declared drag false there)', passed: failed.length === 0 };
}

/** Every electrical joint: the table's, the servo leads', and the packaged cables' reach. */
function jointRows(asm, N) {
  const ids = new Set(asm.parts.map((p) => p.id)), rows = [];
  for (const j of ELECTRICAL_JOINTS) {
    const missing = (j.parts ?? []).filter((p) => !ids.has(p));
    rows.push({ item: `joint: ${j.joint}`, means: j.means, parts: j.parts ?? [], basis: j.basis, ok: j.means === 'soldered' || (j.parts?.length > 0 && missing.length === 0), ...(missing.length ? { why: `no ${missing.join(', ')}` } : {}) });
  }
  for (const L of N.leads ?? []) rows.push({ item: `joint: ${L.joint}`, means: 'connectors and extensions', ...L, pts: undefined, frame: undefined, ok: L.ok === true || L.moving === false });
  rows.push(cableReach('airspeed board to the FC (Matek\'s 20 cm JST-GH-4P cable, in the board\'s package)', asm, 'packed airspeed', 'packed fc', 0.2, N.P));
  return rows;
}

/** The hatch: present with a rated fixing, declared as an opening, and the pack out through it. */
function hatchRows(asm, d, N) {
  const ids = new Set(asm.parts.map((p) => p.id)), has = (n) => (d.structure?.members ?? []).find((m) => m.name === n);
  const missing = ['hatch rail R', 'hatch rail L', 'hatch cover', 'hatch cover arcs'].filter((p) => !ids.has(p)), screws = asm.parts.filter((p) => /^hatch screw \d+$/.test(p.id));
  const fix = ['hatch screws', 'hatch screws in the rails'].map((n) => ({ n, m: has(n) }));
  const op = (d.params.layout.containment?.openings ?? []).find((o) => o.name === 'hatch'), win = d.params.layout.hatch;
  return [
    { item: 'hatch with a rated fixing', parts_missing: missing, screws: screws.length, members: fix.map((f) => ({ name: f.n, check: f.m?.check ?? null })), ok: !missing.length && screws.length > 0 && fix.every((f) => f.m && f.m.check !== 'unrated'), basis: 'ISO 4762 M3 10.9 into ISO 4032 nuts (ISO 898-1 proof loads, catalogued), checked in M7 as a bolt group with their seats in the rails' },
    { item: 'hatch declared as an opening', declared: !!op, window: op?.window ?? null, ok: !!op && JSON.stringify(op.window) === JSON.stringify({ x_mm: win.x_mm, polar_deg: win.polar_deg }) },
    { item: 'the pack comes out through the hatch to charge, the covering uncut', ...removalRecord(asm, d, N.q0, win, N.P) },
  ];
}

/** Armed, bound, charged and set up through the hatch. */
function accessRows(asm, d, N) {
  return [
    reachRow(asm, d, N.q0, 'armed: the arming jumper under the hatch', 'arming jumper', N.P, 'the jumper comes out of its female straight up through the window'),
    reachRow(asm, d, N.q0, 'bound: the receiver\'s bind button under the hatch', 'packed receiver', N.P, 'TBS: press and release the "BIND" button (its place on the board unpublished: checked at the board\'s forward end, clear of the board\'s own tie at its middle)', { x: (xs) => xs[1] - 0.004 }),
    reachRow(asm, d, N.q0, 'charged: the pack\'s plug under the hatch', 'pack plug', N.P, 'unplugged straight up through the window'),
    cableReach('set up: the FC to its USB-C extender at the hatch (Matek\'s 20 cm JST-SH-6P cable, in the FC\'s package)', asm, 'packed fc', null, 0.2, N.P, d),
  ];
}

/** The air path and the leads' exposed span, each with its drag. */
function airRows(d, N) {
  const C = d.params.layout.containment?.openings?.filter((o) => /^cooling (inlet|outlet)$/.test(o.name)) ?? [];
  const cda = C.reduce((a, o) => a + (o.drag !== false ? (o.Cd ?? 1) * (o.area_mm2 ?? 0) * 1e-6 : 0), 0);
  const exposed = (N.leads ?? []).filter((l) => l.wing).map((l) => ({ lead: l.name, length_mm: l.wing.root_span_mm, d_mm: N.P.leads.d_assumed }));
  const cdaLead = exposed.reduce((a, e) => a + 1.2 * e.length_mm * e.d_mm * 1e-6, 0);
  return [
    { item: 'air path declared with its drag', openings: C.map((o) => ({ name: o.name, area_mm2: o.area_mm2, Cd: o.Cd, counts_in_drag: o.drag !== false })), CdA_m2: +cda.toFixed(8), CdA_basis: 'the holes\' area at Cd 1.0: the whole inflow\'s momentum lost (an upper bound on the internal flow\'s drag; the holes stand out of nothing)', cooling: 'UNVERIFIED: no maker publishes a cooling figure (Scorpion: "you must provide adequate cooling", unquantified; Tekko32: "direct airflow cooling", no figure; AM32 cuts duty above its temperature limit); the path is the design\'s own assumption', ok: C.length === 2 && C.every((o) => o.drag !== false && o.area_mm2 > 0) },
    { item: 'the twist-servo leads\' span through the wing root slots, its drag counted', exposed, CdA_m2: +cdaLead.toFixed(8), basis: 'each lead\'s run from the axis\'s side of the slot to the root rib (an upper bound on its exposed length), its assumed diameter, Cd 1.2 (the containment record\'s bluff-body figure)', ok: exposed.length === (N.leads ?? []).filter((l) => l.moving).length },
  ];
}

/** The strap and its cage, the pressure tubing and the antenna. */
function packRows(d, catalog, N) {
  const P = N.P, has = (n) => (d.structure?.members ?? []).some((m) => m.name === n);
  const strap = d.structure?.airframe?.parts?.support?.ties?.find((t) => t.part === 'battery'), sp = byId(catalog, P.strap.id).specs;
  const cage = ['hatch aft stop', 'hatch keepers', 'pack stop forward', 'nose tray flanges: battery lateral'].map((n) => ({ name: n, present: has(n) }));
  const tub = N.tubing;
  return [
    { item: 'the pack on a releasable strap', strap: strap?.tie ?? null, releasable: !!sp.releasable, rating: 'none published: the strap only seats the pack; the cage holds it', cage, ok: strap?.tie === P.strap.id && !!sp.releasable && cage.every((c) => c.present) },
    { item: 'pitot pressure tubing drawn and in the BOM', parts: tub ? tub.lines.map((l) => l.id) : [], stated_length_mm: P.tubing.length_mm ?? null, drawn_mm: tub ? r3(tub.L) : null, ok: !!tub && tub.lines.length === 2 && Math.abs(tub.L - (P.tubing.length_mm ?? 0) / 1000) <= 0.001 },
    { item: 'receiver antenna drawn', drawn_mm: N.antenna ? r3(N.antenna.L) : null, mass: 'within the receiver\'s published 0.5 g (TBS does not say whether it includes the antenna): unsourced', length: 'unsourced: drawn as routed', ok: !!N.antenna },
  ];
}

/** A packaged cable's reach: the shortest stated path between two drawn parts' nearest faces, round the nose tray's edge, against its length (with the slack). */
function cableReach(item, asm, a, b, L, P, d = null) {
  const box = (id) => { const s = asm.parts.find((p) => p.id === id)?.solids ?? []; if (!s.length) return null; const pts = s.flatMap((q) => q.shape?.pts ?? []); return { x: [Math.min(...s.map((q) => q.x[0])), Math.max(...s.map((q) => q.x[1]))], z: [Math.min(...pts.map((p) => p[0])), Math.max(...pts.map((p) => p[0]))], y: [Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[1]))] }; };
  const A = box(a);
  if (!A) return { item, ok: false, why: `${a} not drawn` };
  let pts;
  if (b) {
    const B = box(b);
    if (!B) return { item, ok: false, why: `${b} not drawn` };
    const zOut = -(0.0191 + 0.004), xA = Math.max(A.x[0], Math.min(A.x[1], B.x[1])), xB = Math.min(B.x[1], Math.max(B.x[0], xA));
    pts = [[xA, A.z[0], (A.y[0] + A.y[1]) / 2], [xA, zOut, (A.y[0] + A.y[1]) / 2], [xB, zOut, B.y[1]], [xB, B.z[0], B.y[1]]];
  } else {
    const win = d.params.layout.hatch, xw = Math.max(win.x_mm[0] / 1000 + 0.004, A.x[0]), zOut = 0.0191 + 0.004;
    pts = [[xw, A.z[1], (A.y[0] + A.y[1]) / 2], [xw, zOut, (A.y[0] + A.y[1]) / 2], [xw, zOut, -0.02]];
  }
  const len = pathLength(pts);
  return { item, path_mm: r3(len), with_slack_mm: r3(len * (1 + P.leads.slack_frac)), cable_mm: L * 1000, drawn: false, note: 'the maker\'s own cable (wire; its mass unpublished): reach along a stated path round the nose tray\'s left edge, not drawn', ok: len * (1 + P.leads.slack_frac) <= L + 1e-9 };
}

/**
 * Whether a pod structure piece (a former's band piece at x, its (z, y) points) lies outside the hatch's window (the
 * pod's structure leaves the window open: the contract with pod-structure.js). Its centroid's polar angle and x.
 */
export function outsideWindow(pts, x, loft, hatch) {
  if (!hatch) return true;
  const c = pts.reduce((a, p) => [a[0] + p[0] / pts.length, a[1] + p[1] / pts.length], [0, 0]), h = loftHalf(loft, x);
  const a = (Math.atan2((c[1] - h.y) / h.hh, c[0] / h.hw) * 180) / Math.PI;
  return !(x >= hatch.x_mm[0] / 1000 && x <= hatch.x_mm[1] / 1000 && a >= hatch.polar_deg[0] && a <= hatch.polar_deg[1]);
}
