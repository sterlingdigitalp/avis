// Round 8: the airframe load path (SPEC ruling after physics round 7: "Every part reaches the airframe through a rated
// path"). How the gearbox plates, shoulders, battery, tail boom, pod and the other carried parts connect to each other
// and to the wing loads: every fastener rated against ISO 898-1 minimum proof loads (src: catalogue fastener entries),
// every bond against the adhesive maker's strength on that substrate, every tie against its maker's minimum loop
// strength, the plates and brackets sized in bending, each part in the BOM with its mass. The loads are the flight
// model's channels (flight.js LOAD_CHANNELS): the drive and wing-root channels and round 8's sf_origin, wdot, w2
// (a part of mass m at distance r from the body origin carries at most m (sf_origin + r wdot + r w2)) and tail_F.
//
// Layout (body frame x forward, y up, z right; from the drive layout of sims/avis-shared/model.js, used unchanged):
//  - Each side's two shoulder housings are bosses of a 6061-T6 shoulder bracket bonded on a 7075-T6 shoulder rail,
//    a tube parallel to x at dy above the hinge, seated in both gearbox plates: a beam on the two plates with the
//    aft housing overhung, its torsion locked by screws at the forward plate.
//  - The two plates (solid 6061-T6, outline the hull of every seat and pad, no lightening credited) are tied into a
//    box by two side panels and a floor panel of 6061-T6 sheet bonded with Araldite 2011.
//  - The keel / tail boom (Rock West T-RND-393) runs over the gearbox, bonded in two 6061-T6 clamp blocks screwed
//    to the plates' outer faces; its root moment reaches the plates as a couple of in-plane forces.
//  - A forward tray (6061-T6 sheet) screwed under the motor to the forward plate carries the battery, ESC, receiver
//    and pitot on cable ties; an aft tray screwed to the aft plate carries the tail servos, flight controller and GPS.
//  - The motor bolts to the forward plate with four screws in its own threads; the crank encoder with two.
//  - The pod's formers and the tail surfaces are tied to the boom and trays.
// The frame parts are not in the film's packing check (model.js places no frame part): their positions are stated
// in the design file and their clearances to the gear tip circles and rocker hubs are computed and reported here.
import { byId } from './catalog.js';
import { shaftMomentSections, channelSection } from '../physics/members.js';
import { packedSpecs, partsIndex, loftHalf } from '../../sims/avis-shared/model.js';
import { fixingLayout, fixingCounts } from './fixings.js';

const PI4 = Math.PI / 4;
const AL7 = 'metal-al-7075-t6', AL6 = 'metal-al-6061-t6';
const sect = (name, radial) => ({ name, radial, axial: [] });
const hyp = (a, b, c = 0) => Math.hypot(a, b, c);

/** Merge channel terms (sum k per channel), dropping zeros. */
export function addTerms(...lists) {
  const m = new Map();
  for (const l of lists) for (const t of l ?? []) if (t.k) m.set(t.ch, (m.get(t.ch) ?? 0) + Math.abs(t.k));
  return [...m].map(([ch, k]) => ({ ch, k: +k.toFixed(8) }));
}
export const scaleTerms = (terms, f) => addTerms((terms ?? []).map((t) => ({ ch: t.ch, k: t.k * f })));

/**
 * Bound on the inertial force (N) on a mass m (kg) at body position r (m) from the origin, as channel terms:
 * |f(r)| <= |f_origin| + |wdot| |r| + |w|^2 |r| (rigid body: f(r) = f_o + wdot x r + w x (w x r)).
 */
export function inertialTerms(m, r) {
  const R = hyp(r[0], r[1], r[2]);
  return addTerms([{ ch: 'sf_origin', k: m }, { ch: 'wdot', k: m * R }, { ch: 'w2', k: m * R }]);
}

/** A fastener, tie or adhesive as the checks need it, from its catalogue entry's design values. */
export function fastenerOf(catalog, id) {
  const p = byId(catalog, id);
  const dv = p.specs?.design_values ?? {};
  if (!Number.isFinite(dv.tension_N) && !Number.isFinite(dv.loop_N)) throw new Error(`${id}: no rated load in the catalogue`);
  return { id, name: p.name, mass_g: p.mass_g, tension_N: dv.tension_N ?? null, shear_N: dv.shear_N ?? null, loop_N: dv.loop_N ?? null, basis: dv.basis, source_url: p.source_url };
}
export function adhesiveOf(catalog, id) {
  const p = byId(catalog, id);
  return { id, shear_MPa: p.specs.design_values.shear_MPa, basis: p.specs.design_values.basis };
}

/** A fastener group's geometry: n fasteners evenly on a circle of radius r (mm), or a row of n at pitch s (mm). */
export function circleGroup(n, r) { return { n, r_max_mm: r, sum_r2_mm2: n * r * r }; }
export function rowGroup(n, s) {
  const xs = Array.from({ length: n }, (_, i) => (i - (n - 1) / 2) * s);
  return { n, r_max_mm: Math.max(...xs.map(Math.abs)), sum_r2_mm2: xs.reduce((a, x) => a + x * x, 0) };
}

/** Area (m^2) and polygon of the convex hull of circles [{ c: [z, y], r }] (m), sampled at 72 points each. */
export function hullOf(circles) {
  const pts = [];
  for (const { c, r } of circles) for (let i = 0; i < 72; i++) pts.push([c[0] + r * Math.cos((i * Math.PI) / 36), c[1] + r * Math.sin((i * Math.PI) / 36)]);
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], hi = [];
  for (const p of pts) { while (lo.length >= 2 && cross(lo.at(-2), lo.at(-1), p) <= 0) lo.pop(); lo.push(p); }
  for (const p of [...pts].reverse()) { while (hi.length >= 2 && cross(hi.at(-2), hi.at(-1), p) <= 0) hi.pop(); hi.push(p); }
  const poly = [...lo.slice(0, -1), ...hi.slice(0, -1)];
  let A = 0;
  for (let i = 0; i < poly.length; i++) { const p = poly[i], q = poly[(i + 1) % poly.length]; A += p[0] * q[1] - q[0] * p[1]; }
  return { area_m2: Math.abs(A) / 2, poly };
}
/** Width (m) of a convex polygon [[z, y]] at height y. */
export function hullWidthAt(poly, y) {
  const zs = [];
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i], q = poly[(i + 1) % poly.length];
    if ((p[1] - y) * (q[1] - y) <= 0 && p[1] !== q[1]) zs.push(p[0] + ((y - p[1]) / (q[1] - p[1])) * (q[0] - p[0]));
  }
  return zs.length >= 2 ? Math.max(...zs) - Math.min(...zs) : 0;
}

export const AIRFRAME_DEFAULTS = {
  fastener: 'screw-iso4762-m3x8-10.9', nut: 'nut-iso4032-m3-10',
  adhesiveAl: 'adhesive-araldite-2011-aluminium', adhesiveCf: 'adhesive-araldite-2011',
  tieSmall: 'tie-hellermanntyton-t18r', tieLarge: 'tie-hellermanntyton-t40r',
  rail: { material: AL7, od_mm: 12, id_mm: 10, dz_mm: 4, dy_mm: 11.5, seat_D_mm: 25, aft_boss_mm: 3, sleeve_len_mm: 8, sleeve_wall_mm: 2, arm_material: AL6, arm_b_mm: 10, arm_t_mm: 6, lock_screws: 2, lock_r_mm: 14 },
  plates: { edge_mm: 5, pad_mm: 10, post_w_mm: 12 },
  panels: { material: AL6, flange_mm: 16, list: [{ name: 'side panel right', kind: 'side', z_mm: 31, y0_mm: 3, h_mm: 21.5, t_mm: 0.85 }, { name: 'side panel left', kind: 'side', z_mm: -31, y0_mm: 12.8, h_mm: 11.7, t_mm: 1.5 }, { name: 'top panel', kind: 'top', y_mm: 56.5, w_mm: 20, t_mm: 0.7 }] },
  boom: { y_mm: 62, clamp_material: AL6, clamp_len_mm: 6, clamp_wall_mm: 1.5, screws: 2, screw_pitch_mm: 14, screw_offset_mm: 8 },
  tray: { material: AL6, w_mm: 22, flange_h_mm: 7, t_mm: 0.8, y_mm: -30, screws: 2, screw_pitch_mm: 18, screw_lever_mm: 8, flange_mm: 10, doubler_mm: 2, battery_ties: 2, item_height_mm: 22 },
  aftTray: { material: AL6, w_mm: 26, flange_h_mm: 6, t_mm: 0.6, y_mm: -76, screws: 2, screw_pitch_mm: 16, screw_lever_mm: 8, flange_mm: 10, doubler_mm: 0, servo_ties: 2, item_height_mm: 30 },
  motor: { screws: 2, circle_r_mm: 8, overhang_mm: 22, doubler_mm: 2 },
  encoder: { ties: 2 },
  pod: { formers: 5, ties_each: 1, V_drag_m_s: 25 },
  pitot: { ties: 2 },
  tail: { ties: 2 },
};

/**
 * The airframe's members and mass items, appended to the linkage context cx (linkage.js): cx.members, cx.items.
 * Returns { report } with the frame geometry, plate areas and clearances for the design file.
 */
export function airframeMembers(cx) {
  const { d, catalog, L, lay, jointByName, planes, plateT, members, mat, item, span } = cx;
  const A = mergeDefaults(AIRFRAME_DEFAULTS, L.airframe ?? {});
  const scr = fastenerOf(catalog, A.fastener);
  const nut = byId(catalog, A.nut);
  const tieS = fastenerOf(catalog, A.tieSmall), tieL = fastenerOf(catalog, A.tieLarge);
  const glueAl = adhesiveOf(catalog, A.adhesiveAl), glueCf = adhesiveOf(catalog, A.adhesiveCf);
  const lump = (nm) => { const l = d.mass.lumps.find((q) => q.name === nm); return l ? { m: l.mass_kg, r: l.pos } : null; };
  const counts = { screws: 0, nuts: 0, tieS: 0, tieL: 0 };
  const xA = planes.plateAft, xF = planes.plateFwd, gap = xF - xA;
  const x0 = lay.x0 ?? d.wing.hinge[0];
  const shafts = lay.train.shafts; // [z, y] crank first, motor last
  const crank = shafts[0];
  const report = { layout_note: 'frame parts are not in the film packing check (model.js places none); positions stated here, clearances computed from the drive layout', parts: {} };
  const tFwd = (L.plates.t_fwd_mm ?? L.plates.t_mm), tAft = L.plates.t_mm;
  const screwSeat = (name, terms, t = plateT * 1000) => members.push({ name, kind: 'seat', check: 'seat', material: mat(L.plates.material), dims: { d_mm: 3, t_mm: t, e_over_d: 2 }, sections: [sect('F', terms)], uses: { F: 'F' }, note: 'an M3 screw bearing in the plate, edge distance at least 2 d' });
  const bolts = (name, group, V, T, M, F, note) => {
    members.push({ name, kind: 'fasteners', check: 'bolts', fastener: { id: scr.id, tension_N: scr.tension_N, shear_N: scr.shear_N }, dims: group, sections: [sect('V', V), sect('T', T), sect('M', M), sect('F', F)], uses: { V: 'V', T: 'T', M: 'M', F: 'F' }, note });
  };
  // A screwed flange's prying moment is a local couple on the plate (screw pull, flange heel push) over the flange width.
  const flangePlate = (name, w_mm, Mo, doubler_mm = 0) => {
    const tp = /^forward/.test(name) ? tFwd : tAft;
    // A screwed (unbonded) doubler shares the moment by stiffness: the stack's stress 6 M t_max / (b sum t^3), as a plate of t_eff.
    const tEff = doubler_mm > 0 ? Math.sqrt((tp ** 3 + doubler_mm ** 3) / Math.max(tp, doubler_mm)) : tp;
    members.push({ name, kind: 'plate', check: 'plate', material: mat(L.plates.material), dims: { b_mm: w_mm, t_mm: +tEff.toFixed(4) }, sections: [sect('Mo', Mo)], uses: { Mo: 'Mo' }, note: `the flange's prying moment as out-of-plane bending of the plate over the flange width${doubler_mm > 0 ? `, with a ${doubler_mm} mm 6061-T6 doubler under the same screws (unbonded: t_eff ${tEff.toFixed(2)} mm)` : ''}` });
    // round 16 (tray.doubler_h_mm): the doubler's height as drawn, its screws' edge distance either side
    const dh = /tray/.test(name) && A.tray.doubler_h_mm ? A.tray.doubler_h_mm : 2 * (/motor/.test(name) ? A.motor.circle_r_mm : 12);
    if (doubler_mm > 0) item(`${name.replace(/^(forward|aft) plate under the /, '')} doubler`, mat(AL6), (w_mm / 1000) * (dh / 1000) * (doubler_mm / 1000), `${w_mm} x ${dh} mm x ${doubler_mm} mm 6061-T6 under the flange screws`);
  };
  const perScrewShear = (group, V, T) => addTerms(scaleTerms(V, 1 / group.n), group.sum_r2_mm2 > 0 ? scaleTerms(T, (group.r_max_mm / 1000) / (group.sum_r2_mm2 * 1e-6)) : []);

  const ax = { cx, d, catalog, L, lay, jointByName, planes, plateT, members, mat, item, span, A, scr, nut, tieS, tieL, glueAl, glueCf, lump, counts, xA, xF, gap, x0, shafts, crank, report, tFwd, tAft, screwSeat, bolts, flangePlate, perScrewShear, loftHalfAt: loftHalf };
  af_shoulders(ax);
  af_boom(ax);
  af_trays(ax);
  af_motor(ax);
  af_encoder(ax);
  af_ties(ax);
  af_plateSections(ax);
  af_panels(ax);
  af_bomItems(ax);
  return { report, plates: ax.plates };
}

function af_shoulders(ax) {
  const { L, lay, jointByName, plateT, members, mat, item, span, A, glueAl, counts, xA, xF, gap, x0, tFwd, tAft, screwSeat, bolts, perScrewShear } = ax;
  // ---- Shoulders: bracket on a rail, the rail a beam on the two plates --------------------------------------------
  const ra = A.rail, rm = mat(ra.material);
  const xa = x0 - span / 2, xf = x0 + span / 2;
  const railPlates = { aft: [], fwd: [] };
  const railPos = {};
  for (const [side, w, sg] of [['right', 'R', 1], ['left', 'L', -1]]) {
    const jt = jointByName(`shoulder pivot ${side}`);
    const [sAft, sFwd] = ['aft', 'forward'].map((nm) => jt.supports.find((s) => s.name === nm));
    const Faft = sAft.radial.map((t) => ({ ch: t.ch, k: t.k })), Ffwd = sFwd.radial.map((t) => ({ ch: t.ch, k: t.k }));
    // round 15: the motor's bracket on the rails (af_motor), its whole force on each rail (conservative)
    const mb = motorBracketLoad(ax);
    const loads = [...Faft.map((t) => ({ ch: t.ch, perUnit: t.k, x: xa })), ...Ffwd.map((t) => ({ ch: t.ch, perUnit: t.k, x: xf })), ...(mb ? mb.F.map((t) => ({ ch: t.ch, perUnit: t.k, x: mb.x })) : [])];
    const ms = shaftMomentSections(xA, xF, loads, 'M');
    const RA = addTerms(scaleTerms(Faft, Math.abs(xF - xa) / gap), scaleTerms(Ffwd, Math.abs(xF - xf) / gap), ...(mb ? [scaleTerms(mb.F, Math.abs(xF - mb.x) / gap)] : []));
    const RB = addTerms(scaleTerms(Faft, Math.abs(xa - xA) / gap), scaleTerms(Ffwd, Math.abs(xf - xA) / gap), ...(mb ? [scaleTerms(mb.F, Math.abs(mb.x - xA) / gap)] : []));
    const Ftot = addTerms(Faft, Ffwd);
    const dy = Math.hypot(ra.dz_mm, ra.dy_mm) / 1000; // the bracket arm: the rail's offset from the hinge axis
    const Trail = scaleTerms(Ftot, dy);
    const rootX = [{ ch: `root_x_${w}`, k: 1 }];
    members.push({ name: `shoulder rail ${side}`, kind: 'shaft', check: 'round', material: rm, dims: { d_mm: ra.od_mm, di_mm: ra.id_mm }, sections: [...ms.map((q) => sect(q.name, q.radial)), sect('T', Trail), sect('V aft', RA), sect('V fwd', RB)], uses: { M: ms.map((q) => q.name), T: 'T', V: ['V aft', 'V fwd'] }, note: `a ${ra.od_mm}/${ra.id_mm} mm tube on the two plates (x ${(xA * 1000).toFixed(1)} and ${(xF * 1000).toFixed(1)} mm), the shoulder housings at x ${(xa * 1000).toFixed(1)} (overhung) and ${(xf * 1000).toFixed(1)} mm, ${ra.dz_mm} mm outboard and ${ra.dy_mm} mm above the hinge (the bracket arm, ${(dy * 1000).toFixed(1)} mm, gives the torsion)` });
    for (const [pl, R] of [['aft', RA], ['forward', RB]]) {
      members.push({ name: `plate seat shoulder rail ${side} ${pl}`, kind: 'seat', check: 'lug', material: mat(L.plates.material), dims: { d_mm: ra.od_mm, D_mm: ra.seat_D_mm, t_mm: pl === 'aft' ? tAft + (ra.aft_boss_mm ?? 0) : tFwd }, sections: [sect('F', R)], uses: { F: 'F' } });
      railPlates[pl === 'aft' ? 'aft' : 'fwd'].push(R);
    }
    for (const [nm, Fb] of [['aft', Faft], ['forward', Ffwd]]) {
      members.push({ name: `shoulder bracket ${side} ${nm}, sleeve bond`, kind: 'bond', check: 'bond', material: glueAl, dims: { d_mm: ra.od_mm, L_mm: ra.sleeve_len_mm }, sections: [sect('T', scaleTerms(Fb, dy)), sect('F', rootX)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 aluminium to aluminium: the housing force times the arm as torque, the wing root axial force along the rail' });
      members.push({ name: `shoulder bracket ${side} ${nm}, arm`, kind: 'web', check: 'web', material: mat(ra.arm_material), dims: { b_mm: ra.arm_b_mm, h_mm: ra.arm_t_mm, lever_mm: +(dy * 1000).toFixed(3) }, sections: [sect('F', Fb)], uses: { F: 'F' }, note: 'the housing boss to the rail sleeve, the bushing force at the arm length' });
    }
    const lockG = circleGroup(ra.lock_screws, ra.lock_r_mm);
    // round 14: rail.lock_at 'forward' puts the lock flange at the forward plate's aft face (else the aft plate's front face)
    const lockFwd = ra.lock_at === 'forward';
    bolts(`shoulder rail ${side}, torsion lock screws`, lockG, [], Trail, [], rootX, `a flange on the rail at the ${lockFwd ? 'forward plate\'s aft' : ra.lock_at === 'aft outer' ? 'aft plate\'s aft' : 'aft plate\'s front'} face, screwed to it: the rail torque on ${ra.lock_screws} M3 screws at ${ra.lock_r_mm} mm, the root axial force as tension`);
    // round 16 (lock_at 'aft outer' on the aft seat boss, aft_boss_at 'aft face'): the screws bear in the plate and the boss
    const onBoss = ra.lock_at === 'aft outer' && ra.aft_boss_at === 'aft face';
    screwSeat(`shoulder rail ${side}, torsion lock screws in the plate`, perScrewShear(lockG, [], Trail), lockFwd ? tFwd : tAft + (onBoss ? ra.aft_boss_mm ?? 0 : 0));
    counts.screws += ra.lock_screws; counts.nuts += ra.lock_screws;
    const railLen = gap + plateT + (xA - xa) + ra.sleeve_len_mm / 2000 + 0.002;
    if (ra.aft_boss_mm) item(`shoulder rail ${side} aft seat boss`, mat(L.plates.material), PI4 * ((ra.seat_D_mm / 1000) ** 2 - (ra.od_mm / 1000) ** 2) * (ra.aft_boss_mm / 1000), `ring ${ra.seat_D_mm}/${ra.od_mm} x ${ra.aft_boss_mm} mm on the aft plate`);
    const flangeOD = lockFlangeOD(ra);
    const flangeA = ra.lock_arc_deg ? hullOf(lockFlangeCircles(ra, [0, 0], 1)).area_m2 - PI4 * (ra.od_mm / 1000) ** 2 : PI4 * (flangeOD ** 2 - ra.od_mm ** 2) * 1e-6;
    item(`shoulder rail ${side}`, rm, PI4 * ((ra.od_mm / 1000) ** 2 - (ra.id_mm / 1000) ** 2) * railLen + flangeA * (ra.lock_t_mm ?? 2) / 1000, `${ra.od_mm}/${ra.id_mm} mm tube x ${(railLen * 1000).toFixed(1)} mm plus a ${ra.lock_t_mm ?? 2} mm lock flange (${(flangeA * 1e6).toFixed(0)} mm2); ${rm.density_g_cm3} g/cm3`);
    const al6 = mat(ra.arm_material);
    const sleeveV = 2 * PI4 * ((ra.od_mm + 2 * ra.sleeve_wall_mm) ** 2 - ra.od_mm ** 2) * 1e-9 * ra.sleeve_len_mm;
    // round 16 (the solids, clearance.js shoulderParts): each arm a bar from the shoulder bushing's flange (or bore) to the
    // rail's surface, the two brackets of a side two items
    const shB = jointByName(`shoulder pivot ${side}`), shP = shB ? ax.catalog.find((q) => q.id === shB.part_id)?.specs : null;
    const armL = ax.d.params.layout?.rule === 'clearance' && shP ? dy * 1000 - (shP.d3_mm ?? shP.d2_mm) / 2 - ra.od_mm / 2 : dy * 1000;
    const armV = 2 * (ra.arm_b_mm * ra.arm_t_mm * armL) * 1e-9;
    item(`shoulder bracket ${side}`, al6, sleeveV + armV, `two sleeves ${ra.od_mm + 2 * ra.sleeve_wall_mm}/${ra.od_mm} x ${ra.sleeve_len_mm} mm and two arms ${ra.arm_b_mm} x ${ra.arm_t_mm} x ${armL.toFixed(1)} mm from the bushing to the rail (the housing rings are the shoulder housings' items); two brackets; ${al6.density_g_cm3} g/cm3`, { count: 2 });
    railPos[side] = [(lay.fb.O4[0] + ra.dz_mm / 1000) * sg, lay.fb.O4[1] + ra.dy_mm / 1000];
  }

  Object.assign(ax, { railPlates, railPos });
}

function af_boom(ax) {
  const { d, catalog, plateT, members, mat, item, span, A, glueCf, lump, counts, xA, xF, tFwd, tAft, screwSeat, bolts, flangePlate, perScrewShear } = ax;
  // ---- Boom over the gearbox: two bonded clamp blocks, screwed to the plates' outer faces --------------------------
  const bo = A.boom;
  const tube = byId(catalog, d.params.parts.keel);
  const tube_od = tube.specs.od_mm, tube_id = tube.specs.id_mm;
  // round 15 (params.layout.plate_faces 'actual'): the clamps on the plates' own faces, not the nominal 2 mm plate's
  const [hA, hF] = ax.d.params.layout?.plate_faces === 'actual' ? [tAft / 2000, tFwd / 2000] : [plateT / 2, plateT / 2];
  const xcA = xA - hA - bo.clamp_len_mm / 2000, xcF = xF + hF + bo.clamp_len_mm / 2000, sCl = xcF - xcA;
  const yB = bo.y_mm / 1000;
  const tail = d.tail;
  const Lt = xcA - tail.x_m;
  const tl = lump('tail surfaces'), kb = lump('keel and boom');
  const tailIn = inertialTerms(tl.m, tl.r), boomIn = inertialTerms(kb.m, [-(Lt / 2) + xcA, yB, 0]);
  const Vb = addTerms([{ ch: 'tail_F', k: 1 }], tailIn, boomIn);
  const Mb = addTerms(scaleTerms([{ ch: 'tail_F', k: 1 }], Lt), scaleTerms(tailIn, Lt), scaleTerms(boomIn, Lt / 2));
  const ht = 0.5 * tail.span_each_m + Math.abs((tail.y_m ?? 0) - yB);
  const Tb = scaleTerms([{ ch: 'tail_F', k: 1 }], ht);
  const Fa = addTerms(Vb, scaleTerms(Mb, 1 / sCl)), Ff = scaleTerms(Mb, 1 / sCl);
  const rw = materialOfComposite(catalog, tube);
  members.push({ name: 'tail boom at the aft clamp', kind: 'tube', check: 'cfrp tube', material: { id: tube.id, axial_MPa: rw.axial_MPa, shear_MPa: rw.shear_MPa }, dims: { d_mm: tube_od, di_mm: tube_id }, sections: [sect('M', Mb), sect('T', Tb), sect('V', Vb)], uses: { M: 'M', T: 'T', V: 'V' }, note: `the tail force and the tail and boom inertia at ${(Lt * 1000).toFixed(0)} mm (boom at half that); torsion from the tail force at ${(ht * 1000).toFixed(0)} mm (half the V-tail span plus its height off the boom)` });
  for (const [nm, F] of [['aft', Fa], ['forward', Ff]]) {
    members.push({ name: `tail boom in the ${nm} clamp, bearing`, kind: 'seat', check: 'seat', material: rw.transverse, dims: { d_mm: tube_od, t_mm: bo.clamp_len_mm, crush: true }, sections: [sect('F', F)], uses: { F: 'F' }, note: `the boom moment as a couple between clamps ${(sCl * 1000).toFixed(0)} mm apart, on Rock West's transverse compressive estimate (half)` });
    const g = rowGroup(bo.screws, bo.screw_pitch_mm);
    const Tcl = addTerms(Tb, scaleTerms(F, bo.screw_offset_mm / 1000));
    bolts(`tail boom ${nm} clamp screws`, { ...g, lever_mm: bo.screw_pitch_mm, n_far: 1 }, F, Tcl, scaleTerms(F, (bo.clamp_len_mm / 2 + plateT * 1000 / 2) / 1000), Vb, `${bo.screws} M3 through the plate: the clamp force in shear, the boom torque and the force at the screws' ${bo.screw_offset_mm} mm offset as in-plane torsion, the force at half the clamp length as prying, the boom's axial load as tension`);
    screwSeat(`tail boom ${nm} clamp screws in the plate`, perScrewShear(g, F, Tcl), nm === 'aft' ? tAft : tFwd);
    flangePlate(`${nm === 'aft' ? 'aft' : 'forward'} plate under the ${nm} boom clamp`, bo.screw_pitch_mm + 6, scaleTerms(F, (bo.clamp_len_mm / 2 + plateT * 1000 / 2) / 1000));
    counts.screws += bo.screws; counts.nuts += bo.screws;
  }
  members.push({ name: 'tail boom clamp bonds', kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: tube_od, L_mm: 2 * bo.clamp_len_mm }, sections: [sect('T', Tb), sect('F', Vb)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half) in both clamp bores: the boom torque and its axial load (bounded by the whole tail force and inertia)' });
  if (bo.to_tail_spar_mm != null) {
    // round 16 (SPEC completeness: the BOM's boom is its solid): the boom from the forward clamp's outer face to
    // to_tail_spar_mm aft of the tail's spar (on its root quarter chord), so the spar's root and the ties lie on it;
    // its lump at its middle
    const x1 = xF + hF + bo.clamp_len_mm / 1000, x0 = tail.x_m - bo.to_tail_spar_mm / 1000, Lb = x1 - x0;
    ax.cx.items.push({ name: 'keel and tail boom', material_id: tube.id, material: tube.name, mass_g: tube.mass_g * Lb, basis: `${tube.mass_g} g/m x ${Lb.toFixed(4)} m (the forward clamp's outer face to ${bo.to_tail_spar_mm} mm aft of the tail's spar)`, source_url: tube.source_url, lump: { pos: [+((x0 + x1) / 2).toFixed(5), yB, 0], gyration_m: +(Lb / Math.sqrt(12)).toFixed(5) }, x_m: [x0, x1] });
  }
  const al6c = mat(bo.clamp_material);
  const blk = (tube_od + 2 * bo.clamp_wall_mm) / 1000;
  const cf = clampFlange(bo, blk, yB);
  if (cf) item('tail boom clamp blocks, 2 off', al6c, 2 * (blk * blk - PI4 * (tube_od / 1000) ** 2 + (cf.z[1] - cf.z[0]) * (cf.y[1] - cf.y[0])) * (bo.clamp_len_mm / 1000), `each a ${(blk * 1000).toFixed(1)} mm square collar bored ${tube_od} mm and a ${((cf.z[1] - cf.z[0]) * 1000).toFixed(1)} x ${((cf.y[1] - cf.y[0]) * 1000).toFixed(1)} mm screw flange below it (its screws ${bo.screw_edge_mm} mm from its edges), x ${bo.clamp_len_mm} mm; ${al6c.density_g_cm3} g/cm3`, { count: 2 });
  else item('tail boom clamp blocks, 2 off', al6c, 2 * ((blk * (blk + 0.006)) - PI4 * (tube_od / 1000) ** 2) * (bo.clamp_len_mm / 1000), `${(blk * 1000).toFixed(0)} x ${(blk * 1000 + 6).toFixed(0)} mm blocks x ${bo.clamp_len_mm} mm, bored ${tube_od} mm (a 6 mm screw flange); ${al6c.density_g_cm3} g/cm3`);
  const boomRoot = { aft: Fa, fwd: Ff };

  Object.assign(ax, { sCl, yB, tail, Lt, tailIn, boomRoot });
}

function af_trays(ax) {
  if (ax.A.noseTray) return af_carriers(ax);
  const { plateT, members, mat, item, A, tieS, lump, counts, xA, xF, tFwd, tAft, screwSeat, bolts, flangePlate, perScrewShear, tail } = ax;
  // ---- Forward tray: battery, ESC, receiver and pitot ---------------------------------------------------------------
  const trayChecks = (tag, T, plateX, itemsOn, tiesFor) => {
    const tm = mat(T.material);
    const yT = T.y_mm / 1000;
    const P = [plateX, yT, 0];
    const parts = itemsOn.map(lump).filter(Boolean);
    const far = Math.max(...parts.map((q) => Math.abs(q.r[0] - plateX))) + 0.035;
    const V = addTerms(...parts.map((q) => inertialTerms(q.m, q.r)));
    const M = addTerms(...parts.map((q) => scaleTerms(inertialTerms(q.m, q.r), hyp(q.r[0] - P[0], q.r[1] - P[1], q.r[2]))));
    const cs = channelSection(T.w_mm, T.flange_h_mm, T.t_mm);
    members.push({ name: `${tag} at the root`, kind: 'beam', check: 'beam', material: tm, dims: { A_mm2: +cs.A_mm2.toFixed(3), Zi_mm3: +cs.Zi_mm3.toFixed(3), Zo_mm3: +cs.Zo_mm3.toFixed(3), section: `channel ${T.w_mm} x ${T.flange_h_mm} x ${T.t_mm} mm` }, sections: [sect('M', M), sect('Mo', M)], uses: { M: 'M', Mo: 'Mo' }, note: 'a bent-sheet channel: the carried parts\' inertia moment about the root, taken both vertically and laterally at full value' });
    const g = { ...rowGroup(T.screws, T.screw_pitch_mm), lever_mm: T.screw_lever_mm, n_far: 1 };
    const Ttor = scaleTerms(V, T.item_height_mm / 1000);
    bolts(`${tag} screws`, g, V, Ttor, M, V, `${T.screws} M3 through the tray flange and the plate: the root moment as prying about the flange edge (${T.screw_lever_mm} mm, one screw), the parts' force in shear and as tension, its offset of ${T.item_height_mm} mm above the tray as in-plane torsion`);
    screwSeat(`${tag} screws in the plate`, perScrewShear(g, V, Ttor), tag === 'forward tray' ? tFwd : tAft);
    flangePlate(`${tag === 'forward tray' ? 'forward' : 'aft'} plate under the ${tag} flange`, T.w_mm, M, T.doubler_mm);
    counts.screws += T.screws; counts.nuts += T.screws;
    for (const [nm, n, tie] of tiesFor) {
      const q = lump(nm);
      if (!q) continue;
      members.push({ name: `${tag}: ${nm} ties`, kind: 'tie', check: 'tie', tie: { id: tie.id, loop_N: tie.loop_N }, dims: { n }, sections: [sect('F', inertialTerms(q.m, q.r))], uses: { F: 'F' }, note: `${n} x ${tie.id} round the part and the tray` });
      if (tie === tieS) counts.tieS += n; else counts.tieL += n;
    }
    item(tag, tm, (cs.A_mm2 * 1e-6) * (far + T.flange_mm / 1000), `channel ${T.w_mm} x ${T.flange_h_mm} x ${T.t_mm} mm sheet (${cs.A_mm2.toFixed(1)} mm2) x ${((far + T.flange_mm / 1000) * 1000).toFixed(0)} mm (to ${((far) * 1000).toFixed(0)} mm from the plate, with a ${T.flange_mm} mm root flange); ${tm.density_g_cm3} g/cm3`);
    return { V, length_m: far, plateX, y: yT };
  };
  const fwdTray = trayChecks('forward tray', A.tray, xF + plateT / 2, ['battery', 'ESC', 'receiver and airspeed'],
    [['battery', A.tray.battery_ties, tieS], ['ESC', 1, tieS], ['receiver and airspeed', 1, tieS]]);
  const aftTray = trayChecks('aft tray', A.aftTray, xA - plateT / 2, ['tail servos', 'flight controller', 'GPS'],
    [['tail servos', A.aftTray.servo_ties, tieS], ['flight controller', 1, tieS], ['GPS', 1, tieS]]);

  Object.assign(ax, { fwdTray, aftTray });
}

/**
 * Round 15 (SPEC "Support and containment"): the carriers' geometry (m) from the airframe's parameters and the plate
 * faces, shared by the load checks here, the packing mounts (src/design/round15.mjs) and the assembly check
 * (src/design/clearance.js). A channel's base is centred on y (thickness t), its flanges' mid-planes at z +-w/2 rising
 * h from the base's centre line (as clearance.js has drawn trays since round 14).
 *  - forward tray: from the forward plate's outer face to the riser's forward face, at tray.y_mm, with its root tab
 *    (flange_mm) up the plate face;
 *  - nose tray riser: a bent sheet standing on the forward tray: its web across the pod at noseTray.riser_x_mm (the aft
 *    face), a flange forward along each vertical edge, a foot aft along the forward tray's base and a tab forward
 *    under the nose tray;
 *  - nose tray: from the riser's aft face forward to noseTray.tip_x_mm, at noseTray.y_mm, on the riser's tab;
 *  - aft tray: from the aft plate's outer face aft for aftTray.length_mm, at aftTray.y_mm, its root tab up the plate.
 */
export function carrierGeometry(A, planes, tF, tA, crank = null) {
  const m = (v) => v / 1000, xPf = planes.plateFwd + tF / 2, xPa = planes.plateAft - tA / 2; // the plates' own faces
  const ch = (T) => ({ y: m(T.y_mm), w: m(T.w_mm), h: m(T.flange_h_mm), t: m(T.t_mm), tab: m(T.flange_mm), material: T.material });
  const N = A.noseTray, R = N.riser, xr = m(N.riser_x_mm), tr = m(R.t_mm);
  // round 16 (tray.doubler_at 'outer'): the doubler between the tray's root tab and the forward plate's outer face
  const dbl = A.tray.doubler_at === 'outer' ? m(A.tray.doubler_mm ?? 0) : 0;
  const forward = { ...ch(A.tray), x: [xPf + dbl, xr + tr], root: 'fwd' };
  const nose = { ...ch(N), x: [xr, m(N.tip_x_mm)] };
  // round 16 (riser.actuator_hole_mm): a hole in the web round the crank axis for the encoder's actuator to pass
  const hole = R.actuator_hole && planes.encoder && crank ? { c: crank, r: planes.encoder.od.actuator / 2 + m(R.actuator_hole_mm ?? 0.5) } : null;
  const riser = { x: [xr, xr + tr], y: [forward.y + forward.t / 2, nose.y - nose.t / 2], w: m(R.w_mm), t: tr, f: m(R.flange_mm), foot: m(R.foot_mm), material: R.material ?? A.tray.material, ...(hole ? { hole } : {}) };
  const aft = { ...ch(A.aftTray), x: [xPa - m(A.aftTray.length_mm), xPa], root: 'aft' };
  // the doubler's plate: the tray's width, its height centred on the root tab's screws (tray.doubler_h_mm)
  const ys = forward.y + forward.t / 2 + forward.tab / 2, dh = m(A.tray.doubler_h_mm ?? 24);
  const doubler = dbl ? { x: [xPf, xPf + dbl], y: [ys - dh / 2, ys + dh / 2], w: forward.w } : null;
  return { forward, riser, nose, aft, ...(doubler ? { doubler } : {}) };
}

/** Volume (m3) of a channel of the carrier geometry over length L, with its root tab when it has one. */
const channelVolume = (c, L, tab = 0) => (c.w + 2 * c.h) * c.t * (L + tab);
/** Volume (m3) of the riser: web, two flanges, foot and tab. */
const riserVolume = (r) => { const h = r.y[1] - r.y[0]; return r.t * (r.w * h + 2 * r.f * (h - 2 * r.t) + 2 * r.w * r.foot - (r.hole ? Math.PI * r.hole.r ** 2 : 0)); };

/** A cable tie's catalogue geometry: bundle diameters and band width x thickness (m), from its dimensions text. */
export function tieGeometry(specs) {
  const t = String(specs?.dimensions ?? '');
  const band = t.match(/x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/), bundle = t.match(/bundle\s*([\d.]+)\s*-\s*([\d.]+)\s*mm/);
  return band && bundle ? { b_m: +band[1] / 1000, t_m: +band[2] / 1000, bundle_m: [+bundle[1] / 1000, +bundle[2] / 1000] } : null;
}

/**
 * The tie that holds a part on its carrier (round 15): through two slots in the carrier either side of the part, the
 * loop round the part's cross-section (w x h) and the carrier sheet (tb), 2 (w + h + tb); the smallest catalogue tie
 * whose published bundle range takes that loop (pi x bundle diameter). -> the tie with loop_m, or null.
 */
export function tieFor(ties, w, h, tb) {
  const loop = 2 * (w + h + tb);
  const ok = ties.filter((q) => q.geo && loop <= Math.PI * q.geo.bundle_m[1] + 1e-12 && loop >= Math.PI * q.geo.bundle_m[0]);
  ok.sort((a, b) => a.geo.bundle_m[1] - b.geo.bundle_m[1]);
  return ok.length ? { ...ok[0], loop_m: loop } : null;
}

// The mass lump each packed part belongs to (src/design/fit.js lumpPositionsFrom).
const LUMP_OF = { battery: 'battery', esc: 'ESC', fc: 'flight controller', gps: 'GPS', receiver: 'receiver and airspeed', airspeed: 'receiver and airspeed', servo_tail_L: 'tail servos', servo_tail_R: 'tail servos', regulator: 'power module and regulator', servo_stroke: 'stroke servo' };

/**
 * Round 15 carriers (SPEC "Support and containment"): the forward tray, the nose tray on its riser, the aft tray, the
 * crank encoder readhead on the riser, every mounted part's ties and the pitot's. Each carrier is checked at its root
 * like round 8's trays (a channel beam carrying the inertia moment of everything beyond it, both ways at full value;
 * its screws, their seats), now with the carriers' own masses among the loads.
 */
function af_carriers(ax) {
  const { d, catalog, planes, plateT, members, mat, item, A, tieS, tieL, lump, counts, tFwd, tAft, screwSeat, bolts, flangePlate, perScrewShear } = ax;
  const G = carrierGeometry(A, planes, tFwd / 1000, tAft / 1000, ax.crank), mf = 1 + (d.mass.margin_frac ?? 0.15);
  const at = (nm) => lump(nm);
  const pt = (m, r) => ({ m, r });
  // the carriers' own masses (kg, with the margin) at their centroids
  const own = (c, v, x, y) => { const kg = v * mat(c.material).density_g_cm3 * 1000; return { kg, load: pt(kg * mf, [x, y, 0]) }; };
  const F = G.forward, R = G.riser, N = G.nose, AT = G.aft;
  const mid = (x) => (x[0] + x[1]) / 2;
  const oF = own(F, channelVolume(F, F.x[1] - F.x[0], F.tab), mid(F.x), F.y), oR = own(R, riserVolume(R), mid(R.x), mid(R.y));
  const oN = own(N, channelVolume(N, N.x[1] - N.x[0]), mid(N.x), N.y), oA = own(AT, channelVolume(AT, AT.x[1] - AT.x[0], AT.tab), mid(AT.x), AT.y);
  const enc = at('crank encoder');
  const noseItems = [...['battery', 'ESC', 'receiver and airspeed', 'pitot-static probe'].map(at).filter(Boolean), oN.load];
  const riserItems = [...noseItems, oR.load, ...(enc ? [enc] : [])];
  const fwdItems = [...riserItems, oF.load];
  const aftItems = [...['tail servos', 'flight controller', 'GPS', 'power module and regulator', 'stroke servo'].map(at).filter((q) => q && q.m > 0), oA.load];
  const V = (items) => addTerms(...items.map((q) => inertialTerms(q.m, q.r)));
  const Mabout = (items, P) => addTerms(...items.map((q) => scaleTerms(inertialTerms(q.m, q.r), hyp(q.r[0] - P[0], q.r[1] - P[1], q.r[2]))));
  const rootChecks = (tag, T, c, P, items, seatT, plateName) => {
    const cs = channelSection(T.w_mm, T.flange_h_mm, T.t_mm), Vt = V(items), M = Mabout(items, P);
    members.push({ name: `${tag} at the root`, kind: 'beam', check: 'beam', material: mat(T.material), dims: { A_mm2: +cs.A_mm2.toFixed(3), Zi_mm3: +cs.Zi_mm3.toFixed(3), Zo_mm3: +cs.Zo_mm3.toFixed(3), section: `channel ${T.w_mm} x ${T.flange_h_mm} x ${T.t_mm} mm` }, sections: [sect('M', M), sect('Mo', M)], uses: { M: 'M', Mo: 'Mo' }, note: `a bent-sheet channel: the inertia moment of everything it carries (its own mass included) about its root (${(P[0] * 1000).toFixed(1)}, ${(P[1] * 1000).toFixed(1)}) mm, taken both vertically and laterally at full value` });
    const g = { ...rowGroup(T.screws, T.screw_pitch_mm), lever_mm: T.screw_lever_mm, n_far: 1 };
    const Ttor = scaleTerms(Vt, T.item_height_mm / 1000);
    bolts(`${tag} screws`, g, Vt, Ttor, M, Vt, `${T.screws} M3 with nuts through the ${tag}'s root and ${plateName}: the root moment as prying about the edge (${T.screw_lever_mm} mm, one screw), the carried force in shear and as tension, its offset of ${T.item_height_mm} mm as in-plane torsion`);
    screwSeat(`${tag} screws in ${plateName}`, perScrewShear(g, Vt, Ttor), seatT);
    counts.screws += T.screws; counts.nuts += T.screws;
    return { V: Vt, M, g };
  };
  const fr = rootChecks('forward tray', A.tray, F, [F.x[0], F.y, 0], fwdItems, tFwd, 'the forward plate');
  flangePlate('forward plate under the forward tray flange', A.tray.w_mm, fr.M, A.tray.doubler_mm);
  rootChecks('nose tray riser', { ...A.noseTray.riser, w_mm: A.noseTray.riser.w_mm, flange_h_mm: A.noseTray.riser.flange_mm, material: R.material }, R, [R.x[0], R.y[0], 0], riserItems, A.tray.t_mm, 'the forward tray\'s base');
  rootChecks('nose tray', A.noseTray, N, [R.x[1], N.y, 0], noseItems, A.noseTray.riser.t_mm, 'the riser\'s tab');
  const ar = rootChecks('aft tray', A.aftTray, AT, [AT.x[1], AT.y, 0], aftItems, tAft, 'the aft plate');
  flangePlate('aft plate under the aft tray flange', A.aftTray.w_mm, ar.M, A.aftTray.doubler_mm);
  item('forward tray', mat(F.material), channelVolume(F, F.x[1] - F.x[0], F.tab), `channel ${A.tray.w_mm} x ${A.tray.flange_h_mm} x ${A.tray.t_mm} mm x ${((F.x[1] - F.x[0]) * 1000).toFixed(1)} mm (plate face to the riser) with a ${A.tray.flange_mm} mm root tab; ${mat(F.material).density_g_cm3} g/cm3`);
  item('nose tray riser', mat(R.material), riserVolume(R), `${A.noseTray.riser.t_mm} mm sheet: web ${A.noseTray.riser.w_mm} x ${((R.y[1] - R.y[0]) * 1000).toFixed(1)} mm, two ${A.noseTray.riser.flange_mm} mm flanges, a ${A.noseTray.riser.foot_mm} mm foot and tab; ${mat(R.material).density_g_cm3} g/cm3`);
  item('nose tray', mat(N.material), channelVolume(N, N.x[1] - N.x[0]), `channel ${A.noseTray.w_mm} x ${A.noseTray.flange_h_mm} x ${A.noseTray.t_mm} mm x ${((N.x[1] - N.x[0]) * 1000).toFixed(1)} mm (riser to tip); ${mat(N.material).density_g_cm3} g/cm3`);
  item('aft tray', mat(AT.material), channelVolume(AT, AT.x[1] - AT.x[0], AT.tab), `channel ${A.aftTray.w_mm} x ${A.aftTray.flange_h_mm} x ${A.aftTray.t_mm} mm x ${((AT.x[1] - AT.x[0]) * 1000).toFixed(1)} mm with a ${A.aftTray.flange_mm} mm root tab; ${mat(AT.material).density_g_cm3} g/cm3`);
  // every mounted part's ties: the tie model from the loop round the part and its carrier's sheet
  const pIdx = partsIndex(d, catalog), specs = packedSpecs(d, pIdx).list;
  const ties = [tieS, tieL].map((q) => ({ ...q, geo: tieGeometry(byId(catalog, q.id).specs) }));
  const sheet = { 'forward tray': F.t, 'nose tray': N.t, 'aft tray': AT.t };
  const tieRows = [];
  for (const s of specs) {
    const mt = d.params.layout?.mounts?.[s.name];
    if (!mt) continue;
    const tb = sheet[mt.carrier];
    if (tb == null) throw new Error(`mount ${s.name}: unknown carrier ${mt.carrier}`);
    const tie = tieFor(ties, s.size[2], s.size[1], tb);
    if (!tie) throw new Error(`mount ${s.name}: no catalogue tie takes a ${(2 * (s.size[2] + s.size[1] + tb) * 1000).toFixed(1)} mm loop`);
    const n = mt.ties ?? 1, bomPart = d.bom.parts.find((p) => p.role === s.role), lp = at(LUMP_OF[s.name]);
    const q = pt((bomPart?.unit_mass_g ?? 0) / 1000 * mf, lp?.r ?? s.at);
    members.push({ name: `${mt.carrier}: ${s.name} ties`, kind: 'tie', check: 'tie', tie: { id: tie.id, loop_N: tie.loop_N }, dims: { n }, sections: [sect('F', inertialTerms(q.m, q.r))], uses: { F: 'F' }, note: `${n} x ${tie.id} through slots in the ${mt.carrier} either side of the part: a ${(tie.loop_m * 1000).toFixed(1)} mm loop (its bundle range takes ${(Math.PI * tie.geo.bundle_m[1] * 1000).toFixed(1)} mm)` });
    if (tie.id === tieS.id) counts.tieS += n; else counts.tieL += n;
    tieRows.push({ part: s.name, carrier: mt.carrier, n, tie: tie.id, loop_mm: +(tie.loop_m * 1000).toFixed(2), bundle_max_mm: +(Math.PI * tie.geo.bundle_m[1] * 1000).toFixed(2), band_mm: [tie.geo.b_m * 1000, tie.geo.t_m * 1000] });
  }
  const support = { ties: tieRows, readhead: readheadMount(ax, G, enc), pitot: null };
  ax.fwdTray = { V: fr.V, length_m: F.x[1] - F.x[0], plateX: F.x[0], y: F.y };
  ax.aftTray = { V: ar.V, length_m: AT.x[1] - AT.x[0], plateX: AT.x[1], y: AT.y };
  ax.carriers = { geometry: G, support, masses_g: { forward: oF.kg * 1000, riser: oR.kg * 1000, nose: oN.kg * 1000, aft: oA.kg * 1000 } };
}

/**
 * The pitot-static probe's mount (round 15): the probe along the nose tray's right flange, outside it, its bottom flush
 * with the tray's base, from pitot.x_aft_mm forward for its length; tied to the flange through a slot at the flange's
 * root. -> { c: [z, y], x: [x0, x1], r } (m), the probe's axis and outer radius.
 */
export function pitotMount(A, params, catalog) {
  const N = A.noseTray, P = A.pitot ?? {};
  const r = byId(catalog, 'mtube-tigertitanium-ti3al2.5v-6.35x5.54').specs.od_mm / 2000;
  const s = P.side === 'left' ? -1 : 1;
  const z = s * (N.w_mm / 2 + N.t_mm / 2) / 1000 + s * r, y = (N.y_mm - N.t_mm / 2) / 1000 + r;
  const x0 = P.x_aft_mm / 1000, L = (params.pitotLength_mm ?? 100) / 1000;
  return { c: [z, y], x: [x0, x0 + L], r };
}

/**
 * The probe's ties (round 15): round the probe and the flange, through a slot at the flange's root; the loop is the
 * flange's sheet and the probe's diameter across, the flange's height and the base's sheet up and down.
 */
function pitotTies(ax, pit, rho) {
  const { d, catalog, A, members, counts, tieS, tieL } = ax;
  const N = carrierGeometry(A, ax.planes, ax.tFwd / 1000, ax.tAft / 1000, ax.crank).nose, pm = pitotMount(A, d.params, catalog);
  const ties = [tieS, tieL].map((q) => ({ ...q, geo: tieGeometry(byId(catalog, q.id).specs) }));
  const tie = tieFor(ties, N.t + 2 * pm.r, N.h + N.t, 0);
  if (!tie) throw new Error('pitot: no catalogue tie takes its loop');
  const n = A.pitot.ties ?? 2, side = A.pitot.side ?? 'right';
  members.push({ name: 'pitot-static probe ties', kind: 'tie', check: 'tie', tie: { id: tie.id, loop_N: tie.loop_N }, dims: { n, extra_N: +(0.5 * rho * A.pod.V_drag_m_s ** 2 * Math.PI * (2 * pm.r) ** 2 / 4 * 1.2).toFixed(4) }, sections: [sect('F', inertialTerms(pit.m, pit.r))], uses: { F: 'F' }, note: `${n} x ${tie.id} round the probe and the nose tray's ${side} flange, through a slot at the flange's root (a ${(tie.loop_m * 1000).toFixed(1)} mm loop): its inertia and the drag of its ${(2 * pm.r * 1000).toFixed(2)} mm tip at ${A.pod.V_drag_m_s} m/s (Cd 1.2)` });
  if (tie.id === tieS.id) counts.tieS += n; else counts.tieL += n;
  ax.carriers.support.pitot = { part: 'pitot-static probe', carrier: 'nose tray', face: `${side} flange, outer face`, c_m: pm.c.map((v) => +v.toFixed(6)), x_m: pm.x.map((v) => +v.toFixed(6)), r_m: pm.r, n, tie: tie.id, loop_mm: +(tie.loop_m * 1000).toFixed(2), bundle_max_mm: +(Math.PI * tie.geo.bundle_m[1] * 1000).toFixed(2), band_mm: [tie.geo.b_m * 1000, tie.geo.t_m * 1000], tie_x_mm: A.pitot.tie_x_mm };
}

/**
 * The crank encoder readhead on the riser (round 15): the maker's fixing, 3 x M2 DIN 912 through its mounting holes on
 * the 34 mm circle (BRD01; sensor-rls-orbis-br10 specs.mounting) with spacers of under 4 mm diameter from its front
 * face to the riser's aft face and nuts on the riser's forward face; holes at encoder.holes_deg (from +z, about the
 * crank axis) so every hole's land lies on the web below the nose tray. The readhead's inertia (the whole encoder lump,
 * conservatively) in shear, prying at the spacers' length.
 */
/** The readhead's holes on its mounting circle about the crank axis, and its spacers' length to the riser's web (m). */
export function readheadGeometry(E, mounting, crank, readheadX, riserX0) {
  const pcd = (parseFloat(String(mounting).match(/on a ([\d.]+) mm circle/)?.[1]) || 34) / 1000;
  const holes = (E.holes_deg ?? [30, 150, 270]).map((a) => [crank[0] + (pcd / 2) * Math.cos((a * Math.PI) / 180), crank[1] + (pcd / 2) * Math.sin((a * Math.PI) / 180)]);
  return { pcd, holes, spacer: riserX0 - readheadX[1] };
}

function readheadMount(ax, G, enc) {
  const { catalog, A, members, mat, item, counts, planes, crank } = ax;
  const E = A.encoder ?? {};
  if (!enc || !planes.encoder || !E.screw) return null;
  const scr = fastenerOf(catalog, E.screw), nut = byId(catalog, E.nut), sp = byId(catalog, E.screw).specs;
  const rh = byId(catalog, ax.d.params.parts?.crankEncoder ?? 'sensor-rls-orbis-br10').specs;
  const { pcd, holes, spacer } = readheadGeometry(E, rh.mounting, crank, planes.encoder.readhead, G.riser.x[0]);
  const g = { ...circleGroup(holes.length, (pcd / 2) * 1000), lever_mm: spacer * 1000 + (planes.encoder.readhead[1] - planes.encoder.readhead[0]) * 500, n_far: 1 };
  const Vr = inertialTerms(enc.m, enc.r);
  members.push({ name: 'crank encoder readhead screws', kind: 'fasteners', check: 'bolts', fastener: { id: scr.id, tension_N: scr.tension_N, shear_N: scr.shear_N }, dims: g, sections: [sect('V', Vr), sect('T', []), sect('M', scaleTerms(Vr, g.lever_mm / 1000)), sect('F', Vr)], uses: { V: 'V', T: 'T', M: 'M', F: 'F' }, note: `${holes.length} x ${scr.id} through the readhead's mounting holes (${(pcd * 1000).toFixed(0)} mm circle), ${(spacer * 1000).toFixed(1)} mm spacers and the riser, nuts forward: the encoder's inertia in shear and as tension, prying at the spacers' length plus half the readhead` });
  members.push({ name: 'crank encoder readhead screws in the riser', kind: 'seat', check: 'seat', material: mat(G.riser.material), dims: { d_mm: 2, t_mm: A.noseTray.riser.t_mm, e_over_d: 2 }, sections: [sect('F', scaleTerms(Vr, 1 / holes.length))], uses: { F: 'F' }, note: 'an M2 screw bearing in the riser\'s sheet, edge distance at least 2 d' });
  const so = E.spacer_od_mm / 1000, si = E.spacer_id_mm / 1000;
  item(`crank encoder spacers, ${holes.length} off`, mat(G.riser.material), holes.length * Math.PI / 4 * (so * so - si * si) * spacer, `${E.spacer_od_mm} / ${E.spacer_id_mm} mm 6061-T6 tube x ${(spacer * 1000).toFixed(1)} mm (the maker's spacer: under 4 mm diameter, over 6 mm long)`);
  counts.m2 = (counts.m2 ?? 0) + holes.length;
  return { carrier: 'nose tray riser', face: 'web, aft face', screw: scr.id, nut: nut.id, n: holes.length, pcd_mm: pcd * 1000, holes_m: holes.map((h) => h.map((v) => +v.toFixed(5))), spacer_mm: +(spacer * 1000).toFixed(3), spacer_od_mm: E.spacer_od_mm, spacer_id_mm: E.spacer_id_mm, screw_head: { d_mm: sp.head_d_mm, k_mm: sp.head_k_mm }, nut_s_mm: nut.specs.s_mm, nut_m_mm: nut.specs.m_mm };
}

/**
 * Round 15 (support): the motor's own fixing. Its maker's drawing (Scorpion SII-3008 full dimension, read 2026-11-03;
 * out/downloads/scorpion-sii-3008) puts the 4 x M3 threads on the motor's back face (the stator side, 25 mm across), with
 * the cross mount; its front is the rotating bell. With the bell toward the forward plate and the shaft forward through
 * it, the motor hangs on a bracket at its back: a 6061-T6 plate the hull of the four screw lands and two sleeves bonded
 * on the shoulder rails, bored for the shaft's back stub. -> geometry (m): { x, holes, land_r, outline, sleeves, stub }.
 */
export function motorBracketGeometry(B, planes, c, railPos, railR) {
  const m = (v) => v / 1000, r = m(B.circle_d_mm) / 2, land = m(B.land_r_mm), so = railR + m(B.sleeve_wall_mm);
  const holes = B.angles_deg.map((a) => [c[0] + r * Math.cos((a * Math.PI) / 180), c[1] + r * Math.sin((a * Math.PI) / 180)]);
  const sleeves = ['right', 'left'].map((side) => ({ side, c: railPos[side], ro: so, ri: railR }));
  const h = hullOf([...holes.map((q) => ({ c: q, r: land })), ...sleeves.map((q) => ({ c: q.c, r: q.ro }))]);
  return { x: [planes.motorX[0] - m(B.t_mm), planes.motorX[0]], holes, land_r: land, outline: h.poly, area_m2: h.area_m2, sleeves, stub: { c, r: m(B.stub_hole_d_mm) / 2 } };
}

/** The motor's loads (round 8's terms): its inertia, the first mesh force and the motor torque (crank torque / G / eta). */
function motorLoads(ax) {
  const { d, lay, lump } = ax;
  const mot = lump('flap motor');
  return { mot, F0: [{ ch: 'crank_torque', k: lay.meshPerNm(0) }], Tm: [{ ch: 'crank_torque', k: 1 / (d.drive.gearRatio * (d.drive.gearEfficiency ?? 0.85)) }], motorIn: inertialTerms(mot.m, mot.r) };
}

/**
 * Round 15: the force the motor's bracket puts on a rail: the motor's inertia and the first mesh force shared by the
 * lever rule along the line between the rails (the larger share, on both), plus its torque and its prying moment each
 * in full as a couple over the rails' spacing; at the bracket's mid-plane. -> { F, x, Vm, Mm, pinOff, L, share } or
 * null without a bracket.
 */
function motorBracketLoad(ax) {
  const { A, planes, lay } = ax;
  const B = A.motor?.bracket;
  if (!B || planes.stage1 !== 'overhung') return null;
  const { F0, Tm, motorIn } = motorLoads(ax);
  const ra = A.rail, sg = [1, -1].map((k) => [(lay.fb.O4[0] + ra.dz_mm / 1000) * k, lay.fb.O4[1] + ra.dy_mm / 1000]);
  const L = Math.hypot(sg[0][0] - sg[1][0], sg[0][1] - sg[1][1]);
  const c = lay.train.shafts[lay.train.shafts.length - 1], u = [(sg[1][0] - sg[0][0]) / L, (sg[1][1] - sg[0][1]) / L];
  const t = ((c[0] - sg[0][0]) * u[0] + (c[1] - sg[0][1]) * u[1]) / L; // the motor's place along the line, 0 at the right rail
  const share = Math.max(Math.min(1, Math.max(0, 1 - t)), Math.min(1, Math.max(0, t)));
  const x1 = planes.motorX[0], pinOff = planes.stage[0] - x1;
  const Vm = addTerms(motorIn, F0), Mm = addTerms(scaleTerms(motorIn, (planes.motorX[1] - planes.motorX[0]) / 2), scaleTerms(F0, pinOff));
  return { F: addTerms(scaleTerms(Vm, share), scaleTerms(Tm, 1 / L), scaleTerms(Mm, 1 / L)), x: x1 - B.t_mm / 2000, Vm, Mm, pinOff, L, share };
}

function af_motor(ax) {
  const { lay, planes, plateT, A, lump, counts, xF, tFwd, screwSeat, bolts, flangePlate, perScrewShear, mat, item, members, glueAl, railPos } = ax;
  // ---- Motor on the forward plate ------------------------------------------------------------------------------------
  const mo = A.motor, mot = lump('flap motor');
  const s0 = lay.train.stages[0];
  const { F0, Tm, motorIn } = motorLoads(ax);
  if (mo.bracket && planes.stage1 === 'overhung') {
    // round 15: the motor on its back-face bracket, the bracket bonded on both rails
    const B = mo.bracket, c = lay.train.shafts[lay.train.shafts.length - 1], railR = A.rail.od_mm / 2000;
    const g = motorBracketGeometry(B, planes, c, railPos, railR);
    const bm = mat(B.material), t = B.t_mm;
    const mg = { ...circleGroup(B.angles_deg.length, B.circle_d_mm / 2), lever_mm: B.circle_d_mm / 2 * Math.SQRT2, n_far: 2 };
    const { Vm, Mm, pinOff, L, F: Frail } = motorBracketLoad(ax); // the first mesh force's offset is from the bracket face
    bolts('motor mount screws', mg, Vm, Tm, Mm, motorIn, `${B.angles_deg.length} M3 through the bracket into the motor's back-face threads (${B.circle_d_mm} mm across, the maker's drawing): the motor's inertia and the first mesh force (${s0.pinion.id}) in shear, the motor torque as torsion, the inertia at half the motor's length and the mesh force at its ${(pinOff * 1000).toFixed(1)} mm offset as prying`);
    screwSeat('motor mount screws in the bracket', perScrewShear(mg, Vm, Tm), t);
    const arm = Math.max(...g.sleeves.map((q) => Math.hypot(q.c[0] - c[0], q.c[1] - c[1])));
    members.push({ name: 'motor bracket arm', kind: 'plate', check: 'plate', material: bm, dims: { b_mm: 2 * B.sleeve_wall_mm + A.rail.od_mm, t_mm: t }, sections: [sect('F', Frail), sect('M', scaleTerms(Frail, arm))], uses: { F: 'F', M: 'M' }, note: `the bracket from the motor to its farther rail (${(arm * 1000).toFixed(1)} mm), the sleeve's width: the larger lever-rule share of the motor's force, its torque and prying moment as a couple over the rails' ${(L * 1000).toFixed(0)} mm spacing, bent out of the plane` });
    members.push({ name: 'motor bracket sleeve bond', kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: +(Math.PI * A.rail.od_mm).toFixed(3), L_mm: t }, sections: [sect('F', Frail)], uses: { F: 'F' }, note: `Araldite 2011 aluminium to aluminium, a sleeve round the ${A.rail.od_mm} mm rail over the bracket's ${t} mm, carrying a rail's share` });
    item('motor bracket', bm, (g.area_m2 - g.sleeves.reduce((a, q) => a + Math.PI * q.ri * q.ri, 0) - Math.PI * g.stub.r ** 2) * t / 1000, `${t} mm 6061-T6 plate, the hull of the motor's four screw lands and the two rail sleeves (${(g.area_m2 * 1e6).toFixed(0)} mm2 less the rails and the shaft stub), bonded on the rails`);
    counts.screws += B.angles_deg.length;
    ax.motorBracket = { ...g, t_m: t / 1000, joints: 'screwed to the motor\'s back, bonded on both rails' };
  } else {
    const mg = { ...circleGroup(mo.screws, mo.circle_r_mm), lever_mm: mo.circle_r_mm * Math.SQRT2, n_far: 2 };
    // the pinion's offset from the plate face the motor is screwed to (round 14: the aft face, the pinion overhung forward)
    const pinOff = Math.abs((planes.stage1 === 'overhung' ? xF - plateT / 2 : xF + plateT / 2) - planes.stage[0]);
    bolts('motor mount screws', mg, addTerms(motorIn, F0), Tm, addTerms(scaleTerms(motorIn, mo.overhang_mm / 1000), scaleTerms(F0, pinOff)), motorIn, `${mo.screws} M3 in the motor's own threads on a ${2 * mo.circle_r_mm} mm circle: the motor's inertia and the first mesh force (${s0.pinion.id}) in shear, the motor torque (crank torque / G / efficiency) as torsion, both forces at their offsets as prying`);
    screwSeat('motor mount screws in the plate', perScrewShear(mg, addTerms(motorIn, F0), Tm), tFwd);
    flangePlate('forward plate under the motor', 2 * mo.circle_r_mm + 12, addTerms(scaleTerms(motorIn, mo.overhang_mm / 1000), scaleTerms(F0, pinOff)), mo.doubler_mm);
    counts.screws += mo.screws;
  }

  Object.assign(ax, { mo, mot, F0, Tm, motorIn });
}

function af_encoder(ax) {
  const { members, A, tieS, lump, counts, crank } = ax;
  // ---- Crank encoder on the aft plate --------------------------------------------------------------------------------
  const enc = lump('crank encoder');
  if (enc && !ax.carriers?.support.readhead) {
    const front = ax.planes.encoder && ax.planes.encoder.readhead[0] > ax.planes.plateFwd; // round 14: on the crank's forward stub
    members.push({ name: 'crank encoder ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.encoder.ties }, sections: [sect('F', inertialTerms(enc.m, enc.r))], uses: { F: 'F' }, note: `${A.encoder.ties} x ${tieS.id}: the readhead on its bracket to ${front ? 'the forward plate\'s outer face, round the crank stub' : 'the aft plate\'s crank boss'}` });
    counts.tieS += A.encoder.ties;
  }

}

/**
 * Round 16 (pod.formers_at_mm): the pod's formers, each a 2 mm balsa band round the inside of the loft at its station
 * (its outside on the covering, touching it, at the smaller of the loft's sections across its thickness; band_mm wide),
 * with a tab rising from the band against the outer face of the left flange of the tray above it, to the flange's top:
 * one T18R round the flange and the tab, through a slot at the flange's root (as the pitot is tied).
 * -> [{ x_m: [x0, x1], band_m, section: { y, hw, hh }, tab: { z, y }, tray, flange: { z, y } }] (m).
 */
export function formerGeometry(loft, P, carriers, loftHalfAt) {
  const t = (P.former_t_mm ?? 2) / 1000, band = (P.former_band_mm ?? 5) / 1000, tw = (P.former_tab_mm ?? 2) / 1000;
  return P.formers_at_mm.map((xmm) => {
    const x = xmm / 1000, a = loftHalfAt(loft, x - t / 2), b = loftHalfAt(loft, x + t / 2);
    const h = a.hw * a.hh <= b.hw * b.hh ? a : b; // the smaller section across the former's thickness
    const tray = [['nose tray', carriers.nose], ['aft tray', carriers.aft], ['forward tray', carriers.forward]].find(([, c]) => x >= c.x[0] && x <= c.x[1]);
    if (!tray) throw new Error(`former at x ${xmm} mm: no tray above it to tie it to`);
    const c = tray[1], zf = -(c.w / 2 + c.t / 2), z0 = zf - tw, ri = (z) => h.y - (h.hh - band / 2) * Math.sqrt(Math.max(0, 1 - (z / (h.hw - band / 2)) ** 2));
    // the tab from the band's middle (at its outer edge, the lower of the two) up to the flange's top
    return { x_m: [x - t / 2, x + t / 2], band_m: band, section: h, tab: { z: [z0, zf], y: [Math.max(ri(z0), ri(zf)), c.y + c.h + c.t / 2] }, tray: tray[0], flange: { z: [zf - c.t / 2 * 0, -(c.w / 2 - c.t / 2)], y: [c.y - c.t / 2, c.y + c.h + c.t / 2] } };
  });
}

/** A former's band as n convex pieces between its outside (the loft's ellipse) and its inside (band in from it). */
export function formerPieces(f, n = 48) {
  const { y, hw, hh } = f.section, w = f.band_m, P = (a, k) => [k * hw * Math.cos(a), y + k * hh * Math.sin(a)], Q = (a) => [(hw - w) * Math.cos(a), y + (hh - w) * Math.sin(a)];
  return Array.from({ length: n }, (_, i) => { const a0 = (i / n) * 2 * Math.PI, a1 = ((i + 1) / n) * 2 * Math.PI; return [P(a0, 1), P(a1, 1), Q(a1), Q(a0)]; });
}

function af_ties(ax) {
  const rho = airDensity(ax.d);
  // ---- Pod formers, the pitot and the tail surfaces on ties ------------------------------------------------------
  podTies(ax, rho);
  probeTies(ax, rho);
  tailTies(ax);
}

const airDensity = (d) => d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0;

/** The pitot's ties: round 15's along the nose tray's flange when the design mounts it, else round 8's. */
function probeTies(ax, rho) {
  const pit = ax.lump('pitot-static probe');
  if (!pit) return;
  if (ax.carriers && ax.A.pitot?.x_aft_mm != null) pitotTies(ax, pit, rho);
  else legacyPitotTies(ax, pit, rho);
}

function podTies(ax, rho) {
  const { d, members, A, tieS, lump, counts } = ax;
  if (A.pod.formers_at_mm && ax.carriers) {
    // round 16: the formers where they stand (formerGeometry), their line from their solids' volume
    const fs = formerGeometry(d.body.loft, A.pod, ax.carriers.geometry, ax.loftHalfAt);
    const polyA = (p) => Math.abs(p.reduce((s, q, i) => s + q[0] * p[(i + 1) % p.length][1] - p[(i + 1) % p.length][0] * q[1], 0)) / 2;
    const area = fs.reduce((s, f) => s + formerPieces(f).reduce((a, p) => a + polyA(p), 0) + (f.tab.z[1] - f.tab.z[0]) * (f.tab.y[1] - f.tab.y[0]), 0);
    const bal = ax.catalog.find((q) => q.id === (d.params.parts?.ribMaterial ?? 'balsa-wood-database'));
    const t = (A.pod.former_t_mm ?? 2) / 1000, rhoB = bal.specs.density_kg_m3;
    ax.cx.items.push({ name: 'fuselage formers', material_id: bal.id, material: bal.name, mass_g: area * t * rhoB * 1000, basis: `${fs.length} off, ${A.pod.former_band_mm ?? 5} mm bands round the loft at x ${A.pod.formers_at_mm.join(', ')} mm with a tab to the tray above (${(area * 1e6).toFixed(0)} mm2 in all) x ${t * 1000} mm, ${rhoB} kg/m3`, source_url: bal.source_url, count: fs.length });
    // the stations and sizes the assembly check builds them from on the design's own loft (clearance.js formerParts)
    ax.formers = { formers_at_mm: A.pod.formers_at_mm, band_mm: A.pod.former_band_mm ?? 5, tab_mm: A.pod.former_tab_mm ?? 2, t_mm: A.pod.former_t_mm ?? 2, trays: fs.map((f) => f.tray) };
  }
  const pod = lump('fuselage shell and formers');
  const drag = 0.5 * rho * A.pod.V_drag_m_s ** 2 * (d.body?.CdA_front_m2 ?? 0.003);
  const nPod = A.pod.formers * A.pod.ties_each;
  members.push({ name: 'pod formers ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: nPod, extra_N: +drag.toFixed(4) }, sections: [sect('F', inertialTerms(pod.m, pod.r))], uses: { F: 'F' }, note: `${nPod} x ${tieS.id} (balsa formers to the boom and trays: no published bond strength on balsa); the pod's inertia plus its drag at ${A.pod.V_drag_m_s} m/s (${drag.toFixed(2)} N)` });
  counts.tieS += nPod;
}

/** Round 8 to 14: the probe tied to the battery and the forward tray's nose, 2 T18R. */
function legacyPitotTies(ax, pit, rho) {
  const { members, A, tieS, counts } = ax;
  members.push({ name: 'pitot-static probe ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.pitot.ties, extra_N: +(0.5 * rho * A.pod.V_drag_m_s ** 2 * Math.PI * 0.00635 ** 2 / 4 * 1.2).toFixed(4) }, sections: [sect('F', inertialTerms(pit.m, pit.r))], uses: { F: 'F' } , note: `${A.pitot.ties} x ${tieS.id} to the battery and the forward tray's nose: its inertia and the drag of its 6.35 mm tip at ${A.pod.V_drag_m_s} m/s (Cd 1.2)` }); counts.tieS += A.pitot.ties;
}

function tailTies(ax) {
  const { members, A, tieS, counts, tailIn } = ax;
  members.push({ name: 'tail surfaces ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.tail.ties }, sections: [sect('F', addTerms([{ ch: 'tail_F', k: 1 }], tailIn))], uses: { F: 'F' }, note: `${A.tail.ties} x ${tieS.id}: the tail's aerodynamic force and inertia` });
  counts.tieS += A.tail.ties;
}

/** The circles a plate's outline is the hull of: every shaft seat, the motor's (forward), both rail seats, the panel lands. */
function plateSeatCircles(ax, plateName) {
  const { catalog, jointByName, A, shafts, railPos, mo } = ax;
  const ringR = (jt) => { const p = byId(catalog, jt.part_id).specs; const D = Number.isFinite(p.d2_mm) ? p.d2_mm : +((p.dimensions_mm_d_D_B ?? '').split('x')[1] ?? 10); return (D / 2 + A.plates.edge_mm) / 1000; };
  const out = [];
  const named = { 'crank shaft': 0, 'gear shaft 1': 1, 'gear shaft 2': 2, 'gear shaft 3': 3 };
  for (const [nm, i] of Object.entries(named)) { const jt = jointByName(nm); if (jt && shafts[i]) out.push({ c: shafts[i], r: ringR(jt) }); }
  if (plateName === 'fwd') { const jt = jointByName('motor shaft outboard'); if (jt) out.push({ c: shafts[shafts.length - 1], r: Math.max(ringR(jt), (mo.circle_r_mm + 5) / 1000) }); }
  for (const side of ['right', 'left']) out.push({ c: railPos[side], r: A.rail.seat_D_mm / 2000 });
  // round 16 (plates.screw_edge_mm): the plate reaches that far round each of the trays' root screws it carries
  const G = ax.carriers?.geometry, e = (A.plates.screw_edge_mm ?? 0) / 1000;
  if (G && e) {
    const T = plateName === 'fwd' ? A.tray : A.aftTray, c = plateName === 'fwd' ? G.forward : G.aft, n = T.screws, s = T.screw_pitch_mm / 1000;
    for (let i = 0; i < n; i++) out.push({ c: [(i - (n - 1) / 2) * s, c.y + c.t / 2 + c.tab / 2], r: e });
  }
  // Flange lands for the panels (the flange's own width past the panel line).
  for (const q of A.panels.list) {
    const ends = q.kind === 'side' ? [[q.z_mm, q.y0_mm], [q.z_mm, q.y0_mm + q.h_mm]] : [[-q.w_mm / 2, q.y_mm], [q.w_mm / 2, q.y_mm]];
    for (const e of ends) out.push({ c: [e[0] / 1000, e[1] / 1000], r: 0.002 });
  }
  return out;
}

/**
 * Round 16: the area (m2) of the holes a plate is cut with, as the assembly draws them (clearance.js plateParts): every
 * shaft seat at its journal's outside diameter (the motor's outboard bearing in the forward plate), both rails, and
 * the boom's clearance hole (the running clearance round the tube) in the plate's pad.
 */
function plateHoleArea(ax, pl) {
  const { catalog, jointByName, A, d } = ax;
  const D = (jt) => { const p = byId(catalog, jt.part_id).specs; return (Number.isFinite(p.d2_mm) ? p.d2_mm : +(p.dimensions_mm_d_D_B ?? '').split('x')[1]) / 1000; };
  const seats = ['crank shaft', 'gear shaft 1', 'gear shaft 2', 'gear shaft 3', ...(pl === 'fwd' ? ['motor shaft outboard'] : [])].map(jointByName).filter(Boolean);
  const disc = (dia) => (Math.PI / 4) * dia * dia;
  const boomR = byId(catalog, d.params.parts.keel).specs.od_mm / 2000 + (d.params.layout?.running_mm ?? 0.5) / 1000;
  return seats.reduce((a, jt) => a + disc(D(jt)), 0) + 2 * disc(A.rail.od_mm / 1000) + Math.PI * boomR * boomR;
}

/**
 * Round 16 (rail.lock_edge_mm): the plate the rails' torsion lock screws bear in reaches lock_edge_mm round each screw.
 * A land whose edge circle lies outside the plate's hull gets an ear: the hull of the rail's seat circle and that edge
 * circle, as its own convex piece (the plate's outline stays the hull of its seats, which the ears would otherwise
 * stretch over parts beside the plate). Each ear's area outside the hull is the plate's extra material.
 * -> [{ outline_m, area_m2, rail, land }].
 */
function lockEars(ax, pl, hull) {
  const { A, railPos } = ax, lockPlate = A.rail.lock_at === 'forward' ? 'fwd' : 'aft', er = (A.rail.lock_edge_mm ?? 0) / 1000;
  if (!er || !A.rail.lock_arc_deg || pl !== lockPlate) return [];
  const out = [];
  // one ear per rail: the hull of its seat circle and the edge circles of its lands outside the plate's hull
  for (const [side, sg] of [['right', 1], ['left', -1]]) {
    const outsiders = lockFlangeCircles(A.rail, railPos[side], sg).slice(1).filter((q) => !Array.from({ length: 36 }, (_, i) => [q.c[0] + er * Math.cos((i * Math.PI) / 18), q.c[1] + er * Math.sin((i * Math.PI) / 18)]).every((p) => insideConvex(hull, p)));
    if (!outsiders.length) continue;
    const e = hullOf([{ c: railPos[side], r: A.rail.seat_D_mm / 2000 }, ...outsiders.map((q) => ({ c: q.c, r: er }))]);
    out.push({ rail: side, lands: outsiders.length, outline_m: e.poly.map(([z, y]) => [+z.toFixed(5), +y.toFixed(5)]), area_m2: e.area_m2 - convexArea(clipConvex(e.poly, hull)) });
  }
  return out;
}
const cross2 = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
const signedArea = (P) => P.reduce((s, p, i) => s + p[0] * P[(i + 1) % P.length][1] - P[(i + 1) % P.length][0] * p[1], 0) / 2;
const convexArea = (P) => (P.length < 3 ? 0 : Math.abs(signedArea(P)));
/** Whether p lies in the convex polygon P (either winding). */
export function insideConvex(P, p) {
  const s = Math.sign(signedArea(P));
  return P.every((a, i) => s * cross2(a, P[(i + 1) % P.length], p) >= -1e-12);
}
/** The convex polygon P clipped to the convex polygon C (Sutherland-Hodgman). */
export function clipConvex(P, C) {
  const s = Math.sign(signedArea(C));
  let out = P;
  for (let i = 0; i < C.length && out.length; i++) {
    const a = C[i], b = C[(i + 1) % C.length], inp = out;
    out = [];
    const side = (p) => s * cross2(a, b, p);
    for (let k = 0; k < inp.length; k++) {
      const p = inp[k], q = inp[(k + 1) % inp.length], sp = side(p), sq = side(q);
      if (sp >= 0) out.push(p);
      if ((sp >= 0) !== (sq >= 0)) { const t = sp / (sp - sq); out.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]); }
    }
  }
  return out;
}

/**
 * Round 16: the panels' end flanges (panels.list[].flanges, rectangles [z, y] in mm) on each plate's inner face:
 * -> [{ part, end ('aft' | 'fwd'), x: [x0, x1] (m), t (m), rects: [{ z, y }] (m) }].
 */
export function panelFlanges(A, planes, tA, tF) {
  const out = [];
  for (const q of A.panels.list) {
    if (!q.flanges) continue;
    const t = q.t_mm / 1000, rects = q.flanges.map((r) => ({ z: r.z.map((v) => v / 1000), y: r.y.map((v) => v / 1000) }));
    const a = planes.plateAft + tA / 2, f = planes.plateFwd - tF / 2;
    out.push({ part: `gearbox ${q.name}`, end: 'aft', x: [a, a + t], t, rects }, { part: `gearbox ${q.name}`, end: 'fwd', x: [f - t, f], t, rects });
  }
  return out;
}

/**
 * Round 16 (boom.screw_edge_mm): a boom clamp's screw flange below its collar, wide and deep enough that each of its
 * screws (a row at screw_pitch_mm, screw_offset_mm below the boom's axis) has screw_edge_mm of metal round it.
 * -> { z: [z0, z1], y: [y0, y1] } (m), or null.
 */
export function clampFlange(bo, blk, yB) {
  if (!bo.screw_edge_mm) return null;
  const e = bo.screw_edge_mm / 1000, zs = bo.screw_pitch_mm / 2000 * (bo.screws - 1), ys = yB - bo.screw_offset_mm / 1000;
  return { z: [-(zs + e), zs + e], y: [ys - e, yB - blk / 2] };
}

/** The outline's top (up) or bottom edge y at z, of a convex polygon. */
function edgeAt(poly, z, up) {
  let best = up ? -Infinity : Infinity;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i], q = poly[(i + 1) % poly.length];
    if ((p[0] - z) * (q[0] - z) <= 0 && p[0] !== q[0]) { const y = p[1] + ((z - p[0]) / (q[0] - p[0])) * (q[1] - p[1]); best = up ? Math.max(best, y) : Math.min(best, y); }
  }
  return best;
}

/** A pad outside the plate's hull hangs on a post of post_w_mm: in-plane bending at the hull edge plus the axial force. */
function platePosts(ax, pl, poly, pmat) {
  const { members, A, tFwd, tAft, yB, boomRoot, fwdTray, aftTray } = ax;
  const posts = [];
  const pads = [['boom clamp', [0, yB], boomRoot[pl], A.boom.screw_pitch_mm + 6], [pl === 'fwd' ? 'forward tray' : 'aft tray', [0, (pl === 'fwd' ? A.tray.y_mm : A.aftTray.y_mm) / 1000], pl === 'fwd' ? fwdTray.V : aftTray.V, (pl === 'fwd' ? A.tray : A.aftTray).w_mm]];
  for (const [nm, c, Fp, padW] of pads) {
    const up = c[1] > 0;
    const edge = edgeAt(poly, c[0], up);
    const Lp = Math.max(0, up ? c[1] - edge : edge - c[1]);
    if (!(Lp > 0)) continue;
    const pw = A.plates.post_w_mm;
    members.push({ name: `${pl === 'aft' ? 'aft' : 'forward'} plate post to the ${nm}`, kind: 'plate', check: 'plate', material: pmat, dims: { b_mm: pw, t_mm: pl === 'aft' ? tAft : tFwd }, sections: [sect('F', Fp), sect('M', scaleTerms(Fp, Lp))], uses: { F: 'F', M: 'M' }, note: `a ${pw} mm post ${(Lp * 1000).toFixed(1)} mm from the plate's outline to the pad: the pad force axially and at the post length` });
    // the post and its pad as [z, y] boxes (m), for the assembly clearance check (src/design/clearance.js)
    const r5 = (v) => +v.toFixed(5), hw = pw / 2000, pad = A.plates.pad_mm / 1000;
    // round 16 (clearance rule): the post runs from the outline to the pad's near edge, so post and pad do not overlap
    // (their solids' volume is the plate's material once); the post's bending lever stays the pad's centre
    const near = ax.d.params.layout?.rule === 'clearance' ? (up ? c[1] - pad : c[1] + pad) : c[1], Lq = Math.abs(near - edge);
    // round 16 (boom.screw_edge_mm): the boom clamp's pad reaches round the clamp's screws as the clamp's flange does
    const cf = nm === 'boom clamp' ? clampFlange(A.boom, (ax.catalog.find((q) => q.id === ax.d.params.parts.keel).specs.od_mm + 2 * A.boom.clamp_wall_mm) / 1000, yB) : null;
    const padBox = cf ? [cf.z[0], cf.z[1], Math.min(cf.y[0], c[1] - pad), c[1] + pad] : [c[0] - padW / 2000, c[0] + padW / 2000, c[1] - pad, c[1] + pad];
    const nearC = cf && ax.d.params.layout?.rule === 'clearance' ? padBox[up ? 2 : 3] : near, Lc = Math.abs(nearC - edge);
    posts.push({ to: nm, length_m: Lp, area_m2: pw / 1000 * Lc + (padBox[1] - padBox[0]) * (padBox[3] - padBox[2]), post_m: [r5(c[0] - hw), r5(c[0] + hw), r5(Math.min(edge, nearC)), r5(Math.max(edge, nearC))], pad_m: padBox.map(r5) });
  }
  return posts;
}

function af_plateSections(ax) {
  const { L, members, mat, A, tFwd, tAft, railPlates, railPos, boomRoot } = ax;
  // ---- Plates: solid outlines, sections through the plate below the rail seats -------------------------------------
  const pmat = mat(L.plates.material);
  const plates = {};
  for (const pl of ['aft', 'fwd']) {
    const h = hullOf(plateSeatCircles(ax, pl));
    const yCut = Math.min(railPos.right[1], railPos.left[1]) - A.rail.seat_D_mm / 2000;
    const b = hullWidthAt(h.poly, yCut);
    const zR = Math.abs(railPos.right[0]);
    const F = addTerms(...railPlates[pl], boomRoot[pl]);
    const M = addTerms(...railPlates[pl].map((R) => scaleTerms(R, zR)));
    members.push({ name: `${pl === 'aft' ? 'aft' : 'forward'} plate below the rail seats`, kind: 'plate', check: 'plate', material: pmat, dims: { b_mm: +(b * 1000).toFixed(2), t_mm: pl === 'aft' ? tAft : tFwd }, sections: [sect('F', F), sect('M', M)], uses: { F: 'F', M: 'M' }, note: `the section ${(yCut * 1000).toFixed(1)} mm up, ${(b * 1000).toFixed(1)} mm wide: both rails' reactions and the boom clamp force, the rails' at their ${(zR * 1000).toFixed(0)} mm offset as in-plane bending` });
    const posts = platePosts(ax, pl, h.poly, pmat);
    // outline_m: the hull's corners [z, y] (m), for the assembly clearance check (src/design/clearance.js)
    // round 16 (completeness: the BOM weighs the plate the solids draw): the holes the plate is cut with, every seat at
    // its journal's outside diameter, both rails, the boom's clearance hole in the pad
    const holes = ax.d.params.layout?.rule === 'clearance' ? plateHoleArea(ax, pl) : 0;
    const ears = lockEars(ax, pl, h.poly);
    plates[pl] = { area_m2: h.area_m2 + posts.reduce((a, q) => a + q.area_m2, 0) + ears.reduce((a, e) => a + e.area_m2, 0) - holes, holes_area_m2: holes, hull_area_m2: h.area_m2, posts, ...(ears.length ? { ears } : {}), cut_y_m: yCut, cut_width_m: b, outline_m: h.poly.map(([z, y]) => [+z.toFixed(5), +y.toFixed(5)]) };
  }

  Object.assign(ax, { plates });
}

function af_panels(ax) {
  const { lay, jointByName, planes, members, mat, item, A, glueAl, xA, gap, shafts, crank, railPlates, railPos, yB, boomRoot, fwdTray, mot, F0, Tm, motorIn, tFwd, tAft } = ax;
  // ---- Box: two side panels and a floor panel carry the forward plate's net force and torsion to the aft plate -------
  const pn = A.panels, pm = mat(pn.material);
  const crankFwd = jointByName('crank shaft').supports.find((s) => s.name === 'forward').radial.filter((t) => /^coupler_/.test(t.ch)).map((t) => ({ ch: t.ch, k: t.k }));
  const fwdMounted = addTerms(motorIn, fwdTray.V);
  const Vfwd = addTerms(crankFwd, F0, ...railPlates.fwd, boomRoot.fwd, fwdMounted);
  // Torsion about the crank axis: each mesh's share at the forward plate times its centre distance, the motor torque,
  // and every forward-plate force at its lever about the crank axis.
  const meshT = [];
  for (let s = 0; s < lay.n; s++) {
    const f = (planes.stage[s] - xA) / gap;
    const a = hyp(shafts[lay.n - 1 - s][0] - shafts[lay.n - s][0], shafts[lay.n - 1 - s][1] - shafts[lay.n - s][1]);
    meshT.push({ ch: 'crank_torque', k: lay.meshPerNm(s) * (s === 0 ? 1 : f) * a });
  }
  const lev = (p) => hyp(p[0] - crank[0], p[1] - crank[1]);
  const Tbox = addTerms(meshT, Tm, ...railPlates.fwd.map((R, i) => scaleTerms(R, lev(railPos[i ? 'left' : 'right']))), scaleTerms(boomRoot.fwd, lev([0, yB])), scaleTerms(motorIn, lev([mot.r[2], mot.r[1]])), scaleTerms(fwdTray.V, lev([0, A.tray.y_mm / 1000])), scaleTerms(F0, lev(shafts[shafts.length - 1])));
  // Panels (pn.list): 'side' panels in the x-y plane at z, from y0 for h (vertical shear: half the net force plus the
  // torsion over the two sides' spacing), and a 'top' panel in the x-z plane at y, centred, w wide (the whole net force).
  const sides = pn.list.filter((q) => q.kind === 'side');
  const W = Math.abs(sides[0].z_mm - sides[1].z_mm) / 1000;
  const Vside = addTerms(scaleTerms(Vfwd, 0.5), scaleTerms(Tbox, 1 / W));
  let panelV = 0;
  for (const q of pn.list) {
    const V = q.kind === 'side' ? Vside : Vfwd;
    const h = q.kind === 'side' ? q.h_mm : q.w_mm;
    members.push({ name: `gearbox ${q.name}`, kind: 'panel', check: 'panel', material: pm, dims: { h_mm: h, b_mm: +(gap * 1000).toFixed(2), t_mm: q.t_mm }, sections: [sect('V', V)], uses: { V: 'V' }, note: q.kind === 'side' ? `x-y plane at z ${q.z_mm} mm, y ${q.y0_mm} to ${q.y0_mm + q.h_mm} mm: half the forward plate's net force (every force on it summed as a magnitude) plus its torsion about the crank axis over the sides' ${(W * 1000).toFixed(0)} mm spacing` : `x-z plane at y ${q.y_mm} mm, ${q.w_mm} mm wide: the forward plate's whole net force` });
    // round 16 (q.flanges): each end's flange the stated rectangles on the plate's inner face, the sheet between the inner
    // faces (as the assembly check draws them); the bond over the flange's own area
    const fa = q.flanges ? q.flanges.reduce((a, r) => a + (r.z[1] - r.z[0]) * (r.y[1] - r.y[0]), 0) : null;
    members.push({ name: `gearbox ${q.name} flange bond`, kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: h, L_mm: fa ? +(fa / h).toFixed(3) : pn.flange_mm }, sections: [sect('F', V)], uses: { F: 'F' }, note: fa ? `Araldite 2011 aluminium to aluminium, one end flange of ${fa.toFixed(0)} mm2 on the plate's inner face (each end has one), as a ${h} mm wide lap of the same area` : 'Araldite 2011 aluminium to aluminium, one end flange (each end has one)' });
    const inner = (planes.plateFwd - tFwd / 2000) - (planes.plateAft + tAft / 2000);
    panelV += fa ? (h * 1e-3 * inner + 2 * fa * 1e-6) * (q.t_mm / 1000) : h * 1e-3 * (gap + 2 * pn.flange_mm / 1000) * (q.t_mm / 1000);
  }
  const anyF = pn.list.some((q) => q.flanges);
  item(`gearbox panels, ${pn.list.length} off`, pm, panelV, anyF ? `${pn.list.map((q) => `${q.name} ${q.kind === 'side' ? q.h_mm : q.w_mm} x ${q.t_mm} mm, its flanges ${q.flanges.reduce((a, r) => a + (r.z[1] - r.z[0]) * (r.y[1] - r.y[0]), 0).toFixed(0)} mm2 each end`).join('; ')}; each between the plates' inner faces (${(((planes.plateFwd - tFwd / 2000) - (planes.plateAft + tAft / 2000)) * 1000).toFixed(1)} mm); ${pm.density_g_cm3} g/cm3` : `${pn.list.map((q) => `${q.name} ${q.kind === 'side' ? q.h_mm : q.w_mm} x ${q.t_mm} mm`).join(', ')}, each ${(gap * 1000).toFixed(1)} mm plus two ${pn.flange_mm} mm flanges; ${pm.density_g_cm3} g/cm3`, { count: pn.list.length });

}

function af_bomItems(ax) {
  const { catalog, A, nut, tieS, tieL, counts, report, railPos, sCl, yB, Lt, fwdTray, aftTray, plates } = ax;
  // ---- Fasteners and ties in the BOM ----------------------------------------------------------------------------------
  const sp = byId(catalog, A.fastener);
  if (A.screws) {
    // round 16: every counted screw and nut at a stated place (fixings.js), the BOM by screw length from that layout
    const layout = fixingLayout(ax), fc = fixingCounts(layout);
    const nS = [...fc.screws.values()].reduce((a, b) => a + b, 0), nN = [...fc.nuts.values()].reduce((a, b) => a + b, 0);
    if (nS !== counts.screws || nN !== counts.nuts) throw new Error(`fixing layout: ${nS} screws and ${nN} nuts placed, the bolt checks count ${counts.screws} and ${counts.nuts}`);
    for (const [id, n] of fc.screws) { const f = byId(catalog, id); ax.cx.items.push({ name: `${id === A.fastener ? 'M3 screws' : f.name.replace(/^ISO 4762 /, 'ISO 4762 ')}, ${n} off`, material_id: id, material: f.name, mass_g: n * f.mass_g, basis: `${n} x ${f.mass_g} g (${f.specs.mass_basis})`, source_url: f.source_url, count: n }); }
    for (const [id, n] of fc.nuts) { const f = byId(catalog, id); ax.cx.items.push({ name: `M3 nuts, ${n} off`, material_id: id, material: f.name, mass_g: n * f.mass_g, basis: `${n} x ${f.mass_g} g (${f.specs.mass_basis})`, source_url: f.source_url, count: n }); }
    ax.fixings = layout.map(({ g, ng, ...b }) => b);
  } else {
    ax.cx.items.push({ name: `M3 screws, ${counts.screws} off`, material_id: sp.id, material: sp.name, mass_g: counts.screws * sp.mass_g, basis: `${counts.screws} x ${sp.mass_g} g (${sp.specs.mass_basis})`, source_url: sp.source_url });
    if (counts.nuts) ax.cx.items.push({ name: `M3 nuts, ${counts.nuts} off`, material_id: nut.id, material: nut.name, mass_g: counts.nuts * nut.mass_g, basis: `${counts.nuts} x ${nut.mass_g} g (${nut.specs.mass_basis})`, source_url: nut.source_url });
  }
  if (counts.m2) {
    // round 15: the crank encoder readhead's own fixing (3 x M2 DIN 912 with nuts)
    for (const id of [A.encoder.screw, A.encoder.nut]) { const f = byId(catalog, id); ax.cx.items.push({ name: `${f.name}, ${counts.m2} off`, material_id: f.id, material: f.name, mass_g: counts.m2 * f.mass_g, basis: `${counts.m2} x ${f.mass_g} g (${f.specs.mass_basis})`, source_url: f.source_url }); }
  }
  for (const [t, n] of [[tieS, counts.tieS], [tieL, counts.tieL]]) if (n) ax.cx.items.push({ name: `cable ties ${t.id}, ${n} off`, material_id: t.id, material: t.name, mass_g: n * t.mass_g, basis: `${n} x ${t.mass_g} g (maker's weight per unit)`, source_url: t.source_url });

  report.parts = { rails: railPos, boom_y_m: yB, clamp_spacing_m: sCl, tail_lever_m: Lt, trays: { forward: { y_m: fwdTray.y, length_m: fwdTray.length_m }, aft: { y_m: aftTray.y, length_m: aftTray.length_m } }, plates, counts };
  // round 15: every carrier's geometry (m) and every part's fixing, for the assembly check and the film
  if (ax.carriers) Object.assign(report.parts, { carriers: roundDeep(ax.carriers.geometry), support: ax.carriers.support, carrier_masses_g: roundDeep(ax.carriers.masses_g, 3) });
  if (ax.motorBracket) report.parts.motor_bracket = roundDeep(ax.motorBracket);
  if (ax.fixings) report.parts.fixings = roundDeep(ax.fixings);
  if (ax.formers) report.parts.formers = ax.formers;
  report.clearances_mm = clearances(ax.cx, railPos, A);
}


/** A Rock West pultruded tube as the boom checks need it: axial half the published compressive, its own page's in-plane shear and transverse estimates at half. */
function materialOfComposite(catalog, t) {
  const pn = t.specs?.part_number;
  const tr = pn ? catalog.find((p) => p.id === `composite-rockwest-${pn.toLowerCase()}-transverse`) : null;
  if (!tr || !Number.isFinite(t.specs?.compressive_strength_mpa)) throw new Error(`${t.id}: no published transverse and compressive strength for the boom checks`);
  return { axial_MPa: t.specs.compressive_strength_mpa / 2, shear_MPa: tr.specs.design_values.xy_shear_MPa, transverse: { id: tr.id, yield_MPa: tr.specs.design_values.yield_MPa } };
}

/** Radial clearance (mm) of each rail to every gear's tip circle and to the rocker hub (negative: they overlap). */
function clearances(cx, railPos, A) {
  const { lay, catalog, L } = cx;
  const out = {};
  const sh = lay.train.shafts, n = lay.n;
  for (const side of ['right', 'left']) {
    const p = railPos[side], r = A.rail.od_mm / 2000;
    const row = {};
    for (let s = 0; s < n; s++) {
      const st = lay.train.stages[s];
      for (const [g, idx] of [[st.wheel, n - 1 - s], [st.pinion, n - s]]) {
        const tip = (byId(catalog, g.id).specs.outside_dia_mm ?? (byId(catalog, g.id).specs.teeth + 2)) / 2000;
        row[`${g.id} (stage ${s})`] = +((hyp(p[0] - sh[idx][0], p[1] - sh[idx][1]) - tip - r) * 1000).toFixed(1);
      }
    }
    const hub = (L.rocker?.hub_od_mm ?? 11) / 2000;
    const o4 = [lay.fb.O4[0] * (side === 'right' ? 1 : -1), lay.fb.O4[1]];
    row['rocker hub'] = +((hyp(p[0] - o4[0], p[1] - o4[1]) - hub - r) * 1000).toFixed(1);
    out[side] = row;
  }
  // Side panels (x-y plane at z = +/- side_z, y from side_y to side_y + side_h) and the floor (x-z plane at floor_y):
  // the nearest approach of every gear tip circle (any plane between the plates).
  const pn = A.panels;
  const panels = {};
  for (const [nm, seg] of pn.list.map((q) => [q.name, q.kind === 'side' ? [[q.z_mm, q.y0_mm], [q.z_mm, q.y0_mm + q.h_mm]] : [[-q.w_mm / 2, q.y_mm], [q.w_mm / 2, q.y_mm]]])) {
    let worst = Infinity, what = null;
    for (let s = 0; s < n; s++) {
      const st = lay.train.stages[s];
      for (const [g, idx] of [[st.wheel, n - 1 - s], [st.pinion, n - s]]) {
        const tip = (byId(catalog, g.id).specs.outside_dia_mm ?? (byId(catalog, g.id).specs.teeth + 2)) / 2;
        const c = [sh[idx][0] * 1000, sh[idx][1] * 1000];
        const [a, b] = seg;
        const t = Math.max(0, Math.min(1, ((c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1]) * (b[1] - a[1])) / ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)));
        const dd = hyp(c[0] - (a[0] + t * (b[0] - a[0])), c[1] - (a[1] + t * (b[1] - a[1]))) - tip;
        if (dd < worst) { worst = dd; what = `${g.id} (stage ${s})`; }
      }
    }
    for (const side of ['right', 'left']) {
      const c = railPos[side].map((v) => v * 1000), [a, b] = seg;
      const t = Math.max(0, Math.min(1, ((c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1]) * (b[1] - a[1])) / ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)));
      const dd = hyp(c[0] - (a[0] + t * (b[0] - a[0])), c[1] - (a[1] + t * (b[1] - a[1]))) - A.rail.od_mm / 2;
      if (dd < worst) { worst = dd; what = `shoulder rail ${side}`; }
    }
    panels[nm] = { clearance_mm: +worst.toFixed(1), nearest: what };
  }
  out.panels = panels;
  return out;
}

/**
 * The rail lock flange's outer diameter (mm). v21 took the rail plus 8 mm, too small for screws on lock_r_mm; round 14
 * (rail.lock_at set) takes the screw circle plus an M3 head and its edge each side (lock_r + 3.5 mm in radius).
 */
export function lockFlangeOD(ra) {
  return ra.lock_at ? 2 * (ra.lock_r_mm + 3.5) : ra.od_mm + 8;
}

/**
 * Round 14 (rail.lock_arc_deg [a0, a1]): the lock flange as the hull of the rail (plus 2 mm) and its screws' lands, M3
 * heads with their edge (3.5 mm radius), on an arc of lock_r_mm; angles from +z counter-clockwise, sg -1 mirrors them.
 */
export function lockFlangeCircles(ra, c, sg) {
  const [a0, a1] = ra.lock_arc_deg.map((a) => (a * Math.PI) / 180), n = ra.lock_screws, r = ra.lock_r_mm / 1000;
  const out = [{ c, r: ra.od_mm / 2000 + 0.002 }];
  // round 16: rail.lock_angles_deg states each land's angle (the lands need not be evenly spread)
  const angles = ra.lock_angles_deg ? ra.lock_angles_deg.map((a) => (a * Math.PI) / 180) : Array.from({ length: n }, (_, i) => (n > 1 ? a0 + ((a1 - a0) * i) / (n - 1) : a0));
  for (const a of angles) out.push({ c: [c[0] + sg * r * Math.cos(a), c[1] + r * Math.sin(a)], r: 0.0035 });
  return out;
}

/** A copy with every number rounded to `k` decimals (default 6: micrometres in metres). */
function roundDeep(o, k = 6) {
  if (typeof o === 'number') return +o.toFixed(k);
  if (Array.isArray(o)) return o.map((v) => roundDeep(v, k));
  if (o && typeof o === 'object') return Object.fromEntries(Object.entries(o).map(([a, v]) => [a, roundDeep(v, k)]));
  return o;
}

export function mergeDefaults(def, over) {
  const o = { ...def };
  for (const [k, v] of Object.entries(over)) o[k] = v && typeof v === 'object' && !Array.isArray(v) && def[k] && typeof def[k] === 'object' ? { ...def[k], ...v } : v;
  return o;
}
