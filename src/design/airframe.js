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
    if (doubler_mm > 0) item(`${name.replace(/^(forward|aft) plate under the /, '')} doubler`, mat(AL6), (w_mm / 1000) * (2 * (/motor/.test(name) ? A.motor.circle_r_mm : 12) / 1000) * (doubler_mm / 1000), `${w_mm} mm x ${doubler_mm} mm 6061-T6 under the flange screws`);
  };
  const perScrewShear = (group, V, T) => addTerms(scaleTerms(V, 1 / group.n), group.sum_r2_mm2 > 0 ? scaleTerms(T, (group.r_max_mm / 1000) / (group.sum_r2_mm2 * 1e-6)) : []);

  const ax = { cx, d, catalog, L, lay, jointByName, planes, plateT, members, mat, item, span, A, scr, nut, tieS, tieL, glueAl, glueCf, lump, counts, xA, xF, gap, x0, shafts, crank, report, tFwd, tAft, screwSeat, bolts, flangePlate, perScrewShear };
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
    const loads = [...Faft.map((t) => ({ ch: t.ch, perUnit: t.k, x: xa })), ...Ffwd.map((t) => ({ ch: t.ch, perUnit: t.k, x: xf }))];
    const ms = shaftMomentSections(xA, xF, loads, 'M');
    const RA = addTerms(scaleTerms(Faft, Math.abs(xF - xa) / gap), scaleTerms(Ffwd, Math.abs(xF - xf) / gap));
    const RB = addTerms(scaleTerms(Faft, Math.abs(xa - xA) / gap), scaleTerms(Ffwd, Math.abs(xf - xA) / gap));
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
    bolts(`shoulder rail ${side}, torsion lock screws`, lockG, [], Trail, [], rootX, `a flange on the rail at the aft plate's front face, screwed to it: the rail torque on ${ra.lock_screws} M3 screws at ${ra.lock_r_mm} mm, the root axial force as tension`);
    screwSeat(`shoulder rail ${side}, torsion lock screws in the plate`, perScrewShear(lockG, [], Trail), tAft);
    counts.screws += ra.lock_screws; counts.nuts += ra.lock_screws;
    const railLen = gap + plateT + (xA - xa) + ra.sleeve_len_mm / 2000 + 0.002;
    if (ra.aft_boss_mm) item(`shoulder rail ${side} aft seat boss`, mat(L.plates.material), PI4 * ((ra.seat_D_mm / 1000) ** 2 - (ra.od_mm / 1000) ** 2) * (ra.aft_boss_mm / 1000), `ring ${ra.seat_D_mm}/${ra.od_mm} x ${ra.aft_boss_mm} mm on the aft plate`);
    item(`shoulder rail ${side}`, rm, PI4 * ((ra.od_mm / 1000) ** 2 - (ra.id_mm / 1000) ** 2) * railLen + PI4 * ((ra.od_mm + 8) ** 2 - ra.od_mm ** 2) * 1e-6 * 0.002, `${ra.od_mm}/${ra.id_mm} mm tube x ${(railLen * 1000).toFixed(1)} mm plus a 2 mm lock flange; ${rm.density_g_cm3} g/cm3`);
    const al6 = mat(ra.arm_material);
    const sleeveV = 2 * PI4 * ((ra.od_mm + 2 * ra.sleeve_wall_mm) ** 2 - ra.od_mm ** 2) * 1e-9 * ra.sleeve_len_mm;
    const armV = 2 * (ra.arm_b_mm * ra.arm_t_mm * dy * 1000) * 1e-9;
    item(`shoulder bracket ${side}`, al6, sleeveV + armV, `two sleeves ${ra.od_mm + 2 * ra.sleeve_wall_mm}/${ra.od_mm} x ${ra.sleeve_len_mm} mm and two arms ${ra.arm_b_mm} x ${ra.arm_t_mm} x ${(dy * 1000).toFixed(1)} mm to the housings (the housing rings are the shoulder housings' items); ${al6.density_g_cm3} g/cm3`);
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
  const xcA = xA - plateT / 2 - bo.clamp_len_mm / 2000, xcF = xF + plateT / 2 + bo.clamp_len_mm / 2000, sCl = xcF - xcA;
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
    members.push({ name: `tail boom in the ${nm} clamp, bearing`, kind: 'seat', check: 'seat', material: rw.transverse, dims: { d_mm: tube_od, t_mm: bo.clamp_len_mm }, sections: [sect('F', F)], uses: { F: 'F' }, note: `the boom moment as a couple between clamps ${(sCl * 1000).toFixed(0)} mm apart, on Rock West's transverse compressive estimate (half)` });
    const g = rowGroup(bo.screws, bo.screw_pitch_mm);
    const Tcl = addTerms(Tb, scaleTerms(F, bo.screw_offset_mm / 1000));
    bolts(`tail boom ${nm} clamp screws`, { ...g, lever_mm: bo.screw_pitch_mm, n_far: 1 }, F, Tcl, scaleTerms(F, (bo.clamp_len_mm / 2 + plateT * 1000 / 2) / 1000), Vb, `${bo.screws} M3 through the plate: the clamp force in shear, the boom torque and the force at the screws' ${bo.screw_offset_mm} mm offset as in-plane torsion, the force at half the clamp length as prying, the boom's axial load as tension`);
    screwSeat(`tail boom ${nm} clamp screws in the plate`, perScrewShear(g, F, Tcl), nm === 'aft' ? tAft : tFwd);
    flangePlate(`${nm === 'aft' ? 'aft' : 'forward'} plate under the ${nm} boom clamp`, bo.screw_pitch_mm + 6, scaleTerms(F, (bo.clamp_len_mm / 2 + plateT * 1000 / 2) / 1000));
    counts.screws += bo.screws; counts.nuts += bo.screws;
  }
  members.push({ name: 'tail boom clamp bonds', kind: 'bond', check: 'bond', material: glueCf, dims: { d_mm: tube_od, L_mm: 2 * bo.clamp_len_mm }, sections: [sect('T', Tb), sect('F', Vb)], uses: { T: 'T', F: 'F' }, note: 'Araldite 2011 on CFRP (typical, half) in both clamp bores: the boom torque and its axial load (bounded by the whole tail force and inertia)' });
  const al6c = mat(bo.clamp_material);
  const blk = (tube_od + 2 * bo.clamp_wall_mm) / 1000;
  item('tail boom clamp blocks, 2 off', al6c, 2 * ((blk * (blk + 0.006)) - PI4 * (tube_od / 1000) ** 2) * (bo.clamp_len_mm / 1000), `${(blk * 1000).toFixed(0)} x ${(blk * 1000 + 6).toFixed(0)} mm blocks x ${bo.clamp_len_mm} mm, bored ${tube_od} mm (a 6 mm screw flange); ${al6c.density_g_cm3} g/cm3`);
  const boomRoot = { aft: Fa, fwd: Ff };

  Object.assign(ax, { sCl, yB, tail, Lt, tailIn, boomRoot });
}

function af_trays(ax) {
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

function af_motor(ax) {
  const { d, lay, planes, plateT, A, lump, counts, xF, crank, tFwd, screwSeat, bolts, flangePlate, perScrewShear } = ax;
  // ---- Motor on the forward plate ------------------------------------------------------------------------------------
  const mo = A.motor, mot = lump('flap motor');
  const s0 = lay.train.stages[0];
  const F0 = [{ ch: 'crank_torque', k: lay.meshPerNm(0) }];
  const G = d.drive.gearRatio;
  const Tm = [{ ch: 'crank_torque', k: 1 / (G * (d.drive.gearEfficiency ?? 0.85)) }];
  const motorIn = inertialTerms(mot.m, mot.r);
  const mg = { ...circleGroup(mo.screws, mo.circle_r_mm), lever_mm: mo.circle_r_mm * Math.SQRT2, n_far: 2 };
  const pinOff = Math.abs(xF + plateT / 2 - planes.stage[0]);
  bolts('motor mount screws', mg, addTerms(motorIn, F0), Tm, addTerms(scaleTerms(motorIn, mo.overhang_mm / 1000), scaleTerms(F0, pinOff)), motorIn, `${mo.screws} M3 in the motor's own threads on a ${2 * mo.circle_r_mm} mm circle: the motor's inertia and the first mesh force (${s0.pinion.id}) in shear, the motor torque (crank torque / G / efficiency) as torsion, both forces at their offsets as prying`);
  screwSeat('motor mount screws in the plate', perScrewShear(mg, addTerms(motorIn, F0), Tm), tFwd);
  flangePlate('forward plate under the motor', 2 * mo.circle_r_mm + 12, addTerms(scaleTerms(motorIn, mo.overhang_mm / 1000), scaleTerms(F0, pinOff)), mo.doubler_mm);
  counts.screws += mo.screws;

  Object.assign(ax, { mo, mot, F0, Tm, motorIn });
}

function af_encoder(ax) {
  const { members, A, tieS, lump, counts, crank } = ax;
  // ---- Crank encoder on the aft plate --------------------------------------------------------------------------------
  const enc = lump('crank encoder');
  if (enc) {
    members.push({ name: 'crank encoder ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.encoder.ties }, sections: [sect('F', inertialTerms(enc.m, enc.r))], uses: { F: 'F' }, note: `${A.encoder.ties} x ${tieS.id}: the readhead on its bracket to the aft plate's crank boss` });
    counts.tieS += A.encoder.ties;
  }

}

function af_ties(ax) {
  const { d, members, A, tieS, lump, counts, tail, tailIn } = ax;
  // ---- Pod formers and tail surfaces on ties ---------------------------------------------------------------------
  const pod = lump('fuselage shell and formers');
  const rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0;
  const drag = 0.5 * rho * A.pod.V_drag_m_s ** 2 * (d.body?.CdA_front_m2 ?? 0.003);
  const nPod = A.pod.formers * A.pod.ties_each;
  members.push({ name: 'pod formers ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: nPod, extra_N: +drag.toFixed(4) }, sections: [sect('F', inertialTerms(pod.m, pod.r))], uses: { F: 'F' }, note: `${nPod} x ${tieS.id} (balsa formers to the boom and trays: no published bond strength on balsa); the pod's inertia plus its drag at ${A.pod.V_drag_m_s} m/s (${drag.toFixed(2)} N)` });
  counts.tieS += nPod;
  const pit = lump('pitot-static probe');
  if (pit) { members.push({ name: 'pitot-static probe ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.pitot.ties, extra_N: +(0.5 * rho * A.pod.V_drag_m_s ** 2 * Math.PI * 0.00635 ** 2 / 4 * 1.2).toFixed(4) }, sections: [sect('F', inertialTerms(pit.m, pit.r))], uses: { F: 'F' } , note: `${A.pitot.ties} x ${tieS.id} to the battery and the forward tray's nose: its inertia and the drag of its 6.35 mm tip at ${A.pod.V_drag_m_s} m/s (Cd 1.2)` }); counts.tieS += A.pitot.ties; }
  members.push({ name: 'tail surfaces ties', kind: 'tie', check: 'tie', tie: { id: tieS.id, loop_N: tieS.loop_N }, dims: { n: A.tail.ties }, sections: [sect('F', addTerms([{ ch: 'tail_F', k: 1 }], tailIn))], uses: { F: 'F' }, note: `${A.tail.ties} x ${tieS.id}: the tail's aerodynamic force and inertia` });
  counts.tieS += A.tail.ties;

}

function af_plateSections(ax) {
  const { catalog, L, jointByName, members, mat, A, shafts, crank, tFwd, tAft, railPlates, railPos, yB, boomRoot, fwdTray, aftTray, mo } = ax;
  // ---- Plates: solid outlines, sections through the plate below the rail seats -------------------------------------
  const pmat = mat(L.plates.material);
  const ringR = (jt) => { const p = byId(catalog, jt.part_id).specs; const D = Number.isFinite(p.d2_mm) ? p.d2_mm : +((p.dimensions_mm_d_D_B ?? '').split('x')[1] ?? 10); return (D / 2 + A.plates.edge_mm) / 1000; };
  const seatsOf = (plateName) => {
    const out = [];
    const named = { 'crank shaft': 0, 'gear shaft 1': 1, 'gear shaft 2': 2, 'gear shaft 3': 3 };
    for (const [nm, i] of Object.entries(named)) { const jt = jointByName(nm); if (jt && shafts[i]) out.push({ c: shafts[i], r: ringR(jt) }); }
    if (plateName === 'fwd') { const jt = jointByName('motor shaft outboard'); if (jt) out.push({ c: shafts[shafts.length - 1], r: Math.max(ringR(jt), (mo.circle_r_mm + 5) / 1000) }); }
    for (const side of ['right', 'left']) out.push({ c: railPos[side], r: A.rail.seat_D_mm / 2000 });
    // Flange lands for the panels (the flange's own width past the panel line).
    for (const q of A.panels.list) {
      const ends = q.kind === 'side' ? [[q.z_mm, q.y0_mm], [q.z_mm, q.y0_mm + q.h_mm]] : [[-q.w_mm / 2, q.y_mm], [q.w_mm / 2, q.y_mm]];
      for (const e of ends) out.push({ c: [e[0] / 1000, e[1] / 1000], r: 0.002 });
    }
    return out;
  };
  // A pad outside the plate's hull hangs on a post of post_w_mm: in-plane bending at the hull edge plus the axial force.
  const topAt = (poly, z, up) => { let best = up ? -Infinity : Infinity; for (let i = 0; i < poly.length; i++) { const p = poly[i], q = poly[(i + 1) % poly.length]; if ((p[0] - z) * (q[0] - z) <= 0 && p[0] !== q[0]) { const y = p[1] + ((z - p[0]) / (q[0] - p[0])) * (q[1] - p[1]); best = up ? Math.max(best, y) : Math.min(best, y); } } return best; };
  const plates = {};
  for (const pl of ['aft', 'fwd']) {
    const h = hullOf(seatsOf(pl));
    const yCut = Math.min(railPos.right[1], railPos.left[1]) - A.rail.seat_D_mm / 2000;
    const b = hullWidthAt(h.poly, yCut);
    const zR = Math.abs(railPos.right[0]);
    const F = addTerms(...railPlates[pl], boomRoot[pl]);
    const M = addTerms(...railPlates[pl].map((R) => scaleTerms(R, zR)));
    members.push({ name: `${pl === 'aft' ? 'aft' : 'forward'} plate below the rail seats`, kind: 'plate', check: 'plate', material: pmat, dims: { b_mm: +(b * 1000).toFixed(2), t_mm: pl === 'aft' ? tAft : tFwd }, sections: [sect('F', F), sect('M', M)], uses: { F: 'F', M: 'M' }, note: `the section ${(yCut * 1000).toFixed(1)} mm up, ${(b * 1000).toFixed(1)} mm wide: both rails' reactions and the boom clamp force, the rails' at their ${(zR * 1000).toFixed(0)} mm offset as in-plane bending` });
    const posts = [];
    const pads = [['boom clamp', [0, yB], boomRoot[pl], A.boom.screw_pitch_mm + 6], [pl === 'fwd' ? 'forward tray' : 'aft tray', [0, (pl === 'fwd' ? A.tray.y_mm : A.aftTray.y_mm) / 1000], pl === 'fwd' ? fwdTray.V : aftTray.V, (pl === 'fwd' ? A.tray : A.aftTray).w_mm]];
    for (const [nm, c, Fp, padW] of pads) {
      const up = c[1] > 0;
      const edge = topAt(h.poly, c[0], up);
      const Lp = Math.max(0, up ? c[1] - edge : edge - c[1]);
      if (Lp > 0) {
        const pw = A.plates.post_w_mm;
        members.push({ name: `${pl === 'aft' ? 'aft' : 'forward'} plate post to the ${nm}`, kind: 'plate', check: 'plate', material: pmat, dims: { b_mm: pw, t_mm: pl === 'aft' ? tAft : tFwd }, sections: [sect('F', Fp), sect('M', scaleTerms(Fp, Lp))], uses: { F: 'F', M: 'M' }, note: `a ${pw} mm post ${(Lp * 1000).toFixed(1)} mm from the plate's outline to the pad: the pad force axially and at the post length` });
        posts.push({ to: nm, length_m: Lp, area_m2: pw / 1000 * Lp + (padW / 1000) * (2 * A.plates.pad_mm / 1000) });
      }
    }
    plates[pl] = { area_m2: h.area_m2 + posts.reduce((a, q) => a + q.area_m2, 0), hull_area_m2: h.area_m2, posts, cut_y_m: yCut, cut_width_m: b };
  }

  Object.assign(ax, { plates });
}

function af_panels(ax) {
  const { lay, jointByName, planes, members, mat, item, A, glueAl, xA, gap, shafts, crank, railPlates, railPos, yB, boomRoot, fwdTray, mot, F0, Tm, motorIn } = ax;
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
    members.push({ name: `gearbox ${q.name} flange bond`, kind: 'bond', check: 'lap', material: glueAl, dims: { w_mm: h, L_mm: pn.flange_mm }, sections: [sect('F', V)], uses: { F: 'F' }, note: 'Araldite 2011 aluminium to aluminium, one end flange (each end has one)' });
    panelV += h * 1e-3 * (gap + 2 * pn.flange_mm / 1000) * (q.t_mm / 1000);
  }
  item(`gearbox panels, ${pn.list.length} off`, pm, panelV, `${pn.list.map((q) => `${q.name} ${q.kind === 'side' ? q.h_mm : q.w_mm} x ${q.t_mm} mm`).join(', ')}, each ${(gap * 1000).toFixed(1)} mm plus two ${pn.flange_mm} mm flanges; ${pm.density_g_cm3} g/cm3`);

}

function af_bomItems(ax) {
  const { catalog, A, nut, tieS, tieL, counts, report, railPos, sCl, yB, Lt, fwdTray, aftTray, plates } = ax;
  // ---- Fasteners and ties in the BOM ----------------------------------------------------------------------------------
  const sp = byId(catalog, A.fastener);
  ax.cx.items.push({ name: `M3 screws, ${counts.screws} off`, material_id: sp.id, material: sp.name, mass_g: counts.screws * sp.mass_g, basis: `${counts.screws} x ${sp.mass_g} g (${sp.specs.mass_basis})`, source_url: sp.source_url });
  if (counts.nuts) ax.cx.items.push({ name: `M3 nuts, ${counts.nuts} off`, material_id: nut.id, material: nut.name, mass_g: counts.nuts * nut.mass_g, basis: `${counts.nuts} x ${nut.mass_g} g (${nut.specs.mass_basis})`, source_url: nut.source_url });
  for (const [t, n] of [[tieS, counts.tieS], [tieL, counts.tieL]]) if (n) ax.cx.items.push({ name: `cable ties ${t.id}, ${n} off`, material_id: t.id, material: t.name, mass_g: n * t.mass_g, basis: `${n} x ${t.mass_g} g (maker's weight per unit)`, source_url: t.source_url });

  report.parts = { rails: railPos, boom_y_m: yB, clamp_spacing_m: sCl, tail_lever_m: Lt, trays: { forward: { y_m: fwdTray.y, length_m: fwdTray.length_m }, aft: { y_m: aftTray.y, length_m: aftTray.length_m } }, plates, counts };
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

function mergeDefaults(def, over) {
  const o = { ...def };
  for (const [k, v] of Object.entries(over)) o[k] = v && typeof v === 'object' && !Array.isArray(v) && def[k] && typeof def[k] === 'object' ? { ...def[k], ...v } : v;
  return o;
}
