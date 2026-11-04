// Round 6: every loaded part of the drive is linkage (SPEC ruling after physics round 5). The shafts, pins, gearbox
// plate seats, shoulder housings, rockers, coupler links and keys of the drive, each sized from a catalogue material
// with a published strength and density, put in the BOM with its mass, and given the load sections the flight model
// evaluates at every step (src/physics/members.js checks them in M7). Geometry from the film's drive layout
// (joints.js driveLayout: model.js placeFourBar, gearTrain, gearPlanes, used unchanged).
//
// Idealisations, each conservative and stated in the design file:
//  - forces are summed as magnitudes (directions not credited), exactly as the bearing supports are;
//  - a shaft's bending moment is bounded by sum |F_i| |m_i(x)| at its supports and load planes;
//  - a pin pressed in a boss is judged at its root (the boss face) and the boss by the rigid-pin edge pressure;
//  - a shoulder pin is clamped by the rocker hub between its two bushings, so it bends only within each bushing;
//  - seats with no published bearing strength are judged against the material's tensile yield (a lower bound).
import { byId } from './catalog.js';
import { shaftMomentSections } from '../physics/members.js';
import { linkageAt, journalBodyLength, journalDims, journalEdges } from '../../sims/avis-shared/model.js';
import { airframeMembers, inertialTerms, fastenerOf } from './airframe.js';

const PI4 = Math.PI / 4;
const rodV = (d, L) => PI4 * d * d * L; // m^3
const ringV = (D, d, L) => PI4 * (D * D - d * d) * L;

/** A catalogue metal as the checks need it: density and the design values its catalogue entry states (SPEC rules). */
export function materialOf(catalog, id) {
  const m = byId(catalog, id);
  const s = m.specs ?? {};
  const dv = s.design_values ?? {};
  if (!Number.isFinite(dv.yield_MPa) && !Number.isFinite(dv.shear_MPa)) throw new Error(`${id}: no design yield stated in the catalogue`);
  return { id: m.id, name: m.name, density_g_cm3: s.density_g_cm3, yield_MPa: dv.yield_MPa ?? null, shear_MPa: dv.shear_MPa ?? null, shear_yield_MPa: dv.shear_yield_MPa ?? null, bearing_yield_MPa: dv.bearing_yield_MPa ?? null, bearing_e_over_d_min: dv.bearing_e_over_d_min ?? null, bearing_counted_at_half: dv.bearing_counted_at_half ?? false, ultimate_MPa: dv.ultimate_MPa ?? null, bearing_standard: dv.bearing_standard ?? null, bearing_standard_lists_alloy: dv.bearing_standard_lists_alloy ?? null, basis: dv.basis, source_url: m.source_url, E_GPa: s.elastic_modulus_gpa ?? null };
}

const gOf = (mat, v) => v * 1e6 * mat.density_g_cm3; // m^3 -> g

/** Bore, outside diameter and length (m) of a bushing (d1, d2, b1) or bearing ("d x D x B"). */
function partDims(part) {
  const s = part.specs ?? {};
  if (Number.isFinite(s.d1_mm)) return { d: s.d1_mm / 1000, D: s.d2_mm / 1000, B: s.b1_mm / 1000, flangeB: (s.b2_mm ?? 0) / 1000 };
  const m = /^(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)/.exec(s.dimensions_mm_d_D_B ?? '');
  return m ? { d: +m[1] / 1000, D: +m[2] / 1000, B: +m[3] / 1000, flangeB: 0 } : null;
}

const scale = (terms, f) => terms.map((t) => ({ ch: t.ch, k: +(t.k * f).toFixed(8) }));
const sect = (name, radial) => ({ name, radial, axial: [] });

/** What every part of driveMembers needs: the layout, the parts' dimensions and the lists it fills. */
function linkageContext(d, catalog, L, lay, joints) {
  const items = [];
  const eye = byId(catalog, L.couplerEyes ?? d.params?.joints?.couplerEyes);
  const shoulder = byId(catalog, d.params.joints.shoulder);
  const planes = lay.planes;
  const plateT = planes.plateT ?? 0.002;
  return {
    d, catalog, L, lay, joints, items, members: [], unchecked: [], planes, plateT,
    mat: (id) => materialOf(catalog, id),
    item: (name, m, v, basis, extra = {}) => items.push({ name, material_id: m.id, material: m.name, mass_g: gOf(m, v), basis, source_url: m.source_url, ...extra }),
    jointByName: (nm) => joints.find((j) => j.name === nm),
    shaftLen: planes.plateFwd - planes.plateAft + plateT + 0.002, // plate to plate, through both, 1 mm proud each side
    eye, eyeD: partDims(eye), shoulder, shD: partDims(shoulder),
    span: (d.params.joints.shoulderSpan_mm ?? 24) / 1000,
  };
}
const boreText = (catalog, id) => `${byId(catalog, id).specs?.bore_mm ?? '?'} mm bore`;

/**
 * Members and mass items of the linkage for a built design d (geometry final), its joints and the layout.
 * L: the design's linkage choice (round6.mjs ROUND6.linkage). Returns { members, items, coupler, unchecked }.
 */
export function driveMembers(d, catalog, L, lay, joints) {
  const cx = linkageContext(d, catalog, L, lay, joints);
  crankMembers(cx);
  for (let j = 1; j < lay.n; j++) gearShaftMembers(cx, j);
  const s0 = lay.train.stages[0];
  // The first pinion on the motor shaft: its fixing is the same Duracon question.
  cx.members.push(gearFixing(`${s0.pinion.id} on the motor shaft, fixing`, catalog, s0.pinion.id, L.fixings, byId(catalog, s0.pinion.id).specs?.bore_mm ?? null, [{ ch: 'crank_torque', k: +(1 / lay.below(0) / (s0.wheel.z / s0.pinion.z)).toFixed(8) }]));
  for (const [side, w] of [['right', 'R'], ['left', 'L']]) wingSideMembers(cx, side, w);
  retentionBom(cx);
  const coupler = couplerLinks(cx);
  plateSeats(cx);
  let airframe = null;
  if (L.airframe) {
    // Round 8 (SPEC ruling after round 7: every part reaches the airframe through a rated path): the frame, its
    // fasteners, bonds and ties (airframe.js); the plates' mass from their solid outlines, not the area allowance.
    airframe = airframeMembers(cx).report;
    const pm = cx.mat(L.plates.material);
    const it = cx.items.find((q) => q.name === 'gearbox plates, 2 off');
    const { aft, fwd } = airframe.parts.plates;
    const tF = L.plates.t_fwd_mm ?? L.plates.t_mm;
    Object.assign(it, { mass_g: (aft.area_m2 * L.plates.t_mm + fwd.area_m2 * tF) / 1000 * pm.density_g_cm3 * 1e6, basis: `solid outlines (the hull of every seat and rail seat, posts to the pads; no lightening credited${aft.holes_area_m2 ? '; less the seat, rail and boom holes' : ''}): aft ${(aft.area_m2 * 1e4).toFixed(1)} cm2 x ${L.plates.t_mm} mm, forward ${(fwd.area_m2 * 1e4).toFixed(1)} cm2 x ${tF} mm; ${pm.density_g_cm3} g/cm3` });
  } else {
    // Round 7 (SPEC ruling after round 6: a load path that cannot be rated fails): the plates between their seats.
    cx.members.push(unrated('gearbox plates between seats', 'plate', 'the drive layout (model.js) gives the two plates no mounts to the airframe, so the path from the seats to the keel cannot be sized or rated', [{ ch: 'crank_torque', k: 1 }]));
  }
  return { members: cx.members, items: cx.items, coupler, unchecked: cx.unchecked, airframe };
}

/**
 * Round 17 (audit items 7, 8, F4/F5): one BOM line per retaining-hardware catalogue part (params.linkage.retention),
 * however many places use it: gearShaftMembers and wingSideMembers draw on the same part (washer8mm, shared between
 * the two horn pins and the two bushing shafts' four ends so they consolidate here, not as four separate lines) and
 * only rate and mass what is drawn; this is the one place that counts how many.
 */
function retentionBom(cx) {
  const { d, catalog, lay, items } = cx, Ret = cx.L.retention;
  if (!Ret) return;
  if (Ret.washer8mm) {
    const w = byId(catalog, Ret.washer8mm);
    const shafts = d.params.joints.shafts ?? [];
    const nBushingShafts = Array.from({ length: lay.n - 1 }, (_, i) => shafts[i] ?? shafts.at(-1)).filter((id) => byId(catalog, id).category !== 'bearing').length;
    const nExcluded = (Ret.washer8mmForwardExclude ?? []).length; // found to clash: see clearance.js gearParts
    const nHorn = Ret.hornPinWasher ? 2 : 0; // off by default: see clearance.js pinGroove
    const n = nHorn + 2 * nBushingShafts - nExcluded;
    if (n > 0) items.push({ name: `${w.name}, ${n} off`, material_id: w.id, material: w.name, mass_g: n * w.mass_g, basis: `${n} x ${w.mass_g} g each (${w.specs.mass_basis}): ${nHorn} horn pins and ${nBushingShafts} gear shaft${nBushingShafts === 1 ? '' : 's'} at both ends${nExcluded ? `, less ${nExcluded} excluded (found to clash with an adjacent shaft's gear)` : ''}`, source_url: w.source_url, count: n });
  }
  if (Ret.gearShaft3Circlip) {
    const c = byId(catalog, Ret.gearShaft3Circlip);
    items.push({ name: `${c.name}, 1 off`, material_id: c.id, material: c.name, mass_g: c.mass_g, basis: `1 x ${c.mass_g} g (${c.specs.mass_basis})`, source_url: c.source_url, count: 1 });
  }
}

/** The one-piece crank (shaft, web and pin) of L.crank.material, so torque and pin moment cross no joint. */
function crankMembers(cx) {
  // ---- Crank: one piece (shaft, web and pin) of L.crank.material, so torque and pin moment cross no joint.
  const { d, catalog, L, lay, planes, plateT, members, items, mat, item, jointByName, eyeD } = cx;
  const { train, n, meshPerNm } = lay;
  {
    const c = L.crank, m = mat(c.material);
    const od = c.shaft_od_mm / 1000, id = (c.shaft_id_mm ?? 0) / 1000, tw = c.web_t_mm / 1000, bw = c.web_b_mm / 1000, dp = eyeD.d;
    const xw = planes.crankArm - tw / 2; // the web's aft face
    const pinL = xw - (planes.couplerL - eyeD.B / 2 - 0.001);
    const lenS = (planes.crankEnd ?? planes.plateFwd + plateT / 2 + 0.001) - (planes.crankArm + tw / 2); // round 14: to an encoder stub
    const wheel = train.stages[n - 1].wheel;
    const seatL = (wheel.face ?? 0.01);
    const v = ringV(od, id, lenS) + bw * tw * (d.drive.a + (od + dp) / 2) + rodV(dp, pinL) + (c.seat_od_mm ? ringV(c.seat_od_mm / 1000, od, seatL) : 0);
    if (c.web?.separate) crankSplit(cx, { c, m, od, id, tw, bw, dp, pinL, lenS, wheel, seatL });
    else item('crank: shaft, web and pin in one piece', m, v, `${c.shaft_od_mm}${id ? `/${c.shaft_id_mm}` : ''} mm shaft x ${(lenS * 1000).toFixed(1)} mm, web ${c.web_b_mm} x ${c.web_t_mm} mm over the ${(d.drive.a * 1000).toFixed(0)} mm throw, ${(dp * 1000).toFixed(0)} mm pin x ${(pinL * 1000).toFixed(1)} mm${c.seat_od_mm ? `, ${c.seat_od_mm} mm wheel seat x ${(seatL * 1000).toFixed(0)} mm (the ${wheel.id}, ${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
    const loads = [
      { ch: 'coupler_R', perUnit: 1, x: planes.couplerR },
      { ch: 'coupler_L', perUnit: 1, x: planes.couplerL },
      { ch: 'crank_torque', perUnit: meshPerNm(n - 1), x: planes.stage[n - 1] },
    ];
    const ms = shaftMomentSections(planes.plateAft, planes.plateFwd, loads, 'M').filter((q) => q.x_m >= planes.crankArm - 1e-9);
    const aft = jointByName('crank shaft').supports[0];
    members.push({ name: 'crank shaft', kind: 'shaft', check: 'round', material: m, dims: { d_mm: c.shaft_od_mm, di_mm: c.shaft_id_mm ?? 0 }, sections: [...ms.map((q) => sect(q.name, q.radial)), sect('T', [{ ch: 'crank_torque', k: 1 }]), sect('V aft', aft.radial.map((t) => ({ ch: t.ch, k: t.k })))], uses: { M: ms.map((q) => q.name), T: 'T', V: 'V aft' }, note: 'checked at the shaft section everywhere (the wheel seat is larger); torque between the crank wheel and the web' });
    const Mpin = [{ ch: 'coupler_R', k: +(xw - planes.couplerR).toFixed(6) }, { ch: 'coupler_L', k: +(xw - planes.couplerL).toFixed(6) }];
    members.push({ name: 'crank pin', kind: 'pin', check: 'round', material: m, dims: { d_mm: +(dp * 1000).toFixed(3) }, sections: [sect('M root', Mpin), sect('V root', [{ ch: 'coupler_R', k: 1 }, { ch: 'coupler_L', k: 1 }])], uses: { M: 'M root', V: 'V root' }, note: 'cantilevered from the web\'s aft face, both coupler eyes at their planes' });
    members.push({ name: 'crank web', kind: 'web', check: 'web', material: m, dims: { b_mm: c.web_b_mm, h_mm: c.web_t_mm, lever_mm: +(d.drive.a * 1000).toFixed(3) }, sections: [sect('M out of plane', Mpin), sect('F in plane', [{ ch: 'coupler_R', k: 1 }, { ch: 'coupler_L', k: 1 }])], uses: { M: 'M out of plane', F: 'F in plane' }, note: 'at the shaft root: the pin moment out of the web plane plus the coupler forces at the full throw in it' });
    if (c.key) members.push(keyMember(`key, crank wheel ${wheel.id}`, catalog, c.key, m, c.seat_od_mm, [{ ch: 'crank_torque', k: 1 }], items));
    if (c.key) members.push(keyHubSide(`key, crank wheel ${wheel.id}, gear side`, catalog, c.key, wheel.id, c.seat_od_mm, [{ ch: 'crank_torque', k: 1 }]));
  }
}

/**
 * Round 16 (SPEC "Completeness and assembly order"; crank.web.separate): the one-piece crank could not go together (its
 * web aft of the aft plate and its wheel seat inside the box, neither through the plate's 10 mm bushing). The web and
 * its pin are one part, keyed on a solid stub at the shaft's aft end (a parallel key over the web's thickness, both
 * sides rated by the key rule) and held there axially with LOCTITE 638, whose maker publishes no strength on titanium:
 * that axial fixing is unrated, RC practice (SPEC ruling), reported with the torque its joint carries. The shaft keeps
 * its wheel seat and goes into the aft plate's bushing from forward. The shaft is checked again at the keyseat's root.
 */
function crankSplit(cx, { c, m, od, id, tw, bw, dp, pinL, lenS, wheel, seatL }) {
  const { d, catalog, L, planes, members, items, item } = cx;
  const a = d.drive.a, K = c.web.key, T = [{ ch: 'crank_torque', k: 1 }], W = c.web;
  // round 17: the web's hub boss (W.hub_od_mm) round its bore, and an end lip (W.lip_mm) over the stub's end that takes
  // the axial screw; the key under the rest of the web
  const rh = (W.hub_od_mm ?? c.web_b_mm) / 2000, rb = bw / 2, lip = (W.lip_mm ?? 0) / 1000;
  const hull = hullArea(rh, rb, a), boreA = Math.PI * (od / 2) ** 2;
  const screw = W.axial ? byId(catalog, W.axial.screw) : null, dS = screw ? +/M(\d+(?:\.\d+)?)/.exec(screw.specs.thread ?? '')[1] / 1000 : 0;
  const holeR = screw ? (W.axial.countersunk ? screw.specs.head_d_mm / 2000 : dS / 2) + (W.axial.clearance_mm ?? 0.2) / 1000 : 0;
  const webV = (hull - boreA) * (tw - lip) + (hull - Math.PI * holeR * holeR) * lip;
  const solidEnd = screw && W.axial.solid_mm ? W.axial.solid_mm / 1000 : 0;
  item('crank shaft', m, ringV(od, id, lenS - solidEnd) + PI4 * od * od * (tw - lip + solidEnd) + (c.seat_od_mm ? ringV(c.seat_od_mm / 1000, od, seatL) : 0), `${c.shaft_od_mm}${id ? `/${c.shaft_id_mm}` : ''} mm shaft x ${(lenS * 1000).toFixed(1)} mm, a solid ${c.shaft_od_mm} mm stub x ${((tw - lip) * 1000).toFixed(1)} mm under the web (keyseated${screw ? `, its end tapped for the web's axial screw, solid ${(solidEnd * 1000).toFixed(1)} mm on beyond the web for its thread` : ''})${c.seat_od_mm ? `, ${c.seat_od_mm} mm wheel seat x ${(seatL * 1000).toFixed(0)} mm (the ${wheel.id}, ${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
  item('crank web and pin', m, webV + rodV(dp, pinL), `web ${c.web_b_mm} mm wide at the throw, a ${(rh * 2000).toFixed(0)} mm hub round its ${c.shaft_od_mm} mm bore (keyed on the shaft's stub), ${c.web_t_mm} mm thick${lip ? ` with a ${(lip * 1000).toFixed(1)} mm end lip over the stub (the axial screw's ${(holeR * 2000).toFixed(1)} mm hole)` : ''}, ${(dp * 1000).toFixed(0)} mm pin x ${(pinL * 1000).toFixed(1)} mm in one piece; ${m.density_g_cm3} g/cm3`);
  members.push(keyMember('key, crank web', catalog, K, m, c.shaft_od_mm, T, items));
  const km = materialOf(catalog, K.material);
  members.push({ name: 'key, crank web, web side', kind: 'key', check: 'key', material: km, side_material: m, dims: { d_mm: c.shaft_od_mm, b_mm: K.b_mm, h_mm: +(K.h_mm - K.shaft_depth_mm).toFixed(3), L_mm: K.L_mm }, sections: [sect('T', T)], uses: { T: 'T' }, note: `bearing on the web's keyway (${(K.h_mm - K.shaft_depth_mm).toFixed(1)} mm deep), the web's own ${m.id}` });
  // round 17 (audit item 13): the hub's wall behind the keyway, in shear on two planes under the key's force (T / r):
  // the wall from the keyway's bottom to the hub's outside, over the key's length
  const wall = rh * 1000 - c.shaft_od_mm / 2 - (K.h_mm - K.shaft_depth_mm);
  members.push({ name: 'crank web hub wall at the keyway', kind: 'web', check: 'pin-shear', material: m, dims: { d_mm: +Math.sqrt((4 * 2 * wall * K.L_mm) / Math.PI / 2).toFixed(4), planes: 2, area_mm2: +(2 * wall * K.L_mm).toFixed(3) }, sections: [sect('F', [{ ch: 'crank_torque', k: +(2000 / c.shaft_od_mm).toFixed(6) }])], uses: { F: 'F' }, note: `the key's force (crank torque / ${(c.shaft_od_mm / 2).toFixed(1)} mm) shearing the hub's ${wall.toFixed(2)} mm wall behind the keyway on two planes over the key's ${K.L_mm} mm (as a pin of that shear area in double shear)` });
  const Mweb = [{ ch: 'coupler_R', k: +(planes.crankArm - planes.couplerR).toFixed(6) }, { ch: 'coupler_L', k: +(planes.crankArm - planes.couplerL).toFixed(6) }];
  // the keyseated solid stub as the round section of the same bending modulus: Z = pi d^3 / 32 - b t (d - t)^2 / (2 d)
  // (a shaft with one keyseat of width b and depth t, the standard section property), smaller than its torsion one's
  const D = c.shaft_od_mm, bk = K.b_mm, tk = K.shaft_depth_mm, dEq = Math.cbrt(D ** 3 - (16 * bk * tk * (D - tk) ** 2) / (Math.PI * D));
  members.push({ name: 'crank shaft at the web\'s keyseat', kind: 'shaft', check: 'round', material: m, dims: { d_mm: +dEq.toFixed(3), di_mm: 0 }, sections: [sect('M', Mweb), sect('T', T), sect('V', [{ ch: 'coupler_R', k: 1 }, { ch: 'coupler_L', k: 1 }])], uses: { M: 'M', T: 'T', V: 'V' }, note: `the solid ${D} mm stub under the web with its ${bk} x ${tk} mm keyseat, as a ${dEq.toFixed(2)} mm round of the same bending modulus (pi d^3/32 - b t (d - t)^2 / 2 d): the coupler forces at the web's mid-plane as bending, the whole crank torque` });
  if (!screw) {
    members.push({ name: 'crank web on the crank shaft, axial fixing', kind: 'shaft fixing', check: 'rc practice', material: null, dims: { d_mm: c.shaft_od_mm, L_mm: c.web_t_mm }, sections: [sect('T', T)], uses: { T: 'T' }, rc: { method: `keyed (the key rated above) and held axially with LOCTITE 638 on the web's bore: LOCTITE publishes no strength on titanium, so the axial fixing is unrated`, published_Nm: null, published_basis: null }, note: 'SPEC ruling "RC practice for joints no one can rate": a hub (the crank web) held to its shaft by a bond no maker rates on this material; its torque is the key\'s' });
    return;
  }
  // round 17: the web held axially by one screw into the stub's tapped end, its head on the end lip (ISO 898-1); the
  // web, its pin and the two coupler links that ride on the pin (their eyes retained on it) pull on it under the bird's
  // accelerations (inertial terms: every axis counted, conservative for the axial one)
  items.push({ name: `${screw.name}, 1 off`, material_id: screw.id, material: screw.name, mass_g: screw.mass_g, basis: `1 x ${screw.mass_g} g (${screw.specs.mass_basis})`, source_url: screw.source_url, count: 1 });
  const fx = fastenerOf(catalog, W.axial.screw), mAx = (gOf(m, webV + rodV(dp, pinL)) + couplerMassG(cx)) / 1000;
  const rAx = [planes.crankArm, (cx.lay?.fb?.O2?.[1] ?? 0), 0];
  members.push({ name: 'crank web axial screw', kind: 'fasteners', check: 'bolts', fastener: fx, dims: { n: 1 }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `one ${W.axial.screw} into the stub's tapped end, in tension under the web, pin and couplers' ${(mAx * 1000).toFixed(1)} g` });
  // its thread in the titanium stub: stripping over the engaged length, the pitch cylinder's half (the usual area for a
  // thread in the softer part), against the stub's shear yield
  const pitch = +(/pitch ([\d.]+)/.exec(screw.specs.thread ?? '')?.[1] ?? (dS * 1000 >= 3 ? 0.5 : 0.4)), Le = Math.min(screw.specs.length_mm - (W.lip_mm ?? 0), c.web_t_mm - (W.lip_mm ?? 0) + (W.axial.solid_mm ?? 0)), d2 = dS * 1000 - 0.6495 * pitch;
  members.push({ name: 'crank web axial screw thread in the stub', kind: 'pin', check: 'pin-shear', material: m, dims: { d_mm: +Math.sqrt(4 * (Math.PI * d2 * Le / 2) / Math.PI).toFixed(4), planes: 1, Le_mm: +Le.toFixed(2), area_mm2: +(Math.PI * d2 * Le / 2).toFixed(3) }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `the internal thread in the ${m.id} stub over ${Le.toFixed(1)} mm engaged (the solid stub under the web and the shaft's solid end beyond it): shear area pi d2 Le / 2 = ${(Math.PI * d2 * Le / 2).toFixed(2)} mm2 (d2 the pitch diameter ${d2.toFixed(3)} mm), as a pin of that area` });
  const dPunch = W.axial.countersunk ? (screw.specs.head_d_mm + dS * 1000) / 2 : screw.specs.head_d_mm;
  members.push({ name: 'crank web end lip under the screw head', kind: 'web', check: 'pin-shear', material: m, dims: { d_mm: +Math.sqrt(4 * (Math.PI * dPunch * (W.lip_mm ?? 0)) / Math.PI).toFixed(4), planes: 1, area_mm2: +(Math.PI * dPunch * (W.lip_mm ?? 0)).toFixed(3) }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `the ${W.lip_mm} mm lip punched out round the screw's head (${W.axial.countersunk ? `countersunk: the cone's mean diameter ${dPunch.toFixed(2)} mm` : `${dPunch} mm`}): shear area pi x that x t, as a pin of that area` });
}

/** The area (m2) of the convex hull of two circles of radii r1 and r2, centres a apart (the crank web with its hub). */
export function hullArea(r1, r2, a) {
  if (a <= Math.abs(r1 - r2)) return Math.PI * Math.max(r1, r2) ** 2;
  const phi = Math.asin((r1 - r2) / a), L = a * Math.cos(phi);
  // two tangent segments, the larger circle's arc (pi + 2 phi), the smaller's (pi - 2 phi)
  return L * (r1 + r2) + 0.5 * r1 * r1 * (Math.PI + 2 * phi) + 0.5 * r2 * r2 * (Math.PI - 2 * phi);
}

/** The two coupler links' mass (g), as couplerLinks weighs them. */
function couplerMassG(cx) {
  const { d, L, mat, eyeD } = cx, cm = mat(L.coupler.material), bL = (d.params.coupler_mm ?? 70) / 1000;
  return 2 * gOf(cm, rodV(L.coupler.body_d_mm / 1000, bL - L.coupler.eye_od_mm / 1000) + 2 * ringV(L.coupler.eye_od_mm / 1000, eyeD.D, eyeD.B));
}

const plateTOf = (cx) => cx.plateT ?? 0.002;

/** Gear shaft j (crank side first): the wheel of stage n-1-j and the pinion of stage n-j, with their keys. */
function gearShaftMembers(cx, j) {
  // ---- Gear shafts j = 1..n-1 (crank side first): the wheel of stage n-1-j and the pinion of stage n-j.
  const { d, catalog, L, lay, planes, members, items, unchecked, mat, item, jointByName, shaftLen } = cx;
  const { train, n, below, meshPerNm } = lay;
    const g = L.gearShafts[j - 1], m = mat(g.material);
    const kw = n - 1 - j, kp = n - j;
    const jt = jointByName(`gear shaft ${j}`);
    const loads = [
      { ch: 'crank_torque', perUnit: meshPerNm(kw), x: planes.stage[kw] },
      { ch: 'crank_torque', perUnit: meshPerNm(kp), x: planes.stage[kp] },
    ];
    const ms = shaftMomentSections(planes.plateAft, planes.plateFwd, loads, 'M');
    const Tk = 1 / below(kw);
    const wheel = train.stages[kw].wheel;
    // round 14: a shaft carrying a gear outside the plates (stage 1 overhung) runs to that gear's far face
    const far = (si, gg) => planes.stage[si] + (gg.face ?? 0) / 2 + (gg.hubW ?? 0) - (planes.plateFwd + plateTOf(cx) / 2 + 0.001);
    const len = shaftLen + Math.max(0, far(kw, wheel), far(kp, train.stages[kp].pinion));
    const v = rodV((g.mass_d_mm ?? g.d_mm) / 1000, len) + (g.seat_od_mm ? ringV(g.seat_od_mm / 1000, g.d_mm / 1000, wheel.face ?? 0.01) : 0);
    item(`gear shaft ${j}`, m, v, `${g.mass_d_mm ?? g.d_mm} mm x ${(len * 1000).toFixed(1)} mm${g.seat_od_mm ? ` with a ${g.seat_od_mm} mm wheel seat for the ${wheel.id} (${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
    members.push({ name: `gear shaft ${j}`, kind: 'shaft', check: 'round', material: m, dims: { d_mm: g.d_mm }, sections: [...ms.map((q) => sect(q.name, q.radial)), sect('T', [{ ch: 'crank_torque', k: +Tk.toFixed(8) }]), ...jt.supports.map((sp) => sect(`V ${sp.name}`, sp.radial.map((t) => ({ ch: t.ch, k: t.k }))))], uses: { M: ms.map((q) => q.name), T: 'T', V: jt.supports.map((sp) => `V ${sp.name}`) }, note: `wheel ${wheel.id} and pinion ${train.stages[kp].pinion.id}; checked at ${g.d_mm} mm everywhere` });
    if (g.wheelKey) members.push(keyMember(`key, wheel ${wheel.id} on gear shaft ${j}`, catalog, g.wheelKey, m, g.seat_od_mm, [{ ch: 'crank_torque', k: +Tk.toFixed(8) }], items));
    if (g.pinionKey) members.push(keyMember(`key, pinion ${train.stages[kp].pinion.id} on gear shaft ${j}`, catalog, g.pinionKey, m, g.d_mm, [{ ch: 'crank_torque', k: +Tk.toFixed(8) }], items));
    if (g.wheelKey) members.push(keyHubSide(`key, wheel ${wheel.id} on gear shaft ${j}, gear side`, catalog, g.wheelKey, wheel.id, g.seat_od_mm, [{ ch: 'crank_torque', k: +Tk.toFixed(8) }]));
    if (g.pinionKey) members.push(keyHubSide(`key, pinion ${train.stages[kp].pinion.id} on gear shaft ${j}, gear side`, catalog, g.pinionKey, train.stages[kp].pinion.id, g.d_mm, [{ ch: 'crank_torque', k: +Tk.toFixed(8) }]));
    for (const k of [kw, kp]) {
      const gid = k === kw ? wheel.id : train.stages[kp].pinion.id;
      // A maker keyway only ("keyway 3 x 1.4 mm"); "plain bore, no keyway ..." is not one (round 10 fix).
      if (!/^keyway\s/.test(byId(catalog, gid).specs?.fastening ?? '')) members.push(gearFixing(`${gid} on gear shaft ${j}, fixing`, catalog, gid, L.fixings, byId(catalog, gid).specs?.bore_mm ?? null, [{ ch: 'crank_torque', k: +Tk.toFixed(8) }]));
    }
    // round 17 (audit items 7, 8, F4): this shaft's own axial retention (clearance.js gearShaftRetention draws it):
    // a DIN 6799 washer at each end outside the bushing's flange, or a DIN 471 circlip at the aft end against the
    // aft bearing's inner ring; 6061 spacer tubes fill the gaps so the gears can no longer slide on their keys.
    if (L.retention) {
      const Ret = L.retention, isBearing = byId(catalog, jt.part_id).category === 'bearing', pinion = train.stages[kp].pinion;
      const dShaft = g.d_mm / 1000, spacerOD = dShaft + 0.004, spacerMat = mat(Ret.spacerMaterial ?? 'metal-al-6061-t6');
      const tA = L.plates.t_mm / 1000, tF = (L.plates.t_fwd_mm ?? L.plates.t_mm) / 1000;
      const jd = (id) => journalDims(byId(catalog, id).specs);
      const edges = (side) => journalEdges(L.plates, side, side === 'aft' ? planes.plateAft : planes.plateFwd, side === 'aft' ? tA : tF, jd(jt.part_id));
      const pinionAft = planes.stage[kp] - pinion.face / 2, pinionFwd = planes.stage[kp] + pinion.face / 2 + (pinion.hubW ?? 0);
      const wheelAft = planes.stage[kw] - wheel.face / 2, wheelFwd = planes.stage[kw] + wheel.face / 2 + (wheel.hubW ?? 0);
      let spacerV = 0; const spacerNote = [];
      const addSpacer = (nm, x0, x1) => { const Ls = x1 - x0; if (Ls > 1e-9) { spacerV += ringV(spacerOD, dShaft, Ls); spacerNote.push(`${nm} ${(Ls * 1000).toFixed(1)} mm`); } };
      // the held mass: this shaft's own mass, its wheel and pinion (every axis counted, inertialTerms); the shaft
      // and its gears can only slide one way at a time, so each end's washer or the circlip is checked against it in full
      const shPos = lay.train.shafts[j], heldMass_kg = (gOf(m, v) + byId(catalog, wheel.id).mass_g + byId(catalog, pinion.id).mass_g) / 1000;
      const rPos = [planes.stage[kw], shPos[1], shPos[0]];
      if (!isBearing && Ret.washer8mm) {
        const w = byId(catalog, Ret.washer8mm).specs, jd0 = jd(jt.part_id);
        const aft = edges('aft'), fwd = edges('fwd');
        // found by probing (clearance.js gearParts): this shaft's forward end may be excluded, within 0.5 mm of an
        // adjacent shaft's gear tip circle; its spacer still runs to the bushing's own (unextended) inner edge
        const fwdExcluded = (Ret.washer8mmForwardExclude ?? []).includes(j);
        addSpacer('aft bushing to the pinion', aft.inner, pinionAft);
        addSpacer('pinion to the wheel', pinionFwd, wheelAft);
        addSpacer('wheel to the forward bushing', wheelFwd, fwd.inner - (d.params?.layout?.running_mm ?? 0.5) / 1000);
        for (const end of fwdExcluded ? ['aft'] : ['aft', 'forward']) members.push({ name: `gear shaft ${j} washer ${end}`, kind: 'retainer', check: 'rated', dims: { rating_N: 1000 * Math.min(w.load_columns.FN_kN, w.load_columns.Fs_kN), rating_basis: `${Ret.washer8mm}: the lower of FN ${w.load_columns.FN_kN} kN (groove) and Fs ${w.load_columns.Fs_kN} kN (washer) as tabulated (DIN 6799 convention; the groove's figure is for steel shafts and the shaft is ${m.id})` }, sections: [sect('F', inertialTerms(heldMass_kg, rPos))], uses: { F: 'F' }, note: `the shaft, its wheel and pinion sliding the other way, caught at the ${end} washer against the ${jt.part_id} flange` });
        if (!fwdExcluded) members.push({ name: `gear shaft ${j} bushing flanges, axial thrust`, kind: 'seat', check: 'flange', material: { id: jt.part_id, static_surface_pressure_mpa: byId(catalog, jt.part_id).specs.static_surface_pressure_mpa }, dims: { D_mm: jd0.flangeD * 1000, d_mm: jd0.D * 1000 }, sections: [sect('F', inertialTerms(heldMass_kg, rPos))], uses: { F: 'F' }, note: `the end washer's thrust into the ${jt.part_id} flange's ring, at its published static surface pressure (the same figure the bushing's own radial seat is judged against); both flanges alike` });
        else members.push({ name: `gear shaft ${j} bushing flange aft, axial thrust`, kind: 'seat', check: 'flange', material: { id: jt.part_id, static_surface_pressure_mpa: byId(catalog, jt.part_id).specs.static_surface_pressure_mpa }, dims: { D_mm: jd0.flangeD * 1000, d_mm: jd0.D * 1000 }, sections: [sect('F', inertialTerms(heldMass_kg, rPos))], uses: { F: 'F' }, note: `the aft washer's thrust into the ${jt.part_id} flange's ring, at its published static surface pressure; the forward flange has no washer against it (excluded, F4: stated rather than invented)` });
      } else if (isBearing && Ret.gearShaft3Circlip) {
        const c = byId(catalog, Ret.gearShaft3Circlip).specs;
        const aft = edges('aft'), fwd = edges('fwd');
        addSpacer('aft inner ring to the pinion', aft.inner, pinionAft);
        // the pinion-to-forward-bearing gap is already the running clearance (audit: 0.5 mm): left alone, no spacer
        addSpacer('forward inner ring to the bonded wheel', fwd.outer, wheelAft);
        members.push({ name: `gear shaft ${j} circlip aft`, kind: 'retainer', check: 'rated', dims: { rating_N: 1000 * Math.min(c.load_columns.FN_kN, c.load_columns.FR_kN), rating_basis: `${Ret.gearShaft3Circlip}: the lower of FN ${c.load_columns.FN_kN} kN (groove) and FR ${c.load_columns.FR_kN} kN (ring) as tabulated (DIN 471 convention; the groove's figure is for steel shafts and the shaft is ${m.id})` }, sections: [sect('F', inertialTerms(heldMass_kg, rPos))], uses: { F: 'F' }, note: 'the shaft, its pinion and the bonded DS1-60 sliding aft, caught at the circlip against the aft bearing\'s inner ring; forward, nothing is rated (F4: a direction with no rated retention, stated rather than invented, the aft bearing\'s own axial capacity not credited here)' });
      }
      if (spacerV > 0) item(`gear shaft ${j} spacers`, spacerMat, spacerV, `6061 tube ${(spacerOD * 1000).toFixed(1)}/${(dShaft * 1000).toFixed(1)} mm: ${spacerNote.join(', ')}; ${spacerMat.density_g_cm3} g/cm3`, { count: spacerNote.length });
    }
}

/**
 * Length (m) of the rocker hub between the two shoulder bushings: the gap between them (v21), less the layout's
 * running clearance at each end under the clearance rule (params.layout, v22 on), or the rocker's own hub_len_mm.
 */
export function rockerHubLength(d, rocker, span, bushingB) {
  if (Number.isFinite(rocker?.hub_len_mm)) return rocker.hub_len_mm / 1000;
  const run = d.params?.layout?.rule === 'clearance' ? (d.params.layout.running_mm ?? 0.5) / 1000 : 0;
  return span - bushingB - 2 * run;
}

/**
 * The rocker hub along body x (m). v21 and the clearance rule: centred on the hinge, rockerHubLength long. Round 14:
 * rocker.hub_clear_plate ends it the running clearance aft of the aft gearbox plate (the shoulder pin alone passes the
 * plate's x), and rocker.hub_relief_od_mm turns it down to that diameter where the coupler eyes pass (their planes
 * and the running clearance either side), so the eyes clear it while the rest keeps the edge distance of hub_od_mm.
 * A hub ended short of the forward bushing keeps its axial stop there by a collar on the shoulder pin (rocker.
 * hub_collar_od_mm), from the hub's face to the running clearance short of that bushing's flange.
 * -> { x: [aft, fwd], relief: [aft, fwd] | null, collar: [aft, fwd] | null, seat_m: the length at full diameter,
 * levers: { aft, forward } } where the levers are each bushing centre's distance from the hub face beside it (the
 * shoulder pin's bending arms; the collar is the pin's own, so it does not shorten its arm).
 */
export function rockerHub(d, rocker, span, bushingB, planes, eyeB) {
  const x0 = d.wing.hinge[0], hubL = rockerHubLength(d, rocker, span, bushingB);
  const run = (d.params?.layout?.running_mm ?? 0.5) / 1000;
  const a = x0 - hubL / 2;
  let b = x0 + hubL / 2;
  if (rocker?.hub_clear_plate && planes) b = Math.min(b, planes.plateAft - d.params.linkage.plates.t_mm / 2000 - run);
  let relief = null;
  if (Number.isFinite(rocker?.hub_relief_od_mm) && planes && eyeB) relief = [Math.max(a, planes.couplerL - eyeB / 2 - run), Math.min(b, planes.couplerR + eyeB / 2 + run)];
  const stop = x0 + span / 2 - bushingB / 2 - run; // the forward bushing's flange face, less the running clearance
  const collar = Number.isFinite(rocker?.hub_collar_od_mm) && b < stop - 1e-9 ? [b, stop] : null;
  return { x: [a, b], relief, collar, seat_m: b - a - (relief ? relief[1] - relief[0] : 0), levers: { aft: a - (x0 - span / 2), forward: x0 + span / 2 - b } };
}

/** One side's shoulder pin, rocker (hub, horn boss), horn pin, coupler eye and shoulder housings. */
function wingSideMembers(cx, side, w) {
  // ---- Shoulder pins: clamped in the rocker hub between the two bushings; bending within each bushing.
  const { d, L, lay, members, mat, item, jointByName, eye, eyeD, shoulder, shD, span } = cx;
    const m = mat(L.shoulderPin.material);
    const jt = jointByName(`shoulder pivot ${side}`);
    const r = L.rocker, mr = mat(r.material);
    const hub = rockerHub(d, r, span, shD.B, cx.planes, eyeD.B), hubL = hub.x[1] - hub.x[0];
    const collarL = hub.collar ? hub.collar[1] - hub.collar[0] : 0;
    item(`shoulder pin ${side}`, m, rodV(shD.d, span + shD.B) + (collarL ? ringV(r.hub_collar_od_mm / 1000, shD.d, collarL) : 0), `${(shD.d * 1000).toFixed(0)} mm x ${((span + shD.B) * 1000).toFixed(0)} mm (bushing centres ${(span * 1000).toFixed(0)} mm apart plus one bushing length)${collarL ? `, a ${r.hub_collar_od_mm} mm collar x ${(collarL * 1000).toFixed(1)} mm (the rocker's forward axial stop)` : ''}; ${m.density_g_cm3} g/cm3`);
    // the bushing reaction at its centre, from the hub face beside it (v21: half the bushing length; v22: plus the
    // running gap; round 14: the forward face may end short of the aft plate)
    const pinLever = Math.max(hub.levers.aft, hub.levers.forward);
    members.push({ name: `shoulder pin ${side}`, kind: 'pin', check: 'round', material: m, dims: { d_mm: +(shD.d * 1000).toFixed(3) }, sections: jt.supports.flatMap((sp) => [sect(`M ${sp.name}`, scale(sp.radial, hub.levers[sp.name] ?? pinLever)), sect(`V ${sp.name}`, sp.radial.map((t) => ({ ch: t.ch, k: t.k })))]), uses: { M: jt.supports.map((sp) => `M ${sp.name}`), V: jt.supports.map((sp) => `V ${sp.name}`) }, note: pinLever > shD.B / 2 + 1e-9 ? 'the bushing reaction at its centre, measured from the hub face' : 'the bushing reaction at half the bushing length from the hub face' });
    // Rocker: hub on the shoulder pin, horn arm of the rocker length c, horn boss for the horn pin, spar socket.
    const c = linkageAt(lay.law, d.drive.ampNominal).c;
    const sparOD = (r.spar_od_mm ?? 10) / 1000;
    const relL = hub.relief ? hub.relief[1] - hub.relief[0] : 0;
    const vR = ringV(r.hub_od_mm / 1000, shD.d, hubL - relL) + (relL ? ringV(r.hub_relief_od_mm / 1000, shD.d, relL) : 0) + c * (r.arm_w_mm / 1000) * (r.arm_t_mm / 1000) + ringV(r.boss_od_mm / 1000, eyeD.d, r.boss_len_mm / 1000) + ringV(sparOD + 2 * r.socket_wall_mm / 1000, sparOD, r.socket_len_mm / 1000);
    item(`rocker ${side}`, mr, vR, `hub ${r.hub_od_mm}/${(shD.d * 1000).toFixed(0)} x ${(hubL * 1000).toFixed(0)} mm, horn arm ${(c * 1000).toFixed(1)} x ${r.arm_w_mm} x ${r.arm_t_mm} mm, horn boss ${r.boss_od_mm}/${(eyeD.d * 1000).toFixed(0)} x ${r.boss_len_mm} mm, spar socket ${r.socket_wall_mm} mm wall x ${r.socket_len_mm} mm; ${mr.density_g_cm3} g/cm3 (replaces the carbon-plate rocker)`);
    // Round 7: the rocker's horn arm in bending (the coupler force at the rocker length) and the carbon spar in its
    // socket: the root flap moment (the coupler force times c) and the lead-lag moment as edge pressure on the tube,
    // judged at Rock West's transverse compressive strength (reference only: half), and the axial root force on the
    // Araldite 2011 bond (CFRP lap shear, typical: half).
    members.push({ name: `rocker arm ${side}`, kind: 'web', check: 'web', material: mr, dims: { b_mm: r.arm_w_mm, h_mm: r.arm_t_mm, lever_mm: +(c * 1000).toFixed(3) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }])], uses: { F: 'F' }, note: 'in-plane bending at the hub, the coupler force at the full rocker length' });
    const cf = materialOf(cx.catalog, 'composite-rockwest-t-rnd-393-transverse');
    members.push({ name: `spar root socket ${side}`, kind: 'seat', check: 'seat', material: cf, dims: { d_mm: r.spar_od_mm ?? 10, t_mm: r.socket_len_mm, crush: true }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +c.toFixed(6) }, { ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'carbon tube bearing on the socket: Rock West calls these tubes not good for crush resistance and publishes CLT estimates only (counted at half)' });
    members.push({ name: `spar root bond ${side}`, kind: 'bond', check: 'bond', material: { id: 'adhesive-araldite-2011', shear_MPa: materialOf(cx.catalog, 'adhesive-araldite-2011').shear_MPa }, dims: { d_mm: r.spar_od_mm ?? 10, L_mm: r.socket_len_mm }, sections: [sect('F', [{ ch: `root_x_${w}`, k: 1 }])], uses: { F: 'F' } });
    members.push({ name: `rocker hub ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(shD.d * 1000).toFixed(3), t_mm: +(hub.seat_m * 1000).toFixed(3), e_over_d: +(r.hub_od_mm / 2 / (shD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }, { ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'the wing root force and moment and the coupler force into the shoulder pin over the hub length' });
    // Horn pin: pressed in the horn boss, the coupler eye cantilevered beside it.
    const mh = mat(L.hornPin.material);
    // the eye's gap to the boss and the pin's length past the eye: params.layout (round 14), else 1 mm each (v21)
    const gapH = (d.params?.layout?.horn_gap_mm ?? 1) / 1000, pastH = (d.params?.layout?.horn_pin_past_mm ?? 1) / 1000;
    const lever = eyeD.B / 2 + gapH, pinLen = r.boss_len_mm / 1000 + eyeD.B + gapH + pastH;
    item(`horn pin ${side}`, mh, rodV(eyeD.d, pinLen), `${(eyeD.d * 1000).toFixed(0)} mm x ${(pinLen * 1000).toFixed(1).replace(/\.0$/, '')} mm; ${mh.density_g_cm3} g/cm3`);
    members.push({ name: `horn pin ${side}`, kind: 'pin', check: 'round', material: mh, dims: { d_mm: +(eyeD.d * 1000).toFixed(3) }, sections: [sect('M root', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }]), sect('V root', [{ ch: `coupler_${w}`, k: 1 }])], uses: { M: 'M root', V: 'V root' }, note: `the coupler eye at half its length plus ${+(gapH * 1000).toFixed(2)} mm from the boss face` });
    members.push({ name: `horn boss ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(eyeD.d * 1000).toFixed(3), t_mm: r.boss_len_mm, e_over_d: +(r.boss_od_mm / 2 / (eyeD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }])], uses: { F: 'F', M: 'M' } });
    // Round 17 (audit item 7, F5): a DIN 6799 washer in a groove at the pin's free end, against the coupler's horn
    // eye free face (clearance.js hornPinWasher draws it; params.linkage.retention.washer8mm, shared with the gear
    // shafts' end washers so they consolidate into one BOM line). Held: the coupler and its horn-end bushing sliding
    // off the pin the other way. Gated on retention.hornPinWasher too: see clearance.js pinGroove for why it is off
    // by default on this design's own spacing (both pins' free ends point in toward the opposite coupler).
    if (L.retention?.hornPinWasher && L.retention?.washer8mm) {
      const wC = byId(cx.catalog, L.retention.washer8mm).specs;
      const xHorn = w === 'R' ? cx.planes.couplerR : cx.planes.couplerL, O4 = cx.lay.fb.O4;
      const heldMass_kg = (couplerMassG(cx) / 2 + eye.mass_g) / 1000;
      members.push({ name: `horn pin washer ${side}`, kind: 'retainer', check: 'rated', dims: { rating_N: 1000 * Math.min(wC.load_columns.FN_kN, wC.load_columns.Fs_kN), rating_basis: `${L.retention.washer8mm}: the lower of FN ${wC.load_columns.FN_kN} kN (groove) and Fs ${wC.load_columns.Fs_kN} kN (washer) as tabulated (DIN 6799 convention; the groove's figure is for steel shafts and the pin is ${mh.id})` }, sections: [sect('F', inertialTerms(heldMass_kg, [xHorn, O4[1], O4[0]]))], uses: { F: 'F' }, note: 'the coupler and its horn-end bushing sliding off the pin, caught at the washer against the eye\'s free face' });
    }
    // Coupler link: one piece with its two eyes (no bonded end fitting: no published bond strength to carbon).
    members.push({ name: `coupler eye ${side}`, kind: 'end fitting', check: 'lug', material: mat(L.coupler.material), dims: { d_mm: +(eyeD.D * 1000).toFixed(3), D_mm: L.coupler.eye_od_mm, t_mm: +(eyeD.B * 1000).toFixed(3) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }])], uses: { F: 'F' }, note: `the eye round the ${eye.id} bushing (both ends alike)` });
    // Shoulder housings: a ring round each shoulder bushing.
    const mhs = mat(L.shoulderHousing.material);
    for (const sp of jt.supports) members.push({ name: `shoulder housing ${side} ${sp.name}`, kind: 'seat', check: 'lug', material: mhs, dims: { d_mm: +(shD.D * 1000).toFixed(3), D_mm: L.shoulderHousing.od_mm, t_mm: +(shD.B * 1000).toFixed(3) }, sections: [sect('F', sp.radial.map((t) => ({ ch: t.ch, k: t.k })))], uses: { F: 'F' } });
    item(`shoulder housings ${side}, 2 off`, mhs, 2 * ringV(L.shoulderHousing.od_mm / 1000, shD.D, shD.B), `rings ${L.shoulderHousing.od_mm}/${(shD.D * 1000).toFixed(0)} x ${(shD.B * 1000).toFixed(0)} mm round the ${shoulder.id} bushings; ${mhs.density_g_cm3} g/cm3 (mounting to the frame not counted here: in the 15% margin with fasteners)`);
}

/** The coupler links: mass, and the buckling and stress spec for M7's coupler check. */
function couplerLinks(cx) {
  const { d, L, mat, item, eyeD } = cx;
  // ---- Coupler links (mass and the buckling/stress spec for M7's coupler check)
  const cm = mat(L.coupler.material);
  const bL = (d.params.coupler_mm ?? 70) / 1000;
  const cV = rodV(L.coupler.body_d_mm / 1000, bL - L.coupler.eye_od_mm / 1000) + 2 * ringV(L.coupler.eye_od_mm / 1000, eyeD.D, eyeD.B);
  item('coupler links, 2 off', cm, 2 * cV, `each: ${L.coupler.body_d_mm} mm round body x ${((bL - L.coupler.eye_od_mm / 1000) * 1000).toFixed(0)} mm and two eyes ${L.coupler.eye_od_mm}/${(eyeD.D * 1000).toFixed(0)} x ${(eyeD.B * 1000).toFixed(0)} mm, one piece; ${cm.density_g_cm3} g/cm3 (replaces the carbon tube)`);
  const coupler = { id: cm.id, od_m: L.coupler.body_d_mm / 1000, id_m: 0, length_m: bL, E_GPa: cm.E_GPa, allowable_MPa: cm.yield_MPa, allowable_basis: cm.basis, kind: 'metal link' };
  return coupler;
}

/**
 * Length (mm) over which a plate seat holds its part: the plate's own thickness (round 8: the forward plate may be
 * thinner, L.plates.t_fwd_mm; the aft plate's seats are 'aft'), or the part's length B where L.plates.bossed names
 * the seat ('joint/support'), whose boss is as long as the part.
 */
export function plateSeatLength(plates, jointName, supportName, B_mm, flangeB_mm = 0) {
  const tp = supportName === 'aft' ? plates.t_mm : (plates.t_fwd_mm ?? plates.t_mm);
  const boss = (plates.bossed ?? []).includes(`${jointName}/${supportName}`);
  // v22 on (plates.flange 'face'): a flanged bushing's flange sits on the plate's face, so the seat holds its body only
  const body = journalBodyLength(plates, { B: B_mm, flangeB: flangeB_mm });
  return { plate_mm: tp, bossed: boss, length_mm: boss ? Math.max(tp, body) : tp };
}

const PLATE_JOINT = /^(crank shaft|gear shaft|motor shaft outboard)/;

/**
 * Round 11: each joint support with the length its housing actually holds it over (housing_mm), so a bushing is
 * judged on its supported length, not its full b1. Plate seats per plateSeatLength; the shoulder housings and coupler
 * eyes are rings as long as their part (wingSideMembers, couplerLinks), so they hold it over its full length.
 * Returns new joint objects; the input is not changed.
 */
export function jointHousings(joints, L, catalog) {
  return (joints ?? []).map((j) => {
    const pd = partDims(byId(catalog, j.part_id));
    const B = pd ? pd.B * 1000 : null;
    return { ...j, supports: j.supports.map((sp) => {
      if (!(B > 0)) return sp;
      if (PLATE_JOINT.test(j.name)) {
        // Without the design's plates the seat length is unknown: the support is left as it was (judged on b1).
        if (!L?.plates) return sp;
        const r = plateSeatLength(L.plates, j.name, sp.name, B, pd.flangeB * 1000);
        const body = journalBodyLength(L.plates, { B, flangeB: pd.flangeB * 1000 });
        return { ...sp, housing_mm: +Math.min(body, r.length_mm).toFixed(3), housing: r.bossed ? (body < B ? 'plate boss as long as the bushing body, flange on the plate face' : 'plate boss as long as the part') : `${r.plate_mm} mm plate, no boss` };
      }
      return { ...sp, housing_mm: +B.toFixed(3), housing: /^shoulder/.test(j.name) ? 'shoulder housing ring as long as the part' : 'coupler eye ring as long as the part' };
    }) };
  });
}

/** Gearbox plates and their seats: every shaft support in a plate, bossed where L.plates.bossed names it. */
function plateSeats(cx) {
  const { catalog, L, joints, members, mat, item } = cx;
  // ---- Gearbox plates and their seats: every shaft support in a plate of L.plates.material, a boss of the part's own
  // length round its outside diameter.
  const pm = mat(L.plates.material);
  item('gearbox plates, 2 off', pm, L.plates.area_m2 * (L.plates.t_mm / 1000), `${L.plates.area_m2.toExponential(2)} m2 (round 5's carbon cheek allowance) x ${L.plates.t_mm} mm; ${pm.density_g_cm3} g/cm3`);
  let bossV = 0, nBoss = 0;
  // Seats named in L.plates.bossed ('joint/support') have a boss of the part's own length and boss_wall_mm wall; the
  // others sit in the plate alone, judged as a ring of edge_mm round the part.
  const bossed = new Set(L.plates.bossed ?? []);
  for (const jt of joints.filter((q) => PLATE_JOINT.test(q.name))) {
    const prt = byId(catalog, jt.part_id);
    const pd = partDims(prt);
    for (const sp of jt.supports) {
      const key = `${jt.name}/${sp.name}`;
      const boss = bossed.has(key);
      const D = pd.D * 1000 + 2 * (boss ? L.plates.boss_wall_mm : L.plates.edge_mm);
      const { plate_mm: tp, length_mm: t } = plateSeatLength(L.plates, jt.name, sp.name, pd.B * 1000, pd.flangeB * 1000);
      if (boss && t > tp) { bossV += ringV(D / 1000, pd.D, t / 1000 - tp / 1000); nBoss++; }
      members.push({ name: `plate seat ${jt.name} ${sp.name}`, kind: 'seat', check: 'lug', material: pm, dims: { d_mm: +(pd.D * 1000).toFixed(3), D_mm: +D.toFixed(3), t_mm: +t.toFixed(3) }, sections: [sect('F', sp.radial.map((q) => ({ ch: q.ch, k: q.k })))], uses: { F: 'F' }, note: boss ? `a boss ${D.toFixed(1)} mm round the ${prt.id}, as long as the part` : `the plate alone, judged as a ${L.plates.edge_mm} mm ring round the ${prt.id}` });
    }
  }
  if (bossV > 0) item('gearbox plate bosses', pm, bossV, `bosses ${L.plates.boss_wall_mm} mm wall round the seats ${[...bossed].join(', ')}, as long as the bushing, beyond the plate; ${pm.density_g_cm3} g/cm3`, { count: nBoss });
}

const DS_WHY = 'Duracon gear on a plain bore: KHK advises against secondary operations on DS gears and publishes no keyway, set screw or adhesive fixing; Polyplastics M90-44 data could not be read; KHK DSF clamp hubs publish a reference slipping torque without a basis; Loctite 638 is not for plastics. No published rating for its torque to the shaft.';
/** A load path without a published rating: it fails M7 (SPEC ruling after round 6). */
function unrated(name, kind, why, terms) {
  return { name, kind, check: 'unrated', why, material: null, dims: {}, sections: [sect('T', terms)], uses: { T: 'T' } };
}

/**
 * Round 10 (SPEC ruling "RC practice for joints no one can rate"): how a gear without a maker keyway is held to its
 * shaft, as a member M7 reports as "unrated, RC practice" with its peak torque, within any figure the maker publishes:
 *  - a KHK F-Loc clamp hub (DSF): within KHK's reference slipping torque (specs.reference_slipping_torque_nm) at full
 *    value, the cap screw at KHK's standard tightening torque;
 *  - a maker-tapped set screw: within the holding torque the design cites for it (fixings.set_screw), if any;
 *  - a plain bore: bonded with the design's adhesive for its material (fixings.bond.pom / .steel) only if its maker states it bonds the
 *    gear's material; the torque within the bond's published shear on that material (tau pi d L d / 2) when given.
 * Anything else (no stated method, an adhesive its maker does not state for the material) stays unrated.
 * d_mm: the bore; terms: the fixing's torque terms.
 */
export function gearFixing(name, catalog, gearId, fixings, d_mm, terms) {
  const g = byId(catalog, gearId), s = g.specs ?? {};
  const base = { name, kind: 'gear fixing', check: 'rc practice', material: null, dims: { d_mm }, sections: [sect('T', terms)], uses: { T: 'T' } };
  if (/F-Loc/i.test(s.bore_type ?? '')) return flocFixing(base, g);
  // A maker-tapped set screw ("tapped for M4 set screw ..."); "plain bore, no keyway or set screw" is not one.
  if (/^tapped\b.*set screw/i.test(s.fastening ?? '')) return { ...base, rc: { method: `maker-tapped set screw (${s.fastening})`, published_Nm: Number.isFinite(fixings?.set_screw?.holding_Nm) ? fixings.set_screw.holding_Nm : null, published_basis: fixings?.set_screw?.basis ?? null } };
  return bondFixing(base, catalog, g, fixings?.bond, d_mm) ?? unrated(name, 'gear fixing', DS_WHY, terms);
}

/** A KHK F-Loc clamp hub: within KHK's reference slipping torque at full value, the screw at its standard torque. */
function flocFixing(base, g) {
  const s = g.specs, slip = s.reference_slipping_torque_nm;
  const screw = /M[\d.]+/.exec(s.bore_type)?.[0] ?? 'cap screw';
  const tq = Number.isFinite(s.screw_tightening_torque_nm) ? ` ${s.screw_tightening_torque_nm} N m` : '';
  const pub = Number.isFinite(slip);
  return { ...base, rc: { method: `KHK F-Loc clamp hub (${screw} at KHK's standard tightening torque${tq})`, published_Nm: pub ? slip : null, published_basis: pub ? `KHK reference slipping torque ${slip} N m (${g.source_url})` : null }, note: 'clamp hub: no rating with a basis; KHK publishes a reference slipping torque, which binds at full value' };
}

/** The gear material's bond class: acetal (POM) or steel; null for anything else. */
export function bondClass(material) {
  if (/duracon|acetal|polyacetal/i.test(material ?? '')) return 'pom';
  return /S45C|SCM|steel|SUS/i.test(material ?? '') ? 'steel' : null;
}

/**
 * A plain bore bonded with the design's adhesive for the gear's material (bond: { pom, steel } adhesive ids), only if
 * its maker states it for that material; within its published shear over the bore and face (tau pi d L d / 2). Null
 * when no such adhesive is stated.
 */
function bondFixing(base, catalog, g, bond, d_mm) {
  const s = g.specs, mat = bondClass(s.material);
  const adh = mat && bond?.[mat] ? byId(catalog, bond[mat]) : null;
  const sub = adh?.specs?.substrates?.[mat];
  if (!sub?.maker_states_suitable) return null;
  const L = (s.face_width_mm ?? s.hub_width_mm ?? 0) / 1000;
  const lim = Number.isFinite(sub.shear_MPa) ? sub.shear_MPa * 1e6 * Math.PI * (d_mm / 1000) * L * (d_mm / 2000) : null;
  const basis = lim === null ? null : `${sub.shear_MPa} MPa x pi x ${d_mm} x ${(L * 1000).toFixed(1)} mm x ${d_mm} / 2 (${sub.basis ?? adh.source_url})`;
  return { ...base, dims: { d_mm, L_mm: L * 1000 }, rc: { method: `bonded with ${adh.name} on ${mat === 'pom' ? 'acetal' : 'steel'} (plain bore)`, published_Nm: lim, published_basis: basis } };
}
/** The gear's keyway side of a key: its depth from the catalogue fastening text, its material as KHK states it. */
function keyHubSide(name, catalog, k, gearId, dSeat_mm, torqueTerms) {
  const g = byId(catalog, gearId);
  const depth = +(/x\s*(\d+(?:\.\d+)?)/.exec(g.specs?.fastening ?? '')?.[1] ?? NaN);
  const mid = /SCM/.test(g.specs?.material ?? '') ? 'metal-khk-scm415' : 'metal-khk-s45c';
  const gm = materialOf(catalog, mid);
  return { name, kind: 'key', check: 'key', material: materialOf(catalog, k.material), side_material: gm, dims: { d_mm: dSeat_mm, b_mm: k.b_mm, h_mm: depth, L_mm: k.L_mm }, sections: [sect('T', torqueTerms)], uses: { T: 'T' }, note: `bearing on the ${gearId} keyway (${depth} mm deep), ${gm.basis}` };
}

/** A parallel key between a gear's keyway and a shaft or seat of diameter dSeat_mm, of the shaft's material side. */
function keyMember(name, catalog, k, sideMat, dSeat_mm, torqueTerms, items) {
  const km = materialOf(catalog, k.material);
  items.push({ name, material_id: km.id, material: km.name, mass_g: gOf(km, (k.b_mm / 1000) * (k.h_mm / 1000) * (k.L_mm / 1000)), basis: `${k.b_mm} x ${k.h_mm} x ${k.L_mm} mm; ${km.density_g_cm3} g/cm3`, source_url: km.source_url });
  return { name, kind: 'key', check: 'key', material: km, side_material: sideMat, dims: { d_mm: dSeat_mm, b_mm: k.b_mm, h_mm: k.shaft_depth_mm, L_mm: k.L_mm }, sections: [sect('T', torqueTerms)], uses: { T: 'T' }, note: 'shear and bearing on the shaft keyseat; the gear\'s keyway is KHK\'s (SCM415, no strength published in what was read)' };
}
