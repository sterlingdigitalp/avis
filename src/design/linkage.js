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
import { fastenerGeometry } from './completeness.js';
import { ACTUATOR_BOND } from './interfaces.js'; // round 18: the encoder actuator's bond member

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
  // round 17b (retain2, audit item 5): the motor's outboard bearing bonded in its seat; its inner ring slides on the motor
  // shaft (a non-locating support: the motor's own bearings locate that shaft), so the bond carries the bearing's inertia
  // and, at most, the inner ring's friction drag on the shaft (an assumed mu 0.3 on the radial load: no source)
  const jo = cx.jointByName('motor shaft outboard');
  if (jo && L.retention?.bearingBonds?.['motor shaft outboard']) {
    const p = byId(catalog, jo.part_id), sp = jo.supports[0], pos = [sp.x_m, lay.train.shafts[lay.n][1], lay.train.shafts[lay.n][0]];
    cx.members.push(...bearingBondMembers(cx, 'motor shaft outboard', jo, [...inertialTerms(p.mass_g / 1000, pos), ...scale(sp.radial, 0.3)]));
  }
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
  // round 17b (retain2): every ring the drawing places, counted by catalogue id (clearance.js draws the same set)
  const uses = new Map(), use = (id, what) => { if (id) uses.set(id, [...(uses.get(id) ?? []), what]); };
  const shafts = d.params.joints.shafts ?? [];
  if (pinWasher(catalog, Ret, 'horn')) { use(Ret.washer8mm, 'horn pin right'); use(Ret.washer8mm, 'horn pin left'); }
  if (pinWasher(catalog, Ret, 'crank')) use(Ret.washer8mm, 'crank pin');
  for (let j = 1; j < lay.n; j++) {
    const isBearing = byId(catalog, shafts[j - 1] ?? shafts.at(-1)).category === 'bearing';
    for (const end of ['aft', 'forward']) use(shaftEndRetainer(Ret, j, end, isBearing), `gear shaft ${j} ${end}`);
  }
  use(Ret.crankCirclip, 'crank shaft forward');
  for (const [id, where] of uses) {
    const w = byId(catalog, id), n = where.length;
    items.push({ name: `${w.name}, ${n} off`, material_id: w.id, material: w.name, mass_g: n * w.mass_g, basis: `${n} x ${w.mass_g} g each (${w.specs.mass_basis}): ${where.join(', ')}`, source_url: w.source_url, count: n });
  }
}

/**
 * Round 17b (retain2): the retaining ring at one end of gear shaft j (clearance.js gearParts draws it, gearShaftMembers
 * rates it, retentionBom counts it): a bushing shaft's end takes retention.forwardCirclip[j] (a DIN 471 circlip, whose
 * groove and land are shorter than the washer's) at its forward end if stated, else retention.washer8mm unless that end
 * is excluded (washer8mmForwardExclude); a bearing shaft takes retention.gearShaft3Circlip at its aft end if stated.
 * -> catalogue id or null.
 */
export function shaftEndRetainer(Ret, j, end, isBearing) {
  if (!Ret) return null;
  if (isBearing) return end === 'aft' ? Ret.gearShaft3Circlip ?? null : null;
  if (end === 'aft') return Ret.washer8mm ?? null;
  return Ret.forwardCirclip?.[j] ?? ((Ret.washer8mmForwardExclude ?? []).includes(j) ? null : Ret.washer8mm ?? null);
}

/** Round 17b (retain2): a DIN 6799 washer's or DIN 471 circlip's rating, the lowest of its table's load columns. */
export function ringRating(id, specs, shaftMat) {
  if (/^washer-din6799/.test(id)) return washerRating(id, specs, shaftMat);
  const c = specs.load_columns, cols = [['FN', c.FN_kN], ['FR', c.FR_kN], ['FRg', c.FRg_kN]].filter(([, v]) => Number.isFinite(v));
  const [nm, kN] = cols.reduce((a, b) => (b[1] < a[1] ? b : a));
  return { rating_N: 1000 * kN, rating_basis: `${id}: ${nm} ${kN} kN, the lowest of the table's ${cols.map(([k, v]) => `${k} ${v}`).join(', ')} kN (DIN 471 convention; the groove's FN is for steel shafts and this one is ${shaftMat})` };
}

/**
 * Round 17b (retain2): a LOCTITE 638 retaining-compound bond between a steel part (a gear's bore, a bearing ring) and an
 * aluminium one (a 7075 shaft, a 6061 seat), from the catalogue's TDS figures: ISO 10123 compressive shear on aluminium
 * pins and collars (17 MPa, the lower of the two substrates': steel 29), typical values counted at half, as the design's
 * other adhesives are. None on titanium or thermoplastics (the TDS publishes neither): -> null there.
 */
export function retainingBond(catalog, id, substrate) {
  if (!id || substrate !== 'aluminium') return null;
  const a = byId(catalog, id), t = a.specs?.compressive_shear_iso10123_72h_mpa?.aluminium_pins_collars;
  if (!Number.isFinite(t)) return null;
  return { id: a.id, shear_MPa: t / 2, basis: `${a.name}: ISO 10123 compressive shear, aluminium pins and collars, 72 h, ${t} N/mm2 typical (${a.source_url}), counted at half; a fit gap at or under the TDS's reference (none credited beyond it)` };
}

/** Round 17b (retain2): a bond member: a cylinder d x L of adhesive a under an axial force and a torque (members.js bondCheck). */
const bondMember = (name, a, d_mm, L_mm, F, T, note) => ({ name, kind: 'bond', check: 'bond', material: { id: a.id, shear_MPa: a.shear_MPa, basis: a.basis }, dims: { d_mm: +d_mm.toFixed(3), L_mm: +L_mm.toFixed(3) }, sections: [sect('F', F), ...(T.length ? [sect('T', T)] : [])], uses: { F: 'F', ...(T.length ? { T: 'T' } : {}) }, note });

/**
 * Round 17b (retain2): a keyed gear held axially on its shaft by LOCTITE 638 over its engaged bore (its face on a wheel
 * seat, or its face and hub on the shaft itself), under its own inertia (spur gears: no axial mesh force). On a 7075 shaft
 * the TDS's aluminium figure rates it (a 'bond' member); on the titanium crank no maker publishes one, so the crank's
 * wheel is held by RC practice (a gear on its shaft by a bond: SPEC Rulings after physics round 8), its peak force
 * reported. gear: the train's stage gear (id, face, hubW); onSeat: a wheel on its seat; shaftMat: the shaft's material.
 */
function gearBond(cx, name, gear, onSeat, shaftMat, pos) {
  const { catalog, L } = cx, p = byId(catalog, gear.id), bore = p.specs?.bore_mm;
  const Lb = ((gear.face ?? 0) + (onSeat ? 0 : gear.hubW ?? 0)) * 1000, F = inertialTerms(p.mass_g / 1000, pos);
  const a = retainingBond(catalog, L.retention.bond, /^metal-al-/.test(shaftMat.id) ? 'aluminium' : null);
  const note = `the ${gear.id}'s ${bore} mm bore over ${Lb.toFixed(1)} mm (${onSeat ? 'its face on the wheel seat' : 'its face and hub on the shaft'}), keyed for torque; held axially by LOCTITE 638 under its own inertia`;
  if (a) return bondMember(name, a, bore, Lb, F, [], `${note} (steel on 7075: the aluminium figure)`);
  const adh = byId(catalog, L.retention.bond);
  return { name, kind: 'gear fixing', check: 'rc practice', material: null, dims: { d_mm: bore, L_mm: +Lb.toFixed(3) }, sections: [sect('F', F)], uses: { F: 'F' }, rc: { method: `${adh.name} on the ${gear.id}'s bore over the ${shaftMat.id} seat (keyed for torque; the bond holds it axially): ${adh.manufacturer ?? 'the maker'} publishes no strength on this substrate (${adh.source_url})`, published_Nm: null, published_basis: null }, note: `${note}: SPEC ruling "RC practice for joints no one can rate" (a gear held to its shaft by a bond)` };
}

/**
 * Round 17b (audit item 7, F5): the retaining washer a coupler eye's pin takes at its free end
 * (params.linkage.retention.washer8mm, a DIN 6799 E-clip), read off its catalogue entry: groove width m, the minimum
 * land n beyond the groove, thickness s, outside and groove radii, and its tabulated loads; `past` (m + n) is how far
 * the pin runs on beyond the eye's free face to take it. `which` is 'horn' (retention.hornPinWasher) or 'crank'
 * (retention.crankPinWasher). -> { id, m, n, s, ro, ri, past, specs } (m) or null when that pin has no washer.
 */
export function pinWasher(catalog, Ret, which) {
  const on = which === 'crank' ? Ret?.crankPinWasher : Ret?.hornPinWasher;
  if (!on || !Ret?.washer8mm) return null;
  const w = byId(catalog, Ret.washer8mm).specs, mm = (v) => v / 1000;
  const m = mm(w.groove_width_m_mm), n = mm(w.min_groove_to_shaft_end_n_mm);
  return { id: Ret.washer8mm, m, n, s: mm(w.thickness_s_mm), ro: mm(w.outer_diameter_d3_mm) / 2, ri: mm(w.groove_diameter_d2_mm) / 2, past: m + n, specs: w };
}

/**
 * Where a rocker's horn boss and horn pin sit along x beside its coupler eye (centre xc, length B): the boss
 * params.layout.horn_gap_mm (1 mm, v21) forward of the eye, or round 14 aft of it (params.layout.horns[tag] 'aft'), the
 * pin from the boss's far face through the eye to params.layout.horn_pin_past_mm (1 mm, v21) beyond it, or (round 17b)
 * `pastOverride` (m) beyond it: a retaining washer's groove and land. Round 17b: with `hubX` (the rocker hub's [aft,
 * fwd], m) the boss ends there at most: the hub's ends are the running clearance off the aft shoulder housing and the
 * aft plate, and the boss (23 mm across, 19.6 mm from the hub's axis) overlaps both in the plane. -> { boss, pin, side } (m).
 */
export function hornStack(Ly, tag, xc, B, bossLen, pastOverride = null, hubX = null) {
  const gap = (Ly?.horn_gap_mm ?? 1) / 1000, past = pastOverride ?? (Ly?.horn_pin_past_mm ?? 1) / 1000;
  if ((Ly?.horns?.[tag] ?? 'fwd') === 'aft') {
    const b1 = xc - B / 2 - gap, b0 = hubX ? Math.max(b1 - bossLen, hubX[0]) : b1 - bossLen;
    return { boss: [b0, b1], pin: [b0, xc + B / 2 + past], side: 'aft' };
  }
  const b0 = xc + B / 2 + gap, b1 = hubX ? Math.min(b0 + bossLen, hubX[1]) : b0 + bossLen;
  return { boss: [b0, b1], pin: [xc - B / 2 - past, b1], side: 'fwd' };
}

/**
 * Round 17b: the rocker arm's x span: centred on the horn boss (v21), within both the boss and the hub so it joins
 * both, and no thicker than they overlap. -> { x: [aft, fwd], t } (m).
 */
export function rockerArm(H, hubX, t) {
  const lo = Math.max(H.boss[0], hubX[0]), hi = Math.min(H.boss[1], hubX[1]), tt = Math.min(t, hi - lo);
  const c = Math.min(hi - tt / 2, Math.max(lo + tt / 2, (H.boss[0] + H.boss[1]) / 2));
  return { x: [c - tt / 2, c + tt / 2], t: tt };
}

/**
 * Round 17b: one side's horn stack as drawn (clearance.js sideParts) and rated (wingSideMembers): the pin's washer if
 * fitted (the pin then runs on past its eye, and the boss and arm keep within the hub), the boss, pin and arm.
 * hub: rockerHub's result; xc: the coupler eye's centre (m). -> { H, arm, pw, boss_od_mm }.
 */
export function hornSide(Ly, catalog, L, hub, xc, eyeB, tag) {
  const R = L.rocker, pw = pinWasher(catalog, L.retention, 'horn');
  const H = hornStack(Ly, tag, xc, eyeB, R.boss_len_mm / 1000, pw ? pw.past : null, pw ? hub.x : null);
  // round 17b: a side's own boss diameter (rocker.boss_od_side_mm[tag]), else rocker.boss_od_mm
  return { H, boss_od_mm: R.boss_od_side_mm?.[tag] ?? R.boss_od_mm, arm: pw ? rockerArm(H, hub.x, R.arm_t_mm / 1000) : { x: [(H.boss[0] + H.boss[1]) / 2 - R.arm_t_mm / 2000, (H.boss[0] + H.boss[1]) / 2 + R.arm_t_mm / 2000], t: R.arm_t_mm / 1000 }, pw };
}

/** Round 17b: the crank pin's aft end (m): past the left eye's free face by a retaining washer's groove and land, else 1 mm. */
export const crankPinEnd = (planes, eyeB, pw) => planes.couplerL - eyeB / 2 - (pw ? pw.past : 0.001);

/** The one-piece crank (shaft, web and pin) of L.crank.material, so torque and pin moment cross no joint. */
function crankMembers(cx) {
  // ---- Crank: one piece (shaft, web and pin) of L.crank.material, so torque and pin moment cross no joint.
  const { d, catalog, L, lay, planes, plateT, members, items, mat, item, jointByName, eyeD } = cx;
  const { train, n, meshPerNm } = lay;
  {
    const c = L.crank, m = mat(c.material);
    const od = c.shaft_od_mm / 1000, id = (c.shaft_id_mm ?? 0) / 1000, tw = c.web_t_mm / 1000, bw = c.web_b_mm / 1000, dp = eyeD.d;
    const xw = planes.crankArm - tw / 2; // the web's aft face
    const pinL = xw - crankPinEnd(planes, eyeD.B, pinWasher(catalog, L.retention, 'crank')); // round 17b: to its washer's land
    const lenS = (planes.crankEnd ?? planes.plateFwd + plateT / 2 + 0.001) - (planes.crankArm + tw / 2); // round 14: to an encoder stub
    const wheel = train.stages[n - 1].wheel;
    const seatL = (wheel.face ?? 0.01);
    const v = ringV(od, id, lenS) + bw * tw * (d.drive.a + (od + dp) / 2) + rodV(dp, pinL) + (c.seat_od_mm ? ringV(c.seat_od_mm / 1000, od, seatL) : 0);
    // round 17b (retain2): everything that slides with the crank shaft axially (the crank, its wheel, the encoder's
    // actuator, both couplers and their four eye bushings), held between the web on the aft flange and the forward circlip
    const heldCrank_kg = (gOf(m, v) + byId(catalog, wheel.id).mass_g + (planes.encoder ? byId(catalog, d.params.parts?.encoderMagnet ?? 'sensor-rls-orbis-actuator-10mm').mass_g : 0) + couplerMassG(cx) + 4 * cx.eye.mass_g) / 1000;
    if (c.web?.separate) crankSplit(cx, { c, m, od, id, tw, bw, dp, pinL, lenS, wheel, seatL, heldCrank_kg });
    else item('crank: shaft, web and pin in one piece', m, v, `${c.shaft_od_mm}${id ? `/${c.shaft_id_mm}` : ''} mm shaft x ${(lenS * 1000).toFixed(1)} mm, web ${c.web_b_mm} x ${c.web_t_mm} mm over the ${(d.drive.a * 1000).toFixed(0)} mm throw, ${(dp * 1000).toFixed(0)} mm pin x ${(pinL * 1000).toFixed(1)} mm${c.seat_od_mm ? `, ${c.seat_od_mm} mm wheel seat x ${(seatL * 1000).toFixed(0)} mm (the ${wheel.id}, ${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
    crankShaftPinMembers(cx, { c, m, dp, xw, wheel });
    crankRetentionMembers(cx, m, wheel, heldCrank_kg);
  }
}

/** The crank shaft, its pin (and the pin's washer), its web and the crank wheel's key, as M7 members (split out of crankMembers). */
function crankShaftPinMembers(cx, { c, m, dp, xw, wheel }) {
  const { catalog, L, lay, planes, members, items, jointByName, eyeD } = cx;
  const { n, meshPerNm } = lay, d = cx.d;
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
  // round 17b (audit item 7): the crank pin's washer, holding both couplers' crank ends (the right one stops on the
  // left), and the pin at its groove
  const pwC = pinWasher(catalog, L.retention, 'crank');
  if (pwC) members.push(...pinWasherMembers(cx, 'crank pin washer', 'crank pin', pwC, m, (couplerMassG(cx) + 4 * cx.eye.mass_g) / 1000, [planes.couplerL, lay.fb?.O2?.[1] ?? 0, lay.fb?.O2?.[0] ?? 0], [{ ch: 'coupler_R', k: 1 }, { ch: 'coupler_L', k: 1 }], 'both coupler links and their four bushings sliding aft off the pin (the right eye stops on the left), caught at the washer against the left crank eye\'s bushing flange', (cx.eye.specs.d3_mm ?? eyeD.D * 1000)));
  members.push({ name: 'crank web', kind: 'web', check: 'web', material: m, dims: { b_mm: c.web_b_mm, h_mm: c.web_t_mm, lever_mm: +(d.drive.a * 1000).toFixed(3) }, sections: [sect('M out of plane', Mpin), sect('F in plane', [{ ch: 'coupler_R', k: 1 }, { ch: 'coupler_L', k: 1 }])], uses: { M: 'M out of plane', F: 'F in plane' }, note: 'at the shaft root: the pin moment out of the web plane plus the coupler forces at the full throw in it' });
  if (c.key) members.push(keyMember(`key, crank wheel ${wheel.id}`, catalog, c.key, m, c.seat_od_mm, [{ ch: 'crank_torque', k: 1 }], items));
  if (c.key) members.push(keyHubSide(`key, crank wheel ${wheel.id}, gear side`, catalog, c.key, wheel.id, c.seat_od_mm, [{ ch: 'crank_torque', k: 1 }]));
}

/**
 * Round 17b (retain2, audit items 3 and 4): the crank shaft held axially between the web on the aft bushing's flange
 * (forward) and a DIN 471 circlip at the forward bushing's flange (aft, retention.crankCirclip), both flanges' rings
 * on their plates' outer faces under the held inertia; its wheel held on its seat by LOCTITE 638 (gearBond: the seat
 * is titanium, so RC practice).
 */
function crankRetentionMembers(cx, m, wheel, held_kg) {
  const { catalog, L, lay, planes, members, jointByName } = cx, Ret = L.retention;
  if (!Ret) return;
  const O2 = lay.fb?.O2 ?? [0, 0], F = inertialTerms(held_kg, [planes.stage[lay.n - 1], O2[1], O2[0]]);
  const jp = byId(catalog, jointByName('crank shaft').part_id), jd0 = journalDims(jp.specs);
  if (Ret.crankCirclip) {
    members.push({ name: 'crank shaft circlip forward', kind: 'retainer', check: 'rated', dims: ringRating(Ret.crankCirclip, byId(catalog, Ret.crankCirclip).specs, m.id), sections: [sect('F', F)], uses: { F: 'F' }, note: 'the crank shaft, its wheel, the actuator and both couplers sliding aft, caught at the circlip against the forward bushing\'s flange' });
    members.push({ name: 'crank shaft bushing flanges, axial thrust', kind: 'seat', check: 'flange', material: { id: jp.id, static_surface_pressure_mpa: jp.specs.static_surface_pressure_mpa }, dims: { D_mm: jd0.flangeD * 1000, d_mm: jd0.D * 1000 }, sections: [sect('F', F)], uses: { F: 'F' }, note: `the circlip's (aft) or the web's (forward) thrust into a ${jp.id} flange's ring on its plate, at its published static surface pressure; both flanges alike` });
  }
  if (Ret.bond) members.push(gearBond(cx, `${wheel.id} on the crank, axial bond`, wheel, true, m, [planes.stage[lay.n - 1], O2[1], O2[0]]));
  // round 18: the encoder's actuator bonded on its land (actuatorBond)
  const ab = planes.encoder ? actuatorBond(L) : null;
  if (ab) members.push(actuatorBondMember(cx, ab, m, O2));
}

/**
 * Round 18 (round brief item 4): the crank encoder's actuator (RLS BA100: anodised aluminium, a 10 mm H7 bore) on a rated
 * fixing. RLS holds it by three DIN 913 set screws, which no ruling covers (RC practice: a gear, pulley or coupling only;
 * maker's part: never a joint between our part and a bought part). It is bonded instead with 3M DP490, whose data sheet
 * publishes overlap shear on both substrates (sandblasted titanium 2901 psi, sandblasted aluminium 3336 psi; the lower
 * counted at half) and its best bond line (3-5 mil). An H7 bore on RLS's g6 shaft leaves 2.5-14.5 um, far thinner, so
 * the crank's stub is turned to a land under the actuator (land_d_mm, tolerance land_tol_mm) whose radial gap to the H7
 * bore lies in that bond line at both extremes; the set screws centre the actuator while the bond cures (RLS allows the
 * sensor 0.5 mm radial displacement, BRD01). Its position and ride height are unchanged. L.retention.actuatorBond: null
 * leaves the set screws alone (the ledger refuses them).
 */
export const ACTUATOR_BOND_DEFAULT = { adhesive: 'adhesive-3m-dp490', land_d_mm: 9.8, land_tol_mm: [-0.02, 0], bore_tol_mm: [0, 0.015], bore_tol_basis: "ISO 286-2: H7 over 6 up to 10 mm is +0 / +15 um (RLS BRD01: 'ID tolerances are H7')", set_screws: 3, set_screw_hole_mm: 3 };
export function actuatorBond(L) {
  if (!L?.retention || L.retention.actuatorBond === null) return null;
  return { ...ACTUATOR_BOND_DEFAULT, ...(L.retention.actuatorBond ?? {}) };
}
/** The adhesive's row for a material: titanium or aluminium by the material's id or its stated material, else null. */
const substrateOf = (text) => (/\bti\b|ti-\d|titanium/i.test(text) ? 'titanium' : /\bal\b|al-\d|alumin/i.test(text) ? 'aluminium' : null);

/**
 * The actuator's bond (gapBondCheck): its torque is the actuator's inertia times the crank's angular acceleration, which
 * the flight model's crank equation bounds: Jcrank |thDD| <= |motor torque at the crank| + |crank torque| + friction
 * (flight.js: Jcrank = rotor J G^2 + J_crank). The motor's part is held at its largest possible value, the pack at full
 * charge across the winding alone (V / Rm, through G / Kv over the gear efficiency), as a stated constant (m.fixed); the
 * crank torque's part rides on its channel. The actuator's own J is bounded by its whole mass at its outside radius.
 * Axially: its inertia (sf_origin, wdot, w2). RLS publishes no magnet pull; none is counted (stated).
 */
/** The bond's allowable: the adhesive's lower published row of the shaft's and the bore's substrates, at half. */
function bondAllowable(adh, shaftMat) {
  const sub = adh.specs.substrates ?? {}, sS = substrateOf(shaftMat.id), sB = substrateOf('anodised aluminium (RLS BRD01: "Actuator material: Anodised aluminium")');
  const fS = sub[sS]?.shear_MPa, fB = sub[sB]?.shear_MPa; // a row the maker does not publish: NaN, so the bond is unrated
  return { id: `${adh.id} (${sS} shaft, ${sB} bore)`, shear_MPa: Math.min(fS, fB) / 2, basis: `${adh.id}: the lower of its published rows on the two substrates (${sS} ${fS} MPa, ${sB} ${fB} MPa; typical), counted at half` };
}
/** The radial gap [least, most] (mm) of the bore (bore_tol_mm) on the land (land_tol_mm). */
const bondGap = (ab, bore) => [(bore + ab.bore_tol_mm[0] - ab.land_d_mm - ab.land_tol_mm[1]) / 2, (bore + ab.bore_tol_mm[1] - ab.land_d_mm - ab.land_tol_mm[0]) / 2].map((g) => +g.toFixed(5));
/**
 * A part of inertia Ja turning with the crank: its torque per crank N m (Ja / Jcrank) and its constant part (Ja / Jcrank
 * times the motor's largest torque at the crank, the pack at full charge across the winding alone, plus friction).
 */
function crankAccelTorque(d, Ja) {
  const mo = d.motor, G = d.drive.gearRatio, eta = d.drive.gearEfficiency ?? 0.85;
  const Jc = mo.J_kg_m2 * G * G + (d.drive.J_crank_kg_m2 ?? 0), Vf = d.battery.cells * (4.2 + (d.battery.cell_shift_V ?? 0));
  const tauC = (G * (Vf / mo.R)) / mo.kv_rad_per_v_s / eta + (d.drive.crankFriction_Nm ?? 0);
  return { k: +(Ja / Jc).toExponential(6), T: +((Ja * tauC) / Jc).toExponential(6), basis: `the motor's largest torque at the crank: ${Vf.toFixed(2)} V / ${mo.R} ohm (the pack at full charge across the winding alone) x G ${G.toFixed(3)} / Kv ${mo.kv_rad_per_v_s.toFixed(3)} / ${eta} = ${tauC.toFixed(2)} N m, on the crank's ${Jc.toExponential(4)} kg m2 (the flight model's, its rotor J estimated), times ${Ja.toExponential(3)} kg m2` };
}
function actuatorBondMember(cx, ab, shaftMat, O2) {
  const { d, catalog, planes } = cx, e = planes.encoder;
  const actP = byId(catalog, d.params.parts?.encoderMagnet ?? 'sensor-rls-orbis-actuator-10mm'), act = actP.specs, adh = byId(catalog, ab.adhesive);
  const Le = (Math.min(e.actuator[1], planes.crankEnd ?? e.actuator[1]) - e.actuator[0]) * 1000;
  const Leff = Le - (ab.set_screws * (ab.set_screw_hole_mm / 2) ** 2) / ab.land_d_mm; // the set screws' holes off the bonded area
  const mA = actP.mass_g / 1000, ac = crankAccelTorque(d, mA * (act.od_mm / 2000) ** 2);
  return {
    name: ACTUATOR_BOND, kind: 'bond', check: 'gap bond',
    material: bondAllowable(adh, shaftMat),
    dims: { d_mm: ab.land_d_mm, L_mm: +Leff.toFixed(3), land_d_mm: ab.land_d_mm, bore_d_mm: act.inner_diameter_mm, gap_mm: bondGap(ab, act.inner_diameter_mm), gap_limits_mm: adh.specs.bond_line_max_strength_mm, gap_basis: `${act.inner_diameter_mm} mm bore (${ab.bore_tol_basis}) on the ${ab.land_d_mm} mm land (${ab.land_tol_mm.join('/')} mm); ${adh.specs.bond_line_basis}` },
    sections: [sect('T', [{ ch: 'crank_torque', k: ac.k }]), sect('F', inertialTerms(mA, [(e.actuator[0] + e.actuator[1]) / 2, O2[1], O2[0]]))], uses: { T: 'T', F: 'F' },
    fixed: { T: ac.T },
    fixed_basis: `${ac.basis}: the actuator's own J, ${actP.mass_g} g at ${act.od_mm / 2} mm`,
    note: `the ${actP.id} bonded with ${adh.id} over ${Leff.toFixed(2)} mm of the ${ab.land_d_mm} mm land (its ${Le.toFixed(1)} mm bore less its ${ab.set_screws} set-screw holes); the crank's angular acceleration on the actuator's inertia as torque, its inertia axially; RLS publishes no magnet pull (none counted); the set screws centre it, not credited; the bore and the land sandblasted as the data sheet's specimens were (the bore's anodising off under the bond: an assembly step RLS does not address)`,
  };
}

/**
 * Round 16 (SPEC "Completeness and assembly order"; crank.web.separate): the one-piece crank could not go together (its
 * web aft of the aft plate and its wheel seat inside the box, neither through the plate's 10 mm bushing). The web and
 * its pin are one part, keyed on a solid stub at the shaft's aft end (a parallel key over the web's thickness, both
 * sides rated by the key rule) and held there axially with LOCTITE 638, whose maker publishes no strength on titanium:
 * that axial fixing is unrated, RC practice (SPEC ruling), reported with the torque its joint carries. The shaft keeps
 * its wheel seat and goes into the aft plate's bushing from forward. The shaft is checked again at the keyseat's root.
 */
function crankSplit(cx, { c, m, od, id, tw, bw, dp, pinL, lenS, wheel, seatL, heldCrank_kg = 0 }) {
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
  // round 18: the actuator's bond land turned on the forward stub (actuatorBond), off the shaft's volume
  const ab = planes.encoder ? actuatorBond(L) : null, landL = ab ? Math.max(0, Math.min(planes.encoder.actuator[1], planes.crankEnd ?? planes.encoder.actuator[1]) - planes.encoder.actuator[0]) : 0;
  const landV = ab ? ringV(od, ab.land_d_mm / 1000, landL) : 0;
  item('crank shaft', m, ringV(od, id, lenS - solidEnd) - landV + PI4 * od * od * (tw - lip + solidEnd) + (c.seat_od_mm ? ringV(c.seat_od_mm / 1000, od, seatL) : 0), `${c.shaft_od_mm}${id ? `/${c.shaft_id_mm}` : ''} mm shaft x ${(lenS * 1000).toFixed(1)} mm, a solid ${c.shaft_od_mm} mm stub x ${((tw - lip) * 1000).toFixed(1)} mm under the web (keyseated${screw ? `, its end tapped for the web's axial screw, solid ${(solidEnd * 1000).toFixed(1)} mm on beyond the web for its thread` : ''})${c.seat_od_mm ? `, ${c.seat_od_mm} mm wheel seat x ${(seatL * 1000).toFixed(0)} mm (the ${wheel.id}, ${boreText(catalog, wheel.id)}, keyed)` : ''}${ab ? `, turned to a ${ab.land_d_mm} mm land x ${(landL * 1000).toFixed(1)} mm under the encoder's actuator (its bond)` : ''}; ${m.density_g_cm3} g/cm3`);
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
  crankWebScrewMembers(cx, { c, m, W, screw, dS, webV: gOf(m, webV + rodV(dp, pinL)), heldCrank_kg });
}

/**
 * Round 17: the web held axially by one screw into the stub's tapped end, its head on the end lip (ISO 898-1): the screw,
 * its thread in the stub and the lip under its head (split out of crankSplit in round 17b). webV: the web and pin's mass (g).
 */
function crankWebScrewMembers(cx, { c, m, W, screw, dS, webV, heldCrank_kg }) {
  const { catalog, planes, members, items } = cx;
  // round 17b: the head and length as completeness.js reads them (an ISO 4762 entry states them in its mass basis only)
  const sg = fastenerGeometry(screw.specs ?? {}, screw.id), headD_mm = sg.head_d * 1000, len_mm = sg.L * 1000;
  // round 17: the web held axially by one screw into the stub's tapped end, its head on the end lip (ISO 898-1); the
  // web, its pin and the two coupler links that ride on the pin (their eyes retained on it) pull on it under the bird's
  // accelerations (inertial terms: every axis counted, conservative for the axial one)
  items.push({ name: `${screw.name}, 1 off`, material_id: screw.id, material: screw.name, mass_g: screw.mass_g, basis: `1 x ${screw.mass_g} g (${screw.specs.mass_basis})`, source_url: screw.source_url, count: 1 });
  // round 17b (retain2): with the web on the aft flange as the crank shaft's forward stop, the screw also holds the
  // shaft, its wheel and the actuator to the web: the held mass is the larger of the two sides (heldCrank_kg holds both)
  const fx = fastenerOf(catalog, W.axial.screw), mAx = Math.max((webV + couplerMassG(cx)) / 1000, heldCrank_kg);
  const rAx = [planes.crankArm, (cx.lay?.fb?.O2?.[1] ?? 0), 0];
  members.push({ name: 'crank web axial screw', kind: 'fasteners', check: 'bolts', fastener: fx, dims: { n: 1 }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `one ${W.axial.screw} into the stub's tapped end, in tension under ${(mAx * 1000).toFixed(1)} g (the web, pin and couplers pulling aft off the stub, or the shaft, its wheel and the actuator held forward by the web on the aft flange: the larger)` });
  // its thread in the titanium stub: stripping over the engaged length, the pitch cylinder's half (the usual area for a
  // thread in the softer part), against the stub's shear yield
  const pitch = +(/pitch ([\d.]+)/.exec(screw.specs.thread ?? '')?.[1] ?? (dS * 1000 >= 3 ? 0.5 : 0.4)), Le = Math.min(len_mm - (W.lip_mm ?? 0), c.web_t_mm - (W.lip_mm ?? 0) + (W.axial.solid_mm ?? 0)), d2 = dS * 1000 - 0.6495 * pitch;
  members.push({ name: 'crank web axial screw thread in the stub', kind: 'pin', check: 'pin-shear', material: m, dims: { d_mm: +Math.sqrt(4 * (Math.PI * d2 * Le / 2) / Math.PI).toFixed(4), planes: 1, Le_mm: +Le.toFixed(2), area_mm2: +(Math.PI * d2 * Le / 2).toFixed(3) }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `the internal thread in the ${m.id} stub over ${Le.toFixed(1)} mm engaged (the solid stub under the web and the shaft's solid end beyond it): shear area pi d2 Le / 2 = ${(Math.PI * d2 * Le / 2).toFixed(2)} mm2 (d2 the pitch diameter ${d2.toFixed(3)} mm), as a pin of that area` });
  const dPunch = W.axial.countersunk ? (headD_mm + dS * 1000) / 2 : headD_mm;
  members.push({ name: 'crank web end lip under the screw head', kind: 'web', check: 'pin-shear', material: m, dims: { d_mm: +Math.sqrt(4 * (Math.PI * dPunch * (W.lip_mm ?? 0)) / Math.PI).toFixed(4), planes: 1, area_mm2: +(Math.PI * dPunch * (W.lip_mm ?? 0)).toFixed(3) }, sections: [sect('F', inertialTerms(mAx, rAx))], uses: { F: 'F' }, note: `the ${W.lip_mm} mm lip punched out round the screw's head (${W.axial.countersunk ? `countersunk: the cone's mean diameter ${dPunch.toFixed(2)} mm` : `${dPunch} mm`}): shear area pi x that x t, as a pin of that area` });
}

/** The area (m2) of the convex hull of two circles of radii r1 and r2, centres a apart (the crank web with its hub). */
export function hullArea(r1, r2, a) {
  if (a <= Math.abs(r1 - r2)) return Math.PI * Math.max(r1, r2) ** 2;
  const phi = Math.asin((r1 - r2) / a), L = a * Math.cos(phi);
  // two tangent segments, the larger circle's arc (pi + 2 phi), the smaller's (pi - 2 phi)
  return L * (r1 + r2) + 0.5 * r1 * r1 * (Math.PI + 2 * phi) + 0.5 * r2 * r2 * (Math.PI - 2 * phi);
}

/**
 * Round 17b: a DIN 6799 washer's rating, the lowest of the load columns its table prints (FN, the groove's, at the
 * tabulated shaft diameter; Fs, the washer's; Fsg, the washer's against an abutment with the table's edge g: the eye
 * faces and flanges here state no edge break, so the lowest is taken). -> { rating_N, rating_basis }.
 */
export function washerRating(id, w, shaftMat) {
  const c = w.load_columns, cols = [['FN', c.FN_kN], ['Fs', c.Fs_kN], ['Fsg', c.Fsg_kN]].filter(([, v]) => Number.isFinite(v));
  const [nm, kN] = cols.reduce((a, b) => (b[1] < a[1] ? b : a));
  return { rating_N: 1000 * kN, rating_basis: `${id}: ${nm} ${kN} kN, the lowest of the table's ${cols.map(([k, v]) => `${k} ${v}`).join(', ')} kN (DIN 6799 convention; the groove's FN is for steel shafts and this one is ${shaftMat})` };
}

/**
 * Round 17b (audit item 7): a pin's retaining washer and the pin at its groove. The washer (check 'rated') against the
 * held parts' inertia (every axis counted); the pin's section at the groove (the washer's published groove diameter)
 * in shear under the whole eye force (by statics the groove, outside the eye, carries none: an upper bound).
 */
export function pinWasherMembers(cx, washer, pin, pw, mat, heldMass_kg, rPos, eyeForce, what, contactD_mm) {
  const eb = cx.eye, F = [sect('F', inertialTerms(heldMass_kg, rPos))];
  return [
    { name: washer, kind: 'retainer', check: 'rated', dims: washerRating(pw.id, pw.specs, mat.id), sections: F, uses: { F: 'F' }, note: what },
    // what it bears on: the eye bushing's iglidur end (its flange, or its body's end), at the published static surface
    // pressure over the annulus the washer covers (the 6061 eye face beyond it not credited)
    { name: `${washer} on the bushing's end`, kind: 'seat', check: 'flange', material: { id: eb.id, static_surface_pressure_mpa: eb.specs.static_surface_pressure_mpa }, dims: { D_mm: +Math.min(2 * pw.ro * 1000, contactD_mm).toFixed(3), d_mm: +(cx.eyeD.d * 1000).toFixed(3) }, sections: F, uses: { F: 'F' }, note: `the washer's thrust on the ${eb.id}'s end over ${(cx.eyeD.d * 1000).toFixed(0)}..${Math.min(2 * pw.ro * 1000, contactD_mm).toFixed(1)} mm, at its published static surface pressure` },
    { name: `${pin} at its washer's groove`, kind: 'pin', check: 'pin-shear', material: mat, dims: { d_mm: +(2 * pw.ri * 1000).toFixed(3), planes: 1 }, sections: [sect('F', eyeForce)], uses: { F: 'F' }, note: `the ${(2 * pw.ri * 1000).toFixed(1)} mm groove section in single shear under the whole eye force (outside the eye it carries none by statics: an upper bound)` },
  ];
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
    // round 17b (retain2): as drawn, its ends run on past the journals for their retaining rings' grooves and lands
    const reach = shaftRingReach(cx, j);
    const len = shaftLen + Math.max(0, far(kw, wheel), far(kp, train.stages[kp].pinion), reach.fwd) + reach.aft;
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
      else if (L.retention?.bond) members.push(gearBond(cx, `${gid} on gear shaft ${j}, axial bond`, k === kw ? wheel : train.stages[kp].pinion, k === kw && !!g.seat_od_mm, m, [planes.stage[k], lay.train.shafts[j][1], lay.train.shafts[j][0]]));
    }
    // round 17 (audit items 7, 8, F4): this shaft's own axial retention (clearance.js gearShaftRetention draws it):
    // a DIN 6799 washer at each end outside the bushing's flange, or a DIN 471 circlip at the aft end against the
    // aft bearing's inner ring; 6061 spacer tubes fill the gaps so the gears can no longer slide on their keys.
    if (L.retention) gearShaftRetentionMembers(cx, j, { g, m, v, wheel, kw, kp, jt });
}

/**
 * Round 17 (audit items 7, 8, F4), split out of gearShaftMembers in round 17b: gear shaft j's own axial retention
 * members and its spacers' BOM line. ctx: the shaft's line (g), material (m), drawn volume (v), its wheel and the stage
 * indices, its bearing joint (jt).
 */
function gearShaftRetentionMembers(cx, j, { g, m, v, wheel, kw, kp, jt }) {
  const { catalog, L, lay, planes, members } = cx;
  const { train } = lay;
  const Ret = L.retention, isBearing = byId(catalog, jt.part_id).category === 'bearing', pinion = train.stages[kp].pinion;
  const jd0 = journalDims(byId(catalog, jt.part_id).specs);
  // the held mass: this shaft's own mass, its wheel and pinion (every axis counted, inertialTerms); the shaft and its
  // bonded gears can only slide one way at a time, so each end's ring is checked against it in full
  const shPos = lay.train.shafts[j], heldMass_kg = (gOf(m, v) + byId(catalog, wheel.id).mass_g + byId(catalog, pinion.id).mass_g) / 1000;
  const rPos = [planes.stage[kw], shPos[1], shPos[0]], F = inertialTerms(heldMass_kg, rPos);
  const ends = ['aft', 'forward'].map((end) => [end, shaftEndRetainer(Ret, j, end, isBearing)]).filter(([, id]) => id);
  for (const [end, id] of ends) {
    const kind = /^circlip/.test(id) ? 'circlip' : 'washer';
    members.push({ name: `gear shaft ${j} ${kind} ${end}`, kind: 'retainer', check: 'rated', dims: ringRating(id, byId(catalog, id).specs, m.id), sections: [sect('F', F)], uses: { F: 'F' }, note: `the shaft and its bonded wheel and pinion sliding the other way, caught at the ${end} ${kind} against the ${jt.part_id} ${isBearing ? 'inner ring' : 'flange'}` });
  }
  if (!isBearing && ends.length) {
    const both = ends.length === 2;
    members.push({ name: `gear shaft ${j} bushing flange${both ? 's' : ` ${ends[0][0]}`}, axial thrust`, kind: 'seat', check: 'flange', material: { id: jt.part_id, static_surface_pressure_mpa: byId(catalog, jt.part_id).specs.static_surface_pressure_mpa }, dims: { D_mm: jd0.flangeD * 1000, d_mm: jd0.D * 1000 }, sections: [sect('F', F)], uses: { F: 'F' }, note: `an end ring's thrust into the ${jt.part_id} flange's ring on its plate, at its published static surface pressure (the same figure the bushing's own radial seat is judged against)${both ? '; both flanges alike' : '; the other end has no ring (stated rather than invented)'}` });
  }
  if (isBearing) members.push(...bearingBondMembers(cx, `gear shaft ${j}`, jt, F));
}

/**
 * Round 17b (retain2): a bearing's bonded rings (retention.bearingBonds['<joint>']: 'aft inner', 'aft outer', 'forward
 * outer', ...), each a LOCTITE 638 cylinder over the ring's width (its bore on the 7075 shaft, its outside in the 6061
 * seat) under the held inertia F; with an inner ring bonded, that bearing locates its shaft, so it also carries F
 * axially through its balls (ISO 76 static: P0 = Y0 Fa for a pure axial load, Y0 0.5 for a single-row deep groove
 * bearing, its C0 at FoS 1.5). -> members (none without a stated bond or adhesive).
 */
function bearingBondMembers(cx, jname, jt, F) {
  const { catalog, L } = cx, list = L.retention?.bearingBonds?.[jname] ?? [];
  const a = retainingBond(catalog, L.retention?.bond, 'aluminium'), p = byId(catalog, jt.part_id), dd = journalDims(p.specs);
  if (!a) return [];
  const out = [];
  for (const spec of list) {
    const [end, ring] = spec.split(' ');
    out.push(bondMember(`${jname} bearing ${end}, ${ring} ring bond`, a, (ring === 'inner' ? dd.d : dd.D) * 1000, dd.B * 1000, F, [], `the ${p.id}'s ${ring} ring ${ring === 'inner' ? 'on the 7075 shaft' : 'in its 6061 seat'} over its ${(dd.B * 1000).toFixed(0)} mm width, LOCTITE 638 (steel on aluminium: the aluminium figure); ${ring === 'inner' ? 'this bearing locates the shaft' : 'held in its seat both ways'}`));
    if (ring === 'inner') {
      const C0 = 1000 * p.specs.static_load_rating_kn, Y0 = 0.5;
      out.push({ name: `${jname} bearing ${end}, axial (ISO 76)`, kind: 'bearing', check: 'rated', dims: { rating_N: C0 / Y0, rating_basis: `ISO 76 static, P0 = X0 Fr + Y0 Fa, Y0 ${Y0} (single-row deep groove): the axial load at which Y0 Fa alone reaches C0 ${C0} N (${p.source_url}); the radial check (the bearing joint) is separate and Fa here is the held inertia only` }, sections: [sect('F', F)], uses: { F: 'F' }, note: `the shaft located by this bearing: its held inertia through the balls` });
    }
  }
  return out;
}

/** Round 17b (retain2): how far (m) gear shaft j's drawn ends run past the default (1 mm proud of each plate) for its
 * retaining rings' grooves and lands (clearance.js gearParts draws the same ends). -> { aft, fwd } (fwd may be < 0). */
function shaftRingReach(cx, j) {
  const { catalog, L, planes, plateT } = cx, Ret = L.retention;
  const jid = cx.d.params.joints.shafts[j - 1] ?? cx.d.params.joints.shafts.at(-1), isBearing = byId(catalog, jid).category === 'bearing';
  const jd0 = journalDims(byId(catalog, jid).specs), tA = L.plates.t_mm / 1000, tF = (L.plates.t_fwd_mm ?? L.plates.t_mm) / 1000;
  const land = (id) => { const s = byId(catalog, id).specs; return (s.groove_width_m_mm + s.min_groove_to_shaft_end_n_mm) / 1000; };
  const aId = shaftEndRetainer(Ret, j, 'aft', isBearing), fId = shaftEndRetainer(Ret, j, 'forward', isBearing);
  const aft = aId ? Math.max(0, planes.plateAft - plateT / 2 - 0.001 - (journalEdges(L.plates, 'aft', planes.plateAft, tA, jd0).outer - land(aId))) : 0;
  const fwd = fId ? journalEdges(L.plates, 'fwd', planes.plateFwd, tF, jd0).outer + land(fId) - (planes.plateFwd + plateT / 2 + 0.001) : 0;
  return { aft, fwd };
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
 * levers: { aft, forward, collar } } where the levers are each bushing centre's distance from the hub face beside it (the
 * shoulder pin's bending arms; the collar is the pin's own, so it does not shorten its arm) and, with a collar, the
 * forward bushing centre's distance from the collar's forward end (round 19: where the pin steps down from the
 * collar's diameter to its own, the arm of its own section; the collar's section takes the arm to the hub face).
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
  return { x: [a, b], relief, collar, seat_m: b - a - (relief ? relief[1] - relief[0] : 0), levers: { aft: a - (x0 - span / 2), forward: x0 + span / 2 - b, ...(collar ? { collar: x0 + span / 2 - collar[1] } : {}) } };
}

/**
 * One side's shoulder pin as M7 members (split out of wingSideMembers in round 19). Round 19 (sized to the FoS rule at
 * the sealed design's trims): the pin is a stepped shaft where its collar fills the forward gap (the collar is the pin's
 * own, one piece with it: clearance.js draws it as the pin's), so its own section is checked at the collar's forward end
 * (the forward bushing's arm there) and the collar's section at the hub face (the whole forward arm); with no collar the
 * pin's own section takes the whole arm, as before.
 */
function shoulderPinMembers(side, m, r, jt, hub, shD, pinLever) {
  const stepped = Number.isFinite(hub.levers.collar);
  const ownLever = (sp) => (sp.name === 'forward' && stepped ? hub.levers.collar : hub.levers[sp.name] ?? pinLever);
  const note = stepped ? 'the bushing reaction at its centre, measured from the hub face aft and from the collar\'s forward end forward (where the pin steps down to its own section)' : pinLever > shD.B / 2 + 1e-9 ? 'the bushing reaction at its centre, measured from the hub face' : 'the bushing reaction at half the bushing length from the hub face';
  const out = [{ name: `shoulder pin ${side}`, kind: 'pin', check: 'round', material: m, dims: { d_mm: +(shD.d * 1000).toFixed(3) }, sections: jt.supports.flatMap((sp) => [sect(`M ${sp.name}`, scale(sp.radial, ownLever(sp))), sect(`V ${sp.name}`, sp.radial.map((t) => ({ ch: t.ch, k: t.k })))]), uses: { M: jt.supports.map((sp) => `M ${sp.name}`), V: jt.supports.map((sp) => `V ${sp.name}`) }, note }];
  const spF = jt.supports.find((sp) => sp.name === 'forward');
  if (stepped && spF) out.push({ name: `shoulder pin ${side} at its collar`, kind: 'pin', check: 'round', material: m, dims: { d_mm: r.hub_collar_od_mm }, sections: [sect('M forward', scale(spF.radial, hub.levers.forward)), sect('V forward', spF.radial.map((t) => ({ ch: t.ch, k: t.k })))], uses: { M: 'M forward', V: 'V forward' }, note: `the pin's own ${r.hub_collar_od_mm} mm collar at the hub face: the forward bushing reaction at its centre, ${(hub.levers.forward * 1000).toFixed(2)} mm away` });
  return out;
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
    members.push(...shoulderPinMembers(side, m, r, jt, hub, shD, pinLever));
    // Rocker: hub on the shoulder pin, horn arm of the rocker length c, horn boss for the horn pin, spar socket.
    const c = linkageAt(lay.law, d.drive.ampNominal).c;
    const sparOD = (r.spar_od_mm ?? 10) / 1000;
    const relL = hub.relief ? hub.relief[1] - hub.relief[0] : 0;
    // round 17b: this side's horn stack as drawn (the boss and arm may be cut to the hub's length: hornSide)
    const hs = hornSide(d.params?.layout ?? {}, cx.catalog, L, hub, w === 'R' ? cx.planes.couplerR : cx.planes.couplerL, eyeD.B, w);
    const bossL_mm = +((hs.H.boss[1] - hs.H.boss[0]) * 1000).toFixed(3), armT_mm = +(hs.arm.t * 1000).toFixed(3);
    const vR = ringV(r.hub_od_mm / 1000, shD.d, hubL - relL) + (relL ? ringV(r.hub_relief_od_mm / 1000, shD.d, relL) : 0) + c * (r.arm_w_mm / 1000) * (armT_mm / 1000) + ringV(hs.boss_od_mm / 1000, eyeD.d, bossL_mm / 1000) + ringV(sparOD + 2 * r.socket_wall_mm / 1000, sparOD, r.socket_len_mm / 1000);
    item(`rocker ${side}`, mr, vR, `hub ${r.hub_od_mm}/${(shD.d * 1000).toFixed(0)} x ${(hubL * 1000).toFixed(0)} mm, horn arm ${(c * 1000).toFixed(1)} x ${r.arm_w_mm} x ${armT_mm} mm, horn boss ${hs.boss_od_mm}/${(eyeD.d * 1000).toFixed(0)} x ${bossL_mm} mm, spar socket ${r.socket_wall_mm} mm wall x ${r.socket_len_mm} mm; ${mr.density_g_cm3} g/cm3 (replaces the carbon-plate rocker)`);
    // Round 7: the rocker's horn arm in bending (the coupler force at the rocker length) and the carbon spar in its
    // socket: the root flap moment (the coupler force times c) and the lead-lag moment as edge pressure on the tube,
    // judged at Rock West's transverse compressive strength (reference only: half), and the axial root force on the
    // Araldite 2011 bond (CFRP lap shear, typical: half).
    members.push({ name: `rocker arm ${side}`, kind: 'web', check: 'web', material: mr, dims: { b_mm: r.arm_w_mm, h_mm: armT_mm, lever_mm: +(c * 1000).toFixed(3) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }])], uses: { F: 'F' }, note: 'in-plane bending at the hub, the coupler force at the full rocker length' });
    const cf = materialOf(cx.catalog, 'composite-rockwest-t-rnd-393-transverse');
    members.push({ name: `spar root socket ${side}`, kind: 'seat', check: 'seat', material: cf, dims: { d_mm: r.spar_od_mm ?? 10, t_mm: r.socket_len_mm, crush: true }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +c.toFixed(6) }, { ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'carbon tube bearing on the socket: Rock West calls these tubes not good for crush resistance and publishes CLT estimates only (counted at half)' });
    members.push({ name: `spar root bond ${side}`, kind: 'bond', check: 'bond', material: { id: 'adhesive-araldite-2011', shear_MPa: materialOf(cx.catalog, 'adhesive-araldite-2011').shear_MPa }, dims: { d_mm: r.spar_od_mm ?? 10, L_mm: r.socket_len_mm }, sections: [sect('F', [{ ch: `root_x_${w}`, k: 1 }])], uses: { F: 'F' } });
    members.push({ name: `rocker hub ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(shD.d * 1000).toFixed(3), t_mm: +(hub.seat_m * 1000).toFixed(3), e_over_d: +(r.hub_od_mm / 2 / (shD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }, { ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'the wing root force and moment and the coupler force into the shoulder pin over the hub length' });
    // round 17b (retain2, audit item 22): the shoulder pin held in the 7075 hub by a bond (retention.pinBond), the
    // "clamped" hub of v21-v24 having no clamp drawn: axially the wing's root force (an upper bound: the hub's faces and
    // the pin's collar bear on the bushing flanges directly), in torsion both shoulder bushings' friction on the pin
    const pb = L.retention?.pinBond ? materialOf(cx.catalog, L.retention.pinBond) : null;
    if (pb) members.push(bondMember(`shoulder pin ${side}, bond in the rocker hub`, pb, shD.d * 1000, hub.seat_m * 1000, [{ ch: `root_x_${w}`, k: 1 }], jt.supports.flatMap((sp) => scale(sp.radial, IGUS_G_MU_DRY * shD.d / 2)), `the ${(shD.d * 1000).toFixed(0)} mm pin bonded over the hub's ${(hub.seat_m * 1000).toFixed(2)} mm at full diameter (${pb.id}, 7075 in 7075); torsion: both ${shoulder.id} bushings' friction, ${IGUS_FRICTION_BASIS}`));
    hornMembers(cx, side, w, hs, mr, bossL_mm);
    // Coupler link: one piece with its two eyes (no bonded end fitting: no published bond strength to carbon).
    members.push({ name: `coupler eye ${side}`, kind: 'end fitting', check: 'lug', material: mat(L.coupler.material), dims: { d_mm: +(eyeD.D * 1000).toFixed(3), D_mm: L.coupler.eye_od_mm, t_mm: +(eyeD.B * 1000).toFixed(3) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }])], uses: { F: 'F' }, note: `the eye round the ${eye.id} bushing (both ends alike)` });
    // round 17b: each eye's bushing held in its eye by its flange in the eye's counterbore one way and, the other, by the
    // face it runs against (the crank web, the horn boss) or its pin's washer: the flange's ring at its published static
    // surface pressure under the coupler's and its bushings' inertia
    if (L.retention) members.push({ name: `coupler eye ${side} bushing flanges, axial thrust`, kind: 'seat', check: 'flange', material: { id: eye.id, static_surface_pressure_mpa: eye.specs.static_surface_pressure_mpa }, dims: { D_mm: eye.specs.d3_mm, d_mm: +(eyeD.D * 1000).toFixed(3) }, sections: [sect('F', inertialTerms((couplerMassG(cx) / 2 + 2 * eye.mass_g) / 1000, [w === 'R' ? cx.planes.couplerR : cx.planes.couplerL, cx.lay.fb.O4[1], w === 'R' ? cx.lay.fb.O4[0] : -cx.lay.fb.O4[0]]))], uses: { F: 'F' }, note: `the ${eye.id} flange's ring in the eye's counterbore (or against the web, the horn boss or the pin's washer) under the coupler's and its two bushings' inertia` });
    // Shoulder housings: a ring round each shoulder bushing.
    const mhs = mat(L.shoulderHousing.material);
    for (const sp of jt.supports) members.push({ name: `shoulder housing ${side} ${sp.name}`, kind: 'seat', check: 'lug', material: mhs, dims: { d_mm: +(shD.D * 1000).toFixed(3), D_mm: L.shoulderHousing.od_mm, t_mm: +(shD.B * 1000).toFixed(3) }, sections: [sect('F', sp.radial.map((t) => ({ ch: t.ch, k: t.k })))], uses: { F: 'F' } });
    item(`shoulder housings ${side}, 2 off`, mhs, 2 * ringV(L.shoulderHousing.od_mm / 1000, shD.D, shD.B), `rings ${L.shoulderHousing.od_mm}/${(shD.D * 1000).toFixed(0)} x ${(shD.B * 1000).toFixed(0)} mm round the ${shoulder.id} bushings; ${mhs.density_g_cm3} g/cm3 (mounting to the frame not counted here: in the 15% margin with fasteners)`);
}

/**
 * Round 17b, split out of wingSideMembers: one side's horn pin (pressed in the horn boss, the coupler eye cantilevered
 * beside it), the boss as drawn (hs: hornSide), and the pin's washer and its groove when fitted.
 */
function hornMembers(cx, side, w, hs, mr, bossL_mm) {
  const { d, L, members, mat, item, eye, eyeD } = cx, r = L.rocker;
  // Horn pin: pressed in the horn boss, the coupler eye cantilevered beside it.
  const mh = mat(L.hornPin.material);
  // the eye's gap to the boss and the pin's length past the eye: params.layout (round 14), else 1 mm each (v21)
  const gapH = (d.params?.layout?.horn_gap_mm ?? 1) / 1000, pastH = (d.params?.layout?.horn_pin_past_mm ?? 1) / 1000;
  const pwH = hs.pw; // round 17b: the pin from the boss's far face (as drawn) on past the eye to its washer's land
  const lever = eyeD.B / 2 + gapH, pinLen = pwH ? hs.H.pin[1] - hs.H.pin[0] : r.boss_len_mm / 1000 + eyeD.B + gapH + pastH;
  item(`horn pin ${side}`, mh, rodV(eyeD.d, pinLen), `${(eyeD.d * 1000).toFixed(0)} mm x ${(pinLen * 1000).toFixed(1).replace(/\.0$/, '')} mm; ${mh.density_g_cm3} g/cm3`);
  members.push({ name: `horn pin ${side}`, kind: 'pin', check: 'round', material: mh, dims: { d_mm: +(eyeD.d * 1000).toFixed(3) }, sections: [sect('M root', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }]), sect('V root', [{ ch: `coupler_${w}`, k: 1 }])], uses: { M: 'M root', V: 'V root' }, note: `the coupler eye at half its length plus ${+(gapH * 1000).toFixed(2)} mm from the boss face` });
  members.push({ name: `horn boss ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(eyeD.d * 1000).toFixed(3), t_mm: bossL_mm, e_over_d: +(hs.boss_od_mm / 2 / (eyeD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }])], uses: { F: 'F', M: 'M' } });
  // Round 17 (audit item 7, F5), round 17b: a DIN 6799 washer in a groove at the pin's free end, against the coupler's
  // horn eye free face (clearance.js pinWasherPart draws it); the pin checked again at its groove
  if (pwH) {
    const xHorn = w === 'R' ? cx.planes.couplerR : cx.planes.couplerL, O4 = cx.lay.fb.O4;
    members.push(...pinWasherMembers(cx, `horn pin washer ${side}`, `horn pin ${side}`, pwH, mh, (couplerMassG(cx) / 2 + 2 * eye.mass_g) / 1000, [xHorn, O4[1], w === 'R' ? O4[0] : -O4[0]], [{ ch: `coupler_${w}`, k: 1 }], 'the coupler (both its eyes and body) and its two bushings sliding off the pin, caught at the washer against the horn eye\'s free face', eyeD.D * 1000));
  }
  // round 17b (retain2, audit item 22): the pin held in its 7075 boss by a bond (retention.pinBond), not an unrated press
  // fit: axially the pin, the coupler and its bushings; in torsion the eye bushing's friction on the pin
  const pb = L.retention?.pinBond ? materialOf(cx.catalog, L.retention.pinBond) : null;
  if (pb) {
    const xHorn = w === 'R' ? cx.planes.couplerR : cx.planes.couplerL, O4 = cx.lay.fb.O4;
    const held = (gOf(mh, rodV(eyeD.d, pinLen)) + couplerMassG(cx) / 2 + 2 * eye.mass_g) / 1000;
    members.push(bondMember(`horn pin ${side}, bond in the horn boss`, pb, eyeD.d * 1000, bossL_mm, inertialTerms(held, [xHorn, O4[1], w === 'R' ? O4[0] : -O4[0]]), [{ ch: `coupler_${w}`, k: +(IGUS_G_MU_DRY * eyeD.d / 2).toFixed(8) }], `the ${(eyeD.d * 1000).toFixed(0)} mm pin bonded over the boss's ${bossL_mm} mm (${pb.id}, a 7075 pin in a 7075 boss); axially the pin, the coupler and its bushings' inertia; in torsion the ${eye.id}'s friction on it, ${IGUS_FRICTION_BASIS}`));
  }
}

/** igus iglidur G, dry, the upper end of its published coefficient of friction (against steel, Ra 1 um, 50 HRC). */
export const IGUS_G_MU_DRY = 0.15;
const IGUS_FRICTION_BASIS = "mu 0.15, the upper end of igus's dry 0.08-0.15 for iglidur G (against steel; igus.co.uk/info/plain-bearings-iglidur-g-material-data, read 2026-11-03; no figure is published on hard-anodised 7075)";

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
      // round 17b (audit item 16): a seat with its own boss wall (plates.boss_walls_mm, a thin boss where the rails or
      // gears leave no room for boss_wall_mm) is a housing as long as its part; the lug is judged on the plate alone
      const thin = boss && Number.isFinite(L.plates.boss_walls_mm?.[key]);
      const Db = pd.D * 1000 + 2 * (boss ? L.plates.boss_walls_mm?.[key] ?? L.plates.boss_wall_mm : L.plates.edge_mm);
      const D = thin ? pd.D * 1000 + 2 * L.plates.edge_mm : Db;
      const seat = plateSeatLength(L.plates, jt.name, sp.name, pd.B * 1000, pd.flangeB * 1000), tp = seat.plate_mm;
      const t = thin ? tp : seat.length_mm;
      if (boss && seat.length_mm > tp) { bossV += ringV(Db / 1000, pd.D, seat.length_mm / 1000 - tp / 1000); nBoss++; }
      // the thin boss's root ring (at the plate's face) in shear under the share of the part's radial load it carries
      // beyond the plate (its length over the part's), as a pin of that area
      if (thin && seat.length_mm > tp) {
        const area = Math.PI * (pd.D * 1000 + L.plates.boss_walls_mm[key]) * L.plates.boss_walls_mm[key], share = (seat.length_mm - tp) / seat.length_mm;
        members.push({ name: `plate boss ${jt.name} ${sp.name}, root`, kind: 'seat', check: 'pin-shear', material: pm, dims: { d_mm: +Math.sqrt((4 * area) / Math.PI).toFixed(4), planes: 1, area_mm2: +area.toFixed(3) }, sections: [sect('F', sp.radial.map((q) => ({ ch: q.ch, k: +(q.k * share).toFixed(8) })))], uses: { F: 'F' }, note: `the ${L.plates.boss_walls_mm[key]} mm boss wall's root ring at the plate face (mean diameter ${(pd.D * 1000 + L.plates.boss_walls_mm[key]).toFixed(1)} mm) in shear under ${(share * 100).toFixed(0)}% of the seat's radial load (the boss's share of the part's length), as a pin of that area` });
      }
      members.push({ name: `plate seat ${jt.name} ${sp.name}`, kind: 'seat', check: 'lug', material: pm, dims: { d_mm: +(pd.D * 1000).toFixed(3), D_mm: +D.toFixed(3), t_mm: +t.toFixed(3) }, sections: [sect('F', sp.radial.map((q) => ({ ch: q.ch, k: q.k })))], uses: { F: 'F' }, note: thin ? `the plate alone, judged as a ${L.plates.edge_mm} mm ring round the ${prt.id}; its ${Db.toFixed(1)} mm boss (a ${L.plates.boss_walls_mm[key]} mm wall) houses the part over its full width and is not credited as lug material` : boss ? `a boss ${D.toFixed(1)} mm round the ${prt.id}, as long as the part` : `the plate alone, judged as a ${L.plates.edge_mm} mm ring round the ${prt.id}` });
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
