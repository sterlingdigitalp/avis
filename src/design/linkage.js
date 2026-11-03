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
import { linkageAt } from '../../sims/avis-shared/model.js';
import { airframeMembers } from './airframe.js';

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
  if (Number.isFinite(s.d1_mm)) return { d: s.d1_mm / 1000, D: s.d2_mm / 1000, B: s.b1_mm / 1000 };
  const m = /^(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)/.exec(s.dimensions_mm_d_D_B ?? '');
  return m ? { d: +m[1] / 1000, D: +m[2] / 1000, B: +m[3] / 1000 } : null;
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
    item: (name, m, v, basis) => items.push({ name, material_id: m.id, material: m.name, mass_g: gOf(m, v), basis, source_url: m.source_url }),
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
    Object.assign(it, { mass_g: (aft.area_m2 * L.plates.t_mm + fwd.area_m2 * tF) / 1000 * pm.density_g_cm3 * 1e6, basis: `solid outlines (the hull of every seat and rail seat, posts to the pads; no lightening credited): aft ${(aft.area_m2 * 1e4).toFixed(1)} cm2 x ${L.plates.t_mm} mm, forward ${(fwd.area_m2 * 1e4).toFixed(1)} cm2 x ${tF} mm; ${pm.density_g_cm3} g/cm3` });
  } else {
    // Round 7 (SPEC ruling after round 6: a load path that cannot be rated fails): the plates between their seats.
    cx.members.push(unrated('gearbox plates between seats', 'plate', 'the drive layout (model.js) gives the two plates no mounts to the airframe, so the path from the seats to the keel cannot be sized or rated', [{ ch: 'crank_torque', k: 1 }]));
  }
  return { members: cx.members, items: cx.items, coupler, unchecked: cx.unchecked, airframe };
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
    const lenS = planes.plateFwd + plateT / 2 + 0.001 - (planes.crankArm + tw / 2);
    const wheel = train.stages[n - 1].wheel;
    const seatL = (wheel.face ?? 0.01);
    const v = ringV(od, id, lenS) + bw * tw * (d.drive.a + (od + dp) / 2) + rodV(dp, pinL) + (c.seat_od_mm ? ringV(c.seat_od_mm / 1000, od, seatL) : 0);
    item('crank: shaft, web and pin in one piece', m, v, `${c.shaft_od_mm}${id ? `/${c.shaft_id_mm}` : ''} mm shaft x ${(lenS * 1000).toFixed(1)} mm, web ${c.web_b_mm} x ${c.web_t_mm} mm over the ${(d.drive.a * 1000).toFixed(0)} mm throw, ${(dp * 1000).toFixed(0)} mm pin x ${(pinL * 1000).toFixed(1)} mm${c.seat_od_mm ? `, ${c.seat_od_mm} mm wheel seat x ${(seatL * 1000).toFixed(0)} mm (the ${wheel.id}, ${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
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

/** Gear shaft j (crank side first): the wheel of stage n-1-j and the pinion of stage n-j, with their keys. */
function gearShaftMembers(cx, j) {
  // ---- Gear shafts j = 1..n-1 (crank side first): the wheel of stage n-1-j and the pinion of stage n-j.
  const { catalog, L, lay, planes, members, items, unchecked, mat, item, jointByName, shaftLen } = cx;
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
    const v = rodV((g.mass_d_mm ?? g.d_mm) / 1000, shaftLen) + (g.seat_od_mm ? ringV(g.seat_od_mm / 1000, g.d_mm / 1000, wheel.face ?? 0.01) : 0);
    item(`gear shaft ${j}`, m, v, `${g.mass_d_mm ?? g.d_mm} mm x ${(shaftLen * 1000).toFixed(1)} mm${g.seat_od_mm ? ` with a ${g.seat_od_mm} mm wheel seat for the ${wheel.id} (${boreText(catalog, wheel.id)}, keyed)` : ''}; ${m.density_g_cm3} g/cm3`);
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
}

/** One side's shoulder pin, rocker (hub, horn boss), horn pin, coupler eye and shoulder housings. */
function wingSideMembers(cx, side, w) {
  // ---- Shoulder pins: clamped in the rocker hub between the two bushings; bending within each bushing.
  const { d, L, lay, members, mat, item, jointByName, eye, eyeD, shoulder, shD, span } = cx;
    const m = mat(L.shoulderPin.material);
    const jt = jointByName(`shoulder pivot ${side}`);
    item(`shoulder pin ${side}`, m, rodV(shD.d, span + shD.B), `${(shD.d * 1000).toFixed(0)} mm x ${((span + shD.B) * 1000).toFixed(0)} mm (bushing centres ${(span * 1000).toFixed(0)} mm apart plus one bushing length); ${m.density_g_cm3} g/cm3`);
    members.push({ name: `shoulder pin ${side}`, kind: 'pin', check: 'round', material: m, dims: { d_mm: +(shD.d * 1000).toFixed(3) }, sections: jt.supports.flatMap((sp) => [sect(`M ${sp.name}`, scale(sp.radial, shD.B / 2)), sect(`V ${sp.name}`, sp.radial.map((t) => ({ ch: t.ch, k: t.k })))]), uses: { M: jt.supports.map((sp) => `M ${sp.name}`), V: jt.supports.map((sp) => `V ${sp.name}`) }, note: 'the bushing reaction at half the bushing length from the hub face' });
    // Rocker: hub on the shoulder pin, horn arm of the rocker length c, horn boss for the horn pin, spar socket.
    const r = L.rocker, mr = mat(r.material);
    const c = linkageAt(lay.law, d.drive.ampNominal).c;
    const hubL = span - shD.B;
    const sparOD = (r.spar_od_mm ?? 10) / 1000;
    const vR = ringV(r.hub_od_mm / 1000, shD.d, hubL) + c * (r.arm_w_mm / 1000) * (r.arm_t_mm / 1000) + ringV(r.boss_od_mm / 1000, eyeD.d, r.boss_len_mm / 1000) + ringV(sparOD + 2 * r.socket_wall_mm / 1000, sparOD, r.socket_len_mm / 1000);
    item(`rocker ${side}`, mr, vR, `hub ${r.hub_od_mm}/${(shD.d * 1000).toFixed(0)} x ${(hubL * 1000).toFixed(0)} mm, horn arm ${(c * 1000).toFixed(1)} x ${r.arm_w_mm} x ${r.arm_t_mm} mm, horn boss ${r.boss_od_mm}/${(eyeD.d * 1000).toFixed(0)} x ${r.boss_len_mm} mm, spar socket ${r.socket_wall_mm} mm wall x ${r.socket_len_mm} mm; ${mr.density_g_cm3} g/cm3 (replaces the carbon-plate rocker)`);
    // Round 7: the rocker's horn arm in bending (the coupler force at the rocker length) and the carbon spar in its
    // socket: the root flap moment (the coupler force times c) and the lead-lag moment as edge pressure on the tube,
    // judged at Rock West's transverse compressive strength (reference only: half), and the axial root force on the
    // Araldite 2011 bond (CFRP lap shear, typical: half).
    members.push({ name: `rocker arm ${side}`, kind: 'web', check: 'web', material: mr, dims: { b_mm: r.arm_w_mm, h_mm: r.arm_t_mm, lever_mm: +(c * 1000).toFixed(3) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }])], uses: { F: 'F' }, note: 'in-plane bending at the hub, the coupler force at the full rocker length' });
    const cf = materialOf(cx.catalog, 'composite-rockwest-t-rnd-393-transverse');
    members.push({ name: `spar root socket ${side}`, kind: 'seat', check: 'seat', material: cf, dims: { d_mm: r.spar_od_mm ?? 10, t_mm: r.socket_len_mm, crush: true }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +c.toFixed(6) }, { ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'carbon tube bearing on the socket: Rock West calls these tubes not good for crush resistance and publishes CLT estimates only (counted at half)' });
    members.push({ name: `spar root bond ${side}`, kind: 'bond', check: 'bond', material: { id: 'adhesive-araldite-2011', shear_MPa: materialOf(cx.catalog, 'adhesive-araldite-2011').shear_MPa }, dims: { d_mm: r.spar_od_mm ?? 10, L_mm: r.socket_len_mm }, sections: [sect('F', [{ ch: `root_x_${w}`, k: 1 }])], uses: { F: 'F' } });
    members.push({ name: `rocker hub ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(shD.d * 1000).toFixed(3), t_mm: +(hubL * 1000).toFixed(3), e_over_d: +(r.hub_od_mm / 2 / (shD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `root_yz_${w}`, k: 1 }, { ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `root_mperp_${w}`, k: 1 }])], uses: { F: 'F', M: 'M' }, note: 'the wing root force and moment and the coupler force into the shoulder pin over the hub length' });
    // Horn pin: pressed in the horn boss, the coupler eye cantilevered beside it.
    const mh = mat(L.hornPin.material);
    const lever = eyeD.B / 2 + 0.001;
    item(`horn pin ${side}`, mh, rodV(eyeD.d, r.boss_len_mm / 1000 + eyeD.B + 0.002), `${(eyeD.d * 1000).toFixed(0)} mm x ${(r.boss_len_mm + eyeD.B * 1000 + 2).toFixed(0)} mm; ${mh.density_g_cm3} g/cm3`);
    members.push({ name: `horn pin ${side}`, kind: 'pin', check: 'round', material: mh, dims: { d_mm: +(eyeD.d * 1000).toFixed(3) }, sections: [sect('M root', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }]), sect('V root', [{ ch: `coupler_${w}`, k: 1 }])], uses: { M: 'M root', V: 'V root' }, note: 'the coupler eye at half its length plus 1 mm from the boss face' });
    members.push({ name: `horn boss ${side}`, kind: 'seat', check: 'seat', material: mr, dims: { d_mm: +(eyeD.d * 1000).toFixed(3), t_mm: r.boss_len_mm, e_over_d: +(r.boss_od_mm / 2 / (eyeD.d * 1000)).toFixed(4) }, sections: [sect('F', [{ ch: `coupler_${w}`, k: 1 }]), sect('M', [{ ch: `coupler_${w}`, k: +lever.toFixed(6) }])], uses: { F: 'F', M: 'M' } });
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
export function plateSeatLength(plates, jointName, supportName, B_mm) {
  const tp = supportName === 'aft' ? plates.t_mm : (plates.t_fwd_mm ?? plates.t_mm);
  const boss = (plates.bossed ?? []).includes(`${jointName}/${supportName}`);
  return { plate_mm: tp, bossed: boss, length_mm: boss ? Math.max(tp, B_mm) : tp };
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
        const r = plateSeatLength(L.plates, j.name, sp.name, B);
        return { ...sp, housing_mm: +Math.min(B, r.length_mm).toFixed(3), housing: r.bossed ? 'plate boss as long as the part' : `${r.plate_mm} mm plate, no boss` };
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
  let bossV = 0;
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
      const { plate_mm: tp, length_mm: t } = plateSeatLength(L.plates, jt.name, sp.name, pd.B * 1000);
      if (boss) bossV += ringV(D / 1000, pd.D, Math.max(0, t / 1000 - tp / 1000));
      members.push({ name: `plate seat ${jt.name} ${sp.name}`, kind: 'seat', check: 'lug', material: pm, dims: { d_mm: +(pd.D * 1000).toFixed(3), D_mm: +D.toFixed(3), t_mm: +t.toFixed(3) }, sections: [sect('F', sp.radial.map((q) => ({ ch: q.ch, k: q.k })))], uses: { F: 'F' }, note: boss ? `a boss ${D.toFixed(1)} mm round the ${prt.id}, as long as the part` : `the plate alone, judged as a ${L.plates.edge_mm} mm ring round the ${prt.id}` });
    }
  }
  if (bossV > 0) item('gearbox plate bosses', pm, bossV, `bosses ${L.plates.boss_wall_mm} mm wall round the seats ${[...bossed].join(', ')}, as long as the bushing, beyond the plate; ${pm.density_g_cm3} g/cm3`);
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
