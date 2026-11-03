// Round 5: the drive's load path (SPEC ruling "Bearings are linkage"). Every bearing, bushing and rod end of the drive:
// where it sits and which force it reacts, as load-channel terms the flight model evaluates every step
// (src/physics/flight.js LOAD_CHANNELS) and M7 checks against the part's published static rating
// (src/physics/bearings.js). Geometry from the film workstream's drive layout (sims/avis-shared/model.js placeFourBar,
// gearTrain, gearPlanes, used unchanged), so the page, film and gate see one drive.
import { placeFourBar, gearTrain, gearPlanes, partsIndex } from '../../sims/avis-shared/model.js';
import { makeFlapDrive, linkageCycleStats } from '../physics/kinematics.js';
import { leverShares, jointKind, DEEP_GROOVE } from '../physics/bearings.js';
import { byId } from './catalog.js';

const DEG = Math.PI / 180;

/** The part as the gate needs it: id, kind, name and the rating fields (no catalogue lookup at gate time). */
function ratedPart(part) {
  const s = part.specs ?? {};
  const keep = ['dimensions_mm_d_D_B', 'static_load_rating_kn', 'dynamic_load_rating_kn', 'kind', 'X0', 'Y0', 'type', 'material', 'd1_mm', 'd2_mm', 'b1_mm', 'd3_mm', 'b2_mm', 'static_surface_pressure_mpa', 'max_surface_speed_rotating_m_s', 'static_radial_load_n', 'static_radial_load_kn', 'bore_mm', 'material_source_url'];
  return { id: part.id, category: part.category, name: part.name, kind: jointKind(part), source_url: part.source_url, specs: Object.fromEntries(keep.filter((k) => s[k] !== undefined).map((k) => [k, s[k]])) };
}

/** Bore of a joint part in metres (bushing d1, bearing d, rod end bore), for the surface-speed check. */
export function boreOf(part) {
  const s = part.specs ?? {};
  if (Number.isFinite(s.d1_mm)) return s.d1_mm / 1000;
  if (Number.isFinite(s.bore_mm)) return s.bore_mm / 1000;
  const m = /^(\d+(?:\.\d+)?)x/.exec(s.dimensions_mm_d_D_B ?? '');
  return m ? +m[1] / 1000 : null;
}

/** ISO 76 factors a support of this part carries into the flight model (null for a bushing or rod end). */
function p0Form(part) {
  const k = jointKind(part);
  if (k === 'needle') return { X0: 1, Y0: 0 };
  if (k === 'deep-groove') return Number.isFinite(part.specs?.X0) ? { X0: part.specs.X0, Y0: part.specs.Y0 } : { X0: DEEP_GROOVE.X0, Y0: DEEP_GROOVE.Y0 };
  if (k === 'rolling') return Number.isFinite(part.specs?.X0) ? { X0: part.specs.X0, Y0: part.specs.Y0 } : { X0: null, Y0: null };
  return { X0: null, Y0: null };
}

/**
 * A shaft on two supports at xA (aft) and xB (forward) carrying point loads { ch, perUnit, x, what } in the plane
 * normal to the shaft, plus optional couples { ch, perUnit } (moment normal to the shaft, reacted as F = M / L at
 * both supports) and axial loads { ch } (carried, conservatively, by each support in full).
 */
function shaft(name, part, xA, xB, loads, { couples = [], axial = [], surfacePerCrank = null, reacts }) {
  const L = Math.abs(xB - xA);
  const supports = [['aft', 0, xA], ['forward', 1, xB]].map(([sname, i, x]) => ({
    name: sname, x_m: +x.toFixed(5),
    radial: [...loads.map((l) => ({ ch: l.ch, k: +(l.perUnit * leverShares(xA, xB, l.x)[i]).toFixed(6), what: l.what })), ...couples.map((c) => ({ ch: c.ch, k: +(c.perUnit / L).toFixed(6), what: c.what }))],
    axial: axial.map((a) => ({ ch: a.ch, k: 1, what: a.what })),
    ...p0Form(part),
    part: ratedPart(part),
    ...(surfacePerCrank !== null ? { surface_m_s_per_crank_rad_s: +(surfacePerCrank * (boreOf(part) ?? 0) / 2).toFixed(6) } : {}),
  }));
  return { name, part_id: part.id, count: 2, span_m: +L.toFixed(5), reacts, supports };
}

/** One part carrying one channel in full (a coupler eye): a single support. */
function eye(name, part, ch, surfacePerCrank, reacts) {
  return { name, part_id: part.id, count: 1, reacts, supports: [{ name: 'eye', radial: [{ ch, k: 1, what: 'coupler force' }], axial: [], ...p0Form(part), part: ratedPart(part), surface_m_s_per_crank_rad_s: +(surfacePerCrank * (boreOf(part) ?? 0) / 2).toFixed(6) }] };
}

/**
 * The drive's joints for a built design d (geometry final) and the choice of parts:
 * choice = { shoulder, shoulderSpan_mm, crankShaft, shafts: [part per intermediate shaft, crank side first],
 * couplerEyes }. Returns { joints, unchecked, layout }.
 */
/**
 * The drive's geometry and force ratios from the film's layout (model.js placeFourBar, gearTrain, gearPlanes): shared
 * by the joints and, round 6, the loaded members (src/design/linkage.js).
 */
export function driveLayout(d, catalog) {
  const law = makeFlapDrive(d.drive);
  const fb = placeFourBar(d, law);
  const train = gearTrain(d, partsIndex(d, catalog), fb);
  const planes = gearPlanes(d, train);
  // Torque ratio from the wheel of stage k to the crank, and the mesh force per N m of crank load torque at stage k:
  // the tangential force T_wheel / r_wheel over cos(pressure angle).
  const below = (k) => train.stages.slice(k + 1).reduce((r, s) => r * (s.wheel.z / s.pinion.z), 1);
  const alpha = (k) => (byId(catalog, train.stages[k].wheel.id).specs.pressure_angle_deg ?? 20) * DEG;
  const meshPerNm = (k) => 1 / (below(k) * train.stages[k].wheel.rp * Math.cos(alpha(k)));
  // Stroke rates: the shoulder turns at most at the linkage's peak velocity ratio times the crank rate.
  const amps = [d.drive.ampMin, d.drive.ampMax];
  const ratio = Math.max(...amps.map((a) => linkageCycleStats(law, a).maxRatio));
  return { law, fb, train, planes, n: train.n, x0: d.wing.hinge[0], below, meshPerNm, ratio, aOverB: d.drive.a / d.drive.b };
}

export function driveJoints(d, catalog, choice) {
  const lay = driveLayout(d, catalog);
  const { train, planes, n, x0, below, meshPerNm, ratio, aOverB } = lay;
  const span = (choice.shoulderSpan_mm ?? 24) / 1000;
  const part = (id) => byId(catalog, id);
  const joints = [];
  for (const [side, w, xB] of [['right', 'R', planes.couplerR], ['left', 'L', planes.couplerL]]) {
    joints.push(shaft(`shoulder pivot ${side}`, part(choice.shoulder), x0 - span / 2, x0 + span / 2, [
      { ch: `root_yz_${w}`, perUnit: 1, x: x0, what: 'wing root force normal to the hinge axis (air and inertia)' },
      { ch: `coupler_${w}`, perUnit: 1, x: xB, what: 'coupler force on the rocker horn' },
    ], { couples: [{ ch: `root_mperp_${w}`, perUnit: 1, what: 'wing root moment normal to the hinge axis (lead-lag and pitch), as a couple' }], axial: [{ ch: `root_x_${w}`, what: 'wing root force along the hinge axis' }], surfacePerCrank: ratio, reacts: `the ${side} wing's root force and moment, and the ${side} coupler's force on the rocker horn` }));
  }
  joints.push(shaft('crank shaft', part(choice.crankShaft), planes.plateAft, planes.plateFwd, [
    { ch: 'coupler_R', perUnit: 1, x: planes.couplerR, what: 'right coupler on the crank pin (overhung)' },
    { ch: 'coupler_L', perUnit: 1, x: planes.couplerL, what: 'left coupler on the crank pin (overhung)' },
    { ch: 'crank_torque', perUnit: meshPerNm(n - 1), x: planes.stage[n - 1], what: `stage ${n} mesh, crank wheel ${train.stages[n - 1].wheel.id}` },
  ], { surfacePerCrank: 1, reacts: 'both couplers on the shared crank pin and the last stage\'s mesh' }));
  for (let j = 1; j < n; j++) {
    const kw = n - 1 - j, kp = n - j; // the wheel of stage kw and the pinion of stage kp ride on shaft j
    const id = choice.shafts[j - 1] ?? choice.shafts[choice.shafts.length - 1];
    joints.push(shaft(`gear shaft ${j}`, part(id), planes.plateAft, planes.plateFwd, [
      { ch: 'crank_torque', perUnit: meshPerNm(kw), x: planes.stage[kw], what: `stage ${kw + 1} mesh, wheel ${train.stages[kw].wheel.id}` },
      { ch: 'crank_torque', perUnit: meshPerNm(kp), x: planes.stage[kp], what: `stage ${kp + 1} mesh, pinion ${train.stages[kp].pinion.id}` },
    ], { surfacePerCrank: below(kp - 1), reacts: `stage ${kw + 1} and stage ${kp + 1} meshes` }));
  }
  // Coupler eyes: each coupler's two ends, its whole force on one part each; relative rotation at most (1 + a/b) of the
  // crank rate at the pin, the rocker's rate plus the coupler's at the horn.
  for (const [side, w] of [['right', 'R'], ['left', 'L']]) {
    joints.push(eye(`coupler eye ${side}, crank pin`, part(choice.couplerEyes), `coupler_${w}`, 1 + aOverB, `the ${side} coupler's force at the crank pin`));
    joints.push(eye(`coupler eye ${side}, rocker horn`, part(choice.couplerEyes), `coupler_${w}`, ratio + aOverB, `the ${side} coupler's force at the rocker horn`));
  }
  // Round 6 (SPEC ruling after round 5, motor bearings carrying a pinion): an outboard bearing in the forward gearbox
  // plate on the motor shaft, between the motor and the first pinion. The pinion overhangs it by a; the nearest motor
  // bearing can sit no closer than s = the gap to the motor's face plus half a published motor-bearing width (the
  // motor's bearing positions are not published), and the smallest s gives the largest plate-bearing load,
  // F (a + s) / s, which is what it is rated for. The motor bearing then carries F a / s (reported in unchecked).
  let motorLever = null;
  if (choice.motorOutboard) {
    // a pinion inside the box (aft of the plate) or, round 14, overhung forward of it with the motor inside the box
    const a = Math.abs(planes.plateFwd - planes.stage[0]);
    const s0 = Math.abs((planes.motorFace ?? planes.motorAft) - planes.plateFwd) + (choice.motorBearingHalfWidth_mm ?? 2.5) / 1000;
    motorLever = { a_m: +a.toFixed(5), s_min_m: +s0.toFixed(5), plate_share: (a + s0) / s0, motor_share: a / s0 };
    const ob = part(choice.motorOutboard);
    joints.push({ name: 'motor shaft outboard', part_id: ob.id, count: 1, reacts: `stage 1 mesh at pinion ${train.stages[0].pinion.id}, overhung ${(a * 1000).toFixed(1)} mm from this bearing`, supports: [{ name: 'plate', radial: [{ ch: 'crank_torque', k: +(meshPerNm(0) * motorLever.plate_share).toFixed(6), what: `stage 1 mesh x (a + s) / s, a ${(a * 1000).toFixed(1)} mm, s ${(s0 * 1000).toFixed(1)} mm (nearest a motor bearing can be)` }], axial: [], ...p0Form(ob), part: ratedPart(ob), surface_m_s_per_crank_rad_s: 0 }] });
  }
  const unchecked = [{ name: 'motor shaft', carries: `stage 1 mesh, pinion ${train.stages[0].pinion.id}`, force_per_crank_Nm: +meshPerNm(0).toFixed(4), ...(motorLever ? { motor_bearing_force_per_crank_Nm: +(meshPerNm(0) * motorLever.motor_share).toFixed(4), lever: motorLever, note: 'with the outboard bearing, the motor bearing nearest the pinion carries at most F a / s; the motor maker publishes its bearing sizes (11 x 5 x 5 and 16 x 5 x 5 mm) but not their maker, designation or positions, so it is reported against the lowest same-size C0 read (NMB L-1150ZZ 276 N), not credited' } : { note: 'the motor\'s own bearings carry the first pinion; the motor maker publishes no bearing rating, so this load is reported, not checked' }) }];
  return { joints, unchecked, layout: { source: 'sims/avis-shared/model.js placeFourBar, gearTrain, gearPlanes (film workstream), used unchanged', hinge_x_m: x0, shoulder_span_m: span, planes_m: { plateAft: planes.plateAft, plateFwd: planes.plateFwd, stage: planes.stage, couplerR: planes.couplerR, couplerL: planes.couplerL, crankArm: planes.crankArm }, max_stroke_velocity_ratio: +ratio.toFixed(4), mesh_force_per_crank_Nm: train.stages.map((s, k) => +meshPerNm(k).toFixed(4)) } };
}
