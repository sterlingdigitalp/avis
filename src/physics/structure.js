// Structural checks shared by the M7 gate and the design's stroke envelope. Pure functions of section and load.
import { makeFlapDrive } from './kinematics.js';

/** Round 2 rule for pultruded rods (SPEC M7 materials): the lower of the listed tensile range, 400 MPa. */
export const ROD_ALLOWABLE_MPA = 400;

/**
 * Coupler as a pin-ended two-force member: Euler buckling P = pi^2 E I / L^2 of a solid rod or a tube (id_m > 0),
 * and axial stress F / A against its published allowable (a rod without one: ROD_ALLOWABLE_MPA).
 * cp: { od_m, id_m?, length_m, E_GPa, allowable_MPa? }. F in newtons.
 */
export function couplerCheck(cp, F) {
  const od = cp.od_m;
  const id = cp.id_m ?? 0;
  const I = (Math.PI * (od ** 4 - id ** 4)) / 64;
  const area = (Math.PI * (od ** 2 - id ** 2)) / 4;
  const buckling = (Math.PI ** 2 * cp.E_GPa * 1e9 * I) / cp.length_m ** 2;
  const stress = F / area / 1e6;
  const allowable = Number.isFinite(cp.allowable_MPa) ? cp.allowable_MPa : ROD_ALLOWABLE_MPA;
  return { buckling_N: buckling, fos_buckling: buckling / F, stress_MPa: stress, allowable_MPa: allowable, fos_stress: allowable / stress, section: { I_m4: I, area_m2: area, kind: id > 0 ? 'tube' : 'rod' } };
}

/**
 * Twist torque rod or tube in torsion (round 4, M7 linkage): shear stress T r / J with J = pi (od^4 - id^4) / 32.
 * Allowable shear only from a published figure: a metal tube's published minimum yield through von Mises,
 * tau_y = sigma_y / sqrt(3); a composite rod or tube publishes no shear strength in the catalogue, so it has none
 * (fos null: the check cannot pass). rod: { od_m, id_m?, kind: 'metal'|'composite', yield_MPa? }. T in N m.
 */
export function torsionCheck(rod, T) {
  const od = rod.od_m;
  const id = rod.id_m ?? 0;
  const J = (Math.PI * (od ** 4 - id ** 4)) / 32;
  const stress = (Math.abs(T) * (od / 2)) / J / 1e6;
  const allowable = rod.kind === 'metal' && Number.isFinite(rod.yield_MPa) ? rod.yield_MPa / Math.sqrt(3) : null;
  return { stress_MPa: stress, allowable_shear_MPa: allowable, fos: allowable ? allowable / stress : null, J_m4: J, basis: allowable ? 'published minimum yield / sqrt(3) (von Mises shear yield)' : 'no published shear strength: torsion cannot be checked' };
}

/** Bending stress (MPa) of a tube section { I, c } under moment M (N m), and its factor of safety. */
export function sparCheck(sp, M) {
  const stress = (M * sp.c) / sp.I / 1e6;
  return { stress_MPa: stress, fos: sp.allowable_MPa / stress };
}

/**
 * Stroke servo holding load (round 5, M7 "servo torque within rated values"). On a slotted rocker the coupler pin
 * slides along each rocker to set the stroke, and the coupler force's component along the rocker pushes the pin along
 * the slot; the one stroke servo holds both slots through its pushrods and yoke. Its torque is that pair force
 * (slotPairN, the flight model's simultaneous sum) times the horn radius. The horn is taken at its most favourable
 * size, the slot travel over a 180 degree throw (r = travel / 2), with the force normal to the horn. A fixed stroke has
 * no slot and no stroke servo. The rocker radii come from the drive's own linkage table (kinematics makeFlapDrive).
 */
export function strokeServoHold(design, slotPairN, cRange = null) {
  const sv = design.servos?.stroke;
  const fixed = design.drive?.fixedStroke || !(design.drive?.ampMax - design.drive?.ampMin > 1e-9);
  if (fixed || !sv) return { applicable: false, ok: true, note: fixed ? 'fixed stroke: a pinned rocker, no slot and no stroke servo' : 'no stroke servo' };
  const [cMin, cMax] = cRange ?? rockerRange(design.drive);
  const travel = cMax - cMin;
  const r = travel / 2;
  const torque = slotPairN * r;
  return { applicable: true, slot_force_pair_N: slotPairN, slot_travel_m: travel, horn_radius_m: r, horn_basis: 'slot travel over a 180 degree throw, the most favourable horn; force normal to it', torque_Nm: torque, rated_stall_Nm: sv.stall_Nm, ok: torque <= sv.stall_Nm };
}

/** Smallest and largest rocker radius of a slotted crank-rocker drive, from its linkage table. */
function rockerRange(drive) {
  const t = makeFlapDrive(drive).table;
  return [Math.min(...t.map((e) => e.c)), Math.max(...t.map((e) => e.c))];
}

/** Material family of a gear from its catalogue material text: 'steel', 'acetal', 'nylon' or the text itself. */
export function gearMaterialClass(text) {
  const t = String(text ?? '');
  if (/SCM|S45C|SUS|steel/i.test(t)) return 'steel';
  if (/duracon|acetal|POM/i.test(t)) return 'acetal';
  if (/MC ?9|nylon|PA6/i.test(t)) return 'nylon';
  return t || null;
}

/**
 * Hertz contact stress at the pitch point of a spur mesh (MPa): two cylinders of radii r sin(alpha) under the normal
 * tooth load Ft / cos(alpha) along the real face width b, with the pair's combined modulus.
 * sigma = sqrt( Fn (1/rho1 + 1/rho2) / (pi b ((1 - nu1^2)/E1 + (1 - nu2^2)/E2)) ). SI in, MPa out.
 */
export function hertzSpur({ T_pinion_Nm, r1_m, r2_m, b_m, alpha_rad, E1_Pa, nu1, E2_Pa, nu2 }) {
  const Ft = T_pinion_Nm / r1_m;
  const Fn = Ft / Math.cos(alpha_rad);
  const k = 1 / (r1_m * Math.sin(alpha_rad)) + 1 / (r2_m * Math.sin(alpha_rad));
  const c = (1 - nu1 * nu1) / E1_Pa + (1 - nu2 * nu2) / E2_Pa;
  return Math.sqrt((Fn * k) / (Math.PI * b_m * c)) / 1e6;
}

/**
 * SPEC ruling on mixed-material meshes, per stage: a mesh of two materials is judged by Hertz against each member's
 * published allowable surface stress (stage.pinion_contact / gear_contact: { allowable_MPa, E_Pa, nu }); without
 * both, it cannot be judged and fails. Same-material meshes keep the maker's own ratings (the torque check).
 */
export function mixedMeshCheck(stage, T_pinion_Nm) {
  const a = gearMaterialClass(stage.pinion_material), b = gearMaterialClass(stage.gear_material);
  if (!a || !b || a === b) return { mixed: false, ok: true, pinion_material: stage.pinion_material ?? null, gear_material: stage.gear_material ?? null, note: 'one material: the maker\'s ratings for the pair apply' };
  const P = stage.pinion_contact, G = stage.gear_contact;
  if (!(P?.allowable_MPa && G?.allowable_MPa && P.E_Pa && G.E_Pa)) return { mixed: true, ok: false, pinion_material: stage.pinion_material, gear_material: stage.gear_material, note: 'mixed-material mesh without a published allowable surface stress for both members: cannot be judged (SPEC ruling)' };
  const s = hertzSpur({ T_pinion_Nm, r1_m: stage.pinion_rp_m, r2_m: stage.gear_rp_m, b_m: Math.min(stage.pinion_face_m, stage.gear_face_m), alpha_rad: stage.alpha_rad, E1_Pa: P.E_Pa, nu1: P.nu, E2_Pa: G.E_Pa, nu2: G.nu });
  return { mixed: true, sigma_H_MPa: s, pinion_allowable_MPa: P.allowable_MPa, gear_allowable_MPa: G.allowable_MPa, ok: s <= P.allowable_MPa && s <= G.allowable_MPa, pinion_material: stage.pinion_material, gear_material: stage.gear_material };
}
