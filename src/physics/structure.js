// Structural checks shared by the M7 gate and the design's stroke envelope. Pure functions of section and load.

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

/** Bending stress (MPa) of a tube section { I, c } under moment M (N m), and its factor of safety. */
export function sparCheck(sp, M) {
  const stress = (M * sp.c) / sp.I / 1e6;
  return { stress_MPa: stress, fos: sp.allowable_MPa / stress };
}
