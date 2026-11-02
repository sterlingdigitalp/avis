// The design file: data/designs/<name>.json is the one description of a bird. Built by src/design.
// Every field in SI, degrees only where a name says _deg.

const REQUIRED = {
  name: 'string', label: 'string', candidate: 'string', claim: 'string',
  atmosphere: 'object', wing: 'object', drive: 'object', tail: 'object', body: 'object', mass: 'object',
  motor: 'object', esc: 'object', battery: 'object', servos: 'object', structure: 'object', cruise: 'object', bom: 'object',
};

export const CLAIM = 'simulated flight, real parts';

/** Returns a list of problems, empty if valid. */
export function validateDesign(d) {
  const errs = [];
  for (const [k, t] of Object.entries(REQUIRED)) {
    if (typeof d[k] !== t) errs.push(`${k} must be ${t}`);
  }
  if (errs.length) return errs;
  if (d.claim !== CLAIM) errs.push(`claim must read "${CLAIM}"`);
  if (!Array.isArray(d.wing.segments) || d.wing.segments.length < 2) errs.push('wing needs at least two segments');
  if (!(d.wing.hinge?.[2] > 0)) errs.push('wing.hinge z must be positive: right shoulder');
  for (const p of d.bom.parts ?? []) {
    if (!p.id) errs.push(`BOM entry ${p.name} has no catalogue id`);
    if (!(p.unit_mass_g >= 0)) errs.push(`BOM entry ${p.id} has no mass`);
  }
  const lumpMass = d.mass.lumps.reduce((a, l) => a + l.mass_kg, 0);
  if (!(d.mass.total_kg > 0)) errs.push('mass.total_kg must be positive');
  if (lumpMass <= 0) errs.push('mass.lumps empty');
  return errs;
}
