// Our low-order section model: one set of parameters at one Reynolds number. Separate from the data so that the
// fitting script and the flight model share it.
const W = 2.0 * (Math.PI / 180); // stall transition half-width

/** p: { a, aL0, aStall, aStallNeg, cdMin, clD, kUp, kLo }. Writes [cl, cd, 0, attached]. alpha in rad. */
export function sectionModel(p, alpha, out) {
  const cl0 = p.a * (alpha - p.aL0);
  const pos = (alpha - p.aStall) / W;
  const neg = (p.aStallNeg - alpha) / W;
  const sigma = pos > 30 || neg > 30 ? 0 : 1 / ((1 + Math.exp(pos)) * (1 + Math.exp(neg)));
  const s = Math.sin(alpha);
  const c = Math.cos(alpha);
  const dc = cl0 - p.clD;
  const cdA = p.cdMin + (dc > 0 ? p.kUp : p.kLo) * dc * dc;
  const cdF = p.cdMin + 1.9 * s * s;
  const clF = 1.9 * s * c;
  out[0] = sigma * cl0 + (1 - sigma) * clF;
  // Attached drag never exceeds the post-stall value at the angles where it matters.
  out[1] = sigma * cdA + (1 - sigma) * cdF;
  out[2] = 0;
  out[3] = sigma;
  return out;
}
