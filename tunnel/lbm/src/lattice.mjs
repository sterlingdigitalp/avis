// D3Q19 constants shared by the CPU reference, geometry engine and WGSL generator.
// Ordering from tdf2: 0 rest, 1-6 faces, 7-18 edges; opposite directions are adjacent pairs.
export const Q = 19;
export const CX = [0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0];
export const CY = [0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1];
export const CZ = [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1];
const W0 = 1 / 3, W1 = 1 / 18, W2 = 1 / 36;
export const W = [W0, W1, W1, W1, W1, W1, W1, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2];
export const OPP = [0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17];
/** One direction of each +/- pair; a link is stored from its lower end along one of these. */
export const HALF = [1, 3, 5, 7, 9, 11, 13, 15, 17];

function findDir(x, y, z) {
  for (let i = 0; i < Q; i++) if (CX[i] === x && CY[i] === y && CZ[i] === z) return i;
  throw new Error('no such direction');
}
export const MIRROR_Y = CX.map((_, i) => findDir(CX[i], -CY[i], CZ[i]));
export const MIRROR_Z = CX.map((_, i) => findDir(CX[i], CY[i], -CZ[i]));

export const AXIS = { periodic: 0, freeslip: 1, bounceback: 2, inflow: 3 };

/** Solid-mask value meaning "fluid". Part ids 0-254 mean solid with that part. */
export const FLUID = 255;
export const MAX_PARTS = 16;
/** Accumulator layout per part: Fx Fy Fz Mx My Mz dMass spare. Two u32 per component on the GPU. */
export const ACC_COMPS = 8;
/** Fixed point scales. The force scale resolves f32 population round-off; the moment scale leaves
 *  head room for a lever arm of a few thousand cells before a single link can overflow i32. */
export const SCALE_F = 67108864; // 2^26
export const SCALE_M = 262144; // 2^18

/** Link record: u32 dst, srcA, srcB, partDir; f32 a, b, mw, ux, uy, uz, rx, ry, rz. */
export const LINK_WORDS = 13;
/** Cell change record: u32 cell, mask, kindPart, pad; f32 ux, uy, uz, rx, ry, rz, pad, pad. */
export const CELL_WORDS = 12;
export const CELL_COVERED = 0, CELL_FRESH = 1, CELL_SWEPT = 2;

export function feqShifted(i, drho, ux, uy, uz) {
  const cu = CX[i] * ux + CY[i] * uy + CZ[i] * uz;
  const uu = ux * ux + uy * uy + uz * uz;
  return W[i] * (drho + (1 + drho) * (3 * cu + 4.5 * cu * cu - 1.5 * uu));
}

/** Solve for the total relaxation time with the single-pass local Smagorinsky closure. */
export function smagTauTotal(piNorm, rho, tau0, csSmag) {
  const A = (3 / Math.SQRT2) * csSmag * csSmag * piNorm / Math.max(rho, 1e-9);
  const nuT = (-tau0 + Math.sqrt(tau0 * tau0 + 12 * A)) / 6;
  return tau0 + 3 * Math.max(0, nuT);
}
