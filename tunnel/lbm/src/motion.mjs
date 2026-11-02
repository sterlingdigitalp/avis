// Prescribed-motion surfaces in lattice units. Flow runs toward -x (body frame x forward, y up,
// z right) so a plate's leading edge is on its +x side, matching SPEC.md.

/** Rectangular plate mesh: rows along chord, columns along span. Returns tris and plate coords. */
export function plateMesh(chord, zMin, zMax, nChord = 8, nSpan = 2) {
  const nv = (nChord + 1) * (nSpan + 1);
  const s = new Float64Array(nv), z = new Float64Array(nv);
  for (let j = 0; j <= nSpan; j++) for (let i = 0; i <= nChord; i++) {
    const v = j * (nChord + 1) + i;
    s[v] = chord * (0.5 - i / nChord); // +chord/2 at the leading edge
    z[v] = zMin + (zMax - zMin) * (j / nSpan);
  }
  const tris = new Uint32Array(nChord * nSpan * 6);
  let t = 0;
  for (let j = 0; j < nSpan; j++) for (let i = 0; i < nChord; i++) {
    const a = j * (nChord + 1) + i, b = a + 1, c = a + nChord + 1, d = c + 1;
    tris[t++] = a; tris[t++] = b; tris[t++] = d;
    tris[t++] = a; tris[t++] = d; tris[t++] = c;
  }
  return { s, z, tris, nv };
}

/**
 * Plate at (x0,y0) doing heave h(t) and pitch alpha(t) about a point `axis` chord-lengths aft of
 * the mid-chord (the Theodorsen a*b convention divided by chord: quarter-chord is -0.25).
 * @param m {h(t),hd(t),alpha(t),alphad(t)} in lattice units per lattice step
 */
export function rigidPlateSurface({ part = 0, kind = 'thin', thickness, chord, zMin, zMax, x0, y0, axis = 0, nChord, nSpan, m, mirrorZ = false }) {
  const mesh = plateMesh(chord, zMin, zMax, nChord, nSpan);
  const nv = mesh.nv;
  const pos = new Float64Array(nv * 3), vel = new Float64Array(nv * 3);
  const sAxis = -axis * chord; // axis location in plate coordinate (s positive toward the leading edge)
  const axisX = x0 + sAxis, axisY = y0;
  const pose = (t) => {
    const h = m.h(t), hd = m.hd(t), al = m.alpha(t), ald = m.alphad(t);
    const ca = Math.cos(al), sa = Math.sin(al);
    for (let v = 0; v < nv; v++) {
      const rs = mesh.s[v] - sAxis;
      pos[v * 3] = axisX + rs * ca;
      pos[v * 3 + 1] = axisY + h + rs * sa;
      pos[v * 3 + 2] = mesh.z[v];
      vel[v * 3] = -rs * sa * ald;
      vel[v * 3 + 1] = hd + rs * ca * ald;
      vel[v * 3 + 2] = 0;
    }
    return { pos, vel };
  };
  const velocityAt = (ix, iy, iz, t) => {
    const al = m.alpha(t), ald = m.alphad(t), h = m.h(t), hd = m.hd(t);
    const rx = ix - axisX, ry = iy - (axisY + h);
    return [-ry * ald, hd + rx * ald, 0];
  };
  return {
    part, kind, thickness, tris: mesh.tris, mirrorZ,
    moving: true, pose, velocityAt, ref: (t) => [axisX, axisY + m.h(t), 0],
  };
}

export const noMotion = { h: () => 0, hd: () => 0, alpha: () => 0, alphad: () => 0 };

/** Sinusoidal heave h0 sin(omega t) and pitch alpha0 sin(omega t + phi), optional mean pitch. */
export function harmonic({ h0 = 0, alpha0 = 0, omega, phi = 0, alphaMean = 0, rampSteps = 0 }) {
  // A short smooth ramp avoids starting with a velocity discontinuity; it has decayed by the time
  // any measured cycle starts.
  const R = (t) => (rampSteps > 0 && t < rampSteps ? 0.5 - 0.5 * Math.cos((Math.PI * t) / rampSteps) : 1);
  const Rd = (t) => (rampSteps > 0 && t < rampSteps ? ((0.5 * Math.PI) / rampSteps) * Math.sin((Math.PI * t) / rampSteps) : 0);
  return {
    h: (t) => R(t) * h0 * Math.sin(omega * t),
    hd: (t) => Rd(t) * h0 * Math.sin(omega * t) + R(t) * h0 * omega * Math.cos(omega * t),
    alpha: (t) => alphaMean + R(t) * alpha0 * Math.sin(omega * t + phi),
    alphad: (t) => Rd(t) * alpha0 * Math.sin(omega * t + phi) + R(t) * alpha0 * omega * Math.cos(omega * t + phi),
  };
}
