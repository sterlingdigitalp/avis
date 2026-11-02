// Unsteady vortex lattice method with a prescribed or free wake.
//
// Method: vortex rings on the mean surface (Katz & Plotkin, Low-Speed Aerodynamics, sec. 13.12), with
// the first wake ring taking the trailing-edge panel's strength from the previous step, and forces by the Joukowski segment form plus an unsteady term (Simpson, Palacios & Murua, J. Aircraft
// 2013). This form gets induced drag and leading edge suction right because the force on every bound
// segment is normal to the local velocity, including the induced velocity.
//
// The solver works in a frame that translates with the bird. The air moves through it at Vinf and the
// lattice points follow a pure function of time. Panel velocities come from central differences of
// that function, so no history is involved.
//
// Sign conventions: panel normals point up (+y for a flat wing), j runs in +z, and a ring with Gamma > 0
// carries lift: its leading segment runs in +z, so F = rho Gamma V x l is +y for V = -U x.

const FOUR_PI = 4 * Math.PI;

/**
 * Velocity induced at (px,py,pz) by n segments in S (ax,ay,az,bx,by,bz,gamma per segment), accumulated
 * into out. Regularised with a core of radius `core` (m): den = |r1 x r2|^2 + core^2 |r0|^2.
 */
export function segmentsVelocity(px, py, pz, S, n, core2, out) {
  let ux = 0, uy = 0, uz = 0;
  for (let i = 0, q = 0; i < n; i++, q += 7) {
    const g = S[q + 6];
    if (g === 0) continue;
    const ax = S[q], ay = S[q + 1], az = S[q + 2], bx = S[q + 3], by = S[q + 4], bz = S[q + 5];
    const r1x = px - ax, r1y = py - ay, r1z = pz - az;
    const r2x = px - bx, r2y = py - by, r2z = pz - bz;
    const cx = r1y * r2z - r1z * r2y;
    const cy = r1z * r2x - r1x * r2z;
    const cz = r1x * r2y - r1y * r2x;
    const c2 = cx * cx + cy * cy + cz * cz;
    const r0x = bx - ax, r0y = by - ay, r0z = bz - az;
    const den = c2 + core2 * (r0x * r0x + r0y * r0y + r0z * r0z);
    if (den < 1e-30) continue;
    const r1 = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z);
    const r2 = Math.sqrt(r2x * r2x + r2y * r2y + r2z * r2z);
    if (r1 < 1e-12 || r2 < 1e-12) continue;
    const dot = r0x * (r1x / r1 - r2x / r2) + r0y * (r1y / r1 - r2y / r2) + r0z * (r1z / r1 - r2z / r2);
    const k = (g * dot) / den;
    ux += k * cx;
    uy += k * cy;
    uz += k * cz;
  }
  out[0] += ux / FOUR_PI;
  out[1] += uy / FOUR_PI;
  out[2] += uz / FOUR_PI;
}

export function luFactor(A, n, piv) {
  for (let k = 0; k < n; k++) {
    let p = k;
    let big = Math.abs(A[k * n + k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(A[i * n + k]);
      if (v > big) {
        big = v;
        p = i;
      }
    }
    piv[k] = p;
    if (big === 0) throw new Error('singular influence matrix');
    if (p !== k) {
      for (let j = 0; j < n; j++) {
        const t = A[k * n + j];
        A[k * n + j] = A[p * n + j];
        A[p * n + j] = t;
      }
    }
    const d = A[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = A[i * n + k] / d;
      A[i * n + k] = f;
      if (f !== 0) {
        const ri = i * n;
        const rk = k * n;
        for (let j = k + 1; j < n; j++) A[ri + j] -= f * A[rk + j];
      }
    }
  }
}

export function luSolve(A, n, piv, b) {
  for (let k = 0; k < n; k++) {
    const p = piv[k];
    if (p !== k) {
      const t = b[k];
      b[k] = b[p];
      b[p] = t;
    }
    for (let i = k + 1; i < n; i++) b[i] -= A[i * n + k] * b[k];
  }
  for (let k = n - 1; k >= 0; k--) {
    let v = b[k];
    for (let j = k + 1; j < n; j++) v -= A[k * n + j] * b[j];
    b[k] = v / A[k * n + k];
  }
}

/**
 * surfaces: [{ nc, ns, grid(t, out) }], where grid writes (nc+1)*(ns+1)*3 body-frame points.
 * opts: { Vinf:[x,y,z], rho, dt, symmetric, wake:'prescribed'|'free', maxWakeRows, freeRows,
 *         core (m, bound vortices), wakeCore (m), moment:[x,y,z], t0 }
 * symmetric: mirror image about the plane z = 0; give only the right wing.
 */
export function createUvlm(surfaces, opts) {
  const rho = opts.rho;
  const dt = opts.dt;
  const Vinf = opts.Vinf;
  const sym = !!opts.symmetric;
  const wakeMode = opts.wake ?? 'prescribed';
  const maxRows = opts.maxWakeRows ?? 200;
  const freeRows = opts.freeRows ?? maxRows;
  const coreWing2 = (opts.core ?? 1e-6) ** 2;
  const coreWake2 = (opts.wakeCore ?? 1e-4) ** 2;
  const fd = opts.fdStep ?? dt * 0.05;
  const mp = opts.moment ?? [0, 0, 0];

  const S = surfaces.map((s, idx) => {
    const { nc, ns } = s;
    const nGrid = (nc + 1) * (ns + 1) * 3;
    const wakeLen = (maxRows + 2) * (ns + 1) * 3;
    return {
      idx, nc, ns, gridFn: s.grid,
      P: new Float64Array(nGrid), Pp: new Float64Array(nGrid), Pm: new Float64Array(nGrid),
      R: new Float64Array(nGrid), Rv: new Float64Array(nGrid),
      col: new Float64Array(nc * ns * 3), colV: new Float64Array(nc * ns * 3),
      normal: new Float64Array(nc * ns * 3), area: new Float64Array(nc * ns),
      gamma: new Float64Array(nc * ns), g1: new Float64Array(nc * ns), g2: new Float64Array(nc * ns),
      dGamma: new Float64Array(nc * ns),
      wakeW: new Float64Array(wakeLen), wakeTmp: new Float64Array(wakeLen),
      wakeG: new Float64Array((maxRows + 1) * ns),
      rows: 0,
      offset: 0,
      strip: { fx: new Float64Array(ns), fy: new Float64Array(ns), fz: new Float64Array(ns), area: new Float64Array(ns) },
    };
  });
  let N = 0;
  for (const s of S) {
    s.offset = N;
    N += s.nc * s.ns;
  }
  const A = new Float64Array(N * N);
  const piv = new Int32Array(N);
  const rhs = new Float64Array(N);
  const colAll = new Float64Array(N * 3);
  const nrmAll = new Float64Array(N * 3);
  let wakeSegs = new Float64Array(7 * 4096);
  let nWakeSegs = 0;
  let wingSegs = new Float64Array(7 * 4096);
  let nWingSegs = 0;
  // The newest wake line lies exactly on the wing's TE line, so it must use the wing's core: with the
  // wake's core the two would not cancel in a steady flow, which shows up as a grid-dependent lift.
  let headSegs = new Float64Array(7 * 512);
  let nHeadSegs = 0;

  let t = opts.t0 ?? 0;
  let step = 0;
  const vel = new Float64Array(3);
  const velM = new Float64Array(3);

  function velocityFromList(px, py, pz, list, n, core2, out) {
    segmentsVelocity(px, py, pz, list, n, core2, out);
    if (sym) {
      velM[0] = velM[1] = velM[2] = 0;
      segmentsVelocity(px, py, -pz, list, n, core2, velM);
      out[0] += velM[0];
      out[1] += velM[1];
      out[2] -= velM[2];
    }
  }

  function updateGeometry(time) {
    for (const s of S) {
      s.gridFn(time, s.P);
      s.gridFn(time + fd, s.Pp);
      s.gridFn(time - fd, s.Pm);
      const { nc, ns, P, Pp, Pm, R, Rv, col, colV, normal, area } = s;
      const n1 = ns + 1;
      for (let j = 0; j <= ns; j++) {
        for (let i = 0; i <= nc; i++) {
          const i0 = i < nc ? i : nc - 1;
          const f = i < nc ? 0.25 : 1.25; // quarter panel; the last row sits a quarter panel behind the TE
          const q = 3 * (i * n1 + j);
          const q0 = 3 * (i0 * n1 + j);
          const q1 = 3 * ((i0 + 1) * n1 + j);
          for (let d = 0; d < 3; d++) {
            R[q + d] = P[q0 + d] + f * (P[q1 + d] - P[q0 + d]);
            const pp = Pp[q0 + d] + f * (Pp[q1 + d] - Pp[q0 + d]);
            const pm = Pm[q0 + d] + f * (Pm[q1 + d] - Pm[q0 + d]);
            Rv[q + d] = (pp - pm) / (2 * fd);
          }
        }
      }
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j < ns; j++) {
          const m = 3 * (i * ns + j);
          const a = 3 * (i * n1 + j);
          const b = a + 3;
          const c = 3 * ((i + 1) * n1 + j);
          const e = c + 3;
          for (let d = 0; d < 3; d++) {
            col[m + d] = 0.125 * (P[a + d] + P[b + d]) + 0.375 * (P[c + d] + P[e + d]);
            const pp = 0.125 * (Pp[a + d] + Pp[b + d]) + 0.375 * (Pp[c + d] + Pp[e + d]);
            const pm = 0.125 * (Pm[a + d] + Pm[b + d]) + 0.375 * (Pm[c + d] + Pm[e + d]);
            colV[m + d] = (pp - pm) / (2 * fd);
          }
          const ax = P[e] - P[a], ay = P[e + 1] - P[a + 1], az = P[e + 2] - P[a + 2];
          const bx = P[b] - P[c], by = P[b + 1] - P[c + 1], bz = P[b + 2] - P[c + 2];
          const nx = ay * bz - az * by;
          const ny = az * bx - ax * bz;
          const nz = ax * by - ay * bx;
          const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
          area[i * ns + j] = 0.5 * l;
          normal[m] = nx / l;
          normal[m + 1] = ny / l;
          normal[m + 2] = nz / l;
        }
      }
      colAll.set(col, s.offset * 3);
      nrmAll.set(normal, s.offset * 3);
    }
  }

  function convectWake() {
    const free = wakeMode === 'free';
    if (free) {
      buildWingSegments();
      buildWakeSegments();
    }
    for (const s of S) {
      const n1 = s.ns + 1;
      for (let r = 0; r <= s.rows; r++) {
        for (let j = 0; j < n1; j++) {
          const q = 3 * (r * n1 + j);
          let vx = Vinf[0], vy = Vinf[1], vz = Vinf[2];
          if (free && r <= freeRows) {
            vel[0] = vel[1] = vel[2] = 0;
            velocityFromList(s.wakeW[q], s.wakeW[q + 1], s.wakeW[q + 2], wingSegs, nWingSegs, coreWake2, vel);
            velocityFromList(s.wakeW[q], s.wakeW[q + 1], s.wakeW[q + 2], wakeSegs, nWakeSegs, coreWake2, vel);
            velocityFromList(s.wakeW[q], s.wakeW[q + 1], s.wakeW[q + 2], headSegs, nHeadSegs, coreWake2, vel);
            vx += vel[0];
            vy += vel[1];
            vz += vel[2];
          }
          s.wakeTmp[q] = s.wakeW[q] + vx * dt;
          s.wakeTmp[q + 1] = s.wakeW[q + 1] + vy * dt;
          s.wakeTmp[q + 2] = s.wakeW[q + 2] + vz * dt;
        }
      }
    }
  }

  // Ring r of the wake lies between point rows r and r+1. Point row 0 is the extended TE line, ring 0
  // holds the TE strength of the previous step, so the line carries Gamma_TE(t-dt) - Gamma_TE(t): the
  // vorticity shed in the last step, a quarter panel behind the TE. Theodorsen's kernel sqrt((x+c)/x)
  // weights near-wake vorticity like x^-1/2, so this placement matters: holding it one step further back
  // gives errors that decay only like sqrt(dt), and the reason it was measured and rejected is in V1.
  function shiftWake() {
    for (const s of S) {
      const n1 = s.ns + 1;
      const ns = s.ns;
      if (step > 0) {
        const newRows = Math.min(s.rows + 1, maxRows);
        for (let r = newRows; r >= 1; r--) {
          const from = 3 * (r - 1) * n1;
          const to = 3 * r * n1;
          for (let k = 0; k < 3 * n1; k++) s.wakeW[to + k] = s.wakeTmp[from + k];
        }
        for (let r = newRows - 1; r >= 1; r--) {
          const from = (r - 1) * ns;
          const to = r * ns;
          for (let k = 0; k < ns; k++) s.wakeG[to + k] = s.wakeG[from + k];
        }
        for (let j = 0; j < ns; j++) s.wakeG[j] = s.gamma[(s.nc - 1) * ns + j];
        s.rows = newRows;
      }
      for (let j = 0; j < n1; j++) {
        const q = 3 * (s.nc * n1 + j);
        const w = 3 * j;
        s.wakeW[w] = s.R[q];
        s.wakeW[w + 1] = s.R[q + 1];
        s.wakeW[w + 2] = s.R[q + 2];
      }
    }
  }

  function ensure(list, n) {
    return list.length >= n * 7 ? list : new Float64Array(n * 7 * 2);
  }

  function buildWakeSegments() {
    let cnt = 0;
    let cntHead = 0;
    for (const s of S) {
      cnt += s.rows * s.ns + s.rows * (s.ns + 1);
      cntHead += s.ns;
    }
    wakeSegs = ensure(wakeSegs, cnt);
    headSegs = ensure(headSegs, cntHead);
    let q = 0;
    let h = 0;
    const put = (W, a, b, g) => {
      wakeSegs[q] = W[a]; wakeSegs[q + 1] = W[a + 1]; wakeSegs[q + 2] = W[a + 2];
      wakeSegs[q + 3] = W[b]; wakeSegs[q + 4] = W[b + 1]; wakeSegs[q + 5] = W[b + 2];
      wakeSegs[q + 6] = g;
      q += 7;
    };
    for (const s of S) {
      const { ns, rows, wakeW: W, wakeG: G } = s;
      const n1 = ns + 1;
      for (let j = 0; j < ns; j++) {
        const a = 3 * j;
        headSegs[h] = W[a]; headSegs[h + 1] = W[a + 1]; headSegs[h + 2] = W[a + 2];
        headSegs[h + 3] = W[a + 3]; headSegs[h + 4] = W[a + 4]; headSegs[h + 5] = W[a + 5];
        headSegs[h + 6] = rows > 0 ? G[j] : 0;
        h += 7;
      }
      for (let r = 1; r <= rows; r++) {
        for (let j = 0; j < ns; j++) {
          const behind = r < rows ? G[r * ns + j] : 0;
          const ahead = G[(r - 1) * ns + j];
          const a = 3 * (r * n1 + j);
          put(W, a, a + 3, behind - ahead);
        }
      }
      for (let r = 0; r < rows; r++) {
        for (let j = 0; j <= ns; j++) {
          const gL = j > 0 ? G[r * ns + j - 1] : 0;
          const gR = j < ns ? G[r * ns + j] : 0;
          const a = 3 * (r * n1 + j);
          put(W, a, a + 3 * n1, gL - gR);
        }
      }
    }
    nWakeSegs = q / 7;
    nHeadSegs = h / 7;
  }

  // All wing rings, including the TE line carrying -Gamma_TE (the wake list adds Gamma_TE(t-dt) there).
  function buildWingSegments() {
    let cnt = 0;
    for (const s of S) cnt += (s.nc + 1) * s.ns + s.nc * (s.ns + 1);
    wingSegs = ensure(wingSegs, cnt);
    let q = 0;
    const put = (R, a, b, g) => {
      wingSegs[q] = R[a]; wingSegs[q + 1] = R[a + 1]; wingSegs[q + 2] = R[a + 2];
      wingSegs[q + 3] = R[b]; wingSegs[q + 4] = R[b + 1]; wingSegs[q + 5] = R[b + 2];
      wingSegs[q + 6] = g;
      q += 7;
    };
    for (const s of S) {
      const { nc, ns, R, gamma } = s;
      const n1 = ns + 1;
      for (let i = 0; i <= nc; i++) {
        for (let j = 0; j < ns; j++) {
          const here = i < nc ? gamma[i * ns + j] : 0;
          const ahead = i > 0 ? gamma[(i - 1) * ns + j] : 0;
          const a = 3 * (i * n1 + j);
          put(R, a, a + 3, here - ahead);
        }
      }
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j <= ns; j++) {
          const gL = j > 0 ? gamma[i * ns + j - 1] : 0;
          const gR = j < ns ? gamma[i * ns + j] : 0;
          const a = 3 * (i * n1 + j);
          put(R, a, a + 3 * n1, gL - gR);
        }
      }
    }
    nWingSegs = q / 7;
  }

  const unit = new Float64Array(7 * 4);
  function assemble() {
    for (const src of S) {
      const { nc, ns, R } = src;
      const n1 = ns + 1;
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j < ns; j++) {
          const column = src.offset + i * ns + j;
          const ll = 3 * (i * n1 + j);
          const lr = ll + 3;
          const tl = 3 * ((i + 1) * n1 + j);
          const tr = tl + 3;
          const order = [ll, lr, lr, tr, tr, tl, tl, ll];
          for (let e = 0; e < 4; e++) {
            const m = 7 * e;
            const a = order[2 * e];
            const b = order[2 * e + 1];
            unit[m] = R[a]; unit[m + 1] = R[a + 1]; unit[m + 2] = R[a + 2];
            unit[m + 3] = R[b]; unit[m + 4] = R[b + 1]; unit[m + 5] = R[b + 2];
            unit[m + 6] = 1;
          }
          for (let tgt = 0; tgt < N; tgt++) {
            const c = 3 * tgt;
            vel[0] = vel[1] = vel[2] = 0;
            velocityFromList(colAll[c], colAll[c + 1], colAll[c + 2], unit, 4, coreWing2, vel);
            A[tgt * N + column] = vel[0] * nrmAll[c] + vel[1] * nrmAll[c + 1] + vel[2] * nrmAll[c + 2];
          }
        }
      }
    }
  }

  function buildRhs() {
    buildWakeSegments();
    for (const s of S) {
      const { nc, ns, col, colV, normal } = s;
      for (let i = 0; i < nc * ns; i++) {
        const m = 3 * i;
        vel[0] = vel[1] = vel[2] = 0;
        velocityFromList(col[m], col[m + 1], col[m + 2], wakeSegs, nWakeSegs, coreWake2, vel);
        velocityFromList(col[m], col[m + 1], col[m + 2], headSegs, nHeadSegs, coreWing2, vel);
        const vx = Vinf[0] - colV[m] + vel[0];
        const vy = Vinf[1] - colV[m + 1] + vel[1];
        const vz = Vinf[2] - colV[m + 2] + vel[2];
        rhs[s.offset + i] = -(vx * normal[m] + vy * normal[m + 1] + vz * normal[m + 2]);
      }
    }
  }

  function computeForces() {
    buildWingSegments();
    buildWakeSegments();
    let FsX = 0, FsY = 0, FsZ = 0, FuX = 0, FuY = 0, FuZ = 0;
    let Mx = 0, My = 0, Mz = 0;
    const addMoment = (x, y, z, fx, fy, fz) => {
      const dx = x - mp[0], dy = y - mp[1], dz = z - mp[2];
      Mx += dy * fz - dz * fy;
      My += dz * fx - dx * fz;
      Mz += dx * fy - dy * fx;
    };
    for (const s of S) {
      const { nc, ns, R, Rv, gamma, normal, area, dGamma, strip, col } = s;
      const n1 = ns + 1;
      strip.fx.fill(0);
      strip.fy.fill(0);
      strip.fz.fill(0);
      strip.area.fill(0);
      const jukowski = (a, b, g, jA, jB) => {
        if (g === 0) return;
        const mx = 0.5 * (R[a] + R[b]), my = 0.5 * (R[a + 1] + R[b + 1]), mz = 0.5 * (R[a + 2] + R[b + 2]);
        vel[0] = vel[1] = vel[2] = 0;
        velocityFromList(mx, my, mz, wingSegs, nWingSegs, coreWing2, vel);
        velocityFromList(mx, my, mz, wakeSegs, nWakeSegs, coreWake2, vel);
        velocityFromList(mx, my, mz, headSegs, nHeadSegs, coreWing2, vel);
        const vx = Vinf[0] - 0.5 * (Rv[a] + Rv[b]) + vel[0];
        const vy = Vinf[1] - 0.5 * (Rv[a + 1] + Rv[b + 1]) + vel[1];
        const vz = Vinf[2] - 0.5 * (Rv[a + 2] + Rv[b + 2]) + vel[2];
        const lx = R[b] - R[a], ly = R[b + 1] - R[a + 1], lz = R[b + 2] - R[a + 2];
        const fx = rho * g * (vy * lz - vz * ly);
        const fy = rho * g * (vz * lx - vx * lz);
        const fz = rho * g * (vx * ly - vy * lx);
        FsX += fx; FsY += fy; FsZ += fz;
        addMoment(mx, my, mz, fx, fy, fz);
        const wA = jA >= 0 && jA < ns ? (jB >= 0 && jB < ns && jB !== jA ? 0.5 : 1) : 0;
        const wB = jB >= 0 && jB < ns && jB !== jA ? (jA >= 0 && jA < ns ? 0.5 : 1) : 0;
        if (wA) { strip.fx[jA] += wA * fx; strip.fy[jA] += wA * fy; strip.fz[jA] += wA * fz; }
        if (wB) { strip.fx[jB] += wB * fx; strip.fy[jB] += wB * fy; strip.fz[jB] += wB * fz; }
      };
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j < ns; j++) {
          const ahead = i > 0 ? gamma[(i - 1) * ns + j] : 0;
          const a = 3 * (i * n1 + j);
          jukowski(a, a + 3, gamma[i * ns + j] - ahead, j, j);
        }
      }
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j <= ns; j++) {
          const gL = j > 0 ? gamma[i * ns + j - 1] : 0;
          const gR = j < ns ? gamma[i * ns + j] : 0;
          const a = 3 * (i * n1 + j);
          jukowski(a, a + 3 * n1, gL - gR, j - 1, j);
        }
      }
      // Unsteady pressure term: rho dGamma/dt A along the up-normal.
      for (let i = 0; i < nc; i++) {
        for (let j = 0; j < ns; j++) {
          const k = i * ns + j;
          const c = 3 * k;
          const f = rho * dGamma[k] * area[k];
          const fx = f * normal[c], fy = f * normal[c + 1], fz = f * normal[c + 2];
          FuX += fx; FuY += fy; FuZ += fz;
          addMoment(col[c], col[c + 1], col[c + 2], fx, fy, fz);
          strip.fx[j] += fx;
          strip.fy[j] += fy;
          strip.fz[j] += fz;
          strip.area[j] += area[k];
        }
      }
    }
    return {
      force: [FsX + FuX, FsY + FuY, FsZ + FuZ],
      forceSteady: [FsX, FsY, FsZ],
      forceUnsteady: [FuX, FuY, FuZ],
      moment: [Mx, My, Mz],
    };
  }

  /** Advance one time step, optionally computing forces. */
  function advance(withForces = true) {
    if (step > 0) convectWake();
    updateGeometry(t);
    shiftWake();
    assemble();
    luFactor(A, N, piv);
    buildRhs();
    luSolve(A, N, piv, rhs);
    for (const s of S) {
      const n = s.nc * s.ns;
      for (let k = 0; k < n; k++) {
        const g = rhs[s.offset + k];
        // BDF2 once two past values exist.
        if (step >= 2) s.dGamma[k] = (3 * g - 4 * s.g1[k] + s.g2[k]) / (2 * dt);
        else if (step === 1) s.dGamma[k] = (g - s.g1[k]) / dt;
        else s.dGamma[k] = 0;
        s.g2[k] = s.g1[k];
        s.g1[k] = g;
        s.gamma[k] = g;
      }
    }
    let f = null;
    if (withForces) {
      f = computeForces();
      f.t = t;
    }
    t += dt;
    step++;
    return f;
  }

  return {
    advance,
    get t() { return t; },
    get stepCount() { return step; },
    surfaces: S,
    /** Induced velocity at a point (probes and tests). */
    induced(px, py, pz) {
      buildWingSegments();
      buildWakeSegments();
      vel[0] = vel[1] = vel[2] = 0;
      velocityFromList(px, py, pz, wingSegs, nWingSegs, coreWing2, vel);
      velocityFromList(px, py, pz, wakeSegs, nWakeSegs, coreWake2, vel);
      velocityFromList(px, py, pz, headSegs, nHeadSegs, coreWing2, vel);
      return [vel[0], vel[1], vel[2]];
    },
  };
}

// ---------------------------------------------------------------------------------------------
// Harmonic post-processing helpers
// ---------------------------------------------------------------------------------------------

/** Mean and first-harmonic fit a0 + A cos(w t) + B sin(w t), over whole periods of a uniform series. */
export function fitHarmonic(ts, ys, omega) {
  let m = 0, a = 0, b = 0;
  const n = ys.length;
  for (let i = 0; i < n; i++) {
    m += ys[i];
    a += ys[i] * Math.cos(omega * ts[i]);
    b += ys[i] * Math.sin(omega * ts[i]);
  }
  return { mean: m / n, a: (2 * a) / n, b: (2 * b) / n, amp: (2 * Math.hypot(a, b)) / n, phase: Math.atan2(-b, a) };
}
