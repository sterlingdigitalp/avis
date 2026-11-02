// Pure-JS reference for the GPU solver. Same algorithm, same step order and, in f32 mode, the same
// floating-point operations in the same order as the WGSL in wgsl.mjs, so the two can be compared
// to round-off. In f64 mode the same code measures how much of any answer is f32 round-off.
//
// One step:
//   1. Cell changes from the geometry frame: covered, fresh and swept cells. Refill reads only
//      neighbours that are fluid, uncut and not themselves changing, so it has no order dependence.
//   2. Link phase 1: compute the bounced value of every cut link from post-collision populations,
//      and the momentum exchange. Link phase 2: write it into the slot the pull will read.
//   3. Pull and collide every fluid cell from src into dst, then swap.
//
// Populations are stored shifted, f' = f - w, which keeps far more f32 precision at low Mach.
import { Q, CX, CY, CZ, W, OPP, MIRROR_Y, MIRROR_Z, AXIS, FLUID, MAX_PARTS, ACC_COMPS, SCALE_F, SCALE_M, LINK_WORDS, CELL_WORDS, CELL_COVERED, CELL_FRESH } from './lattice.mjs';

export function roundEven(x) {
  const r = Math.round(x);
  return Math.abs(x % 1) === 0.5 && r % 2 !== 0 ? r - 1 : r;
}

export class CpuLbm {
  constructor(cfg) {
    this.cfg = cfg;
    const { nx, ny, nz } = cfg;
    this.nx = nx; this.ny = ny; this.nz = nz;
    this.n = nx * ny * nz;
    this.f32 = cfg.precision !== 'f64';
    const A = this.f32 ? Float32Array : Float64Array;
    this.fr = this.f32 ? Math.fround : (x) => x;
    this.w = W.map((v) => this.fr(v));
    this.src = new A(this.n * Q);
    this.dst = new A(this.n * Q);
    this.mask = cfg.mask ?? null;
    this.acc = new Float64Array(MAX_PARTS * ACC_COMPS);
    this.frame = null;
    this.linkCount = 0;
    this.linkI = new Uint32Array(0);
    this.linkF = new Float32Array(0);
    this.linkVal = new Float64Array(0);
    this.steps = 0;
    this.xMode = cfg.xMode ?? AXIS.inflow;
    this.yLowMode = cfg.yLowMode ?? AXIS.freeslip;
    this.yHighMode = cfg.yHighMode ?? AXIS.freeslip;
    this.zLowMode = cfg.zLowMode ?? AXIS.freeslip;
    this.zHighMode = cfg.zHighMode ?? AXIS.freeslip;
    this.U = this.inletU();
    this.fillEquilibrium(cfg.u0 ?? [0, 0, 0]);
  }

  inletU() {
    const { inletU = [0, 0, 0], inletRampSteps = 0 } = this.cfg;
    const frac = inletRampSteps > 0 ? Math.min(1, this.steps / inletRampSteps) : 1;
    return [inletU[0] * frac, inletU[1] * frac, inletU[2] * frac];
  }

  fillEquilibrium(u0, drho = 0) { this.setField(() => ({ drho, u: u0 })); }

  /** Set post-collision state to equilibrium of (drho, u) per cell; used for analytic initial data. */
  setField(fn) {
    const { nx, ny, nz, fr } = this;
    for (let iz = 0, c = 0; iz < nz; iz++) for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++, c++) {
      const { drho, u } = fn(ix, iy, iz);
      const ux = fr(u[0]), uy = fr(u[1]), uz = fr(u[2]), dr = fr(drho), rho = fr(1 + dr);
      for (let i = 0; i < Q; i++) this.src[c * Q + i] = this.feqShifted(i, dr, rho, ux, uy, uz, this.uu(ux, uy, uz));
    }
  }

  uu(ux, uy, uz) { const fr = this.fr; return fr(fr(fr(ux * ux) + fr(uy * uy)) + fr(uz * uz)); }

  feqShifted(i, drho, rho, ux, uy, uz, uu) {
    const fr = this.fr;
    let cu = 0;
    if (CX[i]) cu = CX[i] > 0 ? ux : -ux;
    if (CY[i]) cu = fr(cu + (CY[i] > 0 ? uy : -uy));
    if (CZ[i]) cu = fr(cu + (CZ[i] > 0 ? uz : -uz));
    const t = fr(fr(fr(3 * cu) + fr(4.5 * fr(cu * cu))) - fr(1.5 * uu));
    return fr(this.w[i] * fr(drho + fr(rho * t)));
  }

  /** Install a boundary frame. Cell changes run once at the next step; links persist. */
  setFrame(frame) {
    this.frame = frame;
    this.linkCount = frame.linkCount;
    const lb = frame.linkBytes;
    const copy = lb.slice().buffer;
    this.linkI = new Uint32Array(copy);
    this.linkF = new Float32Array(copy);
    if (this.linkVal.length < frame.linkCount) this.linkVal = new Float64Array(frame.linkCount * 2);
    const cb = frame.cellBytes.slice().buffer;
    this.cellCount = frame.initial ? 0 : frame.cellCount;
    this.cellI = new Uint32Array(cb);
    this.cellF = new Float32Array(cb);
  }

  addAcc(part, comp, v, scale) { this.acc[part * ACC_COMPS + comp] += roundEven(this.fr(v * scale)); }

  addForceMoment(part, Fx, Fy, Fz, rx, ry, rz) {
    const fr = this.fr;
    this.addAcc(part, 0, Fx, SCALE_F);
    this.addAcc(part, 1, Fy, SCALE_F);
    this.addAcc(part, 2, Fz, SCALE_F);
    this.addAcc(part, 3, fr(fr(ry * Fz) - fr(rz * Fy)), SCALE_M);
    this.addAcc(part, 4, fr(fr(rz * Fx) - fr(rx * Fz)), SCALE_M);
    this.addAcc(part, 5, fr(fr(rx * Fy) - fr(ry * Fx)), SCALE_M);
  }

  /** Return and clear the accumulated impulses for each part, in lattice units summed over steps. */
  takeForces() {
    const out = [];
    for (let p = 0; p < MAX_PARTS; p++) {
      const o = p * ACC_COMPS, a = this.acc;
      out.push({ F: [a[o] / SCALE_F, a[o + 1] / SCALE_F, a[o + 2] / SCALE_F], M: [a[o + 3] / SCALE_M, a[o + 4] / SCALE_M, a[o + 5] / SCALE_M], dMass: a[o + 6] / SCALE_F });
    }
    this.acc.fill(0);
    return out;
  }

  runCellChanges() {
    const { fr, src, nx } = this;
    const nxy = this.nx * this.ny;
    const I = this.cellI, F = this.cellF;
    for (let r = 0; r < this.cellCount; r++) {
      const o = r * CELL_WORDS;
      const c = I[o], m = I[o + 1], part = I[o + 2] & 255, kind = I[o + 2] >>> 8;
      const ux = fr(F[o + 4]), uy = fr(F[o + 5]), uz = fr(F[o + 6]);
      const rx = fr(F[o + 7]), ry = fr(F[o + 8]), rz = fr(F[o + 9]);
      const base = c * Q;
      let oldDrho = 0, oldPx = 0, oldPy = 0, oldPz = 0;
      if (kind !== CELL_FRESH) {
        for (let i = 0; i < Q; i++) {
          const v = src[base + i];
          oldDrho = fr(oldDrho + v);
          if (CX[i]) oldPx = fr(oldPx + (CX[i] > 0 ? v : -v));
          if (CY[i]) oldPy = fr(oldPy + (CY[i] > 0 ? v : -v));
          if (CZ[i]) oldPz = fr(oldPz + (CZ[i] > 0 ? v : -v));
        }
      }
      if (kind === CELL_COVERED) {
        this.addForceMoment(part, oldPx, oldPy, oldPz, rx, ry, rz);
        this.addAcc(part, 6, -oldDrho, SCALE_F);
        continue;
      }
      let count = 0, sumRho = 0, Sxx = 0, Syy = 0, Szz = 0, Sxy = 0, Sxz = 0, Syz = 0;
      for (let j = 1; j < Q; j++) {
        if (!((m >>> j) & 1)) continue;
        const nb = (c + CX[j] + CY[j] * nx + CZ[j] * nxy) * Q;
        let dr = 0, jx = 0, jy = 0, jz = 0, pxx = 0, pyy = 0, pzz = 0, pxy = 0, pxz = 0, pyz = 0;
        for (let i = 0; i < Q; i++) {
          const v = src[nb + i];
          dr = fr(dr + v);
          if (CX[i]) { jx = fr(jx + (CX[i] > 0 ? v : -v)); pxx = fr(pxx + v); }
          if (CY[i]) { jy = fr(jy + (CY[i] > 0 ? v : -v)); pyy = fr(pyy + v); }
          if (CZ[i]) { jz = fr(jz + (CZ[i] > 0 ? v : -v)); pzz = fr(pzz + v); }
          if (CX[i] && CY[i]) pxy = fr(pxy + (CX[i] * CY[i] > 0 ? v : -v));
          if (CX[i] && CZ[i]) pxz = fr(pxz + (CX[i] * CZ[i] > 0 ? v : -v));
          if (CY[i] && CZ[i]) pyz = fr(pyz + (CY[i] * CZ[i] > 0 ? v : -v));
        }
        const rho = fr(1 + dr), inv = fr(1 / rho);
        const vx = fr(jx * inv), vy = fr(jy * inv), vz = fr(jz * inv);
        const third = fr(dr * fr(1 / 3));
        sumRho = fr(sumRho + dr);
        Sxx = fr(Sxx + fr(fr(pxx - third) - fr(fr(rho * vx) * vx)));
        Syy = fr(Syy + fr(fr(pyy - third) - fr(fr(rho * vy) * vy)));
        Szz = fr(Szz + fr(fr(pzz - third) - fr(fr(rho * vz) * vz)));
        Sxy = fr(Sxy + fr(pxy - fr(fr(rho * vx) * vy)));
        Sxz = fr(Sxz + fr(pxz - fr(fr(rho * vx) * vz)));
        Syz = fr(Syz + fr(pyz - fr(fr(rho * vy) * vz)));
        count++;
      }
      let dr, Pxx = 0, Pyy = 0, Pzz = 0, Pxy = 0, Pxz = 0, Pyz = 0;
      if (count > 0) {
        const k = fr(1 / count);
        dr = fr(sumRho * k);
        Pxx = fr(Sxx * k); Pyy = fr(Syy * k); Pzz = fr(Szz * k); Pxy = fr(Sxy * k); Pxz = fr(Sxz * k); Pyz = fr(Syz * k);
      } else dr = oldDrho;
      const rho = fr(1 + dr), uu = this.uu(ux, uy, uz);
      const t3 = fr(fr(fr(Pxx + Pyy) + Pzz) * fr(1 / 3));
      let newDrho = 0, newPx = 0, newPy = 0, newPz = 0;
      for (let i = 0; i < Q; i++) {
        const v = fr(this.feqShifted(i, dr, rho, ux, uy, uz, uu) + this.regNeq(i, Pxx, Pyy, Pzz, Pxy, Pxz, Pyz, t3));
        src[base + i] = v;
        newDrho = fr(newDrho + v);
        if (CX[i]) newPx = fr(newPx + (CX[i] > 0 ? v : -v));
        if (CY[i]) newPy = fr(newPy + (CY[i] > 0 ? v : -v));
        if (CZ[i]) newPz = fr(newPz + (CZ[i] > 0 ? v : -v));
      }
      this.addForceMoment(part, fr(oldPx - newPx), fr(oldPy - newPy), fr(oldPz - newPz), rx, ry, rz);
      this.addAcc(part, 6, fr(newDrho - oldDrho), SCALE_F);
    }
  }

  /** w_i * 4.5 * (c_i c_i : P - trP/3), exploiting that P is symmetric. */
  regNeq(i, Pxx, Pyy, Pzz, Pxy, Pxz, Pyz, t3) {
    const fr = this.fr;
    const cx = CX[i], cy = CY[i], cz = CZ[i];
    let q = 0;
    if (cx) q = Pxx;
    if (cy) q = cx ? fr(q + Pyy) : Pyy;
    if (cz) q = cx || cy ? fr(q + Pzz) : Pzz;
    if (cx && cy) q = cx * cy > 0 ? fr(q + fr(2 * Pxy)) : fr(q - fr(2 * Pxy));
    if (cx && cz) q = cx * cz > 0 ? fr(q + fr(2 * Pxz)) : fr(q - fr(2 * Pxz));
    if (cy && cz) q = cy * cz > 0 ? fr(q + fr(2 * Pyz)) : fr(q - fr(2 * Pyz));
    q = fr(q - t3);
    return fr(fr(this.w[i] * 4.5) * q);
  }

  runLinks() {
    const { fr, src, w } = this;
    const I = this.linkI, F = this.linkF, val = this.linkVal;
    const gi = this.cfg.giTerm ?? true;
    const localRho = this.cfg.localRho ?? true;
    for (let l = 0; l < this.linkCount; l++) {
      const o = l * LINK_WORDS;
      const pd = I[o + 3], part = pd & 255, d = pd >>> 8;
      const a = fr(F[o + 4]), b = fr(F[o + 5]), mw = fr(F[o + 6]);
      const fA = src[I[o + 1]], fB = src[I[o + 2]];
      // Wall term with the local density (Ladd used rho0). With rho0 the mass one side of a moving
      // sheet gains through its links differs from what it loses through swept nodes by
      // (rho_local - rho0) u, which adds up to a steady drift proportional to pressure work.
      let wall = mw;
      if (localRho) {
        const base = (I[o + 1] - d) ;
        let dr = 0;
        for (let i = 0; i < Q; i++) dr = fr(dr + src[base + i]);
        wall = fr(fr(1 + dr) * mw);
      }
      const out = fr(fr(fr(a * fA) + fr(b * fB)) + wall);
      val[l] = out;
      const sum = fr(fr(fA + out) + fr(2 * w[d]));
      let Fx = CX[d] ? (CX[d] > 0 ? sum : -sum) : 0;
      let Fy = CY[d] ? (CY[d] > 0 ? sum : -sum) : 0;
      let Fz = CZ[d] ? (CZ[d] > 0 ? sum : -sum) : 0;
      const dm = fr(out - fA);
      if (gi) {
        Fx = fr(Fx + fr(F[o + 7] * dm));
        Fy = fr(Fy + fr(F[o + 8] * dm));
        Fz = fr(Fz + fr(F[o + 9] * dm));
      }
      this.addForceMoment(part, Fx, Fy, Fz, F[o + 10], F[o + 11], F[o + 12]);
      this.addAcc(part, 6, dm, SCALE_F);
    }
    for (let l = 0; l < this.linkCount; l++) src[I[l * LINK_WORDS]] = val[l];
  }

  /** Read one population for an edge cell, following the boundary rule of each axis. */
  pullEdge(ix, iy, iz, c, i, drhoC, rhoC, uxC, uyC, uzC, uuC) {
    const { nx, ny, nz, src } = this;
    const fr = this.fr;
    let sx = ix - CX[i], sy = iy - CY[i], sz = iz - CZ[i];
    let d = i;
    if (sx < 0 || sx >= nx) {
      if (this.xMode === AXIS.periodic) sx = (sx + nx) % nx;
      else if (this.xMode === AXIS.inflow) {
        if (sx >= nx) {
          const U = this.U;
          return this.feqShifted(i, drhoC, rhoC, fr(U[0]), fr(U[1]), fr(U[2]), this.uu(fr(U[0]), fr(U[1]), fr(U[2])));
        }
        return this.feqShifted(i, drhoC, rhoC, uxC, uyC, uzC, uuC);
      } else return src[c * Q + OPP[i]];
    }
    if (sy < 0 || sy >= ny) {
      const m = sy < 0 ? this.yLowMode : this.yHighMode;
      if (m === AXIS.periodic) sy = (sy + ny) % ny;
      else if (m === AXIS.freeslip) { sy = sy < 0 ? -1 - sy : 2 * ny - 1 - sy; d = MIRROR_Y[d]; }
      else return src[c * Q + OPP[i]];
    }
    if (sz < 0 || sz >= nz) {
      const m = sz < 0 ? this.zLowMode : this.zHighMode;
      if (m === AXIS.periodic) sz = (sz + nz) % nz;
      else if (m === AXIS.freeslip) { sz = sz < 0 ? -1 - sz : 2 * nz - 1 - sz; d = MIRROR_Z[d]; }
      else return src[c * Q + OPP[i]];
    }
    return src[((sz * ny + sy) * nx + sx) * Q + d];
  }

  spongeAt(ix, iy, iz) {
    const s = this.cfg.sponge;
    if (!s || !(s.strength > 0)) return 0;
    const { nx, ny, nz, fr } = this;
    let t = 0;
    const face = (dist, width) => { if (width > 0 && dist < width) t = Math.max(t, fr(1 - fr(dist / width))); };
    face(ix, s.xLow ?? 0); face(nx - 1 - ix, s.xHigh ?? 0);
    face(iy, s.yLow ?? 0); face(ny - 1 - iy, s.yHigh ?? 0);
    face(iz, s.zLow ?? 0); face(nz - 1 - iz, s.zHigh ?? 0);
    return t > 0 ? fr(fr(s.strength * fr(t * t)) * fr(3 - fr(2 * t))) : 0;
  }

  step(count = 1) {
    for (let s = 0; s < count; s++) this.stepOnce();
  }

  stepOnce() {
    const { nx, ny, nz, fr, src, dst, w, mask } = this;
    const tau0 = fr(this.cfg.tau0), csSmag = fr(this.cfg.csSmag ?? 0);
    this.U = this.inletU();
    if (this.cellCount > 0) { this.runCellChanges(); this.cellCount = 0; }
    this.runLinks();
    const targetUx = fr(this.U[0]), targetUy = fr(this.U[1]), targetUz = fr(this.U[2]);
    const targetUU = this.uu(targetUx, targetUy, targetUz);
    const ff = new Float64Array(Q);
    const nxy = nx * ny;
    const drhoFix = fr(this.drhoFix ?? 0);
    this.drhoFix = 0;
    let c = -1;
    for (let iz = 0; iz < nz; iz++) for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++) {
      c++;
      if (mask && mask[c] !== FLUID) continue;
      const base = c * Q;
      const edge = ix === 0 || ix === nx - 1 || iy === 0 || iy === ny - 1 || iz === 0 || iz === nz - 1;
      if (edge) {
        let drC = 0, jx = 0, jy = 0, jz = 0;
        for (let i = 0; i < Q; i++) {
          const v = src[base + i];
          drC = fr(drC + v);
          if (CX[i]) jx = fr(jx + (CX[i] > 0 ? v : -v));
          if (CY[i]) jy = fr(jy + (CY[i] > 0 ? v : -v));
          if (CZ[i]) jz = fr(jz + (CZ[i] > 0 ? v : -v));
        }
        const rC = fr(1 + drC), invC = fr(1 / rC);
        const uxC = fr(jx * invC), uyC = fr(jy * invC), uzC = fr(jz * invC), uuC = this.uu(uxC, uyC, uzC);
        for (let i = 0; i < Q; i++) ff[i] = this.pullEdge(ix, iy, iz, c, i, drC, rC, uxC, uyC, uzC, uuC);
      } else {
        for (let i = 0; i < Q; i++) ff[i] = src[(c - CX[i] - CY[i] * nx - CZ[i] * nxy) * Q + i];
      }
      // Moments
      let drho = 0, jx = 0, jy = 0, jz = 0, sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
      for (let i = 0; i < Q; i++) {
        const v = ff[i];
        drho = fr(drho + v);
        if (CX[i]) { jx = fr(jx + (CX[i] > 0 ? v : -v)); sxx = fr(sxx + v); }
        if (CY[i]) { jy = fr(jy + (CY[i] > 0 ? v : -v)); syy = fr(syy + v); }
        if (CZ[i]) { jz = fr(jz + (CZ[i] > 0 ? v : -v)); szz = fr(szz + v); }
        if (CX[i] && CY[i]) sxy = fr(sxy + (CX[i] * CY[i] > 0 ? v : -v));
        if (CX[i] && CZ[i]) sxz = fr(sxz + (CX[i] * CZ[i] > 0 ? v : -v));
        if (CY[i] && CZ[i]) syz = fr(syz + (CY[i] * CZ[i] > 0 ? v : -v));
      }
      drho = fr(drho + drhoFix);
      const rho = fr(1 + drho), inv = fr(1 / rho);
      const ux = fr(jx * inv), uy = fr(jy * inv), uz = fr(jz * inv);
      const third = fr(drho * fr(1 / 3));
      const Pxx = fr(fr(sxx - third) - fr(fr(rho * ux) * ux));
      const Pyy = fr(fr(syy - third) - fr(fr(rho * uy) * uy));
      const Pzz = fr(fr(szz - third) - fr(fr(rho * uz) * uz));
      const Pxy = fr(sxy - fr(fr(rho * ux) * uy));
      const Pxz = fr(sxz - fr(fr(rho * ux) * uz));
      const Pyz = fr(syz - fr(fr(rho * uy) * uz));
      let tau = tau0;
      if (csSmag > 0) {
        const n2 = fr(fr(fr(fr(Pxx * Pxx) + fr(Pyy * Pyy)) + fr(Pzz * Pzz)) + fr(2 * fr(fr(fr(Pxy * Pxy) + fr(Pxz * Pxz)) + fr(Pyz * Pyz))));
        const piNorm = fr(Math.sqrt(n2));
        const A = fr(fr(fr(fr(3 / Math.SQRT2) * fr(csSmag * csSmag)) * piNorm) * inv);
        const nuT = fr(fr(fr(Math.sqrt(fr(fr(tau0 * tau0) + fr(12 * A)))) - tau0) * fr(1 / 6));
        tau = fr(tau0 + fr(3 * Math.max(0, nuT)));
      }
      const k = fr(1 - fr(1 / tau));
      const t3 = fr(fr(fr(Pxx + Pyy) + Pzz) * fr(1 / 3));
      const uu = this.uu(ux, uy, uz);
      const sp = this.spongeAt(ix, iy, iz);
      for (let i = 0; i < Q; i++) {
        let v = fr(this.feqShifted(i, drho, rho, ux, uy, uz, uu) + fr(k * this.regNeq(i, Pxx, Pyy, Pzz, Pxy, Pxz, Pyz, t3)));
        if (sp > 0) v = fr(fr(v * fr(1 - sp)) + fr(sp * this.feqShifted(i, 0, 1, targetUx, targetUy, targetUz, targetUU)));
        dst[base + i] = v;
      }
    }
    this.src = dst;
    this.dst = src;
    this.steps++;
  }

  /** Fluid mass excluding the constant 1 per cell, and the number of fluid cells. */
  massDelta() {
    let m = 0, cells = 0;
    for (let c = 0; c < this.n; c++) {
      if (this.mask && this.mask[c] !== FLUID) continue;
      cells++;
      for (let i = 0; i < Q; i++) m += this.src[c * Q + i];
    }
    return { deltaMass: m, cells };
  }

  momentum() {
    const p = [0, 0, 0];
    for (let c = 0; c < this.n; c++) {
      if (this.mask && this.mask[c] !== FLUID) continue;
      for (let i = 0; i < Q; i++) {
        const v = this.src[c * Q + i];
        p[0] += CX[i] * v; p[1] += CY[i] * v; p[2] += CZ[i] * v;
      }
    }
    return p;
  }

  /** rho and u at one cell from the post-collision state. */
  macroAt(ix, iy, iz) {
    const c = (iz * this.ny + iy) * this.nx + ix;
    let dr = 0, jx = 0, jy = 0, jz = 0;
    for (let i = 0; i < Q; i++) {
      const v = this.src[c * Q + i];
      dr += v; jx += CX[i] * v; jy += CY[i] * v; jz += CZ[i] * v;
    }
    const rho = 1 + dr;
    return { rho, u: [jx / rho, jy / rho, jz / rho] };
  }
}
