// Prescribed-motion boundary engine. Turns surfaces at time t into one BoundaryFrame: the cut-link
// list, the solid mask edits, and the cells whose state must change. The CPU reference and the GPU
// solver consume the same frame, so they share this code and differ only in the fluid kernel.
//
// All positions are in lattice units with cell (ix,iy,iz) centred at integer (ix,iy,iz). A physical
// point maps as (x - origin)/dx - 0.5, so a symmetry plane on the low face of z sits at z = -0.5.
//
// Surface kinds:
//   thin   zero-thickness triangle sheet. No solid cells. Every lattice link that crosses a triangle
//          is cut, on both sides, so the two faces of the wing are separate fluid regions.
//   slab   triangle sheet thickened to `thickness` cells: a cell is solid if its centre is within
//          thickness/2 of the sheet. Staircase Ladd bounce-back at q = 1/2.
//   closed watertight mesh. Cell solid if its centre is inside (ray parity). Links cut at the mesh.
import { Q, CX, CY, CZ, W, OPP, HALF, MIRROR_Y, MIRROR_Z, AXIS, FLUID, LINK_WORDS, CELL_WORDS, CELL_COVERED, CELL_FRESH, CELL_SWEPT } from './lattice.mjs';

const EPS_BARY = 1e-9;
const ncS = new Float64Array(9), ordS = new Int32Array(9);

class CutTable {
  constructor(capacity = 1 << 12) {
    this.alloc(capacity);
    this.size = 0;
    this.bits = new Int32Array(1 << 14);
    this.hmask = this.bits.length - 1;
    this.bits.fill(-1);
  }
  alloc(cap) {
    const grow = (Old, words) => { const a = new Old(cap * words); return a; };
    const oldKey = this.key, oldT = this.t, oldVel = this.vel, oldPart = this.part;
    this.key = grow(Float64Array, 1);
    this.t = grow(Float64Array, 1);
    this.vel = grow(Float32Array, 3);
    this.part = grow(Uint8Array, 1);
    if (oldKey) { this.key.set(oldKey); this.t.set(oldT); this.vel.set(oldVel); this.part.set(oldPart); }
    this.cap = cap;
  }
  clear() { this.size = 0; this.bits.fill(-1); }
  snapshot() {
    const n = this.size;
    return { size: n, bits: this.bits.slice(), key: this.key.slice(0, n), t: this.t.slice(0, n), vel: this.vel.slice(0, n * 3), part: this.part.slice(0, n) };
  }
  /** Back to the snapshot's entries, keeping the current (grown) hash size so it is not regrown
   *  every frame. The hash layout for each size is built once and cached on the snapshot. */
  restore(s) {
    this.size = s.size;
    this.key.set(s.key); this.t.set(s.t); this.vel.set(s.vel); this.part.set(s.part);
    s.layouts ??= new Map();
    let bits = s.layouts.get(this.bits.length);
    if (!bits) {
      const save = this.bits;
      this.bits = new Int32Array(save.length);
      this.hmask = save.length - 1;
      this.size = 0;
      this.bits.fill(-1);
      for (let e = 0; e < s.size; e++) {
        let h = hash(this.key[e]) & this.hmask;
        while (this.bits[h] !== -1) h = (h + 1) & this.hmask;
        this.bits[h] = e;
      }
      this.size = s.size;
      bits = this.bits.slice();
      s.layouts.set(save.length, bits);
    }
    this.hmask = bits.length - 1;
    this.bits.set(bits);
  }
  rehash() {
    const bits = new Int32Array(this.bits.length * 2).fill(-1);
    const m = bits.length - 1;
    for (let e = 0; e < this.size; e++) {
      let h = hash(this.key[e]) & m;
      while (bits[h] !== -1) h = (h + 1) & m;
      bits[h] = e;
    }
    this.bits = bits;
    this.hmask = m;
  }
  find(key) {
    let h = hash(key) & this.hmask;
    for (;;) {
      const e = this.bits[h];
      if (e === -1) return -1;
      if (this.key[e] === key) return e;
      h = (h + 1) & this.hmask;
    }
  }
  /** Keep the crossing nearest the node. */
  add(key, t, vx, vy, vz, part) {
    let h = hash(key) & this.hmask;
    for (;;) {
      const e = this.bits[h];
      if (e === -1) break;
      if (this.key[e] === key) {
        if (t < this.t[e]) { this.t[e] = t; this.vel[e * 3] = vx; this.vel[e * 3 + 1] = vy; this.vel[e * 3 + 2] = vz; this.part[e] = part; }
        return;
      }
      h = (h + 1) & this.hmask;
    }
    if (this.size === this.cap) this.alloc(this.cap * 2);
    const e = this.size++;
    this.bits[h] = e;
    this.key[e] = key;
    this.t[e] = t;
    this.vel[e * 3] = vx; this.vel[e * 3 + 1] = vy; this.vel[e * 3 + 2] = vz;
    this.part[e] = part;
    if (this.size * 2 > this.bits.length) this.rehash();
  }
}
function hash(k) {
  // Multiplicative hash on the low bits of a key < 2^33.
  const lo = (k % 4294967296) >>> 0;
  const m = Math.imul(lo, 2654435761) >>> 0;
  return (m ^ (m >>> 15)) | 0;
}

export class BoundaryEngine {
  /**
   * @param grid {nx,ny,nz,xMode,yLowMode,yHighMode,zLowMode,zHighMode} modes are AXIS values
   * @param surfaces [{part, kind, tris:Uint32Array, thickness?, mirrorZ?, moving, pose(t)->{pos,vel}, ref?(t)->[x,y,z]}]
   * @param opts {bouzidi:boolean}
   */
  constructor(grid, surfaces, opts = {}) {
    this.grid = grid;
    this.surfaces = surfaces;
    this.bouzidi = opts.bouzidi ?? true;
    const n = grid.nx * grid.ny * grid.nz;
    this.n = n;
    this.hasSolid = surfaces.some((s) => s.kind !== 'thin');
    this.mask = this.hasSolid ? new Uint8Array(n).fill(FLUID) : null;
    this.table = new CutTable();
    this.prev = surfaces.map(() => null);
    this.solidCells = surfaces.map(() => new Set());
    this.staticSnap = null;
    this.swept = new Map();
    this.linkBuf = new ArrayBuffer(LINK_WORDS * 4 * 1024);
    this.cellBuf = new ArrayBuffer(CELL_WORDS * 4 * 256);
    this.stats = { links: 0, cells: 0, swept: 0, covered: 0, fresh: 0, ms: 0 };
  }

  /** Establish geometry at time t with no state change implied (start or restart). */
  init(t) { return this.frame(t, true); }

  frame(t, initial = false) {
    const t0 = globalThis.performance?.now?.() ?? 0;
    const { surfaces, table } = this;
    table.clear();
    this.swept.clear();
    const poses = surfaces.map((s) => s.pose(t));
    this.lastT = t;
    this.lastPoses = poses;
    // Static surfaces are scanned once, then restored from a snapshot of the table each frame.
    const changes = [];
    if (this.staticSnap) table.restore(this.staticSnap);
    else {
      for (let si = 0; si < surfaces.length; si++) {
        const s = surfaces[si];
        if (s.moving) continue;
        if (s.kind === 'thin' || s.kind === 'closed') this.scanSurface(poses[si], s);
        if (s.kind !== 'thin') {
          const now = s.kind === 'slab' ? this.solidFromSheet(poses[si], s) : this.solidFromClosed(poses[si], s);
          this.diffSolid(si, now, s, poses[si], changes, true);
        }
      }
      for (let si = 0; si < surfaces.length; si++) if (!surfaces[si].moving && surfaces[si].kind !== 'thin') this.addStaircase(si);
      this.staticSnap = table.snapshot();
    }
    for (let si = 0; si < surfaces.length; si++) {
      const s = surfaces[si];
      if (!s.moving) continue;
      if (s.kind === 'thin' || s.kind === 'closed') this.scanSurface(poses[si], s);
      if (s.kind === 'thin' && !initial && this.prev[si]) this.scanSwept(this.prev[si], poses[si], s);
    }
    for (let si = 0; si < surfaces.length; si++) {
      const s = surfaces[si];
      if (s.kind === 'thin' || !s.moving) continue;
      const now = s.kind === 'slab' ? this.solidFromSheet(poses[si], s) : this.solidFromClosed(poses[si], s);
      this.diffSolid(si, now, s, poses[si], changes, initial);
    }
    for (let si = 0; si < surfaces.length; si++) if (surfaces[si].moving && surfaces[si].kind !== 'thin') this.addStaircase(si);
    const refs = surfaces.map((s) => (s.ref ? s.ref(t) : [0, 0, 0]));
    const refByPart = new Map();
    surfaces.forEach((s, i) => refByPart.set(s.part, refs[i]));
    const linkCount = this.assembleLinks(refByPart);
    const cellView = this.assembleCells(changes, poses, refByPart, initial);
    for (let i = 0; i < surfaces.length; i++) if (surfaces[i].moving) this.prev[i] = { pos: Float64Array.from(poses[i].pos) };
    else if (!this.prev[i]) this.prev[i] = { pos: Float64Array.from(poses[i].pos) };
    this.stats.links = linkCount;
    this.stats.ms = (globalThis.performance?.now?.() ?? 0) - t0;
    return {
      linkCount,
      linkBytes: new Uint8Array(this.linkBuf, 0, linkCount * LINK_WORDS * 4),
      cellCount: cellView.count,
      cellBytes: new Uint8Array(this.cellBuf, 0, cellView.count * CELL_WORDS * 4),
      maskEdits: cellView.maskEdits,
      initial,
    };
  }

  // ---- link scan -------------------------------------------------------------------------
  scanSurface(pose, s) {
    this.scanTris(pose.pos, pose.vel, s.tris, s.part, 1);
    if (s.mirrorZ) this.scanTris(pose.pos, pose.vel, s.tris, s.part, -1);
  }

  scanTris(pos, vel, tris, part, zSign) {
    const P = 3;
    const nT = tris.length / 3;
    for (let tI = 0; tI < nT; tI++) {
      const i0 = tris[tI * 3] * P, i1 = tris[tI * 3 + 1] * P, i2 = tris[tI * 3 + 2] * P;
      const ax = pos[i0], ay = pos[i0 + 1], az = zSign > 0 ? pos[i0 + 2] : -1 - pos[i0 + 2];
      const bx = pos[i1], by = pos[i1 + 1], bz = zSign > 0 ? pos[i1 + 2] : -1 - pos[i1 + 2];
      const cx = pos[i2], cy = pos[i2 + 1], cz = zSign > 0 ? pos[i2 + 2] : -1 - pos[i2 + 2];
      const v0x = bx - ax, v0y = by - ay, v0z = bz - az;
      const v1x = cx - ax, v1y = cy - ay, v1z = cz - az;
      const nnx = v0y * v1z - v0z * v1y, nny = v0z * v1x - v0x * v1z, nnz = v0x * v1y - v0y * v1x;
      const d00 = v0x * v0x + v0y * v0y + v0z * v0z, d01 = v0x * v1x + v0y * v1y + v0z * v1z, d11 = v1x * v1x + v1y * v1y + v1z * v1z;
      const den = d00 * d11 - d01 * d01;
      if (den < 1e-24) continue;
      const inv = 1 / den;
      const smax = Math.abs(nnx) + Math.abs(nny) + Math.abs(nnz);
      const R = this.clipBox(Math.min(ax, bx, cx), Math.max(ax, bx, cx), Math.min(ay, by, cy), Math.max(ay, by, cy), Math.min(az, bz, cz), Math.max(az, bz, cz), 1);
      if (!R) continue;
      const [x0, x1, y0, y1, z0, z1] = R;
      // Sweep along the axis the plane is most normal to: only a few nodes per column are near it.
      const anx = Math.abs(nnx), any = Math.abs(nny), anz = Math.abs(nnz);
      const dom = anx >= any && anx >= anz ? 0 : any >= anz ? 1 : 2;
      const a0 = dom === 0 ? y0 : x0, a1 = dom === 0 ? y1 : x1, b0 = dom === 2 ? y0 : z0, b1 = dom === 2 ? y1 : z1;
      const d0 = dom === 0 ? x0 : dom === 1 ? y0 : z0, d1 = dom === 0 ? x1 : dom === 1 ? y1 : z1;
      const nD = dom === 0 ? nnx : dom === 1 ? nny : nnz;
      const reach = smax / Math.abs(nD);
      // Edge lines of the triangle projected along the sweep axis. A column further than two cells
      // outside any of them cannot hold a link that reaches the triangle, since a link spans at most
      // sqrt(2) cells and projection only shortens distances.
      const pa = dom === 0 ? ay : ax, pb = dom === 2 ? ay : az, qa = dom === 0 ? by : bx, qb = dom === 2 ? by : bz, ra = dom === 0 ? cy : cx, rb = dom === 2 ? cy : cz;
      const areaSign = (qa - pa) * (rb - pb) - (qb - pb) * (ra - pa) >= 0 ? 1 : -1;
      const edge = (ea, eb, fa, fb) => { const dA = fa - ea, dB = fb - eb, L = Math.hypot(dA, dB) || 1; return [(-dB / L) * areaSign, (dA / L) * areaSign, ((dB * ea - dA * eb) / L) * areaSign]; };
      const E0 = edge(pa, pb, qa, qb), E1 = edge(qa, qb, ra, rb), E2 = edge(ra, rb, pa, pb);
      const MARGIN = -2;
      // Directions sorted by n.c: a node at plane value s0 crosses along d only if s0 + n.c_d
      // changes sign, so each node visits just the directions that can cut.
      for (let h = 0; h < 9; h++) { const d = HALF[h]; ncS[h] = nnx * CX[d] + nny * CY[d] + nnz * CZ[d]; ordS[h] = d; }
      for (let i = 1; i < 9; i++) for (let j = i; j > 0 && ncS[j - 1] > ncS[j]; j--) {
        const tv = ncS[j]; ncS[j] = ncS[j - 1]; ncS[j - 1] = tv;
        const td = ordS[j]; ordS[j] = ordS[j - 1]; ordS[j - 1] = td;
      }
      for (let ub = b0; ub <= b1; ub++) for (let ua = a0; ua <= a1; ua++) {
        if (E0[0] * ua + E0[1] * ub + E0[2] < MARGIN || E1[0] * ua + E1[1] * ub + E1[2] < MARGIN || E2[0] * ua + E2[1] * ub + E2[2] < MARGIN) continue;
        const rx = dom === 0 ? ax : ua, ry = dom === 1 ? ay : dom === 0 ? ua : ub, rz = dom === 2 ? az : ub;
        const centre = (dom === 0 ? ax : dom === 1 ? ay : az) - (nnx * (rx - ax) + nny * (ry - ay) + nnz * (rz - az)) / nD;
        const lo = Math.max(d0, Math.ceil(centre - reach)), hi = Math.min(d1, Math.floor(centre + reach));
        for (let ud = lo; ud <= hi; ud++) {
          const ix = dom === 0 ? ud : ua, iy = dom === 1 ? ud : dom === 0 ? ua : ub, iz = dom === 2 ? ud : ub;
          const s0 = nnx * (ix - ax) + nny * (iy - ay) + nnz * (iz - az);
          // Half-open plane test so a node exactly on the plane belongs to one side only:
          // s0 > 0 needs n.c <= -s0 (from the low end), s0 <= 0 needs n.c > -s0 (from the high end).
          const up = s0 <= 0;
          for (let k = 0; k < 9; k++) {
            const h = up ? 8 - k : k;
            const ncd = ncS[h];
            if (up ? !(ncd > -s0) : !(ncd <= -s0)) break;
            const d = ordS[h];
            const s1 = s0 + ncd;
            const tt = s0 / (s0 - s1);
            const hx = ix + tt * CX[d] - ax, hy = iy + tt * CY[d] - ay, hz = iz + tt * CZ[d] - az;
            const d20 = hx * v0x + hy * v0y + hz * v0z, d21 = hx * v1x + hy * v1y + hz * v1z;
            const beta = (d11 * d20 - d01 * d21) * inv, gamma = (d00 * d21 - d01 * d20) * inv;
            if (beta < -EPS_BARY || gamma < -EPS_BARY || beta + gamma > 1 + EPS_BARY) continue;
            const alpha = 1 - beta - gamma;
            const vx = alpha * vel[i0] + beta * vel[i1] + gamma * vel[i2];
            const vy = alpha * vel[i0 + 1] + beta * vel[i1 + 1] + gamma * vel[i2 + 1];
            const vz = zSign * (alpha * vel[i0 + 2] + beta * vel[i1 + 2] + gamma * vel[i2 + 2]);
            this.addEnd(ix, iy, iz, d, tt, vx, vy, vz, part);
            this.addEnd(ix + CX[d], iy + CY[d], iz + CZ[d], OPP[d], 1 - tt, vx, vy, vz, part);
          }
        }
      }
    }
  }

  /** Integer node box around an extent, one node past it, clipped to what can touch the domain. */
  clipBox(xa, xb, ya, yb, za, zb, pad) {
    const g = this.grid;
    const clip = (a, b, n, lo, hi) => {
      let p = Math.floor(a) - pad, q = Math.ceil(b) + pad;
      if (lo !== AXIS.periodic) p = Math.max(p, -1);
      if (hi !== AXIS.periodic) q = Math.min(q, n);
      return [p, q];
    };
    const [x0, x1] = clip(xa, xb, g.nx, g.xMode, g.xMode);
    const [y0, y1] = clip(ya, yb, g.ny, g.yLowMode, g.yHighMode);
    const [z0, z1] = clip(za, zb, g.nz, g.zLowMode, g.zHighMode);
    if (x0 > x1 || y0 > y1 || z0 > z1) return null;
    return [x0, x1, y0, y1, z0, z1];
  }

  wrapAxis(v, n, lo, hi) {
    if (v < 0) return lo === AXIS.periodic ? ((v % n) + n) % n : -1;
    if (v >= n) return hi === AXIS.periodic ? v % n : -1;
    return v;
  }

  addEnd(ix, iy, iz, d, t, vx, vy, vz, part) {
    const g = this.grid;
    const jx = this.wrapAxis(ix, g.nx, g.xMode, g.xMode);
    const jy = this.wrapAxis(iy, g.ny, g.yLowMode, g.yHighMode);
    const jz = this.wrapAxis(iz, g.nz, g.zLowMode, g.zHighMode);
    if (jx < 0 || jy < 0 || jz < 0) return;
    const c = (jz * g.ny + jy) * g.nx + jx;
    this.table.add(c * Q + d, t, vx, vy, vz, part);
  }

  /** Resolve the slot a pull at (ix,iy,iz) along direction i actually reads. Null if off-domain
   *  through a boundary that is not a pure re-index (inlet, outlet, bounce-back wall). */
  resolvePull(ix, iy, iz, i) {
    return this.resolvePop(ix - CX[i], iy - CY[i], iz - CZ[i], i);
  }

  /** The stored slot holding population `dir` at a possibly off-grid node, following mirrors. */
  resolvePop(sx, sy, sz, dir) {
    const g = this.grid;
    let d = dir;
    if (sx < 0 || sx >= g.nx) {
      if (g.xMode !== AXIS.periodic) return null;
      sx = ((sx % g.nx) + g.nx) % g.nx;
    }
    if (sy < 0 || sy >= g.ny) {
      const m = sy < 0 ? g.yLowMode : g.yHighMode;
      if (m === AXIS.periodic) sy = ((sy % g.ny) + g.ny) % g.ny;
      else if (m === AXIS.freeslip) { sy = sy < 0 ? -1 - sy : 2 * g.ny - 1 - sy; d = MIRROR_Y[d]; }
      else return null;
    }
    if (sz < 0 || sz >= g.nz) {
      const m = sz < 0 ? g.zLowMode : g.zHighMode;
      if (m === AXIS.periodic) sz = ((sz % g.nz) + g.nz) % g.nz;
      else if (m === AXIS.freeslip) { sz = sz < 0 ? -1 - sz : 2 * g.nz - 1 - sz; d = MIRROR_Z[d]; }
      else return null;
    }
    return ((sz * g.ny + sy) * g.nx + sx) * Q + d;
  }

  // ---- swept nodes -----------------------------------------------------------------------
  /** A node a moving sheet passed over between two frames holds the state of the other side. */
  scanSwept(prev, pose, s) {
    for (const sign of s.mirrorZ ? [1, -1] : [1]) this.scanSweptTris(prev.pos, pose.pos, pose.vel, s.tris, sign, s.part);
  }

  scanSweptTris(p0, p1, vel, tris, zSign, part) {
    const g = this.grid;
    const nT = tris.length / 3;
    const zz = (v) => (zSign > 0 ? v : -1 - v);
    for (let tI = 0; tI < nT; tI++) {
      const a = tris[tI * 3] * 3, b = tris[tI * 3 + 1] * 3, c = tris[tI * 3 + 2] * 3;
      const a0x = p0[a], a0y = p0[a + 1], a0z = zz(p0[a + 2]);
      const b0x = p0[b], b0y = p0[b + 1], b0z = zz(p0[b + 2]);
      const c0x = p0[c], c0y = p0[c + 1], c0z = zz(p0[c + 2]);
      const a1x = p1[a], a1y = p1[a + 1], a1z = zz(p1[a + 2]);
      const b1x = p1[b], b1y = p1[b + 1], b1z = zz(p1[b + 2]);
      const c1x = p1[c], c1y = p1[c + 1], c1z = zz(p1[c + 2]);
      const R = this.clipBox(Math.min(a0x, b0x, c0x, a1x, b1x, c1x), Math.max(a0x, b0x, c0x, a1x, b1x, c1x), Math.min(a0y, b0y, c0y, a1y, b1y, c1y), Math.max(a0y, b0y, c0y, a1y, b1y, c1y), Math.min(a0z, b0z, c0z, a1z, b1z, c1z), Math.max(a0z, b0z, c0z, a1z, b1z, c1z), 0);
      if (!R) continue;
      const [x0, x1, y0, y1, z0, z1] = R;
      const n0 = cross(b0x - a0x, b0y - a0y, b0z - a0z, c0x - a0x, c0y - a0y, c0z - a0z);
      const n1 = cross(b1x - a1x, b1y - a1y, b1z - a1z, c1x - a1x, c1y - a1y, c1z - a1z);
      const an1 = [Math.abs(n1[0]), Math.abs(n1[1]), Math.abs(n1[2])];
      const dom = an1[0] >= an1[1] && an1[0] >= an1[2] ? 0 : an1[1] >= an1[2] ? 1 : 2;
      const u0 = dom === 0 ? y0 : x0, u1 = dom === 0 ? y1 : x1, w0 = dom === 2 ? y0 : z0, w1 = dom === 2 ? y1 : z1;
      const d0 = dom === 0 ? x0 : dom === 1 ? y0 : z0, d1 = dom === 0 ? x1 : dom === 1 ? y1 : z1;
      for (let w = w0; w <= w1; w++) for (let u = u0; u <= u1; u++) {
        // Nodes between the two plane positions along this column, one node of margin.
        const qx = dom === 0 ? 0 : u, qy = dom === 1 ? 0 : dom === 0 ? u : w, qz = dom === 2 ? 0 : w;
        const at = (n, ox, oy, oz) => {
          const rest = n[0] * (qx - ox) + n[1] * (qy - oy) + n[2] * (qz - oz);
          return -rest / n[dom];
        };
        const c0 = n0[dom] !== 0 ? at(n0, a0x, a0y, a0z) : NaN, c1 = at(n1, a1x, a1y, a1z);
        const lo = Math.max(d0, Math.floor(Math.min(c0, c1)) - 1), hi = Math.min(d1, Math.ceil(Math.max(c0, c1)) + 1);
        for (let dd = Number.isFinite(lo) ? lo : d0; dd <= (Number.isFinite(hi) ? hi : d1); dd++) {
        const ix = dom === 0 ? dd : u, iy = dom === 1 ? dd : dom === 0 ? u : w, iz = dom === 2 ? dd : w;
        const s0 = n0[0] * (ix - a0x) + n0[1] * (iy - a0y) + n0[2] * (iz - a0z);
        const s1 = n1[0] * (ix - a1x) + n1[1] * (iy - a1y) + n1[2] * (iz - a1z);
        // Same half-open side convention as the link scan.
        if ((s0 > 0) === (s1 > 0)) continue;
        const tau = s0 / (s0 - s1);
        const ax = a0x + tau * (a1x - a0x), ay = a0y + tau * (a1y - a0y), az = a0z + tau * (a1z - a0z);
        const bx = b0x + tau * (b1x - b0x), by = b0y + tau * (b1y - b0y), bz = b0z + tau * (b1z - b0z);
        const cx = c0x + tau * (c1x - c0x), cy = c0y + tau * (c1y - c0y), cz = c0z + tau * (c1z - c0z);
        if (!insideTri(ix, iy, iz, ax, ay, az, bx, by, bz, cx, cy, cz)) continue;
        const ba = 1 - bary[0] - bary[1];
        const vx = ba * vel[a] + bary[0] * vel[b] + bary[1] * vel[c];
        const vy = ba * vel[a + 1] + bary[0] * vel[b + 1] + bary[1] * vel[c + 1];
        const vz = zSign * (ba * vel[a + 2] + bary[0] * vel[b + 2] + bary[1] * vel[c + 2]);
        const jx = this.wrapAxis(ix, g.nx, g.xMode, g.xMode), jy = this.wrapAxis(iy, g.ny, g.yLowMode, g.yHighMode), jz = this.wrapAxis(iz, g.nz, g.zLowMode, g.zHighMode);
        if (jx < 0 || jy < 0 || jz < 0) continue;
        this.swept.set((jz * g.ny + jy) * g.nx + jx, [vx, vy, vz, part]);
        }
      }
    }
  }

  // ---- solid masks -----------------------------------------------------------------------
  solidFromSheet(pose, s) {
    const out = new Map();
    const { nx, ny, nz } = this.grid;
    const R = s.thickness / 2, R2 = R * R;
    const pos = pose.pos, tris = s.tris;
    for (let tI = 0; tI < tris.length / 3; tI++) {
      const a = tris[tI * 3] * 3, b = tris[tI * 3 + 1] * 3, c = tris[tI * 3 + 2] * 3;
      const ax = pos[a], ay = pos[a + 1], az = pos[a + 2], bx = pos[b], by = pos[b + 1], bz = pos[b + 2], cx = pos[c], cy = pos[c + 1], cz = pos[c + 2];
      const x0 = Math.max(0, Math.floor(Math.min(ax, bx, cx) - R)), x1 = Math.min(nx - 1, Math.ceil(Math.max(ax, bx, cx) + R));
      const y0 = Math.max(0, Math.floor(Math.min(ay, by, cy) - R)), y1 = Math.min(ny - 1, Math.ceil(Math.max(ay, by, cy) + R));
      const z0 = Math.max(0, Math.floor(Math.min(az, bz, cz) - R)), z1 = Math.min(nz - 1, Math.ceil(Math.max(az, bz, cz) + R));
      for (let iz = z0; iz <= z1; iz++) for (let iy = y0; iy <= y1; iy++) for (let ix = x0; ix <= x1; ix++) {
        const key = (iz * ny + iy) * nx + ix;
        if (out.has(key)) continue;
        if (dist2PointTri(ix, iy, iz, ax, ay, az, bx, by, bz, cx, cy, cz) <= R2) out.set(key, tI);
      }
    }
    return out;
  }

  solidFromClosed(pose, s) {
    const out = new Map();
    const { nx, ny, nz } = this.grid;
    const pos = pose.pos, tris = s.tris;
    // Per scan line crossings of a ray along +x through the cell centres (tiny offset avoids edges).
    const lines = new Map();
    const oy = 1.3e-7, oz = 2.7e-7;
    for (let tI = 0; tI < tris.length / 3; tI++) {
      const a = tris[tI * 3] * 3, b = tris[tI * 3 + 1] * 3, c = tris[tI * 3 + 2] * 3;
      const ay = pos[a + 1], az = pos[a + 2], by = pos[b + 1], bz = pos[b + 2], cy = pos[c + 1], cz = pos[c + 2];
      const den = (by - ay) * (cz - az) - (cy - ay) * (bz - az);
      if (Math.abs(den) < 1e-14) continue;
      const y0 = Math.max(0, Math.ceil(Math.min(ay, by, cy))), y1 = Math.min(ny - 1, Math.floor(Math.max(ay, by, cy)));
      const z0 = Math.max(0, Math.ceil(Math.min(az, bz, cz))), z1 = Math.min(nz - 1, Math.floor(Math.max(az, bz, cz)));
      for (let iz = z0; iz <= z1; iz++) for (let iy = y0; iy <= y1; iy++) {
        const py = iy + oy - ay, pz = iz + oz - az;
        const v = (py * (cz - az) - (cy - ay) * pz) / den;
        const w = ((by - ay) * pz - py * (bz - az)) / den;
        if (v < 0 || w < 0 || v + w > 1) continue;
        const x = pos[a] + v * (pos[b] - pos[a]) + w * (pos[c] - pos[a]);
        const k = iz * ny + iy;
        let arr = lines.get(k);
        if (!arr) lines.set(k, (arr = []));
        arr.push(x);
      }
    }
    for (const [k, xs] of lines) {
      xs.sort((p, q) => p - q);
      for (let j = 0; j + 1 < xs.length; j += 2) {
        const x0 = Math.max(0, Math.ceil(xs[j])), x1 = Math.min(nx - 1, Math.floor(xs[j + 1]));
        for (let ix = x0; ix <= x1; ix++) out.set(k * nx + ix, -1);
      }
    }
    return out;
  }

  diffSolid(si, now, s, pose, changes, initial) {
    const old = this.solidCells[si];
    if (!initial) {
      for (const c of now.keys()) if (!old.has(c)) changes.push({ cell: c, kind: CELL_COVERED, part: s.part, si, tri: now.get(c) });
      for (const c of old.keys()) if (!now.has(c)) changes.push({ cell: c, kind: CELL_FRESH, part: s.part, si, tri: old.get(c) });
    }
    for (const c of old.keys()) this.mask[c] = FLUID;
    for (const c of now.keys()) this.mask[c] = s.part;
    this.solidCells[si] = now;
  }

  // ---- assembly --------------------------------------------------------------------------
  ensureLinks(count) {
    const need = count * LINK_WORDS * 4;
    if (this.linkBuf.byteLength < need) {
      const nb = new ArrayBuffer(need * 2);
      new Uint8Array(nb).set(new Uint8Array(this.linkBuf));
      this.linkBuf = nb;
    }
  }

  assembleLinks(refByPart) {
    const { table, grid } = this;
    const { nx, ny } = grid;
    const mask = this.mask;
    this.ensureLinks(table.size);
    const U = new Uint32Array(this.linkBuf), F = new Float32Array(this.linkBuf);
    let w = 0;
    const size = table.size, tKey = table.key, tT = table.t, tVel = table.vel, tPart = table.part;
    const solid = this.hasSolid, bouz = this.bouzidi, nxy = nx * ny;
    let lastPart = -1, ref = null;
    for (let e = 0; e < size; e++) {
      const key = tKey[e];
      const c = Math.floor(key / Q);
      const d = key - c * Q;
      if (solid && mask[c] !== FLUID) continue;
      const iz = Math.floor(c / nxy), rem = c - iz * nxy, iy = Math.floor(rem / nx), ix = rem - iy * nx;
      const i = OPP[d];
      const dst = this.resolvePull(ix, iy, iz, i);
      if (dst === null) continue; // link leaves through an inlet or outlet: geometry has no business there
      const q = Math.max(0, Math.min(1, tT[e]));
      const srcA = key;
      let a = 1, b = 0, srcB = srcA, coef = 1;
      if (bouz) {
        if (q < 0.5) {
          // Second fluid node behind this one, along the population's outgoing direction.
          const slotB = table.find(c * Q + i) >= 0 ? null : this.resolvePop(ix + CX[i], iy + CY[i], iz + CZ[i], d);
          if (slotB !== null && !(solid && mask[Math.floor(slotB / Q)] !== FLUID)) { a = 2 * q; b = 1 - 2 * q; srcB = slotB; }
        } else {
          const k = 1 / (2 * q);
          a = k; b = 1 - k; srcB = c * Q + i; coef = k;
        }
      }
      const vx = tVel[e * 3], vy = tVel[e * 3 + 1], vz = tVel[e * 3 + 2];
      const mw = coef * 6 * W[i] * (CX[i] * vx + CY[i] * vy + CZ[i] * vz);
      const part = tPart[e];
      if (part !== lastPart) { lastPart = part; ref = refByPart.get(part) ?? [0, 0, 0]; }
      const o = w * LINK_WORDS;
      U[o] = dst; U[o + 1] = srcA; U[o + 2] = srcB; U[o + 3] = part | (d << 8);
      F[o + 4] = a; F[o + 5] = b; F[o + 6] = mw;
      F[o + 7] = vx; F[o + 8] = vy; F[o + 9] = vz;
      F[o + 10] = ix + q * CX[d] - ref[0]; F[o + 11] = iy + q * CY[d] - ref[1]; F[o + 12] = iz + q * CZ[d] - ref[2];
      w++;
    }
    return w;
  }

  /** Staircase links for a solid: every fluid node next to a solid node that no mesh cut covers. */
  addStaircase(si) {
    const { table, grid } = this;
    const { nx, ny, nz } = grid;
    const s = this.surfaces[si];
    for (const [y, tri] of this.solidCells[si]) {
      const iz = Math.floor(y / (nx * ny)), iy = Math.floor(y / nx) % ny, ix = y % nx;
      for (let d = 1; d < Q; d++) {
        const jx = this.wrapAxis(ix + CX[d], nx, grid.xMode, grid.xMode), jy = this.wrapAxis(iy + CY[d], ny, grid.yLowMode, grid.yHighMode), jz = this.wrapAxis(iz + CZ[d], nz, grid.zLowMode, grid.zHighMode);
        if (jx < 0 || jy < 0 || jz < 0) continue;
        const c = (jz * ny + jy) * nx + jx;
        if (this.mask[c] !== FLUID) continue;
        const key = c * Q + OPP[d];
        if (table.find(key) >= 0) continue;
        const v = this.solidVelocityAt(si, tri, ix, iy, iz);
        table.add(key, 0.5, v[0], v[1], v[2], s.part);
      }
    }
  }

  isSolidNow(c) { return this.mask[c] !== FLUID; }

  /** Wall velocity at a solid cell: the surface's rigid-motion hook, else the triangle's mean. */
  solidVelocityAt(si, tri, ix, iy, iz) {
    const s = this.surfaces[si];
    if (!s.moving) return [0, 0, 0];
    if (s.velocityAt) return s.velocityAt(ix, iy, iz, this.lastT);
    if (tri < 0) return [0, 0, 0];
    const vel = this.lastPoses[si].vel;
    const a = s.tris[tri * 3] * 3, b = s.tris[tri * 3 + 1] * 3, c = s.tris[tri * 3 + 2] * 3;
    return [(vel[a] + vel[b] + vel[c]) / 3, (vel[a + 1] + vel[b + 1] + vel[c + 1]) / 3, (vel[a + 2] + vel[b + 2] + vel[c + 2]) / 3];
  }

  assembleCells(changes, poses, refByPart, initial) {
    const { grid, table } = this;
    const { nx, ny, nz } = grid;
    const recs = [...changes];
    for (const [c, vel] of this.swept) {
      if (this.hasSolid && this.isSolidNow(c)) continue;
      recs.push({ cell: c, kind: CELL_SWEPT, part: vel[3], si: -1, vel });
    }
    const listed = new Set(recs.map((r) => r.cell));
    const need = recs.length * CELL_WORDS * 4;
    if (this.cellBuf.byteLength < need) this.cellBuf = new ArrayBuffer(need * 2);
    const dv = new DataView(this.cellBuf);
    const maskEdits = [];
    let w = 0;
    for (const r of recs) {
      const c = r.cell;
      const ix = c % nx, iy = Math.floor(c / nx) % ny, iz = Math.floor(c / (nx * ny));
      let m = 0;
      if (r.kind !== CELL_COVERED) {
        for (let i = 1; i < Q; i++) {
          const jx = ix + CX[i], jy = iy + CY[i], jz = iz + CZ[i];
          if (jx < 0 || jy < 0 || jz < 0 || jx >= nx || jy >= ny || jz >= nz) continue;
          const nb = (jz * ny + jy) * nx + jx;
          if (listed.has(nb)) continue;
          if (this.hasSolid && this.isSolidNow(nb)) continue;
          if (table.find(c * Q + i) >= 0) continue;
          m |= 1 << i;
        }
      }
      const v = r.vel ?? this.solidVelocityAt(r.si, r.tri, ix, iy, iz);
      const ref = refByPart.get(r.part) ?? [0, 0, 0];
      const o = w * CELL_WORDS * 4;
      dv.setUint32(o, c, true); dv.setUint32(o + 4, m, true); dv.setUint32(o + 8, r.part | (r.kind << 8), true); dv.setUint32(o + 12, 0, true);
      dv.setFloat32(o + 16, v[0], true); dv.setFloat32(o + 20, v[1], true); dv.setFloat32(o + 24, v[2], true);
      dv.setFloat32(o + 28, ix - ref[0], true); dv.setFloat32(o + 32, iy - ref[1], true); dv.setFloat32(o + 36, iz - ref[2], true);
      dv.setFloat32(o + 40, 0, true); dv.setFloat32(o + 44, 0, true);
      if (r.kind !== CELL_SWEPT) maskEdits.push(c);
      w++;
    }
    this.stats.swept = this.swept.size;
    this.stats.cells = w;
    return { count: w, maskEdits };
  }
}

function cross(ax, ay, az, bx, by, bz) {
  return [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx];
}

const bary = [0, 0];
function insideTri(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz) {
  const v0x = bx - ax, v0y = by - ay, v0z = bz - az, v1x = cx - ax, v1y = cy - ay, v1z = cz - az, v2x = px - ax, v2y = py - ay, v2z = pz - az;
  const d00 = v0x * v0x + v0y * v0y + v0z * v0z, d01 = v0x * v1x + v0y * v1y + v0z * v1z, d11 = v1x * v1x + v1y * v1y + v1z * v1z;
  const d20 = v2x * v0x + v2y * v0y + v2z * v0z, d21 = v2x * v1x + v2y * v1y + v2z * v1z;
  const den = d00 * d11 - d01 * d01;
  if (den < 1e-24) return false;
  const beta = (d11 * d20 - d01 * d21) / den, gamma = (d00 * d21 - d01 * d20) / den;
  bary[0] = beta; bary[1] = gamma;
  return beta >= -1e-6 && gamma >= -1e-6 && beta + gamma <= 1 + 1e-6;
}

/** Squared distance from a point to a triangle (Ericson, Real-Time Collision Detection, 5.1.5). */
export function dist2PointTri(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz) {
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  const dist2 = (qx, qy, qz) => (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
  if (d1 <= 0 && d2 <= 0) return dist2(ax, ay, az);
  const bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return dist2(bx, by, bz);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); return dist2(ax + v * abx, ay + v * aby, az + v * abz); }
  const cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return dist2(cx, cy, cz);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); return dist2(ax + w * acx, ay + w * acy, az + w * acz); }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { const w = (d4 - d3) / (d4 - d3 + (d5 - d6)); return dist2(bx + w * (cx - bx), by + w * (cy - by), bz + w * (cz - bz)); }
  const denom = 1 / (va + vb + vc), v = vb * denom, w = vc * denom;
  return dist2(ax + abx * v + acx * w, ay + aby * v + acy * w, az + abz * v + acz * w);
}
