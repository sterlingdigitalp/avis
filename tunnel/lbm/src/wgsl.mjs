// WGSL generators. The main pass is generated per grid with directions unrolled, which folds the
// lattice constants and grid strides at compile time. Floating-point operations appear in the same
// order as src/cpu.mjs so the two agree to round-off.
//
// Population storage: `nb` buffers, directions split evenly between them, because WebGPU here caps
// any one buffer at 4 GiB (see tools/probe-limits.mjs) and four bindings of 5 directions lift the
// ceiling from 56M to 214M cells. Slots are always numbered cell*19+dir in records; the sparse
// passes turn that into (buffer, index).
import { Q, CX, CY, CZ, W, OPP, MIRROR_Y, MIRROR_Z, AXIS, MAX_PARTS, ACC_COMPS, SCALE_F, SCALE_M, LINK_WORDS, CELL_WORDS, CELL_COVERED, CELL_FRESH } from './lattice.mjs';

export function dirGroups(nb) {
  const groups = Array.from({ length: nb }, () => []);
  const per = Math.ceil(Q / nb);
  for (let i = 0; i < Q; i++) groups[Math.floor(i / per)].push(i);
  return groups;
}

const lit = (x) => {
  const v = Math.fround(x);
  let s = v.toPrecision(9);
  if (s.includes('e')) { const [m, e] = s.split('e'); s = `${m}e${e}`; } else if (!s.includes('.')) s += '.0';
  return `${s}f`;
};
const signed = (first, sign, name) => (sign > 0 ? (first ? name : ` + ${name}`) : first ? `-${name}` : ` - ${name}`);
function sumExpr(terms) {
  // terms: [[sign, name], ...] in direction order
  let s = '';
  terms.forEach(([sg, nm], k) => { s += signed(k === 0, sg, nm); });
  return s || '0.0f';
}

const PARAMS = /* wgsl */ `
struct Params {
  tau0: f32, csSmag: f32, Ux: f32, Uy: f32,
  Uz: f32, UU: f32, spStrength: f32, drhoFix: f32,
  linkCount: u32, cellCount: u32, gi: u32, localRho: u32,
  spXL: f32, spXH: f32, spYL: f32, spYH: f32,
  spZL: f32, spZH: f32, pad0: f32, pad1: f32,
};
`;
export const PARAM_BYTES = 80;

function tables(g) {
  const int = (a) => a.join(',');
  const f = (a) => a.map(lit).join(',');
  return /* wgsl */ `
const Q = 19u;
const NX = ${g.nx}i; const NY = ${g.ny}i; const NZ = ${g.nz}i; const NXY = ${g.nx * g.ny}i;
const CX = array<i32,19>(${int(CX)});
const CY = array<i32,19>(${int(CY)});
const CZ = array<i32,19>(${int(CZ)});
const OPPD = array<u32,19>(${int(OPP)});
const MIRY = array<u32,19>(${int(MIRROR_Y)});
const MIRZ = array<u32,19>(${int(MIRROR_Z)});
const W = array<f32,19>(${f(W)});
const THIRD = ${lit(1 / 3)};
`;
}

function bufferDecls(nb, firstBinding, name, access) {
  let s = '';
  for (let k = 0; k < nb; k++) s += `@group(0) @binding(${firstBinding + k}) var<storage, ${access}> ${name}${k}: array<f32>;\n`;
  return s;
}

function indexExpr(cell, g, layout, gs, j) {
  return layout === 'soa' ? `${j}u * ${g.nx * g.ny * g.nz}u + (${cell})` : `(${cell}) * ${gs}u + ${j}u`;
}

/** Generic slot access for sparse paths and edge cells. */
function slotAccess(g, nb, layout, name, withWrite) {
  const groups = dirGroups(nb);
  const dirG = new Array(Q), dirJ = new Array(Q);
  groups.forEach((list, k) => list.forEach((d, j) => { dirG[d] = k; dirJ[d] = j; }));
  const cases = (fn) => groups.map((list, k) => `case ${k}u: { ${fn(k, list.length)} }`).join('\n    ');
  let s = `
const DIRG = array<u32,19>(${dirG.join(',')});
const DIRJ = array<u32,19>(${dirJ.join(',')});
fn readCD(cell: u32, d: u32) -> f32 {
  let j = DIRJ[d];
  switch (DIRG[d]) {
    ${cases((k, gs) => `return ${name}${k}[${layout === 'soa' ? `j * ${g.nx * g.ny * g.nz}u + cell` : `cell * ${gs}u + j`}];`)}
    default: { return 0.0f; }
  }
}
fn readSlot(slot: u32) -> f32 { return readCD(slot / Q, slot % Q); }
`;
  if (withWrite) s += `
fn writeCD(cell: u32, d: u32, v: f32) {
  let j = DIRJ[d];
  switch (DIRG[d]) {
    ${cases((k, gs) => `${name}${k}[${layout === 'soa' ? `j * ${g.nx * g.ny * g.nz}u + cell` : `cell * ${gs}u + j`}] = v;`)}
    default: { }
  }
}
fn writeSlot(slot: u32, v: f32) { writeCD(slot / Q, slot % Q, v); }
`;
  return s;
}

const FEQ_GENERIC = /* wgsl */ `
fn feqS(i: u32, drho: f32, rho: f32, ux: f32, uy: f32, uz: f32, uu: f32) -> f32 {
  var cu = 0.0f;
  if (CX[i] != 0) { if (CX[i] > 0) { cu = ux; } else { cu = -ux; } }
  if (CY[i] != 0) { if (CY[i] > 0) { cu = cu + uy; } else { cu = cu - uy; } }
  if (CZ[i] != 0) { if (CZ[i] > 0) { cu = cu + uz; } else { cu = cu - uz; } }
  let t = ((3.0f * cu) + (4.5f * (cu * cu))) - (1.5f * uu);
  return W[i] * (drho + (rho * t));
}
fn uuOf(ux: f32, uy: f32, uz: f32) -> f32 { return ((ux * ux) + (uy * uy)) + (uz * uz); }
`;

const ACC_FN = /* wgsl */ `
@group(0) @binding(ACC_BINDING) var<storage, read_write> acc: array<atomic<u32>>;
const SCALE_F = ${lit(SCALE_F)};
const SCALE_M = ${lit(SCALE_M)};
// 64-bit two's complement add from two 32-bit atomics. Order independent, so the sum does not depend
// on GPU thread scheduling and can be compared with the CPU exactly.
fn add64(k: u32, v: i32) {
  let u = u32(v);
  let old = atomicAdd(&acc[k * 2u], u);
  var hi = 0u;
  if (v < 0) { hi = 0xffffffffu; }
  if (old + u < old) { hi = hi + 1u; }
  if (hi != 0u) { atomicAdd(&acc[k * 2u + 1u], hi); }
}
fn addFM(part: u32, Fx: f32, Fy: f32, Fz: f32, rx: f32, ry: f32, rz: f32) {
  let o = part * ${ACC_COMPS}u;
  add64(o + 0u, i32(round(Fx * SCALE_F)));
  add64(o + 1u, i32(round(Fy * SCALE_F)));
  add64(o + 2u, i32(round(Fz * SCALE_F)));
  add64(o + 3u, i32(round(((ry * Fz) - (rz * Fy)) * SCALE_M)));
  add64(o + 4u, i32(round(((rz * Fx) - (rx * Fz)) * SCALE_M)));
  add64(o + 5u, i32(round(((rx * Fy) - (ry * Fx)) * SCALE_M)));
}
fn addMass(part: u32, v: f32) { add64(part * ${ACC_COMPS}u + 6u, i32(round(v * SCALE_F))); }
`;

// ---- main pass ------------------------------------------------------------------------------
export function wgslMain(g, opt) {
  const { nb, layout, wg, useMask, hasSponge } = opt;
  const groups = dirGroups(nb);
  const dirG = new Array(Q), dirJ = new Array(Q);
  groups.forEach((list, k) => list.forEach((d, j) => { dirG[d] = k; dirJ[d] = j; }));
  const gs = (d) => groups[dirG[d]].length;
  const off = (i) => CX[i] + CY[i] * g.nx + CZ[i] * g.nx * g.ny;
  const nbExpr = (i) => { const o = off(i); return o === 0 ? 'c' : o > 0 ? `c - ${o}u` : `c + ${-o}u`; };

  let reads = '';
  for (let i = 0; i < Q; i++) reads += `    f${i} = src${dirG[i]}[${indexExpr(nbExpr(i), g, layout, gs(i), dirJ[i])}];\n`;
  let edgeReads = '';
  for (let i = 0; i < Q; i++) edgeReads += `    f${i} = pullEdge(ix, iy, iz, c, ${i}u, drC, rC, uxC, uyC, uzC, uuC);\n`;
  let vars = '';
  for (let i = 0; i < Q; i++) vars += `  var f${i}: f32;\n`;

  const name = (i) => `f${i}`;
  const all = [...Array(Q).keys()];
  const axisTerms = (C) => all.filter((i) => C[i] !== 0).map((i) => [C[i], name(i)]);
  const sqTerms = (C) => all.filter((i) => C[i] !== 0).map((i) => [1, name(i)]);
  const crossTerms = (A, B) => all.filter((i) => A[i] !== 0 && B[i] !== 0).map((i) => [A[i] * B[i], name(i)]);

  let writes = '';
  const pijCode = (i) => {
    const cx = CX[i], cy = CY[i], cz = CZ[i];
    let q = null;
    if (cx) q = 'Pxx';
    if (cy) q = q ? `(${q} + Pyy)` : 'Pyy';
    if (cz) q = q ? `(${q} + Pzz)` : 'Pzz';
    if (cx && cy) q = `(${q} ${cx * cy > 0 ? '+' : '-'} (2.0f * Pxy))`;
    if (cx && cz) q = `(${q} ${cx * cz > 0 ? '+' : '-'} (2.0f * Pxz))`;
    if (cy && cz) q = `(${q} ${cy * cz > 0 ? '+' : '-'} (2.0f * Pyz))`;
    return q ? `(${q} - t3)` : '(0.0f - t3)';
  };
  const cuCode = (i, ux, uy, uz) => {
    let s = null;
    const add = (c, v) => { if (!c) return; s = s === null ? (c > 0 ? v : `(-${v})`) : `(${s} ${c > 0 ? '+' : '-'} ${v})`; };
    add(CX[i], ux); add(CY[i], uy); add(CZ[i], uz);
    return s ?? '0.0f';
  };
  for (let i = 0; i < Q; i++) {
    const cu = cuCode(i, 'ux', 'uy', 'uz');
    const wI = lit(W[i]), wR = lit(Math.fround(W[i]) * 4.5);
    writes += `  {
    let cu = ${cu};
    var v = (${wI} * (drho + (rho * (((3.0f * cu) + (4.5f * (cu * cu))) - uu15)))) + (k * (${wR} * ${pijCode(i)}));
${hasSponge ? `    if (sp > 0.0f) {
      let tu = ${cuCode(i, 'p.Ux', 'p.Uy', 'p.Uz')};
      v = (v * (1.0f - sp)) + (sp * (${wI} * (0.0f + (1.0f * (((3.0f * tu) + (4.5f * (tu * tu))) - (1.5f * p.UU))))));
    }
` : ''}    dst${dirG[i]}[${indexExpr('c', g, layout, gs(i), dirJ[i])}] = v;
  }
`;
  }

  return /* wgsl */ `
${PARAMS}
${tables(g)}
${bufferDecls(nb, 0, 'src', 'read')}
${bufferDecls(nb, nb, 'dst', 'read_write')}
@group(0) @binding(${2 * nb}) var<uniform> p: Params;
${useMask ? `@group(0) @binding(${2 * nb + 1}) var<storage, read> maskW: array<u32>;` : ''}
${slotAccess(g, nb, layout, 'src', false)}
${FEQ_GENERIC}
const X_MODE = ${g.xMode}u; const Y_LOW = ${g.yLowMode}u; const Y_HIGH = ${g.yHighMode}u; const Z_LOW = ${g.zLowMode}u; const Z_HIGH = ${g.zHighMode}u;

fn pullEdge(ix: i32, iy: i32, iz: i32, c: u32, i: u32, drC: f32, rC: f32, uxC: f32, uyC: f32, uzC: f32, uuC: f32) -> f32 {
  var sx = ix - CX[i]; var sy = iy - CY[i]; var sz = iz - CZ[i];
  var d = i;
  if (sx < 0 || sx >= NX) {
    if (X_MODE == ${AXIS.periodic}u) { sx = (sx + NX) % NX; }
    else if (X_MODE == ${AXIS.inflow}u) {
      if (sx >= NX) { return feqS(i, drC, rC, p.Ux, p.Uy, p.Uz, uuOf(p.Ux, p.Uy, p.Uz)); }
      return feqS(i, drC, rC, uxC, uyC, uzC, uuC);
    } else { return readCD(c, OPPD[i]); }
  }
  if (sy < 0 || sy >= NY) {
    var m = Y_HIGH; if (sy < 0) { m = Y_LOW; }
    if (m == ${AXIS.periodic}u) { sy = (sy + NY) % NY; }
    else if (m == ${AXIS.freeslip}u) { if (sy < 0) { sy = -1 - sy; } else { sy = 2 * NY - 1 - sy; } d = MIRY[d]; }
    else { return readCD(c, OPPD[i]); }
  }
  if (sz < 0 || sz >= NZ) {
    var m = Z_HIGH; if (sz < 0) { m = Z_LOW; }
    if (m == ${AXIS.periodic}u) { sz = (sz + NZ) % NZ; }
    else if (m == ${AXIS.freeslip}u) { if (sz < 0) { sz = -1 - sz; } else { sz = 2 * NZ - 1 - sz; } d = MIRZ[d]; }
    else { return readCD(c, OPPD[i]); }
  }
  return readCD(u32((sz * NY + sy) * NX + sx), d);
}

@compute @workgroup_size(${wg})
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let ix = i32(gid.x); let iy = i32(gid.y); let iz = i32(gid.z);
  if (ix >= NX) { return; }
  let c = u32((iz * NY + iy) * NX + ix);
  ${useMask ? `if (((maskW[c >> 2u] >> ((c & 3u) * 8u)) & 255u) != 255u) { return; }` : ''}
${vars}
  if (ix == 0 || ix == NX - 1 || iy == 0 || iy == NY - 1 || iz == 0 || iz == NZ - 1) {
    var drC = 0.0f; var jxC = 0.0f; var jyC = 0.0f; var jzC = 0.0f;
    for (var i = 0u; i < Q; i = i + 1u) {
      let v = readCD(c, i);
      drC = drC + v;
      if (CX[i] > 0) { jxC = jxC + v; } else if (CX[i] < 0) { jxC = jxC - v; }
      if (CY[i] > 0) { jyC = jyC + v; } else if (CY[i] < 0) { jyC = jyC - v; }
      if (CZ[i] > 0) { jzC = jzC + v; } else if (CZ[i] < 0) { jzC = jzC - v; }
    }
    let rC = 1.0f + drC; let invC = 1.0f / rC;
    let uxC = jxC * invC; let uyC = jyC * invC; let uzC = jzC * invC;
    let uuC = uuOf(uxC, uyC, uzC);
${edgeReads}
  } else {
${reads}
  }
  let drho = ${sumExpr(all.map((i) => [1, name(i)]))} + p.drhoFix;
  let jx = ${sumExpr(axisTerms(CX))};
  let jy = ${sumExpr(axisTerms(CY))};
  let jz = ${sumExpr(axisTerms(CZ))};
  let sxx = ${sumExpr(sqTerms(CX))};
  let syy = ${sumExpr(sqTerms(CY))};
  let szz = ${sumExpr(sqTerms(CZ))};
  let sxy = ${sumExpr(crossTerms(CX, CY))};
  let sxz = ${sumExpr(crossTerms(CX, CZ))};
  let syz = ${sumExpr(crossTerms(CY, CZ))};
  let rho = 1.0f + drho; let inv = 1.0f / rho;
  let ux = jx * inv; let uy = jy * inv; let uz = jz * inv;
  let third = drho * THIRD;
  let Pxx = (sxx - third) - ((rho * ux) * ux);
  let Pyy = (syy - third) - ((rho * uy) * uy);
  let Pzz = (szz - third) - ((rho * uz) * uz);
  let Pxy = sxy - ((rho * ux) * uy);
  let Pxz = sxz - ((rho * ux) * uz);
  let Pyz = syz - ((rho * uy) * uz);
  var tau = p.tau0;
  if (p.csSmag > 0.0f) {
    let n2 = (((Pxx * Pxx) + (Pyy * Pyy)) + (Pzz * Pzz)) + (2.0f * (((Pxy * Pxy) + (Pxz * Pxz)) + (Pyz * Pyz)));
    let piNorm = sqrt(n2);
    let A = ((${lit(3 / Math.SQRT2)} * (p.csSmag * p.csSmag)) * piNorm) * inv;
    let nuT = (sqrt((p.tau0 * p.tau0) + (12.0f * A)) - p.tau0) * ${lit(1 / 6)};
    tau = p.tau0 + (3.0f * max(0.0f, nuT));
  }
  let k = 1.0f - (1.0f / tau);
  let t3 = ((Pxx + Pyy) + Pzz) * THIRD;
  let uu = ((ux * ux) + (uy * uy)) + (uz * uz);
  let uu15 = 1.5f * uu;
${hasSponge ? `  var spT = 0.0f;
  if (f32(ix) < p.spXL) { spT = max(spT, 1.0f - (f32(ix) / p.spXL)); }
  if (f32(NX - 1 - ix) < p.spXH) { spT = max(spT, 1.0f - (f32(NX - 1 - ix) / p.spXH)); }
  if (f32(iy) < p.spYL) { spT = max(spT, 1.0f - (f32(iy) / p.spYL)); }
  if (f32(NY - 1 - iy) < p.spYH) { spT = max(spT, 1.0f - (f32(NY - 1 - iy) / p.spYH)); }
  if (f32(iz) < p.spZL) { spT = max(spT, 1.0f - (f32(iz) / p.spZL)); }
  if (f32(NZ - 1 - iz) < p.spZH) { spT = max(spT, 1.0f - (f32(NZ - 1 - iz) / p.spZH)); }
  var sp = 0.0f;
  if (spT > 0.0f) { sp = (p.spStrength * (spT * spT)) * (3.0f - (2.0f * spT)); }
` : ''}
${writes}
}
`;
}

// ---- sparse passes -------------------------------------------------------------------------
const accDecl = (binding) => ACC_FN.replace('ACC_BINDING', String(binding));

export function wgslLink1(g, opt) {
  const { nb, layout } = opt;
  return /* wgsl */ `
${PARAMS}
${tables(g)}
${bufferDecls(nb, 0, 'src', 'read')}
@group(0) @binding(${nb}) var<storage, read> L: array<u32>;
@group(0) @binding(${nb + 1}) var<storage, read_write> val: array<f32>;
@group(0) @binding(${nb + 2}) var<uniform> p: Params;
${accDecl(nb + 3)}
${slotAccess(g, nb, layout, 'src', false)}
@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let l = gid.x;
  if (l >= p.linkCount) { return; }
  let o = l * ${LINK_WORDS}u;
  let srcA = L[o + 1u]; let srcB = L[o + 2u]; let pd = L[o + 3u];
  let part = pd & 255u; let d = pd >> 8u;
  let a = bitcast<f32>(L[o + 4u]); let b = bitcast<f32>(L[o + 5u]); let mw = bitcast<f32>(L[o + 6u]);
  let fA = readSlot(srcA); let fB = readSlot(srcB);
  var wall = mw;
  if (p.localRho != 0u) {
    let cell = srcA / Q;
    var dr = 0.0f;
    for (var i = 0u; i < Q; i = i + 1u) { dr = dr + readCD(cell, i); }
    wall = (1.0f + dr) * mw;
  }
  let out = ((a * fA) + (b * fB)) + wall;
  val[l] = out;
  let sum = (fA + out) + (2.0f * W[d]);
  var Fx = 0.0f; var Fy = 0.0f; var Fz = 0.0f;
  if (CX[d] > 0) { Fx = sum; } else if (CX[d] < 0) { Fx = -sum; }
  if (CY[d] > 0) { Fy = sum; } else if (CY[d] < 0) { Fy = -sum; }
  if (CZ[d] > 0) { Fz = sum; } else if (CZ[d] < 0) { Fz = -sum; }
  let dm = out - fA;
  if (p.gi != 0u) {
    Fx = Fx + (bitcast<f32>(L[o + 7u]) * dm);
    Fy = Fy + (bitcast<f32>(L[o + 8u]) * dm);
    Fz = Fz + (bitcast<f32>(L[o + 9u]) * dm);
  }
  addFM(part, Fx, Fy, Fz, bitcast<f32>(L[o + 10u]), bitcast<f32>(L[o + 11u]), bitcast<f32>(L[o + 12u]));
  addMass(part, dm);
}
`;
}

export function wgslLink2(g, opt) {
  const { nb, layout } = opt;
  return /* wgsl */ `
${PARAMS}
${tables(g)}
${bufferDecls(nb, 0, 'src', 'read_write')}
@group(0) @binding(${nb}) var<storage, read> L: array<u32>;
@group(0) @binding(${nb + 1}) var<storage, read> val: array<f32>;
@group(0) @binding(${nb + 2}) var<uniform> p: Params;
${slotAccess(g, nb, layout, 'src', true)}
@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let l = gid.x;
  if (l >= p.linkCount) { return; }
  writeSlot(L[l * ${LINK_WORDS}u], val[l]);
}
`;
}

export function wgslCells(g, opt) {
  const { nb, layout } = opt;
  return /* wgsl */ `
${PARAMS}
${tables(g)}
${bufferDecls(nb, 0, 'src', 'read_write')}
@group(0) @binding(${nb}) var<storage, read> R: array<u32>;
@group(0) @binding(${nb + 1}) var<uniform> p: Params;
${accDecl(nb + 2)}
${slotAccess(g, nb, layout, 'src', true)}
${FEQ_GENERIC}
fn regNeq(i: u32, Pxx: f32, Pyy: f32, Pzz: f32, Pxy: f32, Pxz: f32, Pyz: f32, t3: f32) -> f32 {
  let cx = CX[i]; let cy = CY[i]; let cz = CZ[i];
  var q = 0.0f;
  if (cx != 0) { q = Pxx; }
  if (cy != 0) { if (cx != 0) { q = q + Pyy; } else { q = Pyy; } }
  if (cz != 0) { if (cx != 0 || cy != 0) { q = q + Pzz; } else { q = Pzz; } }
  if (cx * cy > 0) { q = q + (2.0f * Pxy); } else if (cx * cy < 0) { q = q - (2.0f * Pxy); }
  if (cx * cz > 0) { q = q + (2.0f * Pxz); } else if (cx * cz < 0) { q = q - (2.0f * Pxz); }
  if (cy * cz > 0) { q = q + (2.0f * Pyz); } else if (cy * cz < 0) { q = q - (2.0f * Pyz); }
  q = q - t3;
  return (W[i] * 4.5f) * q;
}
@compute @workgroup_size(16)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let r = gid.x;
  if (r >= p.cellCount) { return; }
  let o = r * ${CELL_WORDS}u;
  let c = R[o]; let m = R[o + 1u]; let part = R[o + 2u] & 255u; let kind = R[o + 2u] >> 8u;
  let ux = bitcast<f32>(R[o + 4u]); let uy = bitcast<f32>(R[o + 5u]); let uz = bitcast<f32>(R[o + 6u]);
  let rx = bitcast<f32>(R[o + 7u]); let ry = bitcast<f32>(R[o + 8u]); let rz = bitcast<f32>(R[o + 9u]);
  var oldDrho = 0.0f; var oldPx = 0.0f; var oldPy = 0.0f; var oldPz = 0.0f;
  if (kind != ${CELL_FRESH}u) {
    for (var i = 0u; i < Q; i = i + 1u) {
      let v = readCD(c, i);
      oldDrho = oldDrho + v;
      if (CX[i] > 0) { oldPx = oldPx + v; } else if (CX[i] < 0) { oldPx = oldPx - v; }
      if (CY[i] > 0) { oldPy = oldPy + v; } else if (CY[i] < 0) { oldPy = oldPy - v; }
      if (CZ[i] > 0) { oldPz = oldPz + v; } else if (CZ[i] < 0) { oldPz = oldPz - v; }
    }
  }
  if (kind == ${CELL_COVERED}u) {
    addFM(part, oldPx, oldPy, oldPz, rx, ry, rz);
    addMass(part, -oldDrho);
    return;
  }
  var count = 0u; var sumRho = 0.0f;
  var Sxx = 0.0f; var Syy = 0.0f; var Szz = 0.0f; var Sxy = 0.0f; var Sxz = 0.0f; var Syz = 0.0f;
  for (var j = 1u; j < Q; j = j + 1u) {
    if (((m >> j) & 1u) == 0u) { continue; }
    let nb = u32(i32(c) + CX[j] + CY[j] * NX + CZ[j] * NXY);
    var dr = 0.0f; var jx = 0.0f; var jy = 0.0f; var jz = 0.0f;
    var pxx = 0.0f; var pyy = 0.0f; var pzz = 0.0f; var pxy = 0.0f; var pxz = 0.0f; var pyz = 0.0f;
    for (var i = 0u; i < Q; i = i + 1u) {
      let v = readCD(nb, i);
      dr = dr + v;
      if (CX[i] != 0) { pxx = pxx + v; if (CX[i] > 0) { jx = jx + v; } else { jx = jx - v; } }
      if (CY[i] != 0) { pyy = pyy + v; if (CY[i] > 0) { jy = jy + v; } else { jy = jy - v; } }
      if (CZ[i] != 0) { pzz = pzz + v; if (CZ[i] > 0) { jz = jz + v; } else { jz = jz - v; } }
      if (CX[i] * CY[i] > 0) { pxy = pxy + v; } else if (CX[i] * CY[i] < 0) { pxy = pxy - v; }
      if (CX[i] * CZ[i] > 0) { pxz = pxz + v; } else if (CX[i] * CZ[i] < 0) { pxz = pxz - v; }
      if (CY[i] * CZ[i] > 0) { pyz = pyz + v; } else if (CY[i] * CZ[i] < 0) { pyz = pyz - v; }
    }
    let rho = 1.0f + dr; let inv = 1.0f / rho;
    let vx = jx * inv; let vy = jy * inv; let vz = jz * inv;
    let third = dr * THIRD;
    sumRho = sumRho + dr;
    Sxx = Sxx + ((pxx - third) - ((rho * vx) * vx));
    Syy = Syy + ((pyy - third) - ((rho * vy) * vy));
    Szz = Szz + ((pzz - third) - ((rho * vz) * vz));
    Sxy = Sxy + (pxy - ((rho * vx) * vy));
    Sxz = Sxz + (pxz - ((rho * vx) * vz));
    Syz = Syz + (pyz - ((rho * vy) * vz));
    count = count + 1u;
  }
  var dr = oldDrho;
  var Pxx = 0.0f; var Pyy = 0.0f; var Pzz = 0.0f; var Pxy = 0.0f; var Pxz = 0.0f; var Pyz = 0.0f;
  if (count > 0u) {
    let k = 1.0f / f32(count);
    dr = sumRho * k;
    Pxx = Sxx * k; Pyy = Syy * k; Pzz = Szz * k; Pxy = Sxy * k; Pxz = Sxz * k; Pyz = Syz * k;
  }
  let rho = 1.0f + dr; let uu = uuOf(ux, uy, uz);
  let t3 = ((Pxx + Pyy) + Pzz) * THIRD;
  var newDrho = 0.0f; var newPx = 0.0f; var newPy = 0.0f; var newPz = 0.0f;
  for (var i = 0u; i < Q; i = i + 1u) {
    let v = feqS(i, dr, rho, ux, uy, uz, uu) + regNeq(i, Pxx, Pyy, Pzz, Pxy, Pxz, Pyz, t3);
    writeCD(c, i, v);
    newDrho = newDrho + v;
    if (CX[i] > 0) { newPx = newPx + v; } else if (CX[i] < 0) { newPx = newPx - v; }
    if (CY[i] > 0) { newPy = newPy + v; } else if (CY[i] < 0) { newPy = newPy - v; }
    if (CZ[i] > 0) { newPz = newPz + v; } else if (CZ[i] < 0) { newPz = newPz - v; }
  }
  addFM(part, oldPx - newPx, oldPy - newPy, oldPz - newPz, rx, ry, rz);
  addMass(part, newDrho - oldDrho);
}
`;
}

/** Uniform initial state: post-collision equilibrium at drho = 0, velocity u0. */
export function wgslFill(g, opt) {
  const { nb, layout, wg } = opt;
  let writes = '';
  const groups = dirGroups(nb);
  groups.forEach((list, k) => list.forEach((d, j) => {
    writes += `  src${k}[${indexExpr('c', g, layout, list.length, j)}] = feqS(${d}u, 0.0f, 1.0f, p.Ux, p.Uy, p.Uz, uuOf(p.Ux, p.Uy, p.Uz));\n`;
  }));
  return /* wgsl */ `
${PARAMS}
${tables(g)}
${bufferDecls(nb, 0, 'src', 'read_write')}
@group(0) @binding(${nb}) var<uniform> p: Params;
${FEQ_GENERIC}
@compute @workgroup_size(${wg})
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let ix = i32(gid.x); let iy = i32(gid.y); let iz = i32(gid.z);
  if (ix >= NX) { return; }
  let c = u32((iz * NY + iy) * NX + ix);
${writes}
}
`;
}
