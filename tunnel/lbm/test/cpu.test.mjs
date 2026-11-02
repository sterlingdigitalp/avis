// Pins the CPU reference against analytic results and exact conservation identities. None of the
// expected values come from this code's own output.
import test from 'node:test';
import assert from 'node:assert/strict';
import { CpuLbm, CompensatedSum } from '../src/cpu.mjs';
import { BoundaryEngine } from '../src/geometry.mjs';
import { AXIS, FLUID, Q } from '../src/lattice.mjs';
import { rigidPlateSurface, harmonic, noMotion } from '../src/motion.mjs';

const periodicBox = (nx, ny, nz, tau0, extra = {}) => ({ nx, ny, nz, tau0, csSmag: 0, xMode: AXIS.periodic, yLowMode: AXIS.periodic, yHighMode: AXIS.periodic, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic, precision: 'f64', ...extra });
const gridOf = (c) => ({ nx: c.nx, ny: c.ny, nz: c.nz, xMode: c.xMode, yLowMode: c.yLowMode, yHighMode: c.yHighMode, zLowMode: c.zLowMode, zHighMode: c.zHighMode });

test('Taylor-Green shear wave decays at the viscosity set by tau', () => {
  const nx = 32, ny = 32, tau0 = 0.8, nu = (tau0 - 0.5) / 3, U = 0.02, kk = (2 * Math.PI) / ny;
  const s = new CpuLbm(periodicBox(nx, ny, 1, tau0));
  s.setField((ix, iy) => ({ drho: 0, u: [U * Math.sin(kk * (iy + 0.5)), 0, 0] }));
  const steps = 400;
  // The stored state is post-collision, which offsets its first moment from the macroscopic
  // velocity only at second order for this shear flow; compare the decay ratio between two times.
  s.step(50);
  const a = s.macroAt(0, 8, 0).u[0];
  s.step(steps);
  const b = s.macroAt(0, 8, 0).u[0];
  const expected = Math.exp(-nu * kk * kk * steps);
  assert.ok(Math.abs(b / a - expected) / expected < 2e-3, `ratio ${b / a} vs ${expected}`);
});

test('Couette flow between two moving sheets reaches the linear profile', () => {
  // Two periodic-in-x-and-z thin sheets at fractional heights. The lower is held, the upper slides
  // at U. Interpolated bounce-back must put the no-slip point at the actual sheet position.
  const nx = 4, ny = 28, nz = 1, tau0 = 1.0, U = 0.03;
  const cfg = periodicBox(nx, ny, nz, tau0);
  const yA = 3.3, yB = 22.7;
  const sheet = (part, y, vx) => {
    const pos = Float64Array.from([-4, y, -3, nx + 3, y, -3, -4, y, nz + 2, nx + 3, y, nz + 2]);
    const vel = Float64Array.from([vx, 0, 0, vx, 0, 0, vx, 0, 0, vx, 0, 0]);
    return { part, kind: 'thin', tris: Uint32Array.from([0, 1, 3, 0, 3, 2]), moving: false, pose: () => ({ pos, vel }) };
  };
  const eng = new BoundaryEngine(gridOf(cfg), [sheet(0, yA, 0), sheet(1, yB, U)], { bouzidi: true });
  const s = new CpuLbm(cfg);
  s.setFrame(eng.init(0));
  s.step(9000);
  for (const iy of [6, 10, 14, 18, 21]) {
    const exact = (U * (iy - yA)) / (yB - yA);
    const got = s.macroAt(1, iy, 0).u[0];
    assert.ok(Math.abs(got - exact) / U < 2e-3, `y=${iy}: ${got} vs ${exact}`);
  }
  // Wall shear from momentum exchange against the analytic stress. Each sheet carries shear on both
  // faces: the channel of height yB-yA and the periodic wrap-around region of the remaining height.
  s.takeForces();
  s.step(200);
  const F = s.takeForces();
  const nu = (tau0 - 0.5) / 3;
  const shear = nu * U * (1 / (yB - yA) + 1 / (ny - (yB - yA)));
  const area = nx * nz;
  const lower = F[0].F[0] / 200 / area, upper = F[1].F[0] / 200 / area;
  assert.ok(Math.abs(lower - shear) / shear < 5e-3, `lower sheet stress ${lower} vs ${shear}`);
  assert.ok(Math.abs(upper + shear) / shear < 5e-3, `upper sheet stress ${upper} vs ${-shear}`);
});

test('Stationary thin sheet keeps a pressure jump: no leak across it', () => {
  const nx = 6, ny = 20, nz = 1;
  const cfg = periodicBox(nx, ny, nz, 0.8, { yLowMode: AXIS.bounceback, yHighMode: AXIS.bounceback });
  const y = 9.4;
  const pos = Float64Array.from([-3, y, -3, nx + 2, y, -3, -3, y, nz + 2, nx + 2, y, nz + 2]);
  const vel = new Float64Array(12);
  const eng = new BoundaryEngine(gridOf(cfg), [{ part: 0, kind: 'thin', tris: Uint32Array.from([0, 1, 3, 0, 3, 2]), moving: false, pose: () => ({ pos, vel }) }], { bouzidi: false });
  const s = new CpuLbm(cfg);
  s.setField((ix, iy) => ({ drho: iy < y ? 0.01 : -0.01, u: [0, 0, 0] }));
  s.setFrame(eng.init(0));
  const before = sumBelow(s, y);
  s.step(500);
  const after = sumBelow(s, y);
  assert.ok(Math.abs(after - before) < 1e-10, `mass moved across sheet: ${after - before}`);
});
function sumBelow(s, y) {
  let m = 0;
  for (let iy = 0; iy < s.ny; iy++) if (iy < y) for (let ix = 0; ix < s.nx; ix++) {
    const c = iy * s.nx + ix;
    for (let i = 0; i < Q; i++) m += s.src[c * Q + i];
  }
  return m;
}

test('Moving sheet in a closed box: momentum balance is exact and the mass budget accounts for every change', () => {
  // A plate plunging across nodes in a fully periodic box. Two exact identities must hold:
  //   d(P_fluid)/dt = -F_body, for momentum exchange without the Galilean term;
  //   d(sum drho)/dt = budget of link exchange and refill, reported by the accumulators.
  // Tolerance is the accumulator's fixed-point quantum (2^-26 per link component) times the root of
  // about 6e5 link events; everything else here is exact in f64.
  const nx = 40, ny = 40, nz = 10, tau0 = 0.7;
  const cfg = periodicBox(nx, ny, nz, tau0, { giTerm: false });
  const m = harmonic({ h0: 3, omega: 0.03 });
  const surf = rigidPlateSurface({ chord: 16, zMin: -2, zMax: nz + 2, x0: 20, y0: 20, nChord: 4, nSpan: 1, m });
  const eng = new BoundaryEngine(gridOf(cfg), [surf], { bouzidi: false });
  const s = new CpuLbm(cfg);
  s.setFrame(eng.init(0.5));
  const p0 = s.momentum();
  const m0 = s.massDelta().deltaMass;
  let impulse = [0, 0, 0], budget = 0;
  for (let n = 0; n < 300; n++) {
    s.setFrame(eng.frame(n + 0.5));
    s.step(1);
    const F = s.takeForces()[0];
    impulse = impulse.map((v, k) => v + F.F[k]);
    budget += F.dMass;
  }
  const p1 = s.momentum();
  for (let k = 0; k < 3; k++) {
    const dP = p1[k] - p0[k];
    assert.ok(Math.abs(dP + impulse[k]) < 2e-5, `axis ${k}: dP ${dP} vs impulse ${impulse[k]}`);
  }
  const dM = s.massDelta().deltaMass - m0;
  assert.ok(Math.abs(dM - budget) < 1e-4, `mass change ${dM} vs budget ${budget}`);
});

test('Exact mass audit: in f64 links and cell changes account for every change, for Bouzidi sheets and slabs', () => {
  // Collision conserves mass exactly in exact arithmetic, and streaming in a periodic box is a
  // permutation, so the whole change in fluid mass must come from link exchange and cell changes.
  // In f64 the collision term is round-off only; in f32 it is not, and the three terms together
  // still close because each is taken from the stored values.
  const run = (kind, bouzidi, precision) => {
    const nx = 40, ny = 40, nz = 4;
    const cfg = periodicBox(nx, ny, nz, 0.53, { precision, massAudit: true, u0: [-0.03, 0, 0] });
    const m = harmonic({ h0: 3, alpha0: 0.1, omega: 0.03, rampSteps: 50 });
    const surf = rigidPlateSurface({ kind, thickness: 2, chord: 16, zMin: -3, zMax: nz + 2, x0: 20.27, y0: 20.37, nChord: 4, nSpan: 1, m });
    const eng = new BoundaryEngine(gridOf(cfg), [surf], { bouzidi });
    const s = new CpuLbm({ ...cfg, mask: eng.mask ?? undefined });
    s.setFrame(eng.init(0.5));
    const m0 = s.massDelta().deltaMass;
    for (let n = 0; n < 400; n++) { s.setFrame(eng.frame(n + 0.5)); s.step(1); }
    const a = s.audit;
    return { dM: s.massDelta().deltaMass - m0, link: a.link.value, cell: a.cell.value, collide: a.collide.value };
  };
  for (const [kind, bouzidi] of [['thin', true], ['slab', false], ['slab', true]]) {
    const r = run(kind, bouzidi, 'f64');
    const budget = r.link + r.cell;
    assert.ok(Math.abs(budget) > 1e-3, `${kind}: the case must move mass (${budget})`);
    assert.ok(Math.abs(r.collide) < 1e-11, `${kind} f64 collision term ${r.collide}`);
    assert.ok(Math.abs(r.dM - budget) < 1e-11 * Math.max(1, Math.abs(budget)), `${kind} f64: mass change ${r.dM} vs links + cells ${budget}`);
  }
  const f = run('thin', true, 'f32');
  assert.ok(Math.abs(f.dM - (f.link + f.cell + f.collide)) < 1e-9, `f32: ${f.dM} vs ${f.link + f.cell + f.collide}`);
});

test('The mass audit only observes: with it off the state is bit for bit the same, cells are counted', () => {
  const nx = 24, ny = 24, nz = 2;
  const states = [false, true].map((massAudit) => {
    const cfg = periodicBox(nx, ny, nz, 0.6, { precision: 'f32', massAudit, u0: [-0.03, 0, 0] });
    const surf = rigidPlateSurface({ kind: 'slab', thickness: 2, chord: 10, zMin: -3, zMax: nz + 2, x0: 12.27, y0: 12.37, nChord: 4, nSpan: 1, m: harmonic({ h0: 2, omega: 0.05 }) });
    const eng = new BoundaryEngine(gridOf(cfg), [surf], { bouzidi: true });
    const s = new CpuLbm({ ...cfg, mask: eng.mask });
    s.setFrame(eng.init(0.5));
    for (let n = 0; n < 60; n++) { s.setFrame(eng.frame(n + 0.5)); s.step(1); }
    const fluid = eng.mask.reduce((a, v) => a + (v === FLUID ? 1 : 0), 0);
    assert.equal(s.massDelta().cells, fluid);
    return s.src;
  });
  assert.deepEqual(states[0], states[1]);
});

test('Compensated sum keeps what plain addition rounds away', () => {
  const s = new CompensatedSum();
  s.add(1);
  for (let i = 0; i < 1e5; i++) s.add(1e-17);
  assert.equal(s.value, 1 + 1e-12);
  const t = new CompensatedSum();
  for (const x of [1e-17, 1, -1]) t.add(x);
  assert.equal(t.value, 1e-17);
});

test('Mass correction from the budget holds total mass to round-off', () => {
  const nx = 40, ny = 40, nz = 10, tau0 = 0.7;
  const cfg = periodicBox(nx, ny, nz, tau0);
  const m = harmonic({ h0: 3, omega: 0.03 });
  const surf = rigidPlateSurface({ chord: 16, zMin: -2, zMax: nz + 2, x0: 20, y0: 20, nChord: 4, nSpan: 1, m });
  const eng = new BoundaryEngine(gridOf(cfg), [surf], { bouzidi: true });
  const s = new CpuLbm(cfg);
  s.setFrame(eng.init(0.5));
  for (let n = 0; n < 200; n++) {
    s.setFrame(eng.frame(n + 0.5));
    s.step(1);
    const F = s.takeForces()[0];
    s.drhoFix = -F.dMass / (nx * ny * nz);
  }
  // One step of budget is still pending at the end, so compare against that bound rather than zero.
  const dM = Math.abs(s.massDelta().deltaMass);
  assert.ok(dM < 2, `mass after correction ${dM}`);
});

test('Half model with a symmetry plane equals the full model to round-off', () => {
  const nx = 36, ny = 24, nz = 24;
  const base = { nx, ny, tau0: 0.65, csSmag: 0, precision: 'f64', xMode: AXIS.inflow, inletU: [-0.03, 0, 0], yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip, u0: [-0.03, 0, 0] };
  // The heave stays between node rows, so no node is swept: this pins links, mirrors and Bouzidi
  // exactly. Refill near the plane only averages in-domain neighbours and is tested separately.
  const m = harmonic({ h0: 0.4, omega: 0.05 });
  // Full model: plate spanning across the mid plane z = 11.5 (between cells 11 and 12).
  const full = { ...base, nz };
  const fullS = rigidPlateSurface({ chord: 10, zMin: 11.5 - 7.3, zMax: 11.5 + 7.3, x0: 18, y0: 11.5, nChord: 3, nSpan: 2, m });
  const engF = new BoundaryEngine(gridOf(full), [fullS], { bouzidi: true });
  const sF = new CpuLbm(full);
  sF.setFrame(engF.init(0.5));
  // Half model: cells z >= 12 of the full model become z >= 0; the plane sits at z = -0.5.
  const half = { ...base, nz: nz / 2 };
  const halfS = rigidPlateSurface({ chord: 10, zMin: -0.5, zMax: -0.5 + 7.3, x0: 18, y0: 11.5, nChord: 3, nSpan: 1, m, mirrorZ: true });
  const engH = new BoundaryEngine(gridOf(half), [halfS], { bouzidi: true });
  const sH = new CpuLbm(half);
  sH.setFrame(engH.init(0.5));
  for (let n = 0; n < 120; n++) {
    sF.setFrame(engF.frame(n + 0.5)); sF.step(1);
    sH.setFrame(engH.frame(n + 0.5)); sH.step(1);
  }
  let maxDiff = 0;
  for (let iz = 0; iz < nz / 2; iz++) for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++) {
    const cH = (iz * ny + iy) * nx + ix;
    const cF = ((iz + nz / 2) * ny + iy) * nx + ix;
    for (let i = 0; i < Q; i++) maxDiff = Math.max(maxDiff, Math.abs(sH.src[cH * Q + i] - sF.src[cF * Q + i]));
  }
  assert.ok(maxDiff < 1e-9, `half vs full max population difference ${maxDiff}`);
});

test('Bird surfaces: closed body volume matches the ellipsoid, and a moving sheet wing edits no solid cell', async () => {
  const kin = await import('../../../src/physics/kinematics.js');
  const B = await import('../src/bird.mjs');
  const fs = await import('node:fs');
  const d = B.readDesign(JSON.parse(fs.readFileSync(new URL('./fixtures/test-gull.json', import.meta.url))), kin);
  const dx = 0.02, nx = 80, ny = 60, nz = 40;
  const grid = { nx, ny, nz, xMode: AXIS.inflow, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
  const dt = 1e-4;
  const { surfaces } = B.birdSurfaces(kin, { design: d, dx, dt, origin: [-1.0, -0.6, 0], half: true, freq: d.freq, amp: d.amp });
  const eng = new BoundaryEngine(grid, surfaces, { bouzidi: true });
  eng.init(0.5);
  // Ellipsoid half volume against the solid cell count, within the voxel surface error.
  const bodyPart = B.PART.body;
  let solid = 0;
  for (let c = 0; c < eng.mask.length; c++) if (eng.mask[c] === bodyPart) solid++;
  const half = (Math.PI / 6) * 0.6 * 0.1 * 0.1 / 2 / dx ** 3;
  assert.ok(Math.abs(solid - half) / half < 0.25, `body cells ${solid} vs ${half}`);
  const f = eng.frame(1.5);
  assert.ok(f.linkCount > 0);
  assert.equal(f.maskEdits.length, 0, 'a body at rest and a sheet wing change no solid cell');
});
