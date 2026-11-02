// Geometry helpers against brute force and counting, independent of the code under test.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { dist2PointTri, BoundaryEngine } from '../src/geometry.mjs';
import { CpuLbm } from '../src/cpu.mjs';
import { AXIS, Q } from '../src/lattice.mjs';
import { readDesign, trimAt } from '../src/bird.mjs';
import * as kin from '../../../src/physics/kinematics.js';

function bruteDist2(p, a, b, c) {
  let best = Infinity;
  const N = 400;
  for (let i = 0; i <= N; i++) for (let j = 0; j <= N - i; j++) {
    const u = i / N, v = j / N, w = 1 - u - v;
    const q = [0, 1, 2].map((k) => w * a[k] + u * b[k] + v * c[k]);
    best = Math.min(best, (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2);
  }
  return best;
}

test('point-triangle distance matches dense sampling in every Voronoi region', () => {
  const a = [0, 0, 0], b = [3, 0.2, 0.1], c = [0.5, 2.5, -0.3];
  // Points near each vertex, each edge, and the face, on both sides.
  const pts = [[-1, -1, 0.5], [4, -0.5, 0], [0.3, 3.5, 1], [1.5, -1, -0.4], [2.2, 1.8, 0.7], [-0.8, 1.2, 0.2], [1, 0.8, 1.3], [1, 0.8, -0.9], [1.2, 0.5, 0]];
  for (const p of pts) {
    const exact = dist2PointTri(...p, ...a, ...b, ...c);
    const brute = bruteDist2(p, a, b, c);
    // Sampling can only overestimate, by at most (edge / N)^2-scale terms.
    assert.ok(exact <= brute + 1e-12 && brute - exact < 2e-3, `${p}: ${exact} vs ${brute}`);
  }
});

test('a sheet moved up one cell sweeps exactly the nodes it passed', () => {
  const nx = 12, ny = 10, nz = 6;
  const grid = { nx, ny, nz, xMode: AXIS.periodic, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic };
  let y = 3.4;
  const pos = new Float64Array(12), vel = Float64Array.from([0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0]);
  const pose = () => { pos.set([2.3, y, -3, 8.6, y, -3, 2.3, y, nz + 2, 8.6, y, nz + 2]); return { pos, vel }; };
  const eng = new BoundaryEngine(grid, [{ part: 0, kind: 'thin', tris: Uint32Array.from([0, 1, 3, 0, 3, 2]), moving: true, pose }]);
  eng.init(0);
  y = 4.6; // passes the node row iy = 4 only
  const f = eng.frame(1);
  // Columns ix = 3..8 under the sheet, all nz rows.
  assert.equal(eng.swept.size, 6 * nz);
  for (const c of eng.swept.keys()) assert.equal(Math.floor(c / nx) % ny, 4);
  assert.equal(f.cellCount, 6 * nz);
});

test('half-model sweep near the symmetry plane matches the full model', () => {
  const nx = 10, ny = 10;
  const mk = (nz, zMin, zMax, mirrorZ) => {
    let y = 3.4;
    const pos = new Float64Array(12), vel = new Float64Array(12);
    const pose = () => { pos.set([2.3, y, zMin, 7.6, y, zMin, 2.3, y, zMax, 7.6, y, zMax]); return { pos, vel }; };
    const grid = { nx, ny, nz, xMode: AXIS.periodic, yLowMode: AXIS.freeslip, yHighMode: AXIS.freeslip, zLowMode: AXIS.freeslip, zHighMode: AXIS.freeslip };
    const eng = new BoundaryEngine(grid, [{ part: 0, kind: 'thin', tris: Uint32Array.from([0, 1, 3, 0, 3, 2]), moving: true, mirrorZ, pose }]);
    eng.init(0);
    y = 4.6;
    eng.frame(1);
    return eng.swept.size;
  };
  // Full model: plate from z = 5.5 - 6.2 to 5.5 + 6.2 across the plane at 5.5. Half: z from -0.5.
  const full = mk(12, 5.5 - 6.2, 5.5 + 6.2, false);
  const half = mk(6, -0.5, -0.5 + 6.2, true);
  assert.equal(half, 5 * 6);
  assert.equal(full, 2 * half);
});

test('sponge profile: full strength at the face, half at mid-width, zero beyond', () => {
  const s = new CpuLbm({ nx: 20, ny: 4, nz: 4, tau0: 0.8, precision: 'f64', xMode: AXIS.periodic, yLowMode: AXIS.periodic, yHighMode: AXIS.periodic, zLowMode: AXIS.periodic, zHighMode: AXIS.periodic, sponge: { strength: 0.3, xLow: 8 } });
  assert.ok(Math.abs(s.spongeAt(0, 1, 1) - 0.3) < 1e-12);
  assert.ok(Math.abs(s.spongeAt(4, 1, 1) - 0.15) < 1e-12);
  assert.equal(s.spongeAt(8, 1, 1), 0);
  assert.equal(s.spongeAt(15, 1, 1), 0);
  // Sponge drives a disturbed state to the target equilibrium and leaves the target itself alone.
  s.setField((ix) => ({ drho: ix < 8 ? 0.01 : 0, u: [0, 0, 0] }));
  s.step(200);
  let m = 0;
  for (let i = 0; i < Q; i++) m += s.src[(1 * 20 + 0) * Q + i];
  assert.ok(Math.abs(m) < 1e-4, `density at the face ${m}`);
});

test('design reader takes cruise speed and the trim table at that speed', () => {
  const d = JSON.parse(fs.readFileSync(new URL('../../../data/designs/larus-gl-2.json', import.meta.url)));
  const r = readDesign(d, kin);
  const row = d.control.trim.find((t) => t.V === d.cruise.V);
  assert.equal(r.speed, d.cruise.V);
  assert.ok(Math.abs(r.freq - row.freq) < 1e-12 && Math.abs(r.bodyAlpha - row.theta) < 1e-12 && Math.abs(r.tailDeflection - row.tail) < 1e-12);
  assert.equal(r.law.kind ?? 'crank-rocker', 'crank-rocker');
  // Linear interpolation halfway between two rows.
  const [a, b] = d.control.trim.slice(0, 2);
  const mid = trimAt(d, (a.V + b.V) / 2);
  assert.ok(Math.abs(mid.freq - (a.freq + b.freq) / 2) < 1e-12);
  // Clamped outside the table.
  assert.equal(trimAt(d, 0).freq, d.control.trim[0].freq);
  assert.throws(() => readDesign({ name: 'x' }, kin), /wing\.segments/);
});

test('edgeFrac puts both plate edges at the same sub-cell position for odd and even chords', async () => {
  const { plateCase } = await import('../src/jobs.mjs');
  const frac = (x) => x - Math.floor(x);
  for (const c of [80, 113, 160, 226]) {
    const p = plateCase({ c, Re: 5000, k: 0.5, alpha0: 0, axis: -0.25, edgeFrac: 0.27 });
    const pos = p.surf.pose(0).pos;
    let lo = Infinity, hi = -Infinity;
    for (let v = 0; v < pos.length; v += 3) { lo = Math.min(lo, pos[v]); hi = Math.max(hi, pos[v]); }
    assert.ok(Math.abs(hi - lo - c) < 1e-9, `c${c}: chord ${hi - lo}`);
    assert.ok(Math.abs(frac(hi) - 0.27) < 1e-9 && Math.abs(frac(lo) - 0.27) < 1e-9, `c${c}: edges at ${frac(lo)}, ${frac(hi)}`);
  }
  // For a chord with a whole-number half, edgeFrac 0.27 is the default placement, so earlier runs stand.
  const a = plateCase({ c: 80, Re: 5000, k: 0.5 }), b = plateCase({ c: 80, Re: 5000, k: 0.5, edgeFrac: 0.27 });
  assert.deepEqual(Array.from(a.surf.pose(0).pos), Array.from(b.surf.pose(0).pos));
  const odd = plateCase({ c: 113, Re: 5000, k: 0.5 });
  assert.ok(Math.abs(odd.edgeFrac - 0.77) < 1e-9, `default odd-chord edges at ${odd.edgeFrac}`);
});
