// Run: node --import ./film/lib/three-hooks.mjs --test film/test/
// Pins the film workstream's helpers against the physics' own functions and against geometry facts that do not
// depend on this code: rigid links keep their length, meshing gears keep the tooth ratio, the drawn wing is the
// flight model's wing, every BOM line appears in the bird.
import test from 'node:test';
import assert from 'node:assert/strict';
import { loadModel } from '../../sims/avis-shared/node-load.js';
import { syntheticLog } from './fixture.mjs';
import { drivePose, gearAngles, meshAngle, stationsAt, wingStations, sectionPoint, halfThickness, parseDims, boxProtrusion, linkageAt, leftRightAsymmetry, sineSeries, membraneDip } from '../../sims/avis-shared/model.js';
import { strokeAngle, makeLatticeSpec, wingGrid } from '../../src/physics/kinematics.js';
import { sampleLog } from '../bake_pose.mjs';

const { model, design } = loadModel('larus-gl-2');
const TWO_PI = 2 * Math.PI;

test('right wing follows the flight model stroke law exactly', () => {
  for (const amp of [model.law.ampMin, model.law.ampNominal, model.law.ampMax]) {
    for (let i = 0; i < 36; i++) {
      const ph = (i / 36) * TWO_PI;
      assert.ok(Math.abs(drivePose(model, ph, amp).right.phi - strokeAngle(ph, amp, model.law).phi) < 1e-12);
    }
  }
});

test('both couplers stay rigid at the design length through a cycle', () => {
  for (const amp of [model.law.ampMin, model.law.ampNominal, model.law.ampMax]) {
    for (let i = 0; i < 72; i++) {
      const dp = drivePose(model, (i / 72) * TWO_PI, amp);
      assert.ok(Math.abs(dp.couplerR - model.law.b) < 1e-9, `right coupler ${dp.couplerR}`);
      assert.ok(Math.abs(dp.couplerL - model.law.b) < 1e-9, `left coupler ${dp.couplerL}`);
    }
  }
});

test('crank pin stays on its circle and the rocker pin on its radius from the hinge', () => {
  const amp = model.law.ampNominal;
  const { c } = linkageAt(model.law, amp);
  for (let i = 0; i < 24; i++) {
    const dp = drivePose(model, (i / 24) * TWO_PI, amp);
    assert.ok(Math.abs(Math.hypot(dp.A[0] - dp.O2[0], dp.A[1] - dp.O2[1]) - model.law.a) < 1e-12);
    assert.ok(Math.abs(Math.hypot(dp.right.B[0] - dp.right.O4[0], dp.right.B[1] - dp.right.O4[1]) - c) < 1e-9);
  }
});

test('a single crank pin makes the mirrored left wing differ from the right, and the difference is reported', () => {
  const a = leftRightAsymmetry(model, model.law.ampNominal);
  assert.ok(Number.isFinite(a) && a >= 0);
});

test('the crank is placed at the ground-link distance from the hinge, on the centreline', () => {
  const fb = model.fourBar;
  assert.equal(fb.O2[0], 0);
  assert.ok(Math.abs(Math.hypot(fb.O4[0] - fb.O2[0], fb.O4[1] - fb.O2[1]) - model.law.d) < 1e-12);
});

test('gear train: centre distances from module and teeth, total ratio as the design states', () => {
  const t = model.train;
  assert.ok(Math.abs(t.ratio - design.drive.gearRatio) < 1e-9);
  t.links.forEach((_, j) => {
    const st = t.stages[t.n - 1 - j];
    const d = Math.hypot(t.shafts[j + 1][0] - t.shafts[j][0], t.shafts[j + 1][1] - t.shafts[j][1]);
    assert.ok(Math.abs(d - (st.wheel.m * (st.wheel.z + st.pinion.z)) / 2) < 1e-12);
  });
});

test('one crank turn turns the motor by the gear ratio, alternating sense per mesh', () => {
  const a0 = gearAngles(model.train, 0.3);
  const a1 = gearAngles(model.train, 0.3 + 1e-3);
  const n = model.train.n;
  const motorRate = (a1[n] - a0[n]) / 1e-3;
  assert.ok(Math.abs(Math.abs(motorRate) - model.train.ratio) < 1e-6);
  assert.equal(Math.sign(motorRate), n % 2 ? -1 : 1);
});

test('meshing: a tooth of gear 1 on the line of centres faces a gap of gear 2', () => {
  const z1 = 40, z2 = 18, beta = 0.7;
  const th1 = beta; // a tooth of gear 1 points along the line of centres
  const th2 = meshAngle(th1, z1, z2, beta);
  // gear 2 teeth sit at th2 + 2 pi k / z2; the direction back to gear 1 is beta + pi: it must be half a pitch from a tooth
  const rel = ((((beta + Math.PI - th2) % (TWO_PI / z2)) + TWO_PI / z2) % (TWO_PI / z2));
  assert.ok(Math.abs(rel - Math.PI / z2) < 1e-9);
});

test('the drawn wing mean surface is the flight model wing (kinematics.wingGrid)', () => {
  const ls = makeLatticeSpec(model.wing, 6, 10, { cosine: false });
  const st = stationsAt(model.wing, Array.from(ls.span));
  for (const side of [1, -1]) {
    const phi = 0.31, theta = -0.2;
    const ss = wingStations(model, st, phi, theta, side);
    const g = wingGrid(model.wing, ls, { phi, theta }, side);
    for (let m = 0; m <= ls.ns; m++) {
      for (let i = 0; i <= ls.nc; i++) {
        const p = sectionPoint(model, st, ss, m, ls.xi[i], ls.cam[i], [0, 0, 0]);
        const j = side > 0 ? m : ls.ns - m;
        const q = 3 * (i * (ls.ns + 1) + j);
        for (let k = 0; k < 3; k++) assert.ok(Math.abs(p[k] - g[q + k]) < 1e-12, `station ${m} chord ${i} axis ${k}`);
      }
    }
  }
});

test('section thickness: NACA 4-digit form peaks at its stated thickness near 30% chord and closes at both edges', () => {
  let best = 0, at = 0;
  for (let i = 0; i <= 1000; i++) {
    const h = halfThickness(i / 1000, 0.12);
    if (h > best) {
      best = h;
      at = i / 1000;
    }
  }
  assert.ok(Math.abs(2 * best - 0.12) < 0.0006);
  assert.ok(Math.abs(at - 0.3) < 0.01);
  assert.equal(halfThickness(0, 0.12), 0);
  assert.ok(Math.abs(halfThickness(1, 0.12)) < 1e-12);
});

test('catalogue dimension text parses to metres in printed order', () => {
  assert.deepEqual(parseDims('60 x 25 x 8').values, [0.06, 0.025, 0.008]);
  const m = parseDims('35.0 dia x 40.0');
  assert.equal(m.dia, true);
  assert.deepEqual(m.values, [0.035, 0.04]);
  assert.equal(parseDims(null), null);
});

test('a box inside the loft does not protrude; one wider than the loft does', () => {
  const sec = design.body.loft.sections;
  const mid = sec[Math.floor(sec.length / 3)];
  const small = { x0: mid.x_m - 0.005, x1: mid.x_m + 0.005, y0: -0.002, y1: 0.002, z0: -0.002, z1: 0.002 };
  assert.ok(boxProtrusion(design.body.loft, small).out < 0);
  const wide = { ...small, z0: -mid.width_m, z1: mid.width_m };
  assert.ok(boxProtrusion(design.body.loft, wide).out > 0);
});

test('log resampling reproduces the log at its own sample times and unwraps the phase', () => {
  const log = syntheticLog();
  const c = log.columns;
  const fps = 1 / log.dt;
  const fr = sampleLog(log, fps, 0, 200);
  // the log's times are rounded to 0.1 ms, so a resampled frame lands within that of a sample and its values
  // between that sample and the next
  const within = (v, a, b) => v >= Math.min(a, b) - 1e-9 && v <= Math.max(a, b) + 1e-9;
  for (const k of [0, 17, 150]) {
    assert.ok(Math.abs(fr[k].t - c.t[k]) < 1e-4);
    const j = fr[k].t >= c.t[k] ? k : k - 1;
    assert.ok(within(fr[k].amp, c.amplitude[j], c.amplitude[j + 1]));
    assert.ok(within(fr[k].twistR, c.twist_r[j], c.twist_r[j + 1]));
    assert.ok(within(fr[k].tailL, c.tail[j][0], c.tail[j + 1][0]));
  }
  for (let k = 1; k < fr.length; k++) assert.ok(Math.abs(fr[k].phase - fr[k - 1].phase) < Math.PI);
});

test('the rocker is rigid on the wing root: rocker angle minus stroke angle is constant through a cycle', () => {
  const amp = model.law.ampNominal;
  const d0 = drivePose(model, 0, amp);
  for (let i = 1; i < 36; i++) {
    const dp = drivePose(model, (i / 36) * TWO_PI, amp);
    // equal modulo a turn; the flight model interpolates its drive table, so allow its 1e-5 rad
    const d = (dp.right.rocker - dp.right.phi) - (d0.right.rocker - d0.right.phi);
    assert.ok(Math.abs(d - TWO_PI * Math.round(d / TWO_PI)) < 1e-5);
  }
  // and the stroke reaches its top at phase 0
  for (let i = 1; i < 36; i++) assert.ok(drivePose(model, (i / 36) * TWO_PI, amp).right.phi <= d0.right.phi + 1e-12);
});

test('when the crank pin crosses the centreline the mirrored left wing agrees with the right exactly', () => {
  const amp = model.law.ampNominal;
  let hits = 0;
  let worst = 0;
  for (let i = 0; i < 20000; i++) {
    const ph = (i / 20000) * TWO_PI;
    const dp = drivePose(model, ph, amp);
    if (Math.abs(dp.A[0]) < 2e-6) {
      hits++;
      worst = Math.max(worst, Math.abs(dp.leftDelta));
    }
  }
  assert.ok(hits >= 2);
  assert.ok(worst < 1e-3, `left-right ${worst}`);
  // and the reported asymmetry is the largest difference over the cycle
  let m = 0;
  for (let i = 0; i < 360; i++) m = Math.max(m, Math.abs(drivePose(model, (i / 360) * TWO_PI, amp).leftDelta));
  assert.ok(Math.abs(leftRightAsymmetry(model, amp) - m) < 1e-12);
  assert.ok(m > 0.001);
});

test('every mesh of the train satisfies z1 (th1 - beta) + z2 (th2 - beta - pi) = pi (teeth interleave) at any crank angle', () => {
  const t = model.train;
  for (const crank of [0, 0.37, 2.9, -1.4]) {
    const a = gearAngles(t, crank);
    t.links.forEach((beta, j) => {
      const st = t.stages[t.n - 1 - j];
      const v = st.wheel.z * (a[j] - beta) + st.pinion.z * (a[j + 1] - beta - Math.PI);
      assert.ok(Math.abs(v - Math.PI) < 1e-9);
    });
  }
  const r = (meshAngle(1.001, 40, 18, 0.3) - meshAngle(1, 40, 18, 0.3)) / 0.001;
  assert.ok(Math.abs(r + 40 / 18) < 1e-9);
});

test('section frames are orthonormal, start at the hinge, and the film is the section thickness apart', () => {
  const st = stationsAt(model.wing, [0, 0.2, model.wing.L]);
  const h = model.design.wing.hinge;
  for (const side of [1, -1]) {
    const ss = wingStations(model, st, 0.25, 0.1, side);
    for (let i = 0; i < st.n; i++) {
      const j = 3 * i;
      const dot = (u, v) => u[j] * v[j] + u[j + 1] * v[j + 1] + u[j + 2] * v[j + 2];
      assert.ok(Math.abs(dot(ss.t, ss.t) - 1) < 1e-12 && Math.abs(dot(ss.b, ss.b) - 1) < 1e-12 && Math.abs(dot(ss.e, ss.e) - 1) < 1e-12);
      assert.ok(Math.abs(dot(ss.t, ss.b)) < 1e-12 && Math.abs(dot(ss.t, ss.e)) < 1e-12);
      const up = sectionPoint(model, st, ss, i, 0.3, 0.05, [0, 0, 0]);
      const lo = sectionPoint(model, st, ss, i, 0.3, -0.05, [0, 0, 0]);
      assert.ok(Math.abs(Math.hypot(up[0] - lo[0], up[1] - lo[1], up[2] - lo[2]) - 0.1 * st.chord[i]) < 1e-12);
    }
    assert.ok(Math.abs(ss.A[1] - h[1]) < 1e-12 && Math.abs(ss.A[2] - side * h[2]) < 1e-12);
  }
  assert.equal(st.seg[0], 0);
  assert.equal(st.seg[2], model.wing.segments.length - 1);
});

test('drive planes are in order and no two stages overlap along x', () => {
  const P = model.planes, t = model.train;
  assert.ok(P.couplerL < P.couplerR && P.couplerR < P.crankArm && P.crankArm < P.plateAft);
  let prev = P.plateAft + P.plateT / 2;
  for (let s = t.n - 1; s >= 0; s--) {
    const w = Math.max(t.stages[s].wheel.face, t.stages[s].pinion.face);
    assert.ok(P.stage[s] - w / 2 > prev, `stage ${s}`);
    prev = P.stage[s] + w / 2;
  }
  assert.ok(P.plateFwd - P.plateT / 2 > prev && P.motorAft > P.plateFwd);
});

test('catalogue gears: pitch radius from module and teeth, tip radius from the listed outside diameter', () => {
  for (const g of model.train.gears) {
    const s = model.parts[g.id].specs;
    assert.ok(Math.abs(g.rp - (s.module_mm * s.teeth) / 2000) < 1e-12);
    // the listed outside diameter, or a standard full-depth tooth's (ISO 53: addendum one module) when none is listed
    const tip = s.outside_dia_mm ?? s.module_mm * (s.teeth + 2);
    assert.ok(Math.abs(g.ro - tip / 2000) < 1e-12, `${g.id} tip`);
    assert.equal(g.face, s.face_width_mm / 1000);
  }
});

test('fit report: the crank axis below the loft is measured from the loft section at the hinge', () => {
  const sec = design.body.loft.sections;
  const x = design.wing.hinge[0];
  let s = null;
  for (let i = 1; i < sec.length; i++) if (x <= sec[i - 1].x_m && x >= sec[i].x_m) {
    const f = (sec[i - 1].x_m - x) / (sec[i - 1].x_m - sec[i].x_m);
    s = { y: sec[i - 1].y_m + f * (sec[i].y_m - sec[i - 1].y_m), h: sec[i - 1].height_m + f * (sec[i].height_m - sec[i - 1].height_m) };
  }
  const below = s.y - s.h / 2 - model.fourBar.O2[1];
  assert.ok(Math.abs(model.layout.crankBelowLoft_m - below) < 1e-12);
  const issue = model.fit.issues.find((i) => i.part === 'crank axis');
  if (below > 0) assert.ok(issue && issue.text.includes(`${(below * 1000).toFixed(0)} mm below`));
  else assert.equal(issue, undefined);
  // every part box reported as fitting really is inside the loft
  for (const p of model.layout.parts) if (p.protrusion === 0 && p.name !== 'motor') assert.ok(boxProtrusion(design.body.loft, p.box).out <= 0.0005, p.name);
});

test('rib stations: the BOM count split between the wings, evenly root to tip', () => {
  const listed = +design.bom.structure.find((s) => /^ribs/.test(s.name)).name.match(/(\d+)\s*off/)[1];
  assert.equal(model.ribs.length, listed / 2);
  assert.equal(model.ribs[0], 0);
  assert.ok(Math.abs(model.ribs.at(-1) - model.wing.L) < 1e-12);
  const d = model.ribs[1] - model.ribs[0];
  for (let i = 2; i < model.ribs.length; i++) assert.ok(Math.abs(model.ribs[i] - model.ribs[i - 1] - d) < 1e-12);
});

test('sine series of a profile: recovers known harmonics after removing the line through its ends', () => {
  const xs = Array.from({ length: 801 }, (_, i) => i / 800);
  const b = sineSeries(xs, xs.map((x) => 0.2 + 0.1 * x + Math.sin(Math.PI * x) + 0.3 * Math.sin(3 * Math.PI * x) + 0.05 * Math.sin(8 * Math.PI * x)), 8);
  assert.ok(Math.abs(b[0] - 1) < 1e-4 && Math.abs(b[2] - 0.3) < 1e-4 && Math.abs(b[7] - 0.05) < 1e-4, `${b[0]} ${b[2]} ${b[7]}`);
  for (const k of [1, 3, 4, 5, 6]) assert.ok(Math.abs(b[k]) < 1e-4, `b${k + 1} = ${b[k]}`);
});

test('taut film between ribs: the Laplace membrane, zero on the ribs and the edge rods, analytic for one harmonic', () => {
  // f = sin(pi xi) over a bay of gap/chord sigma: z = sin(pi xi) cosh(pi sigma (u - 1/2)) / cosh(pi sigma / 2)
  for (const sigma of [0.3, 0.5, 0.8]) {
    const want = 1 - 1 / Math.cosh((Math.PI * sigma) / 2);
    assert.ok(Math.abs(membraneDip([1], 0.5, 0.5, sigma) - want) < 1e-12);
  }
  const xs = model.section.map((q) => q.xi);
  const b = sineSeries(xs, model.section.map((q) => q.cam + q.half));
  for (const x of [0.1, 0.4, 0.8]) for (const u of [0, 1]) assert.ok(Math.abs(membraneDip(b, x, u, 0.4)) < 1e-12);
  for (const x of [0, 1]) assert.ok(Math.abs(membraneDip(b, x, 0.37, 0.4)) < 1e-12);
  // the film surface f - dip solves z_xx / c^2 + z_yy / g^2 = 0 (finite differences, chord c, rib gap g = sigma c)
  const sigma = 0.4, h = 2e-4;
  const z = (x, u) => b.reduce((s, bn, k) => s + bn * Math.sin((k + 1) * Math.PI * x), 0) - membraneDip(b, x, u, sigma);
  for (const [x, u] of [[0.3, 0.5], [0.5, 0.3], [0.7, 0.6]]) {
    const zxx = (z(x + h, u) - 2 * z(x, u) + z(x - h, u)) / (h * h);
    const zuu = (z(x, u + h) - 2 * z(x, u) + z(x, u - h)) / (h * h) / (sigma * sigma);
    assert.ok(Math.abs(zxx + zuu) < 0.02 * Math.abs(zxx) + 1e-3, `xi ${x} u ${u}: ${zxx} + ${zuu}`);
  }
  // the real section dips a little at mid bay: under 3% of the chord, inward on both faces near the nose
  const bl = sineSeries(xs, model.section.map((q) => q.cam - q.half));
  let worst = 0;
  for (let i = 1; i < 100; i++) worst = Math.max(worst, membraneDip(b, i / 100, 0.5, 0.45));
  assert.ok(worst > 0.005 && worst < 0.03, `upper mid-bay dip ${worst}`);
  assert.ok(membraneDip(bl, 0.05, 0.5, 0.45) < 0);
});

test('journals: each bearing and bushing line split by the joints its role names; a bare "bearing" line is pooled', async () => {
  const { journals, journalDims } = await import('../../sims/avis-shared/model.js');
  const parts = [
    { role: 'bushing: shoulder pivots, two per side (4), coupler eyes, two per coupler (4)', id: 'b1', count: 8 },
    { role: 'bushing: crank shaft (2)', id: 'b2', count: 2 },
    { role: 'bushing: gear shaft 1 (2), gear shaft 12 (2)', id: 'b3', count: 4 },
    { role: 'bearing: elbow knuckles (4)', id: 'b4', count: 4 },
    { role: 'bearing', id: 'b5', count: 6 },
    { role: 'gear', id: 'g', count: 2 },
  ];
  assert.deepEqual(journals({ bom: { parts } }), [
    { id: 'b1', kind: 'bushing', place: 'shoulder', shaft: null, count: 4 },
    { id: 'b1', kind: 'bushing', place: 'coupler', shaft: null, count: 4 },
    { id: 'b2', kind: 'bushing', place: 'crank', shaft: null, count: 2 },
    { id: 'b3', kind: 'bushing', place: 'shaft', shaft: 1, count: 2 },
    { id: 'b3', kind: 'bushing', place: 'shaft', shaft: 12, count: 2 },
    { id: 'b4', kind: 'bearing', place: 'elbow', shaft: null, count: 4 },
    { id: 'b5', kind: 'bearing', place: 'pooled', shaft: null, count: 6 },
  ]);
  // every line of today's design is fully placed
  for (const p of design.bom.parts.filter((x) => /^(bearing|bushing)\b/.test(x.role))) {
    assert.equal(journals(design).filter((j) => j.id === p.id).reduce((a, j) => a + j.count, 0) >= p.count, true, p.role);
  }
  assert.deepEqual(journalDims({ dimensions_mm_d_D_B: '5x13x4' }), { d: 0.005, D: 0.013, B: 0.004, flangeD: 0, flangeB: 0 });
  assert.deepEqual(journalDims({ d1_mm: 6, d2_mm: 8, b1_mm: 6, d3_mm: 12, b2_mm: 1 }), { d: 0.006, D: 0.008, B: 0.006, flangeD: 0.012, flangeB: 0.001 });
  assert.deepEqual(journalDims({ d1_mm: 6, d2_mm: 8, b1_mm: 6 }), { d: 0.006, D: 0.008, B: 0.006, flangeD: 0, flangeB: 0 });
  assert.deepEqual(journalDims({}), { d: 0.004, D: 0.009, B: 0.0025, flangeD: 0, flangeB: 0 });
});

test('servos: tail servos either side of their lump; a stroke servo exactly when the design has a lump for one', async () => {
  const { prepare, parseDims } = await import('../../sims/avis-shared/model.js');
  const box = (m, n) => m.layout.parts.find((p) => p.name === n)?.box;
  const tl = box(model, 'servo_tail_L'), tr = box(model, 'servo_tail_R');
  const lump = design.mass.lumps.find((l) => l.name === 'tail servos').pos;
  // tail servos sized from their own catalogue entry; the stroke servo's entry, then a default, only without one
  const strokeId = design.bom.parts.find((p) => p.role === 'stroke amplitude servo')?.id;
  const sv = parseDims(model.parts[strokeId]?.specs?.dimensions_text_mm)?.values ?? [0.033, 0.01, 0.035];
  const tailId = design.bom.parts.find((p) => p.role === 'tail servo')?.id;
  const tv = parseDims(model.parts[tailId]?.specs?.dimensions_text_mm)?.values ?? sv;
  assert.ok(Math.abs(tr.cz - tl.cz - 2 * 0.55 * tv[1]) < 1e-9 && Math.abs((tl.cz + tr.cz) / 2 - lump[2]) < 1e-9);
  assert.deepEqual([tl.sx, tl.sy, tl.sz].map((v) => +v.toFixed(6)), [tv[0], tv[2], tv[1]].map((v) => +v.toFixed(6)), 'tail servo at its catalogue size');
  for (const b of [tl, tr]) assert.ok(Math.abs(b.cx - lump[0]) < 0.005 && Math.abs(b.cy - lump[1]) < 0.005, 'at the tail-servo lump');
  const hasLump = design.mass.lumps.some((l) => l.name === 'stroke servo');
  assert.equal(!!box(model, 'servo_stroke'), hasLump);
  // fit.js's packing view adds a massless lump for a fixed stroke: the box is kept free there, as before
  const withLump = { ...design, mass: { ...design.mass, lumps: [...design.mass.lumps.filter((l) => l.name !== 'stroke servo'), { name: 'stroke servo', mass_kg: 0, pos: [-0.03, 0, 0] }] } };
  const s = box(prepare(withLump, model.parts), 'servo_stroke');
  assert.ok(s && Math.abs(s.cx + 0.03) < 0.03, 'stroke servo placed at its lump');
  assert.deepEqual([s.sx, s.sy, s.sz].sort().map((v) => +v.toFixed(6)), [...sv].sort().map((v) => +v.toFixed(6)));
});

// Packing finds each part by its own role: an ESC role that mentions the flight controller must not stand in for it,
// and tail servos are sized from their own catalogue entry.
test('layoutParts places the flight controller and tail servos as their own catalogue parts', async () => {
  const { flightControllerRole } = await import('../../sims/avis-shared/model.js');
  const bom = { parts: [{ role: 'flap ESC (no BEC; the flight controller\'s 5 V BEC supplies the avionics)' }, { role: 'flight controller with IMU and baro' }] };
  assert.equal(flightControllerRole(bom), 'flight controller with IMU and baro');
  assert.equal(flightControllerRole({ parts: [{ role: 'ESC for the flight controller' }, { role: 'main flight controller' }] }), 'main flight controller');
  const fc = model.layout.parts.find((p) => p.name === 'fc');
  assert.equal(fc.id, design.bom.parts.find((p) => p.role === flightControllerRole(design.bom)).id);
  assert.doesNotMatch(fc.id, /esc/i);
  const tail = model.layout.parts.find((p) => p.name === 'servo_tail_L');
  const tailSpec = model.parts?.[tail.id]?.specs;
  const dims = parseDims(tailSpec?.dimensions_text_mm)?.values;
  if (dims) {
    const b = tail.box, size = [b.x1 - b.x0, b.y1 - b.y0, b.z1 - b.z0].sort((a, c) => a - c);
    assert.deepEqual(size.map((v) => +v.toFixed(5)), [...dims].sort((a, c) => a - c).map((v) => +v.toFixed(5)));
  }
});

test('catalogue sizes with their units; the regulator is packed for the film only, never in the physics view', async () => {
  const { catalogueDims, prepare, prepareFilm } = await import('../../sims/avis-shared/model.js');
  const near = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-12);
  assert.ok(near(catalogueDims({ dimensions_text: '1 x 1 x 0.375 in' }).values, [0.0254, 0.0254, 0.009525]));
  assert.ok(near(catalogueDims({ dimensions_text: '2 x 3 x 4 cm' }).values, [0.02, 0.03, 0.04]));
  assert.ok(near(catalogueDims({ dimensions_text: '20 x 30 x 4 mm' }).values, [0.02, 0.03, 0.004]));
  assert.ok(near(catalogueDims({ dimensions_text_mm: '29.0 x 13.0 x 30.0', dimensions_text: '9 x 9 x 9 in' }).values, [0.029, 0.013, 0.03]));
  assert.equal(catalogueDims({ dimensions_text: '1 x 1 x 0.375' }), null);
  assert.equal(catalogueDims({}), null);
  const reg = design.bom.parts.find((p) => p.role === 'servo regulator');
  if (!reg) return;
  assert.ok(!prepare(design, model.parts).layout.parts.some((p) => p.name === 'regulator'), 'src/design packs without it');
  const film = prepareFilm(design, model.parts);
  const box = film.layout.parts.find((p) => p.name === 'regulator').box;
  const d = catalogueDims(model.parts[reg.id].specs).values;
  assert.deepEqual([box.sx, box.sy, box.sz].map((v) => +v.toFixed(6)).sort(), [...d].map((v) => +v.toFixed(6)).sort());
  const clash = film.layout.parts.filter((p) => p.name !== 'regulator').some(({ box: o }) => box.x0 < o.x1 - 1e-4 && box.x1 > o.x0 + 1e-4 && box.y0 < o.y1 - 1e-4 && box.y1 > o.y0 + 1e-4 && box.z0 < o.z1 - 1e-4 && box.z1 > o.z0 + 1e-4);
  assert.equal(clash, false, 'the regulator overlaps another part');
});
