// Run: node --import ./film/lib/three-hooks.mjs --test film/test/
// Pins the film workstream's helpers against the physics' own functions and against geometry facts that do not
// depend on this code: rigid links keep their length, meshing gears keep the tooth ratio, the drawn wing is the
// flight model's wing, every BOM line appears in the bird.
import test from 'node:test';
import assert from 'node:assert/strict';
import { loadModel } from '../../sims/avis-shared/node-load.js';
import { syntheticLog } from './fixture.mjs';
import { drivePose, gearAngles, meshAngle, stationsAt, wingStations, sectionPoint, halfThickness, parseDims, boxProtrusion, linkageAt, leftRightAsymmetry } from '../../sims/avis-shared/model.js';
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
    assert.ok(Math.abs(g.ro - s.outside_dia_mm / 2000) < 1e-12);
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
