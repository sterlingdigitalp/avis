// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// Pins the title's framing: while the title is up, no part of either wing (root to tip, leading to trailing edge)
// passes over "Avis", the subtitle or the rule, and the bird stays inside the frame, over the whole 4 s push-in at the
// real cruise flap. Projected through the camera math SEQ.title uses (film.js TITLE_CAM), from the same
// wingStations()/sectionPoint() geometry bird.js poses the wing with, so no browser is needed.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { loadModel, loadVerification } from '../../sims/avis-shared/node-load.js';
import { numbers } from '../../sims/avis-shared/numbers.js';
import { drivePose, stationsAt, wingStations, sectionPoint } from '../../sims/avis-shared/model.js';
import { TITLE_CAM, titleOrbit } from '../../sims/avis-shared/film.js';

const { model } = loadModel('larus-gl-2');
const N = numbers(model, { gates: loadVerification('larus-gl-2'), fit: [] });
const DEG = 180 / Math.PI;
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2);
const DUR = 4.0; // film/edit.json's title

const camera = new THREE.PerspectiveCamera(30, 1920 / 1080, 0.01, 60000);
function orbit(target, azDeg, elDeg, dist, fov) {
  const az = azDeg / DEG, el = elDeg / DEG;
  camera.fov = fov;
  camera.position.set(target.x + dist * Math.cos(el) * Math.cos(az), target.y + dist * Math.sin(el), target.z + dist * Math.cos(el) * Math.sin(az));
  camera.lookAt(target);
  camera.updateProjectionMatrix();
}

// The title block's text in frame pixels (left 120 px, top TITLE_CAM.top; measured on the v21 title frames: "Avis"
// 200 px type 180 px tall and about 400 px wide, the subtitle 28 px under it and ending near x = 1185, the rule 36 px
// under that), each with 16 px of room and 24 px more below for the block's rise as it fades in.
const T = TITLE_CAM.top, M = 16, RISE = 24;
const TEXT = {
  'the title "Avis"': [120 - M, T - M, 520 + M, T + 180 + M + RISE],
  'the subtitle': [120 - M, T + 208 - M, 1190 + M, T + 264 + M + RISE],
  'the rule': [120 - M, T + 300 - M, 216 + M, T + 304 + M + RISE],
};
const under = (x, y) => Object.entries(TEXT).find(([, [x0, y0, x1, y1]]) => x >= x0 && x <= x1 && y >= y0 && y <= y1)?.[0];

test('title camera keeps both wings off "Avis", the subtitle and the rule, and inside the frame, through the push-in', () => {
  const ST = stationsAt(model.wing, [...model.ribSt.s, model.wing.L]);
  const c = TITLE_CAM;
  let top = Infinity, bottom = -Infinity, left = Infinity, right = -Infinity;
  for (let t = 0; t <= DUR + 1e-9; t += 0.01) {
    const u = clamp(t / DUR);
    orbit(new THREE.Vector3(...c.target), c.az0 + c.azSpan * u, c.el0 + c.elSpan * u, c.dist0 + c.distSpan * ease(u), c.fov);
    camera.setViewOffset(1920, 1080, c.shift, 0, 1920, 1080);
    camera.updateMatrixWorld(true);
    const ph = 2 * Math.PI * N.flapHz * t; // the title flaps at the real cruise frequency, full speed
    const dp = drivePose(model, ph, N.ampDeg / DEG);
    for (const side of [1, -1]) {
      const s = wingStations(model, ST, side > 0 ? dp.right.phi : dp.left.phi, N.twistAt(ph, side), side);
      for (let i = 0; i < ST.n; i++) for (const xi of [0, 0.25, 0.5, 0.75, 1]) {
        const p = sectionPoint(model, ST, s, i, xi, 0, [0, 0, 0]);
        const v = new THREE.Vector3(p[0], p[1], p[2]).project(camera);
        const x = (v.x * 0.5 + 0.5) * 1920, y = (-v.y * 0.5 + 0.5) * 1080;
        // the block fades in from u = 0.05 (film.js SEQ.title): from then on the wing keeps off its text
        if (u >= 0.05) assert.ok(!under(x, y), `t=${t.toFixed(2)} s: ${side > 0 ? 'right' : 'left'} wing station ${i} at (${x.toFixed(0)}, ${y.toFixed(0)}) over ${under(x, y)}`);
        top = Math.min(top, y); bottom = Math.max(bottom, y); left = Math.min(left, x); right = Math.max(right, x);
      }
    }
  }
  assert.ok(top >= 20 && bottom <= 1060 && left >= 20 && right <= 1900, `wings reach x ${left.toFixed(0)}..${right.toFixed(0)}, y ${top.toFixed(0)}..${bottom.toFixed(0)} (want 20 px inside the frame)`);
  // the block itself sits clear of the claim, bottom left (.honest: 52 px up, about 36 px tall)
  assert.ok(T + 304 + RISE < 1080 - 52 - 36, `the rule at y ${T + 304} runs into the claim`);
});

test('the page flies the camera this test checks: titleOrbit is the push-in above', () => {
  const c = TITLE_CAM;
  for (const u of [0, 0.13, 0.5, 0.77, 1]) {
    const [target, az, el, dist, fov] = titleOrbit(u);
    assert.deepEqual(target.toArray(), c.target);
    assert.ok(Math.abs(az - (c.az0 + c.azSpan * u)) < 1e-12 && Math.abs(el - (c.el0 + c.elSpan * u)) < 1e-12, `u=${u}`);
    assert.ok(Math.abs(dist - (c.dist0 + c.distSpan * ease(u))) < 1e-12 && fov === c.fov, `u=${u}`);
  }
});
