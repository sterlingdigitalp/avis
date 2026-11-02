// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// Pins the twist sequence's camera framing: both wings (root to tip, leading to trailing edge) stay inside the
// 1920x1080 frame and out from under the caption, the stats panel and the claim, over a full stroke cycle at any
// point in the sequence. Computed from the same wingStations()/sectionPoint() pure functions bird.js poses the wing geometry
// with (model.js), projected through the exact camera math sims/avis-shared/film.js's SEQ.twist uses -- so this
// does not need a browser, and catches the wing leaving the frame without rendering anything.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { loadModel, loadVerification } from '../../sims/avis-shared/node-load.js';
import { numbers } from '../../sims/avis-shared/numbers.js';
import { drivePose, stationsAt, wingStations, sectionPoint } from '../../sims/avis-shared/model.js';
import { TWIST_CAM } from '../../sims/avis-shared/film.js';

const { model } = loadModel('larus-gl-2');
const gates = loadVerification('larus-gl-2');
const N = numbers(model, { gates, fit: [] });

const DEG = 180 / Math.PI;
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2);

// The twist sequence's own camera (film.js exports it), driven as SEQ.twist drives it, with `flap(t, 0.25)`.
const CAM = TWIST_CAM;
const SPEED = 0.25;
const DUR = 6.5;

const camera = new THREE.PerspectiveCamera(30, 1920 / 1080, 0.01, 60000);
function orbit(target, azDeg, elDeg, dist, fov) {
  const az = azDeg / DEG, el = elDeg / DEG;
  camera.fov = fov;
  camera.position.set(target.x + dist * Math.cos(el) * Math.cos(az), target.y + dist * Math.sin(el), target.z + dist * Math.cos(el) * Math.sin(az));
  camera.lookAt(target);
  camera.updateProjectionMatrix();
  camera.updateMatrixWorld(true); // project() reads matrixWorldInverse, which only lookAt()/position do not update
}

// every rib station plus the physical tip, at leading edge / midchord / trailing edge: the drawn wing's own
// silhouette extremes (identical composition to the geometry bird.js builds, per model.js's own doc comment).
const stList = [...model.ribSt.s, model.wing.L];
const ST = stationsAt(model.wing, stList);

// Overlay boxes in frame pixels, measured on out/film/review/polish/studio/after/twist-34.0.png: the stats panel
// (top right), the caption (top left) and the claim (bottom left). No part of either wing may pass under them.
const OVERLAYS = { panel: [1270, 0, 1920, 560], caption: [0, 0, 930, 150], claim: [0, 970, 560, 1080] };
const covered = (x, y) => Object.entries(OVERLAYS).find(([, [x0, y0, x1, y1]]) => x >= x0 && x <= x1 && y >= y0 && y <= y1)?.[0];
function wingScreenBox(dp, ph) {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, under = null;
  // the twist focus draws the ribs and film of both wings, so both must stay clear
  for (const side of [1, -1]) {
    const s = wingStations(model, ST, side > 0 ? dp.right.phi : dp.left.phi, N.twistAt(ph, side), side);
    for (let i = 0; i < ST.n; i++) {
      for (const xi of [0, 0.5, 1]) {
        const p = sectionPoint(model, ST, s, i, xi, 0, [0, 0, 0]);
        const v = new THREE.Vector3(p[0], p[1], p[2]).project(camera);
        const px = (v.x * 0.5 + 0.5) * 1920, py = (-v.y * 0.5 + 0.5) * 1080;
        minX = Math.min(minX, px); maxX = Math.max(maxX, px);
        minY = Math.min(minY, py); maxY = Math.max(maxY, py);
        under ??= covered(px, py) && `${side > 0 ? 'right' : 'left'} wing station ${i} under the ${covered(px, py)}`;
      }
    }
  }
  return { minX, maxX, minY, maxY, under };
}

test('twist camera keeps both wings inside the frame and off the caption, panel and claim over a full stroke cycle', () => {
  let top = Infinity, bottom = -Infinity, left = Infinity, right = -Infinity;
  for (let t = 0; t <= DUR; t += 0.02) {
    const u = clamp(t / DUR);
    const target = new THREE.Vector3(...CAM.target);
    orbit(target, CAM.az0 + CAM.azSpan * ease(u), CAM.el, CAM.dist0 + CAM.distSpan * ease(u), CAM.fov);
    camera.setViewOffset(1920, 1080, CAM.shift, 0, 1920, 1080);
    const ph = 2 * Math.PI * N.flapHz * t * SPEED;
    const dp = drivePose(model, ph, N.ampDeg / DEG);
    const b = wingScreenBox(dp, ph);
    assert.ok(!b.under, `t=${t.toFixed(2)} s: ${b.under}`);
    top = Math.min(top, b.minY);
    bottom = Math.max(bottom, b.maxY);
    left = Math.min(left, b.minX);
    right = Math.max(right, b.maxX);
  }
  // The caption (.fcap) sits in the top-left ~140px; give it a clean margin. Elsewhere, 20px of plain frame margin
  // so the wing is never visibly clipped at the extremes of the stroke.
  assert.ok(top >= 140, `wing's top edge reaches y=${top.toFixed(1)}px (want >= 140, clear of the caption band)`);
  assert.ok(bottom <= 1060, `wing's bottom edge reaches y=${bottom.toFixed(1)}px (want <= 1060, inside the frame)`);
  assert.ok(left >= 20, `wing's left edge reaches x=${left.toFixed(1)}px (want >= 20, inside the frame)`);
  assert.ok(right <= 1900, `wing's right edge reaches x=${right.toFixed(1)}px (want <= 1900, inside the frame)`);
});
