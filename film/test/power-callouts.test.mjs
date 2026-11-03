// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// The power sequence's callout lines (part to label) never cross: the labels' slots are assigned once from the
// parts' screen tracks over the sequence (film.js assignSlots/powerTracks), and the lines checked here on a finer
// time grid than the assignment saw, with the bird built in node exactly as the page builds it.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { loadVerification } from '../../sims/avis-shared/node-load.js';
import { numbers } from '../../sims/avis-shared/numbers.js';
import { POWER_CAM, POWER_LABELS, assignSlots, powerTracks, powerParts, segmentsCross, sphereOf } from '../../sims/avis-shared/film.js';
import { nodeBird } from '../lib/bird-node.mjs';

const DEG = 180 / Math.PI;

test('segments cross only where they properly intersect', () => {
  assert.ok(segmentsCross([[0, 0], [10, 10]], [[0, 10], [10, 0]]));
  assert.ok(!segmentsCross([[0, 0], [10, 0]], [[0, 5], [10, 5]]), 'parallel');
  assert.ok(!segmentsCross([[0, 0], [4, 4]], [[6, 6], [10, 0]]), 'apart');
  assert.ok(!segmentsCross([[0, 0], [10, 10]], [[10, 10], [20, 0]]), 'sharing an end point');
  // one ending on the other's middle touches it, it does not cross it (either way round)
  assert.ok(!segmentsCross([[0, 0], [10, 0]], [[5, 0], [5, 10]]));
  assert.ok(!segmentsCross([[0, 0], [10, 0]], [[5, -10], [5, 0]]));
  assert.ok(!segmentsCross([[5, -10], [5, 0]], [[0, 0], [10, 0]]));
  assert.ok(!segmentsCross([[5, 0], [5, 10]], [[0, 0], [10, 0]]));
});

test('slots: the assignment with no crossing wins, and the listed order wins a tie', () => {
  const slots = [[0, 0], [0, 100]];
  // part 0 sits low, part 1 high: listed order would cross, swapped it does not
  assert.deepEqual(assignSlots([[[100, 100]], [[100, 0]]], slots).perm, [1, 0]);
  assert.deepEqual(assignSlots([[[100, 0]], [[100, 100]]], slots).perm, [0, 1]);
  // a part off screen draws no line, so it constrains nothing
  assert.deepEqual(assignSlots([[null], [[100, 0]]], slots).perm, [0, 1]);
  // the result says how many crossings and moves it costs
  assert.deepEqual(assignSlots([[[100, 100]], [[100, 0]]], slots), { perm: [1, 0], cross: 0, moved: 2 });
  assert.deepEqual(assignSlots([[[100, 0]], [[100, 100]]], slots), { perm: [0, 1], cross: 0, moved: 0 });
  // fewer crossings beat fewer moves: the only clean order moves all three, a later order moving two crosses
  const three = [[0, 0], [0, 100], [0, 200]];
  assert.deepEqual(assignSlots([[[100, 100]], [[100, 200]], [[100, 0]]], three), { perm: [1, 2, 0], cross: 0, moved: 3 });
});

test('power callouts: no two lines cross and every line runs right, away from the labels, through the sequence', () => {
  const { bird, model } = nodeBird('larus-gl-2');
  const N = numbers(model, { gates: loadVerification('larus-gl-2'), fit: [] });
  const flap = (t) => {
    const ph = 2 * Math.PI * N.flapHz * t;
    return { phase: ph, amp: N.ampDeg / DEG, twistL: N.twistAt(ph, -1), twistR: N.twistAt(ph, 1) };
  };
  const parts = powerParts(model);
  const sphere = sphereOf(bird, N.ampDeg / DEG, (o) => o.userData.subsystem === 'power' && !/^wire/.test(o.name));
  const cam = new THREE.PerspectiveCamera(30, 16 / 9, 0.01, 60000);
  const dur = 5.5;
  const { perm } = assignSlots(powerTracks({ bird, cam, sphere, parts, flap, dur }), POWER_LABELS);
  assert.equal(new Set(perm).size, parts.length, 'one slot per part');
  const fine = powerTracks({ bird, cam, sphere, parts, flap, dur, samples: 331 });
  let shown = 0;
  for (let j = 0; j < fine[0].length; j++) {
    const lines = fine.map((tr, k) => tr[j] && { k, seg: [tr[j], POWER_LABELS[perm[k]]] }).filter(Boolean);
    // every part right of the label column, so a label (drawn left of its slot) never sits on another part's line
    for (const { k, seg } of lines) assert.ok(seg[0][0] >= seg[1][0] + 40, `t=${((j / 330) * dur).toFixed(2)} s: ${parts[k].label} at x ${seg[0][0].toFixed(0)}, left of the labels`);
    shown = Math.max(shown, lines.length);
    for (let a = 0; a < lines.length; a++) for (let b = a + 1; b < lines.length; b++) {
      assert.ok(!segmentsCross(lines[a].seg, lines[b].seg), `t=${((j / 330) * dur).toFixed(2)} s: ${parts[lines[a].k].label} and ${parts[lines[b].k].label} cross`);
    }
  }
  assert.ok(shown >= 4, `only ${shown} callouts on screen`);
});

test('power tracks are the parts seen through the power camera: an independent orbit and projection agree', () => {
  const { bird, model } = nodeBird('larus-gl-2');
  const N = numbers(model, { gates: loadVerification('larus-gl-2'), fit: [] });
  const flap = (t) => ({ phase: 2 * Math.PI * N.flapHz * t, amp: N.ampDeg / DEG });
  const parts = powerParts(model);
  const sphere = sphereOf(bird, N.ampDeg / DEG, (o) => o.userData.subsystem === 'power' && !/^wire/.test(o.name));
  const dur = 5.5, samples = 12;
  const got = powerTracks({ bird, cam: new THREE.PerspectiveCamera(30, 16 / 9, 0.01, 60000), sphere, parts, flap, dur, samples });
  const c = POWER_CAM;
  const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2);
  const cam = new THREE.PerspectiveCamera(c.fov, 16 / 9, 0.01, 60000);
  const dist0 = sphere.r / (c.fill * Math.tan((c.fov * Math.PI) / 360)); // the sphere held at `fill` of the height
  let seen = 0;
  for (let j = 0; j < samples; j++) {
    const t = (j / (samples - 1)) * dur, u = t / dur;
    const az = (c.az0 + c.azSpan * ease(u)) * Math.PI / 180, el = (c.el0 + c.elSpan * u) * Math.PI / 180, d = dist0 * (1 + c.distSpan * ease(u));
    cam.position.set(sphere.c.x + d * Math.cos(el) * Math.cos(az), sphere.c.y + d * Math.sin(el), sphere.c.z + d * Math.cos(el) * Math.sin(az));
    cam.lookAt(sphere.c);
    cam.setViewOffset(1920, 1080, c.shift, 0, 1920, 1080);
    cam.updateMatrixWorld(true);
    bird.update(flap(t));
    bird.root.updateMatrixWorld(true);
    parts.forEach((p, k) => {
      const v = bird.root.getObjectByName(p.node).getWorldPosition(new THREE.Vector3()).project(cam);
      const want = [(v.x + 1) * 960, (1 - v.y) * 540];
      const on = v.z <= 1 && want[0] >= 0 && want[0] <= 1920 && want[1] >= 0 && want[1] <= 1080;
      if (!on) return assert.equal(got[k][j], null, `${p.label} at t=${t.toFixed(2)} is off screen`);
      seen++;
      assert.ok(got[k][j] && Math.hypot(got[k][j][0] - want[0], got[k][j][1] - want[1]) < 1e-6, `${p.label} at t=${t.toFixed(2)}: ${got[k][j]} vs ${want}`);
    });
  }
  assert.ok(seen >= 4 * samples, `${seen} points on screen`);
});
