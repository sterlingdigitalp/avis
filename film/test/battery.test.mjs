// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// The battery is drawn at the catalogue's printed size when the entry gives one (dimensions_text_mm, L x W x H; the
// layout may turn the pack to fit the pod, so the three sizes are compared as a set), and otherwise as the slab the
// layout derives from its mass (a 5:5:1 board at 2000 kg/m3, about 71 x 71 x 14 mm for the 143 g pack). The page's
// bundle keeps the size too.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { loadDesign, loadParts, bundleParts } from '../../sims/avis-shared/node-load.js';
import { prepareFilm } from '../../sims/avis-shared/model.js';
import { makeMaterials } from '../../sims/avis-shared/materials.js';
import { buildBird, roundedBox } from '../../sims/avis-shared/bird.js';

const { design } = loadDesign('larus-gl-2');
const battery = design.bom.parts.find((p) => p.role === 'battery');

/** The drawn pack's size in mm [x, y, z] with the battery's catalogue specs replaced by `specs`. */
function drawnPack(specs) {
  const parts = loadParts(design);
  parts[battery.id] = { ...parts[battery.id], specs };
  const bird = buildBird(prepareFilm(design, parts), makeMaterials({ textures: false }));
  const pack = bird.root.getObjectByName('battery_pack');
  pack.geometry.computeBoundingBox();
  return pack.geometry.boundingBox.getSize(new THREE.Vector3()).toArray().map((v) => v * 1000).sort((a, b) => b - a);
}
const near = (a, b, tol = 0.05) => a.every((v, i) => Math.abs(v - b[i]) < tol);

test('battery: the catalogue\'s L x W x H when it prints one', () => {
  const specs = { ...loadParts(design)[battery.id].specs, dimensions_text_mm: '74.7 x 33.5 x 25.4 (L x W x H)' };
  const got = drawnPack(specs);
  assert.ok(near(got, [74.7, 33.5, 25.4]), `drawn ${got.map((v) => v.toFixed(1))} mm`);
});

test('battery: without a printed size, the slab from its mass, as before', () => {
  const { dimensions_text_mm, ...specs } = loadParts(design)[battery.id].specs; // eslint-disable-line no-unused-vars
  const side = Math.cbrt(battery.unit_mass_g / 1000 / 2000 / 0.2) * 1000;
  const got = drawnPack(specs);
  assert.ok(near(got, [side, side, side * 0.2]), `drawn ${got.map((v) => v.toFixed(1))} mm, want ${side.toFixed(1)} x ${(side * 0.2).toFixed(1)} x ${side.toFixed(1)}`);
  assert.ok(Math.abs(side - 71) < 1, `the 143 g pack's slab is ${side.toFixed(1)} mm a side`);
});

test('the page bundle keeps every printed size, however long, and drops long notes', () => {
  const long = `${'107 x 33 x 22 '.repeat(8)}(L x W x H)`;
  const s90 = 'x'.repeat(90), s91 = 'x'.repeat(91);
  const b = bundleParts({ x: { specs: { dimensions_text_mm: long, dimensions_text: long, c_rating_note: 'short', mass_note: 'n', blurb: long, connector: 'XT60', a90: s90, a91: s91, n: 1234 } } });
  assert.equal(b.x.specs.connector, 'XT60', 'short text travels');
  assert.equal(b.x.specs.a90, s90, '90 characters travel');
  assert.equal(b.x.specs.a91, undefined, 'longer text stays in the catalogue');
  assert.equal(b.x.specs.n, 1234);
  assert.equal(b.x.specs.dimensions_text_mm, long);
  assert.equal(b.x.specs.dimensions_text, long);
  assert.equal(b.x.specs.c_rating_note, undefined);
  assert.equal(b.x.specs.blurb, undefined);
});

test('a rounded box is its size exactly, with corners of radius r round its extruded sides and flat faces between', () => {
  // [sx, sy, sz, r] in metres: the battery, a thin board, and one too narrow for the full radius
  for (const [sx, sy, sz, r] of [[0.0747, 0.0335, 0.0254, 0.003], [0.0343, 0.0173, 0.0045, 0.003], [0.03, 0.004, 0.01, 0.003]]) {
    const g = roundedBox(sx, sy, sz, r);
    g.computeBoundingBox();
    const size = g.boundingBox.getSize(new THREE.Vector3());
    assert.ok(near([size.x, size.y, size.z].map((v) => v * 1000), [sx, sy, sz].map((v) => v * 1000), 0.01), `${[sx, sy, sz]}: ${size.toArray()}`); // to 10 um, as below
    const t = Math.min(r, sz / 4), R = Math.min(r, sx / 2, sy / 2);
    const pos = g.attributes.position;
    const reach = { right: [Infinity, -Infinity], top: [Infinity, -Infinity] };
    for (let i = 0; i < pos.count; i++) {
      const x = Math.abs(pos.getX(i)), y = Math.abs(pos.getY(i)), z = Math.abs(pos.getZ(i));
      if (z > sz / 2 - t + 1e-7) continue; // the bevel rings and caps
      // the straight sides: every vertex on the rounded rectangle of corner radius R (to 30 um: the bevel offsets the
      // arc's polyline along its vertex bisectors, 1/cos(7.5 deg) of the bevel, not radially)
      const qx = Math.max(x - (sx / 2 - R), 0), qy = Math.max(y - (sy / 2 - R), 0);
      assert.ok(Math.abs(Math.hypot(qx, qy) - R) < 3e-5, `${[sx, sy, sz]}: vertex (${x}, ${y}) off the rounded outline`);
      const [px, py] = [pos.getX(i), pos.getY(i)];
      if (Math.abs(px - sx / 2) < 3e-5) reach.right = [Math.min(reach.right[0], py), Math.max(reach.right[1], py)];
      if (Math.abs(py - sy / 2) < 3e-5) reach.top = [Math.min(reach.top[0], px), Math.max(reach.top[1], px)];
    }
    // the flat faces run out to their corners, within one arc segment (0.4 mm at r = 3 mm)
    for (const [side, [lo, hi], half] of [['right', reach.right, sy / 2 - R], ['top', reach.top, sx / 2 - R]]) {
      assert.ok(lo <= -half + 4e-4 && hi >= half - 4e-4, `${[sx, sy, sz]}: the ${side} face spans ${lo}..${hi}, want +-${half}`);
    }
  }
});
