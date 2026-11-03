// The couplers ride side by side on the crank pin, never in each other, in the crank web or in a rocker: the planes
// model.js gives them, and the bird as drawn over a whole crank turn. Bounds come from the design's own numbers (the
// crank web's thickness, the coupler-eye bushing's length in the catalogue, the rocker's horn boss), not from model.js.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { prepare } from '../../sims/avis-shared/model.js';
import { drawnBox, poseOf } from './drawn.mjs';

const { bird, design, model } = nodeBird('larus-gl-2');
const P = model.planes;
const eyePart = design.bom.parts.find((p) => /coupler eyes/.test(p.role));
const eyeB = model.parts[eyePart.id].specs.b1_mm / 1000; // flanged bushing: b1 is its whole length
const webT = design.params.linkage.crank.web_t_mm / 1000;
const span = (lo, hi) => ({ lo, hi });
// v22 on: 0.5 mm running clearance between the eyes and the web (SPEC Assembly clearance); 1 um for rounding
const apart = (a, b) => a.hi <= b.lo + 1e-6 || b.hi <= a.lo + 1e-6;

test('the drawn coupler eyes lie aft of the crank web and behind one another, never in each other', () => {
  const web = span(P.crankArm - webT / 2, P.crankArm + webT / 2);
  const R = span(P.couplerR - eyeB / 2, P.couplerR + eyeB / 2);
  const L = span(P.couplerL - eyeB / 2, P.couplerL + eyeB / 2);
  assert.ok(apart(web, R), `web ${web.lo} vs right eye ${R.hi}`);
  assert.ok(apart(R, L), `right eye ${R.lo} vs left eye ${L.hi}`);
  // the design's own crank pin (to 1 mm past the physics' left eye) carries both drawn eyes
  assert.ok(P.crankPinEnd <= L.lo + 1e-9);
});

test("the film draws the physics' own planes, and the crank pin to 1 mm past the left eye", () => {
  const phys = prepare(design, model.parts).planes;
  assert.equal(P.couplerR, phys.couplerR);
  assert.equal(P.couplerL, phys.couplerL);
  assert.ok(Math.abs(P.crankPinEnd - (phys.couplerL - eyeB / 2 - 0.001)) < 1e-12, "the pin is the design's length");
});

/** World x extent of a drawn node (rings and rockers turn about x, so this is the part's own x range). */
function xSpan(name) {
  const o = bird.root.getObjectByName(name);
  assert.ok(o, name);
  const b = new THREE.Box3().setFromObject(o);
  return span(b.min.x, b.max.x);
}
/** World x extent of what is drawn of physics solids (v24 on, the couplers' eyes and the crank are drawn from them). */
function drawnSpan(q, ...tags) {
  const b = new THREE.Box3();
  for (const t of tags) b.union(drawnBox(bird, t, q));
  assert.ok(!b.isEmpty(), tags.join(', '));
  return span(b.min.x, b.max.x);
}
const eyes = (tag, q) => ['crank', 'horn'].map((e) => drawnSpan(q, `coupler ${tag}/${e} eye`, `coupler ${tag}/${e} eye counterbore`));

test('over a whole crank turn the drawn couplers, crank web and rocker bosses and arms never share an x range', () => {
  for (let i = 0; i < 24; i++) {
    const phase = (i / 24) * 2 * Math.PI, q = poseOf(bird, phase, model.law.ampNominal);
    bird.update({ phase, amp: model.law.ampNominal });
    bird.root.updateMatrixWorld(true);
    const c = { R: [...eyes('R', q), xSpan('coupler_rod_R')], L: [...eyes('L', q), xSpan('coupler_rod_L')] };
    const web = drawnSpan(q, 'crank web/web');
    for (const a of c.R) for (const b of c.L) assert.ok(apart(a, b), `couplers meet at phase ${i}/24`);
    for (const a of [...c.R, ...c.L]) assert.ok(apart(a, web), `a coupler meets the crank web at phase ${i}/24`);
    for (const tag of ['R', 'L']) {
      for (const part of [`rocker_boss_${tag}`, `rocker_arm_${tag}`]) {
        const r = xSpan(part);
        for (const a of c[tag]) assert.ok(apart(a, r), `coupler ${tag} meets ${part} at phase ${i}/24`);
      }
    }
  }
});

test('the crank pin carries both eyes; the horn pin carries its boss and its eye', () => {
  bird.update({ phase: 0, amp: model.law.ampNominal });
  const pin = drawnSpan(poseOf(bird, 0, model.law.ampNominal), 'crank web/pin');
  assert.ok(pin.lo <= P.couplerL - eyeB / 2 && Math.abs(pin.hi - (P.crankArm - webT / 2)) < 1e-9);
  const r = design.params.linkage.rocker;
  for (const [tag, x] of [['R', P.couplerR], ['L', P.couplerL]]) {
    const hp = xSpan(`horn_pin_${tag}`), boss = xSpan(`rocker_boss_${tag}`);
    assert.ok(Math.abs(boss.hi - boss.lo - r.boss_len_mm / 1000) < 1e-9, 'boss length');
    assert.ok(hp.lo <= x - eyeB / 2 && hp.hi >= boss.hi - 1e-9, `horn pin ${tag}`);
  }
});
