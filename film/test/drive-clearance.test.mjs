// The couplers ride side by side on the crank pin, never in each other, in the crank web or in a rocker: the planes
// model.js gives them, and the bird as drawn over a whole crank turn. Bounds come from the design's own numbers (the
// crank web's thickness, the coupler-eye bushing's length in the catalogue, the rocker's horn boss), not from model.js.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { filmCouplerPlanes, prepare } from '../../sims/avis-shared/model.js';

const { bird, design, model } = nodeBird('larus-gl-2');
const P = model.planes;
const eyePart = design.bom.parts.find((p) => /coupler eyes/.test(p.role));
const eyeB = model.parts[eyePart.id].specs.b1_mm / 1000; // flanged bushing: b1 is its whole length
const webT = design.params.linkage.crank.web_t_mm / 1000;
const span = (lo, hi) => ({ lo, hi });
// faces may touch (the design's 12 mm crank pin carries two 6 mm eyes with no running clearance); 1 um for rounding
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

test('the physics keeps its own planes: the film moves only what it draws', () => {
  const phys = prepare(design, model.parts).planes;
  assert.equal(phys.couplerR, design.wing.hinge[0] - 0.0045);
  assert.equal(phys.couplerL, design.wing.hinge[0] - 0.0095);
  const near = (a, b) => Math.abs(a - b) < 1e-12;
  assert.ok(!('crankPinEnd' in phys) && !('couplerEye' in phys));
  // v21's physics planes put two 6 mm eyes 1 mm into each other: the drawing moves the left one, nothing else
  assert.ok(phys.couplerR - phys.couplerL < eyeB);
  assert.ok(near(P.couplerR, phys.couplerR));
  assert.ok(near(P.couplerL, phys.couplerR - eyeB));
  assert.ok(Math.abs(P.crankPinEnd - (phys.couplerL - eyeB / 2 - 0.001)) < 1e-12, 'the pin is the design\'s length');
});

test('planes whose eyes already clear each other are drawn where the physics has them', () => {
  const planes = { crankArm: 0.0015, couplerR: -0.0045, couplerL: -0.0095 };
  const bare = { ...design, params: {}, bom: { ...design.bom, parts: design.bom.parts.filter((p) => !/^(bearing|bushing)/.test(p.role)) } };
  const c = filmCouplerPlanes(bare, model.parts, planes);
  assert.deepEqual([c.couplerR, c.couplerL, c.couplerEye], [-0.0045, -0.0095, 0.003]);
  assert.ok(Math.abs(c.crankPinEnd - (-0.0095 - 0.0015 - 0.001)) < 1e-12);
});

/** World x extent of a drawn node (rings and rockers turn about x, so this is the part's own x range). */
function xSpan(name) {
  const o = bird.root.getObjectByName(name);
  assert.ok(o, name);
  const b = new THREE.Box3().setFromObject(o);
  return span(b.min.x, b.max.x);
}

test('over a whole crank turn the drawn couplers, crank web and rocker bosses and arms never share an x range', () => {
  for (let i = 0; i < 24; i++) {
    bird.update({ phase: (i / 24) * 2 * Math.PI, amp: model.law.ampNominal });
    bird.root.updateMatrixWorld(true);
    const c = { R: ['coupler_eye_R_bot', 'coupler_eye_R_top', 'coupler_rod_R'].map(xSpan), L: ['coupler_eye_L_bot', 'coupler_eye_L_top', 'coupler_rod_L'].map(xSpan) };
    const web = xSpan('crank_web');
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
  const pin = xSpan('crank_pin');
  assert.ok(pin.lo <= P.couplerL - eyeB / 2 && Math.abs(pin.hi - (P.crankArm - webT / 2)) < 1e-9);
  const r = design.params.linkage.rocker;
  for (const [tag, x] of [['R', P.couplerR], ['L', P.couplerL]]) {
    const hp = xSpan(`horn_pin_${tag}`), boss = xSpan(`rocker_boss_${tag}`);
    assert.ok(Math.abs(boss.hi - boss.lo - r.boss_len_mm / 1000) < 1e-9, 'boss length');
    assert.ok(hp.lo <= x - eyeB / 2 && hp.hi >= boss.hi - 1e-9, `horn pin ${tag}`);
  }
});
