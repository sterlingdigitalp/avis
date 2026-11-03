// Designs without a machined linkage or an airframe (Falco and Columba, version 2) still build the older drive: carbon
// rockers and couplers, a crank disc, standoffs between the plates. Every BOM part drawn, nothing NaN over a stroke,
// and their couplers too ride clear of each other on the crank pin.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';

for (const name of ['falco-pf-2', 'columba-rp-2']) {
  const { bird, design, model } = nodeBird(name);
  test(`${name}: the older drive is drawn, every BOM part, and poses without NaN`, () => {
    assert.ok(!design.params?.linkage?.rocker && !design.structure?.airframe, 'a design with neither');
    const ids = new Set(bird.parts.map((p) => p.id).filter(Boolean));
    for (const p of design.bom.parts) assert.ok(ids.has(p.id), `${p.role} ${p.id}`);
    for (const n of ['rocker_R', 'rocker_L', 'coupler_R', 'coupler_L', 'crank_disc', 'shaft_crank', 'standoff_0', 'plate_aft']) assert.ok(bird.root.getObjectByName(n), n);
    for (let i = 0; i < 8; i++) {
      bird.update({ phase: (i / 8) * 2 * Math.PI, amp: model.law.ampNominal });
      bird.root.updateMatrixWorld(true);
      bird.root.traverse((o) => assert.ok(o.matrixWorld.elements.every(Number.isFinite), `${o.name} NaN at ${i}/8`));
    }
  });
  test(`${name}: the couplers' eyes do not share an x range`, () => {
    const span = (n) => new THREE.Box3().setFromObject(bird.root.getObjectByName(n));
    const a = span('coupler_eye_R_bot'), b = span('coupler_eye_L_bot');
    assert.ok(a.min.x >= b.max.x || b.min.x >= a.max.x, `${a.min.x}..${a.max.x} vs ${b.min.x}..${b.max.x}`);
  });
}
