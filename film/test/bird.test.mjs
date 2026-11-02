// Run: node --import ./film/lib/three-hooks.mjs --test film/test/
// The three.js bird: every BOM line is drawn, counted parts appear as many times as the BOM says, the gears and
// bearings match the BOM counts, and the pose leaves no NaN anywhere.
import test from 'node:test';
import assert from 'node:assert/strict';
import { nodeBird, posedNodes } from '../lib/bird-node.mjs';

const { bird, design, model } = nodeBird('larus-gl-2');

test('every BOM part and structure item is drawn', () => {
  const ids = new Set(bird.parts.map((p) => p.id).filter(Boolean));
  for (const p of design.bom.parts) assert.ok(ids.has(p.id), `${p.role} ${p.id}`);
  for (const s of design.bom.structure) assert.ok(ids.has(s.material_id), `${s.name} ${s.material_id}`);
});

test('gears, bearings and servos appear exactly as often as the BOM lists them', () => {
  const count = (id) => {
    let n = 0;
    bird.root.traverse((o) => {
      if (o.isMesh && o.userData.partId === id && /teeth|bearing|_case/.test(o.name)) n++;
    });
    return n;
  };
  const bom = (role) => design.bom.parts.filter((p) => p.role === role).reduce((a, p) => a + p.count, 0);
  const gearIds = design.bom.parts.filter((p) => p.role === 'gear');
  let gearsDrawn = 0;
  for (const id of new Set(gearIds.map((g) => g.id))) gearsDrawn += count(id);
  assert.equal(gearsDrawn, bom('gear'));
  const bearing = design.bom.parts.find((p) => p.role === 'bearing');
  assert.equal(count(bearing.id), bearing.count);
  const servoId = design.bom.parts.find((p) => /servo$/.test(p.role)).id;
  const servos = design.bom.parts.filter((p) => p.id === servoId).reduce((a, p) => a + p.count, 0);
  assert.equal(count(servoId), servos);
});

test('ribs: the BOM count, shared between the wings', () => {
  let n = 0;
  bird.root.traverse((o) => {
    if (/^rib_[LR]_\d+$/.test(o.name)) n++;
  });
  const listed = +design.bom.structure.find((s) => /^ribs/.test(s.name)).name.match(/(\d+)\s*off/)[1];
  assert.equal(n, listed);
});

test('a pose from the log leaves every matrix and vertex finite', () => {
  bird.update({ phase: 2.2, amp: model.law.ampMax, twistL: 0.3, twistR: -0.3, tailL: 0.2, tailR: -0.1 });
  bird.root.updateMatrixWorld(true);
  bird.root.traverse((o) => {
    for (const v of o.matrixWorld.elements) assert.ok(Number.isFinite(v), o.name);
  });
  for (const m of bird.deform) for (const v of m.geometry.attributes.position.array) assert.ok(Number.isFinite(v), m.name);
});

test('wing segment nodes are the GLB contract: wing_R_0..1 and wing_L_0..1, chained, the only wing_ nodes', () => {
  const names = [];
  bird.root.traverse((o) => names.push(o.name));
  assert.deepEqual(names.filter((n) => /^wing_/.test(n)).sort(), ['wing_L_0', 'wing_L_1', 'wing_R_0', 'wing_R_1']);
  assert.equal(bird.root.getObjectByName('wing_R_1').parent.name, 'wing_R_0');
  assert.equal(bird.root.name, 'body');
  assert.ok(posedNodes(bird).length >= 4);
});

test('the right wing tip rises with the stroke angle and the left mirrors it', () => {
  const tipY = (phase) => {
    const dp = bird.update({ phase, amp: model.law.ampNominal });
    bird.root.updateMatrixWorld(true);
    const r = bird.root.getObjectByName('rib_R_08').getWorldPosition(new bird.root.position.constructor());
    const l = bird.root.getObjectByName('rib_L_08').getWorldPosition(new bird.root.position.constructor());
    return { r, l, dp };
  };
  const top = tipY(0), bottom = tipY(Math.PI);
  assert.ok(top.r.y > bottom.r.y + 0.2);
  assert.ok(top.l.y > bottom.l.y + 0.2);
  assert.ok(top.r.z > 0 && top.l.z < 0);
});
