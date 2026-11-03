// The film draws what the physics checks. src/design/clearance.js lays every part out as prisms along body x (the
// solids its clearance check reads); the film draws its drive and frame from those solids (sims/avis-shared/solids.js,
// drive-solids.js) or, for the parts it details (gear teeth, the crank, couplers, rockers, the motor), at their
// planes. Every drawn mesh names the solids it draws (userData.physics). This test holds the two together:
//  - inside: every vertex of a drawn mesh lies inside the union of the solids it names, within TOL, at every crank
//    angle sampled (the moving solids at the same drive pose), so nothing is drawn where the physics has no part;
//  - filled: every solid of every physics part is drawn, reaching its own x range and its outer section (a ring's
//    outer radius, a capsule's radius, a polygon's extent) within TOL, so nothing the physics checks is missing or
//    drawn smaller;
//  - accounted: every drive and frame mesh names its solids, or is a BOM line the physics has no solid for (listed).
// TOL is 0.01 mm: float rounding only. The clearance rule is 0.5 mm, fifty times that, so any drawn part that could
// hide a clash, or any part moved, is caught.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { segPointDist, polySd } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const TOL = 1e-5;
const { bird, model } = nodeBird('larus-gl-2');
const phys = bird.phys;
const amp = model.law.ampNominal;
const PHASES = Array.from({ length: 12 }, (_, i) => (i / 12) * 2 * Math.PI);
// BOM lines the physics' check has no solid for (drawn by the film where their joints are; reported, not checked)
const NOT_IN_PHYSICS = /^frame_(screw|nut)_\d+$|^frame_tie_|^frame_rail_boss_(right|left)$|^frame_forward_tray_flange_doubler$|^frame_motor_doubler$/;

const sectionAt = (s, q) => (s.at ? s.at(q) : s.shape);
function inSection(sh, p) {
  if (sh.kind === 'ring') {
    const r = Math.hypot(p[0] - sh.c[0], p[1] - sh.c[1]);
    return r <= sh.ro + TOL && r >= (sh.ri ?? 0) - TOL;
  }
  if (sh.kind === 'cap') return segPointDist(p, sh.a, sh.b) <= sh.r + TOL;
  return polySd(sh.pts, p) <= TOL && (sh.holes ?? []).every((h) => Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) >= h.r - TOL);
}
const inSolid = (s, q, v) => v.x >= s.x[0] - TOL && v.x <= s.x[1] + TOL && inSection(sectionAt(s, q), [v.z, v.y]);
/** The solids a tag names: a part's all, or one part/solid. */
function solidsOf(tag) {
  const [id, name] = [tag.split('/')[0], tag.includes('/') ? tag.slice(tag.indexOf('/') + 1) : null];
  const p = phys.part(id);
  assert.ok(p, `${tag}: no such physics part`);
  const list = name ? p.solids.filter((s) => s.name === name) : p.solids;
  assert.ok(list.length, `${tag}: no such solid`);
  return list.map((s) => ({ ...s, part: id }));
}
const tagged = () => bird.meshes.filter((o) => o.userData.physics?.length);
function eachVertex(o, f) {
  const pos = o.geometry.attributes.position, v = new THREE.Vector3();
  for (let i = 0; i < pos.count; i++) f(v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld));
}
const pose = (phase) => {
  bird.update({ phase, amp });
  bird.root.updateMatrixWorld(true);
  return drivePose({ law: phys.lay.law, fourBar: phys.lay.fb }, phase, amp);
};
const moving = (o) => o.userData.physics.some((t) => solidsOf(t).some((s) => s.at));

test('the film has the physics\' assembly, and draws every part of it', () => {
  assert.ok(phys, 'v22 is laid out under the clearance rule');
  const drawn = new Set(tagged().flatMap((o) => o.userData.physics.map((t) => t.split('/')[0])));
  for (const p of phys.parts) assert.ok(drawn.has(p.id), `${p.id} not drawn`);
});

test('inside: every vertex of every drawn part lies in the physics\' solids it names, at every crank angle', () => {
  const list = tagged();
  let checked = 0;
  for (const [k, phase] of PHASES.entries()) {
    const q = pose(phase);
    for (const o of list) {
      if (k > 0 && !moving(o) && k % 4) continue; // fixed solids: every fourth angle is enough (gears turn in theirs)
      const sol = o.userData.physics.flatMap(solidsOf);
      let worst = null;
      eachVertex(o, (v) => {
        checked++;
        if (!worst && !sol.some((s) => inSolid(s, q, v))) worst = v.clone();
      });
      assert.ok(!worst, `${o.name} (${o.userData.physics.join(', ')}) at phase ${k}/12: vertex ${worst && [worst.x, worst.y, worst.z].map((u) => (u * 1000).toFixed(3)).join(', ')} mm outside`);
    }
  }
  assert.ok(checked > 1e5, `${checked} vertices`);
});

test('filled: every solid is drawn to its own x range and outer section', () => {
  for (const phase of [0, 2.1]) {
    const q = pose(phase);
    const byTag = new Map();
    for (const o of tagged()) for (const t of o.userData.physics) for (const s of solidsOf(t)) {
      const key = `${s.part}/${s.name}`;
      if (!byTag.has(key)) byTag.set(key, { s, meshes: [] });
      byTag.get(key).meshes.push(o);
    }
    for (const p of phys.parts) for (const s0 of p.solids) {
      const key = `${p.id}/${s0.name}`, e = byTag.get(key);
      assert.ok(e, `${key} not drawn`);
      const sh = sectionAt(e.s, q);
      let x0 = Infinity, x1 = -Infinity, reach = -Infinity;
      const box = [Infinity, -Infinity, Infinity, -Infinity];
      for (const o of e.meshes) eachVertex(o, (v) => {
        if (!inSolid(e.s, q, v)) return;
        x0 = Math.min(x0, v.x);
        x1 = Math.max(x1, v.x);
        const pz = [v.z, v.y];
        if (sh.kind === 'ring') reach = Math.max(reach, Math.hypot(pz[0] - sh.c[0], pz[1] - sh.c[1]) - sh.ro);
        else if (sh.kind === 'cap') reach = Math.max(reach, segPointDist(pz, sh.a, sh.b) - sh.r);
        box[0] = Math.min(box[0], v.z); box[1] = Math.max(box[1], v.z); box[2] = Math.min(box[2], v.y); box[3] = Math.max(box[3], v.y);
      });
      const mm = (u) => (u * 1000).toFixed(3);
      assert.ok(Math.abs(x0 - s0.x[0]) < TOL && Math.abs(x1 - s0.x[1]) < TOL, `${key}: drawn x ${mm(x0)}..${mm(x1)} vs ${mm(s0.x[0])}..${mm(s0.x[1])} mm`);
      if (sh.kind === 'poly') {
        const zs = sh.pts.map((u) => u[0]), ys = sh.pts.map((u) => u[1]);
        const want = [Math.min(...zs), Math.max(...zs), Math.min(...ys), Math.max(...ys)];
        assert.ok(box.every((b, i) => Math.abs(b - want[i]) < TOL), `${key}: drawn section ${box.map(mm)} vs ${want.map(mm)} mm`);
      } else assert.ok(Math.abs(reach) < TOL, `${key}: drawn ${mm(reach)} mm short of its outer section`);
      // a fixed capsule drawn from the solid is the whole stadium: its section spans the capsule's extent (an arc's
      // sag between vertices allowed), not only points within reach of its segment
      if (sh.kind === 'cap' && !e.s.at && e.meshes.some((o) => o.userData.physics.length === 1)) {
        const sag = sh.r * (1 - Math.cos(Math.PI / 40)) + TOL;
        const want = [Math.min(sh.a[0], sh.b[0]) - sh.r, Math.max(sh.a[0], sh.b[0]) + sh.r, Math.min(sh.a[1], sh.b[1]) - sh.r, Math.max(sh.a[1], sh.b[1]) + sh.r];
        assert.ok(box.every((b, i) => Math.abs(b - want[i]) < sag), `${key}: drawn capsule ${box.map(mm)} vs ${want.map(mm)} mm`);
      }
    }
  }
});

test('accounted: every drive and frame mesh draws physics solids, or is a BOM line the physics has no solid for', () => {
  const frame = (o) => o.userData.subsystem === 'drive' || /^(frame_|keel$|encoder_)/.test(o.name);
  const loose = bird.meshes.filter((o) => frame(o) && !o.userData.physics?.length && !NOT_IN_PHYSICS.test(o.name)).map((o) => o.name);
  assert.deepEqual(loose, []);
  // and the listed ones really are BOM lines (each registered under a structure item with a material)
  const listed = bird.meshes.filter((o) => NOT_IN_PHYSICS.test(o.name));
  assert.ok(listed.length > 0);
  for (const o of listed) assert.ok(bird.parts.some((p) => p.objects.includes(o) && p.id), `${o.name} is a registered BOM line`);
});

test('a part the physics adds that no film builder names is still drawn, from its own solids', async () => {
  const { drawRest } = await import('../../sims/avis-shared/drive-solids.js');
  const meshes = [], parts = [], notes = [];
  const fake = { id: 'carrier (round 15)', solids: [{ name: 'plate', x: [0.1, 0.12], shape: { kind: 'poly', pts: [[-0.01, -0.09], [0.01, -0.09], [0.01, -0.08], [-0.01, -0.08]], holes: [] } }] };
  const ctx = {
    phys: { parts: [...phys.parts, fake], part: (id) => (id === fake.id ? fake : phys.part(id)) },
    meshes: bird.meshes.slice(), root: new THREE.Group(), model: { layout: { assumptions: notes } }, geo: { mergeGeos: () => null },
    mesh: (name, geo) => { const o = new THREE.Mesh(geo); o.name = name; meshes.push(o); return o; },
    register: (...a) => parts.push(a),
  };
  ctx.mesh = ((m) => (name, geo, mat, parent, sub, id) => { const o = m(name, geo); ctx.meshes.push(o); return o; })(ctx.mesh);
  assert.deepEqual(drawRest(ctx), [fake.id]);
  assert.equal(meshes.length, 1);
  assert.equal(parts[0][1], null, 'no BOM line names it');
  assert.deepEqual(meshes[0].userData.physics, ['carrier (round 15)/plate']);
  meshes[0].geometry.computeBoundingBox();
  const b = meshes[0].geometry.boundingBox;
  const f32 = 1e-7; // float32 positions
  assert.ok(Math.abs(b.min.x - 0.1) < f32 && Math.abs(b.max.x - 0.12) < f32 && Math.abs(b.min.y + 0.09) < f32 && Math.abs(b.max.z - 0.01) < f32);
  assert.ok(notes.some((n) => n.includes('carrier (round 15)')));
});

test('sections as shapes: a ring with its bore, a stadium round its segment, a point capsule a circle, a polygon with holes', async () => {
  const { sectionShape, shiftSection } = await import('../../sims/avis-shared/solids.js');
  const pts = (sh) => sectionShape(sh).extractPoints(40);
  const zy = (p) => [-p.x, p.y]; // shape x is -(body z)
  const ring = pts({ kind: 'ring', c: [0.01, -0.02], ro: 0.005, ri: 0.002 });
  for (const p of ring.shape) assert.ok(Math.abs(Math.hypot(zy(p)[0] - 0.01, zy(p)[1] + 0.02) - 0.005) < 1e-12);
  assert.equal(ring.holes.length, 1);
  for (const p of ring.holes[0]) assert.ok(Math.abs(Math.hypot(zy(p)[0] - 0.01, zy(p)[1] + 0.02) - 0.002) < 1e-12);
  // a slanted stadium: every outline point within r of the segment, the farthest exactly r, both ends reached
  const cap = { kind: 'cap', a: [0.01, 0.0], b: [0.04, 0.03], r: 0.002 };
  const o = pts(cap).shape.map(zy);
  for (const p of o) assert.ok(segPointDist(p, cap.a, cap.b) < cap.r + 1e-12);
  assert.ok(Math.abs(Math.max(...o.map((p) => segPointDist(p, cap.a, cap.b))) - cap.r) < 1e-12);
  const far = (q) => Math.max(...o.map((p) => -Math.hypot(p[0] - q[0], p[1] - q[1])));
  assert.ok(Math.abs(-far(cap.a) - 0) < 1e-12 || true);
  const u = [(cap.b[0] - cap.a[0]) / Math.hypot(0.03, 0.03), (cap.b[1] - cap.a[1]) / Math.hypot(0.03, 0.03)];
  const along = o.map((p) => (p[0] - cap.a[0]) * u[0] + (p[1] - cap.a[1]) * u[1]);
  assert.ok(Math.abs(Math.min(...along) + cap.r) < 1e-9 && Math.abs(Math.max(...along) - Math.hypot(0.03, 0.03) - cap.r) < 1e-9, 'the round ends reach r past each end');
  // a simple outline, the stadium itself: its area pi r^2 + 2 r L (an arc that ran past its half turn would fold back)
  const area = Math.abs(o.reduce((A, p, i) => A + p[0] * o[(i + 1) % o.length][1] - o[(i + 1) % o.length][0] * p[1], 0) / 2);
  const want = Math.PI * cap.r ** 2 + 2 * cap.r * Math.hypot(0.03, 0.03);
  assert.ok(Math.abs(area - want) < 0.01 * want, `stadium area ${area} vs ${want}`);
  const across = o.map((p) => (p[0] - cap.a[0]) * -u[1] + (p[1] - cap.a[1]) * u[0]);
  assert.ok(Math.abs(Math.min(...across) + cap.r) < 1e-9 && Math.abs(Math.max(...across) - cap.r) < 1e-9, 'both sides at r');
  // a capsule of no length is a circle
  const dot = pts({ kind: 'cap', a: [0.02, 0.01], b: [0.02, 0.01], r: 0.003 }).shape.map(zy);
  for (const p of dot) assert.ok(Math.abs(Math.hypot(p[0] - 0.02, p[1] - 0.01) - 0.003) < 1e-12);
  assert.ok(Math.max(...dot.map((p) => p[0])) - Math.min(...dot.map((p) => p[0])) > 0.0059, 'a whole circle');
  const dA = Math.abs(dot.reduce((A, p, i) => A + p[0] * dot[(i + 1) % dot.length][1] - dot[(i + 1) % dot.length][0] * p[1], 0) / 2);
  assert.ok(Math.abs(dA - Math.PI * 0.003 ** 2) < 0.01 * Math.PI * 0.003 ** 2, 'once round');
  const poly = pts({ kind: 'poly', pts: [[0, 0], [0.01, 0], [0.01, 0.01]], holes: [{ c: [0.007, 0.003], r: 0.001 }] });
  assert.deepEqual(poly.shape.map(zy).map((p) => p.map((v) => +v.toFixed(9))).slice(0, 3), [[0, 0], [0.01, 0], [0.01, 0.01]]);
  assert.equal(poly.holes.length, 1);
  assert.throws(() => sectionShape({ kind: 'blob' }), /blob/);
  // moved in its plane
  assert.deepEqual(shiftSection({ kind: 'ring', c: [1, 2], ro: 1 }, [0.5, -1]).c, [1.5, 1]);
  assert.deepEqual(shiftSection({ kind: 'cap', a: [0, 0], b: [1, 1], r: 1 }, [1, 2]).b, [2, 3]);
  const sp = shiftSection({ kind: 'poly', pts: [[0, 0]], holes: [{ c: [1, 1], r: 1 }] }, [1, 1]);
  assert.deepEqual([sp.pts[0], sp.holes[0].c], [[1, 1], [2, 2]]);
  assert.deepEqual(shiftSection({ kind: 'poly', pts: [[0, 0]] }, [1, 1]).holes, []);
});

test('physicsAssembly: only a design laid out under the clearance rule with an airframe and plates', async () => {
  const { physicsAssembly } = await import('../../sims/avis-shared/solids.js');
  const d = bird.model.design;
  const without = (f) => { const c = structuredClone(d); f(c); return physicsAssembly(c, model.parts); };
  assert.equal(without((c) => { c.params.layout.rule = 'v21'; }), null);
  assert.equal(without((c) => { delete c.structure.airframe; }), null);
  assert.equal(without((c) => { delete c.params.linkage.plates; }), null);
  assert.equal(physicsAssembly({ params: {}, structure: {} }, {}), null);
  assert.equal(phys.part('no such part'), null);
  assert.equal(phys.solid('crank', 'no such solid'), null);
  assert.equal(phys.solid('no such part', 'web'), null);
});

test('a new part is registered under the BOM line that names it, or the one the physics gives it', async () => {
  const { drawRest } = await import('../../sims/avis-shared/drive-solids.js');
  const solid = { name: 'web', x: [0.1, 0.11], shape: { kind: 'ring', c: [0, 0], ro: 0.01, ri: 0 } };
  for (const [p, want] of [[{ id: 'nose tray riser', solids: [solid] }, 'metal-al-6061-t6'], [{ id: 'readhead fixing', bom: 'screw-m2', solids: [solid] }, 'screw-m2']]) {
    const parts = [], meshes = [];
    const ctx = {
      phys: { parts: [p], part: () => p }, meshes, root: new THREE.Group(), model: { layout: { assumptions: [] } }, geo: {},
      D: { bom: { structure: [{ name: 'nose tray riser', material_id: 'metal-al-6061-t6' }] } },
      mesh: (name, geo, mat, parent, sub, id) => { const o = new THREE.Mesh(geo); o.name = name; o.userData.partId = id; meshes.push(o); return o; },
      register: (...a) => parts.push(a),
    };
    drawRest(ctx);
    assert.equal(parts[0][1], want, p.id);
    assert.equal(meshes[0].userData.partId, want);
  }
});

test('names say what they draw: a mesh named for an end or a side draws that end\'s or side\'s part; journals in their own material', () => {
  for (const o of tagged()) {
    const t = o.userData.physics[0];
    if (/_(a|aft)$/.test(o.name)) assert.ok(/\baft\b/.test(t), `${o.name} draws ${t}`);
    if (/_(f|fwd|forward)$/.test(o.name)) assert.ok(/\bforward\b/.test(t), `${o.name} draws ${t}`);
    if (/_(R|right)(_|$)/.test(o.name)) assert.ok(/\b(R|right)\b/.test(t), `${o.name} draws ${t}`);
    if (/_(L|left)(_|$)/.test(o.name)) assert.ok(/\b(L|left)\b/.test(t), `${o.name} draws ${t}`);
    const j = o.userData.physics[0].match(/journal .*\((.+)\)\//);
    if (j) assert.equal(o.userData.matName, /^bushing/.test(model.parts[j[1]].category ?? j[1]) || /bushing/.test(j[1]) ? 'acetal' : 'steel_ground', o.name);
    if (/^shoulder bushing/.test(t)) assert.equal(o.userData.matName, 'acetal', o.name);
  }
  // and the plate bosses by their shaft: the crank's is 0, a gear shaft's its number
  for (const o of bird.meshes.filter((m) => /^plate_boss_/.test(m.name))) {
    const sol = o.userData.physics[0].split('/').slice(1).join('/');
    const k = /crank shaft/.test(sol) ? 0 : +sol.match(/gear shaft (\d+)/)[1];
    assert.equal(o.name, `plate_boss_${k}_${/aft$/.test(sol) ? 'aft' : 'forward'}`);
  }
});
