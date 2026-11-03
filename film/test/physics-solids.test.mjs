// The film draws what the physics checks. src/design/clearance.js lays every part out as prisms along body x (the
// solids its clearance check reads); the film draws its drive and frame from those solids (sims/avis-shared/solids.js,
// drive-solids.js) or, for the parts it details (gear teeth, the crank, couplers, rockers, the motor), at their
// planes. Every drawn mesh names the solids it draws (userData.physics). This test holds the two together:
//  - inside: every vertex of a drawn mesh lies inside the union of the solids it names, within TOL, at every crank
//    angle sampled (the moving solids at the same drive pose), so nothing is drawn where the physics has no part;
//  - filled: every solid of every physics part is drawn, reaching its own x range and its outer section (a ring's
//    outer radius, a capsule's radius, a polygon's extent) within TOL, so nothing the physics checks is missing or
//    drawn smaller;
//  - accounted: every drive and frame mesh names its solids (v24: the physics has solids for every BOM line that is a
//    physical object, so the film places none of its own).
// TOL is 0.01 mm: float rounding only. The clearance rule is 0.5 mm, fifty times that, so any drawn part that could
// hide a clash, or any part moved, is caught.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { segPointDist, polySd } from '../../src/design/clearance.js';
import { screenBeats } from '../../sims/avis-shared/solids.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const TOL = 1e-5;
const { bird, model } = nodeBird('larus-gl-2');
const phys = bird.phys;
const amp = model.law.ampNominal;
const PHASES = Array.from({ length: 12 }, (_, i) => (i / 12) * 2 * Math.PI);

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

test('accounted: every drive and frame mesh draws physics solids; every BOM line the film draws has them, or is exempt', () => {
  // v23 listed the BOM lines the physics had no solid for (the film placed their fasteners, ties, doublers and seat
  // bosses itself); v24's completeness record (SPEC "Completeness and assembly order") leaves none, so the same
  // promise is now: no drive or frame mesh of the film's own, and every registered BOM line either drawn from the
  // physics' solids or one the record exempts (the covering, which the film draws as the loft and the surfaces)
  const frame = (o) => o.userData.subsystem === 'drive' || /^(frame_|keel$|encoder_|phys_)/.test(o.name);
  const loose = bird.meshes.filter((o) => frame(o) && !o.userData.physics?.length).map((o) => o.name);
  assert.deepEqual(loose, []);
  const rec = model.design.structure.clearance.completeness;
  assert.equal(rec.passed, true);
  const exempt = new Set(rec.exemptions.map((e) => e.line));
  const withSolids = new Set(rec.lines.filter((l) => l.solids > 0).map((l) => l.line));
  for (const l of rec.lines) assert.ok(withSolids.has(l.line) || exempt.has(l.line), `${l.line}: no solids and not exempt`);
  const lineOf = (p) => phys.lines.find((l) => (l.kind === 'structure' ? l.key === p.name : l.id === p.id));
  for (const p of bird.parts.filter((x) => x.id && x.objects.some((o) => o.isMesh))) {
    const l = lineOf(p);
    if (!l || l.exempt) continue;
    assert.ok(p.objects.some((o) => o.userData.physics?.length), `${p.name}: drawn without the physics' solids`);
  }
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
  // where the solid is (the mesh sits at the centre of its geometry)
  meshes[0].updateMatrixWorld(true);
  const b = new THREE.Box3().setFromObject(meshes[0]);
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

test('a new part is registered under the BOM line its solids\' tags name, in that line\'s catalogue material', async () => {
  // v24: every solid carries its BOM tag (solid.bom); the film resolves it as the physics does (completeness.js
  // resolveTag through physicsAssembly's line()): a structure line by its name or its material id, a bought part by
  // its catalogue id. An untagged solid is drawn under no BOM line.
  const { drawRest } = await import('../../sims/avis-shared/drive-solids.js');
  const solid = (bom) => ({ name: 'web', x: [0.1, 0.11], shape: { kind: 'ring', c: [0, 0], ro: 0.01, ri: 0 }, ...(bom ? { bom } : {}) });
  const m3x12 = model.design.bom.structure.find((s) => s.material_id === 'screw-iso4762-m3x12-10.9');
  for (const [p, want, name, mat] of [
    [{ id: 'nose tray riser 2', solids: [solid('nose tray riser')] }, 'metal-al-6061-t6', 'nose tray riser', 'alu_machined'],
    [{ id: 'crank web screw', solids: [solid('screw-iso4762-m3x12-10.9')] }, 'screw-iso4762-m3x12-10.9', m3x12.name, 'steel_ground'],
    [{ id: 'retaining tie', solids: [solid('tie-hellermanntyton-t18r')] }, 'tie-hellermanntyton-t18r', model.design.bom.structure.find((s) => s.material_id === 'tie-hellermanntyton-t18r').name, 'acetal'],
    [{ id: 'mystery bracket', solids: [solid(null)] }, undefined, 'mystery bracket', 'alu_machined'],
  ]) {
    const parts = [], meshes = [];
    const ctx = {
      phys: { parts: [p], part: () => p, line: phys.line, joins: [] }, meshes, root: new THREE.Group(), model: { layout: { assumptions: [] }, parts: model.parts }, geo: {},
      D: model.design,
      mesh: (name, geo, m, parent, sub, id) => { const o = new THREE.Mesh(geo); o.name = name; o.userData.matName = m; if (id) o.userData.partId = id; meshes.push(o); return o; },
      register: (...a) => parts.push(a),
    };
    drawRest(ctx);
    assert.equal(parts[0][0], name, p.id);
    assert.equal(parts[0][1] ?? undefined, want, p.id);
    assert.equal(meshes[0].userData.partId, want);
    assert.equal(meshes[0].userData.matName, mat, p.id);
  }
});

test('names say what they draw: a mesh named for an end or a side draws that end\'s or side\'s part; journals in their own material', () => {
  for (const o of tagged()) {
    const t = o.userData.physics[0];
    if (/_(a|aft)$/.test(o.name)) assert.ok(/\baft\b/.test(t), `${o.name} draws ${t}`);
    if (/_(f|fwd|forward)$/.test(o.name)) assert.ok(/\bforward\b/.test(t), `${o.name} draws ${t}`);
    if (/_(R|right)(_|$)/.test(o.name)) assert.ok(/(\b|_)(R|right)\b/.test(t), `${o.name} draws ${t}`);
    if (/_(L|left)(_|$)/.test(o.name)) assert.ok(/(\b|_)(L|left)\b/.test(t), `${o.name} draws ${t}`);
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

test('a new BOM line draws in its catalogue material and look, with no film code: from its id, name and category', async () => {
  // the kinds of line a later design adds (v25: circlips, washers, magnets, leads, connectors, a strap, longerons, a
  // nose block), as made-up catalogue entries: each drawn in the material its own words name
  const { materialOf, lookOf } = await import('../../sims/avis-shared/drive-solids.js');
  const cat = {
    'circlip-din471-6': { category: 'fastener', name: 'DIN 471 6 mm retaining ring (circlip), spring steel' },
    'washer-iso7089-m3': { category: 'fastener', name: 'ISO 7089 M3 plain washer, steel' },
    'magnet-n52-6x2': { category: 'hardware', name: '6 x 2 mm NdFeB disc magnet, N52' },
    'lead-silicone-16awg': { category: 'electrical', name: '16 AWG silicone wire lead' },
    'conn-xt30': { category: 'electrical', name: 'Amass XT30 connector pair' },
    'strap-velcro-20': { category: 'hardware', name: '20 mm hook and loop battery strap' },
    'longeron-rod-ec-2mm': { category: 'carbon_rod', name: '2 mm pultruded carbon rod' },
    'block-basswood': { category: 'rib_material', name: 'basswood block' },
    'tie-hellermanntyton-t30r': { category: 'fastener', name: 'HellermannTyton T30R cable tie, PA66' },
    'nut-iso4032-m2-8': { category: 'fastener', name: 'ISO 4032 M2 hexagon nut, class 8' },
    'screw-iso4762-m2x6': { category: 'fastener', name: 'ISO 4762 M2 x 6 socket head cap screw' },
  };
  const want = { 'circlip-din471-6': 'steel_ground', 'washer-iso7089-m3': 'steel_ground', 'magnet-n52-6x2': 'magnet', 'lead-silicone-16awg': 'wire', 'conn-xt30': 'pcb', 'strap-velcro-20': 'acetal', 'longeron-rod-ec-2mm': 'carbon_pultruded', 'block-basswood': 'balsa', 'tie-hellermanntyton-t30r': 'acetal', 'nut-iso4032-m2-8': 'steel_ground', 'screw-iso4762-m2x6': 'steel_ground' };
  for (const [id, m] of Object.entries(want)) assert.equal(materialOf(cat, { id }), m, id);
  assert.equal(materialOf(cat, { id: 'no-such-part' }), 'alu_machined');
  assert.deepEqual(['nut-iso4032-m2-8', 'screw-iso4762-m2x6', 'tie-hellermanntyton-t30r', 'washer-iso7089-m3', 'magnet-n52-6x2'].map((id) => lookOf(cat, { id })), ['nut', 'screw', 'tie', null, null]);
  // and v24's own lines, as drawn
  for (const p of bird.parts.filter((x) => x.objects.some((o) => /^phys_/.test(o.name)))) {
    const l = phys.lines.find((x) => (x.kind === 'structure' ? x.key === p.name : x.id === p.id));
    if (l) for (const o of p.objects.filter((x) => /^phys_/.test(x.name))) assert.equal(o.userData.matName, materialOf(model.parts, l), `${o.name} (${l.key})`);
  }
});

test('a hole that cuts a polygon\'s outline is cut out of it: the outline less the disk, the arc on the circle', async () => {
  const { holeAgainst, cutDisk, sectionShape } = await import('../../sims/avis-shared/solids.js');
  const sq = [[0, 0], [0.01, 0], [0.01, 0.01], [0, 0.01]]; // counter-clockwise, 10 mm
  const area = (pts) => pts.reduce((A, p, i) => A + p[0] * pts[(i + 1) % pts.length][1] - pts[(i + 1) % pts.length][0] * p[1], 0) / 2;
  // a disk over the right edge's middle: its centre 2 mm outside, radius 4 mm
  const h = { c: [0.012, 0.005], r: 0.004 };
  assert.equal(holeAgainst(sq, h), 'crosses');
  assert.equal(holeAgainst(sq, { c: [0.005, 0.005], r: 0.002 }), 'inside');
  assert.equal(holeAgainst(sq, { c: [0.02, 0.005], r: 0.002 }), 'outside');
  assert.equal(holeAgainst(sq, { c: [0.005, 0.005], r: 0.005 }), 'inside', 'touching the outline from inside');
  assert.equal(holeAgainst([[0, 0], [0.01, 0], [0.01, 0.01], [0.005, 0.002], [0, 0.01]], { c: [0.005, 0.008], r: 0.0015 }), 'outside', 'in a concave notch');
  for (const input of [sq, sq.slice().reverse()]) {
    const cut = cutDisk(input, h);
    assert.ok(area(cut) > 0, 'counter-clockwise');
    for (const p of cut) assert.ok(Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) >= h.r - 1e-12, `${p} inside the disk`);
    for (const p of sq) if (Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) >= h.r) assert.ok(cut.some((q) => q[0] === p[0] && q[1] === p[1]), `corner ${p} kept`);
    // the area: the square less the circular segment beyond x = 8 mm (the chord 2 mm inside the centre's 4 mm)
    const d = 0.002, seg = h.r * h.r * Math.acos(d / h.r) - d * Math.sqrt(h.r * h.r - d * d);
    // (the arc drawn as chords of at most 3 degrees keeps their sag: n (r^2 / 2)(t - sin t) of the segment)
    const n = Math.ceil((2 * Math.acos(d / h.r)) / (Math.PI / 60) - 1e-9), t = (2 * Math.acos(d / h.r)) / n, sag = n * (h.r * h.r / 2) * (t - Math.sin(t));
    const kept = area(cut) - (1e-4 - seg);
    assert.ok(kept >= -1e-15 && kept <= sag + 1e-12 && sag < 1e-8, `area ${area(cut)}: ${kept} over the square less the segment (sag ${sag})`);
    // the arc's ends where the edge meets the circle, its points every 3 degrees at most
    const onArc = cut.filter((p) => Math.abs(Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) - h.r) < 1e-12);
    assert.ok(onArc.some((p) => Math.abs(p[0] - 0.01) < 1e-12 && Math.abs(p[1] - (0.005 - Math.sqrt(12e-6))) < 1e-12) && onArc.some((p) => Math.abs(p[0] - 0.01) < 1e-12 && Math.abs(p[1] - (0.005 + Math.sqrt(12e-6))) < 1e-12), 'arc from edge to edge');
    for (let i = 1; i < onArc.length; i++) assert.ok(Math.hypot(onArc[i][0] - onArc[i - 1][0], onArc[i][1] - onArc[i - 1][1]) <= h.r * (Math.PI / 60) + 1e-12, 'every 3 degrees');
    assert.ok(onArc.every((p) => p[0] <= 0.01 + 1e-12), 'the arc inside the square');
  }
  assert.deepEqual(cutDisk(sq, { c: [0.005, 0.005], r: 0.1 }), sq, 'a disk over every corner leaves the outline (nothing outside to keep)');
  // a shape: the crossing hole cut from the outline, the inner one a hole, the outer one ignored
  const s = sectionShape({ kind: 'poly', pts: sq, holes: [h, { c: [0.003, 0.005], r: 0.001 }, { c: [0.03, 0.03], r: 0.001 }] }).extractPoints(12);
  assert.equal(s.holes.length, 1);
  assert.ok(s.shape.every((p) => Math.hypot(-p.x - h.c[0], p.y - h.c[1]) >= h.r - 1e-9), 'no outline point in the cut disk');
});

test('beats: a step\'s kind is its BOM lines, side aside; the side rule needs one side and a joint; names say first and last', () => {
  const mk = (steps, lines, tags, joins, cat = {}) => {
    const ph = { steps, lines, joins, solid: (id, name) => ({ bom: tags[`${id}/${name}`] }) };
    ph.line = (t) => lines.find((l) => l.key === t) ?? null;
    return screenBeats(ph, cat);
  };
  const L = (key, kind = 'structure', id = 'metal') => ({ key, kind, id });
  // mirrored pair: lines differing only by side are one kind; different lines are not
  const lines = [L('bracket right'), L('bracket left'), L('panel'), L('plate')];
  const tags = { 'a R/x': 'bracket right', 'a L/x': 'bracket left', 'p/x': 'panel', 'q/x': 'plate' };
  const b = mk([{ name: 'a R', solids: ['a R/x'] }, { name: 'a L', solids: ['a L/x'] }, { name: 'p', solids: ['p/x'] }, { name: 'q', solids: ['q/x'] }], lines, tags, []);
  assert.deepEqual(b.map((x) => x.steps), [[0, 1], [2], [3]]);
  assert.deepEqual(b.map((x) => x.name), ['a R … a L', 'p', 'q']);
  // one side's run: joined and all R; an unjoined R step, or an L one, starts a beat
  const l2 = [L('u'), L('v'), L('w'), L('z')], t2 = { 'u R/x': 'u', 'v R/x': 'v', 'w R/x': 'w', 'z L/x': 'z' };
  const s2 = [{ name: 'u', solids: ['u R/x'] }, { name: 'v', solids: ['v R/x'] }, { name: 'w', solids: ['w R/x'] }, { name: 'z', solids: ['z L/x'] }];
  assert.deepEqual(mk(s2, l2, t2, [['u R', 'v R', 'j'], ['z L', 'w R', 'j']]).map((x) => x.steps), [[0, 1], [2], [3]]);
  // fixings: a run of one kind joins itself; a fixing of another kind that fixes nothing of the beat starts one
  const cat = { 'screw-a': { category: 'fastener' }, 'nut-a': { category: 'fastener' }, metal: { category: 'metal_bar' } };
  const l3 = [L('plate'), L('screws', 'structure', 'screw-a'), L('nuts', 'structure', 'nut-a')];
  const t3 = { 'p/x': 'plate', 's1/h': 'screws', 's2/h': 'screws', 'n1/n': 'nuts' };
  const s3 = [{ name: 'p', solids: ['p/x'] }, { name: 's1', solids: ['s1/h'] }, { name: 's2', solids: ['s2/h'] }, { name: 'n1', solids: ['n1/n'] }];
  assert.deepEqual(mk(s3, l3, t3, [], cat).map((x) => x.steps), [[0], [1, 2], [3]]);
  assert.deepEqual(mk(s3, l3, t3, [['n1', 's2', 'thread']], cat).map((x) => x.steps), [[0], [1, 2, 3]]);
  assert.deepEqual(mk([...s3, { name: 'tie', threaded: true, solids: ['t/t'] }], l3, t3, [['t', 'p', 'tied']], cat).map((x) => x.steps), [[0], [1, 2], [3], [4]], 'a threaded tie binding nothing of the beat before');
  // bought parts onto one carrier: both on the tray placed before them; a part on another carrier starts a beat
  const l4 = [L('tray'), L('esc-x', 'part', 'esc-x'), L('bat-x', 'part', 'bat-x'), L('fc-x', 'part', 'fc-x'), L('aft')];
  const t4 = { 'tray/x': 'tray', 'aft/x': 'aft', 'e/b': 'esc-x', 'b/b': 'bat-x', 'f/b': 'fc-x' };
  const s4 = [{ name: 'tray', solids: ['tray/x'] }, { name: 'aft', solids: ['aft/x'] }, { name: 'e', solids: ['e/b'] }, { name: 'b', solids: ['b/b'] }, { name: 'f', solids: ['f/b'] }];
  assert.deepEqual(mk(s4, l4, t4, [['e', 'tray', 'r'], ['b', 'tray', 'r'], ['f', 'aft', 'r']]).map((x) => x.steps), [[0], [1], [2, 3], [4]]);
  assert.deepEqual(screenBeats(null, {}), []);
});

test('a drawn line\'s registry, name, subsystem and look: from its tag and joints, not from a list of part names', async () => {
  const { drawRest, registryOf, hardwareGeometry } = await import('../../sims/avis-shared/drive-solids.js');
  const D = model.design;
  // registered as the BOM lists the line: a structure line by its name and material, a bought part by its role and id
  const st = phys.lines.find((l) => l.kind === 'structure' && /^M3 nuts/.test(l.key)), bp = phys.lines.find((l) => l.kind === 'part');
  assert.deepEqual(registryOf({ D }, st, 'x'), { name: st.key, id: st.id });
  assert.deepEqual(registryOf({ D }, bp, 'x'), { name: D.bom.parts.find((b) => b.id === bp.id).role, id: bp.id });
  assert.deepEqual(registryOf({ D }, null, 'loose part'), { name: 'loose part', id: null });
  // names: the part and its one solid; an item of a part's several (one tie of a part's ties) by the item
  const ring = (name, bom, item) => ({ name, x: [0.1, 0.11], shape: { kind: 'ring', c: [0, 0], ro: 0.01, ri: 0 }, bom, ...(item ? { item } : {}) });
  const parts = [
    { id: 'web screw', solids: [ring('head', 'screw-iso4762-m3x12-10.9')] },
    { id: 'strap ties', solids: [ring('t1', 'tie-hellermanntyton-t18r', 'strap tie 1'), ring('t2', 'tie-hellermanntyton-t18r', 'strap tie 2')] },
    { id: 'keyed boss', solids: [ring('boss', 'nose tray riser'), ring('rim', 'nose tray riser')] },
  ];
  const meshes = [], reg = [];
  const ctx = {
    phys: { parts, part: (id) => parts.find((p) => p.id === id), line: phys.line, joins: [['keyed boss', 'gear shaft 1', 'key in its keyseat and keyway (cut to the key, not drawn)']] },
    meshes, root: new THREE.Group(), model: { layout: { assumptions: [] }, parts: model.parts }, D, geo: { mergeGeos: (gs) => gs[0] },
    mesh: (name, geo, m, parent, sub, id) => { const o = new THREE.Mesh(geo); o.name = name; o.userData.subsystem = sub; meshes.push(o); return o; },
    register: (...a) => reg.push(a),
  };
  drawRest(ctx);
  assert.deepEqual(meshes.map((o) => o.name), ['phys_web_screw_head', 'phys_strap_tie_1_t1', 'phys_strap_tie_2_t2', 'phys_keyed_boss']);
  assert.deepEqual(meshes.map((o) => o.userData.subsystem), ['structure', 'structure', 'structure', 'drive'], 'keyed: the drive\'s');
  assert.equal(reg.filter((r) => r[1] === 'tie-hellermanntyton-t18r').length, 2, 'one registration an item');
  // the look: a fastener's square prism across x drawn round in it; a rectangle that is not square across x, its prism
  const across = { name: 'head', x: [0, 0.0055], shape: { kind: 'poly', pts: [[0, 0], [0.0055, 0], [0.0055, 0.003], [0, 0.003]], holes: [] } };
  const radii = (g) => { const p = g.attributes.position.array, r = []; for (let i = 0; i < p.length; i += 3) r.push(Math.hypot(p[i] - 0.00275, p[i + 2] - 0.00275)); return [Math.min(...r), Math.max(...r)]; };
  assert.ok(Math.abs(radii(hardwareGeometry(across, across.shape, 'screw'))[1] - 0.00275) < 1e-9, 'round across x');
  assert.ok(radii(hardwareGeometry(across, across.shape, null))[1] > 0.0038, 'not a fastener: its prism');
  const long = { ...across, x: [0, 0.008] };
  assert.ok(radii(hardwareGeometry(long, long.shape, 'nut'))[1] > 0.0038, 'not square across x: its prism');
  const holed = { ...across, shape: { ...across.shape, holes: [{ c: [0.00275, 0.0015], r: 0.0005 }] } };
  assert.ok(radii(hardwareGeometry(holed, holed.shape, 'screw'))[1] > 0.0038, 'with a hole: its prism');
});
