// Round 15 (SPEC "Design freeze", "Support and containment"): the carriers, mounts, ties and fixings by hand from the
// stated numbers and the catalogue, the packing's mounted placement, and the official build's refusal rule.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { CARRIERS, mountsFor, officialRefusal, round15Set, CONTAINMENT, FIT_GRID } from '../../src/design/round15.mjs';
import { carrierGeometry, tieGeometry, tieFor, pitotMount, motorBracketGeometry } from '../../src/design/airframe.js';
import { mountOrder, packedSpecs, partsIndex, prepare } from '../../sims/avis-shared/model.js';
import { buildDesign } from '../../src/design/build.js';
import { lumpPositionsFrom, packDesign } from '../../src/design/fit.js';
import { byId } from '../../src/design/catalog.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('cable ties: band and bundle range read from the maker\'s text; the smallest tie whose bundle takes the loop', () => {
  const s = tieGeometry(byId(catalog, 'tie-hellermanntyton-t18r').specs), l = tieGeometry(byId(catalog, 'tie-hellermanntyton-t40r').specs);
  // "100 x 2.5 x 1.0 mm, bundle 1.5-22 mm" and "180 (page also says 175) x 4.0 x 1.15 mm, bundle 1.5-40 mm"
  assert.deepEqual([s.b_m, s.t_m, ...s.bundle_m].map((v) => +(v * 1000).toFixed(3)), [2.5, 1, 1.5, 22]);
  assert.deepEqual([l.b_m, l.t_m, ...l.bundle_m].map((v) => +(v * 1000).toFixed(3)), [4, 1.15, 1.5, 40]);
  const ties = [{ id: 'S', geo: s }, { id: 'L', geo: l }];
  // the battery (33.5 wide, 25.4 high) on 0.6 mm sheet: 2 (33.5 + 25.4 + 0.6) = 119.0 mm, over pi x 22 = 69.1, under pi x 40 = 125.7
  const bat = tieFor(ties, 0.0335, 0.0254, 0.0006);
  assert.equal(bat.id, 'L'); near(bat.loop_m, 0.119, 1e-12);
  assert.equal(tieFor(ties, 0.011, 0.0014, 0.0006).id, 'S'); // the receiver: 26.0 mm
  assert.equal(tieFor(ties, 0.0345, 0.0254, 0.0006).id, 'L'); // 121.0 mm
  assert.equal(tieFor(ties, 0.040, 0.0254, 0.0006), null); // 132.0 mm: no catalogue tie
  assert.equal(tieFor(ties, 0.0005, 0.0001, 0.0001), null); // 1.4 mm, under pi x 1.5 = 4.7 mm
  assert.equal(tieFor(ties, 0.0345, 0, 0).id, 'S'); // a 69.0 mm loop: just inside T18R's pi x 22 = 69.1 mm
  assert.equal(tieFor(ties, 0.0346, 0, 0).id, 'L'); // 69.2 mm: just past it
});

test('carriers by hand from round 15\'s stated numbers and the plates\' own faces', () => {
  const planes = { plateFwd: 0.0756, plateAft: 0.014 }, tF = 0.0012, tA = 0.0016;
  const A = { tray: CARRIERS.tray, noseTray: CARRIERS.noseTray, aftTray: CARRIERS.aftTray };
  const G = carrierGeometry(A, planes, tF, tA);
  near(G.forward.x[0], 0.0756 + 0.0006); near(G.forward.x[1], 0.1089 + 0.0008); near(G.forward.y, -0.080); near(G.forward.tab, 0.010);
  near(G.riser.x[0], 0.1089); near(G.riser.x[1], 0.1097); near(G.riser.w, 0.0376); near(G.riser.f, 0.007); near(G.riser.foot, 0.008);
  near(G.riser.y[0], -0.080 + 0.0004); near(G.riser.y[1], -0.0431 - 0.0003); // the forward tray's base top to the nose tray's base bottom
  near(G.nose.x[0], 0.1089); near(G.nose.x[1], 0.228); near(G.nose.h, 0.005); near(G.nose.t, 0.0006); near(G.nose.w, 0.0376);
  near(G.aft.x[1], 0.014 - 0.0008); near(G.aft.x[0], 0.014 - 0.0008 - 0.123); near(G.aft.y, -0.0812); near(G.aft.w, 0.0532);
});

test('every mount rests on its carrier\'s base top; side-by-side parts and their ties fit the channel without overlap', () => {
  const M = mountsFor(CARRIERS), N = CARRIERS.noseTray, A = CARRIERS.aftTray;
  const fN = (N.y_mm + N.t_mm / 2) / 1000, fA = (A.y_mm + A.t_mm / 2) / 1000;
  for (const [k, m] of Object.entries(M)) near(m.face_y_m, m.carrier === 'nose tray' ? fN : fA, 1e-12);
  const iN = (N.w_mm - N.t_mm) / 2000, iA = (A.w_mm - A.t_mm) / 2000;
  // nose tray: the airspeed board (21.15 mm: 3.5 g at 1850 kg/m3 as a 5:1 board) and the receiver (11 mm), each with a
  // T18R either side, 0.5 mm apart, inside the flanges' inner faces
  const airZ1 = M.airspeed.z0_m + 0.02115;
  assert.ok(M.airspeed.z0_m - 0.001 >= -iN - 1e-9);
  assert.ok(M.receiver.z0_m - 0.001 - (airZ1 + 0.001) >= 0.0005 - 1e-5, `${M.receiver.z0_m} after ${airZ1}`);
  assert.ok(M.receiver.z0_m + 0.011 + 0.001 <= iN);
  // the ESC across the tray with a T40R either side: 34.3 + 2.3 mm within the inner width
  assert.ok(0.0343 + 2 * 0.00115 <= 2 * iN + 1e-9);
  // aft tray: the servos side by side, symmetric, their T40R ties 0.5 mm apart at the middle
  near(M.servo_tail_R.z0_m, -(M.servo_tail_L.z0_m + 0.013));
  near(M.servo_tail_R.z0_m - 0.00115 - (M.servo_tail_L.z0_m + 0.013 + 0.00115), 0.0005, 1e-9);
  // the flight controller (22 mm) beside the regulator (25.4 mm), each with a T40R either side, 0.5 mm apart
  assert.ok(M.fc.z0_m - 0.00115 >= -iA - 1e-9);
  near(M.regulator.z0_m - 0.00115 - (M.fc.z0_m + 0.022 + 0.00115), 0.0005, 1e-9);
  assert.ok(M.regulator.z0_m + 0.0254 + 0.00115 <= iA + 1e-9);
  // the ESC starts clear of the riser tab's screw heads (5.5 mm heads at the tab's middle)
  const tabMid = (N.riser_x_mm + N.riser.t_mm + N.riser.foot_mm / 2) / 1000;
  assert.ok(M.esc.x0_m >= tabMid + 0.00275 + 0.0005 - 1e-9);
  assert.equal(M.battery.x, 'lump');
});

test('the pitot along the nose tray\'s right flange, its bottom flush with the base, from its stated aft end', () => {
  const r = byId(catalog, 'mtube-tigertitanium-ti3al2.5v-6.35x5.54').specs.od_mm / 2000;
  const pm = pitotMount({ noseTray: CARRIERS.noseTray, pitot: CARRIERS.pitot }, { pitotLength_mm: 100 }, catalog);
  near(r, 0.003175);
  near(pm.c[0], (37.6 / 2 + 0.6 / 2) / 1000 + r); near(pm.c[1], (-43.1 - 0.3) / 1000 + r);
  near(pm.x[0], 0.196); near(pm.x[1], 0.296);
  const left = pitotMount({ noseTray: CARRIERS.noseTray, pitot: { ...CARRIERS.pitot, side: 'left' } }, {}, catalog);
  near(left.c[0], -pm.c[0]);
  // the set puts the probe's lump at its mount: the middle of its length, on its axis
  const s = round15Set({}, catalog);
  assert.deepEqual(s.lumpPos['pitot-static probe'], [0.246, +pm.c[1].toFixed(5), +pm.c[0].toFixed(5)]);
});

test('the motor bracket: the four back-face threads on the 25 mm circle, the bracket behind the motor, a sleeve on each rail', () => {
  const B = CARRIERS.motor.bracket, c = [0.0066, 0.0177], rails = { right: [0.029, -0.00075], left: [-0.029, -0.00075] };
  const g = motorBracketGeometry(B, { motorX: [0.0393, 0.0731] }, c, rails, 0.0065);
  near(g.x[0], 0.0393 - 0.002); near(g.x[1], 0.0393);
  assert.deepEqual(g.holes.map((h) => h.map((v) => +v.toFixed(6))), [[0.0191, 0.0177], [0.0066, 0.0302], [-0.0059, 0.0177], [0.0066, 0.0052]]);
  assert.deepEqual(g.sleeves.map((q) => [q.side, q.c, q.ro, q.ri]), [['right', rails.right, 0.0085, 0.0065], ['left', rails.left, 0.0085, 0.0065]]);
  near(g.stub.r, 0.003);
  // the outline holds every land and both sleeves (its hull)
  const zs = g.outline.map((p) => p[0]), ys = g.outline.map((p) => p[1]);
  near(Math.max(...zs), 0.029 + 0.0085, 1e-5); near(Math.min(...zs), -0.029 - 0.0085, 1e-5);
  near(Math.max(...ys), 0.0302 + 0.0035, 1e-5); near(Math.min(...ys), -0.00075 - 0.0085, 1e-5);
});

test('the packing order puts every mounted part after the part it follows; a cycle is an error', () => {
  const list = ['battery', 'esc', 'fc', 'airspeed', 'receiver'].map((name) => ({ name }));
  const mounts = { battery: { after: 'airspeed' }, airspeed: { after: 'esc' }, receiver: { after: 'esc' } };
  assert.deepEqual(mountOrder(list, mounts).map((s) => s.name), ['esc', 'airspeed', 'battery', 'fc', 'receiver']);
  assert.deepEqual(mountOrder(list, {}).map((s) => s.name), list.map((s) => s.name));
  assert.throws(() => mountOrder(list, { esc: { after: 'battery' }, battery: { after: 'esc' } }), /cycle/);
});

test('the official build refuses a design with any clash, any unsupported or uncontained part, or a missing record', () => {
  const ok = { passed: true, clashes: [], pod_shell: { clashes: [] }, support: { passed: true, unsupported: [] }, containment: { passed: true, violations: [], outside_violations: [] } };
  assert.equal(officialRefusal(ok), null);
  assert.match(officialRefusal({ ...ok, passed: false, support: { passed: false, unsupported: ['battery'] } }), /unsupported: battery/);
  assert.match(officialRefusal({ ...ok, passed: false, containment: { passed: false, violations: [{ solid: 'nose tray/base' }], outside_violations: ['boom clamp aft'] } }), /not contained: nose tray\/base, boom clamp aft/);
  assert.match(officialRefusal({ ...ok, passed: false, clashes: [{}] }), /1 clashing part pairs/);
  assert.match(officialRefusal({ ...ok, support: undefined }), /no support record/);
  assert.match(officialRefusal({ ...ok, containment: undefined }), /no containment record/);
  assert.match(officialRefusal({ ...ok, passed: false }), /failed/);
  assert.equal(officialRefusal(null), 'no clearance record');
});

test('the openings and outside fittings are declared with reasons; the fit grid starts at v22\'s pod and steps the centre line by 1 mm', () => {
  for (const o of [...CONTAINMENT.openings, ...CONTAINMENT.outside]) assert.ok(o.why && o.why.length > 20, JSON.stringify(o));
  assert.deepEqual(CONTAINMENT.openings.filter((o) => o.drag === false).map((o) => o.name), ['wing root, right', 'wing root, left']);
  assert.deepEqual([FIT_GRID.widths[0], FIT_GRID.heights[0]], [0.11, 0.14]);
  for (let k = 1; k < FIT_GRID.ys.length; k++) near(FIT_GRID.ys[k - 1] - FIT_GRID.ys[k], 0.001, 1e-12);
});

test('mounted parts are placed on their stated faces and x, the battery from its lump along x only (v21 fixture, round 15 set)', () => {
  const v21 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
  const merge = (a, b) => { if (!b || typeof b !== 'object' || Array.isArray(b)) return b; const o = { ...(a ?? {}) }; for (const [k, v] of Object.entries(b)) o[k] = v && typeof v === 'object' && !Array.isArray(v) ? merge(a?.[k], v) : v; return o; };
  const s = round15Set({}, catalog);
  let p = merge(v21.params, { ...s, layout: { ...(s.layout), stage1: 'overhung' } });
  p = { ...p, bodyWidth_m: 0.12, bodyHeight_m: 0.14, bodyY_m: -0.028 };
  const d = buildDesign(p, catalog);
  const model = prepare(d, partsIndex(d, catalog));
  const by = Object.fromEntries(model.layout.parts.map((q) => [q.name, q]));
  const M = s.layout.mounts, sizes = Object.fromEntries(packedSpecs(d, partsIndex(d, catalog)).list.map((q) => [q.name, q.size]));
  for (const [name, m] of Object.entries(M)) {
    const b = by[name].box;
    near(b.y0, m.face_y_m, 1e-12);
    near(b.y1 - b.y0, sizes[name][1], 1e-12);
    if (m.x0_m != null) near(b.x0, m.x0_m, 1e-12);
    if (m.z_m != null) near((b.z0 + b.z1) / 2, m.z_m, 1e-12); else near(b.z0, m.z0_m, 1e-12);
    if (m.after && m.x !== 'lump') near(b.x0, by[m.after].box.x1 + m.gap_m, 1e-12);
  }
  const bat = by.battery, lumpX = d.mass.lumps.find((l) => l.name === 'battery').pos[0];
  near(bat.shift[0], (bat.box.x0 + bat.box.x1) / 2 - lumpX, 1e-12);
  assert.ok(bat.box.x0 >= by.airspeed.box.x1 + M.battery.gap_m - 1e-12);
  // the fit's lumps follow the mounted parts (the regulator's too)
  const lp = lumpPositionsFrom(model);
  near(lp['power module and regulator'][0], +((by.regulator.box.x0 + by.regulator.box.x1) / 2).toFixed(4), 1e-12);
  // a mounted part that the loft cannot hold is reported, never moved off its carrier
  const pk = packDesign({ ...d, body: { ...d.body, loft: { ...d.body.loft, sections: d.body.loft.sections.map((q) => ({ ...q, height_m: q.height_m * 0.6 })) } } }, catalog, { containment: false });
  const gps = pk.model.layout.parts.find((q) => q.name === 'gps');
  assert.ok(gps.protrusion > 0); near(gps.box.y0, M.gps.face_y_m, 1e-12); near(gps.box.x0, M.gps.x0_m, 1e-12);
  assert.ok(pk.issues.some((i) => i.part === 'gps'));
});

test('the mounts by hand: the ESC 0.05 mm past the riser tab screw heads, every z from the channel\'s inner face and the ties, the tie counts', () => {
  const M = mountsFor(CARRIERS), N = CARRIERS.noseTray, A = CARRIERS.aftTray;
  // the riser's aft face 108.9, its 0.8 mm web and 8 mm tab: the tab's forward edge 117.7 mm, its screws at 113.7 mm
  near(M.esc.x0_m, 0.117, 1e-12);
  near(M.esc.x0_m - (0.1137 + 0.00275 + 0.0005), 0.00005, 1e-9);
  const iN = (37.6 - 0.6) / 2000, iA = (53.2 - 0.6) / 2000;
  near(M.airspeed.z0_m, -iN + 0.001, 1e-9); // its T18R on the flange's inner face
  near(M.receiver.z0_m, -iN + 0.001 + 0.02116 + 0.001 + 0.0005 + 0.001, 1e-9);
  near(M.servo_tail_L.z0_m, -(0.00025 + 0.00115 + 0.013), 1e-12);
  near(M.servo_tail_R.z0_m, 0.00025 + 0.00115, 1e-12);
  near(M.fc.z0_m, -iA + 0.00115, 1e-9);
  near(M.regulator.z0_m, -iA + 0.00115 + 0.022 + 0.00115 + 0.0005 + 0.00115, 1e-9);
  for (const k of ['esc', 'battery', 'gps']) assert.equal(M[k].z_m, 0);
  assert.deepEqual(Object.fromEntries(Object.entries(M).map(([k, m]) => [k, m.ties])), { esc: 1, airspeed: 1, receiver: 1, battery: 2, servo_tail_L: 2, servo_tail_R: 2, fc: 1, regulator: 1, gps: 1 });
  assert.deepEqual([M.servo_tail_L.x0_m, M.fc.x0_m, M.regulator.x0_m, M.gps.x0_m], [-0.0513, -0.0838, -0.0772, -0.1073]);
  near(M.servo_tail_R.x0_m, M.servo_tail_L.x0_m, 0);
  assert.deepEqual(Object.fromEntries(Object.entries(M).map(([k, m]) => [k, m.carrier])), { esc: 'nose tray', airspeed: 'nose tray', receiver: 'nose tray', battery: 'nose tray', servo_tail_L: 'aft tray', servo_tail_R: 'aft tray', fc: 'aft tray', regulator: 'aft tray', gps: 'aft tray' });
  for (const m of Object.values(M)) assert.equal(m.face_y_m, m.carrier === 'nose tray' ? (N.y_mm + N.t_mm / 2) / 1000 : (A.y_mm + A.t_mm / 2) / 1000);
});

test('the refusal names the counts and the failing parts exactly, a record without pod rows counting none', () => {
  assert.equal(officialRefusal({ passed: false, clashes: [{}, {}], support: { passed: true }, containment: { passed: true } }), '2 clashing part pairs, 0 against the pod');
  assert.equal(officialRefusal({ passed: false, clashes: [], pod_shell: { clashes: ['packed gps'] }, support: { passed: true }, containment: { passed: true } }), '0 clashing part pairs, 1 against the pod');
  assert.equal(officialRefusal({ passed: false, pod_shell: { clashes: ['x'] }, support: { passed: true }, containment: { passed: true } }), '0 clashing part pairs, 1 against the pod');
  assert.equal(officialRefusal({ passed: false, clashes: [], support: { passed: false, unsupported: ['gps', 'fc'] }, containment: { passed: true } }), 'unsupported: gps, fc');
  assert.equal(officialRefusal({ passed: false, clashes: [], support: { passed: false, unsupported: ['gps'] }, containment: { passed: false, violations: [], outside_violations: ['tail boom'] } }), 'unsupported: gps; not contained: tail boom');
});

test('the fittings left outside add their CdA to the pod\'s frontal drag once, the pod\'s own kept beside it', async () => {
  const { outsideDrag } = await import('../../src/design/round11.mjs');
  const f = { CdA_m2: 0.0008, area_mm2: 666.7, Cd: 1.2, basis: 'b' };
  const d = outsideDrag({ body: { CdA_front_m2: 0.003 }, structure: { clearance: { containment: { frontal: f } } } });
  near(d.body.CdA_front_m2, 0.0038, 1e-15); near(d.body.CdA_front_pod_m2, 0.003, 1e-15); near(d.body.CdA_front_outside_m2, 0.0008, 1e-15);
  assert.deepEqual(d.body.outside_frontal, { area_mm2: 666.7, Cd: 1.2, basis: 'b' });
  near(outsideDrag(d).body.CdA_front_m2, 0.0038, 1e-15); // applied again, still once
  const bare = { body: { CdA_front_m2: 0.003 }, structure: {} };
  assert.equal(outsideDrag(bare).body.CdA_front_m2, 0.003);
});
