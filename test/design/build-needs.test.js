// Build needs (src/design/build-needs.js): the record refuses a design missing a connector, the hatch, its declaration,
// the air path or the strap, or one whose pack cannot come out; geometry helpers checked against hand values.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildNeedsRecord, removalRecord, sweepCrossing, routeSolids, pathLength, needsParams, NEEDS, arcPieces, polarAtZ, hatchGeometry, outsideWindow, needsInterfaceRules, NEEDS_SEQUENCE, needsContainment, mergeContainment, ELECTRICAL_JOINTS, bboxOf } from '../../src/design/build-needs.js';
import { S, poly } from '../../src/design/clearance.js';
import { RULE_HELPERS } from '../../src/design/interfaces.js';
import { SEQUENCE as SEQUENCE16 } from '../../src/design/round16.mjs';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { fullAssembly } from '../../src/design/clearance.js';
import { solidMassParams } from '../../src/design/round11.mjs';
import { round17Set, mergeDeep } from '../../src/design/round17.mjs';
import { NEEDS_INTERFACE_RULES } from '../../src/design/build-needs.js';
import { interfaceLedger } from '../../src/design/interfaces.js';

const box = (name, [x0, x1], [z0, z1], [y0, y1]) => S(name, [x0, x1], poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]));
const part = (id, solids, bom = id) => ({ id, solids: solids.map((s) => ({ ...s, bom })), bom: [bom] });
// a straight elliptic pod 120 x 140 mm centred at y -27 mm, x -0.3..0.3 m
const LOFT = { kind: 'sections', sections: [0.3, 0, -0.3].map((x) => ({ x_m: x, y_m: -0.027, width_m: 0.12, height_m: 0.14 })) };
const HATCH = { x_mm: [133, 234], polar_deg: [40, 140] };
const CAT = [{ id: NEEDS.strap.id, specs: { releasable: true, width_mm: 15, length_mm: 330 } }];

/** A small, complete design: the pack on its tray under the hatch, its plugs, the receiver; every record row met. */
function fixture() {
  const P = needsParams({ needs: { tubing: { length_mm: 100 } } });
  const parts = [
    part('packed battery', [box('box', [0.1565, 0.2312], [-0.0168, 0.0168], [-0.0428, -0.0174])], 'batt'),
    part('packed battery ties', [box('tie top', [0.186, 0.201], [-0.0183, 0.0183], [-0.0174, -0.0159])], NEEDS.strap.id),
    part('nose tray', [box('base', [0.109, 0.228], [-0.0191, 0.0191], [-0.0434, -0.0428])]),
    part('hatch rail R', [box('rail', [0.134, 0.222], [0.0191, 0.0255], [-0.014, -0.0125])], 'hatch frame'),
    part('hatch rail L', [box('rail', [0.134, 0.222], [-0.0255, -0.0191], [-0.014, -0.0125])], 'hatch frame'),
    part('hatch cover', [box('land R', [0.134, 0.222], [0.0191, 0.0255], [-0.0125, -0.0117]), box('keeper', [0.176, 0.180], [-0.0186, 0.0186], [-0.0169, -0.0154])], 'hatch cover'),
    part('hatch cover arcs', [box('arc', [0.164, 0.1655], [-0.03, 0.03], [0.03, 0.033])], 'hatch cover arcs'),
    part('hatch screw 1', [box('head', [0.144, 0.1466], [0.0195, 0.025], [-0.0117, -0.0087])], 'screw'),
    part('pack plug', [box('housing', [0.159, 0.1749], [-0.008, 0.008], [-0.0174, -0.0095])], NEEDS.plugs.male),
    part('arming plug female', [box('housing', [0.202, 0.21], [-0.008, 0.008], [-0.0174, -0.0095])], NEEDS.plugs.female),
    part('arming jumper', [box('housing', [0.21, 0.2179], [-0.008, 0.008], [-0.0174, -0.0095])], NEEDS.plugs.male),
    part('packed receiver', [box('box', [0.1399, 0.1509], [-0.01726, -0.00626], [-0.0428, -0.0414])], 'rx'),
    part('receiver antenna', [box('coax', [0.137, 0.212], [0.032, 0.033], [-0.0355, -0.0345])], 'rx'),
    part('packed airspeed', [box('box', [0.1348, 0.1559], [-0.0037, 0.0174], [-0.0428, -0.0386])], 'aspd'),
    part('packed fc', [box('box', [0.1115, 0.1435], [-0.011, 0.011], [-0.0694, -0.0567])], 'fc'),
  ];
  const joins = [['hatch cover', 'hatch rail R', 'resting on the rail, screwed'], ['pack plug', 'packed battery', 'mated with the pack\'s discharge plug'], ['packed battery ties', 'packed battery', 'round the part']];
  const members = ['hatch screws', 'hatch screws in the rails', 'hatch aft stop', 'hatch keepers', 'pack stop forward', 'nose tray flanges: battery lateral'].map((name) => ({ name, check: /screws$/.test(name) ? 'bolts' : 'plate' }));
  const decl = mergeContainment({ openings: [], outside: [], on_skin: [] }, needsContainment(P, HATCH));
  const d = { body: { loft: LOFT }, params: { layout: { hatch: HATCH, containment: decl } }, structure: { members, airframe: { parts: { support: { ties: [{ part: 'battery', tie: NEEDS.strap.id }] } } } } };
  const asm = { parts, joins, needs: { P, q0: null, leads: [], tubing: { lines: [{}, {}], L: 0.1 }, antenna: { L: 0.08 } } };
  return { asm, d };
}
const without = (asm, re) => ({ ...asm, parts: asm.parts.filter((p) => !re.test(p.id)) });
const row = (rec, item) => rec.rows.find((r) => r.item === item || r.item.startsWith(item));

test('the complete fixture passes every row of the record', () => {
  const { asm, d } = fixture(), rec = buildNeedsRecord(asm, d, CAT);
  assert.deepEqual(rec.failed, [], JSON.stringify(rec.failed));
});

test('a missing connector fails its joint (the pack\'s mate, the arming jumper)', () => {
  const { asm, d } = fixture();
  const a = buildNeedsRecord(without(asm, /^pack plug$/), d, CAT), b = buildNeedsRecord(without(asm, /^arming jumper$/), d, CAT);
  assert.equal(a.passed, false); assert.equal(row(a, 'joint: pack to the power lead').ok, false);
  assert.equal(b.passed, false); assert.equal(row(b, 'joint: arming plug').ok, false);
});

test('a missing hatch fails: no rails or screws, or no declared opening', () => {
  const { asm, d } = fixture();
  const a = buildNeedsRecord(without(asm, /^hatch (rail R|screw \d+)$/), d, CAT);
  assert.equal(row(a, 'hatch with a rated fixing').ok, false); assert.equal(a.passed, false);
  const d2 = { ...d, params: { layout: { ...d.params.layout, containment: { ...d.params.layout.containment, openings: d.params.layout.containment.openings.filter((o) => o.name !== 'hatch') } } } };
  const b = buildNeedsRecord(asm, d2, CAT);
  assert.equal(row(b, 'hatch declared as an opening').ok, false); assert.equal(b.passed, false);
  const c = buildNeedsRecord({ ...asm, needs: null }, d, CAT);
  assert.equal(c.passed, false);
});

test('an unrated hatch fixing fails', () => {
  const { asm, d } = fixture();
  const d2 = { ...d, structure: { ...d.structure, members: d.structure.members.map((m) => (m.name === 'hatch screws' ? { ...m, check: 'unrated' } : m)) } };
  assert.equal(row(buildNeedsRecord(asm, d2, CAT), 'hatch with a rated fixing').ok, false);
});

test('a part left over the pack blocks its removal; a window too short cuts the covering', () => {
  const { asm, d } = fixture();
  const blocked = { ...asm, parts: [...asm.parts, part('former 4', [box('band 7', [0.164, 0.166], [-0.005, 0.005], [0.0, 0.002])], 'fuselage formers')] };
  const r = removalRecord(blocked, d, null, HATCH);
  assert.equal(r.ok, false); assert.ok(r.steps.find((s) => s.step === 'pack').through.some((t) => /former 4/.test(t.placed)));
  const ok = removalRecord(asm, d, null, HATCH);
  assert.equal(ok.ok, true, JSON.stringify(ok.steps.filter((s) => !s.ok)));
  const short = removalRecord(asm, d, null, { x_mm: [160, 234], polar_deg: [40, 140] });
  assert.equal(short.crossing_in_window, false); assert.equal(short.ok, false);
});

test('the sweep crosses the skin where the pack\'s corners meet it (hand value)', () => {
  const pack = [{ part: 'p', id: 'p/box', x: [0.16, 0.2], sec: poly([[-0.0168, -0.0428], [0.0168, -0.0428], [0.0168, -0.0174], [-0.0168, -0.0174]]) }];
  const c = sweepCrossing(LOFT, pack, [0, 1, 0], 0.12);
  // a corner at z 16.8 mm on a 60 mm half-width meets the ellipse at acos(16.8 / 60) = 73.74 deg
  assert.ok(Math.abs(c.polar[0] - (Math.acos(0.0168 / 0.06) * 180) / Math.PI) < 1e-6, String(c.polar[0]));
  assert.ok(Math.abs(c.polar[1] - (180 - (Math.acos(0.0168 / 0.06) * 180) / Math.PI)) < 1e-6);
});

test('the air path, the strap and the tubing rows refuse when missing', () => {
  const { asm, d } = fixture();
  const noAir = { ...d, params: { layout: { ...d.params.layout, containment: { ...d.params.layout.containment, openings: d.params.layout.containment.openings.filter((o) => !/^cooling/.test(o.name)) } } } };
  assert.equal(row(buildNeedsRecord(asm, noAir, CAT), 'air path').ok, false);
  const ties = { ...d, structure: { ...d.structure, airframe: { parts: { support: { ties: [{ part: 'battery', tie: 'tie-hellermanntyton-t40r' }] } } } } };
  assert.equal(row(buildNeedsRecord(asm, ties, CAT), 'the pack on a releasable strap').ok, false);
  assert.equal(row(buildNeedsRecord({ ...asm, needs: { ...asm.needs, tubing: { lines: [{}, {}], L: 0.12 } } }, d, CAT), 'pitot pressure tubing').ok, false);
});

test('the air path counts its drag: Cd 1 on both 8 mm holes', () => {
  const { asm, d } = fixture(), r = row(buildNeedsRecord(asm, d, CAT), 'air path');
  assert.ok(Math.abs(r.CdA_m2 - 2 * Math.PI * 0.004 ** 2) < 2e-8); // the declared areas are rounded to 0.01 mm2
  assert.match(r.cooling, /UNVERIFIED/);
});

test('a reach column blocked by a part fails (the arming jumper under a deck)', () => {
  const { asm, d } = fixture();
  const deck = { ...asm, parts: [...asm.parts, part('deck', [box('plate', [0.2, 0.23], [-0.01, 0.01], [-0.005, -0.004])], 'deck')] };
  assert.equal(row(buildNeedsRecord(deck, d, CAT), 'armed').ok, false);
});

test('routeSolids: the pieces\' lengths sum to the polyline\'s (hand value 3-4-5)', () => {
  const pts = [[0, 0, 0], [0.03, 0, 0], [0.03, 0.04, 0], [0.06, 0.04, 0.04]];
  const r = routeSolids('l', pts, 0.001);
  assert.ok(Math.abs(r.L - (0.03 + 0.04 + 0.05)) < 1e-12);
  assert.ok(Math.abs(r.solids.reduce((a, s) => a + s.tube.L, 0) - r.L) < 1e-12);
  assert.ok(Math.abs(pathLength(pts) - 0.12) < 1e-12);
  for (const s of r.solids) assert.ok(s.x[1] - s.x[0] <= 0.02 + 0.002 + 1e-12);
});

test('arcPieces lie on the skin; polarAtZ on a circle', () => {
  for (const q of arcPieces(LOFT, 0.1, [40, 140], 0.003)) {
    const e = (p) => Math.hypot(p[0] / 0.06, (p[1] + 0.027) / 0.07);
    assert.ok(Math.abs(e(q[0]) - 1) < 1e-9 && e(q[2]) < 1);
  }
  assert.ok(Math.abs(polarAtZ(LOFT, 0, 0.03) - 60) < 1e-9); // acos(30/60)
});

test('the hatch geometry keeps the pack\'s lift path clear: rails outboard of the pack, the window\'s x the param\'s', () => {
  const G = hatchGeometry(LOFT, needsParams({}), { y: -0.0431, w: 0.0376, h: 0.005, t: 0.0006, x: [0.1089, 0.228] });
  for (const r of G.rails) assert.ok(Math.min(...r.z.map(Math.abs)) >= 0.0168 + 0.0005);
  assert.ok(G.screws.length === 4 && G.posts.length === 4);
});

test('outsideWindow: a band piece at the top inside the window is left out, one at the side is kept', () => {
  const top = [[-0.005, 0.042], [0.005, 0.042], [0.005, 0.038], [-0.005, 0.038]], side = [[0.059, -0.03], [0.06, -0.025], [0.056, -0.025], [0.055, -0.03]];
  assert.equal(outsideWindow(top, 0.165, LOFT, HATCH), false);
  assert.equal(outsideWindow(side, 0.165, LOFT, HATCH), true);
  assert.equal(outsideWindow(top, 0.1, LOFT, HATCH), true);
});

test('every sequence insert names a round-16 step, and the rules match their joins', () => {
  const names = new Set(SEQUENCE16.steps.map((s) => s.name));
  for (const q of NEEDS_SEQUENCE) assert.ok(names.has(q.after ?? q.replace), q.after ?? q.replace);
  const rules = needsInterfaceRules(RULE_HELPERS);
  const find = (why, a) => rules.find((r) => r.why.test(why) && (!r.part || r.part.test(a)));
  assert.ok(find('resting on the rail, screwed', 'hatch cover'));
  assert.ok(find('round the part', 'packed battery ties'));
  assert.equal(find('round the part', 'packed esc ties'), undefined);
  assert.deepEqual(find('round the part', 'packed battery ties').axial.rating(), ['hatch aft stop', 'hatch keepers', 'pack stop forward']);
});

test('the joint table: every connector joint names its parts, every solder joint its basis', () => {
  for (const j of ELECTRICAL_JOINTS) assert.ok(j.means === 'soldered' ? j.basis.length > 10 : j.parts.length > 0, j.joint);
  assert.deepEqual(bboxOf({ kind: 'ring', c: [1, 2], ro: 0.5 }), { z: [0.5, 1.5], y: [1.5, 2.5] });
});

// the real round-17 build (v24's parameters, round17Set over them, mass from the drawn solids): ~40 s
test('the round-17 build draws every build need and its record passes; without the pack\'s plug or the hatch it fails', { timeout: 300000 }, () => {
  const root = new URL('../../', import.meta.url);
  const cat = JSON.parse(readFileSync(new URL('data/parts/catalog.json', root), 'utf8'));
  const base = JSON.parse(readFileSync(new URL('data/designs/larus-gl-2.json', root), 'utf8'));
  const set = round17Set({}, cat);
  let p = mergeDeep(base.params, set);
  p.massFromSolids = set.massFromSolids;
  p = solidMassParams(p, cat);
  const d = buildDesign(p, cat), { asm } = fullAssembly(d, cat);
  const rec = buildNeedsRecord(asm, d, cat);
  assert.equal(rec.passed, true, rec.failed.join('; '));
  // every lead within its catalogue length with the stated slack, on its drawn route
  for (const l of asm.needs.leads) assert.equal(l.ok, true, l.name);
  const tw = asm.needs.leads.find((l) => l.name === 'twist-servo lead R');
  assert.ok(tw.wing.drawn_mm * 1.1 <= tw.wing.ext_mm && tw.pod.drawn_mm * 1.1 <= tw.pod.ext_mm && tw.wing.ext !== tw.pod.ext);
  // the hatch's fixing is in the M3 lines and rated in M7; the strap replaced the pack's two T40R
  assert.ok(d.structure.members.some((m) => m.name === 'hatch screws' && m.check === 'bolts'));
  assert.ok(d.bom.structure.some((s) => /^M3 screws, \d+ off$/.test(s.name)) && asm.parts.filter((q) => /^hatch screw \d+$/.test(q.id)).length === 4);
  assert.equal(d.structure.airframe.parts.support.ties.find((t) => t.part === 'battery').tie, NEEDS.strap.id);
  // the build needs' joins all have a rated rule
  const L = interfaceLedger(asm, d, NEEDS_INTERFACE_RULES), mine = /^(hatch|pack plug|arming|pressure tubing|receiver antenna|belly tray|twist-servo lead|tail lead|harness tie|pack stop)/;
  assert.deepEqual(L.failed.filter((r) => mine.test(r.a) || mine.test(r.b)).map((r) => `${r.a} | ${r.b}: ${r.why_failed}`), []);
  // broken designs: a missing connector, a missing hatch, a hatch window that does not reach the pack's front
  assert.equal(buildNeedsRecord(without(asm, /^pack plug$/), d, cat).passed, false);
  const noHatch = buildNeedsRecord(without(asm, /^hatch (rail [RL]|cover|cover arcs|screw \d+)$/), d, cat);
  assert.equal(noHatch.passed, false); assert.equal(row(noHatch, 'hatch with a rated fixing').ok, false);
  const short = { ...d, params: { ...d.params, layout: { ...d.params.layout, hatch: { x_mm: [133, 220], polar_deg: [40, 140] } } } };
  assert.equal(row(buildNeedsRecord(asm, short, cat), 'the pack comes out').ok, false);
});

test('no tubing, no antenna, no strap row or no cage member each refuse', () => {
  const { asm, d } = fixture();
  const r = (a, dd = d) => buildNeedsRecord(a, dd, CAT);
  assert.equal(row(r({ ...asm, needs: { ...asm.needs, tubing: null } }), 'pitot pressure tubing').ok, false);
  assert.equal(row(r({ ...asm, needs: { ...asm.needs, antenna: null } }), 'receiver antenna drawn').ok, false);
  const noTies = { ...d, structure: { ...d.structure, airframe: { parts: { support: { ties: [] } } } } };
  assert.equal(row(r(asm, noTies), 'the pack on a releasable strap').strap, null);
  const noCage = { ...d, structure: { ...d.structure, members: d.structure.members.filter((m) => m.name !== 'pack stop forward') } };
  assert.equal(row(r(asm, noCage), 'the pack on a releasable strap').ok, false);
  const unreleasable = [{ id: NEEDS.strap.id, specs: { releasable: false } }];
  assert.equal(row(buildNeedsRecord(asm, d, unreleasable), 'the pack on a releasable strap').ok, false);
});

test('the record\'s measures against hand values: cable paths, the sweep, the reach column, the leads\' drag', () => {
  const { asm, d } = fixture(), rec = buildNeedsRecord(asm, d, CAT), near = (a, b, t = 1e-3) => assert.ok(Math.abs(a - b) <= t, `${a} vs ${b}`);
  // ASPD to FC round the tray's left edge: 19.4 out, 16.0 down, 12.1 in (mm)
  near(row(rec, 'airspeed board to the FC').path_mm, 19.4 + 16.0 + 12.1, 0.01);
  near(row(rec, 'airspeed board to the FC').with_slack_mm, 1.1 * 47.5, 0.01);
  // the FC's side to the window's aft edge + 4 mm: 12.1 out, 43.05 up
  near(row(rec, 'set up').path_mm, 12.1 + 43.05, 0.01);
  const rm = row(rec, 'the pack comes out');
  assert.deepEqual(rm.crossing.x_mm, [156.5, 231.2]);
  near(rm.crossing.polar_deg[0], 73.7, 0.05); near(rm.crossing.polar_deg[1], 106.3, 0.05);
  assert.equal(rm.steps.find((x) => x.step === 'strap (released)').released, true);
  assert.equal(rm.steps.find((x) => x.step === 'pack').worst_gap_mm >= 0, true);
  const arm = row(rec, 'armed');
  near(arm.column_mm.x, 213.95, 1e-6); near(arm.column_mm.to_skin_y, 43, 1e-6); near(arm.column_mm.from_y, -9.5, 1e-6);
  assert.equal(row(rec, 'bound').column_mm.x, 146.9);
  const hr = row(rec, 'hatch with a rated fixing');
  assert.equal(hr.screws, 1); assert.deepEqual(hr.members.map((m) => m.check), ['bolts', 'plate']);
  // a moving twist-servo lead's exposed span: Cd 1.2 x 40 mm x 3.3 mm; one moving lead without it refuses
  const lead = { name: 'twist-servo lead R', moving: true, ok: true, wing: { root_span_mm: 40 } };
  const r2 = buildNeedsRecord({ ...asm, needs: { ...asm.needs, leads: [lead] } }, d, CAT);
  near(row(r2, 'the twist-servo leads').CdA_m2, 1.2 * 0.04 * 0.0033, 1e-9);
  assert.equal(row(buildNeedsRecord({ ...asm, needs: { ...asm.needs, leads: [{ ...lead, wing: undefined }] } }, d, CAT), 'the twist-servo leads').ok, false);
  assert.equal(row(buildNeedsRecord({ ...asm, needs: { ...asm.needs, leads: [{ ...lead, ok: false }] } }, d, CAT), 'joint: undefined').ok, false);
  // a cable too short fails; a missing end part fails
  assert.equal(row(buildNeedsRecord(without(asm, /^packed fc$/), d, CAT), 'airspeed board to the FC').ok, false);
  const far = { ...asm, parts: asm.parts.map((q) => (q.id === 'packed fc' ? part('packed fc', [box('box', [-0.2, -0.17], [-0.011, 0.011], [-0.08, -0.07])], 'fc') : q)) };
  assert.equal(row(buildNeedsRecord(far, d, CAT), 'airspeed board to the FC').ok, false);
  assert.equal(row(buildNeedsRecord(far, d, CAT), 'set up').ok, true);
});
