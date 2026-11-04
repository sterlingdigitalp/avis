// Round 18 (sizing to the FoS rule): the tail's pivot pin runs in its bushings' bore (the pin and washer sized with
// them), the saddle's webs are rated at their drawn thickness, and the twist lever's tapped thread (a thread in our 7075
// lever) is a member, rated as the tail horn's, which the ledger names. Each refusal is pinned on a hand-built input; the
// thread's figure against a hand calculation.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { TAIL_MECH, tailMechGeometry, tailMechMembers, tappedThreadMember } from '../../src/design/tail-mech.js';
import { TWIST_SECTION, TWIST_INTERFACE_RULES, TW, twistMembers } from '../../src/design/twist-drive.js';
import { interfaceLedger } from '../../src/design/interfaces.js';
import { checkMembers } from '../../src/physics/members.js';
import { materialOf } from '../../src/design/linkage.js';
import { adhesiveOf } from '../../src/design/airframe.js';

const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const base = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const byId = (id) => catalog.find((p) => p.id === id);
const mat = (id) => materialOf(catalog, id);
const glueAl = adhesiveOf(catalog, 'adhesive-araldite-2011-aluminium'), glueCf = adhesiveOf(catalog, 'adhesive-araldite-2011');
const boom = { y: base.tail.y_m, r: byId(base.params.parts.keel).specs.od_mm / 2000 };

test('the tail pin is its bushings\' bore: 5 mm in GFM-0506-05, 6 mm in GFM-0608-06; its spigot in the tube no larger', () => {
  const G = tailMechGeometry(base.tail, boom, TAIL_MECH, catalog);
  near(G.rPin, byId('bushing-igus-GFM-0506-05').specs.d1_mm / 2000);
  near(G.rPin, 0.0025);
  // the pivot tube is still Rock West's 6 x 5 (the only one with a published shear strength): its outside, not the pin's
  near(G.rT, byId('tube-rockwest-pultruded-6x5').specs.od_mm / 2000, 1e-5);
  assert.ok(G.ri - G.run / 10 <= G.rPin, 'the spigot fits inside the pin\'s diameter');
  near(tailMechGeometry(base.tail, boom, { ...TAIL_MECH, bushing: 'bushing-igus-GFM-0608-06' }, catalog).rPin, 0.003);
});

test('a bushing whose bore is under the spigot, or a washer not for the pin\'s size, is refused', () => {
  // a hand-built 4 mm flanged bushing: its pin (4 mm) would be smaller than the 4.88 mm spigot in the tube's 4.978 mm bore
  const small = { ...byId('bushing-igus-GFM-0506-05'), id: 'test-bushing-4mm', specs: { ...byId('bushing-igus-GFM-0506-05').specs, d1_mm: 4, d2_mm: 5.5, d3_mm: 9.5 } };
  assert.throws(() => tailMechGeometry(base.tail, boom, { ...TAIL_MECH, bushing: small.id }, [...catalog, small]), /smaller than the 4\.88 mm spigot.*shoulder/);
  // a hand-built DIN 6799 washer for 4-5 mm shafts on the 6 mm pin of GFM-0608-06
  const w = { ...byId('washer-din6799-4'), id: 'test-washer-4-5', specs: { ...byId('washer-din6799-4').specs, shaft_diameter_d1_range_mm: [4, 5] } };
  assert.throws(() => tailMechGeometry(base.tail, boom, { ...TAIL_MECH, bushing: 'bushing-igus-GFM-0608-06', washer: w.id }, [...catalog, w]), /is for 4-5 mm shafts, not the 6 mm pin/);
  // the catalogue's size 4 is for 5-7 mm shafts: the 5 mm pin takes it, and so would a 7 mm one (the range is inclusive)
  assert.deepEqual(byId(TAIL_MECH.washer).specs.shaft_diameter_d1_range_mm, [5, 7]);
  const b7 = { ...byId('bushing-igus-GFM-0608-06'), id: 'test-bushing-7mm', specs: { ...byId('bushing-igus-GFM-0608-06').specs, d1_mm: 7 } };
  near(tailMechGeometry(base.tail, boom, { ...TAIL_MECH, bushing: b7.id }, [...catalog, b7]).rPin, 0.0035);
  const b75 = { ...b7, id: 'test-bushing-7.5mm', specs: { ...b7.specs, d1_mm: 7.5 } };
  assert.throws(() => tailMechGeometry(base.tail, boom, { ...TAIL_MECH, bushing: b75.id }, [...catalog, b75]), /is for 5-7 mm shafts, not the 7\.5 mm pin/);
});

const tailAx = (tm = {}) => {
  const members = [];
  const d = { tail: base.tail, params: { parts: base.params.parts, tailY_m: base.tail.y_m, tailMech: { pushrod_L_mm: 893.36, ...tm } }, servos: {} };
  tailMechMembers({ d, catalog, members, mat, glueAl, glueCf, lump: () => ({ m: 0.05, r: [-0.95, 0.16, 0] }), tieL: null });
  return Object.fromEntries(members.map((m) => [m.name, m]));
};

test('the saddle\'s webs are rated at their drawn thickness (two webs of web_t), the pin and bushings at the bushing\'s bore', () => {
  const m = tailAx();
  assert.equal(m['tail saddle webs R'].dims.t_mm, 2 * TAIL_MECH.saddle.web_t);
  // a thinner web is a thinner member (round 17 rated them at twice the sleeve's wall, whatever was drawn)
  assert.equal(tailAx({ saddle: { ...TAIL_MECH.saddle, web_t: 0.8, wall: 1.5 } })['tail saddle webs R'].dims.t_mm, 1.6);
  assert.equal(m['tail pivot pin R at the outer bushing'].dims.d_mm, 5);
  assert.equal(m['tail bushing R outer'].dims.d_mm, 5);
  // the horn's thread: the screw's 6 mm less the KBRM-02's 4 mm ball, within the 3 mm arm: 2 mm engaged
  assert.equal(m['tail horn R screw thread in the arm'].dims.Le_mm, 2);
});

test('a tapped M2 thread in an arm we machine: pi d2 Le / 2 at the material\'s shear yield (hand figure)', () => {
  const al7 = mat('metal-al-7075-t6');
  const t = tappedThreadMember('x', al7, { screw_len_mm: 6, ball_B_mm: 4, arm_t_mm: 2.5 }, [{ ch: 'q', k: 1 }], 'the lever', "the link's");
  // Le = min(6 - 4, 2.5) = 2 mm; A = pi x 1.74 x 2 / 2 = 5.4664 mm2; at 38.5 N, tau = 7.043 MPa; 280 / 7.043 = 39.76
  assert.equal(t.dims.Le_mm, 2); near(t.dims.area_mm2, 5.466, 1e-3);
  const r = checkMembers([t], { 'x/F': 38.5 }).members[0];
  near(r.fos, al7.shear_yield_MPa / (38.5 / ((Math.PI * 1.74 * 2) / 2)), 0.02);
  near(r.fos, 39.76, 0.02);
  // a thin arm limits the engagement to its own thickness
  assert.equal(tappedThreadMember('x', al7, { screw_len_mm: 6, ball_B_mm: 4, arm_t_mm: 1.2 }, [], 'w', 'f').dims.Le_mm, 1.2);
});

const withDrive = () => { const d = structuredClone(base); d.params = { ...d.params, tubeSpecSheet: true, parts: { ...d.params.parts, twistServo: 'servo-hitec-md89mw' }, twistDrive: { section: TWIST_SECTION } }; return d; };

test('the twist lever\'s tapped thread is a member on the link\'s force, in 7075, and the ledger names it on the lever\'s screw', () => {
  const D = withDrive(), members = [];
  twistMembers({ d: D, catalog, members, mat, glueAl, glueCf });
  const by = Object.fromEntries(members.map((m) => [m.name, m]));
  for (const t of ['R', 'L']) {
    const th = by[`twist elbow lever ${t} screw thread in the arm`];
    assert.ok(th, `the ${t} lever's thread member`);
    assert.equal(th.check, 'pin-shear'); assert.equal(th.material.id, 'metal-al-7075-t6');
    // the KBRM-02's 4 mm ball on an M2 x 6: 2 mm into the lever's 2.5 mm arm
    assert.equal(th.dims.Le_mm, 2);
    // its load is the link's force, as the rod ends'
    assert.deepEqual(th.sections[0].radial, by[`twist link ${t} rod ends`].sections[0].radial);
  }
  // the lever's tapped joint: rated by the screw and the lever's thread; without the thread member it fails
  const SCREW = 'screw-iso4762-m2x6-8.8';
  const asm = { parts: [{ id: 'twist lever screw R', solids: [{ name: 'shank', bom: SCREW }], bom: [SCREW] }, { id: 'twist elbow lever R', solids: [{ name: 'arm', bom: 'twist levers' }], bom: ['twist levers'] }], joins: [['twist lever screw R', 'twist elbow lever R', TW.tapped]] };
  const rated = (name, check) => ({ name, check, sections: [] });
  const dz = (ms) => ({ params: {}, structure: { members: ms } });
  const ok = interfaceLedger(asm, dz([rated('twist link R screws', 'bolts'), rated('twist elbow lever R screw thread in the arm', 'pin-shear')]), TWIST_INTERFACE_RULES);
  assert.deepEqual(ok.failed.map((r) => r.why_failed), []);
  assert.ok(ok.rows[0].axial.rating.includes('twist elbow lever R screw thread in the arm'));
  assert.match(ok.rows[0].axial.rule, /stripping area/);
  const no = interfaceLedger(asm, dz([rated('twist link R screws', 'bolts')]), TWIST_INTERFACE_RULES);
  assert.equal(no.failed.length, 1);
});

// ---- the hatch cover's screws: the rails' edge distance (EN 1993-1-8 Table 3.3, 1.2 d0) --------------------------------
test('the hatch screws\' edge distance: M3 in the 6.4 mm rails is 0.94 d0 (refused), the cover\'s M2 is 1.33 d0 (hand figures)', async () => {
  const { NEEDS, coverScrew, hatchScrewEdge } = await import('../../src/design/build-needs.js');
  const H = NEEDS.hatch, zIn = 19.1; // the nose tray's flange: the rails 19.1-25.5 mm
  const m2 = coverScrew(catalog, H), m3 = coverScrew(catalog, { ...H, cover_screws: undefined, cover_hole_mm: undefined });
  assert.equal(m2.id, 'screw-iso4762-m2x8-8.8'); assert.equal(m2.d_mm, 2); assert.equal(m2.hole_mm, 2.4);
  assert.equal(m3.id, 'screw-iso4762-m3x8-10.9'); assert.equal(m3.hole_mm, 3.4);
  // the screw on the rail's middle (z 22.3): 3.2 mm to both edges; the ends 11 mm and 27 mm away
  near(hatchScrewEdge(H, zIn, m3).eMin, 3.2, 1e-9);
  assert.equal(hatchScrewEdge(H, zIn, m3).e_over_d0, 0.941);
  assert.equal(hatchScrewEdge(H, zIn, m2).e_over_d0, 1.333);
  // the inner edge counts too: a screw moved out to 1 mm from the outer edge is 5.4 mm from the inner one, still refused
  assert.equal(hatchScrewEdge({ ...H, screw_z: 24.5 }, zIn, m2).e_over_d0, +(1 / 2.4).toFixed(3));
  // a screw 1 mm from the inner edge (the tray's flange side)
  assert.equal(hatchScrewEdge({ ...H, screw_z: 20.1 }, zIn, m2).e_over_d0, +(1 / 2.4).toFixed(3));
  // and the rail's ends: a screw 2 mm from the rail's aft end, or 1.5 mm from its forward end
  assert.equal(hatchScrewEdge({ ...H, screws_x: [136, 195] }, zIn, m2).e_over_d0, +(2 / 2.4).toFixed(3));
  assert.equal(hatchScrewEdge({ ...H, screws_x: [145, 220.5] }, zIn, m2).e_over_d0, +(1.5 / 2.4).toFixed(3));
  // the seat rule itself: unrated under 1.2 d0, rated over it
  const seat = (e) => checkMembers([{ name: 's', check: 'seat', material: mat('metal-al-6061-t6'), dims: { d_mm: 2, t_mm: 1.5, e1_over_d0: e, e2_over_d0: e }, sections: [{ name: 'F', radial: [] }], uses: { F: 'F' } }], { 's/F': 10 }).members[0];
  assert.equal(seat(0.941).fos, null); assert.equal(seat(0.941).ok, false);
  assert.ok(Number.isFinite(seat(1.333).fos));
});

test('the v25 build: the hatch on M2 with its seat rated, and one BOM line per fastener part (the M2 nuts the encoder\'s too)', async () => {
  const { buildDesign } = await import('../../src/design/build.js');
  const { round17Set, mergeDeep } = await import('../../src/design/round17.mjs');
  const { solidMassParams } = await import('../../src/design/round11.mjs');
  const set = round17Set({}, catalog);
  let p = mergeDeep(base.params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog);
  const seat = d.structure.members.find((m) => m.name === 'hatch screws in the rails');
  assert.equal(seat.dims.d_mm, 2); assert.equal(seat.dims.e1_over_d0, 1.333);
  assert.ok(Number.isFinite(checkMembers([seat], { 'hatch screws in the rails/F': 5 }).members[0].fos));
  assert.equal(d.structure.members.find((m) => m.name === 'hatch screws').fastener.id, 'screw-iso4762-m2x8-8.8');
  // every fastener part on one BOM line: the hatch's 4 M2 nuts and the encoder readhead's 3 in one line of 7
  const fast = d.bom.structure.filter((s) => /^(screw|nut)-/.test(s.material_id ?? ''));
  const ids = fast.map((s) => s.material_id);
  assert.deepEqual(ids.filter((id, i) => ids.indexOf(id) !== i), []);
  const m2n = fast.find((s) => s.material_id === 'nut-iso4032-m2-8');
  assert.match(m2n.name, /^ISO 4032 M2 hexagon nut.*, 7 off$/);
  // the key and the boom clamps as sized: a 4 x 4 key 2 mm into the stub, 3.75 mm long; 4 mm clamps
  const key = d.structure.members.find((m) => m.name === 'key, crank web');
  assert.deepEqual([key.dims.b_mm, key.dims.h_mm, key.dims.L_mm], [4, 2, 3.75]);
  assert.equal(d.params.linkage.airframe.boom.clamp_len_mm, 4);
});
