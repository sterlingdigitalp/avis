// Round 15 sub-task (SPEC "Design freeze", "Assembly clearance"; "Support and containment"): the part builders of
// src/design/clearance.js that test/design/clearance.test.js does not reach, plus the round-15 support/containment
// checks. Expected values are derived independently from the design record (params, joints.js driveLayout, the
// airframe record structure.airframe.parts, the catalogue) by hand arithmetic or by composing already-validated
// exported pure helpers (ring, poly, cap, channelSolids, riserSolids, bandSlabs, tieXs, gap2, gap3) -- never by
// re-calling a builder's own internal (non-exported) helper. carrierParts, pitotParts, readheadFixing, the
// motor-bracket block of gearParts, partTies, motorSupport/traySupport/encoderSupport/pitotSupport and
// containmentSummary are not exported, so they are exercised through assembly()/checkClearance()/supportRecord()/
// containmentRecord(), which are.
//
// No golden snapshots are used: every assertion is checked against a hand- or formula-derived value, so it fails if
// a solid moves, grows, shrinks, loses a join, or a record field is computed from the wrong inputs.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  ring, cap, poly, gap2, gap3,
  channelSolids, riserSolids, bandSlabs, tieXs,
  declaredOpening, solidLoftGaps, markExposed,
  supportRecord, containmentRecord, containmentOf, checkClearance, assembly,
  CLEARANCE_M,
} from '../../src/design/clearance.js';
import { driveLayout } from '../../src/design/joints.js';
import { round15Set } from '../../src/design/round15.mjs';
import { buildDesign } from '../../src/design/build.js';
import { packDesign, lumpPositionsFrom } from '../../src/design/fit.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const near = (a, b, tol = 1e-6) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
/** Strip the non-enumerable-free but still-own `withHoles` function boxYZ attaches to its polygons (clearance.js), so
 * a hand-built poly() of the same points deep-equals a builder's boxYZ() one. */
const plain = (v) => JSON.parse(JSON.stringify(v));
const box2 = (z0, z1, y0, y1) => poly([[z0, y0], [z1, y0], [z1, y1], [z0, y1]]);
const S = (name, x, shape) => ({ name, x, shape });

/**
 * A v23-like design (round 15), built once for the file: the frozen v21 fixture's own params deep-merged with
 * round15Set({}, catalog) (src/design/round15.mjs), then buildDesign with the two lump-settling passes -- the same
 * merge and the same two passes as the round-15 probe script (fit.js packDesign / lumpPositionsFrom), with no --fit:
 * the fixture's own pod size is kept. Some packed parts or tip circles may still clash or protrude against that pod;
 * that is fine here, since these tests check the *builders'* geometry against the design's own recorded numbers,
 * never that this particular v23-like design passes clean (that is round15.mjs's and the physics agent's job).
 */
const deepMerge = (a, b) => {
  if (!b || typeof b !== 'object' || Array.isArray(b)) return b;
  const o = { ...(a ?? {}) };
  for (const [k, v] of Object.entries(b)) o[k] = v && typeof v === 'object' && !Array.isArray(v) ? deepMerge(a?.[k], v) : v;
  return o;
};
let V23 = null;
function v23Like() {
  if (V23) return V23;
  const v21 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
  let p = deepMerge(v21.params, round15Set({}, catalog));
  for (let k = 0; k < 2; k++) {
    const d0 = buildDesign(p, catalog);
    const pk = packDesign(d0, catalog, { containment: false });
    p = { ...p, lumpPos: { ...p.lumpPos, ...lumpPositionsFrom(pk.model) } };
  }
  const d = buildDesign(p, catalog);
  const lay = driveLayout(d, catalog);
  const asm = assembly(d, catalog, d.params.linkage, lay, d.structure.airframe);
  const model = packDesign(d, catalog, { containment: false }).model;
  // phases: 2 keeps this cheap: the one pair this file reads back (two static solids) is sampled once regardless of
  // phases, and containmentOf's test below reads the containment field, which never depends on this phases count.
  const r = checkClearance(d, catalog, d.params.linkage, lay, d.structure.airframe, { all: true, phases: 2 });
  V23 = { d, lay, asm, model, r };
  return V23;
}
const partSolids = (parts, id) => { const p = parts.find((q) => q.id === id); assert.ok(p, `no part ${id}`); return p.solids; };
const pair = (r, a, b) => { const p = r.all_pairs[`${a} | ${b}`] ?? r.all_pairs[`${b} | ${a}`]; assert.ok(p, `no pair ${a} / ${b}`); return p.gap_mm; };
const R3 = 6e-4; // tolerance for a value read from the record (rounded to 0.001 mm)

// ---- pure geometry helpers (round 15): hand values, no design needed -----------------------------------------

test('channelSolids: base and two flanges as capsules of radius t/2 over the stated x', () => {
  const c = { w: 0.04, y: -0.08, h: 0.006, t: 0.0008 };
  const x = [0.1, 0.2];
  assert.deepEqual(channelSolids(c, x), [
    S('base', x, cap([-0.02, -0.08], [0.02, -0.08], 0.0004)),
    S('flange right', x, cap([0.02, -0.08], [0.02, -0.074], 0.0004)),
    S('flange left', x, cap([-0.02, -0.08], [-0.02, -0.074], 0.0004)),
  ]);
});

test('riserSolids: web, two flanges, foot and tab as boxes from the riser\'s own x, y, w, t, f, foot', () => {
  const r = { x: [0.1, 0.102], y: [-0.08, -0.04], w: 0.0376, t: 0.0008, f: 0.007, foot: 0.008 };
  const hw = r.w / 2, [x0, x1] = r.x, [y0, y1] = r.y;
  assert.deepEqual(plain(riserSolids(r)), plain([
    S('web', [x0, x1], box2(-hw, hw, y0, y1)),
    S('flange right', [x1, x1 + r.f], box2(hw - r.t, hw, y0, y1 - r.t)),
    S('flange left', [x1, x1 + r.f], box2(-hw, -hw + r.t, y0, y1 - r.t)),
    S('foot', [x0 - r.foot, x0], box2(-hw, hw, y0, y0 + r.t)),
    S('tab', [x1, x1 + r.foot], box2(-hw, hw, y1 - r.t, y1)),
  ]));
});

test('bandSlabs: four slabs round a bundle box, with the default floor and with separate floors per side', () => {
  const x = 0.1, b = 0.004, tt = 0.001, z0 = -0.01, z1 = 0.01, yb = -0.05, yt = -0.02;
  const sol = bandSlabs('tie', x, b, tt, [z0, z1], [yb, yt]);
  const xs = [x - b / 2, x + b / 2];
  assert.deepEqual(plain(sol), plain([
    S('tie top', xs, box2(z0 - tt, z1 + tt, yt, yt + tt)),
    S('tie side -z', xs, box2(z0 - tt, z0, yb, yt)),
    S('tie side +z', xs, box2(z1, z1 + tt, yb, yt)),
    S('tie bottom', xs, box2(z0 - tt, z1 + tt, yb - tt, yb)),
  ]));
  // round 15 (partTies): the two sides may start from different floors (the carrier's face on each side), not the
  // bundle's own bottom
  const floors = [-0.03, -0.028];
  const sol2 = bandSlabs('tie', x, b, tt, [z0, z1], [yb, yt], floors);
  assert.deepEqual(plain(sol2[1]), plain(S('tie side -z', xs, box2(z0 - tt, z0, floors[0], yt))));
  assert.deepEqual(plain(sol2[2]), plain(S('tie side +z', xs, box2(z1, z1 + tt, floors[1], yt))));
});

test('tieXs: n ties spread evenly over [x0, x1], each at the middle of its share', () => {
  assert.deepEqual(tieXs(0, 1, 1), [0.5]);
  assert.deepEqual(tieXs(0, 1, 2), [0.25, 0.75]);
  const xs = tieXs(0.1, 0.4, 3);
  [0.15, 0.25, 0.35].forEach((v, i) => near(xs[i], v, 1e-9));
});

test('declaredOpening: matches a bare part id (the implicit solids: [part]) or an explicit part/solid, else none', () => {
  const list = [{ name: 'a', part: 'wing', why: 'x' }, { name: 'b', solids: ['body/hatch', 'body/door'], why: 'y' }];
  assert.equal(declaredOpening(list, 'wing', 'anything').name, 'a');
  assert.equal(declaredOpening(list, 'body', 'hatch').name, 'b');
  assert.equal(declaredOpening(list, 'body', 'window'), undefined);
  assert.equal(declaredOpening(undefined, 'wing', 'x'), undefined);
});

test('solidLoftGaps: an enclosed solid\'s worst gap is the loft\'s own elliptic measure, by hand on a circular loft', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: 0, y_m: 0, width_m: 0.1, height_m: 0.1 }] }; // a 50 mm-radius circular loft over x in [0, 1]
  const xs = [0.3, 0.5, 0.7];
  const pass = solidLoftGaps(loft, [ring([0, 0], 0.01)], xs, CLEARANCE_M);
  near(pass.worst, 0.05 - 0.01);
  near(pass.clearOut, -(0.05 - 0.01));
  const fail = solidLoftGaps(loft, [ring([0, 0], 0.0497)], xs, CLEARANCE_M);
  near(fail.worst, 0.05 - 0.0497);
  assert.ok(fail.worst < CLEARANCE_M);
});

test('solidLoftGaps: an outside fitting\'s clearOut is how far clear of the skin it sits', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: 0, y_m: 0, width_m: 0.1, height_m: 0.1 }] };
  const g = solidLoftGaps(loft, [ring([0, 0], 0.07)], [0.5], CLEARANCE_M);
  near(g.worst, 0.05 - 0.07);
  near(g.clearOut, 0.07 - 0.05);
});

test('solidLoftGaps: an opening\'s window widens only over the points within `required` of the skin, by x and polar angle', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: 0, y_m: 0, width_m: 0.1, height_m: 0.1 }] };
  // a 30 x 20 mm box at z 40-70 mm, y +-10 mm: its z = 70 mm corners (71 mm from the loft centre) poke past the 50 mm
  // skin; its z = 40 mm corners (41.2 mm from centre) stay 8.8 mm inside it
  const shape = poly([[0.04, -0.01], [0.07, -0.01], [0.07, 0.01], [0.04, 0.01]]);
  const opening = { x: [Infinity, -Infinity], angle: [Infinity, -Infinity], max_out_m: -Infinity, past_loft: false };
  const g = solidLoftGaps(loft, [shape], [0.3, 0.5], CLEARANCE_M, opening);
  near(opening.x[0], 0.3); near(opening.x[1], 0.5);
  const a2 = (Math.atan2(-0.01 / 0.05, 0.07 / 0.05) * 180) / Math.PI;
  const a3 = (Math.atan2(0.01 / 0.05, 0.07 / 0.05) * 180) / Math.PI;
  near(opening.angle[0], a2, 1e-9); near(opening.angle[1], a3, 1e-9);
  near(opening.max_out_m, Math.hypot(0.07, 0.01) - 0.05, 1e-9);
  assert.equal(opening.past_loft, false);
  near(g.worst, 0.05 - Math.hypot(0.07, 0.01), 1e-9);
});

test('solidLoftGaps: an x sample past the loft\'s own range is flagged past_loft and never updates the angle window', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: 0, y_m: 0, width_m: 0.1, height_m: 0.1 }] };
  const opening = { x: [Infinity, -Infinity], angle: [Infinity, -Infinity], max_out_m: -Infinity, past_loft: false };
  const g = solidLoftGaps(loft, [ring([0, 0], 0.01)], [1.5], CLEARANCE_M, opening);
  assert.equal(opening.past_loft, true);
  near(opening.x[0], 1.5); near(opening.x[1], 1.5);
  assert.equal(opening.angle[0], Infinity); assert.equal(opening.angle[1], -Infinity); // the null-loft branch returns before touching angle
  assert.equal(g.worst, -Infinity); // gapIn is -Infinity wherever the loft has no section
});

test('markExposed: marks only the (z, y) cells inside the shape that lie outside the loft at a sampled x, by hand', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.02, height_m: 0.02 }, { x_m: 0, y_m: 0, width_m: 0.02, height_m: 0.02 }] }; // a 10 mm-radius circular loft
  const exposed = new Set();
  markExposed(exposed, loft, ring([0, 0], 0.02), [0.5], 0.02); // a 20 mm disc, cell 20 mm
  // the four cell centres at (+-10, +-10) mm sit 14.14 mm from the origin: inside the 20 mm disc, outside the 10 mm loft;
  // the next ring out, at z or y = +-30 mm, sits over 31.6 mm out: outside the 20 mm disc, so never marked
  assert.deepEqual([...exposed].sort(), ['-1,-1', '-1,0', '0,-1', '0,0']);
});

// ---- supportRecord: hand-built model/airframe fixtures (the function's own inputs are themselves the record) -----

test('supportRecord: a part flush on its carrier, CG over the contact patch, with its stated ties: passes', () => {
  const c = { x: [0, 0.1], y: 0, w: 0.04, t: 0.001 };
  const face = c.y + c.t / 2;
  const box = { x0: 0.01, x1: 0.03, y0: face, z0: -0.01, z1: 0.01, cx: 0.02, cy: face, cz: 0 };
  const model = { layout: { parts: [{ name: 'battery', box, mount: { carrier: 'forward tray' } }] } };
  const af = { carriers: { forward: c }, support: { ties: [{ part: 'battery', n: 2, tie: 'tie-x', loop_mm: 50 }] } };
  const r = supportRecord(model, { params: { linkage: { plates: { t_mm: 1.6 } } } }, af, { planes: {} });
  assert.deepEqual(r.rows[0].contact_x_mm, [10, 30]);
  assert.deepEqual(r.rows[0].contact_z_mm, [-10, 10]);
  assert.equal(r.rows[0].gap_mm, 0);
  assert.equal(r.rows[0].cg_over_contact, true);
  assert.deepEqual(r.rows[0].ties_x_mm, [15, 25]); // tieXs(0.01, 0.03, 2) in mm
  assert.equal(r.rows[0].passed, true);
  assert.equal(r.passed, true);
  assert.deepEqual(r.unsupported, []);
});

test('supportRecord: fails independently on a nonzero gap, no x overlap with the carrier, CG off the contact patch, a missing tie, or no mount', () => {
  const c = { x: [0, 0.1], y: 0, w: 0.04, t: 0.001 };
  const face = c.y + c.t / 2;
  const af = { carriers: { forward: c }, support: { ties: [{ part: 'p1', n: 2, tie: 't', loop_mm: 1 }, { part: 'p3', n: 1, tie: 't', loop_mm: 1 }, { part: 'p4', n: 1, tie: 't', loop_mm: 1 }] } };
  const d = { params: { linkage: { plates: { t_mm: 1.6 } } } };
  const model = {
    layout: {
      parts: [
        // p1: 0.3 mm off the carrier's face
        { name: 'p1', box: { x0: 0.01, x1: 0.03, y0: face + 0.0003, z0: -0.01, z1: 0.01, cx: 0.02, cy: face, cz: 0 }, mount: { carrier: 'forward tray' } },
        // p2: entirely aft of the carrier's x range: no contact overlap at all
        { name: 'p2', box: { x0: -0.05, x1: -0.02, y0: face, z0: -0.01, z1: 0.01, cx: -0.035, cy: face, cz: 0 }, mount: { carrier: 'forward tray' } },
        // p3: overhangs the carrier's +z inner edge (inner = 0.0195 m), so its box centre (z) is clipped out of the contact patch
        { name: 'p3', box: { x0: 0.01, x1: 0.03, y0: face, z0: 0.015, z1: 0.03, cx: 0.02, cy: face, cz: 0.0225 }, mount: { carrier: 'forward tray' } },
        // p4: an otherwise correct fit, but af.support.ties has an entry that isn't actually p4's own ties above (none for 'p4-notie')
      ],
    },
  };
  model.layout.parts.push({ name: 'p4-notie', box: { x0: 0.01, x1: 0.03, y0: face, z0: -0.01, z1: 0.01, cx: 0.02, cy: face, cz: 0 }, mount: { carrier: 'forward tray' } });
  model.layout.parts.push({ name: 'p5', box: { x0: 0, x1: 0.01, y0: 0, z0: 0, z1: 0, cx: 0, cy: 0, cz: 0 } }); // no stated mount
  const r = supportRecord(model, d, af, { planes: {} });
  const row = (n) => r.rows.find((x) => x.part === n);
  assert.equal(row('p1').passed, false); near(row('p1').gap_mm, 0.3);
  assert.equal(row('p2').passed, false); assert.ok(row('p2').contact_x_mm[1] <= row('p2').contact_x_mm[0]);
  assert.equal(row('p3').passed, false); assert.equal(row('p3').cg_over_contact, false); assert.equal(row('p3').gap_mm, 0);
  assert.equal(row('p4-notie').passed, false); assert.equal(row('p4-notie').fixing, null);
  assert.deepEqual(row('p5'), { part: 'p5', carrier: null, face: null, fixing: null, gap_mm: null, passed: false, note: 'no stated mount: not supported' });
  assert.deepEqual(r.unsupported.sort(), ['p1', 'p2', 'p3', 'p4-notie', 'p5']);
  assert.equal(r.passed, false);
});

test('supportRecord: the motor needs its own bracket to be considered supported, whatever the gap', () => {
  const d = { params: { linkage: { plates: { t_fwd_mm: 1.2 } } } };
  // overhung stage 1, no bracket: the gap (plate face to the motor box) comes out exactly zero, but it still fails
  const noBracket = supportRecord({ layout: { parts: [{ name: 'motor', box: { x0: 0.09, x1: 0.0994 } }] } }, d, {}, { planes: { plateFwd: 0.1, stage1: 'overhung' } });
  near(noBracket.rows[0].gap_mm, 0);
  assert.equal(noBracket.rows[0].passed, false);
  assert.equal(noBracket.rows[0].carrier, 'gearbox plate forward');
  // with a bracket: the gap is the motor box's aft end to the bracket's own aft face (mb.x[1])
  const withBracket = supportRecord({ layout: { parts: [{ name: 'motor', box: { x0: 0.052, x1: 0.054 } }] } }, d, { motor_bracket: { x: [0.05, 0.052], holes: [1, 2, 3, 4] } }, { planes: {} });
  near(withBracket.rows[0].gap_mm, 0);
  assert.equal(withBracket.rows[0].passed, true);
  assert.equal(withBracket.rows[0].carrier, 'motor bracket');
  // not overhung, no bracket: the gap is the motor box's forward end to the plate's own forward face
  const inboard = supportRecord({ layout: { parts: [{ name: 'motor', box: { x0: 0.1006, x1: 0.11 } }] } }, d, {}, { planes: { plateFwd: 0.1, stage1: 'inboard' } });
  near(inboard.rows[0].gap_mm, 0);
  assert.equal(inboard.rows[0].passed, false);
});

test('supportRecord: the encoder actuator\'s bore must match the crank shaft; the readhead lands on the riser\'s web through its spacers', () => {
  const emptyModel = { layout: { parts: [] } };
  const plates = { t_mm: 1.6 };
  const matched = supportRecord(emptyModel, { params: { linkage: { plates, crank: { shaft_od_mm: 20 } } } }, {}, { planes: { encoder: { id: { actuator: 0.02 } } } });
  near(matched.rows[0].gap_mm, 0);
  assert.equal(matched.rows[0].passed, true);
  assert.equal(matched.rows[0].part, 'crank encoder actuator');
  assert.deepEqual(matched.rows[1], { part: 'crank encoder readhead', carrier: null, face: null, fixing: null, gap_mm: null, passed: false, note: 'no stated mount: not supported' });
  // the catalogue's 8 mm actuator on a 10 mm shaft, as v21 has it: fails by the radial mismatch (-1 mm)
  const mismatched = supportRecord(emptyModel, { params: { linkage: { plates, crank: { shaft_od_mm: 10 } } } }, {}, { planes: { encoder: { id: { actuator: 0.008 } } } });
  near(mismatched.rows[0].gap_mm, (0.008 / 2 - 0.01 / 2) * 1000);
  assert.equal(mismatched.rows[0].passed, false);
  // a readhead fixing on the riser's web, its holes within the web's bounds: passes
  const d = { params: { linkage: { plates, crank: { shaft_od_mm: 20 } } } };
  const riser = { x: [0.096, 0.098], y: [-0.1, -0.04], w: 0.0376 };
  const fx = { spacer_od_mm: 4, spacer_mm: 6, holes_m: [[0, -0.07]] };
  const e = { id: { actuator: 0.02 }, readhead: [0.05, 0.09] };
  const onWeb = supportRecord(emptyModel, d, { carriers: { riser }, support: { readhead: fx } }, { planes: { encoder: e } });
  near(onWeb.rows[1].gap_mm, 0);
  assert.equal(onWeb.rows[1].lands_on_web, true);
  assert.equal(onWeb.rows[1].passed, true);
  // holes too far out in z for the riser's width: lands_on_web false even though the gap itself is fine
  const offWeb = supportRecord(emptyModel, d, { carriers: { riser }, support: { readhead: { ...fx, holes_m: [[0.02, -0.07]] } } }, { planes: { encoder: e } });
  assert.equal(offWeb.rows[1].lands_on_web, false);
  assert.equal(offWeb.rows[1].passed, false);
});

test('supportRecord: the pitot passes only when its stated ties lie within both the probe\'s and the nose tray\'s x range', () => {
  const N = { x: [0.1, 0.2], w: 0.0376, t: 0.0006 };
  const r0 = N.w / 2 + N.t / 2;
  const pm = (tieX) => ({ c_m: [r0 + 0.003, -0.05], r_m: 0.003, x_m: [0.1, 0.15], tie_x_mm: tieX, n: 2, tie: 'tie-x', loop_mm: 10, face: 'right flange' });
  const emptyModel = { layout: { parts: [] } };
  const d = { params: { linkage: { plates: { t_mm: 1.6 } } } };
  const on = supportRecord(emptyModel, d, { carriers: { nose: N }, support: { pitot: pm([120, 140]) } }, { planes: {} });
  near(on.rows[0].gap_mm, 0);
  assert.equal(on.rows[0].ties_on_carrier, true);
  assert.equal(on.rows[0].passed, true);
  const off = supportRecord(emptyModel, d, { carriers: { nose: N }, support: { pitot: pm([50, 60]) } }, { planes: {} });
  assert.equal(off.rows[0].ties_on_carrier, false);
  assert.equal(off.rows[0].passed, false);
});

// ---- containmentRecord: wiring from assembly solids into rows / openings / outside / frontal, over the already-
// validated solidLoftGaps and markExposed ---------------------------------------------------------------------

test('containmentRecord: an enclosed solid becomes a violation, an opening\'s solid gets its crossing window, an outside fitting gets its clearance; rows sort ascending', () => {
  const loft = { sections: [{ x_m: 1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: 0, y_m: 0, width_m: 0.1, height_m: 0.1 }] };
  const d = { body: { loft }, drive: {} }; // no finite amps: every solid below is static, so drivePose is never called
  const asm = {
    parts: [
      { id: 'inner', solids: [{ name: 'sol', x: [0.2, 0.8], shape: ring([0, 0], 0.01) }] }, // 40 mm inside: passes
      { id: 'tight', solids: [{ name: 'sol', x: [0.3, 0.5], shape: ring([0, 0], 0.0497) }] }, // 0.3 mm short: violates
      { id: 'hatch', solids: [{ name: 'sol', x: [0.3, 0.4], shape: poly([[0.04, -0.01], [0.07, -0.01], [0.07, 0.01], [0.04, 0.01]]) }] }, // the declared opening
      { id: 'boom', solids: [{ name: 'tube', x: [0.3, 0.4], shape: ring([0, 0], 0.07) }] }, // the declared outside fitting
    ],
  };
  const decl = { openings: [{ name: 'hatch', why: 'test', drag: false, solids: ['hatch/sol'] }], outside: [{ part: 'boom', why: 'test' }] };
  const c = containmentRecord(asm, d, { law: null, fb: null }, decl);
  assert.equal(c.solids_enclosed, 2); // only inner and tight: hatch and boom are matched elsewhere
  assert.deepEqual(c.violations.map((v) => v.solid), ['tight/sol']);
  near(c.violations[0].gap_mm, 0.3, 0.01);
  assert.equal(c.closest_enclosed[0].solid, 'tight/sol'); // sorted ascending by gap
  assert.equal(c.closest_enclosed[1].solid, 'inner/sol');
  const hatch = c.openings.find((o) => o.name === 'hatch');
  assert.deepEqual(hatch.solids, ['hatch/sol']);
  assert.equal(hatch.counts_in_drag, false);
  assert.ok(hatch.crossing); // its z = 70 mm corners do cross the skin
  const boom = c.outside.find((o) => o.part === 'boom');
  near(boom.clear_of_skin_mm, 20, 0.01); // a 70 mm ring clear of the 50 mm loft by 20 mm
  assert.deepEqual(c.outside_violations, []);
  assert.equal(c.passed, false); // the 'tight' violation fails it even though nothing else does
  assert.equal(c.required_mm, 0.5);
  // the boom (an outside fitting) stands outside the loft everywhere along its length, so it is real frontal area,
  // even though the drag-false hatch contributes none; the Cd/CdA relationship holds regardless of the exact raster
  assert.ok(c.frontal.area_mm2 > 1000);
  assert.equal(c.frontal.Cd, 1.2);
  near(c.frontal.CdA_m2, (c.frontal.Cd * c.frontal.area_mm2) / 1e6, 1e-6);
});

test('containmentOf: null without a declared containment, without linkage, or without a built airframe; otherwise matches checkClearance\'s own containment field', () => {
  assert.equal(containmentOf({ params: {} }, catalog), null);
  assert.equal(containmentOf({ params: { layout: { containment: {} } } }, catalog), null); // no linkage
  assert.equal(containmentOf({ params: { layout: { containment: {} }, linkage: {} } }, catalog), null); // no structure.airframe
  const { d, r } = v23Like(); // r.containment: containmentRecord has its own phases (72 by default), unaffected by r's own (2)
  const direct = containmentOf(d, catalog);
  assert.ok(direct);
  assert.deepEqual(direct, r.containment);
});

// ---- round-15 builders via assembly() on the v23-like design: every expected solid is rebuilt from the design's
// own airframe record (d.structure.airframe.parts) using the pure helpers validated above -------------------------

test('carrierParts: forward tray, nose tray riser, nose tray and aft tray are channelSolids/riserSolids plus a root tab, exactly as the carriers record states', () => {
  const { asm, d } = v23Like();
  const G = d.structure.airframe.parts.carriers;
  const rootTabExpected = (c, x) => S('root tab', x, box2(-c.w / 2, c.w / 2, c.y + c.t / 2, c.y + c.t / 2 + c.tab));
  // round 16: each solid tagged with its BOM line (the carrier's own structure item)
  const bom = (line, sols) => sols.map((q) => ({ ...q, bom: line }));
  assert.deepEqual(plain(partSolids(asm.parts, 'forward tray')), plain(bom('forward tray', [...channelSolids(G.forward, G.forward.x), rootTabExpected(G.forward, [G.forward.x[0], G.forward.x[0] + G.forward.t])])));
  // the riser's web has the readhead screws' 2.4 mm clearance holes (ISO 273 medium for M2)
  const rh = d.structure.airframe.parts.support.readhead.holes_m.map((c) => ({ c, r: 0.0012 }));
  assert.deepEqual(plain(partSolids(asm.parts, 'nose tray riser')), plain(bom('nose tray riser', riserSolids(G.riser, rh))));
  assert.deepEqual(plain(partSolids(asm.parts, 'nose tray')), plain(bom('nose tray', channelSolids(G.nose, G.nose.x))));
  assert.deepEqual(plain(partSolids(asm.parts, 'aft tray')), plain(bom('aft tray', [...channelSolids(G.aft, G.aft.x), rootTabExpected(G.aft, [G.aft.x[1] - G.aft.t, G.aft.x[1]])])));
});

test('pitotParts: the probe is a ring on the nose flange; its ties are bandSlabs built from the pitot and nose-tray records', () => {
  const { asm, d } = v23Like();
  const af = d.structure.airframe.parts;
  const pm = af.support.pitot, N = af.carriers.nose, s = Math.sign(pm.c_m[0]) || 1;
  assert.deepEqual(plain(partSolids(asm.parts, 'pitot-static probe')), plain([{ ...S('tube', pm.x_m, ring(pm.c_m, pm.r_m)), bom: 'pitot-static probe' }]));
  const zin = s * (N.w / 2 - N.t / 2), zout = pm.c_m[0] + s * pm.r_m, yb = N.y - N.t / 2, yt = Math.max(N.y + N.h + N.t / 2, pm.c_m[1] + pm.r_m);
  const [b, tt] = pm.band_mm.map((v) => v / 1000), top = N.y + N.t / 2;
  const zs = s > 0 ? [zin, zout] : [zout, zin], floors = s > 0 ? [top, yb] : [yb, top];
  // round 16: each tie its own item of the tie's BOM line
  const expected = pm.tie_x_mm.flatMap((x, k) => bandSlabs(`tie ${k + 1}`, x / 1000, b, tt, zs, [yb, yt], floors).map((q) => ({ ...q, bom: pm.tie, item: `pitot tie ${k + 1}` })));
  assert.deepEqual(plain(partSolids(asm.parts, 'pitot-static probe ties')), plain(expected));
});

test('readheadFixing (round 16: each screw, spacer and nut its own part): heads and shanks, spacers and nuts as rings, placed from the readhead fixing and encoder-plane records', () => {
  const { asm, d, lay } = v23Like();
  const af = d.structure.airframe.parts;
  const fx = af.support.readhead, e = lay.planes.encoder, riser = af.carriers.riser;
  // ISO 4762 M2 x 25: head 3.80 x 2.00 mm, shank 2 mm x 25 mm; ISO 4032 M2: 4.00 mm across flats x 1.60 mm
  const mm1 = (v) => v / 1000, hr = mm1(fx.screw_head.d_mm) / 2, k = mm1(fx.screw_head.k_mm), nr = mm1(fx.nut_s_mm) / Math.sqrt(3);
  near(hr, 0.0019, 1e-12); near(k, 0.002, 1e-12); near(nr, 0.004 / Math.sqrt(3), 1e-12);
  fx.holes_m.forEach((c, i) => {
    assert.deepEqual(plain(partSolids(asm.parts, `readhead screw ${i + 1}`)), plain([S('head', [e.readhead[0] - k, e.readhead[0]], ring(c, hr)), S('shank', [e.readhead[0], e.readhead[0] + 0.025], ring(c, 0.001))].map((q) => ({ ...q, bom: fx.screw }))));
    assert.deepEqual(plain(partSolids(asm.parts, `readhead spacer ${i + 1}`)), plain([{ ...S('spacer', [e.readhead[1], e.readhead[1] + mm1(fx.spacer_mm)], ring(c, mm1(fx.spacer_od_mm) / 2, mm1(fx.spacer_id_mm) / 2)), bom: 'crank encoder spacers' }]));
    assert.deepEqual(plain(partSolids(asm.parts, `readhead nut ${i + 1}`)), plain([{ ...S('nut', [riser.x[1], riser.x[1] + mm1(fx.nut_m_mm)], ring(c, nr, 0.001)), bom: fx.nut }]));
  });
  assert.equal(fx.holes_m.length, 3); // BRD01's 3 x M2
  // the shank reaches through the readhead (6.9), the spacer, the riser's 0.8 mm web and the 1.6 mm nut, and past it
  assert.ok(e.readhead[0] + 0.025 > riser.x[1] + mm1(fx.nut_m_mm));
});

test('the motor bracket (gearParts, round 15): plate as the hull polygon with its sleeve and stub holes, plus the two sleeves as rings', () => {
  const { asm, d } = v23Like();
  const mb = d.structure.airframe.parts.motor_bracket;
  const expected = [
    // round 16: the plate stops at the sleeves' outside (the sleeves fill it), so the bracket's material is counted once
    S('plate', mb.x, poly(mb.outline, [...mb.sleeves.map((q) => ({ c: q.c, r: q.ro })), { c: mb.stub.c, r: mb.stub.r }])),
    ...mb.sleeves.map((q) => S(`sleeve ${q.side}`, mb.x, ring(q.c, q.ro, q.ri))),
  ].map((q) => ({ ...q, bom: 'motor bracket' }));
  assert.deepEqual(plain(partSolids(asm.parts, 'motor bracket')), plain(expected));
  assert.deepEqual(mb.sleeves.map((q) => q.side).sort(), ['left', 'right']);
});

test('packed parts\' ties (partTies): a mounted part\'s tie band touches its own box, by hand gap from the box to the band\'s top slab', () => {
  const { d, r, model } = v23Like();
  const af = d.structure.airframe.parts;
  const battery = model.layout.parts.find((p) => p.name === 'battery');
  const c = af.carriers.nose, tr = af.support.ties.find((t) => t.part === 'battery');
  const [b, tt] = tr.band_mm.map((v) => v / 1000), face = c.y + c.t / 2;
  const x0 = tieXs(battery.box.x0, battery.box.x1, tr.n)[0];
  const band = bandSlabs('tie 1', x0, b, tt, [battery.box.z0, battery.box.z1], [face - c.t, battery.box.y1], [face, face]);
  const topSlab = band[0]; // the band's roof sits on the box's own top face (y1), so the gap to the box is zero: touching
  const boxShape = box2(battery.box.z0, battery.box.z1, battery.box.y0, battery.box.y1);
  const hand = gap3([battery.box.x0, battery.box.x1], topSlab.x, gap2(boxShape, topSlab.shape));
  near(pair(r, 'packed battery/box', 'packed battery ties/tie 1 top'), hand * 1000, R3);
  near(hand, 0, 1e-9);
});
