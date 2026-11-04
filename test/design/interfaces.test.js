// SPEC "Design freeze" -> "Function completeness": every pivot, pin, shaft, bearing, sliding or rotating fit, and
// bonded, tied, keyed or fastened joint needs a load path, axial retention and a rule for each, or it fails --
// unless round 8's RC-practice ruling (a gear/pulley/coupling shaft-hub fixing only) covers it. These tests pin
// src/design/interfaces.js's refusals against deliberately broken, hand-built fixtures (never against the
// module's own output), then run it on the frozen v24 design and pin a handful of its results against SPEC.md's
// own text and the independent audit (docs/status/audit-v24.md), which v24 is expected to fail.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fullAssembly } from '../../src/design/clearance.js';
import { interfaceLedger, seatWidths, INTERFACE_RULES } from '../../src/design/interfaces.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));

/** A minimal, otherwise-empty design: no members, no bearing joints, no mounts -- every lookup legitimately
 * comes back empty unless a test adds to it. */
const bareDesign = (over = {}) => ({
  structure: { members: [], joints: [], joints_unchecked: [], ...over.structure },
  params: { layout: { mounts: {} }, ...over.params },
});
const asmOf = (joins, parts = []) => ({ parts, joins, assumptions: [] });

test('INTERFACE_RULES is the ordered table the module exports', () => {
  assert.ok(Array.isArray(INTERFACE_RULES) && INTERFACE_RULES.length > 10);
  for (const r of INTERFACE_RULES) {
    assert.ok(r.why instanceof RegExp, 'every row matches by a why regex');
    assert.ok(typeof r.kind === 'string' && r.kind.length > 0);
    assert.ok(typeof r.load.rating === 'function');
    assert.ok(r.axial === null || typeof r.axial.rating === 'function');
  }
});

// ---- refusals on deliberately broken inputs -------------------------------------------------------------------

test('a join with a reason no INTERFACE_RULES row matches fails', () => {
  const d = bareDesign();
  const asm = asmOf([['widget', 'gadget', 'glued on with hope (not a reason this module knows)']]);
  const L = interfaceLedger(asm, d);
  assert.equal(L.rows.length, 1);
  assert.equal(L.rows[0].ok, false);
  assert.equal(L.rows[0].rc_practice, false, 'an unmatched join is never an RC-practice disclosure');
  assert.equal(L.rows[0].kind, null);
  assert.equal(L.rows[0].load, null);
  assert.equal(L.rows[0].axial, null);
  assert.equal(L.rows[0].why_failed, "no INTERFACE_RULES row matches this join's reason");
  assert.equal(L.passed, false);
  assert.deepEqual(L.failed, L.rows);
});

test('name lookups match by exact name only, never just because the design has something named', () => {
  // A design that has OTHER bearing joints and members, but not the one this join's rule names, must still fail
  // -- finding *a* joint is not the same as finding *the named* joint.
  const d = bareDesign({
    structure: {
      joints: [{ name: 'some other shaft', part_id: 'bushing-x', count: 1, supports: [] }],
      joints_unchecked: [{ name: 'yet another shaft' }],
      members: [{ name: 'an unrelated member', kind: 'tie', check: 'tie' }],
    },
  });
  const asm = asmOf([['widget shaft journal aft (bushing-made-up)', 'widget/shaft', 'journal on its shaft']]);
  const L = interfaceLedger(asm, d);
  assert.deepEqual(L.rows[0].load.rating, ['widget shaft']);
  assert.equal(L.rows[0].load.resolved[0].exists, false, 'present-but-differently-named joints/members do not count');
  assert.equal(L.rows[0].ok, false);
});

test("a rule naming a member the design doesn't have fails", () => {
  // 'journal on its shaft' names the bearing joint from the journal part's own id (bearingJointName); here it
  // names "widget shaft", which this bare design's d.structure.joints/members simply does not contain.
  const d = bareDesign();
  const asm = asmOf([['widget shaft journal aft (bushing-made-up)', 'widget/shaft', 'journal on its shaft']]);
  const L = interfaceLedger(asm, d);
  assert.equal(L.rows.length, 1);
  const row = L.rows[0];
  assert.equal(row.kind, 'shaft in bearing');
  assert.deepEqual(row.load.rating, ['widget shaft'], 'the rule did name a member');
  assert.equal(row.load.rated, false, 'but the design has nothing by that name');
  assert.equal(row.ok, false);
  assert.match(row.why_failed, /load not rated/);
});

test("a member whose own check is 'unrated' fails even though it exists", () => {
  // The 'fastened' family rates a screw group via "<prefix> screws" (bolts) and "<prefix> screws in ..." (seat).
  // Give the design exactly the member the rule will name, but mark its check 'unrated'.
  const d = bareDesign({
    structure: {
      members: [
        { name: 'forward tray screws', kind: 'fasteners', check: 'unrated' },
        { name: 'forward tray screws in the forward plate', kind: 'seat', check: 'seat' },
      ],
    },
  });
  const asm = asmOf([['forward tray screw 1', 'forward tray', 'fastened (its head or its nut on the face)']]);
  const L = interfaceLedger(asm, d);
  const row = L.rows[0];
  assert.deepEqual(row.load.rating, ['forward tray screws']);
  assert.equal(row.load.resolved[0].exists, true);
  assert.equal(row.load.rated, false, "exists, but check is 'unrated'");
  assert.equal(row.ok, false);
  assert.match(row.why_failed, /load not rated/);
});

test('an RC-practice row fails unless the named member\'s own check reads exactly \'rc practice\'', () => {
  // The RC-practice ruling (round 8) is narrower than "not unrated": SPEC Design freeze's own worked example is
  // the crank web, whose fixing is keyed + bonded but never routed through the RC-practice rule at all because a
  // crank web is not a gear, pulley or coupling. Here we instead pin the OTHER half of the scope test: even a row
  // the table *does* route through the RC-practice rule (a KHK DS1 gear's set-screw/bond fixing) must still fail
  // if the member it names is rated under some other check ('key' here) rather than 'rc practice' at full value.
  const d = bareDesign({
    structure: {
      members: [{ name: 'gear-khk-ds1-60 on widget shaft, fixing', kind: 'gear fixing', check: 'key' }],
    },
  });
  const asm = asmOf([['stage 1 wheel gear-khk-ds1-60', 'widget shaft/shaft', 'gear on its shaft (bore on the shaft or seat)']]);
  const L = interfaceLedger(asm, d);
  const row = L.rows[0];
  assert.equal(row.rc_practice, true);
  assert.deepEqual(row.load.rating, ['gear-khk-ds1-60 on widget shaft, fixing']);
  assert.equal(row.load.resolved[0].exists, true, 'the member does exist');
  assert.equal(row.load.resolved[0].rated, true, "and it isn't 'unrated' either");
  assert.equal(row.ok, false, "but RC-practice specifically needs check === 'rc practice'");
  assert.match(row.why_failed, /rc practice/);
});

test('the RC-practice ruling never covers a non-gear/pulley/coupling part (SPEC Design freeze: the crank web)', () => {
  // Independent expectation from SPEC.md ("Design freeze", the v24 result note): "The RC-practice ruling covers
  // only a gear, pulley or coupling held to its shaft... so it does not cover a crank web." So the crank web's
  // keyed+bonded fixing must never appear in the ledger's rc_practice disclosure list, and must fail (its axial
  // LOCTITE bond has no published rating and is not rescued by the ruling).
  const d = bareDesign({
    structure: {
      members: [
        { name: 'key, crank web', kind: 'key', check: 'key' },
        { name: 'key, crank web, web side', kind: 'key', check: 'key' },
      ],
    },
  });
  const asm = asmOf([['crank web', 'crank', "keyed on the shaft's aft stub, held there with LOCTITE 638"]]);
  const L = interfaceLedger(asm, d);
  const row = L.rows[0];
  assert.equal(row.kind, 'keyed');
  assert.equal(row.load.rated, true, 'the key itself is rated');
  assert.equal(row.rc_practice, false, 'the crank web is never routed through the RC-practice rule');
  assert.equal(row.axial.rating.length, 0, 'no member rates the LOCTITE bond');
  assert.equal(row.ok, false);
  assert.match(row.why_failed, /axial retention not rated/);
});

test('the spar-socket BORE rule resolves left as well as right, and neither when no side is named', () => {
  const d = bareDesign({
    structure: {
      members: [
        { name: 'spar root socket left', kind: 'seat', check: 'seat' },
        { name: 'spar root bond left', kind: 'bond', check: 'bond' },
      ],
    },
  });
  const left = interfaceLedger(asmOf([['inner spar L', 'rocker L/spar socket', 'in the other tube\'s bore (not drawn: a tube along the span is drawn as its capsule across x)']]), d);
  assert.deepEqual(left.rows[0].load.rating, ['spar root socket left']);
  assert.equal(left.rows[0].ok, true);

  // Neither "rocker" nor "inner spar" names a side at all: sparSocketSide must come back null, not a guess.
  const noSide = interfaceLedger(asmOf([['spar', 'rocker/spar socket', 'bonded in the spar socket']]), d);
  assert.deepEqual(noSide.rows[0].load.rating, []);
  assert.equal(noSide.rows[0].ok, false);
});

test("a gear-on-shaft join routed to the RC-practice rule by `b` alone still needs `a` in the expected form", () => {
  // The rule's `part` regex matches on `a` OR `b`; gearShaftInfo only parses `a`'s "stage N wheel|pinion <id>"
  // form. If a design ever names the DS1 gear only in `b`, gearShaftInfo(a, b) returns null and the row must
  // fail honestly rather than naming nothing and being silently skipped.
  const d = bareDesign();
  const asm = asmOf([['some gear part', 'gear-khk-ds1-60/shaft', 'gear on its shaft (bore on the shaft or seat)']]);
  const L = interfaceLedger(asm, d);
  const row = L.rows[0];
  assert.equal(row.rc_practice, true);
  assert.deepEqual(row.load.rating, []);
  assert.equal(row.ok, false);
});

test('a narrow seat (overlap ratio < 1) is listed by seatWidths; a full-width one is not', () => {
  const parts = [
    { id: 'journal narrow', solids: [{ name: 'body', x: [0, 0.005], shape: { kind: 'ring', c: [0, 0] } }] },
    { id: 'seat narrow', solids: [{ name: 'plate', x: [0, 0.0012], shape: { kind: 'poly' } }] },
    { id: 'journal wide', solids: [{ name: 'body', x: [0.1, 0.106], shape: { kind: 'ring', c: [0, 1] } }] },
    { id: 'seat wide', solids: [{ name: 'plate', x: [0.1, 0.102], shape: { kind: 'poly' } }, { name: 'boss', x: [0.102, 0.106], shape: { kind: 'ring', c: [0, 1] } }] },
  ];
  const asm = asmOf(
    [
      ['journal narrow', 'seat narrow', 'pressed in its plate seat'],
      ['journal wide', 'seat wide', 'pressed in its plate seat'],
    ],
    parts,
  );
  const rows = seatWidths(asm);
  assert.equal(rows.length, 1, 'only the narrow seat is listed');
  assert.equal(rows[0].journal, 'journal narrow');
  // hand calculation, independent of the module: 1.2 mm of seat material over a 5 mm body = ratio 0.24
  assert.equal(rows[0].body_mm, 5);
  assert.equal(rows[0].overlap_mm, 1.2);
  assert.equal(rows[0].ratio, 0.24);
});

test('two separate (non-overlapping) seat regions add, rather than merge into one span', () => {
  // A 6 mm body with 1 mm of seat material at its near end and 1 mm at its far end, a real 4 mm gap between them:
  // overlap must be 1 + 1 = 2 mm, not the 6 mm a buggy "just take the envelope" merge would give.
  const parts = [
    { id: 'journal gapped', solids: [{ name: 'body', x: [0, 0.006], shape: { kind: 'ring', c: [0, 0] } }] },
    {
      id: 'seat gapped',
      solids: [
        { name: 'near pad', x: [0, 0.001], shape: { kind: 'poly' } },
        { name: 'far pad', x: [0.005, 0.006], shape: { kind: 'poly' } },
      ],
    },
  ];
  const asm = asmOf([['journal gapped', 'seat gapped', 'pressed in the eye']], parts);
  const rows = seatWidths(asm);
  assert.equal(rows.length, 1);
  assert.equal(rows[0].overlap_mm, 2);
  assert.equal(rows[0].ratio, +(2 / 6).toFixed(3));
});

test('a plate boss belonging to a different shaft is not credited to this one (ring centres must match)', () => {
  // Two shafts' journals both seat in the same plate part; shaft A's own boss reaches its full body length, but
  // shaft B has no boss of its own -- only the thin base plate. Shaft A's seat is wide, shaft B's is not, even
  // though shaft A's boss solid happens to span the same x-range as shaft B's journal.
  const parts = [
    { id: 'journal A', solids: [{ name: 'body', x: [0, 0.006], shape: { kind: 'ring', c: [0, 0] } }] },
    { id: 'journal B', solids: [{ name: 'body', x: [0, 0.006], shape: { kind: 'ring', c: [0, 1] } }] },
    {
      id: 'shared plate',
      solids: [
        { name: 'plate', x: [0, 0.002], shape: { kind: 'poly' } },
        { name: 'boss A', x: [0.002, 0.006], shape: { kind: 'ring', c: [0, 0] } },
      ],
    },
  ];
  const asm = asmOf(
    [
      ['journal A', 'shared plate', 'pressed in its plate seat'],
      ['journal B', 'shared plate', 'pressed in its plate seat'],
    ],
    parts,
  );
  const rows = seatWidths(asm);
  assert.equal(rows.length, 1, 'only B (no boss of its own) is narrow');
  assert.equal(rows[0].journal, 'journal B');
  assert.equal(rows[0].ratio, +(0.002 / 0.006).toFixed(3));
});

// ---- v24: expected to fail, pinned against SPEC.md and the independent audit -----------------------------------

test('passed is exactly (no failed rows) AND (no narrow seats), checked at both edges', () => {
  const d = bareDesign();
  const empty = interfaceLedger(asmOf([]), d);
  assert.equal(empty.rows.length, 0);
  assert.deepEqual(empty.failed, []);
  assert.deepEqual(empty.seats, []);
  assert.equal(empty.passed, true, 'no joints at all is vacuously a pass');

  // One single, otherwise-unremarkable tied joint that resolves and passes cleanly (rows/failed stay as they
  // were), alongside an unrelated narrow seat pair (its own join adds one row -- also failing, since a plain
  // bearing's axial is always unrated by this table -- but the point pinned here is seatWidths' own geometry,
  // not that row's `ok`): passed must still be false because `seats` alone is non-empty.
  const d3 = bareDesign({
    structure: { members: [{ name: 'nose tray: esc ties', kind: 'tie', check: 'tie' }] },
    params: { layout: { mounts: { esc: { carrier: 'nose tray' } } } },
  });
  const asm = asmOf(
    [
      ['packed esc ties', 'packed esc', 'round the part'],
      ['journal narrow', 'seat narrow', 'pressed in its plate seat'],
    ],
    [
      { id: 'journal narrow', solids: [{ name: 'body', x: [0, 0.005], shape: { kind: 'ring', c: [0, 0] } }] },
      { id: 'seat narrow', solids: [{ name: 'plate', x: [0, 0.0012], shape: { kind: 'poly' } }] },
    ],
  );
  const L = interfaceLedger(asm, d3);
  const tieRow = L.rows.find((r) => r.a === 'packed esc ties');
  assert.equal(tieRow.ok, true, 'the tie row on its own passes cleanly');
  assert.ok(L.seats.length === 1 && L.seats[0].ratio < 1);
  assert.equal(L.passed, false, 'seatWidths alone is enough to fail passed, independent of any one row\'s own ok');
});

test("v24: a plain bearing's own load is rated via d.structure.joints, even though its axial always fails", () => {
  // Pins nameStatus's 'joint' branch directly (as opposed to only ever seeing it through the aggregate `ok`
  // field). "shoulder pivot right" names no d.structure.members entry at all (unlike "crank shaft", which is
  // ambiguously both a member AND a joint name in v24 and so would resolve via the member branch first) -- it
  // exists only as a bearing joint, a real, round-4-eligible entry in d.structure.joints, so the load side must
  // come back existing and rated specifically *via that table*, while axial still has nothing named.
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d);
  assert.equal(d.structure.members.some((m) => m.name === 'shoulder pivot right'), false, 'precondition: no member of this name');
  const row = L.rows.find((r) => r.kind === 'shaft in bearing' && r.a === 'shoulder bushing R aft');
  assert.ok(row, 'the shoulder pivot journal-on-the-pin row must be in the ledger');
  assert.deepEqual(row.load.rating, ['shoulder pivot right']);
  assert.equal(row.load.resolved[0].exists, true);
  assert.equal(row.load.resolved[0].where, 'joint');
  assert.equal(row.load.resolved[0].rated, true, "d.structure.joints' \"shoulder pivot right\" entry is rated");
  assert.equal(row.load.rated, true);
  assert.equal(row.axial.rating.length, 0);
  assert.equal(row.ok, false, 'rated load, but still fails on axial');
});

test('v24 builds an interface ledger, and it is expected to fail', () => {
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d);
  assert.ok(L.rows.length > 300, `expected a few hundred joints, got ${L.rows.length}`);
  assert.equal(L.passed, false, 'v24 predates the Function completeness round; it must not be made to pass');
  assert.ok(L.failed.length > 0);
});

test("v24: the crank web's axial bond fails, exactly as SPEC's own Design freeze note says", () => {
  // SPEC.md, "Design freeze" -> the v24 result: "The crank web's axial bond. It is LOCTITE 638 on titanium,
  // which has no published rating. The RC-practice ruling covers only a gear, pulley or coupling held to its
  // shaft... so it does not cover a crank web. M7's pass on v24 rests on that bond, and so it is not counted."
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d);
  const row = L.rows.find((r) => r.a === 'crank web' && r.b === 'crank');
  assert.ok(row, 'the crank web / crank join must be in the ledger');
  assert.equal(row.rc_practice, false);
  assert.equal(row.ok, false);
  assert.match(row.why_failed, /axial retention not rated/);
  assert.ok(!L.rc_practice.includes(row));
});

test('v24: the two KHK DS1 set-screw/bond gear fixings pass, disclosed as RC practice (audit-v24.md #12)', () => {
  // audit-v24.md's own interface table, row 12: "DS1-20 on the motor shaft, DS1-60 on gear shaft 3 | LOCTITE 406
  // + SF 770 bond (RC practice) | Same bond | Within the published figure | Disclosed, sound."
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d);
  const rc = L.rows.filter((r) => r.rc_practice);
  assert.ok(rc.length >= 2, 'both DS1 gear fixings (shaft and wheel-seat solids) must be RC-practice rows');
  for (const row of rc) {
    assert.equal(row.ok, true, `${row.a} | ${row.b} should pass as disclosed RC practice`);
    assert.ok(/gear-khk-ds1-/.test(row.a) || /gear-khk-ds1-/.test(row.b));
  }
  assert.deepEqual(L.rc_practice, rc);
});

test('v24: keyed gears on the gear shafts have no named axial retention (audit-v24.md #9, "MISSING (F4)")', () => {
  // audit-v24.md interface #9: "Gears on gear shafts 1 to 3 and the crank wheel | Keys (6 key lines), torque
  // only | Nothing. Keys do not retain axially... MISSING (F4)."
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d);
  const keyedGearRows = L.rows.filter((r) => r.kind === 'keyed' && /^stage \d+ (wheel|pinion) gear-khk-(?!ds1-)/.test(r.a));
  assert.ok(keyedGearRows.length >= 6, `expected at least 6 keyed-gear joints, got ${keyedGearRows.length}`);
  for (const row of keyedGearRows) {
    assert.equal(row.load.rated, true, `${row.a}: the key itself should be rated`);
    assert.equal(row.axial.rating.length, 0, `${row.a}: no axial member should be named`);
    assert.equal(row.ok, false);
  }
});

test('v24: seatWidths reproduces the audit\'s own numbers for the SKF 628/8 and 619/5 seats (audit-v24.md #10/#11)', () => {
  // audit-v24.md: "Gear shaft 3 in the plates | SKF 628/8-2Z (8x16x5)... seated over 2.0 mm (aft) and 1.2 mm
  // (forward) of its 5 mm width" and "Motor shaft outboard | SKF 619/5-2Z (5x13x4)... 1.2 of 4 mm seated."
  const d = v24();
  const { asm } = fullAssembly(d, catalog);
  const rows = seatWidths(asm);
  const gs3aft = rows.find((r) => /gear shaft 3 journal aft/.test(r.journal));
  const gs3fwd = rows.find((r) => /gear shaft 3 journal forward/.test(r.journal));
  const motor = rows.find((r) => /motor shaft outboard/.test(r.journal));
  assert.ok(gs3aft && gs3fwd && motor, 'all three audited seats must appear among the narrow rows');
  assert.equal(gs3aft.body_mm, 5); assert.equal(gs3aft.overlap_mm, 2);
  assert.equal(gs3fwd.body_mm, 5); assert.equal(gs3fwd.overlap_mm, 1.2);
  assert.equal(motor.body_mm, 4); assert.equal(motor.overlap_mm, 1.2);
  assert.ok(rows.every((r) => r.ratio < 1));
});
