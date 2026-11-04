// Round 18: the maker's-part ruling (SPEC "Maker's part", operator's decision 2026-11-03) and the encoder actuator's
// rated fixing. The ledger admits exactly the listed interfaces (the maker's arm on its own servo's spline with its centre
// screw and tapped holes; the motor's own bearings and its mounting threads) and refuses a part we design or machine, a
// joint between our part and a bought part, and a maker's part whose condition fails. The actuator is bonded on a land
// of the crank (3M DP490 on titanium and aluminium, at half, inside its bond line); RLS's set screws alone are refused.
// Every refusal is pinned against a deliberately broken, hand-built input; the figures against hand calculations.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { interfaceLedger, makerAdmission, MAKERS_PARTS, RETAIN_INTERFACE_RULES, MOTOR_BEARINGS, MOTOR_THREADS, ACTUATOR_BOND, ACTUATOR_BONDED } from '../../src/design/interfaces.js';
import { TWIST_INTERFACE_RULES, TW } from '../../src/design/twist-drive.js';
import { TAIL_INTERFACE_RULES } from '../../src/design/tail-mech.js';
import { checkMembers, makerPartCheck, gapBondCheck } from '../../src/physics/members.js';
import { motorFigures, servoArmMaker, ratingsRecord } from '../../src/design/chains.js';
import { driveMembers, actuatorBond } from '../../src/design/linkage.js';
import { fullAssembly } from '../../src/design/clearance.js';
import { driveLayout } from '../../src/design/joints.js';
import { RETENTION } from '../../src/design/round17-retention.mjs';

const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const SERVO = 'servo-hitec-md89mw', HORN = 'horn-hitec-r-ml25', MOTOR = 'motor-scorpion-sii-3008-1090', KBRM = 'rodend-igus-KBRM-02', SCREW = 'screw-iso4762-m2x8';
const RULES = [...TAIL_INTERFACE_RULES, ...TWIST_INTERFACE_RULES, ...RETAIN_INTERFACE_RULES];

// ---- a hand-built design and assembly -------------------------------------------------------------------------------
const armMk = (cond) => ({ part: HORN, of: SERVO, covers: "the maker's arm", report: ['F', 'T'], condition: cond ?? [{ figure: 'servo torque at its spline', of: 'T', published: 0.6, units: 'N m' }] });
const motorMk = (cond) => ({ part: MOTOR, of: MOTOR, covers: "the motor's own", report: ['F'], condition: cond ?? [{ figure: 'voltage (cells)', published: 4, within: true }, { figure: 'current', published: 26, within: null }] });
const mk = (name, maker) => ({ name, kind: 'arm', check: "maker's part", maker, sections: [{ name: 'F', radial: [] }, { name: 'T', radial: [] }], uses: { F: 'F', T: 'T' } });
const rated = (name, check = 'rated') => ({ name, check, sections: [] });
const MEMBERS = () => [
  mk('twist servo arm R', armMk()), mk('tail servo arm R', armMk()), mk(MOTOR_THREADS, motorMk()), mk(MOTOR_BEARINGS, motorMk()),
  rated('twist link R screws', 'bolts'), rated('twist link R rod ends'), rated('twist elbow lever R arm', 'plate'), rated('tail pushrod R screws', 'bolts'), rated('tail pushrod R rod ends'), rated('tail horn R screw thread in the arm', 'pin-shear'),
  rated('motor mount screws', 'bolts'), rated('motor mount screws in the bracket', 'seat'), rated('motor shaft outboard bearing forward, outer ring bond', 'bond'), rated('motor bracket sleeve bond', 'lap'), rated('crank web axial screw', 'bolts'),
];
const design = (members = MEMBERS(), over = {}) => ({
  params: {}, motor: { id: MOTOR }, servos: { twist: { id: SERVO }, tail: { id: SERVO } },
  bom: { parts: [{ id: HORN, role: "twist servo arms (the maker's)" }, { id: HORN, role: "tail servo arms (the maker's)" }, { id: SERVO, role: 'wing twist servo' }, { id: MOTOR, role: 'flap motor' }, { id: KBRM, role: 'rod ends' }] },
  structure: { members, joints: [{ name: 'motor shaft outboard' }], joints_unchecked: [{ name: 'motor shaft' }] }, ...over,
});
const P = (id, bom, solids = ['body']) => ({ id, solids: solids.map((name) => ({ name, bom })), bom: [bom] });
const MO = 'motor shaft outboard journal forward (bearing-skf-619-5-2z)';
const PARTS = [
  P('twist servo arm R', HORN), P('twist servo R', SERVO), P('twist servo arm screw R', SCREW), P('twist rod end R servo', KBRM), P('twist rod end R lever', KBRM), P('twist elbow lever R', 'twist levers'),
  P('tail servo arm R', HORN), P('packed servo_tail_R', SERVO), P('tail servo arm screw R', SCREW), P('tail horn R', 'tail horns'), P('tail horn screw R', SCREW), P('tail rod end R horn', KBRM),
  P('motor', MOTOR, ['can', 'shaft']), P('motor bracket', 'motor bracket'), P(MO, 'bearing-skf-619-5-2z'), P('shoulder rail right', 'shoulder rails'), P('crank web axial screw', 'screw-iso10642-m3x8-10.9'), P('crank web', 'crank web and pin'),
];
const LISTED = [
  ['twist servo arm R', 'twist servo R', TW.spline],
  ['twist servo arm screw R', 'twist servo arm R', TW.tapped],
  ['twist rod end R servo', 'twist servo arm R', TW.ball],
  ['tail servo arm R', 'packed servo_tail_R', "on the servo's output spline (the maker's arm and its centre screw)"],
  ['tail servo arm screw R', 'tail servo arm R', "screwed into the arm's tapped hole"],
  ['motor bracket', 'motor', 'screwed into its back-face threads'],
  [MO, 'motor/shaft', 'journal on its shaft'],
];
const ledger = (joins, d = design(), opts) => interfaceLedger({ parts: PARTS, joins }, d, RULES, opts);

test('(i) the ledger admits each listed maker\'s part, and lists it with the rows that rest on it', () => {
  const L = ledger(LISTED);
  assert.deepEqual(L.failed.map((r) => `${r.a} | ${r.why}: ${r.why_failed}`), []);
  assert.equal(L.makers_parts.length, LISTED.length);
  const names = L.rows.map((r) => r.maker_part.map((q) => q.name));
  assert.deepEqual(names, [['twist servo arm R'], ['twist servo arm R'], ['twist servo arm R'], ['tail servo arm R'], ['tail servo arm R'], [MOTOR_THREADS], [MOTOR_BEARINGS]]);
  // the rule shown on those rows is the ruling's, and the bolts and rod ends (bought, on our linkage) keep their own
  assert.match(L.rows[0].load.rule, /Maker's part/);
  assert.match(L.rows[1].load.rule, /ISO 898-1/);
  assert.match(L.rows[5].load.rule, /ISO 898-1/);
  assert.deepEqual(L.rows[2].load.rating, ['twist link R rod ends', 'twist link R screws', 'twist servo arm R']);
  // exactly four kinds of interface are on the list
  assert.equal(MAKERS_PARTS.length, 4);
});

test('(ii) a part we design or machine marked maker\'s part is refused: the tail horn, the crank web\'s screw, the motor bracket', () => {
  const ours = [
    ['tail horn screw R', 'tail horn R', "screwed into the arm's tapped hole", 'tail horn R screw thread in the arm', 'tail horns'],
    ['motor bracket', 'shoulder rail right', 'its sleeve bonded on the rail', 'motor bracket sleeve bond', 'motor bracket'],
    ['crank web axial screw', 'crank web', "fastened (its countersunk head in the lip's countersink)", 'crank web axial screw', 'crank web and pin'],
  ];
  for (const [a, b, why, name, part] of ours) {
    const d = design(MEMBERS().map((m) => (m.name === name ? mk(name, { part, of: part, condition: [{ figure: 'x', within: true }] }) : m)));
    const L = ledger([[a, b, why]], d);
    assert.equal(L.failed.length, 1, name);
    assert.match(L.failed[0].why_failed, /maker's part refused.*admits only/, name);
  }
  // a listed name whose stated maker's part is our own part is refused too
  const wrong = design(MEMBERS().map((m) => (m.name === 'twist servo arm R' ? mk(m.name, { ...armMk(), part: 'twist levers' }) : m)));
  assert.match(ledger([LISTED[0]], wrong).failed[0].why_failed, /not the bought part the ruling names/);
});

test('(iii) a joint between our part and a bought part keeps every rule: the rod end on our horn, the servo arm named on our lever', () => {
  // the KBRM rod end (bought) on our 6061 horn, its rating marked maker's part: not on the list
  const d = design(MEMBERS().map((m) => (m.name === 'tail pushrod R rod ends' ? mk(m.name, { part: KBRM, of: KBRM, condition: [{ figure: 'x', within: true }] }) : m)));
  const L = ledger([['tail rod end R horn', 'tail horn R', "its ball on the horn's arm, on its screw"]], d);
  assert.equal(L.failed.length, 1);
  assert.match(L.failed[0].why_failed, /admits only/);
  // a rule naming the servo arm's maker's part on the rod end's joint with our lever: the joint is not on the arm
  const r = { why: /^its ball on the arm, on its screw$/, part: /lever/, kind: 'rod end', load: { by: 'x', rating: () => ['twist link R rod ends', 'twist servo arm R'], rule: 'x' }, axial: { by: 'x', rating: () => ['twist link R screws'], rule: 'x' } };
  const L2 = interfaceLedger({ parts: PARTS, joins: [['twist rod end R lever', 'twist elbow lever R', TW.ball]] }, design(), [r]);
  assert.match(L2.failed[0].why_failed, /not on the maker's part/);
  // the real twist rules name our lever's arm there, not the servo's
  const L3 = ledger([['twist rod end R lever', 'twist elbow lever R', TW.ball]]);
  assert.equal(L3.failed.length, 0);
  assert.equal(L3.rows[0].maker_part, undefined);
});

test('(iv) a maker\'s part whose condition fails is refused: the servo over its own rule, the motor over a published figure', () => {
  const d = design();
  // the arm's own check: at the stall it passes (no factor), over it it fails; it never sets the members' minimum
  const arm = d.structure.members.find((m) => m.name === 'twist servo arm R');
  const at = (T) => checkMembers([arm, { name: 'plate', check: 'rated', dims: { rating_N: 30 }, sections: [{ name: 'F', radial: [] }], uses: { F: 'F' } }], { 'twist servo arm R/F': 30, 'twist servo arm R/T': T, 'plate/F': 10 });
  const ok = at(0.6), over = at(0.61);
  assert.equal(ok.members[0].ok, true); assert.equal(ok.members[0].fos, null); assert.equal(ok.members[0].status, "unrated, maker's part");
  assert.deepEqual(ok.members[0].peaks, { F: 30, T: 0.6 });
  assert.equal(ok.fos_min, 3); // the plate's 30 / 10, not the arm's
  assert.equal(over.members[0].ok, false); assert.equal(over.ok, false);
  // the ledger, given the flight's member rows, refuses the servo's spline row over the rule and admits it at it
  assert.equal(ledger([LISTED[0]], d, { memberRows: ok.members }).failed.length, 0);
  assert.match(ledger([LISTED[0]], d, { memberRows: over.members }).failed[0].why_failed, /condition fails/);
  // the motor on a pack above its published cells
  const d5 = design(MEMBERS().map((m) => (m.name === MOTOR_THREADS ? mk(m.name, motorMk([{ figure: 'voltage (cells)', published: 4, within: false }])) : m)));
  assert.match(ledger([LISTED[5]], d5).failed[0].why_failed, /condition fails \(voltage \(cells\)\)/);
  // a maker's part with no condition stated is not valid
  assert.equal(makerPartCheck({ maker: { part: HORN } }, () => 0).ok, false);
  assert.match(makerAdmission(design(MEMBERS().map((m) => (m.name === MOTOR_BEARINGS ? mk(m.name, { part: MOTOR, of: MOTOR, condition: [] }) : m))), MO, 'motor/shaft', MOTOR_BEARINGS).why, /no condition/);
});

test('the motor\'s published figures: current to M8, power and cells decided here (Scorpion: 26 A, 370 W, 4S), temperature unchecked', () => {
  const f = (cells) => Object.fromEntries(motorFigures({ motor: { id: MOTOR }, battery: { cells } }, catalog).map((c) => [c.figure, c]));
  const s3 = f(3);
  assert.equal(s3.current.published, 26); assert.equal(s3.current.within, null);
  // 3 x 4.20 V x 26 A = 327.6 W within 370 W; on 4S, 436.8 W could exceed it: refused
  assert.equal(s3.power.published, 370); assert.equal(s3.power.within, true); assert.match(s3.power.basis, /327\.6 W/);
  assert.equal(f(4).power.within, false);
  assert.equal(s3['voltage (cells)'].published, 4); assert.equal(s3['voltage (cells)'].within, true);
  assert.equal(f(5)['voltage (cells)'].within, false);
  assert.equal(s3.speed.published, null); assert.equal(s3.temperature.within, null);
  // the ratings record carries the two decided here
  const rr = ratingsRecord({ motor: { id: MOTOR }, battery: { cells: 5 }, servos: {} }, catalog);
  assert.deepEqual(rr.failed, [`motor ${MOTOR}: power at its continuous current, at most 21 V x 26 A = 546.0 W`, `motor ${MOTOR} on a 5S pack`]);
});

test('the servo arm\'s condition is its servo\'s rule at the published row for the supply (MD89MW: 0.5198 / 0.6276 N m at 4.8 / 6.0 V)', () => {
  const d = (lo, hi) => ({ servos: { twist: { id: SERVO, stall_Nm: 0.4843, servo_stall_Nm: 0.60604, volts: lo } }, electrical: { servo_supply: { performance_V: lo, current_V: hi } } });
  const m = servoArmMaker(d(5.76, 6.24), catalog, 'twist', HORN);
  // between the 4.8 V and 6.0 V rows at 5.76 V: 0.5198 + 0.8 x (0.6276 - 0.5198) = 0.60604 N m
  near(m.condition[0].published, 0.5198 + 0.8 * (0.6276 - 0.5198), 1e-9);
  assert.equal(m.condition[0].of, 'T'); assert.equal(m.condition[1].within, true);
  // a supply above the top published row (7.4 V): the condition fails, so the arm fails
  const hot = servoArmMaker(d(8.0, 8.2), catalog, 'twist', HORN);
  assert.equal(hot.condition[1].within, false);
  assert.equal(makerPartCheck({ maker: hot }, () => 0.1).ok, false);
});

// ---- the encoder actuator ---------------------------------------------------------------------------------------------
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
function withRetention(ret = {}) {
  const d = v24(), R = RETENTION.linkage;
  d.params = { ...d.params, layout: { ...d.params.layout, ...RETENTION.layout }, linkage: { ...d.params.linkage, crank: { ...d.params.linkage.crank, ...R.crank }, retention: { ...R.retention, ...ret }, plates: { ...d.params.linkage.plates, ...R.plates }, rocker: { ...d.params.linkage.rocker, ...R.rocker } } };
  return d;
}

test('a bond is rated only inside its maker\'s bond line: DP490 at half its titanium row (2901 psi), the torque against a hand figure', () => {
  const adh = { id: 'dp490', shear_MPa: (2901 * 0.00689476) / 2 };
  near(adh.shear_MPa, 10.0, 0.002);
  const dims = { d_mm: 9.8, L_mm: 9.5 - (3 * 1.5 ** 2) / 9.8, gap_mm: [0.1, 0.1175], gap_limits_mm: [0.0762, 0.127] };
  // 1 N m on a 9.8 mm x 8.811 mm bond: 2T/d = 204.1 N over 271.27 mm2 = 0.7524 MPa; 10.0 / 0.7524 = 13.29
  const r = gapBondCheck(dims, adh, { T: 1, F: 0 });
  near(r.fos, adh.shear_MPa / (204.0816 / (Math.PI * 9.8 * dims.L_mm)), 1e-3);
  near(r.fos, 13.29, 0.01);
  // RLS's own H7 bore on a g6 shaft leaves 2.5-14.5 um: the data sheet's figure does not apply there
  const tight = gapBondCheck({ ...dims, gap_mm: [0.0025, 0.0145] }, adh, { T: 1e-3, F: 0 });
  assert.equal(tight.fos, null); assert.equal(tight.ok, false);
  // an adhesive with no figure on the pair (LOCTITE 638 publishes none on titanium) leaves the bond unrated
  const none = gapBondCheck(dims, { id: 'x', shear_MPa: NaN }, { T: 1e-3, F: 0 });
  assert.equal(none.fos, null); assert.equal(none.ok, false);
  const d = withRetention({ actuatorBond: { adhesive: 'adhesive-loctite-638' } }), lay = driveLayout(d, catalog);
  const b = driveMembers(d, catalog, d.params.linkage, lay, d.structure.joints).members.find((m) => m.name === ACTUATOR_BOND);
  assert.ok(Number.isNaN(b.material.shear_MPa));
  assert.equal(checkMembers([b], {}).ok, false);
});

test('the actuator is bonded on a land of the crank: the member, the drawn land, and its ledger rows; the set screws alone are refused', () => {
  const d = withRetention(), lay = driveLayout(d, catalog);
  assert.ok(lay.planes.encoder, 'the design draws the encoder');
  const { members } = driveMembers(d, catalog, d.params.linkage, lay, d.structure.joints);
  const b = members.find((m) => m.name === ACTUATOR_BOND);
  assert.equal(b.check, 'gap bond');
  // the gap: (10.000 - 9.800) / 2 = 0.100 mm to (10.015 - 9.780) / 2 = 0.1175 mm, inside DP490's 3-5 mil
  assert.deepEqual(b.dims.gap_mm, [0.1, 0.1175]); assert.deepEqual(b.dims.gap_limits_mm, [0.076, 0.127]);
  near(b.material.shear_MPa, 10.0, 1e-9); // the lower row (titanium, 20.0 MPa) at half
  // its torque is the actuator's J on the crank's: 5.7 g at 11 mm over rotor J G^2
  near(b.sections[0].radial[0].k, (0.0057 * 0.011 ** 2) / (d.motor.J_kg_m2 * d.drive.gearRatio ** 2 + (d.drive.J_crank_kg_m2 ?? 0)), 1e-9);
  assert.ok(b.fixed.T > 0);
  // the drawn land under the actuator, and the actuator's join names the bond
  const { asm } = fullAssembly(d, catalog);
  const land = asm.parts.find((p) => p.id === 'crank').solids.find((s) => s.name === 'shaft land');
  assert.ok(land, 'a shaft land is drawn');
  const aj = asm.joins.filter(([a]) => a === 'crank encoder actuator');
  assert.ok(aj.length > 0 && aj.every(([, , why]) => why === ACTUATOR_BONDED));
  const withB = { params: {}, structure: { members, joints: d.structure.joints } };
  assert.equal(interfaceLedger({ parts: [], joins: aj }, withB, RETAIN_INTERFACE_RULES).failed.length, 0);
  // the bonded joint without its bond member fails; the set screws alone (actuatorBond null) fail
  assert.equal(interfaceLedger({ parts: [], joins: aj }, { params: {}, structure: { members: members.filter((m) => m.name !== ACTUATOR_BOND) } }, RETAIN_INTERFACE_RULES).failed.length, aj.length);
  const d0 = withRetention({ actuatorBond: null });
  assert.equal(actuatorBond(d0.params.linkage), null);
  const { asm: a0 } = fullAssembly(d0, catalog);
  const sj = a0.joins.filter(([a]) => a === 'crank encoder actuator');
  assert.ok(sj.length > 0 && sj.every(([, , why]) => why === "on the crank shaft by its three set screws (maker's fixing)"));
  assert.equal(a0.parts.find((p) => p.id === 'crank').solids.some((s) => s.name === 'shaft land'), false);
  const L0 = interfaceLedger({ parts: [], joins: sj }, withB, RETAIN_INTERFACE_RULES);
  assert.equal(L0.failed.length, sj.length);
  assert.match(L0.failed[0].why_failed, /no rated member named/);
});

// ---- the round-17 build (v25) with every round-18 piece in place -------------------------------------------------------
test('the v25 build: the four maker\'s parts with the servo\'s and the motor\'s figures, the actuator bonded, and the ledger admits them', async () => {
  const { buildDesign } = await import('../../src/design/build.js');
  const { round17Set, mergeDeep } = await import('../../src/design/round17.mjs');
  const { solidMassParams } = await import('../../src/design/round11.mjs');
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), by = (n) => d.structure.members.find((m) => m.name === n);
  for (const t of ['R', 'L']) {
    for (const role of ['twist', 'tail']) {
      const arm = by(`${role} servo arm ${t}`), sv = d.servos[role];
      assert.equal(arm.check, "maker's part");
      // its torque at the spline is the surface's torque times the linkage ratio, held to the stall at the supply's row
      assert.equal(arm.sections.find((s) => s.name === 'T').radial[0].k, sv.linkage_ratio);
      near(arm.maker.condition[0].published, sv.servo_stall_Nm, 1e-5);
      assert.equal(arm.maker.condition[1].within, true);
    }
  }
  // the tail arm's hole force is the hinge moment over the horn's radius; its spline torque the hinge moment x 16/13
  near(by('tail servo arm R').sections.find((s) => s.name === 'T').radial[0].k, 16 / 13, 1e-6);
  for (const n of [MOTOR_THREADS, MOTOR_BEARINGS]) assert.equal(by(n).maker.part, MOTOR);
  assert.equal(by(ACTUATOR_BOND).check, 'gap bond');
  // the arms no longer set the members' minimum: at loads the old rule failed (38.5 N at the hole, 40% of stall), the
  // arm passes while its servo is within its stall at the spline, and fails over it
  const peaks = (T) => Object.fromEntries(d.structure.members.flatMap((m) => m.sections.map((s) => [`${m.name}/${s.name}`, m.name === 'twist servo arm R' ? (s.name === 'T' ? T : 38.5) : 0])));
  const arm = (T) => checkMembers(d.structure.members, peaks(T)).members.find((r) => r.name === 'twist servo arm R');
  assert.equal(arm(0.47).ok, true); assert.equal(arm(0.47).fos, null);
  assert.equal(arm(0.61).ok, false);
  // the ledger on the drawn assembly: every maker's-part row admitted, the actuator's rows on its bond
  const { asm } = fullAssembly(d, catalog);
  const L = interfaceLedger(asm, d, RULES);
  const mine = L.rows.filter((r) => r.maker_part || /crank encoder actuator/.test(r.a));
  assert.deepEqual(mine.filter((r) => !r.ok).map((r) => `${r.a} | ${r.why}: ${r.why_failed}`), []);
  assert.equal(L.makers_parts.length, 12); // 2 x (spline, ball, tapped) twist; 2 x (spline, tapped) tail; the motor's 2
  assert.ok(mine.some((r) => r.why === ACTUATOR_BONDED));
});

test('the list\'s own tests: the arm must be its own servo\'s and in the BOM as the maker\'s arm; the motor\'s must name the motor both ways', () => {
  const swap = (name, maker, d0 = {}) => design(MEMBERS().map((m) => (m.name === name ? mk(name, maker) : m)), d0);
  const why = (d, j, n) => makerAdmission(d, j[0], j[1], n, { bomsOf: (x) => (PARTS.find((p) => p.id === x.split('/')[0])?.bom ?? []) });
  // the arm of another servo, or one the BOM does not carry as the maker's arm for that servo
  assert.equal(why(swap('twist servo arm R', { ...armMk(), of: 'servo-other' }), LISTED[0], 'twist servo arm R').ok, false);
  const noRole = design(MEMBERS(), { bom: { parts: [{ id: HORN, role: 'twist servo arms' }] } });
  assert.equal(why(noRole, LISTED[0], 'twist servo arm R').ok, false);
  assert.equal(why(design(MEMBERS(), { servos: { twist: { id: 'servo-other' }, tail: { id: SERVO } } }), LISTED[0], 'twist servo arm R').ok, false);
  assert.equal(why(design(), LISTED[0], 'twist servo arm R').ok, true);
  // the motor's: its part and its "of" both the design's motor, and a motor in the design
  assert.equal(why(swap(MOTOR_THREADS, { ...motorMk(), of: 'other' }), LISTED[5], MOTOR_THREADS).ok, false);
  assert.equal(why(swap(MOTOR_THREADS, { ...motorMk(), part: 'other' }), LISTED[5], MOTOR_THREADS).ok, false);
  assert.equal(why(design(MEMBERS(), { motor: {} }), LISTED[5], MOTOR_THREADS).ok, false);
  assert.equal(why(design(), LISTED[5], MOTOR_THREADS).ok, true);
  // not marked maker's part: no admission at all; marked but with no maker block or no condition: refused
  assert.equal(makerAdmission(design(), 'a', 'b', 'twist link R screws'), null);
  assert.equal(makerAdmission(design(), 'a', 'b', 'no such member'), null);
  const bare = why(swap('twist servo arm R', undefined), LISTED[0], 'twist servo arm R');
  assert.equal(bare.ok, false); assert.match(bare.why, /none named/);
  // a design without servos, a BOM or a motor refuses them (no crash); a motor member with no maker block too
  const strip = (k) => { const d = design(); delete d[k]; return d; };
  assert.equal(why(strip('servos'), LISTED[0], 'twist servo arm R').ok, false);
  assert.equal(why(strip('bom'), LISTED[0], 'twist servo arm R').ok, false);
  assert.equal(why(strip('motor'), LISTED[5], MOTOR_THREADS).ok, false);
  assert.equal(why(swap(MOTOR_THREADS, undefined), LISTED[5], MOTOR_THREADS).ok, false);
  const noCond = why(swap(MOTOR_BEARINGS, { ...motorMk(), condition: [] }), LISTED[6], MOTOR_BEARINGS);
  assert.equal(noCond.ok, false); assert.match(noCond.why, /no condition stated/);
});

test('the joint is matched to the maker\'s part by the named solid\'s own tag, not by any solid of its part', () => {
  // a part drawn from two tags: the joint on its 'shaft' solid (the motor's) is on the maker's part, on its 'cap' it is not
  const parts = [...PARTS.filter((p) => p.id !== 'motor'), { id: 'motor', solids: [{ name: 'cap', bom: 'cap tag' }, { name: 'shaft', bom: MOTOR }], bom: ['cap tag', MOTOR] }];
  const on = (sol) => interfaceLedger({ parts, joins: [[MO, `motor/${sol}`, 'journal on its shaft']] }, design(), RULES);
  assert.equal(on('shaft').failed.length, 0);
  assert.match(on('cap').failed[0].why_failed, /not on the maker's part/);
});

test('the bond line\'s limits are inclusive and exact; a stated constant load adds to every section it names', () => {
  const adh = { id: 'a', shear_MPa: 10 }, base = { d_mm: 10, L_mm: 10, gap_limits_mm: [0.076, 0.127] };
  const at = (g) => gapBondCheck({ ...base, gap_mm: g }, adh, { T: 0.01, F: 0 }).ok;
  assert.equal(at([0.076, 0.127]), true);
  assert.equal(at([0.076 - 1e-6, 0.1]), false);
  assert.equal(at([0.1, 0.127 + 1e-6]), false);
  assert.equal(gapBondCheck({ ...base, gap_mm: [0.1, 0.11] }, undefined, { T: 0.01 }).ok, false);
  // m.fixed: { M, T, V, F } added to the sections' peaks (here none), each on its own load
  const m = { name: 'x', check: 'bond', material: adh, dims: { d_mm: 10, L_mm: 10 }, sections: [{ name: 'T', radial: [] }], uses: { M: 'T', T: 'T', V: 'T', F: 'T' }, fixed: { M: 1, T: 2, V: 3, F: 4 } };
  assert.deepEqual(checkMembers([m], { 'x/T': 0.5 }).members[0].load, { M: 1.5, T: 2.5, V: 3.5, F: 4.5 });
  // the maker's part's reason names what it checked here and what it left to another gate
  const r = makerPartCheck({ maker: { condition: [{ figure: 'a', of: 'T', published: 1 }, { figure: 'b', within: null }, { figure: 'c', within: true }] } }, () => 0.5);
  assert.equal(r.why, "within its maker's published figures (2 checked here, 1 elsewhere or not published)");
  const o = makerPartCheck({ maker: { condition: [{ figure: 'torque', of: 'T', published: 1, units: 'N m' }] } }, () => 1.5);
  assert.equal(o.why, "outside its maker's figures: torque 1.5000 > 1 N m");
});

test('the motor\'s figures from a hand-built catalogue: the thermal entry when there is one, the pack\'s charge state, the edges', () => {
  const mot = { id: 'motor-x', specs: { continuous_current_a: 10, continuous_power_w: 126, cells_text: 'Max Lipo Cell 3s', kv_rpm_per_v: 1000 } };
  const th = { id: 'thermal-ref-x', specs: { wire_rating_c: 180, magnet_rating_c: 200, wire_rating_text: 'w', magnet_rating_text: 'm' } };
  const f = (bt, cat = [th, mot]) => Object.fromEntries(motorFigures({ motor: { id: 'motor-x' }, battery: bt }, cat).map((c) => [c.figure, c]));
  // 3 x 4.20 V x 10 A = 126 W: exactly the published power, within; the published 3S on a 3S pack, within
  const a = f({ cells: 3 });
  assert.equal(a.power.within, true); assert.equal(a['voltage (cells)'].within, true);
  assert.equal(a.temperature.published, 'wire 180 C, magnets 200 C'); assert.match(a.temperature.checked_by, /no motor thermal model/);
  assert.equal(f({ cells: 3 }, [mot]).temperature.published, null);
  // a pack charged above 4.20 V a cell (its shift), or limited below it
  assert.equal(f({ cells: 3, cell_shift_V: 0.1 }).power.within, false);
  assert.match(f({ cells: 3, charge_limit_V_per_cell: 4.1 }).power.basis, /12\.3 V/);
  // no published power: no power figure; no battery or no motor part: nothing
  assert.equal(motorFigures({ motor: { id: 'motor-x' }, battery: { cells: 3 } }, [{ ...mot, specs: { ...mot.specs, continuous_power_w: null } }]).some((c) => c.figure === 'power'), false);
  assert.deepEqual(motorFigures({ motor: { id: 'motor-x' } }, [mot]), []);
  assert.deepEqual(motorFigures({ motor: { id: 'motor-y' }, battery: { cells: 3 } }, [mot]), []);
  // the servo's condition names its published rows
  const sv = servoArmMaker({ servos: { twist: { id: SERVO, stall_Nm: 0.5, volts: 6 } }, electrical: { servo_supply_V: 6 } }, catalog, 'twist', HORN);
  assert.equal(sv.condition[0].published, 0.5); // no linkage: the servo's own stall
  assert.match(sv.condition[1].basis, /rows 4\.8, 6, 7\.4 V/);
});
