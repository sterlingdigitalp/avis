// Round 17 (SPEC "Design freeze", "Function completeness"): the new records refuse what they must. Each test builds a
// deliberately broken input and checks the record (and --official's refusal) fails it for the stated reason.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { chainRow, chainRecord, ratingsRecord, joinedParts } from '../../src/design/chains.js';
import { tailChains, tailLinkageRatio, TAIL_MECH } from '../../src/design/tail-mech.js';
import { officialRefusal, ROUND17_RECORDS } from '../../src/design/round17.mjs';
import { interfaceLedger, RULE_HELPERS } from '../../src/design/interfaces.js';
import { massPropsRecord } from '../../src/design/massprops.js';
import { resolveSteps } from '../../src/design/sequence.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const S = (name, x, c, r) => ({ name, x, shape: { kind: 'ring', c, ro: r, ri: 0 } });
const part = (id, bom = 'x') => ({ id, bom: [bom], solids: [{ ...S('s', [0, 0.01], [0, 0], 0.001), bom }] });
const chainOf = (over = {}) => ({ name: 'tail R', actuator: { part: 'a' }, path: ['a', 'b', 'c'], figures: { ratio: { flown: 1.2, drawn: 1.2, tol: 1e-4, rel: true }, travel: { required_rad: 0.5, delivered_rad: 0.6 } }, ...over });
const asm3 = (joins = [['a', 'b', 'j'], ['b/s', 'c', 'j']]) => ({ parts: [part('a'), part('b'), part('c')], joins });

test('a chain passes only with every element drawn, tagged and joined to the next', () => {
  assert.equal(chainRow(asm3(), chainOf()).passed, true);
  assert.ok(joinedParts(asm3(), 'b', 'c'), 'a join naming a part\'s solid joins the part');
  const unjoined = chainRow(asm3([['a', 'b', 'j']]), chainOf());
  assert.equal(unjoined.passed, false);
  assert.deepEqual(unjoined.unjoined, ['b | c']);
  const missing = chainRow(asm3(), chainOf({ path: ['a', 'b', 'nothing'] }));
  assert.equal(missing.passed, false);
  assert.deepEqual(missing.not_drawn_or_untagged, ['nothing']);
});

test('a chain whose figures disagree with the flown ones, or that cannot reach the commanded travel, fails with a finding', () => {
  const ratio = chainRow(asm3(), chainOf({ figures: { ratio: { flown: 1, drawn: 1.2, tol: 1e-4, rel: true } } }));
  assert.equal(ratio.passed, false);
  assert.match(ratio.findings[0], /^ratio: drawn 1\.2 against flown 1/);
  const travel = chainRow(asm3(), chainOf({ figures: { travel: { required_rad: 0.5236, delivered_rad: 0.4 } } }));
  assert.equal(travel.passed, false);
  assert.match(travel.findings[0], /^travel: delivers 22\.92 deg of the 30\.00 deg commanded/);
  const stated = chainRow(asm3(), chainOf({ findings: ['the servo stalls'] }));
  assert.equal(stated.passed, false, 'a finding the module states fails the chain');
});

test('the chains record fails every flown motion that has no chain', () => {
  const r = chainRecord(asm3(), {}, [chainOf()]);
  assert.equal(r.passed, false);
  assert.deepEqual(r.missing, ['twist R', 'twist L', 'tail L', 'flap drive R', 'flap drive L']);
});

test('the tail chain delivers its travel through the drawn ratio, and a servo travel too short is a finding', () => {
  assert.equal(tailLinkageRatio({}), TAIL_MECH.arm_hole_mm / TAIL_MECH.horn.radius);
  const G = { xq: -0.9, s0: 0.02, hubR: 0.004, d3: 0.012, rW: 0.0035, run: 0.0005, rBoss: 0.005 };
  const row = { tag: 'R', zH: 0.024, zs: 0.008, nE: 0.013, o: [0.005, 0.062], u: [Math.cos(0.61), Math.sin(0.61)] };
  const d = { tail: { x_m: -0.9, dihedral_rad: 0.61, deflection_max_rad: 30 * Math.PI / 180 }, servos: { tail: { id: 'servo-hitec-md89mw', linkage_ratio: 16 / 13 } }, structure: { members: [] } };
  const mk = (P) => ({ parts: [], joins: [], tailMech: { G, rows: [row], P: { ...TAIL_MECH, ...P } } });
  const [ok] = tailChains(mk({}), d, catalog);
  assert.ok(ok.figures.travel.delivered_rad > ok.figures.travel.required_rad);
  const [short] = tailChains(mk({ servo_travel_deg: 20 }), d, catalog);
  const r = chainRow({ parts: [], joins: [] }, short);
  assert.ok(r.findings.some((f) => /^travel: delivers/.test(f)), 'at 20 deg of servo travel the 30 deg deflection is out of reach');
});

test('ratings: a servo supplied above its top published row fails (audit item 15: the MD89MW at 7.5 V)', () => {
  const d = (lo, hi) => ({ servos: { tail: { id: 'servo-hitec-md89mw' } }, electrical: { servo_supply: { performance_V: lo, current_V: hi } } });
  assert.equal(ratingsRecord(d(5.76, 6.24), catalog).passed, true);
  const r = ratingsRecord(d(7.5, 7.8), catalog);
  assert.equal(r.passed, false);
  assert.ok(r.rows.some((x) => !x.ok && /7\.5 V is outside the published rows 4\.8-7\.4 V/.test(x.why)));
  assert.equal(ratingsRecord(d(5.76, 8.6), catalog).passed, false, 'a full-charge supply above the published 3.5-8.4 V range fails');
  const reg = ratingsRecord({ ...d(5.76, 6.24), electrical: { servo_supply: { part: 'regulator-pololu-d36v50f6', performance_V: 5.76, current_V: 6.24, input_V: [5.0, 12.6] } } }, catalog);
  assert.equal(reg.passed, false, 'a pack range below the regulator\'s published minimum input fails');
});

test('--official refuses a design missing or failing any round-17 record', () => {
  for (const [k, what] of ROUND17_RECORDS) {
    const c = { massprops: { passed: true, failed: [] }, ...Object.fromEntries(ROUND17_RECORDS.map(([q]) => [q, { passed: true, failed: [], seats: [] }])) };
    delete c[k];
    assert.match(officialRefusal(c) ?? '', new RegExp(`no ${what} record`));
    c[k] = { passed: false, failed: ['x'], seats: [] };
    assert.match(officialRefusal(c) ?? '', new RegExp(what));
  }
});

test('a module\'s interface rule row matches ahead of the generic table', () => {
  const asm = { parts: [], joins: [['tail thing R', 'tail other R', 'a joint only the module knows']] };
  const d = { structure: { members: [{ name: 'rated thing', check: 'bolts' }], joints: [], joints_unchecked: [] }, params: { layout: {} } };
  assert.equal(interfaceLedger(asm, d).passed, false);
  const rule = { why: RULE_HELPERS.exact('a joint only the module knows'), part: null, kind: 'fastened', load: RULE_HELPERS.constSpec(['rated thing'], RULE_HELPERS.BOLT_RULE, 'the thing'), axial: RULE_HELPERS.constSpec(['rated thing'], RULE_HELPERS.BOLT_RULE, 'the thing') };
  assert.equal(interfaceLedger(asm, d, [rule]).passed, true);
});

test('massprops: a lump naming a shared line on its own parts sits at those parts, not at every part of the line', () => {
  const p = (id, z) => ({ id, solids: [{ name: 's', x: [0, 0.01], shape: { kind: 'ring', c: [z, 0], ro: 0.005, ri: 0 }, bom: 'servo-hitec-md89mw' }] });
  const asm = { parts: [p('packed servo_tail_R', 0), p('twist servo R', 0.5)], joins: [] };
  const d = { bom: { parts: [{ id: 'servo-hitec-md89mw', count: 2, mass_g: 43, role: 'servos' }], structure: [] }, mass: { lumps: [{ name: 'tail servos', mass_kg: 0.0215, pos: [0.005, 0, 0], lines: ['servo-hitec-md89mw@^packed servo_tail_'] }] }, body: {}, tail: null };
  const ok = massPropsRecord(asm, d, catalog, null).rows[0];
  assert.ok(ok.err_mm < 1e-6, `the tail servo's own centroid (${ok.drawn_mm})`);
  const all = massPropsRecord(asm, { ...d, mass: { lumps: [{ ...d.mass.lumps[0], lines: ['servo-hitec-md89mw'] }] } }, catalog, null).rows[0];
  assert.ok(all.err_mm > 200, 'unfiltered, the twist servo drags the lump 250 mm out');
});

test('a step may move along the direction a drawn part states, and refuses a part that states none', () => {
  const asm = { parts: [{ id: 'rod', slide: [0, 1, 0], solids: [{ name: 's' }] }], joins: [] };
  const r = resolveSteps(asm, { steps: [{ name: 'rod', members: ['^rod$'], dir: { slide_of: 'rod', sign: -1 } }] });
  assert.deepEqual(r.steps[0].dir.map((v) => v + 0), [0, -1, 0]);
  assert.throws(() => resolveSteps({ parts: [{ id: 'rod', solids: [{ name: 's' }] }], joins: [] }, { steps: [{ name: 'rod', members: ['^rod$'], dir: { slide_of: 'rod' } }] }), /stating a slide direction/);
});

test('a shortfall the flight model flies (end stop, torque saturation) is reported, a module\'s plain finding refuses', () => {
  const short = { travel: { required_rad: 0.98, delivered_rad: 0.675 } };
  const flownLimit = chainRow(asm3(), chainOf({ figures: short, flown_limits: ['travel'], findings: [{ text: 'compliance: winds 0.12 rad', flown: true }] }));
  assert.equal(flownLimit.passed, true);
  assert.equal(flownLimit.flown_findings.length, 2);
  assert.match(flownLimit.flown_findings[1], /^travel: delivers 38\.67 deg of the 56\.15 deg commanded \(flown/);
  assert.equal(chainRow(asm3(), chainOf({ figures: short })).passed, false, 'not flown: the shortfall refuses');
});
