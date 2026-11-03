// The studio sequences' words and numbers come from the design, the BOM and the gate files as written: the drive's
// stages, the power sequence's parts and masses, and the gates card, failures in their own words and numbers.
import test from 'node:test';
import assert from 'node:assert/strict';
import { loadModel, loadVerification } from '../../sims/avis-shared/node-load.js';
import { driveStages, powerParts, gateOrder, shortName, subsystemOf, subsystemMass, gatesCardItems, stageRows, massRows } from '../../sims/avis-shared/film.js';
import { gateLine, gateFailures } from '../../sims/avis-shared/numbers.js';

const { model, design } = loadModel('larus-gl-2');
const gates = loadVerification('larus-gl-2');

test('drive stages: the BOM gear pairs, motor first, multiplying to the design ratio, with BOM masses', () => {
  const st = driveStages(model);
  const ids = design.drive.gears;
  assert.equal(st.length, ids.length / 2);
  const ratio = st.reduce((r, s) => r * s.ratio, 1);
  assert.ok(Math.abs(ratio - design.drive.gearRatio) < 1e-9, `${ratio} vs ${design.drive.gearRatio}`);
  const bom = (id) => design.bom.parts.find((p) => p.id === id);
  st.forEach((s, k) => {
    assert.equal(s.pinion, bom(ids[2 * k]).name.split(' ')[0]);
    assert.equal(s.wheel, bom(ids[2 * k + 1]).name.split(' ')[0]);
    assert.equal(s.zp, model.parts[ids[2 * k]].specs.teeth);
    assert.equal(s.zw, model.parts[ids[2 * k + 1]].specs.teeth);
    assert.equal(s.mass_g, bom(ids[2 * k]).unit_mass_g + bom(ids[2 * k + 1]).unit_mass_g);
  });
});

test('power parts: battery, ESC, regulator and servos by BOM role, each with its BOM mass and count', () => {
  const rows = powerParts(model);
  const byRole = (re) => design.bom.parts.filter((p) => re.test(p.role));
  const expect = [[/^battery$/, `${design.battery.cells}S`], [/\bESC\b/, 'ESC'], [/regulator/, 'regulator'], [/^tail servo$/, 'tail servos'], [/twist servo$/, 'twist servos']].filter(([re]) => byRole(re).length);
  assert.equal(rows.length, expect.length);
  rows.forEach((r, k) => {
    const ps = byRole(expect[k][0]);
    assert.ok(r.label.includes(expect[k][1]), r.label);
    assert.equal(r.mass_g, ps.reduce((a, p) => a + p.mass_g, 0));
    const n = ps.reduce((a, p) => a + p.count, 0);
    assert.equal(r.name.startsWith(`${n} × `), n > 1, r.name);
    assert.ok(r.name.includes(model.parts[ps[0].id].manufacturer), r.name);
  });
  assert.equal(shortName({ manufacturer: 'Pololu', name: 'D36V50F7 7.5 V step-down voltage regulator (#4093)' }), 'Pololu D36V50F7');
  assert.equal(shortName({ manufacturer: 'Gens Ace', name: 'Gens Ace 2200mAh 3S 35C LiPo (GEA22003S35D)' }), 'Gens Ace 2200mAh');
});

test('assembly masses: every BOM line lands in one subsystem and the five add up to parts plus structure', () => {
  const m = subsystemMass(design);
  const sum = Object.values(m).reduce((a, b) => a + b, 0);
  assert.ok(Math.abs(sum - (design.bom.parts_g + design.bom.structure_g)) < 1e-6, `${sum}`);
  assert.equal(subsystemOf('servo regulator'), 'power');
  assert.equal(subsystemOf('rocker right'), 'drive');
  assert.equal(subsystemOf('key, crank wheel gear-khk-msgb1-45'), 'drive');
  assert.equal(subsystemOf('flap ESC (no BEC; the flight controller\'s 5 V BEC supplies the avionics)'), 'power');
  assert.equal(subsystemOf('twist torque rods, 2 off'), 'wing');
  assert.equal(subsystemOf('shoulder rail right'), 'structure');
});

test('the gates card lists every gate file and every declared gate, M before V', () => {
  const order = gateOrder(gates);
  for (const g of gates) assert.ok(order.includes(g.gate), g.gate);
  for (const g of ['M1', 'M8', 'V1', 'V7']) assert.ok(order.includes(g));
  assert.ok(order.indexOf('M8') < order.indexOf('V1'));
  assert.deepEqual(gateOrder([{ gate: 'V10' }, { gate: 'M9' }]).slice(-9), ['M9', 'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7', 'V10']);
});

test('a failed gate shows what failed in the file\'s own words and numbers; a pass shows nothing', () => {
  for (const g of gates) {
    const f = gateFailures(g);
    if (g.passed) {
      assert.deepEqual(f, [], g.gate);
      continue;
    }
    assert.ok(f.length, `${g.gate} failed with no reason shown`);
    const corners = Object.entries(g.measured?.robustness ?? {});
    const bad = corners.filter(([, v]) => v === false).map(([k]) => k);
    for (const k of bad) assert.ok(f.join(' ').includes(k), `${g.gate}: corner ${k} not shown`);
    if (bad.length) assert.ok(f.join(' ').includes(`${bad.length} of ${corners.length}`), g.gate);
  }
  const m1 = gates.find((g) => g.gate === 'M1');
  if (m1) {
    const line = gateLine(m1);
    assert.ok(line.includes(m1.measured.height_lost_m.toFixed(2)) && line.includes(String(m1.limit.height_lost_m)), line);
    assert.ok(line.includes(m1.measured.first_1s_window_climbing_s.toFixed(2)) && line.includes(`${m1.limit.climb_within_s} s`), line);
  }
  // independent of today's files: a nominal failure and an exceeded error limit are both named
  const f = gateFailures({ gate: 'X', passed: false, measured: { nominal_passed: false, max_error_pct: 7.5 }, limit: { max_error_pct: 3 } });
  assert.ok(f.includes('nominal case fails') && f.some((x) => x.includes('7.5') && x.includes('3')), f.join(' / '));
});

test('the card: one row per gate, a failure across the card with its reasons, a pass on half a row', () => {
  const html = gatesCardItems(gates);
  const rows = html.split('<div class="g').slice(1);
  assert.equal(rows.length, gateOrder(gates).length);
  for (const g of gateOrder(gates)) {
    const r = gates.find((x) => x.gate === g);
    const h = rows.find((x) => x.includes(`<em>${g}</em>`));
    assert.ok(h, g);
    if (!r) assert.ok(h.includes('not run'), g);
    else if (r.passed) assert.ok(!h.startsWith(' wide') && h.includes('pass'), g);
    else {
      assert.ok(h.startsWith(' wide') && h.includes('FAIL'), g);
      for (const f of gateFailures(r)) assert.ok(h.includes(f), `${g}: ${f}`);
    }
  }
  assert.ok(gatesCardItems([]).includes('not run'));
});

test('panel rows: every stage by its catalogue numbers; a subsystem dimmed until it is in place', () => {
  const st = driveStages(model);
  const h = stageRows(st);
  for (const x of st) assert.ok(h.includes(`${x.pinion} → ${x.wheel}`) && h.includes(`${x.zp}:${x.zw}`), x.pinion);
  const m = massRows({ drive: 0, power: 0.5 }, { drive: 853.2, power: 156 });
  assert.ok(/class="frow"><span>drive<\/span><b>853 g/.test(m), m);
  assert.ok(/class="frow dim"><span>power<\/span><b>156 g/.test(m), m);
});

test('studio framing: a sphere round the parts shown, held at a fraction of the frame; callouts only on screen', async () => {
  const { framing, onScreen, sphereOf } = await import('../../sims/avis-shared/film.js');
  const { nodeBird } = await import('../lib/bird-node.mjs');
  const f = framing({ x: 1 }, 0.1, 30, 0.5);
  assert.ok(Math.abs(f.dist - 0.1 / (0.5 * Math.tan(Math.PI / 12))) < 1e-12 && f.target.x === 1);
  assert.ok(onScreen(960, 540, 0.5) && !onScreen(-1, 540, 0.5) && !onScreen(960, 1081, 0.5) && !onScreen(960, 540, 1.01) && onScreen(0, 0, 1) && onScreen(1920, 1080, 1));
  const { bird } = nodeBird('larus-gl-2');
  const s = sphereOf(bird, model.law.ampNominal, (o) => o.userData.subsystem === 'drive');
  const gear = bird.root.getObjectByName('gear_crank').getWorldPosition(new (await import('three')).Vector3());
  assert.ok(s.r > 0.03 && s.r < 0.3 && s.c.distanceTo(gear) < s.r, `${s.r}`);
});

test('gate failures, each kind in the file\'s own words', () => {
  const F = (measured, limit = {}) => gateFailures({ gate: 'X', passed: false, measured, limit });
  assert.deepEqual(gateFailures({ gate: 'X', passed: true, measured: { robustness_passed: false } }), []);
  assert.deepEqual(gateFailures({ gate: 'X', measured: { nominal_passed: false } }), []);
  assert.deepEqual(F({ robustness: { a: true, b: false, c: { passed: false }, d: { passed: true } }, robustness_passed: false }), ['fails at 2 of 4 robustness corners: b, c']);
  assert.deepEqual(F({ robustness: { a: true }, robustness_passed: false }), ['fails its robustness check']);
  assert.deepEqual(F({ robustness_passed: true, nominal_passed: true, gears_within_rating: false, ok: true }), ['gears within rating: no']);
  assert.deepEqual(F({ nominal_passed: false }), ['nominal case fails']);
  assert.deepEqual(F({ max_error_pct: 3, height_lost_m: 1.6, endurance_min: 5 }, { max_error_pct: 3, height_lost_m: 1.5, endurance_min: 10, note: 'x' }), ['height lost m 1.6 against a limit of 1.5']);
  assert.deepEqual(F({ max_error_pct: 4.281 }, { error_pct: 3 }), ['max error pct 4.28 against a limit of 3']);
  assert.deepEqual(F({ failures: ['seal', { case: 2 }] }), ['seal', '{"case":2}']);
  assert.deepEqual(F({ points: 26, points_passed: 13, points_unconverged: 8 }), ['13 of 26 points fail, 8 of them unconverged']);
  assert.deepEqual(F({ points: 26, points_passed: 20 }), ['6 of 26 points fail']);
  assert.deepEqual(F({ points: 26, points_passed: 26 }), ['fails (the file names no single check)']);
  assert.deepEqual(F({}), ['fails (the file names no single check)']);
});
