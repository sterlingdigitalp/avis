// Round 19 (decision 5, "Motor temperature"): the maker's-part condition includes every figure the maker publishes.
// Scorpion's operating manual (catalogue thermal-ref-scorpion-sii-3008-1090, source_url_3) publishes a temperature limit
// (100 C at any measured point), the continuous rating's conditions (60 s at 100% throttle, forced airflow of at least
// 30 m/s, ~30 C ambient) and derating tables "for reference only". Nothing it publishes bounds the motor's temperature
// from its load, so the motor's maker's-part interfaces are not counted; the rating's conditions are checked with the
// maker's own tables, read by hand below (ambient 10 C -> the '< 30 C' column 100%; air at most 11 m/s -> the 10 m/s
// column 80%, none -> 0 m/s 50%; a 13.2 min run -> the '30 mins' column 73%; at half, SPEC round 7).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as CH from '../../src/design/chains.js';
import { makerPartCheck } from '../../src/physics/members.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const MOTOR = 'motor-scorpion-sii-3008-1090';
// the design's own conditions: the round-18 x1 build's (10 C at 1800 m; cruise 11 m/s at 111.27 W on a 2.2 Ah 3S pack)
const design = (over = {}) => ({ motor: { id: MOTOR }, battery: { cells: 3, capacity_Ah: 2.2, V_nom: 11.1 }, atmosphere: { temperature_C: 10 }, control: { Vcruise: 11, trimFlight: [{ V: 11, flight_power_w: 111.27 }] }, ...over });
const figs = (d, cat = catalog) => Object.fromEntries(CH.motorFigures(d, cat).map((c) => [c.figure, c]));

test('motor temperature: Scorpion publishes 100 C at any measured point and nothing that bounds it, so the condition is not counted', () => {
  const t = figs(design()).temperature;
  assert.equal(t.published, 100);
  assert.equal(t.within, null);
  assert.equal(t.counted, false, 'an unbounded published limit is never a silent pass');
  assert.match(t.not_counted_because, /100 C at any measured point/);
  assert.match(t.not_counted_because, /cannot be bounded/);
  assert.match(t.basis, /should not exceed 100C at any measured point/);
});

test('the continuous rating\'s stated conditions, derated by the maker\'s tables read by hand', () => {
  const r = figs(design())['continuous rating conditions'];
  assert.ok(r, 'the row exists');
  assert.deepEqual(r.conditions.rating, { duty_s: 60, airflow_m_s: 30, ambient_C: 30 });
  assert.equal(r.conditions.design.duty_min, +((2.2 * 11.1 * 60) / 111.27).toFixed(2)); // 13.17 min on one pack
  const air = r.derated.flight_speed_air, none = r.derated.no_cooling_path;
  assert.deepEqual([air.rows.ambient_C, air.rows.airflow_m_s, air.rows.duty_min], [[30, 100], [10, 80], [30, 73]]);
  assert.ok(Math.abs(air.current_A - 26 * 0.8 * 0.73) < 1e-4, `${air.current_A} A = 26 A x 0.80 x 0.73`);
  assert.ok(Math.abs(air.power_W - 370 * 0.8 * 0.73) < 1e-4);
  assert.ok(Math.abs(air.spec_A - (26 * 0.8 * 0.73) / 2) < 1e-4, 'the tables are "for reference only": at half');
  assert.deepEqual(none.rows.airflow_m_s, [0, 50]);
  assert.ok(Math.abs(none.spec_A - (26 * 0.5 * 0.73) / 2) < 1e-4, `${none.spec_A} A = 26 A x 0.50 x 0.73 / 2`);
  // the current row names the rating's conditions
  assert.match(figs(design()).current.basis, /60 seconds at 100% throttle .* at least 30m\/s/);
});

test('flown: the endurance cruise\'s sustained RMS is held to the derated current with no cooling path; the 2 s RMS to 26 A', () => {
  // at the build the trim is not yet flown (no cruise speed, no trim power): the duty is the tables' longest row
  const built = design({ control: {} }), cond = CH.motorFigures(built, catalog);
  const early = cond.find((c) => c.figure === 'continuous rating conditions');
  assert.equal(early.derated.flight_speed_air, null); assert.deepEqual(early.derated.no_cooling_path.rows.duty_min, [null, 60]);
  // the round-18 x1 build's M2 (scratchpad r19/small/m2rms.log): 18.92 A RMS over the whole flight, 2 s maximum 19.28 A,
  // endurance 11.20 min; the gate passes the flown design, so the readings are those of its cruise and its endurance
  const hot = CH.flownMotorCondition(cond, { rms2sMax: 19.28, sustainedRms: 18.92, endurance_min: 11.2, design: design(), ruling: 'binds' });
  const rc = hot.find((c) => c.figure === 'continuous rating conditions');
  assert.equal(rc.value, 18.92);
  assert.equal(rc.within, false, '18.92 A against 26 x 0.5 x 0.73 / 2 = 4.745 A');
  assert.deepEqual(rc.exceeds.flight_speed_air, { full: true, spec_half: true }, 'even with the air at the flight speed, at full value (15.18 A)');
  assert.deepEqual(rc.derated.no_cooling_path.rows.duty_min, [30, 73], '11.2 min: the 30 mins column');
  assert.equal(hot.find((c) => c.figure === 'current').within, true, '19.28 A within 26 A over 2 s windows');
  assert.ok(hot.not_counted.some((w) => /^temperature: .*cannot be bounded/.test(w)));
  const cool = CH.flownMotorCondition(cond, { rms2sMax: 27, sustainedRms: 4.0, endurance_min: 11.2, design: design(), ruling: 'binds' });
  assert.equal(cool.find((c) => c.figure === 'continuous rating conditions').within, true);
  assert.equal(cool.find((c) => c.figure === 'current').within, false, '27 A over 26 A');
  const none = CH.flownMotorCondition(cond, { rms2sMax: 1, sustainedRms: NaN, design: design() });
  assert.equal(none.find((c) => c.figure === 'continuous rating conditions').counted, false, 'no endurance flight: not counted');
});

test('the derating\'s edges: no column for the condition is not counted; no trim reads the longest duty; air above 30 m/s reads the rated column', () => {
  const T = catalog.find((p) => p.id === 'thermal-ref-scorpion-sii-3008-1090').specs.derating, R = { current_A: 26, power_W: 370 };
  assert.equal(CH.deratedRating(R, T, { airflow_m_s: 0, ambient_C: 61, duty_min: 10 }), null, 'above the 60 C column: no published row');
  assert.deepEqual(CH.deratedRating(R, T, { airflow_m_s: 40, ambient_C: 30, duty_min: null }).rows.duty_min, [null, 60]);
  assert.equal(CH.deratedRating(R, T, { airflow_m_s: 40, ambient_C: 30, duty_min: 0.5 }).ratio, 1);
  assert.ok(Math.abs(CH.deratedRating(R, T, { airflow_m_s: 12, ambient_C: 41, duty_min: 61 }).ratio - 0.93 * 0.8 * 0.6) < 1e-4, '45 C column, 10 m/s column, > 60 mins column');
  const hot = figs(design({ atmosphere: { temperature_C: 61 } }))['continuous rating conditions'];
  assert.equal(hot.derated.no_cooling_path, null);
  const flown = CH.flownMotorCondition([hot], { rms2sMax: 1, sustainedRms: 1 });
  assert.equal(flown[0].counted, false);
  assert.match(flown.not_counted[0], /no row for the design's condition/);
  assert.equal(figs(design({ control: { Vcruise: 11 } }))['continuous rating conditions'].derated.no_cooling_path.rows.duty_min[0], null);
});

test('a maker\'s part whose condition is not counted is itself not counted, and says why; its ok is unchanged', () => {
  const r = makerPartCheck({ maker: { condition: [{ figure: 'temperature', within: null, counted: false, not_counted_because: 'unbounded' }, { figure: 'c', within: true }] } }, () => 0);
  assert.equal(r.ok, true);
  assert.equal(r.counted, false);
  assert.deepEqual(r.not_counted_because, ['temperature: unbounded']);
  assert.match(r.why, /not counted: temperature: unbounded/);
  assert.equal(makerPartCheck({ maker: { condition: [{ figure: 'c', within: true }] } }, () => 0).counted, undefined);
});

test('M7\'s maker\'s parts: the motor\'s members flown, over the derated rating fails, the unbounded temperature leaves them not counted', () => {
  const cond = CH.motorFigures(design(), catalog).map(({ row, ...c }) => c);
  const members = [
    { name: 'motor bearings', check: "maker's part", ok: true, condition: cond, peaks: { F: 1 } },
    { name: 'servo arm', check: "maker's part", ok: true, condition: [{ figure: 'servo torque', within: true }] },
    { name: 'a shaft', check: 'rated', ok: true },
  ];
  const hot = CH.flownMakersParts(members, { rms2sMax: 19.28, sustainedRms: 18.92, endurance_min: 11.2, design: design(), ruling: 'binds' });
  assert.deepEqual(hot.parts.map((q) => q.name), ['motor bearings', 'servo arm']);
  assert.equal(hot.ok, false, 'the motor over its derated rating fails M7');
  assert.equal(hot.parts[1].ok, true); assert.equal(hot.parts[1].counted, undefined);
  assert.equal(hot.parts[0].counted, false);
  assert.ok(hot.notCounted.length === 1 && /^motor bearings \(unrated, maker's part\): temperature: .*cannot be bounded/.test(hot.notCounted[0]));
  assert.equal(CH.flownMakersParts(members, { rms2sMax: 19.28, sustainedRms: 4, endurance_min: 11.2, design: design(), ruling: 'binds' }).ok, true, 'within the derated rating: ok, still not counted');
});

test('round 19: until the operator rules on the rating conditions, the row is held: not counted, its reading beside it', () => {
  assert.equal(CH.RATING_CONDITIONS_RULING, 'pending', 'the ruling (asked 2026-11-04) is not yet made');
  const cond = CH.motorFigures(design(), catalog);
  const held = CH.flownMotorCondition(cond, { rms2sMax: 19.28, sustainedRms: 18.92, endurance_min: 11.2, design: design() });
  const rc = held.find((c) => c.figure === 'continuous rating conditions');
  assert.equal(rc.within, null, 'neither a pass nor a fail while the ruling is pending');
  assert.equal(rc.counted, false);
  assert.equal(rc.would_be_within, false, '18.92 A against 26 x 0.5 x 0.73 / 2 = 4.745 A');
  assert.match(rc.not_counted_because, /pending the operator's ruling/);
  assert.ok(held.not_counted.some((w) => /^continuous rating conditions: held pending/.test(w)) && held.not_counted.some((w) => /^temperature: /.test(w)));
  const cool = CH.flownMotorCondition(cond, { rms2sMax: 19.28, sustainedRms: 4.0, endurance_min: 11.2, design: design() }).find((c) => c.figure === 'continuous rating conditions');
  assert.equal(cool.within, null); assert.equal(cool.would_be_within, true, 'a passing reading is held too: never a silent pass');
  const members = [{ name: 'motor bearings', check: "maker's part", ok: true, condition: cond.map(({ row, ...c }) => c), peaks: { F: 1 } }];
  const m7 = CH.flownMakersParts(members, { rms2sMax: 19.28, sustainedRms: 18.92, endurance_min: 11.2, design: design() });
  assert.equal(m7.ok, true, 'held, so not failed');
  assert.equal(m7.parts[0].counted, false);
  assert.equal(m7.notCounted.length, 2, 'the temperature and the held rating conditions');
});
