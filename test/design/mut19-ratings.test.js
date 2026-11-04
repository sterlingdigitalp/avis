// Round 19 mutation follow-up (chains.js servoSupplyRows): each ratings row on its own, at its published edges. The
// MD89MW's published figures (catalogue): torque and speed rows at 4.8, 6.0 and 7.4 V, stall current rows at the same
// voltages, its range "3.5 ~ 8.4V". A supply exactly on a published edge is inside it; one past it is outside; the rows
// are read one by one, so a failing row cannot hide behind another that fails too.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { ratingsRecord } from '../../src/design/chains.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const SV = 'servo-hitec-md89mw';
const d = (lo, hi) => ({ servos: { tail: { id: SV } }, electrical: { servo_supply: { performance_V: lo, current_V: hi } } });
const rowsOf = (lo, hi, cat = catalog) => {
  const rs = ratingsRecord(d(lo, hi), cat).rows;
  return { rows: rs.find((r) => / torque and speed at /.test(r.what)), range: rs.find((r) => / supply [\d.]+-[\d.]+ V$/.test(r.what)), top: rs.find((r) => /highest supply/.test(r.what)) };
};

test('ratings mut19: a supply exactly on the published rows\' edges is inside them (4.8 V lowest, 7.4 V highest)', () => {
  const lo = rowsOf(4.8, 4.8), hi = rowsOf(7.4, 7.4);
  assert.equal(lo.rows.ok, true, 'lowest output on the 4.8 V row');
  assert.equal(hi.rows.ok, true, 'lowest output on the 7.4 V row');
  assert.equal(hi.top.ok, true, 'highest output on the 7.4 V row (torque, speed and stall current all published there)');
  assert.equal(rowsOf(4.79, 5).rows.ok, false, 'below the 4.8 V row');
});

test('ratings mut19: the published range row on its own: its edges inside, beyond either edge outside', () => {
  // inside the rows (so only the range row can fail): lo 4.8 V; the range's top 8.4 V
  assert.equal(rowsOf(4.8, 8.4).range.ok, true, 'a supply up to the range\'s 8.4 V top is inside it');
  assert.equal(rowsOf(4.8, 8.6).range.ok, false, 'a supply to 8.6 V is outside the 3.5-8.4 V range (its own row, not only the top-row one)');
  // a servo published from 4.8 V rows with its range starting at 4.8 V: a supply from exactly 4.8 V is inside
  const cat = catalog.map((p) => (p.id === SV ? { ...p, specs: { ...p.specs, voltage_range_text: '4.8 ~ 8.4V' } } : p));
  assert.equal(rowsOf(4.8, 6.0, cat).range.ok, true, 'from the range\'s 4.8 V bottom');
  assert.equal(rowsOf(4.7, 6.0, cat).range.ok, false, 'from 4.7 V, below the 4.8 V bottom');
});

test('ratings mut19: the highest supply must be inside the stall-current rows too, not only the torque rows', () => {
  // the MD89MW with its stall current published only to 6.0 V (a deliberately broken entry): 7.0 V is inside the torque
  // rows (to 7.4 V) but above the current's top row
  const cat = catalog.map((p) => (p.id === SV ? { ...p, specs: { ...p.specs, stall_current: p.specs.stall_current.filter((q) => q.voltage_v <= 6) } } : p));
  const r = rowsOf(5.0, 7.0, cat);
  assert.equal(r.top.ok, false);
  assert.match(r.top.why, /above the published top row 6 V/);
  assert.equal(rowsOf(5.0, 6.0, cat).top.ok, true, 'on the current\'s 6.0 V top row');
});
