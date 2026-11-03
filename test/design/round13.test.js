// Round 13: the free-flight trim refine's row hand-on (src/design/trim.js keptTrimRows), pinned on hand-built rows:
// which rows the next pass starts from, and that no unconverged row survives the last pass.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { keptTrimRows, trimRowRecord, flyTrimRow, FLIGHT_TRIM_TOL } from '../../src/design/trim.js';

const flown = (V, extra) => ({ V, theta: 0.07, freq: 2.2, tail: 0.03, amp: 0.66, sym: 0, dyn: 0.52, flight_power_w: 80, flight_power_regen_credit_w: 77, flight_V: V, ...extra });
const rows = [
  flown(9.5, { converged: true, convergence: { freq_drift_frac: 0.016 } }),
  flown(10, { converged: false, unflyable: true, reason: 'not converged', convergence: { freq_drift_frac: 0.021 } }),
  { V: 13.5, theta: 0, freq: 2.6, tail: 0, amp: 0.66, sym: 0, dyn: 0.52, unflyable: true, converged: false, reason: 'diverged' },
];

test('without retry (rounds 3-12): converged rows only, at every pass', () => {
  for (const last of [true, false]) assert.deepEqual(keptTrimRows(rows, { last }).map((r) => r.V), [9.5]);
  assert.deepEqual(keptTrimRows(rows).map((r) => r.V), [9.5]);
});

test('retry before the last pass: a row that only missed a tolerance goes on; a diverged row never does', () => {
  assert.deepEqual(keptTrimRows(rows, { retry: true, last: false }).map((r) => r.V), [9.5, 10]);
  assert.deepEqual(keptTrimRows(rows, { retry: true, last: true }).map((r) => r.V), [9.5]);
  // A call that names no pass is the last pass: retry alone never lets an unconverged row through.
  assert.deepEqual(keptTrimRows(rows, { retry: true }).map((r) => r.V), [9.5]);
});

test('kept rows carry the flight means as the next start, without the flight records', () => {
  const [, r10] = keptTrimRows(rows, { retry: true, last: false });
  assert.deepEqual(r10, { V: 10, theta: 0.07, freq: 2.2, tail: 0.03, amp: 0.66, sym: 0, dyn: 0.52 });
});

test('round-13 candidate sets: the candidate block, then --set over it, parts merged', async () => {
  const { round13Set, ROUND13 } = await import('../../src/design/round13.mjs');
  assert.deepEqual(round13Set('c45'), { AR: 4.5, sparFrac: 0.28 });
  assert.deepEqual(round13Set('c45', { sparFrac: 0.3, parts: { keel: 'k' } }), { AR: 4.5, sparFrac: 0.3, parts: { keel: 'k' } });
  const t = round13Set('c45t', { parts: { keel: 'k' } });
  assert.equal(t.parts.keel, 'k');
  assert.deepEqual(t.parts.gears, ROUND13.candidates.c45t.parts.gears);
  assert.throws(() => round13Set('nope'), /unknown round-13 candidate/);
});

// trimRowRecord on hand sums: a 10 s window of 20 + 20 samples.
const sums = (o = {}) => ({ ok: true, n: 40, tail: 1.2, th: 2.8, P: 3200, PC: 3080, V: 404, y0: 1, y1: 1.5, average: 10, half: [{ f: 40, tail: 0.6, n: 20 }, { f: 41, tail: 0.6, n: 20 }], ...o });
const row = { V: 10, theta: 0.05, freq: 1.6, tail: 0.02, amp: 0.66, sym: 0, dyn: 0.52 };

test('trimRowRecord: window means and the four criteria, by hand', () => {
  const r = trimRowRecord(row, sums());
  // f0 = 40/20 = 2, f1 = 41/20 = 2.05: drift 0.05 / 2.025 = 0.024691 > 0.02, so not converged on drift alone.
  assert.ok(Math.abs(r.convergence.freq_drift_frac - 0.05 / 2.025) < 1e-12);
  assert.ok(Math.abs(r.convergence.climb_m_s - 0.05) < 1e-12); // 0.5 m over 10 s
  assert.ok(Math.abs(r.convergence.speed_err_m_s - 0.1) < 1e-12); // 404 / 40 - 10
  assert.equal(r.convergence.tail_drift_rad, 0);
  assert.equal(r.converged, false);
  assert.equal(r.reason, 'not converged');
  assert.equal(r.unflyable, true);
  assert.ok(Math.abs(r.flight_V - 10.1) < 1e-12);
  assert.ok(Math.abs(r.freq - 81 / 40) < 1e-12);
  assert.ok(Math.abs(r.tail - 0.03) < 1e-12 && Math.abs(r.theta - 0.07) < 1e-12 && r.flight_power_w === 80 && r.flight_power_regen_credit_w === 77);
  const ok = trimRowRecord(row, sums({ half: [{ f: 40, tail: 0.6, n: 20 }, { f: 40.8, tail: 0.6, n: 20 }] }));
  assert.ok(ok.convergence.freq_drift_frac < FLIGHT_TRIM_TOL.freq_drift_frac); // 0.04 / 2.02 = 0.0198
  assert.equal(ok.converged, true);
  assert.equal(ok.reason, undefined);
});

test('trimRowRecord: each tolerance bounds on its own; out of bounds or non-finite is diverged', () => {
  const steady = { half: [{ f: 40, tail: 0.6, n: 20 }, { f: 40, tail: 0.6, n: 20 }] };
  assert.equal(trimRowRecord(row, sums({ ...steady, y1: 1 + 1.01 })).converged, false); // climb 0.101 > 0.1
  assert.equal(trimRowRecord(row, sums({ ...steady, y1: 1 + 0.99 })).converged, true);
  assert.equal(trimRowRecord(row, sums({ ...steady, V: 40 * 10.31 })).converged, false); // speed error 0.31 > 0.3
  assert.equal(trimRowRecord(row, sums({ ...steady, V: 40 * 9.69 })).converged, false);
  assert.equal(trimRowRecord(row, sums({ half: [{ f: 40, tail: 0.6, n: 20 }, { f: 40, tail: 0.81, n: 20 }] })).converged, false); // tail 0.0105
  // A criterion exactly at its tolerance passes (each bound is inclusive).
  const at = trimRowRecord(row, sums());
  const tolAt = { climb_m_s: Math.abs(at.convergence.climb_m_s), speed_m_s: Math.abs(at.convergence.speed_err_m_s), freq_drift_frac: at.convergence.freq_drift_frac, tail_drift_rad: 0.01 };
  assert.equal(trimRowRecord(row, sums(), tolAt).converged, true);
  const tailed = sums({ half: [{ f: 40, tail: 0.6, n: 20 }, { f: 40, tail: 0.8, n: 20 }] });
  assert.equal(trimRowRecord(row, tailed, { ...tolAt, tail_drift_rad: trimRowRecord(row, tailed).convergence.tail_drift_rad }).converged, true);
  for (const bad of [{ ok: false }, { n: 0 }, { P: NaN }, { V: Infinity }, { th: NaN }, { tail: NaN }, { half: [{ f: NaN, tail: 0, n: 20 }, { f: 40, tail: 0, n: 20 }] }, { half: [{ f: 40, tail: 0, n: 20 }, { f: NaN, tail: 0, n: 20 }] }]) {
    const r = trimRowRecord(row, sums(bad));
    assert.deepEqual([r.reason, r.converged, r.unflyable, r.freq], ['diverged', false, true, row.freq]);
  }
});

test('flyTrimRow: the window is average seconds of samples, split in halves (v20 fixture, 0.15 s)', async () => {
  const v20 = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v20.json', import.meta.url), 'utf8'));
  const HZ = Math.round(1 / v20.control.dt);
  const s = await flyTrimRow(v20, v20.control.trim[0], { settle: 0.05, average: 0.1 });
  assert.equal(s.ok, true);
  assert.equal(s.n, Math.round(0.1 * HZ));
  assert.equal(s.half[0].n + s.half[1].n, s.n);
  assert.equal(s.half[1].n, Math.round(0.05 * HZ));
  assert.ok(Number.isFinite(s.y0) && Number.isFinite(s.y1) && s.average === 0.1);
  // Released at height 0 and flown 0.15 s: both window heights stay within a few centimetres of it.
  assert.ok(Math.abs(s.y0) < 0.1 && Math.abs(s.y1) < 0.2, `${s.y0} ${s.y1}`);
  // No window: nothing is summed; a non-finite start leaves the bounds at once and the row is diverged.
  const none = await flyTrimRow(v20, v20.control.trim[0], { settle: 0.05, average: 0 });
  assert.deepEqual([none.n, none.tail, none.th, none.P, none.PC, none.V, none.half], [0, 0, 0, 0, 0, 0, [{ f: 0, tail: 0, n: 0 }, { f: 0, tail: 0, n: 0 }]]);
  const lost = await flyTrimRow(v20, { ...v20.control.trim[0], V: NaN }, { settle: 0.05, average: 0.1 });
  assert.deepEqual([lost.ok, lost.n], [false, 0]);
  assert.equal(trimRowRecord(v20.control.trim[0], lost).reason, 'diverged');
  // The +-30 m bound is on height, not on distance flown: 4 s at the trim speed covers about 40 m and stays in bounds.
  const far = await flyTrimRow(v20, v20.control.trim[0], { settle: 3.5, average: 0.5 });
  assert.deepEqual([far.ok, far.n], [true, Math.round(0.5 * HZ)]);
});

test('finishControl: cruise at the flyable row of least flight power, rough-air speed, Vmin under the TECS floor', async () => {
  const { finishControl } = await import('../../src/design/round3.mjs');
  const mk = (converged) => ({ geometry: { span_m: 2 }, control: { Vmin: 10, trimConverged: converged, trimFlight: [{ V: 9.5, flight_power_w: 84 }, { V: 10, flight_power_w: 80 }, { V: 10.5, flight_power_w: 70, unflyable: true }] } });
  const d = mk(true);
  assert.equal(finishControl(d, { Vfloor: 9.5, Vrough: 15, roughMaxBank_deg: 20 }, 0.66, 0.52), null);
  assert.equal(d.control.Vcruise, 10); // 70 W is the unflyable row
  assert.equal(d.control.Vrough, 15);
  assert.ok(Math.abs(d.control.roughMaxBank - 20 * Math.PI / 180) < 1e-15);
  assert.equal(d.control.Vmin, 9.5);
  assert.deepEqual(d.cruise, { V: 10, amp: 0.66, dyn: 0.52, sym: 0, lead: 0 });
  assert.equal(d.control.dynSchedule.rRef_m, 0.75); // 0.75 of the 1 m semispan
  const e = mk(true);
  finishControl(e, { Vfloor: 11 }, 0.66, 0.52);
  assert.deepEqual([e.control.Vrough, e.control.roughMaxBank, e.control.Vmin], [13, undefined, 10]);
  // Equal flight power: the slower (first) row is cruise.
  const tie = mk(true); tie.control.trimFlight[0].flight_power_w = 80;
  finishControl(tie, {}, 0.66, 0.52);
  assert.equal(tie.control.Vcruise, 9.5);
  const f = mk(false);
  assert.deepEqual(finishControl(f, {}, 0.66, 0.52), { failed: 'no free-flight trim converged: [[9.5,null],[10,null],[10.5,null]]' });
  const g = mk(false); delete g.control.trimFlight;
  assert.deepEqual(finishControl(g, {}, 0.66, 0.52), { failed: 'no free-flight trim converged: []' });
  assert.equal(f.control.Vcruise, undefined);
  assert.ok(f.control.tecs, 'the round-3 options go on before the convergence check');
});
