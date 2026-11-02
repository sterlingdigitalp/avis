// Grid-study arithmetic against sequences whose order and limit are known in closed form.
import test from 'node:test';
import assert from 'node:assert/strict';
import { observedOrder, gridStudy } from '../src/gridstudy.mjs';

const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

test('observed order and Richardson limit of A + B h^p on grids refined by a constant ratio', () => {
  for (const [p, ratio] of [[1, 2], [2, 2], [1.5, Math.SQRT2], [3, Math.SQRT2]]) {
    const cs = [40, 40 * ratio, 40 * ratio * ratio];
    const vals = cs.map((c) => 0.2 + 0.7 * c ** -p);
    const o = observedOrder(vals, ratio);
    close(o.observed_order, p, 1e-9, `order p=${p}`);
    close(o.richardson_limit, 0.2, 1e-12, `limit p=${p}`);
    assert.equal(o.monotone, true);
  }
});

test('growing differences of one sign give a negative order; a zero or degenerate step gives none', () => {
  const g = observedOrder([0.1, 0.11, 0.13], 2);
  close(g.observed_order, -1, 1e-12, 'differences 0.01, 0.02');
  close(g.richardson_limit, 0.09, 1e-12, 'limit of a diverging sequence');
  assert.equal(g.monotone, true);
  assert.ok(Number.isNaN(observedOrder([0.1, 0.1, 0.2], 2).observed_order));
  assert.ok(Number.isNaN(observedOrder([0.3, 0.2, 0.15], 1).richardson_limit));
  const two = observedOrder([0.3, 0.2], 2);
  assert.equal(two.monotone, true);
  assert.ok(Number.isNaN(two.observed_order));
});

test('differences of opposite sign have no observed order', () => {
  const o = observedOrder([0.21827, 0.21977, 0.21728], Math.SQRT2);
  assert.equal(o.monotone, false);
  assert.ok(o.difference_ratio < 0);
  assert.ok(Number.isNaN(o.observed_order) && Number.isNaN(o.richardson_limit));
});

test('grid study skips diverged and missing levels, uses the finest three, reports every triple', () => {
  const r = (amp) => ({ cl: { amp, phaseDeg: 10 }, meanCd: 0.05, ampErrPct: 0, phaseErrDeg: 0 });
  const levels = [{ c: 20, r: { cl: { amp: 99, phaseDeg: 0 }, meanCd: -1, ampErrPct: 0, phaseErrDeg: 0 } }, { c: 40, r: r(0.3) }, { c: 80, r: r(0.25) }, { c: 160, r: r(0.2375) }, { c: 320, r: r(0.234375) }];
  const s = gridStudy(levels, (x) => !(x.meanCd > 0));
  assert.deepEqual(s.diverged, [20]);
  assert.deepEqual(s.used_for_order, [80, 160, 320]);
  assert.equal(s.complete, true);
  assert.equal(s.triples.length, 2);
  close(s.amp.observed_order, 2, 1e-9, 'differences 0.0125, 0.003125');
  const gap = gridStudy([{ c: 40, r: r(0.3) }, { c: 80 }, { c: 160, r: r(0.2) }], () => false);
  assert.deepEqual(gap.missing, [80]);
  assert.equal(gap.complete, false);
  assert.equal(gridStudy([...levels.slice(1), { c: 640 }], () => false).complete, false);
  assert.equal(gridStudy([{ c: 40, r: r(0.3) }, { c: 80, r: r(0.2) }, { c: 160, r: r(0.2) }], (x) => x.cl.amp > 0.25).complete, false);
});
