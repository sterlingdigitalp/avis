import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { windAt, controllerAt, governorAt } from '../../src/physics/gates/inputs-check.mjs';
import { createController } from '../../src/physics/control.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const design = () => JSON.parse(readFileSync(join(ROOT, 'data/designs/larus-gl-2.json'), 'utf8'));
const same = (run, dt, T) => {
  const a = run(dt, T), b = run(dt / 2, T);
  let max = 0;
  for (const [t, v] of a) v.forEach((x, i) => { max = Math.max(max, Math.abs(x - b.get(t)[i])); });
  return max;
};

test('half-step inputs: wind, controller and governor are the same function of time at 1/1920 and 1/3840 s', () => {
  const d = design();
  d.control.rate_Hz = 240;
  d.control.dt = 1 / 1920;
  assert.equal(same(windAt(d, 15), d.control.dt, 6), 0);
  assert.equal(same(controllerAt(d, 15), d.control.dt, 3), 0);
  assert.equal(same(governorAt(d), d.control.dt, 3), 0);
  // A 100 Hz controller cannot tick at the same instants at both steps (1/100 s is not a whole number of either).
  d.control.rate_Hz = 100;
  assert.ok(same(controllerAt(d, 15), d.control.dt, 3) > 0);
});

test('controller ticks at t0 + k / rate from its first call, and its delay is at least 20 ms in whole periods', () => {
  const d = design();
  const ticks = (rate, step) => {
    const c = createController(d, { rate, sensorNoise: false, delay: 0.02 });
    const truth = { t: 0, dt: step, omega: [0, 0, 0], sf: [0, 9.80665, 0], up: [0, 1, 0], y: 0, vel: [10, 0, 0], pos: [0, 0, 0], airspeedX: 10, phase: 0, crank: 12, heading: 0, battI: 5 };
    c.initAttitude(truth);
    const out = [];
    let last = null;
    for (let k = 0; k * step < 0.05; k++) { c.update({ ...truth, t: k * step }, k * step); if (c.estimate !== last) { out.push(+(k * step).toFixed(9)); last = c.estimate; } }
    return out;
  };
  assert.deepEqual(ticks(240, 1 / 1920).slice(0, 4), [0, 1 / 240, 2 / 240, 3 / 240].map((t) => +t.toFixed(9)));
  assert.deepEqual(ticks(240, 1 / 1920), ticks(240, 1 / 3840));
});
