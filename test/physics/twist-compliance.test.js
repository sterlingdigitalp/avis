// Round 18 (diagnosis 2.1): the twist chain's relaxed deflection state (flight.js relax, TAU_CHAIN_S), unit-tested
// against its own closed-form recursion and a first-order lag's known frequency response, independent of any aircraft.
// The behavioural tests at v24's trim point with v25's chain are in twist-compliance-flight.test.js.
import test from 'node:test';
import assert from 'node:assert/strict';
import { relax, TAU_CHAIN_S } from '../../src/physics/flight.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''}: ${a} vs ${b} (tol ${tol})`);

test('relax(): matches its own closed-form recursion for a constant target, and a first-order lag\'s frequency response for a sinusoid', () => {
  // x_{n+1} = relax(x_n, T, dt, tau) = a x_n + (1 - a) T, a = tau / (tau + dt): a linear recursion with closed form
  // x_n = T - (T - x_0) a^n. Independent of the design or any flight physics: pure algebra on the update rule.
  const dt = 1 / 240, tau = TAU_CHAIN_S, T = 0.208256 * 2.5, a = tau / (tau + dt);
  let x = 0;
  for (let n = 1; n <= 50; n++) {
    x = relax(x, T, dt, tau);
    const expected = T - T * a ** n;
    near(x, expected, 1e-12 * Math.max(1, Math.abs(expected)), `relax closed form, step ${n}`);
  }
  // A sinusoidal target through a first-order lag settles to amplitude |input| / sqrt(1 + (w tau)^2), for small dt
  // against both tau and the period. dt 0.5 ms against tau 20 ms and period 200 ms is small on both counts.
  const dt2 = 0.0005, T0 = 2.5, f = 5, w = 2 * Math.PI * f;
  let x2 = 0, t = 0;
  const steps = Math.round(1.0 / dt2), perSteps = Math.round((1 / f) / dt2);
  const last = [];
  for (let i = 0; i < steps; i++) {
    t += dt2;
    x2 = relax(x2, T0 * Math.sin(w * t), dt2, tau);
    if (i >= steps - perSteps) last.push(x2);
  }
  const amp = (Math.max(...last) - Math.min(...last)) / 2;
  const analytic = T0 / Math.sqrt(1 + (w * tau) ** 2);
  assert.ok(Math.abs(amp - analytic) / analytic < 0.01, `sinusoid amplitude ${amp} vs analytic ${analytic}`);
});

