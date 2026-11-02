// Runs a flapping wing pair in UVLM and in the strip model under one kinematic prescription and returns
// time series and cycle means, in body axes at fixed attitude. Lift +y, thrust +x.
import { makeLatticeSpec, wingGrid, jointState, makeStations } from './kinematics.js';
import { createUvlm } from './uvlm.js';
import { createStrip } from './strip.js';

/**
 * op: { U, bodyAlpha (rad), freq, amp, law, sched, rho, mu }
 * The air moves at U past a bird whose body x is pitched by bodyAlpha.
 */
export function uvlmFlappingCycle(wing, op, { nc = 5, ns = 12, stepsPerCycle = 80, cycles = 3, wake = 'prescribed', keepRows = null } = {}) {
  const { U, bodyAlpha = 0, freq, amp, law, sched, rho } = op;
  const ls = makeLatticeSpec(wing, nc, ns);
  const omega = 2 * Math.PI * freq;
  const surface = {
    nc,
    ns,
    grid(t, out) {
      const j = jointState(omega * t, omega, 0, amp, law, sched, +1);
      wingGrid(wing, ls, j, +1, out);
    },
  };
  const dt = 1 / (freq * stepsPerCycle);
  const sim = createUvlm([surface], {
    Vinf: [-U * Math.cos(bodyAlpha), U * Math.sin(bodyAlpha), 0],
    rho, dt, symmetric: true, wake,
    maxWakeRows: keepRows ?? stepsPerCycle * Math.max(1, cycles - 1),
    freeRows: stepsPerCycle,
    wakeCore: 0.02 * (wing.segments[0].chord_in),
  });
  const total = cycles * stepsPerCycle;
  const start = total - stepsPerCycle;
  let Fx = 0, Fy = 0;
  const series = [];
  for (let k = 0; k < total; k++) {
    const f = sim.advance(k >= start);
    if (k >= start) {
      // The solver has the right wing and its image: double x and y, cancel z.
      Fx += 2 * f.force[0];
      Fy += 2 * f.force[1];
      series.push({ t: f.t, phase: (omega * f.t) % (2 * Math.PI), fx: 2 * f.force[0], fy: 2 * f.force[1] });
    }
  }
  return { Fx: Fx / stepsPerCycle, Fy: Fy / stepsPerCycle, series };
}

export function stripFlappingCycle(wing, op, { n = 8, stepsPerCycle = 120, cycles = 4, cal = null, polar = null, inviscid = true, model = null } = {}) {
  const { U, bodyAlpha = 0, freq, amp, law, sched, rho, mu = 1.76e-5 } = op;
  const m = model ?? createStrip(wing, { n, inviscid, cal, polar });
  const omega = 2 * Math.PI * freq;
  const dt = 1 / (freq * stepsPerCycle);
  const vb = [U * Math.cos(bodyAlpha), -U * Math.sin(bodyAlpha), 0];
  m.reset();
  const total = cycles * stepsPerCycle;
  const start = total - stepsPerCycle;
  let Fx = 0, Fy = 0, P = 0, stall = 0;
  const series = [];
  for (let k = 0; k < total; k++) {
    const t = k * dt;
    const R = jointState(omega * t, omega, 0, amp, law, sched, +1);
    const L = jointState(omega * t, omega, 0, amp, law, sched, -1);
    const r = m.evaluate(vb, [0, 0, 0], [0, 0, 0], R, L, rho, mu, dt);
    if (k >= start) {
      Fx += r.F[0];
      Fy += r.F[1];
      P += r.power;
      stall += r.stalledFraction;
      series.push({ t, phase: (omega * t) % (2 * Math.PI), fx: r.F[0], fy: r.F[1] });
    }
  }
  return { Fx: Fx / stepsPerCycle, Fy: Fy / stepsPerCycle, power: P / stepsPerCycle, stalled: stall / stepsPerCycle, series };
}
