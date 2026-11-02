// Crank-speed governor and drive-current clamp in the flight controller (round 5, SPEC ruling "Control functions need
// real hardware"). The law is round 4's (feed-forward voltage for the commanded crank speed, PI on the speed error,
// duty <= (E + I_clamp R) / V when the drive pushes, no integration against the clamp), but it runs sampled at the
// controller's own rate from the sensors in the BOM: the crank angle from a magnetic encoder at its published
// resolution, the speed from encoder differences over a short window, and the battery voltage from the flight
// controller's sense input at its published resolution. Duty computed at one tick reaches the ESC (DShot) after the
// path's latency and is held in between. Round 6 (SPEC ruling after round 5): the robustness corner's 20 ms control
// delay applies to this loop too, so the latency is the delay rounded up to whole controller periods (at least one
// period, the path's own compute and DShot frame time). The twist schedule's crank phase comes from the same tick as
// the duty in effect (the sample taken one latency ago, advanced at the speed measured then).
const TWO_PI = 2 * Math.PI;

const wrap = (a) => {
  const r = a % TWO_PI;
  return r < 0 ? r + TWO_PI : r;
};
const dWrap = (d) => d - TWO_PI * Math.round(d / TWO_PI);

/** Controller periods between computing a duty and the ESC applying it: the delay rounded up, at least one period. */
export function latencyTicks(rate_Hz, delay_s = 0) {
  return Math.max(1, Math.ceil(delay_s * rate_Hz - 1e-9));
}

/**
 * gov: { rate_Hz, encoder: { bits, window }, voltage: { lsb_V, tau_s }, clamp_A }.
 * motor: { G, kv (rad/s/V), R, cells, kp, ki, delay_s } with kp, ki the round-4 governor gains (volts per crank rad/s
 * over G / kv, and per second) and delay_s the control delay (latencyTicks). Returns { st, T, lag, reset, due, tick,
 * phaseAt }.
 */
export function createFcGovernor(gov, { G, kv, R, cells, kp = 0.5, ki = 1.0, delay_s = 0 }) {
  const T = 1 / gov.rate_Hz;
  const lag = latencyTicks(gov.rate_Hz, delay_s);
  const q = TWO_PI / 2 ** gov.encoder.bits;
  const win = Math.max(1, Math.round(gov.encoder.window ?? 2));
  const lsbV = gov.voltage?.lsb_V ?? 0;
  const tauV = gov.voltage?.tau_s ?? 0;
  const KpV = kp * (G / kv);
  const Ki = ki * KpV;
  const iMax = 0.5 * cells * 4.2;
  const clampA = gov.clamp_A;
  const quant = (a) => Math.round(wrap(a) / q) * q;
  const st = { next: 0, hist: [], iInt: 0, vF: null, applied: 0, pending: 0, clampedApplied: false, clampedPending: false, angle: 0, tAngle: 0, rate: 0, fresh: true, ticks: 0, line: [], inEffect: { angle: 0, rate: 0, tAngle: 0 } };

  function measure(t, phase, V) {
    const a = quant(phase);
    st.hist.push({ t, a });
    while (st.hist.length > win + 1) st.hist.shift();
    if (st.hist.length > 1) {
      let sum = 0;
      for (let i = 1; i < st.hist.length; i++) sum += dWrap(st.hist[i].a - st.hist[i - 1].a);
      st.rate = sum / (t - st.hist[0].t);
    }
    st.angle = a;
    st.tAngle = t;
    const vq = lsbV > 0 ? Math.round(V / lsbV) * lsbV : V;
    st.vF = st.vF === null || !(tauV > 0) ? vq : st.vF + (T / (tauV + T)) * (vq - st.vF);
  }

  /** The governor law on the measured speed and voltage. want: crank speed target (rad/s). */
  function law(want, off, integrate) {
    if (off) {
      st.iInt = 0;
      return { duty: 0, clamped: false };
    }
    const V = Math.max(st.vF, 1e-3);
    const err = want - st.rate;
    let d = Math.max(0, Math.min(1, ((want * G) / kv + KpV * err + st.iInt) / V));
    let clamped = false;
    if (Number.isFinite(clampA)) {
      const dMax = ((st.rate * G) / kv + clampA * R) / V;
      if (d > dMax) { d = Math.max(0, dMax); clamped = true; }
    }
    if (integrate) {
      if (!(clamped && err > 0)) st.iInt = Math.max(-iMax, Math.min(iMax, st.iInt + T * Ki * err));
    } else st.iInt = 0;
    return { duty: d, clamped };
  }

  return {
    st, T, lag,
    /** Start from a known state (a flight that begins in trim): encoder history at the true crank rate. */
    reset({ t = 0, phase = 0, crank = 0, V, iInt = 0 }) {
      st.hist = [];
      for (let k = win; k >= 1; k--) st.hist.push({ t: t - k * T, a: quant(phase - crank * k * T) });
      st.vF = null;
      st.iInt = iInt;
      st.next = t;
      st.fresh = true;
      st.applied = 0; st.pending = 0; st.clampedApplied = false; st.clampedPending = false;
      st.line = [];
      measure(t - 1e-12, phase, V); // fills rate and voltage; the first tick re-samples at t
      st.hist.pop();
    },
    due(t) { return t >= st.next - 1e-9; },
    /**
     * One controller tick at time t: the ESC takes the duty computed lag ticks ago, the controller samples and computes
     * the next. wantFn(phaseEst, rateEst). A fresh start (a flight that begins in trim) fills the line with the first
     * duty, as if the same command had been sent for the whole latency.
     */
    tick(t, phase, V, wantFn, off, integrate) {
      measure(t, phase, V);
      const r = law(wantFn(st.angle, st.rate), off, integrate);
      st.pending = r.duty;
      st.clampedPending = r.clamped;
      const e = { duty: r.duty, clamped: r.clamped, angle: st.angle, rate: st.rate, tAngle: st.tAngle };
      if (st.fresh) {
        st.line = Array.from({ length: lag }, () => e);
        st.fresh = false;
      }
      st.line.push(e);
      const a = st.line.shift();
      st.applied = a.duty;
      st.clampedApplied = a.clamped;
      st.inEffect = { angle: a.angle, rate: a.rate, tAngle: a.tAngle };
      while (st.next <= t + 1e-9) st.next += T;
      st.ticks++;
    },
    /**
     * Crank phase as the controller's command in effect at time t knows it: the encoder sample of the tick that computed
     * it (one latency ago) advanced at the speed measured then.
     */
    phaseAt(t) { return st.inEffect.angle + st.inEffect.rate * (t - st.inEffect.tAngle); },
  };
}
