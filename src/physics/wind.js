// Wind model: mean wind, Dryden continuous turbulence and 1-cosine discrete gusts.
//
// Dryden, MIL-F-8785C form per data/references/flight-standards.md. Filters scaled so the rms equals sigma:
//   u: first order with tau = Lu / V, discretised exactly.
//   v,w: H(s) = sigma sqrt(tau) (1 + sqrt(3) tau s) / (1 + tau s)^2 with unit white noise, tau = L / V.
// Frozen-field assumption: turbulence is convected past the bird at airspeed, which is weak for a bird flying
// in a wind of comparable speed. M4 reports state that limit.
// Turbulence is defined in the wind frame aligned with the mean flow, so it can be sampled in world axes.
// Rotary terms p_g, q_g, r_g come from the spatial gradients over the wing and tail (MIL-F-8785C).

import { makeRng } from './rng.js';

/** MIL-F-8785C low altitude scale lengths and rms ratio, with h in metres, at least 3 m. */
export function drydenLowAltitude(h_m) {
  const h = Math.max(h_m, 3) / 0.3048;
  const k = 0.177 + 0.000823 * Math.min(h, 1000);
  return { Lw_m: Math.min(h, 1000) * 0.3048, Lu_m: (Math.min(h, 1000) / Math.pow(k, 1.2)) * 0.3048, ratioW: Math.pow(k, 0.4) };
}

/**
 * spec: { mean: [x,y,z] world m/s, uRms, vRms, wRms, Lu, Lv, Lw, wingspan, gusts: [{t, H, amp:[x,y,z]}] }
 * rms and scales default from drydenLowAltitude at agl 30 m with uRms horizontal.
 */
export function createWind(spec, seed = 1) {
  const rng = makeRng(seed);
  const mean = spec.mean ?? [0, 0, 0];
  const sigU = spec.uRms ?? 0;
  const low = drydenLowAltitude(spec.agl ?? 30);
  const sigV = spec.vRms ?? sigU;
  const sigW = spec.wRms ?? sigU * low.ratioW;
  const Lu = spec.Lu ?? low.Lu_m;
  const Lv = spec.Lv ?? low.Lu_m;
  const Lw = spec.Lw ?? low.Lw_m;
  const b = spec.wingspan ?? 1.5;
  const gusts = spec.gusts ?? [];
  const sqrt3 = Math.sqrt(3);
  // state: filter states. u1; v1,v2; w1,w2; pg; qg-lagged w; rg-lagged v
  let u1 = 0, v1 = 0, v2 = 0, w1 = 0, w2 = 0, pg = 0, qgLag = 0, rgLag = 0;
  let qg = 0, rg = 0;
  const lastOut = { world: [0, 0, 0], rates: [0, 0, 0] };
  let primed = false;

  function second(y1, y2, sigma, tau, dt, n) {
    // y1' = (-y1 + K u)/tau, y2' = (-y2 + y1 + sqrt3 tau y1')/tau, u = n / sqrt(dt)
    const K = sigma * Math.sqrt(tau);
    const u = n / Math.sqrt(dt);
    const dy1 = (-y1 + K * u) / tau;
    const y1n = y1 + dt * dy1;
    const out = y2 + (dt * (-y2 + y1 + sqrt3 * tau * dy1)) / tau;
    return [y1n, out];
  }

  /** Advance turbulence by dt at airspeed V (m/s) and return wind at time t in world axes, plus rotary terms. */
  function step(t, dt, V) {
    const v = Math.max(V, 2);
    if (sigU > 0 || sigV > 0 || sigW > 0) {
      const tauU = Lu / v;
      const eU = Math.exp(-dt / tauU);
      if (!primed) {
        // Start from the stationary distribution so the first seconds are not artificially calm.
        u1 = sigU * rng.normal();
        const tv = Lv / v, tw = Lw / v;
        // Run the 2nd-order filters a few time constants to reach their stationary state.
        const mx = Math.ceil((5 * Math.max(tv, tw)) / dt);
        for (let i = 0; i < Math.min(mx, 20000); i++) {
          [v1, v2] = second(v1, v2, sigV, tv, dt, rng.normal());
          [w1, w2] = second(w1, w2, sigW, tw, dt, rng.normal());
        }
        primed = true;
      }
      u1 = u1 * eU + sigU * Math.sqrt(1 - eU * eU) * rng.normal();
      const tv = Lv / v;
      const tw = Lw / v;
      [v1, v2] = second(v1, v2, sigV, tv, dt, rng.normal());
      [w1, w2] = second(w1, w2, sigW, tw, dt, rng.normal());
      // Rotary terms. MIL-F-8785C H_p = sigma_w sqrt(0.8/V) (pi/(4b))^(1/6) / (L_w^(1/3) (1 + tau s)) with noise
      // density pi. (flight-standards.md prints sigma_w/(0.8 V), which is dimensionally wrong: it should be
      // sqrt(0.8/V), as here.) rms of the first order filter = K sqrt(pi / (2 tau)).
      const tauP = (4 * b) / (Math.PI * v);
      const eP = Math.exp(-dt / tauP);
      const kP = sigW * Math.sqrt(0.8 / v) * Math.pow(Math.PI / (4 * b), 1 / 6) * Math.pow(Lw, -1 / 3);
      const sigP = kP * Math.sqrt(Math.PI / (2 * tauP));
      pg = pg * eP + sigP * Math.sqrt(1 - eP * eP) * rng.normal();
      // Pitch and yaw gust rates: gradients of frozen turbulence along the body, dw/dx = (dw/dt)/V.
      const eQ = Math.exp(-dt / tauP);
      const wPrev = qgLag;
      qgLag = eQ * qgLag + (1 - eQ) * w2;
      qg = (qgLag - wPrev) / (dt * v);
      const vPrev = rgLag;
      const tauR = (3 * b) / (Math.PI * v);
      const eR = Math.exp(-dt / tauR);
      rgLag = eR * rgLag + (1 - eR) * v2;
      rg = (rgLag - vPrev) / (dt * v);
    }
    // Align: u along the mean wind's horizontal direction (or x when calm), w vertical, v lateral to it.
    const mh = Math.hypot(mean[0], mean[2]);
    const ex = mh > 1e-6 ? mean[0] / mh : 1;
    const ez = mh > 1e-6 ? mean[2] / mh : 0;
    const wx = mean[0] + u1 * ex - v2 * ez;
    const wy = mean[1] + w2;
    const wz = mean[2] + u1 * ez + v2 * ex;
    // 1-cosine gust of length H, returning to zero: v(x) = (A/2)(1 - cos(2 pi x / H)), x = V_air (t - t0).
    let gx = 0, gy = 0, gz = 0;
    for (let i = 0; i < gusts.length; i++) {
      const g = gusts[i];
      const T = g.H / v;
      const d = t - g.t;
      if (d >= 0 && d <= T) {
        const k = 0.5 * (1 - Math.cos((2 * Math.PI * d) / T));
        gx += g.amp[0] * k;
        gy += g.amp[1] * k;
        gz += g.amp[2] * k;
      }
    }
    lastOut.world[0] = wx + gx;
    lastOut.world[1] = wy + gy;
    lastOut.world[2] = wz + gz;
    // Rates the air presents to the bird, as subtracted from body rates in the flight model. A lateral gust
    // gradient acts like a nose-left body rate of opposite sign; see test/physics/wind.test.js.
    lastOut.rates[0] = pg;
    lastOut.rates[1] = -rg;
    lastOut.rates[2] = qg;
    return lastOut;
  }

  return { step, sigmas: { sigU, sigV, sigW }, scales: { Lu, Lv, Lw } };
}

/** SPEC M4: 1-cosine gusts with H drawn per seed from {5,10,20,40} m, vertical and lateral, at random phase. */
export const GUST_LENGTHS_M = [5, 10, 20, 40];
export function randomGusts(rng, { t0, t1, count = 4, amp = 4, H = null }) {
  const out = [];
  const pick = H ?? GUST_LENGTHS_M[Math.floor(rng.uniform() * GUST_LENGTHS_M.length) % GUST_LENGTHS_M.length];
  for (let i = 0; i < count; i++) {
    const t = t0 + ((t1 - t0) * (i + rng.uniform())) / count; // uniform in the slot: random phase of the flap
    const vertical = i % 2 === 0;
    const s = rng.uniform() < 0.5 ? -1 : 1;
    out.push({ t, H: pick, amp: vertical ? [0, s * amp, 0] : [0, 0, s * amp], kind: vertical ? 'vertical' : 'lateral' });
  }
  return out;
}
