// Fast strip model: blade element + reduced frequency correction + low-Re polars with stall.
// Runs many times faster than real time in a browser. Calibrated to UVLM and later to CFD with bounded
// multipliers kept in data/calibration/<design>.json.
//
// Each station has:
//  - relative velocity at the 3/4-chord point, from body motion, flap, twist and local wind
//  - induced downwash from a Weissinger horseshoe matrix (the 3/4-chord lifting line) minus the 2D part
//    the section polar already holds
//  - Wagner lag via R.T. Jones's two-state approximation of Theodorsen's C(k), so it works for arbitrary
//    motion and gusts, not just a harmonic
//  - apparent mass along the section normal: rho pi c^2/4 d(Vn)/dt at mid chord (Theodorsen's L_nc)
//  - lift normal to the local velocity (so a section moving down gets a forward force, which is flapping
//    thrust out of Kutta-Joukowski and not out of a thrust factor) and profile drag along it

import { makeStations, makeSectionState, sectionStates } from './kinematics.js';
import { POLARS, polarEval, inviscidEval, thinAirfoilCamber } from './polars.js';
import { luFactor, luSolve } from './uvlm.js';

/**
 * Leading-edge suction efficiency of the viscous model (DeLaurier 1993's eta_s): the fraction of the ideal suction
 * force a section realises. Never fitted to SmartBird or any flight result. The inviscid model (V1, V3, V5) keeps
 * full suction, as the lattice it is compared with does. Source and value: see SUCTION.source.
 */
export const SUCTION = Object.freeze({
  eta: 0.6,
  status: 'assumption: no independent measurement at Re ~ 1e5 for a film-over-rib wing with a small rod leading edge was found',
  source: 'Value taken from the one published ornithopter value tied to a comparable vehicle: Lee, Yang, Lee & Han (2025), J. Bionic Eng. 22:144-156, doi 10.1007/s42235-024-00615-2, Table 3, eta_s = 0.6 for a 1.2 m polyester-fabric-over-spar wing at about 6 m/s (Re about 1e5). That value was identified from their own hardware trim, jointly with an unusually high friction drag coefficient, so it is not an independent measurement of eta_s.',
  published_range: 'Other published values, all assumed or copied rather than measured: 0.98 (Liebeck LPT section, Zakaria et al., via Azam & Banik 2026), 0.18 (flat plate, Valiyff et al. 2010 citing Kim et al. 2008, unverified), 0 (Malik & Ahmad 2010). DeLaurier 1993 (the model eta_s comes from) could not be read in full.',
  how_used: 'Realised chordwise force = min(measured quasi-steady polar form L sin a - D cos a, eta_s L sin a - D_f cos a), D_f the polar minimum (friction) drag at that Re; so the model is never more optimistic than the measured section polar or than eta_s of the ideal suction. eta_s = 1 recovers round 1 exactly. Never fitted to SmartBird or any flight result.',
  to_measure: 'LBM and OpenFOAM: our wing section (film over ribs with the 2 mm rod leading edge, and E387 for comparison) in 2D harmonic plunge at Re 1e5, k = 0.2-0.5, h/c = 0.3-1.5, mean incidence 0-6 deg: cycle-mean chordwise force against the inviscid (Garrick) suction; eta_s = (mean chordwise force + friction drag) / mean ideal suction.',
});

export const UNCALIBRATED = Object.freeze({ lift: 1, suction: 1, induced: 1, lag: 1, apparentMass: 1, drag: 1 });

/** Physically reasonable limits for the multipliers. A fit may not leave these. */
export const CALIBRATION_BOUNDS = Object.freeze({
  lift: [0.8, 1.2],
  suction: [0.6, 1.2],
  induced: [0.5, 1.5],
  lag: [0.5, 2.0],
  apparentMass: [0.5, 1.5],
  drag: [0.7, 1.6],
});

export function clampCalibration(cal) {
  const out = { ...UNCALIBRATED };
  for (const k of Object.keys(CALIBRATION_BOUNDS)) {
    if (cal && Number.isFinite(cal[k])) out[k] = Math.max(CALIBRATION_BOUNDS[k][0], Math.min(CALIBRATION_BOUNDS[k][1], cal[k]));
  }
  return out;
}

const A1 = 0.165, B1 = 0.0455, A2 = 0.335, B2 = 0.3; // Jones's approximation of Wagner's function

function horseshoeW(px, pz, qz1, qz2, x34) {
  // Downward velocity at a point x34 behind the bound line (spanwise coordinate pz, in the plane) from a
  // horseshoe of unit circulation on qz1..qz2 at x = 0, with legs trailing straight back.
  const x = x34;
  const dz1 = pz - qz1;
  const dz2 = pz - qz2;
  const bound = (1 / (4 * Math.PI * x)) * (dz1 / Math.hypot(x, dz1) - dz2 / Math.hypot(x, dz2));
  const leg = (dz) => (Math.abs(dz) < 1e-9 ? 0 : (1 / (4 * Math.PI * dz)) * (1 + x / Math.hypot(x, dz)));
  return bound + leg(dz1) - leg(dz2);
}

/**
 * Influence of circulation on downwash, with the 2D self-effect of a station's own bound vortex removed
 * because the section polar's 2D slope already contains it. Built for the flat wing and both sides.
 * Index: 0..n-1 right root to tip, n..2n-1 left root to tip.
 */
function buildInducedMatrix(wing, st, offsetFrac) {
  const n = st.n;
  const z = new Float64Array(n);
  const z1 = new Float64Array(n);
  const z2 = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    z[i] = wing.hinge[2] + st.s[i];
    z1[i] = z[i] - st.ds[i] / 2;
    z2[i] = z[i] + st.ds[i] / 2;
  }
  const N = 2 * n;
  const D = new Float64Array(N * N);
  for (let i = 0; i < N; i++) {
    const ri = i < n ? i : i - n;
    const pz = i < n ? z[ri] : -z[ri];
    const xp = Math.max(offsetFrac * st.chord[ri], 1e-6 * st.chord[ri]);
    for (let j = 0; j < N; j++) {
      const rj = j < n ? j : j - n;
      const q1 = j < n ? z1[rj] : -z2[rj];
      const q2 = j < n ? z2[rj] : -z1[rj];
      let w = horseshoeW(pz, pz, q1, q2, xp);
      // The station's own bound vortex is already in the polar's 2D slope. It is also the part that is
      // singular as the offset goes to zero, so for the force-tilt matrix (offset 0) it goes too.
      if (i === j) w -= (1 / (2 * Math.PI * xp));
      D[i * N + j] = w;
    }
  }
  return D;
}

/**
 * Create a model for one bird's wing pair.
 *  wing: makeWing() result. opts: { n, polar: name|object, inviscid, cal, x14 }.
 * Inputs each evaluation are in the body frame.
 */
export function createStrip(wing, opts = {}) {
  const n = opts.n ?? 8;
  const st = makeStations(wing, n);
  const polar = typeof opts.polar === 'string' ? POLARS[opts.polar] : opts.polar ?? POLARS['e387-film'];
  const inviscid = !!opts.inviscid;
  const suctionEta = inviscid ? 1 : (opts.suctionEta ?? SUCTION.eta);
  let cal = clampCalibration(opts.cal);
  // The lattice it is compared with uses the wing's own mean line, so the inviscid section does too (round 1 used
  // the parabolic -2f, which is 10% off for a NACA-type line with camber at 40%: 0.4 deg on E387).
  const camber = opts.parabolicCamber ? wing.camber : thinAirfoilCamber(wing.camber ?? 0, wing.camber_pos ?? 0.4);
  let aeroScale = 1; // robustness scaling of all aero coefficients

  const R = makeSectionState(n);
  const L = makeSectionState(n);
  const N = 2 * n;
  const D = buildInducedMatrix(wing, st, 0.5); // downwash at the 3/4 chord: sets the section's angle
  const Df = buildInducedMatrix(wing, st, 1e-3); // downwash at the bound line: tilts the force (induced drag)
  // Frozen Jacobian: (I + diag(c a / 2) D) with the nominal section slope.
  const P = new Float64Array(N * N);
  const piv = new Int32Array(N);
  const slopeNom = inviscid ? 2 * Math.PI : polar.a;
  function factorP() {
    for (let i = 0; i < N; i++) {
      const ri = i < n ? i : i - n;
      const g = 0.5 * st.chord[ri] * slopeNom * cal.lift * cal.induced;
      for (let j = 0; j < N; j++) P[i * N + j] = g * D[i * N + j] + (i === j ? 1 : 0);
    }
    luFactor(P, N, piv);
  }
  factorP();

  // Unsteady and induced state, per station on both wings
  const x1 = new Float64Array(N);
  const x2 = new Float64Array(N);
  const gam = new Float64Array(N);
  const wi = new Float64Array(N);
  const wf = new Float64Array(N);
  const res = new Float64Array(N);
  const vn1 = new Float64Array(N); // previous mid-chord normal velocity
  const vn2 = new Float64Array(N);
  const vnDot = new Float64Array(N);
  // Crank-rate coefficient gc of each section's normal velocity and its history (moving-body apparent mass).
  const gc1 = new Float64Array(N);
  const gc2 = new Float64Array(N);
  const gcDot = new Float64Array(N);
  let histCount = 0;
  let primeNext = false; // start the Wagner states at their quasi-steady value on the next advancing step

  // Section outputs (per evaluation)
  const out = {
    F: new Float64Array(3),
    M: new Float64Array(3),
    Qphi: new Float64Array(2), // aero generalised force on the shoulder angle, right then left
    Qtheta: new Float64Array(2), // aero generalised force on the commanded twist
    fx: new Float64Array(N),
    fy: new Float64Array(N),
    fz: new Float64Array(N),
    alpha: new Float64Array(N),
    alphaEff: new Float64Array(N),
    cl: new Float64Array(N),
    speed: new Float64Array(N),
    attached: new Float64Array(N),
    fAm: new Float64Array(N), // apparent-mass force per unit span along the section normal (N/m)
    fCirc: new Float64Array(N), // circulatory normal force per unit span (N/m)
    stalledFraction: 0,
    power: 0, // aero power into the air by wing motion relative to the body
    gammaSum: 0,
    CLinst: 0,
    meanAlpha: 0,
    Ma: new Float64Array(49), // added-mass matrix on (a, wdot, crank accel): implicit part of apparent mass
    rootMoment: new Float64Array(6), // moment of aero at each shoulder about that shoulder, right then left
    // force and point are given for inertial accumulation in the flight model
    A: R.A,
  };
  const pol = new Float64Array(5);
  const G7 = new Float64Array(7);
  const S = opts.refArea ?? (2 * wing.segments.reduce((p, g) => p + 0.5 * (g.chord_in + g.chord_out) * g.length, 0));

  function reset() {
    x1.fill(0);
    x2.fill(0);
    gam.fill(0);
    wi.fill(0);
    wf.fill(0);
    vn1.fill(0);
    vn2.fill(0);
    vnDot.fill(0);
    gc1.fill(0);
    gc2.fill(0);
    gcDot.fill(0);
    histCount = 0;
  }

  /**
   * After reset(), a flight that starts in trim has had its circulation for a long time: start the lag states at
   * their quasi-steady values instead of from rest (a start from rest gives half the lift on the first step).
   */
  function prime() {
    primeNext = true;
  }

  /**
   * vb body velocity (m/s), om angular velocity (rad/s), wind local air velocity (m/s), all body axes.
   * jR, jL: jointState results. rho, mu: air. dt > 0 advances the internal states; dt = 0 holds them.
   * windFn(x, y, z, out) optional local wind at a point (gust gradient); otherwise uniform.
   * Forces are returned in body axes. Moments about the body origin.
   */
  function evaluate(vb, om, wind, jR, jL, rho, mu, dt, bodyAcc = null) {
    sectionStates(wing, st, jR, +1, R);
    sectionStates(wing, st, jL, -1, L);
    const hx = wing.hinge[0], hy = wing.hinge[1], hz = wing.hinge[2];
    const nu = mu / rho;
    let Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
    let Qpr = 0, Qpl = 0, Qtr = 0, Qtl = 0;
    let stalled = 0, area = 0, gsum = 0, alphaSum = 0;
    let power = 0;
    const Ma = out.Ma;
    Ma.fill(0);
    const crankNow = bodyAcc ? bodyAcc.crank ?? 0 : 0;
    const dphNow = bodyAcc ? bodyAcc.dph : 0;
    // Round 4: each wing's own velocity ratio when one crank pin drives both (flight.js, pinLayout 'shared').
    const dphW = bodyAcc?.dphW;
    let rm0x = 0, rm0y = 0, rm0z = 0, rm1x = 0, rm1y = 0, rm1z = 0;
    const kAm = cal.apparentMass * aeroScale;
    const kCl = cal.lift * aeroScale;
    const kCd = cal.drag * aeroScale;
    const kS0 = cal.suction; // calibration of the inviscid suction to the lattice (V5); eta_s is applied below
    const kI = cal.induced;
    const kLag = cal.lag;
    for (let w = 0; w < 2; w++) {
      const side = w === 0 ? 1 : -1;
      const W = w === 0 ? R : L;
      for (let i = 0; i < n; i++) {
        const q = i + w * n;
        const j3 = 3 * i;
        const c = st.chord[i];
        const ds = st.ds[i];
        const tx = W.t[j3], ty = W.t[j3 + 1], tz = W.t[j3 + 2];
        const bx = W.b[j3], by = W.b[j3 + 1], bz = W.b[j3 + 2];
        const ax = W.A[j3], ay = W.A[j3 + 1], az = W.A[j3 + 2];
        const tdx = W.td[j3], tdy = W.td[j3 + 1], tdz = W.td[j3 + 2];
        const adx = W.Ad[j3], ady = W.Ad[j3 + 1], adz = W.Ad[j3 + 2];
        // Distances forward of the spar axis along t, for the 3/4, 1/4 and mid chord points
        const dLE = st.xle[i] - wing.spar_x;
        const d34 = dLE - 0.75 * c;
        const d14 = dLE - 0.25 * c;
        const d12 = dLE - 0.5 * c;
        const v = (d) => {
          const px = ax + d * tx, py = ay + d * ty, pz = az + d * tz;
          // Air velocity relative to that point: wind - (vb + om x p + pdot)
          vRelX = wind[0] - vb[0] - (om[1] * pz - om[2] * py) - (adx + d * tdx);
          vRelY = wind[1] - vb[1] - (om[2] * px - om[0] * pz) - (ady + d * tdy);
          vRelZ = wind[2] - vb[2] - (om[0] * py - om[1] * px) - (adz + d * tdz);
        };
        v(d34);
        const vc = -(vRelX * tx + vRelY * ty + vRelZ * tz); // along the chord, LE to TE
        const vn = vRelX * bx + vRelY * by + vRelZ * bz; // up through the section
        const V2 = vc * vc + vn * vn;
        const V = Math.sqrt(V2) + 1e-9;
        const alphaGeo = Math.atan2(vn, vc);
        // Induced downwash reduces the angle
        let alphaI = (kI * wi[q]) / V;
        if (alphaI > 0.3) alphaI = 0.3;
        else if (alphaI < -0.3) alphaI = -0.3;
        const alphaE = alphaGeo - alphaI;
        const rate = (2 * V * kLag) / c;
        if (dt > 0 && primeNext) {
          x1[q] = A1 * alphaE;
          x2[q] = A2 * alphaE;
        }
        const alphaLag = (1 - A1 - A2) * alphaE + x1[q] + x2[q];
        if (dt > 0) {
          // Exact exponential integrator of the two lag states with alphaE held over the step.
          const e1 = Math.exp(-B1 * rate * dt);
          const e2 = Math.exp(-B2 * rate * dt);
          x1[q] = x1[q] * e1 + A1 * alphaE * (1 - e1);
          x2[q] = x2[q] * e2 + A2 * alphaE * (1 - e2);
        }
        const Re = (V * c) / nu;
        if (inviscid) inviscidEval(camber, alphaLag, kCl, pol);
        else polarEval(polar, alphaLag, Re, kCl, kCd, pol);
        const cl = pol[0], cd = pol[1], cm = pol[2], sigma = pol[3];
        const qdyn = 0.5 * rho * V2;
        // Circulation target and iteration on the lifting line, frozen Jacobian.
        const gTarget = 0.5 * V * c * cl;
        // Past stall the section's slope is negative, so the frozen-Jacobian step is relaxed there.
        if (dt > 0) res[q] = (gTarget - gam[q]) * (sigma < 0.7 ? 0.3 : 1);
        // Lift normal to the local velocity at the bound line, including trailing-vortex downwash there;
        // profile drag along it. In chord/normal axes:
        v(d14);
        const alphaF = Math.atan2(vRelX * bx + vRelY * by + vRelZ * bz, -(vRelX * tx + vRelY * ty + vRelZ * tz)) - (kI * wf[q]) / V;
        const ca = Math.cos(alphaF);
        const sa = Math.sin(alphaF);
        const Lp = qdyn * c * cl;
        const Dp = qdyn * c * cd;
        // force per span along b (normal) and along t (towards the LE: suction when positive)
        const fN = Lp * ca + Dp * sa;
        // Chordwise: suction towards the LE minus drag. With partial suction (eta_s < 1) the realised force is the
        // lower of the measured-polar form and eta_s of the ideal suction less friction drag (see SUCTION).
        const sIdeal = kS0 * (Lp * sa);
        let fT = sIdeal - Dp * ca;
        if (suctionEta < 1 && sIdeal > 0) {
          const fEta = suctionEta * sIdeal - qdyn * c * pol[4] * ca;
          if (fEta < fT) fT = fEta;
        }
        // Apparent mass acts along the normal at mid chord (Theodorsen's L_nc), with c/2 = b
        v(d12);
        const vnMid = vRelX * bx + vRelY * by + vRelZ * bz;
        const mAm = kAm * rho * Math.PI * 0.25 * c * c;
        const pmx = ax + d12 * tx, pmy = ay + d12 * ty, pmz = az + d12 * tz;
        const gx0 = bx, gy0 = by, gz0 = bz;
        const gx1 = pmy * bz - pmz * by, gy1 = pmz * bx - pmx * bz, gz1 = pmx * by - pmy * bx;
        // The crank's own acceleration moves the section along its normal too: g7 = (dr/dphi . b) dphi/dtheta.
        const gc = ((W.Ap[j3 + 1] + d12 * W.tp[j3 + 1]) * by + (W.Ap[j3 + 2] + d12 * W.tp[j3 + 2]) * bz) * (dphW ? dphW[w] : dphNow);
        let vnDotNow;
        if (bodyAcc) {
          // Apparent mass with a moving body. The body's own accelerations (a, wdot) and the crank's go on the left
          // of the equations of motion through out.Ma: explicit, they are unstable (comparable with the roll
          // inertia). Only the part with no body state in it is differenced in time, q1 = (wind - pdot).b; the
          // terms that mix body velocity with the moving section are written out:
          //   dVn/dt = d(q1)/dt + (w x vb).b - (w x pdot).b - (vb + w x p).bdot  - (a + wdot x p).b - gc thdd
          const pdx = adx + d12 * tdx, pdy = ady + d12 * tdy, pdz = adz + d12 * tdz;
          // q1 = q1a - gc crank: the crank-rate part is split off before differencing, so that the difference never
          // carries the crank's acceleration (gc thdd, implicit through Ma). d(q1)/dt + gc thdd = d(q1a)/dt - crank
          // d(gc)/dt, both differenced from states alone. (Differencing q1 and cancelling the last step's gc thdd
          // left a lagged explicit apparent mass on the crank, unstable step to step when the stroke slowed hard
          // against a light wing: round 3, M4.)
          const q1a = (wind[0] - pdx) * bx + (wind[1] - pdy) * by + (wind[2] - pdz) * bz + gc * crankNow;
          if (dt > 0) {
            if (histCount >= 2) {
              vnDot[q] = (3 * q1a - 4 * vn1[q] + vn2[q]) / (2 * dt);
              gcDot[q] = (3 * gc - 4 * gc1[q] + gc2[q]) / (2 * dt);
            } else if (histCount === 1) {
              vnDot[q] = (q1a - vn1[q]) / dt;
              gcDot[q] = (gc - gc1[q]) / dt;
            }
            vn2[q] = vn1[q];
            vn1[q] = q1a;
            gc2[q] = gc1[q];
            gc1[q] = gc;
          }
          const wvx = om[1] * vb[2] - om[2] * vb[1], wvy = om[2] * vb[0] - om[0] * vb[2], wvz = om[0] * vb[1] - om[1] * vb[0];
          const wpx = om[1] * pdz - om[2] * pdy, wpy = om[2] * pdx - om[0] * pdz, wpz = om[0] * pdy - om[1] * pdx;
          const ux = vb[0] + om[1] * pmz - om[2] * pmy, uy = vb[1] + om[2] * pmx - om[0] * pmz, uz = vb[2] + om[0] * pmy - om[1] * pmx;
          const bdx = W.bd[j3], bdy = W.bd[j3 + 1], bdz = W.bd[j3 + 2];
          vnDotNow = vnDot[q] - crankNow * gcDot[q] + (wvx - wpx) * bx + (wvy - wpy) * by + (wvz - wpz) * bz - (ux * bdx + uy * bdy + uz * bdz);
        } else {
          if (dt > 0) {
            if (histCount >= 2) vnDot[q] = (3 * vnMid - 4 * vn1[q] + vn2[q]) / (2 * dt);
            else if (histCount === 1) vnDot[q] = (vnMid - vn1[q]) / dt;
            vn2[q] = vn1[q];
            vn1[q] = vnMid;
          }
          vnDotNow = vnDot[q];
        }
        const fAm = mAm * vnDotNow;
        if (bodyAcc) {
          const mm = mAm * ds;
          G7[0] = gx0; G7[1] = gy0; G7[2] = gz0; G7[3] = gx1; G7[4] = gy1; G7[5] = gz1; G7[6] = gc;
          for (let a7 = 0; a7 < 7; a7++) for (let b7 = 0; b7 < 7; b7++) Ma[a7 * 7 + b7] += mm * G7[a7] * G7[b7];
        }
        // Forces on the strip in body axes
        const fq = (fN) * ds; // through the quarter chord
        const ft = fT * ds;
        const fa = fAm * ds;
        const fx = fq * bx + ft * tx;
        const fy = fq * by + ft * ty;
        const fz = fq * bz + ft * tz;
        const fxa = fa * bx, fya = fa * by, fza = fa * bz;
        const p14x = ax + d14 * tx, p14y = ay + d14 * ty, p14z = az + d14 * tz;
        const p12x = ax + d12 * tx, p12y = ay + d12 * ty, p12z = az + d12 * tz;
        Fx += fx + fxa; Fy += fy + fya; Fz += fz + fza;
        const mS = qdyn * c * c * cm * ds; // nose-up section moment
        // nose-up unit vector m = t x b
        const mx = ty * bz - tz * by, my = tz * bx - tx * bz, mz = tx * by - ty * bx;
        Mx += p14y * fz - p14z * fy + p12y * fza - p12z * fya + mS * mx;
        My += p14z * fx - p14x * fz + p12z * fxa - p12x * fza + mS * my;
        Mz += p14x * fy - p14y * fx + p12x * fya - p12y * fxa + mS * mz;
        // Generalised forces: on phi from partial velocities, on theta_cmd via nose-up moment about the spar
        const apy = W.Ap[j3 + 1], apz = W.Ap[j3 + 2];
        const tpy = W.tp[j3 + 1], tpz = W.tp[j3 + 2];
        const Qp = fy * (apy + d14 * tpy) + fz * (apz + d14 * tpz) + fya * (apy + d12 * tpy) + fza * (apz + d12 * tpz)
          + fx * 0 + fxa * 0;
        // Moment about the spar axis, nose-up sense: force normal times its forward offset, plus section moment.
        const mSpar = fq * d14 + fa * d12 + mS;
        const wgt = st.weight[i];
        if (w === 0) {
          Qpr += Qp;
          Qtr += wgt * mSpar;
        } else {
          Qpl += Qp;
          Qtl += wgt * mSpar;
        }
        // Moment of aero about the shoulder of that wing, for the structure check
        const dHx = p14x - hx, dHy = p14y - hy, dHz = p14z - side * hz;
        const gx = fx + fxa, gy = fy + fya, gz = fz + fza;
        const hmx = dHy * gz - dHz * gy, hmy = dHz * gx - dHx * gz, hmz = dHx * gy - dHy * gx;
        if (w === 0) { rm0x += hmx; rm0y += hmy; rm0z += hmz; } else { rm1x += hmx; rm1y += hmy; rm1z += hmz; }
        // Output
        out.fx[q] = gx;
        out.fy[q] = gy;
        out.fz[q] = gz;
        out.alpha[q] = alphaGeo;
        out.alphaEff[q] = alphaLag;
        out.cl[q] = cl;
        out.speed[q] = V;
        out.attached[q] = sigma;
        out.fAm[q] = fAm;
        out.fCirc[q] = fN;
        const ar = c * ds;
        area += ar;
        stalled += (1 - sigma) * ar;
        alphaSum += alphaLag * ar;
        gsum += gTarget * ds;
        // Power the wing motion puts into the air: -F . pdot at the force's point
        const pdx14 = adx + d14 * tdx, pdy14 = ady + d14 * tdy, pdz14 = adz + d14 * tdz;
        const pdx12 = adx + d12 * tdx, pdy12 = ady + d12 * tdy, pdz12 = adz + d12 * tdz;
        power -= fx * pdx14 + fy * pdy14 + fz * pdz14 + fxa * pdx12 + fya * pdy12 + fza * pdz12;
        power -= mS * (W.dtheta[i]);
      }
    }
    if (dt > 0) {
      // One frozen-Jacobian Newton step per time step, then recompute the downwash.
      luSolve(P, N, piv, res);
      for (let i = 0; i < N; i++) gam[i] += res[i];
      for (let i = 0; i < N; i++) {
        let s = 0;
        let sf = 0;
        const r = i * N;
        for (let j = 0; j < N; j++) {
          s += D[r + j] * gam[j];
          sf += Df[r + j] * gam[j];
        }
        wi[i] = s;
        wf[i] = sf;
      }
      histCount++;
      primeNext = false;
    }
    out.F[0] = Fx; out.F[1] = Fy; out.F[2] = Fz;
    out.M[0] = Mx; out.M[1] = My; out.M[2] = Mz;
    out.Qphi[0] = Qpr; out.Qphi[1] = Qpl;
    out.Qtheta[0] = Qtr; out.Qtheta[1] = Qtl;
    out.stalledFraction = stalled / area;
    out.power = power;
    out.gammaSum = gsum;
    out.meanAlpha = alphaSum / area;
    out.rootMoment[0] = rm0x; out.rootMoment[1] = rm0y; out.rootMoment[2] = rm0z;
    out.rootMoment[3] = rm1x; out.rootMoment[4] = rm1y; out.rootMoment[5] = rm1z;
    return out;
  }

  return {
    evaluate,
    reset,
    prime,
    out,
    stations: st,
    right: R,
    left: L,
    n,
    area: S,
    wing,
    get calibration() { return cal; },
    set calibration(c) { cal = clampCalibration(c); factorP(); },
    set aeroScale(v) { aeroScale = v; },
    get aeroScale() { return aeroScale; },
  };
}

// Shared temporaries for the inner velocity helper (kept module-level so the closure allocates nothing
// that escapes; a single thread runs one evaluate at a time).
let vRelX = 0, vRelY = 0, vRelZ = 0;

/**
 * Cycle-mean forces at fixed attitude and speed. Body moves with velocity vb, wind zero.
 * sched is passed to jointState (twist schedule). Runs `cycles` flap cycles and averages the last.
 */
export function cycleAverage(model, { vb, om = [0, 0, 0], wind = [0, 0, 0], jointFn, freq, rho, mu, cycles = 3, stepsPerCycle = 120 }) {
  const out = { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, power: 0, stalled: 0, QphiPeak: 0, peakFy: -Infinity, minFy: Infinity };
  model.reset();
  const dt = 1 / (freq * stepsPerCycle);
  const total = cycles * stepsPerCycle;
  const startAvg = total - stepsPerCycle;
  for (let k = 0; k < total; k++) {
    const t = k * dt;
    const { R, L } = jointFn(t);
    const r = model.evaluate(vb, om, wind, R, L, rho, mu, dt);
    if (k >= startAvg) {
      out.Fx += r.F[0];
      out.Fy += r.F[1];
      out.Fz += r.F[2];
      out.Mx += r.M[0];
      out.My += r.M[1];
      out.Mz += r.M[2];
      out.power += r.power;
      out.stalled += r.stalledFraction;
      if (r.F[1] > out.peakFy) out.peakFy = r.F[1];
      if (r.F[1] < out.minFy) out.minFy = r.F[1];
    }
  }
  for (const k of ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'power', 'stalled']) out[k] /= stepsPerCycle;
  return out;
}
