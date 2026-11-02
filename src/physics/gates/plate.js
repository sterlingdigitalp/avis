// A rigid flat plate in plunge, built through kinematics.js so the gates exercise the same geometry path
// as the birds. Lift is reported per unit span at mid-span, which is the 2D section theory refers to.
import { makeWing, makeLatticeSpec, wingGrid } from '../kinematics.js';
import { createUvlm, fitHarmonic } from '../uvlm.js';

const ZERO_JOINT = { phi: 0, dphi: 0, ddphi: 0, theta: 0, dtheta: 0, ddtheta: 0 };

export function plateSurface({ chord, AR, nc, ns, cosine = true, alpha = 0, plunge = null }) {
  const semi = (AR * chord) / 2;
  const wing = makeWing({
    hinge: [0, 0, 0],
    segments: [{ length: semi, chord_in: chord, chord_out: chord, xle_in: 0.25 * chord, xle_out: 0.25 * chord, gain: 0, twist_in: 0, twist_out: 0 }],
    spar_x: 0,
    incidence: alpha,
  });
  const ls = makeLatticeSpec(wing, nc, ns, { cosine });
  const base = wingGrid(wing, ls, ZERO_JOINT, 1);
  return {
    wing,
    ls,
    surface: {
      nc,
      ns,
      grid(t, out) {
        const y = plunge ? plunge(t) : 0;
        for (let i = 0; i < out.length; i += 3) {
          out[i] = base[i];
          out[i + 1] = base[i + 1] + y;
          out[i + 2] = base[i + 2];
        }
      },
    },
  };
}

/** Run a harmonically plunging plate. y = h0 sin(omega t). Returns section and total forces. */
export function runPlunge({ U, chord, k, h0, AR, nc, ns, rho, periods = 3, measure = 1, wakePeriods = 2, stepsPerChord = null, wakeCoreFrac = 0.0 }) {
  const b = chord / 2;
  const omega = (k * U) / b;
  const T = (2 * Math.PI) / omega;
  const per = stepsPerChord ?? nc;
  const dtNominal = chord / (per * U);
  const stepsPerPeriod = Math.round(T / dtNominal);
  const dt = T / stepsPerPeriod;
  const { ls, surface } = plateSurface({ chord, AR, nc, ns, plunge: (t) => h0 * Math.sin(omega * t) });
  const maxWakeRows = Math.ceil(wakePeriods * stepsPerPeriod);
  const sim = createUvlm([surface], {
    Vinf: [-U, 0, 0], rho, dt, symmetric: true, maxWakeRows,
    wakeCore: wakeCoreFrac * chord,
  });
  const nSteps = Math.round(periods * stepsPerPeriod);
  const startForces = nSteps - Math.round(measure * stepsPerPeriod);
  const ts = [];
  const LMid = [];
  const TMid = [];
  const LTot = [];
  const TTot = [];
  const width0 = ls.span[1] - ls.span[0];
  const semi = ls.span[ns];
  for (let n = 0; n < nSteps; n++) {
    const f = sim.advance(n >= startForces);
    if (n >= startForces) {
      const st = sim.surfaces[0].strip;
      ts.push(f.t);
      LMid.push(st.fy[0] / width0);
      TMid.push(-st.fx[0] / width0); // thrust is force towards the nose (+x); see below
      LTot.push(f.force[1] / semi);
      TTot.push(f.force[0] / semi);
    }
  }
  // Force in +x is thrust for a bird flying towards +x.
  for (let i = 0; i < TMid.length; i++) TMid[i] = -TMid[i];
  const L = fitHarmonic(ts, LMid, omega);
  const LT = fitHarmonic(ts, LTot, omega);
  const mean = (a) => a.reduce((p, q) => p + q, 0) / a.length;
  return { omega, T, dt, steps: nSteps, stepsPerPeriod, L, LT, thrustMid: mean(TMid), thrustTotal: mean(TTot) };
}
