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

/**
 * Flat elliptic wing (right half; the solver mirrors it): chord c(z) = c0 sqrt(1 - (z/s)^2) with a straight
 * quarter-chord line, the planform Prandtl's closed form and Helmbold's formula describe. Spanwise edges at
 * z = s sin(pi j / (2 ns)), clustered towards the tip. Grid layout as wingGrid: point (i, j) at 3 (i (ns+1) + j).
 * AR = 8 s / (pi c0).
 */
export function ellipticPlateSurface({ c0 = 1, AR, nc, ns, chordCosine = false }) {
  const s = (Math.PI * AR * c0) / 8;
  const g = new Float64Array((nc + 1) * (ns + 1) * 3);
  for (let j = 0; j <= ns; j++) {
    const z = s * Math.sin((Math.PI * j) / (2 * ns));
    const c = c0 * Math.sqrt(Math.max(0, 1 - (z / s) ** 2));
    for (let i = 0; i <= nc; i++) {
      const xi = chordCosine ? 0.5 * (1 - Math.cos((Math.PI * i) / nc)) : i / nc;
      const p = 3 * (i * (ns + 1) + j);
      g[p] = 0.25 * c - xi * c;
      g[p + 1] = 0;
      g[p + 2] = z;
    }
  }
  return {
    semi: s,
    area: (Math.PI * s * c0) / 2, // whole wing
    surface: { nc, ns, grid(t, out) { out.set(g); } },
  };
}
