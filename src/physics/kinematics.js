// The one source of wing motion. Tunnels, flight model, page and film all import these functions.
//
// Frames (SPEC): body x forward, y up, z right; right-handed. Stroke angle phi > 0 raises the tip.
// Twist theta > 0 is nose up on either wing. Phase 0 is the top of the stroke: the downstroke runs
// from phase 0 towards phase pi. Everything here is a pure function of its inputs.
//
// Wing model: a chain of rigid spanwise segments rotating about axes parallel to body x. So the
// spar polyline stays in a plane normal to body x, which makes the positions, velocities and
// accelerations below closed form. Each station then has a commanded twist about that spar axis:
//   theta(s) = incidence + washout*(s/L) + weight(s) * theta_cmd
// The skin between ribs is a ruled surface, so the wing shape is known at every instant.

const TWO_PI = 2 * Math.PI;

export function wrapAngle(a) {
  const r = a % TWO_PI;
  return r < 0 ? r + TWO_PI : r;
}

// ---------------------------------------------------------------------------------------------
// Crank-rocker four-bar, solved exactly.
//
//   O2 = (0,0) crank pivot, O4 = (d,0) rocker pivot, A = a(cos t, sin t), B = O4 + c(cos psi, sin psi)
//   closure: a e^{i t} + b e^{i beta} = d + c e^{i psi}
// ---------------------------------------------------------------------------------------------

/** True if the shortest link is the crank and Grashof's condition makes it rotate fully. */
export function isCrankRocker({ a, b, c, d }) {
  const links = [a, b, c, d];
  const s = Math.min(...links);
  const l = Math.max(...links);
  const sum = a + b + c + d;
  return s === a && s + l < sum - s - l;
}

/**
 * Exact position and first and second derivatives with respect to crank angle.
 * psi' = dpsi/dtheta is the velocity ratio; rocker torque / crank torque = 1 / psi' for an ideal link.
 * Assumes the open assembly with B above the ground line (rocker pivot at the right).
 */
export function crankRockerSolve(link, theta) {
  const { a, b, c, d } = link;
  const ax = a * Math.cos(theta);
  const ay = a * Math.sin(theta);
  const dx = ax - d;
  const s2 = dx * dx + ay * ay;
  const s = Math.sqrt(s2);
  const cosG = (c * c + s2 - b * b) / (2 * c * s);
  if (cosG > 1 || cosG < -1) return null; // cannot assemble at this angle
  // Angle of O4->A, then rotate to the rocker (branch keeps B above the ground line for a < d).
  // O4->A points to -x for a < d, so measure in 0..2pi to stay continuous as A crosses the ground line.
  let base = Math.atan2(ay, dx);
  if (base < 0) base += TWO_PI;
  const psi = base - Math.acos(cosG);
  const bx = d + c * Math.cos(psi);
  const by = c * Math.sin(psi);
  const beta = Math.atan2(by - ay, bx - ax);
  const sTB = Math.sin(theta - beta);
  const sPB = Math.sin(psi - beta);
  const dpsi = (a * sTB) / (c * sPB);
  const dbeta = (a * Math.sin(theta - psi)) / (b * sPB);
  const ddpsi = (a * Math.cos(theta - beta) + b * dbeta * dbeta - c * dpsi * dpsi * Math.cos(psi - beta)) / (c * sPB);
  // Transmission angle: the acute angle between coupler and rocker.
  let mu = Math.abs(psi - beta) % Math.PI;
  if (mu > Math.PI / 2) mu = Math.PI - mu;
  return { psi, dpsi, ddpsi, beta, dbeta, mu, sinTrans: Math.abs(sPB) };
}

/** Closed form extremes of the rocker: they occur where crank and coupler are collinear. */
export function crankRockerLimits(link) {
  const { a, b, c, d } = link;
  const angleAtO4 = (r) => Math.acos((c * c + d * d - r * r) / (2 * c * d));
  // psi measured from +x at O4; the interior angle at O4 (towards O2 at -x) gives psi = pi - angle.
  const psiFar = Math.PI - angleAtO4(b + a); // crank extended along the coupler: B furthest from O2
  const psiNear = Math.PI - angleAtO4(b - a); // crank folded back on the coupler
  // Crank angles at those instants: direction of O2->B (extended) or B->O2 (folded).
  const bFar = [d + c * Math.cos(psiFar), c * Math.sin(psiFar)];
  const bNear = [d + c * Math.cos(psiNear), c * Math.sin(psiNear)];
  const thetaFar = Math.atan2(bFar[1], bFar[0]);
  const thetaNear = Math.atan2(bNear[1], bNear[0]) + Math.PI;
  return {
    psiMin: Math.min(psiFar, psiNear),
    psiMax: Math.max(psiFar, psiNear),
    thetaAtPsiMax: psiFar >= psiNear ? thetaFar : thetaNear,
    thetaAtPsiMin: psiFar >= psiNear ? thetaNear : thetaFar,
    swing: Math.abs(psiFar - psiNear),
  };
}

/** Rocker radius c that gives a required half-swing for fixed a, b, d, by bisection on the closed form. */
export function rockerForAmplitude({ a, b, d }, amplitude) {
  // The linkage assembles at both collinear positions only when both triangles close.
  const cLo = Math.max(a * 1.001, Math.abs(d - (b + a)) * 1.001, Math.abs(d - (b - a)) * 1.001);
  const cHi = Math.min(d + b - a, b + a + d) * 0.999;
  if (cLo >= cHi) return null;
  const swing = (c) => crankRockerLimits({ a, b, c, d }).swing / 2;
  // Swing falls with c from its peak to a minimum, then rises again as the linkage nears its limit.
  let lo = cLo;
  let hi = cHi;
  let big = -1;
  let small = Infinity;
  const N = 256;
  for (let i = 0; i <= N; i++) {
    const c = cLo + ((cHi - cLo) * i) / N;
    const v = swing(c);
    if (v > big) {
      big = v;
      lo = c;
    }
  }
  for (let i = 0; i <= N; i++) {
    const c = lo + ((cHi - lo) * i) / N;
    const v = swing(c);
    if (v < small) {
      small = v;
      hi = c;
    }
  }
  if (!(big >= amplitude && small <= amplitude)) return null;
  for (let i = 0; i < 60; i++) {
    const m = 0.5 * (lo + hi);
    if (swing(m) > amplitude) lo = m;
    else hi = m;
  }
  return 0.5 * (lo + hi);
}

/**
 * A flap drive: one crank driving each wing's rocker through a coupler. Stroke amplitude is set by
 * a servo that slides the coupler pin along a slotted rocker, which changes c. Amplitude therefore
 * is not free: it is limited to what the slot allows, and is solved from the exact linkage.
 * The slot is curved so the mid-stroke angle stays at phiMean as c changes, hence psiMid per entry.
 */
export function makeFlapDrive({ a, b, d, ampMin, ampMax, ampNominal, phiMean = 0, gearRatio = 1, nTable = 17 }) {
  const table = [];
  // Round 5: a fixed stroke (ampMin = ampMax, a pinned rocker with no slot) is a table of one rocker, held twice so the
  // lookup's interpolation needs no special case.
  const fixed = !(ampMax - ampMin > 1e-9);
  if (fixed) nTable = 2;
  for (let i = 0; i < nTable; i++) {
    const amp = fixed ? ampMax : ampMin + ((ampMax - ampMin) * i) / (nTable - 1);
    const c = rockerForAmplitude({ a, b, d }, amp);
    if (c === null) throw new Error(`no rocker gives amplitude ${amp}`);
    const lim = crankRockerLimits({ a, b, c, d });
    table.push({ amp, c, psiMid: 0.5 * (lim.psiMin + lim.psiMax), thetaTop: lim.thetaAtPsiMax });
  }
  const cNom = rockerForAmplitude({ a, b, d }, ampNominal);
  const limNom = crankRockerLimits({ a, b, c: cNom, d });
  return { kind: 'crank-rocker', a, b, d, ampMin, ampMax, ampNominal, phiMean, gearRatio, table, cNom, psiMidNom: 0.5 * (limNom.psiMin + limNom.psiMax) };
}

function driveLookup(drive, amp) {
  const t = drive.table;
  const span = drive.ampMax - drive.ampMin;
  const x = span > 1e-9 ? ((amp - drive.ampMin) / span) * (t.length - 1) : 0;
  const i = Math.max(0, Math.min(t.length - 2, Math.floor(x)));
  const f = Math.max(0, Math.min(1, x - i));
  const p = t[i];
  const q = t[i + 1];
  return { c: p.c + f * (q.c - p.c), psiMid: p.psiMid + f * (q.psiMid - p.psiMid), thetaTop: p.thetaTop + f * (q.thetaTop - p.thetaTop) };
}

/**
 * Round 4: the LEFT wing's stroke when one crank pin drives both couplers (the drive as built: crank pivot on the
 * centre line, ground link d to each shoulder hinge at hingeZ off centre, the left four-bar the mirror image of the
 * right). In the right four-bar's frame the mirrored pin sits at crank angle 2 beta - theta, beta = asin(hingeZ / d)
 * the ground link's angle from the body's vertical, so the left linkage runs its crank backwards from a shifted
 * start: its stroke is the right one's time-reversed and phase-shifted, and differs from it by the crank-rocker's
 * quick-return. Same fields as strokeAngle; derivatives with respect to the (right) phase.
 */
export function strokeAngleLeft(phase, amp, law, hingeZ) {
  if (!law || law.kind === 'sine') return strokeAngle(phase, amp, law);
  const { c, psiMid, thetaTop } = driveLookup(law, amp);
  const beta = Math.asin(hingeZ / law.d);
  const r = crankRockerSolve({ a: law.a, b: law.b, c, d: law.d }, 2 * beta - (thetaTop + phase));
  if (!r) throw new Error('linkage cannot assemble');
  let dpsi = r.psi - psiMid;
  while (dpsi > Math.PI) dpsi -= TWO_PI;
  while (dpsi < -Math.PI) dpsi += TWO_PI;
  return { phi: law.phiMean + dpsi, dphi: -r.dpsi, ddphi: r.ddpsi, mu: r.mu, sinTrans: r.sinTrans, c };
}

/**
 * Stroke law: phi and its derivatives with respect to phase at a given amplitude.
 * law.kind 'sine':  phi = mean + amp cos(phase).
 * law.kind 'crank-rocker': exact linkage; phase is the crank angle measured from the top of stroke.
 */
export function strokeAngle(phase, amp, law) {
  if (!law || law.kind === 'sine') {
    const m = law?.phiMean ?? 0;
    const co = Math.cos(phase);
    const si = Math.sin(phase);
    return { phi: m + amp * co, dphi: -amp * si, ddphi: -amp * co, mu: Math.PI / 2, sinTrans: 1 };
  }
  const { c, psiMid, thetaTop } = driveLookup(law, amp);
  // Crank runs in the direction that takes the rocker downward from the top: theta = top + phase.
  const r = crankRockerSolve({ a: law.a, b: law.b, c, d: law.d }, thetaTop + phase);
  if (!r) throw new Error('linkage cannot assemble');
  return { phi: law.phiMean + (r.psi - psiMid), dphi: r.dpsi, ddphi: r.ddpsi, mu: r.mu, sinTrans: r.sinTrans, c };
}

/** Bounds of a linkage over a cycle: worst transmission angle, peak velocity ratio, quick return. */
export function linkageCycleStats(drive, amp, n = 720) {
  let muMin = Infinity;
  let maxRatio = 0;
  let minPhi = Infinity;
  let maxPhi = -Infinity;
  let phaseAtMin = 0;
  for (let i = 0; i < n; i++) {
    const ph = (i / n) * TWO_PI;
    const s = strokeAngle(ph, amp, drive);
    muMin = Math.min(muMin, s.mu);
    maxRatio = Math.max(maxRatio, Math.abs(s.dphi));
    if (s.phi > maxPhi) maxPhi = s.phi;
    if (s.phi < minPhi) {
      minPhi = s.phi;
      phaseAtMin = ph;
    }
  }
  return { muMin, maxRatio, halfSwing: (maxPhi - minPhi) / 2, downstrokeFraction: phaseAtMin / TWO_PI, phiMin: minPhi, phiMax: maxPhi };
}

// ---------------------------------------------------------------------------------------------
// Wing planform
// ---------------------------------------------------------------------------------------------

/**
 * spec: {
 *   hinge: [x, y, z]                  right shoulder, body frame (m). z > 0.
 *   segments: [{ length, chord_in, chord_out, xle_in, xle_out, gain, twist_in, twist_out }]
 *        length m along the spar; chords in m; xle = x of leading edge relative to the hinge (m, + fwd);
 *        gain: outer segment angle = phi + gain*(phi - phi_ref); twist_* = weight of theta_cmd at each end.
 *   spar_x      spar axis x relative to the hinge (m)
 *   incidence   rad, fixed
 *   washout     rad at tip, added linearly with span (negative is nose-down at the tip)
 *   camber      max camber / chord
 *   camber_pos  chordwise position of max camber (0-1)
 *   phi_ref     stroke angle where outer segments are straight with the inner one (rad)
 * }
 */
export function makeWing(spec) {
  let S = 0;
  const segs = spec.segments.map((g) => {
    const o = { ...g, s0: S, s1: S + g.length, gain: g.gain ?? 0, twist_in: g.twist_in ?? 1, twist_out: g.twist_out ?? 1 };
    S += g.length;
    return o;
  });
  return {
    hinge: spec.hinge,
    segments: segs,
    L: S,
    spar_x: spec.spar_x ?? 0,
    incidence: spec.incidence ?? 0,
    washout: spec.washout ?? 0,
    camber: spec.camber ?? 0,
    camber_pos: spec.camber_pos ?? 0.5,
    phi_ref: spec.phi_ref ?? 0,
  };
}

function segAt(wing, s) {
  const g = wing.segments;
  for (let k = 0; k < g.length - 1; k++) if (s < g[k].s1) return k;
  return g.length - 1;
}

export function chordAt(wing, s) {
  const g = wing.segments[segAt(wing, s)];
  const f = (s - g.s0) / g.length;
  return g.chord_in + f * (g.chord_out - g.chord_in);
}
export function xleAt(wing, s) {
  const g = wing.segments[segAt(wing, s)];
  const f = (s - g.s0) / g.length;
  return g.xle_in + f * (g.xle_out - g.xle_in);
}
export function twistWeightAt(wing, s) {
  const g = wing.segments[segAt(wing, s)];
  const f = (s - g.s0) / g.length;
  return g.twist_in + f * (g.twist_out - g.twist_in);
}
export function baseTwistAt(wing, s) {
  return wing.incidence + (wing.washout * s) / wing.L;
}

/** Planform area of one wing by exact integration of the piecewise linear chord (flat, unflapped). */
export function wingArea(wing) {
  return wing.segments.reduce((A, g) => A + 0.5 * (g.chord_in + g.chord_out) * g.length, 0);
}
/** Tip-to-tip span with the wing laid flat, including the hinge offset. */
export function wingSpan(wing) {
  return 2 * (wing.hinge[2] + wing.L);
}
/** Mean geometric chord. */
export function meanChord(wing) {
  return wingArea(wing) / wing.L;
}
/** Mean aerodynamic chord of one wing. */
export function mac(wing) {
  let I = 0;
  for (const g of wing.segments) {
    const p = g.chord_in;
    const q = g.chord_out;
    I += (g.length * (p * p + p * q + q * q)) / 3;
  }
  return I / wingArea(wing);
}

/** Non-dimensional camber line height (per chord) and slope at xi from the LE. NACA 4-digit form. */
export function camberLine(xi, f, p) {
  if (f === 0) return { z: 0, dz: 0 };
  if (xi < p) return { z: (f * (2 * p * xi - xi * xi)) / (p * p), dz: (2 * f * (p - xi)) / (p * p) };
  const q = 1 - p;
  return { z: (f * (1 - 2 * p + 2 * p * xi - xi * xi)) / (q * q), dz: (2 * f * (p - xi)) / (q * q) };
}

/** Segment angles from the shoulder stroke angle: gives angles and rates, per segment. */
export function segmentAngles(wing, phi, dphi = 0, ddphi = 0) {
  return wing.segments.map((g) => {
    const m = 1 + g.gain;
    return { phi: phi + g.gain * (phi - wing.phi_ref), dphi: m * dphi, ddphi: m * ddphi };
  });
}

// ---------------------------------------------------------------------------------------------
// Joint state and twist schedule
// ---------------------------------------------------------------------------------------------

/**
 * Commanded twist schedule of a wing's servo, as a function of phase.
 * Downstroke nose-down (phase in 0..pi), upstroke nose-up, by a tip amplitude. `lead` advances it.
 *   theta_cmd = sym + side*diff - dyn * sin(phase + lead)
 * side = +1 for the right wing, -1 for the left: diff > 0 makes the right wing more nose up,
 * so it lifts more and rolls the bird to the left (negative roll).
 */
export function twistCommand(phase, sched, side = 1) {
  const sym = sched.sym ?? 0;
  const diff = sched.diff ?? 0;
  const dyn = sched.dyn ?? 0;
  const lead = sched.lead ?? 0;
  return sym + side * diff - dyn * Math.sin(phase + lead);
}
export function twistCommandRate(phase, sched) {
  const dyn = sched.dyn ?? 0;
  const lead = sched.lead ?? 0;
  return -dyn * Math.cos(phase + lead);
}
export function twistCommandAccel(phase, sched) {
  const dyn = sched.dyn ?? 0;
  const lead = sched.lead ?? 0;
  return dyn * Math.sin(phase + lead);
}

/**
 * Joint state from phase and its rates. phaseRate is the angular speed of the crank (rad/s).
 * Returns the stroke angle in time derivatives, plus the twist rate from the schedule (per wing).
 */
export function jointState(phase, phaseRate, phaseAccel, amp, law, sched, side = 1) {
  const s = strokeAngle(phase, amp, law);
  const dphi = s.dphi * phaseRate;
  const ddphi = s.ddphi * phaseRate * phaseRate + s.dphi * phaseAccel;
  const th = twistCommand(phase, sched, side);
  const d1 = twistCommandRate(phase, sched);
  const d2 = twistCommandAccel(phase, sched);
  return {
    phi: s.phi,
    dphi,
    ddphi,
    theta: th,
    dtheta: d1 * phaseRate,
    ddtheta: d2 * phaseRate * phaseRate + d1 * phaseAccel,
    dphi_dphase: s.dphi,
    mu: s.mu,
  };
}

// ---------------------------------------------------------------------------------------------
// Pose of spanwise stations, with rates. Allocation-free once `out` exists.
// ---------------------------------------------------------------------------------------------

/** Fixed spanwise stations for fast models. n equal-length strips per segment count share. */
export function makeStations(wing, n) {
  const L = wing.L;
  // Split n across segments in proportion to length, at least one per segment.
  const counts = wing.segments.map((g) => Math.max(1, Math.round((n * g.length) / L)));
  let total = counts.reduce((p, q) => p + q, 0);
  while (total > n) {
    const k = counts.indexOf(Math.max(...counts));
    counts[k]--;
    total--;
  }
  while (total < n) {
    const k = counts.indexOf(Math.min(...counts));
    counts[k]++;
    total++;
  }
  const s = new Float64Array(n);
  const ds = new Float64Array(n);
  const chord = new Float64Array(n);
  const xle = new Float64Array(n);
  const weight = new Float64Array(n);
  const base = new Float64Array(n);
  const seg = new Int32Array(n);
  let i = 0;
  wing.segments.forEach((g, k) => {
    for (let m = 0; m < counts[k]; m++, i++) {
      const w = g.length / counts[k];
      const sm = g.s0 + (m + 0.5) * w;
      s[i] = sm;
      ds[i] = w;
      chord[i] = chordAt(wing, sm);
      xle[i] = xleAt(wing, sm);
      weight[i] = twistWeightAt(wing, sm);
      base[i] = baseTwistAt(wing, sm);
      seg[i] = k;
    }
  });
  return { n, s, ds, chord, xle, weight, base, seg };
}

export function makeSectionState(n) {
  const a = () => new Float64Array(n * 3);
  return { A: a(), Ad: a(), Add: a(), Ap: a(), t: a(), td: a(), tdd: a(), tp: a(), b: a(), bd: a(), e: a(), theta: new Float64Array(n), dtheta: new Float64Array(n), ddtheta: new Float64Array(n) };
}

/**
 * Compute spar-axis points and frames at each station for one wing, with first and second time
 * derivatives. side = +1 right, -1 left (mirror in z). joint = { phi, dphi, ddphi, theta, dtheta, ddtheta }.
 *  A(s)     point on the spar axis
 *  t        unit chord vector towards the leading edge (nose-up twist tips it up)
 *  b        section up-normal
 *  e        unit spanwise vector, root to tip
 */
export function sectionStates(wing, st, joint, side, out) {
  const g = wing.segments;
  const nseg = g.length;
  const hx = wing.hinge[0] + wing.spar_x;
  const hy = wing.hinge[1];
  const hz = wing.hinge[2];
  const ref = wing.phi_ref;
  const { phi, dphi, ddphi, theta: thC, dtheta: dthC, ddtheta: ddthC } = joint;
  // Elbow points accumulate along the chain.
  let k = -1;
  let ey = 0, ez = 0, edy = 0, edz = 0, eddy = 0, eddz = 0; // start of current segment relative to hinge
  let epy = 0, epz = 0; // partial of that start point with respect to the shoulder angle
  let sinP = 0, cosP = 1, p1 = 0, p2 = 0, gm = 1;
  for (let i = 0; i < st.n; i++) {
    while (k < st.seg[i]) {
      if (k >= 0) {
        const L = g[k].length;
        ey += L * sinP;
        ez += L * cosP;
        edy += L * cosP * p1;
        edz += -L * sinP * p1;
        eddy += L * (cosP * p2 - sinP * p1 * p1);
        eddz += L * (-sinP * p2 - cosP * p1 * p1);
        epy += L * cosP * gm;
        epz += -L * sinP * gm;
      }
      k++;
      const gn = g[k].gain;
      const a = phi + gn * (phi - ref);
      gm = 1 + gn;
      p1 = gm * dphi;
      p2 = gm * ddphi;
      sinP = Math.sin(a);
      cosP = Math.cos(a);
    }
    const d = st.s[i] - g[k].s0;
    const j3 = i * 3;
    // Spar axis
    const y = ey + d * sinP;
    const z = ez + d * cosP;
    const yd = edy + d * cosP * p1;
    const zd = edz - d * sinP * p1;
    const ydd = eddy + d * (cosP * p2 - sinP * p1 * p1);
    const zdd = eddz + d * (-sinP * p2 - cosP * p1 * p1);
    out.A[j3] = hx;
    out.A[j3 + 1] = hy + y;
    out.A[j3 + 2] = side * (hz + z);
    out.Ad[j3] = 0;
    out.Ad[j3 + 1] = yd;
    out.Ad[j3 + 2] = side * zd;
    out.Add[j3] = 0;
    out.Add[j3 + 1] = ydd;
    out.Add[j3 + 2] = side * zdd;
    out.Ap[j3] = 0;
    out.Ap[j3 + 1] = epy + d * cosP * gm;
    out.Ap[j3 + 2] = side * (epz - d * sinP * gm);
    // Spanwise unit vector and stroke-plane normal (both in the y-z plane)
    const ey_u = sinP;
    const ez_u = cosP;
    const ny = cosP;
    const nz = -sinP;
    const w = st.weight[i];
    const th = st.base[i] + w * thC;
    const dth = w * dthC;
    const ddth = w * ddthC;
    out.theta[i] = th;
    out.dtheta[i] = dth;
    out.ddtheta[i] = ddth;
    const ct = Math.cos(th);
    const sn = Math.sin(th);
    // t = cos(th) x + sin(th) n ; b = -sin(th) x + cos(th) n
    out.t[j3] = ct;
    out.t[j3 + 1] = sn * ny;
    out.t[j3 + 2] = side * sn * nz;
    out.b[j3] = -sn;
    out.b[j3 + 1] = ct * ny;
    out.b[j3 + 2] = side * ct * nz;
    out.e[j3] = 0;
    out.e[j3 + 1] = ey_u;
    out.e[j3 + 2] = side * ez_u;
    // ndot = -phidot e; nddot = -phiddot e - phidot^2 n
    const ndy = -p1 * ey_u;
    const ndz = -p1 * ez_u;
    const nddy = -p2 * ey_u - p1 * p1 * ny;
    const nddz = -p2 * ez_u - p1 * p1 * nz;
    // tdot = thd b + sn ndot ; bdot = -thd t + ct ndot
    out.td[j3] = dth * -sn;
    out.td[j3 + 1] = dth * ct * ny + sn * ndy;
    out.td[j3 + 2] = side * (dth * ct * nz + sn * ndz);
    out.bd[j3] = -dth * ct;
    out.bd[j3 + 1] = -dth * sn * ny + ct * ndy;
    out.bd[j3 + 2] = side * (-dth * sn * nz + ct * ndz);
    // tddot = thdd b + thd bdot + ct thd ndot + sn nddot
    out.tdd[j3] = ddth * -sn + dth * (-dth * ct);
    out.tdd[j3 + 1] = ddth * ct * ny + dth * (-dth * sn * ny + ct * ndy) + ct * dth * ndy + sn * nddy;
    out.tdd[j3 + 2] = side * (ddth * ct * nz + dth * (-dth * sn * nz + ct * ndz) + ct * dth * ndz + sn * nddz);
    // partial of t with respect to the shoulder angle: sin(theta) * dn/dphi = -sin(theta) * gm * e
    out.tp[j3] = 0;
    out.tp[j3 + 1] = -sn * gm * ey_u;
    out.tp[j3 + 2] = -side * sn * gm * ez_u;
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Full lattice of the mean surface, for UVLM, tunnels, page and film.
// ---------------------------------------------------------------------------------------------

/** Spanwise grid stations for a lattice: cosine spacing towards the tip, an edge at each joint. */
export function spanStations(wing, ns, cosine = true) {
  const counts = wing.segments.map((g) => Math.max(1, Math.round((ns * g.length) / wing.L)));
  let tot = counts.reduce((p, q) => p + q, 0);
  while (tot > ns) {
    counts[counts.indexOf(Math.max(...counts))]--;
    tot--;
  }
  while (tot < ns) {
    counts[counts.indexOf(Math.min(...counts))]++;
    tot++;
  }
  const s = [0];
  wing.segments.forEach((g, k) => {
    const last = k === wing.segments.length - 1;
    for (let m = 1; m <= counts[k]; m++) {
      const f = m / counts[k];
      // Half-cosine: clusters at the tip end of the last segment only.
      const q = cosine && last ? Math.sin((f * Math.PI) / 2) : f;
      s.push(g.s0 + q * g.length);
    }
  });
  return Float64Array.from(s);
}

/** Pre-compute what a lattice needs from the planform. xi: chordwise grid stations in 0..1. */
export function makeLatticeSpec(wing, nc, ns, { cosine = true, chordCosine = false } = {}) {
  const span = spanStations(wing, ns, cosine);
  const xi = new Float64Array(nc + 1);
  for (let i = 0; i <= nc; i++) xi[i] = chordCosine ? 0.5 * (1 - Math.cos((Math.PI * i) / nc)) : i / nc;
  const cam = new Float64Array(nc + 1);
  for (let i = 0; i <= nc; i++) cam[i] = camberLine(xi[i], wing.camber, wing.camber_pos).z;
  const chord = span.map((s) => chordAt(wing, s));
  const xle = span.map((s) => xleAt(wing, s));
  const weight = span.map((s) => twistWeightAt(wing, s));
  const base = span.map((s) => baseTwistAt(wing, s));
  const seg = Int32Array.from(span, (s) => segAt(wing, Math.min(s, wing.L - 1e-12)));
  return { nc, ns, span, xi, cam, chord, xle, weight, base, seg };
}

/**
 * Grid of (nc+1)*(ns+1) points of the wing mean surface in the body frame.
 * Point (i,j) index is 3*(i*(ns+1)+j); i runs LE to TE. j runs in +z, so the right wing goes root to
 * tip and the left wing tip to root, which keeps panel normals up on both.
 */
export function wingGrid(wing, ls, joint, side, out = null) {
  const { nc, ns, span, xi, cam, chord, xle, weight, base, seg } = ls;
  const g = out ?? new Float64Array((nc + 1) * (ns + 1) * 3);
  const hx = wing.hinge[0];
  const hy = wing.hinge[1];
  const hz = wing.hinge[2];
  const phi = joint.phi;
  const ref = wing.phi_ref;
  let k = -1;
  let ey = 0, ez = 0, sinP = 0, cosP = 1;
  for (let m = 0; m <= ns; m++) {
    while (k < seg[m]) {
      if (k >= 0) {
        const L = wing.segments[k].length;
        ey += L * sinP;
        ez += L * cosP;
      }
      k++;
      const a = phi + wing.segments[k].gain * (phi - ref);
      sinP = Math.sin(a);
      cosP = Math.cos(a);
    }
    const d = span[m] - wing.segments[k].s0;
    const y = hy + ey + d * sinP;
    const z = hz + ez + d * cosP;
    const th = base[m] + weight[m] * joint.theta;
    const ct = Math.cos(th);
    const st = Math.sin(th);
    const ny = cosP;
    const nz = -sinP;
    const j = side > 0 ? m : ns - m;
    for (let i = 0; i <= nc; i++) {
      // distance forward of the spar axis along t, and height along b
      const dt = xle[m] - xi[i] * chord[m] - wing.spar_x;
      const db = cam[i] * chord[m];
      const f = dt * ct - db * st; // body x offset relative to the spar axis
      const h = dt * st + db * ct; // offset along the stroke-plane normal
      const p = 3 * (i * (ns + 1) + j);
      g[p] = hx + wing.spar_x + f;
      g[p + 1] = y + h * ny;
      g[p + 2] = side * (z + h * nz);
    }
  }
  return g;
}

/** Convenience for pages: all panels of both wings at a phase, as plain arrays. */
export function wingSurfaces(wing, ls, phase, amp, law, sched) {
  const R = jointState(phase, 0, 0, amp, law, sched, +1);
  const L = jointState(phase, 0, 0, amp, law, sched, -1);
  return { right: wingGrid(wing, ls, R, +1), left: wingGrid(wing, ls, L, -1) };
}
