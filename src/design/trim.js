// Level-flight trim of a bird, cycle-averaged, and the control table the flight controller carries.
import { buildBird, createFlight, quatFromEuler } from '../physics/flight.js';
import { designAtmosphere, G0 } from '../physics/atmosphere.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;

/**
 * SPEC ruling: a trim that does not converge is a failure, never a result. A trim is converged when the norm of
 * its cycle-mean residuals (streamwise force, vertical force, 3 x pitching moment per metre) is below 1% of the
 * weight. The same number is the Newton target, so a trim is never accepted short of where Newton aimed.
 */
export const TRIM_TOL = 0.01;

/**
 * Electrical power of the drive from a crank load torque held at constant speed: what the motor model
 * needs, with no flywheel smoothing, the conservative case. P is at the motor terminals and is negative when the
 * crank drives the motor; batteryPower() applies the no-regeneration ruling.
 */
export function drivePowerFromLoad(design, tauLoad, crank) {
  const m = design.motor;
  const G = design.drive.gearRatio;
  const eta = design.drive.gearEfficiency ?? 0.85;
  const w = crank * G;
  const tauM = tauLoad >= 0 ? tauLoad / (G * eta) : (tauLoad * eta) / G;
  const I = tauM * m.kv_rad_per_v_s + m.I0;
  const V = w / m.kv_rad_per_v_s + I * m.R;
  return { P: V * I, I, V };
}

/**
 * Battery-side power of the flap drive for a motor-terminal power P (W). SPEC ruling: electrical power is the
 * cycle mean of max(0, battery power) unless the ESC's own documentation states regenerative operation, when
 * regeneration counts at that ESC's stated efficiency.
 */
export function batteryPower(P, esc = {}) {
  if (P >= 0) return P / (esc?.eta ?? 0.95);
  if (esc?.regenerative === true && Number.isFinite(esc.regen_eta)) return P * esc.regen_eta;
  return 0;
}

/** Cycles needed for the slowest Wagner lag state (Jones b1 = 0.0455) to settle to e^-4 at speed V. */
export function settleCycles(design, V, freq, minCycles = 3) {
  const c = design.geometry?.chord_root_m ?? design.wing?.segments?.[0]?.chord_in ?? 0.2;
  const tW = c / (2 * Math.max(V, 1) * 0.0455);
  return Math.max(minCycles, Math.ceil(4 * tW * Math.max(freq, 0.5)) + 1);
}

/**
 * A frozen-body cycle at given attitude and crank speed. Returns mean residuals of the equations of
 * motion (body axes, zero in trim), plus power, stall and currents. `bird` is reused between calls.
 * power: battery power with no regeneration credit, drive plus the 5 V servo and avionics bus.
 * power_regen_credit: the same with the drive's negative power credited, for information only.
 */
export function cycleResidual(design, bird, { V, theta, freq, tail, amp, sym, dyn, lead, phi = 0, diff = 0, yaw = 0, cycles = null, gamma = 0, parkPhase = Math.PI / 2, omega = [0, 0, 0], imposeTwist = false, dt = null }) {
  // At least 60 steps per flap cycle, as a multiple of 240 Hz (the flight model's rule).
  dt = dt ?? 1 / (240 * Math.max(1, Math.ceil((60 * Math.max(freq, 0)) / 240)));
  const sim = createFlight(design, { bird, dt, frozenBody: true, imposeTwist });
  const qv = quatFromEuler(0, theta, phi);
  const glide = freq <= 0;
  const stepsPerCycle = glide ? Math.round(0.5 / dt) : Math.round(1 / (dt * Math.max(freq, 0.5)));
  const nCycles = cycles ?? (glide ? 2 : settleCycles(design, V, freq));
  const cmd = sim.cmd;
  cmd.freq = glide ? 0 : freq; cmd.amp = amp; cmd.sym = sym; cmd.dyn = glide ? 0 : dyn; cmd.lead = lead; cmd.diff = diff;
  cmd.tailPitch = tail; cmd.tailYaw = yaw;
  if (glide) {
    cmd.throttleOff = true;
    cmd.parkPhase = parkPhase;
  }
  const res = new Float64Array(6);
  let power = 0, powerCredit = 0, pDrive = 0, p5 = 0, stalled = 0, mechP = 0, Irms = 0, n = 0, crankPeak = 0, mechPos = 0;
  const vx = V * Math.cos(gamma), vy = V * Math.sin(gamma);
  sim.setState({ pos: [0, 0, 0], quat: qv, vel: [vx, vy, 0], phase: glide ? parkPhase : 0, crank: glide ? 0 : freq * TWO_PI, amp, twistR: sym + diff, twistL: sym - diff, tailR: -tail + yaw, tailL: -tail - yaw, iInt: 0, epsLag: 0.5 });
  const crank = glide ? 0 : freq * TWO_PI;
  const eta = design.esc?.eta ?? 0.95;
  for (let k = 0; k < nCycles * stepsPerCycle; k++) {
    sim.step();
    const x = sim.x;
    x[0] = x[1] = x[2] = 0;
    x[3] = qv[0]; x[4] = qv[1]; x[5] = qv[2]; x[6] = qv[3];
    x[7] = vx; x[8] = vy; x[9] = 0;
    x[10] = omega[0]; x[11] = omega[1]; x[12] = omega[2];
    x[14] = crank;
    if (glide) x[13] = parkPhase;
    if (k >= (nCycles - 1) * stepsPerCycle) {
      for (let j = 0; j < 6; j++) res[j] += sim.tel.residual[j];
      const dp = glide ? { P: 0, I: 0 } : drivePowerFromLoad(design, sim.tel.crankTorque, crank);
      const pd = batteryPower(dp.P, design.esc);
      const pb = (5.0 * sim.tel.servo_I) / 0.85;
      pDrive += pd;
      p5 += pb;
      power += pd + pb;
      powerCredit += (dp.P >= 0 ? dp.P / eta : dp.P * eta) + pb;
      Irms += dp.I * dp.I;
      mechP += sim.tel.crankTorque * crank;
      mechPos += Math.max(0, sim.tel.crankTorque * crank);
      stalled += sim.tel.stalled;
      crankPeak = Math.max(crankPeak, Math.abs(sim.tel.crankTorque));
      n++;
    }
  }
  for (let j = 0; j < 6; j++) res[j] /= n;
  return { res, power: power / n, power_drive: pDrive / n, power_5v: p5 / n, power_regen_credit: powerCredit / n, stalled: stalled / n, mechP: mechP / n, mechP_positive: mechPos / n, Irms: Math.sqrt(Irms / n), crankPeak, twistPeak: sim.peaks.twistTorque, rootBendPeak: sim.peaks.rootBend, elbowBendPeak: sim.peaks.elbowBend, couplerPeak: sim.peaks.couplerForce, battMean: (power / n) / Math.max(sim.tel.batt_V, 1), cycles: nCycles };
}

/** Newton on theta, freq, tail for fixed schedule. Returns the trim or null if it cannot be found. */
export function trimLevel(design, bird, { V, amp, sym = 0, dyn = 0, lead = 0, start = null, maxFreq = 12, cycles = null, maxIter = 12 }) {
  let x = start ? [start.theta, start.freq, start.tail] : [0.05, 1.5, 0];
  const f = (u) => cycleResidual(design, bird, { V, theta: u[0], freq: Math.max(0.5, u[1]), tail: u[2], amp, sym, dyn, lead, cycles });
  // Thrust is not monotonic in frequency once the outer wing stalls, so Newton can land on a high-frequency
  // root. Bracket the lowest frequency where thrust first balances drag, with the pitch re-solved for lift at
  // each frequency (a secant on theta), then refine all three together from there.
  {
    const liftTheta = (fq, th0) => {
      let a = th0, fa = f([a, fq, x[2]]).res[1];
      let b = a + 0.04, fb = f([b, fq, x[2]]).res[1];
      for (let i = 0; i < 4 && Math.abs(fb) > 0.01 * bird.mTotal * G0 && fb !== fa; i++) {
        const c = Math.max(-0.3, Math.min(0.35, b - (fb * (b - a)) / (fb - fa)));
        a = b; fa = fb; b = c; fb = f([b, fq, x[2]]).res[1];
      }
      return b;
    };
    let prev = null;
    let th = x[0];
    // Continuation: start below the previous trim's frequency rather than from the bottom every time.
    const f0 = start && start.freq ? Math.max(0.8, 0.7 * start.freq) : 0.8;
    for (let fq = f0; fq <= maxFreq; fq += 0.4) {
      th = liftTheta(fq, th);
      const q = f([th, fq, x[2]]);
      if (prev && prev.fx < 0 && q.res[0] >= 0) {
        const t = -prev.fx / (q.res[0] - prev.fx);
        x[1] = fq - 0.4 + 0.4 * t;
        x[0] = prev.th + t * (th - prev.th);
        break;
      }
      prev = { fx: q.res[0], th };
    }
  }
  let r = f(x);
  const norm = (q) => Math.hypot(q.res[0], q.res[1], 3 * q.res[5]);
  for (let it = 0; it < maxIter; it++) {
    const g = [r.res[0], r.res[1], r.res[5]];
    if (norm(r) < TRIM_TOL * bird.mTotal * G0) break;
    const J = [[], [], []];
    const h = [0.01, 0.1, 0.02];
    for (let c = 0; c < 3; c++) {
      const y = x.slice();
      y[c] += h[c];
      const q = f(y);
      J[0][c] = (q.res[0] - g[0]) / h[c];
      J[1][c] = (q.res[1] - g[1]) / h[c];
      J[2][c] = (q.res[5] - g[2]) / h[c];
    }
    const dx = solve3(J, g.map((v) => -v));
    if (!dx) return null;
    // Damped step with limits
    let step = 1;
    const lim = [0.15, 1.5, 0.15];
    for (let c = 0; c < 3; c++) step = Math.min(step, lim[c] / Math.max(Math.abs(dx[c]), 1e-9));
    let best = null;
    for (let a = step; a >= step / 8; a /= 2) {
      const y = [x[0] + a * dx[0], Math.max(0.5, x[1] + a * dx[1]), x[2] + a * dx[2]];
      const q = f(y);
      if (norm(q) < norm(r)) {
        best = { y, q };
        break;
      }
    }
    if (!best) break;
    x = best.y;
    r = best.q;
  }
  const converged = norm(r) < TRIM_TOL * bird.mTotal * G0;
  const noises = { fx: r.res[0], fy: r.res[1], mz: r.res[5] };
  return { V, theta: x[0], freq: x[1], tail: x[2], amp, sym, dyn, lead, power: r.power, power_regen_credit: r.power_regen_credit, Irms: r.Irms, mechP: r.mechP, stalled: r.stalled, crankPeak: r.crankPeak, twistPeak: r.twistPeak, converged: converged && x[1] <= maxFreq, residual_norm_frac_weight: norm(r) / (bird.mTotal * G0), residuals: noises };
}

function solve3(A, b) {
  const m = A.map((r, i) => [...r, b[i]]);
  for (let k = 0; k < 3; k++) {
    let p = k;
    for (let i = k + 1; i < 3; i++) if (Math.abs(m[i][k]) > Math.abs(m[p][k])) p = i;
    [m[k], m[p]] = [m[p], m[k]];
    if (Math.abs(m[k][k]) < 1e-12) return null;
    for (let i = k + 1; i < 3; i++) {
      const f = m[i][k] / m[k][k];
      for (let j = k; j < 4; j++) m[i][j] -= f * m[k][j];
    }
  }
  const x = [0, 0, 0];
  for (let i = 2; i >= 0; i--) {
    let v = m[i][3];
    for (let j = i + 1; j < 3; j++) v -= m[i][j] * x[j];
    x[i] = v / m[i][i];
  }
  return x;
}

/** Glide with the wings parked: unknowns pitch, flight path angle and tail. */
export function trimGlide(design, bird, { V, amp, sym = 0, start = null }) {
  let x = start ? [start.theta, start.gamma, start.tail] : [0.0, -0.1, 0.02];
  const f = (u) => cycleResidual(design, bird, { V, theta: u[0], gamma: u[1], freq: 0, tail: u[2], amp, sym, dyn: 0, lead: 0, cycles: 2 });
  let r = f(x);
  const norm = (q) => Math.hypot(q.res[0], q.res[1], 3 * q.res[5]);
  for (let it = 0; it < 15; it++) {
    if (norm(r) < TRIM_TOL * bird.mTotal * G0) break;
    const g = [r.res[0], r.res[1], r.res[5]];
    const J = [[], [], []];
    const h = [0.01, 0.01, 0.01];
    for (let c = 0; c < 3; c++) {
      const y = x.slice();
      y[c] += h[c];
      const q = f(y);
      J[0][c] = (q.res[0] - g[0]) / h[c];
      J[1][c] = (q.res[1] - g[1]) / h[c];
      J[2][c] = (q.res[5] - g[2]) / h[c];
    }
    const dx = solve3(J, g.map((v) => -v));
    if (!dx) return null;
    let step = 1;
    for (let c = 0; c < 3; c++) step = Math.min(step, 0.1 / Math.max(Math.abs(dx[c]), 1e-9));
    let best = null;
    for (let a = step; a >= step / 16; a /= 2) {
      const y = [x[0] + a * dx[0], x[1] + a * dx[1], x[2] + a * dx[2]];
      const q = f(y);
      if (norm(q) < norm(r)) { best = { y, q }; break; }
    }
    if (!best) break;
    x = best.y;
    r = best.q;
  }
  return { V, theta: x[0], gamma: x[1], tail: x[2], sink: -V * Math.sin(x[1]), LD: -1 / Math.tan(x[1]), stalled: r.stalled, converged: norm(r) < TRIM_TOL * bird.mTotal * G0, residual_norm_frac_weight: norm(r) / (bird.mTotal * G0) };
}

/** Mean inertia about the origin at mid stroke, ignoring products, for gain sizing. */
export function meanInertia(bird) {
  const { lm, st, Ib, Sb, mb, wing, ld } = bird;
  let Ix = Ib[0], Iy = Ib[1], Iz = Ib[2];
  let Sx = Sb[0], Sy = Sb[1], m = mb;
  for (let i = 0; i < st.n; i++) {
    const z = wing.hinge[2] + st.s[i];
    const x = ld[i];
    const M = 2 * lm[i];
    Ix += M * z * z;
    Iy += M * (x * x + z * z);
    Iz += M * x * x;
    Sx += M * x;
    m += M;
  }
  // Move to the CG (parallel axis); controller works about the CG in effect
  const xc = Sx / m;
  return { Ix, Iy: Iy - m * xc * xc, Iz: Iz - m * xc * xc, xCg: xc, m };
}

/** Crank torque the gear train is rated for: each stage's gear and pinion against its published allowable. */
export function trainCrankRating(stages) {
  let r = Infinity;
  for (let k = 0; k < stages.length; k++) {
    const below = stages.slice(k + 1).reduce((a, q) => a * q.ratio, 1);
    const g = stages[k];
    if (Number.isFinite(g.gear_allowable_Nm)) r = Math.min(r, g.gear_allowable_Nm * below);
    if (Number.isFinite(g.pinion_allowable_Nm)) r = Math.min(r, g.pinion_allowable_Nm * g.ratio * below);
  }
  return r;
}

/**
 * Which component ratings a stroke (frequency, amplitude) at a trim condition respects, from a frozen-body cycle:
 * gear train (crank torque peak), motor (rms current within its continuous rating; voltage at the end of usable
 * charge), ESC and battery (mean current), twist servo (peak torque within stall, and the schedule's rate within the
 * loaded speed), inner spar (root bending within the published strength over FoS 1.5). Returns the worst ratio.
 */
export function strokeLimits(design, bird, row, freq, amp) {
  const r = cycleResidual(design, bird, { V: row.V, theta: row.theta, freq, tail: row.tail, amp, sym: row.sym ?? 0, dyn: row.dyn ?? 0, lead: row.lead ?? 0 });
  const m = design.motor, bt = design.battery, sv = design.servos.twist, sp = design.structure.spar_inner;
  const G = design.drive.gearRatio;
  const Vlow = batteryOcvCells(bt, 0.2) - (r.battMean ?? 0) * bt.R_int;
  const Vneed = (freq * TWO_PI * G) / m.kv_rad_per_v_s + r.Irms * m.R;
  const twistRate = (row.dyn ?? 0) * TWO_PI * freq;
  const servoSpeed = sv.speed_rad_s * Math.max(0, 1 - r.twistPeak / sv.stall_Nm);
  const ratios = {
    gear: r.crankPeak / trainCrankRating(design.structure.gear_stages),
    motor_current: r.Irms / (m.I_cont ?? Infinity),
    motor_voltage: Vneed / Vlow,
    battery_esc_current: (r.battMean ?? 0) / Math.min(design.esc.I_cont ?? Infinity, bt.I_cont ?? Infinity),
    twist_servo_torque: r.twistPeak / sv.stall_Nm,
    inner_spar: (r.rootBendPeak * sp.c) / sp.I / 1e6 / (sp.allowable_MPa / 1.5),
  };
  let worst = 'gear';
  for (const k of Object.keys(ratios)) if (ratios[k] > ratios[worst]) worst = k;
  // A twist schedule faster than the servo is not a rating breach: the servo lags and the twist it reaches is
  // smaller, which the flight model simulates. Reported, not limiting.
  const info = { twist_rate_needed_over_loaded_speed: twistRate > 0 ? twistRate / Math.max(servoSpeed, 1e-9) : 0 };
  return { ok: ratios[worst] <= 1, worst, ratio: ratios[worst], ratios, info };
}

function batteryOcvCells(bt, soc) {
  // Same rested curve as the flight model (flight.js batteryOcv), here only for the envelope's low-charge voltage.
  const pts = [[0, 3.3], [0.05, 3.5], [0.1, 3.6], [0.2, 3.68], [0.4, 3.77], [0.6, 3.86], [0.8, 4.0], [1, 4.18]];
  const shift = bt.cell_shift_V ?? 0;
  for (let i = 1; i < pts.length; i++) if (soc <= pts[i][0]) {
    const f = (soc - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0]);
    return bt.cells * (pts[i - 1][1] + f * (pts[i][1] - pts[i - 1][1]) + shift);
  }
  return bt.cells * (4.18 + shift);
}

/**
 * Stroke envelope from component ratings (SPEC ruling: not from fixed multipliers). For amplitudes across the
 * drive's range, the highest frequency at which every rating in strokeLimits holds, at the slowest and fastest trims.
 */
export function strokeEnvelope(design, bird, rows, { nAmp = 5, fCap = null } = {}) {
  const lo = design.drive.ampMin, hi = design.drive.ampMax;
  const fTop = fCap ?? 3 * Math.max(...rows.map((r) => r.freq));
  const out = [];
  for (let i = 0; i < nAmp; i++) {
    const amp = lo + ((hi - lo) * i) / (nAmp - 1);
    let fmax = Infinity, limitedBy = null, at = null;
    for (const row of rows) {
      const okAt = (f) => strokeLimits(design, bird, row, f, amp);
      let a = 0.5, b = fTop;
      const qa = okAt(a);
      if (!qa.ok) { fmax = 0; limitedBy = qa.worst; at = row.V; break; }
      const qb = okAt(b);
      let lim = qb.ok ? 'search cap' : qb.worst;
      if (!qb.ok) {
        for (let it = 0; it < 8; it++) {
          const m = 0.5 * (a + b);
          const q = okAt(m);
          if (q.ok) a = m; else { b = m; lim = q.worst; }
        }
      } else a = b;
      if (a < fmax) { fmax = a; limitedBy = lim; at = row.V; }
    }
    out.push({ amp, freqMax: fmax, limitedBy, atV: at });
  }
  return out;
}

/** Build the controller's table: trims across speed, effectiveness by finite difference, and gains. */
export function buildControl(design, sched, { speeds, bird: given = null, omegaTheta = 3, omegaPhi = 2.5 } = {}) {
  const bird = given ?? buildBird(design);
  const trims = [];
  let start = { theta: 0.08, freq: 1.5, tail: 0.02 };
  for (const V of speeds) {
    const t = trimLevel(design, bird, { V, amp: sched.amp, sym: sched.sym, dyn: sched.dyn, lead: sched.lead, start });
    if (t && t.converged) {
      trims.push(t);
      start = t;
    }
  }
  if (!trims.length) return null;
  // Effectiveness at the mid-speed trim
  const mid = trims[Math.floor(trims.length / 2)];
  const base = { V: mid.V, theta: mid.theta, freq: mid.freq, tail: mid.tail, amp: mid.amp, sym: mid.sym, dyn: mid.dyn, lead: mid.lead };
  const r0 = cycleResidual(design, bird, base);
  const rp = cycleResidual(design, bird, { ...base, tail: base.tail + 0.05 });
  const rd = cycleResidual(design, bird, { ...base, diff: 0.05 });
  const ry = cycleResidual(design, bird, { ...base, yaw: 0.05 });
  const rf = cycleResidual(design, bird, { ...base, freq: base.freq + 0.2 });
  const Md = (rp.res[5] - r0.res[5]) / 0.05;
  const Ld = (rd.res[3] - r0.res[3]) / 0.05;
  const Nd = (ry.res[4] - r0.res[4]) / 0.05;
  const Tf = (rf.res[0] - r0.res[0]) / 0.2;
  // Rate damping, measured the same way: hold a body rate and read the moment.
  const rq = cycleResidual(design, bird, { ...base, omega: [0, 0, 0.2] });
  const rr = cycleResidual(design, bird, { ...base, omega: [0.2, 0, 0] });
  const ryw = cycleResidual(design, bird, { ...base, omega: [0, 0.2, 0] });
  const Mq = -(rq.res[5] - r0.res[5]) / 0.2; // positive: damping
  const Lp = -(rr.res[3] - r0.res[3]) / 0.2;
  const Nr = -(ryw.res[4] - r0.res[4]) / 0.2;
  // Twist servo authority (real torque and speed): torque at trim and per radian of differential twist.
  const sv = design.servos.twist;
  const twistPerDiff = (rd.twistPeak - r0.twistPeak) / 0.05;
  const diffGeom = sched.diffMax ?? 18 * DEG;
  const diffServo = twistPerDiff > 1e-6 ? (sv.stall_Nm - r0.twistPeak) / twistPerDiff : diffGeom;
  const diffMax = Math.max(0, Math.min(diffGeom, diffServo));
  const diffRate = sv.speed_rad_s * Math.max(0, 1 - r0.twistPeak / sv.stall_Nm);
  const I = meanInertia(bird);
  const zeta = 0.85;
  // Gains for a plant J s^2 + D s with control power B: with P gain K, the slow pole is near K B / D when D/J
  // is large (the wings make roll damping-dominated) and near sqrt(K B / J) when it is small. Use both terms.
  const ab = (v) => Math.max(Math.abs(v), 1e-3);
  const kTheta = (I.Iz * omegaTheta * omegaTheta + omegaTheta * Math.max(Mq, 0)) / ab(Md);
  const kQ = Math.max(0, 2 * zeta * omegaTheta * I.Iz - Math.max(Mq, 0)) / ab(Md);
  const kPhi = (I.Ix * omegaPhi * omegaPhi + omegaPhi * Math.max(Lp, 0)) / ab(Ld);
  const kP = Math.max(0, 2 * zeta * omegaPhi * I.Ix - Math.max(Lp, 0)) / ab(Ld);
  const kR = Math.max(0.5 * Math.max(Nr, 0), 2 * 0.7 * 2.0 * I.Iy) / ab(Nd);
  const m = I.m;
  // Energy loop on flap frequency: speed error closes with a time constant of about 4 s.
  const kFv = (m * 0.25) / Math.max(Math.abs(Tf), 1e-3);
  const kFc = (m * 0.3) / Math.max(Math.abs(Tf), 1e-3);
  const kFcFf = (0.5 * m * G0) / Math.max(mid.V, 4) / Math.max(Math.abs(Tf), 1e-3);
  // Glide table, wings parked at mid-stroke
  // The glide table reaches below the flapping trims, so an approach can be flown slower than any flapping trim
  // (a parked wing does not need the stroke's thrust).
  const glideSpeeds = [];
  for (let v = Math.max(4, Math.round(1.4 * Math.min(...speeds)) / 2); v < Math.min(...speeds) - 1e-9; v += 0.5) glideSpeeds.push(+v.toFixed(2));
  glideSpeeds.push(...speeds);
  const glides = [];
  let gs = null;
  for (const V of glideSpeeds) {
    const g = trimGlide(design, bird, { V, amp: sched.amp, start: gs });
    if (g && g.converged) {
      glides.push(g);
      gs = g;
    }
  }
  // Pitch needed per unit of extra load factor, at cruise: dtheta = W / (q S CL_alpha) per g.
  const qS = 0.5 * design.atmosphere.rho_kg_m3 * mid.V * mid.V * design.geometry.area_m2;
  const CLa = (2 * Math.PI) / (1 + 2 / design.geometry.AR);
  const dThetaPerG = (bird.mTotal * G0) / (qS * CLa);
  // Stroke envelope from component ratings, at the slowest and fastest trims.
  const envelope = strokeEnvelope(design, bird, [trims[0], trims[trims.length - 1]]);
  const usable = envelope.filter((e) => e.freqMax > 0);
  const best = usable.length ? usable.reduce((p, q) => (q.amp * q.freqMax > p.amp * p.freqMax ? q : p)) : null;
  const freqMaxAll = usable.length ? Math.max(...usable.map((e) => e.freqMax)) : Math.max(...trims.map((t) => t.freq));
  // Flight time step: at least 60 steps per flap cycle at the envelope's top frequency, as a multiple of 240 Hz.
  const dtFlight = 1 / (240 * Math.max(1, Math.ceil((60 * freqMaxAll) / 240)));
  return {
    envelope, envelopeBest: best, freqMax: freqMaxAll, dt: dtFlight,
    glide: glides.map((g) => ({ V: g.V, theta: g.theta, gamma: g.gamma, tail: g.tail, sink: g.sink, LD: g.LD, stalled: g.stalled })),
    dThetaPerG,
    trim: trims.map((t) => ({ V: t.V, theta: t.theta, freq: t.freq, tail: t.tail, amp: t.amp, sym: t.sym, dyn: t.dyn })),
    trimDetail: trims.map((t) => ({ V: t.V, power_w: t.power, Irms_A: t.Irms, stalled: t.stalled, crankPeak_Nm: t.crankPeak, freq: t.freq, theta_deg: t.theta / DEG, tail_deg: t.tail / DEG })),
    Vcruise: sched.V ?? mid.V,
    Vmin: trims[0].V,
    Md, Ld, Nd, Tf, Mq, Lp, Nr,
    inertia: I,
    kTheta, kQ, kPhi, kP, kR, kFv, kFc, kFcFf: Math.min(kFcFf, 1.0),
    kH: 0.5, kGamma: 1.0, kIGamma: 0.15, kStallPitch: 0.08, kIF: 0.08,
    kChi: 1.0, maxBank: 35 * DEG,
    // Roll authority from the real twist servo: differential twist limited by the servo's stall torque against the
    // measured twist moment, and its rate by the servo's loaded speed.
    diffMax, diffRate_rad_s: diffRate, diffLimit: { geometric_rad: diffGeom, servo_rad: diffServo, twist_torque_trim_Nm: r0.twistPeak, twist_torque_per_rad: twistPerDiff, servo_stall_Nm: sv.stall_Nm, servo_speed_rad_s: sv.speed_rad_s, supply_V: sv.volts },
    climbMax: 2.0, sinkMax: 2.5,
    freqMin: Math.max(0.8, Math.min(...trims.map((t) => t.freq)) * 0.5),
    ampNominal: sched.amp,
    // The flight controller's current limit: the lowest continuous rating in the power path.
    currentLimit_A: Math.min(design.motor.I_cont ?? Infinity, design.esc.I_cont ?? Infinity, design.battery.I_cont ?? Infinity),
    ampMax: design.drive.ampMax,
    lead: sched.lead,
    parkPhase: Math.PI / 2,
    attGain: 0.15,
    notchQ: 1.2,
  };
}

/**
 * Trim found in free flight: the body's recoil against heavy wings makes the frozen-body trim optimistic, so
 * fly the closed loop at each table speed and take the mean commands as the table. This is the simulation's
 * equivalent of a trim flight test. Two passes. A row counts only if the flight converged to steady level flight
 * over the averaging window (FLIGHT_TRIM_TOL); otherwise it is kept in trimFlight as not converged and left out.
 */
export const FLIGHT_TRIM_TOL = { climb_m_s: 0.1, speed_m_s: 0.3, freq_drift_frac: 0.02, tail_drift_rad: 0.01 };

export async function refineTrimByFlight(design, { passes = 2, settle = 15, average = 10 } = {}) {
  const { setupFlight } = await import('../physics/scenarios.js');
  const { eulerFromQuat } = await import('../physics/flight.js');
  const ctl = design.control;
  const tol = FLIGHT_TRIM_TOL;
  for (let pass = 0; pass < passes; pass++) {
    const rows = [];
    for (const row of ctl.trim) {
      const { sim, ctl: c, tick } = setupFlight(design, { sensorNoise: false, init: { V: row.V, y: 0 } });
      c.guidance.speed = row.V;
      let tail = 0, th = 0, P = 0, PC = 0, V = 0, n = 0, ok = true;
      const half = [{ f: 0, tail: 0, n: 0 }, { f: 0, tail: 0, n: 0 }];
      const HZ = Math.round(1 / sim.dt);
      const N = Math.round((settle + average) * HZ);
      let y0 = null;
      for (let k = 0; k < N; k++) {
        tick();
        const s = sim.snapshot();
        if (!Number.isFinite(s.pos[1]) || Math.abs(s.pos[1]) > 30) { ok = false; break; }
        if (k >= settle * HZ) {
          if (y0 === null) y0 = s.pos[1];
          const h = half[k >= (settle + average / 2) * HZ ? 1 : 0];
          h.f += sim.cmd.freq; h.tail += sim.cmd.tailPitch; h.n++;
          tail += sim.cmd.tailPitch;
          th += eulerFromQuat(s.quat).theta;
          P += sim.tel.power_w;
          PC += sim.tel.power_regen_credit_w ?? sim.tel.power_w;
          V += sim.tel.airspeed;
          n++;
        }
      }
      const good = ok && n && [half[0].f, half[1].f, tail, th, P, V].every(Number.isFinite);
      if (!good) { rows.push({ ...row, unflyable: true, converged: false, reason: 'diverged' }); continue; }
      const f0 = half[0].f / half[0].n, f1 = half[1].f / half[1].n;
      const climb = (sim.snapshot().pos[1] - y0) / average;
      const crit = {
        climb_m_s: climb,
        speed_err_m_s: V / n - row.V,
        freq_drift_frac: Math.abs(f1 - f0) / Math.max(0.5 * (f0 + f1), 1e-6),
        tail_drift_rad: Math.abs(half[1].tail / half[1].n - half[0].tail / half[0].n),
      };
      const converged = Math.abs(crit.climb_m_s) <= tol.climb_m_s && Math.abs(crit.speed_err_m_s) <= tol.speed_m_s && crit.freq_drift_frac <= tol.freq_drift_frac && crit.tail_drift_rad <= tol.tail_drift_rad;
      const fm = (half[0].f + half[1].f) / (half[0].n + half[1].n);
      rows.push({ ...row, freq: fm, tail: tail / n, theta: th / n, flight_power_w: P / n, flight_power_regen_credit_w: PC / n, flight_V: V / n, converged, convergence: crit, ...(converged ? {} : { unflyable: true, reason: 'not converged' }) });
    }
    const flyable = rows.filter((r) => !r.unflyable).map(({ flight_power_w, flight_power_regen_credit_w, flight_V, unflyable, converged, convergence, reason, ...r }) => r);
    if (flyable.length) ctl.trim = flyable;
    ctl.trimFlight = rows;
    // No converged row: the frozen-body table must not stand in for a flight trim.
    ctl.trimConverged = flyable.length > 0;
    if (!flyable.length) break;
  }
  return design;
}
