// 6-DOF rigid-body flight with flap-cycle-resolved forces and the inertial reaction of the wings.
//
// State is advanced at a fixed step with a midpoint rule; aero lag states step once per time step. Same
// seed, same trajectory. World: right-handed, Y up. Body: x forward, y up, z right. Quaternion [x,y,z,w].
//
// Equations at the body origin O, with wing lumps moving relative to the body (Newton-Euler for a system
// with prescribed internal motion). S = sum m r, I = inertia about O, f = 2 w x rdot + rddot per lump:
//   m a + wdot x S + w x (w x S) + sum m f              = F_ext
//   S x a + I wdot + w x (I w) + sum m r x f            = M_ext
// The crank is a seventh coordinate with the rotor and gear inertia reflected to it.

import { makeWing, makeFlapDrive, strokeAngle, strokeAngleLeft, makeStations, makeSectionState, sectionStates } from './kinematics.js';
import { createStrip } from './strip.js';
import { POLARS, polarEval } from './polars.js';
import { G0, densityAtAltitude, designAtmosphere } from './atmosphere.js';
import { createFcGovernor } from './governor.js';

const TWO_PI = 2 * Math.PI;

export function quatFromEuler(chi, theta, phi) {
  // chi: course/heading to the right of +X; theta: pitch up; phi: bank, right wing down.
  const cy = Math.cos(-chi / 2), sy = Math.sin(-chi / 2);
  const cp = Math.cos(theta / 2), sp = Math.sin(theta / 2);
  const cr = Math.cos(phi / 2), sr = Math.sin(phi / 2);
  // q = qy(about Y) * qp(about Z) * qr(about X)
  // qy*qp
  const a = [sy * sp, sy * cp, cy * sp, cy * cp]; // x,y,z,w of [0,sy,0,cy]*[0,0,sp,cp]
  // (a)*[sr,0,0,cr]
  return [
    a[3] * sr + a[0] * cr,
    a[1] * cr + a[2] * sr,
    a[2] * cr - a[1] * sr,
    a[3] * cr - a[0] * sr,
  ];
}

export function eulerFromQuat(q) {
  const [x, y, z, w] = q;
  const R00 = 1 - 2 * (y * y + z * z), R10 = 2 * (x * y + w * z), R20 = 2 * (x * z - w * y);
  const R11 = 1 - 2 * (x * x + z * z), R12 = 2 * (y * z - w * x);
  return { chi: Math.atan2(R20, R00), theta: Math.asin(Math.max(-1, Math.min(1, R10))), phi: Math.atan2(-R12, R11) };
}

/** Gaussian elimination with partial pivoting on an n x n flat array; A and b are overwritten. */
export function solveN(A, b, x, n) {
  for (let k = 0; k < n; k++) {
    let p = k;
    let big = Math.abs(A[k * n + k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(A[i * n + k]);
      if (v > big) { big = v; p = i; }
    }
    if (p !== k) {
      for (let j = 0; j < n; j++) { const t = A[k * n + j]; A[k * n + j] = A[p * n + j]; A[p * n + j] = t; }
      const t = b[k]; b[k] = b[p]; b[p] = t;
    }
    const d = 1 / A[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = A[i * n + k] * d;
      if (f === 0) continue;
      for (let j = k; j < n; j++) A[i * n + j] -= f * A[k * n + j];
      b[i] -= f * b[k];
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    let v = b[i];
    for (let j = i + 1; j < n; j++) v -= A[i * n + j] * x[j];
    x[i] = v / A[i * n + i];
  }
}

// State layout
const IX = { px: 0, py: 1, pz: 2, qx: 3, qy: 4, qz: 5, qw: 6, vx: 7, vy: 8, vz: 9, wx: 10, wy: 11, wz: 12, phase: 13, crank: 14, amp: 15, twR: 16, twL: 17, tlR: 18, tlL: 19, charge: 20, twRateR: 21, twRateL: 22 };
const NS = 23;
// Twist servo output rate follows its torque-speed-limited target with this mechanical time constant, so the twist
// acceleration is finite. Not published for any catalogue servo: an assumption (typical of small DC servo motors).
// Round 1 and the first round-2 runs let the rate jump at each command update, which made twist acceleration, servo
// torque and apparent-mass loads scale as 1/dt.
export const SERVO_TAU_MECH_S = 0.01;

/**
 * Round 5: load channels the flight model computes every recorded step for the bearing check (all >= 0, N or N m):
 * each coupler's force; its component along the rocker (what a slotted rocker's stroke servo must hold); the crank's
 * load torque; and per wing the root force normal to the hinge axis, the root force along it, and the root moment
 * normal to it (which a pivot's two bearings react as a couple).
 */
export const LOAD_CHANNELS = ['coupler_R', 'coupler_L', 'slot_R', 'slot_L', 'crank_torque', 'root_yz_R', 'root_x_R', 'root_mperp_R', 'root_yz_L', 'root_x_L', 'root_mperp_L'];

/**
 * Supports of design.structure.joints as channel indices and coefficients: { key, rc, rk, ac, ak, X0, Y0 } per support.
 * A support names its loads as { ch, k } terms (bearings.js supportLoads); X0/Y0 are set for a rolling bearing (its
 * ISO 76 equivalent is tracked over time), null for a bushing or rod end.
 */
export function compileSupports(joints) {
  const idx = (c) => {
    const i = LOAD_CHANNELS.indexOf(c);
    if (i < 0) throw new Error(`unknown load channel ${c}`);
    return i;
  };
  const out = [];
  for (const j of joints ?? []) for (const s of j.supports ?? []) {
    out.push({ key: `${j.name}/${s.name}`, rc: (s.radial ?? []).map((t) => idx(t.ch)), rk: (s.radial ?? []).map((t) => Math.abs(t.k)), ac: (s.axial ?? []).map((t) => idx(t.ch)), ak: (s.axial ?? []).map((t) => Math.abs(t.k)), X0: Number.isFinite(s.X0) ? s.X0 : null, Y0: Number.isFinite(s.Y0) ? s.Y0 : 0 });
  }
  return out;
}

/**
 * Everything physical about a bird, as numbers, from a design file. `perturb` applies the robustness cases:
 * { aero: 1.1, mass: 0.95, cgx: 0.005 (m), servoSpeed: 1 }.
 */
export function buildBird(design, { cal = null, perturb = {}, n = 8, inviscid = false, suctionEta = undefined } = {}) {
  const wing = makeWing(design.wing);
  const polar = design.wing.airfoil ?? 'e387-film';
  const strip = createStrip(wing, { n, polar, cal: cal ?? design.calibration?.multipliers ?? null, inviscid, suctionEta });
  if (perturb.aero) strip.aeroScale = perturb.aero;
  const law = design.drive.kind === 'crank-rocker' ? makeFlapDrive(design.drive) : { kind: 'sine', phiMean: design.drive.phiMean ?? 0 };
  const st = strip.stations;
  const kMass = perturb.mass ?? 1;
  // Body lumps with CG shift applied to the mass centre by moving all body lumps
  let mb = 0;
  const Sb = [0, 0, 0];
  const Ib = [0, 0, 0, 0, 0, 0]; // xx yy zz xy xz yz
  const cgx = perturb.cgx ?? 0;
  for (const l of design.mass.lumps) {
    const m = l.mass_kg * kMass;
    const r = [l.pos[0] + cgx, l.pos[1], l.pos[2]];
    mb += m;
    Sb[0] += m * r[0];
    Sb[1] += m * r[1];
    Sb[2] += m * r[2];
    Ib[0] += m * (r[1] * r[1] + r[2] * r[2]);
    Ib[1] += m * (r[0] * r[0] + r[2] * r[2]);
    Ib[2] += m * (r[0] * r[0] + r[1] * r[1]);
    Ib[3] -= m * r[0] * r[1];
    Ib[4] -= m * r[0] * r[2];
    Ib[5] -= m * r[1] * r[2];
    const k = l.gyration_m ?? 0;
    Ib[0] += m * k * k;
    Ib[1] += m * k * k;
    Ib[2] += m * k * k;
  }
  // Wing lumps: per-station mass and chord offset from the spar axis. mass.wing: spar_kg_per_m per segment
  // weight, skin area density. Point masses on a wing (servo) are assigned to their nearest station.
  const wm = design.mass.wing;
  const lm = new Float64Array(st.n);
  const ld = new Float64Array(st.n);
  const lj = new Float64Array(st.n);
  for (let i = 0; i < st.n; i++) {
    const spar = wm.spar_kg_per_m * st.ds[i];
    const skin = wm.skin_kg_per_m2 * st.chord[i] * st.ds[i];
    const c = st.chord[i];
    const dLE = st.xle[i] - wing.spar_x;
    const dSkin = dLE - 0.45 * c;
    const m = spar + skin;
    lm[i] = m * kMass;
    ld[i] = (skin * dSkin) / Math.max(m, 1e-9);
    // Pitch inertia of the section about the spar
    lj[i] = (skin * (dSkin * dSkin + (c * c) / 12)) * kMass;
  }
  for (const p of wm.point ?? []) {
    let bi = 0;
    for (let i = 0; i < st.n; i++) if (Math.abs(st.s[i] - p.s_m) < Math.abs(st.s[bi] - p.s_m)) bi = i;
    const m = p.mass_kg * kMass;
    ld[bi] = (lm[bi] * ld[bi] + m * (p.d_m ?? 0)) / (lm[bi] + m);
    lm[bi] += m;
  }
  let Jtw = 0;
  for (let i = 0; i < st.n; i++) Jtw += st.weight[i] * st.weight[i] * lj[i];
  const mw = 2 * lm.reduce((p, q) => p + q, 0);
  const tail = design.tail;
  const tailPolar = POLARS[tail.polar ?? 'thin-plate'];
  const motor = design.motor;
  const G = design.drive.gearRatio;
  const Jcrank = (motor.J_kg_m2 * G * G) / 1 + (design.drive.J_crank_kg_m2 ?? 0);
  // Tail stations: two strips per surface
  const tn = 2;
  const tailStations = [];
  for (const side of [1, -1]) {
    for (let k = 0; k < tn; k++) {
      const eta = (k + 0.5) / tn;
      const s = eta * tail.span_each_m;
      const taper = tail.taper ?? 0.7;
      const c = tail.chord_root_m * (1 - (1 - taper) * eta);
      const d = tail.dihedral_rad;
      tailStations.push({
        side,
        r: [tail.x_m - (tail.sweep_m ?? 0) * eta, tail.y_m + s * Math.sin(d), side * (tail.z_m + s * Math.cos(d))],
        n: [0, Math.cos(d), -side * Math.sin(d)],
        c,
        ds: tail.span_each_m / tn,
      });
    }
  }
  return {
    design, wing, polar, strip, law, st, lm, ld, lj, Jtw, mw, mb, Sb, Ib, tail, tailPolar, tailStations, motor, G, Jcrank,
    mTotal: mb + mw,
    kAero: perturb.aero ?? 1,
    servoSpeedScale: perturb.servoSpeed ?? 1,
  };
}

/**
 * Battery-side current of the flap ESC for a motor-side current `IbM` (duty x motor current, A, signed). SPEC ruling:
 * no regeneration credit unless the ESC's own documentation states regenerative operation, in which case the
 * returned current is negative with that ESC's stated efficiency (esc.regenerative, esc.regen_eta).
 */
export function escBatteryCurrent(IbM, esc = {}) {
  if (IbM >= 0) return IbM / (esc?.eta ?? 0.95);
  if (esc?.regenerative === true && Number.isFinite(esc.regen_eta)) return IbM * esc.regen_eta;
  return 0;
}

/**
 * Servo output rate towards a command. Linear torque-speed line from the published no-load speed and stall torque:
 * speed falls to zero as the torque the servo must push against reaches stall. A load that helps the motion does
 * not slow it. A load at or beyond stall stops the servo in that direction (assumed not back-driven; M7 fails such
 * a load anyway). Tail hinge moments are carried as magnitudes (signed = false) and so oppose motion either way.
 */
export function servoSlewRate(want, pos, sv, loadTorque, speedScale = 1, signed = true) {
  const r = (want - pos) / sv.tau_s;
  const opposing = !signed ? Math.abs(loadTorque) : r >= 0 ? loadTorque : -loadTorque;
  const frac = 1 - Math.max(0, opposing) / Math.max(sv.stall_Nm, 1e-6);
  const slew = sv.speed_rad_s * speedScale * Math.max(0, frac);
  return r > slew ? slew : r < -slew ? -slew : r;
}

export function batteryOcv(batt, soc) {
  // Rested LiPo curve per cell. Shape of a common 3.3-4.2 V discharge; capacity comes from the catalogue.
  const s = Math.max(0, Math.min(1, soc));
  const pts = [[0, 3.3], [0.05, 3.5], [0.1, 3.6], [0.2, 3.68], [0.4, 3.77], [0.6, 3.86], [0.8, 4.0], [1, 4.18]];
  const shift = batt.cell_shift_V ?? 0; // published nominal cell voltage minus 3.7 (Li-ion packs)
  for (let i = 1; i < pts.length; i++) {
    if (s <= pts[i][0]) {
      const f = (s - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0]);
      return batt.cells * (pts[i - 1][1] + f * (pts[i][1] - pts[i - 1][1]) + shift);
    }
  }
  return batt.cells * (4.18 + shift);
}

/**
 * Creates a flight. opts: { dt, wind, cal, perturb, delayed commands provided by the controller }.
 * The controller is called at its own rate through opts.controller.update(t, sensorsInput) and returns cmd.
 */
export function createFlight(design, opts = {}) {
  const dt = opts.dt ?? 1 / 240;
  const bird = opts.bird ?? buildBird(design, { cal: opts.cal, perturb: opts.perturb ?? {} });
  const { wing, strip, law, st, lm, ld, lj, Jtw, mb, Sb, Ib, tail, tailPolar, tailStations, motor, G, Jcrank } = bird;
  const n = st.n;
  const mu = designAtmosphere().mu;
  const groundY = opts.groundY ?? -1e9;
  const windModel = opts.wind ?? null;
  const x = new Float64Array(NS);
  const k1 = new Float64Array(NS);
  const k2 = new Float64Array(NS);
  const mid = new Float64Array(NS);

  const servoT = design.servos.twist;
  // Round 5: the crank governor and drive clamp in the flight controller (governor.js), sampled from the encoder and the
  // FC's voltage sense, when the design puts them there (drive.governor.location); otherwise round 4's ESC-side law.
  const fcGov = design.drive.governor?.location === 'flight controller' ? createFcGovernor(design.drive.governor, { G, kv: motor.kv_rad_per_v_s, R: motor.R, cells: design.battery.cells, kp: motor.govKp ?? 0.5, ki: motor.govKi ?? 1.0 }) : null;
  let tEval = 0; // time of the state deriv() is evaluating (the encoder phase is extrapolated to it)
  const servoP = design.servos.tail;
  const servoA = design.servos.stroke ?? null;
  const speedScale = bird.servoSpeedScale;

  const R = strip.right;
  const L = strip.left;
  const lumpsN = 2 * n;
  const lr = new Float64Array(lumpsN * 3);
  const lv = new Float64Array(lumpsN * 3);
  const la = new Float64Array(lumpsN * 3);
  const lp = new Float64Array(lumpsN * 3); // partial of lump position with respect to shoulder angle

  // Commands in effect (from the controller, after its delay)
  const cmd = { freq: 0, amp: design.cruise?.amp ?? 0.4, sym: 0, diff: 0, dyn: 0, lead: 0, tailPitch: 0, tailYaw: 0, parkPhase: null, throttleOff: false };
  // Slow outer feedback
  const aBody = [0, 0, 0]; // absolute acceleration of O last step, body axes
  const wDot = [0, 0, 0];
  let thetaDD = 0; // crank acceleration
  let t = 0;
  let wind = { world: [0, 0, 0], rates: [0, 0, 0] };
  const frozen = { body: !!opts.frozenBody };
  const imposeTwist = !!opts.imposeTwist;
  const rhoOverride = opts.rho ?? design.rho_override ?? null;
  const gAcc = opts.gravity ?? G0;
  const thddPrev = [0];
  const bodyAcc = { a: aBody, wd: wDot, thdd: thddPrev, dph: 0, dphW: [0, 0], crank: 0 };
  // Round 4: one crank pin drives both couplers (design.drive.pinLayout 'shared'): the left wing follows its own
  // mirrored linkage (kinematics strokeAngleLeft), so each wing has its own stroke angle and velocity ratio.
  const sharedPin = design.drive.pinLayout === 'shared' && law.kind === 'crank-rocker';
  const hingeZ = design.wing.hinge[2];
  const A = new Float64Array(36);
  const b = new Float64Array(6);
  const sol = new Float64Array(6);
  const A7 = new Float64Array(49);
  const b7 = new Float64Array(7);
  const sol7 = new Float64Array(7);
  // Tail state
  let epsLag = 0;
  let duty = 0;
  let iInt = 0; // ESC speed-governor integrator
  const lastTwistRate = [0, 0];
  const twistAccel = [0, 0];
  const lastTwistTorque = [0, 0];
  const tailTorque = [0, 0];
  const pol = new Float64Array(4);
  const wingQ = [{}, {}];
  // Telemetry of the most recent step
  const tel = {
    power_w: 0, batt_V: 0, batt_I: 0, motor_I: 0, motor_V: 0, servo_I: 0, duty: 0, airspeed: 0, alpha: 0, beta: 0, sf: 0,
    stalled: 0, rootBend: 0, rootBendChord: 0, elbowBend: 0, shoulderTorque: 0, crankTorque: 0, motorTorque: 0, twistTorque: [0, 0], tailTorque: [0, 0],
    phi: 0, mu: Math.PI / 2, couplerForce: 0, loadFactor: 1, Fx: 0, Fy: 0, Fz: 0, climbRate: 0,
    residual: new Float64Array(6), Mx: 0, My: 0, Mz: 0, aeroPower: 0,
  };
  // Round 5: load channels of the drive and the supports of design.structure.joints, compiled to channel indices.
  const rootLoad = new Float64Array(6); // per wing: |F| normal to the hinge axis, |F| along it, |M| normal to it
  const chv = new Float64Array(LOAD_CHANNELS.length);
  const sup = compileSupports(design.structure?.joints);
  const supLoad = new Float64Array(3 * sup.length);
  // Worst-case tracking for M7, M8
  const peaks = { servoEach: 0, becI: 0, crankDrive: 0, crankBack: 0, rootBend: 0, elbowBend: 0, crankTorque: 0, motorI: 0, battI: 0, servoI: 0, twistTorque: 0, tailTorque: 0, couplerForce: 0, loadFactor: 0, shoulderTorque: 0, crankRate: 0, motorIDrive: 0, motorIBrake: 0, channels: Object.fromEntries(LOAD_CHANNELS.map((c) => [c, 0])), slotPair: 0, supports: sup.map((s) => ({ key: s.key, Fr: 0, Fa: 0, P0: 0 })) };
  const rmsAcc = { motorI2: 0, battI2: 0, servoI2: 0, servoItwist2: 0, n: 0, motorI2Win: 0, nWin: 0, motorRmsMax: 0, battRmsMax: 0, battI2Win: 0 };
  const servoSpec = (s) => s;

  function deriv(S, out, advanceDt, record) {
    const qx = S[3], qy = S[4], qz = S[5], qw = S[6];
    const R00 = 1 - 2 * (qy * qy + qz * qz), R01 = 2 * (qx * qy - qw * qz), R02 = 2 * (qx * qz + qw * qy);
    const R10 = 2 * (qx * qy + qw * qz), R11 = 1 - 2 * (qx * qx + qz * qz), R12 = 2 * (qy * qz - qw * qx);
    const R20 = 2 * (qx * qz - qw * qy), R21 = 2 * (qy * qz + qw * qx), R22 = 1 - 2 * (qx * qx + qy * qy);
    const vwx = S[7], vwy = S[8], vwz = S[9];
    const ub = R00 * vwx + R10 * vwy + R20 * vwz;
    const vb = R01 * vwx + R11 * vwy + R21 * vwz;
    const wb = R02 * vwx + R12 * vwy + R22 * vwz;
    const wx = S[10], wy = S[11], wz = S[12];
    const windW = wind.world;
    const wub = R00 * windW[0] + R10 * windW[1] + R20 * windW[2];
    const wvb = R01 * windW[0] + R11 * windW[1] + R21 * windW[2];
    const wwb = R02 * windW[0] + R12 * windW[1] + R22 * windW[2];
    // Rate seen by the air: body rate minus the rotary part of the gust field
    const wg = wind.rates;
    const omAir = [wx - wg[0], wy - wg[1], wz - wg[2]];
    const rho = rhoOverride ?? densityAtAltitude(Math.max(0, (design.atmosphere?.altitude_m ?? 1800) + S[1]));
    const phase = S[13], crank = S[14], amp = S[15];
    // Stroke
    const sa = strokeAngle(phase, amp, law);
    const saL = sharedPin ? strokeAngleLeft(phase, amp, law, hingeZ) : sa;
    const dphi0 = sa.dphi * crank;
    const ddphi0 = sa.ddphi * crank * crank;
    // Twist command is the servo state; rate from the servo law
    const lead = cmd.lead;
    const twCmdR = cmd.sym + cmd.diff - cmd.dyn * Math.sin(phaseCmd(phase, crank) + lead);
    const twCmdL = cmd.sym - cmd.diff - cmd.dyn * Math.sin(phaseCmd(phase, crank) + lead);
    let rateR, rateL, accR, accL;
    if (imposeTwist) {
      // Kinematics imposed exactly (V4): the twist is the schedule itself, whatever torque that takes.
      const ph = phaseCmd(phase, crank) + lead;
      rateR = rateL = -cmd.dyn * Math.cos(ph) * crank;
      accR = accL = cmd.dyn * Math.sin(ph) * crank * crank;
      S[16] = twCmdR;
      S[17] = twCmdL;
      S[21] = rateR;
      S[22] = rateL;
    } else {
      const tauM = servoT.tau_mech_s ?? SERVO_TAU_MECH_S;
      rateR = S[21];
      rateL = S[22];
      accR = (servoRate(twCmdR, S[16], servoT, lastTwistTorque[0]) - rateR) / tauM;
      accL = (servoRate(twCmdL, S[17], servoT, lastTwistTorque[1]) - rateL) / tauM;
    }
    const jR = { phi: sa.phi, dphi: dphi0, ddphi: ddphi0, theta: S[16], dtheta: rateR, ddtheta: accR };
    const jL = { phi: saL.phi, dphi: saL.dphi * crank, ddphi: saL.ddphi * crank * crank, theta: S[17], dtheta: rateL, ddtheta: accL };
    // Wing aero (fills section states)
    bodyAcc.dph = sa.dphi;
    bodyAcc.dphW[0] = sa.dphi;
    bodyAcc.dphW[1] = saL.dphi;
    bodyAcc.crank = crank;
    const aero = strip.evaluate([ub, vb, wb], omAir, [wub, wvb, wwb], jR, jL, rho, mu, advanceDt, frozen.body ? null : bodyAcc);
    let Fx = aero.F[0], Fy = aero.F[1], Fz = aero.F[2];
    let Mx = aero.M[0], My = aero.M[1], Mz = aero.M[2];

    // Tail surfaces and body: only cycle-averaged downwash reaches the tail.
    const relU = wub - ub, relV = wvb - vb, relW = wwb - wb;
    const Vair = Math.sqrt(relU * relU + relV * relV + relW * relW) + 1e-9;
    const liftNow = aero.F[1];
    const qS = 0.5 * rho * Vair * Vair * strip.area;
    const CLw = liftNow / Math.max(qS, 1e-3);
    if (advanceDt > 0) {
      const tauE = Math.max(0.05, Math.abs(tail.x_m) / Vair);
      epsLag += (advanceDt / tauE) * (CLw - epsLag);
    }
    // Downwash at the tail from the wing's lift (elliptic far wake value scaled by kEps)
    const AR = ((2 * (wing.hinge[2] + wing.L)) ** 2) / strip.area;
    const eps = ((tail.kEps ?? 1.0) * 2 * epsLag) / (Math.PI * AR);
    let tailPower = 0;
    let tailMR = 0, tailML = 0;
    for (let i = 0; i < tailStations.length; i++) {
      const ts = tailStations[i];
      const r = ts.r;
      const deflect = ts.side > 0 ? S[18] : S[19];
      const vrx = relU - (omAir[1] * r[2] - omAir[2] * r[1]);
      const vry = relV - (omAir[2] * r[0] - omAir[0] * r[2]);
      const vrz = relW - (omAir[0] * r[1] - omAir[1] * r[0]);
      // Section axes: normal n (stroke-plane sense), chord towards LE: x
      const vn0 = vrx * 0 + vry * ts.n[1] + vrz * ts.n[2];
      const vc = -vrx;
      const vnE = vn0 - eps * Math.max(vc, 0) * ts.n[1];
      const inc = (tail.incidence_rad ?? 0) + deflect;
      const alphaFlow = Math.atan2(vnE, vc);
      const al = alphaFlow + inc;
      const V2 = vc * vc + vnE * vnE;
      const Vt = Math.sqrt(V2) + 1e-9;
      polarEval(tailPolar, al, (Vt * ts.c * rho) / mu, bird.kAero, bird.kAero, pol);
      const qq = 0.5 * rho * V2 * ts.c * ts.ds;
      // Finite span: reduce lift slope by the surface's own aspect ratio.
      const lf = (tail.lift_factor ?? 0.75);
      const Lt = qq * pol[0] * lf;
      const Dt = qq * (pol[1] + (pol[0] * lf) ** 2 / (Math.PI * (tail.AR_each ?? 4) * 0.8));
      const ca = Math.cos(alphaFlow), sa2 = Math.sin(alphaFlow);
      // Lift normal to the flow in the (x, n) plane; drag along the flow.
      const fn = Lt * ca + Dt * sa2;
      const fc = -Lt * sa2 + Dt * ca; // along -x
      const fx = -fc;
      const fy = fn * ts.n[1];
      const fz = fn * ts.n[2];
      Fx += fx;
      Fy += fy;
      Fz += fz;
      Mx += r[1] * fz - r[2] * fy;
      My += r[2] * fx - r[0] * fz;
      Mz += r[0] * fy - r[1] * fx;
      tailPower += 0;
      // Hinge moment about the surface's quarter chord; pivot assumed at 25% c: from the moment coefficient.
      const hm = Math.abs(qq * ts.c * pol[2]) + 0.02 * Math.abs(qq * ts.c * pol[0]);
      if (ts.side > 0) tailMR += hm;
      else tailML += hm;
    }
    // Fuselage: drag by axis and a Munk moment, which is destabilising.
    const bd = design.body;
    const speedAir = Vair;
    const q0 = 0.5 * rho * speedAir;
    const fuX = q0 * bd.CdA_front_m2 * relU;
    const fuY = q0 * bd.CdA_top_m2 * relV;
    const fuZ = q0 * bd.CdA_side_m2 * relW;
    Fx += fuX * bird.kAero;
    Fy += fuY * bird.kAero;
    Fz += fuZ * bird.kAero;
    {
      const r = bd.cp ?? [0, 0, 0];
      Mx += r[1] * fuZ - r[2] * fuY;
      My += r[2] * fuX - r[0] * fuZ;
      Mz += r[0] * fuY - r[1] * fuX;
      // Munk: M = q Vol sin(2 alpha) about +z for alpha nose up, and analogously in yaw.
      const qv = 0.5 * rho * bd.volume_m3;
      Mz += qv * 2 * (-relU) * relV * (bd.kMunk ?? 0.8);
      My += qv * 2 * relU * relW * (bd.kMunk ?? 0.8);
    }
    // All external force but gravity: its magnitude over the weight is the load factor at the system's mass centre
    // (Newton for the whole system of body and moving wings: M a_cg = F_aero + M g).
    const FaX = Fx, FaY = Fy, FaZ = Fz;
    // Gravity in body axes
    const gxb = -gAcc * R10, gyb = -gAcc * R11, gzb = -gAcc * R12;
    // Lumps: positions, rates, and accelerations without the crank's angular acceleration.
    let Sx = Sb[0], Sy = Sb[1], Sz = Sb[2];
    let Ixx = Ib[0], Iyy = Ib[1], Izz = Ib[2], Ixy = Ib[3], Ixz = Ib[4], Iyz = Ib[5];
    let mT = mb;
    let iPhi = 0; // sum m |dr/dphi|^2, the wing inertia seen by the shoulder, both wings
    let Qrest = 0; // generalised gravity and inertial force on phi, without a, wdot and the crank's own acceleration
    let sumfx = 0, sumfy = 0, sumfz = 0;
    let sumtx = 0, sumty = 0, sumtz = 0;
    let cFx = 0, cFy = 0, cFz = 0, cMx = 0, cMy = 0, cMz = 0; // coupling of crank acceleration into the body rows
    const dph = sa.dphi;
    const dphL = saL.dphi;
    for (let w = 0; w < 2; w++) {
      const W = w === 0 ? R : L;
      // Per-wing shares of the same sums, for each shoulder's own load (couplers are separate per wing).
      const qr0 = Qrest, i0 = iPhi, f0y0 = cFy, f0z0 = cFz, m0x = cMx, m0y = cMy, m0z = cMz;
      for (let i = 0; i < n; i++) {
        const j = 3 * i;
        const q = 3 * (i + w * n);
        const m = lm[i];
        const d = ld[i];
        const rx = W.A[j] + d * W.t[j], ry = W.A[j + 1] + d * W.t[j + 1], rz = W.A[j + 2] + d * W.t[j + 2];
        const dx = W.Ad[j] + d * W.td[j], dy = W.Ad[j + 1] + d * W.td[j + 1], dz = W.Ad[j + 2] + d * W.td[j + 2];
        const ax = W.Add[j] + d * W.tdd[j], ay = W.Add[j + 1] + d * W.tdd[j + 1], az = W.Add[j + 2] + d * W.tdd[j + 2];
        const py = W.Ap[j + 1] + d * W.tp[j + 1], pz = W.Ap[j + 2] + d * W.tp[j + 2];
        lr[q] = rx; lr[q + 1] = ry; lr[q + 2] = rz;
        lv[q] = dx; lv[q + 1] = dy; lv[q + 2] = dz;
        lp[q] = 0; lp[q + 1] = py; lp[q + 2] = pz;
        Sx += m * rx; Sy += m * ry; Sz += m * rz;
        mT += m;
        Ixx += m * (ry * ry + rz * rz);
        Iyy += m * (rx * rx + rz * rz);
        Izz += m * (rx * rx + ry * ry);
        Ixy -= m * rx * ry;
        Ixz -= m * rx * rz;
        Iyz -= m * ry * rz;
        iPhi += m * (py * py + pz * pz);
        // Relative part f0 = 2 w x rdot + rddot0, and the centripetal term w x (w x r)
        const f0x = 2 * (wy * dz - wz * dy) + ax;
        const f0y = 2 * (wz * dx - wx * dz) + ay;
        const f0z = 2 * (wx * dy - wy * dx) + az;
        la[q] = f0x; la[q + 1] = f0y; la[q + 2] = f0z;
        const wrx = wy * rz - wz * ry, wry = wz * rx - wx * rz, wrz = wx * ry - wy * rx;
        const wwx = wy * wrz - wz * wry, wwy = wz * wrx - wx * wrz, wwz = wx * wry - wy * wrx;
        Qrest += m * ((gyb - wwy - f0y) * py + (gzb - wwz - f0z) * pz);
        sumfx += m * f0x; sumfy += m * f0y; sumfz += m * f0z;
        sumtx += m * (ry * f0z - rz * f0y);
        sumty += m * (rz * f0x - rx * f0z);
        sumtz += m * (rx * f0y - ry * f0x);
        cFy += m * py; cFz += m * pz;
        cMx += m * (ry * pz - rz * py);
        cMy += m * (-rx * pz);
        cMz += m * (rx * py);
      }
      const wq = wingQ[w];
      wq.Qrest = Qrest - qr0; wq.iPhi = iPhi - i0; wq.cFy = cFy - f0y0; wq.cFz = cFz - f0z0;
      wq.cMx = cMx - m0x; wq.cMy = cMy - m0y; wq.cMz = cMz - m0z;
    }
    // Crank: rotor, gear and reflected wing inertia, solved together with the body below.
    const Qaero = aero.Qphi[0] + aero.Qphi[1];
    const Vbus = batteryState(S[20]);
    const omegaM = crank * G;
    const wantCrank = crankTarget(phase, crank);
    const motorKt = 1 / motor.kv_rad_per_v_s;
    const KpV = (motor.govKp ?? 0.5) * (G / motor.kv_rad_per_v_s); // volts per crank rad/s
    let dutyNow = duty;
    let clamped = false;
    if (cmd.throttleOff && cmd.parkPhase === null) dutyNow = 0;
    else if (fcGov) {
      dutyNow = fcGov.st.applied;
      clamped = fcGov.st.clampedApplied;
    } else {
      const errW = wantCrank - crank;
      const Vneed = wantCrank * G / motor.kv_rad_per_v_s;
      dutyNow = Math.max(0, Math.min(1, (Vneed + KpV * errW + iInt) / Vbus));
    }
    // Round 4 sensitivity (lever 1, fast form): a drive-side current clamp in the ESC, which knows its duty, its input
    // voltage and the commutation speed: duty <= (E + I_clamp R) / V caps the motor current at I_clamp whenever the
    // drive pushes. It cannot touch braking (the crank driving the motor). The catalogue ESC publishes no such feature.
    const Iclamp = design.drive.escCurrentClamp_A;
    if (!fcGov && Number.isFinite(Iclamp) && !(cmd.throttleOff && cmd.parkPhase === null)) {
      const dMax = (omegaM / motor.kv_rad_per_v_s + Iclamp * motor.R) / Vbus;
      if (dutyNow > dMax) { dutyNow = Math.max(0, dMax); clamped = true; }
    }
    const Vm = dutyNow * Vbus;
    const Iraw = (Vm - omegaM / motor.kv_rad_per_v_s) / motor.R;
    const Ilimit = motor.regenLimit_A ?? -0.5 * (motor.I_cont ?? 4);
    const Im = Math.max(Ilimit, Iraw);
    const tauM = motorKt * (Im - (crank !== 0 ? Math.sign(crank) * motor.I0 : 0));
    const etaG = design.drive.gearEfficiency ?? 0.85;
    const tauC = (tauM * crank >= 0 ? tauM * etaG : tauM / etaG) * G;
    // Each wing's inertia and generalised forces reach the crank through its own velocity ratio.
    const wR = wingQ[0], wL = wingQ[1];
    let Jeff = Jcrank + wR.iPhi * dph * dph + wL.iPhi * dphL * dphL + 1e-6;
    // The motor's speed feedback (back-EMF plus governor) is a fast pole next to the time step: implicit.
    if (Iraw > Ilimit) {
      // At the clamp the current no longer falls with speed: no back-EMF or governor stiffness to treat implicitly.
      // With the governor in the flight controller the duty is held between its ticks: only the back-EMF is implicit.
      const B = fcGov ? (motorKt * etaG * G * (G / motor.kv_rad_per_v_s)) / motor.R : clamped ? 0 : (motorKt * etaG * G * (G / motor.kv_rad_per_v_s + (dutyNow > 0 && dutyNow < 1 ? KpV : 0))) / motor.R;
      Jeff += 0.5 * dt * B;
    }
    const crankRhs = tauC + dph * (aero.Qphi[0] + wR.Qrest) + dphL * (aero.Qphi[1] + wL.Qrest) - (design.drive.crankFriction_Nm ?? 0) * Math.tanh(crank / 0.5);
    const parked = cmd.parkPhase !== null && Math.abs(wantCrank) < 1e-6 && Math.abs(crank) < 0.3 && motor.holdable !== false;
    // Gravity
    Fx += mT * gxb;
    Fy += mT * gyb;
    Fz += mT * gzb;
    Mx += Sy * gzb - Sz * gyb;
    My += Sz * gxb - Sx * gzb;
    Mz += Sx * gyb - Sy * gxb;
    // Assemble: rows m a + wdot x S + c thdd = F - w x (w x S) - sum m f0 ; S x a + I wdot + cM thdd = ...
    const wSx = wy * Sz - wz * Sy, wSy = wz * Sx - wx * Sz, wSz = wx * Sy - wy * Sx;
    const wwSx = wy * wSz - wz * wSy, wwSy = wz * wSx - wx * wSz, wwSz = wx * wSy - wy * wSx;
    const Iwx = Ixx * wx + Ixy * wy + Ixz * wz, Iwy = Ixy * wx + Iyy * wy + Iyz * wz, Iwz = Ixz * wx + Iyz * wy + Izz * wz;
    const gx1 = wy * Iwz - wz * Iwy, gy1 = wz * Iwx - wx * Iwz, gz1 = wx * Iwy - wy * Iwx;
    A7.fill(0);
    const set = (r, c, v) => { A7[r * 7 + c] = v; };
    set(0, 0, mT); set(1, 1, mT); set(2, 2, mT);
    set(0, 4, Sz); set(0, 5, -Sy); set(1, 3, -Sz); set(1, 5, Sx); set(2, 3, Sy); set(2, 4, -Sx);
    set(3, 1, -Sz); set(3, 2, Sy); set(4, 0, Sz); set(4, 2, -Sx); set(5, 0, -Sy); set(5, 1, Sx);
    set(3, 3, Ixx); set(3, 4, Ixy); set(3, 5, Ixz);
    set(4, 3, Ixy); set(4, 4, Iyy); set(4, 5, Iyz);
    set(5, 3, Ixz); set(5, 4, Iyz); set(5, 5, Izz);
    // Apparent mass of the air moved with the body: acceleration-dependent, so on this side.
    const addMa = !frozen.body;
    const cvec = [cFx * dph, wR.cFy * dph + wL.cFy * dphL, wR.cFz * dph + wL.cFz * dphL, wR.cMx * dph + wL.cMx * dphL, wR.cMy * dph + wL.cMy * dphL, wR.cMz * dph + wL.cMz * dphL];
    b7[0] = Fx - wwSx - sumfx;
    b7[1] = Fy - wwSy - sumfy;
    b7[2] = Fz - wwSz - sumfz;
    b7[3] = Mx - gx1 - sumtx;
    b7[4] = My - gy1 - sumty;
    b7[5] = Mz - gz1 - sumtz;
    let thDD;
    if (parked) {
      // The geartrain holds the crank; the wing's load passes through it, not the rotor.
      thDD = -crank / Math.max(advanceDt || dt, 1e-3) * 0.5;
      for (let r = 0; r < 6; r++) b7[r] -= cvec[r] * thDD;
      for (let r = 0; r < 6; r++) { A7[r * 7 + 6] = 0; A7[6 * 7 + r] = 0; }
      A7[48] = 1;
      b7[6] = thDD;
    } else {
      for (let r = 0; r < 6; r++) { A7[r * 7 + 6] = cvec[r]; A7[6 * 7 + r] = cvec[r]; }
      A7[48] = Jeff;
      b7[6] = crankRhs;
    }
    if (addMa) {
      if (parked) for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) A7[r * 7 + c] += aero.Ma[r * 7 + c];
      else for (let i = 0; i < 49; i++) A7[i] += aero.Ma[i];
    }
    for (let i = 0; i < 6; i++) b[i] = b7[i];
    solveN(A7, b7, sol7, 7);
    thDD = sol7[6];
    for (let i = 0; i < 6; i++) sol[i] = sol7[i];
    thetaDD = thDD;
    // Generalised inertial force on phi with the solved accelerations (for torque and structure outputs)
    const Qin = Qrest - (cFy * sol[1] + cFz * sol[2] + cMx * sol[3] + cMy * sol[4] + cMz * sol[5]) - iPhi * dph * thDD;
    const Qtotal = Qaero + Qin;
    const dphOf = (w) => (w === 0 ? dph : dphL);
    // Each shoulder's generalised force (aero plus inertial reaction), what that wing's coupler must carry.
    const qWing = (w) => {
      const s = wingQ[w];
      return aero.Qphi[w] + s.Qrest - (s.cFy * sol[1] + s.cFz * sol[2] + s.cMx * sol[3] + s.cMy * sol[4] + s.cMz * sol[5]) - s.iPhi * dphOf(w) * thDD;
    };
    const abx = sol[0], aby = sol[1], abz = sol[2];
    // Kinematics
    out[0] = vwx;
    out[1] = vwy;
    out[2] = vwz;
    out[3] = 0.5 * (qw * wx + qy * wz - qz * wy);
    out[4] = 0.5 * (qw * wy + qz * wx - qx * wz);
    out[5] = 0.5 * (qw * wz + qx * wy - qy * wx);
    out[6] = 0.5 * (-qx * wx - qy * wy - qz * wz);
    // Absolute acceleration of O in world axes
    out[7] = R00 * abx + R01 * aby + R02 * abz;
    out[8] = R10 * abx + R11 * aby + R12 * abz;
    out[9] = R20 * abx + R21 * aby + R22 * abz;
    out[10] = sol[3];
    out[11] = sol[4];
    out[12] = sol[5];
    // A frozen body does not move, not even inside a step: the midpoint state keeps the pinned attitude, velocity and
    // rates. Round 4: letting the solved body accelerations into the midpoint closed an explicit loop through the
    // differenced apparent mass (strip.js, no Ma on a frozen body) that is unstable once the wing's apparent roll
    // inertia is large enough: the 2.0 m Larus's control-effectiveness cycles grew by 1e19 in one cycle, its gains came
    // out at 1e7-1e8 and every free-flight trim then diverged.
    if (frozen.body) for (let i = 0; i < 13; i++) out[i] = 0;
    out[13] = crank;
    out[14] = thDD;
    // Stroke servo slews amplitude; a slow lag.
    const wantAmp = Math.max(design.drive.ampMin ?? 0.05, Math.min(design.drive.ampMax ?? 1.2, cmd.amp));
    out[15] = servoA ? Math.max(-servoA.amp_rate_rad_s, Math.min(servoA.amp_rate_rad_s, (wantAmp - amp) / 0.1)) : 0;
    out[16] = rateR;
    out[17] = rateL;
    out[21] = imposeTwist ? 0 : accR;
    out[22] = imposeTwist ? 0 : accL;
    // Tail servos
    const tR = -cmd.tailPitch + cmd.tailYaw;
    const tL = -cmd.tailPitch - cmd.tailYaw;
    out[18] = servoRate(clampTail(tR), S[18], servoP, tailMR, false);
    out[19] = servoRate(clampTail(tL), S[19], servoP, tailML, false);
    // Battery draw
    const IbM = Math.max(0, dutyNow) * Im;
    // Servo current, SPEC M8: stall current x (torque / stall torque), plus published idle if any.
    const iServo = (sv, tq) => (sv.stall_A ?? 0) * Math.min(1, Math.abs(tq) / Math.max(sv.stall_Nm, 1e-6)) + (sv.idle_A ?? 0);
    const iTwR = iServo(servoT, lastTwistTorque[0]);
    const iTwL = iServo(servoT, lastTwistTorque[1]);
    const iTlR = iServo(servoP, tailMR);
    const iTlL = iServo(servoP, tailML);
    const iStroke = servoA ? (servoA.idle_A ?? 0) : 0;
    const iServos = iTwR + iTwL + iTlR + iTlL + iStroke;
    const iAvionics = design.electrical?.avionics_A_5V ?? 0.35;
    const I5 = iServos + iAvionics;
    // Round 4: HV servos may run straight off the pack (electrical.servo_source 'battery'); then only the avionics
    // load the BEC, and the servo current is battery current at pack voltage.
    const servosOnBattery = design.electrical?.servo_source === 'battery';
    const Ibec = servosOnBattery ? iAvionics : I5;
    // SPEC ruling: no regeneration credit. When the crank drives the motor (aero or inertia doing work on it), the
    // drive's braking stays in the dynamics but its energy is not returned to the battery, unless the chosen ESC's
    // own documentation states regenerative operation (design.esc.regenerative with its stated efficiency).
    const etaEsc = design.esc?.eta ?? 0.95;
    const Iesc = escBatteryCurrent(IbM, design.esc);
    const I5batt = (5.0 * Ibec) / (0.85 * Vbus) + (servosOnBattery ? iServos : 0);
    const Ibatt = Iesc + I5batt;
    out[20] = Ibatt; // coulombs per second
    let bendR = 0, bendL = 0, elbowR = 0, elbowL = 0, chordR = 0, chordL = 0;
    let QtwR = aero.Qtheta[0], QtwL = aero.Qtheta[1];
    if (record) {
      // Structure and servo loads from the solved accelerations: air force plus inertial reaction per lump.
      const elbowS = wing.segments.length > 1 ? wing.segments[0].length : wing.L;
      for (let w = 0; w < 2; w++) {
        const W = w === 0 ? R : L;
        const side = w === 0 ? 1 : -1;
        const hx = wing.hinge[0], hy = wing.hinge[1], hz = side * wing.hinge[2];
        let bend = 0, elbow = 0, chord = 0;
        // Round 5: the whole root load of the wing at its shoulder, for the pivot bearings: force (air and inertia) and
        // its moment about the hinge point; air at its own points (strip rootMoment), inertia at the lumps.
        let rFx = 0, rFy = 0, rFz = 0, rMy = aero.rootMoment[3 * w + 1], rMz = aero.rootMoment[3 * w + 2];
        for (let i = 0; i < n; i++) {
          const j = 3 * i;
          const q = 3 * (i + w * n);
          const m = lm[i];
          const rx = lr[q], ry = lr[q + 1], rz = lr[q + 2];
          const wrx = wy * rz - wz * ry, wry = wz * rx - wx * rz, wrz = wx * ry - wy * rx;
          const wwx = wy * wrz - wz * wry, wwy = wz * wrx - wx * wrz, wwz = wx * wry - wy * wrx;
          const wdx = sol[4] * rz - sol[5] * ry, wdy = sol[5] * rx - sol[3] * rz, wdz = sol[3] * ry - sol[4] * rx;
          const aix = sol[0] + wdx + wwx + la[q];
          const aiy = sol[1] + wdy + wwy + la[q + 1] + lp[q + 1] * dphOf(w) * thDD;
          const aiz = sol[2] + wdz + wwz + la[q + 2] + lp[q + 2] * dphOf(w) * thDD;
          const fix = m * (gxb - aix), fiy = m * (gyb - aiy), fiz = m * (gzb - aiz);
          const mt = st.weight[i] * ld[i] * (fix * W.b[j] + fiy * W.b[j + 1] + fiz * W.b[j + 2]);
          if (w === 0) QtwR += mt;
          else QtwL += mt;
          const armY = ry - hy, armZ = rz - hz;
          const fax = aero.fx[i + w * n], fay = aero.fy[i + w * n], faz = aero.fz[i + w * n];
          const totY = fay + fiy, totZ = faz + fiz, totX = fax + fix;
          bend += armY * totZ - armZ * totY;
          chord += totX * Math.abs(st.s[i]);
          if (st.s[i] >= elbowS) elbow += (totY * W.e[j + 2] - totZ * W.e[j + 1]) * (st.s[i] - elbowS) * side;
          const armX = rx - hx;
          rFx += totX; rFy += totY; rFz += totZ;
          rMy += armZ * fix - armX * fiz;
          rMz += armX * fiy - armY * fix;
        }
        if (w === 0) { bendR = bend; elbowR = elbow; chordR = chord; } else { bendL = bend; elbowL = elbow; chordL = chord; }
        rootLoad[3 * w] = Math.hypot(rFy, rFz);
        rootLoad[3 * w + 1] = Math.abs(rFx);
        rootLoad[3 * w + 2] = Math.hypot(rMy, rMz);
      }
    }
    if (record) {
      // Inner quantities for the step's end
      thddPrev[0] = frozen.body ? 0 : thDD;
      if (frozen.body) {
        aBody[0] = aBody[1] = aBody[2] = 0;
        wDot[0] = wDot[1] = wDot[2] = 0;
      } else {
        aBody[0] = abx; aBody[1] = aby; aBody[2] = abz;
        wDot[0] = sol[3]; wDot[1] = sol[4]; wDot[2] = sol[5];
      }
      twistAccel[0] = accR;
      twistAccel[1] = accL;
      lastTwistRate[0] = rateR;
      lastTwistRate[1] = rateL;
      // Load on the twist servo: the torque that has to oppose the air and move the wing.
      lastTwistTorque[0] = -QtwR + Jtw * twistAccel[0] + (design.wing.twist_stiffness_Nm_rad ?? 0) * S[16];
      lastTwistTorque[1] = -QtwL + Jtw * twistAccel[1] + (design.wing.twist_stiffness_Nm_rad ?? 0) * S[17];
      tailTorque[0] = tailMR;
      tailTorque[1] = tailML;
      if (advanceDt > 0 && !fcGov) {
        // ESC speed-governor integrator (volts)
        const err = wantCrank - crank;
        const Ki = (motor.govKi ?? 1.0) * KpV;
        // Conditional integration: no wind-up against the current clamp.
        if (cmd.freq > 0 || cmd.parkPhase !== null) { if (!(clamped && err > 0)) iInt = Math.max(-0.5 * design.battery.cells * 4.2, Math.min(0.5 * design.battery.cells * 4.2, iInt + advanceDt * Ki * err)); }
        else iInt = 0;
      }
      duty = dutyNow;
      const fTot = Math.hypot(abx - gxb, aby - gyb, abz - gzb);
      tel.power_w = Vbus * Ibatt;
      // For information only: what the battery would see if the ESC regenerated at its forward efficiency.
      tel.power_regen_credit_w = Vbus * ((IbM >= 0 ? IbM / etaEsc : IbM * etaEsc) + I5batt);
      tel.batt_V = Vbus - 0;
      tel.batt_I = Ibatt;
      tel.motor_I = Im;
      tel.motor_V = Vm;
      tel.duty = dutyNow;
      tel.servo_I = I5;
      tel.bec_I = Ibec;
      tel.aux_power_w = Vbus * I5batt; // servos and avionics, battery side
      tel.servo_each = Math.max(iTwR, iTwL, iTlR, iTlL);
      tel.airspeed = relU * -1;
      tel.alpha = Math.atan2(relV, -relU);
      tel.beta = Math.atan2(relW, -relU);
      tel.sf = fTot; // specific force at the body origin (where the IMU is)
      tel.loadFactorOrigin = fTot / G0;
      tel.loadFactor = Math.hypot(FaX, FaY, FaZ) / (mT * G0); // at the system mass centre
      tel.stalled = aero.stalledFraction;
      tel.rootBend = Math.max(Math.abs(bendR), Math.abs(bendL));
      tel.rootBendChord = Math.max(Math.abs(chordR), Math.abs(chordL));
      tel.elbowBend = Math.max(Math.abs(elbowR), Math.abs(elbowL));
      const qR = qWing(0), qL = qWing(1);
      tel.shoulderTorque = Math.max(Math.abs(qR), Math.abs(qL));
      // Equal to -dph Qtotal when both wings share one stroke law.
      tel.crankTorque = -(dph * qR + dphL * qL);
      tel.motorTorque = tauM;
      tel.twistTorque[0] = lastTwistTorque[0];
      tel.twistTorque[1] = lastTwistTorque[1];
      tel.tailTorque[0] = tailMR;
      tel.tailTorque[1] = tailML;
      tel.phi = sa.phi;
      tel.phiL = saL.phi;
      tel.mu = sa.mu;
      tel.phase = phase;
      tel.amp = amp;
      // Coupler: a pinned two-force member, so the shoulder moment is F c sin(mu), mu the transmission angle. No
      // floor on sin(mu): near toggle the force does grow without bound, and the design must keep mu away from it.
      // A sine stroke law has no linkage, so no coupler force is reported for it.
      tel.couplerForce = sa.c ? Math.max(Math.abs(qR) / (sa.c * sa.sinTrans), Math.abs(qL) / (saL.c * saL.sinTrans)) : 0;
      // Round 5: the drive's load channels (LOAD_CHANNELS) and from them the load on every bearing, bushing and rod end
      // of design.structure.joints at this instant. The slot channels are the coupler force's component along the
      // rocker, which a slotted rocker's stroke servo must hold.
      chv[0] = sa.c ? Math.abs(qR) / (sa.c * sa.sinTrans) : 0;
      chv[1] = saL.c ? Math.abs(qL) / (saL.c * saL.sinTrans) : 0;
      chv[2] = chv[0] * Math.abs(Math.cos(sa.mu ?? Math.PI / 2));
      chv[3] = chv[1] * Math.abs(Math.cos(saL.mu ?? Math.PI / 2));
      chv[4] = Math.abs(tel.crankTorque);
      for (let k = 0; k < 6; k++) chv[5 + k] = rootLoad[k];
      for (let k = 0; k < sup.length; k++) {
        const s = sup[k];
        let Fr = 0, Fa = 0;
        for (let m = 0; m < s.rc.length; m++) Fr += s.rk[m] * chv[s.rc[m]];
        for (let m = 0; m < s.ac.length; m++) Fa += s.ak[m] * chv[s.ac[m]];
        supLoad[3 * k] = Fr;
        supLoad[3 * k + 1] = Fa;
        supLoad[3 * k + 2] = s.X0 !== null ? Math.max(Fr, s.X0 * Fr + s.Y0 * Fa) : Fr;
      }
      tel.transmissionAngle = sa.mu;
      tel.Fx = Fx; tel.Fy = Fy; tel.Fz = Fz;
      tel.Mx = Mx; tel.My = My; tel.Mz = Mz;
      tel.aeroPower = aero.power;
      tel.residual.set(b);
    }
    return out;
  }

  function phaseCmd(phase, crank) {
    // The sync loop runs from a crank encoder, predicting with measured speed. Round 5: with the governor in the flight
    // controller the encoder is modelled (its resolution and rate): the last sample advanced at the measured speed.
    return fcGov ? fcGov.phaseAt(tEval) : phase;
  }
  function crankTarget(phase, crank) {
    if (cmd.parkPhase !== null) {
      let e = (cmd.parkPhase - phase) % TWO_PI;
      if (e < 0) e += TWO_PI;
      // Come down in speed, then creep forward to the parking phase.
      const want = e < 0.02 ? 0 : Math.min(Math.max(crank * 0.995, 2.5), TWO_PI * 1.0, e * 4 + 0.5);
      return want;
    }
    return cmd.freq * TWO_PI;
  }
  function clampTail(a) {
    const m = design.tail.deflection_max_rad ?? 0.5;
    return a > m ? m : a < -m ? -m : a;
  }
  function servoRate(want, pos, sv, loadTorque, signed = true) {
    return servoSlewRate(want, pos, sv, loadTorque, speedScale, signed);
  }
  function batteryState(chargeUsed) {
    const bt = design.battery;
    const soc = 1 - chargeUsed / (bt.capacity_Ah * 3600);
    const v = batteryOcv(bt, soc) - tel.batt_I * bt.R_int;
    return Math.max(v, 0.5 * bt.cells * 3.0);
  }

  let elapsed = 0;
  function setState(s) {
    x[0] = s.pos[0]; x[1] = s.pos[1]; x[2] = s.pos[2];
    x[3] = s.quat[0]; x[4] = s.quat[1]; x[5] = s.quat[2]; x[6] = s.quat[3];
    x[7] = s.vel[0]; x[8] = s.vel[1]; x[9] = s.vel[2];
    const om = s.omega ?? [0, 0, 0];
    x[10] = om[0]; x[11] = om[1]; x[12] = om[2];
    x[13] = s.phase ?? 0;
    x[14] = s.crank ?? 0;
    x[15] = s.amp ?? design.cruise?.amp ?? 0.4;
    x[16] = s.twistR ?? 0; x[17] = s.twistL ?? 0;
    x[18] = s.tailR ?? 0; x[19] = s.tailL ?? 0;
    x[20] = s.charge ?? 0;
    x[21] = s.twistRateR ?? 0; x[22] = s.twistRateL ?? 0;
    strip.reset();
    // Starting in flight (trim, release from the hand at speed): circulation is already established.
    if (s.primeAero !== false) strip.prime();
    aBody.fill(0);
    wDot.fill(0);
    thddPrev[0] = 0;
    epsLag = s.epsLag ?? 0.5;
    iInt = s.iInt ?? 0;
    t = s.t ?? 0;
    if (fcGov) fcGov.reset({ t, phase: x[13], crank: x[14], V: batteryState(x[20]), iInt: s.iInt ?? 0 });
    elapsed = 0;
  }

  function govTick() {
    const off = cmd.throttleOff && cmd.parkPhase === null;
    fcGov.tick(t, x[13], batteryState(x[20]), (ph, w) => crankTarget(ph, w), off, cmd.freq > 0 || cmd.parkPhase !== null);
  }
  function step() {
    if (windModel) {
      const speed = Math.max(tel.airspeed, 3);
      wind = windModel.step(t, dt, speed);
    }
    if (fcGov && fcGov.st.fresh) govTick();
    tEval = t;
    deriv(x, k1, 0, false);
    for (let i = 0; i < NS; i++) mid[i] = x[i] + 0.5 * dt * k1[i];
    tEval = t + 0.5 * dt;
    deriv(mid, k2, dt, true);
    for (let i = 0; i < NS; i++) x[i] += dt * k2[i];
    // Renormalise the quaternion
    const qn = Math.hypot(x[3], x[4], x[5], x[6]);
    x[3] /= qn; x[4] /= qn; x[5] /= qn; x[6] /= qn;
    x[13] %= TWO_PI;
    if (x[13] < 0) x[13] += TWO_PI;
    t += dt;
    elapsed += dt;
    if (fcGov && fcGov.due(t)) govTick();
    // Peaks
    if (tel.rootBend > peaks.rootBend) peaks.rootBend = tel.rootBend;
    if (tel.elbowBend > peaks.elbowBend) peaks.elbowBend = tel.elbowBend;
    if (Math.abs(tel.crankTorque) > peaks.crankTorque) peaks.crankTorque = Math.abs(tel.crankTorque);
    // Split by who does the work: the drive pushing the wing (load torque along the crank's rotation) or the wing's air
    // and inertia back-driving the crank and motor (against it).
    if (tel.crankTorque * x[14] >= 0) { if (Math.abs(tel.crankTorque) > peaks.crankDrive) peaks.crankDrive = Math.abs(tel.crankTorque); }
    else if (Math.abs(tel.crankTorque) > peaks.crankBack) peaks.crankBack = Math.abs(tel.crankTorque);
    if (Math.abs(tel.motor_I) > peaks.motorI) peaks.motorI = Math.abs(tel.motor_I);
    if (Math.abs(tel.batt_I) > peaks.battI) peaks.battI = Math.abs(tel.batt_I);
    if (tel.servo_I > peaks.servoI) peaks.servoI = tel.servo_I;
    if ((tel.bec_I ?? 0) > peaks.becI) peaks.becI = tel.bec_I;
    if ((tel.servo_each ?? 0) > peaks.servoEach) peaks.servoEach = tel.servo_each;
    if (Math.abs(tel.twistTorque[0]) > peaks.twistTorque) peaks.twistTorque = Math.abs(tel.twistTorque[0]);
    if (Math.abs(tel.twistTorque[1]) > peaks.twistTorque) peaks.twistTorque = Math.abs(tel.twistTorque[1]);
    if (tel.tailTorque[0] > peaks.tailTorque) peaks.tailTorque = tel.tailTorque[0];
    if (tel.tailTorque[1] > peaks.tailTorque) peaks.tailTorque = tel.tailTorque[1];
    if (tel.couplerForce > peaks.couplerForce) peaks.couplerForce = tel.couplerForce;
    if (tel.loadFactor > peaks.loadFactor) peaks.loadFactor = tel.loadFactor;
    if (tel.shoulderTorque > peaks.shoulderTorque) peaks.shoulderTorque = tel.shoulderTorque;
    if (Math.abs(x[14]) > peaks.crankRate) peaks.crankRate = Math.abs(x[14]);
    // Round 5: the motor current split by sign, so the drive clamp (driving) and the braking limit show apart.
    if (tel.motor_I > peaks.motorIDrive) peaks.motorIDrive = tel.motor_I;
    if (-tel.motor_I > peaks.motorIBrake) peaks.motorIBrake = -tel.motor_I;
    for (let k = 0; k < LOAD_CHANNELS.length; k++) if (chv[k] > peaks.channels[LOAD_CHANNELS[k]]) peaks.channels[LOAD_CHANNELS[k]] = chv[k];
    // Both slots load the one stroke servo through its yoke at the same instant.
    if (chv[2] + chv[3] > peaks.slotPair) peaks.slotPair = chv[2] + chv[3];
    for (let k = 0; k < sup.length; k++) {
      const p = peaks.supports[k];
      if (supLoad[3 * k] > p.Fr) p.Fr = supLoad[3 * k];
      if (supLoad[3 * k + 1] > p.Fa) p.Fa = supLoad[3 * k + 1];
      if (supLoad[3 * k + 2] > p.P0) p.P0 = supLoad[3 * k + 2];
    }
    // Running rms over 2 s windows for "continuous" current checks.
    rmsAcc.motorI2 += tel.motor_I * tel.motor_I;
    rmsAcc.battI2 += tel.batt_I * tel.batt_I;
    rmsAcc.servoI2 += tel.servo_I * tel.servo_I;
    rmsAcc.n += 1;
    rmsAcc.motorI2Win += tel.motor_I * tel.motor_I;
    rmsAcc.battI2Win += tel.batt_I * tel.batt_I;
    rmsAcc.nWin += 1;
    if (rmsAcc.nWin >= Math.round(2 / dt)) {
      rmsAcc.motorRmsMax = Math.max(rmsAcc.motorRmsMax, Math.sqrt(rmsAcc.motorI2Win / rmsAcc.nWin));
      rmsAcc.battRmsMax = Math.max(rmsAcc.battRmsMax, Math.sqrt(rmsAcc.battI2Win / rmsAcc.nWin));
      rmsAcc.motorI2Win = 0;
      rmsAcc.battI2Win = 0;
      rmsAcc.nWin = 0;
    }
  }

  function snapshot() {
    return {
      t,
      pos: [x[0], x[1], x[2]],
      quat: [x[3], x[4], x[5], x[6]],
      vel: [x[7], x[8], x[9]],
      omega: [x[10], x[11], x[12]],
      phase: x[13],
      crank: x[14],
      amp: x[15],
      twistR: x[16],
      twistL: x[17],
      tailR: x[18],
      tailL: x[19],
      charge: x[20],
    };
  }

  /** Linear momentum of body plus wing lumps, and angular momentum about the world origin, in world axes. */
  function momentum() {
    deriv(x, k1, 0, false);
    const qx = x[3], qy = x[4], qz = x[5], qw = x[6];
    const Rm = [
      [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
      [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
      [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ];
    const toW = (v) => [Rm[0][0] * v[0] + Rm[0][1] * v[1] + Rm[0][2] * v[2], Rm[1][0] * v[0] + Rm[1][1] * v[1] + Rm[1][2] * v[2], Rm[2][0] * v[0] + Rm[2][1] * v[1] + Rm[2][2] * v[2]];
    const w = [x[10], x[11], x[12]];
    const P = [0, 0, 0];
    const H = [0, 0, 0];
    const vO = [x[7], x[8], x[9]];
    const pO = [x[0], x[1], x[2]];
    const addPoint = (m, rB, vRelB) => {
      // velocity in world: vO + R (w x r + vrel)
      const wr = [w[1] * rB[2] - w[2] * rB[1], w[2] * rB[0] - w[0] * rB[2], w[0] * rB[1] - w[1] * rB[0]];
      const vb = toW([wr[0] + vRelB[0], wr[1] + vRelB[1], wr[2] + vRelB[2]]);
      const v = [vO[0] + vb[0], vO[1] + vb[1], vO[2] + vb[2]];
      const rw = toW(rB);
      const r = [pO[0] + rw[0], pO[1] + rw[1], pO[2] + rw[2]];
      for (let k = 0; k < 3; k++) P[k] += m * v[k];
      H[0] += m * (r[1] * v[2] - r[2] * v[1]);
      H[1] += m * (r[2] * v[0] - r[0] * v[2]);
      H[2] += m * (r[0] * v[1] - r[1] * v[0]);
    };
    // Body lumps (with their gyration radius as extra rotational inertia)
    for (const l of design.mass.lumps) {
      addPoint(l.mass_kg * (opts.perturb?.mass ?? 1), [l.pos[0] + (opts.perturb?.cgx ?? 0), l.pos[1], l.pos[2]], [0, 0, 0]);
      const k = l.gyration_m ?? 0;
      if (k) {
        const Iw = toW([l.mass_kg * k * k * w[0], l.mass_kg * k * k * w[1], l.mass_kg * k * k * w[2]]);
        H[0] += Iw[0]; H[1] += Iw[1]; H[2] += Iw[2];
      }
    }
    for (let i = 0; i < lumpsN; i++) {
      const q = 3 * i;
      addPoint(lm[i < n ? i : i - n], [lr[q], lr[q + 1], lr[q + 2]], [lv[q], lv[q + 1], lv[q + 2]]);
    }
    return { P, H };
  }

  /**
   * Kinetic energy of body, wing lumps and the drive (rotor and gears reflected to the crank, bird.Jcrank), for
   * conservation tests: with no air, no gravity and a free crank it is constant whatever the stroke law.
   */
  function energy() {
    deriv(x, k1, 0, false);
    const qx = x[3], qy = x[4], qz = x[5], qw = x[6];
    const R = [
      [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
      [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
      [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ];
    const w = [x[10], x[11], x[12]];
    const vO = [x[7], x[8], x[9]];
    let KE = 0;
    const addPoint = (m, rB, vRelB) => {
      const vb = [w[1] * rB[2] - w[2] * rB[1] + vRelB[0], w[2] * rB[0] - w[0] * rB[2] + vRelB[1], w[0] * rB[1] - w[1] * rB[0] + vRelB[2]];
      let v2 = 0;
      for (let i = 0; i < 3; i++) { const vi = vO[i] + R[i][0] * vb[0] + R[i][1] * vb[1] + R[i][2] * vb[2]; v2 += vi * vi; }
      KE += 0.5 * m * v2;
    };
    const km = opts.perturb?.mass ?? 1, cg = opts.perturb?.cgx ?? 0;
    for (const l of design.mass.lumps) {
      addPoint(l.mass_kg * km, [l.pos[0] + cg, l.pos[1], l.pos[2]], [0, 0, 0]);
      const k = l.gyration_m ?? 0;
      KE += 0.5 * l.mass_kg * km * k * k * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    }
    for (let i = 0; i < lumpsN; i++) {
      const q = 3 * i;
      addPoint(lm[i < n ? i : i - n], [lr[q], lr[q + 1], lr[q + 2]], [lv[q], lv[q + 1], lv[q + 2]]);
    }
    return KE + 0.5 * Jcrank * x[14] * x[14];
  }

  /** Zero every peak (scalars, the load channels and the supports) in place. */
  function resetPeaks() {
    for (const k of Object.keys(peaks)) {
      if (k === 'channels') for (const c of Object.keys(peaks.channels)) peaks.channels[c] = 0;
      else if (k === 'supports') for (const p of peaks.supports) { p.Fr = 0; p.Fa = 0; p.P0 = 0; }
      else peaks[k] = 0;
    }
  }
  return { dt, bird, cmd, x, step, setState, snapshot, momentum, energy, tel, peaks, resetPeaks, rms: rmsAcc, get t() { return t; }, get wind() { return wind; }, ix: IX, get aBody() { return aBody; }, get wDot() { return wDot; }, get thetaDD() { return thetaDD; } };
}

/** Total momentum of the rigid body plus wing lumps in world axes, for conservation tests. */
export function systemMomentum(sim) {
  const { bird } = sim;
  const { lm, st } = bird;
  const s = sim.snapshot();
  return { mass: bird.mTotal, vel: s.vel };
}
