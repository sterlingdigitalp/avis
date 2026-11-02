// Flight controller. It sees only simulated sensors: IMU with noise and bias, baro, airspeed, GPS. It runs at
// a fixed rate and its commands reach the actuators after a transport delay (20 ms by default). Nothing here
// reads the true state. Gains come from control effectiveness measured on the nominal bird at design time
// (design.control) so perturbed birds fly with a controller that does not know they are perturbed.

import { G0 } from './atmosphere.js';
import { makeRng } from './rng.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;

/** Noise levels as 1-sigma: typical of an ICM-42688 class IMU, a BMP388 class baro, a DLVR pitot, an M10 GPS. */
export const SENSOR_DEFAULTS = {
  gyroNoise: 0.005, // rad/s at the sample rate
  gyroBias: 0.003, // rad/s, constant per flight
  accelNoise: 0.05, // m/s^2
  accelBias: 0.05, // m/s^2
  baroNoise: 0.15, // m
  baroDriftRate: 0.005, // m/s random walk intensity
  airspeedNoise: 0.15, // m/s
  gpsPosNoise: 0.5, // m
  gpsVelNoise: 0.08, // m/s
  gpsRate: 10, // Hz
  gpsDelay: 0.1, // s
  magNoise: 0.035, // rad, compass in the GPS module
  magBias: 0.03, // rad, residual after calibration
};

class Notch {
  constructor() {
    this.x1 = this.x2 = this.y1 = this.y2 = 0;
    this.a0 = 1; this.a1 = 0; this.a2 = 0; this.b1 = 0; this.b2 = 0;
  }
  set(freq, fs, q) {
    const w = (TWO_PI * Math.max(freq, 0.3)) / fs;
    const c = Math.cos(w);
    const s = Math.sin(w);
    const alpha = s / (2 * q);
    const a0 = 1 + alpha;
    this.a0 = 1 / a0;
    this.a1 = (-2 * c) / a0;
    this.a2 = 1 / a0;
    this.b1 = (-2 * c) / a0;
    this.b2 = (1 - alpha) / a0;
  }
  step(x) {
    const y = this.a0 * x + this.a1 * this.x1 + this.a2 * this.x2 - this.b1 * this.y1 - this.b2 * this.y2;
    this.x2 = this.x1;
    this.x1 = x;
    this.y2 = this.y1;
    this.y1 = y;
    return y;
  }
}

function interpTable(tab, v) {
  // tab: array of objects with .V, sorted
  if (v <= tab[0].V) return tab[0];
  for (let i = 1; i < tab.length; i++) {
    if (v <= tab[i].V) {
      const a = tab[i - 1], b = tab[i];
      const f = (v - a.V) / (b.V - a.V);
      const out = {};
      for (const k of Object.keys(a)) out[k] = a[k] + f * (b[k] - a[k]);
      return out;
    }
  }
  return tab[tab.length - 1];
}

/**
 * Twist amplitude for a stroke (round 3, cfg.dynSchedule = { rRef_m, gain, min, max }): the trim's twist plus `gain`
 * times the change in plunge angle atan(2 pi f A r / V) at the reference radius from the trim's, so the outer wing's
 * angle of attack swing stays near the trim's as stroke speed or airspeed change. Without a schedule: the trim's twist.
 */
export function twistSchedule(cfg, tr, freq, amp, V) {
  const s = cfg.dynSchedule;
  if (!s) return tr.dyn;
  const r = s.rRef_m;
  const plunge = (f, a, v) => Math.atan((TWO_PI * Math.max(f, 0) * a * r) / Math.max(v, 3));
  const d = tr.dyn + s.gain * (plunge(freq, amp, V) - plunge(tr.freq, tr.amp, tr.V));
  return Math.max(s.min, Math.min(s.max, d));
}

/**
 * ctl config (design.control): { trim: [{V, theta, freq, tail, amp, sym, dyn}], Iz, Ix, Iy, Md, Ld, Nd, Vcruise,
 *   Vmin, freqMax, freqMin, ampMax, climbMax, sinkMax, maxBank, lead }.
 * Round 3 options: dynSchedule (twistSchedule above) and tecs = { Vfloor, Vmax, kV, wV, kGammaE, kIE, band }: total
 * energy control. Stroke (thrust) closes the total energy rate error, pitch closes the balance between height and
 * speed, and below Vfloor + band the balance moves to speed only, so the bird never trades its speed into the back
 * side of the power curve when the stroke is at its envelope.
 */
export function createController(design, opts = {}) {
  const cfg = design.control;
  const fs = opts.rate ?? 100;
  const dt = 1 / fs;
  const delay = opts.delay ?? 0.02;
  const delaySteps = Math.max(0, Math.round(delay * fs));
  const rng = makeRng((opts.seed ?? 1) * 7919 + 17);
  const noise = { ...SENSOR_DEFAULTS, ...(opts.noise ?? {}) };
  const noisy = opts.sensorNoise !== false;
  const k = noisy ? 1 : 0;
  // Bias per flight
  const bg = [rng.normal() * noise.gyroBias * k, rng.normal() * noise.gyroBias * k, rng.normal() * noise.gyroBias * k];
  const ba = [rng.normal() * noise.accelBias * k, rng.normal() * noise.accelBias * k, rng.normal() * noise.accelBias * k];
  let baroBias = rng.normal() * 0.3 * k;
  const magBias = rng.normal() * noise.magBias * k;
  let psiEst = 0;

  // Estimator state
  let up = [0, 1, 0]; // world up in body axes
  let hEst = 0, vyEst = 0;
  let chiEst = 0;
  let VasEst = cfg.Vcruise;
  let gpsTimer = 0;
  const gpsBuf = [];
  let gnd = [0, 0, 0];
  let gpsPos = [0, 0, 0];
  let chiGps = null;
  // Rate filters
  const notchQ = cfg.notchQ ?? 1.2;
  const nP = new Notch(), nQ = new Notch(), nR = new Notch(), nPhi = new Notch(), nTh = new Notch(), nA = new Notch();
  const nP2 = new Notch(), nQ2 = new Notch(), nR2 = new Notch();
  const nH = new Notch(), nV = new Notch();
  let vyF = 0, hF = 0, VF2 = cfg.Vcruise;
  let ampsF = 0;
  let iLimit = 0;
  let lastNotchF = -1;
  let pF = 0, qF = 0, rF = 0;
  let phiF = 0, thF = 0;
  let VFilt = cfg.Vcruise;
  let vdotF = 0;
  let vPrevF = null;

  // Control integrators
  let iTheta = 0;
  let iFreq = 0;
  let iPhi = 0;
  let timer = 0;
  // Command delay line
  const blank = () => ({ freq: 0, amp: cfg.ampNominal, sym: 0, diff: 0, dyn: 0, lead: cfg.lead ?? 0, tailPitch: 0, tailYaw: 0, parkPhase: null, throttleOff: false });
  const line = [];
  for (let i = 0; i <= delaySteps; i++) line.push(blank());
  let thetaCmdRate = 0;
  let diffPrev = 0;
  /** Highest stroke frequency the component ratings allow at a stroke amplitude (design.control.envelope). */
  function envelopeFreq(amp) {
    const env = cfg.envelope;
    if (!env || !env.length) return cfg.freqMax ?? Infinity;
    if (amp <= env[0].amp) return env[0].freqMax;
    for (let i = 1; i < env.length; i++) if (amp <= env[i].amp) {
      const f = (amp - env[i - 1].amp) / (env[i].amp - env[i - 1].amp);
      return env[i - 1].freqMax + f * (env[i].freqMax - env[i - 1].freqMax);
    }
    return env[env.length - 1].freqMax;
  }
  let phiCmdF = 0;
  let thCmdFilt = cfg.trim[0].theta;

  const guidance = {
    mode: 'cruise', // cruise | climb | glide | flare | launch
    altitude: 0,
    speed: cfg.Vcruise,
    course: 0,
    bank: null, // direct bank command if set
    climb: null,
    flapping: true,
    pitchCmd: null,
  };

  function sense(truth) {
    const t = truth;
    // truth: omega [3], specific force [3] (body), up [3], y, vel [3], airspeedX, phase, crank
    const gyro = [t.omega[0] + bg[0] + k * noise.gyroNoise * rng.normal(), t.omega[1] + bg[1] + k * noise.gyroNoise * rng.normal(), t.omega[2] + bg[2] + k * noise.gyroNoise * rng.normal()];
    const acc = [t.sf[0] + ba[0] + k * noise.accelNoise * rng.normal(), t.sf[1] + ba[1] + k * noise.accelNoise * rng.normal(), t.sf[2] + ba[2] + k * noise.accelNoise * rng.normal()];
    baroBias += k * noise.baroDriftRate * Math.sqrt(dt) * rng.normal();
    const baro = t.y + baroBias + k * noise.baroNoise * rng.normal();
    const pitot = t.airspeedX + k * noise.airspeedNoise * rng.normal();
    const mag = t.heading + magBias + k * noise.magNoise * rng.normal();
    const amps = t.battI + k * 0.1 * rng.normal(); // power-module current sense
    // GPS: buffered for latency
    gpsBuf.push({ t: t.t, pos: t.pos, vel: t.vel });
    gpsTimer += dt;
    let gps = null;
    if (gpsTimer >= 1 / noise.gpsRate) {
      gpsTimer -= 1 / noise.gpsRate;
      let s = gpsBuf[0];
      for (const b of gpsBuf) {
        if (b.t <= t.t - noise.gpsDelay) s = b;
      }
      while (gpsBuf.length && gpsBuf[0].t < t.t - 2 * noise.gpsDelay) gpsBuf.shift();
      gps = {
        pos: [s.pos[0] + k * noise.gpsPosNoise * rng.normal(), s.pos[1] + k * noise.gpsPosNoise * rng.normal(), s.pos[2] + k * noise.gpsPosNoise * rng.normal()],
        vel: [s.vel[0] + k * noise.gpsVelNoise * rng.normal(), s.vel[1] + k * noise.gpsVelNoise * rng.normal(), s.vel[2] + k * noise.gpsVelNoise * rng.normal()],
      };
    }
    return { gyro, acc, baro, pitot, gps, mag, amps };
  }

  function estimate(s, crankHz) {
    const [p, q, r] = s.gyro;
    // Up-vector propagation: d(up)/dt = -w x up
    let ux = up[0] + dt * -(q * up[2] - r * up[1]);
    let uy = up[1] + dt * -(r * up[0] - p * up[2]);
    let uz = up[2] + dt * -(p * up[1] - q * up[0]);
    // Accelerometer gives up after removing the centripetal term of forward flight:
    // f = w x v + G up, with v about (V, 0, 0), so w x v = (0, wz V, -wy V).
    const V = VasEst;
    const ax = s.acc[0];
    const ay = s.acc[1] - s.gyro[2] * V;
    const az = s.acc[2] + s.gyro[1] * V;
    const mag = Math.hypot(ax, ay, az);
    const gain = (cfg.attGain ?? 0.15) * dt;
    if (mag > 0.4 * G0 && mag < 1.8 * G0) {
      ux += gain * (ax / mag - ux);
      uy += gain * (ay / mag - uy);
      uz += gain * (az / mag - uz);
    }
    const un = Math.hypot(ux, uy, uz);
    up = [ux / un, uy / un, uz / un];
    // Vertical channel: complementary baro + vertical specific force, with GPS velocity when it arrives.
    const azWorld = up[0] * s.acc[0] + up[1] * s.acc[1] + up[2] * s.acc[2] - G0;
    vyEst += dt * azWorld;
    hEst += dt * vyEst;
    const eh = s.baro - hEst;
    hEst += dt * 1.5 * eh;
    vyEst += dt * 0.6 * eh;
    // Heading: gyro about world vertical, corrected by GPS course
    const yawRate = -(up[0] * p + up[1] * q + up[2] * r);
    chiEst += dt * yawRate;
    psiEst += dt * yawRate;
    {
      let e = s.mag - psiEst;
      while (e > Math.PI) e -= TWO_PI;
      while (e < -Math.PI) e += TWO_PI;
      psiEst += dt * 0.5 * e;
    }
    if (s.gps) {
      gnd = s.gps.vel;
      gpsPos = s.gps.pos;
      const gs = Math.hypot(gnd[0], gnd[2]);
      if (gs > 1.5) {
        chiGps = Math.atan2(gnd[2], gnd[0]);
        let e = chiGps - chiEst;
        while (e > Math.PI) e -= TWO_PI;
        while (e < -Math.PI) e += TWO_PI;
        chiEst += 0.1 * e;
      }
      vyEst += 0.1 * (s.gps.vel[1] - vyEst);
    }
    VFilt += dt * 8 * (s.pitot - VFilt);
    ampsF += dt * 3 * (s.amps - ampsF);
    VasEst = Math.max(VFilt, 2);
    // Flap-synchronous notch on rates and attitude
    if (Math.abs(crankHz - lastNotchF) > 0.05) {
      lastNotchF = crankHz;
      for (const n of [nP, nQ, nR, nPhi, nTh, nA, nH, nV]) n.set(crankHz, fs, notchQ);
      for (const n of [nP2, nQ2, nR2]) n.set(2 * crankHz, fs, notchQ * 1.5);
    }
    // With the wings parked there is nothing to notch out, and a notch at a very low frequency would sit on
    // the lateral modes. Filters keep running so they are warm when flapping resumes.
    const bypass = crankHz < 0.7;
    pF = nP2.step(nP.step(p));
    qF = nQ2.step(nQ.step(q));
    rF = nR2.step(nR.step(r));
    if (bypass) {
      pF = p;
      qF = q;
      rF = r;
    }
    if (lastNotchF >= 0) {
      vyF = nA.step(vyEst);
      hF = nH.step(hEst);
      VF2 = nV.step(VasEst);
    }
    const phi = Math.atan2(-up[2], up[1]);
    const th = Math.asin(Math.max(-1, Math.min(1, up[0])));
    phiF = nPhi.step(phi);
    thF = nTh.step(th);
    if (bypass) {
      phiF = phi;
      thF = th;
      vyF = vyEst;
      hF = hEst;
      VF2 = VasEst;
    }
    // Airspeed rate for energy control: derivative of the notched airspeed through a 0.4 s low-pass.
    const vNow = Math.max(VF2, 2);
    if (vPrevF === null) vPrevF = vNow;
    vdotF += (dt / 0.4) * ((vNow - vPrevF) / dt - vdotF);
    vPrevF = vNow;
    return { wx: pF, wy: qF, wz: rF, phi: phiF, theta: thF, V: vNow, h: hF, vy: vyF, amps: ampsF, vdot: vdotF };
  }

  function law(e) {
    const g = guidance;
    const c = blank();
    const tr = interpTable(cfg.trim, Math.max(g.speed, cfg.trim[0].V));
    const trV = interpTable(cfg.trim, Math.max(e.V, cfg.trim[0].V));
    // Longitudinal
    let thetaC;
    let climbC = 0;
    let steErr = null; // total energy rate error (m/s), when tecs is on
    const glideRow = cfg.glide && cfg.glide.length ? interpTable(cfg.glide, Math.max(g.speed, cfg.glide[0].V)) : null;
    if (g.mode === 'glide' || g.mode === 'flare') {
      thetaC = g.pitchCmd ?? (glideRow ? glideRow.theta : tr.theta - 0.1);
    } else if (cfg.tecs && g.mode !== 'launch') {
      // Total energy control. Pitch closes the energy balance rate: wH (climb error) against wV (speed rate error).
      // Balanced (1, 1) above Vfloor + band; speed only (0, 2) at Vfloor and below.
      const T = cfg.tecs;
      const hErr = g.altitude - e.h;
      climbC = g.climb ?? Math.max(-cfg.sinkMax, Math.min(cfg.climbMax, cfg.kH * hErr));
      const Vc = Math.max(T.Vfloor, Math.min(T.Vmax, g.speed));
      const vdC = T.kV * (Vc - e.V);
      const x = Math.max(0, Math.min(1, (e.V - T.Vfloor) / T.band));
      const wH = x, wV = 2 - x;
      const sebErr = wH * (climbC - e.vy) - wV * (e.V / G0) * (vdC - vdotF);
      thetaC = tr.theta + (cfg.kGamma * sebErr) / Math.max(e.V, 4) + iTheta;
      iTheta += (dt * cfg.kIGamma * sebErr) / Math.max(e.V, 4);
      iTheta = Math.max(-0.3, Math.min(0.3, iTheta));
      const cb = Math.cos(Math.max(-1.2, Math.min(1.2, e.phi)));
      thetaC += (cfg.dThetaPerG ?? 0) * (1 / cb - 1);
      steErr = (climbC - e.vy) + (e.V / G0) * (vdC - vdotF);
    } else {
      const hErr = g.altitude - e.h;
      climbC = g.climb ?? Math.max(-cfg.sinkMax, Math.min(cfg.climbMax, cfg.kH * hErr));
      const climbErr = climbC - e.vy;
      const speedErr = e.V - g.speed;
      // Pitch moves speed into height. The integrator absorbs trim errors slowly.
      thetaC = tr.theta + (cfg.kGamma * climbErr) / Math.max(e.V, 4) + iTheta;
      iTheta += dt * cfg.kIGamma * climbErr / Math.max(e.V, 4);
      iTheta = Math.max(-0.3, Math.min(0.3, iTheta));
      // Stall protection: if slow, nose down in proportion to the shortfall.
      const shortfall = cfg.Vmin * 1.1 - e.V;
      if (shortfall > 0) thetaC -= cfg.kStallPitch * shortfall;
      // Turn: more lift needed, n = 1/cos(bank).
      const cb = Math.cos(Math.max(-1.2, Math.min(1.2, e.phi)));
      thetaC += (cfg.dThetaPerG ?? 0) * (1 / cb - 1);
    }
    thetaC = Math.max(-40 * DEG, Math.min(40 * DEG, thetaC));
    thCmdFilt += dt * 4 * (thetaC - thCmdFilt);
    // Trim feed-forward from the commanded speed. Scheduling it on measured speed would feed the phugoid:
    // slower -> more nose-up trim -> slower.
    const tailTrim = (g.mode === 'glide' || g.mode === 'flare') && glideRow ? glideRow.tail : tr.tail;
    // Elevator: positive tailPitch brings the nose up (tail trailing edge up).
    const tp = tailTrim + cfg.kTheta * (thCmdFilt - e.theta) - cfg.kQ * e.wz;
    c.tailPitch = Math.max(-0.5, Math.min(0.5, tp));
    // Speed via flap frequency: total energy rate
    let freq = 0;
    if (g.flapping) {
      if (steErr !== null) {
        // Total energy rate error, in metres per second of climb, on the stroke.
        const kTE = cfg.tecs.kTE ?? 1;
        freq = tr.freq + kTE * cfg.kFc * steErr + climbC * cfg.kFcFf + iFreq;
        iFreq += dt * cfg.kIF * 2 * steErr;
      } else {
        const energyErr = (g.speed - e.V) * cfg.kFv + (climbC - e.vy) * cfg.kFc + climbC * cfg.kFcFf;
        freq = tr.freq + energyErr + iFreq;
        iFreq += dt * cfg.kIF * ((g.speed - e.V) + 0.5 * (climbC - e.vy));
      }
      iFreq = Math.max(-1.5, Math.min(2.5, iFreq));
      // Thrust demand beyond trim is shared between frequency and stroke amplitude: stroke speed goes as f*amp.
      // Both are bounded by the stroke envelope built from component ratings (gear, motor, servo, spar).
      const best = cfg.envelopeBest;
      const ampTop = best ? Math.max(tr.amp, best.amp) : cfg.ampMax;
      const ratio = Math.max(0.3, freq / Math.max(tr.freq, 0.5));
      if (ratio > 1) {
        const share = Math.sqrt(ratio);
        c.amp = Math.min(ampTop, cfg.ampMax, tr.amp * share);
        freq = (tr.freq * ratio * tr.amp) / c.amp;
      } else c.amp = tr.amp;
      if (g.mode === 'launch' && best) {
        // Launch: the most stroke speed the ratings allow.
        c.amp = Math.min(cfg.ampMax, best.amp);
        freq = best.freqMax;
      }
      freq = Math.max(cfg.freqMin, Math.min(envelopeFreq(c.amp), freq));
      // Battery current limit, as flight controllers apply it: back the stroke off above the continuous rating.
      const lim = cfg.currentLimit_A ?? Infinity;
      iLimit = Math.max(0, Math.min(1, iLimit + dt * 0.8 * (e.amps - 0.9 * lim) / lim));
      if (iLimit > 0) {
        freq = Math.max(cfg.freqMin, freq * (1 - 0.4 * iLimit));
        c.amp = Math.max(tr.amp * 0.8, c.amp * (1 - 0.3 * iLimit));
      }
      c.dyn = twistSchedule(cfg, tr, freq, c.amp, e.V);
      c.sym = tr.sym;
    } else {
      c.amp = cfg.ampNominal * 0.8;
      c.throttleOff = true;
      c.parkPhase = cfg.parkPhase ?? Math.PI / 2;
      c.sym = cfg.glideSym ?? 0;
      c.dyn = 0;
    }
    c.freq = freq;
    c.lead = cfg.lead ?? 0;
    // Lateral
    let phiC = g.bank;
    if (phiC === null || phiC === undefined) {
      let e1 = g.holdHeading ? g.heading - psiEst : g.course - chiEst;
      while (e1 > Math.PI) e1 -= TWO_PI;
      while (e1 < -Math.PI) e1 += TWO_PI;
      const mb = g.maxBank ?? cfg.maxBank;
      phiC = Math.max(-mb, Math.min(mb, (g.kChi ?? cfg.kChi) * e1));
    }
    const bankRate = cfg.bankRate ?? 0.35; // rad/s
    phiCmdF += Math.max(-bankRate * dt, Math.min(bankRate * dt, phiC - phiCmdF));
    phiC = phiCmdF;
    iPhi += dt * 0.3 * (phiC - e.phi);
    iPhi = Math.max(-0.1, Math.min(0.1, iPhi));
    // diff > 0 makes the right wing more nose up, so it lifts more and rolls left (negative phi).
    const diff = -(cfg.kPhi * (phiC - e.phi) + iPhi) + cfg.kP * e.wx;
    let dCmd = Math.max(-cfg.diffMax, Math.min(cfg.diffMax, diff));
    // The twist servo cannot move faster than its loaded speed: command no faster, so the loop does not wind up.
    if (Number.isFinite(cfg.diffRate_rad_s)) {
      const step = cfg.diffRate_rad_s * dt;
      dCmd = diffPrev + Math.max(-step, Math.min(step, dCmd - diffPrev));
    }
    diffPrev = dCmd;
    c.diff = dCmd;
    if (!g.flapping && cfg.glideDiffGain) c.diff *= cfg.glideDiffGain;
    // Rudder: yaw damping and coordination. tailYaw > 0 yaws the nose right. Body wy > 0 is nose left.
    // Turning right at rate chi-dot means wy is about -chi-dot cos(phi).
    const chiDot = (G0 * Math.tan(Math.max(-1, Math.min(1, phiC)))) / Math.max(e.V, 4);
    c.tailYaw = Math.max(-0.5, Math.min(0.5, cfg.kR * (e.wy + chiDot * Math.cos(e.phi))));
    return c;
  }

  /** Called from the flight loop. truth is the sensor inputs; returns the commands to apply now. */
  function update(truth, tNow) {
    timer += truth.dt;
    if (timer + 1e-9 >= dt) {
      timer -= dt;
      const s = sense(truth);
      const e = estimate(s, truth.crank / TWO_PI);
      this.estimate = e;
      line.push(law(e));
      line.shift();
    }
    return line[0];
  }

  function initAttitude(truth) {
    up = truth.up.slice();
    hEst = truth.y;
    vyEst = truth.vel[1];
    VasEst = truth.airspeedX;
    VFilt = VasEst;
    chiEst = Math.atan2(truth.vel[2], truth.vel[0]);
    psiEst = truth.heading;
    guidance.altitude = truth.y;
    thCmdFilt = Math.asin(Math.max(-1, Math.min(1, up[0])));
    phiF = 0;
    thF = thCmdFilt;
    // Pre-load the notch filters' state with the current attitude, and the command delay with trim.
    vyF = vyEst; hF = hEst; VF2 = VasEst;
    vdotF = 0;
    vPrevF = null;
    for (const [f, v] of [[nPhi, phiF], [nTh, thF], [nA, vyEst], [nH, hEst], [nV, VasEst]]) {
      f.x1 = f.x2 = f.y1 = f.y2 = v;
    }
    const tr = interpTable(cfg.trim, Math.max(guidance.speed, cfg.trim[0].V));
    for (const c of line) {
      c.freq = tr.freq; c.amp = tr.amp; c.dyn = tr.dyn; c.sym = tr.sym; c.tailPitch = tr.tail;
    }
  }

  function currentCommand() {
    return line[0];
  }

  return {
    update,
    currentCommand,
    guidance,
    initAttitude,
    estimate: null,
    get chiEst() { return chiEst; },
    get psiEst() { return psiEst; },
    get hEst() { return hEst; },
    get gpsPos() { return gpsPos; },
    get gnd() { return gnd; },
    get VEst() { return VasEst; },
    rate: fs,
  };
}

/** The truth vector a controller's sensors read, from a flight's state. Specific force includes wing motion. */
export function sensorTruth(sim) {
  const s = sim.snapshot();
  const [qx, qy, qz, qw] = s.quat;
  const R00 = 1 - 2 * (qy * qy + qz * qz), R10 = 2 * (qx * qy + qw * qz), R20 = 2 * (qx * qz - qw * qy);
  const R01 = 2 * (qx * qy - qw * qz), R11 = 1 - 2 * (qx * qx + qz * qz), R21 = 2 * (qy * qz + qw * qx);
  const R02 = 2 * (qx * qz + qw * qy), R12 = 2 * (qy * qz - qw * qx), R22 = 1 - 2 * (qx * qx + qy * qy);
  const a = sim.aBody;
  // Specific force = absolute acceleration - gravity = a + g * up (at the sensor, near the origin).
  const sf = [a[0] + G0 * R10, a[1] + G0 * R11, a[2] + G0 * R12];
  const w = sim.wind.world;
  const vr = [s.vel[0] - w[0], s.vel[1] - w[1], s.vel[2] - w[2]];
  const airspeedX = R00 * vr[0] + R10 * vr[1] + R20 * vr[2];
  return { t: s.t, dt: sim.dt, omega: s.omega, sf, up: [R10, R11, R12], y: s.pos[1], vel: s.vel, pos: s.pos, airspeedX, phase: s.phase, crank: s.crank, heading: Math.atan2(R20, R00), battI: sim.tel.batt_I };
}
