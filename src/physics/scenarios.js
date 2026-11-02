// Mission scenarios M1-M5, the robustness sweep and corridor tracking. They run the 6-DOF flight model with the
// controller and measure against the SPEC limits. No limit lives anywhere but here and in SPEC.md.

import { buildBird, createFlight, quatFromEuler, eulerFromQuat } from './flight.js';
import { createController, sensorTruth } from './control.js';
import { createWind, randomGusts, drydenLowAltitude } from './wind.js';
import { makeRng } from './rng.js';
import { G0 } from './atmosphere.js';
import { makeFlapDrive, strokeAngle } from './kinematics.js';

/**
 * The stroke law as an angle table for players that cannot run the linkage: phi(phase, amplitude) from the same
 * strokeAngle() the flight model uses. phase 0..2pi in 73 points, amplitudes across the drive's range.
 */
export function strokeLawTable(design, nPhase = 73, nAmp = 9) {
  const law = design.drive.kind === 'crank-rocker' ? makeFlapDrive(design.drive) : { kind: 'sine', phiMean: design.drive.phiMean ?? 0 };
  const lo = design.drive.ampMin ?? 0.1, hi = design.drive.ampMax ?? 0.5;
  const phase = Array.from({ length: nPhase }, (_, i) => +((2 * Math.PI * i) / (nPhase - 1)).toFixed(5));
  const amplitude = Array.from({ length: nAmp }, (_, j) => +(lo + ((hi - lo) * j) / (nAmp - 1)).toFixed(5));
  const phi = amplitude.map((a) => phase.map((p) => +strokeAngle(p, a, law).phi.toFixed(5)));
  return { kind: law.kind, units: 'rad', note: 'phi[j][i] at amplitude[j], phase[i]; interpolate bilinearly. Same strokeAngle() as the flight model.', phase, amplitude, phi };
}

const DEG = Math.PI / 180;
const TWO_PI = 2 * Math.PI;

export const LIMITS = {
  M1: { release_speed_m_s: 8, release_pitch_deg: [0, 10], climb_within_s: 3, height_lost_m: 1.5 },
  M2: { endurance_min: 10, power_ratio: 1.3 },
  M3: { bank_deg: 30, altitude_band_m: 3, heading_deg: 360 },
  M4: { seeds: 100, pass_seeds: 98, roll_deg: 60, pitch_deg: 40, altitude_band_m: 10 },
  M5: { sink_m_s: 1.5, speed_m_s: 9 },
  M7: { factor_of_safety: 1.5 },
};

/** Fixed-stride recorder in the SPEC flight-log format. */
function makeLog(sim, design, scenario) {
  const cols = { t: [], pos: [], quat: [], vel: [], phase: [], amplitude: [], twist_l: [], twist_r: [], tail: [], airspeed: [], power_w: [], battery_pct: [], wind: [] };
  const bt = design.battery;
  const r4 = (v) => +v.toFixed(4);
  return {
    add() {
      const s = sim.snapshot();
      cols.t.push(r4(s.t));
      cols.pos.push(s.pos.map(r4));
      cols.quat.push(s.quat.map((v) => +v.toFixed(5)));
      cols.vel.push(s.vel.map(r4));
      cols.phase.push(r4(s.phase));
      cols.amplitude.push(r4(s.amp));
      cols.twist_l.push(r4(s.twistL));
      cols.twist_r.push(r4(s.twistR));
      cols.tail.push([r4(s.tailL), r4(s.tailR)]);
      cols.airspeed.push(r4(sim.tel.airspeed));
      cols.power_w.push(+sim.tel.power_w.toFixed(2));
      cols.battery_pct.push(+(100 * (1 - s.charge / (bt.capacity_Ah * 3600))).toFixed(2));
      cols.wind.push(sim.wind.world.map(r4));
    },
    output() {
      return { design: design.name, scenario, claim: 'simulated flight, real parts', dt: 1 / 120, stroke_law: strokeLawTable(design), columns: cols };
    },
  };
}

/** Sets up one flight in steady cruise, or from given initial conditions, with controller wired in. */
export function setupFlight(design, { seed = 1, perturb = {}, wind = null, delay = 0.02, sensorNoise = true, init = {}, groundY = -1e9, servoSpeedScale } = {}) {
  const bird = buildBird(design, { perturb: { ...perturb, servoSpeed: perturb.servoSpeed ?? servoSpeedScale } });
  const sim = createFlight(design, { bird, wind, dt: 1 / 240, groundY });
  const ctl = design.control;
  const tr = ctl.trim;
  const Vc = init.V ?? ctl.Vcruise;
  let row = tr[0];
  for (const r of tr) if (Math.abs(r.V - Vc) < Math.abs(row.V - Vc)) row = r;
  const theta = init.theta ?? row.theta;
  const chi = init.chi ?? 0;
  const q = quatFromEuler(chi, theta, init.phi ?? 0);
  const freq = init.freq ?? row.freq;
  const gam = init.gamma ?? 0;
  const vh = Vc * Math.cos(gam);
  // Start at the given airspeed in whatever wind is blowing at t = 0, so the bird is not dropped into a gust.
  const w0 = wind ? wind.step(0, 1 / 240, Vc).world.slice() : [0, 0, 0];
  sim.setState({
    pos: init.pos ?? [0, init.y ?? 0, 0],
    quat: q,
    vel: [vh * Math.cos(chi) + w0[0], Vc * Math.sin(gam) + w0[1], vh * Math.sin(chi) + w0[2]],
    phase: 0,
    crank: freq * TWO_PI,
    amp: row.amp,
    twistR: row.sym, twistL: row.sym,
    tailR: -row.tail, tailL: -row.tail,
    epsLag: 0.5,
  });
  const c = createController(design, { seed, delay, sensorNoise });
  const truth0 = sensorTruth(sim);
  c.guidance.speed = Vc;
  c.guidance.altitude = init.y ?? 0;
  c.guidance.course = chi;
  c.guidance.heading = chi;
  c.initAttitude(truth0);
  c.guidance.altitude = init.y ?? 0;
  Object.assign(sim.cmd, c.currentCommand());
  const tick = () => {
    const cmd = c.update(sensorTruth(sim), sim.t);
    Object.assign(sim.cmd, cmd);
    sim.step();
  };
  return { sim, ctl: c, tick };
}

function attitude(sim) {
  return eulerFromQuat(sim.snapshot().quat);
}

// ---- M1 hand launch: released at the speed and angle limits of the gate. ---------------------------------------
export function runM1(design, { seed = 1, perturb = {}, speed = 8, pitchDeg = 5, sensorNoise = true, delay = 0.02, log = false, servoSpeedScale } = {}) {
  // Released at the gate's speed and angle limits. "Positive climb within 3 s" is read as: the climb rate
  // averaged over the last whole second before 3 s is positive. Height lost is measured from release.
  const row = design.control.trim[0];
  const { sim, ctl, tick } = setupFlight(design, {
    seed, perturb, sensorNoise, delay, servoSpeedScale,
    init: { V: speed, theta: pitchDeg * DEG, gamma: pitchDeg * DEG, freq: row.freq, y: 0 },
  });
  ctl.guidance.mode = 'launch';
  ctl.guidance.altitude = 10;
  ctl.guidance.speed = design.control.Vcruise;
  ctl.guidance.climb = design.launch?.climb ?? 1.0;
  const L = log ? makeLog(sim, design, 'M1') : null;
  let minY = 0;
  const ys = [];
  const T = 6;
  for (let k = 0; k < T * 240; k++) {
    if (k === 4 * 240) {
      ctl.guidance.mode = 'cruise';
      ctl.guidance.climb = null;
    }
    tick();
    if (L && k % 2 === 0) L.add();
    const s = sim.snapshot();
    ys.push(s.pos[1]);
    if (k <= 3 * 240 && s.pos[1] < minY) minY = s.pos[1];
  }
  const climb3 = ys[3 * 240 - 1] - ys[2 * 240 - 1]; // metres over the second before 3 s
  let firstPositive = null;
  for (let k = 240; k < ys.length; k++) if (ys[k] - ys[k - 240] > 0) { firstPositive = k / 240; break; }
  const passed = climb3 > 0 && -minY < 1.5;
  return { passed, measured: { release_speed_m_s: speed, release_pitch_deg: pitchDeg, climb_rate_2_to_3s_m_s: climb3, first_1s_window_climbing_s: firstPositive, height_lost_m: -minY, height_at_6s_m: ys[ys.length - 1], peak_motor_A: sim.peaks.motorI, peak_batt_A: sim.peaks.battI }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- M2 endurance: level cruise for long enough to measure draw, extrapolated to empty against the real battery. --
export function runM2(design, { seed = 1, perturb = {}, sensorNoise = true, delay = 0.02, log = false, tEnd = 120, servoSpeedScale } = {}) {
  const { sim, ctl, tick } = setupFlight(design, { seed, perturb, sensorNoise, delay, servoSpeedScale, init: { y: 0 } });
  const L = log ? makeLog(sim, design, 'M2') : null;
  const N = Math.round(tEnd * 240);
  let sumP = 0, sumI = 0, n = 0, maxDev = 0;
  for (let k = 0; k < N; k++) {
    tick();
    if (L && k % 2 === 0) L.add();
    if (k > 20 * 240) {
      sumP += sim.tel.power_w;
      sumI += sim.tel.batt_I;
      n++;
      maxDev = Math.max(maxDev, Math.abs(sim.snapshot().pos[1]));
    }
  }
  const Pc = sumP / n;
  const Ic = sumI / n;
  const bt = design.battery;
  // Usable capacity: 80% of nominal, to keep the pack above its low-voltage knee.
  const usable = (bt.capacity_Ah * 3600 * 0.8) / Ic;
  const Vmid = bt.cells * 3.7 - Ic * bt.R_int;
  // Max continuous electrical power available at the shaft side: limited by motor, ESC and battery ratings.
  const Icont = Math.min(design.motor.I_cont ?? Infinity, design.esc.I_cont ?? Infinity, bt.I_cont ?? Infinity);
  const motorAvail = Icont * Vmid;
  const mean = Pc;
  const ratio = motorAvail / mean;
  const passed = usable >= 600 && ratio >= 1.3;
  return { passed, measured: { cruise_power_w: Pc, cruise_batt_A: Ic, endurance_min: usable / 60, usable_fraction: 0.8, continuous_limit_A: Icont, power_available_w: motorAvail, power_ratio: ratio, altitude_dev_m: maxDev, simulated_s: tEnd, motor_rating_rule: design.motor.I_cont_rule, extrapolated_from_s: tEnd - 20 }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- M3 coordinated turn ---------------------------------------------------------------------------------------
export function runM3(design, { seed = 1, perturb = {}, sensorNoise = true, delay = 0.02, log = false, bankDeg = 30, servoSpeedScale } = {}) {
  const { sim, ctl, tick } = setupFlight(design, { seed, perturb, sensorNoise, delay, servoSpeedScale, init: { y: 0 } });
  const L = log ? makeLog(sim, design, 'M3') : null;
  let k = 0;
  // Settle
  while (k < 6 * 240) {
    tick();
    if (L && k % 2 === 0) L.add();
    k++;
  }
  const y0 = 0;
  ctl.guidance.bank = bankDeg * DEG;
  ctl.guidance.altitude = 0;
  let turned = 0;
  let chiPrev = Math.atan2(sim.snapshot().vel[2], sim.snapshot().vel[0]);
  let maxDev = 0;
  let bankSum = 0, n = 0, maxBank = 0, slipMax = 0;
  const Tmax = 80 * 240;
  while (k < Tmax && Math.abs(turned) < TWO_PI) {
    tick();
    if (L && k % 2 === 0) L.add();
    const s = sim.snapshot();
    const chi = Math.atan2(s.vel[2], s.vel[0]);
    let d = chi - chiPrev;
    if (d > Math.PI) d -= TWO_PI;
    if (d < -Math.PI) d += TWO_PI;
    turned += d;
    chiPrev = chi;
    maxDev = Math.max(maxDev, Math.abs(s.pos[1] - y0));
    const a = attitude(sim);
    bankSum += Math.abs(a.phi);
    n++;
    maxBank = Math.max(maxBank, Math.abs(a.phi));
    slipMax = Math.max(slipMax, Math.abs(sim.tel.beta));
    k++;
  }
  const ok = Math.abs(turned) >= TWO_PI - 1e-3 && maxDev <= 3;
  return { passed: ok, measured: { heading_change_deg: turned / DEG, altitude_dev_m: maxDev, mean_bank_deg: bankSum / Math.max(n, 1) / DEG, max_bank_deg: maxBank / DEG, max_sideslip_deg: slipMax / DEG, turn_time_s: n / 240 }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- M4 rough air: one seed ------------------------------------------------------------------------------------
export function m4Wind(design, seed, { agl = 30, Hforce = null } = {}) {
  const rng = makeRng(seed * 104729 + 5);
  const dir = rng.range(0, TWO_PI);
  const low = drydenLowAltitude(agl);
  const span = design.geometry?.span_m ?? 1.5;
  const T = 60;
  const gusts = randomGusts(rng, { t0: 8, t1: T - 4, count: 4, amp: 4, H: Hforce });
  return { spec: { mean: [6 * Math.cos(dir), 0, 6 * Math.sin(dir)], uRms: 2, vRms: 2, wRms: 2 * low.ratioW, Lu: low.Lu_m, Lv: low.Lu_m, Lw: low.Lw_m, wingspan: span, gusts, agl }, H: gusts[0].H, dir, T };
}

export function runM4Seed(design, seed, { perturb = {}, sensorNoise = true, delay = 0.02, log = false, Hforce = null, servoSpeedScale } = {}) {
  const w = m4Wind(design, seed, { Hforce });
  const wind = createWind(w.spec, seed);
  const { sim, ctl, tick } = setupFlight(design, { seed, perturb, wind, sensorNoise, delay, servoSpeedScale, init: { y: 0, chi: 0, V: design.control.Vrough ?? design.control.Vcruise } });
  ctl.guidance.holdHeading = true;
  // Rough-air technique: wings-level priority, gentle heading hold, penetration speed from the design.
  ctl.guidance.maxBank = design.control.roughMaxBank ?? 15 * DEG;
  ctl.guidance.kChi = 0.6;
  ctl.guidance.speed = design.control.Vrough ?? design.control.Vcruise;
  const L = log ? makeLog(sim, design, 'M4') : null;
  const N = Math.round(w.T * 240);
  let maxRoll = 0, maxPitch = 0, maxDev = 0, minV = 1e9, stallTime = 0, longest = 0, run = 0;
  let departure = false;
  for (let k = 0; k < N; k++) {
    tick();
    if (L && k % 2 === 0) L.add();
    const a = attitude(sim);
    const s = sim.snapshot();
    if (k > 3 * 240) {
      // The first seconds are the wind arriving on a bird started in calm trim, kept out of the band check only
      // to the extent that nothing is excluded for attitude.
      maxDev = Math.max(maxDev, Math.abs(s.pos[1]));
    }
    maxRoll = Math.max(maxRoll, Math.abs(a.phi));
    maxPitch = Math.max(maxPitch, Math.abs(a.theta));
    if (sim.tel.airspeed < minV) minV = sim.tel.airspeed;
    // Stall departure: more than half the wing stalled through a whole second with falling nose or speed.
    if (sim.tel.stalled > 0.5) {
      run += 1 / 240;
      stallTime += 1 / 240;
      longest = Math.max(longest, run);
    } else run = 0;
    if (longest > 1.0) departure = true;
    if (!Number.isFinite(s.pos[1]) || Math.abs(s.pos[1]) > 200) {
      departure = true;
      break;
    }
  }
  const passed = maxRoll < 60 * DEG && maxPitch < 40 * DEG && maxDev <= 10 && !departure;
  return { seed, passed, H: w.H, measured: { max_roll_deg: maxRoll / DEG, max_pitch_deg: maxPitch / DEG, altitude_dev_m: maxDev, min_airspeed_m_s: minV, stall_time_s: stallTime, longest_stall_s: longest, departure }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- M5 belly landing ------------------------------------------------------------------------------------------
export function runM5(design, { seed = 1, perturb = {}, sensorNoise = true, delay = 0.02, log = false, startHeight = 20, servoSpeedScale } = {}) {
  const groundY = 0;
  const ctlC = design.control;
  const { sim, ctl, tick } = setupFlight(design, { seed, perturb, sensorNoise, delay, servoSpeedScale, init: { y: startHeight, V: ctlC.Vcruise } });
  const L = log ? makeLog(sim, design, 'M5') : null;
  ctl.guidance.altitude = startHeight;
  const bottom = design.body.bottom_y_m ?? -0.04;
  let t = 0;
  // Settle, then park the wings and glide at the approach speed from the glide table, flaring near the ground.
  let touchdown = null;
  let flareStart = null;
  const gl = ctlC.glide ?? [];
  const land = design.landing ?? {};
  // Approach speed: the glide-table speed of least sink with a margin over the slowest trimmed glide.
  let app = gl.length ? gl.reduce((p, q) => (q.sink < p.sink && q.V >= gl[0].V * 1.1 ? q : p), gl[gl.length - 1]) : null;
  const Vapp = land.Vapp ?? (app ? app.V : ctlC.Vmin * 1.1);
  const thApp = app ? app.theta : -0.05;
  ctl.guidance.speed = Vapp;
  while (t < 90) {
    if (t > 3) {
      ctl.guidance.flapping = false;
      ctl.guidance.mode = 'glide';
    }
    const est = ctl.hEst - groundY;
    const v = ctl.VEst;
    if (ctl.guidance.mode === 'glide') {
      // Hold approach speed with pitch: fast -> nose up.
      let pitch = thApp + (land.kV ?? 0.03) * (v - Vapp);
      // Flare: arrest the sink as the belly nears the ground.
      const flareH = land.flareHeight ?? 1.5;
      if (est < flareH) {
        if (flareStart === null) flareStart = t;
        const f = 1 - Math.max(est, 0) / flareH;
        pitch += f * (land.flareTheta ?? 10 * DEG);
      }
      ctl.guidance.pitchCmd = pitch;
    }
    tick();
    if (L && Math.round(t * 240) % 2 === 0) L.add();
    t += 1 / 240;
    const s2 = sim.snapshot();
    if (s2.pos[1] + bottom <= groundY && t > 3) {
      const speed = Math.hypot(s2.vel[0], s2.vel[1], s2.vel[2]);
      touchdown = { t, sink: -s2.vel[1], speed, pitch: attitude(sim).theta / DEG, roll: attitude(sim).phi / DEG, Vapp };
      break;
    }
  }
  const passed = !!touchdown && touchdown.sink <= 1.5 && touchdown.speed <= 9;
  return { passed, measured: touchdown ? { touchdown_sink_m_s: touchdown.sink, touchdown_speed_m_s: touchdown.speed, pitch_deg: touchdown.pitch, roll_deg: touchdown.roll, time_s: touchdown.t, from_height_m: startHeight } : { touchdown: false }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- Corridor: same controller and wind, following data/env/corridor.json ----------------------------------------
export function runCorridor(design0, corridor, { seed = 1, wind = null, log = true, speed = null, servoSpeedScale } = {}) {
  // Flown at the corridor's real elevation: world y is height above the corridor origin.
  const design = { ...design0, atmosphere: { ...design0.atmosphere, altitude_m: corridor.elevation_m_at_origin ?? design0.atmosphere.altitude_m } };
  const wp = corridor.waypoints;
  const V = speed ?? design.control.Vcruise;
  const d0 = [wp[1][0] - wp[0][0], wp[1][1] - wp[0][1], wp[1][2] - wp[0][2]];
  const chi0 = Math.atan2(d0[2], d0[0]);
  const windM = wind ?? createWind({ mean: [0, 0, 0], uRms: 0 }, seed);
  const { sim, ctl, tick } = setupFlight(design, { seed, wind: windM, servoSpeedScale, init: { pos: wp[0].slice(), y: wp[0][1], chi: chi0, V } });
  const L = log ? makeLog(sim, design, 'corridor') : null;
  let leg = 0;
  let maxCross = 0, maxVert = 0;
  let t = 0;
  const Tmax = 1.6 * corridor.length_m / Math.max(V, 3) + 60;
  let completed = false;
  while (t < Tmax) {
    const s = sim.snapshot();
    // Advance when within reach of the next waypoint or past its normal plane.
    while (leg < wp.length - 2) {
      const a = wp[leg + 1];
      const b = wp[leg + 2];
      const dir = [b[0] - a[0], b[2] - a[2]];
      const rel = [s.pos[0] - a[0], s.pos[2] - a[2]];
      if (rel[0] * dir[0] + rel[1] * dir[1] > 0 || Math.hypot(rel[0], rel[1]) < 10) leg++;
      else break;
    }
    const a = wp[leg], b = wp[leg + 1];
    const dx = b[0] - a[0], dz = b[2] - a[2];
    const lenH = Math.hypot(dx, dz) + 1e-9;
    const ex = dx / lenH, ez = dz / lenH;
    const rx = s.pos[0] - a[0], rz = s.pos[2] - a[2];
    const along = rx * ex + rz * ez;
    const cross = rx * -ez + rz * ex; // positive right of track
    const frac = Math.max(0, Math.min(1, along / lenH));
    const yTrack = a[1] + frac * (b[1] - a[1]);
    // Vector-field guidance: head back to the track, up to 45 deg.
    const chiTrack = Math.atan2(dz, dx);
    const chiCmd = chiTrack - Math.atan(cross / 15) * (Math.PI / 4) / (Math.PI / 2);
    ctl.guidance.course = chiCmd;
    ctl.guidance.altitude = yTrack;
    tick();
    if (L && Math.round(t * 240) % 2 === 0) L.add();
    t += 1 / 240;
    maxCross = Math.max(maxCross, Math.abs(cross));
    maxVert = Math.max(maxVert, Math.abs(s.pos[1] - yTrack));
    if (leg === wp.length - 2 && along >= lenH) {
      completed = true;
      break;
    }
    if (!Number.isFinite(s.pos[1]) || maxCross > 1e3) {
      completed = false;
      break;
    }
  }
  if (maxCross > 1e3) completed = false;
  return { completed, measured: { time_s: t, max_cross_track_m: maxCross, max_vertical_error_m: maxVert, leg, legs: wp.length - 1 }, peaks: { ...sim.peaks }, rms: { ...sim.rms }, log: L?.output() };
}

// ---- Sustained climb at a speed, inside the continuous current rating (the flight controller's limiter) ------------
export function runClimb(design0, { V, altitude_m = null, climbCmd = 3, seconds = 40, seed = 1 } = {}) {
  const design = altitude_m === null ? design0 : { ...design0, atmosphere: { ...design0.atmosphere, altitude_m } };
  const { sim, ctl, tick } = setupFlight(design, { seed, sensorNoise: false, init: { V, y: 0 } });
  ctl.guidance.speed = V;
  ctl.guidance.climb = climbCmd;
  ctl.guidance.altitude = 1e4;
  const N = Math.round(seconds * 240);
  let y0 = null, t0 = null, sumV = 0, sumI = 0, n = 0, Ipk = 0;
  for (let k = 0; k < N; k++) {
    tick();
    if (k === Math.round(15 * 240)) {
      y0 = sim.snapshot().pos[1];
      t0 = sim.t;
    }
    if (y0 !== null) {
      sumV += sim.tel.airspeed;
      sumI += sim.tel.batt_I;
      n++;
      Ipk = Math.max(Ipk, sim.tel.batt_I);
    }
  }
  const s = sim.snapshot();
  return { V_target: V, climb_m_s: (s.pos[1] - y0) / (sim.t - t0), mean_airspeed_m_s: sumV / n, mean_batt_A: sumI / n, current_limit_A: design.control.currentLimit_A, motor_rms_2s_max_A: sim.rms.motorRmsMax, altitude_m: design.atmosphere.altitude_m };
}

// ---- Robustness corners ------------------------------------------------------------------------------------------
export function robustnessCorners() {
  // SPEC: +/-10% aero, +/-5% mass, +/-5 mm CG, sensor noise, actuator lag from real servo speed, 20 ms delay.
  const out = [];
  for (const aero of [0.9, 1.1]) for (const mass of [0.95, 1.05]) for (const cgx of [-0.005, 0.005]) out.push({ aero, mass, cgx, label: `aero${aero} mass${mass} cg${cgx * 1000}mm` });
  return out;
}
