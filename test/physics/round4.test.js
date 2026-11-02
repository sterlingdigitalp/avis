import test from 'node:test';
import assert from 'node:assert/strict';
import { motorCurrentEstimate, createCurrentLimiter, glaWashout } from '../../src/physics/control.js';
import { createFlight, buildBird, quatFromEuler } from '../../src/physics/flight.js';
import { testDesign } from './fixtures.js';

const TWO_PI = 2 * Math.PI;
const DEG = Math.PI / 180;

test('motor current estimate inverts the DC motor and ESC power balance worked by hand', () => {
  // Motor 1050 rpm/V (109.956 rad/s/V), R 0.045 ohm, train 19.75, crank 2.5 Hz: E = 15.708 * 19.75 / 109.956 = 2.8214 V.
  // At I_m = 30 A: V_m = 2.8214 + 1.35 = 4.1714 V, P = 125.14 W; ESC 0.95 on a 7.4 V pack: I_batt = 17.802 A.
  const kv = (1050 * TWO_PI) / 60, R = 0.045, G = 19.75, crank = TWO_PI * 2.5;
  const E = (crank * G) / kv;
  assert.ok(Math.abs(E - 2.8214) < 1e-4, `E ${E}`);
  const battI = ((E + 30 * R) * 30) / (0.95 * 7.4);
  assert.ok(Math.abs(battI - 17.802) < 1e-3, `battI ${battI}`);
  const I = motorCurrentEstimate({ battI, battV: 7.4, crankRate: crank, G, kv, R, etaEsc: 0.95 });
  assert.ok(Math.abs(I - 30) < 1e-9, `I ${I}`);
  // At low duty the motor current is far above the battery current: here 30 A against 17.8 A.
  assert.ok(I > 1.6 * battI);
  // No resistance: I = P / E. Nothing drawn: zero. A negative (charging) battery reading counts as zero.
  assert.ok(Math.abs(motorCurrentEstimate({ battI: 10, battV: 8, crankRate: crank, G, kv, R: 0, etaEsc: 1 }) - 80 / E) < 1e-9);
  assert.equal(motorCurrentEstimate({ battI: 0, battV: 8, crankRate: crank, G, kv, R }), 0);
  assert.equal(motorCurrentEstimate({ battI: -3, battV: 8, crankRate: crank, G, kv, R }), 0);
  // Stalled crank: all input power is copper loss, I = sqrt(P / R).
  assert.ok(Math.abs(motorCurrentEstimate({ battI: 5, battV: 8, crankRate: 0, G, kv, R, etaEsc: 1 }) - Math.sqrt(40 / R)) < 1e-9);
});

test('motor current estimate reads the flight model\'s own motor current back from its battery current', () => {
  // Independent of the estimator's derivation: the flight model computes the motor current from its duty and back-EMF
  // and the battery current from duty x current / eta plus the 5 V bus. Removing the bus share, the estimate from the
  // model's battery current, voltage and crank rate must return the model's motor current whenever the drive pushes.
  const d = testDesign();
  const sim = createFlight(d, { dt: 1 / 480 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.05, 0), vel: [9, 0, 0], phase: 0, crank: TWO_PI * 2, amp: 18 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
  Object.assign(sim.cmd, { freq: 2.2, amp: 18 * DEG, dyn: 0.3, sym: 0, tailPitch: 0 });
  let checked = 0, worst = 0;
  for (let k = 0; k < 480; k++) {
    sim.step();
    const t = sim.tel;
    if (t.motor_I <= 1 || t.duty <= 0.02 || t.duty >= 1) continue;
    // The estimate sees the state at the end of the step, the telemetry the midpoint: compare at the midpoint state by
    // using the duty-cycle identity directly, V_m = duty * V_batt and I_batt_drive = duty * I_m / eta.
    const battDrive = t.batt_I - (5 * t.servo_I) / (0.85 * t.batt_V);
    const E = t.motor_V - t.motor_I * d.motor.R; // back-EMF at the midpoint, from the model's own voltage drop
    const crankMid = (E * d.motor.kv_rad_per_v_s) / d.drive.gearRatio;
    const est = motorCurrentEstimate({ battI: battDrive, battV: t.batt_V, crankRate: crankMid, G: d.drive.gearRatio, kv: d.motor.kv_rad_per_v_s, R: d.motor.R, etaEsc: d.esc.eta });
    worst = Math.max(worst, Math.abs(est - t.motor_I));
    checked++;
  }
  assert.ok(checked > 100, `checked ${checked}`);
  assert.ok(worst < 1e-6, `worst ${worst} A`);
});

test('current limiter: silent under its setpoint, PI on the rms over-current, PT1 output at its cutoff', () => {
  const dt = 0.01;
  const quiet = createCurrentLimiter({ setpoint_A: 27, tauRms_s: 0.5, kP: 0.01, kI: 0.05, attnCutoff_Hz: 1.2 });
  for (let k = 0; k < 2000; k++) quiet.step(26.9, dt);
  assert.equal(quiet.attenuation, 0);
  // The rms filter: a first-order lag on I^2 with time constant tau (discrete: 1 - (1 - dt/tau)^n).
  const L = createCurrentLimiter({ setpoint_A: 20, tauRms_s: 0.5, kP: 0, kI: 0.01, attnCutoff_Hz: 1.2, maxAttn: 0.5 });
  for (let k = 0; k < 50; k++) L.step(30, dt);
  const msWant = 900 * (1 - (1 - dt / 0.5) ** 50);
  assert.ok(Math.abs(L.rms - Math.sqrt(msWant)) < 1e-9, `rms ${L.rms}`);
  // A swinging current with the same rms reads the same: the limiter regulates rms, not the peak.
  const S = createCurrentLimiter({ setpoint_A: 100, tauRms_s: 1 });
  for (let k = 0; k < 4000; k++) S.step(30 * Math.SQRT2 * Math.sin(TWO_PI * 2.5 * k * dt), dt);
  assert.ok(Math.abs(S.rms - 30) < 0.6, `swing rms ${S.rms}`);
  // Integrator alone, step to a constant over-current of 10 A held from the start of a primed rms: the PI output ramps
  // at kI * 10 per second; the PT1 (cutoff 1.2 Hz, tau 0.1326 s) follows a ramp with that lag.
  const R = createCurrentLimiter({ setpoint_A: 20, tauRms_s: 0.5, kP: 0, kI: 0.01, attnCutoff_Hz: 1.2, maxAttn: 0.5, initial_A: 30 });
  for (let k = 0; k < 200; k++) R.step(30, dt);
  const tauF = 1 / (TWO_PI * 1.2);
  const ramp = 0.1 * (2 - tauF); // kI * over * (t - tau) for a ramp through a first-order lag, t = 2 s
  assert.ok(Math.abs(R.attenuation - ramp) < 0.004, `ramp ${R.attenuation} want ${ramp}`);
  // Saturates at maxAttn and never goes negative.
  for (let k = 0; k < 20000; k++) R.step(30, dt);
  assert.ok(Math.abs(R.attenuation - 0.5) < 1e-6);
  for (let k = 0; k < 20000; k++) R.step(0, dt);
  assert.ok(R.attenuation >= 0 && R.attenuation < 1e-6);
});

test('current limiter in a loop with a stroke-speed-squared plant settles at its setpoint', () => {
  // Motor current goes as the square of stroke speed: I = I0 (1 - a)^2. With I0 = 40 A and the setpoint 27 A the
  // integral action must settle at a = 1 - sqrt(27/40) = 0.1784, rms = 27 A, without overshooting past 30 A.
  const L = createCurrentLimiter({ setpoint_A: 27, tauRms_s: 0.5, kP: 0.005, kI: 0.02, attnCutoff_Hz: 1.2, maxAttn: 0.5 });
  const dt = 0.01;
  let a = 0, peakRms = 0, sumRms = 0, sumA = 0, n = 0;
  for (let k = 0; k < 6000; k++) {
    const I = 40 * (1 - a) ** 2 * (1 + 0.3 * Math.sin(TWO_PI * 2.5 * k * dt));
    a = L.step(I, dt);
    if (k > 300) peakRms = Math.max(peakRms, L.rms);
    // The rms estimate ripples with the stroke; the integrator settles its mean, so judge the means over 2 s.
    if (k >= 5800) { sumRms += L.rms; sumA += a; n++; }
  }
  const want = 1 - Math.sqrt(27 / 40 / (1 + 0.3 ** 2 / 2) ** 0.5);
  assert.ok(Math.abs(sumRms / n - 27) < 0.3, `settled rms ${sumRms / n}`);
  assert.ok(Math.abs(sumA / n - want) < 0.01, `attenuation ${sumA / n} want ${want}`);
  assert.ok(peakRms < 30, `peak rms after 3 s ${peakRms}`);
});

test('gust load alleviation: washout only above the needed load factor, linear, capped', () => {
  const cfg = { k: 0.1, deadband_g: 0.2, max_rad: 0.15 };
  assert.equal(glaWashout(null, 3, 0), 0);
  assert.equal(glaWashout(cfg, 1.2, 0), 0, 'inside the deadband');
  assert.equal(glaWashout(cfg, 0.4, 0), 0, 'a down gust is not alleviated');
  assert.ok(Math.abs(glaWashout(cfg, 2.2, 0) - -0.1) < 1e-12, '1 g over the deadband: k');
  assert.equal(glaWashout(cfg, 9, 0), -0.15, 'capped');
  // In a 60 deg bank the flight needs 2 g: 2.5 g is only 0.3 g past the deadband.
  assert.ok(Math.abs(glaWashout(cfg, 2.5, 60 * DEG) - -0.03) < 1e-9);
});

test('frozen body: the body does not move inside a step, so a frozen cycle ends where it started', () => {
  const d = testDesign();
  const bird = buildBird(d);
  const sim = createFlight(d, { bird, dt: 1 / 480, frozenBody: true });
  const q = quatFromEuler(0, 0.06, 0);
  sim.setState({ pos: [0, 0, 0], quat: q, vel: [9, 0, 0], phase: 0, crank: TWO_PI * 2, amp: 18 * DEG, twistR: 0.05, twistL: -0.05, tailR: 0, tailL: 0 });
  Object.assign(sim.cmd, { freq: 2, amp: 18 * DEG, dyn: 0.3, diff: 0.05, tailPitch: 0 });
  for (let k = 0; k < 240; k++) {
    sim.step();
    assert.deepEqual([sim.x[0], sim.x[1], sim.x[2]], [0, 0, 0]);
    for (let i = 0; i < 4; i++) assert.ok(Math.abs(sim.x[3 + i] - q[i]) < 1e-12);
    assert.deepEqual([sim.x[7], sim.x[8], sim.x[9], sim.x[10], sim.x[11], sim.x[12]], [9, 0, 0, 0, 0, 0]);
  }
  // The residual it reports is still the force that would accelerate the body.
  assert.ok(Number.isFinite(sim.tel.residual[3]) && Math.abs(sim.tel.residual[3]) < 50);
});

test('ESC current clamp: the drive never pushes more than its clamp, braking is untouched', () => {
  const run = (clamp) => {
    const d = testDesign();
    if (clamp) d.drive.escCurrentClamp_A = clamp;
    const sim = createFlight(d, { dt: 1 / 960 });
    sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0, 0.05, 0), vel: [9, 0, 0], phase: 0, crank: TWO_PI * 1.5, amp: 24 * DEG, twistR: 0, twistL: 0, tailR: 0, tailL: 0 });
    // A hard stroke spin-up: the governor wants far more current than the clamp.
    Object.assign(sim.cmd, { freq: 3.2, amp: 24 * DEG, dyn: 0.3, tailPitch: 0 });
    let maxI = 0, minI = 0, crank = 0;
    for (let k = 0; k < 960; k++) {
      sim.step();
      maxI = Math.max(maxI, sim.tel.motor_I);
      minI = Math.min(minI, sim.tel.motor_I);
      crank = sim.x[14];
    }
    return { maxI, minI, crank };
  };
  const free = run(null);
  const lim = 0.6 * free.maxI;
  const c = run(lim);
  assert.ok(free.maxI > lim * 1.5, `unclamped peak ${free.maxI}`);
  assert.ok(c.maxI <= lim + 1e-9, `clamped peak ${c.maxI} limit ${lim}`);
  // ... and it is reached: at the clamp the duty is set so that (d V - E) / R is exactly the limit.
  assert.ok(c.maxI > lim - 1e-6, `clamped peak ${c.maxI} should sit on the limit ${lim}`);
  // Clamped, the stroke comes up more slowly but does come up (no wind-up stall).
  assert.ok(c.crank > 0.5 * TWO_PI * 3.2, `crank ${c.crank}`);
  // The clamp is on the drive side only: the regenerative floor is the motor model's, -0.5 I_cont.
  assert.ok(c.minI >= -0.5 * testDesign().motor.I_cont - 1e-9);
});

test('torque rod in torsion: T r / J against the published metal yield through von Mises; a composite has no allowable', async () => {
  const { torsionCheck } = await import('../../src/physics/structure.js');
  // 3 mm solid rod, 0.61 N m: J = pi 3^4 / 32 = 7.952 mm^4, tau = 0.61 N m x 1.5 mm / J = 115.06 MPa.
  const rod = torsionCheck({ od_m: 0.003, kind: 'composite' }, 0.61);
  assert.ok(Math.abs(rod.stress_MPa - 115.06) < 0.01, `${rod.stress_MPa}`);
  assert.equal(rod.fos, null);
  // Ti-3Al-2.5V 6.35 x 5.537 mm, yield 725 MPa: J = pi (6.35^4 - 5.537^4) / 32 = 67.34 mm^4, tau = 28.76 MPa;
  // shear yield 725 / sqrt 3 = 418.6 MPa, FoS 14.55.
  const ti = torsionCheck({ od_m: 0.00635, id_m: 0.005537, kind: 'metal', yield_MPa: 725 }, 0.61);
  assert.ok(Math.abs(ti.J_m4 - 67.34e-12) < 0.01e-12, `${ti.J_m4}`);
  assert.ok(Math.abs(ti.stress_MPa - 28.76) < 0.01);
  assert.ok(Math.abs(ti.allowable_shear_MPa - 418.58) < 0.01);
  assert.ok(Math.abs(ti.fos - 418.58 / 28.76) < 0.01);
  // Negative torque reads the same.
  assert.equal(torsionCheck({ od_m: 0.003, kind: 'composite' }, -0.61).stress_MPa, rod.stress_MPa);
  // Only a metal with a published yield has an allowable: a metal without one, or a composite given a number, has none.
  assert.equal(torsionCheck({ od_m: 0.00635, id_m: 0.005537, kind: 'metal', yield_MPa: null }, 0.61).fos, null);
  assert.equal(torsionCheck({ od_m: 0.003, kind: 'composite', yield_MPa: 400 }, 0.61).allowable_shear_MPa, null);
});

test('limits: edge cases and defaults of the estimate, the limiter and the washout', () => {
  const kv = 100, G = 20;
  // No resistance and a slow crank: still P / E (E = 0.1 x 20 / 100 = 0.02 V, P = 1 A x 1 V).
  assert.ok(Math.abs(motorCurrentEstimate({ battI: 1, battV: 1, crankRate: 0.1, G, kv, R: 0, etaEsc: 1 }) - 50) < 1e-9);
  // No resistance and a stopped crank: nothing can be said, 0 (not infinity).
  assert.equal(motorCurrentEstimate({ battI: 1, battV: 1, crankRate: 0, G, kv, R: 0, etaEsc: 1 }), 0);
  // A dead voltage reading gives no power and no current.
  assert.equal(motorCurrentEstimate({ battI: 5, battV: 0, crankRate: 10, G, kv, R: 0.05 }), 0);
  assert.equal(motorCurrentEstimate({ battI: 5, battV: 0.5, crankRate: 0, G, kv, R: 0.05, etaEsc: 1 }), Math.sqrt(2.5 / 0.05));
  // Limiter: a fresh one reads zero; without gains it never attenuates; a sub-ampere setpoint is honoured.
  const fresh = createCurrentLimiter({ setpoint_A: 0.5, kP: 1, kI: 1 });
  assert.equal(fresh.rms, 0);
  for (let k = 0; k < 500; k++) fresh.step(0, 0.01);
  assert.equal(fresh.attenuation, 0);
  const noGain = createCurrentLimiter({ setpoint_A: 10 });
  for (let k = 0; k < 500; k++) noGain.step(40, 0.01);
  assert.equal(noGain.attenuation, 0, 'kP and kI default to zero');
  // Integral only (kP omitted): the output follows the integral ramp, no proportional step.
  const I = createCurrentLimiter({ setpoint_A: 20, kI: 0.01, initial_A: 30 });
  I.step(30, 0.01);
  const a1 = 0.01 / (0.01 + 1 / (2 * Math.PI * 1.2)); // PT1 coefficient at the default 1.2 Hz
  assert.ok(Math.abs(I.attenuation - a1 * 0.01 * 10 * 0.01) < 1e-12, `${I.attenuation}`);
  // Default cap 0.5 on the attenuation.
  const cap = createCurrentLimiter({ setpoint_A: 1, kI: 10, initial_A: 50 });
  for (let k = 0; k < 2000; k++) cap.step(50, 0.01);
  assert.ok(Math.abs(cap.attenuation - 0.5) < 1e-9);
  // Explicit zeros mean zero: no attenuation allowed, a frozen output filter, an instantaneous rms, no washout allowed.
  const z = createCurrentLimiter({ setpoint_A: 1, kI: 10, initial_A: 50, maxAttn: 0 });
  for (let k = 0; k < 100; k++) z.step(50, 0.01);
  assert.equal(z.attenuation, 0);
  const frozen = createCurrentLimiter({ setpoint_A: 1, kI: 10, initial_A: 50, attnCutoff_Hz: 0 });
  for (let k = 0; k < 100; k++) frozen.step(50, 0.01);
  assert.equal(frozen.attenuation, 0);
  const inst = createCurrentLimiter({ setpoint_A: 100, tauRms_s: 0 });
  inst.step(-7, 0.01);
  assert.equal(inst.rms, 7);
  assert.equal(glaWashout({ k: 1, max_rad: 0 }, 9, 0), -0);
  // Washout defaults: no deadband, cap 0.2 rad.
  assert.ok(Math.abs(glaWashout({ k: 0.1 }, 1.5, 0) - -0.05) < 1e-12);
  assert.equal(glaWashout({ k: 1 }, 9, 0), -0.2);
});

test('shared crank pin: the left wing follows its mirrored linkage, momentum is still conserved in vacuum', async () => {
  const { strokeAngle, strokeAngleLeft, makeFlapDrive } = await import('../../src/physics/kinematics.js');
  const d = testDesign();
  const law = makeFlapDrive(d.drive);
  const hz = d.wing.hinge[2];
  // Hand check of the mirrored pin: beta = asin(hz / d); the left crank angle is 2 beta - theta. At the phase where the
  // right crank angle equals beta the two coincide, so both wings must have the same stroke angle there.
  const beta = Math.asin(hz / law.d);
  const thetaTop = law.table[0] && (await import('../../src/physics/kinematics.js')).crankRockerLimits({ a: law.a, b: law.b, c: law.cNom, d: law.d }).thetaAtPsiMax;
  const phaseAtBeta = beta - thetaTop;
  const R = strokeAngle(phaseAtBeta, law.ampNominal, law), L = strokeAngleLeft(phaseAtBeta, law.ampNominal, law, hz);
  // (The drive interpolates its rocker and top-of-stroke angle from a 17-row table: agreement to 1e-4 rad.)
  assert.ok(Math.abs(R.phi - L.phi) < 1e-4, `${R.phi} vs ${L.phi}`);
  // Elsewhere they differ (quick return), with the same stroke extremes over a cycle.
  let maxDiff = 0, minR = 9, maxR = -9, minL = 9, maxL = -9;
  for (let i = 0; i < 720; i++) {
    const ph = (i / 720) * TWO_PI;
    const r = strokeAngle(ph, law.ampNominal, law), l = strokeAngleLeft(ph, law.ampNominal, law, hz);
    maxDiff = Math.max(maxDiff, Math.abs(r.phi - l.phi));
    minR = Math.min(minR, r.phi); maxR = Math.max(maxR, r.phi); minL = Math.min(minL, l.phi); maxL = Math.max(maxL, l.phi);
    // dphi is the derivative of phi (central difference)
    const h = 1e-6;
    const fd = (strokeAngleLeft(ph + h, law.ampNominal, law, hz).phi - strokeAngleLeft(ph - h, law.ampNominal, law, hz).phi) / (2 * h);
    assert.ok(Math.abs(fd - l.dphi) < 1e-6);
  }
  assert.ok(maxDiff > 1 * DEG, `asymmetry ${maxDiff / DEG} deg`);
  assert.ok(Math.abs(maxR - maxL) < 1e-4 && Math.abs(minR - minL) < 1e-4, 'same extremes');
  // The flight model with the shared pin: no air, no gravity, momentum conserved as with the symmetric law.
  const ds = testDesign();
  ds.drive.pinLayout = 'shared';
  const sim = createFlight(ds, { rho: 1e-9, gravity: 0, dt: 1 / 240 });
  Object.assign(sim.cmd, { freq: 2.5, amp: 0.26, dyn: 0.2, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
  sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0.3, 0.1, 0.05), vel: [3, 0.5, -0.2], omega: [0.1, -0.2, 0.3], phase: 0, crank: 2.5 * TWO_PI, amp: 0.26 });
  const m0 = sim.momentum();
  let worstP = 0, worstH = 0, rollSwing = 0;
  for (let k = 0; k < 360; k++) {
    sim.step();
    const m = sim.momentum();
    worstP = Math.max(worstP, Math.hypot(m.P[0] - m0.P[0], m.P[1] - m0.P[1], m.P[2] - m0.P[2]));
    worstH = Math.max(worstH, Math.hypot(m.H[0] - m0.H[0], m.H[1] - m0.H[1], m.H[2] - m0.H[2]));
    rollSwing = Math.max(rollSwing, Math.abs(sim.tel.phi - sim.tel.phiL));
  }
  assert.ok(worstP < 2e-3, `linear momentum drift ${worstP}`);
  assert.ok(worstH < 2e-3, `angular momentum drift ${worstH}`);
  assert.ok(rollSwing > 1 * DEG, 'the wings really do beat differently');
});

test('free crank in vacuum: body, wings and rotor conserve kinetic energy, symmetric and shared-pin stroke alike', () => {
  // No air, no gravity, a motor that draws no current (R huge, I0 0) and no crank friction: nothing does work, so the
  // kinetic energy of body, wing lumps and the reflected rotor stays put while the crank speed swings freely. This pins
  // the crank equation's per-wing terms: with the right wing's velocity ratio used for the left (the round-3 coupling
  // under a shared pin), the drift is 0.14 J here instead of 5e-5 J.
  for (const shared of [false, true]) {
    const d = testDesign();
    if (shared) d.drive.pinLayout = 'shared';
    d.motor = { ...d.motor, R: 1e9, I0: 0 };
    d.drive.crankFriction_Nm = 0;
    const sim = createFlight(d, { rho: 1e-9, gravity: 0, dt: 1 / 960 });
    Object.assign(sim.cmd, { freq: 2.5, amp: 0.3, dyn: 0, sym: 0, lead: 0, tailPitch: 0, tailYaw: 0 });
    sim.setState({ pos: [0, 0, 0], quat: quatFromEuler(0.3, 0.1, 0.05), vel: [3, 0.5, -0.2], omega: [0.4, -0.3, 0.5], phase: 0.3, crank: 5 * Math.PI, amp: 0.3 });
    const E0 = sim.energy();
    let worst = 0, cmin = Infinity, cmax = -Infinity;
    for (let k = 0; k < 960; k++) {
      sim.step();
      worst = Math.max(worst, Math.abs(sim.energy() - E0));
      cmin = Math.min(cmin, sim.x[14]); cmax = Math.max(cmax, sim.x[14]);
    }
    assert.ok(E0 > 1, `energy scale ${E0}`);
    assert.ok(cmax - cmin > 3, `the crank must really swing (${cmin}..${cmax} rad/s)`);
    assert.ok(worst < 5e-4, `${shared ? 'shared' : 'symmetric'} energy drift ${worst} J of ${E0}`);
  }
});
