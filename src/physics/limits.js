// Round 4 drive and load limits used by the flight controller (control.js): the stroke envelope lookup, the motor
// current estimate and limiter, and gust load alleviation through twist. Pure functions, no state outside a limiter.

const TWO_PI = 2 * Math.PI;

/** Linear interpolation of y over sorted x, held at the ends. */
export function interp1(xs, ys, x) {
  if (x <= xs[0]) return ys[0];
  for (let i = 1; i < xs.length; i++) if (x <= xs[i]) return ys[i - 1] + ((x - xs[i - 1]) / (xs[i] - xs[i - 1])) * (ys[i] - ys[i - 1]);
  return ys[ys.length - 1];
}

/**
 * Stroke envelope lookup: freqMax at amplitude amp, linear between the measured amplitudes and held beyond them. Rows
 * with bySpeed ([{V, freqMax}] sorted by V) are first interpolated at airspeed V, held at the slowest and fastest
 * measured speeds; rows without it, or V not given, use the row's freqMax (the minimum over speeds).
 */
export function envelopeAt(env, amp, V) {
  const at = (row) => (row.bySpeed && Number.isFinite(V) ? interp1(row.bySpeed.map((q) => q.V), row.bySpeed.map((q) => q.freqMax), V) : row.freqMax);
  return interp1(env.map((r) => r.amp), env.map(at), amp);
}

/**
 * Motor (phase-side) current from what the flight controller senses: the power module's battery current and voltage
 * and the crank encoder's rate. The ESC's input power less its losses is the motor's terminal power, V_m I_m with
 * V_m = E + I_m R and E = G * crank rate / Kv the back-EMF, so I_m is the positive root of R I^2 + E I - P = 0.
 * Kv, R, G are the published motor constants and the train's tooth ratio; etaEsc the ESC efficiency (assumed).
 * Current drawn by servos and avionics through the same sensor reads as motor current: the estimate errs high.
 */
export function motorCurrentEstimate({ battI, battV, crankRate, G, kv, R, etaEsc = 0.95 }) {
  const P = Math.max(0, battI) * Math.max(0, battV) * etaEsc;
  const E = (Math.abs(crankRate) * G) / kv;
  if (!(R > 0)) return E > 0 ? P / E : 0;
  return (-E + Math.sqrt(E * E + 4 * R * P)) / (2 * R);
}

/**
 * Round 4 motor-current limiter (lever 1), with the structure of INAV's power limiter (src/main/flight/power_limits.c:
 * a PI on the over-current whose output passes a PT1 at limit_attn_filter_cutoff, default 1.2 Hz, and attenuates the
 * throttle). Two changes for a flapping drive: it regulates the running rms of the motor-current estimate, because the
 * motor's rating is continuous and the current swings through every stroke, and its output attenuates the stroke
 * frequency (the governed drive's throttle). cfg: { setpoint_A, tauRms_s, kP (per A), kI (per A s), attnCutoff_Hz,
 * maxAttn }. step(I, dt) returns the attenuation, a fraction in [0, maxAttn].
 */
export function createCurrentLimiter(cfg) {
  const tau = cfg.tauRms_s ?? 0.5;
  const kP = cfg.kP ?? 0;
  const kI = cfg.kI ?? 0;
  const fc = cfg.attnCutoff_Hz ?? 1.2;
  const aMax = cfg.maxAttn ?? 0.5;
  let ms = (cfg.initial_A ?? 0) ** 2;
  let integ = 0;
  let out = 0;
  return {
    step(I, dt) {
      ms += Math.min(1, dt / tau) * (I * I - ms);
      const over = Math.sqrt(Math.max(ms, 0)) - cfg.setpoint_A;
      integ = Math.max(0, Math.min(aMax, integ + kI * over * dt));
      const raw = Math.min(aMax, kP * Math.max(0, over) + integ);
      // PT1 with cutoff fc: alpha = dt / (dt + 1/(2 pi fc)), as INAV's pt1FilterApply.
      const a = dt / (dt + 1 / (TWO_PI * fc));
      out += a * (raw - out);
      return out;
    },
    get rms() { return Math.sqrt(Math.max(ms, 0)); },
    get attenuation() { return out; },
  };
}

/**
 * Round 4 gust load alleviation through twist (lever 2): both hand wings wash out in proportion to the sensed normal
 * load factor above what the commanded flight needs. Sensed: the IMU's body-normal specific force through the same
 * flap-synchronous notch as the other channels. n_need = 1 / cos(bank) (from the estimated bank). cfg: { k (rad per g),
 * deadband_g, max_rad }. Returns the symmetric twist increment (rad, negative is nose down).
 */
export function glaWashout(cfg, nSensed, bank) {
  if (!cfg) return 0;
  const need = 1 / Math.max(0.3, Math.cos(Math.max(-1.2, Math.min(1.2, bank))));
  const excess = nSensed - need - (cfg.deadband_g ?? 0);
  if (excess <= 0) return 0;
  return -Math.min(cfg.max_rad ?? 0.2, cfg.k * excess);
}

