// International Standard Atmosphere (troposphere) with a temperature offset.
// Pressure follows the standard hydrostatic profile for the geopotential altitude, which is what
// a "pressure altitude" means; the offset then changes temperature and therefore density.
// References: ICAO Doc 7488 / ISO 2533 constants; Sutherland's law for viscosity.

export const G0 = 9.80665; // m/s^2
export const R_AIR = 287.05287; // J/(kg K)
export const GAMMA_AIR = 1.4;
export const T0_ISA = 288.15; // K
export const P0_ISA = 101325; // Pa
export const LAPSE = 0.0065; // K/m

const SUTH_MU0 = 1.716e-5; // Pa s at 273.15 K
const SUTH_T0 = 273.15;
const SUTH_S = 110.4;

/** Atmosphere at geopotential altitude h (m, 0-11000), with temperature offset dT (K). */
export function isa(h, dT = 0) {
  const Tstd = T0_ISA - LAPSE * h;
  const p = P0_ISA * Math.pow(Tstd / T0_ISA, G0 / (LAPSE * R_AIR));
  const T = Tstd + dT;
  const rho = p / (R_AIR * T);
  const mu = SUTH_MU0 * Math.pow(T / SUTH_T0, 1.5) * (SUTH_T0 + SUTH_S) / (T + SUTH_S);
  return { h, T, p, rho, a: Math.sqrt(GAMMA_AIR * R_AIR * T), mu, nu: mu / rho, dT };
}

/** Atmosphere at altitude h where the actual temperature is known to be T_C Celsius. */
export function atmosphereAt(h, T_C) {
  const dT = (T_C + 273.15) - (T0_ISA - LAPSE * h);
  return isa(h, dT);
}

/** SPEC design condition: Bow Valley, Banff National Park, 1800 m, 10 C. */
export const DESIGN_ALTITUDE_M = 1800;
export const DESIGN_TEMPERATURE_C = 10;
export function designAtmosphere() {
  return atmosphereAt(DESIGN_ALTITUDE_M, DESIGN_TEMPERATURE_C);
}

/** Density at altitude h, keeping the same temperature offset as the design condition. */
export function densityAtAltitude(h) {
  const d = designAtmosphere();
  return isa(h, d.dT).rho;
}
