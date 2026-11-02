// A small self-contained bird for tests. Numbers are round values for a test article, not a real design;
// nothing here is presented as a part. Real designs come from src/design and the catalogue.
export function testDesign() {
  const D = Math.PI / 180;
  return {
    name: 'test-article', atmosphere: { altitude_m: 1800, rho_kg_m3: 1.0 },
    wing: {
      hinge: [0, 0.02, 0.025], spar_x: 0, incidence: 3 * D, washout: -2 * D, camber: 0.03, camber_pos: 0.4, phi_ref: 0, airfoil: 'e387-film',
      segments: [
        { length: 0.4, chord_in: 0.25, chord_out: 0.24, xle_in: 0.075, xle_out: 0.075, gain: 0, twist_in: 0, twist_out: 0.3 },
        { length: 0.4, chord_in: 0.24, chord_out: 0.13, xle_in: 0.075, xle_out: 0.03, gain: 0.5, twist_in: 0.3, twist_out: 1 },
      ],
    },
    drive: { kind: 'crank-rocker', a: 0.008, b: 0.07, d: 0.07, ampMin: 9 * D, ampMax: 28 * D, ampNominal: 15 * D, phiMean: 5 * D, gearRatio: 30, gearEfficiency: 0.85, crankFriction_Nm: 0 },
    tail: { x_m: -0.6, y_m: 0.02, z_m: 0.005, span_each_m: 0.2, chord_root_m: 0.1, taper: 0.7, dihedral_rad: 35 * D, incidence_rad: 0, polar: 'thin-plate', AR_each: 3.5, lift_factor: 0.7, kEps: 1 },
    body: { CdA_front_m2: 0.001, CdA_top_m2: 0.02, CdA_side_m2: 0.02, volume_m3: 0.001, kMunk: 0.8, cp: [0.05, 0, 0], bottom_y_m: -0.035 },
    mass: {
      lumps: [
        { name: 'battery', mass_kg: 0.1, pos: [0.08, -0.01, 0] },
        { name: 'motor', mass_kg: 0.05, pos: [0.04, 0, 0] },
        { name: 'boom', mass_kg: 0.05, pos: [-0.25, 0, 0], gyration_m: 0.2 },
        { name: 'tail', mass_kg: 0.03, pos: [-0.6, 0.02, 0] },
      ],
      wing: { spar_kg_per_m: 0.06, skin_kg_per_m2: 0.06, point: [{ s_m: 0.6, mass_kg: 0.007, d_m: 0 }] },
    },
    motor: { kv_rad_per_v_s: 113, R: 0.19, I0: 0.7, I_cont: 9, J_kg_m2: 3e-6 },
    esc: { I_cont: 20, eta: 0.95 },
    battery: { cells: 3, capacity_Ah: 1, R_int: 0.04, I_cont: 25 },
    servos: {
      twist: { stall_Nm: 0.12, speed_rad_s: 9, tau_s: 0.02, stall_A: 0.5, idle_A: 0 },
      tail: { stall_Nm: 0.12, speed_rad_s: 9, tau_s: 0.02, stall_A: 0.5, idle_A: 0 },
      stroke: { stall_Nm: 0.12, speed_rad_s: 9, tau_s: 0.02, stall_A: 0.5, idle_A: 0, amp_rate_rad_s: 0.5 },
    },
  };
}
