// Round 8: each new formula pinned against a hand calculation.
import test from 'node:test';
import assert from 'node:assert/strict';
import { tecsWeights } from '../../src/physics/control.js';
import { LOAD_CHANNELS } from '../../src/physics/flight.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

test('TECS weights: speed only at the floor, (2 - spdWeight, spdWeight) above the band, linear between, sum 2', () => {
  const T = { Vfloor: 9.5, band: 1.5 };
  assert.deepEqual(tecsWeights(T, 9), { wH: 0, wV: 2 });
  assert.deepEqual(tecsWeights(T, 9.5), { wH: 0, wV: 2 });
  // Default spdWeight 1: the round 3-7 law, wH = x, wV = 2 - x.
  near(tecsWeights(T, 10.25).wH, 0.5, 1e-12);
  near(tecsWeights(T, 10.25).wV, 1.5, 1e-12);
  assert.deepEqual(tecsWeights(T, 20), { wH: 1, wV: 1 });
  // spdWeight 0.5: height-first above the band; halfway through the band x = 0.5 gives wH = 0.5 x 1.5.
  const H = { ...T, spdWeight: 0.5 };
  assert.deepEqual(tecsWeights(H, 11), { wH: 1.5, wV: 0.5 });
  near(tecsWeights(H, 10.25).wH, 0.75, 1e-12);
  near(tecsWeights(H, 10.25).wV, 1.25, 1e-12);
  // Clamped to [0, 2].
  assert.deepEqual(tecsWeights({ ...T, spdWeight: -1 }, 30), { wH: 2, wV: 0 });
  assert.deepEqual(tecsWeights({ ...T, spdWeight: 3 }, 30), { wH: 0, wV: 2 });
});

test('airframe load channels: the four round-8 channels follow the drive channels in this order', () => {
  assert.deepEqual(LOAD_CHANNELS.slice(11, 15), ['sf_origin', 'wdot', 'w2', 'tail_F']);
  // round 17: the per-surface tail and per-wing twist channels come after them, so no earlier index moves
  assert.deepEqual(LOAD_CHANNELS.slice(15), ['tail_F_R', 'tail_F_L', 'tail_M_R', 'tail_M_L', 'twist_R', 'twist_L']);
  assert.equal(LOAD_CHANNELS.length, 21);
});

test('airframe checks by hand: fastener group, strut, shear panel, ties and a flat lap', async () => {
  const m = await import('../../src/physics/members.js');
  // 4 fasteners on a 10 mm radius (sum r^2 400 mm^2), rows 20 mm apart, 2 in the far row; rated 2 kN shear, 4 kN tension.
  const b = m.boltGroupCheck({ n: 4, r_max_mm: 10, sum_r2_mm2: 400, lever_mm: 20, n_far: 2 }, { shear_N: 2000, tension_N: 4000 }, { V: 400, T: 2, M: 4, F: 200 });
  near(b.shear_each_N, 100 + 50, 1e-9, 'V/n + T r / sum r^2');
  near(b.tension_each_N, 50 + 100, 1e-9, 'F/n + M / (n_far lever)');
  near(b.fos, 1 / (150 / 2000 + 150 / 4000), 1e-9);
  // Strut 10 x 2 mm, 100 mm, pinned: I = 10 x 2^3 / 12 mm^4, Pcr = pi^2 E I / L^2 = 460.6 N at 70 GPa; 100 N buckles first.
  const al = { E_GPa: 70, yield_MPa: 240 };
  const s = m.strutCheck({ L_mm: 100, b_mm: 10, t_mm: 2 }, al, { F: 100 });
  near(s.euler_N, Math.PI ** 2 * 70e9 * (0.01 * 0.002 ** 3 / 12) / 0.1 ** 2, 1e-6);
  near(s.fos, s.euler_N / 100, 1e-9);
  // Bending alone: sigma = M c / I = 0.1 x 0.001 / 6.667e-12 = 15 MPa, fos 240 / 15.
  near(m.strutCheck({ L_mm: 100, b_mm: 10, t_mm: 2 }, al, { M: 0.1 }).fos, 16, 1e-6);
  // Beam-column: F 100 N with M 0.05 N m: at load factor f, 5 f + 7.5 f / (1 - 100 f / Pcr) = 240 MPa.
  const bc = m.strutCheck({ L_mm: 100, b_mm: 10, t_mm: 2 }, al, { F: 100, M: 0.05 });
  const f = bc.fos;
  near(5 * f + 7.5 * f / (1 - 100 * f / s.euler_N), 240, 1e-6);
  // Square panel 60 x 60 x 1 mm, 69 GPa: k = 9.35, tau_cr = 165.4 MPa; tau = 1000 / 60 = 16.67 MPa; shear yield 240 / sqrt 3 governs.
  const p = m.panelShearCheck({ h_mm: 60, b_mm: 60, t_mm: 1 }, { E_GPa: 69, yield_MPa: 240 }, 1000);
  near(p.tau_buckling_MPa, 9.35 * Math.PI ** 2 * 69000 / (12 * (1 - 0.33 ** 2)) / 3600, 1e-9);
  near(p.fos, (240 / Math.sqrt(3)) / (1000 / 60), 1e-9);
  near(m.tieCheck({ n: 2 }, { loop_N: 80 }, 40).fos, 4, 1e-12);
  near(m.lapCheck({ w_mm: 10, L_mm: 20 }, { shear_MPa: 9.55 }, 100).fos, 19.1, 1e-9);
  // Dispatch: each new kind through checkMembers.
  const peaks = { 'g/V': 400, 'g/T': 2, 'g/M': 4, 'g/F': 200, 's/F': 100, 'p/V': 1000, 't/F': 40, 'l/F': 100 };
  const rows = m.checkMembers([
    { name: 'g', check: 'bolts', dims: { n: 4, r_max_mm: 10, sum_r2_mm2: 400, lever_mm: 20, n_far: 2 }, fastener: { id: 'f', shear_N: 2000, tension_N: 4000 }, uses: { V: 'V', T: 'T', M: 'M', F: 'F' } },
    { name: 's', check: 'strut', dims: { L_mm: 100, b_mm: 10, t_mm: 2 }, material: al, uses: { F: 'F' } },
    { name: 'p', check: 'panel', dims: { h_mm: 60, b_mm: 60, t_mm: 1 }, material: { E_GPa: 69, yield_MPa: 240 }, uses: { V: 'V' } },
    { name: 't', check: 'tie', dims: { n: 2 }, tie: { id: 'tie', loop_N: 80 }, uses: { F: 'F' } },
    { name: 'l', check: 'lap', dims: { w_mm: 10, L_mm: 20 }, material: { shear_MPa: 9.55 }, uses: { F: 'F' } },
  ], peaks).members;
  near(rows[0].fos, b.fos, 1e-12);
  assert.equal(rows[0].material, 'f');
  near(rows[1].fos, s.fos, 1e-12);
  near(rows[2].fos, p.fos, 1e-12);
  near(rows[3].fos, 4, 1e-12);
  assert.equal(rows[3].material, 'tie');
  near(rows[4].fos, 19.1, 1e-9);
});

test('plate section and unidirectional carbon tube by hand', async () => {
  const m = await import('../../src/physics/members.js');
  // 20 x 2 mm: F / A = 400 / 40e-6 = 10 MPa; in-plane 6 M / (t b^2) = 6 / (0.002 x 0.02^2) = 7.5 MPa; out of plane 6 Mo / (b t^2) = 0.6 / (0.02 x 0.002^2) = 7.5 MPa.
  const p = m.plateSectionCheck({ b_mm: 20, t_mm: 2 }, { yield_MPa: 240 }, { F: 400, M: 1, Mo: 0.1 });
  near(p.sigma_MPa, 25, 1e-9);
  near(p.fos, 9.6, 1e-9);
  // 10 / 8 mm tube: Z = pi (d^4 - di^4) / (32 d), Zp = 2 Z, A = pi / 4 (d^2 - di^2).
  const Z = Math.PI * (0.01 ** 4 - 0.008 ** 4) / (32 * 0.01), A = Math.PI / 4 * (0.01 ** 2 - 0.008 ** 2);
  const c = m.cfrpTubeCheck({ d_mm: 10, di_mm: 8 }, { axial_MPa: 841, shear_MPa: 48.2 }, { M: 5, T: 1, V: 10 });
  near(c.sigma_MPa, 5 / Z / 1e6, 1e-9);
  near(c.tau_MPa, (1 / (2 * Z) + 2 * 10 / A) / 1e6, 1e-9);
  near(c.fos, 1 / (c.sigma_MPa / 841 + c.tau_MPa / 48.2), 1e-12);
  // A tie with a fixed extra force: 2 x 80 N over (30 + 10) N.
  near(m.tieCheck({ n: 2, extra_N: 10 }, { loop_N: 80 }, 30).fos, 4, 1e-12);
});

test('bent-sheet channel section and the beam check by hand', async () => {
  const m = await import('../../src/physics/members.js');
  // Channel 30 x 8 x 0.8 mm: web 24 mm2, flanges 6.4 mm2 each, centroid 51.2 / 36.8 = 1.3913 mm from the web's mid-plane.
  const c = m.channelSection(30, 8, 0.8);
  near(c.A_mm2, 36.8, 1e-9);
  const yc = 51.2 / 36.8;
  const Io = 30 * 0.8 ** 3 / 12 + 24 * yc ** 2 + 2 * (0.8 * 8 ** 3 / 12 + 6.4 * (4 - yc) ** 2);
  near(c.Zo_mm3, Io / (8 - yc), 1e-9);
  const Ii = 0.8 * 30 ** 3 / 12 + 2 * (8 * 0.8 ** 3 / 12 + 6.4 * 15 ** 2);
  near(c.Zi_mm3, Ii / 15.4, 1e-9);
  // sigma = F / A + M / Zi + Mo / Zo (N, N m -> N mm).
  const b = m.beamSectionCheck({ A_mm2: 36.8, Zi_mm3: 300, Zo_mm3: 30 }, { yield_MPa: 240 }, { F: 36.8, M: 0.3, Mo: 0.3 });
  near(b.sigma_MPa, 1 + 1 + 10, 1e-9);
  near(b.fos, 20, 1e-9);
});

test('hand-launch climb command: the controller setting first, then the old launch field, else 1 m/s', async () => {
  const { launchClimbOf } = await import('../../src/physics/scenarios.js');
  assert.equal(launchClimbOf({ control: { launchClimb_m_s: 3 }, launch: { climb: 2 } }), 3);
  assert.equal(launchClimbOf({ control: {}, launch: { climb: 2 } }), 2);
  assert.equal(launchClimbOf({ control: {} }), 1);
});
