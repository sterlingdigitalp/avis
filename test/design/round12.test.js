// Round 12: gear life factors at the declared design life (SPEC ruling after physics round 11) and the servo supply
// through a step-down regulator, pinned against hand calculations from the published tables and datasheet figures.
import test from 'node:test';
import assert from 'node:assert/strict';
import { lifeRow, jgmaLifeFactors, duraconLifeFactor, jgmaGearRating, lewisDuraconRating } from '../../src/physics/gears.js';
import { lifeCycles, lifeColumnOf } from '../../src/design/gearing.js';
import { regulatorServoSupply } from '../../src/design/build.js';

// JGMA 401-01 K_L and 402-01 K_HL as KHK prints them (Tables 10.2, 10.10; scratchpad research/round12-parts.json Q1):
// rows 1e4 / 1e5 / 1e6 / 1e7 and over; K_L columns HB 120-220, over HB 220, carburized or nitrided.
test('JGMA life rows: a count reads the row at or above it; 1e7 and over is 1', () => {
  assert.equal(lifeRow(1e4), 0);
  assert.equal(lifeRow(10001), 1);
  assert.equal(lifeRow(1e6), 2);
  assert.equal(lifeRow(1.0001e6), -1);
  assert.deepEqual(jgmaLifeFactors(4.458e5, 'carburized_nitrided'), { row: 1e6, KL: 1.1, KHL: 1.15 });
  assert.deepEqual(jgmaLifeFactors(9e4, 'carburized_nitrided'), { row: 1e5, KL: 1.5, KHL: 1.3 });
  assert.deepEqual(jgmaLifeFactors(9e4, 'hb120_220'), { row: 1e5, KL: 1.2, KHL: 1.3 });
  assert.deepEqual(jgmaLifeFactors(9e4, 'hb_over_220'), { row: 1e5, KL: 1.4, KHL: 1.3 });
  assert.deepEqual(jgmaLifeFactors(5000, 'hb120_220'), { row: 1e4, KL: 1.4, KHL: 1.5 });
  assert.deepEqual(jgmaLifeFactors(1.2e6), { row: '1e7 and over', KL: 1, KHL: 1 });
  assert.deepEqual(jgmaLifeFactors(NaN), { row: '1e7 and over', KL: 1, KHL: 1 });
});

test('life factors on a KHK rating: K_L on bending, K_HL squared on surface', () => {
  const mesh = { m: 1, z: 45, zMate: 45, b_mm: 10, v_m_s: 0.3, support: 'equidistant', lubricant_cSt_50C: 127, reversing: false, hardened: true, kv_row: 1 };
  const base = jgmaGearRating({ bending_Nm: 42.3, surface_Nm: 20 }, mesh);
  const life = jgmaGearRating({ bending_Nm: 42.3, surface_Nm: 20 }, { ...mesh, cycles: 4.458e5, lifeColumn: 'carburized_nitrided' });
  assert.ok(Math.abs(life.bending_Nm / base.bending_Nm - 1.1) < 1e-12);
  assert.ok(Math.abs(life.surface_Nm / base.surface_Nm - 1.15 * 1.15) < 1e-12);
});

// KHK Fig. 11.3 (Duracon, m 1) read at 1e5 and 1e6 cycles: 4.54 and 3.97 kgf/mm2, less 0.05 for the reading, over the
// catalogue's 3.5.
test('Duracon life factor from KHK Fig. 11.3, m 1', () => {
  assert.ok(Math.abs(duraconLifeFactor(5e5, 1).factor - (3.97 - 0.05) / 3.5) < 1e-12);
  assert.ok(Math.abs(duraconLifeFactor(1e5, 1).factor - (4.54 - 0.05) / 3.5) < 1e-12);
  assert.equal(duraconLifeFactor(2e6, 1).factor, 1);
  assert.equal(duraconLifeFactor(5e5, 1.5).factor, 1); // module not on the chart: no credit
  const mesh = { m: 1, z: 64, v_m_s: 0.5, mateDuracon: false, reversing: false, grease: true, temperature_C: 40 };
  const a = lewisDuraconRating({ bending_Nm: 4.07 }, mesh), b = lewisDuraconRating({ bending_Nm: 4.07 }, { ...mesh, cycles: 5e5 });
  assert.ok(Math.abs(b.bending_Nm / a.bending_Nm - 3.92 / 3.5) < 1e-12);
});

test('cycles over the design life and the K_L column', () => {
  // 200 flights of 743 s at 180 rpm: 200 * 743 * 3 = 445800.
  assert.equal(lifeCycles({ flights: 200, flight_s: 743 }, 180), 445800);
  assert.equal(lifeCycles(null, 180), null);
  assert.equal(lifeColumnOf({ specs: { material: 'SCM415', heat_treatment: 'Carburized (KHK); teeth ground' } }), 'carburized_nitrided');
  assert.equal(lifeColumnOf({ specs: { material: 'S45C', heat_treatment: 'Gear teeth induction hardened' } }), 'hb120_220');
  assert.equal(lifeColumnOf({ specs: { material: 'C3604 free-cutting brass' } }), null);
});

// Pololu D36V50F7 (#4093 specs page): 7.5 V with 4% accuracy, minimum input 8.3 V, 5 A continuous, typical efficiency
// 80-95%, dropout read near 1.38 V at 5 A. A 3S pack whose lowest terminal voltage is 9.93 V and full charge 12.6 V.
test('regulated servo supply: torque at the lowest output, current at the highest', () => {
  const reg = { id: 'r', specs: { output_v: 7.5, output_accuracy_frac: 0.04, min_input_v: 8.3, max_input_v: 50, continuous_current_a: 5, efficiency_min_frac: 0.8, dropout_v_max: 1.38 } };
  // batteryServoSupply: rested OCV at 20% (the model's curve) less I_cont * R; here a pack chosen to give known values.
  const batt = { cells: 3, I_cont: 0, R_int: 0, capacity_Ah: 2.2 };
  const s = regulatorServoSupply(reg, batt);
  assert.equal(s.performance_V, 7.2); // 7.5 * 0.96, the dropout does not bind at a 3S input
  assert.equal(s.current_V, 7.8); // 7.5 * 1.04
  assert.equal(s.eta, 0.8);
  assert.equal(s.I_cont_A, 5);
  // A 2S pack cannot feed the 8.3 V-minimum regulator.
  assert.throws(() => regulatorServoSupply(reg, { cells: 2, I_cont: 0, R_int: 0, capacity_Ah: 3 }), /below its published minimum input/);
  // Dropout binds when the input is low: a 6 V part on 2S, lowest input minus 1.41 V.
  const r6 = { id: 'r6', specs: { output_v: 6, output_accuracy_frac: 0.04, min_input_v: 6.3, max_input_v: 50, continuous_current_a: 2.7, efficiency_min_frac: 0.85, dropout_v_max: 1.41 } };
  const s6 = regulatorServoSupply(r6, { cells: 2, I_cont: 30, R_int: 0.0247, capacity_Ah: 3, cell_shift_V: -0.1 });
  assert.ok(s6.performance_V < 5.76 && Math.abs(s6.performance_V - (s6.input_V[0] - 1.41)) < 1e-3);
});

// The envelope's member ratio (round 12, opt-in allowance.member): 1.5 over the lowest member FoS at the peaks. A bonded
// sleeve of d 10 mm, L 10 mm, adhesive 10 MPa, under F 1000 N: tau = 1000 / (pi 0.01 0.01) = 3.183 MPa, FoS 3.1416.
test('member ratio for the flight envelope', async () => {
  const { membersRatio, flightRatios } = await import('../../src/design/trim.js');
  const bond = { name: 'b', check: 'bond', material: { shear_MPa: 10 }, dims: { d_mm: 10, L_mm: 10 }, sections: [{ name: 'F' }], uses: { F: 'F' } };
  const d = { structure: { joints: [], members: [bond] } };
  assert.ok(Math.abs(membersRatio(d, [{ Fr: 1000 }]) - 1.5 / Math.PI) < 1e-9);
  assert.equal(membersRatio({ structure: { joints: [], members: [bond, { name: 'u', check: 'unrated', sections: [], uses: {} }] } }, [{ Fr: 1000 }]), 2);
  assert.equal(membersRatio({ structure: {} }, []), 0);
  // Opt-in: without allowance.member the envelope ignores it; with 0.7 the ratio is divided by it.
  const st = { spar_inner: { c: 0.005, I: 1e-9, allowable_MPa: 800 }, spar_outer: { c: 0.004, I: 1e-9, allowable_MPa: 800 }, coupler: { od_m: 0.005, id_m: 0, length_m: 0.08, E_GPa: 69, allowable_MPa: 240 }, gear_stages: [{ ratio: 1, gear_allowable_Nm: 10, pinion_allowable_Nm: 10 }] };
  const des = { structure: st, servos: { twist: { stall_Nm: 1 } }, motor: { I_cont: 30 }, esc: { I_cont: 40 }, battery: { I_cont: 50 } };
  const pk = { couplerForce: 10, crankTorque: 1, rootBend: 0, elbowBend: 0, twistTorque: 0, motorRms: 0, battMean: 0, dutySat: 0, loadFactor: 1, memberRatio: 0.84 };
  assert.equal(flightRatios(des, pk, {}).ratios.members, 0);
  assert.ok(Math.abs(flightRatios(des, pk, { member: 0.7 }).ratios.members - 1.2) < 1e-12);
});

// Round 12 mutation follow-ups: boundaries and the records the ratings carry.
test('life rows at zero cycles, Duracon modules without a catalogue figure, the cycles records', () => {
  assert.equal(lifeRow(0), 0);
  assert.equal(lifeRow(0.5), 0);
  assert.equal(duraconLifeFactor(5e5, 2).factor, 1); // a curve but no catalogue stress: no credit
  assert.ok(Math.abs(duraconLifeFactor(0, 1).factor - (4.54 - 0.05) / 3.5) < 1e-12);
  const mesh = { m: 1, z: 45, zMate: 45, b_mm: 10, v_m_s: 0.3, support: 'equidistant', lubricant_cSt_50C: 127, reversing: false, hardened: true, kv_row: 1 };
  const cyc = (r) => r.conditions.find((c) => c.condition === 'cycles');
  const life = jgmaGearRating({ bending_Nm: 42.3, surface_Nm: 20 }, { ...mesh, cycles: 4.458e5, lifeColumn: 'carburized_nitrided' });
  assert.equal(cyc(life).holds, false);
  assert.equal(cyc(life).bending_factor, 1.1);
  assert.ok(Math.abs(cyc(life).surface_factor - 1.15 * 1.15) < 1e-12);
  assert.match(cyc(life).correction, /carburized_nitrided/);
  assert.match(jgmaGearRating({ bending_Nm: 1 }, { ...mesh, cycles: 4.458e5 }).conditions.find((c) => c.condition === 'cycles').correction, /hb120_220/);
  const none = jgmaGearRating({ bending_Nm: 42.3, surface_Nm: 20 }, mesh);
  assert.equal(cyc(none).holds, true);
  assert.equal(none.conditions.find((c) => c.condition === 'impact').holds, true);
  assert.equal(cyc(jgmaGearRating({ bending_Nm: 1 }, { ...mesh, cycles: 2e7, lifeColumn: 'hb120_220' })).holds, true);
  const dm = { m: 1, z: 64, v_m_s: 0.5, mateDuracon: false, reversing: false, grease: true, temperature_C: 40 };
  assert.equal(cyc(lewisDuraconRating({ bending_Nm: 4.07 }, { ...dm, cycles: 5e5 })).holds, false);
  assert.equal(cyc(lewisDuraconRating({ bending_Nm: 4.07 }, { ...dm, cycles: 2e7 })).holds, true);
  assert.equal(cyc(lewisDuraconRating({ bending_Nm: 4.07 }, dm)).holds, true);
  assert.equal(lifeCycles({ flights: 200 }, 180), null);
  assert.equal(lifeCycles({ flight_s: 743 }, 180), null);
  assert.equal(lifeColumnOf({}), null);
});

test('regulator input limits are inclusive; the rule states the accuracy', async () => {
  const { batteryServoSupply } = await import('../../src/design/build.js');
  const batt = { cells: 3, I_cont: 10, R_int: 0.02, capacity_Ah: 2.2 };
  const pack = batteryServoSupply('battery', batt);
  const reg = { id: 'r', specs: { output_v: 7.5, output_accuracy_frac: 0.04, min_input_v: pack.performance_V, max_input_v: pack.current_V, continuous_current_a: 5, efficiency_min_frac: 0.8, dropout_v_max: 1.38 } };
  const s = regulatorServoSupply(reg, batt);
  assert.match(s.rule, /\+-4%/);
  assert.throws(() => regulatorServoSupply({ ...reg, specs: { ...reg.specs, max_input_v: pack.current_V - 0.01 } }, batt), /above its published maximum input/);
});

test('member ratio: no structure, no supports, the joint offset, a rated member below FoS 1', async () => {
  const { membersRatio } = await import('../../src/design/trim.js');
  const bond = { name: 'b', check: 'bond', material: { shear_MPa: 10 }, dims: { d_mm: 10, L_mm: 10 }, sections: [{ name: 'F' }], uses: { F: 'F' } };
  assert.equal(membersRatio({}, []), 0);
  assert.equal(membersRatio({ structure: { joints: [], members: [bond] } }, null), 0);
  // One joint support first: the member's section reads the second support entry.
  const withJoint = { structure: { joints: [{ supports: [{}] }], members: [bond] } };
  assert.ok(Math.abs(membersRatio(withJoint, [{ Fr: 1e6 }, { Fr: 1000 }]) - 1.5 / Math.PI) < 1e-9);
  // F 6283.19 N: tau = 20 MPa, FoS 0.5, ratio 3 (a rated member failing keeps its own ratio, not the unrated 2).
  assert.ok(Math.abs(membersRatio({ structure: { joints: [], members: [bond] } }, [{ Fr: 2000 * Math.PI }]) - 3) < 1e-9);
});

test('landing law from the corner glide floors: VREF 1.23 x the worst, flare at tau x the approach sink', async () => {
  const { landingLawFor, slowestGlide, ROUND12 } = await import('../../src/design/round12.mjs');
  // slowestGlide on a stub trim that converges at and above 7.13 m/s: steps of 0.25 from 9.5 fail first at 7.0, then
  // bisection between 7.0 and 7.25 to 0.05 m/s: 7.125 converges, 7.0625 fails: 7.125.
  const stub = (V) => ({ converged: V >= 7.13 - 1e-9 || V >= 7.125, V, theta: 0, gamma: -0.1, tail: 0, sink: 1 });
  const d0 = { control: { trim: [{ amp: 0.66 }], glide: [{ V: 7, sink: 1.2 }, { V: 8, sink: 1.1 }, { V: 9, sink: 1.05 }, { V: 10, sink: 1.06 }] } };
  assert.equal(slowestGlide(d0, {}, { trim: stub }).V, 7.125);
  // Per-corner floors: the worst 7.4 at one corner. VREF 1.23 x 7.4 = 9.102; flare at 1.5 x the sink of the row at or
  // below 9.102 (9 m/s: 1.05) = 1.575 m; stall guard 1.1 x 7.4 = 8.14.
  const law = landingLawFor(d0, ROUND12.landing, (d, perturb) => ({ V: perturb.aero === 0.9 && perturb.mass === 1.05 && perturb.cgx > 0 ? 7.4 : 7.0, row: null }));
  assert.equal(law.Vapp_m_s, 9.102);
  assert.equal(law.flareH_m, 1.575);
  assert.equal(law.Vfloor_m_s, 8.14);
  assert.equal(law.basis.stall_reference_m_s, 7.4);
});
