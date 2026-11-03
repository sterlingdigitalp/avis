// Round 12: the design-file fixture is v20 (test/fixtures/larus-gl-2-v20.json, the committed HEAD design), the design these hand calculations were written for; v21 has its own tests (round12).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { batteryModel, byId } from '../../src/design/catalog.js';
import { buildDesign, batteryServoSupply, applyChargeLimit, stages } from '../../src/design/build.js';
import { round5Params, calibrationMatches, ROUND5 } from '../../src/design/round5.mjs';
import { packingView } from '../../src/design/fit.js';
import { boreOf } from '../../src/design/joints.js';
import { ampGrid } from '../../src/design/trim.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
const base = () => JSON.parse(readFileSync(join(ROOT, 'test/fixtures/larus-gl-2-v20.json'), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

test('charge limit: capacity is the lower of the rested curve and the published figure; the pack presents at most the limit', () => {
  const b = batteryModel(byId(catalog, 'batt-gensace-1300-2s-45c'));
  // Model curve: 4.00 V at 80%, 4.18 V at 100%, linear between: 4.10 V is 80% + 20% x 0.10 / 0.18 = 91.11%.
  const m = applyChargeLimit(b, { V_per_cell: 4.1 });
  near(m.charge_fraction, 0.8 + (0.2 * 0.1) / 0.18, 1e-4);
  const p = applyChargeLimit(b, { V_per_cell: 4.1, published_fraction: 0.861, source: 'test' });
  assert.equal(p.charge_fraction, 0.861);
  // Servos straight on the pack see at most 2 x 4.10 = 8.20 V, the HV6160's top published row.
  near(batteryServoSupply('battery', p).current_V, 8.2, 1e-9);
  near(batteryServoSupply('battery', b).current_V, 8.4, 1e-9);
});

test('round 5 BOM: no stroke servo, a complete flight controller, the encoder and its actuator, every joint part rated', () => {
  const p = round5Params(base(), {}, catalog);
  const d = buildDesign(p, catalog);
  const roles = d.bom.parts.map((b) => b.role);
  assert.ok(!roles.includes('stroke amplitude servo'));
  assert.equal(d.servos.stroke, undefined);
  assert.equal(d.drive.ampMin, d.drive.ampMax);
  assert.ok(d.bom.parts.some((b) => b.id === 'fc-matek-f405-wmo'));
  assert.ok(d.bom.parts.some((b) => b.id === 'sensor-rls-orbis-br10') && d.bom.parts.some((b) => b.id === 'sensor-rls-orbis-actuator-8mm'));
  // The FC's own 5 V BEC (2 A) feeds the avionics, which include the encoder's published 65 mA.
  assert.equal(d.electrical.bec.part, 'fc-matek-f405-wmo');
  assert.equal(d.esc.bec_A, 2);
  near(d.electrical.avionics_A_5V, 0.415, 1e-9);
  // Joint parts: four shoulder, two crank, two per intermediate shaft, four coupler eyes.
  const n = d.bom.parts.filter((b) => /^(bearing|bushing|rod end)/.test(b.role)).reduce((a, b) => a + b.count, 0);
  assert.equal(n, 4 + 2 + 2 * 3 + 4);
  for (const j of d.structure.joints) for (const s of j.supports) assert.ok(s.part.specs.static_load_rating_kn || s.part.specs.static_surface_pressure_mpa, `${j.name} has a rated part`);
  // Every mesh is one material (the steel-on-MC901 stage is gone).
  for (const g of stages(p.parts.gears, catalog)) assert.equal(/SCM/.test(g.pinion_material), /SCM/.test(g.gear_material));
  assert.equal(d.drive.governor.location, 'flight controller');
});

test('load path: the crank shaft\'s overhung couplers and the shoulder couple, from the drive planes by hand', () => {
  const d = buildDesign(round5Params(base(), {}, catalog), catalog);
  const L = d.structure.joints_layout.planes_m;
  const crank = d.structure.joints.find((j) => j.name === 'crank shaft');
  const aft = crank.supports.find((s) => s.name === 'aft');
  const span = L.plateFwd - L.plateAft;
  // A coupler behind the aft plate loads the aft support by (L + e) / L and the forward one by e / L.
  near(aft.radial.find((t) => t.ch === 'coupler_R').k, (L.plateFwd - L.couplerR) / span, 1e-5);
  near(crank.supports[1].radial.find((t) => t.ch === 'coupler_L').k, (L.plateAft - L.couplerL) / span, 1e-5);
  // Shoulder: the root moment normal to the hinge axis is a couple over the bearing span (24 mm): 1 / 0.024 per N m.
  const sh = d.structure.joints.find((j) => j.name === 'shoulder pivot right');
  for (const s of sh.supports) near(s.radial.find((t) => t.ch === 'root_mperp_R').k, 1 / 0.024, 1e-3);
  // Root force at the hinge plane, centred between the bearings: half each.
  for (const s of sh.supports) near(s.radial.find((t) => t.ch === 'root_yz_R').k, 0.5, 1e-9);
  // Mesh force of the crank wheel per N m: 1 / (r cos 20) with r 20 mm for the MSGB1-40.
  near(aft.radial.find((t) => t.ch === 'crank_torque').k + crank.supports[1].radial.find((t) => t.ch === 'crank_torque').k, 1 / (0.02 * Math.cos((20 * Math.PI) / 180)), 1e-3);
  near(boreOf({ specs: { d1_mm: 6 } }), 0.006, 1e-12);
  near(boreOf({ specs: { dimensions_mm_d_D_B: '5x13x4' } }), 0.005, 1e-12);
});

test('calibration belongs to the wing it was fitted on, stroke kind included; one amplitude for a fixed drive', () => {
  const d = { geometry: { span_m: 2, area_m2: 0.5 }, wing: { segments: [{ twist_in: 0, twist_out: 0 }, { twist_in: 0, twist_out: 1 }] }, drive: { fixedStroke: true } };
  const cal = { wing: { span_m: 2, area_m2: 0.5, twist_weights: [[0, 0], [0, 1]], fixed_stroke: true } };
  assert.equal(calibrationMatches(cal, d), true);
  assert.equal(calibrationMatches({ wing: { ...cal.wing, twist_weights: [[0, 0.3], [0.3, 1]] } }, d), false);
  assert.equal(calibrationMatches({ wing: { ...cal.wing, fixed_stroke: false } }, d), false);
  assert.equal(calibrationMatches({ wing: { span_m: 2, area_m2: 0.5 } }, d), false);
  assert.deepEqual(ampGrid(0.5, 0.5, 5), [0.5]);
  assert.deepEqual(ampGrid(0.2, 0.6, 3), [0.2, 0.4, 0.6]);
  assert.equal(ROUND5.fixedStroke, true);
  // Round 5's view of the film's packing check (placeholder on): a massless stroke-servo lump, the design left alone.
  // Round 6 drops it by default (model.js no longer places a stroke servo without a lump): the design as it is.
  const dd = { params: {}, mass: { lumps: [{ name: 'battery', mass_kg: 0.1, pos: [0, 0, 0] }] } };
  const v = packingView(dd, { placeholder: true });
  assert.equal(v.mass.lumps.find((l) => l.name === 'stroke servo').mass_kg, 0);
  assert.equal(dd.mass.lumps.length, 1);
  assert.equal(packingView(dd), dd);
});

test('round 5 assumptions name what replaced round 4\'s and drop what no longer holds', async () => {
  const { round5Assumptions } = await import('../../src/design/round5.mjs');
  const d = buildDesign(round5Params(base(), {}, catalog), catalog);
  const old = { assumptions: ['Crank speed governor: a drive-firmware function', 'Servos run straight off the 2S pack', 'Section polar kept'] };
  const a = round5Assumptions(old, d);
  assert.ok(!a.some((x) => /^Crank speed governor|^Servos run straight off/.test(x)));
  assert.ok(a.includes('Section polar kept'));
  assert.ok(a.some((x) => /governor and the 34 A drive-current clamp run in our controller on the flight controller at 960 Hz/.test(x)));
  assert.ok(a.some((x) => /charged to 4.1 V per cell/.test(x)));
  assert.ok(a.some((x) => /fixed stroke, half-swing 38.0 deg/.test(x)));
  assert.ok(a.some((x) => /motor's own bearings carry the first pinion/.test(x)));
  // Without the governor, charge limit, fixed stroke or joints, their lines are not claimed.
  const bare = { ...d, drive: { ...d.drive, governor: undefined, fixedStroke: false }, structure: { ...d.structure, joints: undefined }, electrical: { ...d.electrical, servo_source: 'bec' } };
  const b = round5Assumptions({}, bare);
  assert.ok(!b.some((x) => /flight controller at|fixed stroke|rated bearing|charged to/.test(x)));
});

test('drive joints: support stations, surface speeds per crank rate and the static form each part carries', async () => {
  const { driveJoints } = await import('../../src/design/joints.js');
  const d = buildDesign(round5Params(base(), {}, catalog), catalog);
  const J = Object.fromEntries(d.structure.joints.map((j) => [j.name, j]));
  const L = d.structure.joints_layout;
  // Shoulder bearings 24 mm apart about the hinge; the gearbox shafts between the plates.
  assert.deepEqual(J['shoulder pivot right'].supports.map((s) => s.x_m), [+(L.hinge_x_m - 0.012).toFixed(5), +(L.hinge_x_m + 0.012).toFixed(5)]);
  assert.deepEqual(J['crank shaft'].supports.map((s) => s.x_m), [+L.planes_m.plateAft.toFixed(5), +L.planes_m.plateFwd.toFixed(5)]);
  near(J['crank shaft'].span_m, L.planes_m.plateFwd - L.planes_m.plateAft, 1e-5);
  assert.equal(J['crank shaft'].count, 2);
  assert.equal(J['coupler eye left, rocker horn'].count, 1);
  // Surface speed per crank rad/s: the crank at its own rate on an 8 mm bore; gear shaft 1 at the last stage's ratio
  // (40/18); the shoulder at the linkage's peak velocity ratio on 6 mm; the crank-pin eye at (1 + a/b) of the crank.
  near(J['crank shaft'].supports[0].surface_m_s_per_crank_rad_s, 0.004, 1e-6);
  near(J['gear shaft 1'].supports[0].surface_m_s_per_crank_rad_s, (40 / 18) * 0.004, 1e-6);
  near(J['gear shaft 2'].supports[0].surface_m_s_per_crank_rad_s, (40 / 18) ** 2 * 0.004, 1e-6);
  near(J['shoulder pivot left'].supports[1].surface_m_s_per_crank_rad_s, L.max_stroke_velocity_ratio * 0.003, 1e-6);
  near(J['coupler eye right, crank pin'].supports[0].surface_m_s_per_crank_rad_s, (1 + d.drive.a / d.drive.b) * 0.003, 1e-6);
  near(J['coupler eye right, rocker horn'].supports[0].surface_m_s_per_crank_rad_s, (L.max_stroke_velocity_ratio + d.drive.a / d.drive.b) * 0.003, 1e-5);
  // Axial: the wing's force along the hinge, in full on each shoulder bearing; no axial term on spur-gear shafts.
  assert.deepEqual(J['shoulder pivot left'].supports[0].axial.map((t) => [t.ch, t.k]), [['root_x_L', 1]]);
  assert.equal(J['gear shaft 1'].supports[0].axial.length, 0);
  // Static forms: bushings none; the SKF deep groove pair carries SKF's 0.6 / 0.5.
  assert.equal(J['crank shaft'].supports[0].X0, null);
  assert.deepEqual([J['gear shaft 3'].supports[0].X0, J['gear shaft 3'].supports[0].Y0], [0.6, 0.5]);
  assert.equal(J['crank shaft'].supports[0].part.specs.static_surface_pressure_mpa, 80);
  assert.equal(J['crank shaft'].supports[0].part.specs.mass_note, undefined);
  // The motor's own bearings: the first mesh per crank N m, 1 / (r cos 20) / (product of the later ratios).
  const later = (64 / 20) * (40 / 18) * (40 / 18);
  near(d.structure.joints_unchecked[0].force_per_crank_Nm, 1 / (later * 0.016 * Math.cos((20 * Math.PI) / 180)), 1e-3);
  // A radial needle bearing (a test entry, not a catalogue part) carries X0 1, Y0 0; a rolling part with its own
  // factors keeps them.
  const extra = [
    { id: 'test-needle', category: 'bearing', name: 'test drawn cup', specs: { kind: 'drawn cup needle roller', static_load_rating_kn: 2, dimensions_mm_d_D_B: '8x12x10' } },
    { id: 'test-rolling', category: 'bearing', name: 'test angular contact', specs: { static_load_rating_kn: 2, X0: 0.5, Y0: 0.9, dimensions_mm_d_D_B: '8x22x7' } },
    { id: 'test-rolling-bare', category: 'bearing', name: 'test angular contact', specs: { static_load_rating_kn: 2, dimensions_mm_d_D_B: '8x22x7' } },
  ];
  const cat2 = [...catalog, ...extra];
  const jj = driveJoints(d, cat2, { ...d.params.joints, crankShaft: 'test-needle', shafts: ['test-rolling', 'test-rolling-bare', 'bearing-skf-619-5-2z'], shoulderSpan_mm: 30 });
  const K = Object.fromEntries(jj.joints.map((j) => [j.name, j]));
  assert.deepEqual([K['crank shaft'].supports[0].X0, K['crank shaft'].supports[0].Y0], [1, 0]);
  assert.deepEqual([K['gear shaft 1'].supports[0].X0, K['gear shaft 1'].supports[0].Y0], [0.5, 0.9]);
  assert.equal(K['gear shaft 2'].supports[0].X0, null);
  near(jj.layout.shoulder_span_m, 0.03, 1e-12);
  near(K['shoulder pivot right'].supports[0].radial.find((t) => t.ch === 'root_mperp_R').k, 1 / 0.03, 1e-4);
});

test('drive joints by hand: shoulder stations about the hinge x, mesh forces per shaft, eye loads in full', () => {
  const d = buildDesign(round5Params(base(), {}, catalog), catalog);
  const J = Object.fromEntries(d.structure.joints.map((j) => [j.name, j]));
  const hx = d.wing.hinge[0];
  assert.deepEqual(J['shoulder pivot left'].supports.map((s) => s.x_m), [+(hx - 0.012).toFixed(5), +(hx + 0.012).toFixed(5)]);
  // The coupler's share at the horn plane (film layout couplerR) by the lever rule, computed here from the stations.
  const P = d.structure.joints_layout.planes_m;
  const aft = J['shoulder pivot right'].supports[0];
  near(aft.radial.find((t) => t.ch === 'coupler_R').k, (hx + 0.012 - P.couplerR) / 0.024, 1e-5);
  // Mesh force per crank N m, 1 / (below x r_wheel x cos 20): stage 4 (r 20 mm) 53.209, stage 3 (r 20, below 40/18)
  // 23.944, stage 2 (DS1-64, r 32 mm, below (40/18)^2) 6.734, stage 1 (DS1-32, r 16, below 3.2 (40/18)^2) 4.209.
  const c20 = Math.cos((20 * Math.PI) / 180), r = 40 / 18;
  const F = [1 / (3.2 * r * r * 0.016 * c20), 1 / (r * r * 0.032 * c20), 1 / (r * 0.02 * c20), 1 / (0.02 * c20)];
  // A load between the plates splits so the two shares sum to the load: per shaft, each mesh term sums to its force.
  const sums = (name) => {
    const [a, b] = J[name].supports.map((s) => s.radial.filter((t) => t.ch === 'crank_torque').map((t) => t.k));
    return a.map((k, i) => k + b[i]);
  };
  for (const [name, ks] of [['gear shaft 1', [F[2], F[3]]], ['gear shaft 2', [F[1], F[2]]], ['gear shaft 3', [F[0], F[1]]]]) sums(name).forEach((v, i) => near(v, ks[i], 2e-3, `${name} ${i}`));
  // Each coupler eye carries its coupler's force in full.
  for (const n of ['coupler eye right, crank pin', 'coupler eye left, rocker horn']) assert.deepEqual(J[n].supports[0].radial.map((t) => [t.ch, t.k]), [[/right/.test(n) ? 'coupler_R' : 'coupler_L', 1]]);
  near(boreOf({ specs: { bore_mm: 8 } }), 0.008, 1e-12);
});

test('a charge-limited pack starts the flight at its charge fraction, and M2 counts only what it holds above the 20% reserve', async () => {
  const { setupFlight, runM2 } = await import('../../src/physics/scenarios.js');
  const d = base();
  const bt = d.battery;
  assert.ok(bt.charge_fraction < 1, 'the design charges below full');
  const { sim } = setupFlight(d, { sensorNoise: false });
  near(sim.x[20], (1 - bt.charge_fraction) * bt.capacity_Ah * 3600, 1e-6);
  const r = runM2(d, { tEnd: 21, sensorNoise: false });
  near(r.measured.usable_fraction, bt.charge_fraction - 0.2, 1e-12);
  near(r.measured.endurance_min, (bt.capacity_Ah * 3600 * (bt.charge_fraction - 0.2)) / r.measured.cruise_batt_A / 60, 1e-9);
  assert.equal(r.measured.charge_limit_V_per_cell, bt.charge_limit_V_per_cell);
  // A pack charged to full keeps round 4's 80%.
  const full = runM2({ ...d, battery: { ...bt, charge_fraction: undefined } }, { tEnd: 21, sensorNoise: false });
  assert.equal(full.measured.usable_fraction, 0.8);
  assert.equal(full.measured.charge_limit_V_per_cell, undefined);
});

test('stage geometry for the mesh ruling, and the charge limit on a shifted (Li-ion) curve', async () => {
  const { mixedMeshCheck } = await import('../../src/physics/structure.js');
  // v12's stage 2: a steel MSGA1-18 on an MC901 PS1-60 (m1, 10 mm faces, 20 deg).
  const [s] = stages(['gear-khk-msga1-18', 'gear-khk-ps1-60'], catalog);
  assert.equal(s.ratio, 60 / 18);
  near(s.pinion_rp_m, 0.009, 1e-12);
  near(s.gear_rp_m, 0.03, 1e-12);
  near(s.pinion_face_m, 0.01, 1e-12);
  near(s.gear_face_m, 0.01, 1e-12);
  near(s.alpha_rad, (20 * Math.PI) / 180, 1e-12);
  assert.match(s.pinion_material, /SCM415/);
  assert.match(s.gear_material, /MC901/);
  // KHK publishes no surface allowable for MC901: under the ruling this stage cannot be judged.
  const m = mixedMeshCheck(s, 0.68);
  assert.deepEqual([m.mixed, m.ok], [true, false]);
  // A Li-ion pack's rested curve sits 0.1 V per cell lower (3.6 V nominal). Charged to 4.10 V per cell it presents
  // 2 x 4.10 = 8.20 V, and 4.10 V is the top of its shifted curve (4.18 - 0.1 = 4.08 V at full): it holds all of it.
  const li = batteryModel(byId(catalog, 'batt-nexus-2s1p-p30b'));
  near(li.cell_shift_V, -0.1, 1e-9);
  const lim = applyChargeLimit(li, { V_per_cell: 4.1 });
  near(batteryServoSupply('battery', lim).current_V, 8.2, 1e-9);
  near(lim.charge_fraction, 1, 1e-4);
  // At 4.00 V per cell it holds what the curve gives at 4.10 LiPo-equivalent: 80% + 20% x 0.10 / 0.18.
  near(applyChargeLimit(li, { V_per_cell: 4.0 }).charge_fraction, 0.8 + (0.2 * 0.1) / 0.18, 1e-4);
});
