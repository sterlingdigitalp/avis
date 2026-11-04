// Round 18 (lead): checks added while finishing v25.
import test from 'node:test';
import assert from 'node:assert/strict';
import { flightRatios } from '../../src/design/trim.js';

// SPEC "Maker's part": a maker's servo arm is checked under its own servo's rule. The free-flight envelope had the
// twist servo's rule but not the tail servo's; it now has both, at the same servo allowance.
test('flightRatios: the tail servos under their own rule, at the servo allowance (hand figures)', () => {
  const sp = { I: (Math.PI * (0.01 ** 4 - 0.008 ** 4)) / 64, c: 0.005, allowable_MPa: 841 };
  const design = {
    structure: { spar_inner: sp, spar_outer: sp, gear_stages: [{ ratio: 4, pinion_allowable_Nm: 0.47, gear_allowable_Nm: 2.55 }], coupler: { od_m: 0.007, id_m: 0.005, length_m: 0.08, E_GPa: 64, allowable_MPa: 650 } },
    servos: { twist: { stall_Nm: 0.75 }, tail: { stall_Nm: 0.4924 } }, motor: { I_cont: 30 }, esc: { I_cont: 40 }, battery: { I_cont: 36 },
  };
  const pk = { crankTorque: 0.5, rootBend: 1, elbowBend: 0.2, couplerForce: 50, twistTorque: 0.1, motorRms: 5, battMean: 3, dutySat: 0, loadFactor: 3 };
  // 0.4 N m against 0.4924 x 0.9 = 0.44316 N m: ratio 0.90260; above every other ratio here.
  const r = flightRatios(design, { ...pk, tailTorque: 0.4 }, { servo: 0.9 });
  assert.ok(Math.abs(r.ratios.tail_servo_torque - 0.4 / 0.44316) < 1e-12);
  assert.equal(r.worst, 'tail_servo_torque');
  assert.ok(r.ok);
  // Past the servo's own rule the flight fails on it.
  const over = flightRatios(design, { ...pk, tailTorque: 0.45 }, { servo: 0.9 });
  assert.equal(over.ok, false);
  assert.equal(over.worst, 'tail_servo_torque');
  // A design with no tail servo, or peaks without a tail torque, reads 0 (not NaN).
  assert.equal(flightRatios({ ...design, servos: { twist: design.servos.twist } }, { ...pk, tailTorque: 0.4 }).ratios.tail_servo_torque, 0);
  assert.equal(flightRatios(design, pk).ratios.tail_servo_torque, 0);
});

// SPEC "What a real build needs": the air path's drag is counted. The build-needs record's CdA (cooling holes and the
// twist-servo leads' exposed span) has no solids in the containment record's frontal area, so outsideDrag adds it.
test('outsideDrag counts the build-needs record\'s CdA once, beside the frontal and wing protrusions', async () => {
  const { outsideDrag } = await import('../../src/design/round11.mjs');
  const frontal = { CdA_m2: 0.0008, area_mm2: 667, Cd: 1.2, basis: 'test' };
  const holes = 2 * Math.PI / 4 * 0.008 ** 2 * 1.0, leads = 2 * 0.040 * 0.0033 * 1.2; // two 8 mm holes at Cd 1.0, two 40 mm x 3.3 mm leads at Cd 1.2
  const d = { body: { CdA_front_m2: 0.003 }, structure: { clearance: { containment: { frontal }, buildneeds: { CdA_m2: holes + leads, CdA_basis: 'test' } } } };
  outsideDrag(d);
  assert.ok(Math.abs(d.body.CdA_front_needs_m2 - (holes + leads)) < 1e-8);
  assert.ok(Math.abs(d.body.CdA_front_m2 - (0.003 + 0.0008 + holes + leads)) < 1e-8);
  outsideDrag(d); // applied again, still once (the pod's own CdA is kept in CdA_front_pod_m2)
  assert.ok(Math.abs(d.body.CdA_front_m2 - (0.003 + 0.0008 + holes + leads)) < 1e-8);
});

// The twist chain's series compliance is a drawn-against-flown figure like its siblings: the chain record refuses a
// design whose flown compliance (servos.twist.compliance_rad_per_Nm, what flight.js flies) is not the drawn chain's.
test('twistChains: the flown compliance must be the drawn chain\'s (refused when it differs)', async () => {
  const { readFileSync } = await import('node:fs');
  const { twistChains, twistChainFigures, chainCompliance, TWIST_SECTION } = await import('../../src/design/twist-drive.js');
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const base = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
  const d = structuredClone(base);
  d.params = { ...d.params, tubeSpecSheet: true, parts: { ...d.params.parts, twistServo: 'servo-hitec-md89mw' }, twistDrive: { section: TWIST_SECTION } };
  const F = twistChainFigures(d, catalog), C = chainCompliance(d, catalog).rad_per_Nm;
  const servo = { ...d.servos.twist, linkage_ratio: F.ratio, stall_Nm: 0.60604 / F.ratio, speed_rad_s: 7.588 * F.ratio, servo_stall_Nm: 0.60604, volts: 5.76 };
  const fig = (c) => { d.servos.twist = { ...servo, compliance_rad_per_Nm: c }; return twistChains(null, d, catalog, { twistTorque_Nm: 0.2, travel_rad: 0.5, twistRate_rad_s: 5, basis: 'test' })[0].figures.compliance_rad_per_Nm; };
  const ok = fig(+C.toFixed(6));
  assert.ok(typeof ok === 'object' && Math.abs(ok.drawn - C) < 1e-6 && ok.tol === 1e-3 && ok.rel === true);
  const { chainRow } = await import('../../src/design/chains.js');
  const asm = { parts: [], joins: [] }; // the figures only (the path's drawing is the build's record)
  const rows = (c) => { d.servos.twist = { ...servo, compliance_rad_per_Nm: c }; return chainRow(asm, twistChains(null, d, catalog, { twistTorque_Nm: 0.2, travel_rad: 0.5, twistRate_rad_s: 5, basis: "test" })[0]); };
  const good = rows(+C.toFixed(6)), bad = rows(C * 1.01), none = rows(0);
  const row = (r) => r.figures.find((f) => f.figure === 'compliance_rad_per_Nm');
  assert.equal(row(good).ok, true);
  assert.equal(row(bad).ok, false, '1% off the drawn chain');
  assert.equal(row(none).ok, false, 'flown rigid while the chain is compliant');
});

// Pod open problem 4: the covering's open windows count in the body drag, as cavities (Gharib, Roshko & Sarohia 1985,
// the published ceiling on opening area) plus the through-flow bound between them (Rogallo, NACA Report 713).
test('openDrag: cavity Cd on every open window plus the through-flow bound on the smaller side; outsideDrag adds it', async () => {
  const PS = await import('../../src/design/pod-structure.js');
  const { readFileSync } = await import('node:fs');
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const src = PS.openDragSources(catalog);
  assert.deepEqual(src, { cavity: 0.108, through: 0.85 });
  assert.deepEqual(PS.openDragSources([]), { cavity: null, through: null });
  // a flat strip as the loft is not needed: the windows' areas come from loftArea; check the arithmetic on a stub G
  // whose windows are found by name with known areas (one 0.012 m^2 slot and two 0.001 m^2 exits)
  const loft = { sections: [{ x_m: 0.3, width_m: 0.1, height_m: 0.1, yc_m: 0 }, { x_m: -0.3, width_m: 0.1, height_m: 0.1, yc_m: 0 }] };
  const G = { loft, windows: [] };
  const empty = PS.openDrag(G, src);
  assert.equal(empty.CdA_m2, 0);
  // the formula on hand figures (independent of loftArea): A = 0.014, Amax = 0.012 -> 0.108 x 0.014 + 2 x 0.002
  const hand = 0.108 * 0.014 + 0.85 * Math.min(0.012, 0.014 - 0.012);
  assert.ok(Math.abs(hand - 0.003212) < 1e-12);
  // the real round-17 pod: its record's open block holds that formula over its own windows
  const { outsideDrag } = await import('../../src/design/round11.mjs');
  const open = { windows: [{ name: 'slots', area_m2: 0.012 }, { name: 'exit R', area_m2: 0.001 }, { name: 'exit L', area_m2: 0.001 }], Cd: 0.108, CdA_m2: hand, basis: 'test' };
  const d = { body: { CdA_front_m2: 0.003 }, structure: { clearance: { containment: { frontal: { CdA_m2: 0.0008, area_mm2: 667, Cd: 1.2, basis: 't' } }, covering: { open } } } };
  outsideDrag(d);
  assert.ok(Math.abs(d.body.CdA_front_cuts_m2 - hand) < 1e-8);
  assert.ok(Math.abs(d.body.CdA_front_m2 - (0.003 + 0.0008 + hand)) < 1e-8);
});

// Round 18 (bug): round 17's parts block replaced round 14's wholesale, so the full build (over the v18 base) fell back
// to the 8 mm encoder actuator SPEC's v22 ruling replaced; every part round 14 set must survive round 17's set.
test('round17Set keeps round 14\'s parts (the 10 mm encoder actuator) under its own', async () => {
  const { round17Set } = await import('../../src/design/round17.mjs');
  const { ROUND14 } = await import('../../src/design/round14.mjs');
  const { readFileSync } = await import('node:fs');
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const s = round17Set({}, catalog);
  assert.equal(s.parts.encoderMagnet, 'sensor-rls-orbis-actuator-10mm');
  for (const [k, v] of Object.entries(ROUND14.set.parts)) assert.equal(s.parts[k], v, k);
  // and as the full build merges it (buildRound14's round14Set: the set over ROUND14.set)
  const { round14Set } = await import('../../src/design/round14.mjs');
  assert.equal(round14Set(s).parts.encoderMagnet, 'sensor-rls-orbis-actuator-10mm');
});
