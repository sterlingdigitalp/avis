// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// Panel numbers come from the design, the catalogue and the gate files; the bake has the shape Blender expects;
// focus and explode change what update() promises.
import test from 'node:test';
import assert from 'node:assert/strict';
import { loadModel, loadVerification } from '../../sims/avis-shared/node-load.js';
import { syntheticLogFile } from './fixture.mjs';
import { numbers, gateLine } from '../../sims/avis-shared/numbers.js';
import { nodeBird, posedNodes } from '../lib/bird-node.mjs';
import { bake } from '../bake_pose.mjs';

const { model, design } = loadModel('larus-gl-2');
const gates = loadVerification('larus-gl-2');
const N = numbers(model, { gates, fit: model.fit });

test('drive numbers: ratio from the teeth, motor speed from the trimmed flap frequency', () => {
  assert.ok(Math.abs(N.ratio - design.drive.gearRatio) < 1e-9);
  const row = design.control.trim.reduce((b, r) => (Math.abs(r.V - design.control.Vcruise) < Math.abs(b.V - design.control.Vcruise) ? r : b));
  assert.equal(N.flapHz, row.freq);
  assert.ok(Math.abs(N.motorRpm - row.freq * 60 * N.ratio) < 1e-9);
  assert.equal(N.stages.length, design.drive.gears.length / 2);
});

test('power numbers: battery energy from its cells and capacity, cruise power from the M2 gate file', () => {
  assert.ok(Math.abs(N.batteryWh - design.battery.V_nom * design.battery.capacity_Ah) < 1e-12);
  const m2 = gates.find((g) => g.gate === 'M2');
  if (m2) assert.equal(N.cruisePowerW, m2.measured.cruise_power_w);
  else assert.equal(N.cruisePowerW, null);
  assert.equal(N.passCount, gates.filter((g) => g.passed).length);
});

test('mass numbers are the BOM totals', () => {
  assert.equal(N.mass_g, design.mass.total_g);
  assert.equal(N.parts_g, design.bom.parts_g);
  assert.equal(N.margin_g, design.bom.margin_g);
  assert.equal(N.partCount, design.bom.parts.reduce((a, p) => a + p.count, 0));
});

test('twist schedule: equal and opposite about sym over half a cycle', () => {
  const a = N.twistAt(0.4, 1), b = N.twistAt(0.4 + Math.PI, 1);
  assert.ok(Math.abs(a + b - 2 * N.sched.sym) < 1e-12);
});

test('gate lines carry the numbers from the gate files', () => {
  assert.ok(gateLine({ gate: 'M4', measured: { seeds_passed: 31, seeds_run: 100 } }).includes('31 of 100'));
  assert.ok(gateLine({ gate: 'M2', measured: { endurance_min: 11.27, cruise_power_w: 41.4, robustness_passed: false } }).includes('11.3 min at 41.4 W; fails'));
  assert.ok(gateLine({ gate: 'M7', measured: { spar_inner: { fos: 0.83 }, coupler: { fos_buckling: 0.49 } } }).includes('FoS 0.83, coupler buckling 0.49'));
  assert.equal(gateLine({ gate: 'M8', measured: { motor_rms_2s_max_A: 38, motor_continuous_A: 30, esc_rms_2s_max_A: 38, esc_continuous_A: 40 } }), 'Currents: motor 38.0 A against 30 A continuous');
  assert.ok(gateLine({ gate: 'M1', measured: { height_lost_m: 0.844, robustness_passed: false } }).includes('0.84 m lost, fails'));
  assert.ok(gateLine({ gate: 'M3', measured: { altitude_dev_m: 1.15 } }).includes('1.1 m') || gateLine({ gate: 'M3', measured: { altitude_dev_m: 1.15 } }).includes('1.2 m'));
  assert.ok(gateLine({ gate: 'M6', measured: { total_g: 934.2 } }).includes('934 g'));
  assert.ok(gateLine({ gate: 'V2', measured: { max_error_pct: 4.27 } }).includes('4.3%'));
});

test('every gate line is written from its own file, and a stored line wins', () => {
  for (const g of gates) {
    const line = gateLine(g);
    assert.ok(typeof line === 'string' && line.length > 3, g.gate);
  }
  const m4 = gates.find((g) => g.gate === 'M4');
  if (m4) assert.ok(gateLine(m4).includes(String(m4.measured.seeds_passed)));
  const m2 = gates.find((g) => g.gate === 'M2');
  if (m2) assert.ok(gateLine(m2).includes(m2.measured.endurance_min.toFixed(1)));
  const m8 = gates.find((g) => g.gate === 'M8');
  if (m8) assert.ok(/A against/.test(gateLine(m8)));
  assert.equal(gateLine({ gate: 'X9', measured: {} }), 'X9');
  assert.equal(gateLine({ gate: 'M1', line: 'kept' }), 'kept');
  assert.ok(/—/.test(gateLine({ gate: 'M5', measured: {} })));
});

test('the pose bake holds every posed node and deforming mesh for every frame, all finite', () => {
  const { header, buf } = bake(syntheticLogFile(), { fps: 30, from: 10, to: 14 });
  const { bird } = nodeBird('larus-gl-2');
  assert.deepEqual(header.frames, [10, 11, 12, 13]);
  assert.equal(header.nodes.length, posedNodes(bird).length);
  const per = 16 * header.nodes.length + header.deform.reduce((a, d) => a + 3 * d.count, 0);
  assert.equal(header.floats_per_frame, per);
  assert.equal(buf.length, 4 * per);
  for (const v of buf) assert.ok(Number.isFinite(v));
});

test('focus ghosts the rest and shows the accents; explode moves box parts; the shell lifts with the structure', () => {
  const { bird } = nodeBird('larus-gl-2');
  bird.update({ phase: 1, amp: model.law.ampNominal, focus: 'power', accents: true, explode: { power: 1, structure: 1 } });
  const batt = bird.parts.find((p) => p.role === 'battery').objects[0];
  const rib = bird.root.getObjectByName('rib_R_03');
  assert.equal(batt.material.name, 'battery_wrap');
  assert.notEqual(rib.material.name, 'balsa');
  const node = bird.root.getObjectByName('part_battery');
  assert.ok(node.position.distanceTo(node.userData.home) > 0.05);
  assert.ok(bird.accents.some((a) => a.visible && a.userData.subsystem === 'power') || bird.accents.every((a) => a.userData.subsystem !== 'power'));
  assert.equal(bird.root.getObjectByName('shell').visible, false);
  bird.update({ phase: 1, amp: model.law.ampNominal, explode: { structure: 1 } });
  assert.ok(bird.root.getObjectByName('shell').position.y > 0.05);
  assert.equal(rib.material.name, 'balsa');
  bird.update({ phase: 1, amp: model.law.ampNominal });
  assert.ok(node.position.distanceTo(node.userData.home) < 1e-12);
});

test('HUD climb readout: signed with a true minus, rounded before the sign so a near-level climb never reads −0.0', async () => {
  const { signed } = await import('../../sims/avis-shared/film.js');
  assert.equal(signed(1.13, 1), '+1.1');
  assert.equal(signed(-0.6, 1), '−0.6');
  assert.equal(signed(-0.04, 1), '+0.0');
  assert.equal(signed(-0.06, 1), '−0.1');
  assert.equal(signed(NaN, 1), '—');
});

test('flight HUD: airspeed, flap-cycle power, climb and battery from the segment data, nothing without it', async () => {
  const { flightHud } = await import('../../sims/avis-shared/film.js');
  const d = { airspeed: [12.34], power_w: [95.4], climb_m_s: [1.13], battery_pct: [91.2] };
  const h = flightHud(d, 0);
  for (const s of ['12.3 m/s', '95 W', '+1.1 m/s', '91%']) assert.ok(h.includes(s), `${s} in ${h}`);
  assert.ok(!flightHud({ ...d, climb_m_s: undefined }, 0).includes('climb'));
  assert.equal(flightHud(null, 0), '');
  assert.equal(flightHud(d, -1), '');
});

test('drive callouts follow the design: no stroke servo on a fixed-stroke drive, the crank wheel its own teeth', async () => {
  const { driveLabels } = await import('../../sims/avis-shared/film.js');
  const { model } = loadModel('larus-gl-2');
  const fixed = { ...model, law: { ...model.law, ampMin: 0.6, ampMax: 0.6 } };
  const ranged = { ...model, law: { ...model.law, ampMin: 0.4, ampMax: 0.7 } };
  assert.deepEqual(driveLabels(fixed).late, { node: 'encoder_readhead', text: 'Crank angle encoder', at: [450, 640] });
  assert.deepEqual(driveLabels(ranged).late, { node: 'slot_pin_R', text: 'Stroke servo slides the pin', at: [700, 195] });
  assert.equal(driveLabels(model).crankWheel, `Crank wheel ${model.train.stages.at(-1).wheel.z}T`);
  // whichever it is, the node it points at exists in the bird
  const { bird } = nodeBird('larus-gl-2');
  assert.ok(bird.root.getObjectByName(driveLabels(model).late.node));
});

test('captions take their numbers from the design, not from the edit', async () => {
  const { fillDesign } = await import('../../sims/avis-shared/film.js');
  const { design } = loadModel('larus-gl-2');
  const edit = JSON.parse((await import('node:fs')).readFileSync(new URL('../edit.json', import.meta.url), 'utf8'));
  const wide = edit.segments.find((s) => s.id === 'wide');
  const text = fillDesign(wide.sub, design);
  assert.ok(text.includes(`${design.atmosphere.altitude_m.toLocaleString('en-US')} m`) && text.includes(`${design.atmosphere.temperature_C} °C`), text);
  assert.ok(!/\{/.test(text));
  // no caption types a number of its own: digits in captions come from the design or name the log
  for (const s of edit.segments) for (const t of [s.caption, s.sub]) if (t) assert.ok(!/\d/.test(t.replace(/larus-gl-2-corridor|\{\w+\}/g, '')), `${s.id}: "${t}"`);
});
