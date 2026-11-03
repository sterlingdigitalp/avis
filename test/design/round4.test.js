// Round 12: the design-file fixture is v20 (test/fixtures/larus-gl-2-v20.json, the committed HEAD design), the design these hand calculations were written for; v21 has its own tests (round12).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { servoVoltageRange, servoStallCurrentConservative, escCellsCover, byId, batteryModel } from '../../src/design/catalog.js';
import { buildDesign, batteryServoSupply } from '../../src/design/build.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
// The current design's parameters (any round): the servo and pack checks below set the parts they depend on.
// The live design's parameters as the round-4 base, without the round-9 gear rating conditions (p.gearRating), which
// re-rate the train and belong to round 9's pipeline (test/design/round9.test.js), not to round 4's.
const v9params = () => { const { gearRating, ...p } = JSON.parse(readFileSync(join(ROOT, 'test/fixtures/larus-gl-2-v20.json'), 'utf8')).params; return p; };

test('servo voltage range is read from the published text', () => {
  assert.deepEqual(servoVoltageRange({ specs: { voltage_range_text: '4.8V - 8.4V DC Volt' } }), [4.8, 8.4]);
  assert.deepEqual(servoVoltageRange({ specs: { voltage_range_text: '6.0V - 8.4V' } }), [6, 8.4]);
  assert.deepEqual(servoVoltageRange({ specs: { voltage_range_text: '4.8V ~ 6.0V ~ 7.4V' } }), [4.8, 7.4]);
  assert.equal(servoVoltageRange({ specs: {} }), null);
});

test('stall current on the conservative side: published rows, scaled by voltage above the top row', () => {
  const hv = byId(catalog, 'servo-mks-hv6160'); // rows 1.56 A @ 4.8, 1.9 @ 6.0, 2.2 @ 7.4, 2.5 @ 8.2
  assert.equal(servoStallCurrentConservative(hv, 7.4), 2.2);
  assert.ok(Math.abs(servoStallCurrentConservative(hv, 6.7) - (1.9 + (0.7 / 1.4) * 0.3)) < 1e-12);
  assert.ok(Math.abs(servoStallCurrentConservative(hv, 8.4) - 2.5 * (8.4 / 8.2)) < 1e-12);
  // A figure published without a voltage is used as published at any supply (SPEC ruling after round 2).
  const free = { specs: { stall_current: [{ voltage_v: null, current_a: 4.2 }] } };
  assert.equal(servoStallCurrentConservative(free, 8.4), 4.2);
});

test('ESC cell range: a 3-4S ESC may not run a 2S pack', () => {
  assert.equal(escCellsCover(byId(catalog, 'esc-hobbywing-skywalker-40a-v2'), 2), false);
  assert.equal(escCellsCover(byId(catalog, 'esc-hobbywing-skywalker-40a-v2'), 3), true);
  assert.equal(escCellsCover(byId(catalog, 'esc-hobbywing-skywalker-30a-v2-mini'), 2), true);
  assert.equal(escCellsCover({ specs: {} }, 2), null);
});

test('battery servo supply: 20% charge less sag at the pack rating for performance, full charge for current', () => {
  const b = batteryModel(byId(catalog, 'batt-gensace-1300-2s-45c'));
  // Gens Ace 1300 2S 45C: I_cont = 45 C x 1.3 Ah = 58.5 A; R_int estimated 0.1 x 7.4 V / 58.5 A = 0.01265 ohm.
  // Rested 20% charge on the model curve: 3.68 V per cell = 7.36 V; sag 58.5 x 0.01265 = 0.74 V: 6.62 V.
  assert.ok(Math.abs(b.I_cont - 58.5) < 1e-9);
  const s = batteryServoSupply('battery', b);
  assert.ok(Math.abs(s.performance_V - 6.62) < 0.001, `${s.performance_V}`);
  assert.ok(Math.abs(s.current_V - 8.4) < 1e-9);
  assert.equal(batteryServoSupply('bec', b), null);
});

test('a design with HV servos on the 2S pack: servo figures interpolated at 6.62 V, through the horn ratio', () => {
  const p = v9params();
  p.servoSupply = 'battery';
  // Round 4's configuration: no charge limit, the ESC's BEC (the design file may carry round-5 parameters).
  p.chargeLimit = null;
  p.becFrom = null;
  p.parts = { ...p.parts, battery: 'batt-gensace-1300-2s-45c', esc: 'esc-hobbywing-skywalker-30a-v2-mini' };
  const d = buildDesign(p, catalog);
  // HV6160 rows 6.0 V: 1.422 N m, 0.182 s/60; 7.4 V: 1.6671, 0.147. At 6.62 V: f = 0.4429; 1.5306 N m, 0.16650 s/60.
  const f = (6.62 - 6) / 1.4;
  const T = 1.422 + f * (1.6671 - 1.422), sp = Math.PI / 3 / (0.182 - f * 0.035);
  assert.ok(Math.abs(d.servos.tail.stall_Nm - T) < 1e-3, `${d.servos.tail.stall_Nm}`);
  assert.ok(Math.abs(d.servos.tail.speed_rad_s - sp) < 1e-2);
  assert.ok(Math.abs(d.servos.twist.stall_Nm - T / 1.6) < 1e-3);
  assert.ok(Math.abs(d.servos.twist.speed_rad_s - sp * 1.6) < 2e-2);
  assert.ok(Math.abs(d.servos.twist.stall_A - 2.5 * 8.4 / 8.2) < 1e-9);
  assert.equal(d.electrical.servo_source, 'battery');
  assert.equal(d.electrical.bec.carries, 'avionics only');
  // The same parts on the 3-4S ESC are refused.
  p.parts.esc = 'esc-hobbywing-skywalker-40a-v2';
  assert.throws(() => buildDesign(p, catalog), /not rated for a 2S pack/);
});

test('limiter configuration: setpoint a fraction of the lower of the motor and ESC continuous ratings', async () => {
  const { limiterConfig } = await import('../../src/design/trim.js');
  const d = { motor: { I_cont: 36 }, esc: { I_cont: 30 } };
  const c = limiterConfig(d, { fraction: 0.9 });
  assert.equal(c.rated_A, 30);
  assert.equal(c.setpoint_A, 27);
  assert.deepEqual([c.tauRms_s, c.kP, c.kI, c.attnCutoff_Hz, c.maxAttn], [0.5, 0.005, 0.02, 1.2, 0.5]);
  assert.equal(limiterConfig(d, { fraction: 0.8, kI: 0.05 }).kI, 0.05);
  assert.equal(limiterConfig({ motor: { I_cont: 20 }, esc: { I_cont: 40 } }, { fraction: 1 }).setpoint_A, 20);
  assert.equal(limiterConfig(d, null), null);
  assert.throws(() => limiterConfig(d, { fraction: 1.2 }), /fraction/);
  assert.throws(() => limiterConfig({ motor: {}, esc: {} }, { fraction: 0.9 }), /rating/);
});

test('round-4 parameters: the base, then ROUND4, then the overrides; the train ratio from the gears', async () => {
  const { round4Params, ROUND4 } = await import('../../src/design/round4.mjs');
  const base = { params: { ...v9params(), parts: { ...v9params().parts, gears: ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'] } } };
  const p = round4Params(base, { span_m: 1.9, parts: { motor: 'motor-cobra-c-2814-16', gears: base.params.parts.gears } }, catalog);
  assert.equal(p.span_m, 1.9, 'the override wins over ROUND4');
  assert.equal(p.parts.motor, 'motor-cobra-c-2814-16');
  assert.equal(p.parts.battery, ROUND4.parts.battery, 'ROUND4 wins over the base');
  assert.equal(p.servoSupply, 'battery');
  // 80/20 x 40/18 x 40/18 = 19.753; crank rating governed by the DS0.8-20 pinion: 0.47 N m x 19.753 = 9.284 N m.
  assert.ok(Math.abs(p.gearRatio - 4 * (40 / 18) ** 2) < 1e-12);
  assert.ok(Math.abs(p.crankRating - 0.47 * 4 * (40 / 18) ** 2) < 1e-9);
  // Four stages: 30/20 x 60/18 x (40/18)^2 = 24.69; the DS0.8-20 pinion still governs at 0.47 x 24.69 = 11.60 N m.
  const q = round4Params(base, {}, catalog);
  assert.deepEqual(q.parts.gears, ROUND4.parts.gears);
  assert.ok(Math.abs(q.gearRatio - 1.5 * (60 / 18) * (40 / 18) ** 2) < 1e-12);
  assert.ok(Math.abs(q.crankRating - 0.47 * q.gearRatio) < 1e-9);
});

test('per-speed envelope: each speed bisected to its own limit, below the trim stroke speed when needed', async () => {
  const { flightEnvelope } = await import('../../src/design/trim.js');
  const design = { drive: { ampMin: 0.3, ampMax: 0.6 }, control: { trim: [{ V: 10, freq: 2, amp: 0.5 }, { V: 14, freq: 2.5, amp: 0.5 }] } };
  // Synthetic rating on stroke speed amp x f: 2.0 at 10 m/s, 1.6 at 14, 0.6 at 17 (below the 14 m/s trim's 1.25).
  const lim = { 10: 2.0, 14: 1.6, 17: 0.6 };
  const tester = async (V, f, amp) => ({ ok: amp * f <= lim[V], worst: V === 17 ? 'gear' : 'inner_spar', pk: { crankTorque: amp * f } });
  const env = await flightEnvelope(design, { speeds: [10, 14, 17], nAmp: 3, tester, iters: 40, perSpeed: true });
  for (const e of env) {
    assert.deepEqual(e.bySpeed.map((q) => q.V), [10, 14, 17]);
    for (const q of e.bySpeed) {
      // Search between the 0.8 Hz floor (0 below it) and the cap 2.5 x the fastest trim frequency, 6.25 Hz.
      const f = lim[q.V] / e.amp;
      const want = f < 0.8 ? 0 : Math.min(f, 6.25);
      assert.ok(Math.abs(q.freqMax - want) < 1e-6, `${e.amp} ${q.V}: ${q.freqMax} vs ${want}`);
    }
    assert.equal(e.atV, 17);
    assert.ok(Math.abs(e.freqMax - 0.6 / e.amp) < 1e-6, 'freqMax is the minimum over speeds');
  }
  // A speed where even 0.8 Hz breaks a rating gets 0 there, and the others keep theirs.
  const t2 = async (V, f, amp) => ({ ok: V === 17 ? false : amp * f <= lim[V], worst: 'coupler', pk: {} });
  const e2 = await flightEnvelope(design, { speeds: [10, 17], nAmp: 2, tester: t2, iters: 30, perSpeed: true });
  assert.deepEqual(e2.map((e) => e.bySpeed[1].freqMax), [0, 0]);
  assert.ok(Math.abs(e2[0].bySpeed[0].freqMax - 6.25) < 1e-6, 'the 10 m/s limit (6.67 Hz) is above the 6.25 Hz search cap');
});

test('envelope lookup: linear in amplitude and in airspeed, held outside the measured ranges', async () => {
  const { envelopeAt } = await import('../../src/physics/control.js');
  const env = [
    { amp: 0.4, freqMax: 3, bySpeed: [{ V: 10, freqMax: 5 }, { V: 14, freqMax: 3 }] },
    { amp: 0.6, freqMax: 2, bySpeed: [{ V: 10, freqMax: 4 }, { V: 14, freqMax: 2 }] },
  ];
  assert.equal(envelopeAt(env, 0.4, 10), 5);
  assert.equal(envelopeAt(env, 0.4, 12), 4);
  assert.equal(envelopeAt(env, 0.5, 12), 3.5); // (4 + 3) / 2
  assert.equal(envelopeAt(env, 0.6, 20), 2); // held at the fastest speed
  assert.equal(envelopeAt(env, 0.2, 8), 5); // held at the smallest amplitude and slowest speed
  assert.equal(envelopeAt(env, 0.5), 2.5); // no airspeed: the per-amplitude minimum
  assert.equal(envelopeAt([{ amp: 0.4, freqMax: 3 }, { amp: 0.6, freqMax: 2 }], 0.5, 12), 2.5); // rounds 1-3 rows
});

test('bearing layout and derived parts: counts from the drive, masses from catalogue materials', () => {
  // Round 4's slotted drive with one bearing type (the design file may carry round-5 parameters: joints, fixed stroke).
  const p = { ...v9params(), joints: null, fixedStroke: false, bearingLayout: 'auto', derivedParts: true, elbowGain: 0.5, parts: { ...v9params().parts, strokeServo: 'servo-mks-hv6160', bearing: v9params().parts.bearing ?? 'bearing-skf-618-4', gears: ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'] } };
  const hinged = buildDesign(p, catalog);
  // 2 per shoulder + 2 per gearbox shaft (3 stages) + 2 per hinged elbow = 4 + 6 + 4.
  assert.deepEqual(hinged.drive.bearings.map((b) => b.count), [4, 6, 4]);
  assert.equal(hinged.bom.parts.find((b) => b.role === 'bearing').count, 14);
  const rigid = buildDesign({ ...p, elbowGain: 0 }, catalog);
  assert.deepEqual(rigid.drive.bearings.map((b) => b.count), [4, 6, 0]);
  // Stroke linkage: 2 x 60 mm of 2 mm rod at 4.5 g/m + 40 x 20 mm of 2 mm plate at 2.71 kg/m2 = 0.54 + 2.168 g.
  const s = (d, re) => d.bom.structure.find((x) => re.test(x.name)).mass_g;
  assert.ok(Math.abs(s(hinged, /stroke servo linkage/) - (2 * 4.5 * 0.06 + 2710 * 0.0008)) < 1e-9);
  // Pitot-static probe: 100 mm of Ti tube at 34 g/m and of 5x4 carbon tube at 11.1 g/m = 4.51 g.
  assert.ok(Math.abs(s(hinged, /pitot/) - 4.51) < 1e-9);
  // The margin is 15% of parts and structure, with the new items in it.
  assert.ok(Math.abs(hinged.bom.margin_g - 0.15 * (hinged.bom.parts_g + hinged.bom.structure_g)) < 1e-9);
});

test('body fit: the pod found holds every part, its centre line and the lumps follow the packing', async () => {
  const { fitBody, packDesign } = await import('../../src/design/fit.js');
  const p = { ...v9params(), parts: { ...v9params().parts, gears: ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'] } };
  const before = packDesign(buildDesign({ ...p, bodyWidth_m: 0.06, bodyHeight_m: 0.07, bodyY_m: 0, lumpPos: undefined }, catalog), catalog);
  assert.ok(before.issues.length > 0, 'the round-3 pod does not hold the drive');
  const r = fitBody(p, catalog);
  assert.ok(!r.failed, r.failed);
  assert.deepEqual(r.packing.issues, []);
  const d = buildDesign(r.params, catalog);
  assert.equal(d.body.width_m, r.body.W);
  assert.equal(d.body.height_m, r.body.H);
  assert.ok(Math.abs(d.body.bottom_y_m - (r.body.Y - r.body.H / 2)) < 1e-12);
  // The drive's crank axis is inside the pod, and the motor lump sits where the packing put the motor.
  assert.ok(d.body.bottom_y_m < r.packing.model.fourBar.O2[1]);
  assert.deepEqual(d.mass.lumps.find((l) => l.name === 'flap motor').pos, r.params.lumpPos['flap motor']);
  // A bigger pod: more frontal drag area and more shell and former mass than the round-3 one.
  const old = buildDesign({ ...p, bodyWidth_m: 0.06, bodyHeight_m: 0.07 }, catalog);
  assert.ok(d.body.CdA_front_m2 > old.body.CdA_front_m2);
  assert.ok(d.bom.structure.find((x) => x.name === 'fuselage formers').mass_g > old.bom.structure.find((x) => x.name === 'fuselage formers').mass_g);
});

test('round-4 options on a control table: step, limiter and alleviation only when asked', async () => {
  const { applyRound4Options } = await import('../../src/design/round3.mjs');
  const d = { motor: { I_cont: 36 }, esc: { I_cont: 30 }, control: { dt: 1 / 480 } };
  applyRound4Options(d, {});
  assert.deepEqual(d.control, { dt: 1 / 480 }, 'rounds 1-3: untouched');
  applyRound4Options(d, { dt: 1 / 960, motorCurrentLimit: { fraction: 0.9 }, gla: { k: 0.1 } });
  assert.equal(d.control.dt, 1 / 960);
  assert.match(d.control.dtRule, /converge/);
  assert.equal(d.control.motorCurrentLimit.setpoint_A, 27);
  assert.deepEqual(d.control.gla, { k: 0.1 });
});
