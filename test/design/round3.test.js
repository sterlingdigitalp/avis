import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign, couplerSpec, stages } from '../../src/design/build.js';
import { servoAt } from '../../src/design/catalog.js';
import { trainCrankRating, flightRatios } from '../../src/design/trim.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url)));
const byId = (id) => catalog.find((p) => p.id === id);
// Round-2 Larus parameters as they stood (v7), written out so the test does not follow the design file.
const base = {
  candidate: 'larus', span_m: 1.8, AR: 7, taper: 0.6, elbowFrac: 0.5, elbowGain: 0.5, amp_deg: 20, ampMax_deg: 28, phiMean_deg: 5,
  gearRatio: 14.8148, staticMargin: 0.1, sparFrac: 0.35, crank_mm: 12, coupler_mm: 80,
  parts: {
    coupler: 'rod-ec-4mm', motor: 'motor-cobra-c-2820-14', esc: 'esc-hobbywing-skywalker-40a-v2', battery: 'batt-gensace-800-3s-45c', twistServo: 'servo-mks-hv6160', tailServo: 'servo-mks-hv6160', strokeServo: 'servo-mks-hv6160',
    fc: 'fc-ark-v6x', gps: 'gps-3dr-mini-m10136', airspeed: 'aspd-matek-4525', receiver: 'rx-tbs-crossfire-nano', sparInner: 'tube-cst-pultruded-8x7', sparOuter: 'tube-cst-pultruded-8x7', leRod: 'rod-ec-2mm', teRod: 'rod-ec-1-5mm',
    ribMaterial: 'balsa-wood-database', film: 'film-oracover-air-light', keel: 'tube-ec-woven-8x6', gears: ['gear-khk-ds0.8-12', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'], bearing: 'bearing-skf-618-4',
  },
};
const ready = Object.values(base.parts).flat().every(byId) && ['rod-ec-3mm', 'tube-ec-woven-7x5'].every(byId);

/** Second moment of a wing's point masses about the shoulder, sum m s^2 (kg m^2). */
const pointInertia = (d) => d.mass.wing.point.reduce((a, p) => a + p.mass_kg * p.s_m ** 2, 0);

test('twist servo at the elbow: its mass moves inboard, the torque rod is counted on the hand wing, the BOM grows by the rod', { skip: !ready }, () => {
  const legacy = buildDesign(base, catalog);
  const elbow = buildDesign({ ...base, twistServoAt: 'elbow', parts: { ...base.parts, twistRod: 'rod-ec-3mm' } }, catalog);
  const semi = base.span_m / 2 - 0.025;
  const elbowS = semi * 0.5;
  const servo = elbow.mass.wing.point.find((p) => p.name === 'twist servo');
  assert.ok(Math.abs(servo.s_m - elbowS) < 1e-12);
  // Rod: 11 g/m over the hand wing (semi - elbow), two wings, in the structure list and on the wing at 1.15 margin.
  const rodG = 11 * (semi - elbowS);
  const item = elbow.bom.structure.find((s) => s.name.startsWith('twist torque rods'));
  assert.ok(Math.abs(item.mass_g - 2 * rodG) < 1e-9);
  const rodPts = elbow.mass.wing.point.filter((p) => p.name === 'twist torque rod');
  assert.equal(rodPts.length, 2);
  assert.ok(Math.abs(rodPts.reduce((a, p) => a + p.mass_kg, 0) - (rodG / 1000) * 1.15) < 1e-12);
  assert.ok(Math.abs(elbow.bom.structure_g - legacy.bom.structure_g - 2 * rodG) < 1e-9);
  assert.deepEqual(rodPts.map((p) => +p.s_m.toFixed(12)), [elbowS + 0.25 * (semi - elbowS), elbowS + 0.75 * (semi - elbowS)].map((v) => +v.toFixed(12)));
  // The wing's own mass grows by the rod (margin on), nothing else.
  assert.ok(Math.abs(elbow.mass.wing.one_wing_kg - legacy.mass.wing.one_wing_kg - (rodG / 1000) * 1.15) < 1e-12);
  // A rod named without the elbow servo is not fitted.
  const noElbow = buildDesign({ ...base, parts: { ...base.parts, twistRod: 'rod-ec-3mm' } }, catalog);
  assert.equal(noElbow.bom.structure.some((s) => s.name.startsWith('twist torque rods')), false);
  assert.equal(noElbow.mass.wing.one_wing_kg, legacy.mass.wing.one_wing_kg);
  // The point masses' inertia about the shoulder falls (servo from 0.72 m to 0.44 m outweighs the rod).
  assert.ok(pointInertia(elbow) < 0.6 * pointInertia(legacy), `${pointInertia(elbow)} vs ${pointInertia(legacy)}`);
});

test('twist horn linkage k: the wing sees stall torque / k and no-load speed x k; stall current is unchanged', { skip: !ready }, () => {
  const k = 1.6;
  const d = buildDesign({ ...base, twistLinkageRatio: k }, catalog);
  const pub = servoAt(byId('servo-mks-hv6160'), 5);
  assert.ok(Math.abs(d.servos.twist.stall_Nm - pub.stall_Nm / k) < 1e-12);
  assert.ok(Math.abs(d.servos.twist.speed_rad_s - pub.speed_rad_s * k) < 1e-12);
  assert.equal(d.servos.twist.stall_A, pub.stall_A);
  assert.equal(d.servos.twist.servo_stall_Nm, pub.stall_Nm);
  // M8 current at a wing torque tau, stall_A x tau / stall_at_wing, is the servo's own estimate at its torque k tau.
  const tau = 0.3;
  assert.ok(Math.abs((d.servos.twist.stall_A * tau) / d.servos.twist.stall_Nm - (pub.stall_A * k * tau) / pub.stall_Nm) < 1e-12);
  // The tail servo is not geared.
  assert.ok(Math.abs(d.servos.tail.stall_Nm - pub.stall_Nm) < 1e-12);
  assert.throws(() => buildDesign({ ...base, twistLinkageRatio: 0 }, catalog));
});

test('coupler spec: a woven tube carries its bore and its published 650 MPa / 64 GPa; a rod keeps the round-2 fields', { skip: !ready }, () => {
  const t = couplerSpec(byId('tube-ec-woven-7x5'), 80);
  assert.deepEqual([t.od_m, t.id_m, t.length_m, t.E_GPa, t.allowable_MPa], [0.007, 0.005, 0.08, 64, 650]);
  const r = couplerSpec(byId('rod-ec-4mm'), 80);
  assert.deepEqual([r.od_m, r.length_m, r.E_GPa, r.compressive_MPa], [0.004, 0.08, 28, 200]);
  assert.equal(r.id_m, undefined);
  // Missing published fields stay missing (null), never a default number.
  const bare = couplerSpec({ id: 'x', category: 'carbon_rod', specs: { od_mm: 3 } }, 50);
  assert.deepEqual([bare.E_GPa, bare.compressive_MPa], [null, null]);
  const solid = couplerSpec({ id: 'y', category: 'carbon_tube', specs: { od_mm: 6, tensile_strength_lengthways_mpa: 650, tensile_modulus_lengthways_gpa: 64 } }, 50);
  assert.equal(solid.id_m, 0);
});

test('three-stage train: ratio and crank rating from the published KHK ratings, worked by hand', { skip: !ready }, () => {
  const ids = ['gear-khk-ds0.8-20', 'gear-khk-ds0.8-80', 'gear-khk-msga1-18', 'gear-khk-msgb1-40', 'gear-khk-msga1-18', 'gear-khk-msgb1-40'];
  const st = stages(ids, catalog);
  const G = st.reduce((a, s) => a * s.ratio, 1);
  assert.ok(Math.abs(G - 4 * (40 / 18) ** 2) < 1e-12);
  // DS0.8-20 pinion 0.47 N m x 19.75 = 9.284 N m governs: DS0.8-80 2.55 x 4.94 = 12.6, MSGA1-18 6.37 x 2.22 = 14.2.
  assert.ok(Math.abs(trainCrankRating(st) - 0.47 * G) < 1e-9);
});

test('flight ratios: each peak against its rating times the allowance, the worst named', () => {
  const sp = { I: (Math.PI * (0.01 ** 4 - 0.008 ** 4)) / 64, c: 0.005, allowable_MPa: 841 };
  const design = {
    structure: { spar_inner: sp, spar_outer: sp, gear_stages: [{ ratio: 4, pinion_allowable_Nm: 0.47, gear_allowable_Nm: 2.55 }], coupler: { od_m: 0.007, id_m: 0.005, length_m: 0.08, E_GPa: 64, allowable_MPa: 650 } },
    servos: { twist: { stall_Nm: 0.75 } }, motor: { I_cont: 30 }, esc: { I_cont: 40 }, battery: { I_cont: 36 },
  };
  const pk = { crankTorque: 1.5, rootBend: 10, elbowBend: 2, couplerForce: 500, twistTorque: 0.3, motorRms: 15, battMean: 9, dutySat: 0, loadFactor: 3 };
  const r = flightRatios(design, pk, { gear: 0.8 });
  // Gear rating min(0.47 x 4, 2.55) = 1.88 N m; 1.5 / (1.88 x 0.8) = 0.997.
  assert.ok(Math.abs(r.ratios.gear - 1.5 / (1.88 * 0.8)) < 1e-12);
  assert.ok(Math.abs(r.ratios.inner_spar - (10 * 0.005) / sp.I / 1e6 / (841 / 1.5)) < 1e-12);
  assert.ok(Math.abs(r.ratios.motor_current - 0.5) < 1e-12);
  assert.equal(r.worst, 'gear');
  assert.ok(r.ok);
  assert.equal(flightRatios(design, { ...pk, dutySat: 0.5 }).worst, 'motor_speed');
  assert.equal(flightRatios(design, { ...pk, rootBend: NaN }).ok, false);
});

test('flight envelope search: bisection to the frequency where the first rating is reached, the lowest over speeds', async () => {
  const { flightEnvelope } = await import('../../src/design/trim.js');
  const design = { drive: { ampMin: 0.3, ampMax: 0.6 }, control: { trim: [{ V: 10, freq: 2, amp: 0.5 }, { V: 14, freq: 2.5, amp: 0.5 }] } };
  // A synthetic rating on stroke speed amp x f: at most 2.0 at 10 m/s and 1.6 at 14 m/s (the trims fly 1.0 and 1.25).
  const tester = async (V, f, amp) => {
    const lim = V === 10 ? 2.0 : 1.6;
    return { ok: amp * f <= lim, worst: 'gear', pk: { crankTorque: amp * f } };
  };
  const env = await flightEnvelope(design, { speeds: [10, 14], nAmp: 3, tester, iters: 30 });
  assert.equal(env.length, 3);
  for (const e of env) {
    const want = 1.6 / e.amp; // the faster speed's limit is lower and governs
    assert.ok(Math.abs(e.freqMax - want) < 1e-6, `${e.amp}: ${e.freqMax} vs ${want}`);
    assert.equal(e.atV, 14);
    assert.equal(e.limitedBy, 'gear');
  }
  // A stroke that fails at the trim's own stroke speed has no envelope.
  const none = await flightEnvelope(design, { speeds: [10], nAmp: 2, tester: async () => ({ ok: false, worst: 'inner_spar', pk: { crankTorque: 9 } }) });
  assert.deepEqual(none.map((e) => [e.freqMax, e.limitedBy]), [[0, 'inner_spar'], [0, 'inner_spar']]);
});

test('round-3 controller options: twist schedule at 3/4 semispan, energy control defaults and overrides', async () => {
  const { round3ControlOptions } = await import('../../src/design/round3.mjs');
  const o = round3ControlOptions({ geometry: { span_m: 1.8 } });
  assert.ok(Math.abs(o.dynSchedule.rRef_m - 0.675) < 1e-12);
  assert.deepEqual([o.dynSchedule.gain, o.tecs.Vfloor, o.tecs.Vmax, o.tecs.kV, o.tecs.band, o.tecs.kTE], [1, 9.5, 15, 0.4, 1.5, 1]);
  assert.ok(Math.abs(o.dynSchedule.max - (38 * Math.PI) / 180) < 1e-12);
  const p = round3ControlOptions({ geometry: { span_m: 2 } }, { Vfloor: 10, band: 2, dynMax_deg: 30, dynGain: 0.5 });
  assert.deepEqual([p.tecs.Vfloor, p.tecs.band, p.dynSchedule.gain], [10, 2, 0.5]);
  assert.ok(Math.abs(p.dynSchedule.max - Math.PI / 6) < 1e-12);
});
