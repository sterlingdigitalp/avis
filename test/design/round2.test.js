// Round 2: body and tail geometry for the other workstreams, and atomic, versioned design files.
import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, readFileSync, readdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { podAndBoomLoft, loftVolume, bodySectionAt, tailPanels, bodyMesh } from '../../src/physics/geometry.js';
import { writeDesignAtomic, fileSha256 } from '../../src/design/io.js';

test('loft volume: a cylinder of elliptic section is pi/4 w h L; a cone is a third of that', () => {
  const cyl = { sections: [{ x_m: 0, y_m: 0, width_m: 0.06, height_m: 0.08 }, { x_m: -0.5, y_m: 0, width_m: 0.06, height_m: 0.08 }] };
  assert.ok(Math.abs(loftVolume(cyl) - (Math.PI / 4) * 0.06 * 0.08 * 0.5) < 1e-12);
  const cone = { sections: [{ x_m: 0, y_m: 0, width_m: 0, height_m: 0 }, { x_m: -0.3, y_m: 0, width_m: 0.06, height_m: 0.06 }] };
  assert.ok(Math.abs(loftVolume(cone) - ((Math.PI / 4) * 0.06 * 0.06 * 0.3) / 3) < 1e-12);
});

test('pod-and-boom loft runs nose to tail, holds the pod size mid-body and ends at the boom diameter', () => {
  const L = podAndBoomLoft({ noseX_m: 0.15, podLength_m: 0.3, width_m: 0.06, height_m: 0.07, boomOd_m: 0.008, tailEndX_m: -0.6 });
  const s = L.sections;
  assert.equal(s[0].x_m, 0.15);
  assert.equal(s[0].width_m, 0);
  assert.ok(s.every((q, i) => i === 0 || q.x_m < s[i - 1].x_m));
  const mid = bodySectionAt(L, 0.15 - 0.5 * 0.3);
  assert.ok(Math.abs(mid.width_m - 0.06) < 1e-12 && Math.abs(mid.height_m - 0.07) < 1e-12);
  assert.equal(s.at(-1).x_m, -0.6);
  assert.equal(s.at(-1).width_m, 0.008);
  assert.equal(bodyMesh(L, 12).points.length, s.length * 12);
});

test('tail panels: area of a trapezoid and the quarter chord where the flight model puts the tail force', () => {
  const tail = { x_m: -0.5, y_m: 0.02, z_m: 0.005, span_each_m: 0.2, chord_root_m: 0.1, taper: 0.6, dihedral_rad: 0.5 };
  const [r, l] = tailPanels(tail);
  assert.ok(Math.abs(r.area_m2 - 0.5 * (0.1 + 0.06) * 0.2) < 1e-12);
  // root quarter chord at x_m: LE root at x_m + c/4, TE root at x_m - 3c/4
  assert.ok(Math.abs(r.corners[0][0] - (-0.475)) < 1e-12 && Math.abs(r.corners[3][0] - (-0.575)) < 1e-12);
  assert.ok(Math.abs(r.corners[1][1] - (0.02 + 0.2 * Math.sin(0.5))) < 1e-12);
  assert.ok(Math.abs(l.corners[1][2] + r.corners[1][2]) < 1e-12, 'mirror image');
});

test('design files are written atomically with a version that counts rewrites', () => {
  const dir = mkdtempSync(join(tmpdir(), 'avis-io-'));
  const p = join(dir, 'x.json');
  const a = writeDesignAtomic(p, { name: 'x', n: 1 });
  assert.equal(a.version, 1);
  const shaA = fileSha256(p);
  const b = writeDesignAtomic(p, { name: 'x', n: 2 });
  assert.equal(b.version, 2);
  const d = JSON.parse(readFileSync(p, 'utf8'));
  assert.equal(d.supersedes_sha256, shaA);
  assert.deepEqual(readdirSync(dir), ['x.json'], 'no temporary file left behind');
});

import { servoAt, tubeAllowable, servoStallCurrent, becOf, isEligible } from '../../src/design/catalog.js';

test('catalogue ruling: no servo figures outside the published voltage rows; stall current rules', () => {
  const sv = { specs: { performance: [{ voltage_v: 6, torque_nm: 1.0, speed_s_per_60deg: 0.12 }, { voltage_v: 7.4, torque_nm: 1.4, speed_s_per_60deg: 0.1 }], stall_current: [{ voltage_v: null, current_a: 3 }] } };
  assert.equal(servoAt(sv, 5), null, '5 V is below the 6 V row: refused, not clamped');
  const r = servoAt(sv, 6.7);
  assert.ok(Math.abs(r.stall_Nm - 1.2) < 1e-12 && Math.abs(r.stall_A - 3) < 1e-12);
  const two = { specs: { stall_current: [{ voltage_v: 4.8, current_a: 1.5 }, { voltage_v: 6, current_a: 2.1 }] } };
  assert.ok(Math.abs(servoStallCurrent(two, 5.4) - 1.8) < 1e-12);
  assert.equal(servoStallCurrent(two, 4.0), 1.5);
  assert.deepEqual(becOf({ specs: { bec: 'Switch 5V 3A' } }), { volts: 5, amps: 3, kind: 'switching', text: 'Switch 5V 3A' });
  assert.equal(isEligible({ specs: { availability: 'Sorry, this product is no longer available' } }), false);
});

test('catalogue ruling: disclaimed tube strength at half value, metal tube at minimum yield, compressive governs', () => {
  const disclaimed = { category: 'carbon_tube', specs: { tensile_strength_lengthways_mpa: 2400, compressive_strength_mpa: 1600, tensile_modulus_lengthways_gpa: 130, strength_note: "... should be used as a reference only ..." } };
  const a = tubeAllowable(disclaimed);
  assert.equal(a.sigma_MPa, 800);
  assert.equal(a.E_GPa, 130, 'modulus counts in full');
  const ti = { category: 'metal_tube', specs: { yield_strength_mpa: 725, tensile_strength_mpa: 860, elastic_modulus_gpa: 107 } };
  assert.deepEqual([tubeAllowable(ti).sigma_MPa, tubeAllowable(ti).E_GPa], [725, 107]);
  const woven = { category: 'carbon_tube', specs: { tensile_strength_lengthways_mpa: 650, tensile_modulus_lengthways_gpa: 64, strength_note: 'Family value, not per size' } };
  assert.equal(tubeAllowable(woven).sigma_MPa, 650);
});
