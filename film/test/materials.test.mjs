// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// Pins the wing covering's optical parameters against the physical spec in SPEC.md's refinements and the task that
// set them: Oracover Air Light (catalogue id film-oracover-air-light) is clear, unpigmented PET film, IOR ~1.64,
// reflecting about 11% at normal incidence from its two interfaces and rising toward a mirror at grazing. No colour
// is recorded for it in the catalogue, so the material must render white/uncoloured, never the old blue tint.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { makeMaterials, FILM_IOR, FILM_REFLECTANCE } from '../../sims/avis-shared/materials.js';

const mats = makeMaterials({ textures: false });

test('film IOR and reflectance match the two-interface thin-slab formula for PET', () => {
  assert.ok(Math.abs(FILM_IOR - 1.64) < 1e-9);
  // r = ((n-1)/(n+1))^2 for one interface; two interfaces combine (incoherently) to 2r/(1+r)
  const r1 = ((FILM_IOR - 1) / (FILM_IOR + 1)) ** 2;
  const expected = (2 * r1) / (1 + r1);
  assert.ok(Math.abs(FILM_REFLECTANCE - expected) < 1e-9);
  // about 11% at normal incidence, per the task's own figure
  assert.ok(Math.abs(FILM_REFLECTANCE - 0.111) < 0.002, `FILM_REFLECTANCE=${FILM_REFLECTANCE}`);
});

for (const name of ['film_wing', 'film_shell']) {
  test(`${name}: clear film, not the old blue tint`, () => {
    const mat = mats[name];
    assert.ok(mat instanceof THREE.MeshPhysicalMaterial);
    assert.equal(mat.color.getHex(), 0xffffff, 'base colour must be white: the catalogue records no colour for the film');
    assert.equal(mat.ior, FILM_IOR);
  });

  test(`${name}: specularIntensity * single-interface Fresnel reproduces the two-interface reflectance`, () => {
    const mat = mats[name];
    const r1 = ((mat.ior - 1) / (mat.ior + 1)) ** 2;
    const f0 = r1 * mat.specularIntensity; // this is what three.js's physical shader computes as normal-incidence F0
    assert.ok(Math.abs(f0 - FILM_REFLECTANCE) < 1e-6, `effective F0=${f0}, want ${FILM_REFLECTANCE}`);
  });

  test(`${name}: thin-walled (not a volume to refract through) and smooth but not mirror-flat`, () => {
    const mat = mats[name];
    assert.equal(mat.thickness, 0);
    assert.ok(mat.roughness >= 0.06 && mat.roughness <= 0.08, `roughness=${mat.roughness}, want 0.06-0.08`);
    // transmission < 1 leaves a few percent diffuse haze; it should stay close to, but short of, fully clear
    assert.ok(mat.transmission >= 0.94 && mat.transmission < 1, `transmission=${mat.transmission}`);
    assert.equal(mat.side, THREE.DoubleSide);
  });
}

test('carbon: woven tube is clear-coated (hard, glossy coat), pultruded rod is satin (duller, little or no coat)', () => {
  const woven = mats.carbon_woven, pul = mats.carbon_pultruded;
  assert.ok(woven.clearcoat > pul.clearcoat, 'woven should carry more clearcoat than pultruded');
  assert.ok(woven.clearcoatRoughness < pul.clearcoatRoughness, "woven's coat should be glossier (lower clearcoatRoughness)");
  assert.ok(pul.roughness > woven.roughness, 'pultruded (satin) should read duller than woven (clear-coated) at the base');
});

test('machined/brushed metals carry anisotropy, oriented along the machining', () => {
  // flat brushed parts: the brushed() canvas streaks run along local U, so anisotropyRotation stays 0 (aligned)
  for (const name of ['steel_ground', 'alu_anodised', 'alu_machined', 'servo_case']) {
    const mat = mats[name];
    assert.ok(mat.anisotropy > 0, `${name} should have anisotropy > 0`);
    assert.equal(mat.anisotropyRotation, 0, `${name} should align with its brushed texture (rotation 0)`);
  }
  // turned shafts built with cylX(): CylinderGeometry's V runs along the cylinder axis (the machining direction),
  // not U (circumference), so the default U-aligned anisotropy is rotated 90 deg onto V.
  for (const name of ['steel_shaft', 'titanium']) {
    const mat = mats[name];
    assert.ok(mat.anisotropy > 0, `${name} should have anisotropy > 0`);
    assert.ok(Math.abs(mat.anisotropyRotation - Math.PI / 2) < 1e-9, `${name} should be rotated 90 deg onto the shaft axis`);
  }
});
