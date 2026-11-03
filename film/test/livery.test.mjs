// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// The covering's livery (sims/avis-shared/livery.js LIVERIES): by default the white the BOM records (the catalogue's
// film-oracover-air-light is light scale white, 331-099, and the BOM covers every surface with it), the clear film for
// a BOM without a colour, the gull only ever as a labelled preview; every covering mesh carries its surface's colour
// as vertex colours without a change to the geometry the bake and the GLB depend on.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { LIVERIES, COVERING_LIVERY, COVERING_SURFACES, livery, liveryInBom, defaultLivery, makeMaterials, patchLiveryShader } from '../../sims/avis-shared/materials.js';
import { tipStation, wingSurface, buildBird } from '../../sims/avis-shared/bird.js';
import { nodeBird } from '../lib/bird-node.mjs';

const covering = (bird) => bird.meshes.filter((m) => /^(film_|tail_film_)/.test(m.name) || m.name === 'shell');
const same = (a, b) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) < 1e-6);

test('the white the BOM records by default; the clear film for a BOM without a colour; never the gull', () => {
  assert.equal(COVERING_LIVERY, 'white');
  assert.equal(livery(), null);
  assert.equal(livery('clear'), null);
  assert.throws(() => livery('seagull'), /no livery "seagull"/);
  const { bird, model, design, livery: l } = nodeBird();
  assert.equal(l, LIVERIES.white, 'the catalogue records light scale white and the BOM covers every surface with it');
  const white = LIVERIES.white.surfaces.pod.rgb;
  for (const m of covering(bird)) {
    const c = m.geometry.attributes.color;
    assert.ok(c, m.name);
    for (let i = 0; i < c.count; i += 97) assert.ok(same([c.getX(i), c.getY(i), c.getZ(i)], white), `${m.name} white`);
  }
  // a catalogue entry with no colour: the clear film, no vertex colours
  const film = LIVERIES.white.film, { colour, ...specs } = model.parts[film].specs;
  const bare = { ...model.parts, [film]: { ...model.parts[film], specs } };
  assert.equal(defaultLivery(bare, design.bom), null);
  const clear = nodeBird('larus-gl-2', { livery: null }).bird;
  for (const m of covering(clear)) assert.equal(m.geometry.attributes.color, undefined, m.name);
  const mats = makeMaterials({ textures: false });
  assert.ok(mats.film_wing.transmission > 0.9 && !mats.film_wing.vertexColors);
  // a BOM covering any surface with another film: not the white
  const other = { ...design.bom, structure: design.bom.structure.map((s) => (/fuselage film/.test(s.name) ? { ...s, material_id: 'film-other' } : s)) };
  assert.equal(defaultLivery({ ...model.parts, 'film-other': { category: model.parts[film].category, specs: {} } }, other), null);
  // a BOM with no covering line records no covering; one line on the film is enough
  assert.equal(liveryInBom(LIVERIES.white, model.parts, { structure: [] }), false);
  assert.equal(liveryInBom(LIVERIES.white, model.parts, { structure: design.bom.structure.filter((s) => /fuselage film/.test(s.name)) }), true);
  // the gull's chrome is not in the BOM: never the default, always labelled
  assert.equal(liveryInBom(LIVERIES.gull, model.parts, design.bom), false);
});

// ORACOVER Air Light's published range (maker's 2010 catalogue and shop, read 2026-11-03): light scale white 331-099
// and the light chromes 331-090 (silver), -093 red, -094 yellow, -096 violet, -097 blue; all approx. 18 g/m2, 12 um.
const AIR_LIGHT = { '331-099': 'light scale white', '331-090': 'light chrome', '331-093': 'light chrome red', '331-094': 'light chrome yellow', '331-096': 'light chrome violet', '331-097': 'light chrome blue' };

test('every livery colour is one the maker publishes for the film, with its order number, mass and linear RGB', () => {
  for (const l of Object.values(LIVERIES)) {
    assert.equal(l.film, 'film-oracover-air-light');
    for (const s of COVERING_SURFACES) {
      const c = l.surfaces[s];
      if (c === null && s === 'wing_tip') continue; // no tip colour: the tip keeps its faces' colours
      assert.ok(c && c.rgb.length === 3 && c.rgb.every((v) => v >= 0 && v <= 1), `${l.name}.${s}`);
      assert.equal(AIR_LIGHT[c.order], c.colour, `${l.name}.${s}: ${c.colour} ${c.order} is not in the Air Light range`);
      // the white is the catalogue's own entry (its technical data sheet's 17 g/m2, the ruling's figure); the
      // chromes the maker's catalogue's approx. 18 g/m2
      assert.equal(c.g_m2, c.order === '331-099' ? 17 : 18, `${l.name}.${s}: ${c.g_m2} g/m2`);
      assert.equal(!!c.metallic, /chrome/.test(c.colour), `${l.name}.${s}: the chromes are metallised, the white is not`);
    }
    assert.ok(l.tipFrom > 0.5 && l.tipFrom < 1, `${l.name}: tip from ${l.tipFrom}`);
  }
});

test('white livery (the bird as built): every surface light scale white, the vertex counts unchanged', () => {
  const { livery: l } = nodeBird('larus-gl-2', { livery: 'white' });
  assert.equal(l.name, 'white');
  for (const s of COVERING_SURFACES) assert.equal(l.surfaces[s].order, '331-099');
  checkLivery(l);
});

test('gull livery: upper face, lower face and tip by geometry, tail and pod in theirs, the vertex counts unchanged', () => {
  const { livery: l } = nodeBird('larus-gl-2', { livery: 'gull' });
  assert.equal(l.name, 'gull');
  checkLivery(l);
});

test('a livery with a tip colour paints the tip, both faces, outboard of the rib nearest tipFrom', () => {
  const red = { colour: 'light chrome red', order: '331-093', g_m2: 18, rgb: [0.5, 0.02, 0.03], metallic: true };
  for (const tipFrom of [0.6, 0.8]) checkLivery({ ...LIVERIES.gull, name: 'tipped', tipFrom, surfaces: { ...LIVERIES.gull.surfaces, wing_tip: red } });
});

test('tip station: the rib nearest tipFrom of the wing; a surface: upper face before the LE, lower from it, tip outboard', () => {
  const { model } = nodeBird();
  for (const f of [0.3, 0.55, 0.7, 0.8, 0.9]) {
    const want = f * model.wing.L, got = tipStation(model, f);
    assert.ok(model.ribs.every((s) => Math.abs(s - want) >= Math.abs(got - want) - 1e-12), `tipFrom ${f}: rib ${got} is not the nearest to ${want}`);
  }
  const nSec = 10, sTip = 0.7;
  assert.equal(wingSurface(0, nSec, 0.1, sTip), 'wing_upper');
  assert.equal(wingSurface(nSec - 2, nSec, 0.1, sTip), 'wing_upper');
  assert.equal(wingSurface(nSec - 1, nSec, 0.1, sTip), 'wing_lower', 'the leading-edge vertex takes the lower face');
  assert.equal(wingSurface(2 * nSec - 2, nSec, 0.1, sTip), 'wing_lower');
  assert.equal(wingSurface(3, nSec, sTip, sTip), 'wing_upper', 'the tip rib itself is the last row of the inner colour');
  assert.equal(wingSurface(3, nSec, sTip + 1e-6, sTip), 'wing_tip');
  assert.equal(wingSurface(15, nSec, sTip + 1e-6, sTip), 'wing_tip');
  assert.equal(wingSurface(3, nSec, 5, null), 'wing_upper', 'no tip colour, no tip');
});

function checkLivery(l) {
  const clear = nodeBird('larus-gl-2', { livery: null });
  const { model } = clear;
  const bird = buildBird(model, makeMaterials({ textures: false, livery: l }), { livery: l });
  // same geometry as the clear bird: the bake and the GLB's deform list do not move
  assert.deepEqual(bird.deform.map((m) => [m.name, m.geometry.attributes.position.count]), clear.bird.deform.map((m) => [m.name, m.geometry.attributes.position.count]));
  const mats = makeMaterials({ textures: false, livery: l });
  for (const k of ['film_wing', 'film_shell']) {
    const m = mats[k];
    // pigmented, thin-walled film; full metalness scaled per face by the `metal` attribute (liveryFilm)
    assert.ok(m.vertexColors && m.transmission < 0.5 && m.transmission > 0, k);
    assert.equal(m.metalness, 1, k);
    assert.equal(m.thickness, 0, k);
    assert.equal(typeof m.onBeforeCompile, 'function', k);
  }
  // and the patch reaches the shader three.js compiles: onBeforeCompile sees the physical material's source with its
  // #include chunks unexpanded, so the metal scaling must land in them (it once matched nothing: a dark mirror)
  const sh = patchLiveryShader({ vertexShader: THREE.ShaderLib.physical.vertexShader, fragmentShader: THREE.ShaderLib.physical.fragmentShader });
  assert.ok(sh.vertexShader.includes('vMetal = metal;'));
  assert.ok(sh.fragmentShader.includes('float metalnessFactor = metalness * vMetal;') && !sh.fragmentShader.includes('#include <metalnessmap_fragment>'), 'metalness scaled by the face');
  assert.ok(sh.fragmentShader.includes('material.transmission = transmission * (1.0 - vMetal);') && !sh.fragmentShader.includes('#include <transmission_fragment>'), 'a chrome face passes no light');
  // a tip colour, where a livery has one, starts at a rib; the gull has none (Air Light has no black)
  const S = { ...l.surfaces, wing_tip: l.surfaces.wing_tip ?? { rgb: [-1, -1, -1] } };
  const tipColour = (v) => !!l.surfaces.wing_tip && v;
  const tipped = tipStation(model, l.tipFrom);
  assert.ok(model.ribs.includes(tipped) && Math.abs(tipped / model.wing.L - l.tipFrom) < 0.15, `tip rib at ${(tipped / model.wing.L).toFixed(2)} of the wing`);
  const sTip = tipped;
  const nSec = model.section.length, ringN = 2 * nSec - 1;
  bird.update({ phase: 0, amp: 0, twistL: 0, twistR: 0 });
  let n = { upper: 0, lower: 0, tip: 0 };
  for (const w of Object.values(bird.wings)) {
    for (const f of w.films) {
      const col = f.geometry.attributes.color, pos = f.geometry.attributes.position, metal = f.geometry.attributes.metal;
      assert.equal(col.count, pos.count);
      assert.equal(metal.count, pos.count);
      const rows = f.userData.rows;
      for (let r = 0; r < rows.length; r++) {
        // the station's span position, from the film's own rows against the rib table
        const tip = stationS(model, rows[r]) > sTip + 1e-9;
        for (let c = 0; c < nSec - 1; c++) {
          const up = r * ringN + c, lo = r * ringN + (ringN - 1 - c);
          // the face higher in the segment's own frame (wing level, untwisted) is the upper face
          const [hi, low] = pos.getY(up) >= pos.getY(lo) ? [up, lo] : [lo, up];
          const rgb = (v) => [col.getX(v), col.getY(v), col.getZ(v)];
          if (tipColour(tip)) {
            assert.ok(same(rgb(hi), S.wing_tip.rgb) && same(rgb(low), S.wing_tip.rgb), `${f.name} row ${r}: tip`);
            n.tip++;
          } else {
            assert.ok(same(rgb(hi), S.wing_upper.rgb), `${f.name} row ${r} c ${c}: upper face is ${rgb(hi)}`);
            assert.ok(same(rgb(low), S.wing_lower.rgb), `${f.name} row ${r} c ${c}: lower face is ${rgb(low)}`);
            assert.equal(metal.getX(hi), S.wing_upper.metallic ? 1 : 0);
            assert.equal(metal.getX(low), S.wing_lower.metallic ? 1 : 0);
            n.upper++;
            n.lower++;
          }
        }
      }
    }
  }
  assert.ok(n.upper > 0 && (l.surfaces.wing_tip ? n.tip > 0 && n.tip < n.upper : n.tip === 0), JSON.stringify(n));
  for (const m of covering(bird).filter((m) => !/^film_/.test(m.name))) {
    const want = m.name === 'shell' ? S.pod.rgb : S.tail.rgb;
    const col = m.geometry.attributes.color;
    for (let v = 0; v < col.count; v++) assert.ok(same([col.getX(v), col.getY(v), col.getZ(v)], want), `${m.name} vertex ${v}`);
  }
}

test('a livery is "in the BOM" only when the catalogue records every colour it uses for that film', () => {
  const { model } = nodeBird();
  const l = LIVERIES.gull;
  assert.equal(liveryInBom(l, model.parts), false, 'the catalogue records no colour for the film today');
  assert.equal(liveryInBom(null, model.parts), true, 'the clear film is what the catalogue records');
  const colours = [...new Set(COVERING_SURFACES.filter((s) => l.surfaces[s]).map((s) => l.surfaces[s].colour))];
  const parts = { ...model.parts, [l.film]: { ...model.parts[l.film], specs: { ...model.parts[l.film].specs, colours } } };
  assert.equal(liveryInBom(l, parts), true);
  const partial = { ...parts, [l.film]: { ...parts[l.film], specs: { ...parts[l.film].specs, colours: colours.slice(1) } } };
  assert.equal(liveryInBom(l, partial), false);
  // no catalogue, no entry for the film, or an entry without specs: nothing is recorded
  for (const p of [undefined, {}, { [l.film]: {} }]) assert.equal(liveryInBom(l, p), false, JSON.stringify(p));
  // a one-colour livery is recorded by the entry's single `colour`
  const white = { ...l, surfaces: Object.fromEntries(COVERING_SURFACES.map((s) => [s, l.surfaces.wing_lower])) };
  assert.equal(liveryInBom(white, { [l.film]: { specs: { colour: 'light scale white' } } }), true);
  assert.equal(liveryInBom(white, { [l.film]: { specs: { colour: 'light chrome' } } }), false);
});

/** Span station of film station index i: every rib plus three stations between each pair (bird.js's filmRows 4). */
function stationS(model, i) {
  const fs = [];
  const rows = 4;
  model.ribs.forEach((s, k) => {
    fs.push(s);
    if (k < model.ribs.length - 1) for (let r = 1; r < rows; r++) fs.push(s + ((model.ribs[k + 1] - s) * r) / rows);
  });
  return fs[i];
}

test('the film page labels a livery the catalogue does not record, on every frame, and nothing on the white or the clear film', async () => {
  const { filmLivery } = await import('../../sims/avis-shared/film.js');
  const { model, design } = nodeBird();
  assert.deepEqual(filmLivery({}, model.parts, design.bom), { liv: LIVERIES.white, tag: '' });
  assert.deepEqual(filmLivery({ livery: 'white' }, model.parts, design.bom), { liv: LIVERIES.white, tag: '' });
  assert.deepEqual(filmLivery({ livery: null }, model.parts), { liv: null, tag: '' });
  const on = filmLivery({ livery: 'gull' }, model.parts);
  assert.equal(on.liv, LIVERIES.gull);
  assert.ok(/preview, livery not yet in BOM · gull/.test(on.tag), on.tag);
  const l = LIVERIES.gull;
  const colours = COVERING_SURFACES.filter((s) => l.surfaces[s]).map((s) => l.surfaces[s].colour);
  const recorded = { ...model.parts, [l.film]: { ...model.parts[l.film], specs: { ...model.parts[l.film].specs, colours } } };
  assert.equal(filmLivery({ livery: 'gull' }, recorded).tag, '', 'recorded in the catalogue: no label');
});
