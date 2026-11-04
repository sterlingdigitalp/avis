// Round 19 mutation follow-up (twist-drive.js servoLabel, twistMembersSide): the twist drive's round-19 members on the
// built v25 design. References by hand: a sleeve bond carries a bending moment M as axial shear tau = M / (pi r^2 L),
// i.e. an equivalent axial force 4 M / d, so a moment's coefficient times the bond's diameter is 4 on either sleeve;
// the servo mount's reaction about the spar and the bar's elbow force Re e add (both the drive's reaction on the spar,
// round 19 header); a pin bonded through the frame's two lips is bonded over both lips' thickness.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { TWIST_DRIVE, twistLoads, servoLabel } from '../../src/design/twist-drive.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
let built = null;
async function v25() {
  if (built) return built;
  const { buildDesign } = await import('../../src/design/build.js');
  const { round17Set, mergeDeep } = await import('../../src/design/round17.mjs');
  const { solidMassParams } = await import('../../src/design/round11.mjs');
  const v24 = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24.params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  built = buildDesign(p, catalog);
  return built;
}
const member = (d, name) => { const m = d.structure.members.find((q) => q.name === name); assert.ok(m, `no member ${name}`); return m; };
const coef = (m, sec, ch) => m.sections.find((s) => s.name === sec).radial.filter((q) => q.ch === ch).reduce((a, q) => a + q.k, 0);
const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

test('twist mut19: servoLabel is the part number, else the first word of the name, else "servo"', () => {
  assert.equal(servoLabel({ specs: { part_number: 'HS-5065MG' }, name: 'Hitec micro' }), 'HS-5065MG');
  assert.equal(servoLabel({ name: 'MD89MW magnetic-encoder metal gear' }), 'MD89MW');
  assert.equal(servoLabel({ id: 'servo-x' }), 'servo-x');
  assert.equal(servoLabel(undefined), 'servo');
  assert.equal(servoLabel(catalog.find((p) => p.id === 'servo-hitec-md89mw')), 'MD89MW');
});

test('twist mut19: each sleeve bond takes the elbow moment as 4 M / d, and the hand wing\'s whole spanwise force', async () => {
  const d = await v25();
  for (const t of ['R', 'L']) {
    const inner = member(d, `twist joiner ${t} inner sleeve bond`), outer = member(d, `twist joiner ${t} outer sleeve bond`);
    for (const m of [inner, outer]) {
      near(coef(m, 'F', `elbow_${t}`) * (m.dims.d_mm / 1000), 4, 1e-4);
      assert.equal(coef(m, 'F', `root_x_${t}`), 1, `${m.name}: the spanwise force`);
    }
  }
});

test('twist mut19: the inner sleeve and the spar\'s root bond carry the mount\'s reaction plus the bar\'s elbow force about the spar', async () => {
  const d = await v25(), L = twistLoads(d, catalog), want = L.kTm + L.Re * L.G.e;
  assert.ok(L.Re * L.G.e > 0, 'the bar\'s term is not zero (so its sign shows)');
  for (const t of ['R', 'L']) {
    near(coef(member(d, `twist joiner ${t} inner sleeve bond`), 'T', `twist_${t}`), want, 2e-6);
    near(coef(member(d, `twist spar root bond ${t} torsion`), 'T', `twist_${t}`), want, 2e-6);
  }
});

test('twist mut19: the frame pins are bonded over both lips (2 x 1.0 mm), their normal load the servo\'s share of the root force', async () => {
  const d = await v25(), sv = catalog.find((p) => p.id === d.params.parts.twistServo);
  for (const t of ['R', 'L']) {
    assert.equal(member(d, `twist frame pin bonds ${t}`).dims.L_mm, 2 * TWIST_DRIVE.hanger.lip_mm);
    // the stated basis: the servo's mass share of the wing's root force (its mass over about 110 g), simply supported
    // over the case's length and its two lips (M = F L / 4)
    const pins = member(d, `twist frame pins ${t}`), L = twistLoads(d, catalog), caseL = L.G.caseDims[0];
    near(coef(pins, 'M', `root_yz_${t}`), ((sv.mass_g / 1000 / 0.11) * (caseL + 2 * TWIST_DRIVE.hanger.lip_mm / 1000)) / 4, 2e-6);
  }
});
