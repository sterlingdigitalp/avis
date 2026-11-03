// The physics' own solids, for the film to draw from. src/design/clearance.js builds the assembly the clearance check
// reads (every part as prisms along body x: rings, capsules and polygons with holes in the (z, y) plane, moving ones
// as functions of the drive pose); the film draws the same solids, so the drawing and the check cannot disagree.
// This module is film-only: it imports the physics and changes nothing in it.
import * as THREE from 'three';
import { assembly } from '../../src/design/clearance.js';
import { driveLayout } from '../../src/design/joints.js';

const TWO_PI = 2 * Math.PI;

/**
 * The physics' assembly of a design laid out under the clearance rule (v22 on), or null for an older design. `parts`
 * is the film's parts index (model.parts); the physics takes the catalogue as a list of its entries.
 * -> { lay, parts: [{ id, solids: [{ name, x: [x0, x1], shape | at(q) }] }], joins, part(id), solid(id, name) }
 */
export function physicsAssembly(design, parts) {
  if (design.params?.layout?.rule !== 'clearance' || !design.structure?.airframe || !design.params?.linkage?.plates) return null;
  const catalog = Object.values(parts);
  const lay = driveLayout(design, catalog);
  const asm = assembly(design, catalog, design.params.linkage, lay, design.structure.airframe);
  const byId = new Map(asm.parts.map((p) => [p.id, p]));
  return {
    lay, parts: asm.parts, joins: asm.joins, assumptions: asm.assumptions,
    part: (id) => byId.get(id) ?? null,
    solid: (id, name) => byId.get(id)?.solids.find((s) => s.name === name) ?? null,
  };
}

// ---- (z, y) sections as three.js shapes: shape x = -(body z) so the extrusion, turned onto +x, is not mirrored ----

const hole = (s, c, r) => {
  const h = new THREE.Path();
  h.absarc(-c[0], c[1], r, 0, TWO_PI, true);
  s.holes.push(h);
};

/** A section (ring, cap or poly) as a THREE.Shape in (-z, y). */
export function sectionShape(sh) {
  const s = new THREE.Shape();
  if (sh.kind === 'ring') {
    s.absarc(-sh.c[0], sh.c[1], sh.ro, 0, TWO_PI, false);
    if (sh.ri > 0) hole(s, sh.c, sh.ri);
    return s;
  }
  if (sh.kind === 'cap') {
    const a = [-sh.a[0], sh.a[1]], b = [-sh.b[0], sh.b[1]];
    const th = Math.atan2(b[1] - a[1], b[0] - a[0]);
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9) s.absarc(a[0], a[1], sh.r, 0, TWO_PI, false);
    else {
      s.absarc(b[0], b[1], sh.r, th - Math.PI / 2, th + Math.PI / 2, false);
      s.absarc(a[0], a[1], sh.r, th + Math.PI / 2, th + 1.5 * Math.PI, false);
    }
    return s;
  }
  if (sh.kind === 'poly') {
    sh.pts.forEach(([z, y], i) => (i ? s.lineTo(-z, y) : s.moveTo(-z, y)));
    s.closePath();
    for (const h of sh.holes ?? []) hole(s, h.c, h.r);
    return s;
  }
  throw new Error(`sectionShape: ${sh.kind}`);
}

/** A solid's prism in body coordinates (x from x0 to x1), from its section (a moving solid's at a given pose). */
export function solidGeometry(solid, shape = solid.shape, seg = 40) {
  const g = new THREE.ExtrudeGeometry(sectionShape(shape), { depth: solid.x[1] - solid.x[0], bevelEnabled: false, curveSegments: seg });
  g.rotateY(Math.PI / 2);
  g.translate(solid.x[0], 0, 0);
  return g;
}

/** A section moved by d = [dz, dy] in its plane. */
export function shiftSection(sh, d) {
  const m = (p) => [p[0] + d[0], p[1] + d[1]];
  if (sh.kind === 'ring') return { ...sh, c: m(sh.c) };
  if (sh.kind === 'cap') return { ...sh, a: m(sh.a), b: m(sh.b) };
  return { ...sh, pts: sh.pts.map(m), holes: (sh.holes ?? []).map((h) => ({ ...h, c: m(h.c) })) };
}

// ---- the order of assembly ---------------------------------------------------------------------------------------

/**
 * The order the parts go together, as steps of physics part ids (or part/solid) and the straight line each step
 * arrives along (its exploded offset, m, body frame). Read from the v22 layout, and pinned by film/test/assembly-
 * order.test.mjs, which slides every step in along its line against everything already in place:
 *  - the aft plate, then the crank from aft (its pin cantilevered aft of the web);
 *  - the motor (it sits inside the box on the forward plate), then the gear train inside the box, both from forward;
 *  - the right linkage: the coupler onto the crank pin from aft, the shoulder brackets from the side, the rocker hub
 *    between them, the shoulder pin from aft, the horn pin from aft; then the left linkage the same way (the left
 *    coupler passes aft of the right horn pin, the right horn pin aft of the left coupler's place);
 *  - the rails from forward through the bracket sleeves and the aft plate (their lock flanges stop on the forward
 *    plate's aft face), then the forward plate, then (overhung) stage 1 and the encoder onto the stubs forward of it;
 *  - the panels, the boom from aft through both plates, its clamps, the trays from below (onto the plates' pads);
 *  - a part no step names goes on with a part it is joined to (joinedParts).
 * The packed parts, the pod and the wings follow in bird.js (they are not in the physics' assembly).
 */
export function assemblyPlan(phys) {
  if (!phys) return [];
  const ids = phys.parts.map((p) => p.id);
  const P = phys.lay.planes, over = P.stage1 === 'overhung';
  const pick = (re) => ids.filter((id) => re.test(id));
  const side = (tag) => (tag === 'R' ? 1 : -1);
  const linkage = (tag) => {
    const sg = side(tag);
    return [
      { name: `coupler ${tag}`, sub: 'drive', members: pick(new RegExp(`^coupler ${tag}( |$)`)), dir: [-0.05, 0, 0] },
      { name: `shoulder brackets ${tag}`, sub: 'structure', members: pick(new RegExp(`^shoulder (bracket|bushing) ${tag} `)), dir: [0, 0, 0.06 * sg] },
      { name: `rocker ${tag}`, sub: 'drive', members: phys.part(`rocker ${tag}`).solids.filter((s) => !/pin$/.test(s.name)).map((s) => `rocker ${tag}/${s.name}`), dir: [0, 0, 0.06 * sg] },
      { name: `shoulder pin ${tag}`, sub: 'drive', members: [`rocker ${tag}/shoulder pin`], dir: [-0.05, 0, 0] },
      { name: `horn pin ${tag}`, sub: 'drive', members: [`rocker ${tag}/horn pin`], dir: [-0.04, 0, 0] },
    ];
  };
  const steps = [
    { name: 'aft plate', sub: 'drive', members: pick(/^gearbox plate aft$|journal aft \(/), dir: [-0.06, 0, 0] },
    { name: 'crank', sub: 'drive', members: ['crank'], dir: [-0.07, 0, 0] },
    { name: 'motor', sub: 'drive', members: ['motor'], dir: [0.1, 0, 0] },
    { name: 'gear train', sub: 'drive', members: pick(over ? /^gear shaft \d+$|^stage ([2-9]|\d\d+) (wheel|pinion) / : /^gear shaft \d+$|^stage \d+ (wheel|pinion) /), dir: [0.1, 0, 0] },
    ...linkage('R'),
    ...linkage('L'),
    { name: 'rails', sub: 'structure', members: pick(/^shoulder rail /), dir: [0.12, 0, 0] },
    { name: 'forward plate', sub: 'drive', members: pick(/^gearbox plate forward$|journal forward \(/), dir: [0.07, 0, 0] },
    ...(over ? [{ name: 'stage 1 and encoder', sub: 'drive', members: pick(/^stage 1 (wheel|pinion) |^crank encoder /), dir: [0.07, 0, 0] }] : [{ name: 'encoder', sub: 'drive', members: pick(/^crank encoder /), dir: [0.07, 0, 0] }]),
    { name: 'side panels', sub: 'structure', members: pick(/^gearbox side panel /), dirOf: (id) => [0, 0, /right/.test(id) ? 0.05 : -0.05] },
    { name: 'floor', sub: 'structure', members: pick(/^gearbox top panel$/), dir: [0, -0.05, 0] },
    { name: 'tail boom', sub: 'structure', members: pick(/^tail boom$/), dir: [-0.12, 0, 0] },
    { name: 'boom clamps', sub: 'structure', members: pick(/^boom clamp /), dirOf: (id) => [/aft/.test(id) ? -0.05 : 0.05, 0, 0] },
    // the forward and aft trays up onto the plates' pads; anything carried forward of them (a nose tray, a probe) on from forward
    { name: 'trays', sub: 'structure', members: pick(/ tray$/), dirOf: (id) => (/^(forward|aft) tray$/.test(id) ? [0, -0.06, 0] : [0.06, 0, 0]) },
  ];
  joinedParts(phys, steps);
  return steps.filter((s) => s.members.length).map((s) => ({ ...s, dirOf: s.dirOf ?? (() => s.dir) }));
}

/**
 * A part no step names (a later design's fixing, bracket or probe) goes on with the earliest step of a part it is
 * joined to (phys.joins), as one piece with it: the parts joined to it later then go on against it. Any part with no
 * joined part in a step arrives last, from above.
 */
function joinedParts(phys, steps) {
  const at = new Map();
  steps.forEach((s, k) => s.members.forEach((m) => at.set(m.split('/')[0], k)));
  const ids = phys.parts.map((p) => p.id).filter((id) => !at.has(id));
  for (let pass = 0; pass < ids.length; pass++) {
    let moved = false;
    for (const id of ids) {
      if (at.has(id)) continue;
      const hosts = phys.joins.flatMap(([a, b]) => { const [pa, pb] = [a.split('/')[0], b.split('/')[0]]; return pa === id ? [pb] : pb === id ? [pa] : []; }).filter((h) => at.has(h));
      if (!hosts.length) continue;
      const k = Math.min(...hosts.map((h) => at.get(h)));
      steps[k].members.push(id);
      at.set(id, k);
      moved = true;
    }
    if (!moved) break;
  }
  const rest = ids.filter((id) => !at.has(id));
  if (rest.length) steps.push({ name: 'other parts', sub: 'structure', members: rest, dir: [0, 0.06, 0] });
}

/** The step a physics tag (part id or part/solid) arrives in, or -1. */
export function stepOf(plan, tag) {
  const part = tag.split('/')[0];
  let best = -1;
  plan.forEach((s, k) => {
    if (s.members.includes(tag)) best = k;
    else if (best < 0 && s.members.includes(part)) best = k;
  });
  return best;
}

/** Each step's exploded fraction at build progress p (0: all apart, 1: all in place), steps one after another, each
 * easing in and out along its line. */
export const stepFractions = (n, p) => Array.from({ length: n }, (_, k) => {
  const s = Math.min(1, Math.max(0, p * n - k));
  return 1 - s * s * (3 - 2 * s);
});
