// The physics' own solids, for the film to draw from. src/design/clearance.js builds the assembly the clearance check
// reads (every part as prisms along body x: rings, capsules and polygons with holes in the (z, y) plane, moving ones
// as functions of the drive pose); the film draws the same solids, so the drawing and the check cannot disagree.
// This module is film-only: it imports the physics and changes nothing in it.
import * as THREE from 'three';
import { fullAssembly } from '../../src/design/clearance.js';
import { driveLayout } from '../../src/design/joints.js';
import { bomLines, resolveTag } from '../../src/design/completeness.js';
import { resolveSteps } from '../../src/design/sequence.js';

const TWO_PI = 2 * Math.PI;

/**
 * The physics' assembly of a design laid out under the clearance rule (v22 on), or null for an older design: every
 * solid its checks read (clearance.js fullAssembly: the assembly and the packed parts with their ties). `parts` is the
 * film's parts index (model.parts); the physics takes the catalogue as a list of its entries.
 *  - Each solid carries its BOM tag (solid.bom, round 16); line(tag) is the BOM line it names (completeness.js
 *    bomLines / resolveTag): { key, kind: 'part' | 'structure', id, name, count }, or null for an untagged solid.
 *  - steps: the design's own assembly sequence (params.layout.sequence) resolved on these solids (sequence.js
 *    resolveSteps): [{ name, dir (unit), dist, threaded, solids: ['part/solid'] }], in the physics' order; null for a
 *    design that states none.
 * -> { lay, parts: [{ id, bom, solids: [{ name, x: [x0, x1], shape | at(q), bom }] }], joins, lines, line(tag),
 *      part(id), solid(id, name), steps, pose }
 */
export function physicsAssembly(design, parts) {
  if (design.params?.layout?.rule !== 'clearance' || !design.structure?.airframe || !design.params?.linkage?.plates) return null;
  const catalog = Object.values(parts);
  const lay = driveLayout(design, catalog);
  const { asm } = fullAssembly(design, catalog, design.params.linkage, lay, design.structure.airframe);
  const byId = new Map(asm.parts.map((p) => [p.id, p]));
  const lines = bomLines(design), byKey = new Map(lines.map((l) => [l.key, l]));
  const seq = design.params.layout.sequence;
  return {
    lay, parts: asm.parts, joins: asm.joins, assumptions: asm.assumptions, lines,
    line: (tag) => byKey.get(resolveTag(lines, tag)) ?? null,
    part: (id) => byId.get(id) ?? null,
    solid: (id, name) => byId.get(id)?.solids.find((s) => s.name === name) ?? null,
    steps: seq?.steps ? resolveSteps(asm, seq).steps : null,
    pose: seq?.pose ?? null,
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
    // a hole wholly inside is a hole; one that crosses the outline (a rail's seat through a lock ear) is cut out of it
    let pts = sh.pts;
    const inner = [];
    for (const h of sh.holes ?? []) {
      const where = holeAgainst(pts, h);
      if (where === 'inside') inner.push(h);
      else if (where === 'crosses') pts = cutDisk(pts, h);
    }
    pts.forEach(([z, y], i) => (i ? s.lineTo(-z, y) : s.moveTo(-z, y)));
    s.closePath();
    for (const h of inner) hole(s, h.c, h.r);
    return s;
  }
  throw new Error(`sectionShape: ${sh.kind}`);
}

const insidePoly = (pts, p) => {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [xi, yi] = pts[i], [xj, yj] = pts[j];
    if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
};

/** Where the segment a -> b meets the circle of a hole: the parameters t strictly inside (0, 1), in order. */
function meets(a, b, h) {
  const d = [b[0] - a[0], b[1] - a[1]], f = [a[0] - h.c[0], a[1] - h.c[1]];
  const A = d[0] * d[0] + d[1] * d[1], B = 2 * (f[0] * d[0] + f[1] * d[1]), C = f[0] * f[0] + f[1] * f[1] - h.r * h.r;
  const disc = B * B - 4 * A * C;
  if (!(A > 0) || disc <= 0) return [];
  const r = Math.sqrt(disc);
  return [(-B - r) / (2 * A), (-B + r) / (2 * A)].filter((t) => t > 1e-9 && t < 1 - 1e-9);
}

/**
 * A round hole against a polygon's outline: 'crosses' when it cuts the outline (a corner inside it, or an edge passing
 * through it: a rail's seat through a lock ear), else 'inside' or 'outside' by its centre (a hole that only touches
 * the outline from inside, as a bracket's bore at its rounded end, is a hole).
 */
export function holeAgainst(pts, h) {
  const inDisk = (p) => Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) < h.r - 1e-7;
  if (pts.some(inDisk) || pts.some((p, i) => meets(p, pts[(i + 1) % pts.length], h).length === 2)) return 'crosses';
  return insidePoly(pts, h.c) ? 'inside' : 'outside';
}

/**
 * A polygon less a disk that cuts its outline: each run of the outline inside the disk is replaced by the disk's arc
 * from where the outline enters it to where it leaves (an edge may enter and leave), the disk kept outside (arc
 * points every 3 degrees, on the circle). -> the cut outline, counter-clockwise
 */
export function cutDisk(pts0, h) {
  const area = pts0.reduce((A, p, i) => A + p[0] * pts0[(i + 1) % pts0.length][1] - pts0[(i + 1) % pts0.length][0] * p[1], 0);
  const pts = area < 0 ? pts0.slice().reverse() : pts0;
  const n = pts.length, inDisk = (p) => Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) < h.r - 1e-7;
  const at = (a, b, t) => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
  const ang = (p) => Math.atan2(p[1] - h.c[1], p[0] - h.c[0]);
  const arc = (e, x, out) => {
    // clockwise about the centre from e to x: the disk on the outline's right, outside the part
    const a0 = ang(e);
    let a1 = ang(x);
    while (a1 > a0) a1 -= 2 * Math.PI;
    const k = Math.max(1, Math.ceil((a0 - a1) / (Math.PI / 60)));
    for (let j = 0; j <= k; j++) { const th = a0 + ((a1 - a0) * j) / k; out.push([h.c[0] + h.r * Math.cos(th), h.c[1] + h.r * Math.sin(th)]); }
  };
  const k0 = pts.findIndex((p) => !inDisk(p));
  if (k0 < 0) return pts;
  const out = [];
  let entry = null;
  for (let i = 0; i < n; i++) {
    const a = pts[(k0 + i) % n], b = pts[(k0 + i + 1) % n], ts = meets(a, b, h);
    if (entry == null) {
      out.push(a);
      if (ts.length === 2) arc(at(a, b, ts[0]), at(a, b, ts[1]), out); // in and out along one edge
      else if (ts.length === 1) entry = at(a, b, ts[0]);
    } else if (ts.length >= 1) {
      arc(entry, at(a, b, ts.at(-1)), out);
      entry = null;
    }
  }
  return out;
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

// ---- the order of assembly: the physics' own sequence, grouped into beats for the screen -----------------------

const SIDE = /\b(R|L|right|left)\b/g;
const sideOf = (id) => {
  const m = id.match(/\b(R|L|right|left)\b/);
  return m ? m[1][0].toUpperCase() : null;
};

/** The parts a step moves, and the BOM lines its solids are tagged with. */
function stepFacts(phys, st) {
  const parts = [...new Set(st.solids.map((t) => t.split('/')[0]))];
  const lines = [...new Set(st.solids.map((t) => {
    const [id, name] = [t.split('/')[0], t.slice(t.indexOf('/') + 1)];
    return phys.line(phys.solid(id, name)?.bom)?.key ?? null;
  }))];
  return { parts, lines };
}

/** Whether every solid of a step is a fixing: a catalogue fastener (a screw, nut or tie), or a tie threaded in place. */
function isFixing(phys, st, lines, catalogue) {
  if (st.threaded) return true;
  return lines.length > 0 && lines.every((k) => {
    const l = phys.lines.find((x) => x.key === k);
    return l && catalogue[l.id]?.category === 'fastener';
  });
}

/** The parts, of a set, that the parts of one step are joined to (phys.joins). */
function joinedOf(phys, parts, others) {
  const mine = new Set(parts), them = new Set(others), out = new Set();
  for (const [a, b] of phys.joins) {
    const [pa, pb] = [a.split('/')[0], b.split('/')[0]];
    if (mine.has(pa) && them.has(pb)) out.add(pb);
    if (mine.has(pb) && them.has(pa)) out.add(pa);
  }
  return out;
}
const joinedTo = (phys, parts, others) => joinedOf(phys, parts, others).size > 0;

/**
 * The physics' steps grouped into beats for the screen, in the physics' order (a beat is a run of consecutive steps;
 * nothing is reordered). A step joins the beat before it when:
 *  - it is a fixing (catalogue screws, nuts or ties, or a threaded tie) that fixes a part of that beat, or continues
 *    a run of fixings of the same kind (the same BOM lines);
 *  - it is the same kind of part as the step before it (the same BOM lines, side aside: a mirrored pair, a run of
 *    formers or surfaces);
 *  - it is one side's sub-assembly going on: every step of the beat and this one move parts of one side (R or L),
 *    and this one is joined to a part of the beat;
 *  - it is a run of bought parts onto one carrier: this step and the beat's first are catalogue parts only, joined to
 *    a part placed before the beat that both rest on.
 * Otherwise it starts a beat. -> [{ name, steps: [step indices] }]
 */
export function screenBeats(phys, catalogue) {
  if (!phys?.steps) return [];
  const facts = phys.steps.map((st) => stepFacts(phys, st));
  const kind = (k) => facts[k].lines.map((l) => (l ?? '').replace(SIDE, '·')).sort().join('|');
  const bought = (k) => facts[k].lines.every((l) => phys.lines.find((x) => x.key === l)?.kind === 'part');
  const beats = [], placed = [];
  phys.steps.forEach((st, k) => {
    const cur = beats.at(-1);
    const f = facts[k], fixing = isFixing(phys, st, f.lines, catalogue);
    let joins = false;
    if (cur) {
      const beatParts = cur.steps.flatMap((j) => facts[j].parts), prev = cur.steps.at(-1);
      const sides = new Set([...beatParts, ...f.parts].map(sideOf));
      const before = placed.slice(0, placed.length - beatParts.length);
      const carrier = () => [...joinedOf(phys, f.parts, before)].some((h) => joinedOf(phys, facts[cur.steps[0]].parts, before).has(h));
      if (fixing) joins = joinedTo(phys, f.parts, beatParts) || (cur.fixing && kind(prev) === kind(k));
      else joins = kind(prev) === kind(k) || (!cur.fixing && sides.size === 1 && !sides.has(null) && joinedTo(phys, f.parts, beatParts))
        || (!cur.fixing && bought(k) && bought(cur.steps[0]) && carrier());
    }
    if (joins) cur.steps.push(k);
    else beats.push({ steps: [k], fixing });
    placed.push(...f.parts);
  });
  return beats.map((b) => ({ name: beatName(phys.steps, b.steps), steps: b.steps }));
}

/** A beat's caption: its one step's name, or its first step's with what follows it. */
function beatName(steps, ks) {
  if (ks.length === 1) return steps[ks[0]].name;
  return `${steps[ks[0]].name} … ${steps[ks.at(-1)].name}`;
}

/**
 * When each step moves, in build progress (0: all apart, 1: all in place): the beats evenly one after another; inside
 * a beat its n steps start one after another at 1/(n + 1) of the beat apart, each moving for 2/(n + 1) of it, so
 * they arrive in the physics' order and the last lands as the beat ends. `extra` film-only beats (the covering, which
 * the sequence exempts) follow the physics' steps. -> [[t0, t1]] per step, then per extra beat.
 */
export function stepTimes(beats, extra = 0) {
  const B = beats.length + extra, out = [];
  beats.forEach((b, k) => {
    const n = b.steps.length;
    b.steps.forEach((s, j) => (out[s] = [(k + j / (n + 1)) / B, (k + (j + 2) / (n + 1)) / B]));
  });
  for (let e = 0; e < extra; e++) out.push([(beats.length + e) / B, (beats.length + e + 1) / B]);
  return out;
}

/** Each step's exploded fraction at build progress p (1: out along its line, 0: in place), easing in and out. */
export const stepFractions = (times, p) => times.map(([t0, t1]) => {
  const s = Math.min(1, Math.max(0, (p - t0) / (t1 - t0)));
  return 1 - s * s * (3 - 2 * s);
});
