// The bird in three.js, every part from the BOM, sized from its catalogue entry and placed per the design and
// model.js. update(state) poses it from the same functions as the physics: kinematics.js for the wings and the
// linkage, gear angles from the crank at the true tooth ratios, servos from the commands in the state.
//
// Node names are the GLB contract (film/export_glb.mjs, env/import_flight.py): `body` is the root, the wing
// segments are wing_R_0, wing_R_1, wing_L_0, wing_L_1, each with its origin on its joint and parented in chain,
// laid flat (stroke 0) at rest. Every other moving node is posed by update(); its local matrix is what the pose
// bake records.
//
// state = { phase, amp, twistL, twistR, tailL, tailR, explode: {wing, drive, ...} 0..1, focus: name|null,
//           shell: 0..1 opacity scale }
import * as THREE from 'three';
import { stationsAt, wingStations, sectionPoint, drivePose, gearAngles, sineSeries, membraneDip, journals, journalDims, sizeOf } from './model.js';
import { segmentAngles } from '../../src/physics/kinematics.js';
import { ACCENT } from './materials.js';
import { buildFrame } from './frame.js';
import { physicsAssembly, screenBeats, stepTimes, stepFractions } from './solids.js';
import { buildFixedFromPhysics, drawRest, drawPosed } from './drive-solids.js';
import { wingRibStations } from '../../src/design/wing-solids.js';

export const SUBSYSTEMS = ['wing', 'drive', 'twist', 'power', 'avionics', 'structure'];
const TWO_PI = 2 * Math.PI;
const mm = (v) => v / 1000;
/** The design's horn pin is the boss, the eye and 2 mm (src/design/linkage.js): the eye 1 mm off the boss face. */
const HORN_GAP = 0.001;
const V3 = THREE.Vector3;

// ---- geometry helpers ------------------------------------------------------------------------------------------

/** Shape geometry (shape in XY, extruded along +Z) turned so the extrusion runs along body +x. */
const alongX = (geo) => geo.rotateY(Math.PI / 2);
/** Cylinder with its axis along +x, centred. */
function cylX(r, len, seg = 24, rTop = r) {
  const g = new THREE.CylinderGeometry(rTop, r, len, seg);
  g.rotateZ(-Math.PI / 2);
  return g;
}
/** Cylinder with its axis along +z, from z = 0 to len. */
function cylZ(r, len, seg = 16) {
  const g = new THREE.CylinderGeometry(r, r, len, seg, 1, false);
  g.rotateX(Math.PI / 2);
  g.translate(0, 0, len / 2);
  return g;
}
/** Hollow tube along +z from 0 to len. */
function tubeZ(ro, ri, len, seg = 28) {
  const s = new THREE.Shape();
  s.absarc(0, 0, ro, 0, TWO_PI, false);
  const h = new THREE.Path();
  h.absarc(0, 0, ri, 0, TWO_PI, true);
  s.holes.push(h);
  return new THREE.ExtrudeGeometry(s, { depth: len, bevelEnabled: false, curveSegments: seg });
}
/** Ring (annulus) along +x, centred, for bearings and eyes. */
function ringX(ro, ri, w, seg = 28) {
  const s = new THREE.Shape();
  s.absarc(0, 0, ro, 0, TWO_PI, false);
  const h = new THREE.Path();
  h.absarc(0, 0, ri, 0, TWO_PI, true);
  s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth: w, bevelEnabled: false, curveSegments: seg });
  g.translate(0, 0, -w / 2);
  return alongX(g);
}
export function roundedBox(sx, sy, sz, r) {
  // the bevel grows the extruded shape by `bs` on every side, so the shape is drawn that much smaller and the box
  // comes out at its catalogue size, sx x sy x sz; on a thin side the bevel shrinks to fit (a stadium end at worst)
  const t = Math.min(r, sz / 4), bs = 0.8 * Math.min(t, sx / 4, sy / 4);
  const ex = sx - 2 * bs, ey = sy - 2 * bs, er = Math.max(1e-5, Math.min(r - bs, ex / 2, ey / 2));
  const s = new THREE.Shape();
  const hx = ex / 2 - er, hy = ey / 2 - er;
  s.moveTo(-hx, -ey / 2);
  s.lineTo(hx, -ey / 2);
  s.absarc(hx, -hy, er, -Math.PI / 2, 0);
  s.lineTo(ex / 2, hy);
  s.absarc(hx, hy, er, 0, Math.PI / 2);
  s.lineTo(-hx, ey / 2);
  s.absarc(-hx, hy, er, Math.PI / 2, Math.PI);
  s.lineTo(-ex / 2, -hy);
  s.absarc(-hx, -hy, er, Math.PI, 1.5 * Math.PI);
  const g = new THREE.ExtrudeGeometry(s, { depth: sz - 2 * t, bevelEnabled: true, bevelSize: bs, bevelThickness: t, bevelSegments: 2, curveSegments: 6 });
  g.translate(0, 0, -(sz - 2 * t) / 2);
  return g;
}

/**
 * Involute spur gear outline: z teeth of module m, 20 degree pressure angle, addendum m, dedendum 1.25 m,
 * tooth 0 centred so that after alongX() it points along body +z. Extruded by the face width along x, centred.
 */
export function gearGeometry(z, m, face, bore, { hubR = 0, hubW = 0, key = 0, pa = (20 * Math.PI) / 180, ro = null } = {}) {
  // the edge bevel grows the outline by bs and the face by bt each side: drawn that much inside, so the gear comes out
  // at its tip circle (the catalogue's outside diameter when given), its bore and its face width
  const bt = Math.min(0.0003, face * 0.05), bs = Math.min(0.00025, m * 0.2);
  const rp = (m * z) / 2, ra = (ro ?? rp + m) - bs, rb = rp * Math.cos(pa);
  const rf = Math.max(rp - 1.25 * m, bore / 2 + 0.6 * m);
  const inv = (a) => Math.tan(a) - a;
  const half = Math.PI / (2 * z) + inv(pa);
  const th = (r) => half - inv(Math.acos(Math.min(1, rb / Math.max(r, rb))));
  const s = new THREE.Shape();
  const pts = [];
  const nF = 6;
  for (let k = 0; k < z; k++) {
    const c = Math.PI + (TWO_PI * k) / z;
    const r0 = Math.max(rf, rb);
    // root, left flank up, tip, right flank down (counter-clockwise)
    pts.push([rf, c - th(r0)]);
    for (let i = 0; i <= nF; i++) {
      const r = r0 + ((ra - r0) * i) / nF;
      pts.push([r, c - th(r)]);
    }
    for (let i = nF; i >= 0; i--) {
      const r = r0 + ((ra - r0) * i) / nF;
      pts.push([r, c + th(r)]);
    }
    pts.push([rf, c + th(r0)]);
  }
  pts.forEach(([r, a], i) => (i ? s.lineTo(r * Math.cos(a), r * Math.sin(a)) : s.moveTo(r * Math.cos(a), r * Math.sin(a))));
  const hole = new THREE.Path();
  if (key > 0) {
    const kw = key / 2, br = bore / 2 + bs;
    const a0 = Math.asin(kw / br);
    hole.moveTo(br * Math.cos(a0), br * Math.sin(a0));
    hole.absarc(0, 0, br, a0, TWO_PI - a0, false);
    hole.lineTo(br + key * 0.45, -kw);
    hole.lineTo(br + key * 0.45, kw);
    hole.closePath();
  } else hole.absarc(0, 0, bore / 2 + bs, 0, TWO_PI, true);
  s.holes.push(hole);
  const g = new THREE.ExtrudeGeometry(s, { depth: face - 2 * bt, bevelEnabled: true, bevelThickness: bt, bevelSize: bs, bevelSegments: 1, curveSegments: 24 });
  g.translate(0, 0, -(face - 2 * bt) / 2);
  alongX(g);
  const parts = [g];
  if (hubR > bore / 2 && hubW > 0) {
    const hub = ringX(hubR, bore / 2, hubW, 40);
    hub.translate(face / 2 + hubW / 2, 0, 0);
    parts.push(hub);
  }
  return parts.length === 1 ? g : mergeGeos(parts);
}

function mergeGeos(list) {
  // minimal merge (positions, normals, uvs) for non-indexed concatenation
  const out = new THREE.BufferGeometry();
  const geos = list.map((g) => (g.index ? g.toNonIndexed() : g));
  for (const name of ['position', 'normal', 'uv']) {
    if (!geos.every((g) => g.attributes[name])) continue;
    const size = geos[0].attributes[name].itemSize;
    const n = geos.reduce((p, g) => p + g.attributes[name].count, 0);
    const arr = new Float32Array(n * size);
    let o = 0;
    for (const g of geos) {
      arr.set(g.attributes[name].array, o);
      o += g.attributes[name].array.length;
    }
    out.setAttribute(name, new THREE.BufferAttribute(arr, size));
  }
  return out;
}

/** A tube whose centreline moves every frame: fixed topology, update(points) rewrites the vertices. */
export function deformTube(n, radius, seg = 8) {
  const g = new THREE.BufferGeometry();
  const pos = new Float32Array(n * seg * 3);
  const idx = [];
  for (let i = 0; i < n - 1; i++) for (let k = 0; k < seg; k++) {
    const a = i * seg + k, b = i * seg + ((k + 1) % seg), c = (i + 1) * seg + k, d = (i + 1) * seg + ((k + 1) % seg);
    idx.push(a, c, b, b, c, d);
  }
  g.setIndex(idx);
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const uv = new Float32Array(n * seg * 2);
  for (let i = 0; i < n; i++) for (let k = 0; k < seg; k++) {
    uv[(i * seg + k) * 2] = i / Math.max(1, n - 1);
    uv[(i * seg + k) * 2 + 1] = k / seg;
  }
  g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  const T = new V3(), N = new V3(), B = new V3(), up = new V3(0, 1, 0), alt = new V3(1, 0, 0);
  g.userData.update = (pts) => {
    for (let i = 0; i < n; i++) {
      const p = pts[i];
      const q = pts[Math.min(n - 1, i + 1)], r = pts[Math.max(0, i - 1)];
      T.set(q[0] - r[0], q[1] - r[1], q[2] - r[2]).normalize();
      N.crossVectors(T, Math.abs(T.dot(up)) > 0.9 ? alt : up).normalize();
      B.crossVectors(T, N);
      for (let k = 0; k < seg; k++) {
        const a = (k / seg) * TWO_PI, ca = Math.cos(a) * radius, sa = Math.sin(a) * radius;
        const j = (i * seg + k) * 3;
        pos[j] = p[0] + N.x * ca + B.x * sa;
        pos[j + 1] = p[1] + N.y * ca + B.y * sa;
        pos[j + 2] = p[2] + N.z * ca + B.z * sa;
      }
    }
    g.attributes.position.needsUpdate = true;
    g.computeVertexNormals();
  };
  return g;
}

/** Static wire along a smooth curve through points. */
function wire(points, r, seg = 40) {
  const curve = new THREE.CatmullRomCurve3(points.map((p) => new V3(...p)), false, 'centripetal');
  return new THREE.TubeGeometry(curve, seg, r, 8, false);
}

// ---- matrix helpers --------------------------------------------------------------------------------------------

const _m = new THREE.Matrix4(), _inv = new THREE.Matrix4();
/** Rotation about body +x by the angle a measured counter-clockwise in the [z, y] plane (seen from behind). */
const rotCCW = (a) => new THREE.Matrix4().makeRotationX(-a);
function basisAt(A, t, b, j) {
  const X = new V3(t[j], t[j + 1], t[j + 2]);
  const Y = new V3(b[j], b[j + 1], b[j + 2]);
  const Z = new V3().crossVectors(X, Y);
  return new THREE.Matrix4().makeBasis(X, Y, Z).setPosition(A[j], A[j + 1], A[j + 2]);
}
function setLocal(obj, bodyMatrix, parentBody) {
  obj.matrixAutoUpdate = false;
  if (parentBody) obj.matrix.copy(_inv.copy(parentBody).invert().multiply(bodyMatrix));
  else obj.matrix.copy(bodyMatrix);
  obj.matrixWorldNeedsUpdate = true;
}

// ---- the bird --------------------------------------------------------------------------------------------------

/** Shared state and helpers for the builders below. */
function makeCtx(model, mats, { filmRows = 4 } = {}) {
  const D = model.design;
  const W = model.wing;
  const P = model.planes;
  const train = model.train;
  const parts = []; // {name, id, role, subsystem, objects}
  const root = new THREE.Group();
  root.name = 'body';
  const meshes = [];
  const accents = [];
  const posed = []; // nodes whose matrix update() sets (the bake records these)
  const deform = []; // meshes whose vertices update() rewrites

  const mesh = (name, geo, mat, parent = root, sub = 'structure', id = null) => {
    const o = new THREE.Mesh(geo, mats[mat]);
    o.name = name;
    o.castShadow = o.receiveShadow = true;
    o.userData.subsystem = sub;
    o.userData.matName = mat;
    if (id) o.userData.partId = id;
    parent.add(o);
    meshes.push(o);
    return o;
  };
  const group = (name, parent = root, sub = 'structure') => {
    const g = new THREE.Group();
    g.name = name;
    g.userData.subsystem = sub;
    parent.add(g);
    return g;
  };
  // one entry per BOM line as the film draws it: a line registered again (one mesh per item) adds its objects
  const register = (name, id, role, subsystem, objects) => {
    const e = parts.find((p) => p.name === name && p.id === id && p.subsystem === subsystem);
    if (e) e.objects.push(...[].concat(objects));
    else parts.push({ name, id, role, subsystem, objects: [].concat(objects) });
  };
  const accentLine = (name, pts, parent, sub, closed = true) => {
    const g = new THREE.BufferGeometry().setFromPoints(pts.map((p) => new V3(...p)));
    const l = closed ? new THREE.LineLoop(g, new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.7, depthTest: false })) : new THREE.Line(g, new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.7, depthTest: false }));
    l.name = name;
    l.renderOrder = 20;
    l.visible = false;
    l.userData.subsystem = sub;
    l.userData.accent = true;
    parent.add(l);
    accents.push(l);
    return l;
  };
  const bomPart = (role) => D.bom.parts.find((p) => p.role === role);
  const structItem = (re) => D.bom.structure.find((s) => re.test(s.name));
  const spec = (id) => model.parts[id]?.specs ?? {};
  // the BOM's bearings and bushings by the joint each carries (model.journals), with their catalogue dimensions
  const jl = journals(D);
  const nShafts = Math.max(0, (train.shafts?.length ?? 1) - 1);
  const journal = (place, shaft = null) => {
    const j = jl.find((x) => x.place === place && (shaft == null || x.shaft === shaft)) ?? pooledShare(jl, place, shaft, nShafts);
    return j && j.count > 0 ? { ...j, dims: journalDims(model.parts[j.id]?.specs) } : null;
  };
  // one ring per journal, flanged when the part is, along x; bearings in steel, polymer bushings in plastic
  const journalMesh = (name, j, parent, sub) => mesh(name, journalGeometry(j.dims), j.kind === 'bushing' ? 'acetal' : 'steel_ground', parent, sub, j.id);
  // geometry helpers for the builders in other modules (frame.js)
  const geo = { cylX, cylZ, tubeZ, ringX, roundedBox, mergeGeos, alongX, wire, convexHull };
  // the physics' own solids (v22 on: a design laid out under the clearance rule), which the fixed parts are drawn from
  const phys = physicsAssembly(D, model.parts);
  // round 16: the ribs where the physics builds them (params.wingStructure: the root rib outside the pod, evenly to
  // the tip), so the covering's stations, its dips and the ribs drawn from the physics' solids agree
  if (phys && D.params?.wingStructure) {
    model.ribs = wingRibStations(D);
    model.ribSt = stationsAt(W, model.ribs);
  }
  return { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows, journal, journalMesh, geo, phys };
}

/** BOM line words for the film's own drawings of things the margin covers (wires and leads, pushrods, servo horns). */
const LEADS = /\b(lead|leads|wire|wiring)\b|\bcables?\b(?! ties?\b)/i, PUSHRODS = /\bpushrods?\b/i, HORNS = /\b(servo|control|output) (horns?|arms?)\b|\bhorns?\b(?!\s+(pins?|boss|bosses)\b)/i;

/** Whether the physics has solids for a BOM line (its key matching re): the film then draws that line from them. */
const physHas = (ctx, re) => !!ctx.phys?.parts.some((p) => p.solids.some((s) => re.test(ctx.phys.line(s.bom)?.key ?? '')));

/**
 * Older designs list their bearings as one pooled "bearing" line; they were drawn two per shoulder hinge, then two per
 * gearbox shaft from the crank's (shaft 0) up, then the rest at the elbows. This is that split, for one place.
 */
function pooledShare(jl, place, shaft, nShafts) {
  const pool = jl.find((x) => x.place === 'pooled');
  if (!pool) return null;
  const N = pool.count;
  const k = place === 'crank' ? 0 : shaft ?? 0;
  const count = place === 'shoulder' ? Math.min(4, N)
    : place === 'crank' || place === 'shaft' ? Math.min(2, Math.max(0, N - 4 - 2 * k))
      : place === 'elbow' ? 2 * Math.min(2, Math.floor(Math.max(0, N - 4 - 2 * nShafts) / 2)) : 0;
  return { ...pool, place, shaft, count };
}

function journalGeometry(dims) {
  const ring = ringX(dims.D / 2, dims.d / 2, dims.B);
  if (!dims.flangeD) return ring;
  const flange = ringX(dims.flangeD / 2, dims.d / 2, dims.flangeB);
  flange.translate(dims.B / 2 - dims.flangeB / 2, 0, 0);
  return mergeGeos([ring, flange]);
}

/** Fuselage: film shell, formers, keel and boom. */
function buildFuselage(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { rings, nAround, points } = model.loftMesh;
  const pos = [];
  const idx = [];
  for (const p of points) pos.push(...p);
  for (let i = 0; i < rings - 1; i++) for (let k = 0; k < nAround; k++) {
    const a = i * nAround + k, b = i * nAround + ((k + 1) % nAround), c = (i + 1) * nAround + k, d = (i + 1) * nAround + ((k + 1) % nAround);
    idx.push(a, b, c, b, d, c);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  const shell = mesh('shell', g, 'film_shell', root, 'structure', structItem(/fuselage film/)?.material_id);
  shell.castShadow = false;
  shell.renderOrder = 10;
  register('fuselage film shell', structItem(/fuselage film/)?.material_id, 'fuselage film shell', 'structure', shell);
  // round 16: the physics' formers (drive-solids.js drawRest), else the film's
  if (physHas(ctx, /formers/)) return buildKeel(ctx);
  // formers: the BOM's count, evenly through the pod, 2 mm balsa rings lightened by half their area
  const fItem = structItem(/formers/);
  const nF = +(fItem?.basis.match(/(\d+)\s*off/)?.[1] ?? 5);
  const sec = model.loft.sections;
  const podEnd = sec.findIndex((s, i) => i > 0 && s.width_m <= sec[sec.length - 1].width_m + 1e-6);
  const x0 = sec[1].x_m - 0.01, x1 = sec[Math.max(1, podEnd - 1)].x_m + 0.01;
  const formers = [];
  for (let i = 0; i < nF; i++) {
    const x = clearFormerX(model, sec, x0 + ((x1 - x0) * i) / Math.max(1, nF - 1), ctx.phys);
    const s = sectionAtX(sec, x);
    if (!s) continue;
    const a = s.width_m / 2 - 0.0008, b = s.height_m / 2 - 0.0008;
    const shp = new THREE.Shape();
    shp.absellipse(0, 0, a, b, 0, TWO_PI, false);
    // lightened by 50% of the area: inner ellipse scaled by sqrt(0.5)
    const h = new THREE.Path();
    h.absellipse(0, 0, a * Math.SQRT1_2, b * Math.SQRT1_2, 0, TWO_PI, true);
    shp.holes.push(h);
    const fg = new THREE.ExtrudeGeometry(shp, { depth: 0.002, bevelEnabled: false, curveSegments: 40 });
    fg.translate(0, 0, -0.001);
    alongX(fg);
    const f = mesh(`former_${i}`, fg, 'balsa', root, 'structure', fItem?.material_id);
    f.position.set(x, s.y_m, 0);
    formers.push(f);
  }
  register('fuselage formers', fItem?.material_id, 'fuselage formers', 'structure', formers);
  buildKeel(ctx);
}

/** Keel and boom: the physics' tail boom where it lays one out (drive-solids.js draws it), else the woven tube, its
 * listed length ending at the tail. */
function buildKeel(ctx) {
  const { model, root, mesh, register, structItem, spec } = ctx;
  const boom = ctx.phys?.solid('tail boom', 'tube');
  if (boom) {
    model._keel = { x0: boom.x[0], x1: boom.x[1], y: boom.shape.c[1], r: boom.shape.ro };
    return;
  }
  const sec = model.loft.sections;
  const kItem = structItem(/keel/);
  const ks = spec(kItem?.material_id);
  const kLen = +(kItem?.basis.match(/x\s*([\d.]+)\s*m/)?.[1] ?? 0.837);
  const xEnd = sec[sec.length - 1].x_m;
  const keelY = -0.0055;
  const kg = tubeZ((ks.od_mm ?? 8) / 2000, (ks.id_mm ?? 6) / 2000, kLen, 32);
  kg.rotateY(-Math.PI / 2); // +z -> -x
  const keel = mesh('keel', kg, 'carbon_woven', root, 'structure', kItem?.material_id);
  keel.position.set(xEnd + kLen, keelY, 0);
  register('keel and tail boom', kItem?.material_id, 'keel and tail boom', 'structure', keel);
  model._keel = { x0: xEnd, x1: xEnd + kLen, y: keelY, r: (ks.od_mm ?? 8) / 2000 };
}

/** Tail: two all-moving V surfaces about their quarter-chord lines. */
function buildTail(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const tailNodes = [];
  const tItem = structItem(/^tail film/) ?? structItem(/tail surfaces/);
  // round 16: the tail's rods and ribs are the physics' solids (drive-solids.js drawRest); the film draws the covering
  const ownRods = !physHas(ctx, /^tail (rods|ribs)/);
  for (const panel of model.tail) {
    const side = panel.side === 'right' ? 1 : -1;
    const [leR, leT, teT, teR] = panel.corners;
    const qr = [leR[0] - 0.25 * panel.chord_root_m, leR[1], leR[2]];
    const qt = [leT[0] - 0.25 * panel.chord_tip_m, leT[1], leT[2]];
    const axis = new V3(qt[0] - qr[0], qt[1] - qr[1], qt[2] - qr[2]).normalize();
    const pivot = group(`tail_${side > 0 ? 'R' : 'L'}`, root, 'structure');
    pivot.userData.axis = axis;
    pivot.userData.origin = new V3(...qr);
    // geometry in body coordinates relative to the pivot origin
    const rel = (p) => [p[0] - qr[0], p[1] - qr[1], p[2] - qr[2]];
    const quad = new THREE.BufferGeometry();
    const v = [leR, leT, teT, teR].map(rel).flat();
    quad.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
    quad.setIndex(side > 0 ? [0, 1, 2, 0, 2, 3] : [0, 2, 1, 0, 3, 2]);
    quad.computeVertexNormals();
    const film = mesh(`tail_film_${side > 0 ? 'R' : 'L'}`, quad, 'film_wing', pivot, 'structure', tItem?.material_id);
    film.renderOrder = 11;
    const rods = [];
    const rodBetween = (name, a, b, r) => {
      const A = new V3(...rel(a)), B = new V3(...rel(b));
      const len = A.distanceTo(B);
      const g = cylZ(r, len, 10);
      const o = mesh(name, g, 'carbon_pultruded', pivot, 'structure', tItem?.material_id);
      o.position.copy(A);
      o.quaternion.setFromUnitVectors(new V3(0, 0, 1), B.clone().sub(A).normalize());
      rods.push(o);
    };
    const tag = side > 0 ? 'R' : 'L';
    if (ownRods) {
      rodBetween(`tail_spar_${tag}`, qr, qt, 0.001);
      rodBetween(`tail_le_${tag}`, leR, leT, 0.00075);
      rodBetween(`tail_te_${tag}`, teR, teT, 0.00075);
      rodBetween(`tail_root_${tag}`, leR, teR, 0.00075);
      rodBetween(`tail_tip_${tag}`, leT, teT, 0.00075);
    }
    tailNodes.push({ node: pivot, side, axis, origin: new V3(...qr) });
    posed.push(pivot);
    register(`tail surface ${tag}`, tItem?.material_id, 'tail surfaces, film, spar and ribs', 'structure', [film, ...rods]);
  }
  return tailNodes;
}

/** Wings: spars, elbow, torque rods, ribs, film, edge rods, twist servos. */
/** Spars of one wing, one per segment, along local z (outboard). */
function wingSpars(ctx, tag, side, segs) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  const spar0 = mesh(`spar_${tag}_0`, tubeZ(sp0.od_mm / 2000, sp0.id_mm / 2000, L0), 'carbon_pultruded', seg0, 'wing', sIn?.material_id);
  const spar1 = mesh(`spar_${tag}_1`, tubeZ(sp1.od_mm / 2000, sp1.id_mm / 2000, L1), 'carbon_pultruded', seg1, 'wing', sOut?.material_id);
  if (side < 0) {
    spar0.scale.z = -1;
    spar1.scale.z = -1;
  }
  spar0.position.x = spar1.position.x = W.spar_x;
  register(`inner spar ${tag}`, sIn?.material_id, 'inner spar', 'wing', spar0);
  register(`outer spar ${tag}`, sOut?.material_id, 'outer spar', 'wing', spar1);
}

/** Elbow knuckle: pin, and bearings if the BOM has any left for it. */
function wingElbow(ctx, tag, side, segs) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // elbow knuckle: a pin through the joint, in bearings or bushings only if the BOM puts any there
  const je = ctx.journal('elbow');
  const elbow = [];
  for (const dx of [-0.007, 0.007].slice(0, (je?.count ?? 0) / 2)) {
    const b = ctx.journalMesh(`elbow_${je.kind}_${tag}_${dx > 0 ? 'f' : 'a'}`, je, seg1, 'wing');
    b.position.set(W.spar_x + dx, 0, 0);
    elbow.push(b);
  }
  const pin = mesh(`elbow_pin_${tag}`, cylX((je?.dims.d ?? 0.004) / 2, 0.02, 12), 'steel_shaft', seg1, 'wing');
  pin.position.set(W.spar_x, 0, 0);
  if (elbow.length) register(`elbow ${je.kind}s ${tag}`, je.id, je.kind, 'wing', elbow);
}

/** Twist torque rod inside the outer spar, and its input arm. */
function wingTorqueRod(ctx, tag, side, segs) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // twist torque rod (if the BOM has one): runs inside the outer spar from the elbow to the tip
  const trI = structItem(/torque rod/);
  if (trI) {
    const rs = spec(trI.material_id);
    const len = +(trI.basis.match(/x\s*([\d.]+)\s*m/)?.[1] ?? L1);
    const rod = mesh(`torque_rod_${tag}`, cylZ((rs.od_mm ?? 3) / 2000, len + 0.006, 12), 'carbon_pultruded', seg1, 'twist', trI.material_id);
    rod.position.set(W.spar_x, 0, side * (L1 - len - 0.003));
    if (side < 0) rod.scale.z = -1;
    register(`twist torque rod ${tag}`, trI.material_id, 'twist torque rod', 'twist', rod);
    // the rod's input arm at the elbow end, turned by the servo horn through the linkage
    const arm = group(`torque_arm_${tag}`, seg1, 'twist');
    arm.position.set(W.spar_x, 0, side * (L1 - len + 0.004));
    mesh(`torque_arm_plate_${tag}`, roundedBox(0.003, 0.02, 0.002, 0.001), 'alu_machined', arm, 'twist').position.set(0, 0.008, 0);
    posed.push(arm);
  }
}

/** Ribs at their stations, parented to their segment. */
function wingRibs(ctx, tag, side, segs) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // ribs
  const ribs = [];
  for (let i = 0; i < ribSt.n; i++) {
    const geo = ribGeometry(model, ribSt, i, sp0.od_mm / 2000);
    const parent = segs[ribSt.seg[i]];
    const r = mesh(`rib_${tag}_${String(i).padStart(2, '0')}`, geo, 'balsa', parent, 'wing', ribI?.material_id);
    r.userData.station = i;
    ribs.push(r);
    posed.push(r);
  }
  register(`ribs ${tag}`, ribI?.material_id, 'ribs', 'wing', ribs);
  return ribs;
}

/** Film pieces, one per segment; vertices are rewritten every frame. */
function wingFilm(ctx, tag, side, segs, filmStations, ringN) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // film pieces, one per segment, vertices rewritten every frame in the segment's frame
  const films = [];
  for (let k = 0; k < 2; k++) {
    const rows = [];
    for (let i = 0; i < filmStations.n; i++) {
      const s = filmStations.s[i];
      if (s >= W.segments[k].s0 - 1e-9 && s <= W.segments[k].s1 + 1e-9) rows.push(i);
    }
    const g = new THREE.BufferGeometry();
    const pos = new Float32Array(rows.length * ringN * 3);
    const idx = [];
    for (let r = 0; r < rows.length - 1; r++) for (let c = 0; c < ringN - 1; c++) {
      const a = r * ringN + c, b = a + 1, cc = a + ringN, d = cc + 1;
      if (side > 0) idx.push(a, cc, b, b, cc, d);
      else idx.push(a, b, cc, b, d, cc);
    }
    g.setIndex(idx);
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const f = mesh(`film_${tag}_${k}`, g, 'film_wing', segs[k], 'wing', filmI?.material_id);
    f.renderOrder = 11;
    f.castShadow = true;
    f.userData.rows = rows;
    f.userData.s = rows.map((i) => filmStations.s[i]);
    f.frustumCulled = false;
    films.push(f);
    deform.push(f);
  }
  register(`wing film ${tag}`, filmI?.material_id, 'wing film', 'wing', films);
  return films;
}

/**
 * Per film station and section point, how far the taut film lies inside the rib profile, per chord (subtracted from
 * the section height, upper and lower face): zero on the ribs, the membrane dip between them (model.membraneDip).
 * Where the dip would carry the film into a spar the film rests on the spar instead, and the faces never close to
 * less than the edge rods' diameter.
 */
function filmDips(ctx, st) {
  const { model, W, structItem, spec } = ctx;
  const sec = model.section, nSec = sec.length, ribs = model.ribs;
  const xs = sec.map((q) => q.xi);
  const bUp = sineSeries(xs, sec.map((q) => q.cam + q.half));
  const bLo = sineSeries(xs, sec.map((q) => q.cam - q.half));
  const sparR = [/inner spar/, /outer spar/].map((re) => (spec(structItem(re)?.material_id).od_mm ?? 8) / 2000);
  const dipUp = new Float64Array(st.n * nSec), dipLo = new Float64Array(st.n * nSec);
  for (let i = 0; i < st.n; i++) {
    let k = 0;
    while (k < ribs.length - 2 && st.s[i] > ribs[k + 1] + 1e-9) k++;
    const gap = ribs[k + 1] - ribs[k];
    const u = gap > 0 ? (st.s[i] - ribs[k]) / gap : 0;
    if (u < 1e-6 || u > 1 - 1e-6) continue; // on a rib: the rib holds the profile
    const c = st.chord[i], sigma = gap / c;
    const r = sparR[st.seg[i]] / c, xSpar = (st.xle[i] - W.spar_x) / c, minHalf = model._teR / c;
    for (let j = 0; j < nSec; j++) {
      const q = sec[j];
      const half = Math.max(q.half, minHalf);
      let up = membraneDip(bUp, q.xi, u, sigma);
      let lo = membraneDip(bLo, q.xi, u, sigma);
      const dx = q.xi - xSpar;
      if (Math.abs(dx) < r) {
        const y = Math.sqrt(r * r - dx * dx);
        up = Math.min(up, Math.max(q.cam + half - y, 0));
        lo = Math.max(lo, Math.min(q.cam - half + y, 0));
      }
      const room = 2 * (half - minHalf);
      if (up - lo > room) {
        const f = room / (up - lo);
        up *= f;
        lo *= f;
      }
      dipUp[i * nSec + j] = up;
      dipLo[i * nSec + j] = lo;
    }
  }
  return { dipUp, dipLo };
}

/** Leading and trailing edge rods as deforming tubes. */
function wingEdgeRods(ctx, tag, side, segs, filmStations) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // LE and TE rods: tubes through the film's leading and trailing edges
  const le = mesh(`le_rod_${tag}`, deformTube(filmStations.n, spec(leI?.material_id).od_mm / 2000), 'carbon_pultruded', root, 'wing', leI?.material_id);
  const te = mesh(`te_rod_${tag}`, deformTube(filmStations.n, spec(teI?.material_id).od_mm / 2000), 'carbon_pultruded', root, 'wing', teI?.material_id);
  le.frustumCulled = te.frustumCulled = false;
  deform.push(le, te);
  register(`leading edge rod ${tag}`, leI?.material_id, 'leading edge rod', 'wing', le);
  register(`trailing edge rod ${tag}`, teI?.material_id, 'trailing edge rod', 'wing', te);
  return { le, te };
}

/** Twist servo at its station, horn on the spar axis. */
function wingTwistServo(ctx, tag, side, segs) {
  const { model, D, W, P, train, root, mesh, group, register, accentLine, bomPart, structItem, spec, posed, deform } = ctx;
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  const [seg0, seg1] = segs;
  const sp0 = spec(sIn?.material_id), sp1 = spec(sOut?.material_id);
  const L0 = W.segments[0].length, L1 = W.segments[1].length;
  // twist servo on the outer spar, horn on the spar axis
  let tsv = null;
  if (model.twistServo) {
    const sv = bomPart('wing twist servo');
    const dims = (spec(sv?.id).dimensions_text_mm ?? '33 x 10 x 35.2').match(/[\d.]+/g).map((v) => +v / 1000);
    const node = group(`twist_servo_${tag}`, seg1, 'twist');
    const caseG = roundedBox(dims[0], dims[1], dims[2], 0.0015);
    const c = mesh(`twist_servo_case_${tag}`, caseG, 'servo_case', node, 'twist', sv?.id);
    const horn = group(`twist_horn_${tag}`, node, 'twist');
    const hornArm = mesh(`twist_horn_arm_${tag}`, roundedBox(0.004, 0.018, 0.002, 0.0015), 'alu_machined', horn, 'twist', sv?.id);
    hornArm.position.set(0, 0.006, 0);
    // case centred chordwise just aft of the spar, servo shaft on the spar axis
    node.userData.s = model.twistServo.s;
    c.position.set(-dims[0] / 2 - 0.004, 0, 0);
    horn.position.set(0, 0, dims[2] / 2 + 0.002);
    tsv = { node, horn, dims };
    posed.push(node, horn);
    register(`wing twist servo ${tag}`, sv?.id, 'wing twist servo', 'twist', [c, hornArm]);
    accentLine(`twist_arc_${tag}`, Array.from({ length: 25 }, (_, i) => {
      const a = -0.6 + (1.2 * i) / 24;
      return [0.03 * Math.cos(a), 0.03 * Math.sin(a), 0];
    }), horn, 'twist', false);
  }
  return tsv;
}

/** Wings: spars, elbow, torque rods, ribs, film, edge rods, twist servos. */
function buildWings(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const wings = {};
  const filmSt = [];
  const sIn = structItem(/inner spar/), sOut = structItem(/outer spar/);
  const leI = structItem(/leading edge rod/), teI = structItem(/trailing edge rod/);
  const ribI = structItem(/^ribs/), filmI = structItem(/wing film/);
  const ribSt = model.ribSt;
  model._teR = (spec(teI?.material_id).od_mm ?? 1.5) / 2000;
  // film stations: every rib, plus filmRows - 1 between each pair
  const fs = [];
  model.ribs.forEach((s, k) => {
    fs.push(s);
    if (k < model.ribs.length - 1) for (let r = 1; r < filmRows; r++) fs.push(s + ((model.ribs[k + 1] - s) * r) / filmRows);
  });
  // and a row where the segments meet (the elbow), which both segments' films share, so no bay is left uncovered:
  // the nearest row between ribs moved onto it (the rows per bay kept), or one more row if it is a rib's
  let s0 = 0;
  for (const g of W.segments.slice(0, -1)) {
    s0 += g.length;
    if (!(s0 > fs[0] && s0 < fs.at(-1)) || fs.some((v) => Math.abs(v - s0) < 1e-9)) continue;
    const k = fs.reduce((b, v, i) => (!model.ribs.includes(v) && (b < 0 || Math.abs(v - s0) < Math.abs(fs[b] - s0)) ? i : b), -1);
    if (k >= 0) fs[k] = s0;
    else fs.push(s0);
  }
  fs.sort((x, y) => x - y);
  const filmStations = stationsAt(W, fs);
  Object.assign(filmStations, filmDips(ctx, filmStations));
  const nSec = model.section.length; // chordwise points per surface, LE..TE
  const ringN = 2 * nSec - 1; // TE upper -> LE -> TE lower
  const own = !physHas(ctx, /^(ribs|inner spar|outer spar|leading edge rod|trailing edge rod)\b/);
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    const seg0 = group(`wing_${tag}_0`, root, 'wing');
    const seg1 = group(`wing_${tag}_1`, seg0, 'wing');
    posed.push(seg0, seg1);
    const segs = [seg0, seg1];
    wingElbow(ctx, tag, side, segs);
    const films = wingFilm(ctx, tag, side, segs, filmStations, ringN);
    // round 16: the wing's structure is the physics' solids (drive-solids.js drawPosed, carried by these segments)
    if (own) {
      wingSpars(ctx, tag, side, segs);
      wingTorqueRod(ctx, tag, side, segs);
      const ribs = wingRibs(ctx, tag, side, segs);
      const { le, te } = wingEdgeRods(ctx, tag, side, segs, filmStations);
      const tsv = wingTwistServo(ctx, tag, side, segs);
      wings[tag] = { side, segs, ribs, films, le, te, tsv };
    } else wings[tag] = { side, segs, ribs: [], films, le: null, te: null, tsv: null };
  }
  filmSt.push(filmStations);
  return { wings, filmSt };
}

/** The rib station where a livery's tip colour starts: the rib nearest `tipFrom` of the wing's length. */
export function tipStation(model, tipFrom) {
  const want = tipFrom * model.wing.L;
  return model.ribs.reduce((best, s) => (Math.abs(s - want) < Math.abs(best - want) ? s : best), model.ribs[0]);
}

/** The covered surface a wing film vertex lies on: ring index c (TE upper -> LE -> TE lower) at span station s; the
 * tip only where the livery colours it (sTip null: no tip colour). */
export function wingSurface(c, nSec, s, sTip) {
  if (sTip != null && s > sTip + 1e-9) return 'wing_tip';
  return c < nSec - 1 ? 'wing_upper' : 'wing_lower';
}

/** A livery as vertex colours (linear RGB) on every covering mesh: wing film by surface, tail film, pod shell. The
 * geometry is unchanged, so the bake and the GLB keep their vertex counts. */
function paintCovering(ctx, l) {
  const { model, wings, filmSt, meshes } = ctx;
  const nSec = model.section.length, ringN = 2 * nSec - 1;
  const st = filmSt[0];
  const sTip = l.surfaces.wing_tip ? tipStation(model, l.tipFrom) : null;
  const paint = (m, surfaceAt) => {
    const n = m.geometry.attributes.position.count;
    const col = new Float32Array(n * 3), metal = new Float32Array(n);
    for (let v = 0; v < n; v++) {
      const c = l.surfaces[surfaceAt(v)];
      col.set(c.rgb, v * 3);
      metal[v] = c.metallic ? 1 : 0;
    }
    m.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
    m.geometry.setAttribute('metal', new THREE.BufferAttribute(metal, 1));
  };
  for (const w of Object.values(wings)) {
    for (const f of w.films) paint(f, (v) => wingSurface(v % ringN, nSec, st.s[f.userData.rows[Math.floor(v / ringN)]], sTip));
  }
  for (const m of meshes) {
    if (/^tail_film_/.test(m.name)) paint(m, () => 'tail');
    if (m.name === 'shell') paint(m, () => 'pod');
  }
}

/** Points `half` either side of a polyline (left, right) at each of its vertices, square to the local direction. */
export function offsetBand(line, half) {
  return line.map(([z, y], i) => {
    const [z1, y1] = line[Math.min(i + 1, line.length - 1)], [z0, y0] = line[Math.max(i - 1, 0)];
    const nx = i < line.length - 1 ? z1 - z : z - z0, ny = i < line.length - 1 ? y1 - y : y - y0;
    const nl = Math.hypot(nx, ny) || 1;
    return [[z - (ny / nl) * half, y + (nx / nl) * half], [z + (ny / nl) * half, y - (nx / nl) * half]];
  });
}

/**
 * The rocker's pin track as a hole (shape coordinates, mirrored for the left wing): a closed band `half` either side
 * of the slot's centreline, or a round hole where the slot has no length (a fixed-stroke drive).
 */
export function slotPath(slot, side, half) {
  const path = new THREE.Path();
  const [z0, y0] = slot[0], [z1, y1] = slot[slot.length - 1];
  if (Math.hypot(z1 - z0, y1 - y0) < 2 * half) {
    path.absarc(-side * z0, y0, half, 0, TWO_PI, true);
    return path;
  }
  const sp = offsetBand(slot, half);
  const ring = [...sp.map((p) => p[0]), ...sp.reverse().map((p) => p[1])];
  ring.forEach(([z, y], i) => (i ? path.lineTo(-side * z, y) : path.moveTo(-side * z, y)));
  path.closePath();
  return path;
}

/** Thickness of the aft and forward gearbox plates: the design's (params.linkage.plates), else model.planes.plateT. */
function plateThickness(ctx) {
  const PL = ctx.D.params?.linkage?.plates;
  return PL ? { aft: mm(PL.t_mm), fwd: mm(PL.t_fwd_mm ?? PL.t_mm) } : { aft: ctx.P.plateT, fwd: ctx.P.plateT };
}

/**
 * The journals in the gearbox plates, aft then forward: the crank shaft (shaft 0) and each gear shaft, as the BOM says,
 * and the motor's outboard bearing in the forward plate when the BOM has one. A journal longer than its plate sits
 * flush with the plate's outer face and reaches into the gearbox, flange in the plate.
 */
function gearboxJournals(ctx, shaftsN) {
  const { P, root, register } = ctx;
  const t = plateThickness(ctx);
  const byId = new Map();
  const put = (jj, name, x, c, aft) => {
    const b = ctx.journalMesh(name, jj, root, 'drive');
    b.position.set(aft ? x - t.aft / 2 + jj.dims.B / 2 : x + t.fwd / 2 - jj.dims.B / 2, c[1], c[0]);
    if (aft) b.rotation.y = Math.PI; // flange on the outer face
    if (!byId.has(jj.id)) byId.set(jj.id, { kind: jj.kind, meshes: [] });
    byId.get(jj.id).meshes.push(b);
  };
  for (let j = 0; j < shaftsN.length - 1; j++) {
    const jj = j === 0 ? ctx.journal('crank') : ctx.journal('shaft', j);
    for (const [name, x] of [['aft', P.plateAft], ['fwd', P.plateFwd]].slice(0, jj?.count ?? 0)) put(jj, `gearbox_${jj.kind}_${j}_${name}`, x, shaftsN[j], name === 'aft');
  }
  const mo = journals(ctx.D).find((x) => x.place === 'other' && /motor/.test(ctx.D.bom.parts.find((p) => p.id === x.id)?.role ?? ''));
  if (mo && shaftsN.length) put({ ...mo, dims: journalDims(ctx.model.parts[mo.id]?.specs) }, `motor_outboard_${mo.kind}`, P.plateFwd, shaftsN[shaftsN.length - 1], false);
  for (const [id, { kind, meshes }] of byId) register(`gearbox ${kind}s`, id, kind, 'drive', meshes);
}

/** A metal gear shaft (params.linkage.gearShafts[j - 1]): its material, and the wheel seat where the design has one. */
function gearShaft(ctx, o, g, j, centre) {
  const { mats, P, train, root, mesh, register } = ctx;
  o.material = mats.alu_machined;
  o.userData.matName = 'alu_machined';
  o.userData.partId = g.material;
  const objs = [o];
  const wheel = train.stages[train.n - 1 - j]?.wheel;
  if (g.seat_od_mm && wheel) {
    const seat = mesh(`shaft_${j}_wheel_seat`, ringX(mm(g.seat_od_mm) / 2, mm(g.d_mm) / 2, wheel.face, 24), 'alu_machined', root, 'drive', g.material);
    seat.position.set(P.stage[train.n - 1 - j], centre[1], centre[0]);
    objs.push(seat);
  }
  register(`gear shaft ${j}`, g.material, 'gear shaft', 'drive', objs);
}

/** The key of a gear on its shaft (params.linkage: the crank's key, each gear shaft's wheel and pinion keys). */
function gearKey(ctx, name, node, bore, gear, shaft, isWheel) {
  const L = ctx.D.params?.linkage;
  const k = !L ? null : isWheel ? (shaft === 0 ? L.crank?.key : L.gearShafts?.[shaft - 1]?.wheelKey) : L.gearShafts?.[shaft - 1]?.pinionKey;
  if (!k) return;
  const o = ctx.mesh(`${name}_key`, new THREE.BoxGeometry(Math.min(mm(k.L_mm), gear.face), mm(k.h_mm), mm(k.b_mm)), 'titanium', node, 'drive', k.material);
  o.position.y = bore / 2 - mm(k.shaft_depth_mm ?? k.h_mm / 2) + mm(k.h_mm) / 2;
  ctx.register(`key ${name}`, k.material, 'key', 'drive', o);
}

/** Bosses round the seats params.linkage.plates.bossed names ('joint/support'), as long as the part, into the gearbox. */
function plateBosses(ctx, PL, shaftsN) {
  const { P, root, mesh, register, structItem } = ctx;
  const t = plateThickness(ctx);
  const objs = [];
  for (const key of PL.bossed) {
    const [joint, support] = key.split('/');
    const k = joint === 'crank shaft' ? 0 : +(joint.match(/gear shaft\s*(\d+)/)?.[1] ?? -1);
    const jj = k === 0 ? ctx.journal('crank') : ctx.journal('shaft', k);
    if (!jj || !shaftsN[k]) continue;
    const aft = support === 'aft';
    const tp = aft ? t.aft : t.fwd, len = jj.dims.B - tp;
    if (len <= 0) continue;
    const o = mesh(`plate_boss_${k}_${support}`, ringX(jj.dims.D / 2 + mm(PL.boss_wall_mm), jj.dims.D / 2, len, 32), 'alu_machined', root, 'drive', structItem(/plate bosses/)?.material_id ?? PL.material);
    o.position.set(aft ? P.plateAft + tp / 2 + len / 2 : P.plateFwd - tp / 2 - len / 2, shaftsN[k][1], shaftsN[k][0]);
    objs.push(o);
  }
  if (objs.length) register('gearbox plate bosses', structItem(/plate bosses/)?.material_id ?? PL.material, 'gearbox plate bosses', 'drive', objs);
}

/**
 * The crank angle encoder (a ring readhead round the crank shaft) and its magnet actuator, just inside the aft
 * gearbox plate where the design's mass lump puts them, when the BOM has them. The actuator's bore is its catalogue
 * figure; neither lists outside dimensions, so each ring is sized from its catalogue mass at an assumed density and
 * width, and the assumption is recorded with the layout's others.
 */
function buildEncoder(ctx, c) {
  const { model, D, P, root, mesh, register } = ctx;
  const enc = D.bom.parts.find((p) => /encoder/.test(p.role) && !/magnet/.test(p.role));
  const mag = D.bom.parts.find((p) => /encoder magnet/.test(p.role));
  const notes = model.layout?.assumptions;
  const w = 0.003, x = P.plateAft + 0.0035;
  const ring = (part, name, rho, ri, mat, what) => {
    const m = model.parts[part.id];
    const ro = Math.sqrt((m?.mass_g ?? 5) / 1000 / (rho * w * Math.PI) + ri * ri);
    notes?.push(`${what}: no outside dimensions in the catalogue; drawn as a ${(2 * ro * 1000).toFixed(0)} mm ring, ${w * 1000} mm wide, from ${m?.mass_g} g at ${rho} kg/m3`);
    const o = mesh(name, ringX(ro, ri, w), mat, root, 'avionics', part.id);
    o.position.set(x, c[1], c[0]);
    register(what, part.id, part.role, 'avionics', o);
    return ro;
  };
  let ri = 0.004;
  if (mag) ri = ring(mag, 'encoder_magnet', 7500, (model.parts[mag.id]?.specs?.inner_diameter_mm ?? 8) / 2000, 'magnet', 'crank encoder magnet') + 0.0015;
  if (enc) ring(enc, 'encoder_readhead', 1850, ri, 'pcb', 'crank angle encoder');
}

/**
 * The machined linkage a design lists in params.linkage (v21 on): per side an aluminium rocker (hub on the shoulder
 * pin between its two bushings, horn arm of the rocker length, horn boss, spar socket), its horn pin, the shoulder
 * pin and the two shoulder housings round the bushings; one-piece coupler links with an eye round a bushing at each
 * end. The coupler's eye rides at its coupler plane (the physics' own, model.gearPlanes); the horn boss sits forward of the eye,
 * HORN_GAP clear of it, as the design's horn pin length (boss + eye + 2 mm) has it.
 */
function buildMetalLinkage(ctx, L) {
  const { model, D, P, wings, drive } = ctx;
  const fb = model.fourBar, law = model.law;
  const sh = ctx.journal('shoulder')?.dims ?? { d: 0.006, D: 0.008, B: 0.006 };
  const span = (D.params?.joints?.shoulderSpan_mm ?? 24) / 1000;
  // a fixed-stroke rocker has one horn position (the drive table's middle entry for a servo drive's slot)
  const e = law.table[Math.floor((law.table.length - 1) / 2)];
  const delta = fb.alpha + e.psiMid - law.phiMean;
  const horn = [e.c * Math.cos(delta), e.c * Math.sin(delta)];
  drive.nodes = {};
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    const x = side > 0 ? P.couplerR : P.couplerL;
    const seg0 = wings[tag].segs[0];
    // with the physics' solids the rocker, its shoulder pin and its horn pin are their own nodes, posed with the wing
    // root (poseWing) but each arriving in its own step of the assembly
    const nodes = ctx.phys ? ['rocker_node', 'shoulder_pin_node', 'horn_pin_node'].map((n) => ctx.group(`${n}_${tag}`, ctx.root, 'drive')) : [seg0, seg0, seg0];
    if (ctx.phys) {
      drive.nodes[tag] = nodes;
      ctx.posed.push(...nodes); // the bake records them (film/bake_pose.mjs), so Blender turns the rocker with the wing
    }
    const k = { tag, side, x, seg0: nodes[0], pinNode: nodes[1], hornNode: nodes[2], horn, span, sh };
    metalRocker(ctx, L, k);
    shoulderJoint(ctx, L, k);
    const cn = metalCoupler(ctx, L, tag);
    drive[tag] = { rocker: ctx.root.getObjectByName(`rocker_${tag}`), coupler: cn, pin: null, x };
  }
}

/** A physics solid's x range (m), or the fallback. */
const solidX = (ctx, part, name, fallback) => ctx.phys?.solid(part, name)?.x ?? fallback;
const tagMesh = (o, ...tags) => ((o.userData.physics = tags), o);

/**
 * One rocker in its wing root's frame (local (x - hinge x, y, side z) for a point (z, y) about the hinge). Along x
 * it takes the physics' own solids where there are any (src/design/clearance.js sideParts: the hub, its relief and
 * collar, the horn stack at params.layout.horns, the arm on the boss), else the v21 stack (boss HORN_GAP forward of
 * the eye).
 */
function metalRocker(ctx, L, { tag, side, x, seg0, hornNode, horn, sh, span }) {
  const { model, P, mesh, register, journal } = ctx;
  const r = L.rocker, fbx = model.fourBar.x, c = Math.hypot(horn[0], horn[1]);
  const eye = journal('coupler')?.dims ?? { d: 0.008, B: P.couplerEye };
  const id = `rocker ${tag}`;
  const b0 = x + P.couplerEye / 2 + HORN_GAP;
  const bossX = solidX(ctx, id, 'horn boss', [b0, b0 + mm(r.boss_len_mm)]);
  const armX = solidX(ctx, id, 'arm', [(bossX[0] + bossX[1]) / 2 - mm(r.arm_t_mm) / 2, (bossX[0] + bossX[1]) / 2 + mm(r.arm_t_mm) / 2]);
  const mid = (v) => (v[0] + v[1]) / 2, len = (v) => v[1] - v[0];
  const at = (o, xx, [z, y]) => (o.position.set(xx - fbx, y, side * z), o);
  // the hub: one ring between the bushings (v21), or the physics' hub, its relief where the eyes pass, and the collar
  const hubSolids = ctx.phys ? ctx.phys.part(id).solids.filter((s) => /^hub|^pin collar/.test(s.name)) : [];
  const hubs = hubSolids.length
    ? hubSolids.map((s) => tagMesh(at(mesh(s.name === 'hub' ? `rocker_${tag}` : `rocker_${tag}_${s.name.replace(/^hub |\s+/g, '')}`, ringX(s.shape.ro, s.shape.ri, len(s.x)), 'alu_machined', seg0, 'drive', r.material), mid(s.x), [0, 0]), `${id}/${s.name}`))
    : [at(mesh(`rocker_${tag}`, ringX(mm(r.hub_od_mm) / 2, sh.d / 2, span - sh.B), 'alu_machined', seg0, 'drive', r.material), fbx, [0, 0])];
  const boss = tagMesh(at(mesh(`rocker_boss_${tag}`, ringX(mm(r.boss_od_mm) / 2, eye.d / 2, len(bossX)), 'alu_machined', seg0, 'drive', r.material), mid(bossX), horn), `${id}/horn boss`);
  const arm = tagMesh(at(mesh(`rocker_arm_${tag}`, new THREE.BoxGeometry(len(armX), c, mm(r.arm_w_mm)), 'alu_machined', seg0, 'drive', r.material), mid(armX), [horn[0] / 2, horn[1] / 2]), `${id}/arm`);
  arm.rotation.x = Math.atan2(side * horn[0], horn[1]); // +y onto the horn
  const so = mm(r.spar_od_mm) / 2, sl = mm(r.socket_len_mm);
  const sock = tagMesh(at(mesh(`rocker_socket_${tag}`, tubeZ(so + mm(r.socket_wall_mm), so, sl), 'alu_machined', seg0, 'drive', r.material), fbx, [0, 0]), `${id}/spar socket`);
  sock.position.z = side * mm(r.hub_od_mm) / 2 - (side > 0 ? 0 : sl);
  register(`rocker ${tag}`, r.material, `rocker ${side > 0 ? 'right' : 'left'}`, 'drive', [...hubs, boss, arm, sock]);
  // horn pin: pressed in the boss, through the coupler eye (the physics' horn pin, else the boss + eye + 2 mm)
  const pinX = solidX(ctx, id, 'horn pin', [bossX[1] - (mm(r.boss_len_mm) + eye.B + 0.002), bossX[1]]);
  const pin = tagMesh(at(mesh(`horn_pin_${tag}`, cylX(eye.d / 2, len(pinX), 16), 'alu_machined', hornNode, 'drive', L.hornPin?.material ?? r.material), mid(pinX), horn), `${id}/horn pin`);
  register(`horn pin ${tag}`, L.hornPin?.material ?? r.material, 'horn pin', 'drive', pin);
}

/**
 * Shoulder pin at the design's span; and, for a design without the physics' solids, the bushings and the housings
 * round them (fixed to the frame). With them, drive-solids.js draws the bushings and the brackets.
 */
function shoulderJoint(ctx, L, { tag, side, seg0, pinNode, sh, span }) {
  const { model, root, mesh, register, journal, journalMesh } = ctx;
  const fb = model.fourBar, jh = journal('shoulder');
  const pinX = solidX(ctx, `rocker ${tag}`, 'shoulder pin', [fb.x - (span + sh.B) / 2, fb.x + (span + sh.B) / 2]);
  const pin = tagMesh(mesh(`hinge_pin_${tag}`, cylX(sh.d / 2, pinX[1] - pinX[0], 16), 'alu_machined', pinNode, 'drive', L.shoulderPin?.material), `rocker ${tag}/shoulder pin`);
  pin.position.x = (pinX[0] + pinX[1]) / 2 - fb.x;
  register(`shoulder pin ${tag}`, L.shoulderPin?.material, 'shoulder pin', 'drive', pin);
  if (ctx.phys) return;
  const bush = [], hous = [];
  for (const dx of [-span / 2, span / 2]) {
    if (jh) {
      const b = journalMesh(`hinge_${jh.kind}_${tag}_${dx > 0 ? 'f' : 'a'}`, jh, seg0, 'drive');
      b.position.set(dx, 0, 0);
      bush.push(b);
    }
    if (!L.shoulderHousing) continue;
    const h = mesh(`shoulder_housing_${tag}_${dx > 0 ? 'f' : 'a'}`, ringX(mm(L.shoulderHousing.od_mm) / 2, sh.D / 2, sh.B), 'alu_machined', root, 'drive', L.shoulderHousing.material);
    h.position.set(fb.x + dx, fb.O4[1], side * fb.O4[0]);
    hous.push(h);
  }
  if (bush.length) register(`hinge ${jh.kind}s ${tag}`, jh.id, jh.kind, 'drive', bush);
  if (hous.length) register(`shoulder housings ${tag}`, L.shoulderHousing.material, 'shoulder housings', 'drive', hous);
}

/** One coupler link, one piece: a round body between two eyes, each eye round a coupler-eye bushing; y along the link. */
function metalCoupler(ctx, L, tag) {
  const { model, root, mesh, group, register, journal, journalMesh, accentLine } = ctx;
  const cp = L.coupler, b = model.law.b, jc = journal('coupler');
  const eye = jc?.dims ?? { D: 0.01, B: ctx.P.couplerEye };
  const cn = group(`coupler_${tag}`, root, 'drive');
  const eo = mm(cp.eye_od_mm) / 2;
  const body = mesh(`coupler_rod_${tag}`, cylZ(mm(cp.body_d_mm) / 2, b - 2 * eo + 0.002, 16), 'alu_machined', cn, 'drive', cp.material);
  body.rotation.x = -Math.PI / 2; // +z -> +y
  body.position.y = eo - 0.001;
  tagMesh(body, `coupler ${tag}/body`);
  const eyes = [], bush = [];
  // round 16: the eyes (counterbored for the bushings' flanges) and the bushings are the physics' solids, drawn in this
  // node by drawPosed; else the film's
  for (const y of ctx.phys?.part(`coupler ${tag} crank bushing`) ? [] : [0, b]) {
    const end = y ? 'horn' : 'crank';
    const o = tagMesh(mesh(`coupler_eye_${tag}_${y ? 'top' : 'bot'}`, ringX(eo, eye.D / 2, eye.B), 'alu_machined', cn, 'drive', cp.material), `coupler ${tag}/${end} eye`);
    o.position.y = y;
    eyes.push(o);
    if (!jc) continue;
    // a flanged bushing's flange is drawn inside its eye's length (the physics checks the bushing as its body)
    const j = tagMesh(journalMesh(`coupler_${jc.kind}_${tag}_${y ? 'top' : 'bot'}`, jc, cn, 'drive'), `coupler ${tag} ${end} bushing`, `coupler ${tag}/${end} eye`);
    j.position.y = y;
    bush.push(j);
  }
  ctx.posed.push(cn);
  register(`coupler ${tag}`, cp.material, 'coupler links', 'drive', [body, ...eyes]);
  if (bush.length) register(`coupler eye ${jc.kind}s ${tag}`, jc.id, jc.kind, 'drive', bush);
  accentLine(`coupler_line_${tag}`, [[0, 0, 0], [0, b, 0]], cn, 'drive', false);
  return cn;
}

/**
 * The one-piece crank of params.linkage.crank: a web web_b wide and web_t thick round the shaft and the pin over the
 * throw, the hollow shaft from the web to 1 mm past the forward plate, the seat of the crank wheel, and the pin
 * cantilevered aft of the web to 1 mm past the left coupler's eye (src/design/linkage.js crankMembers).
 */
function buildMetalCrank(ctx, C) {
  const { model, P, train, root, mesh, group, register, accentLine, drive } = ctx;
  const a = model.law.a, tw = mm(C.web_t_mm), bw = mm(C.web_b_mm);
  if (ctx.phys?.part('crank')) {
    // the physics' crank, whatever its pieces (round 16: the shaft, and the web and pin keyed on its stub): the
    // fixed ones drawn by drawRest, the turning ones by drawPosed in this node, which turns with the crank
    const crank = group('crank', root, 'drive');
    ctx.posed.push(crank);
    accentLine('crank_circle', Array.from({ length: 48 }, (_, i) => [P.crankArm, a * Math.cos((i / 48) * TWO_PI), a * Math.sin((i / 48) * TWO_PI)]), crank, 'drive');
    drive.crank = crank;
    return;
  }
  const pinD = ctx.journal('coupler')?.dims.d ?? 0.008;
  const crank = group('crank', root, 'drive');
  const s = new THREE.Shape();
  s.absarc(0, a, bw / 2, 0, Math.PI, false);
  s.absarc(0, 0, bw / 2, Math.PI, TWO_PI, false);
  s.closePath();
  const wg = new THREE.ExtrudeGeometry(s, { depth: tw, bevelEnabled: false, curveSegments: 24 });
  wg.translate(0, 0, -tw / 2);
  const web = tagMesh(mesh('crank_web', alongX(wg), 'titanium', crank, 'drive', C.material), 'crank/web');
  web.position.x = P.crankArm;
  // the shaft to 1 mm past the forward plate, or (round 14) to the end of the encoder's stub: the physics' own
  const [x0, x1] = solidX(ctx, 'crank', 'shaft', [P.crankArm + tw / 2, P.plateFwd + P.plateT / 2 + 0.001]);
  const shaft = mesh('crank_shaft', ringX(mm(C.shaft_od_mm) / 2, mm(C.shaft_id_mm ?? 0) / 2 + 1e-5, x1 - x0, 24), 'titanium', crank, 'drive', C.material);
  shaft.position.x = (x0 + x1) / 2;
  tagMesh(shaft, 'crank/shaft');
  const wheel = train.stages?.[train.n - 1]?.wheel;
  const objs = [web, shaft];
  if (wheel && C.seat_od_mm) {
    const seat = tagMesh(mesh('crank_wheel_seat', ringX(mm(C.seat_od_mm) / 2, mm(C.shaft_od_mm) / 2, wheel.face, 24), 'titanium', crank, 'drive', C.material), 'crank/wheel seat');
    seat.position.x = P.stage[train.n - 1];
    objs.push(seat);
  }
  const xw = P.crankArm - tw / 2, pinL = xw - P.crankPinEnd;
  const pin = tagMesh(mesh('crank_pin', cylX(pinD / 2, pinL, 16), 'titanium', crank, 'drive', C.material), 'crank/pin');
  pin.position.set(xw - pinL / 2, a, 0);
  objs.push(pin);
  ctx.posed.push(crank);
  register('crank', C.material, 'crank: shaft, web and pin in one piece', 'drive', objs);
  accentLine('crank_circle', Array.from({ length: 48 }, (_, i) => [P.crankArm, a * Math.cos((i / 48) * TWO_PI), a * Math.sin((i / 48) * TWO_PI)]), crank, 'drive');
  drive.crank = crank;
}

/** Rockers on the wing roots and the couplers. */
function buildLinkage(ctx) {
  const L = ctx.D.params?.linkage;
  return L?.rocker && L?.coupler ? buildMetalLinkage(ctx, L) : buildCarbonLinkage(ctx);
}

/** Older designs: carbon-plate rockers (a slot for a stroke servo's pin, else a plain hole) and carbon-tube couplers. */
function buildCarbonLinkage(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const jh = ctx.journal('shoulder'), jc = ctx.journal('coupler');
  const bd = jh ? [jh.dims.d, jh.dims.D, jh.dims.B] : [0.004, 0.009, 0.0025];
  // the coupler pins run in the coupler-eye bushings when the BOM has them
  const pinR = jc ? jc.dims.d / 2 : 0.001;
  const plate = 0.002;
  // rockers: carbon levers on each wing root; with a stroke servo the coupler pin runs in a slot, the locus of the
  // pin over the servo's range (c and psiMid from the drive table), so the mid-stroke angle stays at phiMean; a
  // fixed-stroke drive has one pin position, so a plain hole.
  const slot = law.table.map((e) => {
    const delta = fb.alpha + e.psiMid - law.phiMean;
    return [e.c * Math.cos(delta), e.c * Math.sin(delta)];
  });
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    const seg0 = wings[tag].segs[0];
    const xr = side > 0 ? P.couplerR : P.couplerL;
    const shp = new THREE.Shape();
    // outline: spar socket, then a band 6 mm either side of the slot
    const band = offsetBand(slot, 0.006);
    const poly = [[0.014, -0.006], [0.014, 0.006], ...band.map((p) => p[0]), ...band.reverse().map((p) => p[1])];
    // shape coords (sx, sy) = (z, y) local; mirrored for the left wing
    // shape x = -(local z) so that alongX() maps it back to +local z without mirroring the mesh
    poly.forEach(([z, y], i) => (i ? shp.lineTo(-side * z, y) : shp.moveTo(-side * z, y)));
    shp.closePath();
    const h = new THREE.Path();
    h.absarc(0, 0, bd[1] / 2, 0, TWO_PI, true);
    shp.holes.push(h);
    shp.holes.push(slotPath(slot, side, pinR + 0.0001));
    const g = new THREE.ExtrudeGeometry(shp, { depth: plate, bevelEnabled: false, curveSegments: 16 });
    alongX(g);
    g.translate(xr - plate / 2 - fb.x, 0, 0);
    const rk = mesh(`rocker_${tag}`, g, 'carbon_plate', seg0, 'drive', plateI?.material_id);
    // hinge bearings, aft and forward of the rocker
    const hb = [];
    for (const dx of [-0.006, 0.006].slice(0, (jh?.count ?? 0) / 2)) {
      const b = ctx.journalMesh(`hinge_${jh.kind}_${tag}_${dx > 0 ? 'f' : 'a'}`, jh, seg0, 'drive');
      b.position.set(dx - fb.x + xr, 0, 0);
      hb.push(b);
    }
    const pin = mesh(`hinge_pin_${tag}`, cylX(bd[0] / 2, 0.022, 12), 'steel_shaft', seg0, 'drive');
    pin.position.set(xr - fb.x, 0, 0);
    // the coupler pin, a small steel pin that slides in the slot
    const cpin = mesh(`slot_pin_${tag}`, cylX(pinR, 0.008, 10), 'steel_shaft', seg0, 'drive');
    posed.push(cpin);
    register(`rocker ${tag}`, plateI?.material_id, 'crank, rockers and gearbox plates', 'drive', [rk]);
    if (hb.length) register(`hinge ${jh.kind}s ${tag}`, jh.id, jh.kind, 'drive', hb);
    // coupler: 4 mm rod of length b with an eye at each end
    const cs = spec(coupI?.material_id);
    const cn = group(`coupler_${tag}`, root, 'drive');
    const rodLen = law.b;
    const rod = mesh(`coupler_rod_${tag}`, cylZ(cs.od_mm / 2000, rodLen - 0.008, 16), 'carbon_pultruded', cn, 'drive', coupI?.material_id);
    rod.rotation.x = -Math.PI / 2; // +z -> +y
    rod.position.y = 0.004;
    const eyes = [], eyeBush = [];
    for (const y of [0, rodLen]) {
      const ri = jc ? jc.dims.D / 2 : 0.001;
      const e = mesh(`coupler_eye_${tag}_${y ? 'top' : 'bot'}`, ringX(ri + 0.0022, ri, Math.max(0.003, jc?.dims.B ?? 0)), 'alu_machined', cn, 'drive');
      e.position.y = y;
      eyes.push(e);
      if (jc) {
        const b = ctx.journalMesh(`coupler_${jc.kind}_${tag}_${y ? 'top' : 'bot'}`, jc, cn, 'drive');
        b.position.y = y;
        eyeBush.push(b);
      }
    }
    posed.push(cn);
    register(`coupler ${tag}`, coupI?.material_id, 'couplers', 'drive', [rod, ...eyes]);
    if (eyeBush.length) register(`coupler eye ${jc.kind}s ${tag}`, jc.id, jc.kind, 'drive', eyeBush);
    accentLine(`coupler_line_${tag}`, [[0, 0, 0], [0, rodLen, 0]], cn, 'drive', false);
    drive[tag] = { rocker: rk, coupler: cn, pin: cpin, x: xr };
  }
}

/** Crank disc and pin. */
function buildCrank(ctx) {
  const C = ctx.D.params?.linkage?.crank;
  if (C?.web_t_mm) return buildMetalCrank(ctx, C);
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const jk = ctx.journal('crank'), jc = ctx.journal('coupler');
  const bd = jk ? [jk.dims.d, jk.dims.D, jk.dims.B] : [0.004, 0.009, 0.0025];
  const plate = 0.002;
  // crank disc and pin
  const crank = group('crank', root, 'drive');
  const disc = new THREE.Shape();
  disc.absarc(0, 0, law.a + 0.006, 0, TWO_PI, false);
  const dh = new THREE.Path();
  dh.absarc(0, 0, bd[0] / 2, 0, TWO_PI, true);
  disc.holes.push(dh);
  const dg = new THREE.ExtrudeGeometry(disc, { depth: plate, bevelEnabled: false, curveSegments: 40 });
  dg.translate(0, 0, -plate / 2);
  alongX(dg);
  const crankDisc = mesh('crank_disc', dg, 'carbon_plate', crank, 'drive', plateI?.material_id);
  crankDisc.position.x = P.crankArm;
  const pinLen = P.crankArm - P.couplerL + 0.004;
  const crankPin = mesh('crank_pin', cylX(jc ? jc.dims.d / 2 : 0.001, pinLen, 12), 'steel_shaft', crank, 'drive');
  crankPin.position.set(P.crankArm - pinLen / 2 + 0.001, law.a, 0); // pin at +y in the crank frame
  posed.push(crank);
  register('crank', plateI?.material_id, 'crank, rockers and gearbox plates', 'drive', [crankDisc, crankPin]);
  accentLine('crank_circle', Array.from({ length: 48 }, (_, i) => [P.crankArm, law.a * Math.cos((i / 48) * TWO_PI), law.a * Math.sin((i / 48) * TWO_PI)]), crank, 'drive');
  drive.crank = crank;
}

/** Gears, shafts, gearbox plates, bearings and fasteners. */
function buildGears(ctx) {
  ctx.drive.gears = [];
  const shaftsN = ctx.train.shafts ?? [];
  if (!ctx.train.links?.length) return;
  gearMeshes(ctx, shaftsN);
  // a design the physics lays out: every fixed part from its solids (drive-solids.js)
  if (ctx.phys) return buildFixedFromPhysics(ctx);
  driveShafts(ctx, shaftsN);
  gearboxPlates(ctx, shaftsN);
  const PL = ctx.D.params?.linkage?.plates;
  if (PL?.bossed?.length) plateBosses(ctx, PL, shaftsN);
  gearboxJournals(ctx, shaftsN);
  buildEncoder(ctx, shaftsN[0]);
  // a design with an airframe draws its own fasteners and panels (frame.js); older ones get the plate standoffs
  if (!ctx.D.structure?.airframe) plateStandoffs(ctx, shaftsN);
}

/** Each shaft's journal (the crank is shaft 0): bore for the shaft and gears, outside diameter for the plate hole. */
const shaftJournal = (ctx, k) => (k === 0 ? ctx.journal('crank') : ctx.journal('shaft', k))?.dims ?? { d: 0.004, D: 0.009 };

/** Gears: one node per gear, named by stage (stage 1 is the motor stage), at its plane on its shaft. */
function gearMeshes(ctx, shaftsN) {
  const { model, P, train, root, posed, mesh, group, register, accentLine, spec, drive } = ctx;
  const shaftPart = (k) => (k === 0 ? 'crank' : k === shaftsN.length - 1 ? 'motor' : `gear shaft ${k}`);
  const gearMesh = (name, gear, centre, x, shaftIndex, physId) => {
    const node = group(name, root, 'drive');
    node.position.set(x, centre[1], centre[0]);
    const isSteel = /SCM|steel/i.test(gear.material);
    // the physics draws every catalogue hub (src/design/clearance.js gearParts); older designs, the steel gears' only
    const hub = gear.hub && gear.hubW > 0 && (isSteel || !!ctx.phys);
    const bore = gear.bore || (shaftIndex === shaftsN.length - 1 ? (spec(model.design.motor.id).shaft_mm ?? 5) / 1000 : shaftJournal(ctx, shaftIndex).d);
    const key = hub ? (bore >= 0.012 ? 0.005 : 0.003) : 0;
    const g = gearGeometry(gear.z, gear.m, gear.face, bore, { hubR: hub ? gear.hub / 2 : 0, hubW: hub ? gear.hubW : 0, key, ro: gear.ro });
    const o = tagMesh(mesh(`${name}_teeth`, g, isSteel ? 'steel_ground' : 'acetal', node, 'drive', gear.id), physId);
    // round 16: the keys are the physics' own solids (drive-solids.js drawRest), else the film's in the gear
    if (!physHas(ctx, /^key,/)) gearKey(ctx, name, node, bore, gear, shaftIndex, /wheel|crank/.test(name));
    const keyMesh = node.getObjectByName(`${name}_key`);
    // a key sits half in the gear's keyway, half in its shaft's keyseat
    if (keyMesh) tagMesh(keyMesh, physId, shaftPart(shaftIndex));
    posed.push(node);
    register(`gear ${gear.z}T m${+(gear.m * 1000).toFixed(2)}`, gear.id, 'gear', 'drive', o);
    accentLine(`${name}_pitch`, Array.from({ length: 64 }, (_, i) => [gear.face / 2 + 0.0004, gear.rp * Math.cos((i / 64) * TWO_PI), gear.rp * Math.sin((i / 64) * TWO_PI)]), node, 'drive');
    drive.gears.push({ node, shaft: shaftIndex });
  };
  train.links.forEach((_, j) => {
    const si = train.n - 1 - j;
    const st = train.stages[si];
    gearMesh(j === 0 ? 'gear_crank' : `gear_s${si + 1}_wheel`, st.wheel, shaftsN[j], P.stage[si], j, `stage ${si + 1} wheel ${st.wheel.id}`);
    gearMesh(si === 0 ? 'gear_motor' : `gear_s${si + 1}_pinion`, st.pinion, shaftsN[j + 1], P.stage[si], j + 1, `stage ${si + 1} pinion ${st.pinion.id}`);
  });
  drive.gCrank = root.getObjectByName('gear_crank');
}

/** Shafts plate to plate, 1 mm proud of each: the crank's (unless the crank is one piece with it) and each gear shaft. */
function driveShafts(ctx, shaftsN) {
  const { D, P, root, mesh } = ctx;
  const shaft = (name, centre, x0, x1, r) => {
    const o = mesh(name, cylX(r, x1 - x0, 16), 'steel_shaft', root, 'drive');
    o.position.set((x0 + x1) / 2, centre[1], centre[0]);
    return o;
  };
  if (!D.params?.linkage?.crank?.web_t_mm) shaft('shaft_crank', shaftsN[0], P.crankArm - 0.001, P.plateFwd + 0.002, shaftJournal(ctx, 0).d / 2);
  const gs = D.params?.linkage?.gearShafts ?? [];
  for (let j = 1; j < shaftsN.length - 1; j++) {
    const r = gs[j - 1]?.d_mm ? mm(gs[j - 1].d_mm) / 2 : shaftJournal(ctx, j).d / 2;
    const o = shaft(`shaft_${j}`, shaftsN[j], P.plateAft - P.plateT / 2 - 0.001, P.plateFwd + P.plateT / 2 + 0.001, r);
    if (gs[j - 1]?.material) gearShaft(ctx, o, gs[j - 1], j, shaftsN[j]);
  }
}

/** Gearbox plates: a rounded hull around every shaft with a seat for each, at the design's material and thicknesses. */
function gearboxPlates(ctx, shaftsN) {
  const { D, P, root, mesh, register, structItem } = ctx;
  const r = 0.0085;
  const ring = [];
  for (const c of shaftsN) for (let k = 0; k < 24; k++) ring.push([c[0] + r * Math.cos((k / 24) * TWO_PI), c[1] + r * Math.sin((k / 24) * TWO_PI)]);
  const hull = convexHull(ring);
  const plateShape = () => {
    const sh = new THREE.Shape();
    // shape x = -(body z) so alongX() maps it back without mirroring
    hull.forEach(([z, y], i) => (i ? sh.lineTo(-z, y) : sh.moveTo(-z, y)));
    sh.closePath();
    shaftsN.forEach((c, i) => {
      const h = new THREE.Path();
      h.absarc(-c[0], c[1], i === shaftsN.length - 1 ? 0.0035 : shaftJournal(ctx, i).D / 2, 0, TWO_PI, true);
      sh.holes.push(h);
    });
    return sh;
  };
  const PL = D.params?.linkage?.plates;
  const t = plateThickness(ctx);
  const item = structItem(/^gearbox plates/) ?? structItem(/crank, rockers/);
  const plates = [['plate_aft', P.plateAft, t.aft], ['plate_fwd', P.plateFwd, t.fwd]].map(([name, x, th]) => {
    const g = new THREE.ExtrudeGeometry(plateShape(), { depth: th, bevelEnabled: false, curveSegments: 24 });
    g.translate(0, 0, -th / 2);
    alongX(g);
    const o = mesh(name, g, PL ? 'alu_machined' : 'carbon_plate', root, 'drive', item?.material_id);
    o.position.x = x;
    return o;
  });
  register('gearbox plates', item?.material_id, 'gearbox plates', 'drive', plates);
}

/** Older designs: four titanium standoffs with screw heads holding the plates apart, inside the 15% margin. */
function plateStandoffs(ctx, shaftsN) {
  const { P, root, mesh, register } = ctx;
  const standoffs = [];
  const lowest = shaftsN.reduce((m, c) => (c[1] < m[1] ? c : m), shaftsN[0]);
  const highest = shaftsN.reduce((m, c) => (c[1] > m[1] ? c : m), shaftsN[0]);
  for (const [dz, y] of [[-0.0062, lowest[1] - 0.004], [0.0062, lowest[1] - 0.004], [-0.0062, highest[1] + 0.004], [0.0062, highest[1] + 0.004]]) {
    const zc = (dz < 0 ? -1 : 1) * 0.0062 + (y < 0 ? lowest[0] : highest[0]);
    const o = mesh(`standoff_${standoffs.length}`, cylX(0.001, P.plateFwd - P.plateAft + 0.004, 10), 'titanium', root, 'drive');
    o.position.set((P.plateAft + P.plateFwd) / 2, y, zc);
    standoffs.push(o);
    for (const x of [P.plateAft - 0.0022, P.plateFwd + 0.0022]) {
      const head = mesh(`screw_head_${standoffs.length}_${x > P.plateAft ? 'f' : 'a'}`, cylX(0.0018, 0.0012, 12), 'titanium', root, 'drive');
      head.position.set(x, y, zc);
      standoffs.push(head);
    }
  }
  register('fasteners (15% margin)', null, 'fasteners', 'drive', standoffs);
}

/** The flap motor: outrunner bell, mount, shaft to the pinion. */
function buildMotor(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const plate = 0.002;
  // motor: outrunner bell, mount, shaft to the pinion
  const mp = model.layout.parts.find((p) => p.name === 'motor');
  const motor = group('motor', root, 'drive');
  // the physics' can and shaft where it lays the drive out (v22: the can inside the box, its face on the forward
  // plate's aft face, the shaft forward through the plate to the overhung pinion); else the layout's box
  const can = ctx.phys?.solid('motor', 'can'), shs = ctx.phys?.solid('motor', 'shaft');
  const r = can ? can.shape.ro : mp.cylinder.r, L = can ? can.x[1] - can.x[0] : mp.cylinder.len;
  if (can) motor.position.set((can.x[0] + can.x[1]) / 2, can.shape.c[1], can.shape.c[0]);
  else motor.position.set(mp.box.cx, mp.box.cy, mp.box.cz);
  const tagged = (o, t) => (can ? tagMesh(o, t) : o);
  const bellG = new THREE.LatheGeometry([[0.003, -L / 2], [r * 0.82, -L / 2], [r, -L / 2 + 0.003], [r, L / 2 - 0.006], [r * 0.9, L / 2], [0.004, L / 2]].map(([a, b]) => new THREE.Vector2(a, b)), 64);
  bellG.rotateZ(-Math.PI / 2); // lathe axis y -> x; bell open aft
  const bellNode = group('motor_bell', motor, 'drive');
  const bell = tagged(mesh('motor_bell_can', bellG, 'alu_anodised', bellNode, 'drive', mp.id), 'motor/can');
  const ring = tagged(mesh('motor_magnet_band', ringX(r * 0.985, r * 0.9, L * 0.55, 64), 'magnet', bellNode, 'drive', mp.id), 'motor/can');
  ring.position.x = -0.002;
  for (let k = 0; k < 6; k++) {
    const vent = tagged(mesh(`motor_vent_${k}`, new THREE.BoxGeometry(0.0015, 0.004, r * 0.55), 'alu_machined', bellNode, 'drive', mp.id), 'motor/can');
    const a = (k / 6) * TWO_PI;
    vent.position.set(L / 2 - 0.0008, Math.cos(a) * r * 0.55, Math.sin(a) * r * 0.55);
    vent.rotation.x = -a;
  }
  const objs = [bell, ring];
  if (!can) {
    const mount = mesh('motor_mount', cylX(r * 0.95, 0.004, 48), 'alu_machined', motor, 'drive', mp.id);
    mount.position.x = -L / 2 - 0.0035;
    objs.push(mount);
  }
  const pin = P.stage?.[0] ?? mp.box.x0 - 0.01;
  const shL = shs ? shs.x[1] - shs.x[0] : Math.max(0.01, mp.box.x0 - pin + 0.006);
  const mshaft = tagged(mesh('motor_shaft', cylX(shs ? shs.shape.ro : (spec(mp.id).shaft_mm ?? 5) / 2000, shL, 16), 'steel_shaft', motor, 'drive', mp.id), 'motor/shaft');
  mshaft.position.x = shs ? (shs.x[0] + shs.x[1]) / 2 - motor.position.x : -L / 2 - (mp.box.x0 - pin + 0.006) / 2 + 0.001;
  objs.push(mshaft);
  posed.push(bellNode);
  register('flap motor', mp.id, 'flap motor', 'drive', objs);
  drive.bell = bellNode;
  drive.motor = motor;
}

/** Box parts at their layout places: battery, ESC, FC, GPS, receiver, airspeed, servos. */
function buildBoxParts(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const servos = {};
  for (const lp of model.layout.parts) {
    if (lp.name === 'motor') continue;
    const p = lp.name === 'fc' ? flightController(model, lp) : lp;
    const b = p.box;
    const node = group(`part_${p.name}`, root, p.subsystem);
    node.position.set(b.cx, b.cy, b.cz);
    let objs = [];
    if (p.name === 'battery') {
      objs.push(mesh('battery_pack', roundedBox(b.sx, b.sy, b.sz, 0.003), 'battery_wrap', node, 'power', p.id));
      // its lead, until the physics draws the leads as BOM lines (then they are its solids)
      if (!physHas(ctx, LEADS)) objs.push(mesh('battery_lead', wire([[b.sx / 2 - 0.004, b.sy / 2, 0.004], [b.sx / 2 - 0.02, b.sy / 2 + 0.004, 0.006], [-b.sx / 2 - 0.01, b.sy / 2 + 0.002, 0.004]], 0.0012), 'wire', node, 'power'));
    } else if (p.name === 'esc') {
      objs.push(mesh('esc_body', roundedBox(b.sx, b.sy, b.sz, Math.min(b.sy, b.sz) * 0.45), 'esc_wrap', node, 'power', p.id));
    } else if (p.name.startsWith('servo')) {
      const caseO = mesh(`${p.name}_case`, roundedBox(b.sx, b.sy, b.sz, 0.0015), 'servo_case', node, p.subsystem, p.id);
      const horn = group(`${p.name}_horn`, node, p.subsystem);
      // output shaft on the largest face, near one end
      const big = [b.sx, b.sy, b.sz];
      const thin = big.indexOf(Math.min(...big));
      const axis = [0, 0, 0];
      axis[thin] = 1;
      horn.position.set(axis[0] * (b.sx / 2 + 0.0015), axis[1] * (b.sy / 2 + 0.0015), axis[2] * (b.sz / 2 + 0.0015));
      horn.userData.axis = new V3(...axis);
      objs.push(caseO);
      // its horn's arm, until the physics draws the horns as BOM lines
      if (!physHas(ctx, HORNS)) {
        const arm = mesh(`${p.name}_horn_arm`, new THREE.BoxGeometry(thin === 0 ? 0.002 : 0.004, thin === 1 ? 0.002 : 0.016, thin === 2 ? 0.002 : 0.004), 'alu_machined', horn, p.subsystem, p.id);
        arm.position.set(thin === 0 ? 0 : 0, thin === 1 ? 0 : 0.006, 0);
        objs.push(arm);
      }
      servos[p.name] = { node, horn, box: b };
      posed.push(horn);
      if (p.name === 'servo_stroke') accentLine('stroke_horn_line', [[0, 0, 0], [0, 0.012, 0]], horn, 'drive', false);
    } else if (p.name === 'gps') {
      objs.push(mesh('gps_puck', roundedBox(b.sx, b.sy, b.sz, Math.min(b.sx, b.sz) * 0.2), 'gps_case', node, p.subsystem, p.id));
    } else {
      objs.push(mesh(`${p.name}_board`, roundedBox(b.sx, b.sy, b.sz, Math.min(b.sx, b.sy, b.sz) * 0.4), 'pcb', node, p.subsystem, p.id));
    }
    // the body is the physics' packed solid (clearance.js packedParts: the same layout box), its tag the BOM's
    if (ctx.phys?.part(`packed ${p.name}`)) tagMesh(objs[0], `packed ${p.name}/box`);
    register(p.name, p.id, p.role, p.subsystem, objs);
  }
  return servos;
}

/**
 * The flight controller as its own BOM line. layoutParts (shared with src/design/fit.js, so left as it is) finds the
 * FC by the first role that mentions "flight controller", which v14's ESC role does ("the flight controller's 5 V
 * BEC"), and so places the ESC's id and size there. Draw the real FC, sized from its own catalogue entry as
 * layoutParts sizes boards, at the place the packing gave it.
 */
function flightController(model, lp) {
  const fc = model.design.bom.parts.find((p) => /^flight controller/.test(p.role));
  if (!fc || fc.id === lp.id) return lp;
  const [sx, sy, sz] = sizeOf(model.parts[fc.id]?.specs, fc.unit_mass_g ?? model.parts[fc.id]?.mass_g, (v) => [v[0], v[2], v[1]], 1850, [], 'flight controller');
  const box = { ...lp.box, sx, sy, sz, x0: lp.box.cx - sx / 2, x1: lp.box.cx + sx / 2, y0: lp.box.cy - sy / 2, y1: lp.box.cy + sy / 2, z0: lp.box.cz - sz / 2, z1: lp.box.cz + sz / 2 };
  return { ...lp, id: fc.id, role: fc.role, box };
}

/** Wiring (inside the 15% margin). */
function buildWiring(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  // until the physics draws the leads as BOM lines (then they are its solids)
  if (physHas(ctx, LEADS)) return;
  const L = Object.fromEntries(model.layout.parts.map((p) => [p.name, p.box]));
  const W_ = [];
  const route = (name, pts, r, sub) => W_.push(mesh(name, wire(pts, r), 'wire', root, sub));
  const bat = L.battery, esc = L.esc, mot = L.motor, fc = L.fc;
  if (bat && esc) {
    route('wire_batt_pos', [[bat.x0, bat.cy + 0.004, 0.004], [(bat.x0 + esc.x1) / 2, esc.cy + 0.006, 0.006], [esc.x1, esc.cy, 0.004]], 0.0011, 'power');
    route('wire_batt_neg', [[bat.x0, bat.cy - 0.004, -0.004], [(bat.x0 + esc.x1) / 2, esc.cy - 0.002, -0.006], [esc.x1, esc.cy, -0.004]], 0.0011, 'power');
  }
  if (esc && mot) for (const [k, dz] of [[0, -0.005], [1, 0], [2, 0.005]]) route(`wire_phase_${k}`, [[esc.x0, esc.cy, dz], [esc.x0 - 0.006, esc.cy + 0.006, dz * 1.2], [mot.x1 + 0.002, mot.y0 + 0.004, dz * 1.4]], 0.0007, 'power');
  if (fc && esc) route('wire_bec', [[esc.x0, esc.cy - 0.002, 0.009], [(esc.x0 + fc.x1) / 2, esc.cy - 0.012, 0.016], [fc.x1, fc.cy, 0.008]], 0.0005, 'power');
  for (const n of ['gps', 'receiver', 'airspeed', 'servo_stroke', 'servo_tail_L', 'servo_tail_R']) {
    const b = L[n];
    if (!b || !fc) continue;
    route(`wire_sig_${n}`, [[fc.cx, fc.y1, fc.cz], [(fc.cx + b.cx) / 2, Math.max(fc.y1, b.cy) + 0.004, (fc.cz + b.cz) / 2 + 0.006], [b.cx, b.cy, b.cz]], 0.00045, 'avionics');
  }
  register('wiring (15% margin)', null, 'wiring', 'power', W_);
}

/** Tail pushrods: steel wire from each tail servo horn to its surface's root. */
function buildPushrods(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { tailNodes, servos } = ctx;
  const pushrods = [];
  // until the physics draws the pushrods as BOM lines (then they are its solids)
  if (physHas(ctx, PUSHRODS)) return pushrods;
  for (const t of tailNodes) {
    const sv = servos[t.side > 0 ? 'servo_tail_R' : 'servo_tail_L'];
    if (!sv) continue;
    const g = deformTube(12, 0.0004, 6);
    const o = mesh(`pushrod_${t.side > 0 ? 'R' : 'L'}`, g, 'steel_shaft', root, 'avionics');
    o.frustumCulled = false;
    deform.push(o);
    pushrods.push({ mesh: o, servo: sv, tail: t });
  }
  return pushrods;
}

function poseWing(ctx, tag, phi, theta, explodeWing) {
  const { model, D, W, P, train, root, meshes, accents, mats, wings, drive, servos, tailNodes, pushrods, segBody, ss, filmSt, tmp, ghost, exState } = ctx;
  const w = wings[tag];
  const side = w.side;
  const segA = segmentAngles(W, phi).map((s) => s.phi);
  // segment frames in the body: origin on the joint, rotated about body x by the segment angle
  const h = D.wing.hinge;
  const M0 = new THREE.Matrix4().makeTranslation(h[0], h[1], side * h[2]).multiply(side > 0 ? rotCCW(segA[0]) : rotCCW(-segA[0]));
  const L0 = W.segments[0].length;
  const M1 = M0.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, side * L0)).multiply(side > 0 ? rotCCW(segA[1] - segA[0]) : rotCCW(-(segA[1] - segA[0])));
  segBody[tag][0].copy(M0);
  segBody[tag][1].copy(M1);
  // explode: the whole wing slides outboard; children keep their local frames, edge rods get the same offset
  const out = side * 0.14 * explodeWing;
  setLocal(w.segs[0], new THREE.Matrix4().makeTranslation(0, 0, out).multiply(M0), null);
  setLocal(w.segs[1], M1, M0);
  // the rocker, its shoulder pin and its horn pin turn with the wing root but arrive in their own steps
  for (const n of drive?.nodes?.[tag] ?? []) setLocal(n, M0, null);
  // ribs
  const st = model.ribSt;
  const s = wingStations(model, st, phi, theta, side);
  ss[tag] = s;
  for (const r of w.ribs) {
    const i = r.userData.station;
    const B = basisAt(s.A, s.t, s.b, 3 * i);
    setLocal(r, B, segBody[tag][st.seg[i]]);
  }
  // film and edge rods
  const fst = filmSt[0];
  const fs = wingStations(model, fst, phi, theta, side);
  const lift = 0.05 * explodeWing;
  const edgeR = model._teR ?? 0.00075;
  const lePts = [], tePts = [];
  for (let i = 0; i < fst.n; i++) {
    const a = sectionPoint(model, fst, fs, i, 0, 0, [0, 0, 0]);
    const b = sectionPoint(model, fst, fs, i, 1, 0, [0, 0, 0]);
    a[2] += side * 0.14 * explodeWing;
    b[2] += side * 0.14 * explodeWing;
    lePts.push(a);
    tePts.push(b);
  }
  w.le?.geometry.userData.update(lePts);
  w.te?.geometry.userData.update(tePts);
  for (let k = 0; k < 2; k++) {
    const f = w.films[k];
    const pos = f.geometry.attributes.position.array;
    _inv.copy(segBody[tag][k]).invert();
    const v = new V3();
    let o = 0;
    const sec = model.section;
    const nSec = sec.length;
    for (const i of f.userData.rows) {
      const j = 3 * i;
      for (let c = 0; c < 2 * nSec - 1; c++) {
        const up = c < nSec;
        const q = sec[up ? nSec - 1 - c : c - nSec + 1];
        // the film wraps the edge rods, so the section never closes to zero thickness (no degenerate normals)
        const half = Math.max(q.half, edgeR / fst.chord[i]);
        const d = i * nSec + (up ? nSec - 1 - c : c - nSec + 1);
        const hh = up ? q.cam + half - fst.dipUp[d] : q.cam - half - fst.dipLo[d];
        sectionPoint(model, fst, fs, i, q.xi, hh, tmp);
        v.set(tmp[0] + lift * fs.b[j], tmp[1] + lift * fs.b[j + 1], tmp[2] + lift * fs.b[j + 2]).applyMatrix4(_inv);
        pos[o++] = v.x;
        pos[o++] = v.y;
        pos[o++] = v.z;
      }
    }
    f.geometry.attributes.position.needsUpdate = true;
    f.geometry.computeVertexNormals();
    f.geometry.computeBoundingSphere();
  }
  // twist servo: on the spar at its station, horn turned by the commanded twist at that station
  if (w.tsv && model.twistServo) {
    const tst = model.twistServo.st;
    const ts = wingStations(model, tst, phi, theta, side);
    const A = new V3(ts.A[0], ts.A[1], ts.A[2]);
    const e = new V3(ts.e[0], ts.e[1], ts.e[2]);
    // case frame: chord along body x of the segment, spanwise e, up = e x x
    const X = new V3(1, 0, 0);
    const Y = new V3().crossVectors(e, X).multiplyScalar(side);
    const Z = new V3().crossVectors(X, Y);
    const Mc = new THREE.Matrix4().makeBasis(X, Y, Z).setPosition(A);
    setLocal(w.tsv.node, Mc, segBody[tag][1]);
    w.tsv.horn.matrixAutoUpdate = false;
    // horn linkage: the design's ratio is wing twist per servo turn, so the horn turns the command over it
    const ratio = D.servos?.twist?.linkage_ratio ?? 1;
    w.tsv.horn.matrix.makeRotationZ((side * theta) / ratio).setPosition(0, 0, w.tsv.dims[2] / 2 + 0.002);
    w.tsv.horn.matrixWorldNeedsUpdate = true;
    const arm = w.segs[1].getObjectByName(`torque_arm_${tag}`);
    if (arm) {
      const p = arm.userData.home ?? (arm.userData.home = arm.position.clone());
      arm.matrixAutoUpdate = false;
      arm.matrix.makeRotationZ(side * theta).setPosition(p);
      arm.matrixWorldNeedsUpdate = true;
    }
  }
}

/** One side's coupler from the crank pin A to its rocker pin B, and a servo drive's slot pin at B. */
function poseCoupler(ctx, dp, tag, side, symmetric) {
  const { drive, segBody } = ctx;
  // symmetric: both wings at the flight model's stroke angle (as in a flight log); the left pin mirrors the right
  const B = side > 0 ? dp.right.B : symmetric ? [-dp.right.B[0], dp.right.B[1]] : dp.left.B;
  const x = drive[tag].x;
  if (drive[tag].pin) setLocal(drive[tag].pin, new THREE.Matrix4().makeTranslation(x, B[1], B[0]), segBody[tag][0]);
  const ang = Math.atan2(B[1] - dp.A[1], B[0] - dp.A[0]); // CCW from +z
  setLocal(drive[tag].coupler, new THREE.Matrix4().makeTranslation(x, dp.A[1], dp.A[0]).multiply(rotCCW(ang - Math.PI / 2)), null);
}

function updateBird(ctx, state) {
  const { model, D, W, P, train, root, meshes, accents, mats, wings, drive, servos, tailNodes, pushrods, segBody, ss, filmSt, tmp, ghost, exState } = ctx;
  const phase = state.phase ?? 0, amp = state.amp ?? D.cruise?.amp ?? model.law.ampNominal ?? 0.35;
  const ex = state.explode ?? {};
  const { exS, exW } = stepState(ctx, state, ex);
  const dp = drivePose(model, phase, amp);
  poseWing(ctx, 'R', dp.right.phi, state.twistR ?? 0, exW);
  poseWing(ctx, 'L', state.symmetric ? dp.right.phi : dp.left.phi, state.twistL ?? 0, exW);
  for (const [tag, side] of [['R', 1], ['L', -1]]) poseCoupler(ctx, dp, tag, side, state.symmetric);
  // crank: pin at +y in its own frame, turned to point at A
  const ca = Math.atan2(dp.A[1] - dp.O2[1], dp.A[0] - dp.O2[0]);
  setLocal(drive.crank, new THREE.Matrix4().makeTranslation(0, dp.O2[1], dp.O2[0]).multiply(rotCCW(ca - Math.PI / 2)), null);
  // gears at the true tooth ratios from the crank angle
  const ga = gearAngles(train, ca);
  const gearPose = (node, ang) => {
    const p = node.userData.home ?? (node.userData.home = node.position.clone());
    setLocal(node, new THREE.Matrix4().makeTranslation(p.x, p.y, p.z).multiply(rotCCW(ang)), null);
  };
  for (const g of drive.gears) gearPose(g.node, ga[g.shaft]);
  if (drive.bell && ga.length > 1) {
    drive.bell.matrixAutoUpdate = false;
    drive.bell.matrix.copy(rotCCW(ga[ga.length - 1]));
    drive.bell.matrixWorldNeedsUpdate = true;
  }
  poseServos(ctx, state, amp);
  // tail surfaces about their quarter-chord axes
  for (const t of tailNodes) {
    const defl = t.side > 0 ? state.tailR ?? 0 : state.tailL ?? 0;
    // nose up is a right-hand turn about the outboard span axis on the right, the reverse on the left
    const q = new THREE.Quaternion().setFromAxisAngle(t.axis, t.side * defl);
    const o = t.origin.clone().add(new V3(-0.16 * exS, 0.03 * exS, 0));
    setLocal(t.node, new THREE.Matrix4().compose(o, q, new V3(1, 1, 1)), null);
  }
  // pushrods from horn tips to tail roots
  root.updateMatrixWorld(true);
  _inv.copy(root.matrixWorld).invert();
  for (const pr of pushrods) {
    const a = new V3(0, 0.012, 0).applyMatrix4(pr.servo.horn.matrixWorld).applyMatrix4(_inv);
    const b = new V3(-0.02, 0.008, 0).applyMatrix4(pr.tail.node.matrixWorld).applyMatrix4(_inv);
    const pts = [];
    const k = model._keel;
    for (let i = 0; i < 12; i++) {
      const f = i / 11;
      const y = a.y + (b.y - a.y) * f + (k ? Math.sin(Math.PI * f) * 0.002 : 0);
      pts.push([a.x + (b.x - a.x) * f, y, a.z + (b.z - a.z) * f]);
    }
    pr.mesh.geometry.userData.update(pts);
  }
  applyLook(ctx, state);
  return dp;
}

/** Servo horns: the stroke servo's from the amplitude command across the drive's range; the tail horns from deflection. */
function poseServos({ model, servos }, state, amp) {
  const sv = servos.servo_stroke;
  if (sv) poseHorn(sv, ((amp - model.law.ampMin) / (model.law.ampMax - model.law.ampMin) - 0.5) * (Math.PI / 2));
  for (const [name, defl] of [['servo_tail_L', state.tailL ?? 0], ['servo_tail_R', state.tailR ?? 0]]) if (servos[name]) poseHorn(servos[name], defl);
}

/**
 * The assembly: each step's exploded fraction (ctx.stepF), the physics' steps beat by beat at build progress
 * state.build (solids.js stepTimes), else by subsystem; the pod covering's (ctx.exS). The steps move the tail and the
 * wings along their own lines, so the poses' own offsets for them are 0. A design without steps explodes by
 * subsystem, the poses offsetting the tail and the wings.
 */
function stepState(ctx, state, ex) {
  if (!ctx.steps) return { exS: (ctx.exS = ex.structure ?? 0), exW: ex.wing ?? 0 };
  ctx.stepF = typeof state.build === 'number' ? stepFractions(ctx.times, state.build) : ctx.steps.map((s) => ex[s.sub] ?? 0);
  ctx.exS = ctx.stepF[ctx.steps.findIndex((s) => s.pod)];
  return { exS: 0, exW: 0 };
}

function poseHorn(sv, angle) {
  const ax = sv.horn.userData.axis;
  sv.horn.quaternion.setFromAxisAngle(ax, angle);
  sv.horn.matrixAutoUpdate = true;
}

/** Whether a mesh is drawn as itself under a focus: its subsystem's, the twist's ribs and film, or a part id in `also`
 * (the power sequence shows the servos its regulator feeds). */
function inFocus(o, focus, also) {
  if (!focus || o.userData.subsystem === focus) return true;
  if (focus === 'twist' && o.userData.subsystem === 'wing' && /rib|film/.test(o.name)) return true;
  return !!also?.has?.(o.userData.partId);
}

function applyLook(ctx, state) {
  const { model, D, W, P, train, root, meshes, accents, mats, wings, drive, servos, tailNodes, pushrods, segBody, ss, filmSt, tmp, ghost, exState } = ctx;
  const focus = state.focus ?? null;
  const ex = state.explode ?? {};
  for (const o of meshes) {
    const sub = o.userData.subsystem;
    const on = inFocus(o, focus, state.also);
    const want = on ? mats[o.userData.matName] : ghost;
    if (o.material !== want) o.material = want;
    // out of focus, the film and the edge rods would only veil the part in focus: hide them, ghost the rest
    if (/^(film_|le_rod|te_rod|tail_film)/.test(o.name)) o.visible = on;
    buildVisibility(ctx, o, state);
    if (o.name === 'shell') {
      // in the assembly the covering goes on last, in the pod's step: not drawn before it (a white skin would hide the
      // gearbox going together under it)
      o.visible = (state.shell ?? 1) > 0.02 && (!focus || focus === 'structure') && !(typeof state.build === 'number' && ctx.exS > 0.999);
      o.position.y = 0.11 * (ctx.exS ?? ex.structure ?? 0);
    }
  }
  // the structure under a drawn, pigmented covering is not drawn (markUnder)
  if (ctx.opaque) for (const o of meshes) if (o.userData.under?.some((c) => c.visible)) o.visible = false;
  for (const l of accents) l.visible = !!state.accents && (!focus || l.userData.subsystem === focus);
  if (ctx.steps) applySteps(ctx);
  else explodeBySubsystem(ctx, ex);
  exState.last = state;
}

/** A design without the physics' steps: each subsystem's parts out along their own lines by its fraction. */
function explodeBySubsystem(ctx, ex) {
  const { root, drive } = ctx;
  for (const node of root.children) {
    if (!node.name.startsWith('part_')) continue;
    const sub = node.userData.subsystem;
    const f = ex[sub] ?? 0;
    const home = node.userData.home ?? (node.userData.home = node.position.clone());
    const dir = explodeDir(node.name);
    node.position.set(home.x + dir[0] * f, home.y + dir[1] * f, home.z + dir[2] * f);
  }
  const fd = ex.drive ?? 0;
  const explodeDrive = [['motor', 0.07, 0.02], ...drive.gears.map((g, k) => [g.node.name, 0.012 + 0.012 * Math.floor(k / 2), -0.01 * (k % 2)])];
  for (const [n, dx, dy] of explodeDrive) {
    const node = root.getObjectByName(n);
    if (!node) continue;
    const home = node.userData.home ?? (node.userData.home = node.position.clone());
    if (node.matrixAutoUpdate) node.position.set(home.x + dx * fd, home.y + dy * fd, home.z);
    else {
      node.matrix.elements[12] += dx * fd;
      node.matrix.elements[13] += dy * fd;
    }
  }
}

/**
 * The assembly order: the physics' own sequence (solids.js physicsAssembly steps: params.layout.sequence on the
 * physics' solids), each step moving the top node of every mesh that draws one of its solids along its stated line,
 * from its stated distance out; a threaded tie does not slide, it is drawn from its step on. A film-only part (a wire,
 * a pushrod, the tail's covering) goes on with the last step whose parts it touches; the pod's covering, which the
 * sequence exempts, is a last beat of its own. The steps are grouped into beats for the screen (solids.js
 * screenBeats) and timed beat by beat (stepTimes), in the physics' order. A design without the physics' sequence has
 * none (applyLook explodes it by subsystem).
 */
function buildSteps(ctx) {
  const ph = ctx.phys;
  if (!ph?.steps) return null;
  const top = (o) => {
    while (o.parent && o.parent !== ctx.root) o = o.parent;
    return o;
  };
  const stepOf = new Map();
  ph.steps.forEach((st, k) => st.solids.forEach((t) => stepOf.set(t, k)));
  const firstStep = (tags) => {
    for (const t of tags) {
      const k = stepOf.get(t) ?? stepOf.get(`${t}/${ph.part(t)?.solids[0]?.name}`);
      if (k != null) return k;
    }
    return -1;
  };
  const steps = ph.steps.map((st) => ({ name: st.name, threaded: !!st.threaded, line: st.dir ? new V3(...st.dir).multiplyScalar(st.dist) : new V3(), nodes: new Map(), subs: {} }));
  const nodeStep = new Map();
  for (const o of ctx.meshes) {
    const k = firstStep(o.userData.physics ?? []);
    if (k < 0) continue;
    const n = top(o);
    if (!nodeStep.has(n)) nodeStep.set(n, k);
    steps[k].subs[o.userData.subsystem] = (steps[k].subs[o.userData.subsystem] ?? 0) + 1;
  }
  // film-only nodes: with the last step whose parts they touch (their boxes at rest overlap); the pod's covering apart
  ctx.root.updateMatrixWorld(true);
  const box = (n) => new THREE.Box3().setFromObject(n);
  const boxes = [...nodeStep].map(([n, k]) => [box(n), k]);
  const shell = ctx.root.getObjectByName('shell');
  for (const n of ctx.root.children) {
    if (nodeStep.has(n) || n === shell || !(n.isMesh || n.children.some((c) => c.isMesh))) continue;
    const b = box(n);
    if (b.isEmpty()) continue;
    const k = Math.max(-1, ...boxes.filter(([bb]) => bb.intersectsBox(b)).map(([, kk]) => kk));
    if (k >= 0) nodeStep.set(n, k);
  }
  for (const [n, k] of nodeStep) steps[k].nodes.set(n, steps[k].line);
  for (const st of steps) st.sub = Object.entries(st.subs).sort((x, y) => y[1] - x[1])[0]?.[0] ?? 'structure';
  steps.push({ name: 'pod covering', sub: 'structure', pod: true, line: new V3(0, 0.11, 0), nodes: new Map() });
  ctx.beats = [...screenBeats(ph, ctx.model.parts), { name: 'pod covering', steps: [steps.length - 1], film: true }];
  ctx.times = stepTimes(ctx.beats.slice(0, -1), 1);
  ctx.meshStep = meshSteps(ctx, steps);
  return steps;
}

/**
 * The structure under the covering (round 16: the wings' and the tail's rods and ribs, the formers): every physics
 * mesh of a step that carries a covering (a surface arrives covered), and the formers under the pod's. Where the
 * covering is a pigmented film (a livery) and drawn, they are not drawn (userData.under: the coverings over them), so
 * they show only where the covering is cut away or not yet on, and never through it.
 */
function markUnder(ctx) {
  const covering = (o) => /^film_(wing|shell)$/.test(o.userData.matName) && !o.userData.physics?.length;
  const byStep = new Map();
  for (const [o, k] of ctx.meshStep) if (covering(o)) byStep.set(k, [...(byStep.get(k) ?? []), o]);
  const shell = ctx.root.getObjectByName('shell');
  for (const [o, k] of ctx.meshStep) {
    if (!o.userData.physics?.length) continue;
    if (byStep.has(k)) o.userData.under = byStep.get(k);
    else if (shell && ctx.parts.some((p) => /formers/.test(p.name) && p.objects.includes(o))) o.userData.under = [shell];
  }
}

/** Every mesh's assembly step: the step its top node goes on in; the covering's with the pod's covering beat. */
function meshSteps(ctx, steps) {
  const map = new Map();
  steps.forEach((s, k) => {
    for (const n of s.nodes.keys()) n.traverse((o) => o.isMesh && map.set(o, k));
  });
  const shell = ctx.root.getObjectByName('shell');
  if (shell) map.set(shell, steps.findIndex((s) => s.pod));
  return map;
}

/** Building (state.build): a part is drawn once its step has begun; otherwise as its own rules say (the film's
 * covering and rods follow the focus, every other part is drawn). */
function buildVisibility(ctx, o, state) {
  if (!ctx.meshStep?.has(o)) return;
  const own = /^(film_|le_rod|te_rod|tail_film)/.test(o.name);
  if (typeof state.build === 'number') o.visible = ctx.stepF[ctx.meshStep.get(o)] < 0.999 && (o.visible || !own);
  else if (!own) o.visible = true;
}

/** Every step's nodes at its exploded fraction along its line (ctx.stepF, set by updateBird). */
function applySteps(ctx) {
  ctx.steps.forEach((s, k) => {
    const f = ctx.stepF[k];
    for (const [node, dir] of s.nodes) {
      if (node.matrixAutoUpdate) {
        const home = node.userData.home ?? (node.userData.home = node.position.clone());
        node.position.copy(home).addScaledVector(dir, f);
      } else if (f > 0) {
        // posed this update (setLocal): move its matrix along the line
        node.matrix.elements[12] += dir.x * f;
        node.matrix.elements[13] += dir.y * f;
        node.matrix.elements[14] += dir.z * f;
        node.matrixWorldNeedsUpdate = true;
      }
    }
  });
}

/** Each subsystem's exploded fraction at build progress p: the mean of its steps' (the assembly panel's mass rows). */
function buildExplode(ctx, p) {
  if (!ctx.steps) return null;
  const f = stepFractions(ctx.times, p), out = {};
  ctx.steps.forEach((s, k) => (out[s.sub] ??= []).push(f[k]));
  return Object.fromEntries(Object.entries(out).map(([k, v]) => [k, v.reduce((a, b) => a + b, 0) / v.length]));
}

function explodeDir(name) {
  if (/battery/.test(name)) return [0.08, -0.05, 0];
  if (/esc/.test(name)) return [0.03, -0.07, 0];
  if (/gps/.test(name)) return [0, 0.08, 0];
  if (/fc/.test(name)) return [-0.02, -0.07, 0];
  if (/receiver|airspeed/.test(name)) return [0.03, 0.06, 0];
  if (/servo_tail_L/.test(name)) return [-0.03, 0.05, -0.05];
  if (/servo_tail_R/.test(name)) return [-0.03, 0.05, 0.05];
  if (/servo_stroke/.test(name)) return [-0.04, 0.07, 0];
  if (/regulator/.test(name)) return [0.02, -0.06, 0.04];
  return [0, 0, 0];
}

export function buildBird(model, mats, opts = {}) {
  const ctx = makeCtx(model, mats, opts);
  buildFuselage(ctx);
  ctx.tailNodes = buildTail(ctx);
  Object.assign(ctx, buildWings(ctx));
  if (opts.livery) paintCovering(ctx, opts.livery);
  ctx.drive = {};
  buildLinkage(ctx);
  buildCrank(ctx);
  buildGears(ctx);
  buildMotor(ctx);
  buildFrame(ctx);
  ctx.servos = buildBoxParts(ctx);
  buildWiring(ctx);
  ctx.pushrods = buildPushrods(ctx);
  Object.assign(ctx, {
    ss: { R: null, L: null },
    segBody: { R: [new THREE.Matrix4(), new THREE.Matrix4()], L: [new THREE.Matrix4(), new THREE.Matrix4()] },
    tmp: [0, 0, 0],
    ghost: new THREE.MeshBasicMaterial({ color: 0x6f7a87, transparent: true, opacity: 0.03, depthWrite: false }),
    exState: {},
  });
  if (ctx.phys) {
    // every physics solid no builder drew: the fixed ones in place, the moving ones in the node they move with, found
    // by posing the bird (the physics' drive pose at the same phase is what they are drawn against)
    drawRest(ctx);
    const amp0 = model.design.cruise?.amp ?? model.law.ampNominal;
    drawPosed(ctx, (phase) => {
      updateBird(ctx, { phase, amp: amp0 });
      return drivePose({ law: ctx.phys.lay.law, fourBar: ctx.phys.lay.fb }, phase, amp0);
    });
  }
  ctx.steps = buildSteps(ctx);
  ctx.opaque = !!opts.livery;
  if (ctx.steps) markUnder(ctx);
  const update = (state) => updateBird(ctx, state);
  update({ phase: 0, amp: model.design.cruise?.amp ?? model.law.ampNominal });
  const { root, parts, meshes, posed, deform, accents, servos, drive, wings, phys } = ctx;
  const steps = ctx.steps?.map((s) => ({ name: s.name, sub: s.sub, threaded: !!s.threaded, pod: !!s.pod, line: s.line.toArray(), nodes: [...s.nodes.keys()] })) ?? null;
  const assembly = ctx.steps ? { beats: ctx.beats, times: ctx.times } : null;
  return { root, update, parts, meshes, posed, deform, accents, model, servos, drive, wings, phys, steps, assembly, meshStep: ctx.meshStep, buildExplode: (p) => buildExplode(ctx, p) };
}


/**
 * The drive's x range a former keeps out of: the physics' solids (v22 on: shoulder rails to the encoder's stub), less
 * the trays, which run the pod's length (a former crosses them: reported, they have no cutout in the design), and the
 * boom; else the v21 planes.
 */
function driveZone(model, phys) {
  const P = model.planes;
  if (!phys) return { x0: P.couplerL - 0.008, x1: P.plateFwd + 0.004 };
  const xs = phys.parts.filter((p) => !/ tray$|^tail boom$/.test(p.id)).flatMap((p) => p.solids.map((s) => s.x));
  return { x0: Math.min(...xs.map((v) => v[0])) - 0.001, x1: Math.max(...xs.map((v) => v[1])) + 0.001 };
}

/** Nearest x (within 30 mm) where no part box crosses the former's ring (between the loft and its lightening hole). */
function clearFormerX(model, sec, x, phys = null) {
  const boxes = model.layout.parts.map((p) => p.box);
  const drive = driveZone(model, phys);
  const ok = (xx) => {
    if (xx > drive.x0 && xx < drive.x1) return false;
    const s = sectionAtX(sec, xx);
    if (!s) return false;
    const a = (s.width_m / 2) * Math.SQRT1_2, b = (s.height_m / 2) * Math.SQRT1_2;
    for (const bx of boxes) {
      if (xx < bx.x0 - 0.001 || xx > bx.x1 + 0.001) continue;
      for (const y of [bx.y0, bx.y1]) for (const z of [bx.z0, bx.z1]) if (((y - s.y_m) / b) ** 2 + (z / a) ** 2 > 1) return false;
    }
    return true;
  };
  for (let k = 0; k <= 30; k++) for (const sg of [1, -1]) if (ok(x + sg * k * 0.001)) return x + sg * k * 0.001;
  return x;
}

/** Convex hull of 2D points (monotone chain), counter-clockwise. */
function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], hi = [];
  for (const q of p) {
    while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop();
    lo.push(q);
  }
  for (let i = p.length - 1; i >= 0; i--) {
    const q = p[i];
    while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], q) <= 0) hi.pop();
    hi.push(q);
  }
  return lo.slice(0, -1).concat(hi.slice(0, -1));
}

function sectionAtX(sec, x) {
  for (let i = 1; i < sec.length; i++) {
    const a = sec[i - 1], b = sec[i];
    if (x <= a.x_m && x >= b.x_m) {
      const f = (a.x_m - x) / (a.x_m - b.x_m || 1);
      return { y_m: a.y_m + f * (b.y_m - a.y_m), width_m: a.width_m + f * (b.width_m - a.width_m), height_m: a.height_m + f * (b.height_m - a.height_m) };
    }
  }
  return null;
}

/**
 * A rib: the section envelope (design camber line, E387 thickness) at its station's chord, 2 mm balsa, with the
 * spar hole and three lightening holes so its area matches the BOM's per-rib figure roughly.
 * Rib-local frame: x forward along the chord (towards the LE), y up the section normal, z spanwise.
 */
function ribGeometry(model, st, i, sparOd) {
  const c = st.chord[i];
  const off = st.xle[i] - model.wing.spar_x; // LE distance forward of the spar axis
  const sec = model.section;
  const s = new THREE.Shape();
  const pts = [];
  const minH = 0.0006 / c;
  for (let k = sec.length - 1; k >= 0; k--) pts.push([off - sec[k].xi * c, (sec[k].cam + Math.max(sec[k].half, k ? minH : 0)) * c]);
  for (let k = 1; k < sec.length; k++) pts.push([off - sec[k].xi * c, (sec[k].cam - Math.max(sec[k].half, minH)) * c]);
  pts.forEach(([x, y], k) => (k ? s.lineTo(x, y) : s.moveTo(x, y)));
  s.closePath();
  // spar hole on the spar axis at (0, 0)
  const sh = new THREE.Path();
  sh.absarc(0, 0, sparOd / 2, 0, TWO_PI, true);
  s.holes.push(sh);
  // lightening holes between the spar and the TE, sized from the local thickness
  for (const xi of [0.52, 0.66, 0.8]) {
    const x = off - xi * c;
    const k = Math.round(xi * (sec.length - 1));
    const q = sec.reduce((best, e) => (Math.abs(e.xi - xi) < Math.abs(best.xi - xi) ? e : best), sec[0]);
    const r = Math.max(0.0012, q.half * c * 0.55);
    const h = new THREE.Path();
    h.absarc(x, q.cam * c, r, 0, TWO_PI, true);
    if (Math.abs(x) > sparOd && k > 0) s.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: 0.002, bevelEnabled: false, curveSegments: 12 });
  g.translate(0, 0, -0.001);
  return g;
}
