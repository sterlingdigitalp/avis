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
import { stationsAt, wingStations, sectionPoint, drivePose, gearAngles } from './model.js';
import { segmentAngles } from '../../src/physics/kinematics.js';
import { ACCENT } from './materials.js';

export const SUBSYSTEMS = ['wing', 'drive', 'twist', 'power', 'avionics', 'structure'];
const TWO_PI = 2 * Math.PI;
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
function roundedBox(sx, sy, sz, r) {
  const s = new THREE.Shape();
  const hx = sx / 2 - r, hy = sy / 2 - r;
  s.moveTo(-hx, -sy / 2);
  s.lineTo(hx, -sy / 2);
  s.absarc(hx, -hy, r, -Math.PI / 2, 0);
  s.lineTo(sx / 2, hy);
  s.absarc(hx, hy, r, 0, Math.PI / 2);
  s.lineTo(-hx, sy / 2);
  s.absarc(-hx, hy, r, Math.PI / 2, Math.PI);
  s.lineTo(-sx / 2, -hy);
  s.absarc(-hx, -hy, r, Math.PI, 1.5 * Math.PI);
  const g = new THREE.ExtrudeGeometry(s, { depth: sz - 2 * Math.min(r, sz / 4), bevelEnabled: true, bevelSize: Math.min(r, sz / 4) * 0.8, bevelThickness: Math.min(r, sz / 4), bevelSegments: 2, curveSegments: 6 });
  g.translate(0, 0, -(sz - 2 * Math.min(r, sz / 4)) / 2);
  return g;
}

/**
 * Involute spur gear outline: z teeth of module m, 20 degree pressure angle, addendum m, dedendum 1.25 m,
 * tooth 0 centred so that after alongX() it points along body +z. Extruded by the face width along x, centred.
 */
export function gearGeometry(z, m, face, bore, { hubR = 0, hubW = 0, key = 0, pa = (20 * Math.PI) / 180 } = {}) {
  const rp = (m * z) / 2, ra = rp + m, rb = rp * Math.cos(pa);
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
    const kw = key / 2, br = bore / 2;
    const a0 = Math.asin(kw / br);
    hole.moveTo(br * Math.cos(a0), br * Math.sin(a0));
    hole.absarc(0, 0, br, a0, TWO_PI - a0, false);
    hole.lineTo(br + key * 0.45, -kw);
    hole.lineTo(br + key * 0.45, kw);
    hole.closePath();
  } else hole.absarc(0, 0, bore / 2, 0, TWO_PI, true);
  s.holes.push(hole);
  const g = new THREE.ExtrudeGeometry(s, { depth: face, bevelEnabled: true, bevelThickness: Math.min(0.0003, face * 0.05), bevelSize: Math.min(0.00025, m * 0.2), bevelSegments: 1, curveSegments: 24 });
  g.translate(0, 0, -face / 2);
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
function makeCtx(model, mats, { filmRows = 3 } = {}) {
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
  const register = (name, id, role, subsystem, objects) => parts.push({ name, id, role, subsystem, objects: [].concat(objects) });
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
  return { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows };
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
  // formers: the BOM's count, evenly through the pod, 2 mm balsa rings lightened by half their area
  const fItem = structItem(/formers/);
  const nF = +(fItem?.basis.match(/(\d+)\s*off/)?.[1] ?? 5);
  const sec = model.loft.sections;
  const podEnd = sec.findIndex((s, i) => i > 0 && s.width_m <= sec[sec.length - 1].width_m + 1e-6);
  const x0 = sec[1].x_m - 0.01, x1 = sec[Math.max(1, podEnd - 1)].x_m + 0.01;
  const formers = [];
  for (let i = 0; i < nF; i++) {
    const x = clearFormerX(model, sec, x0 + ((x1 - x0) * i) / Math.max(1, nF - 1));
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
  // keel and boom: the woven tube, its listed length ending at the tail
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
  const tItem = structItem(/tail surfaces/);
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
    rodBetween(`tail_spar_${tag}`, qr, qt, 0.001);
    rodBetween(`tail_le_${tag}`, leR, leT, 0.00075);
    rodBetween(`tail_te_${tag}`, teR, teT, 0.00075);
    rodBetween(`tail_root_${tag}`, leR, teR, 0.00075);
    rodBetween(`tail_tip_${tag}`, leT, teT, 0.00075);
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
  // elbow knuckle: a pin through the joint; bearings only if the BOM's count has any left after the shoulder
  // hinges (2 each) and the gearbox shafts (2 each)
  const bearing = bomPart('bearing');
  const bd = (spec(bearing?.id).dimensions_mm_d_D_B ?? '4x9x2.5').split('x').map((v) => +v / 1000);
  const gearboxShafts = Math.max(0, (train.shafts?.length ?? 1) - 1);
  const elbowEach = Math.min(2, Math.floor(Math.max(0, (bearing?.count ?? 0) - 4 - 2 * gearboxShafts) / 2));
  const elbow = [];
  for (const dx of [-0.007, 0.007].slice(0, elbowEach)) {
    const b = mesh(`elbow_bearing_${tag}_${dx > 0 ? 'f' : 'a'}`, ringX(bd[1] / 2, bd[0] / 2, bd[2]), 'steel_ground', seg1, 'wing', bearing?.id);
    b.position.set(W.spar_x + dx, 0, 0);
    elbow.push(b);
  }
  const pin = mesh(`elbow_pin_${tag}`, cylX(bd[0] / 2, 0.02, 12), 'steel_shaft', seg1, 'wing');
  pin.position.set(W.spar_x, 0, 0);
  if (elbow.length) register(`elbow bearings ${tag}`, bearing?.id, 'bearing', 'wing', elbow);
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
    f.frustumCulled = false;
    films.push(f);
    deform.push(f);
  }
  register(`wing film ${tag}`, filmI?.material_id, 'wing film', 'wing', films);
  return films;
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
  const filmStations = stationsAt(W, fs);
  const nSec = model.section.length; // chordwise points per surface, LE..TE
  const ringN = 2 * nSec - 1; // TE upper -> LE -> TE lower
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    const seg0 = group(`wing_${tag}_0`, root, 'wing');
    const seg1 = group(`wing_${tag}_1`, seg0, 'wing');
    posed.push(seg0, seg1);
    const segs = [seg0, seg1];
    wingSpars(ctx, tag, side, segs);
    wingElbow(ctx, tag, side, segs);
    wingTorqueRod(ctx, tag, side, segs);
    const ribs = wingRibs(ctx, tag, side, segs);
    const films = wingFilm(ctx, tag, side, segs, filmStations, ringN);
    const { le, te } = wingEdgeRods(ctx, tag, side, segs, filmStations);
    const tsv = wingTwistServo(ctx, tag, side, segs);
    wings[tag] = { side, segs, ribs, films, le, te, tsv };
  }
  filmSt.push(filmStations);
  return { wings, filmSt };
}

/** Rockers on the wing roots and the couplers. */
function buildLinkage(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const bearing = bomPart('bearing');
  const bd = (spec(bearing?.id).dimensions_mm_d_D_B ?? '4x9x2.5').split('x').map((v) => +v / 1000);
  const plate = 0.002;
  // rockers: slotted carbon levers on each wing root; the slot is the locus of the coupler pin over the stroke
  // servo's range (c and psiMid from the drive table), so the mid-stroke angle stays at phiMean.
  const slot = law.table.map((e) => {
    const delta = fb.alpha + e.psiMid - law.phiMean;
    return [e.c * Math.cos(delta), e.c * Math.sin(delta)];
  });
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    const seg0 = wings[tag].segs[0];
    const xr = side > 0 ? P.couplerR : P.couplerL;
    const shp = new THREE.Shape();
    // outline: hinge boss, band along the slot, spar socket
    const outline = [];
    const w = 0.006;
    for (let i = 0; i < slot.length; i++) {
      const [z, y] = slot[i];
      const nx = i < slot.length - 1 ? slot[i + 1][0] - z : z - slot[i - 1][0];
      const ny = i < slot.length - 1 ? slot[i + 1][1] - y : y - slot[i - 1][1];
      const nl = Math.hypot(nx, ny) || 1;
      outline.push([z - (ny / nl) * w, y + (nx / nl) * w]);
    }
    const back = [];
    for (let i = slot.length - 1; i >= 0; i--) {
      const [z, y] = slot[i];
      const nx = i < slot.length - 1 ? slot[i + 1][0] - z : z - slot[i - 1][0];
      const ny = i < slot.length - 1 ? slot[i + 1][1] - y : y - slot[i - 1][1];
      const nl = Math.hypot(nx, ny) || 1;
      back.push([z + (ny / nl) * w, y - (nx / nl) * w]);
    }
    const sock = [[0.014, -0.006], [0.014, 0.006]];
    const poly = [...sock, ...outline, ...back];
    // shape coords (sx, sy) = (z, y) local; mirrored for the left wing
    // shape x = -(local z) so that alongX() maps it back to +local z without mirroring the mesh
    poly.forEach(([z, y], i) => (i ? shp.lineTo(-side * z, y) : shp.moveTo(-side * z, y)));
    shp.closePath();
    const h = new THREE.Path();
    h.absarc(0, 0, bd[1] / 2, 0, TWO_PI, true);
    shp.holes.push(h);
    // slot: a thin closed band 2 mm wide
    const sl = new THREE.Path();
    const half = 0.0011;
    const sp = slot.map(([z, y], i) => {
      const nx = i < slot.length - 1 ? slot[i + 1][0] - z : z - slot[i - 1][0];
      const ny = i < slot.length - 1 ? slot[i + 1][1] - y : y - slot[i - 1][1];
      const nl = Math.hypot(nx, ny) || 1;
      return [[z - (ny / nl) * half, y + (nx / nl) * half], [z + (ny / nl) * half, y - (nx / nl) * half]];
    });
    const ring = [...sp.map((p) => p[0]), ...sp.reverse().map((p) => p[1])];
    ring.forEach(([z, y], i) => (i ? sl.lineTo(-side * z, y) : sl.moveTo(-side * z, y)));
    sl.closePath();
    shp.holes.push(sl);
    const g = new THREE.ExtrudeGeometry(shp, { depth: plate, bevelEnabled: false, curveSegments: 16 });
    alongX(g);
    g.translate(xr - plate / 2 - fb.x, 0, 0);
    const rk = mesh(`rocker_${tag}`, g, 'carbon_plate', seg0, 'drive', plateI?.material_id);
    // hinge bearings, aft and forward of the rocker
    const hb = [];
    for (const dx of [-0.006, 0.006]) {
      const b = mesh(`hinge_bearing_${tag}_${dx > 0 ? 'f' : 'a'}`, ringX(bd[1] / 2, bd[0] / 2, bd[2]), 'steel_ground', seg0, 'drive', bearing?.id);
      b.position.set(dx - fb.x + xr, 0, 0);
      hb.push(b);
    }
    const pin = mesh(`hinge_pin_${tag}`, cylX(bd[0] / 2, 0.022, 12), 'steel_shaft', seg0, 'drive');
    pin.position.set(xr - fb.x, 0, 0);
    // the coupler pin, a small steel pin that slides in the slot
    const cpin = mesh(`slot_pin_${tag}`, cylX(0.001, 0.008, 10), 'steel_shaft', seg0, 'drive');
    posed.push(cpin);
    register(`rocker ${tag}`, plateI?.material_id, 'crank, rockers and gearbox plates', 'drive', [rk]);
    register(`hinge bearings ${tag}`, bearing?.id, 'bearing', 'drive', hb);
    // coupler: 4 mm rod of length b with an eye at each end
    const cs = spec(coupI?.material_id);
    const cn = group(`coupler_${tag}`, root, 'drive');
    const rodLen = law.b;
    const rod = mesh(`coupler_rod_${tag}`, cylZ(cs.od_mm / 2000, rodLen - 0.008, 16), 'carbon_pultruded', cn, 'drive', coupI?.material_id);
    rod.rotation.x = -Math.PI / 2; // +z -> +y
    rod.position.y = 0.004;
    const eyes = [];
    for (const y of [0, rodLen]) {
      const e = mesh(`coupler_eye_${tag}_${y ? 'top' : 'bot'}`, ringX(0.0042, 0.001, 0.003), 'alu_machined', cn, 'drive');
      e.position.y = y;
      eyes.push(e);
    }
    posed.push(cn);
    register(`coupler ${tag}`, coupI?.material_id, 'couplers', 'drive', [rod, ...eyes]);
    accentLine(`coupler_line_${tag}`, [[0, 0, 0], [0, rodLen, 0]], cn, 'drive', false);
    drive[tag] = { rocker: rk, coupler: cn, pin: cpin, x: xr };
  }
}

/** Crank disc and pin. */
function buildCrank(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const bearing = bomPart('bearing');
  const bd = (spec(bearing?.id).dimensions_mm_d_D_B ?? '4x9x2.5').split('x').map((v) => +v / 1000);
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
  const crankPin = mesh('crank_pin', cylX(0.001, pinLen, 12), 'steel_shaft', crank, 'drive');
  crankPin.position.set(P.crankArm - pinLen / 2 + 0.001, law.a, 0); // pin at +y in the crank frame
  posed.push(crank);
  register('crank', plateI?.material_id, 'crank, rockers and gearbox plates', 'drive', [crankDisc, crankPin]);
  accentLine('crank_circle', Array.from({ length: 48 }, (_, i) => [P.crankArm, law.a * Math.cos((i / 48) * TWO_PI), law.a * Math.sin((i / 48) * TWO_PI)]), crank, 'drive');
  drive.crank = crank;
}

/** Gears, shafts, gearbox plates, bearings and fasteners. */
function buildGears(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const bearing = bomPart('bearing');
  const bd = (spec(bearing?.id).dimensions_mm_d_D_B ?? '4x9x2.5').split('x').map((v) => +v / 1000);
  const plate = 0.002;
  // gears: one node per gear, named by stage (stage 1 is the motor stage); shafts run plate to plate
  drive.gears = [];
  const shaftsN = train.shafts ?? [];
  if (train.links?.length) {
    const gearMesh = (name, gear, centre, x, shaftIndex) => {
      const node = group(name, root, 'drive');
      node.position.set(x, centre[1], centre[0]);
      const isSteel = /SCM|steel/i.test(gear.material);
      const hub = isSteel && gear.hub && gear.hubW > 0;
      const bore = gear.bore || (shaftIndex === shaftsN.length - 1 ? (spec(model.design.motor.id).shaft_mm ?? 5) / 1000 : bd[0]);
      const key = hub ? (bore >= 0.012 ? 0.005 : 0.003) : 0;
      const g = gearGeometry(gear.z, gear.m, gear.face, bore, { hubR: hub ? gear.hub / 2 : 0, hubW: hub ? gear.hubW : 0, key });
      const o = mesh(`${name}_teeth`, g, isSteel ? 'steel_ground' : 'acetal', node, 'drive', gear.id);
      posed.push(node);
      register(`gear ${gear.z}T m${+(gear.m * 1000).toFixed(2)}`, gear.id, 'gear', 'drive', o);
      accentLine(`${name}_pitch`, Array.from({ length: 64 }, (_, i) => [gear.face / 2 + 0.0004, gear.rp * Math.cos((i / 64) * TWO_PI), gear.rp * Math.sin((i / 64) * TWO_PI)]), node, 'drive');
      drive.gears.push({ node, shaft: shaftIndex });
      return node;
    };
    train.links.forEach((_, j) => {
      const si = train.n - 1 - j;
      const st = train.stages[si];
      const x = P.stage[si];
      gearMesh(j === 0 ? 'gear_crank' : `gear_s${si + 1}_wheel`, st.wheel, shaftsN[j], x, j);
      gearMesh(si === 0 ? 'gear_motor' : `gear_s${si + 1}_pinion`, st.pinion, shaftsN[j + 1], x, j + 1);
    });
    drive.gCrank = root.getObjectByName('gear_crank');
    const shaft = (name, centre, x0, x1, r) => {
      const o = mesh(name, cylX(r, x1 - x0, 16), 'steel_shaft', root, 'drive');
      o.position.set((x0 + x1) / 2, centre[1], centre[0]);
      return o;
    };
    shaft('shaft_crank', shaftsN[0], P.crankArm - 0.001, P.plateFwd + 0.002, bd[0] / 2);
    for (let j = 1; j < shaftsN.length - 1; j++) shaft(`shaft_${j}`, shaftsN[j], P.plateAft - 0.002, P.plateFwd + 0.002, bd[0] / 2);
    // gearbox plates: a rounded hull around every shaft, bearing seats for the crank and intermediate shafts
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
        h.absarc(-c[0], c[1], i === shaftsN.length - 1 ? 0.0035 : bd[1] / 2, 0, TWO_PI, true);
        sh.holes.push(h);
      });
      return sh;
    };
    // the BOM's bearings: two per shoulder hinge first, then two per gearbox shaft, then the elbows
    const nBear = bearing?.count ?? 0;
    let bearLeft = Math.max(0, nBear - 4);
    model._elbowBearings = 0;
    const plates = [];
    for (const [name, x] of [['plate_aft', P.plateAft], ['plate_fwd', P.plateFwd]]) {
      const g = new THREE.ExtrudeGeometry(plateShape(), { depth: plate, bevelEnabled: false, curveSegments: 24 });
      g.translate(0, 0, -plate / 2);
      alongX(g);
      const o = mesh(name, g, 'carbon_plate', root, 'drive', plateI?.material_id);
      o.position.x = x;
      plates.push(o);
    }
    for (let j = 0; j < shaftsN.length - 1; j++) for (const [name, x] of [['aft', P.plateAft], ['fwd', P.plateFwd]]) {
      if (bearLeft <= 0) break;
      bearLeft--;
      const b = mesh(`gearbox_bearing_${j}_${name}`, ringX(bd[1] / 2, bd[0] / 2, bd[2]), 'steel_ground', root, 'drive', bearing?.id);
      b.position.set(x, shaftsN[j][1], shaftsN[j][0]);
      plates.push(b);
    }
    model._elbowBearings = bearLeft;
    register('gearbox plates', plateI?.material_id, 'crank, rockers and gearbox plates', 'drive', plates.filter((p) => /plate_(aft|fwd)$/.test(p.name)));
    register('gearbox bearings', bearing?.id, 'bearing', 'drive', plates.filter((p) => /bearing/.test(p.name)));
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
}

/** The flap motor: outrunner bell, mount, shaft to the pinion. */
function buildMotor(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const { wings, drive } = ctx;
  const fb = model.fourBar;
  const law = model.law;
  const plateI = structItem(/crank, rockers/);
  const coupI = structItem(/couplers/);
  const bearing = bomPart('bearing');
  const bd = (spec(bearing?.id).dimensions_mm_d_D_B ?? '4x9x2.5').split('x').map((v) => +v / 1000);
  const plate = 0.002;
  // motor: outrunner bell, mount, shaft to the pinion
  const mp = model.layout.parts.find((p) => p.name === 'motor');
  const motor = group('motor', root, 'drive');
  const r = mp.cylinder.r, L = mp.cylinder.len;
  motor.position.set(mp.box.cx, mp.box.cy, mp.box.cz);
  const bellG = new THREE.LatheGeometry([[0.003, -L / 2], [r * 0.82, -L / 2], [r, -L / 2 + 0.003], [r, L / 2 - 0.006], [r * 0.9, L / 2], [0.004, L / 2]].map(([a, b]) => new THREE.Vector2(a, b)), 64);
  bellG.rotateZ(-Math.PI / 2); // lathe axis y -> x; bell open aft
  const bellNode = group('motor_bell', motor, 'drive');
  const bell = mesh('motor_bell_can', bellG, 'alu_anodised', bellNode, 'drive', mp.id);
  const ring = mesh('motor_magnet_band', ringX(r * 0.985, r * 0.9, L * 0.55, 64), 'magnet', bellNode, 'drive', mp.id);
  ring.position.x = -0.002;
  for (let k = 0; k < 6; k++) {
    const vent = mesh(`motor_vent_${k}`, new THREE.BoxGeometry(0.0015, 0.004, r * 0.55), 'alu_machined', bellNode, 'drive', mp.id);
    const a = (k / 6) * TWO_PI;
    vent.position.set(L / 2 - 0.0005, Math.cos(a) * r * 0.55, Math.sin(a) * r * 0.55);
    vent.rotation.x = -a;
  }
  const mount = mesh('motor_mount', cylX(r * 0.95, 0.004, 48), 'alu_machined', motor, 'drive', mp.id);
  mount.position.x = -L / 2 - 0.0035;
  const pin = P.stage?.[0] ?? mp.box.x0 - 0.01;
  const mshaft = mesh('motor_shaft', cylX((spec(mp.id).shaft_mm ?? 5) / 2000, Math.max(0.01, mp.box.x0 - pin + 0.006), 16), 'steel_shaft', motor, 'drive', mp.id);
  mshaft.position.x = -L / 2 - (mp.box.x0 - pin + 0.006) / 2 + 0.001;
  posed.push(bellNode);
  register('flap motor', mp.id, 'flap motor', 'drive', [bell, ring, mount, mshaft]);
  drive.bell = bellNode;
  drive.motor = motor;
}

/** Box parts at their layout places: battery, ESC, FC, GPS, receiver, airspeed, servos. */
function buildBoxParts(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
  const servos = {};
  for (const p of model.layout.parts) {
    if (p.name === 'motor') continue;
    const b = p.box;
    const node = group(`part_${p.name}`, root, p.subsystem);
    node.position.set(b.cx, b.cy, b.cz);
    let objs = [];
    if (p.name === 'battery') {
      objs.push(mesh('battery_pack', roundedBox(b.sx, b.sy, b.sz, 0.003), 'battery_wrap', node, 'power', p.id));
      const lead = mesh('battery_lead', wire([[b.sx / 2 - 0.004, b.sy / 2, 0.004], [b.sx / 2 - 0.02, b.sy / 2 + 0.004, 0.006], [-b.sx / 2 - 0.01, b.sy / 2 + 0.002, 0.004]], 0.0012), 'wire', node, 'power');
      objs.push(lead);
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
      const arm = mesh(`${p.name}_horn_arm`, new THREE.BoxGeometry(thin === 0 ? 0.002 : 0.004, thin === 1 ? 0.002 : 0.016, thin === 2 ? 0.002 : 0.004), 'alu_machined', horn, p.subsystem, p.id);
      arm.position.set(thin === 0 ? 0 : 0, thin === 1 ? 0 : 0.006, 0);
      objs.push(caseO, arm);
      servos[p.name] = { node, horn, box: b };
      posed.push(horn);
      if (p.name === 'servo_stroke') accentLine('stroke_horn_line', [[0, 0, 0], [0, 0.012, 0]], horn, 'drive', false);
    } else if (p.name === 'gps') {
      objs.push(mesh('gps_puck', roundedBox(b.sx, b.sy, b.sz, Math.min(b.sx, b.sz) * 0.2), 'gps_case', node, p.subsystem, p.id));
    } else {
      objs.push(mesh(`${p.name}_board`, roundedBox(b.sx, b.sy, b.sz, Math.min(b.sx, b.sy, b.sz) * 0.4), 'pcb', node, p.subsystem, p.id));
    }
    register(p.name, p.id, p.role, p.subsystem, objs);
  }
  return servos;
}

/** Wiring (inside the 15% margin). */
function buildWiring(ctx) {
  const { model, mats, D, W, P, train, root, parts, meshes, accents, posed, deform, mesh, group, register, accentLine, bomPart, structItem, spec, filmRows } = ctx;
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
  w.le.geometry.userData.update(lePts);
  w.te.geometry.userData.update(tePts);
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
        const hh = up ? q.cam + half : q.cam - half;
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

function updateBird(ctx, state) {
  const { model, D, W, P, train, root, meshes, accents, mats, wings, drive, servos, tailNodes, pushrods, segBody, ss, filmSt, tmp, ghost, exState } = ctx;
  const phase = state.phase ?? 0, amp = state.amp ?? D.cruise?.amp ?? model.law.ampNominal ?? 0.35;
  const ex = state.explode ?? {};
  const dp = drivePose(model, phase, amp);
  poseWing(ctx, 'R', dp.right.phi, state.twistR ?? 0, ex.wing ?? 0);
  poseWing(ctx, 'L', state.symmetric ? dp.right.phi : dp.left.phi, state.twistL ?? 0, ex.wing ?? 0);
  // rocker slot pins and couplers
  for (const [tag, side] of [['R', 1], ['L', -1]]) {
    // symmetric: both wings at the flight model's stroke angle (as in a flight log); the left pin mirrors the right
    const B = side > 0 ? dp.right.B : state.symmetric ? [-dp.right.B[0], dp.right.B[1]] : dp.left.B;
    const x = drive[tag].x;
    const pinBody = new THREE.Matrix4().makeTranslation(x, B[1], B[0]);
    setLocal(drive[tag].pin, pinBody, segBody[tag][0]);
    const dir = [B[0] - dp.A[0], B[1] - dp.A[1]];
    const ang = Math.atan2(dir[1], dir[0]); // CCW from +z
    const Mc = new THREE.Matrix4().makeTranslation(x, dp.A[1], dp.A[0]).multiply(rotCCW(ang - Math.PI / 2));
    setLocal(drive[tag].coupler, Mc, null);
  }
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
  // servos: stroke servo horn from the amplitude command across the drive's range; tail horns from deflection
  const sv = servos.servo_stroke;
  if (sv) {
    const f = (amp - model.law.ampMin) / (model.law.ampMax - model.law.ampMin);
    poseHorn(sv, (f - 0.5) * (Math.PI / 2));
  }
  for (const [name, defl] of [['servo_tail_L', state.tailL ?? 0], ['servo_tail_R', state.tailR ?? 0]]) if (servos[name]) poseHorn(servos[name], defl);
  // tail surfaces about their quarter-chord axes
  for (const t of tailNodes) {
    const defl = t.side > 0 ? state.tailR ?? 0 : state.tailL ?? 0;
    // nose up is a right-hand turn about the outboard span axis on the right, the reverse on the left
    const q = new THREE.Quaternion().setFromAxisAngle(t.axis, t.side * defl);
    const o = t.origin.clone().add(new V3(-0.16 * (ex.structure ?? 0), 0.03 * (ex.structure ?? 0), 0));
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

function poseHorn(sv, angle) {
  const ax = sv.horn.userData.axis;
  sv.horn.quaternion.setFromAxisAngle(ax, angle);
  sv.horn.matrixAutoUpdate = true;
}

function applyLook(ctx, state) {
  const { model, D, W, P, train, root, meshes, accents, mats, wings, drive, servos, tailNodes, pushrods, segBody, ss, filmSt, tmp, ghost, exState } = ctx;
  const focus = state.focus ?? null;
  const ex = state.explode ?? {};
  for (const o of meshes) {
    const sub = o.userData.subsystem;
    const on = !focus || sub === focus || (focus === 'twist' && sub === 'wing' && /rib|film/.test(o.name));
    const want = on ? mats[o.userData.matName] : ghost;
    if (o.material !== want) o.material = want;
    // out of focus, the film and the edge rods would only veil the part in focus: hide them, ghost the rest
    if (/^(film_|le_rod|te_rod|tail_film)/.test(o.name)) o.visible = on;
    if (o.name === 'shell') {
      o.visible = (state.shell ?? 1) > 0.02 && (!focus || focus === 'structure');
      o.position.y = 0.11 * (ex.structure ?? 0);
    }
  }
  for (const l of accents) l.visible = !!state.accents && (!focus || l.userData.subsystem === focus);
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
  exState.last = state;
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
  return [0, 0, 0];
}

export function buildBird(model, mats, opts = {}) {
  const ctx = makeCtx(model, mats, opts);
  buildFuselage(ctx);
  ctx.tailNodes = buildTail(ctx);
  Object.assign(ctx, buildWings(ctx));
  ctx.drive = {};
  buildLinkage(ctx);
  buildCrank(ctx);
  buildGears(ctx);
  buildMotor(ctx);
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
  const update = (state) => updateBird(ctx, state);
  update({ phase: 0, amp: model.design.cruise?.amp ?? model.law.ampNominal });
  const { root, parts, meshes, posed, deform, accents, servos, drive, wings } = ctx;
  return { root, update, parts, meshes, posed, deform, accents, model, servos, drive, wings };
}


/** Nearest x (within 30 mm) where no part box crosses the former's ring (between the loft and its lightening hole). */
function clearFormerX(model, sec, x) {
  const boxes = model.layout.parts.map((p) => p.box);
  const fb = model.fourBar, P = model.planes;
  const drive = { x0: P.couplerL - 0.008, x1: P.plateFwd + 0.004 };
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
