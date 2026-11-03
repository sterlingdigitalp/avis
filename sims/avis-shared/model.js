// The bird as numbers: everything the page, the film and the GLB need to place and pose a part, derived from the
// design file and the catalogue entries it names. Pure functions, no three.js, node and browser.
//
// Every pose goes through src/physics/kinematics.js and src/physics/geometry.js. What this file adds is placement
// that the design leaves open (where the crank sits, how the gears are stacked, where a box-shaped part goes), each
// derived from the design's own numbers and checked: model.fit lists every part that does not fit the body loft.
//
// Frames (SPEC): body x forward, y up, z right, metres. The drive works in the body's y-z plane; 2D points there
// are written [z, y] ("Z right, Y up", seen from behind the bird), angles counter-clockwise from +z.
import {
  makeWing, makeFlapDrive, strokeAngle, crankRockerSolve, crankRockerLimits, chordAt, xleAt, twistWeightAt,
  baseTwistAt, camberLine, makeSectionState, sectionStates, wingSpan, wingArea,
} from '../../src/physics/kinematics.js';
import { tailPanels, bodySectionAt, bodyMesh } from '../../src/physics/geometry.js';

const TWO_PI = 2 * Math.PI;
export const E387_THICKNESS = 0.0906; // max t/c of the Eppler 387 (published section property); render envelope only

const add2 = (p, q) => [p[0] + q[0], p[1] + q[1]];
const sub2 = (p, q) => [p[0] - q[0], p[1] - q[1]];
const mul2 = (p, k) => [p[0] * k, p[1] * k];
const len2 = (p) => Math.hypot(p[0], p[1]);
const ang2 = (p) => Math.atan2(p[1], p[0]);
const dir2 = (a) => [Math.cos(a), Math.sin(a)];

/** "60 x 25 x 8", "35.0 dia x 40.0" -> numbers in metres, in the order printed. */
export function parseDims(text) {
  if (!text) return null;
  const nums = String(text).match(/[\d.]+/g);
  if (!nums) return null;
  return { dia: /dia/i.test(text), values: nums.map((v) => +v / 1000) };
}

/** Catalogue entries by id, for the parts the design names. */
export function partsIndex(design, catalog) {
  const ids = new Set([...design.bom.parts.map((p) => p.id), ...design.bom.structure.map((s) => s.material_id)]);
  const out = {};
  for (const c of catalog) if (ids.has(c.id)) out[c.id] = { id: c.id, category: c.category, manufacturer: c.manufacturer, name: c.name, mass_g: c.mass_g, specs: c.specs, source_url: c.source_url };
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Wing
// ---------------------------------------------------------------------------------------------------------------

function segIndex(wing, s) {
  const g = wing.segments;
  for (let k = 0; k < g.length - 1; k++) if (s < g[k].s1) return k;
  return g.length - 1;
}

/** Station table for sectionStates() at arbitrary spanwise positions s (m from the hinge). */
export function stationsAt(wing, sList) {
  const n = sList.length;
  const st = { n, s: new Float64Array(n), ds: new Float64Array(n), chord: new Float64Array(n), xle: new Float64Array(n), weight: new Float64Array(n), base: new Float64Array(n), seg: new Int32Array(n) };
  sList.forEach((s, i) => {
    const sc = Math.min(Math.max(s, 0), wing.L);
    st.s[i] = sc;
    st.chord[i] = chordAt(wing, sc);
    st.xle[i] = xleAt(wing, sc);
    st.weight[i] = twistWeightAt(wing, sc);
    st.base[i] = baseTwistAt(wing, sc);
    // A station on a joint belongs to the outer segment except at the tip.
    st.seg[i] = segIndex(wing, Math.min(sc, wing.L - 1e-9));
  });
  return st;
}

/** NACA 4-digit half-thickness per chord at xi, closed trailing edge, scaled to max t/c `t`. */
export function halfThickness(xi, t) {
  const x = Math.min(Math.max(xi, 0), 1);
  return 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
}

/** Chordwise stations, clustered at both edges, and the section shape there (camber and half thickness per chord). */
export function sectionTable(wing, nc = 28) {
  const xi = [];
  for (let i = 0; i <= nc; i++) xi.push(0.5 * (1 - Math.cos((Math.PI * i) / nc)));
  return xi.map((x) => ({ xi: x, cam: camberLine(x, wing.camber, wing.camber_pos).z, half: halfThickness(x, E387_THICKNESS) }));
}

/**
 * Sine coefficients b_n (n = 1..N) of a profile f(xi) on [0, 1] less the straight line through its two ends:
 * f - line = sum b_n sin(n pi xi). f is the piecewise-linear curve through (xs, fs); trapezoid rule on M panels.
 */
export function sineSeries(xs, fs, N = 48, M = 2048) {
  const f0 = fs[0], f1 = fs[fs.length - 1];
  const g = new Float64Array(M + 1);
  for (let j = 0, k = 1; j <= M; j++) {
    const x = j / M;
    while (k < xs.length - 1 && xs[k] < x) k++;
    const t = Math.min(Math.max((x - xs[k - 1]) / (xs[k] - xs[k - 1]), 0), 1);
    g[j] = fs[k - 1] * (1 - t) + fs[k] * t - (f0 + (f1 - f0) * x);
  }
  const b = new Float64Array(N);
  for (let n = 1; n <= N; n++) {
    let s = 0;
    for (let j = 1; j < M; j++) s += g[j] * Math.sin((n * Math.PI * j) / M);
    b[n - 1] = (2 * s) / M; // g(0) = g(1) = 0, so the end panels add nothing
  }
  return b;
}

/**
 * How far a taut film between two ribs lies inside the rib profile, per chord, at chordwise xi and bay fraction u
 * (0 and 1 on the ribs); sigma = rib gap / chord. With equal tension both ways and no air load the film is a
 * minimal surface, which at these slopes solves Laplace's equation over the bay, held to the rib profile on the
 * ribs and to the edge rods at xi = 0 and 1. Harmonic n of the profile (coefficients from sineSeries) falls
 * between the ribs to cosh(n pi sigma (u - 1/2)) / cosh(n pi sigma / 2) of itself; the dip is what it loses.
 */
export function membraneDip(b, xi, u, sigma) {
  let d = 0;
  for (let k = 0; k < b.length; k++) {
    const a = (k + 1) * Math.PI;
    d += b[k] * Math.sin(a * xi) * (1 - Math.cosh(a * sigma * (u - 0.5)) / Math.cosh((a * sigma) / 2));
  }
  return d;
}

/**
 * Pose of a wing at given stations: spar point A, chord vector t (towards the LE), up-normal b, spanwise e, twist.
 * joint = { phi, theta } (stroke angle at the shoulder, commanded twist). Same sectionStates() as the flight model.
 */
export function wingStations(model, st, phi, theta, side) {
  const out = makeSectionState(st.n);
  sectionStates(model.wing, st, { phi, dphi: 0, ddphi: 0, theta, dtheta: 0, ddtheta: 0 }, side, out);
  return out;
}

/**
 * A point of the posed section at station i, chordwise xi, offset h (per chord, along b; + up).
 * Identical composition to kinematics.wingGrid: A + (xle - xi c - spar_x) t + h c b.
 */
export function sectionPoint(model, st, ss, i, xi, h, out = [0, 0, 0]) {
  const c = st.chord[i];
  const dt = st.xle[i] - xi * c - model.wing.spar_x;
  const j = 3 * i;
  for (let k = 0; k < 3; k++) out[k] = ss.A[j + k] + dt * ss.t[j + k] + h * c * ss.b[j + k];
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Drive: the crank-rocker placed in the body, both wings from one crank pin
// ---------------------------------------------------------------------------------------------------------------

/**
 * Embeds the design's four-bar in the body. The rocker pivot O4 is the shoulder hinge. The crank pivot O2 is on
 * the centreline (one crank drives both wings), at the ground length d from O4, below the hinge. The four-bar's
 * own frame has x from O2 to O4 and y 90 degrees counter-clockwise (inboard for the right wing).
 */
export function placeFourBar(design, law) {
  const [hx, hy, hz] = design.wing.hinge;
  const d = law.d;
  if (d <= hz) throw new Error(`ground link ${d} m cannot reach a hinge ${hz} m off the centreline`);
  const O4 = [hz, hy];
  const O2 = [0, hy - Math.sqrt(d * d - hz * hz)];
  const u = mul2(sub2(O4, O2), 1 / d);
  const v = [-u[1], u[0]];
  return { x: hx, O2, O4, u, v, alpha: ang2(u) };
}

const toBody = (fb, p) => add2(fb.O2, add2(mul2(fb.u, p[0]), mul2(fb.v, p[1])));
const mirror = (p) => [-p[0], p[1]];

/** thetaTop and c for an amplitude, from the exact linkage (kinematics strokeAngle and crankRockerLimits). */
export function linkageAt(law, amp) {
  const s0 = strokeAngle(0, amp, law);
  const lim = crankRockerLimits({ a: law.a, b: law.b, c: s0.c, d: law.d });
  return { c: s0.c, thetaTop: lim.thetaAtPsiMax, psiMid: 0.5 * (lim.psiMin + lim.psiMax) };
}

/**
 * The drive at a phase and amplitude. The right wing follows the flight model exactly (strokeAngle). The left
 * four-bar is the mirror image, driven by the same crank pin, so its rocker angle comes from crankRockerSolve at
 * the crank angle the pin makes in the mirrored frame. leftDelta is the difference this makes to the left stroke
 * angle; the flight model assumes it is zero.
 */
export function drivePose(model, phase, amp) {
  const { law, fourBar: fb } = model;
  const { c, thetaTop, psiMid } = linkageAt(law, amp);
  const theta = thetaTop + phase; // crank angle in the right four-bar's frame
  const sR = strokeAngle(phase, amp, law);
  const rR = crankRockerSolve({ a: law.a, b: law.b, c, d: law.d }, theta);
  const A = toBody(fb, [law.a * Math.cos(theta), law.a * Math.sin(theta)]);
  const BR = toBody(fb, [law.d + c * Math.cos(rR.psi), c * Math.sin(rR.psi)]);
  // Left: the pin in the mirrored frame.
  const Am = mirror(A);
  const rel = sub2(Am, fb.O2);
  const thetaL = Math.atan2(rel[0] * fb.v[0] + rel[1] * fb.v[1], rel[0] * fb.u[0] + rel[1] * fb.u[1]);
  const rL = crankRockerSolve({ a: law.a, b: law.b, c, d: law.d }, thetaL);
  let psiL = rL ? rL.psi : rR.psi;
  // keep psiL on the same branch as psi (crankRockerSolve returns angles in a 0..2pi based window)
  while (psiL - rR.psi > Math.PI) psiL -= TWO_PI;
  while (rR.psi - psiL > Math.PI) psiL += TWO_PI;
  const phiL = law.phiMean + (psiL - psiMid);
  const BL = mirror(toBody(fb, [law.d + c * Math.cos(psiL), c * Math.sin(psiL)]));
  const crankBody = fb.alpha + theta; // absolute crank angle in the body plane, counter-clockwise from +z
  return {
    crank: crankBody, theta, c, A, O2: fb.O2,
    right: { O4: fb.O4, B: BR, phi: sR.phi, rocker: ang2(sub2(BR, fb.O4)), mu: sR.mu },
    left: { O4: mirror(fb.O4), B: BL, phi: phiL, rocker: ang2(sub2(BL, mirror(fb.O4))) },
    leftDelta: phiL - sR.phi,
    couplerR: len2(sub2(BR, A)), couplerL: len2(sub2(BL, A)),
  };
}

/** A catalogue gear as numbers (m, teeth, radii, face, bore, hub). */
function gearOf(parts, id) {
  const s = parts[id]?.specs ?? {};
  const m = s.module_mm / 1000, z = s.teeth;
  return { id, m, z, rp: (m * z) / 2, ro: s.outside_dia_mm ? s.outside_dia_mm / 2000 : (m * (z + 2)) / 2, face: (s.face_width_mm ?? 4) / 1000, bore: (s.bore_mm ?? 0) / 1000, hub: s.hub_dia_mm ? s.hub_dia_mm / 1000 : null, hubW: (s.hub_width_mm ?? 0) / 1000, material: s.material ?? '', mass_g: parts[id]?.mass_g };
}

/**
 * Gear train from the catalogue (module, teeth, widths). The BOM lists gears in pairs, [pinion, wheel] per stage,
 * from the motor to the crank. All axes run along body x; every stage has its own x plane. Shaft 0 is the crank;
 * link j joins shaft j (the wheel of stage n-1-j) to shaft j+1 (its pinion, and the next stage's wheel); the last
 * shaft is the motor's. The link directions are chosen to keep every tip circle, the shafts and the motor inside the
 * body loft as far as the numbers allow (the design does not say how the stages are folded); what still sticks out
 * is reported by layoutParts.
 */
export function gearTrain(design, parts, fourBar) {
  const ids = design.drive.gears ?? design.params?.parts?.gears ?? [];
  const g = ids.map((id) => gearOf(parts, id));
  const n = Math.floor(g.length / 2);
  if (n < 1) return { gears: g, stages: [], shafts: [], links: [], ratio: design.drive.gearRatio, ratioDesign: design.drive.gearRatio };
  const stages = [];
  for (let k = 0; k < n; k++) stages.push({ pinion: g[2 * k], wheel: g[2 * k + 1], centreDistance: g[2 * k].rp + g[2 * k + 1].rp });
  const ratio = stages.reduce((r, st) => r * (st.wheel.z / st.pinion.z), 1);
  const train = { gears: g, stages, ratio, ratioDesign: design.drive.gearRatio, n };
  train.planes = stagePlanes(design, train);
  const motorSpec = parts[design.motor?.id ?? design.params?.parts?.motor]?.specs;
  const md = parseDims(motorSpec?.dimensions_text_mm)?.values ?? [0.035, 0.04];
  const lumpM = design.mass.lumps.find((l) => l.name === 'flap motor')?.pos ?? [0.04, 0, 0];
  const motorX = [train.planes.motorAft, train.planes.motorAft + md[1]];
  const loft = design.body.loft;
  const chain = (ang) => {
    const pts = [fourBar.O2];
    for (let j = 0; j < n; j++) pts.push(add2(pts[j], mul2(dir2(ang[j]), stages[n - 1 - j].centreDistance)));
    return pts;
  };
  const outAt = (x, c, r) => {
    const h = loftHalf(loft, x);
    if (!h) return r;
    let worst = -Infinity;
    for (let k = 0; k < 24; k++) {
      const a = (k / 24) * TWO_PI;
      const rr = Math.hypot((c[1] + r * Math.sin(a) - h.y) / h.hh, (c[0] + r * Math.cos(a)) / h.hw);
      worst = Math.max(worst, (rr - 1) * Math.min(h.hh, h.hw));
    }
    return worst;
  };
  const cost = (ang) => {
    const pts = chain(ang);
    let c = 0;
    for (let j = 0; j < n; j++) {
      const st = stages[n - 1 - j];
      const x = train.planes.stage[n - 1 - j];
      // the crank wheel cannot leave the crank (it hangs where the four-bar puts it): count it, but it is fixed
      c += Math.max(0, outAt(x, pts[j], st.wheel.ro)) + Math.max(0, outAt(x, pts[j + 1], st.pinion.ro));
      // shafts pass through every plane between the plates: keep them clear of the other stages' gears
      for (let q = 0; q < n; q++) {
        if (q === n - 1 - j || q === n - j) continue;
        const sq = stages[q], jq = n - 1 - q;
        for (const [centre, gear] of [[pts[jq], sq.wheel], [pts[jq + 1], sq.pinion]]) {
          for (const shaft of [pts[j], pts[j + 1]]) {
            if (shaft === centre) continue;
            const d = len2(sub2(shaft, centre));
            if (d < gear.ro + 0.003) c += 4 * (gear.ro + 0.003 - d);
          }
        }
      }
    }
    const m = pts[n];
    c += 2 * Math.max(0, Math.max(outAt(motorX[0], m, md[0] / 2), outAt(motorX[1], m, md[0] / 2)));
    c += 0.02 * len2(sub2(m, [lumpM[2], lumpM[1]])) + 0.01 * Math.abs(m[0]);
    return c;
  };
  let best = null;
  const starts = [];
  for (const a0 of [0, 20, 40, 60]) starts.push(Array.from({ length: n }, (_, j) => (90 + (j % 2 ? -a0 : a0)) / 57.29578), Array.from({ length: n }, (_, j) => (90 + (j % 2 ? a0 : -a0)) / 57.29578));
  for (const s0 of starts) {
    const ang = s0.slice();
    let c = cost(ang);
    for (let step = 0.5; step > 0.002; step /= 2) {
      let improved = true;
      while (improved) {
        improved = false;
        for (let j = 0; j < n; j++) for (const d of [-step, step]) {
          ang[j] += d;
          const cc = cost(ang);
          if (cc < c - 1e-9) {
            c = cc;
            improved = true;
          } else ang[j] -= d;
        }
      }
    }
    if (!best || c < best.c) best = { c, ang: ang.slice() };
  }
  train.links = best.ang;
  train.shafts = chain(best.ang);
  train.centres = { crank: train.shafts[0], motor: train.shafts[n] };
  return train;
}

/** Thickness along x of the crank web: the design's (params.linkage.crank.web_t_mm), else the 2 mm crank disc. */
export function crankWebThickness(design) {
  return (design.params?.linkage?.crank?.web_t_mm ?? 2) / 1000;
}

/** Length along x of a coupler eye: its bushing's (flange included), at least 3 mm. */
export function couplerEyeLength(design, parts) {
  const j = journals(design).find((x) => x.place === 'coupler');
  return Math.max(0.003, j && parts?.[j.id] ? journalDims(parts[j.id].specs).B : 0);
}

/**
 * Film only: where the two couplers are drawn on the crank pin. The physics' planes (stagePlanes, which src/design
 * sizes the crank pin from) are kept wherever their eyes clear each other and the web. Where they would overlap
 * (v21: two 6 mm eyes on planes 5 mm apart, 1 mm into each other), the right eye is drawn against the web's aft face
 * and the left eye against the right one, so both stay on the design's own crank pin. The physics never reads this.
 */
export function filmCouplerPlanes(design, parts, planes) {
  const eye = couplerEyeLength(design, parts);
  const couplerR = Math.min(planes.couplerR, planes.crankArm - crankWebThickness(design) / 2 - eye / 2);
  const couplerL = Math.min(planes.couplerL, couplerR - eye);
  // the crank pin as the design sizes it (src/design/linkage.js crankMembers): to 1 mm past the physics' left eye
  const pinEnd = Math.min(planes.couplerL - eye / 2 - 0.001, couplerL - eye / 2);
  return { couplerR, couplerL, couplerEye: eye, crankPinEnd: pinEnd };
}

/** x planes of the drive: crank disc at the hinge, aft plate, one plane per stage (crank wheel first), front plate. */
function stagePlanes(design, train) {
  const x0 = design.wing.hinge[0];
  const plateT = 0.002;
  const crankArm = x0 + 0.0015;
  const plateAft = crankArm + 0.004;
  const stage = new Array(train.n);
  let x = plateAft + plateT / 2 + 0.001;
  for (let s = train.n - 1; s >= 0; s--) {
    const st = train.stages[s];
    const w = Math.max(st.wheel.face, st.pinion.face);
    const hub = Math.max(st.wheel.hubW, st.pinion.hubW);
    stage[s] = x + w / 2;
    x += w + hub + 0.0015;
  }
  const plateFwd = x + plateT / 2;
  return { crankArm, plateAft, stage, plateFwd, plateT, motorAft: plateFwd + plateT / 2 + 0.0015, couplerR: x0 - 0.0045, couplerL: x0 - 0.0095 };
}

/**
 * Gear angles (counter-clockwise in the [z, y] plane; at angle 0 a gear has a tooth on +z). For a mesh whose line
 * of centres points at beta from gear 1 to gear 2: when a tooth of gear 1 points at beta, a gap of gear 2 points
 * back at it, and gear 2 turns the other way at z1/z2 of the speed.
 */
export function meshAngle(theta1, z1, z2, beta = Math.PI / 2) {
  return -(z1 / z2) * (theta1 - beta) + beta + Math.PI + Math.PI / z2;
}

/** Angle of every shaft from the crank angle, crank first, motor last. */
export function gearAngles(train, crank) {
  const out = [crank];
  for (let j = 0; j < (train.links?.length ?? 0); j++) {
    const st = train.stages[train.n - 1 - j];
    out.push(meshAngle(out[j], st.wheel.z, st.pinion.z, train.links[j]));
  }
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Parts layout: catalogue boxes at the design's mass-lump positions, checked against the loft
// ---------------------------------------------------------------------------------------------------------------

/** Half extents of the loft at x, or null outside it. */
function loftHalf(loft, x) {
  const s = bodySectionAt(loft, x);
  return s ? { y: s.y_m, hw: s.width_m / 2, hh: s.height_m / 2 } : null;
}

/** Worst protrusion (m, > 0 outside) of a box [x0..x1] x [y0..y1] x [z0..z1] beyond the elliptic loft. */
export function boxProtrusion(loft, box) {
  let worst = -Infinity;
  for (let k = 0; k <= 8; k++) {
    const x = box.x0 + ((box.x1 - box.x0) * k) / 8;
    const h = loftHalf(loft, x);
    if (!h) return { out: Infinity, at: x };
    for (const y of [box.y0, box.y1]) for (const z of [box.z0, box.z1]) {
      const r = Math.hypot((y - h.y) / Math.max(h.hh, 1e-6), z / Math.max(h.hw, 1e-6));
      // radial distance outside the ellipse, scaled back to metres along the worse axis
      const out = (r - 1) * Math.min(h.hh, h.hw);
      if (out > worst) worst = out;
    }
  }
  return { out: worst };
}

const lump = (design, name) => design.mass.lumps.find((l) => l.name === name);

/** A box part centred at (x, y, z) with sizes along body x, y, z. */
function boxAt(x, y, z, sx, sy, sz) {
  return { x0: x - sx / 2, x1: x + sx / 2, y0: y - sy / 2, y1: y + sy / 2, z0: z - sz / 2, z1: z + sz / 2, cx: x, cy: y, cz: z, sx, sy, sz };
}

/**
 * Where each box-shaped part goes. Start at its mass lump in the orientation given; if it does not fit the loft
 * there (or overlaps a part already placed), try the other orientations and slide it, along x up to 60 mm and in y up
 * to 15 mm, to the nearest place it fits. The shift is recorded, so a CG change is reported, not hidden. If nothing
 * fits, the least bad place is kept and its protrusion reported.
 */
function placeBox(loft, name, at, size, others, { rotate = true } = {}) {
  const clash = (b) => others.some((o) => b.x0 < o.x1 - 1e-4 && b.x1 > o.x0 + 1e-4 && b.y0 < o.y1 - 1e-4 && b.y1 > o.y0 + 1e-4 && b.z0 < o.z1 - 1e-4 && b.z1 > o.z0 + 1e-4);
  const [a0, a1, a2] = size;
  const orients = rotate ? [[a0, a1, a2], [a0, a2, a1], [a1, a0, a2], [a1, a2, a0], [a2, a0, a1], [a2, a1, a0]] : [size];
  const score = (b) => boxProtrusion(loft, b).out + (clash(b) ? 1 : 0);
  let best = null;
  for (let k = 0; k <= 60; k++) {
    for (const dy of [0, -0.003, 0.003, -0.006, 0.006, -0.01, 0.01, -0.015, 0.015]) {
      for (const sgn of k ? [-1, 1] : [1]) {
        for (let o = 0; o < orients.length; o++) {
          const sz = orients[o];
          const b = boxAt(at[0] + sgn * k * 0.001, at[1] + dy, at[2], sz[0], sz[1], sz[2]);
          const sc = score(b);
          const cost = k + Math.abs(dy) * 1000 + o * 0.5;
          if (sc <= 0.0005 && (!best || best.sc > 0.0005 || cost < best.cost)) best = { b, sc, cost, o };
          else if (!best || (best.sc > 0.0005 && sc < best.sc)) best = { b, sc, cost, o };
        }
      }
    }
    if (best && best.sc <= 0.0005) break;
  }
  const bx = best.b;
  return { name, box: bx, orient: orients[best.o], shift: [bx.cx - at[0], bx.cy - at[1], bx.cz - at[2]], protrusion: Math.max(0, best.sc > 0.5 ? best.sc - 1 : best.sc), clash: best.sc > 0.5 };
}

/**
 * The stroke servo, if the design has a mass lump for one, and the two tail servos side by side. Both servo kinds are
 * sized from the stroke servo's catalogue entry (or a 33 x 10 x 35 mm default without one), as they always were:
 * src/design/fit.js packs with this function and passes a massless 'stroke servo' lump for a fixed-stroke drive, so
 * these boxes must not change under it. The raw design of a fixed-stroke drive has no such lump, and the bird no
 * stroke servo.
 */
export const SERVO_DEFAULT_DIMS = [0.033, 0.01, 0.035];

function placeServos(design, put, spec) {
  const dims = (role) => parseDims(spec(role)?.dimensions_text_mm)?.values;
  const sv = dims('stroke amplitude servo') ?? SERVO_DEFAULT_DIMS;
  const st = lump(design, 'stroke servo');
  if (st) put('servo_stroke', 'stroke amplitude servo', [sv[0], sv[2], sv[1]], st.pos, 'drive');
  // Tail servos are sized from their own catalogue entry; the stroke servo's is only a fallback.
  const tv = dims('tail servo') ?? sv;
  const ts = lump(design, 'tail servos');
  put('servo_tail_L', 'tail servo', [tv[0], tv[2], tv[1]], [ts.pos[0], ts.pos[1], ts.pos[2] - tv[1] * 0.55], 'avionics');
  put('servo_tail_R', 'tail servo', [tv[0], tv[2], tv[1]], [ts.pos[0], ts.pos[1], ts.pos[2] + tv[1] * 0.55], 'avionics');
}

/** The BOM role of the flight controller. An ESC role may mention the flight controller, so ESC roles never match. */
export function flightControllerRole(bom) {
  const roles = bom.parts.map((p) => p.role);
  return roles.find((r) => /^flight controller/.test(r)) ?? roles.find((r) => /flight controller/.test(r) && !/\bESC\b/.test(r));
}

/** Sizes from catalogue text: a box [x, y, z]. Unknown third dimensions come from mass at a stated density. */
/** A catalogue entry's printed size: dimensions_text_mm, or dimensions_text with its unit ("1 x 1 x 0.375 in"). */
export function catalogueDims(spec) {
  if (spec?.dimensions_text_mm) return parseDims(spec.dimensions_text_mm);
  const text = String(spec?.dimensions_text ?? '');
  const k = { in: 25.4, inch: 25.4, inches: 25.4, cm: 10, mm: 1 }[text.match(/([a-z]+)\s*$/i)?.[1].toLowerCase()];
  const d = parseDims(text);
  return d && k ? { ...d, values: d.values.map((v) => v * k) } : null;
}

export function sizeOf(spec, mass_g, order, assumeDensity, assumptions, label) {
  const d = parseDims(spec?.dimensions_text_mm);
  if (d && d.values.length >= 3) return order(d.values);
  const known = [];
  if (spec?.width_mm) known.push(spec.width_mm / 1000);
  if (spec?.height_mm) known.push(spec.height_mm / 1000);
  if (known.length === 2) {
    const t = mass_g / 1000 / (assumeDensity * known[0] * known[1]);
    assumptions.push(`${label}: thickness ${(t * 1000).toFixed(1)} mm from ${mass_g} g at ${assumeDensity} kg/m3 (catalogue lists two dimensions)`);
    return order([...known, t]);
  }
  const side = Math.cbrt(mass_g / 1000 / assumeDensity / 0.2);
  assumptions.push(`${label}: no dimensions in the catalogue; drawn as a ${(side * 1000).toFixed(0)} x ${(side * 1000).toFixed(0)} x ${(side * 200).toFixed(1)} mm board from ${mass_g} g at ${assumeDensity} kg/m3`);
  return order([side, side, side * 0.2]);
}

/** The box the drive sweeps (crank, couplers, every gear stage, plate to plate), which no packed part may enter. */
function driveReserve(train, fourBar, planes) {
  let y0 = Infinity, y1 = -Infinity, z0 = -(fourBar.O4[0] + 0.004), z1 = fourBar.O4[0] + 0.004;
  train.links.forEach((_, j) => {
    const st = train.stages[train.n - 1 - j];
    for (const [c, r] of [[train.shafts[j], st.wheel.ro], [train.shafts[j + 1], st.pinion.ro]]) {
      y0 = Math.min(y0, c[1] - r);
      y1 = Math.max(y1, c[1] + r);
      z0 = Math.min(z0, c[0] - r);
      z1 = Math.max(z1, c[0] + r);
    }
  });
  return { x0: planes.couplerL - 0.004, x1: planes.plateFwd + planes.plateT / 2, y0, y1, z0, z1 };
}

export function layoutParts(design, parts, train, fourBar) {
  const loft = design.body.loft;
  const P = (role) => design.bom.parts.find((p) => p.role === role);
  const spec = (role) => parts[P(role)?.id]?.specs;
  const assumptions = [];
  const out = [];
  const FR4 = 1850; // kg/m3, glass-epoxy board, for boards whose catalogue entry lists fewer than three sizes
  const placed = [];
  const put = (name, role, size, at, subsystem, extra = {}) => {
    const p = placeBox(loft, name, at, size, placed);
    placed.push(p.box);
    out.push({ ...p, role, id: P(role)?.id, subsystem, ...extra });
    return p;
  };
  // Motor: a cylinder along x, its shaft aft to the stage-1 pinion.
  const md = parseDims(spec('flap motor')?.dimensions_text_mm)?.values ?? [0.035, 0.04];
  const ml = lump(design, 'flap motor');
  const motorC = train.centres?.motor ?? [0, 0];
  const planes = gearPlanes(design, train);
  const motorX = Math.max(ml.pos[0], planes.motorAft + md[1] / 2);
  const motorBox = boxAt(motorX, motorC[1], motorC[0], md[1], md[0], md[0]);
  out.push({ name: 'motor', role: 'flap motor', id: P('flap motor').id, subsystem: 'drive', box: motorBox, shift: [motorX - ml.pos[0], motorC[1] - ml.pos[1], motorC[0] - ml.pos[2]], protrusion: Math.max(0, boxProtrusion(loft, motorBox).out), cylinder: { r: md[0] / 2, len: md[1] } });
  placed.push(motorBox);
  // The drive's swept volume is kept clear: crank, couplers and both gear stages, plate to plate.
  if (train.links?.length) placed.push(driveReserve(train, fourBar, planes));
  // Battery: long axis along x, at the design's battery station.
  const bat = lump(design, 'battery');
  put('battery', 'battery', sizeOf(spec('battery'), P('battery').unit_mass_g, (v) => [v[0], v[2], v[1]], 2000, assumptions, 'battery'), bat.pos, 'power');
  // ESC under the drive, long axis along x.
  const esc = lump(design, 'ESC');
  const escRole = design.bom.parts.find((p) => /ESC/.test(p.role)).role;
  put('esc', escRole, sizeOf(spec(escRole), P(escRole).unit_mass_g, (v) => [v[0], v[2], v[1]], FR4, assumptions, 'ESC'), esc.pos, 'power');
  // Flight controller flat.
  const fc = lump(design, 'flight controller');
  const fcRole = flightControllerRole(design.bom);
  put('fc', fcRole, sizeOf(spec(fcRole), P(fcRole).unit_mass_g, (v) => [v[0], v[2], v[1]], FR4, assumptions, 'flight controller'), fc.pos, 'avionics');
  // GPS on top.
  const gps = lump(design, 'GPS');
  put('gps', 'GPS', sizeOf(spec('GPS'), P('GPS').unit_mass_g, (v) => [v[0], v[2], v[1]], FR4, assumptions, 'GPS'), gps.pos, 'avionics');
  // Receiver and airspeed board share a lump.
  const rx = lump(design, 'receiver and airspeed');
  put('receiver', 'receiver', sizeOf(spec('receiver'), P('receiver').unit_mass_g, (v) => [v[0], v[2], v[1]], FR4, assumptions, 'receiver'), [rx.pos[0], rx.pos[1] + 0.008, rx.pos[2]], 'avionics');
  put('airspeed', 'airspeed sensor', sizeOf(spec('airspeed sensor'), P('airspeed sensor').unit_mass_g, (v) => [v[0], v[2], v[1]], FR4, assumptions, 'airspeed sensor'), [rx.pos[0], rx.pos[1] - 0.008, rx.pos[2]], 'avionics');
  placeServos(design, put, spec);
  // Gears: report how far each tip circle leaves the loft at its plane.
  const gearFit = [];
  for (let j = 0; j < (train.links?.length ?? 0); j++) {
    const si = train.n - 1 - j;
    const st = train.stages[si];
    const x = planes.stage[si];
    for (const [name, gear, c] of [[`stage ${si + 1} wheel ${st.wheel.z}T`, st.wheel, train.shafts[j]], [`stage ${si + 1} pinion ${st.pinion.z}T`, st.pinion, train.shafts[j + 1]]]) {
      const h = loftHalf(loft, x);
      let worst = -Infinity;
      for (let k = 0; k < 72; k++) {
        const a = (k / 72) * TWO_PI;
        const z = c[0] + gear.ro * Math.cos(a), y = c[1] + gear.ro * Math.sin(a);
        const r = h ? Math.hypot((y - h.y) / h.hh, z / h.hw) : Infinity;
        worst = Math.max(worst, (r - 1) * (h ? Math.min(h.hh, h.hw) : 1));
      }
      gearFit.push({ name, id: gear.id, x, protrusion: Math.max(0, worst) });
    }
  }
  // Crank axis against the loft bottom.
  const hC = loftHalf(loft, fourBar.x);
  const crankBelow = hC ? (hC.y - hC.hh) - fourBar.O2[1] : null;
  return { parts: out, gearFit, crankBelowLoft_m: crankBelow, assumptions };
}

/** Body-x planes of the drive parts (see stagePlanes). */
export function gearPlanes(design, train) {
  return train?.planes ?? stagePlanes(design, { n: 0, stages: [] });
}

// ---------------------------------------------------------------------------------------------------------------
// The model
// ---------------------------------------------------------------------------------------------------------------

/** Rib stations: the BOM's rib count shared equally between the wings, evenly from root to tip. */
export function ribStations(design, wing) {
  const ribItem = design.bom.structure.find((s) => /^ribs/.test(s.name));
  const total = +(ribItem?.name.match(/(\d+)\s*off/)?.[1] ?? 18);
  const per = Math.max(2, Math.round(total / 2));
  return Array.from({ length: per }, (_, k) => (k / (per - 1)) * wing.L);
}

/**
 * Where the BOM's bearings and bushings go. The design names the joint each one carries: a role written
 * "bushing: shoulder pivots, two per side (4), coupler eyes, two per coupler (4)" or "bearing: gear shaft 3 (2)"
 * gives one entry per place, `{ id, kind, place, shaft, count }`, place one of shoulder, coupler, crank, shaft (gear
 * shaft `shaft`, numbered from the crank), elbow. A bare "bearing" role (older designs) is one entry with place
 * 'pooled': two per shoulder hinge, then two per gearbox shaft, then the elbows, as those designs were drawn.
 */
export function journals(design) {
  const out = [];
  for (const p of design.bom.parts) {
    const m = p.role.match(/^(bearing|bushing)\b:?\s*(.*)$/);
    if (!m) continue;
    if (!m[2]) {
      out.push({ id: p.id, kind: m[1], place: 'pooled', shaft: null, count: p.count });
      continue;
    }
    for (const where of m[2].split(/\),\s*/)) {
      const shaft = where.match(/gear shaft\s*(\d+)/);
      const place = /shoulder/.test(where) ? 'shoulder' : /coupler/.test(where) ? 'coupler' : /crank/.test(where) ? 'crank' : shaft ? 'shaft' : /elbow/.test(where) ? 'elbow' : 'other';
      out.push({ id: p.id, kind: m[1], place, shaft: shaft ? +shaft[1] : null, count: +(where.match(/\((\d+)\)?\s*$/)?.[1] ?? 0) });
    }
  }
  return out;
}

/** Bore, outside diameter and width (m) of a bearing ("d x D x B") or a flanged bushing (d1, d2, b1; flange d3, b2). */
export function journalDims(specs) {
  if (specs?.dimensions_mm_d_D_B) {
    const [d, D, B] = specs.dimensions_mm_d_D_B.split('x').map((v) => +v / 1000);
    return { d, D, B, flangeD: 0, flangeB: 0 };
  }
  if (specs?.d1_mm) return { d: specs.d1_mm / 1000, D: specs.d2_mm / 1000, B: specs.b1_mm / 1000, flangeD: (specs.d3_mm ?? 0) / 1000, flangeB: (specs.b2_mm ?? 0) / 1000 };
  return { d: 0.004, D: 0.009, B: 0.0025, flangeD: 0, flangeB: 0 };
}

export function prepare(design, parts) {
  const wing = makeWing(design.wing);
  const law = design.drive.kind === 'crank-rocker' ? makeFlapDrive(design.drive) : { kind: 'sine', phiMean: design.drive.phiMean ?? 0 };
  const fourBar = placeFourBar(design, law);
  const train = gearTrain(design, parts, fourBar);
  const ribs = ribStations(design, wing);
  const twistServo = design.mass.wing.point.find((p) => /twist servo/.test(p.name));
  const model = {
    design, parts, wing, law, fourBar, train, ribs,
    ribSt: stationsAt(wing, ribs),
    section: sectionTable(wing),
    tail: tailPanels(design.tail),
    loft: design.body.loft,
    loftMesh: bodyMesh(design.body.loft, 48),
    planes: gearPlanes(design, train),
    twistServo: twistServo ? { s: twistServo.s_m, d: twistServo.d_m ?? 0, st: stationsAt(wing, [twistServo.s_m]) } : null,
    span: wingSpan(wing),
    area: 2 * wingArea(wing),
  };
  model.layout = layoutParts(design, parts, train, fourBar);
  model.fit = fitReport(model);
  return model;
}

/**
 * The film's model: prepare() exactly as src/design uses it, plus the parts only the film packs. The servo regulator
 * (v21 on), sized from its catalogue entry (printed in inches), is packed last at its mass lump so no other part
 * moves. src/design packs with prepare()/layoutParts alone, so nothing here reaches a number the physics computes.
 */
export function prepareFilm(design, parts) {
  const model = prepare(design, parts);
  packRegulator(model, design, parts);
  model.planes = { ...model.planes, ...filmCouplerPlanes(design, parts, model.planes) };
  return model;
}

/** The servo regulator, packed last against the physics' layout so no other part moves (film only). */
function packRegulator(model, design, parts) {
  const reg = design.bom.parts.find((p) => p.role === 'servo regulator');
  const at = lump(design, 'power module and regulator');
  if (!reg || !at) return;
  const d = catalogueDims(parts[reg.id]?.specs);
  const size = d?.values.length >= 3 ? [d.values[0], d.values[2], d.values[1]] : sizeOf(parts[reg.id]?.specs, reg.unit_mass_g, (v) => [v[0], v[2], v[1]], 1850, model.layout.assumptions, 'servo regulator');
  const boxes = model.layout.parts.map((p) => p.box);
  if (model.train.links?.length) boxes.push(driveReserve(model.train, model.fourBar, model.planes));
  model.layout.parts.push({ ...placeBox(design.body.loft, 'regulator', at.pos, size, boxes), role: reg.role, id: reg.id, subsystem: 'power' });
  model.fit = fitReport(model);
}

/** Plain-language list of what does not fit, with numbers, for the page and the report. */
export function fitReport(model) {
  const issues = [];
  const L = model.layout;
  if (L.crankBelowLoft_m != null && L.crankBelowLoft_m > 0) issues.push({ part: 'crank axis', text: `crank axis ${(L.crankBelowLoft_m * 1000).toFixed(0)} mm below the body loft (ground link ${(model.law.d * 1000).toFixed(0)} mm from a hinge ${(model.design.wing.hinge[2] * 1000).toFixed(0)} mm off centre)`, mm: L.crankBelowLoft_m * 1000 });
  for (const g of L.gearFit) if (g.protrusion > 0.0005) issues.push({ part: g.name, text: `${g.name} (${g.id}) tip circle ${(g.protrusion * 1000).toFixed(0)} mm outside the loft`, mm: g.protrusion * 1000 });
  for (const p of L.parts) {
    if (p.protrusion > 0.0005) issues.push({ part: p.name, text: `${p.name} ${(p.protrusion * 1000).toFixed(0)} mm outside the loft${Math.abs(p.shift[0]) > 1e-4 ? ` after a ${(p.shift[0] * 1000).toFixed(0)} mm slide along x` : ''}`, mm: p.protrusion * 1000 });
    if (p.protrusion <= 0.0005 && Math.abs(p.shift[0]) > 1e-4) issues.push({ part: p.name, text: `${p.name} moved ${(p.shift[0] * 1000).toFixed(0)} mm along x from its mass lump to fit`, mm: 0, shift: p.shift[0] });
    if (p.clash) issues.push({ part: p.name, text: `${p.name} overlaps another part`, mm: 0 });
  }
  // CG shift from the slides, for physics.
  let dm = 0;
  for (const p of L.parts) {
    const part = model.design.bom.parts.find((b) => b.id === p.id);
    if (part && Math.abs(p.shift[0]) > 1e-4) dm += (part.unit_mass_g / 1000) * p.shift[0];
  }
  return { issues, cgShift_m: dm / model.design.mass.total_kg };
}

/** Max |left - right| stroke angle over a cycle from one crank pin (rad). */
export function leftRightAsymmetry(model, amp, n = 360) {
  let worst = 0;
  for (let i = 0; i < n; i++) worst = Math.max(worst, Math.abs(drivePose(model, (i / n) * TWO_PI, amp).leftDelta));
  return worst;
}
