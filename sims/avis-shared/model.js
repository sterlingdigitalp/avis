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
/** The design's gear ids, [pinion, wheel] per stage from the motor (drive.gears, else params.parts.gears). */
const trainIds = (design) => design.drive.gears ?? design.params?.parts?.gears ?? [];

/** What the fold needs of the motor: its catalogue [diameter, length] (35 x 40 mm without one) and its mass lump's position. */
export function motorOf(design, parts) {
  const spec = parts[design.motor?.id ?? design.params?.parts?.motor]?.specs;
  return { md: parseDims(spec?.dimensions_text_mm)?.values ?? [0.035, 0.04], lumpM: design.mass.lumps.find((l) => l.name === 'flap motor')?.pos ?? [0.04, 0, 0] };
}

export function gearTrain(design, parts, fourBar) {
  const g = trainIds(design).map((id) => gearOf(parts, id));
  const n = Math.floor(g.length / 2);
  if (n < 1) return { gears: g, stages: [], shafts: [], links: [], ratio: design.drive.gearRatio, ratioDesign: design.drive.gearRatio };
  const stages = [];
  for (let k = 0; k < n; k++) stages.push({ pinion: g[2 * k], wheel: g[2 * k + 1], centreDistance: g[2 * k].rp + g[2 * k + 1].rp });
  const ratio = stages.reduce((r, st) => r * (st.wheel.z / st.pinion.z), 1);
  const train = { gears: g, stages, ratio, ratioDesign: design.drive.gearRatio, n };
  train.planes = stagePlanes(design, train, parts);
  const { md, lumpM } = motorOf(design, parts);
  const fold = { stages, n, planes: train.planes, loft: design.body.loft, md, lumpM, O2: fourBar.O2, motorX: train.planes.motorX ?? [train.planes.motorAft, train.planes.motorAft + md[1]] };
  // Round 14: a design may state its fold (params.layout.links_deg, one angle per link from the crank, counter-clockwise
  // from +z), chosen against the parts this search does not see (rails, panels, the motor inside the box).
  const stated = design.params?.layout?.links_deg;
  const best = Array.isArray(stated) && stated.length === n ? { ang: stated.map((a) => (a * Math.PI) / 180) } : foldSearch((ang) => trainCost(fold, ang), n);
  train.links = best.ang;
  train.shafts = trainChain(fourBar.O2, stages, n, best.ang);
  train.centres = { crank: train.shafts[0], motor: train.shafts[n] };
  return train;
}

/** Shaft centres of a folded train: from the crank pivot, link j at angle ang[j] for stage n-1-j's centre distance. */
function trainChain(O2, stages, n, ang) {
  const pts = [O2];
  for (let j = 0; j < n; j++) pts.push(add2(pts[j], mul2(dir2(ang[j]), stages[n - 1 - j].centreDistance)));
  return pts;
}

/** How far (m) a circle of radius r at c leaves the loft at x (negative inside); r itself where the loft has no section. */
function outAt(loft, x, c, r) {
  const h = loftHalf(loft, x);
  if (!h) return r;
  let worst = -Infinity;
  for (let k = 0; k < 24; k++) {
    const a = (k / 24) * TWO_PI;
    const rr = Math.hypot((c[1] + r * Math.sin(a) - h.y) / h.hh, (c[0] + r * Math.cos(a)) / h.hw);
    worst = Math.max(worst, (rr - 1) * Math.min(h.hh, h.hw));
  }
  return worst;
}

/** The fold's cost: tip circles and the motor outside the loft, shafts through other stages' gears, the motor off its lump. */
function trainCost({ stages, n, planes, loft, md, lumpM, O2, motorX }, ang) {
  const pts = trainChain(O2, stages, n, ang);
  let c = 0;
  for (let j = 0; j < n; j++) {
    const st = stages[n - 1 - j];
    const x = planes.stage[n - 1 - j];
    // the crank wheel cannot leave the crank (it hangs where the four-bar puts it): count it, but it is fixed
    c += Math.max(0, outAt(loft, x, pts[j], st.wheel.ro)) + Math.max(0, outAt(loft, x, pts[j + 1], st.pinion.ro));
    c += shaftCrossings(stages, n, pts, j);
  }
  const m = pts[n];
  c += 2 * Math.max(0, Math.max(outAt(loft, motorX[0], m, md[0] / 2), outAt(loft, motorX[1], m, md[0] / 2)));
  c += 0.02 * len2(sub2(m, [lumpM[2], lumpM[1]])) + 0.01 * Math.abs(m[0]);
  return c;
}

/** Shafts pass through every plane between the plates: link j's two shafts against the other stages' gears, 3 mm clear. */
function shaftCrossings(stages, n, pts, j) {
  let c = 0;
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
  return c;
}

/** The link angles of least cost: a coordinate search with halving steps from eight zig-zag starts. -> { c, ang } */
function foldSearch(cost, n) {
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
  return best;
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
 * Length (m) a plate journal's body occupies along its seat: a flanged bushing whose flange sits on the plate's face
 * (params.linkage.plates.flange 'face', v22 on) has its body b1 - b2 in the seat; otherwise (v21) the whole part.
 */
export function journalBodyLength(plates, dims) {
  return plates?.flange === 'face' && dims.flangeB > 0 ? dims.B - dims.flangeB : dims.B;
}

/**
 * Drive planes under the assembly clearance rule (params.layout.rule 'clearance', SPEC "Assembly clearance"): every
 * running pair along the crank pin and the plate-adjacent stages keeps params.layout.running_mm (0.5 mm):
 *  - the right coupler eye that far aft of the crank web, the left eye that far aft of the right one;
 *  - the aft plate's crank bushing flange on the plate's aft face, that far forward of the web;
 *  - the crank stage that far forward of the aft-plate journals and bosses on its two shafts (they reach past the
 *    plate by their body length less the plate), and the forward plate that far forward of stage 1 by the same rule
 *    for the forward plate's journals on stage 1's shafts.
 * Everything else is stagePlanes' (the crank web 1.5 mm forward of the hinge, 1.5 mm between stages, the motor).
 *
 * Round 14 layout options (params.layout; each absent keeps the rule above):
 *  - crank_x_mm: the crank web's centre this far forward of the hinge (the shoulder spacing and horn stacking decide it);
 *  - eye_gap_mm: the gap between the two coupler eyes on the crank pin;
 *  - encoder 'front' (with stage1 'overhung'): the crank encoder (readhead, ride height, actuator: their catalogue
 *    dimensions) on a stub of the crank shaft forward of stage 1, each running gap the running clearance;
 *  - stage1 'overhung': stage 1 outside the box, forward of the forward plate (its wheel overhung on its shaft, the
 *    pinion on the motor shaft through the forward plate), and the motor inside the box, screwed to the forward plate's
 *    aft face (its can aft of it). The forward plate then follows stage 2 by the same rule as above.
 * motorX is the motor can's [aft, forward] faces and motorFace the face it is screwed by (its shaft side).
 */
function clearancePlanes(design, train, parts) {
  const k = planeKit(design, train, parts), { Ly, J, run, x0, plateT, tA, tF, dims, beyond } = k;
  const eye = dims(J.couplerEyes).B, tw = crankWebThickness(design);
  const crankArm = x0 + (Ly.crank_x_mm ?? 1.5) / 1000;
  const plateAft = crankArm + tw / 2 + run + dims(J.crankShaft).flangeB + tA / 2;
  const over = Ly.stage1 === 'overhung' && train.n > 1;
  const last = over ? 1 : 0; // the forward-most stage inside the box
  const { stage, x } = boxStages(train, last, plateAft + tA / 2 + Math.max(beyond(0, tA), beyond(1, tA)) + run);
  const kw = train.n - 1 - last; // the last inside stage's wheel shaft (its pinion shaft is kw + 1)
  const plateFwd = x + Math.max(beyond(kw, tF), beyond(kw + 1, tF)) + run + tF / 2;
  const md = motorOf(design, parts ?? {}).md;
  // round 15 (support): params.layout.motor_gap_mm between the motor's mounting face and the plate's own face
  const motorGap = Ly.motor_gap_mm != null ? Ly.motor_gap_mm / 1000 : null;
  const m = over ? overhungStage1(train.stages[0], plateFwd, plateT, tF, run, k.outboardB, md, motorGap) : { motorFace: plateFwd + plateT / 2 + 0.0015, motorX: null };
  if (over) stage[0] = m.stage0;
  const motorX = m.motorX ?? [m.motorFace, m.motorFace + md[1]];
  // encoder 'front': on a stub of the crank shaft forward of stage 1's teeth (its wheel's hub clears the crank in the
  // plane), so the box stays as long as its three stages; round 15: params.layout.encoder_gap_mm from stage 1's teeth
  // (the readhead's mounting screw heads on its back face, the maker's 3 x M2, clear of the wheel by the running gap)
  const encGap = Ly.encoder_gap_mm != null ? Ly.encoder_gap_mm / 1000 : run;
  const encoder = over && Ly.encoder === 'front' ? encoderPlanes(encoderDims(design, parts), m.stage0 + m.w0 / 2 + encGap) : null;
  const couplerR = crankArm - tw / 2 - run - eye / 2;
  return { crankArm, plateAft, stage, plateFwd, plateT, motorAft: motorX[0], motorX, motorFace: m.motorFace, stage1: over ? 'overhung' : 'inside', ...(encoder ? { encoder } : {}), shoulderAft: k.shoulderAft, crankEnd: Math.max(plateFwd + plateT / 2 + 0.001, encoder ? encoder.actuator[1] : -Infinity), couplerR, couplerL: couplerR - eye - k.eyeGap, rule: 'clearance' };
}

/**
 * What the clearance planes read of the design: the running clearance, the plates, the journals' catalogue sizes and
 * how far a journal reaches past its plate (beyond), the eye gap, the motor's outboard bearing width, and the aft
 * shoulder housings' aft face (the crank shaft ends 1 mm proud of the forward plate, or at an encoder stub's end).
 */
function planeKit(design, train, parts) {
  const Ly = design.params.layout, J = design.params.joints, L = design.params.linkage;
  const run = (Ly.running_mm ?? 0.5) / 1000, x0 = design.wing.hinge[0];
  const dims = (id) => journalDims(parts?.[id]?.specs);
  const shaftJ = (k) => (k === 0 ? J.crankShaft : k === train.n ? J.motorOutboard : J.shafts?.[k - 1] ?? J.shafts?.at(-1));
  return {
    Ly, J, run, x0, plateT: 0.002, dims,
    tA: L.plates.t_mm / 1000, tF: (L.plates.t_fwd_mm ?? L.plates.t_mm) / 1000,
    beyond: (k, tp) => (shaftJ(k) ? Math.max(0, journalBodyLength(L.plates, dims(shaftJ(k))) - tp) : 0),
    eyeGap: Ly.eye_gap_mm != null ? Ly.eye_gap_mm / 1000 : run,
    outboardB: J.motorOutboard ? dims(J.motorOutboard).B : null,
    shoulderAft: x0 - (J.shoulderSpan_mm ?? 24) / 2000 - dims(J.shoulder).B / 2,
  };
}

/** Stage planes inside the box, crank stage first, from x: each stage its widest face and hub, 1.5 mm between. */
function boxStages(train, last, x) {
  const stage = new Array(train.n);
  for (let s = train.n - 1; s >= last; s--) {
    const st = train.stages[s];
    stage[s] = x + Math.max(st.wheel.face, st.pinion.face) / 2;
    x += Math.max(st.wheel.face, st.pinion.face) + Math.max(st.wheel.hubW, st.pinion.hubW) + (s > last ? 0.0015 : 0);
  }
  return { stage, x };
}

/**
 * Stage 1 overhung forward of the forward plate, the motor inside the box on the plate's aft face: the gears clear the
 * shaft ends (1 mm proud of the plate, as every shaft is drawn and checked) and the motor's outboard bearing (seated
 * from the plate's inner face outward) by the running gap. -> { stage0, w0, motorFace, motorX }
 */
function overhungStage1(st, plateFwd, plateT, tF, run, outboardB, md, motorGap = null) {
  const ob = outboardB != null ? plateFwd - tF / 2 + outboardB : -Infinity;
  const w0 = Math.max(st.wheel.face, st.pinion.face);
  // 1.5 mm from the nominal 2 mm plate's face (v22); round 15: motorGap from the plate's own aft face (its mount spacer)
  const motorFace = motorGap != null ? plateFwd - tF / 2 - motorGap : plateFwd - plateT / 2 - 0.0015;
  return { stage0: Math.max(plateFwd + plateT / 2 + 0.001, ob) + run + w0 / 2, w0, motorFace, motorX: [motorFace - md[1], motorFace] };
}

/** The encoder from x along the crank: readhead, its ride height, the actuator. -> planes.encoder (m) */
function encoderPlanes(e, x) {
  const a = x + e.readhead_t + e.ride;
  return { readhead: [x, x + e.readhead_t], actuator: [a, a + e.actuator_h], od: { readhead: e.readhead_od, actuator: e.actuator_od }, id: { readhead: e.readhead_id, actuator: e.actuator_id } };
}

/** The crank encoder's catalogue dimensions (m): readhead ring and thickness, ride height, actuator ring and height (0 if absent). */
export function encoderDims(design, parts) {
  const P = design.params?.parts ?? {};
  const rh = parts?.[P.crankEncoder]?.specs?.dimensions ?? {}, ac = parts?.[P.encoderMagnet]?.specs ?? {};
  const v = (o, k) => (o[k] ?? 0) / 1000;
  return { readhead_od: v(rh, 'od_mm'), readhead_id: v(rh, 'id_mm'), readhead_t: v(rh, 'thickness_mm'), ride: v(ac, 'ride_height_mm'), actuator_od: v(ac, 'od_mm'), actuator_id: v(ac, 'inner_diameter_mm'), actuator_h: v(ac, 'h_mm') };
}

/** x planes of the drive: crank disc at the hinge, aft plate, one plane per stage (crank wheel first), front plate. */
function stagePlanes(design, train, parts) {
  if (design.params?.layout?.rule === 'clearance' && train.n > 0) return clearancePlanes(design, train, parts);
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

/** The motor can's signed gap (m) to the loft: its round end faces at both x ends (positive inside). */
function motorShellGap(loft, box, md, c) {
  let worst = -Infinity;
  for (const x of [box.x0, box.x1]) {
    const h = loftHalf(loft, x);
    if (!h) return -Infinity;
    for (let k = 0; k < 72; k++) {
      const a = (k / 72) * TWO_PI;
      const r = Math.hypot((c[1] + (md[0] / 2) * Math.sin(a) - h.y) / h.hh, (c[0] + (md[0] / 2) * Math.cos(a)) / h.hw);
      worst = Math.max(worst, (r - 1) * Math.min(h.hh, h.hw));
    }
  }
  return -worst;
}

/** Half extents of the loft at x, or null outside it (src/design/clearance.js reads the loft through it too). */
export function loftHalf(loft, x) {
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
function placeBox(loft, name, at, size, others, { rotate = true, gap = -1e-4, shell = null } = {}) {
  // two boxes clash when they come closer than `gap` on every axis (by default, overlap by more than 0.1 mm)
  const g = gap;
  const clash = (b) => others.some((o) => b.x0 < o.x1 + g && b.x1 > o.x0 - g && b.y0 < o.y1 + g && b.y1 > o.y0 - g && b.z0 < o.z1 + g && b.z1 > o.z0 - g);
  const [a0, a1, a2] = size;
  const orients = rotate ? [[a0, a1, a2], [a0, a2, a1], [a1, a0, a2], [a1, a2, a0], [a2, a0, a1], [a2, a1, a0]] : [size];
  // Round 14 (the clearance rule): `shell` is the clearance a part keeps inside the loft, and a place fits only with it.
  const tol = shell != null ? 0 : 0.0005;
  const score = (b) => boxProtrusion(loft, b).out + (shell ?? 0) + (clash(b) ? 1 : 0);
  let best = null;
  for (let k = 0; k <= 60; k++) {
    for (const dy of [0, -0.003, 0.003, -0.006, 0.006, -0.01, 0.01, -0.015, 0.015]) {
      for (const sgn of k ? [-1, 1] : [1]) {
        for (let o = 0; o < orients.length; o++) {
          const sz = orients[o];
          const b = boxAt(at[0] + sgn * k * 0.001, at[1] + dy, at[2], sz[0], sz[1], sz[2]);
          const sc = score(b);
          const cost = k + Math.abs(dy) * 1000 + o * 0.5;
          if (sc <= tol && (!best || best.sc > tol || cost < best.cost)) best = { b, sc, cost, o };
          else if (!best || (best.sc > tol && sc < best.sc)) best = { b, sc, cost, o };
        }
      }
    }
    if (best && best.sc <= tol) break;
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

function servoSpecs(design, add, spec) {
  const dims = (role) => parseDims(spec(role)?.dimensions_text_mm)?.values;
  const sv = dims('stroke amplitude servo') ?? SERVO_DEFAULT_DIMS;
  const st = lump(design, 'stroke servo');
  if (st) add('servo_stroke', 'stroke amplitude servo', [sv[0], sv[2], sv[1]], st.pos, 'drive');
  // Tail servos are sized from their own catalogue entry; the stroke servo's is only a fallback.
  const tv = dims('tail servo') ?? sv;
  const ts = lump(design, 'tail servos');
  add('servo_tail_L', 'tail servo', [tv[0], tv[2], tv[1]], [ts.pos[0], ts.pos[1], ts.pos[2] - tv[1] * 0.55], 'avionics');
  add('servo_tail_R', 'tail servo', [tv[0], tv[2], tv[1]], [ts.pos[0], ts.pos[1], ts.pos[2] + tv[1] * 0.55], 'avionics');
}

/**
 * The packed parts as the design names them, before placement, in packing order: { list: [{ name, role, size [x, y,
 * z] (m, the base orientation), at (where its lump puts it), subsystem }], assumptions }. The servo regulator is listed
 * only when the design mounts it (round 15, params.layout.mounts.regulator); otherwise the film packs it last
 * (packRegulator), as before.
 */
export function packedSpecs(design, parts) {
  const P = (role) => design.bom.parts.find((p) => p.role === role);
  const spec = (role) => parts[P(role)?.id]?.specs;
  const assumptions = [];
  const list = [];
  const add = (name, role, size, at, subsystem) => list.push({ name, role, size, at, subsystem });
  const FR4 = 1850; // kg/m3, glass-epoxy board, for boards whose catalogue entry lists fewer than three sizes
  const flat = (v) => [v[0], v[2], v[1]];
  // Battery: long axis along x, at the design's battery station.
  add('battery', 'battery', sizeOf(spec('battery'), P('battery').unit_mass_g, flat, 2000, assumptions, 'battery'), lump(design, 'battery').pos, 'power');
  // ESC, long axis along x.
  const escRole = design.bom.parts.find((p) => /ESC/.test(p.role)).role;
  add('esc', escRole, sizeOf(spec(escRole), P(escRole).unit_mass_g, flat, FR4, assumptions, 'ESC'), lump(design, 'ESC').pos, 'power');
  // Flight controller flat.
  const fcRole = flightControllerRole(design.bom);
  add('fc', fcRole, sizeOf(spec(fcRole), P(fcRole).unit_mass_g, flat, FR4, assumptions, 'flight controller'), lump(design, 'flight controller').pos, 'avionics');
  add('gps', 'GPS', sizeOf(spec('GPS'), P('GPS').unit_mass_g, flat, FR4, assumptions, 'GPS'), lump(design, 'GPS').pos, 'avionics');
  // Receiver and airspeed board share a lump.
  const rx = lump(design, 'receiver and airspeed');
  add('receiver', 'receiver', sizeOf(spec('receiver'), P('receiver').unit_mass_g, flat, FR4, assumptions, 'receiver'), [rx.pos[0], rx.pos[1] + 0.008, rx.pos[2]], 'avionics');
  add('airspeed', 'airspeed sensor', sizeOf(spec('airspeed sensor'), P('airspeed sensor').unit_mass_g, flat, FR4, assumptions, 'airspeed sensor'), [rx.pos[0], rx.pos[1] - 0.008, rx.pos[2]], 'avionics');
  servoSpecs(design, add, spec);
  const reg = P('servo regulator'), at = lump(design, 'power module and regulator');
  if (design.params?.layout?.mounts?.regulator && reg && at) add('regulator', reg.role, regulatorSize(spec('servo regulator'), reg, assumptions), at.pos, 'power');
  return { list, assumptions };
}

/** The servo regulator's box: its catalogue size (printed in inches), flat, else from its mass as a board. */
function regulatorSize(specs, reg, assumptions) {
  const d = catalogueDims(specs);
  return d?.values.length >= 3 ? [d.values[0], d.values[2], d.values[1]] : sizeOf(specs, reg.unit_mass_g, (v) => [v[0], v[2], v[1]], 1850, assumptions, 'servo regulator');
}

/** The packing order with every mounted part after the part its mount follows (`after`), otherwise unchanged. */
export function mountOrder(list, mounts) {
  const out = [], done = new Set(), byName = new Map(list.map((s) => [s.name, s]));
  const visit = (s, seen = new Set()) => {
    if (done.has(s.name)) return;
    if (seen.has(s.name)) throw new Error(`mounts: a cycle through ${s.name}`);
    seen.add(s.name);
    const a = mounts[s.name]?.after;
    if (a && byName.has(a)) visit(byName.get(a), seen);
    done.add(s.name);
    out.push(s);
  };
  for (const s of list) visit(s);
  return out;
}

/** Two boxes closer than g on every axis (the packing's clash test). */
const boxesClash = (b, o, g) => b.x0 < o.x1 + g && b.x1 > o.x0 - g && b.y0 < o.y1 + g && b.y1 > o.y0 - g && b.z0 < o.z1 + g && b.z1 > o.z0 - g;

/**
 * Round 15 (SPEC "Support and containment"): a part with a stated mount (params.layout.mounts[name]) rests on its
 * carrier. Its box keeps the base orientation, its bottom on the carrier's face (face_y_m), at the stated z (z_m, its
 * centre, or z0_m, its -z side) and x (x0_m, its aft end, or gap_m after the part named `after`). A part at x 'lump'
 * (the battery: the CG trim) starts at its lump's x and slides along x only, never aft of `after`, to the nearest
 * place where the loft holds it with the shell clearance and it clears every part placed. Nothing else moves a mounted
 * part: one that does not fit is reported (protrusion, clash), never moved off its carrier.
 */
function placeMounted(loft, name, mt, size, at, others, byName, shell) {
  const [sx, sy, sz] = size;
  const y = mt.face_y_m + sy / 2, z = mt.z_m ?? mt.z0_m + sz / 2;
  const after = mt.after ? byName.get(mt.after)?.box : null;
  if (mt.after && !after) throw new Error(`mount ${name}: ${mt.after} is not placed before it`);
  const xMin = after ? after.x1 + (mt.gap_m ?? 0) + sx / 2 : -Infinity;
  const boxX = (x) => boxAt(x, y, z, sx, sy, sz);
  const out = (b) => boxProtrusion(loft, b).out + shell;
  const clash = (b) => others.some((o) => boxesClash(b, o, shell));
  let x = mt.x0_m != null ? mt.x0_m + sx / 2 : mt.x === 'lump' ? Math.max(at[0], xMin) : xMin;
  if (!Number.isFinite(x)) throw new Error(`mount ${name}: no x (x0_m, after or 'lump')`);
  if (mt.x === 'lump') {
    const x0 = x;
    let found = false;
    for (let k = 0; k <= 80; k++) {
      const hit = [x0 - k * 0.001, x0 + k * 0.001].find((xx) => xx >= xMin - 1e-12 && out(boxX(xx)) <= 0 && !clash(boxX(xx)));
      if (hit != null) { x = hit; found = true; break; }
    }
    // round 16: the window between the part it follows and the loft can be narrower than the 1 mm search step; its
    // aft end (right after that part) is then the place, if it fits
    if (!found && Number.isFinite(xMin) && xMin >= x0 - 0.08 && out(boxX(xMin)) <= 0 && !clash(boxX(xMin))) x = xMin;
  }
  const b = boxX(x);
  return { name, box: b, orient: size, shift: [x - at[0], y - at[1], z - at[2]], protrusion: Math.max(0, out(b)), clash: clash(b), mount: { ...mt } };
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
  // Round 14: an overhung stage 1 sits forward of the forward plate, so the box reaches its far face and hubs.
  const s0 = train.stages[0];
  const x1 = planes.stage1 === 'overhung' ? Math.max(planes.plateFwd + planes.plateT / 2, planes.stage[0] + Math.max(s0.wheel.face, s0.pinion.face) / 2 + Math.max(s0.wheel.hubW, s0.pinion.hubW), planes.crankEnd ?? -Infinity) : planes.plateFwd + planes.plateT / 2;
  // Round 14 (the clearance rule): the box starts at the aft shoulder housings, so it holds the whole shoulder assembly
  // (both horn stacks sit between the housings).
  const x0 = planes.shoulderAft != null ? Math.min(planes.couplerL - 0.004, planes.shoulderAft) : planes.couplerL - 0.004;
  return { x0, x1, y0, y1, z0, z1 };
}

/**
 * The motor: a cylinder along x on its shaft's centre, at its lump (forward of the box it may slide forward to it).
 * Round 14: a motor inside the box (stage 1 overhung) is screwed to the forward plate's aft face, so it cannot slide;
 * under the clearance rule its can's end circles keep packGap inside the loft (shell_gap_m is their signed gap).
 */
function placeMotor(design, loft, spec, train, planes, packGap) {
  const md = parseDims(spec?.dimensions_text_mm)?.values ?? [0.035, 0.04];
  const ml = lump(design, 'flap motor');
  const c = train.centres?.motor ?? [0, 0];
  const x = planes.stage1 === 'overhung' ? (planes.motorX[0] + planes.motorX[1]) / 2 : Math.max(ml.pos[0], planes.motorAft + md[1] / 2);
  const box = boxAt(x, c[1], c[0], md[1], md[0], md[0]);
  const shell = packGap != null ? { shell_gap_m: motorShellGap(loft, box, md, c) } : {};
  const protrusion = packGap != null ? Math.max(0, packGap - shell.shell_gap_m) : Math.max(0, boxProtrusion(loft, box).out);
  return { name: 'motor', box, shift: [x - ml.pos[0], c[1] - ml.pos[1], c[0] - ml.pos[2]], ...shell, protrusion, cylinder: { r: md[0] / 2, len: md[1] } };
}

export function layoutParts(design, parts, train, fourBar) {
  const loft = design.body.loft;
  const P = (role) => design.bom.parts.find((p) => p.role === role);
  const spec = (role) => parts[P(role)?.id]?.specs;
  const out = [];
  const placed = [];
  const byName = new Map();
  // Round 14 (params.layout.rule 'clearance'): packed parts keep the running clearance to each other and the drive box.
  const packGap = design.params?.layout?.rule === 'clearance' ? (design.params.layout.running_mm ?? 0.5) / 1000 : undefined;
  const mounts = design.params?.layout?.mounts ?? {};
  const put = (name, role, size, at, subsystem) => {
    const p = mounts[name] ? placeMounted(loft, name, mounts[name], size, at, placed, byName, packGap ?? 0) : placeBox(loft, name, at, size, placed, { gap: packGap, shell: packGap ?? null });
    placed.push(p.box);
    const row = { ...p, role, id: P(role)?.id, subsystem };
    out.push(row);
    byName.set(name, row);
    return p;
  };
  const planes = gearPlanes(design, train);
  const motor = placeMotor(design, loft, spec('flap motor'), train, planes, packGap);
  out.push({ ...motor, role: 'flap motor', id: P('flap motor').id, subsystem: 'drive' });
  placed.push(motor.box);
  // The drive's swept volume is kept clear: crank, couplers and both gear stages, plate to plate.
  if (train.links?.length) placed.push(driveReserve(train, fourBar, planes));
  const { list, assumptions } = packedSpecs(design, parts);
  for (const s of mountOrder(list, mounts)) put(s.name, s.role, s.size, s.at, s.subsystem);
  // Gears: report how far each tip circle leaves the loft at its plane.
  const gearFit = gearTipFit(loft, train, planes, packGap);
  // Crank axis against the loft bottom.
  const hC = loftHalf(loft, fourBar.x);
  const crankBelow = hC ? (hC.y - hC.hh) - fourBar.O2[1] : null;
  return { parts: out, gearFit, crankBelowLoft_m: crankBelow, assumptions };
}

/**
 * How far each gear's tip circle leaves the loft at its plane (72 points round it): protrusion beyond the loft less
 * packGap (the running clearance a tip circle keeps inside it under the clearance rule), and its signed gap.
 */
function gearTipFit(loft, train, planes, packGap) {
  const out = [];
  for (let j = 0; j < (train.links?.length ?? 0); j++) {
    const si = train.n - 1 - j, st = train.stages[si], x = planes.stage[si], h = loftHalf(loft, x);
    for (const [name, gear, c] of [[`stage ${si + 1} wheel ${st.wheel.z}T`, st.wheel, train.shafts[j]], [`stage ${si + 1} pinion ${st.pinion.z}T`, st.pinion, train.shafts[j + 1]]]) {
      let worst = -Infinity;
      for (let k = 0; k < 72; k++) {
        const a = (k / 72) * TWO_PI;
        const z = c[0] + gear.ro * Math.cos(a), y = c[1] + gear.ro * Math.sin(a);
        const r = h ? Math.hypot((y - h.y) / h.hh, z / h.hw) : Infinity;
        worst = Math.max(worst, (r - 1) * (h ? Math.min(h.hh, h.hw) : 1));
      }
      out.push({ name, id: gear.id, x, protrusion: Math.max(0, worst + (packGap ?? 0)), gap_m: -worst });
    }
  }
  return out;
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
  // the physics' own coupler planes (v22: clear of each other and the web); the crank pin as src/design sizes it,
  // to 1 mm past the left coupler's eye (linkage.js crankMembers)
  const eye = couplerEyeLength(design, parts);
  model.planes = { ...model.planes, couplerEye: eye, crankPinEnd: model.planes.couplerL - eye / 2 - 0.001 };
  return model;
}

/**
 * The servo regulator, packed last against the physics' layout so no other part moves. The film packs it in
 * prepareFilm; src/design/clearance.js packs it the same way for the assembly clearance check (round 14), so the
 * physics checks the part where the film draws it; neither changes any other part's place or the body fit.
 */
export function packRegulator(model, design, parts) {
  const reg = design.bom.parts.find((p) => p.role === 'servo regulator');
  const at = lump(design, 'power module and regulator');
  // round 15: a design that mounts the regulator packs it in layoutParts, on its carrier
  if (!reg || !at || model.layout.parts.some((p) => p.name === 'regulator')) return;
  const size = regulatorSize(parts[reg.id]?.specs, reg, model.layout.assumptions);
  const boxes = model.layout.parts.map((p) => p.box);
  if (model.train.links?.length) boxes.push(driveReserve(model.train, model.fourBar, model.planes));
  const gap = design.params?.layout?.rule === 'clearance' ? (design.params.layout.running_mm ?? 0.5) / 1000 : undefined;
  model.layout.parts.push({ ...placeBox(design.body.loft, 'regulator', at.pos, size, boxes, { gap }), role: reg.role, id: reg.id, subsystem: 'power' });
  model.fit = fitReport(model);
}

/** Plain-language list of what does not fit, with numbers, for the page and the report. */
export function fitReport(model) {
  const issues = [];
  const L = model.layout;
  if (L.crankBelowLoft_m != null && L.crankBelowLoft_m > 0) issues.push({ part: 'crank axis', text: `crank axis ${(L.crankBelowLoft_m * 1000).toFixed(0)} mm below the body loft (ground link ${(model.law.d * 1000).toFixed(0)} mm from a hinge ${(model.design.wing.hinge[2] * 1000).toFixed(0)} mm off centre)`, mm: L.crankBelowLoft_m * 1000 });
  const tol = model.design.params?.layout?.rule === 'clearance' ? 1e-7 : 0.0005; // round 14: no shortfall at all
  for (const g of L.gearFit) if (g.protrusion > tol) issues.push({ part: g.name, text: `${g.name} (${g.id}) tip circle ${(g.protrusion * 1000).toFixed(0)} mm outside the loft`, mm: g.protrusion * 1000 });
  for (const p of L.parts) {
    if (p.protrusion > tol) issues.push({ part: p.name, text: `${p.name} ${(p.protrusion * 1000).toFixed(0)} mm outside the loft${Math.abs(p.shift[0]) > 1e-4 ? ` after a ${(p.shift[0] * 1000).toFixed(0)} mm slide along x` : ''}`, mm: p.protrusion * 1000 });
    if (p.protrusion <= tol && Math.abs(p.shift[0]) > 1e-4) issues.push({ part: p.name, text: `${p.name} moved ${(p.shift[0] * 1000).toFixed(0)} mm along x from its mass lump to fit`, mm: 0, shift: p.shift[0] });
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
