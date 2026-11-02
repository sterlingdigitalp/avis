#!/usr/bin/env node
// Glide pose frames for a design, from the physics module's own kinematics (SPEC: one source).
//   node cfd/pose.mjs data/designs/<name>.json [V] > frames.json
// Prints, for the right wing, each span station's leading edge, chord vector and section up vector in the body
// frame (x forward, y up, z right), plus the glide state the pose came from. Wings parked at the mean stroke angle
// (zero amplitude), zero commanded twist: the gliding configuration.
import { readFileSync } from 'node:fs';
import { makeWing, makeLatticeSpec, wingGrid, jointState } from '../src/physics/kinematics.js';
import { bodyAndTail } from '../src/physics/geometry.js';

const design = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const V = Number(process.argv[3] ?? design.cruise.V);
const glide = design.control.glide.reduce((a, b) => (Math.abs(b.V - V) < Math.abs(a.V - V) ? b : a));
const wing = makeWing(design.wing);
const nc = 40;
const ns = 60;
const ls = makeLatticeSpec(wing, nc, ns, { cosine: true, chordCosine: true });
const law = { kind: 'sine', phiMean: design.drive.phiMean ?? 0 };
const joint = jointState(0, 0, 0, 0, law, { sym: 0, dyn: 0, lead: 0 }, +1);
const g = wingGrid(wing, ls, joint, +1);
const at = (i, m) => [g[3 * (i * (ns + 1) + m)], g[3 * (i * (ns + 1) + m) + 1], g[3 * (i * (ns + 1) + m) + 2]];
const sub = (a, b) => a.map((v, k) => v - b[k]);
const norm = (a) => Math.hypot(...a);
const iMid = ls.xi.reduce((best, x, i) => (Math.abs(x - 0.4) < Math.abs(ls.xi[best] - 0.4) ? i : best), 0);
const stations = [];
for (let m = 0; m <= ns; m++) {
  const le = at(0, m);
  const te = at(nc, m);
  const c = sub(te, le);
  const chord = norm(c);
  const mid = at(iMid, m);
  const lin = le.map((v, k) => v + ls.xi[iMid] * c[k]);
  let up = sub(mid, lin);
  // Zero camber leaves no offset: then up is the chord rotated by 90 deg about the local span direction.
  if (norm(up) < 1e-9 * chord) {
    const nxt = at(0, Math.min(m + 1, ns));
    const prv = at(0, Math.max(m - 1, 0));
    const s = sub(nxt, prv);
    up = [s[1] * c[2] - s[2] * c[1], s[2] * c[0] - s[0] * c[2], s[0] * c[1] - s[1] * c[0]];
  }
  const u = norm(up);
  // Remove any chordwise part so the section frame is orthogonal.
  const cu = c.map((v) => v / chord);
  const dot = up.reduce((s, v, k) => s + v * cu[k], 0);
  up = up.map((v, k) => (v - dot * cu[k]) / u);
  const un = norm(up);
  stations.push({ s: ls.span[m], le, chord_vec: cu, chord, up: up.map((v) => v / un) });
}
// Body loft and tail panels from the physics geometry module (the one description for tunnels and OpenFOAM).
const bt = design.body.loft ? bodyAndTail(design) : null;
if (bt) bt.mesh = (await import('../src/physics/geometry.js')).bodyMesh(design.body.loft, 48);
console.log(JSON.stringify({ design: design.name, version: design.version ?? null, V: glide.V, glide,
  aoa_rad: glide.theta - glide.gamma, phi: joint.phi, airfoil: design.wing.airfoil, tail: design.tail, body: design.body,
  body_mesh: bt ? bt.mesh : null, tail_panel: bt ? bt.tail.find((t) => t.side === 'right') : null, stations }));
