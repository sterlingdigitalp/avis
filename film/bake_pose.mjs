#!/usr/bin/env node
// Bakes the bird's pose from a flight log, frame by frame, for Blender: every posed node's local matrix and every
// deforming mesh's vertices, computed by the same bird.update() as the page (kinematics.js inside). Frames are the
// ones env/import_flight.py load_flight() makes: t = t0 + i / fps, phase unwrapped and interpolated linearly, the
// other columns interpolated linearly. Values stay in the GLB's Y-up axes; Blender converts them on import.
//
//   node --import ./film/lib/three-hooks.mjs film/bake_pose.mjs <log.json> --fps 30 --from i0 --to i1 --out <file>
//   writes <file>.json (header) and <file>.bin (float32: per frame, per node 16 matrix values, then each deform
//   mesh's xyz in node order)
import { readFileSync, writeFileSync } from 'node:fs';
import { nodeBird, posedNodes } from './lib/bird-node.mjs';

const TWO_PI = 2 * Math.PI;

/** Columns of a SPEC flight log resampled at t0 + i / fps, the way import_flight.load_flight does. */
export function sampleLog(log, fps, i0, i1) {
  const c = log.columns;
  const t = c.t;
  const n = t.length;
  // unwrap the phase
  const ph = new Float64Array(n);
  ph[0] = c.phase[0];
  for (let k = 1; k < n; k++) {
    let d = c.phase[k] - c.phase[k - 1];
    d -= TWO_PI * Math.round(d / TWO_PI);
    ph[k] = ph[k - 1] + d;
  }
  const out = [];
  let k = 0;
  for (let i = i0; i < i1; i++) {
    const tq = t[0] + i / fps;
    if (tq > t[n - 1]) break;
    while (k < n - 2 && t[k + 1] < tq) k++;
    const f = Math.min(1, Math.max(0, (tq - t[k]) / (t[k + 1] - t[k])));
    const lerp = (a, b) => a + f * (b - a);
    out.push({
      i, t: tq,
      phase: lerp(ph[k], ph[k + 1]),
      amp: lerp(c.amplitude[k], c.amplitude[k + 1]),
      twistL: lerp(c.twist_l[k], c.twist_l[k + 1]),
      twistR: lerp(c.twist_r[k], c.twist_r[k + 1]),
      tailL: lerp(c.tail[k][0], c.tail[k + 1][0]),
      tailR: lerp(c.tail[k][1], c.tail[k + 1][1]),
    });
  }
  return out;
}

export function bake(logPath, { fps = 30, from = 0, to = Infinity, out, design } = {}) {
  const log = JSON.parse(readFileSync(logPath, 'utf8'));
  const { bird, sha256, design: d } = nodeBird(design ?? log.design);
  const nodes = posedNodes(bird);
  const deform = bird.deform;
  const frames = sampleLog(log, fps, from, Math.min(to, Math.ceil((log.columns.t.at(-1) - log.columns.t[0]) * fps)));
  const per = nodes.length * 16 + deform.reduce((p, m) => p + m.geometry.attributes.position.count * 3, 0);
  const buf = new Float32Array(frames.length * per);
  let o = 0;
  let worstDelta = 0;
  for (const fr of frames) {
    // Flight shots show the wings as the flight model flew them: both at its stroke angle. The mechanical difference a
    // single crank pin makes on the left wing is measured and recorded, not drawn here.
    const dp = bird.update({ phase: fr.phase, amp: fr.amp, twistL: fr.twistL, twistR: fr.twistR, tailL: fr.tailL, tailR: fr.tailR, symmetric: true });
    worstDelta = Math.max(worstDelta, Math.abs(dp.leftDelta));
    bird.root.updateMatrixWorld(true);
    for (const n of nodes) {
      buf.set(n.matrix.elements, o);
      o += 16;
    }
    for (const m of deform) {
      const a = m.geometry.attributes.position.array;
      buf.set(a, o);
      o += a.length;
    }
  }
  const header = {
    log: logPath, design: d.name, design_version: d.version, design_sha256: sha256, fps, from, to: from + frames.length,
    frames: frames.map((f) => f.i), t: frames.map((f) => f.t),
    nodes: nodes.map((n) => n.name), deform: deform.map((m) => ({ name: m.name, count: m.geometry.attributes.position.count })),
    floats_per_frame: per, axes: 'glTF Y-up, column-major local matrices; vertices in the mesh node frame',
    left_minus_right_stroke_max_rad: worstDelta,
  };
  if (out) {
    writeFileSync(`${out}.json`, JSON.stringify(header));
    writeFileSync(`${out}.bin`, Buffer.from(buf.buffer));
  }
  return { header, buf };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const a = process.argv.slice(2);
  const opt = (k, d) => (a.includes(`--${k}`) ? a[a.indexOf(`--${k}`) + 1] : d);
  const r = bake(a[0], { fps: +opt('fps', 30), from: +opt('from', 0), to: +opt('to', Infinity), out: opt('out') });
  console.log(`baked ${r.header.frames.length} frames x ${r.header.floats_per_frame} floats; max left-right stroke difference ${((r.header.left_minus_right_stroke_max_rad * 180) / Math.PI).toFixed(2)} deg`);
}
