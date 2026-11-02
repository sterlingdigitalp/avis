// node --test env/terrain.test.js
// Pins the reader against values that do not come from this code: the raw lidar samples in terrain.fixture.json and
// published summit and river elevations.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import { parseTerrain } from "./terrain.js";

const root = new URL("../", import.meta.url);
const bytes = readFileSync(new URL("data/env/terrain_height.bin", root));
const buffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
const terrain = parseTerrain(buffer);
const corridor = JSON.parse(readFileSync(new URL("data/env/corridor.json", root), "utf8"));
const fixture = JSON.parse(readFileSync(new URL("env/terrain.fixture.json", root), "utf8"));
const originZ = corridor.elevation_m_at_origin;

test("file stays small enough for the repo (SPEC: under 5 MB)", () => {
  assert.ok(bytes.byteLength < 5e6, `${bytes.byteLength} bytes`);
});

test("near grid matches the raw lidar DTM within half a metre", () => {
  for (const p of fixture) {
    const [x, z] = p.world_xz;
    const asl = terrain.groundHeight(x, z) + originZ;
    assert.ok(Math.abs(asl - p.raw_lidar_dtm_asl) < 0.5, `${p.utm}: ${asl} vs ${p.raw_lidar_dtm_asl}`);
  }
});

test("Mount Rundle summit ridge in the far grid reaches its published 2948 m within 30 m at 40 m cells", () => {
  // Published summit: 51.1322 N 115.4725 W, about UTM 606900 E 5666100 N. The far grid is coarse, so take the max nearby.
  const b = terrain.farBounds;
  let best = -Infinity;
  for (let x = -6500; x <= -1000; x += 40) {
    for (let z = -3500; z <= 4500; z += 40) {
      if (x < b.xMin || x > b.xMax || z < b.zMin || z > b.zMax) continue;
      best = Math.max(best, terrain.groundHeight(x, z));
    }
  }
  assert.ok(Math.abs(best + originZ - 2948) < 30, `max ${best + originZ} m`);
});

test("Bow River at Canmore sits near the published 1300-1330 m valley floor", () => {
  const river = terrain.groundHeight(1000, -560) + originZ; // fixture region, channel next to the corridor start
  assert.ok(river > 1300 && river < 1345, `${river} m`);
});

test("corridor waypoints stay above ground and canopy by the declared clearance, to within grid error", () => {
  const margin = 2.0; // the height file is 4 m; the corridor solved its clearance on the 2 m lidar
  for (const [x, y, z] of corridor.waypoints) {
    const c = terrain.clearance(x, y, z);
    assert.ok(c >= corridor.min_clearance_m - margin, `clearance ${c} at ${x},${z}`);
  }
});

test("corridor keeps within the climb limit it declares", () => {
  const w = corridor.waypoints;
  for (let i = 1; i < w.length; i++) {
    const ds = Math.hypot(w[i][0] - w[i - 1][0], w[i][2] - w[i - 1][2]);
    const grad = (w[i][1] - w[i - 1][1]) / ds;
    assert.ok(grad <= corridor.max_climb_gradient + 1e-3, `gradient ${grad} at waypoint ${i}`);
    assert.ok(grad >= -corridor.max_sink_gradient - 1e-3, `gradient ${grad} at waypoint ${i}`);
  }
});

test("corridor length and sun direction match the declared frame", () => {
  let len = 0;
  const w = corridor.waypoints;
  for (let i = 1; i < w.length; i++) len += Math.hypot(w[i][0] - w[i - 1][0], w[i][1] - w[i - 1][1], w[i][2] - w[i - 1][2]);
  assert.ok(len > 800 && len < 1600, `length ${len} m`);
  const s = corridor.sun_dir;
  assert.ok(Math.abs(Math.hypot(...s) - 1) < 1e-6);
  // Morning sun in the east and low: x east positive, y up small positive.
  assert.ok(s[0] > 0.8 && s[1] > 0.1 && s[1] < 0.3, `sun ${s}`);
});

test("reader rejects a corrupt file", () => {
  assert.throws(() => parseTerrain(new ArrayBuffer(64)), /bad magic/);
  const truncated = buffer.slice(0, 1000);
  assert.throws(() => parseTerrain(truncated), /expected/);
});

// Synthetic files: a bilinear reader must reproduce a plane exactly, so a plane is an independent reference.
function makeFile({ near, far, canopy = 0 }) {
  const nNear = near.nx * near.nz;
  const nFar = far.nx * far.nz;
  const buf = new ArrayBuffer(56 + nNear * 3 + nFar * 2);
  const v = new DataView(buf);
  "AVTH".split("").forEach((ch, i) => v.setUint8(i, ch.charCodeAt(0)));
  v.setUint32(4, 1, true);
  v.setFloat32(8, -500, true);
  v.setFloat32(12, 0.05, true);
  const hdr = (at, g) => {
    v.setUint32(at, g.nx, true);
    v.setUint32(at + 4, g.nz, true);
    v.setFloat32(at + 8, g.x0, true);
    v.setFloat32(at + 12, g.z0, true);
    v.setFloat32(at + 16, g.dx, true);
  };
  hdr(16, near);
  hdr(36, far);
  const put = (base, g) => {
    for (let r = 0; r < g.nz; r++)
      for (let c = 0; c < g.nx; c++) {
        const h = g.fn(g.x0 + c * g.dx, g.z0 + r * g.dx);
        v.setUint16(base + 2 * (r * g.nx + c), Math.round((h + 500) / 0.05), true);
      }
  };
  put(56, near);
  for (let i = 0; i < nNear; i++) v.setUint8(56 + nNear * 2 + i, Math.round(canopy / 0.25));
  put(56 + nNear * 3, far);
  return buf;
}

const nearPlane = (x, z) => 100 + 0.5 * x - 0.25 * z;
const farPlane = (x, z) => 300 - 0.1 * x + 0.2 * z;
// 7 x 5 near cells: an odd count puts the far block on an odd byte offset.
const synth = parseTerrain(
  makeFile({
    near: { nx: 7, nz: 5, x0: -30, z0: -20, dx: 10, fn: nearPlane },
    far: { nx: 9, nz: 9, x0: -400, z0: -400, dx: 100, fn: farPlane },
    canopy: 6.5,
  }),
);
const close = (a, b, tol = 0.03) => Math.abs(a - b) <= tol;

test("bilinear reproduces a plane exactly at off-grid points inside the near grid", () => {
  for (const [x, z] of [[-30, -20], [-24.3, -11.7], [0.1, 0.1], [17.5, 13.25], [29.99, 19.99], [30, 20]]) {
    assert.ok(close(synth.groundHeight(x, z), nearPlane(x, z)), `${x},${z}: ${synth.groundHeight(x, z)} vs ${nearPlane(x, z)}`);
  }
});

test("points outside the near grid read the far grid, including just past its edge", () => {
  for (const [x, z] of [[30.5, 0], [-30.5, 0], [0, 20.5], [0, -20.5], [250, -333], [-399, 399]]) {
    assert.ok(close(synth.groundHeight(x, z), farPlane(x, z)), `${x},${z}: ${synth.groundHeight(x, z)} vs ${farPlane(x, z)}`);
  }
});

test("outside the far grid the edge value is held, not extrapolated", () => {
  assert.ok(close(synth.groundHeight(5000, 0), farPlane(400, 0)));
  assert.ok(close(synth.groundHeight(-5000, -5000), farPlane(-400, -400)));
  assert.ok(close(synth.groundHeight(0, 9000), farPlane(0, 400)));
});

test("canopy is read only inside the near grid and adds to the surface", () => {
  assert.equal(synth.canopyHeight(0, 0), 6.5);
  assert.equal(synth.canopyHeight(29.9, 19.9), 6.5);
  assert.equal(synth.canopyHeight(31, 0), 0);
  assert.ok(close(synth.surfaceHeight(10, 10), nearPlane(10, 10) + 6.5));
  assert.ok(close(synth.clearance(10, 200, 10), 200 - nearPlane(10, 10) - 6.5));
});

test("bounds report the grid extents", () => {
  assert.deepEqual(synth.nearBounds, { xMin: -30, xMax: 30, zMin: -20, zMax: 20 });
  assert.deepEqual(synth.farBounds, { xMin: -400, xMax: 400, zMin: -400, zMax: 400 });
});

test("reader rejects a wrong version", () => {
  const b = makeFile({ near: { nx: 2, nz: 2, x0: 0, z0: 0, dx: 1, fn: nearPlane }, far: { nx: 2, nz: 2, x0: 0, z0: 0, dx: 1, fn: farPlane } });
  new DataView(b).setUint32(4, 2, true);
  assert.throws(() => parseTerrain(b), /version/);
});
