// Pure terrain height query for the flight model and the live page. No dependencies; runs in node and the browser.
//
// data/env/terrain_height.bin, little-endian:
//   header 56 bytes: magic "AVTH", version u32, heightOffset f32, heightScale f32,
//     nearNx u32, nearNz u32, nearX0 f32, nearZ0 f32, nearDx f32,
//     farNx u32, farNz u32, farX0 f32, farZ0 f32, farDx f32
//   near heights  u16[nearNz * nearNx]   ground height, row 0 at nearZ0, z increasing (to the south)
//   near canopy   u8 [nearNz * nearNx]   canopy height above ground in 0.25 m units, from lidar DSM minus DTM
//   far heights   u16[farNz * farNx]
//
// Coordinates are the SPEC world frame: x east, y up, z south, metres, relative to the corridor origin.
// Heights returned are world y, so add corridor.elevation_m_at_origin for metres above sea level.

const HEADER_BYTES = 56;

export function parseTerrain(arrayBuffer) {
  const v = new DataView(arrayBuffer);
  const magic = String.fromCharCode(v.getUint8(0), v.getUint8(1), v.getUint8(2), v.getUint8(3));
  if (magic !== "AVTH") throw new Error(`terrain_height.bin: bad magic ${magic}`);
  const version = v.getUint32(4, true);
  if (version !== 1) throw new Error(`terrain_height.bin: unsupported version ${version}`);
  const heightOffset = v.getFloat32(8, true);
  const heightScale = v.getFloat32(12, true);
  const near = readGridHeader(v, 16);
  const far = readGridHeader(v, 36);
  const nearCount = near.nx * near.nz;
  const farCount = far.nx * far.nz;
  const expected = HEADER_BYTES + nearCount * 3 + farCount * 2;
  if (arrayBuffer.byteLength !== expected) {
    throw new Error(`terrain_height.bin: expected ${expected} bytes, got ${arrayBuffer.byteLength}`);
  }
  // Copy to aligned arrays: the u8 canopy block may leave the far block on an odd offset.
  near.h = copyU16(arrayBuffer, HEADER_BYTES, nearCount);
  near.canopy = new Uint8Array(arrayBuffer, HEADER_BYTES + nearCount * 2, nearCount);
  far.h = copyU16(arrayBuffer, HEADER_BYTES + nearCount * 3, farCount);

  const decode = (raw) => raw * heightScale + heightOffset;

  function groundHeight(x, z) {
    const g = inside(near, x, z) ? near : far;
    return decode(bilinear(g, g.h, x, z));
  }

  function canopyHeight(x, z) {
    if (!inside(near, x, z)) return 0;
    return bilinear(near, near.canopy, x, z) * 0.25;
  }

  function surfaceHeight(x, z) {
    return groundHeight(x, z) + canopyHeight(x, z);
  }

  return {
    groundHeight,
    canopyHeight,
    surfaceHeight,
    clearance: (x, y, z) => y - surfaceHeight(x, z),
    nearBounds: bounds(near),
    farBounds: bounds(far),
  };
}

function readGridHeader(v, at) {
  return {
    nx: v.getUint32(at, true),
    nz: v.getUint32(at + 4, true),
    x0: v.getFloat32(at + 8, true),
    z0: v.getFloat32(at + 12, true),
    dx: v.getFloat32(at + 16, true),
  };
}

function copyU16(buffer, offset, count) {
  const out = new Uint16Array(count);
  const src = new DataView(buffer, offset, count * 2);
  for (let i = 0; i < count; i++) out[i] = src.getUint16(i * 2, true);
  return out;
}

function inside(g, x, z) {
  const c = (x - g.x0) / g.dx;
  const r = (z - g.z0) / g.dx;
  return c >= 0 && r >= 0 && c <= g.nx - 1 && r <= g.nz - 1;
}

function bilinear(g, data, x, z) {
  const c = clamp((x - g.x0) / g.dx, 0, g.nx - 1);
  const r = clamp((z - g.z0) / g.dx, 0, g.nz - 1);
  const c0 = Math.min(Math.floor(c), g.nx - 2);
  const r0 = Math.min(Math.floor(r), g.nz - 2);
  const fc = c - c0;
  const fr = r - r0;
  const i = r0 * g.nx + c0;
  return (
    data[i] * (1 - fc) * (1 - fr) +
    data[i + 1] * fc * (1 - fr) +
    data[i + g.nx] * (1 - fc) * fr +
    data[i + g.nx + 1] * fc * fr
  );
}

function bounds(g) {
  return { xMin: g.x0, xMax: g.x0 + (g.nx - 1) * g.dx, zMin: g.z0, zMax: g.z0 + (g.nz - 1) * g.dx };
}

function clamp(a, lo, hi) {
  return a < lo ? lo : a > hi ? hi : a;
}
