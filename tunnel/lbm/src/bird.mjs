// Surfaces for a whole bird from the one description of it: a design file plus the shared pure
// kinematics in src/physics/kinematics.js. Nothing here keyframes or re-derives motion.
//
// Tunnel axes: wind toward -x, y up, z right, so a body pitched nose-up by bodyAlpha is rotated
// about +z. The body frame is the SPEC's x forward, y up, z right.

/** Trim state at speed V from the design's trim table (control.trimFlight or control.trim), linear in V. */
export function trimAt(d, V) {
  const table = (d.control?.trimFlight ?? d.control?.trim ?? []).filter((r) => Number.isFinite(r.V)).sort((p, q) => p.V - q.V);
  if (!table.length) return null;
  if (V <= table[0].V) return { ...table[0], source: `control.trim at V=${table[0].V}` };
  if (V >= table[table.length - 1].V) return { ...table[table.length - 1], source: `control.trim at V=${table[table.length - 1].V}` };
  const j = table.findIndex((r) => r.V >= V);
  const a = table[j - 1], b = table[j], f = (V - a.V) / (b.V - a.V);
  const out = { V, source: `control.trim interpolated between V=${a.V} and V=${b.V}` };
  for (const k of ['theta', 'freq', 'tail', 'amp', 'sym', 'dyn', 'lead']) if (Number.isFinite(a[k]) && Number.isFinite(b[k])) out[k] = a[k] + f * (b[k] - a[k]);
  return out;
}

/** Read a data/designs file (src/design/schema.js). A missing field gives a clear error, not bad physics.
 *  Cruise speed from cruise.V; the rest of the trim (frequency, body pitch, tail, twist) from the
 *  design's own trim table at that speed, unless cruise states it. */
export function readDesign(d, kin, speedOverride) {
  if (!d.wing?.segments) throw new Error(`design has no wing.segments (top-level keys: ${Object.keys(d).join(', ')})`);
  const c = d.cruise ?? {};
  const drive = d.drive ?? null;
  const law = drive?.kind === 'crank-rocker' ? kin.makeFlapDrive(drive) : { kind: 'sine', phiMean: drive?.phiMean ?? 0 };
  const V = speedOverride ?? c.V ?? d.control?.Vcruise;
  const tr = trimAt(d, V) ?? {};
  const val = (k, dflt) => c[k] ?? tr[k] ?? dflt;
  return {
    name: d.name ?? 'unnamed',
    wingSpec: d.wing,
    speed: V,
    freq: val('freq'),
    amp: val('amp', drive?.ampNominal),
    law,
    sched: { sym: val('sym', 0), dyn: val('dyn', 0), lead: val('lead', 0) },
    bodyAlpha: val('theta', 0),
    tailDeflection: val('tail', 0),
    trimSource: c.freq !== undefined ? 'cruise' : tr.source ?? 'none',
    body: d.body ?? null,
    tail: d.tail ?? null,
  };
}

/** Closed ellipsoid about body x. The design gives a body only by length, width and height. */
function ellipsoidMesh(length, width, height, nLat = 24, nLon = 32) {
  const v = [];
  const tris = [];
  v.push([length / 2, 0, 0]);
  for (let i = 1; i < nLat; i++) {
    const th = (Math.PI * i) / nLat;
    for (let j = 0; j < nLon; j++) {
      const ph = (2 * Math.PI * j) / nLon;
      v.push([(length / 2) * Math.cos(th), (height / 2) * Math.sin(th) * Math.cos(ph), (width / 2) * Math.sin(th) * Math.sin(ph)]);
    }
  }
  v.push([-length / 2, 0, 0]);
  const id = (i, j) => 1 + (i - 1) * nLon + (j % nLon);
  for (let j = 0; j < nLon; j++) tris.push(0, id(1, j), id(1, j + 1));
  for (let i = 1; i < nLat - 1; i++) for (let j = 0; j < nLon; j++) {
    tris.push(id(i, j), id(i + 1, j), id(i + 1, j + 1));
    tris.push(id(i, j), id(i + 1, j + 1), id(i, j + 1));
  }
  const last = v.length - 1;
  for (let j = 0; j < nLon; j++) tris.push(last, id(nLat - 1, j + 1), id(nLat - 1, j));
  return { pos: v, tris: Uint32Array.from(tris) };
}

export const PART = { right_wing: 0, left_wing: 1, body: 2, tail: 3 };

/**
 * @param kin the kinematics module
 * @param o {design: readDesign result, dx, dt, origin:[x,y,z] physical position of the low corner of
 *           cell (0,0,0), half:boolean, panel: target triangle edge in cells, freq, amp}
 */
export function birdSurfaces(kin, o) {
  const { design, dx, dt, origin, half, panel = 4, freq, amp } = o;
  const wing = kin.makeWing(design.wingSpec);
  const omega = 2 * Math.PI * freq;
  const alpha = design.bodyAlpha ?? 0;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const chordMax = Math.max(...wing.segments.flatMap((g) => [g.chord_in, g.chord_out]));
  const nc = Math.max(4, Math.ceil(chordMax / (panel * dx)));
  const ns = Math.max(8, Math.ceil(wing.L / (panel * dx)));
  const ls = kin.makeLatticeSpec(wing, nc, ns, { cosine: false });
  const nv = (nc + 1) * (ns + 1);
  const tris = new Uint32Array(nc * ns * 6);
  let t = 0;
  for (let i = 0; i < nc; i++) for (let j = 0; j < ns; j++) {
    const a = i * (ns + 1) + j, b = a + 1, c = a + ns + 1, d = c + 1;
    tris[t++] = a; tris[t++] = b; tris[t++] = d;
    tris[t++] = a; tris[t++] = d; tris[t++] = c;
  }
  const toLat = (x, y, z) => [(ca * x - sa * y - origin[0]) / dx - 0.5, (sa * x + ca * y - origin[1]) / dx - 0.5, (z - origin[2]) / dx - 0.5];
  const toLatV = (vx, vy, vz) => [((ca * vx - sa * vy) * dt) / dx, ((sa * vx + ca * vy) * dt) / dx, (vz * dt) / dx];
  const ref = toLat(0, 0, 0);
  const eps = 1e-4;
  const wingSurface = (side, part, mirrorZ) => {
    const g0 = new Float64Array(nv * 3), gm = new Float64Array(nv * 3), gp = new Float64Array(nv * 3);
    const pos = new Float64Array(nv * 3), vel = new Float64Array(nv * 3);
    return {
      part, kind: 'thin', tris, mirrorZ, moving: true, ref: () => ref,
      pose(tStep) {
        const phase = omega * tStep * dt;
        kin.wingGrid(wing, ls, kin.jointState(phase, omega, 0, amp, design.law, design.sched, side), side, g0);
        kin.wingGrid(wing, ls, kin.jointState(phase - eps, omega, 0, amp, design.law, design.sched, side), side, gm);
        kin.wingGrid(wing, ls, kin.jointState(phase + eps, omega, 0, amp, design.law, design.sched, side), side, gp);
        const k = omega / (2 * eps);
        for (let v = 0; v < nv * 3; v += 3) {
          const L = toLat(g0[v], g0[v + 1], g0[v + 2]);
          pos[v] = L[0]; pos[v + 1] = L[1]; pos[v + 2] = L[2];
          const V = toLatV((gp[v] - gm[v]) * k, (gp[v + 1] - gm[v + 1]) * k, (gp[v + 2] - gm[v + 2]) * k);
          vel[v] = V[0]; vel[v + 1] = V[1]; vel[v + 2] = V[2];
        }
        return { pos, vel };
      },
    };
  };
  const surfaces = [wingSurface(+1, PART.right_wing, half)];
  if (!half) surfaces.push(wingSurface(-1, PART.left_wing, false));
  if (design.body?.length_m) {
    const e = ellipsoidMesh(design.body.length_m, design.body.width_m ?? design.body.height_m, design.body.height_m ?? design.body.width_m);
    const off = design.body.center_m ?? [0, 0, 0];
    const pos = new Float64Array(e.pos.length * 3);
    e.pos.forEach((p, i) => { const L = toLat(p[0] + off[0], p[1] + off[1], p[2] + off[2]); pos[i * 3] = L[0]; pos[i * 3 + 1] = L[1]; pos[i * 3 + 2] = L[2]; });
    const vel = new Float64Array(pos.length);
    surfaces.push({ part: PART.body, kind: 'closed', tris: e.tris, moving: false, ref: () => ref, pose: () => ({ pos, vel }) });
  }
  if (design.tail?.span_each_m) surfaces.push(...tailSurfaces(design, half, toLat, ref, panel * dx));
  return { surfaces, wing, nc, ns, ref };
}

/** Fixed thin tail panels at trim deflection, positioned the way src/physics/flight.js places them. */
function tailSurfaces(design, half, toLat, ref, panelSize) {
  const t = design.tail;
  const inc = (t.incidence_rad ?? 0) + (design.tailDeflection ?? 0);
  const ns = Math.max(2, Math.ceil(t.span_each_m / panelSize));
  const nc = Math.max(2, Math.ceil(t.chord_root_m / panelSize));
  const tris = [];
  for (let j = 0; j < ns; j++) for (let i = 0; i < nc; i++) {
    const a = j * (nc + 1) + i, b = a + 1, c = a + nc + 1, d = c + 1;
    tris.push(a, b, d, a, d, c);
  }
  const mk = (side, mirrorZ) => {
    const nv = (ns + 1) * (nc + 1);
    const pos = new Float64Array(nv * 3), vel = new Float64Array(nv * 3);
    for (let j = 0; j <= ns; j++) for (let i = 0; i <= nc; i++) {
      const eta = j / ns, xi = i / nc;
      const s = eta * t.span_each_m;
      const c = t.chord_root_m * (1 - (1 - (t.taper ?? 0.7)) * eta);
      // flight.js takes (x_m, y_m, z_m) as the aerodynamic reference of a section: quarter chord here.
      const dc = (0.25 - xi) * c;
      const x = t.x_m - (t.sweep_m ?? 0) * eta + dc * Math.cos(inc);
      const y = t.y_m + s * Math.sin(t.dihedral_rad ?? 0) + dc * Math.sin(inc);
      const z = side * (t.z_m + s * Math.cos(t.dihedral_rad ?? 0));
      const L = toLat(x, y, z);
      const v = (j * (nc + 1) + i) * 3;
      pos[v] = L[0]; pos[v + 1] = L[1]; pos[v + 2] = L[2];
    }
    return { part: PART.tail, kind: 'thin', tris: Uint32Array.from(tris), mirrorZ, moving: false, ref: () => ref, pose: () => ({ pos, vel }) };
  };
  return half ? [mk(1, true)] : [mk(1, false), mk(-1, false)];
}

/** Peak wall speed over a cycle, m/s, for choosing the lattice time step. */
export function peakWallSpeed(kin, design, freq, amp) {
  const wing = kin.makeWing(design.wingSpec);
  const ls = kin.makeLatticeSpec(wing, 2, 8, { cosine: false });
  const omega = 2 * Math.PI * freq, eps = 1e-3, N = (3 * 9) * 3;
  let vmax = 0;
  for (let i = 0; i < 64; i++) {
    const ph = (2 * Math.PI * i) / 64;
    const a = kin.wingGrid(wing, ls, kin.jointState(ph - eps, omega, 0, amp, design.law, design.sched, 1), 1);
    const b = kin.wingGrid(wing, ls, kin.jointState(ph + eps, omega, 0, amp, design.law, design.sched, 1), 1);
    const k = omega / (2 * eps);
    for (let v = 0; v < N; v += 3) vmax = Math.max(vmax, Math.hypot((b[v] - a[v]) * k, (b[v + 1] - a[v + 1]) * k, (b[v + 2] - a[v + 2]) * k));
  }
  return vmax;
}
