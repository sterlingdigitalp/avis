// Body and tail geometry: the one description the tunnels, OpenFOAM, page and film build from. Pure functions of
// the design file, no dependencies, node and browser. Body frame (SPEC): x forward, y up, z right, metres; the
// wing hinge line is at x = 0.
//
// Body: a loft of elliptic cross-sections along x (design.body.loft.sections), from the nose to the end of the
// tail boom. Each section { x_m, y_m, width_m, height_m } is an ellipse centred at (x_m, y_m, 0); width is along z.
// Tail: two V surfaces (design.tail), each a straight-tapered flat panel from the root at the boom to the tip,
// dihedral about body x, quarter-chord at tail.x_m (where the flight model puts the tail force).

const TWO_PI = 2 * Math.PI;

/**
 * Lofted section list for a pod-and-boom fuselage. Inputs are the builder's body numbers:
 *  { noseX_m, podLength_m, width_m, height_m, boomOd_m, tailEndX_m, y_m }.
 * Nose: elliptic profile over the first 40% of the pod; mid-pod constant; aft 35% tapers to the boom diameter.
 */
export function podAndBoomLoft({ noseX_m, podLength_m, width_m, height_m, boomOd_m, tailEndX_m, y_m = 0 }, nPod = 14) {
  const sections = [];
  const xTail = noseX_m - podLength_m;
  const fNose = 0.4, fAft = 0.35;
  for (let i = 0; i <= nPod; i++) {
    const u = i / nPod; // 0 nose tip, 1 pod end
    let k;
    if (u <= fNose) {
      const v = 1 - u / fNose;
      k = Math.sqrt(Math.max(0, 1 - v * v));
    } else if (u <= 1 - fAft) k = 1;
    else k = 1 - (u - (1 - fAft)) / fAft; // linear to the boom
    const w = Math.max(boomOd_m, k * width_m);
    const h = Math.max(boomOd_m, k * height_m);
    sections.push({ x_m: noseX_m - u * podLength_m, y_m, width_m: i === 0 ? 0 : w, height_m: i === 0 ? 0 : h });
  }
  if (tailEndX_m < xTail) sections.push({ x_m: tailEndX_m, y_m, width_m: boomOd_m, height_m: boomOd_m });
  return { kind: 'elliptic-loft', sections };
}

/** Half widths of the body at x by linear interpolation of the loft; zero outside it. */
export function bodySectionAt(loft, x) {
  const s = loft.sections;
  if (x > s[0].x_m || x < s[s.length - 1].x_m) return null;
  for (let i = 1; i < s.length; i++) {
    if (x >= s[i].x_m) {
      const a = s[i - 1], b = s[i];
      const f = a.x_m === b.x_m ? 0 : (a.x_m - x) / (a.x_m - b.x_m);
      return { x_m: x, y_m: a.y_m + f * (b.y_m - a.y_m), width_m: a.width_m + f * (b.width_m - a.width_m), height_m: a.height_m + f * (b.height_m - a.height_m) };
    }
  }
  return null;
}

/** Volume of the loft (frustum rule on elliptic areas, exact for linear radius variation per segment). */
export function loftVolume(loft) {
  let v = 0;
  const s = loft.sections;
  for (let i = 1; i < s.length; i++) {
    const a = s[i - 1], b = s[i];
    const dx = Math.abs(a.x_m - b.x_m);
    // Elliptic area pi/4 w h with both semi-axes linear in x: integral = pi/4 dx (w0h0 + (w0h1 + w1h0)/2 + w1h1)/3 ... exact form
    const w0 = a.width_m, w1 = b.width_m, h0 = a.height_m, h1 = b.height_m;
    v += (Math.PI / 4) * dx * (w0 * h0 / 3 + (w0 * h1 + w1 * h0) / 6 + w1 * h1 / 3);
  }
  return v;
}

/** Wetted area of the loft (ellipse perimeters by Ramanujan, trapezoid along x with the slant included). */
export function loftWettedArea(loft) {
  const per = (w, h) => {
    const a = w / 2, b = h / 2;
    if (a + b === 0) return 0;
    const H = ((a - b) / (a + b)) ** 2;
    return Math.PI * (a + b) * (1 + (3 * H) / (10 + Math.sqrt(4 - 3 * H)));
  };
  let A = 0;
  const s = loft.sections;
  for (let i = 1; i < s.length; i++) {
    const p0 = per(s[i - 1].width_m, s[i - 1].height_m), p1 = per(s[i].width_m, s[i].height_m);
    const dx = Math.abs(s[i - 1].x_m - s[i].x_m);
    const dr = ((s[i - 1].width_m + s[i - 1].height_m) - (s[i].width_m + s[i].height_m)) / 4;
    A += 0.5 * (p0 + p1) * Math.hypot(dx, dr);
  }
  return A;
}

/**
 * Surface mesh of the loft: rings of nAround points per section, as a flat array of [x, y, z] triples, ring-major,
 * starting at the top (y max) and running towards +z. Triangulate ring i, i+1 as quads.
 */
export function bodyMesh(loft, nAround = 24) {
  const pts = [];
  for (const s of loft.sections) {
    for (let k = 0; k < nAround; k++) {
      const t = (TWO_PI * k) / nAround;
      pts.push([s.x_m, s.y_m + 0.5 * s.height_m * Math.cos(t), 0.5 * s.width_m * Math.sin(t)]);
    }
  }
  return { rings: loft.sections.length, nAround, points: pts };
}

/**
 * The two tail surfaces as flat panels. Matches the flight model's tail stations (flight.js buildBird): root at
 * (x_m, y_m, +/-z_m), span along (0, sin d, +/-cos d), quarter-chord at x_m - sweep_m * eta, chord tapering
 * linearly from chord_root_m to taper * chord_root_m. Corners in order LE root, LE tip, TE tip, TE root.
 */
export function tailPanels(tail) {
  const out = [];
  const taper = tail.taper ?? 0.7;
  const d = tail.dihedral_rad;
  const sweep = tail.sweep_m ?? 0;
  for (const side of [1, -1]) {
    const cr = tail.chord_root_m, ct = cr * taper;
    const root = [tail.x_m, tail.y_m, side * tail.z_m];
    const tip = [tail.x_m - sweep, tail.y_m + tail.span_each_m * Math.sin(d), side * (tail.z_m + tail.span_each_m * Math.cos(d))];
    out.push({
      side: side > 0 ? 'right' : 'left',
      corners: [
        [root[0] + 0.25 * cr, root[1], root[2]],
        [tip[0] + 0.25 * ct, tip[1], tip[2]],
        [tip[0] - 0.75 * ct, tip[1], tip[2]],
        [root[0] - 0.75 * cr, root[1], root[2]],
      ],
      normal: [0, Math.cos(d), -side * Math.sin(d)],
      chord_root_m: cr, chord_tip_m: ct, span_m: tail.span_each_m, dihedral_rad: d,
      area_m2: 0.5 * (cr + ct) * tail.span_each_m,
      section: tail.polar ?? 'thin-plate',
      deflection: 'whole surface rotates about its quarter-chord line (all-moving ruddervator), +/- tail.deflection_max_rad',
    });
  }
  return out;
}

/** Everything a renderer or mesher needs from a design, besides the wing (kinematics.js wingGrid). */
export function bodyAndTail(design) {
  const loft = design.body.loft;
  return { loft, mesh: loft ? bodyMesh(loft) : null, tail: tailPanels(design.tail), volume_m3: loft ? loftVolume(loft) : null };
}
