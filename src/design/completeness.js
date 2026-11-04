// Completeness (SPEC "Design freeze", "Completeness and assembly order", 2026-11-03): a check can only clear the parts
// it can see. Every BOM line that is a physical object has solids in the assembly check, each solid tagged with its
// BOM line (src/design/clearance.js addPart: solid.bom, solid.item), and their count and size match the line:
//  - count: a discrete line (a bought part's count, a structure line's "N off" or count) has that many items, an item
//    being the solids of one physical piece (solid.item, else its part);
//  - size: a bought part's solids match its catalogue dimensions; a fastener's head, shank, nut and band match its
//    catalogue entry; stock bought by the metre (tube, rod) matches its section and the BOM's length; a part sized
//    from a material's density holds the BOM's volume (mass / density) within VOLUME_TOL.
// Only things with no fixed place are exempt: glue, wire and the unlisted margin (none of them is a BOM line), and the
// covering film (EXEMPT): the pod's covering is the loft the containment check already holds every part inside, the
// wing's and tail's covering is the surface the flight model flies. Each exemption is in the record with its reason.
import { byId, tubeSpec } from './catalog.js';
import { parseDims, catalogueDims } from '../../sims/avis-shared/model.js';

const TWO_PI = 2 * Math.PI;
/** Diameter, thickness and section tolerance (m): catalogue numbers are used as stated, so only rounding is allowed. */
export const DIM_TOL = 5e-5;
/** Length tolerance per item (m), for stock cut to length. */
export const LEN_TOL = 5e-4;
/**
 * Volume tolerance for parts sized from a material's density: the solids stand for the part as the clearance check
 * reads it (a capsule for a bar contains the bar, its round ends included), so they may hold a little more than the
 * BOM's own pieces; never less than the BOM by more than the tolerance (the BOM would then count material that is
 * not there) and never more by more than the tolerance (material in the check that the BOM does not weigh).
 */
export const VOLUME_TOL = 0.1;

/** Lines exempt from solids, with the reason: things with no fixed place. */
export const EXEMPT = [
  { re: /^wing film\b/, why: 'covering film (no fixed place as a solid): the wing\'s covering is the lifting surface the flight model flies, stretched over the ribs and edge rods that are in the check' },
  { re: /^fuselage film shell$/, why: 'covering film (no fixed place as a solid): the pod\'s covering is the loft itself, and the containment check keeps every enclosed part 0.5 mm inside it' },
  { re: /^tail film\b/, why: 'covering film (no fixed place as a solid): the tail\'s covering is the surface the flight model flies, stretched over the tail\'s rods and ribs that are in the check' },
];
/** What the BOM does not list line by line, so has no solids: the SPEC's 15% unlisted margin. */
export const NOT_LINES = [
  { what: 'glue (Araldite 2011, LOCTITE 406 with SF 770, LOCTITE 638)', why: 'no fixed place: a film in the bonded joints, in the 15% unlisted margin (the catalogue counts adhesives at no mass)' },
  { what: 'wire', why: 'no fixed place: routed between the parts, in the 15% unlisted margin' },
  { what: 'the unlisted margin itself (15%)', why: 'not a part: the SPEC M6 allowance for wire, glue and fasteners' },
];

const offRe = /,\s*(\d+)\s*off$/;
/**
 * The design's BOM lines: bought parts (bom.parts, one line per catalogue id, counts summed over the roles that list
 * it) and structure items (bom.structure, by name). -> [{ key, kind, id, name, count, mass_g, exempt }].
 */
export function bomLines(d) {
  const parts = new Map();
  for (const p of d.bom.parts) {
    const e = parts.get(p.id) ?? { key: p.id, kind: 'part', id: p.id, name: p.name ?? p.id, roles: [], count: 0, mass_g: 0 };
    e.roles.push(p.role); e.count += p.count; e.mass_g += p.mass_g;
    parts.set(p.id, e);
  }
  const structure = d.bom.structure.map((s) => {
    const ex = EXEMPT.find((x) => x.re.test(s.name));
    return { key: s.name, kind: 'structure', id: s.material_id, name: s.name, count: s.count ?? +(s.name.match(offRe)?.[1] ?? 1), mass_g: s.mass_g, basis: s.basis, ...(ex ? { exempt: ex.why } : {}) };
  });
  return [...parts.values(), ...structure];
}

/**
 * The BOM line a solid's tag names: a bought part's catalogue id, a structure line's name (with or without its
 * ", N off"), or a structure line's material id where only one line has it. -> the line's key, or null.
 */
export function resolveTag(lines, tag) {
  if (!tag) return null;
  const exact = lines.find((l) => l.key === tag);
  if (exact) return exact.key;
  const stem = lines.filter((l) => l.kind === 'structure' && l.name.replace(offRe, '') === tag);
  if (stem.length === 1) return stem[0].key;
  const byMat = lines.filter((l) => l.kind === 'structure' && l.id === tag);
  return byMat.length === 1 ? byMat[0].key : null;
}

// ---- measures of solids ----------------------------------------------------------------------------------------

const shapeOf = (s, q0) => (s.at ? s.at(q0) : s.shape);
const hyp = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]);
const polyArea = (pts) => Math.abs(pts.reduce((A, p, i) => A + p[0] * pts[(i + 1) % pts.length][1] - pts[(i + 1) % pts.length][0] * p[1], 0)) / 2;

/** Area (m2) of a cross-section: a ring's annulus, a capsule's stadium, a polygon less its holes. */
export function shapeArea(sh) {
  if (sh.kind === 'ring') return Math.PI * (sh.ro ** 2 - (sh.ri ?? 0) ** 2);
  if (sh.kind === 'cap') return Math.PI * sh.r ** 2 + 2 * sh.r * hyp(sh.a, sh.b);
  return polyArea(sh.pts) - (sh.holes ?? []).reduce((a, h) => a + Math.PI * h.r * h.r, 0);
}

/**
 * Volume (m3) of a solid: its section's area over its x range, or, for a tube along another axis (solid.tube: a wing
 * spar, drawn as its filled envelope across x), the tube's own annulus over its length.
 */
export function solidVolume(s, q0) {
  if (s.tube) return Math.PI * (s.tube.ro ** 2 - (s.tube.ri ?? 0) ** 2) * s.tube.L;
  // a piece overlapping its part's other solids states the area it adds (a plate's lock ear outside the plate's hull)
  if (Number.isFinite(s.area_m2)) return s.area_m2 * (s.x[1] - s.x[0]);
  return shapeArea(shapeOf(s, q0)) * (s.x[1] - s.x[0]);
}

/** A solid's section as a tube: { ro, ri, L } (m), from solid.tube or an x-prism ring. */
function tubeOf(s, q0) {
  if (s.tube) return s.tube;
  const sh = shapeOf(s, q0);
  return sh.kind === 'ring' ? { ro: sh.ro, ri: sh.ri ?? 0, L: s.x[1] - s.x[0] } : null;
}

const near = (a, b, tol = DIM_TOL) => Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tol + 1e-12;
const r4 = (v) => (Number.isFinite(v) ? +(v * 1000).toFixed(3) : null);
const row = (what, found, want, ok, tol = DIM_TOL) => ({ what, found_mm: r4(found), bom_mm: r4(want), tol_mm: r4(tol), ok });

// ---- size checks by kind -----------------------------------------------------------------------------------------

/** A check of one measured value against the catalogue's (both in m). */
const chk = (what, found, want) => row(what, found, want, near(found, want));
/** The first solid of an item whose name matches, as a tube (see tubeOf). */
const tubeNamed = (sols, re, q0) => { const x = sols.find((q) => re.test(q.name)); return x ? tubeOf(x, q0) : null; };

/** A gear's teeth solid: its tip circle, bore and face width against the catalogue's. */
function gearSize(s, it, sols, q0) {
  const od = (s.outside_dia_mm ?? s.module_mm * ((s.teeth ?? 0) + 2)) / 1000, t = tubeNamed(sols, /^teeth$/, q0);
  return [chk(`${it}: tip diameter`, t && 2 * t.ro, od), chk(`${it}: bore`, t && 2 * t.ri, s.bore_mm / 1000), chk(`${it}: face width`, t?.L, s.face_width_mm / 1000)];
}

/** A bushing or bearing: its bore, outside diameter, length (body and flange together) and flange diameter. */
function journalSize(s, it, sols, q0) {
  const m = String(s.dimensions_mm_d_D_B ?? '').split('x').map(Number);
  const dims = Number.isFinite(s.d1_mm) ? { d: s.d1_mm, D: s.d2_mm, B: s.b1_mm, F: s.d3_mm } : { d: m[0], D: m[1], B: m[2], F: null };
  const b = tubeNamed(sols, /^body$/, q0), f = tubeNamed(sols, /^flange$/, q0);
  const len = Math.max(...sols.map((x) => x.x[1])) - Math.min(...sols.map((x) => x.x[0])); // body and flange overlap in x
  return [chk(`${it}: bore`, b && 2 * b.ri, dims.d / 1000), chk(`${it}: outside diameter`, b && 2 * b.ro, dims.D / 1000), chk(`${it}: length`, len, dims.B / 1000), ...(dims.F ? [chk(`${it}: flange diameter`, f && 2 * f.ro, dims.F / 1000)] : [])];
}

/** The motor: its can's diameter and length and its shaft's diameter. */
function motorSize(s, it, sols, q0) {
  const md = parseDims(s.dimensions_text_mm)?.values, c = tubeNamed(sols, /^can$/, q0), sh = tubeNamed(sols, /^shaft$/, q0);
  return [chk(`${it}: can diameter`, c && 2 * c.ro, md?.[0]), chk(`${it}: can length`, c?.L, md?.[1]), chk(`${it}: shaft`, sh && 2 * sh.ro, s.shaft_mm / 1000)];
}

/** A ring-shaped part (the crank encoder's readhead: od, id, thickness; its actuator: od, inner diameter, height). */
function ringPartSize(s, it, sols, q0) {
  const od = (s.dimensions?.od_mm ?? s.od_mm) / 1000, id = (s.dimensions?.id_mm ?? s.inner_diameter_mm) / 1000, h = (s.dimensions?.thickness_mm ?? s.h_mm) / 1000;
  const r = envelope(sols, q0);
  return [chk(`${it}: outside diameter`, r.od, od), chk(`${it}: bore`, r.id, id), chk(`${it}: thickness`, r.len, h)];
}

/** A packed box: its three sides against the catalogue's three dimensions, sorted (a box in a turning frame states its own). */
function boxSize(s, it, sols, q0) {
  const dims = catalogueDims(s)?.values;
  if (!dims || dims.length < 3) return [{ what: `${it}: no catalogue dimensions (drawn from its mass, model.js sizeOf)`, ok: true, note: 'nothing to compare' }];
  const e = envelope(sols, q0), got = (sols.length === 1 && sols[0].box ? [...sols[0].box] : [e.len, e.dz, e.dy]).sort((a, b) => a - b), want = [...dims].sort((a, b) => a - b);
  return got.map((g, i) => chk(`${it}: box dimension ${i + 1}`, g, want[i]));
}

/** How a bought part's items are measured, from its catalogue category and dimensions. */
function partMeasure(p) {
  const s = p.specs ?? {};
  if (p.category === 'gear') return gearSize;
  if (p.category === 'bushing' || p.category === 'bearing') return journalSize;
  if (p.category === 'motor') return motorSize;
  if (s.dimensions?.od_mm || Number.isFinite(s.inner_diameter_mm)) return ringPartSize;
  return boxSize;
}

/** The catalogue's dimensions of a bought part, each item measured by its kind. */
function catalogueSize(line, items, catalog, q0) {
  const p = byId(catalog, line.id), f = partMeasure(p);
  return items.flatMap((it) => f(p.specs ?? {}, it.item, it.solids, q0));
}

/** The extents of an item's solids: x length, (z, y) bounding box, and, for coaxial rings, outer and bore diameters. */
function envelope(sols, q0) {
  const xs = sols.flatMap((x) => x.x), shapes = sols.map((x) => shapeOf(x, q0));
  const pts = shapes.flatMap((sh) => (sh.kind === 'ring' ? [[sh.c[0] - sh.ro, sh.c[1] - sh.ro], [sh.c[0] + sh.ro, sh.c[1] + sh.ro]] : sh.kind === 'cap' ? [[Math.min(sh.a[0], sh.b[0]) - sh.r, Math.min(sh.a[1], sh.b[1]) - sh.r], [Math.max(sh.a[0], sh.b[0]) + sh.r, Math.max(sh.a[1], sh.b[1]) + sh.r]] : sh.pts));
  const z = pts.map((p) => p[0]), y = pts.map((p) => p[1]);
  const rings = shapes.filter((sh) => sh.kind === 'ring'), polys = shapes.filter((sh) => sh.kind === 'poly');
  // a ring, or a polygon drawn round a circle with its bore as a hole (the readhead with its screw holes)
  const od = rings.length ? 2 * Math.max(...rings.map((r) => r.ro)) : polys.length ? Math.max(...z) - Math.min(...z) : null;
  const bores = [...rings.map((r) => r.ri ?? 0), ...polys.flatMap((p) => (p.holes ?? []).map((h) => h.r))];
  return { len: Math.max(...xs) - Math.min(...xs), dz: Math.max(...z) - Math.min(...z), dy: Math.max(...y) - Math.min(...y), od, id: bores.length ? 2 * Math.max(...bores) : 0 };
}

/** A fastener's catalogue geometry: screw head d x k and shank d x L, nut across flats s x m, tie band b x t and length. */
export function fastenerGeometry(spec, id) {
  const t = `${spec.mass_basis ?? ''} ${spec.dimensions ?? ''}`;
  if (/^screw/.test(id)) {
    const h = t.match(/head ([\d.]+) mm x ([\d.]+) mm/), d = +(id.match(/-m(\d+(?:\.\d+)?)x/)?.[1] ?? NaN);
    const L = (spec.length_mm ?? +(id.match(/x(\d+(?:\.\d+)?)-/)?.[1] ?? NaN)) / 1000, k = (spec.head_k_mm ?? +h?.[2]) / 1000;
    // round 17: a countersunk screw's nominal length includes its head, so its shank is the rest
    const csk = /countersunk/.test(spec.head ?? '');
    return { kind: 'screw', head_d: (spec.head_d_mm ?? +h?.[1]) / 1000, head_k: k, d: d / 1000, L, shank_L: csk ? L - k : L, countersunk: csk };
  }
  if (/^nut/.test(id)) {
    const n = t.match(/([\d.]+) mm across flats x ([\d.]+) mm high/);
    return { kind: 'nut', s: (spec.s_mm ?? +n?.[1]) / 1000, m: (spec.m_mm ?? +n?.[2]) / 1000 };
  }
  // Round 17 (audit items 7, 8): a DIN 6799 retaining washer or DIN 471 circlip, drawn as one ring of its own
  // thickness; the circlip's "outside diameter" is read as d4, the installed ring's swept diameter (the only one of
  // its dimensions that bounds how far it reaches, SOURCES-retention.md), not its free, unsprung d3.
  if (/^washer-din6799/.test(id)) return { kind: 'washer', od: spec.outer_diameter_d3_mm / 1000, id: spec.groove_diameter_d2_mm / 1000, t: spec.thickness_s_mm / 1000 };
  if (/^circlip-din471/.test(id)) return { kind: 'washer', od: spec.d4_mm / 1000, id: spec.groove_diameter_d2_mm / 1000, t: spec.thickness_s_mm / 1000 };
  const b = t.match(/^\s*([\d.]+)[^x]*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/) ?? t.match(/([\d.]+)\D+x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/);
  return { kind: 'tie', L: +b?.[1] / 1000, b: +b?.[2] / 1000, t: +b?.[3] / 1000 };
}

/** Each fastener item against its catalogue geometry: a screw's head and shank, a nut's flats and height, a tie's band. */
function fastenerSize(line, items, catalog, q0) {
  const g = fastenerGeometry(byId(catalog, line.id).specs ?? {}, line.id), out = [];
  for (const { item, solids } of items) {
    const by = (re) => solids.find((x) => re.test(x.name));
    if (g.kind === 'screw') {
      const h = by(/head/), s = by(/shank/);
      const ht = h && tubeOf(h, q0), st = s && tubeOf(s, q0);
      out.push(row(`${item}: head diameter`, ht && 2 * ht.ro, g.head_d, near(ht && 2 * ht.ro, g.head_d)), row(`${item}: head height`, ht?.L, g.head_k, near(ht?.L, g.head_k)), row(`${item}: shank diameter`, st && 2 * st.ro, g.d, near(st && 2 * st.ro, g.d)), row(`${item}: shank length`, st?.L, g.shank_L, near(st?.L, g.shank_L)));
    } else if (g.kind === 'nut') {
      const n = tubeOf(solids[0], q0);
      // drawn as its corner circle: across flats = corner diameter x cos 30 deg
      out.push(row(`${item}: across flats`, n && 2 * n.ro * Math.cos(Math.PI / 6), g.s, near(n && 2 * n.ro * Math.cos(Math.PI / 6), g.s)), row(`${item}: height`, n?.L, g.m, near(n?.L, g.m)));
    } else if (g.kind === 'washer') {
      const t = tubeOf(solids[0], q0);
      out.push(row(`${item}: outside diameter`, t && 2 * t.ro, g.od, near(t && 2 * t.ro, g.od)), row(`${item}: bore`, t && 2 * t.ri, g.id, near(t && 2 * t.ri, g.id)), row(`${item}: thickness`, t?.L, g.t, near(t?.L, g.t)));
    } else {
      // a tie's slabs: the band width along x and the band thickness across it; the loop no longer than the tie
      const widths = solids.map((x) => x.x[1] - x.x[0]), thin = solids.map((x) => { const e = envelope([x], q0); return Math.min(e.dz, e.dy); });
      const loop = tieLoop(solids, q0);
      out.push(row(`${item}: band width`, Math.max(...widths), g.b, widths.every((w) => near(w, g.b))), row(`${item}: band thickness`, Math.max(...thin), g.t, thin.every((w) => near(w, g.t))), { ...row(`${item}: loop length (at most the tie's)`, loop, g.L, loop <= g.L + 1e-9, 0), rule: 'at most' });
    }
  }
  return out;
}

/** A tie's loop length: the centre line round its four slabs (top, two sides, bottom). */
function tieLoop(solids, q0) {
  const e = envelope(solids, q0), t = Math.min(...solids.map((x) => { const v = envelope([x], q0); return Math.min(v.dz, v.dy); }));
  return 2 * (e.dz - t) + 2 * (e.dy - t);
}

/** Stock bought by the metre: every item's section against the catalogue, the items' total length against the BOM's. */
function stockSize(line, items, catalog, q0) {
  const p = byId(catalog, line.id), s = p.specs ?? {}, out = [], ts = tubeSpec(p, !!line.specSheet);
  // round 17: a tube with two published sections is drawn on their envelope and weighed at the heavier figure
  const lenBom = /^([\d.]+) mm of /.test(line.basis ?? '') ? +line.basis.match(/^([\d.]+) mm of /)[1] / 1000 * (line.count ?? 1) : line.mass_g / ts.g_per_m;
  let total = 0;
  for (const { item, solids } of items) {
    const t = solids.map((x) => tubeOf(x, q0)).filter(Boolean);
    const L = t.reduce((a, q) => a + q.L, 0);
    total += L;
    const od = Math.max(...t.map((q) => 2 * q.ro)), id = Math.min(...t.map((q) => 2 * q.ri));
    out.push(row(`${item}: outside diameter`, od, ts.od_draw / 1000, near(od, ts.od_draw / 1000)));
    if (!/^pitot/.test(line.name)) out.push(row(`${item}: bore`, id, ts.id_draw / 1000, near(id, ts.id_draw / 1000)));
  }
  out.push(row('total length', total, lenBom, near(total, lenBom, LEN_TOL * items.length), LEN_TOL * items.length));
  return out;
}

/** A part sized from its material's density: the solids' volume against the BOM's mass / density. */
function volumeSize(line, items, catalog, q0) {
  const m = byId(catalog, line.id), rho = (m.specs?.density_g_cm3 ?? (m.specs?.density_kg_m3 ?? NaN) / 1000) * 1e6 / 1000; // kg/m3
  const vBom = line.mass_g / 1000 / rho;
  const v = items.reduce((a, it) => a + it.solids.reduce((b, x) => b + solidVolume(x, q0), 0), 0);
  const ratio = v / vBom;
  // a lightened part drawn by its envelope (a rib): the BOM's material must fit inside the solids
  const env = items.every((it) => it.solids.every((x) => x.envelope));
  return [{ what: env ? 'volume (the BOM\'s lightened material within the solids\' envelope)' : 'volume (solids / BOM mass over density)', solids_mm3: +(v * 1e9).toFixed(1), bom_mm3: +(vBom * 1e9).toFixed(1), ratio: +ratio.toFixed(4), tol: VOLUME_TOL, ...(env ? { rule: 'envelope: at least the BOM\'s volume' } : {}), ok: env ? ratio >= 1 - VOLUME_TOL : Math.abs(ratio - 1) <= VOLUME_TOL }];
}

/** How a line's size is checked, from its catalogue entry. */
function sizeKind(line, catalog) {
  if (line.kind === 'part') return 'catalogue';
  const p = byId(catalog, line.id);
  if (p.category === 'fastener') return 'fastener';
  if (/per_metre/.test(p.specs?.mass_basis ?? '')) return 'stock';
  return 'volume';
}

/**
 * The completeness record of an assembly (clearance.js assembly plus the packed parts) against the design's BOM, the
 * moving solids measured at pose q0. Every line: its items and solids, its count against the BOM's, its size checks;
 * every solid with no line, or a tag naming no line, is listed. -> { rule, lines, untagged, unknown_tags, passed }.
 */
export function completenessRecord(asm, d, catalog, q0) {
  const lines = bomLines(d), byKey = new Map(lines.map((l) => [l.key, { ...l, items: new Map() }]));
  const untagged = [], unknown = new Map();
  for (const part of asm.parts) for (const s of part.solids) {
    const id = `${part.id}/${s.name}`;
    if (!s.bom) { untagged.push(id); continue; }
    const key = resolveTag(lines, s.bom);
    if (!key) { unknown.set(s.bom, [...(unknown.get(s.bom) ?? []), id]); continue; }
    const L = byKey.get(key), item = s.item ?? part.id;
    if (!L.items.has(item)) L.items.set(item, { item, solids: [] });
    L.items.get(item).solids.push({ ...s, id });
  }
  const rows = [...byKey.values()].map((L) => lineRow({ ...L, specSheet: !!d.params?.tubeSpecSheet }, catalog, q0));
  return {
    rule: 'SPEC Completeness and assembly order (2026-11-03): every BOM line that is a physical object has solids in the check, each tagged with its BOM line; a discrete line has as many items as the BOM counts; each item\'s size matches its line (catalogue dimensions within the rounding tolerance, stock length within the length tolerance, volume within the volume tolerance); only covering film, glue, wire and the unlisted margin are exempt; src/design/completeness.js',
    tolerances: { dimension_mm: DIM_TOL * 1000, length_per_item_mm: LEN_TOL * 1000, volume_frac: VOLUME_TOL },
    exemptions: [...rows.filter((r) => r.exempt).map((r) => ({ line: r.line, why: r.exempt })), ...NOT_LINES.map((n) => ({ line: n.what, why: n.why, not_a_bom_line: true }))],
    lines: rows,
    untagged,
    unknown_tags: [...unknown].map(([tag, solids]) => ({ tag, solids })),
    failed: rows.filter((r) => !r.passed).map((r) => r.line),
    passed: rows.every((r) => r.passed) && untagged.length === 0 && unknown.size === 0,
  };
}

/** One line's row: exempt, or its items and solids, count check and size checks. */
function lineRow(L, catalog, q0) {
  const items = [...L.items.values()];
  const base = { line: L.key, kind: L.kind, id: L.id, bom_count: L.count };
  if (L.exempt) return { ...base, exempt: L.exempt, items: items.length, passed: items.length === 0 };
  if (!items.length) return { ...base, items: 0, solids: 0, size: [], passed: false, why: 'no solids' };
  const kind = sizeKind(L, catalog);
  const size = { catalogue: catalogueSize, fastener: fastenerSize, stock: stockSize, volume: volumeSize }[kind](L, items, catalog, q0);
  const countOk = items.length === L.count;
  return { ...base, items: items.length, solids: items.reduce((a, it) => a + it.solids.length, 0), count_ok: countOk, size_kind: kind, size, passed: countOk && size.every((r) => r.ok), ...(countOk ? {} : { why: `${items.length} items, the BOM counts ${L.count}` }) };
}
