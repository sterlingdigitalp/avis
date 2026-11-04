// Round 17: axial retention in the drive, and bearing seats as wide as their bearings (docs/status/audit-v24.md
// rows 7, 8, 16, 23; section B findings F4, F5, F6). Exports RETENTION (deep-merge into params.linkage; its `layout`
// block deep-merges into params.layout instead, kept separate as asked) and RETENTION_SEQUENCE (insert/replace
// entries for round17.mjs's sequence17, in the same shape as that file's own SEQUENCE_INSERTS).
//
// Parts chosen from the staged catalogue (parts-v25/retention.json, SOURCES-retention.md):
//  - washer8mm = washer-din6799-6 (DIN 6799 size 6): tabulated AT d1 8 mm exactly (FN_at_shaft_d1_mm: 8), not just
//    within the size-5 range the audit's own estimate used. Sized for gear shafts 1 and 2 (8 mm). The eye bushing
//    bore (and so the crank and horn pins) is also 8 mm, so the same part would consolidate into one BOM line if
//    hornPinWasher below is ever turned on (see its note: left off this round, probed and found to clash).
//  - gearShaft3Circlip = circlip-din471-8 (DIN 471, 8 mm shaft): the aft bearing's inner ring stop on gear shaft 3.
//  - spacerMaterial = metal-al-6061-t6 (already in the catalogue; machined spacer tubes, as the crank encoder
//    spacers already are): fills the gear stacks' gaps instead of the catalogue's Albion tubes, whose alloy is
//    unstated (do not let a rating rest on them) and which are sized in 1 mm wall steps that do not fit this design's
//    gaps; collars are not used either (Climax's own sheet publishes no axial holding power for them).
//
// Round 17b (module "retain", audit items 7 and 16): hornPinWasher and crankPinWasher on, with the eye gap opened to
// make room (RETENTION.layout; the round-17 probe had found the horn pins' tips clash with the other coupler's eye at
// the 0.5 mm gap), the left horn boss cut to the hub's length and 25 mm across, and every plate seat bossed as long as
// its part (the six new bosses thin-walled; clearance.js bossSpan fixes the outward-seat boss that grew into the motor
// can). retentionContainment adds the pins' new solids to the horn slots.
export const RETENTION = {
  // Round 17b (audit item 7): the two coupler eyes' gap opened from the running clearance (0.5 mm) to the horn pins'
  // washer land plus it: both horn pins' free ends point into this gap (the right horn boss forward of its eye, the left
  // one aft of its own, so each boss stays clear of the other coupler's eye, which overlaps it in the plane: the horn
  // points come within 10.98 mm of each other over the stroke, the eyes are 24.5 mm across), and each pin's tip runs on
  // the washer's groove width m plus its land n (washer-din6799-6: 0.74 + 1.2 mm) past its eye, which must then clear the
  // other eye by the running clearance: 0.74 + 1.2 + 0.5 = 2.44 mm (test/design/retain.test.js pins this sum). The two
  // tips and washers clear each other in the plane (4 + 6.15 + 0.5 <= 10.98 mm). The left coupler, its horn stack and
  // the crank pin's overhang move 1.94 mm aft.
  layout: { eye_gap_mm: 2.44 },
  linkage: {
    // Round 17b (audit item 7): the left horn boss 25 mm across (23 in v24). It is cut to 5.46 mm by the wider eye gap
    // (it may not pass the hub's aft face: linkage.js hornStack), and at 23 mm its seat check (EN 1999-1-1 bolt bearing,
    // e/d 1.44) falls to an estimated FoS 1.44 under v24's peak coupler force; at 25 mm (e/d 1.56) about 1.56. The right
    // boss keeps its 7 mm and 23 mm: at 25 mm it comes within 0.36 mm of the aft boom clamp's flange (probed)
    rocker: { boss_od_side_mm: { L: 25 } },
    // Round 17b (retain2, item 9): the crank shaft's bore 5 -> 4 mm. Its FoS, estimated from v24's M7 peaks scaled by each
    // section's sum-of-channel-peaks bound (the eye gap moves the left coupler 1.94 mm aft, lengthening its lever at the
    // aft bushing), falls from v24's 1.589 to 1.504 at the 5 mm bore: under 1.5 once v25's 3 % higher mass raises the
    // wing's loads in proportion. At 4 mm it is 1.563 by the same estimate (+3.4 g). The lead's M7 run confirms. Wiring
    // (round17.mjs, the lead's): crank = mergeDeep(crank, RETENTION.linkage.crank).
    crank: { shaft_id_mm: 4 },
    // Round 17b (audit item 16, F6): every plate seat bossed as long as its part (round 17: three of nine). The six new
    // bosses take a thin wall of their own (boss_walls_mm, 1.5 mm): boss_wall_mm (8.5) round a 16 mm or 13 mm bearing
    // reaches into the shoulder rails and the bracket sleeve, and round the 10 mm bushings it would weigh 14 g for lugs
    // that the plate alone already carries at FoS 7 or more (v24 M7). Their lug stays judged on the plate alone
    // (linkage.js plateSeats); the boss houses the part over its full width, its root ring checked in shear
    plates: {
      bossed: ['crank shaft/aft', 'crank shaft/forward', 'gear shaft 1/aft', 'gear shaft 1/forward', 'gear shaft 2/aft', 'gear shaft 2/forward', 'gear shaft 3/aft', 'gear shaft 3/forward', 'motor shaft outboard/plate'],
      boss_walls_mm: { 'gear shaft 1/forward': 1.5, 'gear shaft 2/aft': 1.5, 'gear shaft 2/forward': 1.5, 'gear shaft 3/aft': 1.5, 'gear shaft 3/forward': 1.5, 'motor shaft outboard/plate': 1.5 },
    },
    retention: {
      washer8mm: 'washer-din6799-6',
      // round 17b (retain2): the round-17 circlip on gear shaft 3's aft inner ring is gone: its swept diameter d4
      // (14.7 mm) passes SKF's largest shaft abutment for the 628/8, da max 10.0 mm (SKF's table as Coolen Bearings
      // reproduces it: da min 9.4, da max 10.0, Da max 14.6, ra max 0.2 mm), so it would ride the shield. That bearing's
      // inner ring is bonded instead (bearingBonds)
      gearShaft3Circlip: null,
      // round 17b (retain2, item 1): gear shaft 2's forward end takes a DIN 471-8 circlip, whose groove and land (0.9 +
      // 0.6 mm) end 0.80 mm short of the DS1-60 overhung on gear shaft 3 where the washer's (0.74 + 1.2 mm) ended 0.36 mm
      // short of it
      forwardCirclip: { 2: 'circlip-din471-8' },
      // round 17b (retain2, item 3): the crank shaft's aft stop, a DIN 471-10 circlip at the forward bushing's flange
      crankCirclip: 'circlip-din471-10',
      // round 17b (retain2, items 2, 4, 5, 8): LOCTITE 638 (catalogue: ISO 10123 on aluminium pins and collars 17 MPa,
      // counted at half) holds every keyed gear on its 7075 shaft (on the titanium crank: RC practice, no Ti figure) and
      // the bearings' rings below: the round-17 spacer tubes are gone (linkage.js gearBond, clearance.js
      // gearShaftRetention)
      bond: 'adhesive-loctite-638',
      // the bonded rings: both of gear shaft 3's 628/8s, inner and outer (its shaft turns, so an inner ring left loose on it
      // would creep; held at both plates, the 7075 shaft and the 6061 plates, panels and 7075 rails between them grow
      // alike with temperature); the motor's outboard 619/5 bonded in its seat, its inner ring sliding on the motor
      // shaft (a non-locating support: the motor's own bearings locate that shaft)
      bearingBonds: { 'gear shaft 3': ['aft inner', 'aft outer', 'forward inner', 'forward outer'], 'motor shaft outboard': ['forward outer'] },
      // round 17b (retain2, audit item 22): the horn pins in their bosses and the shoulder pins in the rocker hubs, all
      // 7075, bonded with Araldite 2011 (aluminium lap shear, typical, counted at half) instead of unrated press fits
      pinBond: 'adhesive-araldite-2011-aluminium',
      // round 17b (audit item 7): the horn pins and the crank pin take washer8mm too (clearance.js pinWasherPart), the
      // eye gap opened to make room (RETENTION.layout above)
      hornPinWasher: true,
      crankPinWasher: true,
      // Found by probing: gear shaft 2's forward end, lengthened for the washer, comes within 0.36 mm of the DS1-60
      // wheel overhung on gear shaft 3 (its 31 mm tip circle reaches toward the adjacent shaft): no washer there, the
      // circlip above instead.
      washer8mmForwardExclude: [2],
    },
  },
};

/**
 * Round 17b: the containment declarations (params.layout.containment) with the pins' new solids: the horn pins'
 * grooves, tips and washers pass through the horn slots with the eyes they retain (round 15's 'horn slots' opening).
 * The crank pin's washer stays inside the pod. -> a new containment object (the input is not changed).
 */
export function retentionContainment(c) {
  if (!c?.openings) return c;
  const add = ['R', 'L'].flatMap((t) => [`rocker ${t}/horn pin groove`, `rocker ${t}/horn pin tip`, `horn pin washer ${t === 'R' ? 'right' : 'left'}/washer`]);
  return { ...c, openings: c.openings.map((o) => (o.name === 'horn slots' ? { ...o, solids: [...new Set([...(o.solids ?? []), ...add])] } : o)) };
}

const step = (name, members, dir, dist_m, why) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
const X = (s) => [s, 0, 0];
// A DIN 6799 washer or DIN 471 circlip is a sprung E-clip, snapped on sideways at its groove, not threaded axially
// over the shaft's end (its own catalogue ID is the groove's diameter, narrower than the shaft it sits on, which a
// rigid ring could not pass over). Its assembly step approaches radially, in y, not along the shaft's axis.
const Y = (s) => [0, s, 0];
const Z = (s) => [0, 0, s];

/**
 * Round 17's assembly-sequence patches for retention (round17.mjs sequence17 convention: `after` appends following
 * the named round-16 step, `replace` swaps it; both may target the same original name, applied in that order).
 *  - An aft washer sits outside the aft plate, already in place from the sequence's first step, so it is reachable
 *    right after its shaft's own rigid-group step.
 *  - A forward ring sits outside the forward plate, which does not arrive until later (round16's own "forward plate
 *    with its journals" step, well after every gear shaft's): fitting it right after the shaft's own step would have
 *    the forward plate's own journals, which still have to travel up to 80 mm to reach their seats, sweep straight
 *    through it. So forward rings (gear shaft 1's washer, gear shaft 2's and the crank's circlips) wait for `after:
 *    'forward plate with its journals'`.
 *  - Round 17b (retain2): the bonds (gears on their shafts, gear shaft 3's and the motor's outboard bearings' rings,
 *    the pins in their bosses) are made as each part goes on in its own step; nothing new to place.
 */
export const RETENTION_SEQUENCE = [
  // round 17b: the pins' washers, snapped on sideways once each coupler is on its pins (the crank pin's after the left
  // coupler, the last on it)
  { after: 'horn pin R', steps: [step('horn pin washer right', ['^horn pin washer right$'], Y(1), 0.008, 'snapped on sideways at the horn pin\'s groove, against the right coupler\'s horn eye')] },
  { after: 'horn pin L', steps: [
    step('horn pin washer left', ['^horn pin washer left$'], Y(1), 0.008, 'snapped on sideways at the horn pin\'s groove, against the left coupler\'s horn eye'),
    step('crank pin washer', ['^crank pin washer$'], Z(1), 0.008, 'snapped on sideways at the crank pin\'s groove, against the left coupler\'s crank eye flange'),
  ] },
  // round 17b (retain2): no spacer tubes (each gear bonded on its shaft), so the shafts go in with their gears and keys
  // as round 16 has them; the end rings snap on after
  { after: 'gear shaft 1 with its gears and keys', steps: [step('gear shaft 1 washer aft', ['^gear shaft 1 washer aft$'], Y(1), 0.008, 'snapped on sideways at its groove, against the aft bushing\'s flange (the aft plate is already placed)')] },
  { after: 'gear shaft 2 with its gears and keys', steps: [step('gear shaft 2 washer aft', ['^gear shaft 2 washer aft$'], Y(1), 0.008, 'snapped on sideways at its groove, against the aft bushing\'s flange (the aft plate is already placed)')] },
  { after: 'forward plate with its journals', steps: [
    step('gear shaft 1 washer forward', ['^gear shaft 1 washer forward$'], Y(1), 0.008, 'snapped on sideways at its groove, against the forward bushing\'s flange, now that the forward plate has reached its seat'),
    step('gear shaft 2 circlip forward', ['^gear shaft 2 circlip forward$'], Y(1), 0.008, 'opened over the shaft\'s forward tip with pliers and let close in its groove, against the forward bushing\'s flange'),
    step('crank shaft circlip forward', ['^crank shaft circlip forward$'], Y(-1), 0.01, 'opened over the crank shaft with pliers and let close in its groove, against the forward bushing\'s flange'),
  ] },
];
