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
// hornPinWasher: false (audit item 7, F5, horn half; the crank-pin half is not attempted at all — see the report).
// Probed on the actual geometry (tools/run-guarded.py quick17.mjs with this file's RETENTION + RETENTION_SEQUENCE
// merged in): lengthening either horn pin by washer8mm's groove and land (1.94 mm) clashes with the OPPOSITE side,
// about -1.4 mm, because both pins' free ends point in toward the coupler eyes the shared crank pin stacks together
// (params.layout.horns: R 'fwd', L 'aft', chosen in an earlier round and not revisited here), and that stack has no
// mm to spare today. The code (clearance.js pinGroove/hornPinWasher, linkage.js wingSideMembers) is written and
// wired, gated on retention.hornPinWasher so it is inert until there is room: flip it to true once eye_gap_mm or the
// horn stacking itself is opened up (a layout call, not guessed here), and it draws and rates correctly (tested with
// hornPinWasher: true against params.layout.horns flipped to {R:'aft',L:'fwd'}, which only made the clash larger,
// so that is not the fix either).
//
// plates.bossed: unchanged from v24's three (crank shaft both ends, gear shaft 1 aft). Audit item 16 / F6 asks for
// three more (both 628/8 seats on gear shaft 3, the 619/5 seat): probed and dropped. Gear shaft 3's bosses clash
// with the shoulder rail and its forward bracket's sleeve (about -1.6 to -2.2 mm): the boss's own radius plus wall
// (plates.boss_wall_mm) reaches into where the rail already runs, found only once drawn to its full, correct length,
// not estimated. The motor's outboard 619/5 boss clashes with the motor can (-0.9 mm): plateJournal/bossBeyond build
// every boss as if it is seated inward of its plate, but this seat is the one case built outward (plateJournalX's
// `outward` flag, for the overhung motor), and the boss code does not carry that flag through, so it grows the wrong
// way. That is a bug in the shared boss code (src/design/clearance.js plateJournal), not a param here, and fixing it
// safely needs the plates.bossed rule reread for every other bossed seat too (none of which uses `outward`) before
// trusting a change there. Both are stated rather than forced; the minimum bearing FoS (3.41, from the audit) is not
// at stake either way.
export const RETENTION = {
  linkage: {
    retention: {
      washer8mm: 'washer-din6799-6',
      gearShaft3Circlip: 'circlip-din471-8',
      spacerMaterial: 'metal-al-6061-t6',
      hornPinWasher: false,
      // Found by probing: gear shaft 2's forward end, lengthened for the washer, comes within 0.36 mm of the
      // DS1-60 wheel overhung on gear shaft 3 (its 31 mm tip circle reaches toward the adjacent shaft) — short of
      // the 0.5 mm rule. Excluded rather than forced; its spacer (to the bushing's own, unextended inner edge) and
      // its aft washer are unaffected, so the shaft is still located one way.
      washer8mmForwardExclude: [2],
    },
  },
};

const step = (name, members, dir, dist_m, why) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
const X = (s) => [s, 0, 0];
// A DIN 6799 washer or DIN 471 circlip is a sprung E-clip, snapped on sideways at its groove, not threaded axially
// over the shaft's end (its own catalogue ID is the groove's diameter, narrower than the shaft it sits on, which a
// rigid ring could not pass over). Its assembly step approaches radially, in y, not along the shaft's axis.
const Y = (s) => [0, s, 0];

/**
 * Round 17's assembly-sequence patches for retention (round17.mjs sequence17 convention: `after` appends following
 * the named round-16 step, `replace` swaps it; both may target the same original name, applied in that order).
 * Only the gear-shaft parts are here (hornPinWasher is off; add its two `after: 'horn pin R'/'horn pin L'` steps,
 * each a single `horn pin washer <side>` member, back in if it is turned on).
 *  - An aft washer or the circlip sits outside the aft plate, already in place from the sequence's first step, so it
 *    is reachable right after its shaft's own rigid-group step.
 *  - A forward washer sits outside the forward plate, which does not arrive until later (round16's own
 *    "forward plate with its journals" step, well after every gear shaft's): fitting it right after the shaft's own
 *    step would have the forward plate's own journals, which still have to travel up to 80 mm to reach their seats,
 *    sweep straight through it. So forward washers wait for `after: 'forward plate with its journals'`. Gear
 *    shaft 3's forward spacer additionally needs the bonded DS1-60 (round16's "stage 1" step, after the forward
 *    plate) already there to butt against, so it waits one step more, `after: 'stage 1'`.
 *  - Spacer tubes that sit in a gap enclosed once the gear stack is in place must be threaded onto the shaft before
 *    it is fed into the plates: folded into the shaft's own rigid-group step via `replace`.
 */
export const RETENTION_SEQUENCE = [
  { replace: 'gear shaft 1 with its gears and keys', steps: [step('gear shaft 1 with its gears, keys and spacers', ['^gear shaft 1$', ' on gear shaft 1$', '^stage 3 wheel ', '^stage 4 pinion ', '^gear shaft 1 spacer '], X(1), 0.1, 'the spacer tubes threaded on with the gears before the shaft is fed into the plates (they sit in gaps that would otherwise be enclosed)')] },
  { after: 'gear shaft 1 with its gears and keys', steps: [step('gear shaft 1 washer aft', ['^gear shaft 1 washer aft$'], Y(1), 0.008, 'snapped on sideways at its groove, against the aft bushing\'s flange (the aft plate is already placed)')] },

  { replace: 'gear shaft 2 with its gears and keys', steps: [step('gear shaft 2 with its gears, keys and spacers', ['^gear shaft 2$', ' on gear shaft 2$', '^stage 2 wheel ', '^stage 3 pinion ', '^gear shaft 2 spacer '], X(1), 0.1, 'the spacer tubes threaded on with the gears before the shaft is fed into the plates')] },
  { after: 'gear shaft 2 with its gears and keys', steps: [step('gear shaft 2 washer aft', ['^gear shaft 2 washer aft$'], Y(1), 0.008, 'snapped on sideways at its groove, against the aft bushing\'s flange (the aft plate is already placed)')] },

  { replace: 'gear shaft 3 with its pinion and key', steps: [step('gear shaft 3 with its pinion, key and aft spacer', ['^gear shaft 3$', ' on gear shaft 3$', '^stage 2 pinion ', '^gear shaft 3 spacer aft$'], X(1), 0.1, 'the aft spacer threaded on with the pinion before the shaft is fed into the plates')] },
  { after: 'gear shaft 3 with its pinion and key', steps: [step('gear shaft 3 circlip aft', ['^gear shaft 3 circlip aft$'], Y(-1), 0.003, 'snapped on sideways at its groove, against the aft bearing\'s inner ring (the aft plate is already placed)')] },

  { after: 'forward plate with its journals', steps: [
    // gear shaft 2's forward washer is excluded (RETENTION.linkage.retention.washer8mmForwardExclude), so no step
    // for it: gear shaft 1's is the only forward washer this round.
    step('gear shaft 1 washer forward', ['^gear shaft 1 washer forward$'], Y(1), 0.008, 'snapped on sideways at its groove, against the forward bushing\'s flange, now that the forward plate has reached its seat'),
    // gear shaft 3's forward spacer slides onto the shaft's forward tip here too, fitting against the forward
    // bearing's outer face while the space ahead of it (where DS1-60 bonds on, in "stage 1", next) is still open
    step('gear shaft 3 spacer forward', ['^gear shaft 3 spacer forward$'], X(1), 0.004, 'onto the shaft\'s forward tip, against the forward bearing\'s outer face, before the bonded DS1-60 (open space ahead of it until then)'),
  ] },
];
