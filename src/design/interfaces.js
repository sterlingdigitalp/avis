// src/design/interfaces.js
//
// SPEC "Design freeze" -> "Function completeness" (2026-11-03): every pivot, pin, shaft, bearing, sliding or
// rotating fit, and bonded, tied, keyed or fastened joint must carry (1) what carries its load and that part's
// rating, (2) what holds it axially and its rating, (3) the rule that rates each. A joint with no rule fails,
// unless the "RC practice for joints no one can rate" ruling (Rulings after physics round 8) covers it. That
// ruling's scope is narrow: a shaft-to-hub fixing on a gear, pulley or coupling ONLY (set screw, clamp hub,
// D-flat or bond). It never covers a structural member, a bearing, gear teeth, a seat, a lug or a fastener in
// tension or shear -- so a member whose own `check` happens to read 'rc practice' still fails here if the joint
// it fixes is not a gear/pulley/coupling. The worked example is the Design freeze section's own note on v24: the
// crank web's axial LOCTITE 638 bond is "not counted until a rated fixing replaces it", precisely because a crank
// web is not a gear, pulley or coupling.
//
// This module is read-only analysis: it reads `asm.joins` (clearance.js `fullAssembly`) and rates each physical
// joint against the design's own M7 members (`d.structure.members`) and bearing joints (`d.structure.joints` /
// `joints_unchecked`). It never edits clearance.js, linkage.js, airframe.js or the design file, and it never
// invents a member name that is not already in the design -- a joint with no rated member in the design fails,
// honestly, in the ledger.
//
// Decisions on the exemption-only reasons (documented per the task brief, since clearance.js uses them only to
// skip the *clearance* check, not to say whether a joint is load-rated):
//   - MESH ('gears in mesh'), KEYWAY, THROUGH and THREAD never enter this ledger as their own rows. Each
//     accompanies a joint that is *also* recorded under a different, descriptive `why` text at the same or a
//     related part pair (the screw's 'fastened' row next to its THROUGH/THREAD solids; the key's own crushing
//     captured by d.structure.members' "key, ..." entries next to its KEYWAY solids). Gear-tooth contact (MESH)
//     is rated elsewhere in M7 by Hertz/Lewis against d.structure.gear_stages, which is outside the set this
//     ledger enumerates (SPEC's Function completeness text: "every pivot, pin, shaft, bearing, sliding fit and
//     bonded or tied joint" -- gear mesh is not in that list), so it stays a pure exemption here too.
//   - RIB_HOLE and BORE are different: the task brief says RIB_HOLE stands for a rib bonded or threaded on a
//     spar or rod, and BORE for a tube inside another's bore (src/design/wing-solids.js) -- i.e. for these two,
//     the "exempt from the clearance check" pair *is* the real physical joint, with no separate descriptive join
//     recording it elsewhere. So both enter the ledger as real rows (kind 'bonded' / 'sliding fit'), and in v24
//     both come up unrated: no rib member exists in d.structure.members (audit-v24.md interface #20, "no rib
//     check"), and the twist torque rod's fit in the outer spar's bore has "no bearing, no declared plain pair"
//     (audit-v24.md interface #18) outside the one BORE pair that is the rocker's spar-socket seat, which *is*
//     rated (seat + bond members) and passes.
//
// `rating` is always implemented as a plain function `(d, a, b) => [names]`; this module never uses the "string
// pattern" alternative the brief allows, for determinism (every row's names are either resolved or empty, never
// ambiguous glob matches).

import { MESH, KEYWAY, THROUGH, THREAD, RIB_HOLE, BORE } from './clearance.js';

// ---- name lookups against the design file -------------------------------------------------------------------

const findMember = (d, name) => (d.structure.members ?? []).find((m) => m.name === name) ?? null;
const findJoint = (d, name) => (d.structure.joints ?? []).find((j) => j.name === name) ?? null;
const findUnchecked = (d, name) => (d.structure.joints_unchecked ?? []).find((j) => j.name === name) ?? null;

/**
 * Whether a named member/joint exists and is rated.
 * - A member in d.structure.members is rated unless its own `check` is 'unrated' (SPEC: "the named member's
 *   check is 'unrated'" fails the row).
 * - A bearing/bushing joint in d.structure.joints is rated by construction: round 4's ruling ("Bearings are
 *   linkage... a bearing or bushing without a published static rating is ineligible for a loaded joint") means
 *   only an eligible, published-rating bearing is listed there at all.
 * - A name only in joints_unchecked (the design's own "not credited" bucket, e.g. the motor's own bearings) is
 *   not rated.
 * - A name found nowhere does not exist.
 */
function nameStatus(d, name) {
  const m = findMember(d, name);
  if (m) return { exists: true, rated: m.check !== 'unrated', where: 'member', check: m.check };
  const j = findJoint(d, name);
  if (j) return { exists: true, rated: true, where: 'joint', check: null };
  const ju = findUnchecked(d, name);
  if (ju) return { exists: true, rated: false, where: 'joint_unchecked', check: null };
  return { exists: false, rated: false, where: null, check: null };
}

function evalSide(d, a, b, spec) {
  const rating = (typeof spec.rating === 'function' ? spec.rating(d, a, b) : []) ?? [];
  const resolved = rating.map((n) => ({ name: n, ...nameStatus(d, n) }));
  const rated = rating.length > 0 && resolved.every((r) => r.exists && r.rated);
  return { by: spec.by, rating, rated, rule: spec.rule, resolved };
}

// ---- rule-basis text (the SPEC/standard each kind of interface is rated against) ------------------------------

const BEARING_RULE = "ISO 76 static C0/P0 at FoS >= 1.5 (ball bearings), or the maker's published static surface pressure at FoS 1.5 (plain bushings); every bearing/bushing is linkage (SPEC Rulings after physics round 4, 'Bearings are linkage')";
const KEY_RULE = "key/keyway crushing at the material's rated (or half, if unrated) yield (SPEC Rulings after physics round 7, 'Crushing on a keyway or seat')";
const BOLT_RULE = "ISO 898-1 (or ISO 3506) proof load, or the fastener maker's rated load (SPEC Rulings after physics round 7, 'Every part reaches the airframe through a rated path')";
const SEAT_RULE = "pin-in-hole / lug bearing strength at the source's edge distance, full-or-half rule, with the EN 1999-1-1 / EN 1993-1-8 bolt-bearing fallback below the source's edge distance (SPEC Rulings after physics round 6 and round 8)";
const BOND_RULE = "the adhesive maker's published lap-shear strength on that substrate (SPEC Rulings after physics round 7, 'Every part reaches the airframe through a rated path')";
const TIE_RULE = "the cable tie maker's published loop/tensile strength (SPEC round 15 carriers; src/design/airframe.js tieFor)";
const BEAM_RULE = "the carrier beam's own bending check at the root, both ways at full value (SPEC round 15 carriers)";
const RC_RULE = "RC practice for joints no one can rate (SPEC Rulings after physics round 8, operator's decision 2026-11-02): scope is a gear, pulley or coupling held to its shaft by set screw, clamp hub, D-flat or bond, only; any published figure for the fixing (even a reference one) still binds at full value";
const NO_MEMBER_RULE = "SPEC Design freeze, Function completeness: no M7 member or bearing joint in this design rates this interface";
const NO_AXIAL_MEMBER_RULE = "SPEC Design freeze, Function completeness: what holds a joint axially needs its own named, rated member; none is named for this joint in this design";

// ---- v24's own free-text naming conventions in asm.joins' `a`/`b` ids -----------------------------------------

const esc = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
const exact = (text) => new RegExp(`^${esc(text)}$`);

/** SPEC round 8's RC-practice scope: a gear, pulley or coupling only -- never a structural member (e.g. a crank
 * web) or a sensor/encoder actuator, even when it is also a shaft-to-hub, set-screw-style fixing. */
const isGearPulleyCoupling = (a, b) => /\b(gear|pulley|coupling)\b/i.test(a) || /\b(gear|pulley|coupling)\b/i.test(b);

/** The bearing/bushing joint (d.structure.joints) that a drawn journal part's id names, by v24's own naming:
 * "<joint> journal aft|forward (<bom id>)" for plate/panel journals, "shoulder bushing R|L aft|forward" for the
 * shoulder pivots, "coupler R|L crank|horn bushing" for the coupler eyes. */
function bearingJointName(a) {
  let m;
  if ((m = /^(.+?) journal (?:aft|forward) \(/.exec(a))) return m[1];
  if ((m = /^shoulder bushing ([RL]) (aft|forward)$/.exec(a))) return `shoulder pivot ${m[1] === 'R' ? 'right' : 'left'}`;
  if ((m = /^coupler ([RL]) (crank|horn) bushing$/.exec(a))) return `coupler eye ${m[1] === 'R' ? 'right' : 'left'}, ${m[2] === 'crank' ? 'crank pin' : 'rocker horn'}`;
  return null;
}

/** Fixed, exact-match dictionary from a fastener's own part-id prefix (as clearance.js's screw/nut ids read) to
 * the M7 member-name prefix that rates it (the two differ in several places: "rail lock right" -> "shoulder rail
 * right, torsion lock"; "motor bracket" -> "motor mount"; "boom clamp aft/forward" -> "tail boom aft/forward
 * clamp"). Every other family (forward tray, aft tray, nose tray riser, nose tray) already shares its own tag. */
const FASTENER_PREFIX = {
  'rail lock right': 'shoulder rail right, torsion lock',
  'rail lock left': 'shoulder rail left, torsion lock',
  'boom clamp aft': 'tail boom aft clamp',
  'boom clamp forward': 'tail boom forward clamp',
  'forward tray': 'forward tray',
  'aft tray': 'aft tray',
  'nose tray riser': 'nose tray riser',
  'nose tray': 'nose tray',
  'motor bracket': 'motor mount',
  readhead: 'crank encoder readhead',
};
const stripFastenerSuffix = (s) => s.replace(/\/shank$/, '').replace(/ (?:screw|nut) \d+$/, '');
function fastenerPrefixFrom(s) {
  const base = stripFastenerSuffix(s);
  if (FASTENER_PREFIX[base]) return FASTENER_PREFIX[base];
  if (base === 'readhead' || /^readhead (?:spacer|nut) \d+$/.test(base)) return 'crank encoder readhead';
  return null;
}
/** The bolts (load) and seat (axial) members for a fastener family, found by exact name and by the "<prefix>
 * screws in ..." name the design gives the seat (its suffix after "in" varies: "in the plate", "in the riser's
 * tab", "in the forward tray's base", ...). Parts with no recognised prefix (e.g. the flange doubler) resolve to
 * no names at all, which the ledger then honestly fails rather than guessing a member. */
function fastenerMembers(d, a, b) {
  const prefix = fastenerPrefixFrom(a) ?? fastenerPrefixFrom(b);
  if (!prefix) return { load: [], axial: [] };
  const load = findMember(d, `${prefix} screws`) ? [`${prefix} screws`] : [];
  const seat = (d.structure.members ?? []).find((m) => m.name.startsWith(`${prefix} screws in`));
  return { load, axial: seat ? [seat.name] : [] };
}
const FASTENED_LOAD = { by: "the screw(s), in shear and tension", rating: (d, a, b) => fastenerMembers(d, a, b).load, rule: BOLT_RULE };
const FASTENED_AXIAL = { by: "the nut, or the screw's thread engagement", rating: (d, a, b) => fastenerMembers(d, a, b).axial, rule: SEAT_RULE };

/** A packed component's role (as src/design/airframe.js's LUMP_OF / params.layout.mounts key it) from either its
 * "packed <role>" part id or its "packed <role> ties" tie-part id. */
const roleFromPackedId = (s) => { const m = /^packed (.+?)(?: ties)?$/.exec(s); return m ? m[1] : null; };
const packedRole = (a, b) => roleFromPackedId(a) ?? roleFromPackedId(b);
/** The tie member for a packed role, named `${carrier}: ${role} ties` by airframe.js's af_carriers (the carrier,
 * 'nose tray' or 'aft tray', comes from the design's own params.layout.mounts, not guessed). */
const tieMemberFor = (d, role) => { const mt = d.params.layout?.mounts?.[role]; return mt ? [`${mt.carrier}: ${role} ties`] : []; };
const tieSpec = (roleFn) => ({ by: 'the cable tie round it', rating: (d, a, b) => { const role = roleFn(a, b); return role ? tieMemberFor(d, role) : []; }, rule: TIE_RULE });
const constSpec = (names, rule, by) => ({ by, rating: () => names, rule });

/** A gear's own shaft-fixing: role (wheel/pinion) and gear id from "stage N wheel|pinion <gearId>" (a), the
 * shaft/wheel-seat part from "<shaftPart>/shaft" or "<shaftPart>/wheel seat" (b). */
function gearShaftInfo(a, b) {
  const gm = /^stage \d+ (wheel|pinion) (\S+)$/.exec(a);
  if (!gm) return null;
  return { role: gm[1], gearId: gm[2], shaftPart: b.split('/')[0] };
}
/** The RC-practice "<gearId> on <shaft>, fixing" member (set screw / bond, KHK DS1-series gears in v24). */
function gearFixingNames(gi) {
  return [gi.shaftPart === 'motor' ? `${gi.gearId} on the motor shaft, fixing` : `${gi.gearId} on ${gi.shaftPart}, fixing`];
}
/** The two "key, ..." members (shaft side, then its ", gear side" pair) that crush-check a keyed gear's key. */
function gearKeyNames(gi) {
  const base = gi.shaftPart === 'crank' ? `key, crank ${gi.role} ${gi.gearId}` : `key, ${gi.role} ${gi.gearId} on ${gi.shaftPart}`;
  return [base, `${base}, gear side`];
}

/** right/left from a token "R"/"L" standing alone in an id such as "inner spar R" or "rocker R/spar socket". */
function sparSocketSide(a, b) {
  const m = /(?:rocker|inner spar) ([RL])\b/.exec(a) ?? /(?:rocker|inner spar) ([RL])\b/.exec(b);
  return m ? (m[1] === 'R' ? 'right' : 'left') : null;
}

const bearingLoadSpec = { by: 'the bearing/bushing joint (d.structure.joints)', rating: (d, a, b) => { const n = bearingJointName(a); return n ? [n] : []; }, rule: BEARING_RULE };
const noAxialSpec = { by: 'nothing named retains it axially', rating: () => [], rule: NO_AXIAL_MEMBER_RULE };
const noMemberBoth = (by = 'nothing named in this design') => ({ load: { by, rating: () => [], rule: NO_MEMBER_RULE }, axial: { by, rating: () => [], rule: NO_AXIAL_MEMBER_RULE } });

// ---- the ordered rule table --------------------------------------------------------------------------------

/**
 * INTERFACE_RULES: an ordered table. Each row matches a join by its `why` text (regex) and, where one `why` text
 * covers more than one real fixing (gear-on-shaft; the shared BORE exemption), by a part-id regex too -- the
 * first row whose `why` (and, if given, `part`) matches wins. `load`/`axial` are `{ by, rating(d,a,b)->[names],
 * rule }`; `axial: null` only where a join has no retention concept of its own (none in v24 -- kept for designs
 * that may add one). `rc_practice: true` marks a row the RC-practice ruling (round 8) is being asked to cover;
 * interfaceLedger then also requires the resolved member's own check to read exactly 'rc practice' and the joint
 * to actually be a gear/pulley/coupling shaft-hub fixing, never just "not unrated".
 */
export const INTERFACE_RULES = [
  // -- the crank web on the crank shaft: keyed (load) + LOCTITE (axial, unrated, out of RC-practice's scope) ----
  {
    why: /^keyed on the shaft's aft stub, held there with LOCTITE \d+$/,
    part: null,
    kind: 'keyed',
    load: { by: 'the key crushing between the web and the shaft stub', rating: (d, a) => [`key, ${a}`, `key, ${a}, web side`], rule: KEY_RULE },
    axial: { by: "the LOCTITE bond holding the web on the shaft's aft stub", rating: () => [], rule: NO_AXIAL_MEMBER_RULE },
    note: "SPEC Design freeze, 'The crank web's axial bond': LOCTITE on titanium has no published rating, and a crank web is not a gear, pulley or coupling, so round 8's RC-practice ruling does not cover it either.",
  },
  // -- a sensor/encoder actuator on a shaft by set screws: a shaft-hub fixing, but not in RC-practice's scope ----
  {
    why: exact("on the crank shaft by its three set screws (maker's fixing)"),
    part: null,
    kind: 'fastened',
    ...noMemberBoth(),
    note: 'a crank encoder actuator is a sensor, not a gear, pulley or coupling, so round 8\'s RC-practice ruling does not cover its set-screw fixing, and no other member rates it in this design.',
  },
  // -- the readhead screw/spacer/nut stack on the riser ------------------------------------------------------
  ...[
    'through its mounting hole, the head on its back face',
    'through the spacer',
    "through its hole in the riser's web",
    'threaded',
    "on the readhead's front face",
    "on the riser's web",
    "on the riser's forward face",
  ].map((why) => ({ why: exact(why), part: /readhead/, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' })),
  // -- the motor bracket: screwed to the motor's back-face threads, bonded (sleeve) to the rail ---------------
  { why: exact('screwed into its back-face threads'), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact('its sleeve bonded on the rail'), part: null, kind: 'bonded', load: constSpec(['motor bracket sleeve bond'], BOND_RULE, 'the sleeve bond'), axial: constSpec(['motor bracket sleeve bond'], BOND_RULE, 'the same bond'), note: '' },
  // -- a gear on its shaft: either RC-practice (set screw/bond, KHK DS1-series) or a key (no axial retention) --
  {
    why: /^gear on its shaft \(bore on the shaft or seat\)$/,
    part: /gear-khk-ds1-/,
    kind: 'fastened',
    rc_practice: true,
    load: { by: 'the set-screw/bond fixing between the gear and its shaft', rating: (d, a, b) => { const gi = gearShaftInfo(a, b); return gi ? gearFixingNames(gi) : []; }, rule: RC_RULE },
    axial: { by: 'the same set-screw/bond fixing', rating: (d, a, b) => { const gi = gearShaftInfo(a, b); return gi ? gearFixingNames(gi) : []; }, rule: RC_RULE },
    note: 'KHK DS1-series spur gears publish no keyway; audit-v24.md interface #12: "within the published figure, disclosed, sound".',
  },
  {
    why: /^gear on its shaft \(bore on the shaft or seat\)$/,
    part: null,
    kind: 'keyed',
    load: { by: 'the key carrying torque between the gear and its shaft', rating: (d, a, b) => { const gi = gearShaftInfo(a, b); return gi ? gearKeyNames(gi) : []; }, rule: KEY_RULE },
    axial: { by: 'nothing named retains the gear axially on its shaft', rating: () => [], rule: NO_AXIAL_MEMBER_RULE },
    note: 'audit-v24.md interface #9: keys carry torque only; the design names no clip, collar or shoulder retaining these gears axially.',
  },
  // -- gear mesh (MESH) is rated elsewhere in M7 (Hertz/Lewis against d.structure.gear_stages); not this ledger --
  // (no row: MESH is a pure exemption, see the module banner)

  // -- plain bearings/bushings: journal on its shaft/pin, and pressed in its seat/housing/eye -------------------
  { why: exact('journal on its shaft'), part: null, kind: 'shaft in bearing', load: bearingLoadSpec, axial: noAxialSpec, note: '' },
  { why: exact('journal on the shoulder pin'), part: null, kind: 'shaft in bearing', load: bearingLoadSpec, axial: noAxialSpec, note: '' },
  { why: exact('journal on its pin'), part: null, kind: 'shaft in bearing', load: bearingLoadSpec, axial: noAxialSpec, note: '' },
  { why: exact('pressed in its plate seat'), part: null, kind: 'press fit', load: bearingLoadSpec, axial: noAxialSpec, note: '' },
  { why: exact('pressed in the housing'), part: null, kind: 'press fit', load: bearingLoadSpec, axial: noAxialSpec, note: '' },
  { why: exact('pressed in the eye'), part: null, kind: 'press fit', load: bearingLoadSpec, axial: noAxialSpec, note: '' },

  // -- ties: every mounted part's loop, the formers, the pitot probe and the tail surfaces -------------------
  { why: exact('round the part'), part: null, kind: 'tied', load: tieSpec(packedRole), axial: tieSpec(packedRole), note: '' },
  { why: exact('through its slots'), part: null, kind: 'tied', load: tieSpec(packedRole), axial: tieSpec(packedRole), note: '' },
  { why: exact("its tab against the tray's flange, tied"), part: null, kind: 'tied', load: constSpec(['pod formers ties'], TIE_RULE, 'the tie'), axial: constSpec(['pod formers ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact('round the tab'), part: null, kind: 'tied', load: constSpec(['pod formers ties'], TIE_RULE, 'the tie'), axial: constSpec(['pod formers ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact("against the right flange's outer face, tied"), part: null, kind: 'tied', load: constSpec(['pitot-static probe ties'], TIE_RULE, 'the tie'), axial: constSpec(['pitot-static probe ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact('round the probe'), part: null, kind: 'tied', load: constSpec(['pitot-static probe ties'], TIE_RULE, 'the tie'), axial: constSpec(['pitot-static probe ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact('round the flange, through a slot at its root'), part: null, kind: 'tied', load: constSpec(['pitot-static probe ties'], TIE_RULE, 'the tie'), axial: constSpec(['pitot-static probe ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact('against the boom, tied'), part: null, kind: 'tied', load: constSpec(['tail surfaces ties'], TIE_RULE, 'the tie'), axial: constSpec(['tail surfaces ties'], TIE_RULE, 'the same tie'), note: '' },
  { why: exact('round the boom'), part: null, kind: 'tied', load: constSpec(['tail surfaces ties'], TIE_RULE, 'the tie'), axial: constSpec(['tail surfaces ties'], TIE_RULE, 'the same tie'), note: '' },
  // the spar rod's own butt against the boom (as opposed to the root rib's tie, rated just above) is unrated
  { why: exact("its root against the boom, through the root rib"), part: null, kind: 'resting', ...noMemberBoth(), note: "audit-v24.md interface #21: only the root rib's own tie is rated; the rod's butt against the boom is not." },

  // -- resting on a tray: the tray beam carries it, its tie retains it ---------------------------------------
  { why: exact('resting on the aft tray'), part: null, kind: 'resting', load: constSpec(['aft tray at the root'], BEAM_RULE, 'the tray beam'), axial: tieSpec(packedRole), note: '' },
  { why: exact('resting on the nose tray'), part: null, kind: 'resting', load: constSpec(['nose tray at the root'], BEAM_RULE, 'the tray beam'), axial: tieSpec(packedRole), note: '' },

  // -- bonded flanges/sleeves/elbows -------------------------------------------------------------------------
  { why: exact('flange bonded to the plate'), part: null, kind: 'bonded', load: { by: "the panel's flange bond", rating: (d, a) => [`${a} flange bond`], rule: BOND_RULE }, axial: { by: 'the same bond', rating: (d, a) => [`${a} flange bond`], rule: BOND_RULE }, note: '' },
  {
    why: exact('bonded on the rail'),
    part: null,
    kind: 'bonded',
    load: { by: "the bracket's sleeve bond on the rail", rating: (d, a) => { const m = /^shoulder bracket ([RL]) (aft|forward)$/.exec(a); return m ? [`shoulder bracket ${m[1] === 'R' ? 'right' : 'left'} ${m[2]}, sleeve bond`] : []; }, rule: BOND_RULE },
    axial: { by: 'the same bond', rating: (d, a) => { const m = /^shoulder bracket ([RL]) (aft|forward)$/.exec(a); return m ? [`shoulder bracket ${m[1] === 'R' ? 'right' : 'left'} ${m[2]}, sleeve bond`] : []; }, rule: BOND_RULE },
    note: '',
  },
  { why: exact('bonded on the boom'), part: null, kind: 'bonded', load: constSpec(['tail boom clamp bonds'], BOND_RULE, 'the clamp bond'), axial: constSpec(['tail boom clamp bonds'], BOND_RULE, 'the same bond'), note: '' },
  {
    why: exact(BORE),
    part: /spar socket/,
    kind: 'bonded',
    load: { by: 'the spar root socket seat (radial crush)', rating: (d, a, b) => { const s = sparSocketSide(a, b); return s ? [`spar root socket ${s}`] : []; }, rule: SEAT_RULE },
    axial: { by: 'the spar root bond (axial force)', rating: (d, a, b) => { const s = sparSocketSide(a, b); return s ? [`spar root bond ${s}`] : []; }, rule: BOND_RULE },
    note: "the design rates root_x only; torsion through this joint is a separate finding outside this ledger's load/axial fields (audit-v24.md interface #14).",
  },
  {
    why: exact('bonded in the spar socket'),
    part: null,
    kind: 'bonded',
    load: { by: 'the spar root socket seat (radial crush)', rating: (d, a, b) => { const s = sparSocketSide(a, b); return s ? [`spar root socket ${s}`] : []; }, rule: SEAT_RULE },
    axial: { by: 'the spar root bond (axial force)', rating: (d, a, b) => { const s = sparSocketSide(a, b); return s ? [`spar root bond ${s}`] : []; }, rule: BOND_RULE },
    note: '',
  },
  // the torque rod in the outer spar's own bore (the other BORE pair): no bearing, clip or declared plain pair
  { why: exact(BORE), part: null, kind: 'sliding fit', ...noMemberBoth(), note: 'audit-v24.md interface #18: "no bearing, no declared plain pair" for the twist torque rod in the outer spar\'s bore.' },
  { why: exact("butted to the inner spar at the elbow, bonded"), part: null, kind: 'bonded', ...noMemberBoth(), note: 'audit-v24.md interface #15: the elbow butt has no rated bond member.' },
  { why: exact('on the spar at the elbow'), part: null, kind: 'bonded', ...noMemberBoth(), note: "audit-v24.md interface #16: the twist servo's case against the spar is not even touching, and no frame, screw or tie is named." },
  { why: exact('on the spar'), part: null, kind: 'bonded', ...noMemberBoth(), note: "audit-v24.md interface #20: \"no rib check\" -- no rib member exists in d.structure.members." },
  { why: exact(RIB_HOLE), part: null, kind: 'bonded', ...noMemberBoth('nothing named in this design'), note: 'the task brief: RIB_HOLE stands for a rib bonded or threaded on a spar or rod; no rib member rates it (audit-v24.md interface #20).' },

  // -- seats and their lock screws ---------------------------------------------------------------------------
  {
    why: exact('seated in the plate'),
    part: null,
    kind: 'bearing seat',
    load: { by: 'the plate seat (lug)', rating: (d, a, b) => { const sm = /^shoulder rail (right|left)$/.exec(a); const pm = /^gearbox plate (aft|forward)$/.exec(b); return sm && pm ? [`plate seat shoulder rail ${sm[1]} ${pm[1]}`] : []; }, rule: SEAT_RULE },
    axial: { by: 'the torsion-lock screws', rating: (d, a) => { const sm = /^shoulder rail (right|left)$/.exec(a); return sm ? [`shoulder rail ${sm[1]}, torsion lock screws`, `shoulder rail ${sm[1]}, torsion lock screws in the plate`] : []; }, rule: BOLT_RULE },
    note: '',
  },

  // -- the remaining fastener families: rail locks, boom clamps, trays, the doubler --------------------------
  { why: exact('fastened (its head or its nut on the face)'), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact('threaded on the screw'), part: null, kind: 'threaded', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact('on its face'), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact('screwed into its threads'), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact('screwed to the plate'), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact("its root tab screwed to the plate"), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact("its foot screwed to the tray's base"), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  { why: exact("screwed to the riser's tab"), part: null, kind: 'fastened', load: FASTENED_LOAD, axial: FASTENED_AXIAL, note: '' },
  // the flange doubler: not an M7 member, so it correctly resolves to no rated name on either side
  { why: exact("on the plate's outer face, under the tray's screws"), part: null, kind: 'resting', ...noMemberBoth(), note: 'the forward tray flange doubler is not an M7 member in this design.' },
  { why: exact("under the tray's root tab"), part: null, kind: 'resting', ...noMemberBoth(), note: 'the forward tray flange doubler is not an M7 member in this design.' },
];

const PURE_EXEMPT = new Set([MESH, KEYWAY, THROUGH, THREAD]);

function matchRule(a, b, why) {
  for (const r of INTERFACE_RULES) {
    if (!r.why.test(why)) continue;
    if (r.part && !(r.part.test(a) || r.part.test(b))) continue;
    return r;
  }
  return null;
}

function buildRow(d, a, b, why, rule) {
  if (!rule) {
    return { a, b, why, kind: null, load: null, axial: null, rc_practice: false, ok: false, why_failed: "no INTERFACE_RULES row matches this join's reason" };
  }
  const load = evalSide(d, a, b, rule.load);
  const axial = rule.axial ? evalSide(d, a, b, rule.axial) : null;
  const scopeOk = !rule.rc_practice || isGearPulleyCoupling(a, b);
  const rcCheckOk = (side) => !rule.rc_practice || (side.rating.length > 0 && side.resolved.every((r) => r.check === 'rc practice'));

  let ok = load.rated && scopeOk && rcCheckOk(load) && (!axial || (axial.rated && rcCheckOk(axial)));
  let why_failed = null;
  if (!ok) {
    if (!load.rated) why_failed = `load not rated (${rule.load.by}): ${load.rating.length ? load.rating.join(', ') + ' missing or unrated' : 'no rated member named'}`;
    else if (rule.rc_practice && !scopeOk) why_failed = "RC-practice ruling (SPEC round 8) covers only a gear, pulley or coupling shaft-hub fixing; this joint is not one";
    else if (rule.rc_practice && !rcCheckOk(load)) why_failed = `${rule.load.by}: a named member exists but its check is not 'rc practice'`;
    else if (axial && !axial.rated) why_failed = `axial retention not rated (${rule.axial.by}): ${axial.rating.length ? axial.rating.join(', ') + ' missing or unrated' : 'no rated member named'}`;
    else if (axial && rule.rc_practice && !rcCheckOk(axial)) why_failed = `${rule.axial.by}: a named member exists but its check is not 'rc practice'`;
  }
  return { a, b, why, kind: rule.kind, load, axial, rc_practice: !!rule.rc_practice, ok, why_failed, note: rule.note ?? '' };
}

/**
 * The interface ledger for a built design: one row per physical joint in `asm.joins` (clearance.js's
 * `fullAssembly`), excluding the pure clearance-only exemptions (MESH, KEYWAY, THROUGH, THREAD -- see the module
 * banner). Each row is rated against `d.structure.members` / `d.structure.joints` / `joints_unchecked` only; no
 * member name is ever invented. `seatWidths(asm)` is folded into `passed`.
 */
export function interfaceLedger(asm, d) {
  const rows = [];
  for (const [a, b, why] of asm.joins) {
    if (PURE_EXEMPT.has(why)) continue;
    rows.push(buildRow(d, a, b, why, matchRule(a, b, why)));
  }
  const failed = rows.filter((r) => !r.ok);
  const rc_practice = rows.filter((r) => r.rc_practice);
  const seats = seatWidths(asm);
  return {
    rule: "SPEC Design freeze, Function completeness (2026-11-03): every pivot, pin, shaft, bearing, sliding or rotating fit, and bonded, tied, keyed or fastened joint carries what carries its load, what holds it axially, and the rule that rates each; a joint with no rule fails unless round 8's RC-practice ruling covers it. src/design/interfaces.js.",
    rows,
    failed,
    rc_practice,
    seats,
    passed: failed.length === 0 && seats.length === 0,
  };
}

// ---- seat widths --------------------------------------------------------------------------------------------

/** Join reasons naming a journal (bushing/bearing) part pressed into its seat material. */
const JOURNAL_SEAT_WHY = new Set(['pressed in its plate seat', 'pressed in the housing', 'pressed in the eye']);

/**
 * "Seats are as wide as the part they carry" (SPEC Function completeness). For every journal part pressed into a
 * seat, the axial (body-x) length of the seat part's own solids (its plate thickness, any boss, its housing
 * ring, its eye) that overlaps the journal's own drawn 'body' solid, against that body solid's own length. Pure
 * geometry from the drawn solids in `asm` (no catalogue lookup) -- the same two numbers the v24 audit found (SKF
 * 628/8 seated over 1.2 and 2.0 of its 5 mm width; SKF 619/5 over 1.2 of its 4 mm width) fall out directly,
 * because those two bearings are drawn at their full published width. Three supports in v24 (the crank shaft's
 * two seats and gear shaft 1's aft seat) are drawn *already shortened to fit* their available seat material, so
 * they come back at ratio 1.0 here even though their true catalogue body is 1 mm longer than what is drawn --
 * geometry alone cannot see a part that was quietly shrunk to match its seat; see this task's report for that
 * caveat. Only rows with ratio < 1 are returned (as the task brief asks: "rows with ratio < 1 are listed").
 */
/** A static (non-pose-dependent) solid's ring centre, or null for a poly sheet or a moving (pose-dependent) one. */
const ringCentre = (s) => (s.shape?.kind === 'ring' ? s.shape.c : null);

export function seatWidths(asm) {
  const byId = new Map(asm.parts.map((p) => [p.id, p]));
  const rows = [];
  for (const [a, b, why] of asm.joins) {
    if (!JOURNAL_SEAT_WHY.has(why)) continue;
    const journal = byId.get(a), seat = byId.get(b);
    const body = journal?.solids.find((s) => s.name === 'body');
    if (!body || !seat) continue;
    const [j0, j1] = body.x;
    const bodyLen = j1 - j0;
    if (!(bodyLen > 0)) continue;
    // A plate (or other seat part) commonly hosts more than one shaft's own boss at the same x: a boss is only
    // this journal's seat material if its ring is centred on the same axis (within 1 um). A plain sheet (poly,
    // no centre) or a pose-dependent solid (the coupler eyes, moving with the crank) is always counted.
    const jc = ringCentre(body);
    const spans = seat.solids
      .map((s) => {
        const sc = ringCentre(s);
        if (jc && sc && (Math.abs(sc[0] - jc[0]) > 1e-6 || Math.abs(sc[1] - jc[1]) > 1e-6)) return null;
        return [Math.max(s.x[0], j0), Math.min(s.x[1], j1)];
      })
      .filter((sp) => sp && sp[1] > sp[0])
      .sort((p, q) => p[0] - q[0]);
    let overlap = 0, cur = null;
    for (const span of spans) {
      if (!cur) cur = span;
      else if (span[0] <= cur[1] + 1e-12) cur[1] = Math.max(cur[1], span[1]);
      else { overlap += cur[1] - cur[0]; cur = span; }
    }
    if (cur) overlap += cur[1] - cur[0];
    const ratio = overlap / bodyLen;
    if (ratio < 1 - 1e-9) {
      rows.push({ journal: a, seat: b, why, body_mm: +(bodyLen * 1000).toFixed(3), overlap_mm: +(overlap * 1000).toFixed(3), ratio: +ratio.toFixed(3) });
    }
  }
  return rows.sort((p, q) => p.ratio - q.ratio);
}
