// The join reasons the clearance check exempts from its pair check (moved from clearance.js in round 17 so that
// interfaces.js can read them while clearance.js imports interfaces.js; clearance.js re-exports them unchanged).
export const MESH = 'gears in mesh';
/** Round 16: a key in its keyseat and keyway, which are cut to it and not drawn (exempt like a mesh). */
export const KEYWAY = 'key in its keyseat and keyway (cut to the key, not drawn)';
/** Round 16: a screw's shank through its hole in a part it clamps (the hole is cut to it, not drawn in every part). */
export const THROUGH = 'shank through its hole in a part it clamps (the hole is cut to it and not drawn)';
/** Round 16: a screw's shank in the tapped hole of the part it threads into (not drawn). */
export const THREAD = 'shank in its tapped hole (not drawn)';
/** Round 16: the spar's or a rod's hole in a rib (not drawn: the rib lies across the span, not across x). */
export const RIB_HOLE = 'through its hole in the rib (not drawn: the rib lies across the span, not across x)';
/** Round 16: a tube inside another's bore (not drawn: a tube along the span is drawn as its capsule across x). */
export const BORE = 'in the other tube\'s bore (not drawn: a tube along the span is drawn as its capsule across x)';
/** Round 17: a retaining washer or circlip in its groove, cut into the shaft or pin to its own published diameter
 * (drawn, unlike a key's keyseat, but still exactly touching there by construction: like KEYWAY, a step sampling the
 * slide at 0.1 mm resolution can read that exact touch as a hair of overlap). */
export const GROOVE = 'in its groove, cut to its own published diameter';
/** The joint classes exempt from the pair check, with their reasons. */
export const EXEMPT_JOINTS = [MESH, KEYWAY, THROUGH, THREAD, RIB_HOLE, BORE, GROOVE];
