// The three.js bird in node, for the GLB export and the pose bake: same model.js and bird.js as the page.
import { loadModel } from '../../sims/avis-shared/node-load.js';
import { makeMaterials, livery as liveryOf, defaultLivery } from '../../sims/avis-shared/materials.js';
import { buildBird } from '../../sims/avis-shared/bird.js';
import { strokeAngle } from '../../src/physics/kinematics.js';

/** Phase at which the right wing is level (phi = 0) at amplitude amp: the GLB's rest pose, wings flat. */
export function levelPhase(model, amp) {
  let lo = 0, hi = Math.PI; // phi falls from the top (phase 0) to the bottom near pi
  const f = (p) => strokeAngle(p, amp, model.law).phi;
  if (f(lo) * f(hi) > 0) return 0;
  for (let i = 0; i < 60; i++) {
    const m = 0.5 * (lo + hi);
    if (f(lo) * f(m) <= 0) hi = m;
    else lo = m;
  }
  return 0.5 * (lo + hi);
}

/** The bird in node; `livery` (a LIVERIES key, or null for the clear film; default the white the BOM records, else
 * the clear film) paints the covering's vertex colours. */
export function nodeBird(name = 'larus-gl-2', { livery } = {}) {
  const { model, design, sha256, path } = loadModel(name);
  const l = liveryOf(livery === undefined ? defaultLivery(model.parts, design.bom) : livery);
  const mats = makeMaterials({ textures: false, livery: l });
  const bird = buildBird(model, mats, { livery: l });
  const amp = model.law.ampNominal ?? design.cruise.amp;
  const rest = { phase: levelPhase(model, amp), amp, symmetric: true, twistL: 0, twistR: 0, tailL: 0, tailR: 0 };
  return { bird, model, design, sha256, path, rest, livery: l };
}

/** Every node whose matrix the pose drives: update()'s posed list plus servo horns (matrixAutoUpdate). */
export function posedNodes(bird) {
  const set = new Set(bird.posed);
  for (const sv of Object.values(bird.servos)) set.add(sv.horn);
  return [...set];
}
