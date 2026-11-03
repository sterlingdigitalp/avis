// What the film draws of one physics solid. Since v24 every part is drawn from the physics' own solids, one mesh per
// part and BOM line (sims/avis-shared/drive-solids.js), so a test of one solid (a coupler's crank eye, the crank's
// pin) reads the vertices of the meshes that draw it which lie inside it: the technique physics-solids.test.mjs
// "filled" uses. Nothing here reads the film's own numbers.
import * as THREE from 'three';
import { segPointDist, polySd } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

const TOL = 1e-5;

function inSection(sh, p) {
  if (sh.kind === 'ring') {
    const r = Math.hypot(p[0] - sh.c[0], p[1] - sh.c[1]);
    return r <= sh.ro + TOL && r >= (sh.ri ?? 0) - TOL;
  }
  if (sh.kind === 'cap') return segPointDist(p, sh.a, sh.b) <= sh.r + TOL;
  return polySd(sh.pts, p) <= TOL && (sh.holes ?? []).every((h) => Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) >= h.r - TOL);
}

/** The meshes that draw a physics tag ('part/solid'), or the part's own when it names a part. */
export const meshesOf = (bird, tag) => bird.meshes.filter((o) => (o.userData.physics ?? []).some((t) => t === tag || t === tag.split('/')[0]));

/** The physics' drive pose at a phase and amplitude (the pose bird.update draws). */
export const poseOf = (bird, phase, amp) => drivePose({ law: bird.phys.lay.law, fourBar: bird.phys.lay.fb }, phase, amp);

/**
 * The drawn box of one solid at the bird's current pose (bird.update at phase, amp first): the world box of the
 * vertices of its meshes that lie inside the solid there. -> THREE.Box3 (empty if nothing of it is drawn)
 */
export function drawnBox(bird, tag, q) {
  const [id, name] = [tag.split('/')[0], tag.slice(tag.indexOf('/') + 1)];
  const s = bird.phys.solid(id, name);
  if (!s) throw new Error(`${tag}: no such solid`);
  const sh = s.at ? s.at(q) : s.shape;
  const box = new THREE.Box3(), v = new THREE.Vector3();
  bird.root.updateMatrixWorld(true);
  for (const o of meshesOf(bird, tag)) {
    const pos = o.geometry.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
      if (v.x >= s.x[0] - TOL && v.x <= s.x[1] + TOL && inSection(sh, [v.z, v.y])) box.expandByPoint(v);
    }
  }
  return box;
}

/** The radii, about the centre c ([z, y]), of what is drawn of one solid at the current pose. -> { ri, ro } */
export function drawnRadii(bird, tag, q, c) {
  const [id, name] = [tag.split('/')[0], tag.slice(tag.indexOf('/') + 1)];
  const s = bird.phys.solid(id, name), sh = s.at ? s.at(q) : s.shape, v = new THREE.Vector3();
  let ri = Infinity, ro = 0;
  bird.root.updateMatrixWorld(true);
  for (const o of meshesOf(bird, tag)) {
    const pos = o.geometry.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
      if (!(v.x >= s.x[0] - TOL && v.x <= s.x[1] + TOL && inSection(sh, [v.z, v.y]))) continue;
      const r = Math.hypot(v.z - c[0], v.y - c[1]);
      ri = Math.min(ri, r); ro = Math.max(ro, r);
    }
  }
  return { ri, ro };
}
