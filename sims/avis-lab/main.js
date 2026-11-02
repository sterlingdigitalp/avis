// Review stills of the bird in the studio: ?s=<base64 json> {state, cam:{pos,target,fov}, size}. Used by the review
// loop (film/still.mjs) to look at the mechanism from any angle at any phase without building a film.
import * as THREE from 'three';
import { model } from '../avis-shared/data.js';
import { makeMaterials } from '../avis-shared/materials.js';
import { buildBird } from '../avis-shared/bird.js';
import { createStage } from '../avis-shared/stage.js';

const q = new URLSearchParams(location.search);
const cfg = JSON.parse(q.get('s') ? atob(q.get('s')) : '{}');
document.body.style.cssText = 'margin:0;background:#0b0e12;overflow:hidden';
const canvas = document.createElement('canvas');
canvas.style.cssText = 'width:100vw;height:100vh;display:block';
document.getElementById('app').appendChild(canvas);
const stage = createStage(canvas, { capture: true, pr: 1 });
const mats = makeMaterials({ textures: true });
for (const m of Object.values(mats)) m.envMap = null;
const bird = buildBird(model, mats);
stage.scene.add(bird.root);
const st = cfg.state ?? {};
bird.update({ accents: true, ...st });
const cam = cfg.cam ?? { pos: [0.9, 0.45, 1.3], target: [0, 0, 0], fov: 30 };
stage.camera.fov = cam.fov ?? 30;
stage.camera.position.set(...cam.pos);
stage.camera.lookAt(new THREE.Vector3(...cam.target));
stage.aimLights(new THREE.Vector3(...cam.target), 1.2);
const resize = () => stage.resize(innerWidth, innerHeight);
resize();
if (cfg.plain) stage.renderer.render(stage.scene, stage.camera);
else {
  if (cfg.nogtao && stage.gtao) stage.gtao.enabled = false;
  if (cfg.nobloom) stage.bloom.enabled = false;
  stage.render();
}
const px = new Uint8Array(4); const gl0 = stage.renderer.getContext(); gl0.readPixels(innerWidth/2|0, innerHeight/2|0, 1, 1, gl0.RGBA, gl0.UNSIGNED_BYTE, px);
const gl = stage.renderer.getContext();
window.DEBUG = { w: canvas.width, h: canvas.height, calls: stage.renderer.info.render.calls, tris: stage.renderer.info.render.triangles, err: gl.getError(), cam: stage.camera.position.toArray(), px: Array.from(px) };
window.READY = true;
