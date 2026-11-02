// The bird's stage: a dark studio (design principles: dark-first, one accent) with a key, a cool rim and a low
// fill, a room environment for reflections, soft shadows on a dark floor, GTAO for the mechanism's depth and a
// light bloom held under the threshold so it never eats detail. Or, with a backdrop, the Bow Valley from
// out/env/web: terrain, tree impostors and the sky panorama in the world frame of SPEC.
import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';

export const STUDIO = { bg: '#0b0e12', floor: '#0f1318', key: '#fff1e0', rim: '#b9d4ff', fill: '#6d7c90' };

/**
 * A procedural studio light tent for PMREMGenerator.fromScene(): a few large, soft emissive panels (not Room
 * Environment's small indoor-room light strips), positioned on the same key/rim/fill directions as the stage's own
 * directional lights, so reflections and the direct lighting agree. Built only from primitives, no downloads.
 */
function buildStudioEnvironment() {
  const env = new THREE.Scene();
  env.name = 'studioEnvironment';
  const panel = (w, h, color, intensity, pos) => {
    const mat = new THREE.MeshLambertMaterial({ color: 0x000000, emissive: new THREE.Color(color), emissiveIntensity: intensity, side: THREE.DoubleSide });
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), mat);
    m.position.set(...pos);
    m.lookAt(0, 0, 0);
    return m;
  };
  // a large dim bounce enclosure so metal and clearcoat reflections meet a believable dark studio surround, never
  // a void: a bit lighter below (floor bounce) than above.
  const room = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial({ color: 0x12161d, side: THREE.BackSide }));
  room.scale.set(24, 18, 24);
  env.add(room);
  const floorBounce = new THREE.Mesh(new THREE.CircleGeometry(10, 32), new THREE.MeshLambertMaterial({ color: 0x1b2026, side: THREE.DoubleSide }));
  floorBounce.rotation.x = -Math.PI / 2;
  floorBounce.position.y = -6;
  env.add(floorBounce);
  // key: one large soft panel, up and front-right -- same direction as the key DirectionalLight (1.2, 2.2, 1.6).
  // Large and soft on purpose (that is the point), but kept modest in brightness: the wing covering's low
  // roughness (0.06-0.08) means even a "soft" panel reflects as a wide, bright band across a curved surface at
  // some point in every stroke, and a too-bright panel turns that band into a flat white sheet with bloom.
  env.add(panel(9, 7, STUDIO.key, 3, [7, 10, 9]));
  // rim: a large cool panel behind and to the left -- same direction as the rim light (-1.6, 1.0, -2.0), for the
  // glint along silhouettes and through the wing film's Fresnel edge
  env.add(panel(8, 9, STUDIO.rim, 3.5, [-8, 6, -10]));
  // fill: broad and dim, low and from the front, so the shadowed side never reads as dead black
  env.add(panel(11, 6, STUDIO.fill, 1.2, [0, -1.5, 10]));
  // a soft overhead glow: a top highlight on rounded parts without a hard hotspot
  const top = panel(11, 11, '#dfe8f5', 0.9, [0.001, 15, 0]);
  env.add(top);
  return env;
}

// A fixed-pivot S-curve plus a touch of extra chroma, applied after AgXToneMapping + sRGB encoding, to move three.js's
// "AgX Base" rendering toward Blender's "AgX - Medium High Contrast" look (see film/edit.json's blender.look and
// the flight plates it renders, e.g. out/env/review/pass3/after/track_track_side_long_01566.png). Parameters tuned
// by eye against that plate's shadow depth and highlight roll-off; film/test/materials.test.mjs pins the curve
// shape (monotonic, fixed point at mid-grey, no clipping) rather than the exact constants.
export const AGX_CONTRAST = { contrast: 1.1, pivot: 0.46, saturation: 1.06 };
const AGX_CONTRAST_SHADER = {
  uniforms: { tDiffuse: { value: null }, uContrast: { value: AGX_CONTRAST.contrast }, uPivot: { value: AGX_CONTRAST.pivot }, uSaturation: { value: AGX_CONTRAST.saturation } },
  vertexShader: /* glsl */ `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
  fragmentShader: /* glsl */ `uniform sampler2D tDiffuse; uniform float uContrast; uniform float uPivot; uniform float uSaturation; varying vec2 vUv;
    void main(){
      vec4 c = texture2D(tDiffuse, vUv);
      vec3 col = (c.rgb - uPivot) * uContrast + uPivot;
      float l = dot(col, vec3(0.2126, 0.7152, 0.0722));
      col = mix(vec3(l), col, uSaturation);
      gl_FragColor = vec4(clamp(col, 0.0, 1.0), c.a);
    }`,
};

const BG_VERT = /* glsl */ `varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0.9999, 1.0); }`;
const BG_FRAG = /* glsl */ `uniform vec3 uA; uniform vec3 uB; uniform vec2 uC; varying vec2 vUv;
  void main(){ float d = length((vUv - uC) * vec2(1.6, 1.0)); vec3 c = mix(uA, uB, clamp(d * 1.1, 0.0, 1.0));
  // a faint vignette and grain-free falloff
  gl_FragColor = vec4(c, 1.0); }`;

export function createStage(canvas, { quality = 'high', capture = false, pr = null } = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, preserveDrawingBuffer: capture, powerPreference: 'high-performance' });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.AgXToneMapping;
  renderer.toneMappingExposure = 1.15;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.VSMShadowMap; // soft contact shadow, cheap for a single subject (PCFSoftShadowMap is a removed alias of PCFShadowMap as of r186)
  renderer.localClippingEnabled = true;
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(30, 16 / 9, 0.01, 60000);

  // studio background: a radial gradient quad drawn behind everything
  const bgMat = new THREE.ShaderMaterial({
    vertexShader: BG_VERT, fragmentShader: BG_FRAG, depthWrite: false, depthTest: false,
    uniforms: { uA: { value: new THREE.Color('#1a2029') }, uB: { value: new THREE.Color(STUDIO.bg) }, uC: { value: new THREE.Vector2(0.5, 0.62) } },
  });
  const bg = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), bgMat);
  bg.frustumCulled = false;
  bg.renderOrder = -10;
  const bgScene = new THREE.Scene();
  bgScene.add(bg);

  const pmrem = new THREE.PMREMGenerator(renderer);
  const room = pmrem.fromScene(buildStudioEnvironment(), 0.035).texture;
  scene.environment = room;
  scene.environmentIntensity = 0.7;

  // A DirectionalLight has zero angular size, so its specular reflection on any low-roughness surface (the film
  // covering, clearcoat, polished metal) is a hard, pin-sharp hotspot no matter how it is tone mapped -- real
  // studio keys are softboxes, not points. These stay dim and mostly carry shading shape and the cast shadow; the
  // soft, large panels in buildStudioEnvironment() (reflected via scene.environment) carry the specular highlight's
  // shape and size instead, the way an area light would.
  const key = new THREE.DirectionalLight(STUDIO.key, 1.1);
  key.position.set(1.2, 2.2, 1.6);
  key.castShadow = true;
  key.shadow.mapSize.set(quality === 'low' ? 1024 : 2048, quality === 'low' ? 1024 : 2048);
  key.shadow.camera.left = key.shadow.camera.bottom = -1.2;
  key.shadow.camera.right = key.shadow.camera.top = 1.2;
  key.shadow.camera.near = 0.1;
  key.shadow.camera.far = 8;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.002;
  key.shadow.radius = 3;
  const rim = new THREE.DirectionalLight(STUDIO.rim, 0.8);
  rim.position.set(-1.6, 1.0, -2.0);
  const fill = new THREE.HemisphereLight('#c7d6ea', '#0b0d10', 0.4);
  scene.add(key, key.target, rim, fill);

  // dark floor that fades into the backdrop: no horizon line, the bird floats over its own soft shadow
  const floorMat = new THREE.ShaderMaterial({
    uniforms: { uC: { value: new THREE.Color('#10151b') }, uBg: { value: new THREE.Color(STUDIO.bg) }, uR: { value: 2.2 } },
    vertexShader: /* glsl */ `varying vec3 vW; void main(){ vec4 w = modelMatrix * vec4(position, 1.0); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }`,
    fragmentShader: /* glsl */ `uniform vec3 uC; uniform vec3 uBg; uniform float uR; varying vec3 vW;
      void main(){ float d = length(vW.xz); vec3 c = mix(uC, uBg, smoothstep(0.2 * uR, uR, d)); gl_FragColor = vec4(c, 1.0);
      #include <colorspace_fragment>
      }`,
  });
  const floor = new THREE.Mesh(new THREE.CircleGeometry(6, 64), floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.45;
  floor.receiveShadow = true;
  const shadowCatcher = new THREE.Mesh(new THREE.CircleGeometry(6, 64), new THREE.ShadowMaterial({ opacity: 0.32, polygonOffset: true, polygonOffsetFactor: -1, transparent: true, depthWrite: false }));
  shadowCatcher.rotation.x = -Math.PI / 2;
  shadowCatcher.position.y = -0.4499;
  shadowCatcher.receiveShadow = true;
  const studio = new THREE.Group();
  // no visible floor: only its shadow, over the backdrop gradient, so there is never a horizon line
  studio.add(shadowCatcher);
  scene.add(studio);

  const target = new THREE.WebGLRenderTarget(4, 4, { type: THREE.HalfFloatType, samples: quality === 'low' ? 2 : 4 });
  const composer = new EffectComposer(renderer, target);
  const rp = new RenderPass(scene, camera);
  rp.clear = false;
  // draw the gradient first, inside the composer's buffer
  const bgPass = { enabled: true, needsSwap: false, clear: false, renderToScreen: false, setSize() {}, render(r, w, readBuffer) {
    r.setRenderTarget(readBuffer);
    r.autoClear = false;
    r.clear(true, true, true);
    if (studioMode.on) r.render(bgScene, camera);
    r.autoClear = true;
  }, dispose() {} };
  composer.addPass(bgPass);
  composer.addPass(rp);
  const gtao = quality === 'low' ? null : new GTAOPass(scene, camera, 4, 4);
  if (gtao) {
    gtao.output = GTAOPass.OUTPUT.Default;
    gtao.blendIntensity = 0.75;
    gtao.updateGtaoMaterial({ radius: 0.025, distanceExponent: 1.3, thickness: 0.4, scale: 1.0, samples: 16, distanceFallOff: 1.0, screenSpaceRadius: false });
    gtao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 5, rings: 2, samples: 16 });
    composer.addPass(gtao);
  }
  const bloom = new UnrealBloomPass(new THREE.Vector2(4, 4), 0.12, 0.35, 0.97);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());
  // three.js's AgXToneMapping is the Blender "AgX Base" curve; the flight plates render with Blender's "AgX -
  // Medium High Contrast" look, which is Base plus a fixed S-curve lift around middle grey. Reproduced here as a
  // small post pass (after tonemapping and sRGB encoding, same as Blender applies its look) rather than refitting
  // the tonemapping curve itself.
  composer.addPass(new ShaderPass(AGX_CONTRAST_SHADER));

  const studioMode = { on: true };
  const setStudio = (on) => {
    studioMode.on = on;
    studio.visible = on;
    scene.environmentIntensity = on ? 0.85 : 0.5;
    if (gtao) gtao.enabled = on;
  };

  const resize = (w, h) => {
    const p = pr ?? Math.min(devicePixelRatio || 1, 2);
    renderer.setPixelRatio(p);
    renderer.setSize(w, h, false);
    composer.setPixelRatio(p);
    composer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  /** Keep the key light's shadow box on the subject. */
  const aimLights = (centre, radius = 1.2) => {
    key.target.position.copy(centre);
    key.position.copy(centre).add(new THREE.Vector3(1.2, 2.2, 1.6));
    const c = key.shadow.camera;
    c.left = c.bottom = -radius;
    c.right = c.top = radius;
    c.updateProjectionMatrix();
    rim.position.copy(centre).add(new THREE.Vector3(-1.6, 1.0, -2.0));
  };
  const render = () => composer.render();
  return { renderer, scene, camera, composer, key, rim, fill, floor, studio, bloom, gtao, resize, render, setStudio, aimLights, pmrem };
}

/**
 * The Bow Valley backdrop from out/env/web (manifest.json): terrain mesh with its albedo, the sky panorama as the
 * background, and tree impostors as instanced cross quads. Loaded at runtime from `base` (files beside the page).
 * Resolves to null if the files cannot be read (a page opened from file://): the bird then flies in the studio.
 */
export async function loadValley(stage, base) {
  try {
    const man = await (await fetch(`${base}/manifest.json`)).json();
    const tl = new THREE.TextureLoader();
    const load = (f) => new Promise((res, rej) => tl.load(`${base}/${f}`, res, undefined, rej));
    const [terrainMesh, sky, albedo] = await Promise.all([
      fetch(`${base}/${man.terrain.file}`).then((r) => r.arrayBuffer()).then(readGlbMesh),
      load(man.panorama.file),
      load('terrain_albedo.jpg').catch(() => null),
    ]);
    const gltf = { scene: terrainMesh };
    sky.mapping = THREE.EquirectangularReflectionMapping;
    sky.colorSpace = THREE.SRGBColorSpace;
    const group = new THREE.Group();
    group.name = 'valley';
    const terrain = gltf.scene;
    terrain.traverse((o) => {
      if (o.isMesh) {
        o.receiveShadow = true;
        if (albedo && o.material && !o.material.map) {
          albedo.colorSpace = THREE.SRGBColorSpace;
          o.material.map = albedo;
        }
        if (o.material) {
          o.material.roughness = 0.95;
          o.material.metalness = 0;
          o.material.envMapIntensity = 0.3;
        }
      }
    });
    group.add(terrain);
    // tree impostors
    const trees = await loadTrees(base, man).catch(() => null);
    if (trees) group.add(trees);
    const sun = new THREE.Vector3(...man.sun_dir).normalize();
    return { group, sky, sun, manifest: man };
  } catch (e) {
    console.warn('valley backdrop unavailable', e);
    return null;
  }
}

/**
 * The single-mesh GLB that env/export_web.py writes (POSITION, NORMAL, TEXCOORD_0, indices), read without the full
 * GLTFLoader so the page stays under 1 MB.
 */
function readGlbMesh(buf) {
  const dv = new DataView(buf);
  const jsonLen = dv.getUint32(12, true);
  const json = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 20, jsonLen)));
  const binStart = 20 + jsonLen + 8;
  const acc = (i) => {
    const a = json.accessors[i];
    const v = json.bufferViews[a.bufferView];
    const n = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4 }[a.type] * a.count;
    const off = binStart + (v.byteOffset ?? 0) + (a.byteOffset ?? 0);
    const Ctor = a.componentType === 5125 ? Uint32Array : a.componentType === 5123 ? Uint16Array : Float32Array;
    return new Ctor(buf.slice(off, off + n * Ctor.BYTES_PER_ELEMENT));
  };
  const prim = json.meshes[0].primitives[0];
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(acc(prim.attributes.POSITION), 3));
  if (prim.attributes.NORMAL != null) g.setAttribute('normal', new THREE.BufferAttribute(acc(prim.attributes.NORMAL), 3));
  if (prim.attributes.TEXCOORD_0 != null) g.setAttribute('uv', new THREE.BufferAttribute(acc(prim.attributes.TEXCOORD_0), 2));
  g.setIndex(new THREE.BufferAttribute(acc(prim.indices), 1));
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.92, metalness: 0 }));
  m.name = 'terrain';
  const root = new THREE.Group();
  root.add(m);
  return root;
}

async function loadTrees(base, man) {
  const t = man.trees;
  const buf = await (await fetch(`${base}/${t.file}`)).arrayBuffer();
  const n = Math.min(t.count, Math.floor(buf.byteLength / t.bytes_per_instance));
  const dv = new DataView(buf);
  const tex = await new THREE.TextureLoader().loadAsync(`${base}/${t.atlas.file}`);
  tex.colorSpace = THREE.SRGBColorSpace;
  const names = Object.keys(t.types);
  const cells = t.atlas.cells;
  // two crossed quads per tree, uv per type through an instanced attribute
  const geo = new THREE.InstancedBufferGeometry();
  const quad = new THREE.PlaneGeometry(1, 1, 1, 1);
  quad.translate(0, 0.5, 0);
  const q2 = quad.clone().rotateY(Math.PI / 2);
  const pos = [...quad.attributes.position.array, ...q2.attributes.position.array];
  const uv = [...quad.attributes.uv.array, ...q2.attributes.uv.array];
  const idx = [...quad.index.array, ...Array.from(q2.index.array, (v) => v + 4)];
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  const off = new Float32Array(n * 4), uvr = new Float32Array(n * 4);
  for (let i = 0; i < n; i++) {
    const o = i * t.bytes_per_instance;
    off[i * 4] = dv.getFloat32(o, true);
    off[i * 4 + 1] = dv.getFloat32(o + 4, true);
    off[i * 4 + 2] = dv.getFloat32(o + 8, true);
    const ty = dv.getUint8(o + 16);
    const cell = cells[names[ty]] ?? cells[names[0]];
    const hgt = dv.getFloat32(o + 12, true);
    off[i * 4 + 3] = hgt;
    uvr[i * 4] = cell.u0;
    uvr[i * 4 + 1] = cell.u1;
    uvr[i * 4 + 2] = cell.v0;
    uvr[i * 4 + 3] = cell.v1;
  }
  geo.setAttribute('aOff', new THREE.InstancedBufferAttribute(off, 4));
  geo.setAttribute('aUv', new THREE.InstancedBufferAttribute(uvr, 4));
  geo.instanceCount = n;
  const aspect = t.atlas.aspect_cell_w_over_h;
  const mat = new THREE.ShaderMaterial({
    uniforms: { map: { value: tex }, uAspect: { value: aspect }, uFog: { value: new THREE.Color('#9fb3c8') } },
    vertexShader: /* glsl */ `attribute vec4 aOff; attribute vec4 aUv; uniform float uAspect; varying vec2 vUv; varying float vD;
      void main(){ vec3 p = position; p.xz *= aOff.w * uAspect; p.y *= aOff.w; vec4 w = vec4(p + aOff.xyz, 1.0);
        vUv = vec2(mix(aUv.x, aUv.y, uv.x), mix(aUv.z, aUv.w, uv.y)); vec4 mv = viewMatrix * w; vD = -mv.z; gl_Position = projectionMatrix * mv; }`,
    fragmentShader: /* glsl */ `uniform sampler2D map; uniform vec3 uFog; varying vec2 vUv; varying float vD;
      void main(){ vec4 c = texture2D(map, vUv); if (c.a < 0.45) discard; float f = clamp(vD / 9000.0, 0.0, 0.65); gl_FragColor = vec4(mix(c.rgb, uFog, f), 1.0); }`,
    side: THREE.DoubleSide,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.frustumCulled = false;
  mesh.name = 'trees';
  return mesh;
}
