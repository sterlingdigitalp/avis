// Materials for the bird. Each is named; Blender (film/blender_bird.py) rebuilds the same names as Cycles node
// trees, so the page, the take-apart film and the flight shots read as one object. Procedural canvas textures
// (carbon weave, unidirectional streaks, balsa grain, brushed metal) are made only where a canvas exists.
import * as THREE from 'three';

export const ACCENT = '#ffb547';

function canvasTex(size, draw, { repeat = [1, 1], srgb = true } = {}) {
  if (typeof document === 'undefined') return null;
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const g = c.getContext('2d');
  draw(g, size);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(repeat[0], repeat[1]);
  t.anisotropy = 8;
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// deterministic noise for textures (no Math.random: the film must render the same every time)
function hash(i) {
  let x = Math.imul(i ^ 0x9e3779b9, 0x85ebca6b);
  x ^= x >>> 13;
  x = Math.imul(x, 0xc2b2ae35);
  x ^= x >>> 16;
  return (x >>> 0) / 4294967296;
}

/** 2x2 twill weave: tows alternate direction per cell, each with a soft sheen across its width. */
function weave(g, n, cells = 8) {
  const s = n / cells;
  for (let i = 0; i < cells; i++) for (let j = 0; j < cells; j++) {
    const horiz = ((i + j) >> 1) % 2 === 0;
    const grad = horiz ? g.createLinearGradient(0, j * s, 0, (j + 1) * s) : g.createLinearGradient(i * s, 0, (i + 1) * s, 0);
    const base = 26 + 10 * hash(i * 31 + j);
    grad.addColorStop(0, `rgb(${base - 12},${base - 11},${base - 9})`);
    grad.addColorStop(0.5, `rgb(${base + 26},${base + 27},${base + 30})`);
    grad.addColorStop(1, `rgb(${base - 12},${base - 11},${base - 9})`);
    g.fillStyle = grad;
    g.fillRect(i * s, j * s, s, s);
  }
}

function streaks(g, n, dark = 18, light = 46) {
  g.fillStyle = `rgb(${dark},${dark},${dark + 2})`;
  g.fillRect(0, 0, n, n);
  for (let i = 0; i < n; i++) {
    const v = dark + (light - dark) * hash(i * 7 + 3) ** 3;
    g.fillStyle = `rgba(${v},${v},${v + 3},0.55)`;
    g.fillRect(0, i, n, 1);
  }
}

function balsa(g, n) {
  g.fillStyle = '#c9a77a';
  g.fillRect(0, 0, n, n);
  for (let i = 0; i < 90; i++) {
    const y = hash(i * 13) * n;
    const w = 0.5 + 2.5 * hash(i * 17);
    g.strokeStyle = `rgba(${150 + 40 * hash(i)},${115 + 30 * hash(i + 1)},${70 + 20 * hash(i + 2)},0.35)`;
    g.lineWidth = w;
    g.beginPath();
    g.moveTo(0, y);
    for (let x = 0; x <= n; x += n / 16) g.lineTo(x, y + 3 * Math.sin(x * 0.02 + i));
    g.stroke();
  }
}

function brushed(g, n, base = 170) {
  g.fillStyle = `rgb(${base},${base},${base + 4})`;
  g.fillRect(0, 0, n, n);
  for (let i = 0; i < n * 2; i++) {
    const v = base + 40 * (hash(i * 5 + 1) - 0.5);
    g.fillStyle = `rgba(${v},${v},${v + 3},0.5)`;
    g.fillRect(0, hash(i * 3) * n, n, 1);
  }
}

// Oracover Air Light (film-oracover-air-light, catalogue specs: 12 um, 17 g/m2, no colour recorded): clear,
// unpigmented polyester (PET) film, IOR about 1.64. A thin slab has two parallel air/PET interfaces; for a single
// interface the normal-incidence Fresnel reflectance is r = ((n-1)/(n+1))^2, and the two interfaces combine
// (incoherently, no colour fringing) to about 2r/(1+r) ~ 11%, rising toward a mirror at grazing incidence. Three.js
// only derives r itself from `ior` (one interface), so `specularIntensity` scales that up to the two-interface
// figure; the Schlick curve built into the specular BRDF then supplies the grazing-angle rise for free.
export const FILM_IOR = 1.64;
const filmR1 = ((FILM_IOR - 1) / (FILM_IOR + 1)) ** 2;
export const FILM_REFLECTANCE = (2 * filmR1) / (1 + filmR1); // ~0.111 at normal incidence
const filmSpecularIntensity = FILM_REFLECTANCE / filmR1; // ~1.89, see test/materials.test.mjs

// The covering's colour, per surface: the one place it is set (film/blender_bird.py reads it from the GLB). The
// default is no livery: the clear film, as catalogued. A livery gives each covered surface a real colour of the
// covering film from the maker's published range, with its order number and areal mass, and linear RGB for the
// renderers (the maker publishes no colour values, so the RGB is a renderer's approximation of the named colour).
// The film labels any livery the catalogue does not record yet (`liveryInBom`), so a preview never passes for the
// built bird. A surface set to null keeps the colour of the face it lies on.
export const COVERING_SURFACES = ['wing_upper', 'wing_lower', 'wing_tip', 'tail', 'pod'];
// ORACOVER Air Light (Lanitz-Prena), 12 um, approx. 18 g/m2 in every colour (maker's 2010 catalogue p. 21, and its
// shop pages): light scale white 331-099 and the light chromes 331-090 (silver), -093, -094, -096, -097. There is no
// clear Air Light (the clear 17 g/m2 film is Air Indoor 331-000), no grey and no black, so the gull livery is white
// below with the silver chrome standing in for gull grey above, and no black tips.
const AIR_LIGHT_WHITE = { colour: 'light scale white', order: '331-099', g_m2: 18, rgb: [0.871, 0.871, 0.84] };
const AIR_LIGHT_CHROME = { colour: 'light chrome', order: '331-090', g_m2: 18, rgb: [0.55, 0.565, 0.59], metallic: true };
export const LIVERIES = {
  gull: {
    name: 'gull',
    film: 'film-oracover-air-light',
    // where a tip colour would start: the rib nearest this fraction of the wing's length from the root (a film panel
    // joins at a rib, where it is ironed down); unused while wing_tip is null
    tipFrom: 0.8,
    surfaces: { wing_upper: AIR_LIGHT_CHROME, wing_lower: AIR_LIGHT_WHITE, wing_tip: null, tail: AIR_LIGHT_WHITE, pod: AIR_LIGHT_WHITE },
  },
};
/** The livery switched on for the film: null (clear, the default) or a key of LIVERIES. */
export const COVERING_LIVERY = null;

/** A livery by name (null for none, the clear film); unknown names throw rather than quietly drawing clear. */
export function livery(name = COVERING_LIVERY) {
  if (name == null || name === '' || name === 'clear') return null;
  const l = LIVERIES[name];
  if (!l) throw new Error(`no livery "${name}" (known: clear, ${Object.keys(LIVERIES).join(', ')})`);
  return l;
}

/** True when every surface's colour is a colour the catalogue records for the livery's film (none do yet). */
export function liveryInBom(l, parts) {
  if (!l) return true;
  const recorded = [parts?.[l.film]?.specs?.colour, ...(parts?.[l.film]?.specs?.colours ?? [])].filter(Boolean);
  return COVERING_SURFACES.every((s) => !l.surfaces[s] || recorded.includes(l.surfaces[s].colour));
}

/** A livery film: colour from the vertex colours, and metal (0..1, the `metal` vertex attribute bird.js writes) turning
 * a face into a metallised chrome film, which reflects like metal and passes no light. */
function liveryFilm(mat) {
  mat.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader.replace('void main() {', 'attribute float metal;\nvarying float vMetal;\nvoid main() {\n  vMetal = metal;');
    sh.fragmentShader = sh.fragmentShader.replace('void main() {', 'varying float vMetal;\nvoid main() {')
      .replace('float metalnessFactor = metalness;', 'float metalnessFactor = metalness * vMetal;')
      .replace('material.transmission = transmission;', 'material.transmission = transmission * (1.0 - vMetal);');
  };
  mat.customProgramCacheKey = () => 'avis-livery-film';
  return mat;
}

/** All materials, by name. `accent` is the single colour that marks the part working. With a livery the two film
 * materials take the per-surface colour from the meshes' vertex colours (bird.js writes them) as a pigmented film. */
export function makeMaterials({ textures = true, envIntensity = 0.6, livery: l = null } = {}) {
  const T = textures ? (f, o) => canvasTex(256, f, o) : () => null;
  const weaveTex = T((g, n) => weave(g, n), { repeat: [6, 1] });
  const uniTex = T((g, n) => streaks(g, n), { repeat: [1, 1] });
  const plateTex = T((g, n) => weave(g, n, 16), { repeat: [2, 2] });
  const balsaTex = T(balsa, { repeat: [1, 1] });
  const brushTex = T((g, n) => brushed(g, n, 165), { repeat: [1, 1] });
  const m = {};
  const std = (name, o) => {
    const mat = new THREE.MeshPhysicalMaterial({ envMapIntensity: envIntensity, ...o });
    mat.name = name;
    m[name] = mat;
    return mat;
  };
  // woven tube: clear-coated twill, a hard glossy resin coat over the weave
  std('carbon_woven', { color: weaveTex ? 0xffffff : 0x1d1f22, map: weaveTex, roughness: 0.3, metalness: 0.0, clearcoat: 1.0, clearcoatRoughness: 0.08 });
  // pultruded rod: satin, unidirectional fibre streaks, no separate coat (just the pultrusion resin's own sheen)
  std('carbon_pultruded', { color: uniTex ? 0xffffff : 0x15171a, map: uniTex, roughness: 0.42, metalness: 0.0, clearcoat: 0.18, clearcoatRoughness: 0.3 });
  std('carbon_plate', { color: plateTex ? 0xffffff : 0x1b1d20, map: plateTex, roughness: 0.4, metalness: 0.0, clearcoat: 0.5, clearcoatRoughness: 0.25 });
  std('balsa', { color: balsaTex ? 0xffffff : 0xc9a77a, map: balsaTex, roughness: 0.75, metalness: 0.0, sheen: 0.3, sheenColor: new THREE.Color(0xe8d2ad) });
  // wing covering, Oracover Air Light: clear film, not glass (transmission 0.97 leaves ~3% diffuse haze), thin-walled
  // (thickness 0, no interior volume to refract through), smooth but not a mirror (roughness in 0.06-0.08).
  std('film_wing', { color: 0xffffff, roughness: 0.08, metalness: 0.0, transmission: 0.97, thickness: 0, ior: FILM_IOR, specularIntensity: filmSpecularIntensity, side: THREE.DoubleSide });
  // fuselage pod covering and tail film: the same Oracover Air Light, so the same optical parameters
  std('film_shell', { color: 0xffffff, roughness: 0.08, metalness: 0.0, transmission: 0.97, thickness: 0, ior: FILM_IOR, specularIntensity: filmSpecularIntensity, side: THREE.DoubleSide });
  // a pigmented film: the colour scatters, so it reads as a thin coloured skin passing a little light, not a window;
  // the same glossy polyester face (same IOR and two-face reflectance) as the clear film
  if (l) for (const k of ['film_wing', 'film_shell']) liveryFilm(std(k, { color: 0xffffff, vertexColors: true, roughness: 0.08, metalness: 1.0, transmission: 1 / 6, thickness: 0, ior: FILM_IOR, specularIntensity: filmSpecularIntensity, side: THREE.DoubleSide }));
  std('steel_ground', { color: 0xb9bcc0, map: brushTex, roughness: 0.3, metalness: 1.0, anisotropy: 0.65, anisotropyRotation: 0 });
  // a turned/ground shaft: CylinderGeometry's U wraps the circumference and V runs the length, so the machining
  // marks (and the anisotropic highlight they cause) run along V -- rotate the default U-aligned anisotropy 90 deg.
  std('steel_shaft', { color: 0xd0d3d6, roughness: 0.2, metalness: 1.0, anisotropy: 0.7, anisotropyRotation: Math.PI / 2 });
  std('acetal', { color: 0xbdb7aa, roughness: 0.55, metalness: 0.0, sheen: 0.15, sheenColor: new THREE.Color(0xffffff) });
  std('alu_anodised', { color: 0x3a3f46, map: brushTex, roughness: 0.38, metalness: 0.9, anisotropy: 0.6, anisotropyRotation: 0 });
  std('alu_machined', { color: 0xc8ccd1, map: brushTex, roughness: 0.32, metalness: 1.0, anisotropy: 0.6, anisotropyRotation: 0 });
  std('servo_case', { color: 0x2b2f35, map: brushTex, roughness: 0.42, metalness: 0.85, anisotropy: 0.5, anisotropyRotation: 0 });
  std('battery_wrap', { color: 0x1f2226, roughness: 0.35, metalness: 0.2, clearcoat: 0.8, clearcoatRoughness: 0.3 });
  std('esc_wrap', { color: 0x24272b, roughness: 0.5, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.35 });
  std('pcb', { color: 0x14301f, roughness: 0.5, metalness: 0.1, clearcoat: 0.7, clearcoatRoughness: 0.2 });
  std('gps_case', { color: 0x2a2d31, roughness: 0.55, metalness: 0.0 });
  std('wire', { color: 0x16181b, roughness: 0.55, metalness: 0.0, sheen: 0.4, sheenColor: new THREE.Color(0x555b63) });
  std('copper', { color: 0xc98a5b, roughness: 0.3, metalness: 1.0 });
  std('titanium', { color: 0x9a9690, roughness: 0.35, metalness: 1.0, anisotropy: 0.55, anisotropyRotation: Math.PI / 2 });
  std('magnet', { color: 0x6d7177, roughness: 0.4, metalness: 0.9 });
  std('accent', { color: new THREE.Color(ACCENT), emissive: new THREE.Color(ACCENT), emissiveIntensity: 0.6, roughness: 0.4, metalness: 0.0 });
  return m;
}
