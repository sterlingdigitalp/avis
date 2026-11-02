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

/** All materials, by name. `accent` is the single colour that marks the part working. */
export function makeMaterials({ textures = true, envIntensity = 0.6 } = {}) {
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
  std('carbon_woven', { color: weaveTex ? 0xffffff : 0x1d1f22, map: weaveTex, roughness: 0.32, metalness: 0.0, clearcoat: 0.9, clearcoatRoughness: 0.12 });
  std('carbon_pultruded', { color: uniTex ? 0xffffff : 0x15171a, map: uniTex, roughness: 0.28, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.2 });
  std('carbon_plate', { color: plateTex ? 0xffffff : 0x1b1d20, map: plateTex, roughness: 0.4, metalness: 0.0, clearcoat: 0.5, clearcoatRoughness: 0.25 });
  std('balsa', { color: balsaTex ? 0xffffff : 0xc9a77a, map: balsaTex, roughness: 0.75, metalness: 0.0, sheen: 0.3, sheenColor: new THREE.Color(0xe8d2ad) });
  std('film_wing', { color: 0xd8e3ec, roughness: 0.22, metalness: 0.0, transmission: 0.0, transparent: true, opacity: 0.26, side: THREE.DoubleSide, depthWrite: false, clearcoat: 0.35, clearcoatRoughness: 0.15, specularIntensity: 0.6 });
  std('film_shell', { color: 0xdde4ea, roughness: 0.2, metalness: 0.0, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false, clearcoat: 1.0, clearcoatRoughness: 0.1 });
  std('steel_ground', { color: 0xb9bcc0, map: brushTex, roughness: 0.3, metalness: 1.0 });
  std('steel_shaft', { color: 0xd0d3d6, roughness: 0.2, metalness: 1.0 });
  std('acetal', { color: 0xbdb7aa, roughness: 0.55, metalness: 0.0, sheen: 0.15, sheenColor: new THREE.Color(0xffffff) });
  std('alu_anodised', { color: 0x3a3f46, map: brushTex, roughness: 0.38, metalness: 0.9 });
  std('alu_machined', { color: 0xc8ccd1, map: brushTex, roughness: 0.32, metalness: 1.0 });
  std('servo_case', { color: 0x2b2f35, map: brushTex, roughness: 0.42, metalness: 0.85 });
  std('battery_wrap', { color: 0x1f2226, roughness: 0.35, metalness: 0.2, clearcoat: 0.8, clearcoatRoughness: 0.3 });
  std('esc_wrap', { color: 0x24272b, roughness: 0.5, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.35 });
  std('pcb', { color: 0x14301f, roughness: 0.5, metalness: 0.1, clearcoat: 0.7, clearcoatRoughness: 0.2 });
  std('gps_case', { color: 0x2a2d31, roughness: 0.55, metalness: 0.0 });
  std('wire', { color: 0x16181b, roughness: 0.55, metalness: 0.0, sheen: 0.4, sheenColor: new THREE.Color(0x555b63) });
  std('copper', { color: 0xc98a5b, roughness: 0.3, metalness: 1.0 });
  std('titanium', { color: 0x9a9690, roughness: 0.35, metalness: 1.0 });
  std('magnet', { color: 0x6d7177, roughness: 0.4, metalness: 0.9 });
  std('accent', { color: new THREE.Color(ACCENT), emissive: new THREE.Color(ACCENT), emissiveIntensity: 0.6, roughness: 0.4, metalness: 0.0 });
  return m;
}
