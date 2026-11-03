// The covering's colour, per surface: the one place it is set (film/blender_bird.py reads it from the GLB, through the
// vertex colours bird.js writes). Pure (no three.js), so film/render.mjs resolves the default livery with it.
// The default is the white the BOM records (defaultLivery), else no livery: the clear film. A livery gives each
// covered surface a real colour of the covering film from the maker's published range, with its order number and areal mass, and linear RGB for the
// renderers (the maker publishes no colour values, so the RGB is a renderer's approximation of the named colour).
// The film labels any livery the catalogue does not record yet (`liveryInBom`), so a preview never passes for the
// built bird. A surface set to null keeps the colour of the face it lies on.
export const COVERING_SURFACES = ['wing_upper', 'wing_lower', 'wing_tip', 'tail', 'pod'];
// ORACOVER Air Light (Lanitz-Prena), 12 um, approx. 18 g/m2 in every colour (maker's 2010 catalogue p. 21, and its
// shop pages): light scale white 331-099 and the light chromes 331-090 (silver), -093, -094, -096, -097. There is no
// clear Air Light (the clear 17 g/m2 film is Air Indoor 331-000), no grey and no black, so the gull livery is white
// below with the silver chrome standing in for gull grey above, and no black tips. The catalogue entry the BOM uses
// (film-oracover-air-light) is the white, 331-099, at the maker's technical data sheet's 17 g/m2 (the ruling's figure).
const AIR_LIGHT_WHITE = { colour: 'light scale white', order: '331-099', g_m2: 17, rgb: [0.871, 0.871, 0.84] };
const AIR_LIGHT_CHROME = { colour: 'light chrome', order: '331-090', g_m2: 18, rgb: [0.55, 0.565, 0.59], metallic: true };
export const LIVERIES = {
  // the bird as built (v22 on): the BOM's one film, light scale white, on every covered surface
  white: {
    name: 'white',
    film: 'film-oracover-air-light',
    tipFrom: 0.8,
    surfaces: Object.fromEntries(COVERING_SURFACES.map((s) => [s, AIR_LIGHT_WHITE])),
  },
  // a preview only: its chrome is not in the BOM
  gull: {
    name: 'gull',
    film: 'film-oracover-air-light',
    // where a tip colour would start: the rib nearest this fraction of the wing's length from the root (a film panel
    // joins at a rib, where it is ironed down); unused while wing_tip is null
    tipFrom: 0.8,
    surfaces: { wing_upper: AIR_LIGHT_CHROME, wing_lower: AIR_LIGHT_WHITE, wing_tip: null, tail: AIR_LIGHT_WHITE, pod: AIR_LIGHT_WHITE },
  },
};
/** The livery the film draws by default when the BOM records it (defaultLivery); never the gull, a preview. */
export const COVERING_LIVERY = 'white';

/** A livery by name (null, '' or 'clear' for none, the clear film); unknown names throw rather than quietly drawing clear. */
export function livery(name = null) {
  if (name == null || name === '' || name === 'clear') return null;
  const l = LIVERIES[name];
  if (!l) throw new Error(`no livery "${name}" (known: clear, ${Object.keys(LIVERIES).join(', ')})`);
  return l;
}

/**
 * True when the BOM records the livery: the catalogue entry of its film records every colour it uses, and (given the
 * BOM) every covering line of the BOM, each structure line whose material is a film of that entry's category, is that
 * film. The clear film (no livery) is always recorded.
 */
export function liveryInBom(l, parts, bom = null) {
  if (!l) return true;
  const recorded = [parts?.[l.film]?.specs?.colour, ...(parts?.[l.film]?.specs?.colours ?? [])].filter(Boolean);
  if (!COVERING_SURFACES.every((s) => !l.surfaces[s] || recorded.includes(l.surfaces[s].colour))) return false;
  if (!bom) return true;
  const cat = parts[l.film].category;
  const covering = (bom.structure ?? []).filter((x) => parts[x.material_id]?.category === cat);
  return covering.length > 0 && covering.every((x) => x.material_id === l.film);
}

/** The film's livery by default: COVERING_LIVERY when the BOM records it, else none (the clear film). */
export function defaultLivery(parts, bom) {
  return liveryInBom(LIVERIES[COVERING_LIVERY], parts, bom) ? COVERING_LIVERY : null;
}

