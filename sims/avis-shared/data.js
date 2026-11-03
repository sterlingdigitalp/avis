// The bundled data for the page and the film: written by film/gen_data.mjs from the design file, the catalogue and
// out/verification. Rebuild the page after regenerating it.
import data from '../../out/film/gen/model-data.json';
import { prepareFilm } from './model.js';

export const DATA = data;
export const model = prepareFilm(data.design, data.parts);
