// The Avis film page. film/capture.mjs serves it with data/edit.json (resolved by film/render.mjs) and steps
// window.FILM.frame(t) frame by frame; frame 0 already shows the bird.
import { model, DATA } from '../avis-shared/data.js';
import { createFilm } from '../avis-shared/film.js';

document.body.style.cssText = 'margin:0;background:#0b0e12;overflow:hidden';
const app = document.getElementById('app');
const q = new URLSearchParams(location.search);
const editName = q.get('edit') ?? 'edit';

(async () => {
  const edit = await (await fetch(`data/${editName}.json`)).json();
  const segData = {};
  await Promise.all(edit.segments.filter((s) => s.kind === 'bl').map(async (s) => {
    const r = await fetch(`data/seg-${s.id}.json`).catch(() => null);
    if (r?.ok) segData[s.id] = await r.json();
  }));
  await document.fonts.ready;
  const film = createFilm(app, { model, data: DATA, edit, plateBase: 'plates', segData });
  window.FILM = film;
  await film.frame(+(q.get('t') ?? 0));
  window.READY = true;
})();
