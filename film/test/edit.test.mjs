// Run: node --import ./film/lib/three-hooks.mjs --test film/test/*.test.mjs
// The edit's transitions and the wide shot's marker. A change of segment either cuts or dips to black for at most
// half a second (the outgoing fade-out plus the incoming fade-in); only the film's last fade may be longer. The wide
// shot, where the 2 m bird is a few pixels across, rings it from the log.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { ROOT } from '../../sims/avis-shared/node-load.js';
import { birdMarker } from '../../sims/avis-shared/film.js';

const edits = ['edit.json', 'edit.preview.json'].map((f) => [f, JSON.parse(readFileSync(join(ROOT, 'film', f), 'utf8'))]);

test('every cut dips to black for at most 0.5 s, or cuts straight', () => {
  for (const [f, e] of edits) {
    const s = e.segments;
    for (let k = 0; k + 1 < s.length; k++) {
      const dip = (s[k].fadeOut ?? 0) + (s[k + 1].fadeIn ?? 0);
      assert.ok(dip <= 0.5 + 1e-9, `${f}: ${s[k].id} -> ${s[k + 1].id} dips for ${dip.toFixed(2)} s`);
    }
    assert.ok((s[0].fadeIn ?? 0) <= 0.5, `${f}: the first segment fades in for ${s[0].fadeIn} s`);
  }
});

test('the wide shot carries a marker, and only shots where the bird is too small to read do', () => {
  const [, e] = edits[0];
  const wide = e.segments.find((s) => s.shot === 'wide_valley');
  assert.equal(wide.marker, true);
  for (const s of e.segments.filter((x) => x.marker)) assert.equal(s.shot, 'wide_valley', `${s.id} has a marker`);
});

test('the marker rings the bird where it is, names it, its span and its distance; nothing off screen', () => {
  const html = birdMarker([1170.4, 850.2, 1243.6], 2, 'Larus GL-2');
  const ring = html.match(/class="mark" style="left:([\d.]+)px;top:([\d.]+)px;width:(\d+)px;height:(\d+)px/);
  assert.ok(ring, html);
  const [x, y, w] = [+ring[1], +ring[2], +ring[3]];
  assert.ok(Math.abs(x + w / 2 - 1170.4) < 1e-6 && Math.abs(y + w / 2 - 850.2) < 1e-6, 'centred on the bird');
  assert.ok(/Larus GL-2 · 2\.00 m span, 1\.2 km away/.test(html), html);
  assert.ok(/430 m away/.test(birdMarker([400, 500, 430], 2, 'X')));
  // near the right edge the label goes on the left of the ring, so it stays in frame
  assert.ok(/translateX\(-100%\)/.test(birdMarker([1700, 500, 900], 2, 'X')) && !/translateX/.test(birdMarker([400, 500, 900], 2, 'X')));
  for (const px of [null, undefined, [-5, 500, 900], [960, 1200, 900], [960, 540, -3], [960, 540, 0]]) assert.equal(birdMarker(px, 2, 'X'), '', JSON.stringify(px));
  // the label sits 14 px clear of the 26 px ring, 34 px above its top; on the left from x = 1360 (560 px from the edge)
  const lab = (h) => h.match(/class="markl" style="left:([-\d.]+)px;top:([-\d.]+)px;([^"]*)"/).slice(1);
  assert.deepEqual(lab(birdMarker([1000, 500, 900], 2, 'X')), ['1040', '440', '']);
  assert.deepEqual(lab(birdMarker([1360, 500, 900], 2, 'X')), ['1400', '440', '']);
  assert.deepEqual(lab(birdMarker([1361, 500, 900], 2, 'X')), ['1321', '440', 'transform:translateX(-100%)']);
  assert.ok(/1\.0 km away/.test(birdMarker([960, 540, 1000], 2, 'X')) && /999 m away/.test(birdMarker([960, 540, 999], 2, 'X')));
});

test('only a marker segment draws the ring, from its own frame of data; durations come from the edit', async () => {
  const { segMarker, seqDur } = await import('../../sims/avis-shared/film.js');
  const N = { span_m: 2, name: 'Larus GL-2' };
  const d = { px: [[100, 100, 900], [960, 540, 1240]] };
  assert.equal(segMarker({ marker: false }, d, 1, N), '');
  assert.equal(segMarker({}, d, 1, N), '');
  assert.ok(/left:934px;top:514px/.test(segMarker({ marker: true }, d, 1, N)), 'frame 1 is at (960, 540)');
  assert.equal(segMarker({ marker: true }, undefined, 0, N), '', 'no data, no ring');
  assert.equal(segMarker({ marker: true }, {}, 0, N), '', 'data without positions, no ring');
  const e = { segments: [{ seq: 'title', dur: 4 }, { seq: 'power', dur: 5.5 }, { seq: 'power', dur: 2 }] };
  assert.equal(seqDur(e, 'power'), 5.5);
  assert.equal(seqDur(e, 'twist', 6.5), 6.5);
  assert.equal(seqDur({ segments: [{ seq: 'power', dur: 3.25 }] }, 'power'), 3.25);
  assert.equal(seqDur({ segments: [{ seq: 'power', dur: 0 }] }, 'power'), 0, 'the edit\'s own duration, even none');
});
