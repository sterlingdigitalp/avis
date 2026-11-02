#!/usr/bin/env node
// Serves out/film/site on localhost so the live page can load its backdrop and logs (a page opened from file://
// cannot fetch them; it then flies in the studio only).
//   node film/serve.mjs [port]     then open http://127.0.0.1:<port>/avis-live.html
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const SITE = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'out', 'film', 'site');
const MIME = { '.html': 'text/html', '.json': 'application/json', '.glb': 'model/gltf-binary', '.jpg': 'image/jpeg', '.png': 'image/png', '.bin': 'application/octet-stream' };
const port = +(process.argv[2] ?? 8417);
http.createServer((rq, rs) => {
  const u = decodeURIComponent(new URL(rq.url, 'http://x').pathname);
  const f = path.join(SITE, path.normalize(u === '/' ? '/avis-live.html' : u).replace(/^(\.\.[/\\])+/, ''));
  if (!f.startsWith(SITE) || !fs.existsSync(f) || fs.statSync(f).isDirectory()) {
    rs.writeHead(404);
    rs.end();
    return;
  }
  rs.writeHead(200, { 'content-type': MIME[path.extname(f)] ?? 'application/octet-stream' });
  fs.createReadStream(f).pipe(rs);
}).listen(port, '127.0.0.1', () => console.log(`http://127.0.0.1:${port}/avis-live.html`));
