// Node-side browser harness. Process-hygiene pattern adapted from ~/tdf2/e2e/lbm-common.mjs
// (tracked browser, exit/signal handlers, watchdog, SIGKILL fallback). Always run under
// tools/run-guarded.py --gpu, which owns the process group and the machine-wide GPU lock.
//
// Serves the Avis tree read-only over an ephemeral localhost port (WebGPU needs a secure context,
// and localhost counts), plus a write endpoint confined to out/lbm/ used for checkpoints and run
// output, because page.evaluate cannot carry gigabytes.
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
const WRITE_ROOT = path.join(ROOT, 'out', 'lbm');
const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json' };

const browsers = new Set();
const servers = new Set();

function killTree(pid, sig) {
  try { process.kill(-pid, sig); } catch { try { process.kill(pid, sig); } catch { /* gone */ } }
}
export function cleanup() {
  for (const e of browsers) {
    e.browser.close().catch(() => {});
    if (e.pid) {
      killTree(e.pid, 'SIGTERM');
      const pid = e.pid;
      setTimeout(() => killTree(pid, 'SIGKILL'), 1500).unref();
    }
  }
  browsers.clear();
  for (const s of servers) s.close();
  servers.clear();
}
process.on('exit', () => { for (const e of browsers) if (e.pid) killTree(e.pid, 'SIGKILL'); });
for (const sig of ['SIGINT', 'SIGTERM', 'SIGHUP']) process.on(sig, () => { cleanup(); setTimeout(() => process.exit(130), 1800); });
process.on('uncaughtException', (e) => { console.error('uncaught', e); cleanup(); setTimeout(() => process.exit(1), 1800); });

/** Own timeout, shorter than the run-guarded one, so we exit cleanly before the harness SIGKILLs. */
export function watchdog(ms) {
  const t = setTimeout(() => { console.error(`watchdog: exceeded ${ms} ms`); cleanup(); setTimeout(() => process.exit(2), 1800); }, ms);
  t.unref();
  return t;
}

export function startServer() {
  const server = http.createServer((req, res) => {
    const url = new URL(req.url, 'http://x');
    const rel = decodeURIComponent(url.pathname).replace(/^\/+/, '');
    const abs = path.resolve(ROOT, rel);
    if (!abs.startsWith(ROOT + path.sep)) { res.writeHead(403); res.end(); return; }
    if (req.method === 'PUT' || req.method === 'POST') {
      if (!abs.startsWith(WRITE_ROOT + path.sep)) { res.writeHead(403); res.end('write outside out/lbm'); return; }
      fs.mkdirSync(path.dirname(abs), { recursive: true });
      const append = url.searchParams.get('append') === '1';
      const out = fs.createWriteStream(abs, { flags: append ? 'a' : 'w' });
      req.pipe(out);
      out.on('finish', () => { res.writeHead(200); res.end('ok'); });
      out.on('error', (e) => { res.writeHead(500); res.end(String(e)); });
      return;
    }
    if (req.method === 'DELETE' || req.method === 'PATCH') {
      if (!abs.startsWith(WRITE_ROOT + path.sep)) { res.writeHead(403); res.end(); return; }
      try {
        if (req.method === 'DELETE') fs.rmSync(abs, { force: true });
        else fs.renameSync(abs, path.resolve(ROOT, url.searchParams.get('to').replace(/^\/+/, '')));
        res.writeHead(200); res.end('ok');
      } catch (e) { res.writeHead(500); res.end(String(e)); }
      return;
    }
    if (!fs.existsSync(abs) || !fs.statSync(abs).isFile()) { res.writeHead(404); res.end(); return; }
    const size = fs.statSync(abs).size;
    const range = /bytes=(\d+)-(\d+)/.exec(req.headers.range ?? '');
    const headers = { 'Content-Type': MIME[path.extname(abs)] ?? 'application/octet-stream', 'Cache-Control': 'no-store' };
    if (range) {
      const a = Number(range[1]), b = Math.min(Number(range[2]), size - 1);
      res.writeHead(206, { ...headers, 'Content-Length': b - a + 1, 'Content-Range': `bytes ${a}-${b}/${size}` });
      fs.createReadStream(abs, { start: a, end: b }).pipe(res);
    } else {
      res.writeHead(200, { ...headers, 'Content-Length': size });
      fs.createReadStream(abs).pipe(res);
    }
  });
  servers.add(server);
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(`http://127.0.0.1:${server.address().port}`)));
}

/** Launch headless Chromium on the Metal WebGPU backend and open the solver page. */
export async function openPage({ executablePath, onConsole } = {}) {
  const base = await startServer();
  const args = ['--enable-unsafe-webgpu', '--enable-features=WebGPU', '--use-angle=metal', '--enable-gpu',
    '--disable-gpu-watchdog', '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
    '--js-flags=--max-old-space-size=16384'];
  const browser = await chromium.launch({ headless: true, args, executablePath });
  browsers.add({ browser, pid: browser.process?.()?.pid });
  const page = await browser.newPage();
  page.on('pageerror', (e) => console.error('pageerror', e.message));
  page.on('console', (m) => {
    const t = m.type();
    if (onConsole) onConsole(t, m.text());
    else if (t === 'error' || t === 'warning') console.error(`console.${t}:`, m.text().slice(0, 600));
  });
  page.on('crash', () => console.error('page crashed'));
  await page.goto(`${base}/tunnel/lbm/src/page.html`, { waitUntil: 'load' });
  return { browser, page, base };
}
