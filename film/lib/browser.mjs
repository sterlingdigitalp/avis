// Headless Chromium for the film workstream: a static server over a folder and a browser that cannot outlive this
// process. Run every caller under tools/run-guarded.py, which tracks the whole process tree; on top of that this
// module closes the browser on exit and on SIGTERM/SIGINT/SIGHUP, and arms a self-timeout below the guard's.
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const req = createRequire(path.join(ROOT, 'vendor', 'teardown', 'node_modules', 'x.js'));
const { chromium } = req('playwright');

export const CHROMIUM = process.env.CHROMIUM || path.join(process.env.HOME, 'Library/Caches/ms-playwright/chromium-1248/chrome-mac-arm64/Google Chrome for Testing.app/Contents/MacOS/Google Chrome for Testing');
export const CHROMIUM_ARGS = ['--force_high_performance_gpu', '--ignore-gpu-blocklist', '--enable-gpu-rasterization', '--enable-webgl', '--hide-scrollbars', '--use-angle=metal'];

const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json', '.glb': 'model/gltf-binary', '.jpg': 'image/jpeg', '.png': 'image/png', '.bin': 'application/octet-stream', '.wav': 'audio/wav', '.css': 'text/css', '.woff2': 'font/woff2' };

/** Serve `dir` on 127.0.0.1 at a free port. Returns { url, close }. */
export function serve(dir) {
  const server = http.createServer((rq, rs) => {
    const u = decodeURIComponent(new URL(rq.url, 'http://x').pathname);
    const f = path.join(dir, path.normalize(u).replace(/^(\.\.[/\\])+/, ''));
    if (!f.startsWith(dir) || !fs.existsSync(f) || fs.statSync(f).isDirectory()) {
      rs.writeHead(404);
      rs.end();
      return;
    }
    rs.writeHead(200, { 'content-type': MIME[path.extname(f)] ?? 'application/octet-stream', 'cache-control': 'no-store' });
    fs.createReadStream(f).pipe(rs);
  });
  return new Promise((res) => server.listen(0, '127.0.0.1', () => res({ url: `http://127.0.0.1:${server.address().port}`, close: () => server.close() })));
}

let browser = null;
const cleanups = [];
async function cleanup(code) {
  for (const c of cleanups.splice(0)) {
    try {
      await c();
    } catch {}
  }
  if (browser) {
    const b = browser;
    browser = null;
    await Promise.race([b.close().catch(() => {}), new Promise((r) => setTimeout(r, 3000))]);
  }
  if (code != null) process.exit(code);
}
for (const sig of ['SIGTERM', 'SIGINT', 'SIGHUP']) process.on(sig, () => cleanup(143));
process.on('exit', () => {
  if (browser) browser.close().catch(() => {});
});

/** Launch the browser; `selfTimeoutS` kills this process (and so the browser) if the job hangs. */
export async function launch({ selfTimeoutS = 540 } = {}) {
  const t = setTimeout(() => {
    console.error(`[browser] self-timeout after ${selfTimeoutS}s`);
    cleanup(124);
  }, selfTimeoutS * 1000);
  t.unref();
  browser = await chromium.launch({ executablePath: CHROMIUM, args: CHROMIUM_ARGS, headless: true });
  return browser;
}

export function onCleanup(fn) {
  cleanups.push(fn);
}

export async function done(code = 0) {
  await cleanup(null);
  process.exitCode = code;
}
