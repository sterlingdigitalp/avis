// Writing design files safely (node only). Other workstreams read data/designs/*.json while sweeps run, and the
// tunnel records each design's sha256, so a design file is never seen half-written and every rewrite is numbered.
import { existsSync, readFileSync, writeFileSync, renameSync } from 'node:fs';
import { createHash } from 'node:crypto';

/** sha256 of a file's bytes, hex. */
export function fileSha256(path) {
  return createHash('sha256').update(readFileSync(path)).digest('hex');
}

/**
 * Write a design atomically: to a temporary file in the same directory, then rename over the target (atomic on
 * one filesystem). `version` counts rewrites of that path: one more than the file it replaces, 1 for a new file.
 * `supersedes_sha256` records the bytes it replaced. Returns { version, sha256 }.
 */
export function writeDesignAtomic(path, design) {
  let prevVersion = 0;
  let prevSha = null;
  if (existsSync(path)) {
    try {
      prevVersion = +(JSON.parse(readFileSync(path, 'utf8')).version ?? 0) || 0;
    } catch {
      prevVersion = 0;
    }
    prevSha = fileSha256(path);
  }
  design.version = prevVersion + 1;
  design.version_written = new Date().toISOString();
  design.supersedes_sha256 = prevSha;
  const tmp = `${path}.tmp-${process.pid}-${Date.now()}`;
  writeFileSync(tmp, JSON.stringify(design, null, 1));
  renameSync(tmp, path);
  return { version: design.version, sha256: fileSha256(path) };
}
