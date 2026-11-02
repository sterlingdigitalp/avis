// Lets node resolve the bare 'three' imports of sims/avis-shared/*.js against the vendored Teardown kit, so the GLB
// export and the pose bake run the exact module the page and the film bundle. Use: node --import ./film/lib/three-hooks.mjs
import { registerHooks } from 'node:module';
import { pathToFileURL } from 'node:url';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const THREE = join(ROOT, 'vendor', 'teardown', 'node_modules', 'three');

registerHooks({
  resolve(specifier, context, next) {
    if (specifier === 'three') return { url: pathToFileURL(join(THREE, 'build', 'three.module.js')).href, shortCircuit: true };
    if (specifier.startsWith('three/addons/')) return { url: pathToFileURL(join(THREE, 'examples', 'jsm', specifier.slice('three/addons/'.length))).href, shortCircuit: true };
    if (specifier.startsWith('three/examples/jsm/')) return { url: pathToFileURL(join(THREE, specifier.slice('three/'.length))).href, shortCircuit: true };
    return next(specifier, context);
  },
});
