#!/usr/bin/env node
// The bird as a GLB for Blender, from the current design: out/film/bird/<design>.glb plus <design>.glb.json (the
// node list and the design's sha256). Same bird.js as the page. Rest pose: wings level (phi = 0), no twist.
// Node names follow env/import_flight.py: one root `body`, wing segments wing_R_0..1 and wing_L_0..1 with origins
// on their joints, parented in chain. Every other moving node is named and listed for the pose bake.
// With a livery (materials.js LIVERIES; AVIS_LIVERY=<name> or a second argument) the covering carries per-surface
// vertex colours and the file is <design>-<livery>.glb, its meta naming the livery so Blender pigments the film.
//   node --import ./film/lib/three-hooks.mjs film/export_glb.mjs [design] [livery]
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { ROOT } from '../sims/avis-shared/node-load.js';
import { nodeBird, posedNodes } from './lib/bird-node.mjs';

// GLTFExporter reads blobs through FileReader, which node does not have.
if (typeof globalThis.FileReader === 'undefined') {
  globalThis.FileReader = class {
    readAsArrayBuffer(blob) {
      blob.arrayBuffer().then((b) => {
        this.result = b;
        this.onloadend?.();
        this.onload?.({ target: this });
      });
    }
    readAsDataURL(blob) {
      blob.arrayBuffer().then((b) => {
        this.result = `data:${blob.type || 'application/octet-stream'};base64,${Buffer.from(b).toString('base64')}`;
        this.onloadend?.();
        this.onload?.({ target: this });
      });
    }
  };
}

export async function exportGlb(name = 'larus-gl-2', outDir = join(ROOT, 'out', 'film', 'bird'), { livery = null } = {}) {
  const { bird, design, sha256, rest, livery: l } = nodeBird(name, { livery });
  bird.update(rest);
  bird.root.updateMatrixWorld(true);
  const glb = await new GLTFExporter().parseAsync(bird.root, { binary: true, onlyVisible: true, trs: false });
  mkdirSync(outDir, { recursive: true });
  const file = join(outDir, `${name}${l ? `-${l.name}` : ''}.glb`);
  writeFileSync(file, Buffer.from(glb));
  const meta = {
    design: name, design_version: design.version, design_sha256: sha256, rest,
    posed: posedNodes(bird).map((n) => n.name),
    deform: bird.deform.map((m) => ({ name: m.name, count: m.geometry.attributes.position.count })),
    materials: [...new Set(bird.meshes.map((m) => m.userData.matName))],
    parts: bird.parts.map((p) => ({ name: p.name, id: p.id, role: p.role, subsystem: p.subsystem, nodes: p.objects.map((o) => o.name) })),
    livery: l,
  };
  writeFileSync(`${file}.json`, JSON.stringify(meta, null, 1));
  return { file, bytes: glb.byteLength, meta };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const r = await exportGlb(process.argv[2] ?? 'larus-gl-2', undefined, { livery: process.argv[3] ?? process.env.AVIS_LIVERY ?? null });
  console.log(`${r.file}  ${(r.bytes / 1024).toFixed(0)} KB  ${r.meta.posed.length} posed nodes, ${r.meta.deform.length} deforming meshes, design v${r.meta.design_version}`);
}
