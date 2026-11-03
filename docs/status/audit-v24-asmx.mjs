import fs from 'node:fs';
const root = new URL('../..', import.meta.url).pathname;
const { fullAssembly } = await import(root + '/src/design/clearance.js');
const d = JSON.parse(fs.readFileSync(root + '/data/designs/larus-gl-2.json', 'utf8'));
const catalog = JSON.parse(fs.readFileSync(root + '/data/parts/catalog.json', 'utf8'));
const asm = fullAssembly(d, catalog);
const pat = new RegExp(process.argv[2] ?? '.');
const A = asm.asm; const parts = A.parts;
const list = Array.isArray(parts) ? parts : Object.values(parts);
console.log('keys', Object.keys(asm).slice(0,20).join(','), 'n', list.length);
for (const p of list) { if (!pat.test(p.id)) continue; for (const s of p.solids) console.log(p.id.padEnd(48), (s.name||'').padEnd(22), s.x.map(v => (v*1000).toFixed(2)).join('..'), s.shape ? s.shape.kind + (s.shape.ro? ' ro'+(s.shape.ro*1000).toFixed(2)+' ri'+((s.shape.ri||0)*1000).toFixed(2):'') : 'moving'); }
if (process.argv[3]) for (const j of A.joins) { const s = JSON.stringify(j); if (pat.test(s)) console.log('JOIN', s.slice(0, 200)); }
