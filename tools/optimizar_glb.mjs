// optimizar_glb.mjs - deja un GLB del CAD listo para la vista 3D.
//
// Uso (una vez: npm i @gltf-transform/core@4 @gltf-transform/extensions@4
//      @gltf-transform/functions@4 meshoptimizer sharp):
//
//   node optimizar_glb.mjs ../../APOLO.glb ../static/assets/modelos/apolo.glb
//
// Junta las piezas del CAD en pocas mallas (cientos de llamadas de dibujo a
// un punado), funde materiales de color liso en una paleta, simplifica con
// un error de 0.05% del tamaño, pasa las texturas a WebP de 1024 y comprime
// con meshopt. NO cambia la escala: glTF va en metros y la vista 3D los pasa
// a px con un solo factor (PX_POR_METRO_3D en config.js).
//
// Quita KHR_materials_transmission: en three.js obliga a renderizar la
// escena otra vez cada cuadro, y en estos modelos son piezas diminutas.
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { dedup, instance, palette, flatten, join, weld, simplify, prune, textureCompress, meshopt, inspect } from '@gltf-transform/functions';
import { MeshoptEncoder, MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'node:fs';

const [,, entrada, salida] = process.argv;
await MeshoptEncoder.ready; await MeshoptDecoder.ready; await MeshoptSimplifier.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS)
  .registerDependencies({ 'meshopt.encoder': MeshoptEncoder, 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(entrada);
const root = doc.getRoot();

const resumen = d => {
  const r = d.getRoot();
  let tris = 0, prims = 0;
  r.listMeshes().forEach(m => m.listPrimitives().forEach(p => { prims++; const i = p.getIndices(); const pos = p.getAttribute('POSITION'); tris += (i ? i.getCount() : pos.getCount()) / 3; }));
  return { mallas: r.listMeshes().length, primitivas: prims, materiales: r.listMaterials().length, nodos: r.listNodes().length, texturas: r.listTextures().length, tris: Math.round(tris) };
};
console.log('antes  ', resumen(doc));

for (const ext of root.listExtensionsUsed()) {
  if (ext.extensionName === 'KHR_materials_transmission') {
    root.listMaterials().forEach(m => m.setExtension('KHR_materials_transmission', null));
    ext.dispose();
  }
}

await doc.transform(
  dedup(),
  instance({ min: 5 }),
  palette({ min: 5 }),
  flatten(),
  join(),
  weld(),
  simplify({ simplifier: MeshoptSimplifier, ratio: 0, error: 0.0005 }),
  prune(),
  textureCompress({ encoder: sharp, targetFormat: 'webp', resize: [1024, 1024] }),
  meshopt({ encoder: MeshoptEncoder, level: 'medium' }),
);
console.log('despues', resumen(doc));
await io.write(salida, doc);
console.log('bytes', fs.statSync(entrada).size, '->', fs.statSync(salida).size, 'ext', root.listExtensionsUsed().map(e => e.extensionName));
