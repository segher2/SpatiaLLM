import { load } from '@loaders.gl/core';
import { PLYLoader } from '@loaders.gl/ply';

export type LoadedMesh = {
  positions: Float32Array;
  normals?: Float32Array;
  indices?: Uint16Array | Uint32Array;
};

// Load a PLY as a mesh suitable for deck.gl SimpleMeshLayer
export async function loadMeshPly(url: string): Promise<LoadedMesh> {
  const data: any = await load(url, PLYLoader, { worker: false });
  const attr = data.attributes || data;
  // Debug: surface keys
  try {
    const keys = Object.keys(data || {});
    const attrKeys = Object.keys(attr || {});
    // eslint-disable-next-line no-console
    console.log('[PLY-MESH] loaded', url, { keys, attrKeys, hasIndices: Boolean(data.indices || attr.indices) });
  } catch {}
  const positions: Float32Array | undefined = (attr.POSITION?.value ?? attr.positions) as Float32Array | undefined;
  if (!positions) throw new Error('PLY mesh missing POSITION/positions');
  const normals: Float32Array | undefined = (attr.NORMAL?.value ?? attr.normals) as Float32Array | undefined;
  const rawIdx: any = (data.indices || attr.indices);
  const indices: Uint16Array | Uint32Array | undefined = (rawIdx?.value ? rawIdx.value : rawIdx) as any;
  try {
    // eslint-disable-next-line no-console
    console.log('[PLY-MESH] meta', { pos: positions.length / 3, idx: indices ? (indices.length / 3) : 0 });
  } catch {}
  return { positions, normals, indices };
}
