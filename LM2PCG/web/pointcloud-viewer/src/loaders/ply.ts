import { load } from '@loaders.gl/core';
import { PLYLoader } from '@loaders.gl/ply';
import type { LoadedPointCloud } from '../types';

type LoadOptions = {
  filterLabelNotIn?: Set<number>;
  filterLabelIn?: Set<number>;
  dropColor?: boolean; // if true, do not expose COLOR attribute (reduces GPU upload/memory)
};

function toIntArray(arr: any): Int32Array | Uint32Array | Uint16Array | Uint8Array | undefined {
  if (!arr) return undefined;
  if (arr instanceof Int32Array || arr instanceof Uint32Array || arr instanceof Uint16Array || arr instanceof Uint8Array) return arr;
  if (Array.isArray(arr)) return new Int32Array(arr);
  return undefined;
}

export async function loadPly(url: string, options?: LoadOptions): Promise<LoadedPointCloud> {
  // Disable worker to simplify debugging and ensure full parse happens in main thread
  const data: any = await load(url, PLYLoader, { worker: false });

  // loaders.gl PLY returns either {attributes: {POSITION, COLOR_0, NORMAL}} or flat typed arrays depending on version
  // Normalize into our structure
  const attributes: any = {};
  let length = 0;

  // Try accessors-style
  const attr = data.attributes || data;
  // Debug visibility into loader output shape
  try {
    const keys = Object.keys(data || {});
    const attrKeys = Object.keys(attr || {});
    // Attempt to capture lengths if present
    const posLen = attr?.POSITION?.value?.length ?? attr?.positions?.length ?? 0;
    const colLen = attr?.COLOR_0?.value?.length ?? attr?.colors?.length ?? 0;
    // eslint-disable-next-line no-console
    console.log('[PLY] loaded', url, { keys, attrKeys, posLen, colLen });
  } catch {}
  if (attr.POSITION) {
    attributes.positions = attr.POSITION.value as Float32Array;
    length = attr.POSITION.value.length / (attr.POSITION.size || 3);
  } else if (attr.positions) {
    attributes.positions = attr.positions as Float32Array;
    length = attributes.positions.length / 3;
  }

  if (!options?.dropColor) {
    if (attr.COLOR_0?.value) attributes.colors = attr.COLOR_0.value as any;
    if (attr.colors) attributes.colors = attr.colors as any;
  }
  if (attr.NORMAL?.value) attributes.normals = attr.NORMAL.value as Float32Array;
  if (attr.normals) attributes.normals = attr.normals as Float32Array;

  // Custom properties: try common names
  const label = toIntArray(attr.label?.value ?? attr.label ?? attr.Label);
  if (label) attributes.label = label;
  const point_id = toIntArray(attr.point_id?.value ?? attr.point_id ?? attr.id ?? attr.vertex_id);
  if (point_id) attributes.point_id = point_id;

  const propertyNames: string[] = Object.keys(attr)
    .map((k) => (attr[k]?.value ? k : k))
    .filter((k) => typeof attr[k] !== 'function');

  // Optional filtering by label include/exclude
  if ((options?.filterLabelNotIn || options?.filterLabelIn) && attributes.label) {
    const keep = new Uint8Array(length);
    let kept = 0;
    for (let i = 0; i < length; i++) {
      const lb = (attributes.label as any)[i] as number;
      const inInclude = options.filterLabelIn ? options.filterLabelIn.has(lb) : true;
      const notInExclude = options.filterLabelNotIn ? !options.filterLabelNotIn.has(lb) : true;
      if (inInclude && notInExclude) {
        keep[i] = 1;
        kept++;
      }
    }

    if (kept < length) {
      const remap = new Int32Array(kept);
      let w = 0;
      for (let i = 0; i < length; i++) if (keep[i]) remap[w++] = i;

      const filterArray = (typed: any, size = 1) => {
        if (!typed) return undefined;
        const ctor = Object.getPrototypeOf(typed).constructor;
        const out = new ctor(kept * size);
        for (let i = 0; i < kept; i++) {
          const srcIndex = remap[i] * size;
          const dstIndex = i * size;
          for (let j = 0; j < size; j++) out[dstIndex + j] = typed[srcIndex + j];
        }
        return out;
      };

      attributes.positions = filterArray(attributes.positions, 3)!;
      if (attributes.colors) attributes.colors = filterArray(attributes.colors, 3);
      if (attributes.normals) attributes.normals = filterArray(attributes.normals, 3);
      if (attributes.label) attributes.label = filterArray(attributes.label, 1);
      if (attributes.point_id) attributes.point_id = filterArray(attributes.point_id, 1);
      length = kept;
    }
  }

  return {
    length,
    attributes,
    meta: {
      hasColor: Boolean(attributes.colors),
      hasNormal: Boolean(attributes.normals),
      hasLabel: Boolean(attributes.label),
      propertyNames
    }
  };
}
