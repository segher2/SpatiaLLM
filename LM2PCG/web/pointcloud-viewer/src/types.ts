// Unified manifest types (v1)
export type ManifestItemStyle = {
  pointSize?: number;
  colorMode?: 'file' | 'constant';
  color?: [number, number, number];
  opacity?: number; // for mesh or overlay transparency (0..1)
  wireframe?: boolean; // for mesh rendering
};

export type ManifestItemFilters = {
  labelInclude?: number[];
  labelExclude?: number[];
  pointIdInclude?: number[];
};

export type ManifestItemTransform = {
  translate?: [number, number, number];
  scale?: [number, number, number];
  rotateEulerDeg?: [number, number, number];
};

export type ManifestItem = {
  id: string;
  name: string;
  kind: 'pointcloud' | 'mesh';
  role?: 'shell' | 'cluster' | 'object' | 'room' | string;
  source: { url: string };
  group?: string;
  visible?: boolean;
  style?: ManifestItemStyle;
  filters?: ManifestItemFilters;
  transform?: ManifestItemTransform;
  meta?: Record<string, any>;
};

export type UnifiedManifest = {
  version: 1;
  title?: string;
  description?: string;
  defaults?: ManifestItemStyle & { shellHideLabels?: number[] };
  items: ManifestItem[];
};

export type LoadedPointCloud = {
  length: number;
  attributes: {
    positions: Float32Array;
    colors?: Uint8Array | Uint16Array | Float32Array;
    normals?: Float32Array;
    label?: Int32Array | Uint32Array | Uint16Array | Uint8Array;
    point_id?: Int32Array | Uint32Array | Uint16Array | Uint8Array;
    [key: string]: any;
  };
  meta: {
    hasColor: boolean;
    hasNormal: boolean;
    hasLabel: boolean;
    propertyNames: string[];
  };
};
