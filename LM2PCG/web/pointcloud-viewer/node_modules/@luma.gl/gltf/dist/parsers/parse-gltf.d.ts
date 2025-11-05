import { Device } from '@luma.gl/core';
import { GroupNode, type ModelProps } from '@luma.gl/engine';
import { type GLTFPostprocessed } from '@loaders.gl/gltf';
import { type PBREnvironment } from "../pbr/pbr-environment.js";
export type ParseGLTFOptions = {
    modelOptions?: Partial<ModelProps>;
    pbrDebug?: boolean;
    imageBasedLightingEnvironment?: PBREnvironment;
    lights?: boolean;
    useTangents?: boolean;
};
/**
 * GLTF instantiator for luma.gl
 * Walks the parsed and resolved glTF structure and builds a luma.gl scenegraph
 */
export declare function parseGLTF(device: Device, gltf: GLTFPostprocessed, options_?: ParseGLTFOptions): GroupNode[];
//# sourceMappingURL=parse-gltf.d.ts.map