import { Device } from '@luma.gl/core';
import { GroupNode } from '@luma.gl/engine';
import { GLTFPostprocessed } from '@loaders.gl/gltf';
import { type ParseGLTFOptions } from "../parsers/parse-gltf.js";
import { GLTFAnimator } from "./gltf-animator.js";
export declare function createScenegraphsFromGLTF(device: Device, gltf: GLTFPostprocessed, options?: ParseGLTFOptions): {
    scenes: GroupNode[];
    animator: GLTFAnimator;
};
//# sourceMappingURL=create-scenegraph-from-gltf.d.ts.map