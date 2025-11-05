import { Device } from '@luma.gl/core';
import { Geometry, ModelNode, type ModelProps } from '@luma.gl/engine';
import { type ParsedPBRMaterial } from "../pbr/pbr-material.js";
export type CreateGLTFModelOptions = {
    id?: string;
    vertexCount?: number;
    geometry: Geometry;
    parsedPPBRMaterial: ParsedPBRMaterial;
    modelOptions?: Partial<ModelProps>;
};
/** Creates a luma.gl Model from GLTF data*/
export declare function createGLTFModel(device: Device, options: CreateGLTFModelOptions): ModelNode;
//# sourceMappingURL=create-gltf-model.d.ts.map