import type { Device } from '@luma.gl/core';
import { type ParsedPBRMaterial } from "../pbr/pbr-material.js";
import { type PBREnvironment } from "../pbr/pbr-environment.js";
type GLTFTexture = {
    id: string;
    texture: {
        source: {
            image: any;
        };
        sampler: {
            parameters: any;
        };
    };
    uniformName?: string;
    scale?: number;
    strength?: number;
};
type GLTFPBRMetallicRoughness = {
    baseColorTexture?: GLTFTexture;
    baseColorFactor?: [number, number, number, number];
    metallicRoughnessTexture?: GLTFTexture;
    metallicFactor?: number;
    roughnessFactor?: number;
};
type GLTFPBRMaterial = {
    unlit?: boolean;
    pbrMetallicRoughness?: GLTFPBRMetallicRoughness;
    normalTexture?: GLTFTexture;
    occlusionTexture?: GLTFTexture;
    emissiveTexture?: GLTFTexture;
    emissiveFactor?: [number, number, number];
    alphaMode?: 'MASK' | 'BLEND';
    alphaCutoff?: number;
};
export type ParsePBRMaterialOptions = {
    /** Debug PBR shader */
    pbrDebug?: boolean;
    /** Enable lights */
    lights?: any;
    /** Use tangents */
    useTangents?: boolean;
    /** provide an image based (texture cube) lighting environment */
    imageBasedLightingEnvironment?: PBREnvironment;
};
/**
 * Parses a GLTF material definition into uniforms and parameters for the PBR shader module
 */
export declare function parsePBRMaterial(device: Device, material: GLTFPBRMaterial, attributes: Record<string, any>, options: ParsePBRMaterialOptions): ParsedPBRMaterial;
export {};
//# sourceMappingURL=parse-pbr-material.d.ts.map