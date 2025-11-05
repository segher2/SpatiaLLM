// luma.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { GL } from '@luma.gl/constants';
export function convertSampler(gltfSampler) {
    return {
        addressModeU: convertSamplerWrapMode(gltfSampler.wrapS),
        addressModeV: convertSamplerWrapMode(gltfSampler.wrapT),
        magFilter: convertSamplerMagFilter(gltfSampler.magFilter),
        ...convertSamplerMinFilter(gltfSampler.minFilter)
    };
}
function convertSamplerWrapMode(mode) {
    switch (mode) {
        case 33071:
            return 'clamp-to-edge';
        case 10497:
            return 'repeat';
        case 33648:
            return 'mirror-repeat';
        default:
            return undefined;
    }
}
function convertSamplerMagFilter(mode) {
    switch (mode) {
        case 9728:
            return 'nearest';
        case 9729:
            return 'linear';
        default:
            return undefined;
    }
}
function convertSamplerMinFilter(mode) {
    switch (mode) {
        case 9728:
            return { minFilter: 'nearest' };
        case 9729:
            return { minFilter: 'linear' };
        case 9984:
            return { minFilter: 'nearest', mipmapFilter: 'nearest' };
        case 9985:
            return { minFilter: 'linear', mipmapFilter: 'nearest' };
        case 9986:
            return { minFilter: 'nearest', mipmapFilter: 'linear' };
        case 9987:
            return { minFilter: 'linear', mipmapFilter: 'linear' };
        default:
            return {};
    }
}
//# sourceMappingURL=convert-webgl-sampler.js.map