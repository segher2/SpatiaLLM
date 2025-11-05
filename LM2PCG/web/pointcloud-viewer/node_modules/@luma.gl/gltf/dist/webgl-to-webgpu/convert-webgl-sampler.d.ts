import type { SamplerProps } from '@luma.gl/core';
import { GL } from '@luma.gl/constants';
type GLTFSampler = {
    wrapS?: GL.CLAMP_TO_EDGE | GL.REPEAT | GL.MIRRORED_REPEAT;
    wrapT?: GL.CLAMP_TO_EDGE | GL.REPEAT | GL.MIRRORED_REPEAT;
    magFilter?: GL.NEAREST | GL.LINEAR;
    minFilter?: GL.NEAREST | GL.LINEAR | GL.NEAREST_MIPMAP_NEAREST | GL.LINEAR_MIPMAP_NEAREST | GL.NEAREST_MIPMAP_LINEAR | GL.LINEAR_MIPMAP_LINEAR;
};
export declare function convertSampler(gltfSampler: GLTFSampler): SamplerProps;
export {};
//# sourceMappingURL=convert-webgl-sampler.d.ts.map