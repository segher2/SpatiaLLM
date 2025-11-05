// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
const wgslUniformBlock = /* wgsl */ `\
struct PointCloudUniforms {
  radiusPixels: f32,
  sizeUnits: i32,
};

@group(0) @binding(3)
var<uniform> pointCloud: PointCloudUniforms;
`;
const glslUniformBlock = `\
uniform pointCloudUniforms {
  float radiusPixels;
  highp int sizeUnits;
} pointCloud;
`;
export const pointCloudUniforms = {
    name: 'pointCloud',
    source: wgslUniformBlock,
    vs: glslUniformBlock,
    fs: glslUniformBlock,
    uniformTypes: {
        radiusPixels: 'f32',
        sizeUnits: 'i32'
    }
};
//# sourceMappingURL=point-cloud-layer-uniforms.js.map