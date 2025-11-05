// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
const colorWGSL = /* WGSL */ `

struct ColorUniforms {
  opacity: f32,
};

var<private> color: ColorUniforms = ColorUniforms(1.0);
// TODO (kaapp) avoiding binding index collisions to handle layer opacity 
// requires some thought.
// @group(0) @binding(0) var<uniform> color: ColorUniforms;

@must_use
fn deckgl_premultiplied_alpha(fragColor: vec4<f32>) -> vec4<f32> {
    return vec4(fragColor.rgb * fragColor.a, fragColor.a); 
};
`;
export default {
    name: 'color',
    dependencies: [],
    source: colorWGSL,
    getUniforms: (_props) => {
        // TODO (kaapp) Handle layer opacity
        // apply gamma to opacity to make it visually "linear"
        // TODO - v10: use raw opacity?
        // opacity: Math.pow(props.opacity!, 1 / 2.2)
        return {};
    },
    uniformTypes: {
        opacity: 'f32'
    }
    // @ts-ignore TODO v9.1
};
//# sourceMappingURL=color.js.map