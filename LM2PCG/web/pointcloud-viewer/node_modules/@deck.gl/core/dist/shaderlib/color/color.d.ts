export type ColorProps = {
    /**
     * Opacity of the layer, between 0 and 1. Default 1.
     */
    opacity?: number;
};
export type ColorUniforms = {
    opacity?: number;
};
declare const _default: {
    readonly name: "color";
    readonly dependencies: [];
    readonly source: "\n\nstruct ColorUniforms {\n  opacity: f32,\n};\n\nvar<private> color: ColorUniforms = ColorUniforms(1.0);\n// TODO (kaapp) avoiding binding index collisions to handle layer opacity \n// requires some thought.\n// @group(0) @binding(0) var<uniform> color: ColorUniforms;\n\n@must_use\nfn deckgl_premultiplied_alpha(fragColor: vec4<f32>) -> vec4<f32> {\n    return vec4(fragColor.rgb * fragColor.a, fragColor.a); \n};\n";
    readonly getUniforms: (_props: Partial<ColorProps>) => {};
    readonly uniformTypes: {
        readonly opacity: "f32";
    };
};
export default _default;
//# sourceMappingURL=color.d.ts.map