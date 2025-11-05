export type PointCloudProps = {
    radiusPixels: number;
    sizeUnits: number;
};
export declare const pointCloudUniforms: {
    readonly name: "pointCloud";
    readonly source: "struct PointCloudUniforms {\n  radiusPixels: f32,\n  sizeUnits: i32,\n};\n\n@group(0) @binding(3)\nvar<uniform> pointCloud: PointCloudUniforms;\n";
    readonly vs: "uniform pointCloudUniforms {\n  float radiusPixels;\n  highp int sizeUnits;\n} pointCloud;\n";
    readonly fs: "uniform pointCloudUniforms {\n  float radiusPixels;\n  highp int sizeUnits;\n} pointCloud;\n";
    readonly uniformTypes: {
        readonly radiusPixels: "f32";
        readonly sizeUnits: "i32";
    };
};
//# sourceMappingURL=point-cloud-layer-uniforms.d.ts.map