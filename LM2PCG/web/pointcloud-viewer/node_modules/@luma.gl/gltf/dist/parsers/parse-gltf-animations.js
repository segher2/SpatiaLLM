// luma.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { accessorToTypedArray } from "..//webgl-to-webgpu/convert-webgl-attribute.js";
export function parseGLTFAnimations(gltf) {
    const gltfAnimations = gltf.animations || [];
    return gltfAnimations.map((animation, index) => {
        const name = animation.name || `Animation-${index}`;
        const samplers = animation.samplers.map(({ input, interpolation = 'LINEAR', output }) => ({
            input: accessorToJsArray(gltf.accessors[input]),
            interpolation,
            output: accessorToJsArray(gltf.accessors[output])
        }));
        const channels = animation.channels.map(({ sampler, target }) => ({
            sampler: samplers[sampler],
            target: gltf.nodes[target.node ?? 0],
            path: target.path
        }));
        return { name, channels };
    });
}
//
function accessorToJsArray(accessor) {
    if (!accessor._animation) {
        const { typedArray: array, components } = accessorToTypedArray(accessor);
        if (components === 1) {
            accessor._animation = Array.from(array);
        }
        else {
            // Slice array
            const slicedArray = [];
            for (let i = 0; i < array.length; i += components) {
                slicedArray.push(Array.from(array.slice(i, i + components)));
            }
            accessor._animation = slicedArray;
        }
    }
    return accessor._animation;
}
//# sourceMappingURL=parse-gltf-animations.js.map