// luma.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { parseGLTF } from "../parsers/parse-gltf.js";
import { GLTFAnimator } from "./gltf-animator.js";
import { parseGLTFAnimations } from "../parsers/parse-gltf-animations.js";
import { deepCopy } from "../utils/deep-copy.js";
export function createScenegraphsFromGLTF(device, gltf, options) {
    gltf = deepCopy(gltf);
    const scenes = parseGLTF(device, gltf, options);
    // Note: There is a nasty dependency on injected nodes in the glTF
    const animations = parseGLTFAnimations(gltf);
    const animator = new GLTFAnimator({ animations });
    return { scenes, animator };
}
//# sourceMappingURL=create-scenegraph-from-gltf.js.map