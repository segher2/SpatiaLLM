// luma.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { log } from '@luma.gl/core';
import { Matrix4 } from '@math.gl/core';
import { interpolate } from "./animations/interpolate.js";
class GLTFSingleAnimator {
    animation;
    startTime = 0;
    playing = true;
    speed = 1;
    constructor(props) {
        this.animation = props.animation;
        this.animation.name ||= 'unnamed';
        Object.assign(this, props);
    }
    setTime(timeMs) {
        if (!this.playing) {
            return;
        }
        const absTime = timeMs / 1000;
        const time = (absTime - this.startTime) * this.speed;
        this.animation.channels.forEach(({ sampler, target, path }) => {
            interpolate(time, sampler, target, path);
            applyTranslationRotationScale(target, target._node);
        });
    }
}
export class GLTFAnimator {
    animations;
    constructor(props) {
        this.animations = props.animations.map((animation, index) => {
            const name = animation.name || `Animation-${index}`;
            return new GLTFSingleAnimator({
                animation: { name, channels: animation.channels }
            });
        });
    }
    /** @deprecated Use .setTime(). Will be removed (deck.gl is using this) */
    animate(time) {
        log.warn('GLTFAnimator#animate is deprecated. Use GLTFAnimator#setTime instead')();
        this.setTime(time);
    }
    setTime(time) {
        this.animations.forEach(animation => animation.setTime(time));
    }
    getAnimations() {
        return this.animations;
    }
}
// TODO: share with GLTFInstantiator
const scratchMatrix = new Matrix4();
function applyTranslationRotationScale(gltfNode, node) {
    node.matrix.identity();
    if (gltfNode.translation) {
        node.matrix.translate(gltfNode.translation);
    }
    if (gltfNode.rotation) {
        const rotationMatrix = scratchMatrix.fromQuaternion(gltfNode.rotation);
        node.matrix.multiplyRight(rotationMatrix);
    }
    if (gltfNode.scale) {
        node.matrix.scale(gltfNode.scale);
    }
}
//# sourceMappingURL=gltf-animator.js.map