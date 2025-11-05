import { GLTFAnimation } from "./animations/animations.js";
type GLTFSingleAnimatorProps = {
    animation: GLTFAnimation;
    startTime?: number;
    playing?: boolean;
    speed?: number;
};
declare class GLTFSingleAnimator {
    animation: GLTFAnimation;
    startTime: number;
    playing: boolean;
    speed: number;
    constructor(props: GLTFSingleAnimatorProps);
    setTime(timeMs: number): void;
}
export type GLTFAnimatorProps = {
    animations: GLTFAnimation[];
};
export declare class GLTFAnimator {
    animations: GLTFSingleAnimator[];
    constructor(props: GLTFAnimatorProps);
    /** @deprecated Use .setTime(). Will be removed (deck.gl is using this) */
    animate(time: number): void;
    setTime(time: number): void;
    getAnimations(): GLTFSingleAnimator[];
}
export {};
//# sourceMappingURL=gltf-animator.d.ts.map