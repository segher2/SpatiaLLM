import { Widget } from '@deck.gl/core';
import type { Viewport, WidgetPlacement, WidgetProps } from '@deck.gl/core';
export type GimbalWidgetProps = WidgetProps & {
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip message. */
    label?: string;
    /** Width of gimbal lines. */
    strokeWidth?: number;
    /** Transition duration in ms when resetting rotation. */
    transitionDuration?: number;
};
export declare class GimbalWidget extends Widget<GimbalWidgetProps> {
    static defaultProps: Required<GimbalWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    viewports: {
        [id: string]: Viewport;
    };
    constructor(props?: GimbalWidgetProps);
    setProps(props: Partial<GimbalWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onViewportChange(viewport: Viewport): void;
    resetOrbitView(viewport?: Viewport): void;
    getNormalizedRotation(viewport?: Viewport): {
        rotationOrbit: number;
        rotationX: number;
    };
    getRotation(viewState?: any): [number, number];
    getViewId(viewport?: Viewport): string;
    getViewState(viewId: string): import("modules/core/src/lib/view-manager").ViewStateObject<import("@deck.gl/core").View<import("modules/core/src/controllers/transition-manager").TransitionProps, import("modules/core/src/views/view").CommonViewProps<import("modules/core/src/controllers/transition-manager").TransitionProps>>[]>;
    getViewManager(): import("modules/core/src/lib/view-manager").default<import("@deck.gl/core").View<import("modules/core/src/controllers/transition-manager").TransitionProps, import("modules/core/src/views/view").CommonViewProps<import("modules/core/src/controllers/transition-manager").TransitionProps>>[]>;
}
//# sourceMappingURL=gimbal-widget.d.ts.map