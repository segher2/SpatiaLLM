import { Widget } from '@deck.gl/core';
import type { Viewport, WidgetProps, WidgetPlacement } from '@deck.gl/core';
export type ZoomWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Button orientation. */
    orientation?: 'vertical' | 'horizontal';
    /** Tooltip message on zoom in button. */
    zoomInLabel?: string;
    /** Tooltip message on zoom out button. */
    zoomOutLabel?: string;
    /** Zoom transition duration in ms. 0 disables the transition */
    transitionDuration?: number;
};
export declare class ZoomWidget extends Widget<ZoomWidgetProps> {
    static defaultProps: Required<ZoomWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    viewports: {
        [id: string]: Viewport;
    };
    constructor(props?: ZoomWidgetProps);
    setProps(props: Partial<ZoomWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onViewportChange(viewport: Viewport): void;
    handleZoom(viewport: Viewport, nextZoom: number): void;
    handleZoomIn(): void;
    handleZoomOut(): void;
    /** @todo - move to deck or widget manager */
    private setViewState;
}
//# sourceMappingURL=zoom-widget.d.ts.map