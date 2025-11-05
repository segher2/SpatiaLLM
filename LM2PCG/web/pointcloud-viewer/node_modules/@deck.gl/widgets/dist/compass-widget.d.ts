import { Widget } from '@deck.gl/core';
import type { Viewport, WidgetPlacement, WidgetProps } from '@deck.gl/core';
export type CompassWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip message. */
    label?: string;
    /** Bearing and pitch reset transition duration in ms. */
    transitionDuration?: number;
};
export declare class CompassWidget extends Widget<CompassWidgetProps> {
    static defaultProps: Required<CompassWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    viewports: {
        [id: string]: Viewport;
    };
    constructor(props?: CompassWidgetProps);
    setProps(props: Partial<CompassWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onViewportChange(viewport: Viewport): void;
    getRotation(viewport?: Viewport): number[];
    handleCompassReset(viewport: Viewport): void;
}
//# sourceMappingURL=compass-widget.d.ts.map