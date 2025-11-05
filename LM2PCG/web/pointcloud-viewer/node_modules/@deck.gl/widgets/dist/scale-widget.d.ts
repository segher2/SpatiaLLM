import type { WidgetPlacement, Viewport, WidgetProps } from '@deck.gl/core';
import { Widget } from '@deck.gl/core';
export type ScaleWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'bottom-left'. */
    placement?: WidgetPlacement;
    /** Label for the scale widget */
    label?: string;
    /** View to attach to and interact with. Required when using multiple views */
    viewId?: string | null;
};
/**
 * A scale widget that displays a Google Maps–like scale indicator.
 * Instead of text inside a div, this widget renders an SVG that contains a horizontal line
 * with two vertical tick marks (extending upward from the line only) and a pretty distance label
 * positioned to the left of the line. The horizontal line’s length is computed from a “nice”
 * candidate distance (e.g. 200, 500, 1000 m, etc.) so that its pixel width is between 100 and 200.
 */
export declare class ScaleWidget extends Widget<ScaleWidgetProps> {
    static defaultProps: Required<ScaleWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    scaleWidth: number;
    scaleValue: number;
    scaleText: string;
    constructor(props?: ScaleWidgetProps);
    setProps(props: Partial<ScaleWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onViewportChange(viewport: Viewport): void;
    handleClick(): void;
}
//# sourceMappingURL=scale-widget.d.ts.map