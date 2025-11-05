import { Widget, type WidgetProps } from '@deck.gl/core';
/** Properties for the SplitterWidget */
export type SplitterWidgetProps = WidgetProps & {
    /** The view id for the first (resizable) view */
    viewId1: string;
    /** The view id for the second view */
    viewId2: string;
    /** Orientation of the splitter: vertical (default) or horizontal */
    orientation?: 'vertical' | 'horizontal';
    /** The initial split percentage (0 to 1) for the first view, default 0.5 */
    initialSplit?: number;
    /** Callback invoked when the splitter is dragged with the new split value */
    onChange?: (newSplit: number) => void;
    /** Callback invoked when dragging starts */
    onDragStart?: () => void;
    /** Callback invoked when dragging ends */
    onDragEnd?: () => void;
};
/**
 * A draggable splitter widget that appears as a vertical or horizontal line
 * across the deck.gl canvas. It positions itself based on the split percentage
 * of the first view and provides callbacks when dragged.
 */
export declare class SplitterWidget extends Widget<SplitterWidgetProps> {
    static defaultProps: Required<SplitterWidgetProps>;
    className: string;
    placement: "fill";
    constructor(props: SplitterWidgetProps);
    setProps(props: Partial<SplitterWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
}
//# sourceMappingURL=splitter-widget.d.ts.map