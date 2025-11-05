import { Widget } from '@deck.gl/core';
import type { Deck, PickingInfo, Viewport, WidgetProps } from '@deck.gl/core';
export type InfoWidgetProps = WidgetProps & {
    /** View to attach to and interact with. Required when using multiple views */
    viewId?: string | null;
    /** Determines the interaction mode of the widget */
    mode: 'click' | 'hover' | 'static';
    /** Function to generate the popup contents from the selected element */
    getTooltip?: (info: PickingInfo, widget: InfoWidget) => InfoWidgetProps | null;
    /** Position at which to place popup (clicked point: [longitude, latitude]) */
    position: [number, number];
    /** Text content of popup */
    text?: string;
    /** Visibility of info widget */
    visible?: boolean;
    /** Minimum offset (in pixels) to keep the popup away from the canvas edges. */
    minOffset?: number;
    /** Callback triggered when the widget is clicked. */
    onClick?: (widget: InfoWidget, info: PickingInfo) => boolean;
};
export declare class InfoWidget extends Widget<InfoWidgetProps> {
    static defaultProps: Required<InfoWidgetProps>;
    className: string;
    placement: "fill";
    viewport?: Viewport;
    constructor(props: InfoWidgetProps);
    setProps(props: Partial<InfoWidgetProps>): void;
    onCreateRootElement(): HTMLDivElement;
    onViewportChange(viewport: any): void;
    onHover(info: PickingInfo): void;
    onClick(info: PickingInfo): boolean;
    onAdd({ deck, viewId }: {
        deck: Deck<any>;
        viewId: string | null;
    }): void;
    onRenderHTML(rootElement: HTMLElement): void;
}
//# sourceMappingURL=info-widget.d.ts.map