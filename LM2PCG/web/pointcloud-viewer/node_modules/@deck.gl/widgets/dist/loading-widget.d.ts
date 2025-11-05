import type { WidgetPlacement, Layer, WidgetProps } from '@deck.gl/core';
import { Widget } from '@deck.gl/core';
/** Properties for the LoadingWidget */
export type LoadingWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views */
    viewId?: string | null;
    /** Tooltip message when loading */
    label?: string;
};
/**
 * A non-interactive widget that shows a loading spinner if any layers are loading data
 */
export declare class LoadingWidget extends Widget<LoadingWidgetProps> {
    static defaultProps: Required<LoadingWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    loading: boolean;
    constructor(props?: LoadingWidgetProps);
    setProps(props: Partial<LoadingWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onRedraw({ layers }: {
        layers: Layer[];
    }): void;
    handleClick(): void;
}
//# sourceMappingURL=loading-widget.d.ts.map