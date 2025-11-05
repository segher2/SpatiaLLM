import { Widget } from '@deck.gl/core';
import type { WidgetPlacement, Deck, WidgetProps } from '@deck.gl/core';
/** Properties for the FpsWidget. */
export type FpsWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
};
/**
 * Displays the average frames per second reported by the Deck instance.
 */
export declare class FpsWidget extends Widget<FpsWidgetProps> {
    static defaultProps: Required<FpsWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    private _lastFps;
    constructor(props?: FpsWidgetProps);
    setProps(props: Partial<FpsWidgetProps>): void;
    onAdd({}: {
        deck: Deck<any>;
        viewId: string | null;
    }): void;
    onRenderHTML(rootElement: HTMLElement): void;
    _animate(): void;
    _getFps(): number;
}
//# sourceMappingURL=fps-widget.d.ts.map