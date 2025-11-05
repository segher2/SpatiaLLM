import { Widget, type WidgetProps, type WidgetPlacement } from '@deck.gl/core';
export type FullscreenWidgetProps = WidgetProps & {
    id?: string;
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip message when out of fullscreen. */
    enterLabel?: string;
    /** Tooltip message when fullscreen. */
    exitLabel?: string;
    /**
     * A compatible DOM element which should be made full screen. By default, the map container element will be made full screen.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/requestFullScreen#Compatible_elements
     */
    container?: HTMLElement;
};
export declare class FullscreenWidget extends Widget<FullscreenWidgetProps> {
    static defaultProps: Required<FullscreenWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    fullscreen: boolean;
    constructor(props?: FullscreenWidgetProps);
    onAdd(): void;
    onRemove(): void;
    onRenderHTML(rootElement: HTMLElement): void;
    setProps(props: Partial<FullscreenWidgetProps>): void;
    getContainer(): HTMLElement;
    onFullscreenChange(): void;
    handleClick(): Promise<void>;
    requestFullscreen(): Promise<void>;
    exitFullscreen(): Promise<void>;
    togglePseudoFullscreen(): void;
}
//# sourceMappingURL=fullscreen-widget.d.ts.map