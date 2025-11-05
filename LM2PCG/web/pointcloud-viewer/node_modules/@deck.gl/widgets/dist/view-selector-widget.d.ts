import { Widget, type WidgetProps, type WidgetPlacement } from '@deck.gl/core';
/** The available view modes */
export type ViewMode = 'single' | 'split-horizontal' | 'split-vertical';
/** Properties for the ViewSelectorWidget */
export type ViewSelectorWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip label */
    label?: string;
    /** The initial view mode. Defaults to 'single'. */
    initialViewMode?: ViewMode;
    /** Callback invoked when the view mode changes */
    onViewModeChange?: (mode: ViewMode) => void;
};
/**
 * A widget that renders a popup menu for selecting a view mode.
 * It displays a button with the current view mode icon. Clicking the button
 * toggles a popup that shows three icons for:
 * - Single view
 * - Two views, split horizontally
 * - Two views, split vertically
 */
export declare class ViewSelectorWidget extends Widget<ViewSelectorWidgetProps> {
    static defaultProps: Required<ViewSelectorWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    viewMode: ViewMode;
    constructor(props?: ViewSelectorWidgetProps);
    setProps(props: Partial<ViewSelectorWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    handleSelectMode: (viewMode: ViewMode) => void;
}
//# sourceMappingURL=view-selector-widget.d.ts.map