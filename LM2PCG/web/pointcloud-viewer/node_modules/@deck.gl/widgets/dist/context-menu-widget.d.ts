import { Widget } from '@deck.gl/core';
import type { Deck, PickingInfo, WidgetProps } from '@deck.gl/core';
export type ContextWidgetMenuItem = {
    label: string;
    key: string;
};
export type ContextMenuWidgetProps = WidgetProps & {
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Controls visibility of the context menu */
    visible?: boolean;
    /** Screen position at which to place the menu */
    position: {
        x: number;
        y: number;
    };
    /** Items to render */
    menuItems: ContextWidgetMenuItem[];
    /** Provide menu items for the menu given the picked object */
    getMenuItems: (info: PickingInfo, widget: ContextMenuWidget) => ContextWidgetMenuItem[] | null;
    /** Callback with the selected item */
    onMenuItemSelected?: (key: string, pickInfo: PickingInfo | null) => void;
};
export declare class ContextMenuWidget extends Widget<ContextMenuWidgetProps> {
    static defaultProps: Required<ContextMenuWidgetProps>;
    className: string;
    placement: "fill";
    pickInfo: PickingInfo | null;
    constructor(props: ContextMenuWidgetProps);
    onAdd({ deck }: {
        deck: Deck<any>;
    }): HTMLDivElement;
    onRenderHTML(rootElement: HTMLElement): void;
    handleContextMenu(srcEvent: MouseEvent): boolean;
    hide(): void;
}
//# sourceMappingURL=context-menu-widget.d.ts.map