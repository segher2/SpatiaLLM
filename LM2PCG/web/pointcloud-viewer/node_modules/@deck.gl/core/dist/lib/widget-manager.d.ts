import type Deck from "./deck.js";
import type Viewport from "../viewports/viewport.js";
import type { PickingInfo } from "./picking/pick-info.js";
import type { MjolnirPointerEvent, MjolnirGestureEvent } from 'mjolnir.js';
import type Layer from "./layer.js";
import { Widget } from "./widget.js";
declare const PLACEMENTS: {
    readonly 'top-left': {
        readonly top: 0;
        readonly left: 0;
    };
    readonly 'top-right': {
        readonly top: 0;
        readonly right: 0;
    };
    readonly 'bottom-left': {
        readonly bottom: 0;
        readonly left: 0;
    };
    readonly 'bottom-right': {
        readonly bottom: 0;
        readonly right: 0;
    };
    readonly fill: {
        readonly top: 0;
        readonly left: 0;
        readonly bottom: 0;
        readonly right: 0;
    };
};
export type WidgetPlacement = keyof typeof PLACEMENTS;
export type WidgetManagerProps = {
    deck: Deck<any>;
    parentElement?: HTMLElement | null;
};
export declare class WidgetManager {
    deck: Deck<any>;
    parentElement?: HTMLElement | null;
    /** Widgets added via the imperative API */
    private defaultWidgets;
    /** Widgets received from the declarative API */
    private widgets;
    /** Resolved widgets from both imperative and declarative APIs */
    private resolvedWidgets;
    /** Mounted HTML containers */
    private containers;
    /** Viewport provided to widget on redraw */
    private lastViewports;
    constructor({ deck, parentElement }: WidgetManagerProps);
    getWidgets(): Widget[];
    /** Declarative API to configure widgets */
    setProps(props: {
        widgets?: (Widget | null | undefined)[];
    }): void;
    finalize(): void;
    /** Imperative API. Widgets added this way are not affected by the declarative prop. */
    addDefault(widget: Widget): void;
    onRedraw({ viewports, layers }: {
        viewports: Viewport[];
        layers: Layer[];
    }): void;
    onHover(info: PickingInfo, event: MjolnirPointerEvent): void;
    onEvent(info: PickingInfo, event: MjolnirGestureEvent): void;
    /**
     * Resolve widgets from the declarative prop
     * Initialize new widgets and remove old ones
     * Update props of existing widgets
     */
    private _setWidgets;
    /** Initialize new widget */
    private _addWidget;
    /** Destroy an old widget */
    private _removeWidget;
    /** Get a container element based on view and placement */
    private _getContainer;
    private _updateContainers;
}
export {};
//# sourceMappingURL=widget-manager.d.ts.map