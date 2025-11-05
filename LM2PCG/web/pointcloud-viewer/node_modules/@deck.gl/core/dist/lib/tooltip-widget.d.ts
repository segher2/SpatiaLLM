import { Widget, WidgetProps } from "./widget.js";
import type { WidgetPlacement } from "./widget-manager.js";
import type { PickingInfo } from "./picking/pick-info.js";
import type Viewport from "../viewports/viewport.js";
export type TooltipContent = null | string | {
    text?: string;
    html?: string;
    className?: string;
    style?: Partial<CSSStyleDeclaration>;
};
export type TooltipWidgetProps = WidgetProps;
export declare class TooltipWidget extends Widget<TooltipWidgetProps> {
    static defaultProps: Required<TooltipWidgetProps>;
    id: string;
    placement: WidgetPlacement;
    className: string;
    isVisible: boolean;
    lastViewport?: Viewport;
    constructor(props?: TooltipWidgetProps);
    onCreateRootElement(): HTMLDivElement;
    onRenderHTML(rootElement: HTMLElement): void;
    onViewportChange(viewport: Viewport): void;
    onHover(info: PickingInfo): void;
    setTooltip(displayInfo: TooltipContent, x?: number, y?: number): void;
}
//# sourceMappingURL=tooltip-widget.d.ts.map