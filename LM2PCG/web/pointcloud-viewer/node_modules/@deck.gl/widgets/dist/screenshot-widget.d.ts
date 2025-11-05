import type { WidgetPlacement, WidgetProps } from '@deck.gl/core';
import { Widget } from '@deck.gl/core';
/** Properties for the ScreenshotWidget */
export type ScreenshotWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip message */
    label?: string;
    /** Filename to save to */
    filename?: string;
    /** Image format */
    imageFormat?: 'image/png' | 'image/jpeg';
    /** Callback, if defined user overrides the capture logic */
    onCapture?: (widget: ScreenshotWidget) => void;
};
/**
 * A button widget that captures a screenshot of the current canvas and downloads it as a (png) file.
 * @note only captures canvas contents, not HTML DOM or CSS styles
 */
export declare class ScreenshotWidget extends Widget<ScreenshotWidgetProps> {
    static defaultProps: Required<ScreenshotWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    constructor(props?: ScreenshotWidgetProps);
    setProps(props: Partial<ScreenshotWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    handleClick(): void;
    /** @note only captures canvas contents, not HTML DOM or CSS styles */
    captureScreenToDataURL(imageFormat: string): string | undefined;
    /** Download a data URL */
    downloadDataURL(dataURL: string, filename: string): void;
}
//# sourceMappingURL=screenshot-widget.d.ts.map