import { jsx as _jsx } from "preact/jsx-runtime";
import { render } from 'preact';
import { Widget } from '@deck.gl/core';
import { IconButton } from "./lib/components/icon-button.js";
/**
 * A button widget that captures a screenshot of the current canvas and downloads it as a (png) file.
 * @note only captures canvas contents, not HTML DOM or CSS styles
 */
export class ScreenshotWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-screenshot';
        this.placement = 'top-left';
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        render(_jsx(IconButton, { className: "deck-widget-camera", label: this.props.label, onClick: this.handleClick.bind(this) }), rootElement);
    }
    handleClick() {
        // Allow user to override the capture logic
        if (this.props.onCapture) {
            this.props.onCapture(this);
            return;
        }
        const dataURL = this.captureScreenToDataURL(this.props.imageFormat);
        if (dataURL) {
            this.downloadDataURL(dataURL, this.props.filename);
        }
    }
    /** @note only captures canvas contents, not HTML DOM or CSS styles */
    captureScreenToDataURL(imageFormat) {
        const canvas = this.deck?.getCanvas();
        return canvas?.toDataURL(imageFormat);
    }
    /** Download a data URL */
    downloadDataURL(dataURL, filename) {
        const link = document.createElement('a');
        link.href = dataURL;
        link.download = filename;
        link.click();
    }
}
ScreenshotWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'screenshot',
    placement: 'top-left',
    viewId: null,
    label: 'Screenshot',
    filename: 'screenshot.png',
    imageFormat: 'image/png',
    onCapture: undefined
};
//# sourceMappingURL=screenshot-widget.js.map