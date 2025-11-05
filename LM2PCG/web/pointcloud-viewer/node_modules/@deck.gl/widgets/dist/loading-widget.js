import { jsx as _jsx } from "preact/jsx-runtime";
import { render } from 'preact';
import { Widget } from '@deck.gl/core';
import { IconButton } from "./lib/components/icon-button.js";
/**
 * A non-interactive widget that shows a loading spinner if any layers are loading data
 */
export class LoadingWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-loading';
        this.placement = 'top-left';
        this.loading = true;
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        render(
        // TODO(ibgreen) - this should not be a button, but styling is so nested that it is easier to reuse this component.
        this.loading && (_jsx(IconButton, { className: "deck-widget-spinner", label: this.props.label, onClick: this.handleClick.bind(this) })), rootElement);
    }
    onRedraw({ layers }) {
        const loading = layers.some(layer => !layer.isLoaded);
        if (loading !== this.loading) {
            this.loading = loading;
            this.updateHTML();
        }
    }
    // TODO(ibgreen) - this should not be a button, see above.
    handleClick() { }
}
LoadingWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'loading',
    placement: 'top-left',
    viewId: null,
    label: 'Loading layer data'
};
//# sourceMappingURL=loading-widget.js.map