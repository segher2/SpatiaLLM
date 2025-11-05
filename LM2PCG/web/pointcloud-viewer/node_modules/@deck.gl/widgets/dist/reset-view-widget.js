import { jsx as _jsx } from "preact/jsx-runtime";
import { render } from 'preact';
import { Widget } from '@deck.gl/core';
import { IconButton } from "./lib/components/icon-button.js";
/**
 * A button widget that resets the view state of deck to an initial state.
 */
export class ResetViewWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-reset-view';
        this.placement = 'top-left';
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        render(_jsx(IconButton, { className: "deck-widget-reset-focus", label: this.props.label, onClick: this.handleClick.bind(this) }), rootElement);
    }
    handleClick() {
        const initialViewState = this.props.initialViewState || this.deck?.props.initialViewState;
        this.setViewState(initialViewState);
    }
    setViewState(viewState) {
        const viewId = (this.props.viewId || 'default-view');
        const nextViewState = {
            ...(viewId !== 'default-view' ? viewState?.[viewId] : viewState)
            // only works for geospatial?
            // transitionDuration: this.props.transitionDuration,
            // transitionInterpolator: new FlyToInterpolator()
        };
        // @ts-ignore Using private method temporary until there's a public one
        this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
}
ResetViewWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'reset-view',
    placement: 'top-left',
    label: 'Reset View',
    initialViewState: undefined,
    viewId: null
};
//# sourceMappingURL=reset-view-widget.js.map