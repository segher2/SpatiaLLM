import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget, FlyToInterpolator, LinearInterpolator } from '@deck.gl/core';
import { render } from 'preact';
import { ButtonGroup } from "./lib/components/button-group.js";
import { GroupedIconButton } from "./lib/components/grouped-icon-button.js";
export class ZoomWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-zoom';
        this.placement = 'top-left';
        this.viewports = {};
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        const ui = (_jsxs(ButtonGroup, { orientation: this.props.orientation, children: [_jsx(GroupedIconButton, { onClick: () => this.handleZoomIn(), label: this.props.zoomInLabel, className: "deck-widget-zoom-in" }), _jsx(GroupedIconButton, { onClick: () => this.handleZoomOut(), label: this.props.zoomOutLabel, className: "deck-widget-zoom-out" })] }));
        render(ui, rootElement);
    }
    onViewportChange(viewport) {
        this.viewports[viewport.id] = viewport;
    }
    handleZoom(viewport, nextZoom) {
        const viewId = this.viewId || viewport?.id || 'default-view';
        const nextViewState = {
            ...viewport,
            zoom: nextZoom
        };
        if (this.props.transitionDuration > 0) {
            nextViewState.transitionDuration = this.props.transitionDuration;
            nextViewState.transitionInterpolator =
                'latitude' in nextViewState
                    ? new FlyToInterpolator()
                    : new LinearInterpolator({
                        transitionProps: ['zoom']
                    });
        }
        this.setViewState(viewId, nextViewState);
    }
    handleZoomIn() {
        for (const viewport of Object.values(this.viewports)) {
            this.handleZoom(viewport, viewport.zoom + 1);
        }
    }
    handleZoomOut() {
        for (const viewport of Object.values(this.viewports)) {
            this.handleZoom(viewport, viewport.zoom - 1);
        }
    }
    /** @todo - move to deck or widget manager */
    setViewState(viewId, viewState) {
        // @ts-ignore Using private method temporary until there's a public one
        this.deck._onViewStateChange({ viewId, viewState, interactionState: {} });
    }
}
ZoomWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'zoom',
    placement: 'top-left',
    orientation: 'vertical',
    transitionDuration: 200,
    zoomInLabel: 'Zoom In',
    zoomOutLabel: 'Zoom Out',
    viewId: null
};
//# sourceMappingURL=zoom-widget.js.map