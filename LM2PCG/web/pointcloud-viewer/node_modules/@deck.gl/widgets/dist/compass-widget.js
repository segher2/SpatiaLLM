import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget, FlyToInterpolator, WebMercatorViewport, _GlobeViewport } from '@deck.gl/core';
import { render } from 'preact';
export class CompassWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-compass';
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
        const viewId = this.viewId || Object.values(this.viewports)[0]?.id || 'default-view';
        const widgetViewport = this.viewports[viewId];
        const [rz, rx] = this.getRotation(widgetViewport);
        const ui = (_jsx("div", { className: "deck-widget-button", style: { perspective: 100 }, children: _jsx("button", { type: "button", onClick: () => {
                    for (const viewport of Object.values(this.viewports)) {
                        this.handleCompassReset(viewport);
                    }
                }, title: this.props.label, style: { transform: `rotateX(${rx}deg)` }, children: _jsx("svg", { fill: "none", width: "100%", height: "100%", viewBox: "0 0 26 26", children: _jsxs("g", { transform: `rotate(${rz},13,13)`, children: [_jsx("path", { d: "M10 13.0001L12.9999 5L15.9997 13.0001H10Z", fill: "var(--icon-compass-north-color, rgb(240, 92, 68))" }), _jsx("path", { d: "M16.0002 12.9999L13.0004 21L10.0005 12.9999H16.0002Z", fill: "var(--icon-compass-south-color, rgb(204, 204, 204))" })] }) }) }) }));
        render(ui, rootElement);
    }
    onViewportChange(viewport) {
        // no need to update if viewport is the same
        if (!viewport.equals(this.viewports[viewport.id])) {
            this.viewports[viewport.id] = viewport;
            this.updateHTML();
        }
    }
    getRotation(viewport) {
        if (viewport instanceof WebMercatorViewport) {
            return [-viewport.bearing, viewport.pitch];
        }
        else if (viewport instanceof _GlobeViewport) {
            return [0, Math.max(-80, Math.min(80, viewport.latitude))];
        }
        return [0, 0];
    }
    handleCompassReset(viewport) {
        const viewId = this.viewId || viewport.id || 'default-view';
        if (viewport instanceof WebMercatorViewport) {
            const nextViewState = {
                ...viewport,
                bearing: 0,
                ...(this.getRotation(viewport)[0] === 0 ? { pitch: 0 } : {}),
                transitionDuration: this.props.transitionDuration,
                transitionInterpolator: new FlyToInterpolator()
            };
            // @ts-ignore Using private method temporary until there's a public one
            this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
        }
    }
}
CompassWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'compass',
    placement: 'top-left',
    viewId: null,
    label: 'Reset Compass',
    transitionDuration: 200
};
//# sourceMappingURL=compass-widget.js.map