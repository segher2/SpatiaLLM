import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget } from '@deck.gl/core';
import { render } from 'preact';
import { IconButton } from "./lib/components/icon-button.js";
/**
 * Displays the average frames per second reported by the Deck instance.
 */
export class FpsWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-fps';
        this.placement = 'top-left';
        this._lastFps = -1;
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onAdd({}) {
        this._lastFps = this._getFps();
        requestAnimationFrame(() => this._animate());
    }
    onRenderHTML(rootElement) {
        const fps = this._getFps();
        render(_jsx(IconButton, { children: _jsxs("div", { className: "text", children: ["FPS", _jsx("br", {}), fps] }) }), rootElement);
    }
    _animate() {
        const fps = this._getFps();
        if (this._lastFps !== fps) {
            this._lastFps = fps;
            this.updateHTML();
        }
        requestAnimationFrame(() => this._animate());
    }
    _getFps() {
        // @ts-expect-error protected
        return Math.round(this.deck?.metrics.fps ?? 0);
    }
}
FpsWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'fps',
    placement: 'top-left',
    viewId: null
};
//# sourceMappingURL=fps-widget.js.map