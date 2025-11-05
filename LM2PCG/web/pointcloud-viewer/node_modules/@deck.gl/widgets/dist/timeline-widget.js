import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget } from '@deck.gl/core';
import { render } from 'preact';
import { IconButton } from "./lib/components/icon-button.js";
export class TimelineWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.id = 'timeline';
        this.className = 'deck-widget-timeline';
        this.placement = 'bottom-left';
        this.playing = false;
        this.timerId = null;
        this.handlePlayPause = () => {
            if (this.playing) {
                this.stop();
            }
            else {
                this.start();
            }
        };
        this.handleSliderChange = (e) => {
            const input = e.target;
            const val = Number(input.value);
            this.currentTime = val;
            this.props.onTimeChange(val);
            this.updateHTML();
        };
        this.tick = () => {
            const [min, max] = this.props.timeRange;
            let next = this.currentTime + this.props.step;
            if (next > max) {
                next = min;
            }
            this.currentTime = next;
            this.props.onTimeChange(next);
            this.updateHTML();
            if (this.playing) {
                this.timerId = window.setTimeout(this.tick, this.props.playInterval);
            }
        };
        this.currentTime = this.props.initialTime ?? this.props.timeRange[0];
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onAdd() {
        this.playing = false;
        this.timerId = null;
    }
    onRemove() {
        this.stop();
    }
    onRenderHTML(rootElement) {
        render(_jsxs("div", { style: { display: 'flex', alignItems: 'center', pointerEvents: 'auto' }, children: [_jsx(IconButton, { label: this.playing ? 'Pause' : 'Play', onClick: this.handlePlayPause, children: _jsx("div", { className: "text", children: this.playing ? '⏸' : '▶' }) }), _jsx("input", { type: "range", className: "timeline-slider", min: this.props.timeRange[0], max: this.props.timeRange[1], step: this.props.step, value: this.currentTime, onInput: this.handleSliderChange })] }), rootElement);
    }
    start() {
        this.playing = true;
        this.updateHTML();
        this.tick();
    }
    stop() {
        this.playing = false;
        if (this.timerId !== null) {
            window.clearTimeout(this.timerId);
            this.timerId = null;
        }
        this.updateHTML();
    }
}
TimelineWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'timeline',
    placement: 'bottom-left',
    viewId: null,
    timeRange: [0, 100],
    step: 1,
    initialTime: undefined,
    onTimeChange: () => { },
    playInterval: 1000
};
//# sourceMappingURL=timeline-widget.js.map