import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget } from '@deck.gl/core';
import { luma } from '@luma.gl/core';
import { render } from 'preact';
const RIGHT_ARROW = '\u25b6';
const DOWN_ARROW = '\u2b07';
const DEFAULT_COUNT_FORMATTER = (stat) => `${stat.name}: ${stat.count}`;
function formatTime(time) {
    return time < 1000 ? `${time.toFixed(2)}ms` : `${(time / 1000).toFixed(2)}s`;
}
function formatMemory(bytes) {
    const mb = bytes / 1e6;
    return `${mb.toFixed(1)} MB`;
}
export const DEFAULT_FORMATTERS = {
    count: DEFAULT_COUNT_FORMATTER,
    averageTime: (stat) => `${stat.name}: ${formatTime(stat.getAverageTime())}`,
    totalTime: (stat) => `${stat.name}: ${formatTime(stat.time)}`,
    fps: (stat) => `${stat.name}: ${Math.round(stat.getHz())}fps`,
    memory: (stat) => `${stat.name}: ${formatMemory(stat.count)}`
};
/** Displays probe.gl stats in a floating pop-up. */
export class StatsWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-stats';
        this.placement = 'top-left';
        this._counter = 0;
        this.collapsed = true;
        this._toggleCollapsed = () => {
            this.collapsed = !this.collapsed;
            this.updateHTML();
        };
        this._formatters = { ...DEFAULT_FORMATTERS };
        this._resetOnUpdate = { ...this.props.resetOnUpdate };
        this._stats = this.props.stats;
        this.setProps(props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        this._stats = this._getStats();
        if (props.formatters) {
            for (const name in props.formatters) {
                const f = props.formatters[name];
                this._formatters[name] =
                    typeof f === 'string' ? DEFAULT_FORMATTERS[f] || DEFAULT_COUNT_FORMATTER : f;
            }
        }
        if (props.resetOnUpdate) {
            this._resetOnUpdate = { ...props.resetOnUpdate };
        }
        super.setProps(props);
    }
    onAdd() {
        this._stats = this._getStats();
        this.updateHTML();
    }
    onRenderHTML(rootElement) {
        const stats = this._stats;
        const collapsed = this.collapsed;
        const title = this.props.title || stats?.id || 'Stats';
        const items = [];
        if (!collapsed && stats) {
            stats.forEach(stat => {
                const lines = this._getLines(stat);
                if (this._resetOnUpdate && this._resetOnUpdate[stat.name]) {
                    stat.reset();
                }
                lines.forEach((line, i) => {
                    items.push(_jsx("div", { style: { whiteSpace: 'pre' }, children: line }, `${stat.name}-${i}`));
                });
            });
        }
        render(_jsxs("div", { className: "deck-widget-stats-container", style: { cursor: 'default' }, children: [_jsxs("div", { className: "deck-widget-stats-header", style: { cursor: 'pointer', pointerEvents: 'auto' }, onClick: this._toggleCollapsed, children: [collapsed ? RIGHT_ARROW : DOWN_ARROW, " ", title] }), !collapsed && _jsx("div", { className: "deck-widget-stats-content", children: items })] }), rootElement);
    }
    onRedraw() {
        const framesPerUpdate = Math.max(1, this.props.framesPerUpdate || 1);
        if (this._counter++ % framesPerUpdate === 0) {
            this._stats = this._getStats();
            this.updateHTML();
        }
    }
    _getStats() {
        switch (this.props.type) {
            case 'deck':
                // @ts-expect-error stats is protected
                return this.deck?.stats;
            case 'luma':
                return Array.from(luma.stats.stats.values())[0];
            case 'device':
                // @ts-expect-error is protected
                const device = this.deck?.device;
                const stats = device?.statsManager.stats.values();
                return stats ? Array.from(stats)[0] : undefined;
            case 'custom':
                return this.props.stats;
            default:
                throw new Error(`Unknown stats type: ${this.props.type}`);
        }
    }
    _getLines(stat) {
        const formatter = this._formatters[stat.name] || this._formatters[stat.type || ''] || DEFAULT_COUNT_FORMATTER;
        return formatter(stat).split('\n');
    }
}
StatsWidget.defaultProps = {
    ...Widget.defaultProps,
    type: 'deck',
    placement: 'top-left',
    viewId: null,
    stats: undefined,
    title: 'Stats',
    framesPerUpdate: 1,
    formatters: {},
    resetOnUpdate: {},
    id: 'stats'
};
//# sourceMappingURL=stats-widget.js.map