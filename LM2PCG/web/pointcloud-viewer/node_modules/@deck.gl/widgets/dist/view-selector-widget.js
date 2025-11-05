import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { render } from 'preact';
import { Widget } from '@deck.gl/core';
import { IconMenu } from "./lib/components/icon-menu.js";
/**
 * A widget that renders a popup menu for selecting a view mode.
 * It displays a button with the current view mode icon. Clicking the button
 * toggles a popup that shows three icons for:
 * - Single view
 * - Two views, split horizontally
 * - Two views, split vertically
 */
export class ViewSelectorWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-view-selector';
        this.placement = 'top-left';
        this.handleSelectMode = (viewMode) => {
            this.viewMode = viewMode;
            this.updateHTML();
            this.props.onViewModeChange(viewMode);
        };
        this.viewMode = this.props.initialViewMode;
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        render(_jsx(IconMenu, { className: "deck-widget-view-selector", menuItems: MENU_ITEMS.map(item => ({
                ...item,
                icon: item.icon()
            })), initialItem: this.props.initialViewMode, onItemSelected: this.handleSelectMode }), rootElement);
    }
}
ViewSelectorWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'view-selector',
    placement: 'top-left',
    viewId: null,
    label: 'Split View',
    initialViewMode: 'single',
    onViewModeChange: () => { }
};
const ICON_STYLE = { width: '24px', height: '24px' };
// JSX wrapped in a function to fix deck's Node tests
const ICONS = {
    single: () => (_jsx("svg", { width: "24", height: "24", style: ICON_STYLE, children: _jsx("rect", { x: "4", y: "4", width: "16", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }) })),
    'split-horizontal': () => (_jsxs("svg", { width: "24", height: "24", style: ICON_STYLE, children: [_jsx("rect", { x: "4", y: "4", width: "16", height: "7", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }), _jsx("rect", { x: "4", y: "13", width: "16", height: "7", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" })] })),
    'split-vertical': () => (_jsxs("svg", { width: "24", height: "24", style: ICON_STYLE, children: [_jsx("rect", { x: "4", y: "4", width: "7", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }), _jsx("rect", { x: "13", y: "4", width: "7", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" })] }))
};
// Define menu items for the popup menu.
const MENU_ITEMS = [
    { value: 'single', icon: ICONS.single, label: 'Single View' },
    { value: 'split-horizontal', icon: ICONS['split-horizontal'], label: 'Split Horizontal' },
    { value: 'split-vertical', icon: ICONS['split-vertical'], label: 'Split Vertical' }
];
//# sourceMappingURL=view-selector-widget.js.map