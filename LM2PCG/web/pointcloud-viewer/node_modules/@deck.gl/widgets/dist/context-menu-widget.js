import { jsx as _jsx } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
/* global document */
import { Widget } from '@deck.gl/core';
import { render } from 'preact';
import { SimpleMenu } from "./lib/components/simple-menu.js";
/** The standard, modern way is to use event.button === 2, where button is the standardized property (0 = left, 1 = middle, 2 = right). */
const MOUSE_BUTTON_RIGHT = 2;
/** A name for the legacy MouseEvent.which value that corresponds to the right-mouse button. In older browsers, the check is: if (event.which === 3) */
const MOUSE_WHICH_RIGHT = 3;
export class ContextMenuWidget extends Widget {
    constructor(props) {
        super(props);
        this.className = 'deck-widget-context-menu';
        this.placement = 'fill';
        this.pickInfo = null;
        this.pickInfo = null;
        this.setProps(this.props);
    }
    onAdd({ deck }) {
        const element = document.createElement('div');
        element.classList.add('deck-widget', 'deck-widget-context-menu');
        const style = {
            margin: '0px',
            top: '0px',
            left: '0px',
            position: 'absolute',
            pointerEvents: 'auto'
        };
        Object.entries(style).forEach(([key, value]) => element.style.setProperty(key, value));
        deck.getCanvas()?.addEventListener('click', () => this.hide());
        deck.getCanvas()?.addEventListener('contextmenu', event => this.handleContextMenu(event));
        return element;
    }
    onRenderHTML(rootElement) {
        const { visible, position, menuItems } = this.props;
        const ui = visible && menuItems.length ? (_jsx(SimpleMenu, { menuItems: menuItems, onItemSelected: key => this.props.onMenuItemSelected(key, this.pickInfo), position: position, style: { pointerEvents: 'auto' } })) : null;
        render(ui, rootElement);
    }
    handleContextMenu(srcEvent) {
        if (srcEvent &&
            (srcEvent.button === MOUSE_BUTTON_RIGHT || srcEvent.which === MOUSE_WHICH_RIGHT)) {
            this.pickInfo =
                this.deck?.pickObject({
                    x: srcEvent.clientX,
                    y: srcEvent.clientY
                }) || null;
            const menuItems = (this.pickInfo && this.props.getMenuItems?.(this.pickInfo, this)) || [];
            const visible = menuItems.length > 0;
            this.setProps({
                visible,
                position: { x: srcEvent.clientX, y: srcEvent.clientY },
                menuItems
            });
            this.updateHTML();
            srcEvent.preventDefault();
            return visible;
        }
        return false;
    }
    hide() {
        this.setProps({ visible: false });
    }
}
ContextMenuWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'context',
    viewId: null,
    visible: false,
    position: { x: 0, y: 0 },
    getMenuItems: undefined,
    menuItems: [],
    // eslint-disable-next-line no-console
    onMenuItemSelected: (key, pickInfo) => console.log('Context menu item selected:', key, pickInfo)
};
//# sourceMappingURL=context-menu-widget.js.map