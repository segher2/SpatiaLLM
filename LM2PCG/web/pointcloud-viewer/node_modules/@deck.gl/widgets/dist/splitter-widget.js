import { jsx as _jsx } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { render } from 'preact';
import { useState, useRef } from 'preact/hooks';
import { Widget } from '@deck.gl/core';
/**
 * A draggable splitter widget that appears as a vertical or horizontal line
 * across the deck.gl canvas. It positions itself based on the split percentage
 * of the first view and provides callbacks when dragged.
 */
export class SplitterWidget extends Widget {
    constructor(props) {
        super(props);
        this.className = 'deck-widget-splitter';
        this.placement = 'fill';
    }
    setProps(props) {
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        // Ensure the widget container fills the deck.gl canvas.
        // TODO - Move styling to CSS
        rootElement.style.position = 'absolute';
        rootElement.style.top = '0';
        rootElement.style.left = '0';
        rootElement.style.width = '100%';
        rootElement.style.height = '100%';
        rootElement.style.margin = '0px';
        render(_jsx(Splitter, { orientation: this.props.orientation, initialSplit: this.props.initialSplit, onChange: this.props.onChange, onDragStart: this.props.onDragStart, onDragEnd: this.props.onDragEnd }), rootElement);
    }
}
SplitterWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'splitter-widget',
    viewId1: '',
    viewId2: '',
    orientation: 'vertical',
    initialSplit: 0.5,
    onChange: () => { },
    onDragStart: () => { },
    onDragEnd: () => { }
};
/**
 * A functional component that renders a draggable splitter line.
 * It computes its position based on the provided split percentage and
 * updates it during mouse drag events.
 */
function Splitter({ orientation, initialSplit, onChange, onDragStart, onDragEnd }) {
    const [split, setSplit] = useState(initialSplit);
    const dragging = useRef(false);
    const containerRef = useRef(null);
    const handleDragStart = (event) => {
        dragging.current = true;
        onDragStart?.();
        document.addEventListener('mousemove', handleDragging);
        document.addEventListener('mouseup', handleDragEnd);
        event.preventDefault();
    };
    const handleDragging = (event) => {
        if (!dragging.current || !containerRef.current)
            return;
        const rect = containerRef.current.getBoundingClientRect();
        let newSplit;
        if (orientation === 'vertical') {
            newSplit = (event.clientX - rect.left) / rect.width;
        }
        else {
            newSplit = (event.clientY - rect.top) / rect.height;
        }
        // Clamp newSplit between 5% and 95%
        newSplit = Math.min(Math.max(newSplit, 0.05), 0.95);
        setSplit(newSplit);
        onChange?.(newSplit);
    };
    const handleDragEnd = (event) => {
        if (!dragging.current)
            return;
        dragging.current = false;
        onDragEnd?.();
        document.removeEventListener('mousemove', handleDragging);
        document.removeEventListener('mouseup', handleDragEnd);
    };
    // The splitter line style based on orientation and the current split percentage.
    const splitterStyle = orientation === 'vertical'
        ? {
            position: 'absolute',
            top: 0,
            bottom: 0,
            left: `${split * 100}%`,
            width: '4px',
            cursor: 'col-resize',
            background: '#ccc',
            zIndex: 10,
            pointerEvents: 'auto',
            boxShadow: 'inset -1px 0 0 white, inset 1px 0 0 white'
        }
        : {
            position: 'absolute',
            left: 0,
            right: 0,
            top: `${split * 100}%`,
            height: '4px',
            cursor: 'row-resize',
            background: '#ccc',
            zIndex: 10,
            pointerEvents: 'auto',
            boxShadow: 'inset -1px 0 0 white, inset 1px 0 0 white'
        };
    // Container style to fill the entire deck.gl canvas.
    const containerStyle = {
        position: 'absolute',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0
    };
    return (_jsx("div", { ref: containerRef, style: containerStyle, children: _jsx("div", { style: splitterStyle, onMouseDown: handleDragStart }) }));
}
//# sourceMappingURL=splitter-widget.js.map