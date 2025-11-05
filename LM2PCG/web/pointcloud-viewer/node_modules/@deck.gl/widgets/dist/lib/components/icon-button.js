import { jsx as _jsx } from "preact/jsx-runtime";
/** Renders a button component with widget CSS */
export const IconButton = (props) => {
    const { className = '', label, onClick, children } = props;
    return (_jsx("div", { className: "deck-widget-button", children: _jsx("button", { className: `deck-widget-icon-button ${className}`, type: "button", onClick: onClick, title: label, children: children ? children : _jsx("div", { className: "deck-widget-icon" }) }) }));
};
//# sourceMappingURL=icon-button.js.map