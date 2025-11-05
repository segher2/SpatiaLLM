import { jsx as _jsx } from "preact/jsx-runtime";
/** Renders an icon button as part of a ButtonGroup */
export const GroupedIconButton = (props) => {
    const { className = '', label, onClick, children } = props;
    return (_jsx("button", { className: `deck-widget-icon-button ${className}`, type: "button", onClick: onClick, title: label, children: children ? children : _jsx("div", { className: "deck-widget-icon" }) }));
};
//# sourceMappingURL=grouped-icon-button.js.map