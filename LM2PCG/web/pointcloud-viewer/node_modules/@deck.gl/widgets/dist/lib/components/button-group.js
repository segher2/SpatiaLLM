import { jsx as _jsx } from "preact/jsx-runtime";
/** Renders a group of buttons with Widget CSS */
export const ButtonGroup = (props) => {
    const { children, orientation = 'horizontal' } = props;
    return _jsx("div", { className: `deck-widget-button-group ${orientation}`, children: children });
};
//# sourceMappingURL=button-group.js.map