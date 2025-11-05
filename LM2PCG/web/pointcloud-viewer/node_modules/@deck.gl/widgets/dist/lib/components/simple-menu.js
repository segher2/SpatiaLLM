import { jsx as _jsx } from "preact/jsx-runtime";
const MENU_STYLE = {
    position: 'absolute',
    top: '100%',
    left: 0,
    background: 'white',
    border: '1px solid #ccc',
    borderRadius: '4px',
    marginTop: 'var(--menu-gap, 4px)',
    zIndex: 100
};
const MENU_ITEM_STYLE = {
    background: 'white',
    border: 'none',
    padding: '4px',
    cursor: 'pointer',
    pointerEvents: 'auto'
};
/** Renders a simple dropdown menu at an arbitrary position */
export const SimpleMenu = (props) => {
    const { menuItems, onItemSelected, position, style } = props;
    const styleOverride = {
        ...MENU_STYLE,
        ...style,
        left: `${position.x}px`,
        top: `${position.y}px`
    };
    return (_jsx("div", { style: styleOverride, children: menuItems.map(({ key, label }) => (_jsx("button", { style: { ...MENU_ITEM_STYLE, display: 'block' }, onClick: _ => onItemSelected(key), children: label }, key))) }));
};
//# sourceMappingURL=simple-menu.js.map