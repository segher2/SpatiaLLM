import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
import { useState, useRef, useEffect } from 'preact/hooks';
import { IconButton } from "./icon-button.js";
import { ButtonGroup } from "./button-group.js";
import { GroupedIconButton } from "./grouped-icon-button.js";
/** A component that renders an icon popup menu */
export function IconMenu(props) {
    const [menuOpen, setMenuOpen] = useState(false);
    const containerRef = useRef(null);
    // Close the menu when clicking outside.
    const handleClickOutside = (event) => {
        if (containerRef.current && !containerRef.current.contains(event.target)) {
            setMenuOpen(false);
        }
    };
    useEffect(() => {
        document.addEventListener('mousedown', handleClickOutside);
        return () => {
            document.removeEventListener('mousedown', handleClickOutside);
        };
    }, [containerRef]);
    const [selectedItem, setSelectedItem] = useState(props.initialItem);
    const handleSelectItem = (item) => {
        setSelectedItem(item);
        setMenuOpen(false);
        props.onItemSelected(item);
    };
    const handleButtonClick = () => setMenuOpen(!menuOpen);
    const selectedMenuItem = props.menuItems.find(item => item.value === selectedItem);
    const label = props.label || selectedMenuItem?.label || '';
    const icon = (props.icon || selectedMenuItem?.icon);
    return (_jsxs("div", { style: { position: 'relative', display: 'inline-block' }, ref: containerRef, children: [_jsx(IconButton, { className: props.className, label: label, onClick: handleButtonClick, children: icon }), menuOpen && (_jsx("div", { className: "deck-widget-icon-menu", children: _jsx(ButtonGroup, { orientation: "vertical", children: props.menuItems.map(item => (_jsx(GroupedIconButton, { label: item.label, onClick: () => handleSelectItem(item.value), children: item.icon }, item.value))) }) }))] }));
}
//# sourceMappingURL=icon-menu.js.map