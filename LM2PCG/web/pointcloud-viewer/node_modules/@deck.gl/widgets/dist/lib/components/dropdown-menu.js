import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
import { useState, useRef, useEffect } from 'preact/hooks';
export const DropdownMenu = (props) => {
    const [isOpen, setIsOpen] = useState(false);
    const dropdownRef = useRef(null);
    const toggleDropdown = () => setIsOpen(!isOpen);
    const handleClickOutside = (event) => {
        if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
            setIsOpen(false);
        }
    };
    useEffect(() => {
        document.addEventListener('mousedown', handleClickOutside);
        return () => {
            document.removeEventListener('mousedown', handleClickOutside);
        };
    }, []);
    const handleSelect = (value) => {
        props.onSelect(value);
        setIsOpen(false);
    };
    return (_jsxs("div", { className: "dropdown-container", ref: dropdownRef, style: {
            position: 'relative',
            display: 'inline-block',
            ...props.style
        }, children: [_jsx("button", { onClick: toggleDropdown, style: {
                    width: '30px',
                    height: '30px',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    background: '#fff',
                    cursor: 'pointer',
                    padding: 0
                }, children: "\u25BC" }), isOpen && (_jsx("ul", { style: {
                    position: 'absolute',
                    top: '100%',
                    right: '100%',
                    background: '#fff',
                    border: '1px solid #ccc',
                    borderRadius: '4px',
                    listStyle: 'none',
                    padding: '4px 0',
                    margin: 0,
                    zIndex: 1000,
                    minWidth: '200px'
                }, children: props.menuItems.map(item => (_jsx("li", { onClick: () => handleSelect(item), style: {
                        padding: '4px 8px',
                        cursor: 'pointer',
                        whiteSpace: 'nowrap'
                    }, children: item }, item))) }))] }));
};
//# sourceMappingURL=dropdown-menu.js.map