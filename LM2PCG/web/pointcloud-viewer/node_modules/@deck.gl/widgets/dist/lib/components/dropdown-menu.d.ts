import { type JSX } from 'preact';
export type DropdownMenuProps = {
    menuItems: string[];
    onSelect: (value: string) => void;
    style?: JSX.CSSProperties;
};
export declare const DropdownMenu: (props: DropdownMenuProps) => JSX.Element;
//# sourceMappingURL=dropdown-menu.d.ts.map