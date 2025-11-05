import type { JSX } from 'preact';
export type IconMenuProps<KeyType = string> = {
    className: string;
    icon?: JSX.Element;
    label?: string;
    menuItems: {
        value: KeyType;
        icon: JSX.Element;
        label: string;
    }[];
    initialItem: KeyType;
    onItemSelected: (item: KeyType) => void;
};
/** A component that renders an icon popup menu */
export declare function IconMenu<KeyType extends string>(props: IconMenuProps<KeyType>): JSX.Element;
//# sourceMappingURL=icon-menu.d.ts.map