import type { JSX } from 'preact';
export type SimpleMenuProps = {
    menuItems: {
        key: string;
        label: string;
    }[];
    onItemSelected: (key: string) => void;
    position: {
        x: number;
        y: number;
    };
    style?: JSX.CSSProperties;
};
/** Renders a simple dropdown menu at an arbitrary position */
export declare const SimpleMenu: (props: SimpleMenuProps) => JSX.Element;
//# sourceMappingURL=simple-menu.d.ts.map