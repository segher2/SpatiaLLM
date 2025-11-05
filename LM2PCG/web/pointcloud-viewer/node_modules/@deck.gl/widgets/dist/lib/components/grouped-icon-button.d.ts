import type { ComponentChildren, JSX } from 'preact';
export type GroupedIconButtonProps = {
    className?: string;
    label?: string;
    onClick?: JSX.MouseEventHandler<HTMLButtonElement>;
    children?: ComponentChildren;
};
/** Renders an icon button as part of a ButtonGroup */
export declare const GroupedIconButton: (props: GroupedIconButtonProps) => JSX.Element;
//# sourceMappingURL=grouped-icon-button.d.ts.map