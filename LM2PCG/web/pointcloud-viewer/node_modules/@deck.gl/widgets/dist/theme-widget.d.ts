import { Widget, type WidgetProps, type WidgetPlacement } from '@deck.gl/core';
import type { DeckWidgetTheme } from "./themes.js";
export type ThemeWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Tooltip message when dark mode is selected. */
    lightModeLabel?: string;
    /** Styles for light mode theme */
    lightModeTheme?: DeckWidgetTheme;
    /** Tooltip message when light mode is selected. */
    darkModeLabel?: string;
    /** Styles for dark mode theme */
    darkModeTheme?: DeckWidgetTheme;
    /** Initial theme mode. 'auto' reads the browser default setting */
    initialThemeMode?: 'auto' | 'light' | 'dark';
};
export declare class ThemeWidget extends Widget<ThemeWidgetProps> {
    static defaultProps: Required<ThemeWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    themeMode: 'light' | 'dark';
    constructor(props?: ThemeWidgetProps);
    setProps(props: Partial<ThemeWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onAdd(): void;
    _handleClick(): void;
    _setThemeMode(themeMode: 'light' | 'dark'): void;
    /** Read browser preference */
    _getInitialThemeMode(): 'light' | 'dark';
}
//# sourceMappingURL=theme-widget.d.ts.map