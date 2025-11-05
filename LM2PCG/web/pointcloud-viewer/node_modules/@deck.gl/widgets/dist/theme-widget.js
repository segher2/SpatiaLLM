import { jsx as _jsx } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { log, _deepEqual as deepEqual, _applyStyles as applyStyles } from '@deck.gl/core';
import { Widget } from '@deck.gl/core';
import { render } from 'preact';
// import {useCallback} from 'preact/hooks';
import { IconButton } from "./lib/components/icon-button.js";
import { LightGlassTheme, DarkGlassTheme } from "./themes.js";
export class ThemeWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-theme';
        this.placement = 'top-left';
        this.themeMode = 'dark';
        this.themeMode = this._getInitialThemeMode();
        this.setProps(this.props);
    }
    // eslint-disable-next-line complexity
    setProps(props) {
        const { lightModeTheme, darkModeTheme } = this.props;
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        super.setProps(props);
        switch (this.themeMode) {
            case 'light':
                if (props.lightModeTheme && !deepEqual(props.lightModeTheme, lightModeTheme, 1)) {
                    this._setThemeMode('light');
                }
                break;
            case 'dark':
                if (props.darkModeTheme && !deepEqual(props.darkModeTheme, darkModeTheme, 1)) {
                    this._setThemeMode('dark');
                }
                break;
            default:
                log.warn(`Invalid theme mode ${this.themeMode}`)();
        }
    }
    onRenderHTML(rootElement) {
        const { lightModeLabel, darkModeLabel } = this.props;
        // const onClick = useCallback(this._handleClick.bind(this), [this._handleClick]);
        render(_jsx(IconButton, { onClick: this._handleClick.bind(this), label: this.themeMode === 'dark' ? darkModeLabel : lightModeLabel, className: this.themeMode === 'dark' ? 'deck-widget-moon' : 'deck-widget-sun' }), rootElement);
    }
    onAdd() {
        // Note: theme styling is applied in here onAdd() once DOM element is created
        this._setThemeMode(this.themeMode);
    }
    _handleClick() {
        const newThemeMode = this.themeMode === 'dark' ? 'light' : 'dark';
        this._setThemeMode(newThemeMode);
    }
    _setThemeMode(themeMode) {
        this.themeMode = themeMode;
        const container = this.rootElement?.closest('.deck-widget-container');
        if (container) {
            const themeStyle = themeMode === 'dark' ? this.props.darkModeTheme : this.props.lightModeTheme;
            applyStyles(container, themeStyle);
            const label = this.themeMode === 'dark' ? this.props.darkModeLabel : this.props.lightModeLabel;
            log.log(1, `Switched theme to ${label}`, themeStyle)();
            this.updateHTML();
        }
    }
    /** Read browser preference */
    _getInitialThemeMode() {
        const { initialThemeMode } = this.props;
        return initialThemeMode === 'auto'
            ? window.matchMedia('(prefers-color-scheme: dark)').matches
                ? 'dark'
                : 'light'
            : initialThemeMode;
    }
}
ThemeWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'theme',
    placement: 'top-left',
    viewId: null,
    lightModeLabel: 'Light Mode',
    lightModeTheme: LightGlassTheme,
    darkModeLabel: 'Dark Mode',
    darkModeTheme: DarkGlassTheme,
    initialThemeMode: 'auto'
};
//# sourceMappingURL=theme-widget.js.map