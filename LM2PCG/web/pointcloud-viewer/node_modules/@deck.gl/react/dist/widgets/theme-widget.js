// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { _ThemeWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the ThemeWidget.
 */
export const ThemeWidget = (props = {}) => {
    useWidget(_ThemeWidget, props);
    return null;
};
//# sourceMappingURL=theme-widget.js.map