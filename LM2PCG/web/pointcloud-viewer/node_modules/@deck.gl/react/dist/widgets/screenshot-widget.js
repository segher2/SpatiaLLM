// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { ScreenshotWidget as _ScreenshotWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the ScreenshotWidget.
 */
export const ScreenshotWidget = (props = {}) => {
    useWidget(_ScreenshotWidget, props);
    return null;
};
//# sourceMappingURL=screenshot-widget.js.map