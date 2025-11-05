// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { _LoadingWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the LoadingWidget.
 */
export const LoadingWidget = (props = {}) => {
    useWidget(_LoadingWidget, props);
    return null;
};
//# sourceMappingURL=loading-widget.js.map