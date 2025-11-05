// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { ResetViewWidget as _ResetViewWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the ResetViewWidget.
 */
export const ResetViewWidget = (props = {}) => {
    useWidget(_ResetViewWidget, props);
    return null;
};
//# sourceMappingURL=reset-view-widget.js.map