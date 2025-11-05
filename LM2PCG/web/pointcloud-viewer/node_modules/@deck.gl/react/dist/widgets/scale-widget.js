// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { _ScaleWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the ScaleWidget.
 */
export const ScaleWidget = (props = {}) => {
    useWidget(_ScaleWidget, props);
    return null;
};
//# sourceMappingURL=scale-widget.js.map