// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { _InfoWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the InfoWidget.
 */
export const InfoWidget = (props) => {
    useWidget(_InfoWidget, props);
    return null;
};
//# sourceMappingURL=info-widget.js.map