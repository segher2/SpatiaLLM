// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { _GeocoderWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
/**
 * React wrapper for the GeocoderWidget.
 */
export const GeocoderWidget = (props = {}) => {
    useWidget(_GeocoderWidget, props);
    return null;
};
//# sourceMappingURL=geocoder-widget.js.map