// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { ZoomWidget as _ZoomWidget } from '@deck.gl/widgets';
import { useWidget } from "../utils/use-widget.js";
export const ZoomWidget = (props = {}) => {
    useWidget(_ZoomWidget, props);
    return null;
};
//# sourceMappingURL=zoom-widget.js.map