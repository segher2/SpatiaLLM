import { Widget } from '@deck.gl/core';
import type { WidgetPlacement, Viewport, WidgetProps } from '@deck.gl/core';
import { type Geocoder } from "./lib/geocode/geocoder.js";
import { GeocoderHistory } from "./lib/geocode/geocoder-history.js";
/** @todo - is the the best we can do? */
type ViewState = Record<string, unknown>;
/** Properties for the GeocoderWidget */
export type GeocoderWidgetProps = WidgetProps & {
    viewId?: string | null;
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** Tooltip message */
    label?: string;
    /** View state reset transition duration in ms. 0 disables the transition */
    transitionDuration?: number;
    /** Geocoding service selector, for declarative usage */
    geocoder?: 'google' | 'mapbox' | 'opencage' | 'coordinates' | 'custom';
    /** Custom geocoding service (Used when geocoder = 'custom') */
    customGeocoder?: Geocoder;
    /** API key used for geocoding services */
    apiKey?: string;
    /** Whether to use geolocation @note Experimental*/
    _geolocation?: boolean;
};
/**
 * A widget that display a text box that lets user type in a location
 * and a button that moves the view to that location.
 * @todo For now only supports coordinates, Could be extended with location service integrations.
 */
export declare class GeocoderWidget extends Widget<GeocoderWidgetProps> {
    static defaultProps: Required<GeocoderWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    geocodeHistory: GeocoderHistory;
    addressText: string;
    geocoder: Geocoder;
    constructor(props?: GeocoderWidgetProps);
    setProps(props: Partial<GeocoderWidgetProps>): void;
    onRenderHTML(rootElement: HTMLElement): void;
    setInput: (text: string) => void;
    handleKeyPress: (e: any) => void;
    handleSelect: (address: string) => void;
    /** Sync wrapper for async geocode() */
    handleSubmit: () => void;
    /** Perform geocoding */
    geocode: (address: string) => Promise<void>;
    setViewState(viewState: ViewState): void;
    onViewportChange(viewport: Viewport): void;
    viewports: Record<string, Viewport>;
}
export {};
//# sourceMappingURL=geocoder-widget.d.ts.map