/**
 * A geocoder that uses the google geocoding service
 * @note Requires an API key from Google
 * @see https://developers.google.com/maps/documentation/geocoding/get-api-key
 */
export declare const GoogleGeocoder: {
    readonly name: "google";
    readonly requiresApiKey: true;
    readonly geocode: (address: string, apiKey: string) => Promise<{
        longitude: number;
        latitude: number;
    } | null>;
};
/**
 * A geocoder that uses the google geocoding service
 * @note Requires an API key from Mapbox
 * @see https://docs.mapbox.com/api/search/geocoding/
 */
export declare const MapboxGeocoder: {
    readonly name: "google";
    readonly requiresApiKey: true;
    readonly geocode: (address: string, apiKey: string) => Promise<{
        longitude: number;
        latitude: number;
    } | null>;
};
/**
 * A geocoder that uses the google geocoding service
 * @note Requires an API key from OpenCageData
 * @see https://opencagedata.com/api
 */
export declare const OpenCageGeocoder: {
    readonly name: "opencage";
    readonly requiresApiKey: true;
    readonly geocode: (address: string, key: string) => Promise<{
        longitude: number;
        latitude: number;
    } | null>;
};
/**
 * A geocoder adapter that wraps the browser's geolocation API. Always returns the user's current location.
 * @note Not technically a geocoder, but a geolocation service that provides a source of locations.
 * @note The user must allow location access for this to work.
 */
export declare const CurrentLocationGeocoder: {
    readonly name: "current";
    readonly requiresApiKey: false;
    /** Attempt to call browsers geolocation API */
    readonly geocode: () => Promise<{
        longitude: number;
        latitude: number;
    } | null>;
};
/**
 * Parse a coordinate string.
 * Supports comma- or semicolon-separated values.
 * Heuristically determines which value is longitude and which is latitude.
 */
export declare const CoordinatesGeocoder: {
    readonly name: "coordinates";
    readonly requiresApiKey: false;
    readonly placeholderLocation: "-122.45, 37.8 or 37°48'N, 122°27'W";
    readonly geocode: (address: string) => Promise<{
        longitude: number;
        latitude: number;
    } | null>;
};
//# sourceMappingURL=geocoders.d.ts.map