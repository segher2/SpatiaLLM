import { type Geocoder } from "./geocoder.js";
export type GeocoderHistoryProps = {
    maxEntries?: number;
};
/**
 * An internal, experimental helper class for storing a list of locations in local storage.
 * @todo Remove the UI related state.
 */
export declare class GeocoderHistory {
    props: Required<GeocoderHistoryProps>;
    addressText: string;
    errorText: string;
    addressHistory: string[];
    constructor(props: GeocoderHistoryProps);
    /** PErform geocoding */
    geocode(geocoder: Geocoder, address: string, apiKey: string): Promise<{
        longitude: number;
        latitude: number;
    } | null>;
    loadPreviousAddresses(): string[];
    storeAddress(address: string): void;
}
//# sourceMappingURL=geocoder-history.d.ts.map