// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
const CURRENT_LOCATION = 'current';
const LOCAL_STORAGE_KEY = 'deck-geocoder-history';
/**
 * An internal, experimental helper class for storing a list of locations in local storage.
 * @todo Remove the UI related state.
 */
export class GeocoderHistory {
    constructor(props) {
        this.addressText = '';
        this.errorText = '';
        this.addressHistory = [];
        this.props = { maxEntries: 5, ...props };
        this.addressHistory = this.loadPreviousAddresses();
    }
    /** PErform geocoding */
    async geocode(geocoder, address, apiKey) {
        this.errorText = '';
        this.addressText = address;
        try {
            const coordinates = await geocoder.geocode(address, apiKey);
            if (coordinates) {
                this.storeAddress(this.addressText);
                return coordinates;
            }
            this.errorText = 'Invalid address';
        }
        catch (error) {
            this.errorText = `${error.message}`;
        }
        return null;
    }
    loadPreviousAddresses() {
        try {
            const stored = window.localStorage.getItem(LOCAL_STORAGE_KEY);
            const list = stored && JSON.parse(stored);
            const addresses = Array.isArray(list)
                ? list.filter((v) => typeof v === 'string')
                : [];
            return addresses;
        }
        catch {
            // ignore
        }
        return [];
    }
    storeAddress(address) {
        const cleaned = address.trim();
        if (!cleaned || cleaned === CURRENT_LOCATION) {
            return;
        }
        const deduped = [cleaned, ...this.addressHistory.filter(a => a !== cleaned)];
        this.addressHistory = deduped.slice(0, this.props.maxEntries);
        try {
            window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(this.addressHistory));
        }
        catch {
            // ignore
        }
    }
}
//# sourceMappingURL=geocoder-history.js.map