import { jsx as _jsx, jsxs as _jsxs } from "preact/jsx-runtime";
// deck.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Widget } from '@deck.gl/core';
import { FlyToInterpolator, LinearInterpolator } from '@deck.gl/core';
import { render } from 'preact';
import { DropdownMenu } from "./lib/components/dropdown-menu.js";
import { GeocoderHistory } from "./lib/geocode/geocoder-history.js";
import { GoogleGeocoder, MapboxGeocoder, OpenCageGeocoder, CoordinatesGeocoder, CurrentLocationGeocoder } from "./lib/geocode/geocoders.js";
const CURRENT_LOCATION = 'current';
/**
 * A widget that display a text box that lets user type in a location
 * and a button that moves the view to that location.
 * @todo For now only supports coordinates, Could be extended with location service integrations.
 */
export class GeocoderWidget extends Widget {
    constructor(props = {}) {
        super(props);
        this.className = 'deck-widget-geocoder';
        this.placement = 'top-left';
        this.geocodeHistory = new GeocoderHistory({});
        this.addressText = '';
        this.geocoder = CoordinatesGeocoder;
        this.setInput = (text) => {
            this.addressText = text;
        };
        this.handleKeyPress = e => {
            if (e.key === 'Enter') {
                this.handleSubmit();
            }
        };
        this.handleSelect = (address) => {
            this.setInput(address);
            this.handleSubmit();
        };
        /** Sync wrapper for async geocode() */
        this.handleSubmit = () => {
            // eslint-disable-next-line @typescript-eslint/no-floating-promises
            this.geocode(this.addressText);
        };
        /** Perform geocoding */
        this.geocode = async (address) => {
            const useGeolocation = this.props._geolocation && address === CURRENT_LOCATION;
            const geocoder = useGeolocation ? CurrentLocationGeocoder : this.geocoder;
            const coordinates = await this.geocodeHistory.geocode(geocoder, this.addressText, this.props.apiKey);
            if (coordinates) {
                this.setViewState(coordinates);
            }
        };
        this.viewports = {};
        this.setProps(this.props);
    }
    setProps(props) {
        this.placement = props.placement ?? this.placement;
        this.viewId = props.viewId ?? this.viewId;
        this.geocoder = getGeocoder(this.props);
        if (this.geocoder.requiresApiKey && !this.props.apiKey) {
            throw new Error(`API key is required for the ${this.geocoder.name} geocoder`);
        }
        super.setProps(props);
    }
    onRenderHTML(rootElement) {
        const menuItems = this.props._geolocation
            ? [CURRENT_LOCATION, ...this.geocodeHistory.addressHistory]
            : [...this.geocodeHistory.addressHistory];
        render(_jsxs("div", { className: "deck-widget-geocoder", style: {
                pointerEvents: 'auto',
                display: 'flex',
                alignItems: 'center',
                flexWrap: 'wrap' // Allows wrapping on smaller screens
            }, children: [_jsx("input", { type: "text", placeholder: this.geocoder.placeholderLocation ?? 'Enter address or location', value: this.geocodeHistory.addressText, 
                    // @ts-expect-error event type
                    onInput: e => this.setInput(e.target?.value || ''), onKeyPress: this.handleKeyPress, style: {
                        flex: '1 1 auto',
                        minWidth: '200px',
                        margin: 0,
                        padding: '8px',
                        boxSizing: 'border-box'
                    } }), _jsx(DropdownMenu, { menuItems: menuItems, onSelect: this.handleSelect, style: {
                        margin: 2,
                        padding: '4px 2px',
                        boxSizing: 'border-box'
                    } }), this.geocodeHistory.errorText && (_jsx("div", { className: "error", children: this.geocodeHistory.errorText }))] }), rootElement);
    }
    // TODO - MOVE TO WIDGETIMPL?
    setViewState(viewState) {
        const viewId = this.props.viewId || viewState?.id || 'default-view';
        const viewport = this.viewports[viewId] || {};
        const nextViewState = {
            ...viewport,
            ...viewState
        };
        if (this.props.transitionDuration > 0) {
            nextViewState.transitionDuration = this.props.transitionDuration;
            nextViewState.transitionInterpolator =
                'latitude' in nextViewState ? new FlyToInterpolator() : new LinearInterpolator();
        }
        // @ts-ignore Using private method temporary until there's a public one
        this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
    onViewportChange(viewport) {
        this.viewports[viewport.id] = viewport;
    }
}
GeocoderWidget.defaultProps = {
    ...Widget.defaultProps,
    id: 'geocoder',
    viewId: null,
    placement: 'top-left',
    label: 'Geocoder',
    transitionDuration: 200,
    geocoder: 'coordinates',
    customGeocoder: CoordinatesGeocoder,
    apiKey: '',
    _geolocation: false
};
function getGeocoder(props) {
    switch (props.geocoder) {
        case 'google':
            return GoogleGeocoder;
        case 'mapbox':
            return MapboxGeocoder;
        case 'opencage':
            return OpenCageGeocoder;
        case 'coordinates':
            return CoordinatesGeocoder;
        case 'custom':
            if (!props.customGeocoder) {
                throw new Error('Custom geocoder is not defined');
            }
            return props.customGeocoder;
        default:
            throw new Error(`Unknown geocoder: ${props.geocoder}`);
    }
}
//# sourceMappingURL=geocoder-widget.js.map