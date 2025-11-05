"use strict";
var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};
var __copyProps = (to, from, except, desc) => {
  if (from && typeof from === "object" || typeof from === "function") {
    for (let key of __getOwnPropNames(from))
      if (!__hasOwnProp.call(to, key) && key !== except)
        __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
  }
  return to;
};
var __toCommonJS = (mod) => __copyProps(__defProp({}, "__esModule", { value: true }), mod);

// dist/index.js
var dist_exports = {};
__export(dist_exports, {
  CompassWidget: () => CompassWidget,
  DarkGlassTheme: () => DarkGlassTheme,
  DarkTheme: () => DarkTheme,
  FullscreenWidget: () => FullscreenWidget,
  GimbalWidget: () => GimbalWidget,
  LightGlassTheme: () => LightGlassTheme,
  LightTheme: () => LightTheme,
  ResetViewWidget: () => ResetViewWidget,
  ScreenshotWidget: () => ScreenshotWidget,
  ZoomWidget: () => ZoomWidget,
  _ButtonGroup: () => ButtonGroup,
  _ContextMenuWidget: () => ContextMenuWidget,
  _CoordinatesGeocoder: () => CoordinatesGeocoder,
  _CurrentLocationGeocoder: () => CurrentLocationGeocoder,
  _DropdownMenu: () => DropdownMenu,
  _FpsWidget: () => FpsWidget,
  _GeocoderWidget: () => GeocoderWidget,
  _GoogleGeocoder: () => GoogleGeocoder,
  _GroupedIconButton: () => GroupedIconButton,
  _IconButton: () => IconButton,
  _IconMenu: () => IconMenu,
  _InfoWidget: () => InfoWidget,
  _LoadingWidget: () => LoadingWidget,
  _MapboxGeocoder: () => MapboxGeocoder,
  _OpenCageGeocoder: () => OpenCageGeocoder,
  _ScaleWidget: () => ScaleWidget,
  _SimpleMenu: () => SimpleMenu,
  _SplitterWidget: () => SplitterWidget,
  _StatsWidget: () => StatsWidget,
  _ThemeWidget: () => ThemeWidget,
  _TimelineWidget: () => TimelineWidget,
  _ViewSelectorWidget: () => ViewSelectorWidget
});
module.exports = __toCommonJS(dist_exports);

// dist/zoom-widget.js
var import_jsx_runtime3 = require("preact/jsx-runtime");
var import_core = require("@deck.gl/core");
var import_preact = require("preact");

// dist/lib/components/button-group.js
var import_jsx_runtime = require("preact/jsx-runtime");
var ButtonGroup = (props) => {
  const { children, orientation = "horizontal" } = props;
  return (0, import_jsx_runtime.jsx)("div", { className: `deck-widget-button-group ${orientation}`, children });
};

// dist/lib/components/grouped-icon-button.js
var import_jsx_runtime2 = require("preact/jsx-runtime");
var GroupedIconButton = (props) => {
  const { className = "", label, onClick, children } = props;
  return (0, import_jsx_runtime2.jsx)("button", { className: `deck-widget-icon-button ${className}`, type: "button", onClick, title: label, children: children ? children : (0, import_jsx_runtime2.jsx)("div", { className: "deck-widget-icon" }) });
};

// dist/zoom-widget.js
var ZoomWidget = class extends import_core.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-zoom";
    this.placement = "top-left";
    this.viewports = {};
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    const ui = (0, import_jsx_runtime3.jsxs)(ButtonGroup, { orientation: this.props.orientation, children: [(0, import_jsx_runtime3.jsx)(GroupedIconButton, { onClick: () => this.handleZoomIn(), label: this.props.zoomInLabel, className: "deck-widget-zoom-in" }), (0, import_jsx_runtime3.jsx)(GroupedIconButton, { onClick: () => this.handleZoomOut(), label: this.props.zoomOutLabel, className: "deck-widget-zoom-out" })] });
    (0, import_preact.render)(ui, rootElement);
  }
  onViewportChange(viewport) {
    this.viewports[viewport.id] = viewport;
  }
  handleZoom(viewport, nextZoom) {
    const viewId = this.viewId || (viewport == null ? void 0 : viewport.id) || "default-view";
    const nextViewState = {
      ...viewport,
      zoom: nextZoom
    };
    if (this.props.transitionDuration > 0) {
      nextViewState.transitionDuration = this.props.transitionDuration;
      nextViewState.transitionInterpolator = "latitude" in nextViewState ? new import_core.FlyToInterpolator() : new import_core.LinearInterpolator({
        transitionProps: ["zoom"]
      });
    }
    this.setViewState(viewId, nextViewState);
  }
  handleZoomIn() {
    for (const viewport of Object.values(this.viewports)) {
      this.handleZoom(viewport, viewport.zoom + 1);
    }
  }
  handleZoomOut() {
    for (const viewport of Object.values(this.viewports)) {
      this.handleZoom(viewport, viewport.zoom - 1);
    }
  }
  /** @todo - move to deck or widget manager */
  setViewState(viewId, viewState) {
    this.deck._onViewStateChange({ viewId, viewState, interactionState: {} });
  }
};
ZoomWidget.defaultProps = {
  ...import_core.Widget.defaultProps,
  id: "zoom",
  placement: "top-left",
  orientation: "vertical",
  transitionDuration: 200,
  zoomInLabel: "Zoom In",
  zoomOutLabel: "Zoom Out",
  viewId: null
};

// dist/reset-view-widget.js
var import_jsx_runtime5 = require("preact/jsx-runtime");
var import_preact2 = require("preact");
var import_core2 = require("@deck.gl/core");

// dist/lib/components/icon-button.js
var import_jsx_runtime4 = require("preact/jsx-runtime");
var IconButton = (props) => {
  const { className = "", label, onClick, children } = props;
  return (0, import_jsx_runtime4.jsx)("div", { className: "deck-widget-button", children: (0, import_jsx_runtime4.jsx)("button", { className: `deck-widget-icon-button ${className}`, type: "button", onClick, title: label, children: children ? children : (0, import_jsx_runtime4.jsx)("div", { className: "deck-widget-icon" }) }) });
};

// dist/reset-view-widget.js
var ResetViewWidget = class extends import_core2.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-reset-view";
    this.placement = "top-left";
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    (0, import_preact2.render)((0, import_jsx_runtime5.jsx)(IconButton, { className: "deck-widget-reset-focus", label: this.props.label, onClick: this.handleClick.bind(this) }), rootElement);
  }
  handleClick() {
    var _a;
    const initialViewState = this.props.initialViewState || ((_a = this.deck) == null ? void 0 : _a.props.initialViewState);
    this.setViewState(initialViewState);
  }
  setViewState(viewState) {
    const viewId = this.props.viewId || "default-view";
    const nextViewState = {
      ...viewId !== "default-view" ? viewState == null ? void 0 : viewState[viewId] : viewState
      // only works for geospatial?
      // transitionDuration: this.props.transitionDuration,
      // transitionInterpolator: new FlyToInterpolator()
    };
    this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
  }
};
ResetViewWidget.defaultProps = {
  ...import_core2.Widget.defaultProps,
  id: "reset-view",
  placement: "top-left",
  label: "Reset View",
  initialViewState: void 0,
  viewId: null
};

// dist/gimbal-widget.js
var import_jsx_runtime6 = require("preact/jsx-runtime");
var import_core3 = require("@deck.gl/core");
var import_preact3 = require("preact");
var GimbalWidget = class extends import_core3.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-gimbal";
    this.placement = "top-left";
    this.viewports = {};
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    var _a;
    const viewId = this.viewId || ((_a = Object.values(this.viewports)[0]) == null ? void 0 : _a.id) || "default-view";
    const widgetViewport = this.viewports[viewId];
    const { rotationOrbit, rotationX } = this.getNormalizedRotation(widgetViewport);
    const ui = (0, import_jsx_runtime6.jsx)("div", { className: "deck-widget-button", style: { perspective: 100, pointerEvents: "auto" }, children: (0, import_jsx_runtime6.jsxs)("button", { type: "button", onClick: () => {
      for (const viewport of Object.values(this.viewports)) {
        this.resetOrbitView(viewport);
      }
    }, title: this.props.label, style: { position: "relative", width: 26, height: 26 }, children: [(0, import_jsx_runtime6.jsx)("svg", { className: "gimbal-outer-ring", width: "100%", height: "100%", viewBox: "0 0 26 26", style: {
      position: "absolute",
      top: 0,
      left: 0,
      transform: `rotateY(${rotationOrbit}deg)`
    }, children: (0, import_jsx_runtime6.jsx)("circle", { cx: "13", cy: "13", r: "10", stroke: "var(--icon-gimbal-outer-color, rgb(68, 92, 204))", strokeWidth: this.props.strokeWidth, fill: "none" }) }), (0, import_jsx_runtime6.jsx)("svg", { className: "gimbal-inner-ring", width: "100%", height: "100%", viewBox: "0 0 26 26", style: {
      position: "absolute",
      top: 0,
      left: 0,
      transform: `rotateX(${rotationX}deg)`
    }, children: (0, import_jsx_runtime6.jsx)("circle", { cx: "13", cy: "13", r: "7", stroke: "var(--icon-gimbal-inner-color, rgb(240, 92, 68))", strokeWidth: this.props.strokeWidth, fill: "none" }) })] }) });
    (0, import_preact3.render)(ui, rootElement);
  }
  onViewportChange(viewport) {
    this.viewports[viewport.id] = viewport;
    this.updateHTML();
  }
  resetOrbitView(viewport) {
    const viewId = this.getViewId(viewport);
    const viewState = this.getViewState(viewId);
    if ("rotationOrbit" in viewState || "rotationX" in viewState) {
      const nextViewState = {
        ...viewState,
        rotationOrbit: 0,
        rotationX: 0,
        transitionDuration: this.props.transitionDuration,
        transitionInterpolator: new import_core3.LinearInterpolator({
          transitionProps: ["rotationOrbit", "rotationX"]
        })
      };
      this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
  }
  getNormalizedRotation(viewport) {
    const viewState = this.getViewState(this.getViewId(viewport));
    const [rz, rx] = this.getRotation(viewState);
    const rotationOrbit = normalizeAndClampAngle(rz);
    const rotationX = normalizeAndClampAngle(rx);
    return { rotationOrbit, rotationX };
  }
  getRotation(viewState) {
    if (viewState && ("rotationOrbit" in viewState || "rotationX" in viewState)) {
      return [-(viewState.rotationOrbit || 0), viewState.rotationX || 0];
    }
    return [0, 0];
  }
  // Move to Widget/WidgetManager?
  getViewId(viewport) {
    const viewId = this.viewId || (viewport == null ? void 0 : viewport.id) || "OrbitView";
    return viewId;
  }
  getViewState(viewId) {
    const viewManager = this.getViewManager();
    const viewState = viewId && viewManager.getViewState(viewId) || viewManager.viewState;
    return viewState;
  }
  getViewManager() {
    var _a;
    const viewManager = (_a = this.deck) == null ? void 0 : _a.viewManager;
    if (!viewManager) {
      throw new Error("wigdet must be added to a deck instance");
    }
    return viewManager;
  }
};
GimbalWidget.defaultProps = {
  ...import_core3.Widget.defaultProps,
  id: "gimbal",
  placement: "top-left",
  viewId: null,
  label: "Gimbal",
  strokeWidth: 1.5,
  transitionDuration: 200
};
function normalizeAndClampAngle(angle) {
  let normalized = ((angle + 180) % 360 + 360) % 360 - 180;
  const AVOID_ANGLE_DELTA = 10;
  const distanceFrom90 = normalized - 90;
  if (Math.abs(distanceFrom90) < AVOID_ANGLE_DELTA) {
    if (distanceFrom90 < AVOID_ANGLE_DELTA) {
      normalized = 90 + AVOID_ANGLE_DELTA;
    } else if (distanceFrom90 > -AVOID_ANGLE_DELTA) {
      normalized = 90 - AVOID_ANGLE_DELTA;
    }
  }
  return normalized;
}

// dist/compass-widget.js
var import_jsx_runtime7 = require("preact/jsx-runtime");
var import_core4 = require("@deck.gl/core");
var import_preact4 = require("preact");
var CompassWidget = class extends import_core4.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-compass";
    this.placement = "top-left";
    this.viewports = {};
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    var _a;
    const viewId = this.viewId || ((_a = Object.values(this.viewports)[0]) == null ? void 0 : _a.id) || "default-view";
    const widgetViewport = this.viewports[viewId];
    const [rz, rx] = this.getRotation(widgetViewport);
    const ui = (0, import_jsx_runtime7.jsx)("div", { className: "deck-widget-button", style: { perspective: 100 }, children: (0, import_jsx_runtime7.jsx)("button", { type: "button", onClick: () => {
      for (const viewport of Object.values(this.viewports)) {
        this.handleCompassReset(viewport);
      }
    }, title: this.props.label, style: { transform: `rotateX(${rx}deg)` }, children: (0, import_jsx_runtime7.jsx)("svg", { fill: "none", width: "100%", height: "100%", viewBox: "0 0 26 26", children: (0, import_jsx_runtime7.jsxs)("g", { transform: `rotate(${rz},13,13)`, children: [(0, import_jsx_runtime7.jsx)("path", { d: "M10 13.0001L12.9999 5L15.9997 13.0001H10Z", fill: "var(--icon-compass-north-color, rgb(240, 92, 68))" }), (0, import_jsx_runtime7.jsx)("path", { d: "M16.0002 12.9999L13.0004 21L10.0005 12.9999H16.0002Z", fill: "var(--icon-compass-south-color, rgb(204, 204, 204))" })] }) }) }) });
    (0, import_preact4.render)(ui, rootElement);
  }
  onViewportChange(viewport) {
    if (!viewport.equals(this.viewports[viewport.id])) {
      this.viewports[viewport.id] = viewport;
      this.updateHTML();
    }
  }
  getRotation(viewport) {
    if (viewport instanceof import_core4.WebMercatorViewport) {
      return [-viewport.bearing, viewport.pitch];
    } else if (viewport instanceof import_core4._GlobeViewport) {
      return [0, Math.max(-80, Math.min(80, viewport.latitude))];
    }
    return [0, 0];
  }
  handleCompassReset(viewport) {
    const viewId = this.viewId || viewport.id || "default-view";
    if (viewport instanceof import_core4.WebMercatorViewport) {
      const nextViewState = {
        ...viewport,
        bearing: 0,
        ...this.getRotation(viewport)[0] === 0 ? { pitch: 0 } : {},
        transitionDuration: this.props.transitionDuration,
        transitionInterpolator: new import_core4.FlyToInterpolator()
      };
      this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
  }
};
CompassWidget.defaultProps = {
  ...import_core4.Widget.defaultProps,
  id: "compass",
  placement: "top-left",
  viewId: null,
  label: "Reset Compass",
  transitionDuration: 200
};

// dist/scale-widget.js
var import_jsx_runtime8 = require("preact/jsx-runtime");
var import_preact5 = require("preact");
var import_core5 = require("@deck.gl/core");
var ScaleWidget = class extends import_core5.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-scale";
    this.placement = "bottom-left";
    this.scaleWidth = 10;
    this.scaleValue = 0;
    this.scaleText = "";
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    const lineOffsetX = 50;
    const svgWidth = lineOffsetX + this.scaleWidth;
    const tickHeight = 10;
    (0, import_preact5.render)((0, import_jsx_runtime8.jsxs)("svg", { className: "deck-widget-scale", width: svgWidth, height: 30, style: { overflow: "visible", background: "transparent" }, onClick: this.handleClick.bind(this), children: [(0, import_jsx_runtime8.jsx)("text", { x: lineOffsetX + 5, y: "10", textAnchor: "end", alignmentBaseline: "middle", style: { fontSize: "16px", fill: "black", fontWeight: "bold", fontFamily: "sans-serif" }, children: this.scaleText }), (0, import_jsx_runtime8.jsx)("line", { x1: lineOffsetX, y1: "15", x2: lineOffsetX + this.scaleWidth, y2: "15", stroke: "black", strokeWidth: "6" }), (0, import_jsx_runtime8.jsx)("line", { x1: lineOffsetX, y1: "15", x2: lineOffsetX, y2: 15 - tickHeight, stroke: "black", strokeWidth: "6" }), (0, import_jsx_runtime8.jsx)("line", { x1: lineOffsetX + this.scaleWidth, y1: "15", x2: lineOffsetX + this.scaleWidth, y2: 15 - tickHeight, stroke: "black", strokeWidth: "6" })] }), rootElement);
  }
  onViewportChange(viewport) {
    if (!("latitude" in viewport))
      return;
    const { latitude, zoom } = viewport;
    const metersPerPixel = getMetersPerPixel(latitude, zoom);
    const { candidate, candidatePixels } = computeScaleCandidate(metersPerPixel);
    this.scaleValue = candidate;
    this.scaleWidth = candidatePixels;
    if (candidate >= 1e3) {
      this.scaleText = `${(candidate / 1e3).toFixed(1)} km`;
    } else {
      this.scaleText = `${candidate} m`;
    }
    this.updateHTML();
  }
  handleClick() {
  }
};
ScaleWidget.defaultProps = {
  ...import_core5.Widget.defaultProps,
  id: "scale",
  placement: "bottom-left",
  label: "Scale",
  viewId: null
};
function getMetersPerPixel(latitude, zoom) {
  const earthCircumference = 40075016686e-3;
  return earthCircumference * Math.cos(latitude * Math.PI / 180) / Math.pow(2, zoom + 8);
}
function computeScaleCandidate(metersPerPixel) {
  const minPixels = 100;
  const maxPixels = 200;
  const targetPixels = (minPixels + maxPixels) / 2;
  const targetDistance = targetPixels * metersPerPixel;
  const exponent = Math.floor(Math.log10(targetDistance));
  const base = Math.pow(10, exponent);
  const multipliers = [1, 2, 5];
  let candidate = multipliers[0] * base;
  let candidatePixels = candidate / metersPerPixel;
  for (let i = 0; i < multipliers.length; i++) {
    const currentCandidate = multipliers[i] * base;
    const currentPixels = currentCandidate / metersPerPixel;
    if (currentPixels >= minPixels && currentPixels <= maxPixels) {
      candidate = currentCandidate;
      candidatePixels = currentPixels;
      break;
    }
    if (currentPixels > maxPixels) {
      candidate = i > 0 ? multipliers[i - 1] * base : currentCandidate;
      candidatePixels = candidate / metersPerPixel;
      break;
    }
    if (i === multipliers.length - 1 && currentPixels < minPixels) {
      candidate = multipliers[0] * base * 10;
      candidatePixels = candidate / metersPerPixel;
    }
  }
  return { candidate, candidatePixels };
}

// dist/geocoder-widget.js
var import_jsx_runtime10 = require("preact/jsx-runtime");
var import_core6 = require("@deck.gl/core");
var import_core7 = require("@deck.gl/core");
var import_preact6 = require("preact");

// dist/lib/components/dropdown-menu.js
var import_jsx_runtime9 = require("preact/jsx-runtime");
var import_hooks = require("preact/hooks");
var DropdownMenu = (props) => {
  const [isOpen, setIsOpen] = (0, import_hooks.useState)(false);
  const dropdownRef = (0, import_hooks.useRef)(null);
  const toggleDropdown = () => setIsOpen(!isOpen);
  const handleClickOutside = (event) => {
    if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
      setIsOpen(false);
    }
  };
  (0, import_hooks.useEffect)(() => {
    document.addEventListener("mousedown", handleClickOutside);
    return () => {
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, []);
  const handleSelect = (value) => {
    props.onSelect(value);
    setIsOpen(false);
  };
  return (0, import_jsx_runtime9.jsxs)("div", { className: "dropdown-container", ref: dropdownRef, style: {
    position: "relative",
    display: "inline-block",
    ...props.style
  }, children: [(0, import_jsx_runtime9.jsx)("button", { onClick: toggleDropdown, style: {
    width: "30px",
    height: "30px",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    border: "1px solid #ccc",
    borderRadius: "4px",
    background: "#fff",
    cursor: "pointer",
    padding: 0
  }, children: "\u25BC" }), isOpen && (0, import_jsx_runtime9.jsx)("ul", { style: {
    position: "absolute",
    top: "100%",
    right: "100%",
    background: "#fff",
    border: "1px solid #ccc",
    borderRadius: "4px",
    listStyle: "none",
    padding: "4px 0",
    margin: 0,
    zIndex: 1e3,
    minWidth: "200px"
  }, children: props.menuItems.map((item) => (0, import_jsx_runtime9.jsx)("li", { onClick: () => handleSelect(item), style: {
    padding: "4px 8px",
    cursor: "pointer",
    whiteSpace: "nowrap"
  }, children: item }, item)) })] });
};

// dist/lib/geocode/geocoder-history.js
var CURRENT_LOCATION = "current";
var LOCAL_STORAGE_KEY = "deck-geocoder-history";
var GeocoderHistory = class {
  constructor(props) {
    this.addressText = "";
    this.errorText = "";
    this.addressHistory = [];
    this.props = { maxEntries: 5, ...props };
    this.addressHistory = this.loadPreviousAddresses();
  }
  /** PErform geocoding */
  async geocode(geocoder, address, apiKey) {
    this.errorText = "";
    this.addressText = address;
    try {
      const coordinates = await geocoder.geocode(address, apiKey);
      if (coordinates) {
        this.storeAddress(this.addressText);
        return coordinates;
      }
      this.errorText = "Invalid address";
    } catch (error) {
      this.errorText = `${error.message}`;
    }
    return null;
  }
  loadPreviousAddresses() {
    try {
      const stored = window.localStorage.getItem(LOCAL_STORAGE_KEY);
      const list = stored && JSON.parse(stored);
      const addresses = Array.isArray(list) ? list.filter((v) => typeof v === "string") : [];
      return addresses;
    } catch {
    }
    return [];
  }
  storeAddress(address) {
    const cleaned = address.trim();
    if (!cleaned || cleaned === CURRENT_LOCATION) {
      return;
    }
    const deduped = [cleaned, ...this.addressHistory.filter((a) => a !== cleaned)];
    this.addressHistory = deduped.slice(0, this.props.maxEntries);
    try {
      window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(this.addressHistory));
    } catch {
    }
  }
};

// dist/lib/geocode/geocoders.js
var GOOGLE_URL = "https://maps.googleapis.com/maps/api/geocode/json";
var MAPBOX_URL = "https://api.mapbox.com/geocoding/v5/mapbox.places";
var OPENCAGE_API_URL = "https://api.opencagedata.com/geocode/v1/json";
var GoogleGeocoder = {
  name: "google",
  requiresApiKey: true,
  async geocode(address, apiKey) {
    const encodedAddress = encodeURIComponent(address);
    const json = await fetchJson(`${GOOGLE_URL}?address=${encodedAddress}&key=${apiKey}`);
    switch (json.status) {
      case "OK":
        const loc = json.results.length > 0 && json.results[0].geometry.location;
        return loc ? { longitude: loc.lng, latitude: loc.lat } : null;
      default:
        throw new Error(`Google Geocoder failed: ${json.status}`);
    }
  }
};
var MapboxGeocoder = {
  name: "google",
  requiresApiKey: true,
  async geocode(address, apiKey) {
    const encodedAddress = encodeURIComponent(address);
    const json = await fetchJson(`${MAPBOX_URL}/${encodedAddress}.json?access_token=${apiKey}`);
    if (Array.isArray(json.features) && json.features.length > 0) {
      const center = json.features[0].center;
      if (Array.isArray(center) && center.length >= 2) {
        return { longitude: center[0], latitude: center[1] };
      }
    }
    return null;
  }
};
var OpenCageGeocoder = {
  name: "opencage",
  requiresApiKey: true,
  async geocode(address, key) {
    const encodedAddress = encodeURIComponent(address);
    const data = await fetchJson(`${OPENCAGE_API_URL}?q=${encodedAddress}&key=${key}`);
    if (Array.isArray(data.results) && data.results.length > 0) {
      const geometry = data.results[0].geometry;
      return { longitude: geometry.lng, latitude: geometry.lat };
    }
    return null;
  }
};
var CurrentLocationGeocoder = {
  name: "current",
  requiresApiKey: false,
  /** Attempt to call browsers geolocation API */
  async geocode() {
    if (!navigator.geolocation) {
      throw new Error("Geolocation not supported");
    }
    return new Promise((resolve, reject) => {
      navigator.geolocation.getCurrentPosition(
        /** @see https://developer.mozilla.org/docs/Web/API/GeolocationPosition */
        (position) => {
          const { longitude, latitude } = position.coords;
          resolve({ longitude, latitude });
        },
        /** @see https://developer.mozilla.org/docs/Web/API/GeolocationPositionError */
        (error) => reject(new Error(error.message))
      );
    });
  }
};
async function fetchJson(url) {
  let response;
  try {
    response = await fetch(url);
  } catch (error) {
    throw new Error(`CORS error? ${error}. ${url}: `);
  }
  if (!response.ok) {
    throw new Error(`${response.statusText}. ${url}: `);
  }
  const data = await response.json();
  if (!data) {
    throw new Error(`No data returned. ${url}`);
  }
  return data;
}
var CoordinatesGeocoder = {
  name: "coordinates",
  requiresApiKey: false,
  placeholderLocation: `-122.45, 37.8 or 37\xB048'N, 122\xB027'W`,
  async geocode(address) {
    return parseCoordinates(address) || null;
  }
};
function parseCoordinates(input) {
  input = input.trim();
  const parts = input.split(/[,;]/).map((p) => p.trim());
  if (parts.length < 2)
    return null;
  const first = parseCoordinatePart(parts[0]);
  const second = parseCoordinatePart(parts[1]);
  if (first === null || second === null)
    return null;
  if (Math.abs(first) > 90 && Math.abs(second) <= 90) {
    return { longitude: first, latitude: second };
  } else if (Math.abs(second) > 90 && Math.abs(first) <= 90) {
    return { longitude: second, latitude: first };
  }
  return { latitude: first, longitude: second };
}
function parseCoordinatePart(s) {
  s = s.trim();
  if (s.includes("\xB0") || s.includes("'") || s.includes('"')) {
    const value2 = dmsToDecimal(s);
    return isNaN(value2) ? null : value2;
  }
  let sign = 1;
  if (/[SW]/i.test(s))
    sign = -1;
  s = s.replace(/[NSEW]/gi, "");
  const value = parseFloat(s);
  return isNaN(value) ? null : sign * value;
}
function dmsToDecimal(s) {
  const regex = /(\d+)[°d]\s*(\d+)?['′m]?\s*(\d+(?:\.\d+)?)?[\"″s]?\s*([NSEW])?/i;
  const match = s.match(regex);
  if (!match)
    return NaN;
  const degrees = parseFloat(match[1]) || 0;
  const minutes = parseFloat(match[2]) || 0;
  const seconds = parseFloat(match[3]) || 0;
  const direction = match[4] || "";
  let dec = degrees + minutes / 60 + seconds / 3600;
  if (/[SW]/i.test(direction)) {
    dec = -dec;
  }
  return dec;
}

// dist/geocoder-widget.js
var CURRENT_LOCATION2 = "current";
var GeocoderWidget = class extends import_core6.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-geocoder";
    this.placement = "top-left";
    this.geocodeHistory = new GeocoderHistory({});
    this.addressText = "";
    this.geocoder = CoordinatesGeocoder;
    this.setInput = (text) => {
      this.addressText = text;
    };
    this.handleKeyPress = (e) => {
      if (e.key === "Enter") {
        this.handleSubmit();
      }
    };
    this.handleSelect = (address) => {
      this.setInput(address);
      this.handleSubmit();
    };
    this.handleSubmit = () => {
      this.geocode(this.addressText);
    };
    this.geocode = async (address) => {
      const useGeolocation = this.props._geolocation && address === CURRENT_LOCATION2;
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
    const menuItems = this.props._geolocation ? [CURRENT_LOCATION2, ...this.geocodeHistory.addressHistory] : [...this.geocodeHistory.addressHistory];
    (0, import_preact6.render)((0, import_jsx_runtime10.jsxs)("div", { className: "deck-widget-geocoder", style: {
      pointerEvents: "auto",
      display: "flex",
      alignItems: "center",
      flexWrap: "wrap"
      // Allows wrapping on smaller screens
    }, children: [(0, import_jsx_runtime10.jsx)("input", {
      type: "text",
      placeholder: this.geocoder.placeholderLocation ?? "Enter address or location",
      value: this.geocodeHistory.addressText,
      // @ts-expect-error event type
      onInput: (e) => {
        var _a;
        return this.setInput(((_a = e.target) == null ? void 0 : _a.value) || "");
      },
      onKeyPress: this.handleKeyPress,
      style: {
        flex: "1 1 auto",
        minWidth: "200px",
        margin: 0,
        padding: "8px",
        boxSizing: "border-box"
      }
    }), (0, import_jsx_runtime10.jsx)(DropdownMenu, { menuItems, onSelect: this.handleSelect, style: {
      margin: 2,
      padding: "4px 2px",
      boxSizing: "border-box"
    } }), this.geocodeHistory.errorText && (0, import_jsx_runtime10.jsx)("div", { className: "error", children: this.geocodeHistory.errorText })] }), rootElement);
  }
  // TODO - MOVE TO WIDGETIMPL?
  setViewState(viewState) {
    const viewId = this.props.viewId || (viewState == null ? void 0 : viewState.id) || "default-view";
    const viewport = this.viewports[viewId] || {};
    const nextViewState = {
      ...viewport,
      ...viewState
    };
    if (this.props.transitionDuration > 0) {
      nextViewState.transitionDuration = this.props.transitionDuration;
      nextViewState.transitionInterpolator = "latitude" in nextViewState ? new import_core7.FlyToInterpolator() : new import_core7.LinearInterpolator();
    }
    this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
  }
  onViewportChange(viewport) {
    this.viewports[viewport.id] = viewport;
  }
};
GeocoderWidget.defaultProps = {
  ...import_core6.Widget.defaultProps,
  id: "geocoder",
  viewId: null,
  placement: "top-left",
  label: "Geocoder",
  transitionDuration: 200,
  geocoder: "coordinates",
  customGeocoder: CoordinatesGeocoder,
  apiKey: "",
  _geolocation: false
};
function getGeocoder(props) {
  switch (props.geocoder) {
    case "google":
      return GoogleGeocoder;
    case "mapbox":
      return MapboxGeocoder;
    case "opencage":
      return OpenCageGeocoder;
    case "coordinates":
      return CoordinatesGeocoder;
    case "custom":
      if (!props.customGeocoder) {
        throw new Error("Custom geocoder is not defined");
      }
      return props.customGeocoder;
    default:
      throw new Error(`Unknown geocoder: ${props.geocoder}`);
  }
}

// dist/fullscreen-widget.js
var import_jsx_runtime11 = require("preact/jsx-runtime");
var import_core8 = require("@deck.gl/core");
var import_preact7 = require("preact");
var FullscreenWidget = class extends import_core8.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-fullscreen";
    this.placement = "top-left";
    this.fullscreen = false;
    this.setProps(this.props);
  }
  onAdd() {
    document.addEventListener("fullscreenchange", this.onFullscreenChange.bind(this));
  }
  onRemove() {
    document.removeEventListener("fullscreenchange", this.onFullscreenChange.bind(this));
  }
  onRenderHTML(rootElement) {
    (0, import_preact7.render)((0, import_jsx_runtime11.jsx)(IconButton, { onClick: () => {
      this.handleClick().catch((err) => import_core8.log.error(err)());
    }, label: this.fullscreen ? this.props.exitLabel : this.props.enterLabel, className: this.fullscreen ? "deck-widget-fullscreen-exit" : "deck-widget-fullscreen-enter" }), rootElement);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  getContainer() {
    var _a, _b;
    return this.props.container || ((_b = (_a = this.deck) == null ? void 0 : _a.getCanvas()) == null ? void 0 : _b.parentElement);
  }
  onFullscreenChange() {
    const prevFullscreen = this.fullscreen;
    const fullscreen = document.fullscreenElement === this.getContainer();
    if (prevFullscreen !== fullscreen) {
      this.fullscreen = !this.fullscreen;
    }
    this.updateHTML();
  }
  async handleClick() {
    if (this.fullscreen) {
      await this.exitFullscreen();
    } else {
      await this.requestFullscreen();
    }
    this.updateHTML();
  }
  async requestFullscreen() {
    const container = this.getContainer();
    if (container == null ? void 0 : container.requestFullscreen) {
      await container.requestFullscreen({ navigationUI: "hide" });
    } else {
      this.togglePseudoFullscreen();
    }
  }
  async exitFullscreen() {
    if (document.exitFullscreen) {
      await document.exitFullscreen();
    } else {
      this.togglePseudoFullscreen();
    }
  }
  togglePseudoFullscreen() {
    var _a;
    (_a = this.getContainer()) == null ? void 0 : _a.classList.toggle("deck-pseudo-fullscreen");
  }
};
FullscreenWidget.defaultProps = {
  ...import_core8.Widget.defaultProps,
  id: "fullscreen",
  placement: "top-left",
  viewId: null,
  enterLabel: "Enter Fullscreen",
  exitLabel: "Exit Fullscreen",
  container: void 0
};

// dist/splitter-widget.js
var import_jsx_runtime12 = require("preact/jsx-runtime");
var import_preact8 = require("preact");
var import_hooks2 = require("preact/hooks");
var import_core9 = require("@deck.gl/core");
var SplitterWidget = class extends import_core9.Widget {
  constructor(props) {
    super(props);
    this.className = "deck-widget-splitter";
    this.placement = "fill";
  }
  setProps(props) {
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    rootElement.style.position = "absolute";
    rootElement.style.top = "0";
    rootElement.style.left = "0";
    rootElement.style.width = "100%";
    rootElement.style.height = "100%";
    rootElement.style.margin = "0px";
    (0, import_preact8.render)((0, import_jsx_runtime12.jsx)(Splitter, { orientation: this.props.orientation, initialSplit: this.props.initialSplit, onChange: this.props.onChange, onDragStart: this.props.onDragStart, onDragEnd: this.props.onDragEnd }), rootElement);
  }
};
SplitterWidget.defaultProps = {
  ...import_core9.Widget.defaultProps,
  id: "splitter-widget",
  viewId1: "",
  viewId2: "",
  orientation: "vertical",
  initialSplit: 0.5,
  onChange: () => {
  },
  onDragStart: () => {
  },
  onDragEnd: () => {
  }
};
function Splitter({ orientation, initialSplit, onChange, onDragStart, onDragEnd }) {
  const [split, setSplit] = (0, import_hooks2.useState)(initialSplit);
  const dragging = (0, import_hooks2.useRef)(false);
  const containerRef = (0, import_hooks2.useRef)(null);
  const handleDragStart = (event) => {
    dragging.current = true;
    onDragStart == null ? void 0 : onDragStart();
    document.addEventListener("mousemove", handleDragging);
    document.addEventListener("mouseup", handleDragEnd);
    event.preventDefault();
  };
  const handleDragging = (event) => {
    if (!dragging.current || !containerRef.current)
      return;
    const rect = containerRef.current.getBoundingClientRect();
    let newSplit;
    if (orientation === "vertical") {
      newSplit = (event.clientX - rect.left) / rect.width;
    } else {
      newSplit = (event.clientY - rect.top) / rect.height;
    }
    newSplit = Math.min(Math.max(newSplit, 0.05), 0.95);
    setSplit(newSplit);
    onChange == null ? void 0 : onChange(newSplit);
  };
  const handleDragEnd = (event) => {
    if (!dragging.current)
      return;
    dragging.current = false;
    onDragEnd == null ? void 0 : onDragEnd();
    document.removeEventListener("mousemove", handleDragging);
    document.removeEventListener("mouseup", handleDragEnd);
  };
  const splitterStyle = orientation === "vertical" ? {
    position: "absolute",
    top: 0,
    bottom: 0,
    left: `${split * 100}%`,
    width: "4px",
    cursor: "col-resize",
    background: "#ccc",
    zIndex: 10,
    pointerEvents: "auto",
    boxShadow: "inset -1px 0 0 white, inset 1px 0 0 white"
  } : {
    position: "absolute",
    left: 0,
    right: 0,
    top: `${split * 100}%`,
    height: "4px",
    cursor: "row-resize",
    background: "#ccc",
    zIndex: 10,
    pointerEvents: "auto",
    boxShadow: "inset -1px 0 0 white, inset 1px 0 0 white"
  };
  const containerStyle = {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0
  };
  return (0, import_jsx_runtime12.jsx)("div", { ref: containerRef, style: containerStyle, children: (0, import_jsx_runtime12.jsx)("div", { style: splitterStyle, onMouseDown: handleDragStart }) });
}

// dist/view-selector-widget.js
var import_jsx_runtime14 = require("preact/jsx-runtime");
var import_preact9 = require("preact");
var import_core10 = require("@deck.gl/core");

// dist/lib/components/icon-menu.js
var import_jsx_runtime13 = require("preact/jsx-runtime");
var import_hooks3 = require("preact/hooks");
function IconMenu(props) {
  const [menuOpen, setMenuOpen] = (0, import_hooks3.useState)(false);
  const containerRef = (0, import_hooks3.useRef)(null);
  const handleClickOutside = (event) => {
    if (containerRef.current && !containerRef.current.contains(event.target)) {
      setMenuOpen(false);
    }
  };
  (0, import_hooks3.useEffect)(() => {
    document.addEventListener("mousedown", handleClickOutside);
    return () => {
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, [containerRef]);
  const [selectedItem, setSelectedItem] = (0, import_hooks3.useState)(props.initialItem);
  const handleSelectItem = (item) => {
    setSelectedItem(item);
    setMenuOpen(false);
    props.onItemSelected(item);
  };
  const handleButtonClick = () => setMenuOpen(!menuOpen);
  const selectedMenuItem = props.menuItems.find((item) => item.value === selectedItem);
  const label = props.label || (selectedMenuItem == null ? void 0 : selectedMenuItem.label) || "";
  const icon = props.icon || (selectedMenuItem == null ? void 0 : selectedMenuItem.icon);
  return (0, import_jsx_runtime13.jsxs)("div", { style: { position: "relative", display: "inline-block" }, ref: containerRef, children: [(0, import_jsx_runtime13.jsx)(IconButton, { className: props.className, label, onClick: handleButtonClick, children: icon }), menuOpen && (0, import_jsx_runtime13.jsx)("div", { className: "deck-widget-icon-menu", children: (0, import_jsx_runtime13.jsx)(ButtonGroup, { orientation: "vertical", children: props.menuItems.map((item) => (0, import_jsx_runtime13.jsx)(GroupedIconButton, { label: item.label, onClick: () => handleSelectItem(item.value), children: item.icon }, item.value)) }) })] });
}

// dist/view-selector-widget.js
var ViewSelectorWidget = class extends import_core10.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-view-selector";
    this.placement = "top-left";
    this.handleSelectMode = (viewMode) => {
      this.viewMode = viewMode;
      this.updateHTML();
      this.props.onViewModeChange(viewMode);
    };
    this.viewMode = this.props.initialViewMode;
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    (0, import_preact9.render)((0, import_jsx_runtime14.jsx)(IconMenu, { className: "deck-widget-view-selector", menuItems: MENU_ITEMS.map((item) => ({
      ...item,
      icon: item.icon()
    })), initialItem: this.props.initialViewMode, onItemSelected: this.handleSelectMode }), rootElement);
  }
};
ViewSelectorWidget.defaultProps = {
  ...import_core10.Widget.defaultProps,
  id: "view-selector",
  placement: "top-left",
  viewId: null,
  label: "Split View",
  initialViewMode: "single",
  onViewModeChange: () => {
  }
};
var ICON_STYLE = { width: "24px", height: "24px" };
var ICONS = {
  single: () => (0, import_jsx_runtime14.jsx)("svg", { width: "24", height: "24", style: ICON_STYLE, children: (0, import_jsx_runtime14.jsx)("rect", { x: "4", y: "4", width: "16", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }) }),
  "split-horizontal": () => (0, import_jsx_runtime14.jsxs)("svg", { width: "24", height: "24", style: ICON_STYLE, children: [(0, import_jsx_runtime14.jsx)("rect", { x: "4", y: "4", width: "16", height: "7", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }), (0, import_jsx_runtime14.jsx)("rect", { x: "4", y: "13", width: "16", height: "7", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" })] }),
  "split-vertical": () => (0, import_jsx_runtime14.jsxs)("svg", { width: "24", height: "24", style: ICON_STYLE, children: [(0, import_jsx_runtime14.jsx)("rect", { x: "4", y: "4", width: "7", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" }), (0, import_jsx_runtime14.jsx)("rect", { x: "13", y: "4", width: "7", height: "16", stroke: "var(--button-icon-hover, rgb(24, 24, 26))", fill: "none", strokeWidth: "2" })] })
};
var MENU_ITEMS = [
  { value: "single", icon: ICONS.single, label: "Single View" },
  { value: "split-horizontal", icon: ICONS["split-horizontal"], label: "Split Horizontal" },
  { value: "split-vertical", icon: ICONS["split-vertical"], label: "Split Vertical" }
];

// dist/info-widget.js
var import_jsx_runtime15 = require("preact/jsx-runtime");
var import_core11 = require("@deck.gl/core");
var import_preact10 = require("preact");
var InfoWidget = class extends import_core11.Widget {
  constructor(props) {
    super(props);
    this.className = "deck-widget-info";
    this.placement = "fill";
    this.setProps(this.props);
  }
  setProps(props) {
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onCreateRootElement() {
    const element = super.onCreateRootElement();
    const style = { margin: "0px", top: "0px", left: "0px", position: "absolute" };
    Object.entries(style).forEach(([key, value]) => element.style.setProperty(key, value));
    return element;
  }
  onViewportChange(viewport) {
    this.viewport = viewport;
    this.updateHTML();
  }
  onHover(info) {
    if (this.props.mode === "hover" && this.props.getTooltip) {
      const tooltip = this.props.getTooltip(info, this);
      this.setProps({
        visible: tooltip !== null,
        ...tooltip,
        style: { zIndex: "1", ...tooltip == null ? void 0 : tooltip.style }
      });
    }
  }
  onClick(info) {
    var _a, _b;
    if (this.props.mode === "click" && this.props.getTooltip) {
      const tooltip = this.props.getTooltip(info, this);
      this.setProps({
        visible: tooltip !== null,
        ...tooltip
      });
      return tooltip !== null;
    }
    return ((_b = (_a = this.props).onClick) == null ? void 0 : _b.call(_a, this, info)) || false;
  }
  onAdd({ deck, viewId }) {
    this.deck = deck;
    if (!viewId) {
      this.viewport = deck.getViewports()[0];
    } else {
      this.viewport = deck.getViewports().find((viewport) => viewport.id === viewId);
    }
  }
  onRenderHTML(rootElement) {
    if (!this.viewport) {
      return;
    }
    const [longitude, latitude] = this.props.position;
    const [x, y] = this.viewport.project([longitude, latitude]);
    const minOffset = this.props.minOffset || 0;
    const gap = 10;
    const arrowHeight = 8;
    const arrowWidth = 16;
    const isAbove = y > this.viewport.height / 2;
    const background = this.props.style && this.props.style.background || "rgba(255,255,255,0.9)";
    const ui = this.props.visible ? (0, import_jsx_runtime15.jsxs)("div", { className: "popup-container", style: { position: "absolute", left: 0, top: 0 }, children: [(0, import_jsx_runtime15.jsx)("div", { className: "popup-content", style: {
      background,
      padding: "10px",
      position: "relative",
      // Include any additional styles
      ...this.props.style
    }, children: this.props.text }), (0, import_jsx_runtime15.jsx)("div", { className: "popup-arrow", style: { position: "absolute", width: "0px", height: "0px" } })] }) : null;
    (0, import_preact10.render)(ui, rootElement);
    requestAnimationFrame(() => {
      if (!this.props.visible || !rootElement.firstChild || !this.viewport)
        return;
      const container = rootElement.firstChild;
      const contentEl = container.querySelector(".popup-content");
      const arrowEl = container.querySelector(".popup-arrow");
      if (!contentEl || !arrowEl)
        return;
      const contentRect = contentEl.getBoundingClientRect();
      const popupWidth = contentRect.width;
      const popupHeight = contentRect.height;
      let computedLeft = x - popupWidth / 2;
      let computedTop;
      if (isAbove) {
        computedTop = y - gap - arrowHeight - popupHeight;
      } else {
        computedTop = y + gap + arrowHeight;
      }
      if (computedLeft < minOffset) {
        computedLeft = minOffset;
      }
      if (computedLeft + popupWidth > this.viewport.width - minOffset) {
        computedLeft = this.viewport.width - minOffset - popupWidth;
      }
      if (isAbove) {
        if (computedTop < minOffset) {
          computedTop = minOffset;
        }
      } else if (computedTop + popupHeight + arrowHeight > this.viewport.height - minOffset) {
        computedTop = this.viewport.height - minOffset - popupHeight - arrowHeight;
      }
      container.style.left = `${computedLeft}px`;
      container.style.top = `${computedTop}px`;
      container.style.transform = "";
      let arrowLeft = x - computedLeft - arrowWidth / 2;
      arrowLeft = Math.max(arrowLeft, 0);
      arrowLeft = Math.min(arrowLeft, popupWidth - arrowWidth);
      if (isAbove) {
        arrowEl.style.left = `${arrowLeft}px`;
        arrowEl.style.bottom = `-${arrowHeight}px`;
        arrowEl.style.top = "";
        arrowEl.style.borderLeft = `${arrowWidth / 2}px solid transparent`;
        arrowEl.style.borderRight = `${arrowWidth / 2}px solid transparent`;
        arrowEl.style.borderTop = `${arrowHeight}px solid ${background}`;
        arrowEl.style.borderBottom = "";
      } else {
        arrowEl.style.left = `${arrowLeft}px`;
        arrowEl.style.top = `-${arrowHeight}px`;
        arrowEl.style.bottom = "";
        arrowEl.style.borderLeft = `${arrowWidth / 2}px solid transparent`;
        arrowEl.style.borderRight = `${arrowWidth / 2}px solid transparent`;
        arrowEl.style.borderBottom = `${arrowHeight}px solid ${background}`;
        arrowEl.style.borderTop = "";
      }
    });
  }
};
InfoWidget.defaultProps = {
  ...import_core11.Widget.defaultProps,
  id: "info",
  position: [0, 0],
  text: "",
  visible: false,
  minOffset: 0,
  viewId: null,
  mode: "hover",
  getTooltip: void 0,
  onClick: void 0
};

// dist/context-menu-widget.js
var import_jsx_runtime17 = require("preact/jsx-runtime");
var import_core12 = require("@deck.gl/core");
var import_preact11 = require("preact");

// dist/lib/components/simple-menu.js
var import_jsx_runtime16 = require("preact/jsx-runtime");
var MENU_STYLE = {
  position: "absolute",
  top: "100%",
  left: 0,
  background: "white",
  border: "1px solid #ccc",
  borderRadius: "4px",
  marginTop: "var(--menu-gap, 4px)",
  zIndex: 100
};
var MENU_ITEM_STYLE = {
  background: "white",
  border: "none",
  padding: "4px",
  cursor: "pointer",
  pointerEvents: "auto"
};
var SimpleMenu = (props) => {
  const { menuItems, onItemSelected, position, style } = props;
  const styleOverride = {
    ...MENU_STYLE,
    ...style,
    left: `${position.x}px`,
    top: `${position.y}px`
  };
  return (0, import_jsx_runtime16.jsx)("div", { style: styleOverride, children: menuItems.map(({ key, label }) => (0, import_jsx_runtime16.jsx)("button", { style: { ...MENU_ITEM_STYLE, display: "block" }, onClick: (_) => onItemSelected(key), children: label }, key)) });
};

// dist/context-menu-widget.js
var MOUSE_BUTTON_RIGHT = 2;
var MOUSE_WHICH_RIGHT = 3;
var ContextMenuWidget = class extends import_core12.Widget {
  constructor(props) {
    super(props);
    this.className = "deck-widget-context-menu";
    this.placement = "fill";
    this.pickInfo = null;
    this.pickInfo = null;
    this.setProps(this.props);
  }
  onAdd({ deck }) {
    var _a, _b;
    const element = document.createElement("div");
    element.classList.add("deck-widget", "deck-widget-context-menu");
    const style = {
      margin: "0px",
      top: "0px",
      left: "0px",
      position: "absolute",
      pointerEvents: "auto"
    };
    Object.entries(style).forEach(([key, value]) => element.style.setProperty(key, value));
    (_a = deck.getCanvas()) == null ? void 0 : _a.addEventListener("click", () => this.hide());
    (_b = deck.getCanvas()) == null ? void 0 : _b.addEventListener("contextmenu", (event) => this.handleContextMenu(event));
    return element;
  }
  onRenderHTML(rootElement) {
    const { visible, position, menuItems } = this.props;
    const ui = visible && menuItems.length ? (0, import_jsx_runtime17.jsx)(SimpleMenu, { menuItems, onItemSelected: (key) => this.props.onMenuItemSelected(key, this.pickInfo), position, style: { pointerEvents: "auto" } }) : null;
    (0, import_preact11.render)(ui, rootElement);
  }
  handleContextMenu(srcEvent) {
    var _a, _b, _c;
    if (srcEvent && (srcEvent.button === MOUSE_BUTTON_RIGHT || srcEvent.which === MOUSE_WHICH_RIGHT)) {
      this.pickInfo = ((_a = this.deck) == null ? void 0 : _a.pickObject({
        x: srcEvent.clientX,
        y: srcEvent.clientY
      })) || null;
      const menuItems = this.pickInfo && ((_c = (_b = this.props).getMenuItems) == null ? void 0 : _c.call(_b, this.pickInfo, this)) || [];
      const visible = menuItems.length > 0;
      this.setProps({
        visible,
        position: { x: srcEvent.clientX, y: srcEvent.clientY },
        menuItems
      });
      this.updateHTML();
      srcEvent.preventDefault();
      return visible;
    }
    return false;
  }
  hide() {
    this.setProps({ visible: false });
  }
};
ContextMenuWidget.defaultProps = {
  ...import_core12.Widget.defaultProps,
  id: "context",
  viewId: null,
  visible: false,
  position: { x: 0, y: 0 },
  getMenuItems: void 0,
  menuItems: [],
  // eslint-disable-next-line no-console
  onMenuItemSelected: (key, pickInfo) => console.log("Context menu item selected:", key, pickInfo)
};

// dist/timeline-widget.js
var import_jsx_runtime18 = require("preact/jsx-runtime");
var import_core13 = require("@deck.gl/core");
var import_preact12 = require("preact");
var TimelineWidget = class extends import_core13.Widget {
  constructor(props = {}) {
    super(props);
    this.id = "timeline";
    this.className = "deck-widget-timeline";
    this.placement = "bottom-left";
    this.playing = false;
    this.timerId = null;
    this.handlePlayPause = () => {
      if (this.playing) {
        this.stop();
      } else {
        this.start();
      }
    };
    this.handleSliderChange = (e) => {
      const input = e.target;
      const val = Number(input.value);
      this.currentTime = val;
      this.props.onTimeChange(val);
      this.updateHTML();
    };
    this.tick = () => {
      const [min, max] = this.props.timeRange;
      let next = this.currentTime + this.props.step;
      if (next > max) {
        next = min;
      }
      this.currentTime = next;
      this.props.onTimeChange(next);
      this.updateHTML();
      if (this.playing) {
        this.timerId = window.setTimeout(this.tick, this.props.playInterval);
      }
    };
    this.currentTime = this.props.initialTime ?? this.props.timeRange[0];
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onAdd() {
    this.playing = false;
    this.timerId = null;
  }
  onRemove() {
    this.stop();
  }
  onRenderHTML(rootElement) {
    (0, import_preact12.render)((0, import_jsx_runtime18.jsxs)("div", { style: { display: "flex", alignItems: "center", pointerEvents: "auto" }, children: [(0, import_jsx_runtime18.jsx)(IconButton, { label: this.playing ? "Pause" : "Play", onClick: this.handlePlayPause, children: (0, import_jsx_runtime18.jsx)("div", { className: "text", children: this.playing ? "\u23F8" : "\u25B6" }) }), (0, import_jsx_runtime18.jsx)("input", { type: "range", className: "timeline-slider", min: this.props.timeRange[0], max: this.props.timeRange[1], step: this.props.step, value: this.currentTime, onInput: this.handleSliderChange })] }), rootElement);
  }
  start() {
    this.playing = true;
    this.updateHTML();
    this.tick();
  }
  stop() {
    this.playing = false;
    if (this.timerId !== null) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }
    this.updateHTML();
  }
};
TimelineWidget.defaultProps = {
  ...import_core13.Widget.defaultProps,
  id: "timeline",
  placement: "bottom-left",
  viewId: null,
  timeRange: [0, 100],
  step: 1,
  initialTime: void 0,
  onTimeChange: () => {
  },
  playInterval: 1e3
};

// dist/screenshot-widget.js
var import_jsx_runtime19 = require("preact/jsx-runtime");
var import_preact13 = require("preact");
var import_core14 = require("@deck.gl/core");
var ScreenshotWidget = class extends import_core14.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-screenshot";
    this.placement = "top-left";
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    (0, import_preact13.render)((0, import_jsx_runtime19.jsx)(IconButton, { className: "deck-widget-camera", label: this.props.label, onClick: this.handleClick.bind(this) }), rootElement);
  }
  handleClick() {
    if (this.props.onCapture) {
      this.props.onCapture(this);
      return;
    }
    const dataURL = this.captureScreenToDataURL(this.props.imageFormat);
    if (dataURL) {
      this.downloadDataURL(dataURL, this.props.filename);
    }
  }
  /** @note only captures canvas contents, not HTML DOM or CSS styles */
  captureScreenToDataURL(imageFormat) {
    var _a;
    const canvas = (_a = this.deck) == null ? void 0 : _a.getCanvas();
    return canvas == null ? void 0 : canvas.toDataURL(imageFormat);
  }
  /** Download a data URL */
  downloadDataURL(dataURL, filename) {
    const link = document.createElement("a");
    link.href = dataURL;
    link.download = filename;
    link.click();
  }
};
ScreenshotWidget.defaultProps = {
  ...import_core14.Widget.defaultProps,
  id: "screenshot",
  placement: "top-left",
  viewId: null,
  label: "Screenshot",
  filename: "screenshot.png",
  imageFormat: "image/png",
  onCapture: void 0
};

// dist/theme-widget.js
var import_jsx_runtime20 = require("preact/jsx-runtime");
var import_core15 = require("@deck.gl/core");
var import_core16 = require("@deck.gl/core");
var import_preact14 = require("preact");

// dist/themes.js
var LightTheme = {
  "--widget-margin": "12px",
  "--button-size": "28px",
  "--button-corner-radius": "8px",
  "--button-background": "#fff",
  "--button-stroke": "rgba(255, 255, 255, 0.3)",
  "--button-inner-stroke": "unset",
  "--button-shadow": "0px 0px 8px 0px rgba(0, 0, 0, 0.25)",
  "--button-backdrop-filter": "unset",
  "--button-icon-idle": "rgba(97, 97, 102, 1)",
  "--button-icon-hover": "rgba(24, 24, 26, 1)",
  "--button-text": "rgb(24, 24, 26, 1)",
  "--icon-compass-north-color": "rgb(240, 92, 68)",
  "--icon-compass-south-color": "rgb(204, 204, 204)",
  "--menu-gap": "4px"
};
var DarkTheme = {
  "--widget-margin": "12px",
  "--button-size": "28px",
  "--button-corner-radius": "8px",
  "--button-background": "rgba(18, 18, 20, 1)",
  "--button-stroke": "rgba(18, 18, 20, 0.30)",
  "--button-inner-stroke": "unset",
  "--button-shadow": "0px 0px 8px 0px rgba(0, 0, 0, 0.25)",
  "--button-backdrop-filter": "unset",
  "--button-icon-idle": "rgba(158, 157, 168, 1)",
  "--button-icon-hover": "rgba(215, 214, 229, 1)",
  "--button-text": "rgb(215, 214, 229, 1)",
  "--icon-compass-north-color": "rgb(240, 92, 68)",
  "--icon-compass-south-color": "rgb(200, 199, 209)",
  "--menu-gap": "4px"
};
var LightGlassTheme = {
  "--widget-margin": "12px",
  "--button-size": "28px",
  "--button-corner-radius": "8px",
  "--button-background": "rgba(255, 255, 255, 0.6)",
  "--button-stroke": "rgba(255, 255, 255, 0.3)",
  "--button-inner-stroke": "1px solid rgba(255, 255, 255, 0.6)",
  "--button-shadow": "0px 0px 8px 0px rgba(0, 0, 0, 0.25), 0px 0px 8px 0px rgba(0, 0, 0, 0.1) inset",
  "--button-backdrop-filter": "blur(4px)",
  "--button-icon-idle": "rgba(97, 97, 102, 1)",
  "--button-icon-hover": "rgba(24, 24, 26, 1)",
  "--button-text": "rgb(24, 24, 26, 1)",
  "--icon-compass-north-color": "rgb(240, 92, 68)",
  "--icon-compass-south-color": "rgb(204, 204, 204)",
  "--menu-gap": "4px"
};
var DarkGlassTheme = {
  "--widget-margin": "12px",
  "--button-size": "28px",
  "--button-corner-radius": "8px",
  "--button-background": "rgba(18, 18, 20, 0.75)",
  "--button-stroke": "rgba(18, 18, 20, 0.30)",
  "--button-inner-stroke": "1px solid rgba(18, 18, 20, 0.75)",
  "--button-shadow": "0px 0px 8px 0px rgba(0, 0, 0, 0.25), 0px 0px 8px 0px rgba(0, 0, 0, 0.1) inset",
  "--button-backdrop-filter": "blur(4px)",
  "--button-icon-idle": "rgba(158, 157, 168, 1)",
  "--button-icon-hover": "rgba(215, 214, 229, 1)",
  "--button-text": "rgb(215, 214, 229, 1)",
  "--icon-compass-north-color": "rgb(240, 92, 68)",
  "--icon-compass-south-color": "rgb(200, 199, 209)",
  "--menu-gap": "4px"
};

// dist/theme-widget.js
var ThemeWidget = class extends import_core16.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-theme";
    this.placement = "top-left";
    this.themeMode = "dark";
    this.themeMode = this._getInitialThemeMode();
    this.setProps(this.props);
  }
  // eslint-disable-next-line complexity
  setProps(props) {
    const { lightModeTheme, darkModeTheme } = this.props;
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
    switch (this.themeMode) {
      case "light":
        if (props.lightModeTheme && !(0, import_core15._deepEqual)(props.lightModeTheme, lightModeTheme, 1)) {
          this._setThemeMode("light");
        }
        break;
      case "dark":
        if (props.darkModeTheme && !(0, import_core15._deepEqual)(props.darkModeTheme, darkModeTheme, 1)) {
          this._setThemeMode("dark");
        }
        break;
      default:
        import_core15.log.warn(`Invalid theme mode ${this.themeMode}`)();
    }
  }
  onRenderHTML(rootElement) {
    const { lightModeLabel, darkModeLabel } = this.props;
    (0, import_preact14.render)((0, import_jsx_runtime20.jsx)(IconButton, { onClick: this._handleClick.bind(this), label: this.themeMode === "dark" ? darkModeLabel : lightModeLabel, className: this.themeMode === "dark" ? "deck-widget-moon" : "deck-widget-sun" }), rootElement);
  }
  onAdd() {
    this._setThemeMode(this.themeMode);
  }
  _handleClick() {
    const newThemeMode = this.themeMode === "dark" ? "light" : "dark";
    this._setThemeMode(newThemeMode);
  }
  _setThemeMode(themeMode) {
    var _a;
    this.themeMode = themeMode;
    const container = (_a = this.rootElement) == null ? void 0 : _a.closest(".deck-widget-container");
    if (container) {
      const themeStyle = themeMode === "dark" ? this.props.darkModeTheme : this.props.lightModeTheme;
      (0, import_core15._applyStyles)(container, themeStyle);
      const label = this.themeMode === "dark" ? this.props.darkModeLabel : this.props.lightModeLabel;
      import_core15.log.log(1, `Switched theme to ${label}`, themeStyle)();
      this.updateHTML();
    }
  }
  /** Read browser preference */
  _getInitialThemeMode() {
    const { initialThemeMode } = this.props;
    return initialThemeMode === "auto" ? window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light" : initialThemeMode;
  }
};
ThemeWidget.defaultProps = {
  ...import_core16.Widget.defaultProps,
  id: "theme",
  placement: "top-left",
  viewId: null,
  lightModeLabel: "Light Mode",
  lightModeTheme: LightGlassTheme,
  darkModeLabel: "Dark Mode",
  darkModeTheme: DarkGlassTheme,
  initialThemeMode: "auto"
};

// dist/loading-widget.js
var import_jsx_runtime21 = require("preact/jsx-runtime");
var import_preact15 = require("preact");
var import_core17 = require("@deck.gl/core");
var LoadingWidget = class extends import_core17.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-loading";
    this.placement = "top-left";
    this.loading = true;
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onRenderHTML(rootElement) {
    (0, import_preact15.render)(
      // TODO(ibgreen) - this should not be a button, but styling is so nested that it is easier to reuse this component.
      this.loading && (0, import_jsx_runtime21.jsx)(IconButton, { className: "deck-widget-spinner", label: this.props.label, onClick: this.handleClick.bind(this) }),
      rootElement
    );
  }
  onRedraw({ layers }) {
    const loading = layers.some((layer) => !layer.isLoaded);
    if (loading !== this.loading) {
      this.loading = loading;
      this.updateHTML();
    }
  }
  // TODO(ibgreen) - this should not be a button, see above.
  handleClick() {
  }
};
LoadingWidget.defaultProps = {
  ...import_core17.Widget.defaultProps,
  id: "loading",
  placement: "top-left",
  viewId: null,
  label: "Loading layer data"
};

// dist/fps-widget.js
var import_jsx_runtime22 = require("preact/jsx-runtime");
var import_core18 = require("@deck.gl/core");
var import_preact16 = require("preact");
var FpsWidget = class extends import_core18.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-fps";
    this.placement = "top-left";
    this._lastFps = -1;
    this.setProps(this.props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    super.setProps(props);
  }
  onAdd({}) {
    this._lastFps = this._getFps();
    requestAnimationFrame(() => this._animate());
  }
  onRenderHTML(rootElement) {
    const fps = this._getFps();
    (0, import_preact16.render)((0, import_jsx_runtime22.jsx)(IconButton, { children: (0, import_jsx_runtime22.jsxs)("div", { className: "text", children: ["FPS", (0, import_jsx_runtime22.jsx)("br", {}), fps] }) }), rootElement);
  }
  _animate() {
    const fps = this._getFps();
    if (this._lastFps !== fps) {
      this._lastFps = fps;
      this.updateHTML();
    }
    requestAnimationFrame(() => this._animate());
  }
  _getFps() {
    var _a;
    return Math.round(((_a = this.deck) == null ? void 0 : _a.metrics.fps) ?? 0);
  }
};
FpsWidget.defaultProps = {
  ...import_core18.Widget.defaultProps,
  id: "fps",
  placement: "top-left",
  viewId: null
};

// dist/stats-widget.js
var import_jsx_runtime23 = require("preact/jsx-runtime");
var import_core19 = require("@deck.gl/core");
var import_core20 = require("@luma.gl/core");
var import_preact17 = require("preact");
var RIGHT_ARROW = "\u25B6";
var DOWN_ARROW = "\u2B07";
var DEFAULT_COUNT_FORMATTER = (stat) => `${stat.name}: ${stat.count}`;
function formatTime(time) {
  return time < 1e3 ? `${time.toFixed(2)}ms` : `${(time / 1e3).toFixed(2)}s`;
}
function formatMemory(bytes) {
  const mb = bytes / 1e6;
  return `${mb.toFixed(1)} MB`;
}
var DEFAULT_FORMATTERS = {
  count: DEFAULT_COUNT_FORMATTER,
  averageTime: (stat) => `${stat.name}: ${formatTime(stat.getAverageTime())}`,
  totalTime: (stat) => `${stat.name}: ${formatTime(stat.time)}`,
  fps: (stat) => `${stat.name}: ${Math.round(stat.getHz())}fps`,
  memory: (stat) => `${stat.name}: ${formatMemory(stat.count)}`
};
var StatsWidget = class extends import_core19.Widget {
  constructor(props = {}) {
    super(props);
    this.className = "deck-widget-stats";
    this.placement = "top-left";
    this._counter = 0;
    this.collapsed = true;
    this._toggleCollapsed = () => {
      this.collapsed = !this.collapsed;
      this.updateHTML();
    };
    this._formatters = { ...DEFAULT_FORMATTERS };
    this._resetOnUpdate = { ...this.props.resetOnUpdate };
    this._stats = this.props.stats;
    this.setProps(props);
  }
  setProps(props) {
    this.placement = props.placement ?? this.placement;
    this.viewId = props.viewId ?? this.viewId;
    this._stats = this._getStats();
    if (props.formatters) {
      for (const name in props.formatters) {
        const f = props.formatters[name];
        this._formatters[name] = typeof f === "string" ? DEFAULT_FORMATTERS[f] || DEFAULT_COUNT_FORMATTER : f;
      }
    }
    if (props.resetOnUpdate) {
      this._resetOnUpdate = { ...props.resetOnUpdate };
    }
    super.setProps(props);
  }
  onAdd() {
    this._stats = this._getStats();
    this.updateHTML();
  }
  onRenderHTML(rootElement) {
    const stats = this._stats;
    const collapsed = this.collapsed;
    const title = this.props.title || (stats == null ? void 0 : stats.id) || "Stats";
    const items = [];
    if (!collapsed && stats) {
      stats.forEach((stat) => {
        const lines = this._getLines(stat);
        if (this._resetOnUpdate && this._resetOnUpdate[stat.name]) {
          stat.reset();
        }
        lines.forEach((line, i) => {
          items.push((0, import_jsx_runtime23.jsx)("div", { style: { whiteSpace: "pre" }, children: line }, `${stat.name}-${i}`));
        });
      });
    }
    (0, import_preact17.render)((0, import_jsx_runtime23.jsxs)("div", { className: "deck-widget-stats-container", style: { cursor: "default" }, children: [(0, import_jsx_runtime23.jsxs)("div", { className: "deck-widget-stats-header", style: { cursor: "pointer", pointerEvents: "auto" }, onClick: this._toggleCollapsed, children: [collapsed ? RIGHT_ARROW : DOWN_ARROW, " ", title] }), !collapsed && (0, import_jsx_runtime23.jsx)("div", { className: "deck-widget-stats-content", children: items })] }), rootElement);
  }
  onRedraw() {
    const framesPerUpdate = Math.max(1, this.props.framesPerUpdate || 1);
    if (this._counter++ % framesPerUpdate === 0) {
      this._stats = this._getStats();
      this.updateHTML();
    }
  }
  _getStats() {
    var _a, _b;
    switch (this.props.type) {
      case "deck":
        return (_a = this.deck) == null ? void 0 : _a.stats;
      case "luma":
        return Array.from(import_core20.luma.stats.stats.values())[0];
      case "device":
        const device = (_b = this.deck) == null ? void 0 : _b.device;
        const stats = device == null ? void 0 : device.statsManager.stats.values();
        return stats ? Array.from(stats)[0] : void 0;
      case "custom":
        return this.props.stats;
      default:
        throw new Error(`Unknown stats type: ${this.props.type}`);
    }
  }
  _getLines(stat) {
    const formatter = this._formatters[stat.name] || this._formatters[stat.type || ""] || DEFAULT_COUNT_FORMATTER;
    return formatter(stat).split("\n");
  }
};
StatsWidget.defaultProps = {
  ...import_core19.Widget.defaultProps,
  type: "deck",
  placement: "top-left",
  viewId: null,
  stats: void 0,
  title: "Stats",
  framesPerUpdate: 1,
  formatters: {},
  resetOnUpdate: {},
  id: "stats"
};
//# sourceMappingURL=index.cjs.map
