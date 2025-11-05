(function webpackUniversalModuleDefinition(root, factory) {
  if (typeof exports === 'object' && typeof module === 'object')
    module.exports = factory();
  else if (typeof define === 'function' && define.amd) define([], factory);
        else if (typeof exports === 'object') exports['deck'] = factory();
  else root['deck'] = factory();})(globalThis, function () {
"use strict";
var __exports__ = (() => {
  var __create = Object.create;
  var __defProp = Object.defineProperty;
  var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
  var __getOwnPropNames = Object.getOwnPropertyNames;
  var __getProtoOf = Object.getPrototypeOf;
  var __hasOwnProp = Object.prototype.hasOwnProperty;
  var __commonJS = (cb, mod) => function __require() {
    return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
  };
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
  var __reExport = (target, mod, secondTarget) => (__copyProps(target, mod, "default"), secondTarget && __copyProps(secondTarget, mod, "default"));
  var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
    // If the importer is in node compatibility mode or this is not an ESM
    // file that has been converted to a CommonJS file using a Babel-
    // compatible transform (i.e. "__esModule" has not been set), then set
    // "default" to the CommonJS "module.exports" for node compatibility.
    isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
    mod
  ));
  var __toCommonJS = (mod) => __copyProps(__defProp({}, "__esModule", { value: true }), mod);

  // external-global-plugin:@deck.gl/core
  var require_core = __commonJS({
    "external-global-plugin:@deck.gl/core"(exports, module) {
      module.exports = globalThis.deck;
    }
  });

  // external-global-plugin:@luma.gl/core
  var require_core2 = __commonJS({
    "external-global-plugin:@luma.gl/core"(exports, module) {
      module.exports = globalThis.luma;
    }
  });

  // bundle.ts
  var bundle_exports = {};
  __export(bundle_exports, {
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

  // ../core/bundle/peer-dependency.ts
  var peer_dependency_exports = {};
  var import_core = __toESM(require_core(), 1);
  __reExport(peer_dependency_exports, __toESM(require_core(), 1));
  if (!import_core.Layer) {
    throw new Error("@deck.gl/core is not found");
  }

  // bundle.ts
  __reExport(bundle_exports, peer_dependency_exports);

  // src/zoom-widget.tsx
  var import_core2 = __toESM(require_core(), 1);

  // ../../node_modules/preact/dist/preact.module.js
  var n;
  var l;
  var u;
  var t;
  var i;
  var o;
  var r;
  var f;
  var e;
  var c;
  var s;
  var a;
  var h = {};
  var p = [];
  var v = /acit|ex(?:s|g|n|p|$)|rph|grid|ows|mnc|ntw|ine[ch]|zoo|^ord|itera/i;
  var y = Array.isArray;
  function d(n2, l3) {
    for (var u4 in l3)
      n2[u4] = l3[u4];
    return n2;
  }
  function w(n2) {
    var l3 = n2.parentNode;
    l3 && l3.removeChild(n2);
  }
  function _(l3, u4, t3) {
    var i4, o3, r3, f4 = {};
    for (r3 in u4)
      "key" == r3 ? i4 = u4[r3] : "ref" == r3 ? o3 = u4[r3] : f4[r3] = u4[r3];
    if (arguments.length > 2 && (f4.children = arguments.length > 3 ? n.call(arguments, 2) : t3), "function" == typeof l3 && null != l3.defaultProps)
      for (r3 in l3.defaultProps)
        void 0 === f4[r3] && (f4[r3] = l3.defaultProps[r3]);
    return g(l3, f4, i4, o3, null);
  }
  function g(n2, t3, i4, o3, r3) {
    var f4 = { type: n2, props: t3, key: i4, ref: o3, __k: null, __: null, __b: 0, __e: null, __d: void 0, __c: null, constructor: void 0, __v: null == r3 ? ++u : r3, __i: -1, __u: 0 };
    return null == r3 && null != l.vnode && l.vnode(f4), f4;
  }
  function k(n2) {
    return n2.children;
  }
  function b(n2, l3) {
    this.props = n2, this.context = l3;
  }
  function x(n2, l3) {
    if (null == l3)
      return n2.__ ? x(n2.__, n2.__i + 1) : null;
    for (var u4; l3 < n2.__k.length; l3++)
      if (null != (u4 = n2.__k[l3]) && null != u4.__e)
        return u4.__e;
    return "function" == typeof n2.type ? x(n2) : null;
  }
  function C(n2) {
    var l3, u4;
    if (null != (n2 = n2.__) && null != n2.__c) {
      for (n2.__e = n2.__c.base = null, l3 = 0; l3 < n2.__k.length; l3++)
        if (null != (u4 = n2.__k[l3]) && null != u4.__e) {
          n2.__e = n2.__c.base = u4.__e;
          break;
        }
      return C(n2);
    }
  }
  function M(n2) {
    (!n2.__d && (n2.__d = true) && i.push(n2) && !P.__r++ || o !== l.debounceRendering) && ((o = l.debounceRendering) || r)(P);
  }
  function P() {
    var n2, u4, t3, o3, r3, e3, c3, s3;
    for (i.sort(f); n2 = i.shift(); )
      n2.__d && (u4 = i.length, o3 = void 0, e3 = (r3 = (t3 = n2).__v).__e, c3 = [], s3 = [], t3.__P && ((o3 = d({}, r3)).__v = r3.__v + 1, l.vnode && l.vnode(o3), O(t3.__P, o3, r3, t3.__n, t3.__P.namespaceURI, 32 & r3.__u ? [e3] : null, c3, null == e3 ? x(r3) : e3, !!(32 & r3.__u), s3), o3.__v = r3.__v, o3.__.__k[o3.__i] = o3, j(c3, o3, s3), o3.__e != e3 && C(o3)), i.length > u4 && i.sort(f));
    P.__r = 0;
  }
  function S(n2, l3, u4, t3, i4, o3, r3, f4, e3, c3, s3) {
    var a3, v3, y3, d3, w3, _2 = t3 && t3.__k || p, g2 = l3.length;
    for (u4.__d = e3, $(u4, l3, _2), e3 = u4.__d, a3 = 0; a3 < g2; a3++)
      null != (y3 = u4.__k[a3]) && "boolean" != typeof y3 && "function" != typeof y3 && (v3 = -1 === y3.__i ? h : _2[y3.__i] || h, y3.__i = a3, O(n2, y3, v3, i4, o3, r3, f4, e3, c3, s3), d3 = y3.__e, y3.ref && v3.ref != y3.ref && (v3.ref && N(v3.ref, null, y3), s3.push(y3.ref, y3.__c || d3, y3)), null == w3 && null != d3 && (w3 = d3), 65536 & y3.__u || v3.__k === y3.__k ? e3 = I(y3, e3, n2) : "function" == typeof y3.type && void 0 !== y3.__d ? e3 = y3.__d : d3 && (e3 = d3.nextSibling), y3.__d = void 0, y3.__u &= -196609);
    u4.__d = e3, u4.__e = w3;
  }
  function $(n2, l3, u4) {
    var t3, i4, o3, r3, f4, e3 = l3.length, c3 = u4.length, s3 = c3, a3 = 0;
    for (n2.__k = [], t3 = 0; t3 < e3; t3++)
      r3 = t3 + a3, null != (i4 = n2.__k[t3] = null == (i4 = l3[t3]) || "boolean" == typeof i4 || "function" == typeof i4 ? null : "string" == typeof i4 || "number" == typeof i4 || "bigint" == typeof i4 || i4.constructor == String ? g(null, i4, null, null, null) : y(i4) ? g(k, { children: i4 }, null, null, null) : void 0 === i4.constructor && i4.__b > 0 ? g(i4.type, i4.props, i4.key, i4.ref ? i4.ref : null, i4.__v) : i4) ? (i4.__ = n2, i4.__b = n2.__b + 1, f4 = L(i4, u4, r3, s3), i4.__i = f4, o3 = null, -1 !== f4 && (s3--, (o3 = u4[f4]) && (o3.__u |= 131072)), null == o3 || null === o3.__v ? (-1 == f4 && a3--, "function" != typeof i4.type && (i4.__u |= 65536)) : f4 !== r3 && (f4 == r3 - 1 ? a3-- : f4 == r3 + 1 ? a3++ : f4 > r3 ? s3 > e3 - r3 ? a3 += f4 - r3 : a3-- : f4 < r3 && (f4 == r3 - a3 ? a3 -= f4 - r3 : a3++), f4 !== t3 + a3 && (i4.__u |= 65536))) : (o3 = u4[r3]) && null == o3.key && o3.__e && 0 == (131072 & o3.__u) && (o3.__e == n2.__d && (n2.__d = x(o3)), V(o3, o3, false), u4[r3] = null, s3--);
    if (s3)
      for (t3 = 0; t3 < c3; t3++)
        null != (o3 = u4[t3]) && 0 == (131072 & o3.__u) && (o3.__e == n2.__d && (n2.__d = x(o3)), V(o3, o3));
  }
  function I(n2, l3, u4) {
    var t3, i4;
    if ("function" == typeof n2.type) {
      for (t3 = n2.__k, i4 = 0; t3 && i4 < t3.length; i4++)
        t3[i4] && (t3[i4].__ = n2, l3 = I(t3[i4], l3, u4));
      return l3;
    }
    n2.__e != l3 && (l3 && n2.type && !u4.contains(l3) && (l3 = x(n2)), u4.insertBefore(n2.__e, l3 || null), l3 = n2.__e);
    do {
      l3 = l3 && l3.nextSibling;
    } while (null != l3 && 8 === l3.nodeType);
    return l3;
  }
  function L(n2, l3, u4, t3) {
    var i4 = n2.key, o3 = n2.type, r3 = u4 - 1, f4 = u4 + 1, e3 = l3[u4];
    if (null === e3 || e3 && i4 == e3.key && o3 === e3.type && 0 == (131072 & e3.__u))
      return u4;
    if (t3 > (null != e3 && 0 == (131072 & e3.__u) ? 1 : 0))
      for (; r3 >= 0 || f4 < l3.length; ) {
        if (r3 >= 0) {
          if ((e3 = l3[r3]) && 0 == (131072 & e3.__u) && i4 == e3.key && o3 === e3.type)
            return r3;
          r3--;
        }
        if (f4 < l3.length) {
          if ((e3 = l3[f4]) && 0 == (131072 & e3.__u) && i4 == e3.key && o3 === e3.type)
            return f4;
          f4++;
        }
      }
    return -1;
  }
  function T(n2, l3, u4) {
    "-" === l3[0] ? n2.setProperty(l3, null == u4 ? "" : u4) : n2[l3] = null == u4 ? "" : "number" != typeof u4 || v.test(l3) ? u4 : u4 + "px";
  }
  function A(n2, l3, u4, t3, i4) {
    var o3;
    n:
      if ("style" === l3)
        if ("string" == typeof u4)
          n2.style.cssText = u4;
        else {
          if ("string" == typeof t3 && (n2.style.cssText = t3 = ""), t3)
            for (l3 in t3)
              u4 && l3 in u4 || T(n2.style, l3, "");
          if (u4)
            for (l3 in u4)
              t3 && u4[l3] === t3[l3] || T(n2.style, l3, u4[l3]);
        }
      else if ("o" === l3[0] && "n" === l3[1])
        o3 = l3 !== (l3 = l3.replace(/(PointerCapture)$|Capture$/i, "$1")), l3 = l3.toLowerCase() in n2 || "onFocusOut" === l3 || "onFocusIn" === l3 ? l3.toLowerCase().slice(2) : l3.slice(2), n2.l || (n2.l = {}), n2.l[l3 + o3] = u4, u4 ? t3 ? u4.u = t3.u : (u4.u = e, n2.addEventListener(l3, o3 ? s : c, o3)) : n2.removeEventListener(l3, o3 ? s : c, o3);
      else {
        if ("http://www.w3.org/2000/svg" == i4)
          l3 = l3.replace(/xlink(H|:h)/, "h").replace(/sName$/, "s");
        else if ("width" != l3 && "height" != l3 && "href" != l3 && "list" != l3 && "form" != l3 && "tabIndex" != l3 && "download" != l3 && "rowSpan" != l3 && "colSpan" != l3 && "role" != l3 && "popover" != l3 && l3 in n2)
          try {
            n2[l3] = null == u4 ? "" : u4;
            break n;
          } catch (n3) {
          }
        "function" == typeof u4 || (null == u4 || false === u4 && "-" !== l3[4] ? n2.removeAttribute(l3) : n2.setAttribute(l3, "popover" == l3 && 1 == u4 ? "" : u4));
      }
  }
  function F(n2) {
    return function(u4) {
      if (this.l) {
        var t3 = this.l[u4.type + n2];
        if (null == u4.t)
          u4.t = e++;
        else if (u4.t < t3.u)
          return;
        return t3(l.event ? l.event(u4) : u4);
      }
    };
  }
  function O(n2, u4, t3, i4, o3, r3, f4, e3, c3, s3) {
    var a3, h4, p3, v3, w3, _2, g2, m2, x2, C3, M2, P2, $2, I2, H, L2, T3 = u4.type;
    if (void 0 !== u4.constructor)
      return null;
    128 & t3.__u && (c3 = !!(32 & t3.__u), r3 = [e3 = u4.__e = t3.__e]), (a3 = l.__b) && a3(u4);
    n:
      if ("function" == typeof T3)
        try {
          if (m2 = u4.props, x2 = "prototype" in T3 && T3.prototype.render, C3 = (a3 = T3.contextType) && i4[a3.__c], M2 = a3 ? C3 ? C3.props.value : a3.__ : i4, t3.__c ? g2 = (h4 = u4.__c = t3.__c).__ = h4.__E : (x2 ? u4.__c = h4 = new T3(m2, M2) : (u4.__c = h4 = new b(m2, M2), h4.constructor = T3, h4.render = q), C3 && C3.sub(h4), h4.props = m2, h4.state || (h4.state = {}), h4.context = M2, h4.__n = i4, p3 = h4.__d = true, h4.__h = [], h4._sb = []), x2 && null == h4.__s && (h4.__s = h4.state), x2 && null != T3.getDerivedStateFromProps && (h4.__s == h4.state && (h4.__s = d({}, h4.__s)), d(h4.__s, T3.getDerivedStateFromProps(m2, h4.__s))), v3 = h4.props, w3 = h4.state, h4.__v = u4, p3)
            x2 && null == T3.getDerivedStateFromProps && null != h4.componentWillMount && h4.componentWillMount(), x2 && null != h4.componentDidMount && h4.__h.push(h4.componentDidMount);
          else {
            if (x2 && null == T3.getDerivedStateFromProps && m2 !== v3 && null != h4.componentWillReceiveProps && h4.componentWillReceiveProps(m2, M2), !h4.__e && (null != h4.shouldComponentUpdate && false === h4.shouldComponentUpdate(m2, h4.__s, M2) || u4.__v === t3.__v)) {
              for (u4.__v !== t3.__v && (h4.props = m2, h4.state = h4.__s, h4.__d = false), u4.__e = t3.__e, u4.__k = t3.__k, u4.__k.forEach(function(n3) {
                n3 && (n3.__ = u4);
              }), P2 = 0; P2 < h4._sb.length; P2++)
                h4.__h.push(h4._sb[P2]);
              h4._sb = [], h4.__h.length && f4.push(h4);
              break n;
            }
            null != h4.componentWillUpdate && h4.componentWillUpdate(m2, h4.__s, M2), x2 && null != h4.componentDidUpdate && h4.__h.push(function() {
              h4.componentDidUpdate(v3, w3, _2);
            });
          }
          if (h4.context = M2, h4.props = m2, h4.__P = n2, h4.__e = false, $2 = l.__r, I2 = 0, x2) {
            for (h4.state = h4.__s, h4.__d = false, $2 && $2(u4), a3 = h4.render(h4.props, h4.state, h4.context), H = 0; H < h4._sb.length; H++)
              h4.__h.push(h4._sb[H]);
            h4._sb = [];
          } else
            do {
              h4.__d = false, $2 && $2(u4), a3 = h4.render(h4.props, h4.state, h4.context), h4.state = h4.__s;
            } while (h4.__d && ++I2 < 25);
          h4.state = h4.__s, null != h4.getChildContext && (i4 = d(d({}, i4), h4.getChildContext())), x2 && !p3 && null != h4.getSnapshotBeforeUpdate && (_2 = h4.getSnapshotBeforeUpdate(v3, w3)), S(n2, y(L2 = null != a3 && a3.type === k && null == a3.key ? a3.props.children : a3) ? L2 : [L2], u4, t3, i4, o3, r3, f4, e3, c3, s3), h4.base = u4.__e, u4.__u &= -161, h4.__h.length && f4.push(h4), g2 && (h4.__E = h4.__ = null);
        } catch (n3) {
          if (u4.__v = null, c3 || null != r3) {
            for (u4.__u |= c3 ? 160 : 32; e3 && 8 === e3.nodeType && e3.nextSibling; )
              e3 = e3.nextSibling;
            r3[r3.indexOf(e3)] = null, u4.__e = e3;
          } else
            u4.__e = t3.__e, u4.__k = t3.__k;
          l.__e(n3, u4, t3);
        }
      else
        null == r3 && u4.__v === t3.__v ? (u4.__k = t3.__k, u4.__e = t3.__e) : u4.__e = z(t3.__e, u4, t3, i4, o3, r3, f4, c3, s3);
    (a3 = l.diffed) && a3(u4);
  }
  function j(n2, u4, t3) {
    u4.__d = void 0;
    for (var i4 = 0; i4 < t3.length; i4++)
      N(t3[i4], t3[++i4], t3[++i4]);
    l.__c && l.__c(u4, n2), n2.some(function(u5) {
      try {
        n2 = u5.__h, u5.__h = [], n2.some(function(n3) {
          n3.call(u5);
        });
      } catch (n3) {
        l.__e(n3, u5.__v);
      }
    });
  }
  function z(l3, u4, t3, i4, o3, r3, f4, e3, c3) {
    var s3, a3, p3, v3, d3, _2, g2, m2 = t3.props, k3 = u4.props, b2 = u4.type;
    if ("svg" === b2 ? o3 = "http://www.w3.org/2000/svg" : "math" === b2 ? o3 = "http://www.w3.org/1998/Math/MathML" : o3 || (o3 = "http://www.w3.org/1999/xhtml"), null != r3) {
      for (s3 = 0; s3 < r3.length; s3++)
        if ((d3 = r3[s3]) && "setAttribute" in d3 == !!b2 && (b2 ? d3.localName === b2 : 3 === d3.nodeType)) {
          l3 = d3, r3[s3] = null;
          break;
        }
    }
    if (null == l3) {
      if (null === b2)
        return document.createTextNode(k3);
      l3 = document.createElementNS(o3, b2, k3.is && k3), r3 = null, e3 = false;
    }
    if (null === b2)
      m2 === k3 || e3 && l3.data === k3 || (l3.data = k3);
    else {
      if (r3 = r3 && n.call(l3.childNodes), m2 = t3.props || h, !e3 && null != r3)
        for (m2 = {}, s3 = 0; s3 < l3.attributes.length; s3++)
          m2[(d3 = l3.attributes[s3]).name] = d3.value;
      for (s3 in m2)
        if (d3 = m2[s3], "children" == s3)
          ;
        else if ("dangerouslySetInnerHTML" == s3)
          p3 = d3;
        else if ("key" !== s3 && !(s3 in k3)) {
          if ("value" == s3 && "defaultValue" in k3 || "checked" == s3 && "defaultChecked" in k3)
            continue;
          A(l3, s3, null, d3, o3);
        }
      for (s3 in k3)
        d3 = k3[s3], "children" == s3 ? v3 = d3 : "dangerouslySetInnerHTML" == s3 ? a3 = d3 : "value" == s3 ? _2 = d3 : "checked" == s3 ? g2 = d3 : "key" === s3 || e3 && "function" != typeof d3 || m2[s3] === d3 || A(l3, s3, d3, m2[s3], o3);
      if (a3)
        e3 || p3 && (a3.__html === p3.__html || a3.__html === l3.innerHTML) || (l3.innerHTML = a3.__html), u4.__k = [];
      else if (p3 && (l3.innerHTML = ""), S(l3, y(v3) ? v3 : [v3], u4, t3, i4, "foreignObject" === b2 ? "http://www.w3.org/1999/xhtml" : o3, r3, f4, r3 ? r3[0] : t3.__k && x(t3, 0), e3, c3), null != r3)
        for (s3 = r3.length; s3--; )
          null != r3[s3] && w(r3[s3]);
      e3 || (s3 = "value", void 0 !== _2 && (_2 !== l3[s3] || "progress" === b2 && !_2 || "option" === b2 && _2 !== m2[s3]) && A(l3, s3, _2, m2[s3], o3), s3 = "checked", void 0 !== g2 && g2 !== l3[s3] && A(l3, s3, g2, m2[s3], o3));
    }
    return l3;
  }
  function N(n2, u4, t3) {
    try {
      if ("function" == typeof n2) {
        var i4 = "function" == typeof n2.__u;
        i4 && n2.__u(), i4 && null == u4 || (n2.__u = n2(u4));
      } else
        n2.current = u4;
    } catch (n3) {
      l.__e(n3, t3);
    }
  }
  function V(n2, u4, t3) {
    var i4, o3;
    if (l.unmount && l.unmount(n2), (i4 = n2.ref) && (i4.current && i4.current !== n2.__e || N(i4, null, u4)), null != (i4 = n2.__c)) {
      if (i4.componentWillUnmount)
        try {
          i4.componentWillUnmount();
        } catch (n3) {
          l.__e(n3, u4);
        }
      i4.base = i4.__P = null;
    }
    if (i4 = n2.__k)
      for (o3 = 0; o3 < i4.length; o3++)
        i4[o3] && V(i4[o3], u4, t3 || "function" != typeof n2.type);
    t3 || null == n2.__e || w(n2.__e), n2.__c = n2.__ = n2.__e = n2.__d = void 0;
  }
  function q(n2, l3, u4) {
    return this.constructor(n2, u4);
  }
  function B(u4, t3, i4) {
    var o3, r3, f4, e3;
    l.__ && l.__(u4, t3), r3 = (o3 = "function" == typeof i4) ? null : i4 && i4.__k || t3.__k, f4 = [], e3 = [], O(t3, u4 = (!o3 && i4 || t3).__k = _(k, null, [u4]), r3 || h, h, t3.namespaceURI, !o3 && i4 ? [i4] : r3 ? null : t3.firstChild ? n.call(t3.childNodes) : null, f4, !o3 && i4 ? i4 : r3 ? r3.__e : t3.firstChild, o3, e3), j(f4, u4, e3);
  }
  n = p.slice, l = { __e: function(n2, l3, u4, t3) {
    for (var i4, o3, r3; l3 = l3.__; )
      if ((i4 = l3.__c) && !i4.__)
        try {
          if ((o3 = i4.constructor) && null != o3.getDerivedStateFromError && (i4.setState(o3.getDerivedStateFromError(n2)), r3 = i4.__d), null != i4.componentDidCatch && (i4.componentDidCatch(n2, t3 || {}), r3 = i4.__d), r3)
            return i4.__E = i4;
        } catch (l4) {
          n2 = l4;
        }
    throw n2;
  } }, u = 0, t = function(n2) {
    return null != n2 && null == n2.constructor;
  }, b.prototype.setState = function(n2, l3) {
    var u4;
    u4 = null != this.__s && this.__s !== this.state ? this.__s : this.__s = d({}, this.state), "function" == typeof n2 && (n2 = n2(d({}, u4), this.props)), n2 && d(u4, n2), null != n2 && this.__v && (l3 && this._sb.push(l3), M(this));
  }, b.prototype.forceUpdate = function(n2) {
    this.__v && (this.__e = true, n2 && this.__h.push(n2), M(this));
  }, b.prototype.render = k, i = [], r = "function" == typeof Promise ? Promise.prototype.then.bind(Promise.resolve()) : setTimeout, f = function(n2, l3) {
    return n2.__v.__b - l3.__v.__b;
  }, P.__r = 0, e = 0, c = F(false), s = F(true), a = 0;

  // ../../node_modules/preact/jsx-runtime/dist/jsxRuntime.module.js
  var f2 = 0;
  var i2 = Array.isArray;
  function u2(e3, t3, n2, o3, i4, u4) {
    t3 || (t3 = {});
    var a3, c3, p3 = t3;
    if ("ref" in p3)
      for (c3 in p3 = {}, t3)
        "ref" == c3 ? a3 = t3[c3] : p3[c3] = t3[c3];
    var l3 = { type: e3, props: p3, key: n2, ref: a3, __k: null, __: null, __b: 0, __e: null, __d: void 0, __c: null, constructor: void 0, __v: --f2, __i: -1, __u: 0, __source: i4, __self: u4 };
    if ("function" == typeof e3 && (a3 = e3.defaultProps))
      for (c3 in a3)
        void 0 === p3[c3] && (p3[c3] = a3[c3]);
    return l.vnode && l.vnode(l3), l3;
  }

  // src/lib/components/button-group.tsx
  var ButtonGroup = (props) => {
    const { children, orientation = "horizontal" } = props;
    return /* @__PURE__ */ u2("div", { className: `deck-widget-button-group ${orientation}`, children });
  };

  // src/lib/components/grouped-icon-button.tsx
  var GroupedIconButton = (props) => {
    const { className = "", label, onClick, children } = props;
    return /* @__PURE__ */ u2(
      "button",
      {
        className: `deck-widget-icon-button ${className}`,
        type: "button",
        onClick,
        title: label,
        children: children ? children : /* @__PURE__ */ u2("div", { className: "deck-widget-icon" })
      }
    );
  };

  // src/zoom-widget.tsx
  var ZoomWidget = class extends import_core2.Widget {
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
      const ui = /* @__PURE__ */ u2(ButtonGroup, { orientation: this.props.orientation, children: [
        /* @__PURE__ */ u2(
          GroupedIconButton,
          {
            onClick: () => this.handleZoomIn(),
            label: this.props.zoomInLabel,
            className: "deck-widget-zoom-in"
          }
        ),
        /* @__PURE__ */ u2(
          GroupedIconButton,
          {
            onClick: () => this.handleZoomOut(),
            label: this.props.zoomOutLabel,
            className: "deck-widget-zoom-out"
          }
        )
      ] });
      B(ui, rootElement);
    }
    onViewportChange(viewport) {
      this.viewports[viewport.id] = viewport;
    }
    handleZoom(viewport, nextZoom) {
      const viewId = this.viewId || viewport?.id || "default-view";
      const nextViewState = {
        ...viewport,
        zoom: nextZoom
      };
      if (this.props.transitionDuration > 0) {
        nextViewState.transitionDuration = this.props.transitionDuration;
        nextViewState.transitionInterpolator = "latitude" in nextViewState ? new import_core2.FlyToInterpolator() : new import_core2.LinearInterpolator({
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
    ...import_core2.Widget.defaultProps,
    id: "zoom",
    placement: "top-left",
    orientation: "vertical",
    transitionDuration: 200,
    zoomInLabel: "Zoom In",
    zoomOutLabel: "Zoom Out",
    viewId: null
  };

  // src/reset-view-widget.tsx
  var import_core3 = __toESM(require_core(), 1);

  // src/lib/components/icon-button.tsx
  var IconButton = (props) => {
    const { className = "", label, onClick, children } = props;
    return /* @__PURE__ */ u2("div", { className: "deck-widget-button", children: /* @__PURE__ */ u2(
      "button",
      {
        className: `deck-widget-icon-button ${className}`,
        type: "button",
        onClick,
        title: label,
        children: children ? children : /* @__PURE__ */ u2("div", { className: "deck-widget-icon" })
      }
    ) });
  };

  // src/reset-view-widget.tsx
  var ResetViewWidget = class extends import_core3.Widget {
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
      B(
        /* @__PURE__ */ u2(
          IconButton,
          {
            className: "deck-widget-reset-focus",
            label: this.props.label,
            onClick: this.handleClick.bind(this)
          }
        ),
        rootElement
      );
    }
    handleClick() {
      const initialViewState = this.props.initialViewState || this.deck?.props.initialViewState;
      this.setViewState(initialViewState);
    }
    setViewState(viewState) {
      const viewId = this.props.viewId || "default-view";
      const nextViewState = {
        ...viewId !== "default-view" ? viewState?.[viewId] : viewState
        // only works for geospatial?
        // transitionDuration: this.props.transitionDuration,
        // transitionInterpolator: new FlyToInterpolator()
      };
      this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
  };
  ResetViewWidget.defaultProps = {
    ...import_core3.Widget.defaultProps,
    id: "reset-view",
    placement: "top-left",
    label: "Reset View",
    initialViewState: void 0,
    viewId: null
  };

  // src/gimbal-widget.tsx
  var import_core4 = __toESM(require_core(), 1);
  var GimbalWidget = class extends import_core4.Widget {
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
      const viewId = this.viewId || Object.values(this.viewports)[0]?.id || "default-view";
      const widgetViewport = this.viewports[viewId];
      const { rotationOrbit, rotationX } = this.getNormalizedRotation(widgetViewport);
      const ui = /* @__PURE__ */ u2("div", { className: "deck-widget-button", style: { perspective: 100, pointerEvents: "auto" }, children: /* @__PURE__ */ u2(
        "button",
        {
          type: "button",
          onClick: () => {
            for (const viewport of Object.values(this.viewports)) {
              this.resetOrbitView(viewport);
            }
          },
          title: this.props.label,
          style: { position: "relative", width: 26, height: 26 },
          children: [
            /* @__PURE__ */ u2(
              "svg",
              {
                className: "gimbal-outer-ring",
                width: "100%",
                height: "100%",
                viewBox: "0 0 26 26",
                style: {
                  position: "absolute",
                  top: 0,
                  left: 0,
                  transform: `rotateY(${rotationOrbit}deg)`
                },
                children: /* @__PURE__ */ u2(
                  "circle",
                  {
                    cx: "13",
                    cy: "13",
                    r: "10",
                    stroke: "var(--icon-gimbal-outer-color, rgb(68, 92, 204))",
                    strokeWidth: this.props.strokeWidth,
                    fill: "none"
                  }
                )
              }
            ),
            /* @__PURE__ */ u2(
              "svg",
              {
                className: "gimbal-inner-ring",
                width: "100%",
                height: "100%",
                viewBox: "0 0 26 26",
                style: {
                  position: "absolute",
                  top: 0,
                  left: 0,
                  transform: `rotateX(${rotationX}deg)`
                },
                children: /* @__PURE__ */ u2(
                  "circle",
                  {
                    cx: "13",
                    cy: "13",
                    r: "7",
                    stroke: "var(--icon-gimbal-inner-color, rgb(240, 92, 68))",
                    strokeWidth: this.props.strokeWidth,
                    fill: "none"
                  }
                )
              }
            )
          ]
        }
      ) });
      B(ui, rootElement);
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
          transitionInterpolator: new import_core4.LinearInterpolator({
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
      const viewId = this.viewId || viewport?.id || "OrbitView";
      return viewId;
    }
    getViewState(viewId) {
      const viewManager = this.getViewManager();
      const viewState = viewId && viewManager.getViewState(viewId) || viewManager.viewState;
      return viewState;
    }
    getViewManager() {
      const viewManager = this.deck?.viewManager;
      if (!viewManager) {
        throw new Error("wigdet must be added to a deck instance");
      }
      return viewManager;
    }
  };
  GimbalWidget.defaultProps = {
    ...import_core4.Widget.defaultProps,
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

  // src/compass-widget.tsx
  var import_core5 = __toESM(require_core(), 1);
  var CompassWidget = class extends import_core5.Widget {
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
      const viewId = this.viewId || Object.values(this.viewports)[0]?.id || "default-view";
      const widgetViewport = this.viewports[viewId];
      const [rz, rx] = this.getRotation(widgetViewport);
      const ui = /* @__PURE__ */ u2("div", { className: "deck-widget-button", style: { perspective: 100 }, children: /* @__PURE__ */ u2(
        "button",
        {
          type: "button",
          onClick: () => {
            for (const viewport of Object.values(this.viewports)) {
              this.handleCompassReset(viewport);
            }
          },
          title: this.props.label,
          style: { transform: `rotateX(${rx}deg)` },
          children: /* @__PURE__ */ u2("svg", { fill: "none", width: "100%", height: "100%", viewBox: "0 0 26 26", children: /* @__PURE__ */ u2("g", { transform: `rotate(${rz},13,13)`, children: [
            /* @__PURE__ */ u2(
              "path",
              {
                d: "M10 13.0001L12.9999 5L15.9997 13.0001H10Z",
                fill: "var(--icon-compass-north-color, rgb(240, 92, 68))"
              }
            ),
            /* @__PURE__ */ u2(
              "path",
              {
                d: "M16.0002 12.9999L13.0004 21L10.0005 12.9999H16.0002Z",
                fill: "var(--icon-compass-south-color, rgb(204, 204, 204))"
              }
            )
          ] }) })
        }
      ) });
      B(ui, rootElement);
    }
    onViewportChange(viewport) {
      if (!viewport.equals(this.viewports[viewport.id])) {
        this.viewports[viewport.id] = viewport;
        this.updateHTML();
      }
    }
    getRotation(viewport) {
      if (viewport instanceof import_core5.WebMercatorViewport) {
        return [-viewport.bearing, viewport.pitch];
      } else if (viewport instanceof import_core5._GlobeViewport) {
        return [0, Math.max(-80, Math.min(80, viewport.latitude))];
      }
      return [0, 0];
    }
    handleCompassReset(viewport) {
      const viewId = this.viewId || viewport.id || "default-view";
      if (viewport instanceof import_core5.WebMercatorViewport) {
        const nextViewState = {
          ...viewport,
          bearing: 0,
          ...this.getRotation(viewport)[0] === 0 ? { pitch: 0 } : {},
          transitionDuration: this.props.transitionDuration,
          transitionInterpolator: new import_core5.FlyToInterpolator()
        };
        this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
      }
    }
  };
  CompassWidget.defaultProps = {
    ...import_core5.Widget.defaultProps,
    id: "compass",
    placement: "top-left",
    viewId: null,
    label: "Reset Compass",
    transitionDuration: 200
  };

  // src/scale-widget.tsx
  var import_core6 = __toESM(require_core(), 1);
  var ScaleWidget = class extends import_core6.Widget {
    constructor(props = {}) {
      super(props);
      this.className = "deck-widget-scale";
      this.placement = "bottom-left";
      // The pixel width of the scale line (computed from a candidate distance)
      this.scaleWidth = 10;
      // The candidate distance (in meters) corresponding to the scale line length.
      this.scaleValue = 0;
      // The formatted distance label (e.g. "200 m" or "1.0 km")
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
      B(
        /* @__PURE__ */ u2(
          "svg",
          {
            className: "deck-widget-scale",
            width: svgWidth,
            height: 30,
            style: { overflow: "visible", background: "transparent" },
            onClick: this.handleClick.bind(this),
            children: [
              /* @__PURE__ */ u2(
                "text",
                {
                  x: lineOffsetX + 5,
                  y: "10",
                  textAnchor: "end",
                  alignmentBaseline: "middle",
                  style: { fontSize: "16px", fill: "black", fontWeight: "bold", fontFamily: "sans-serif" },
                  children: this.scaleText
                }
              ),
              /* @__PURE__ */ u2(
                "line",
                {
                  x1: lineOffsetX,
                  y1: "15",
                  x2: lineOffsetX + this.scaleWidth,
                  y2: "15",
                  stroke: "black",
                  strokeWidth: "6"
                }
              ),
              /* @__PURE__ */ u2(
                "line",
                {
                  x1: lineOffsetX,
                  y1: "15",
                  x2: lineOffsetX,
                  y2: 15 - tickHeight,
                  stroke: "black",
                  strokeWidth: "6"
                }
              ),
              /* @__PURE__ */ u2(
                "line",
                {
                  x1: lineOffsetX + this.scaleWidth,
                  y1: "15",
                  x2: lineOffsetX + this.scaleWidth,
                  y2: 15 - tickHeight,
                  stroke: "black",
                  strokeWidth: "6"
                }
              )
            ]
          }
        ),
        rootElement
      );
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
    ...import_core6.Widget.defaultProps,
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
    for (let i4 = 0; i4 < multipliers.length; i4++) {
      const currentCandidate = multipliers[i4] * base;
      const currentPixels = currentCandidate / metersPerPixel;
      if (currentPixels >= minPixels && currentPixels <= maxPixels) {
        candidate = currentCandidate;
        candidatePixels = currentPixels;
        break;
      }
      if (currentPixels > maxPixels) {
        candidate = i4 > 0 ? multipliers[i4 - 1] * base : currentCandidate;
        candidatePixels = candidate / metersPerPixel;
        break;
      }
      if (i4 === multipliers.length - 1 && currentPixels < minPixels) {
        candidate = multipliers[0] * base * 10;
        candidatePixels = candidate / metersPerPixel;
      }
    }
    return { candidate, candidatePixels };
  }

  // src/geocoder-widget.tsx
  var import_core7 = __toESM(require_core(), 1);
  var import_core8 = __toESM(require_core(), 1);

  // ../../node_modules/preact/hooks/dist/hooks.module.js
  var t2;
  var r2;
  var u3;
  var i3;
  var o2 = 0;
  var f3 = [];
  var c2 = l;
  var e2 = c2.__b;
  var a2 = c2.__r;
  var v2 = c2.diffed;
  var l2 = c2.__c;
  var m = c2.unmount;
  var s2 = c2.__;
  function d2(n2, t3) {
    c2.__h && c2.__h(r2, n2, o2 || t3), o2 = 0;
    var u4 = r2.__H || (r2.__H = { __: [], __h: [] });
    return n2 >= u4.__.length && u4.__.push({}), u4.__[n2];
  }
  function h2(n2) {
    return o2 = 1, p2(D, n2);
  }
  function p2(n2, u4, i4) {
    var o3 = d2(t2++, 2);
    if (o3.t = n2, !o3.__c && (o3.__ = [i4 ? i4(u4) : D(void 0, u4), function(n3) {
      var t3 = o3.__N ? o3.__N[0] : o3.__[0], r3 = o3.t(t3, n3);
      t3 !== r3 && (o3.__N = [r3, o3.__[1]], o3.__c.setState({}));
    }], o3.__c = r2, !r2.u)) {
      var f4 = function(n3, t3, r3) {
        if (!o3.__c.__H)
          return true;
        var u5 = o3.__c.__H.__.filter(function(n4) {
          return !!n4.__c;
        });
        if (u5.every(function(n4) {
          return !n4.__N;
        }))
          return !c3 || c3.call(this, n3, t3, r3);
        var i5 = false;
        return u5.forEach(function(n4) {
          if (n4.__N) {
            var t4 = n4.__[0];
            n4.__ = n4.__N, n4.__N = void 0, t4 !== n4.__[0] && (i5 = true);
          }
        }), !(!i5 && o3.__c.props === n3) && (!c3 || c3.call(this, n3, t3, r3));
      };
      r2.u = true;
      var c3 = r2.shouldComponentUpdate, e3 = r2.componentWillUpdate;
      r2.componentWillUpdate = function(n3, t3, r3) {
        if (this.__e) {
          var u5 = c3;
          c3 = void 0, f4(n3, t3, r3), c3 = u5;
        }
        e3 && e3.call(this, n3, t3, r3);
      }, r2.shouldComponentUpdate = f4;
    }
    return o3.__N || o3.__;
  }
  function y2(n2, u4) {
    var i4 = d2(t2++, 3);
    !c2.__s && C2(i4.__H, u4) && (i4.__ = n2, i4.i = u4, r2.__H.__h.push(i4));
  }
  function A2(n2) {
    return o2 = 5, T2(function() {
      return { current: n2 };
    }, []);
  }
  function T2(n2, r3) {
    var u4 = d2(t2++, 7);
    return C2(u4.__H, r3) && (u4.__ = n2(), u4.__H = r3, u4.__h = n2), u4.__;
  }
  function j2() {
    for (var n2; n2 = f3.shift(); )
      if (n2.__P && n2.__H)
        try {
          n2.__H.__h.forEach(z2), n2.__H.__h.forEach(B2), n2.__H.__h = [];
        } catch (t3) {
          n2.__H.__h = [], c2.__e(t3, n2.__v);
        }
  }
  c2.__b = function(n2) {
    r2 = null, e2 && e2(n2);
  }, c2.__ = function(n2, t3) {
    n2 && t3.__k && t3.__k.__m && (n2.__m = t3.__k.__m), s2 && s2(n2, t3);
  }, c2.__r = function(n2) {
    a2 && a2(n2), t2 = 0;
    var i4 = (r2 = n2.__c).__H;
    i4 && (u3 === r2 ? (i4.__h = [], r2.__h = [], i4.__.forEach(function(n3) {
      n3.__N && (n3.__ = n3.__N), n3.i = n3.__N = void 0;
    })) : (i4.__h.forEach(z2), i4.__h.forEach(B2), i4.__h = [], t2 = 0)), u3 = r2;
  }, c2.diffed = function(n2) {
    v2 && v2(n2);
    var t3 = n2.__c;
    t3 && t3.__H && (t3.__H.__h.length && (1 !== f3.push(t3) && i3 === c2.requestAnimationFrame || ((i3 = c2.requestAnimationFrame) || w2)(j2)), t3.__H.__.forEach(function(n3) {
      n3.i && (n3.__H = n3.i), n3.i = void 0;
    })), u3 = r2 = null;
  }, c2.__c = function(n2, t3) {
    t3.some(function(n3) {
      try {
        n3.__h.forEach(z2), n3.__h = n3.__h.filter(function(n4) {
          return !n4.__ || B2(n4);
        });
      } catch (r3) {
        t3.some(function(n4) {
          n4.__h && (n4.__h = []);
        }), t3 = [], c2.__e(r3, n3.__v);
      }
    }), l2 && l2(n2, t3);
  }, c2.unmount = function(n2) {
    m && m(n2);
    var t3, r3 = n2.__c;
    r3 && r3.__H && (r3.__H.__.forEach(function(n3) {
      try {
        z2(n3);
      } catch (n4) {
        t3 = n4;
      }
    }), r3.__H = void 0, t3 && c2.__e(t3, r3.__v));
  };
  var k2 = "function" == typeof requestAnimationFrame;
  function w2(n2) {
    var t3, r3 = function() {
      clearTimeout(u4), k2 && cancelAnimationFrame(t3), setTimeout(n2);
    }, u4 = setTimeout(r3, 100);
    k2 && (t3 = requestAnimationFrame(r3));
  }
  function z2(n2) {
    var t3 = r2, u4 = n2.__c;
    "function" == typeof u4 && (n2.__c = void 0, u4()), r2 = t3;
  }
  function B2(n2) {
    var t3 = r2;
    n2.__c = n2.__(), r2 = t3;
  }
  function C2(n2, t3) {
    return !n2 || n2.length !== t3.length || t3.some(function(t4, r3) {
      return t4 !== n2[r3];
    });
  }
  function D(n2, t3) {
    return "function" == typeof t3 ? t3(n2) : t3;
  }

  // src/lib/components/dropdown-menu.tsx
  var DropdownMenu = (props) => {
    const [isOpen, setIsOpen] = h2(false);
    const dropdownRef = A2(null);
    const toggleDropdown = () => setIsOpen(!isOpen);
    const handleClickOutside = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };
    y2(() => {
      document.addEventListener("mousedown", handleClickOutside);
      return () => {
        document.removeEventListener("mousedown", handleClickOutside);
      };
    }, []);
    const handleSelect = (value) => {
      props.onSelect(value);
      setIsOpen(false);
    };
    return /* @__PURE__ */ u2(
      "div",
      {
        className: "dropdown-container",
        ref: dropdownRef,
        style: {
          position: "relative",
          display: "inline-block",
          ...props.style
        },
        children: [
          /* @__PURE__ */ u2(
            "button",
            {
              onClick: toggleDropdown,
              style: {
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
              },
              children: "\u25BC"
            }
          ),
          isOpen && /* @__PURE__ */ u2(
            "ul",
            {
              style: {
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
              },
              children: props.menuItems.map((item) => /* @__PURE__ */ u2(
                "li",
                {
                  onClick: () => handleSelect(item),
                  style: {
                    padding: "4px 8px",
                    cursor: "pointer",
                    whiteSpace: "nowrap"
                  },
                  children: item
                },
                item
              ))
            }
          )
        ]
      }
    );
  };

  // src/lib/geocode/geocoder-history.ts
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
        const addresses = Array.isArray(list) ? list.filter((v3) => typeof v3 === "string") : [];
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
      const deduped = [cleaned, ...this.addressHistory.filter((a3) => a3 !== cleaned)];
      this.addressHistory = deduped.slice(0, this.props.maxEntries);
      try {
        window.localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(this.addressHistory));
      } catch {
      }
    }
  };

  // src/lib/geocode/geocoders.ts
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
    const parts = input.split(/[,;]/).map((p3) => p3.trim());
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
  function parseCoordinatePart(s3) {
    s3 = s3.trim();
    if (s3.includes("\xB0") || s3.includes("'") || s3.includes('"')) {
      const value2 = dmsToDecimal(s3);
      return isNaN(value2) ? null : value2;
    }
    let sign = 1;
    if (/[SW]/i.test(s3))
      sign = -1;
    s3 = s3.replace(/[NSEW]/gi, "");
    const value = parseFloat(s3);
    return isNaN(value) ? null : sign * value;
  }
  function dmsToDecimal(s3) {
    const regex = /(\d+)[°d]\s*(\d+)?['′m]?\s*(\d+(?:\.\d+)?)?[\"″s]?\s*([NSEW])?/i;
    const match = s3.match(regex);
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

  // src/geocoder-widget.tsx
  var CURRENT_LOCATION2 = "current";
  var GeocoderWidget = class extends import_core7.Widget {
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
      this.handleKeyPress = (e3) => {
        if (e3.key === "Enter") {
          this.handleSubmit();
        }
      };
      this.handleSelect = (address) => {
        this.setInput(address);
        this.handleSubmit();
      };
      /** Sync wrapper for async geocode() */
      this.handleSubmit = () => {
        this.geocode(this.addressText);
      };
      /** Perform geocoding */
      this.geocode = async (address) => {
        const useGeolocation = this.props._geolocation && address === CURRENT_LOCATION2;
        const geocoder = useGeolocation ? CurrentLocationGeocoder : this.geocoder;
        const coordinates = await this.geocodeHistory.geocode(
          geocoder,
          this.addressText,
          this.props.apiKey
        );
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
      B(
        /* @__PURE__ */ u2(
          "div",
          {
            className: "deck-widget-geocoder",
            style: {
              pointerEvents: "auto",
              display: "flex",
              alignItems: "center",
              flexWrap: "wrap"
              // Allows wrapping on smaller screens
            },
            children: [
              /* @__PURE__ */ u2(
                "input",
                {
                  type: "text",
                  placeholder: this.geocoder.placeholderLocation ?? "Enter address or location",
                  value: this.geocodeHistory.addressText,
                  onInput: (e3) => this.setInput(e3.target?.value || ""),
                  onKeyPress: this.handleKeyPress,
                  style: {
                    flex: "1 1 auto",
                    minWidth: "200px",
                    margin: 0,
                    padding: "8px",
                    boxSizing: "border-box"
                  }
                }
              ),
              /* @__PURE__ */ u2(
                DropdownMenu,
                {
                  menuItems,
                  onSelect: this.handleSelect,
                  style: {
                    margin: 2,
                    padding: "4px 2px",
                    boxSizing: "border-box"
                  }
                }
              ),
              this.geocodeHistory.errorText && /* @__PURE__ */ u2("div", { className: "error", children: this.geocodeHistory.errorText })
            ]
          }
        ),
        rootElement
      );
    }
    // TODO - MOVE TO WIDGETIMPL?
    setViewState(viewState) {
      const viewId = this.props.viewId || viewState?.id || "default-view";
      const viewport = this.viewports[viewId] || {};
      const nextViewState = {
        ...viewport,
        ...viewState
      };
      if (this.props.transitionDuration > 0) {
        nextViewState.transitionDuration = this.props.transitionDuration;
        nextViewState.transitionInterpolator = "latitude" in nextViewState ? new import_core8.FlyToInterpolator() : new import_core8.LinearInterpolator();
      }
      this.deck._onViewStateChange({ viewId, viewState: nextViewState, interactionState: {} });
    }
    onViewportChange(viewport) {
      this.viewports[viewport.id] = viewport;
    }
  };
  GeocoderWidget.defaultProps = {
    ...import_core7.Widget.defaultProps,
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

  // src/fullscreen-widget.tsx
  var import_core9 = __toESM(require_core(), 1);
  var FullscreenWidget = class extends import_core9.Widget {
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
      B(
        /* @__PURE__ */ u2(
          IconButton,
          {
            onClick: () => {
              this.handleClick().catch((err) => import_core9.log.error(err)());
            },
            label: this.fullscreen ? this.props.exitLabel : this.props.enterLabel,
            className: this.fullscreen ? "deck-widget-fullscreen-exit" : "deck-widget-fullscreen-enter"
          }
        ),
        rootElement
      );
    }
    setProps(props) {
      this.placement = props.placement ?? this.placement;
      this.viewId = props.viewId ?? this.viewId;
      super.setProps(props);
    }
    getContainer() {
      return this.props.container || this.deck?.getCanvas()?.parentElement;
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
      if (container?.requestFullscreen) {
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
      this.getContainer()?.classList.toggle("deck-pseudo-fullscreen");
    }
  };
  FullscreenWidget.defaultProps = {
    ...import_core9.Widget.defaultProps,
    id: "fullscreen",
    placement: "top-left",
    viewId: null,
    enterLabel: "Enter Fullscreen",
    exitLabel: "Exit Fullscreen",
    container: void 0
  };

  // src/splitter-widget.tsx
  var import_core10 = __toESM(require_core(), 1);
  var SplitterWidget = class extends import_core10.Widget {
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
      B(
        /* @__PURE__ */ u2(
          Splitter,
          {
            orientation: this.props.orientation,
            initialSplit: this.props.initialSplit,
            onChange: this.props.onChange,
            onDragStart: this.props.onDragStart,
            onDragEnd: this.props.onDragEnd
          }
        ),
        rootElement
      );
    }
  };
  SplitterWidget.defaultProps = {
    ...import_core10.Widget.defaultProps,
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
  function Splitter({
    orientation,
    initialSplit,
    onChange,
    onDragStart,
    onDragEnd
  }) {
    const [split, setSplit] = h2(initialSplit);
    const dragging = A2(false);
    const containerRef = A2(null);
    const handleDragStart = (event) => {
      dragging.current = true;
      onDragStart?.();
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
      onChange?.(newSplit);
    };
    const handleDragEnd = (event) => {
      if (!dragging.current)
        return;
      dragging.current = false;
      onDragEnd?.();
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
    return /* @__PURE__ */ u2("div", { ref: containerRef, style: containerStyle, children: /* @__PURE__ */ u2(
      "div",
      {
        style: splitterStyle,
        onMouseDown: handleDragStart
      }
    ) });
  }

  // src/view-selector-widget.tsx
  var import_core11 = __toESM(require_core(), 1);

  // src/lib/components/icon-menu.tsx
  function IconMenu(props) {
    const [menuOpen, setMenuOpen] = h2(false);
    const containerRef = A2(null);
    const handleClickOutside = (event) => {
      if (containerRef.current && !containerRef.current.contains(event.target)) {
        setMenuOpen(false);
      }
    };
    y2(() => {
      document.addEventListener("mousedown", handleClickOutside);
      return () => {
        document.removeEventListener("mousedown", handleClickOutside);
      };
    }, [containerRef]);
    const [selectedItem, setSelectedItem] = h2(props.initialItem);
    const handleSelectItem = (item) => {
      setSelectedItem(item);
      setMenuOpen(false);
      props.onItemSelected(item);
    };
    const handleButtonClick = () => setMenuOpen(!menuOpen);
    const selectedMenuItem = props.menuItems.find((item) => item.value === selectedItem);
    const label = props.label || selectedMenuItem?.label || "";
    const icon = props.icon || selectedMenuItem?.icon;
    return /* @__PURE__ */ u2("div", { style: { position: "relative", display: "inline-block" }, ref: containerRef, children: [
      /* @__PURE__ */ u2(IconButton, { className: props.className, label, onClick: handleButtonClick, children: icon }),
      menuOpen && /* @__PURE__ */ u2("div", { className: "deck-widget-icon-menu", children: /* @__PURE__ */ u2(ButtonGroup, { orientation: "vertical", children: props.menuItems.map((item) => /* @__PURE__ */ u2(
        GroupedIconButton,
        {
          label: item.label,
          onClick: () => handleSelectItem(item.value),
          children: item.icon
        },
        item.value
      )) }) })
    ] });
  }

  // src/view-selector-widget.tsx
  var ViewSelectorWidget = class extends import_core11.Widget {
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
      B(
        /* @__PURE__ */ u2(
          IconMenu,
          {
            className: "deck-widget-view-selector",
            menuItems: MENU_ITEMS.map((item) => ({
              ...item,
              icon: item.icon()
            })),
            initialItem: this.props.initialViewMode,
            onItemSelected: this.handleSelectMode
          }
        ),
        rootElement
      );
    }
  };
  ViewSelectorWidget.defaultProps = {
    ...import_core11.Widget.defaultProps,
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
    single: () => /* @__PURE__ */ u2("svg", { width: "24", height: "24", style: ICON_STYLE, children: /* @__PURE__ */ u2(
      "rect",
      {
        x: "4",
        y: "4",
        width: "16",
        height: "16",
        stroke: "var(--button-icon-hover, rgb(24, 24, 26))",
        fill: "none",
        strokeWidth: "2"
      }
    ) }),
    "split-horizontal": () => /* @__PURE__ */ u2("svg", { width: "24", height: "24", style: ICON_STYLE, children: [
      /* @__PURE__ */ u2(
        "rect",
        {
          x: "4",
          y: "4",
          width: "16",
          height: "7",
          stroke: "var(--button-icon-hover, rgb(24, 24, 26))",
          fill: "none",
          strokeWidth: "2"
        }
      ),
      /* @__PURE__ */ u2(
        "rect",
        {
          x: "4",
          y: "13",
          width: "16",
          height: "7",
          stroke: "var(--button-icon-hover, rgb(24, 24, 26))",
          fill: "none",
          strokeWidth: "2"
        }
      )
    ] }),
    "split-vertical": () => /* @__PURE__ */ u2("svg", { width: "24", height: "24", style: ICON_STYLE, children: [
      /* @__PURE__ */ u2(
        "rect",
        {
          x: "4",
          y: "4",
          width: "7",
          height: "16",
          stroke: "var(--button-icon-hover, rgb(24, 24, 26))",
          fill: "none",
          strokeWidth: "2"
        }
      ),
      /* @__PURE__ */ u2(
        "rect",
        {
          x: "13",
          y: "4",
          width: "7",
          height: "16",
          stroke: "var(--button-icon-hover, rgb(24, 24, 26))",
          fill: "none",
          strokeWidth: "2"
        }
      )
    ] })
  };
  var MENU_ITEMS = [
    { value: "single", icon: ICONS.single, label: "Single View" },
    { value: "split-horizontal", icon: ICONS["split-horizontal"], label: "Split Horizontal" },
    { value: "split-vertical", icon: ICONS["split-vertical"], label: "Split Vertical" }
  ];

  // src/info-widget.tsx
  var import_core12 = __toESM(require_core(), 1);
  var InfoWidget = class extends import_core12.Widget {
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
          style: { zIndex: "1", ...tooltip?.style }
        });
      }
    }
    onClick(info) {
      if (this.props.mode === "click" && this.props.getTooltip) {
        const tooltip = this.props.getTooltip(info, this);
        this.setProps({
          visible: tooltip !== null,
          ...tooltip
        });
        return tooltip !== null;
      }
      return this.props.onClick?.(this, info) || false;
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
      const [x2, y3] = this.viewport.project([longitude, latitude]);
      const minOffset = this.props.minOffset || 0;
      const gap = 10;
      const arrowHeight = 8;
      const arrowWidth = 16;
      const isAbove = y3 > this.viewport.height / 2;
      const background = this.props.style && this.props.style.background || "rgba(255,255,255,0.9)";
      const ui = this.props.visible ? /* @__PURE__ */ u2("div", { className: "popup-container", style: { position: "absolute", left: 0, top: 0 }, children: [
        /* @__PURE__ */ u2(
          "div",
          {
            className: "popup-content",
            style: {
              background,
              padding: "10px",
              position: "relative",
              // Include any additional styles
              ...this.props.style
            },
            children: this.props.text
          }
        ),
        /* @__PURE__ */ u2("div", { className: "popup-arrow", style: { position: "absolute", width: "0px", height: "0px" } })
      ] }) : null;
      B(ui, rootElement);
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
        let computedLeft = x2 - popupWidth / 2;
        let computedTop;
        if (isAbove) {
          computedTop = y3 - gap - arrowHeight - popupHeight;
        } else {
          computedTop = y3 + gap + arrowHeight;
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
        let arrowLeft = x2 - computedLeft - arrowWidth / 2;
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
    ...import_core12.Widget.defaultProps,
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

  // src/context-menu-widget.tsx
  var import_core13 = __toESM(require_core(), 1);

  // src/lib/components/simple-menu.tsx
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
    return /* @__PURE__ */ u2("div", { style: styleOverride, children: menuItems.map(({ key, label }) => /* @__PURE__ */ u2(
      "button",
      {
        style: { ...MENU_ITEM_STYLE, display: "block" },
        onClick: (_2) => onItemSelected(key),
        children: label
      },
      key
    )) });
  };

  // src/context-menu-widget.tsx
  var MOUSE_BUTTON_RIGHT = 2;
  var MOUSE_WHICH_RIGHT = 3;
  var ContextMenuWidget = class extends import_core13.Widget {
    constructor(props) {
      super(props);
      this.className = "deck-widget-context-menu";
      this.placement = "fill";
      this.pickInfo = null;
      this.pickInfo = null;
      this.setProps(this.props);
    }
    onAdd({ deck }) {
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
      deck.getCanvas()?.addEventListener("click", () => this.hide());
      deck.getCanvas()?.addEventListener("contextmenu", (event) => this.handleContextMenu(event));
      return element;
    }
    onRenderHTML(rootElement) {
      const { visible, position, menuItems } = this.props;
      const ui = visible && menuItems.length ? /* @__PURE__ */ u2(
        SimpleMenu,
        {
          menuItems,
          onItemSelected: (key) => this.props.onMenuItemSelected(key, this.pickInfo),
          position,
          style: { pointerEvents: "auto" }
        }
      ) : null;
      B(ui, rootElement);
    }
    handleContextMenu(srcEvent) {
      if (srcEvent && (srcEvent.button === MOUSE_BUTTON_RIGHT || srcEvent.which === MOUSE_WHICH_RIGHT)) {
        this.pickInfo = this.deck?.pickObject({
          x: srcEvent.clientX,
          y: srcEvent.clientY
        }) || null;
        const menuItems = this.pickInfo && this.props.getMenuItems?.(this.pickInfo, this) || [];
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
    ...import_core13.Widget.defaultProps,
    id: "context",
    viewId: null,
    visible: false,
    position: { x: 0, y: 0 },
    getMenuItems: void 0,
    menuItems: [],
    // eslint-disable-next-line no-console
    onMenuItemSelected: (key, pickInfo) => console.log("Context menu item selected:", key, pickInfo)
  };

  // src/timeline-widget.tsx
  var import_core14 = __toESM(require_core(), 1);
  var TimelineWidget = class extends import_core14.Widget {
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
      this.handleSliderChange = (e3) => {
        const input = e3.target;
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
      B(
        /* @__PURE__ */ u2("div", { style: { display: "flex", alignItems: "center", pointerEvents: "auto" }, children: [
          /* @__PURE__ */ u2(IconButton, { label: this.playing ? "Pause" : "Play", onClick: this.handlePlayPause, children: /* @__PURE__ */ u2("div", { className: "text", children: this.playing ? "\u23F8" : "\u25B6" }) }),
          /* @__PURE__ */ u2(
            "input",
            {
              type: "range",
              className: "timeline-slider",
              min: this.props.timeRange[0],
              max: this.props.timeRange[1],
              step: this.props.step,
              value: this.currentTime,
              onInput: this.handleSliderChange
            }
          )
        ] }),
        rootElement
      );
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
    ...import_core14.Widget.defaultProps,
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

  // src/screenshot-widget.tsx
  var import_core15 = __toESM(require_core(), 1);
  var ScreenshotWidget = class extends import_core15.Widget {
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
      B(
        /* @__PURE__ */ u2(
          IconButton,
          {
            className: "deck-widget-camera",
            label: this.props.label,
            onClick: this.handleClick.bind(this)
          }
        ),
        rootElement
      );
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
      const canvas = this.deck?.getCanvas();
      return canvas?.toDataURL(imageFormat);
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
    ...import_core15.Widget.defaultProps,
    id: "screenshot",
    placement: "top-left",
    viewId: null,
    label: "Screenshot",
    filename: "screenshot.png",
    imageFormat: "image/png",
    onCapture: void 0
  };

  // src/theme-widget.tsx
  var import_core16 = __toESM(require_core(), 1);
  var import_core17 = __toESM(require_core(), 1);

  // src/themes.ts
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

  // src/theme-widget.tsx
  var ThemeWidget = class extends import_core17.Widget {
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
          if (props.lightModeTheme && !(0, import_core16._deepEqual)(props.lightModeTheme, lightModeTheme, 1)) {
            this._setThemeMode("light");
          }
          break;
        case "dark":
          if (props.darkModeTheme && !(0, import_core16._deepEqual)(props.darkModeTheme, darkModeTheme, 1)) {
            this._setThemeMode("dark");
          }
          break;
        default:
          import_core16.log.warn(`Invalid theme mode ${this.themeMode}`)();
      }
    }
    onRenderHTML(rootElement) {
      const { lightModeLabel, darkModeLabel } = this.props;
      B(
        /* @__PURE__ */ u2(
          IconButton,
          {
            onClick: this._handleClick.bind(this),
            label: this.themeMode === "dark" ? darkModeLabel : lightModeLabel,
            className: this.themeMode === "dark" ? "deck-widget-moon" : "deck-widget-sun"
          }
        ),
        rootElement
      );
    }
    onAdd() {
      this._setThemeMode(this.themeMode);
    }
    _handleClick() {
      const newThemeMode = this.themeMode === "dark" ? "light" : "dark";
      this._setThemeMode(newThemeMode);
    }
    _setThemeMode(themeMode) {
      this.themeMode = themeMode;
      const container = this.rootElement?.closest(".deck-widget-container");
      if (container) {
        const themeStyle = themeMode === "dark" ? this.props.darkModeTheme : this.props.lightModeTheme;
        (0, import_core16._applyStyles)(container, themeStyle);
        const label = this.themeMode === "dark" ? this.props.darkModeLabel : this.props.lightModeLabel;
        import_core16.log.log(1, `Switched theme to ${label}`, themeStyle)();
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
    ...import_core17.Widget.defaultProps,
    id: "theme",
    placement: "top-left",
    viewId: null,
    lightModeLabel: "Light Mode",
    lightModeTheme: LightGlassTheme,
    darkModeLabel: "Dark Mode",
    darkModeTheme: DarkGlassTheme,
    initialThemeMode: "auto"
  };

  // src/loading-widget.tsx
  var import_core18 = __toESM(require_core(), 1);
  var LoadingWidget = class extends import_core18.Widget {
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
      B(
        // TODO(ibgreen) - this should not be a button, but styling is so nested that it is easier to reuse this component.
        this.loading && /* @__PURE__ */ u2(
          IconButton,
          {
            className: "deck-widget-spinner",
            label: this.props.label,
            onClick: this.handleClick.bind(this)
          }
        ),
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
    ...import_core18.Widget.defaultProps,
    id: "loading",
    placement: "top-left",
    viewId: null,
    label: "Loading layer data"
  };

  // src/fps-widget.tsx
  var import_core19 = __toESM(require_core(), 1);
  var FpsWidget = class extends import_core19.Widget {
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
      B(
        /* @__PURE__ */ u2(IconButton, { children: /* @__PURE__ */ u2("div", { className: "text", children: [
          "FPS",
          /* @__PURE__ */ u2("br", {}),
          fps
        ] }) }),
        rootElement
      );
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
      return Math.round(this.deck?.metrics.fps ?? 0);
    }
  };
  FpsWidget.defaultProps = {
    ...import_core19.Widget.defaultProps,
    id: "fps",
    placement: "top-left",
    viewId: null
  };

  // src/stats-widget.tsx
  var import_core20 = __toESM(require_core(), 1);
  var import_core21 = __toESM(require_core2(), 1);
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
  var StatsWidget = class extends import_core20.Widget {
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
          const f4 = props.formatters[name];
          this._formatters[name] = typeof f4 === "string" ? DEFAULT_FORMATTERS[f4] || DEFAULT_COUNT_FORMATTER : f4;
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
      const title = this.props.title || stats?.id || "Stats";
      const items = [];
      if (!collapsed && stats) {
        stats.forEach((stat) => {
          const lines = this._getLines(stat);
          if (this._resetOnUpdate && this._resetOnUpdate[stat.name]) {
            stat.reset();
          }
          lines.forEach((line, i4) => {
            items.push(
              /* @__PURE__ */ u2("div", { style: { whiteSpace: "pre" }, children: line }, `${stat.name}-${i4}`)
            );
          });
        });
      }
      B(
        /* @__PURE__ */ u2("div", { className: "deck-widget-stats-container", style: { cursor: "default" }, children: [
          /* @__PURE__ */ u2(
            "div",
            {
              className: "deck-widget-stats-header",
              style: { cursor: "pointer", pointerEvents: "auto" },
              onClick: this._toggleCollapsed,
              children: [
                collapsed ? RIGHT_ARROW : DOWN_ARROW,
                " ",
                title
              ]
            }
          ),
          !collapsed && /* @__PURE__ */ u2("div", { className: "deck-widget-stats-content", children: items })
        ] }),
        rootElement
      );
    }
    onRedraw() {
      const framesPerUpdate = Math.max(1, this.props.framesPerUpdate || 1);
      if (this._counter++ % framesPerUpdate === 0) {
        this._stats = this._getStats();
        this.updateHTML();
      }
    }
    _getStats() {
      switch (this.props.type) {
        case "deck":
          return this.deck?.stats;
        case "luma":
          return Array.from(import_core21.luma.stats.stats.values())[0];
        case "device":
          const device = this.deck?.device;
          const stats = device?.statsManager.stats.values();
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
    ...import_core20.Widget.defaultProps,
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
  return __toCommonJS(bundle_exports);
})();
      return __exports__;
      });
