/**
 * A Deck Theme is a set of CSS variables that control CSS styling of the official widgets.
 */
export type DeckWidgetTheme = {
    '--widget-margin'?: string;
    '--button-size'?: string;
    '--button-corner-radius'?: string;
    '--button-background'?: string;
    '--button-stroke'?: string;
    '--button-inner-stroke'?: string;
    '--button-shadow'?: string;
    '--button-backdrop-filter'?: string;
    '--button-icon-idle'?: string;
    '--button-icon-hover'?: string;
    '--button-text'?: string;
    '--icon-compass-north-color'?: string;
    '--icon-compass-south-color'?: string;
    '--menu-gap'?: string;
};
export declare const LightTheme: {
    readonly '--widget-margin': "12px";
    readonly '--button-size': "28px";
    readonly '--button-corner-radius': "8px";
    readonly '--button-background': "#fff";
    readonly '--button-stroke': "rgba(255, 255, 255, 0.3)";
    readonly '--button-inner-stroke': "unset";
    readonly '--button-shadow': "0px 0px 8px 0px rgba(0, 0, 0, 0.25)";
    readonly '--button-backdrop-filter': "unset";
    readonly '--button-icon-idle': "rgba(97, 97, 102, 1)";
    readonly '--button-icon-hover': "rgba(24, 24, 26, 1)";
    readonly '--button-text': "rgb(24, 24, 26, 1)";
    readonly '--icon-compass-north-color': "rgb(240, 92, 68)";
    readonly '--icon-compass-south-color': "rgb(204, 204, 204)";
    readonly '--menu-gap': "4px";
};
export declare const DarkTheme: {
    readonly '--widget-margin': "12px";
    readonly '--button-size': "28px";
    readonly '--button-corner-radius': "8px";
    readonly '--button-background': "rgba(18, 18, 20, 1)";
    readonly '--button-stroke': "rgba(18, 18, 20, 0.30)";
    readonly '--button-inner-stroke': "unset";
    readonly '--button-shadow': "0px 0px 8px 0px rgba(0, 0, 0, 0.25)";
    readonly '--button-backdrop-filter': "unset";
    readonly '--button-icon-idle': "rgba(158, 157, 168, 1)";
    readonly '--button-icon-hover': "rgba(215, 214, 229, 1)";
    readonly '--button-text': "rgb(215, 214, 229, 1)";
    readonly '--icon-compass-north-color': "rgb(240, 92, 68)";
    readonly '--icon-compass-south-color': "rgb(200, 199, 209)";
    readonly '--menu-gap': "4px";
};
export declare const LightGlassTheme: {
    readonly '--widget-margin': "12px";
    readonly '--button-size': "28px";
    readonly '--button-corner-radius': "8px";
    readonly '--button-background': "rgba(255, 255, 255, 0.6)";
    readonly '--button-stroke': "rgba(255, 255, 255, 0.3)";
    readonly '--button-inner-stroke': "1px solid rgba(255, 255, 255, 0.6)";
    readonly '--button-shadow': "0px 0px 8px 0px rgba(0, 0, 0, 0.25), 0px 0px 8px 0px rgba(0, 0, 0, 0.1) inset";
    readonly '--button-backdrop-filter': "blur(4px)";
    readonly '--button-icon-idle': "rgba(97, 97, 102, 1)";
    readonly '--button-icon-hover': "rgba(24, 24, 26, 1)";
    readonly '--button-text': "rgb(24, 24, 26, 1)";
    readonly '--icon-compass-north-color': "rgb(240, 92, 68)";
    readonly '--icon-compass-south-color': "rgb(204, 204, 204)";
    readonly '--menu-gap': "4px";
};
export declare const DarkGlassTheme: {
    readonly '--widget-margin': "12px";
    readonly '--button-size': "28px";
    readonly '--button-corner-radius': "8px";
    readonly '--button-background': "rgba(18, 18, 20, 0.75)";
    readonly '--button-stroke': "rgba(18, 18, 20, 0.30)";
    readonly '--button-inner-stroke': "1px solid rgba(18, 18, 20, 0.75)";
    readonly '--button-shadow': "0px 0px 8px 0px rgba(0, 0, 0, 0.25), 0px 0px 8px 0px rgba(0, 0, 0, 0.1) inset";
    readonly '--button-backdrop-filter': "blur(4px)";
    readonly '--button-icon-idle': "rgba(158, 157, 168, 1)";
    readonly '--button-icon-hover': "rgba(215, 214, 229, 1)";
    readonly '--button-text': "rgb(215, 214, 229, 1)";
    readonly '--icon-compass-north-color': "rgb(240, 92, 68)";
    readonly '--icon-compass-south-color': "rgb(200, 199, 209)";
    readonly '--menu-gap': "4px";
};
//# sourceMappingURL=themes.d.ts.map