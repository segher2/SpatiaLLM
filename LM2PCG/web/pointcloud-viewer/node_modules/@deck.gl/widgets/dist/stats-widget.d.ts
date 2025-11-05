import { Widget, type WidgetPlacement, type WidgetProps } from '@deck.gl/core';
import type { Stats, Stat } from '@probe.gl/stats';
export declare const DEFAULT_FORMATTERS: Record<string, (stat: Stat) => string>;
export type StatsWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'top-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Type of stats to display. */
    type?: 'deck' | 'luma' | 'device' | 'custom';
    /** Stats object to visualize. */
    stats?: Stats;
    /** Title shown in the header of the pop-up. Defaults to stats.id. */
    title?: string;
    /** How many redraws to wait between updates. */
    framesPerUpdate?: number;
    /** Custom formatters for stat values. */
    formatters?: Record<string, string | ((stat: Stat) => string)>;
    /** Whether to reset particular stats after each update. */
    resetOnUpdate?: Record<string, boolean>;
};
/** Displays probe.gl stats in a floating pop-up. */
export declare class StatsWidget extends Widget<StatsWidgetProps> {
    static defaultProps: Required<StatsWidgetProps>;
    className: string;
    placement: WidgetPlacement;
    private _counter;
    private _formatters;
    private _resetOnUpdate;
    collapsed: boolean;
    _stats: Stats;
    constructor(props?: StatsWidgetProps);
    setProps(props: Partial<StatsWidgetProps>): void;
    onAdd(): void;
    onRenderHTML(rootElement: HTMLElement): void;
    onRedraw(): void;
    protected _getStats(): Stats;
    protected _toggleCollapsed: () => void;
    protected _getLines(stat: Stat): string[];
}
//# sourceMappingURL=stats-widget.d.ts.map