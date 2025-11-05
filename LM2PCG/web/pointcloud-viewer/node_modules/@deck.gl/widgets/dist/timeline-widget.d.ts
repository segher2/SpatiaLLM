import { Widget, type WidgetPlacement, type WidgetProps } from '@deck.gl/core';
export type TimelineWidgetProps = WidgetProps & {
    /** Widget positioning within the view. Default 'bottom-left'. */
    placement?: WidgetPlacement;
    /** View to attach to and interact with. Required when using multiple views. */
    viewId?: string | null;
    /** Slider timeRange [min, max]. */
    timeRange?: [number, number];
    /** Slider step. */
    step?: number;
    /** Initial slider value. */
    initialTime?: number;
    /** Callback when value changes. */
    onTimeChange?: (value: number) => void;
    /** Play interval in milliseconds. */
    playInterval?: number;
};
export declare class TimelineWidget extends Widget<TimelineWidgetProps> {
    id: string;
    className: string;
    placement: WidgetPlacement;
    private playing;
    private timerId;
    currentTime: number;
    static defaultProps: Required<TimelineWidgetProps>;
    constructor(props?: TimelineWidgetProps);
    setProps(props: Partial<TimelineWidgetProps>): void;
    onAdd(): void;
    onRemove(): void;
    onRenderHTML(rootElement: HTMLElement): void;
    private handlePlayPause;
    private handleSliderChange;
    private start;
    private stop;
    private tick;
}
//# sourceMappingURL=timeline-widget.d.ts.map