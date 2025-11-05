import { Widget, type WidgetProps } from '@deck.gl/core';
export declare function useWidget<WidgetT extends Widget, WidgetPropsT extends WidgetProps>(WidgetClass: {
    new (props_: WidgetPropsT): WidgetT;
}, props: WidgetPropsT): WidgetT;
//# sourceMappingURL=use-widget.d.ts.map