import type { TypedArray } from '@math.gl/types';
export declare const ATTRIBUTE_TYPE_TO_COMPONENTS: Record<string, number>;
export declare const ATTRIBUTE_COMPONENT_TYPE_TO_ARRAY: Record<number, any>;
type GLTFAccessor = {
    componentType: number;
    type: string;
    count: number;
    bufferView?: {
        data: {
            buffer: ArrayBuffer;
            byteOffset?: number;
        };
    };
    byteOffset?: number;
};
export declare function accessorToTypedArray(accessor: GLTFAccessor): {
    typedArray: TypedArray;
    components: number;
};
export {};
//# sourceMappingURL=convert-webgl-attribute.d.ts.map