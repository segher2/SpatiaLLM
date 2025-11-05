/** Deeply copies a JS data structure */
export function deepCopy(object) {
    // don't copy binary data
    if (ArrayBuffer.isView(object) ||
        object instanceof ArrayBuffer ||
        object instanceof ImageBitmap) {
        return object;
    }
    if (Array.isArray(object)) {
        return object.map(deepCopy);
    }
    if (object && typeof object === 'object') {
        const result = {};
        for (const key in object) {
            result[key] = deepCopy(object[key]);
        }
        return result;
    }
    return object;
}
//# sourceMappingURL=deep-copy.js.map