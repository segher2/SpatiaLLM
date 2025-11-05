// luma.gl
// SPDX-License-Identifier: MIT
// Copyright (c) vis.gl contributors
import { Geometry, GroupNode } from '@luma.gl/engine';
import { Matrix4 } from '@math.gl/core';
import { convertGLDrawModeToTopology } from "../webgl-to-webgpu/convert-webgl-topology.js";
import { createGLTFModel } from "../gltf/create-gltf-model.js";
import { parsePBRMaterial } from "./parse-pbr-material.js";
const defaultOptions = {
    modelOptions: {},
    pbrDebug: false,
    imageBasedLightingEnvironment: undefined,
    lights: true,
    useTangents: false
};
/**
 * GLTF instantiator for luma.gl
 * Walks the parsed and resolved glTF structure and builds a luma.gl scenegraph
 */
export function parseGLTF(device, gltf, options_ = {}) {
    const options = { ...defaultOptions, ...options_ };
    const sceneNodes = gltf.scenes.map(gltfScene => createScene(device, gltfScene, gltf.nodes, options));
    return sceneNodes;
}
function createScene(device, gltfScene, gltfNodes, options) {
    const gltfSceneNodes = gltfScene.nodes || [];
    const nodes = gltfSceneNodes.map(node => createNode(device, node, gltfNodes, options));
    const sceneNode = new GroupNode({
        id: gltfScene.name || gltfScene.id,
        children: nodes
    });
    return sceneNode;
}
function createNode(device, gltfNode, gltfNodes, options) {
    if (!gltfNode._node) {
        const gltfChildren = gltfNode.children || [];
        const children = gltfChildren.map(child => createNode(device, child, gltfNodes, options));
        // Node can have children nodes and meshes at the same time
        if (gltfNode.mesh) {
            children.push(createMesh(device, gltfNode.mesh, options));
        }
        const node = new GroupNode({
            id: gltfNode.name || gltfNode.id,
            children
        });
        if (gltfNode.matrix) {
            node.setMatrix(gltfNode.matrix);
        }
        else {
            node.matrix.identity();
            if (gltfNode.translation) {
                node.matrix.translate(gltfNode.translation);
            }
            if (gltfNode.rotation) {
                const rotationMatrix = new Matrix4().fromQuaternion(gltfNode.rotation);
                node.matrix.multiplyRight(rotationMatrix);
            }
            if (gltfNode.scale) {
                node.matrix.scale(gltfNode.scale);
            }
        }
        gltfNode._node = node;
    }
    // Copy _node so that gltf-animator can access
    const topLevelNode = gltfNodes.find(node => node.id === gltfNode.id);
    topLevelNode._node = gltfNode._node;
    return gltfNode._node;
}
function createMesh(device, gltfMesh, options) {
    // TODO: avoid changing the gltf
    if (!gltfMesh._mesh) {
        const gltfPrimitives = gltfMesh.primitives || [];
        const primitives = gltfPrimitives.map((gltfPrimitive, i) => createPrimitive(device, gltfPrimitive, i, gltfMesh, options));
        const mesh = new GroupNode({
            id: gltfMesh.name || gltfMesh.id,
            children: primitives
        });
        gltfMesh._mesh = mesh;
    }
    return gltfMesh._mesh;
}
function createPrimitive(device, gltfPrimitive, i, gltfMesh, options) {
    const id = gltfPrimitive.name || `${gltfMesh.name || gltfMesh.id}-primitive-${i}`;
    const topology = convertGLDrawModeToTopology(gltfPrimitive.mode || 4);
    const vertexCount = gltfPrimitive.indices
        ? gltfPrimitive.indices.count
        : getVertexCount(gltfPrimitive.attributes);
    const geometry = createGeometry(id, gltfPrimitive, topology);
    const parsedPPBRMaterial = parsePBRMaterial(device, gltfPrimitive.material, geometry.attributes, options);
    const modelNode = createGLTFModel(device, {
        id,
        geometry: createGeometry(id, gltfPrimitive, topology),
        parsedPPBRMaterial,
        modelOptions: options.modelOptions,
        vertexCount
    });
    modelNode.bounds = [gltfPrimitive.attributes.POSITION.min, gltfPrimitive.attributes.POSITION.max];
    // TODO this holds on to all the CPU side texture and attribute data
    // modelNode.material =  gltfPrimitive.material;
    return modelNode;
}
function getVertexCount(attributes) {
    throw new Error('getVertexCount not implemented');
}
function createGeometry(id, gltfPrimitive, topology) {
    const attributes = {};
    for (const [attributeName, attribute] of Object.entries(gltfPrimitive.attributes)) {
        const { components, size, value } = attribute;
        attributes[attributeName] = { size: size ?? components, value };
    }
    return new Geometry({
        id,
        topology,
        indices: gltfPrimitive.indices.value,
        attributes
    });
}
//# sourceMappingURL=parse-gltf.js.map