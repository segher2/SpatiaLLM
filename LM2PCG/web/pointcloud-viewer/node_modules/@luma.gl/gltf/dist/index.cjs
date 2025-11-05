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
  GLTFAnimator: () => GLTFAnimator,
  createScenegraphsFromGLTF: () => createScenegraphsFromGLTF,
  loadPBREnvironment: () => loadPBREnvironment,
  parsePBRMaterial: () => parsePBRMaterial
});
module.exports = __toCommonJS(dist_exports);

// dist/pbr/pbr-environment.js
var import_engine = require("@luma.gl/engine");
var import_textures = require("@loaders.gl/textures");
function loadPBREnvironment(device, props) {
  const brdfLutTexture = new import_engine.AsyncTexture(device, {
    id: "brdfLUT",
    sampler: {
      addressModeU: "clamp-to-edge",
      addressModeV: "clamp-to-edge",
      minFilter: "linear",
      magFilter: "linear"
    },
    // Texture accepts a promise that returns an image as data (Async Textures)
    data: (0, import_textures.loadImageTexture)(props.brdfLutUrl)
  });
  const diffuseEnvSampler = makeCube(device, {
    id: "DiffuseEnvSampler",
    getTextureForFace: (dir) => (0, import_textures.loadImageTexture)(props.getTexUrl("diffuse", dir, 0)),
    sampler: {
      addressModeU: "clamp-to-edge",
      addressModeV: "clamp-to-edge",
      minFilter: "linear",
      magFilter: "linear"
    }
  });
  const specularEnvSampler = makeCube(device, {
    id: "SpecularEnvSampler",
    getTextureForFace: (dir) => {
      const imageArray = [];
      for (let lod = 0; lod <= props.specularMipLevels - 1; lod++) {
        imageArray.push((0, import_textures.loadImageTexture)(props.getTexUrl("specular", dir, lod)));
      }
      return imageArray;
    },
    sampler: {
      addressModeU: "clamp-to-edge",
      addressModeV: "clamp-to-edge",
      minFilter: "linear",
      // [GL.TEXTURE_MIN_FILTER]: GL.LINEAR_MIPMAP_LINEAR,
      magFilter: "linear"
    }
  });
  return {
    brdfLutTexture,
    diffuseEnvSampler,
    specularEnvSampler
  };
}
var FACES = [0, 1, 2, 3, 4, 5];
function makeCube(device, { id, getTextureForFace, sampler }) {
  const data = {};
  FACES.forEach((face) => {
    data[String(face)] = getTextureForFace(face);
  });
  return new import_engine.AsyncTexture(device, {
    id,
    dimension: "cube",
    mipmaps: false,
    sampler,
    // @ts-expect-error
    data
  });
}

// dist/parsers/parse-pbr-material.js
var import_constants2 = require("@luma.gl/constants");
var import_core = require("@luma.gl/core");

// dist/webgl-to-webgpu/convert-webgl-sampler.js
var import_constants = require("@luma.gl/constants");
function convertSampler(gltfSampler) {
  return {
    addressModeU: convertSamplerWrapMode(gltfSampler.wrapS),
    addressModeV: convertSamplerWrapMode(gltfSampler.wrapT),
    magFilter: convertSamplerMagFilter(gltfSampler.magFilter),
    ...convertSamplerMinFilter(gltfSampler.minFilter)
  };
}
function convertSamplerWrapMode(mode) {
  switch (mode) {
    case 33071:
      return "clamp-to-edge";
    case 10497:
      return "repeat";
    case 33648:
      return "mirror-repeat";
    default:
      return void 0;
  }
}
function convertSamplerMagFilter(mode) {
  switch (mode) {
    case 9728:
      return "nearest";
    case 9729:
      return "linear";
    default:
      return void 0;
  }
}
function convertSamplerMinFilter(mode) {
  switch (mode) {
    case 9728:
      return { minFilter: "nearest" };
    case 9729:
      return { minFilter: "linear" };
    case 9984:
      return { minFilter: "nearest", mipmapFilter: "nearest" };
    case 9985:
      return { minFilter: "linear", mipmapFilter: "nearest" };
    case 9986:
      return { minFilter: "nearest", mipmapFilter: "linear" };
    case 9987:
      return { minFilter: "linear", mipmapFilter: "linear" };
    default:
      return {};
  }
}

// dist/parsers/parse-pbr-material.js
function parsePBRMaterial(device, material, attributes, options) {
  const parsedMaterial = {
    defines: {
      // TODO: Use EXT_sRGB if available (Standard in WebGL 2.0)
      MANUAL_SRGB: true,
      SRGB_FAST_APPROXIMATION: true
    },
    bindings: {},
    uniforms: {
      // TODO: find better values?
      camera: [0, 0, 0],
      // Model should override
      metallicRoughnessValues: [1, 1]
      // Default is 1 and 1
    },
    parameters: {},
    glParameters: {},
    generatedTextures: []
  };
  parsedMaterial.defines["USE_TEX_LOD"] = true;
  const { imageBasedLightingEnvironment } = options;
  if (imageBasedLightingEnvironment) {
    parsedMaterial.bindings.pbr_diffuseEnvSampler = imageBasedLightingEnvironment.diffuseEnvSampler.texture;
    parsedMaterial.bindings.pbr_specularEnvSampler = imageBasedLightingEnvironment.specularEnvSampler.texture;
    parsedMaterial.bindings.pbr_BrdfLUT = imageBasedLightingEnvironment.brdfLutTexture.texture;
    parsedMaterial.uniforms.scaleIBLAmbient = [1, 1];
  }
  if (options == null ? void 0 : options.pbrDebug) {
    parsedMaterial.defines["PBR_DEBUG"] = true;
    parsedMaterial.uniforms.scaleDiffBaseMR = [0, 0, 0, 0];
    parsedMaterial.uniforms.scaleFGDSpec = [0, 0, 0, 0];
  }
  if (attributes["NORMAL"])
    parsedMaterial.defines["HAS_NORMALS"] = true;
  if (attributes["TANGENT"] && (options == null ? void 0 : options.useTangents))
    parsedMaterial.defines["HAS_TANGENTS"] = true;
  if (attributes["TEXCOORD_0"])
    parsedMaterial.defines["HAS_UV"] = true;
  if (options == null ? void 0 : options.imageBasedLightingEnvironment)
    parsedMaterial.defines["USE_IBL"] = true;
  if (options == null ? void 0 : options.lights)
    parsedMaterial.defines["USE_LIGHTS"] = true;
  if (material) {
    parseMaterial(device, material, parsedMaterial);
  }
  return parsedMaterial;
}
function parseMaterial(device, material, parsedMaterial) {
  parsedMaterial.uniforms.unlit = Boolean(material.unlit);
  if (material.pbrMetallicRoughness) {
    parsePbrMetallicRoughness(device, material.pbrMetallicRoughness, parsedMaterial);
  }
  if (material.normalTexture) {
    addTexture(device, material.normalTexture, "pbr_normalSampler", "HAS_NORMALMAP", parsedMaterial);
    const { scale = 1 } = material.normalTexture;
    parsedMaterial.uniforms.normalScale = scale;
  }
  if (material.occlusionTexture) {
    addTexture(device, material.occlusionTexture, "pbr_occlusionSampler", "HAS_OCCLUSIONMAP", parsedMaterial);
    const { strength = 1 } = material.occlusionTexture;
    parsedMaterial.uniforms.occlusionStrength = strength;
  }
  if (material.emissiveTexture) {
    addTexture(device, material.emissiveTexture, "pbr_emissiveSampler", "HAS_EMISSIVEMAP", parsedMaterial);
    parsedMaterial.uniforms.emissiveFactor = material.emissiveFactor || [0, 0, 0];
  }
  switch (material.alphaMode || "MASK") {
    case "MASK":
      const { alphaCutoff = 0.5 } = material;
      parsedMaterial.defines["ALPHA_CUTOFF"] = true;
      parsedMaterial.uniforms.alphaCutoff = alphaCutoff;
      break;
    case "BLEND":
      import_core.log.warn("glTF BLEND alphaMode might not work well because it requires mesh sorting")();
      parsedMaterial.parameters.blend = true;
      parsedMaterial.parameters.blendColorOperation = "add";
      parsedMaterial.parameters.blendColorSrcFactor = "src-alpha";
      parsedMaterial.parameters.blendColorDstFactor = "one-minus-src-alpha";
      parsedMaterial.parameters.blendAlphaOperation = "add";
      parsedMaterial.parameters.blendAlphaSrcFactor = "one";
      parsedMaterial.parameters.blendAlphaDstFactor = "one-minus-src-alpha";
      parsedMaterial.glParameters["blend"] = true;
      parsedMaterial.glParameters["blendEquation"] = 32774;
      parsedMaterial.glParameters["blendFunc"] = [
        770,
        771,
        1,
        771
      ];
      break;
  }
}
function parsePbrMetallicRoughness(device, pbrMetallicRoughness, parsedMaterial) {
  if (pbrMetallicRoughness.baseColorTexture) {
    addTexture(device, pbrMetallicRoughness.baseColorTexture, "pbr_baseColorSampler", "HAS_BASECOLORMAP", parsedMaterial);
  }
  parsedMaterial.uniforms.baseColorFactor = pbrMetallicRoughness.baseColorFactor || [1, 1, 1, 1];
  if (pbrMetallicRoughness.metallicRoughnessTexture) {
    addTexture(device, pbrMetallicRoughness.metallicRoughnessTexture, "pbr_metallicRoughnessSampler", "HAS_METALROUGHNESSMAP", parsedMaterial);
  }
  const { metallicFactor = 1, roughnessFactor = 1 } = pbrMetallicRoughness;
  parsedMaterial.uniforms.metallicRoughnessValues = [metallicFactor, roughnessFactor];
}
function addTexture(device, gltfTexture, uniformName, define, parsedMaterial) {
  var _a;
  const image = gltfTexture.texture.source.image;
  let textureOptions;
  if (image.compressed) {
    textureOptions = image;
  } else {
    textureOptions = { data: image };
  }
  const gltfSampler = {
    wrapS: 10497,
    // default REPEAT S (U) wrapping mode.
    wrapT: 10497,
    // default REPEAT T (V) wrapping mode.
    ...(_a = gltfTexture == null ? void 0 : gltfTexture.texture) == null ? void 0 : _a.sampler
  };
  const texture = device.createTexture({
    id: gltfTexture.uniformName || gltfTexture.id,
    sampler: convertSampler(gltfSampler),
    ...textureOptions
  });
  parsedMaterial.bindings[uniformName] = texture;
  if (define)
    parsedMaterial.defines[define] = true;
  parsedMaterial.generatedTextures.push(texture);
}

// dist/parsers/parse-gltf.js
var import_engine3 = require("@luma.gl/engine");
var import_core3 = require("@math.gl/core");

// dist/webgl-to-webgpu/convert-webgl-topology.js
var GLEnum;
(function(GLEnum2) {
  GLEnum2[GLEnum2["POINTS"] = 0] = "POINTS";
  GLEnum2[GLEnum2["LINES"] = 1] = "LINES";
  GLEnum2[GLEnum2["LINE_LOOP"] = 2] = "LINE_LOOP";
  GLEnum2[GLEnum2["LINE_STRIP"] = 3] = "LINE_STRIP";
  GLEnum2[GLEnum2["TRIANGLES"] = 4] = "TRIANGLES";
  GLEnum2[GLEnum2["TRIANGLE_STRIP"] = 5] = "TRIANGLE_STRIP";
  GLEnum2[GLEnum2["TRIANGLE_FAN"] = 6] = "TRIANGLE_FAN";
})(GLEnum || (GLEnum = {}));
function convertGLDrawModeToTopology(drawMode) {
  switch (drawMode) {
    case GLEnum.POINTS:
      return "point-list";
    case GLEnum.LINES:
      return "line-list";
    case GLEnum.LINE_STRIP:
      return "line-strip";
    case GLEnum.TRIANGLES:
      return "triangle-list";
    case GLEnum.TRIANGLE_STRIP:
      return "triangle-strip";
    default:
      throw new Error(String(drawMode));
  }
}

// dist/gltf/create-gltf-model.js
var import_core2 = require("@luma.gl/core");
var import_shadertools = require("@luma.gl/shadertools");
var import_engine2 = require("@luma.gl/engine");
var SHADER = (
  /* WGSL */
  `
layout(0) positions: vec4; // in vec4 POSITION;

  #ifdef HAS_NORMALS
    in vec4 normals; // in vec4 NORMAL;
  #endif

  #ifdef HAS_TANGENTS
    in vec4 TANGENT;
  #endif

  #ifdef HAS_UV
    // in vec2 TEXCOORD_0;
    in vec2 texCoords;
  #endif

@vertex
  void main(void) {
    vec4 _NORMAL = vec4(0.);
    vec4 _TANGENT = vec4(0.);
    vec2 _TEXCOORD_0 = vec2(0.);

    #ifdef HAS_NORMALS
      _NORMAL = normals;
    #endif

    #ifdef HAS_TANGENTS
      _TANGENT = TANGENT;
    #endif

    #ifdef HAS_UV
      _TEXCOORD_0 = texCoords;
    #endif

    pbr_setPositionNormalTangentUV(positions, _NORMAL, _TANGENT, _TEXCOORD_0);
    gl_Position = u_MVPMatrix * positions;
  }

@fragment
  out vec4 fragmentColor;

  void main(void) {
    vec3 pos = pbr_vPosition;
    fragmentColor = pbr_filterColor(vec4(1.0));
  }
`
);
var vs = (
  /* glsl */
  `#version 300 es

  // in vec4 POSITION;
  in vec4 positions;

  #ifdef HAS_NORMALS
    // in vec4 NORMAL;
    in vec4 normals;
  #endif

  #ifdef HAS_TANGENTS
    in vec4 TANGENT;
  #endif

  #ifdef HAS_UV
    // in vec2 TEXCOORD_0;
    in vec2 texCoords;
  #endif

  void main(void) {
    vec4 _NORMAL = vec4(0.);
    vec4 _TANGENT = vec4(0.);
    vec2 _TEXCOORD_0 = vec2(0.);

    #ifdef HAS_NORMALS
      _NORMAL = normals;
    #endif

    #ifdef HAS_TANGENTS
      _TANGENT = TANGENT;
    #endif

    #ifdef HAS_UV
      _TEXCOORD_0 = texCoords;
    #endif

    pbr_setPositionNormalTangentUV(positions, _NORMAL, _TANGENT, _TEXCOORD_0);
    gl_Position = pbrProjection.modelViewProjectionMatrix * positions;
  }
`
);
var fs = (
  /* glsl */
  `#version 300 es
  out vec4 fragmentColor;

  void main(void) {
    vec3 pos = pbr_vPosition;
    fragmentColor = pbr_filterColor(vec4(1.0));
  }
`
);
function createGLTFModel(device, options) {
  const { id, geometry, parsedPPBRMaterial, vertexCount, modelOptions = {} } = options;
  import_core2.log.info(4, "createGLTFModel defines: ", parsedPPBRMaterial.defines)();
  const managedResources = [];
  const parameters = {
    depthWriteEnabled: true,
    depthCompare: "less",
    depthFormat: "depth24plus",
    cullMode: "back"
  };
  const modelProps = {
    id,
    source: SHADER,
    vs,
    fs,
    geometry,
    topology: geometry.topology,
    vertexCount,
    modules: [import_shadertools.pbrMaterial],
    ...modelOptions,
    defines: { ...parsedPPBRMaterial.defines, ...modelOptions.defines },
    parameters: { ...parameters, ...parsedPPBRMaterial.parameters, ...modelOptions.parameters }
  };
  const model = new import_engine2.Model(device, modelProps);
  const { camera, ...pbrMaterialProps } = {
    ...parsedPPBRMaterial.uniforms,
    ...modelOptions.uniforms,
    ...parsedPPBRMaterial.bindings,
    ...modelOptions.bindings
  };
  model.shaderInputs.setProps({ pbrMaterial: pbrMaterialProps, pbrProjection: { camera } });
  return new import_engine2.ModelNode({ managedResources, model });
}

// dist/parsers/parse-gltf.js
var defaultOptions = {
  modelOptions: {},
  pbrDebug: false,
  imageBasedLightingEnvironment: void 0,
  lights: true,
  useTangents: false
};
function parseGLTF(device, gltf, options_ = {}) {
  const options = { ...defaultOptions, ...options_ };
  const sceneNodes = gltf.scenes.map((gltfScene) => createScene(device, gltfScene, gltf.nodes, options));
  return sceneNodes;
}
function createScene(device, gltfScene, gltfNodes, options) {
  const gltfSceneNodes = gltfScene.nodes || [];
  const nodes = gltfSceneNodes.map((node) => createNode(device, node, gltfNodes, options));
  const sceneNode = new import_engine3.GroupNode({
    id: gltfScene.name || gltfScene.id,
    children: nodes
  });
  return sceneNode;
}
function createNode(device, gltfNode, gltfNodes, options) {
  if (!gltfNode._node) {
    const gltfChildren = gltfNode.children || [];
    const children = gltfChildren.map((child) => createNode(device, child, gltfNodes, options));
    if (gltfNode.mesh) {
      children.push(createMesh(device, gltfNode.mesh, options));
    }
    const node = new import_engine3.GroupNode({
      id: gltfNode.name || gltfNode.id,
      children
    });
    if (gltfNode.matrix) {
      node.setMatrix(gltfNode.matrix);
    } else {
      node.matrix.identity();
      if (gltfNode.translation) {
        node.matrix.translate(gltfNode.translation);
      }
      if (gltfNode.rotation) {
        const rotationMatrix = new import_core3.Matrix4().fromQuaternion(gltfNode.rotation);
        node.matrix.multiplyRight(rotationMatrix);
      }
      if (gltfNode.scale) {
        node.matrix.scale(gltfNode.scale);
      }
    }
    gltfNode._node = node;
  }
  const topLevelNode = gltfNodes.find((node) => node.id === gltfNode.id);
  topLevelNode._node = gltfNode._node;
  return gltfNode._node;
}
function createMesh(device, gltfMesh, options) {
  if (!gltfMesh._mesh) {
    const gltfPrimitives = gltfMesh.primitives || [];
    const primitives = gltfPrimitives.map((gltfPrimitive, i) => createPrimitive(device, gltfPrimitive, i, gltfMesh, options));
    const mesh = new import_engine3.GroupNode({
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
  const vertexCount = gltfPrimitive.indices ? gltfPrimitive.indices.count : getVertexCount(gltfPrimitive.attributes);
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
  return modelNode;
}
function getVertexCount(attributes) {
  throw new Error("getVertexCount not implemented");
}
function createGeometry(id, gltfPrimitive, topology) {
  const attributes = {};
  for (const [attributeName, attribute] of Object.entries(gltfPrimitive.attributes)) {
    const { components, size, value } = attribute;
    attributes[attributeName] = { size: size ?? components, value };
  }
  return new import_engine3.Geometry({
    id,
    topology,
    indices: gltfPrimitive.indices.value,
    attributes
  });
}

// dist/gltf/gltf-animator.js
var import_core6 = require("@luma.gl/core");
var import_core7 = require("@math.gl/core");

// dist/gltf/animations/interpolate.js
var import_core4 = require("@luma.gl/core");
var import_core5 = require("@math.gl/core");
var scratchQuaternion = new import_core5.Quaternion();
function interpolate(time, { input, interpolation, output }, target, path) {
  const maxTime = input[input.length - 1];
  const animationTime = time % maxTime;
  const nextIndex = input.findIndex((t) => t >= animationTime);
  const previousIndex = Math.max(0, nextIndex - 1);
  if (!Array.isArray(target[path])) {
    switch (path) {
      case "translation":
        target[path] = [0, 0, 0];
        break;
      case "rotation":
        target[path] = [0, 0, 0, 1];
        break;
      case "scale":
        target[path] = [1, 1, 1];
        break;
      default:
        import_core4.log.warn(`Bad animation path ${path}`)();
    }
  }
  const previousTime = input[previousIndex];
  const nextTime = input[nextIndex];
  switch (interpolation) {
    case "STEP":
      stepInterpolate(target, path, output[previousIndex]);
      break;
    case "LINEAR":
      if (nextTime > previousTime) {
        const ratio = (animationTime - previousTime) / (nextTime - previousTime);
        linearInterpolate(target, path, output[previousIndex], output[nextIndex], ratio);
      }
      break;
    case "CUBICSPLINE":
      if (nextTime > previousTime) {
        const ratio = (animationTime - previousTime) / (nextTime - previousTime);
        const tDiff = nextTime - previousTime;
        const p0 = output[3 * previousIndex + 1];
        const outTangent0 = output[3 * previousIndex + 2];
        const inTangent1 = output[3 * nextIndex + 0];
        const p1 = output[3 * nextIndex + 1];
        cubicsplineInterpolate(target, path, { p0, outTangent0, inTangent1, p1, tDiff, ratio });
      }
      break;
    default:
      import_core4.log.warn(`Interpolation ${interpolation} not supported`)();
      break;
  }
}
function linearInterpolate(target, path, start, stop, ratio) {
  if (!target[path]) {
    throw new Error();
  }
  if (path === "rotation") {
    scratchQuaternion.slerp({ start, target: stop, ratio });
    for (let i = 0; i < scratchQuaternion.length; i++) {
      target[path][i] = scratchQuaternion[i];
    }
  } else {
    for (let i = 0; i < start.length; i++) {
      target[path][i] = ratio * stop[i] + (1 - ratio) * start[i];
    }
  }
}
function cubicsplineInterpolate(target, path, { p0, outTangent0, inTangent1, p1, tDiff, ratio: t }) {
  if (!target[path]) {
    throw new Error();
  }
  for (let i = 0; i < target[path].length; i++) {
    const m0 = outTangent0[i] * tDiff;
    const m1 = inTangent1[i] * tDiff;
    target[path][i] = (2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1) * p0[i] + (Math.pow(t, 3) - 2 * Math.pow(t, 2) + t) * m0 + (-2 * Math.pow(t, 3) + 3 * Math.pow(t, 2)) * p1[i] + (Math.pow(t, 3) - Math.pow(t, 2)) * m1;
  }
}
function stepInterpolate(target, path, value) {
  if (!target[path]) {
    throw new Error();
  }
  for (let i = 0; i < value.length; i++) {
    target[path][i] = value[i];
  }
}

// dist/gltf/gltf-animator.js
var GLTFSingleAnimator = class {
  animation;
  startTime = 0;
  playing = true;
  speed = 1;
  constructor(props) {
    this.animation = props.animation;
    this.animation.name ||= "unnamed";
    Object.assign(this, props);
  }
  setTime(timeMs) {
    if (!this.playing) {
      return;
    }
    const absTime = timeMs / 1e3;
    const time = (absTime - this.startTime) * this.speed;
    this.animation.channels.forEach(({ sampler, target, path }) => {
      interpolate(time, sampler, target, path);
      applyTranslationRotationScale(target, target._node);
    });
  }
};
var GLTFAnimator = class {
  animations;
  constructor(props) {
    this.animations = props.animations.map((animation, index) => {
      const name = animation.name || `Animation-${index}`;
      return new GLTFSingleAnimator({
        animation: { name, channels: animation.channels }
      });
    });
  }
  /** @deprecated Use .setTime(). Will be removed (deck.gl is using this) */
  animate(time) {
    import_core6.log.warn("GLTFAnimator#animate is deprecated. Use GLTFAnimator#setTime instead")();
    this.setTime(time);
  }
  setTime(time) {
    this.animations.forEach((animation) => animation.setTime(time));
  }
  getAnimations() {
    return this.animations;
  }
};
var scratchMatrix = new import_core7.Matrix4();
function applyTranslationRotationScale(gltfNode, node) {
  node.matrix.identity();
  if (gltfNode.translation) {
    node.matrix.translate(gltfNode.translation);
  }
  if (gltfNode.rotation) {
    const rotationMatrix = scratchMatrix.fromQuaternion(gltfNode.rotation);
    node.matrix.multiplyRight(rotationMatrix);
  }
  if (gltfNode.scale) {
    node.matrix.scale(gltfNode.scale);
  }
}

// dist/webgl-to-webgpu/convert-webgl-attribute.js
var ATTRIBUTE_TYPE_TO_COMPONENTS = {
  SCALAR: 1,
  VEC2: 2,
  VEC3: 3,
  VEC4: 4,
  MAT2: 4,
  MAT3: 9,
  MAT4: 16
};
var ATTRIBUTE_COMPONENT_TYPE_TO_ARRAY = {
  5120: Int8Array,
  5121: Uint8Array,
  5122: Int16Array,
  5123: Uint16Array,
  5125: Uint32Array,
  5126: Float32Array
};
function accessorToTypedArray(accessor) {
  var _a;
  const ArrayType = ATTRIBUTE_COMPONENT_TYPE_TO_ARRAY[accessor.componentType];
  const components = ATTRIBUTE_TYPE_TO_COMPONENTS[accessor.type];
  const length = components * accessor.count;
  const { buffer, byteOffset = 0 } = ((_a = accessor.bufferView) == null ? void 0 : _a.data) ?? {};
  const typedArray = new ArrayType(buffer, byteOffset + (accessor.byteOffset || 0), length);
  return { typedArray, components };
}

// dist/parsers/parse-gltf-animations.js
function parseGLTFAnimations(gltf) {
  const gltfAnimations = gltf.animations || [];
  return gltfAnimations.map((animation, index) => {
    const name = animation.name || `Animation-${index}`;
    const samplers = animation.samplers.map(({ input, interpolation = "LINEAR", output }) => ({
      input: accessorToJsArray(gltf.accessors[input]),
      interpolation,
      output: accessorToJsArray(gltf.accessors[output])
    }));
    const channels = animation.channels.map(({ sampler, target }) => ({
      sampler: samplers[sampler],
      target: gltf.nodes[target.node ?? 0],
      path: target.path
    }));
    return { name, channels };
  });
}
function accessorToJsArray(accessor) {
  if (!accessor._animation) {
    const { typedArray: array, components } = accessorToTypedArray(accessor);
    if (components === 1) {
      accessor._animation = Array.from(array);
    } else {
      const slicedArray = [];
      for (let i = 0; i < array.length; i += components) {
        slicedArray.push(Array.from(array.slice(i, i + components)));
      }
      accessor._animation = slicedArray;
    }
  }
  return accessor._animation;
}

// dist/utils/deep-copy.js
function deepCopy(object) {
  if (ArrayBuffer.isView(object) || object instanceof ArrayBuffer || object instanceof ImageBitmap) {
    return object;
  }
  if (Array.isArray(object)) {
    return object.map(deepCopy);
  }
  if (object && typeof object === "object") {
    const result = {};
    for (const key in object) {
      result[key] = deepCopy(object[key]);
    }
    return result;
  }
  return object;
}

// dist/gltf/create-scenegraph-from-gltf.js
function createScenegraphsFromGLTF(device, gltf, options) {
  gltf = deepCopy(gltf);
  const scenes = parseGLTF(device, gltf, options);
  const animations = parseGLTFAnimations(gltf);
  const animator = new GLTFAnimator({ animations });
  return { scenes, animator };
}
//# sourceMappingURL=index.cjs.map
