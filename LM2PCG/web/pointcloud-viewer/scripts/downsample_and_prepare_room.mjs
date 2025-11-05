#!/usr/bin/env node
// Simple PLY downsampler and manifest generator for the pointcloud viewer.
// Usage (examples):
//   node scripts/downsample_and_prepare_room.mjs \
//     --room room_007 \
//     --shell ../../output/full_house/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply \
//     --clustersDir ../../output/full_house/floor_0/room_007/results/filtered_clusters \
//     --ratio 0.2 \
//     --outDir public/data \
//     [--shellNoColor] [--clean]
// This will downsample all clusters to ratio 0.2, copy the shell as-is, and write manifest to public/manifests/room_007.json

import fs from 'fs';
import fsp from 'fs/promises';
import path from 'path';
import { fileURLToPath, pathToFileURL } from 'url';
import { load, parse } from '@loaders.gl/core';
import { PLYLoader } from '@loaders.gl/ply';
import { computeUOBB, writeUOBBPly } from './uobb_compute.mjs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

function parseArgs(argv) {
  const args = {};
  const arrayKeys = ['cluster']; // Keys that should accumulate multiple values
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    if (a.startsWith('--')) {
      const key = a.slice(2);
      const val = argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[++i] : true;
      // Handle array arguments
      if (arrayKeys.includes(key)) {
        if (!args[key]) args[key] = [];
        if (Array.isArray(args[key])) args[key].push(val);
        else args[key] = [args[key], val];
      } else {
        args[key] = val;
      }
    }
  }
  return args;
}

function ensureDirSync(dir) {
  if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
}

async function* walkDir(dir) {
  const entries = await fsp.readdir(dir, { withFileTypes: true });
  for (const e of entries) {
    const full = path.join(dir, e.name);
    if (e.isDirectory()) yield* walkDir(full);
    else yield full;
  }
}

function toIntArrayLike(arr) {
  if (!arr) return null;
  if (arr instanceof Int32Array || arr instanceof Uint32Array || arr instanceof Int16Array || arr instanceof Uint16Array || arr instanceof Int8Array || arr instanceof Uint8Array) return arr;
  if (arr instanceof Float32Array || arr instanceof Float64Array) {
    // Cast floats to int by rounding; common in CloudCompare scalar fields
    const out = new Int32Array(arr.length);
    for (let i = 0; i < arr.length; i++) out[i] = Math.round(arr[i]);
    return out;
  }
  if (Array.isArray(arr)) return new Int32Array(arr.map((v) => Math.round(Number(v) || 0)));
  return null;
}

function normalizeAttributes(data) {
  const attr = data.attributes || data;
  let positions = null;
  let colors = null;
  let label = null;
  let point_id = null;
  if (attr.POSITION?.value) positions = attr.POSITION.value;
  else if (attr.positions) positions = attr.positions;
  if (attr.COLOR_0?.value) colors = attr.COLOR_0.value;
  else if (attr.colors) colors = attr.colors;
  const length = positions ? (positions.length / 3) : 0;
  // Try common custom properties
  const directKeys = ['label', 'Label', 'class', 'Class', 'vertex_label'];
  for (const k of directKeys) {
    const v = attr[k]?.value ?? attr[k];
    if (v && v.length === length) { label = toIntArrayLike(v); break; }
  }
  if (!label) {
    // scan scalar_* fields for label/class
    for (const k of Object.keys(attr)) {
      const lk = k.toLowerCase();
      if (lk.includes('label') || lk.includes('class')) {
        const v = attr[k]?.value ?? attr[k];
        if (v && v.length === length) { label = toIntArrayLike(v); break; }
      }
    }
  }
  const pidKeys = ['point_id', 'vertex_id', 'id'];
  for (const k of pidKeys) {
    const v = attr[k]?.value ?? attr[k];
    if (v && v.length === length) { point_id = toIntArrayLike(v); break; }
  }
  return { length, positions, colors, label, point_id };
}

function randomSampleIndices(n, ratio, seed = 12345) {
  const target = Math.max(1, Math.floor(n * ratio));
  // Simple LCG for deterministic sampling
  let s = seed >>> 0;
  function rand() { s = (1664525 * s + 1013904223) >>> 0; return (s & 0xffffffff) / 0x100000000; }
  const idx = new Int32Array(n);
  for (let i = 0; i < n; i++) idx[i] = i;
  // Fisher-Yates shuffle partial
  for (let i = n - 1; i > n - 1 - target; i--) {
    const j = Math.floor(rand() * (i + 1));
    const tmp = idx[i]; idx[i] = idx[j]; idx[j] = tmp;
  }
  return Array.from(idx.slice(n - target));
}

function voxelSampleIndices(positions, voxelSize) {
  const seen = new Map();
  const out = [];
  for (let i = 0, p = 0; i < positions.length; i += 3, p++) {
    const x = Math.floor(positions[i] / voxelSize);
    const y = Math.floor(positions[i + 1] / voxelSize);
    const z = Math.floor(positions[i + 2] / voxelSize);
    const key = x + ',' + y + ',' + z;
    if (!seen.has(key)) { seen.set(key, p); out.push(p); }
  }
  return out;
}

function gatherByIndices(positions, colors, indices, extra = {}) {
  const outPos = new Float32Array(indices.length * 3);
  let outCol = null;
  if (colors) {
    const ctor = colors.constructor; // likely Uint8Array
    outCol = new ctor(indices.length * 3);
  }
  // prepare extra attribute outputs (e.g., label, point_id)
  const outExtra = {};
  for (const [k, arr] of Object.entries(extra)) {
    if (!arr) continue;
    const ctor = arr.constructor;
    outExtra[k] = new ctor(indices.length);
  }
  for (let i = 0; i < indices.length; i++) {
    const pi = indices[i] * 3;
    const po = i * 3;
    outPos[po] = positions[pi];
    outPos[po + 1] = positions[pi + 1];
    outPos[po + 2] = positions[pi + 2];
    if (outCol && colors) {
      outCol[po] = colors[pi];
      outCol[po + 1] = colors[pi + 1];
      outCol[po + 2] = colors[pi + 2];
    }
    for (const [k, arr] of Object.entries(extra)) {
      if (!arr) continue;
      outExtra[k][i] = arr[indices[i]];
    }
  }
  return { positions: outPos, colors: outCol, ...outExtra };
}

function plyTypeOfArray(arr) {
  if (!arr) return null;
  if (arr instanceof Int8Array) return 'char';
  if (arr instanceof Uint8Array) return 'uchar';
  if (arr instanceof Int16Array) return 'short';
  if (arr instanceof Uint16Array) return 'ushort';
  if (arr instanceof Int32Array) return 'int';
  if (arr instanceof Uint32Array) return 'uint';
  if (arr instanceof Float32Array) return 'float';
  if (arr instanceof Float64Array) return 'double';
  return 'int';
}

function writeAsciiPLY(filePath, positions, colors, { label = null, point_id = null } = {}) {
  const n = positions.length / 3;
  const lines = [];
  lines.push('ply');
  lines.push('format ascii 1.0');
  lines.push('comment generated by downsample_and_prepare_room.mjs');
  lines.push(`element vertex ${n}`);
  lines.push('property float x');
  lines.push('property float y');
  lines.push('property float z');
  const hasColor = Boolean(colors);
  if (hasColor) {
    lines.push('property uchar red');
    lines.push('property uchar green');
    lines.push('property uchar blue');
  }
  const hasLabel = Boolean(label);
  const hasPointId = Boolean(point_id);
  if (hasLabel) lines.push(`property ${plyTypeOfArray(label)} label`);
  if (hasPointId) lines.push(`property ${plyTypeOfArray(point_id)} point_id`);
  lines.push('end_header');
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i].toFixed(6);
    const y = positions[i + 1].toFixed(6);
    const z = positions[i + 2].toFixed(6);
    const row = [x, y, z];
    if (hasColor && colors) {
      row.push((colors[i] | 0).toString(), (colors[i + 1] | 0).toString(), (colors[i + 2] | 0).toString());
    }
    if (hasLabel) row.push((label[i / 3] | 0).toString());
    if (hasPointId) row.push((point_id[i / 3] | 0).toString());
    lines.push(row.join(' '));
  }
  fs.writeFileSync(filePath, lines.join('\n'));
}

async function downsamplePly(inputPath, outputPath, { ratio, voxel, debug = false, stripColor = false }) {
  const isRemote = /^(https?:)?\/\//.test(inputPath);
  let data;
  if (isRemote) {
    data = await load(inputPath, PLYLoader, { worker: false });
  } else {
    const buf = await fsp.readFile(inputPath);
    data = await parse(buf, PLYLoader, { worker: false });
  }
  if (debug) {
    const attr = data.attributes || data;
    const keys = Object.keys(attr);
    const lengths = Object.fromEntries(keys.map((k) => [k, (attr[k]?.value?.length ?? attr[k]?.length ?? '-') ]));
    console.log('[debug] attr keys:', keys);
    console.log('[debug] attr lengths:', lengths);
  }
  const { length, positions, colors, label, point_id } = normalizeAttributes(data);
  if (!positions || length === 0) throw new Error(`No positions in ${inputPath}`);
  let indices = null;
  if (voxel && Number(voxel) > 0) {
    indices = voxelSampleIndices(positions, Number(voxel));
  }
  if (ratio && Number(ratio) > 0 && Number(ratio) < 1) {
    const r = Number(ratio);
    const base = indices ? indices.length : length;
    const target = Math.max(1, Math.floor(base * r));
    if (!indices) {
      indices = randomSampleIndices(length, r);
    } else if (indices.length > target) {
      // re-sample the voxel-picked set to target proportion
      const picked = randomSampleIndices(indices.length, r);
      indices = picked.map(i => indices[i]);
    }
  }
  if (!indices) {
    // no downsampling requested -> just write all attributes (optionally stripping color)
    writeAsciiPLY(outputPath, positions, stripColor ? null : (colors || null), { label: label || null, point_id: point_id || null });
    return { in: length, out: length, positions };
  }
  const gathered = gatherByIndices(positions, colors || null, indices, { label: label || null, point_id: point_id || null });
  writeAsciiPLY(outputPath, gathered.positions, stripColor ? null : (gathered.colors || null), { label: gathered.label || null, point_id: gathered.point_id || null });
  return { in: length, out: indices.length, positions: gathered.positions };
}

async function main() {
  const args = parseArgs(process.argv);
  if (args.help || (!args.shell && !args.shellCopy) || !args.room || (!args.clustersDir && !args.cluster)) {
    console.log(`Usage:
  node scripts/downsample_and_prepare_room.mjs \
    --room room_007 \
    --shell <path/to/shell.ply> \
    --clustersDir <path/to/filtered_clusters> \
    [--ratio 0.2] [--voxel 0.01] \
    [--outDir public/data] [--shellNoColor] [--clean]
Notes:
  - If --ratio not provided, clusters will be voxel-only (or copied if no voxel).
  - Shell will be copied as-is unless you point --ratioShell.
  - --clean removes previous outputs for this room under outDir before generating.
    `);
    process.exit(0);
  }
  const room = String(args.room);
  // roomCode is the actual room identifier for display (e.g., "0-7"), defaults to room if not provided
  const roomCode = args.roomCode ? String(args.roomCode) : room;
  const outRoot = args.outDir ? String(args.outDir) : 'public/data';
  // Handle both absolute and relative paths for outDir
  const outDir = path.isAbsolute(outRoot) 
    ? path.join(outRoot, room)
    : path.resolve(path.join(__dirname, '..', outRoot, room));
  const outClustersDir = path.join(outDir, 'clusters');
  // optional clean of previous outputs to avoid stale files
  if (args.clean) {
    try {
      await fsp.rm(outDir, { recursive: true, force: true });
    } catch {}
    const manifestDir = path.resolve(path.join(__dirname, '..', 'public', 'manifests'));
    const manifestPath = path.join(manifestDir, `${room}.json`);
    try { await fsp.rm(manifestPath, { force: true }); } catch {}
  }
  ensureDirSync(outClustersDir);

  const manifest = {
    version: 1,
    title: `Room ${room}`,
    defaults: { pointSize: 3 },
    items: []
  };

  // Handle shell
  if (args.shell) {
    const shellSrc = path.resolve(String(args.shell));
    const shellDst = path.join(outDir, 'shell.ply');
    // Use shell-specific params if provided; otherwise fall back to global ratio/voxel
    const ratioShell = args.ratioShell ? Number(args.ratioShell) : (args.ratio ? Number(args.ratio) : undefined);
    const voxelShell = args.voxelShell ? Number(args.voxelShell) : (args.voxel ? Number(args.voxel) : undefined);
    const stripShellColor = Boolean(args.shellNoColor);
    
    let shellPositions = null; // Store positions for UOBB computation
    
    if (ratioShell || voxelShell || stripShellColor) {
      console.log(`[shell] downsampling ${shellSrc} -> ${shellDst} (ratio=${ratioShell ?? '-'}, voxel=${voxelShell ?? '-'})`);
      const result = await downsamplePly(shellSrc, shellDst, { ratio: ratioShell, voxel: voxelShell, debug: Boolean(args.debug), stripColor: stripShellColor });
      console.log(`[shell] ${result.in} -> ${result.out}`);
      shellPositions = result.positions; // Get positions for UOBB
    } else {
      console.log(`[shell] copying ${shellSrc} -> ${shellDst}`);
      await fsp.copyFile(shellSrc, shellDst);
      // Load positions for UOBB computation
      const buf = await fsp.readFile(shellSrc);
      const data = await parse(buf, PLYLoader, { worker: false });
      const { positions } = normalizeAttributes(data);
      shellPositions = positions;
    }
    
    manifest.items.push({
      id: `shell-${room}`,
      name: `room_shell (room_id: ${roomCode})`,
      kind: 'pointcloud',
      role: 'shell',
      source: { url: `/data/${room}/shell.ply` },
      group: room,
      visible: true,
      style: stripShellColor ? { colorMode: 'constant', color: [180, 180, 180], pointSize: 3 } : { colorMode: 'file', pointSize: 3 },
      filters: args.shellHide ? { labelExclude: String(args.shellHide).split(',').map((s) => Number(s.trim())).filter((n) => !Number.isNaN(n)) } : undefined
    });

    // Compute UOBB from downsampled/copied shell positions
    if (shellPositions) {
      try {
        console.log(`[uobb] computing UOBB from shell positions...`);
        const uobb = computeUOBB(shellPositions);
        if (uobb) {
          const uobbBuffer = writeUOBBPly(uobb);
          const uobbDst = path.join(outDir, 'shell_uobb.ply');
          await fsp.writeFile(uobbDst, uobbBuffer);
          console.log(`[uobb] wrote computed UOBB to ${uobbDst}`);
          manifest.items.push({
            id: `uobb-${room}`,
            name: `room_uobb (room_id: ${roomCode})`,
            kind: 'mesh',
            role: 'uobb',
            source: { url: `/data/${room}/shell_uobb.ply` },
            group: `${room}_uobbs`,
            visible: true,
            style: { color: [30, 144, 255], opacity: 0.3 }
          });
        }
      } catch (e) {
        console.warn(`[uobb] Failed to compute UOBB: ${e.message}`);
      }
    }
  }

  // Handle clusters
  const ratio = args.ratio ? Number(args.ratio) : undefined;
  const voxel = args.voxel ? Number(args.voxel) : undefined;
  const acceptAnyPly = Boolean(args.acceptAnyPly);
  const clusters = [];
  if (args.clustersDir) {
    const dir = path.resolve(String(args.clustersDir));
    for await (const file of walkDir(dir)) {
      const lower = file.toLowerCase();
      if (acceptAnyPly ? lower.endsWith('.ply') : /_cluster\.ply$/i.test(lower)) {
        clusters.push(file);
      }
    }
  }
  if (args.cluster) {
    const files = Array.isArray(args.cluster) ? args.cluster : [args.cluster];
    for (const f of files) {
      const full = path.resolve(String(f));
      if (acceptAnyPly ? full.toLowerCase().endsWith('.ply') : /_cluster\.ply$/i.test(full.toLowerCase())) {
        clusters.push(full);
      }
    }
  }
  clusters.sort();
  console.log(`[clusters] found ${clusters.length} ply files`);
  for (const c of clusters) {
    const base = path.basename(c).replace(/\.ply$/i, '');
    const name = base; // keep simple; could also include parent folder
    const dst = path.join(outClustersDir, `${name}.ply`);
    console.log(`[cluster] downsampling ${c} -> ${dst} (ratio=${ratio ?? '-'}, voxel=${voxel ?? '-'})`);
    const stats = await downsamplePly(c, dst, { ratio, voxel, debug: Boolean(args.debug) });
    console.log(`[cluster] ${name}: ${stats.in} -> ${stats.out}`);
    
    // Check if this is a shell file (for multi-rooms mode)
    const isShell = /_shell$/i.test(base);
    
    // Parse display name based on file format
    let displayName = name;
    if (isShell) {
      // Shell format: <floor>-<room>-<object>_shell (e.g., "0-7-0_shell")
      const shellMatch = base.match(/^(\d+-\d+)-\d+_shell$/);
      if (shellMatch) {
        const roomId = shellMatch[1];
        displayName = `room_shell (room_id: ${roomId})`;
      }
    } else {
      // Cluster format: <floor>-<room>-<object>_<class>_cluster (e.g., "0-7-12_couch_cluster")
      const clusterMatch = base.match(/^(\d+-\d+-\d+)_([^_]+)_cluster$/);
      if (clusterMatch) {
        const objectId = clusterMatch[1];
        const className = clusterMatch[2];
        displayName = `${className} (object_id: ${objectId})`;
      }
    }
    
    manifest.items.push({
      id: `cluster-${name}`,
      name: displayName,
      kind: 'pointcloud',
      role: isShell ? 'shell' : 'object',
      source: { url: `/data/${room}/clusters/${name}.ply` },
      group: room,
      visible: true,
      style: { colorMode: 'file', pointSize: 3 }
    });
    
    // Compute UOBB for shell files in multi-rooms mode
    if (isShell && stats.positions) {
      try {
        console.log(`[uobb] computing UOBB for ${name}...`);
        const uobb = computeUOBB(stats.positions);
        if (uobb) {
          const uobbBuffer = writeUOBBPly(uobb);
          const uobbDst = path.join(outClustersDir, `${name}_uobb.ply`);
          await fsp.writeFile(uobbDst, uobbBuffer);
          console.log(`[uobb] wrote computed UOBB to ${uobbDst}`);
          
          // Extract room code from shell filename (e.g., "0-7-0_shell" -> "0-7")
          const shellMatch = base.match(/^(\d+-\d+)-\d+_shell$/);
          const roomCode = shellMatch ? shellMatch[1] : name;
          const uobbDisplayName = shellMatch ? `room_uobb (room_id: ${roomCode})` : `${name} uobb`;
          
          manifest.items.push({
            id: `uobb-${roomCode}`,
            name: uobbDisplayName,
            kind: 'mesh',
            role: 'uobb',
            source: { url: `/data/${room}/clusters/${name}_uobb.ply` },
            group: `${room}_uobbs`,
            visible: true,
            style: { color: [30, 144, 255], opacity: 0.3 }
          });
        }
      } catch (e) {
        console.warn(`[uobb] Failed to compute UOBB for ${name}: ${e.message}`);
      }
    }
  }

  // Write manifest
  const manifestDir = path.resolve(path.join(__dirname, '..', 'public', 'manifests'));
  ensureDirSync(manifestDir);
  const manifestPath = path.join(manifestDir, `${room}.json`);
  await fsp.writeFile(manifestPath, JSON.stringify(manifest, null, 2));
  console.log(`[manifest] wrote ${manifestPath}`);

  console.log('\nDone. In the viewer URL, use:');
  console.log(`  ?manifest=/manifests/${room}.json`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
