#!/usr/bin/env node
/**
 * Automated visualization preparation script.
 * Integrates with ai_api.py's PathIndex system and supports multiple visualization modes.
 * 
 * Usage examples:
 * 
 * 1) Visualize entire room (shell + all clusters):
 *    node scripts/prepare_visualization.mjs \
 *      --mode room \
 *      --room "0-7" \
 *      --name room_007
 * 
 * 2) Visualize selected clusters by object codes:
 *    node scripts/prepare_visualization.mjs \
 *      --mode clusters \
 *      --objects "0-7-12,0-7-15,0-7-3" \
 *      --name selected_objects
 * 
 * 3) Visualize specific cluster files:
 *    node scripts/prepare_visualization.mjs \
 *      --mode clusters \
 *      --clusters "path/to/0-7-12_couch_cluster.ply" \
 *      --clusters "path/to/0-7-15_door_cluster.ply" \
 *      --name custom_selection
 * 
 * 4) Visualize multiple rooms (shells only):
 *    node scripts/prepare_visualization.mjs \
 *      --mode multi-rooms \
 *      --rooms "0-7,0-8,1-7" \
 *      --name floor_overview
 * 
 * 5) Visualize N random clusters from output tree:
 *    node scripts/prepare_visualization.mjs \
 *      --mode random \
 *      --count 5 \
 *      --source "../../output/Full House" \
 *      --name random_5_clusters
 * 
 * 6) Room with selected objects (shell + specific clusters):
 *    node scripts/prepare_visualization.mjs \
 *      --mode room-with-objects \
 *      --room "0-7" \
 *      --objects "0-7-12,0-7-15" \
 *      --name room_007_selected
 * 
 * Options:
 *   --clean          : Clean previous outputs before generating (default: true)
 *   --ratio <float>  : Downsample ratio for clusters (default: 0.2)
 *   --ratioShell <f> : Downsample ratio for shell (default: 0.05)
 *   --voxel <float>  : Voxel size for spatial sampling (optional)
 *   --shellNoColor   : Strip color from shell (gray rendering)
 *   --outDir <path>  : Output directory (default: public/data)
 */

import fs from 'fs';
import fsp from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';
import { execSync, spawn } from 'child_process';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// ==================== Argument parsing ====================

function parseArgs(argv) {
  const args = {};
  const arrays = ['clusters', 'rooms', 'objects'];
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    if (a.startsWith('--')) {
      const key = a.slice(2);
      let val = true;
      if (argv[i + 1] && !argv[i + 1].startsWith('--')) {
        val = argv[++i];
      }
      // Handle array arguments
      if (arrays.includes(key)) {
        args[key] = args[key] || [];
        args[key].push(val);
      } else {
        args[key] = val;
      }
    }
  }
  return args;
}

// ==================== Utility functions ====================

function ensureDirSync(dir) {
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }
}

// ==================== AI API integration ====================

function getRepoRoot() {
  // Walk up from __dirname to find CMakeLists.txt
  let current = path.resolve(__dirname, '..');
  for (let i = 0; i < 10; i++) {
    if (fs.existsSync(path.join(current, 'CMakeLists.txt'))) {
      return current;
    }
    const parent = path.dirname(current);
    if (parent === current) break;
    current = parent;
  }
  throw new Error('Cannot find repository root (CMakeLists.txt not found)');
}

function callAiApi(args) {
  const root = getRepoRoot();
  const apiScript = path.join(root, 'scripts', 'ai_api.py');
  if (!fs.existsSync(apiScript)) {
    throw new Error(`ai_api.py not found at ${apiScript}`);
  }
  const cmd = ['python3', apiScript, ...args];
  try {
    const output = execSync(cmd.join(' '), { encoding: 'utf-8', cwd: root });
    return JSON.parse(output);
  } catch (e) {
    console.error(`[ai_api] Error calling: ${cmd.join(' ')}`);
    console.error(e.stderr || e.message);
    throw e;
  }
}

function resolveObjectAssets(objectCode) {
  return callAiApi(['resolve-object', objectCode]);
}

function resolveRoomAssets(roomCode) {
  return callAiApi(['resolve-room', roomCode]);
}

// ==================== Cleanup ====================

async function cleanPreviousOutputs(name, outRoot) {
  const dataDir = path.join(outRoot, name);
  const manifestDir = path.join(path.dirname(outRoot), 'manifests');
  const manifestPath = path.join(manifestDir, `${name}.json`);
  
  console.log(`[clean] Removing previous outputs for '${name}'...`);
  try {
    await fsp.rm(dataDir, { recursive: true, force: true });
    console.log(`[clean] Removed data directory: ${dataDir}`);
  } catch (e) {
    // ignore
  }
  try {
    await fsp.rm(manifestPath, { force: true });
    console.log(`[clean] Removed manifest: ${manifestPath}`);
  } catch (e) {
    // ignore
  }
}

async function cleanAllOutputs(outRoot) {
  const manifestDir = path.join(path.dirname(outRoot), 'manifests');
  
  console.log('[clean-all] Removing ALL previous outputs...');
  
  // Clean data directory (keep examples)
  try {
    const dataPath = path.resolve(outRoot);
    if (fs.existsSync(dataPath)) {
      const entries = await fsp.readdir(dataPath);
      for (const entry of entries) {
        const fullPath = path.join(dataPath, entry);
        const stat = await fsp.stat(fullPath);
        if (stat.isDirectory()) {
          await fsp.rm(fullPath, { recursive: true, force: true });
          console.log(`[clean-all] Removed: ${fullPath}`);
        }
      }
    }
  } catch (e) {
    console.warn(`[clean-all] Warning: ${e.message}`);
  }
  
  // Clean manifests (keep examples folder)
  try {
    if (fs.existsSync(manifestDir)) {
      const entries = await fsp.readdir(manifestDir);
      for (const entry of entries) {
        if (entry === 'examples') continue; // Keep examples
        const fullPath = path.join(manifestDir, entry);
        const stat = await fsp.stat(fullPath);
        if (stat.isFile() && entry.endsWith('.json')) {
          await fsp.rm(fullPath, { force: true });
          console.log(`[clean-all] Removed: ${fullPath}`);
        }
      }
    }
  } catch (e) {
    console.warn(`[clean-all] Warning: ${e.message}`);
  }
  
  console.log('[clean-all] Cleanup complete!');
}

// ==================== Server ====================

async function startDevServer(port = 5173, manifestName) {
  console.log(`\n[serve] Stopping any existing servers...`);
  
  // Stop existing servers using stop_dev.sh
  const stopScript = path.join(__dirname, '../stop_dev.sh');
  try {
    execSync(`bash ${stopScript}`, { stdio: 'inherit' });
    console.log(`[serve] Previous servers stopped`);
  } catch (e) {
    console.log(`[serve] No previous servers to stop (or stop failed)`);
  }
  
  // Wait a moment for ports to be released
  await new Promise(resolve => setTimeout(resolve, 1000));
  
  console.log(`\n[serve] Starting both frontend and backend servers...`);
  console.log(`[serve] Frontend: http://localhost:${port}/`);
  console.log(`[serve] Backend API: http://localhost:8090/`);
  
  // Start API server first
  // __dirname is .../web/pointcloud-viewer/scripts
  // Go up 3 levels: scripts -> pointcloud-viewer -> web -> project_root
  const projectRoot = path.resolve(__dirname, '../../..');
  const apiServerPath = path.join(projectRoot, 'scripts/api_server.py');
  
  console.log(`[serve] Project root: ${projectRoot}`);
  console.log(`[serve] API server path: ${apiServerPath}`);
  
  const apiServer = spawn('python3', [
    apiServerPath,
    '--port', '8090'
  ], {
    stdio: 'pipe',
    cwd: projectRoot
  });
  
  apiServer.stdout.on('data', (data) => {
    const lines = data.toString().split('\n').filter(line => line.trim());
    lines.forEach(line => console.log(`[API] ${line}`));
  });
  
  apiServer.stderr.on('data', (data) => {
    const lines = data.toString().split('\n').filter(line => line.trim());
    lines.forEach(line => console.error(`[API ERROR] ${line}`));
  });
  
  // Wait for API server to start
  await new Promise(resolve => setTimeout(resolve, 2000));
  
  // Start Vite dev server
  const viteCmd = spawn('npm', ['run', 'dev', '--', '--port', String(port)], {
    stdio: 'inherit',
    cwd: path.join(__dirname, '..'),
  });
  
  // Wait a bit for server to start
  await new Promise(resolve => setTimeout(resolve, 2000));
  
  const url = `http://localhost:${port}/?manifest=/manifests/${manifestName}.json`;
  console.log(`\n✅ Both servers started!`);
  console.log(`🌐 Open in browser: ${url}\n`);
  console.log(`Press Ctrl+C to stop both servers.\n`);
  
  // Handle Ctrl+C to stop both servers
  process.on('SIGINT', () => {
    console.log('\n[shutdown] Stopping servers...');
    apiServer.kill();
    viteCmd.kill();
    process.exit(0);
  });
  
  // Keep the process running
  return new Promise((resolve, reject) => {
    viteCmd.on('close', (code) => {
      apiServer.kill();
      if (code !== 0) {
        reject(new Error(`Dev server exited with code ${code}`));
      } else {
        resolve();
      }
    });
    viteCmd.on('error', reject);
    
    apiServer.on('close', (code) => {
      if (code !== 0) {
        console.error(`[API] Server exited with code ${code}`);
      }
    });
  });
}

// ==================== Downsampling wrapper ====================

async function downsampleAndPrepare(options) {
  const {
    name,
    roomCode = null,  // Optional: actual room code for display names
    shell = null,
    shellCopy = null,
    clusters = [],
    clustersDir = null,
    ratio = 0.2,
    ratioShell = 0.05,
    voxel = null,
    shellNoColor = false,
    outDir = 'public/data',
    acceptAnyPly = false,
  } = options;

  const downsampleScript = path.join(__dirname, 'downsample_and_prepare_room.mjs');
  const args = [
    'node',
    downsampleScript,
    '--room', name,
    '--outDir', outDir,
  ];
  
  // Pass room code for proper display names if provided
  if (roomCode) {
    args.push('--roomCode', roomCode);
  }

  if (shell) {
    args.push('--shell', shell);
    if (ratioShell) args.push('--ratioShell', String(ratioShell));
  } else if (!shellCopy && !clustersDir && clusters.length === 0) {
    // No shell and no clusters/clustersDir - error
    throw new Error('Either shell, shellCopy, clusters, or clustersDir must be provided');
  } else if (!shell && !shellCopy) {
    // Provide a dummy shellCopy to satisfy the script requirement
    args.push('--shellCopy', '/dev/null');
  }
  if (shellCopy && shellCopy !== '/dev/null') {
    args.push('--shellCopy', shellCopy);
  }
  if (shellNoColor) {
    args.push('--shellNoColor');
  }
  if (clustersDir) {
    args.push('--clustersDir', clustersDir);
  }
  for (const c of clusters) {
    args.push('--cluster', c);
  }
  if (acceptAnyPly) {
    args.push('--acceptAnyPly');
  }
  if (ratio) {
    args.push('--ratio', String(ratio));
  }
  if (voxel) {
    args.push('--voxel', String(voxel));
  }

  console.log(`[downsample] Running: ${args.join(' ')}`);
  return new Promise((resolve, reject) => {
    const proc = spawn(args[0], args.slice(1), { stdio: 'inherit' });
    proc.on('close', (code) => {
      if (code === 0) {
        resolve();
      } else {
        reject(new Error(`Downsample script exited with code ${code}`));
      }
    });
    proc.on('error', reject);
  });
}

// ==================== Mode handlers ====================

async function handleRoomMode(args, config) {
  const roomCode = args.room;
  if (!roomCode) {
    throw new Error('--room <floor>-<room> required for room mode');
  }
  
  console.log(`[mode:room] Resolving room ${roomCode}...`);
  const roomData = resolveRoomAssets(roomCode);
  
  const shells = roomData.shell || [];
  const shell = shells.length > 0 ? shells[0] : null;
  
  // Find clusters directory from room structure
  // Shell path: .../room_007/results/shell/shell_007/0-7-0_shell.ply
  // Clusters:   .../room_007/results/filtered_clusters/
  let clustersDir = null;
  if (shell) {
    const shellDir = path.dirname(shell); // .../shell_007
    const shellParent = path.dirname(shellDir); // .../shell
    const resultsDir = path.dirname(shellParent); // .../results
    clustersDir = path.join(resultsDir, 'filtered_clusters');
    if (!fs.existsSync(clustersDir)) {
      console.warn(`[mode:room] Clusters directory not found: ${clustersDir}`);
      clustersDir = null;
    }
  }
  
  await downsampleAndPrepare({
    name: config.name,
    roomCode,  // Pass room code for proper display names
    shell,
    clustersDir,
    ratio: config.ratio,
    ratioShell: config.ratioShell,
    voxel: config.voxel,
    shellNoColor: config.shellNoColor,
    outDir: config.outDir,
  });
}

async function handleClustersMode(args, config) {
  let clusterFiles = args.clusters || [];
  
  // If object codes provided, resolve them
  if (args.objects) {
    const objectCodes = Array.isArray(args.objects)
      ? args.objects.flatMap(s => s.split(',').map(x => x.trim()))
      : args.objects.split(',').map(x => x.trim());
    
    console.log(`[mode:clusters] Resolving ${objectCodes.length} object codes...`);
    for (const oc of objectCodes) {
      try {
        const assets = resolveObjectAssets(oc);
        if (assets.clusters && assets.clusters.length > 0) {
          clusterFiles.push(assets.clusters[0]);
        } else {
          console.warn(`[mode:clusters] No cluster found for object ${oc}`);
        }
      } catch (e) {
        console.warn(`[mode:clusters] Error resolving ${oc}:`, e.message);
      }
    }
  }
  
  if (clusterFiles.length === 0) {
    throw new Error('No cluster files found. Provide --clusters or --objects');
  }
  
  console.log(`[mode:clusters] Processing ${clusterFiles.length} clusters`);
  await downsampleAndPrepare({
    name: config.name,
    clusters: clusterFiles,
    ratio: config.ratio,
    voxel: config.voxel,
    outDir: config.outDir,
  });
}

async function handleMultiRoomsMode(args, config) {
  const roomCodes = Array.isArray(args.rooms)
    ? args.rooms.flatMap(s => s.split(',').map(x => x.trim()))
    : (args.rooms || '').split(',').map(x => x.trim());
  
  if (roomCodes.length === 0) {
    throw new Error('--rooms required for multi-rooms mode (comma-separated or multiple flags)');
  }
  
  console.log(`[mode:multi-rooms] Processing ${roomCodes.length} rooms...`);
  
  // Collect all shells
  const allShells = [];
  for (const rc of roomCodes) {
    try {
      const roomData = resolveRoomAssets(rc);
      if (roomData.shell && roomData.shell.length > 0) {
        allShells.push(roomData.shell[0]);
      }
    } catch (e) {
      console.warn(`[mode:multi-rooms] Error resolving room ${rc}:`, e.message);
    }
  }
  
  if (allShells.length === 0) {
    throw new Error('No shells found for the specified rooms');
  }
  
  // Process each shell as a separate "cluster" (shells are point clouds too)
  // Note: In multi-rooms mode, shells are passed as clusters, so we need to use ratioShell
  // to control their downsample ratio (not ratio, which is for regular clusters)
  // UOBB will be computed automatically for each shell by downsample_and_prepare_room.mjs
  await downsampleAndPrepare({
    name: config.name,
    clusters: allShells,
    ratio: config.ratioShell, // Use ratioShell for shell downsampling in multi-rooms mode
    voxel: config.voxel,
    shellNoColor: config.shellNoColor,
    outDir: config.outDir,
    acceptAnyPly: true, // Allow non-cluster PLY files
  });
  
  // Update manifest to mark these as shells
  await updateManifestRoles(config.name, allShells, 'shell');
}

async function handleRoomWithObjectsMode(args, config) {
  const roomCode = args.room;
  if (!roomCode) {
    throw new Error('--room required for room-with-objects mode');
  }
  
  const objectCodes = Array.isArray(args.objects)
    ? args.objects.flatMap(s => s.split(',').map(x => x.trim()))
    : (args.objects || '').split(',').map(x => x.trim());
  
  if (objectCodes.length === 0) {
    throw new Error('--objects required for room-with-objects mode');
  }
  
  console.log(`[mode:room-with-objects] Resolving room ${roomCode} and ${objectCodes.length} objects...`);
  
  // Get room shell
  const roomData = resolveRoomAssets(roomCode);
  const shell = roomData.shell && roomData.shell.length > 0 ? roomData.shell[0] : null;
  
  // Get object clusters
  const clusterFiles = [];
  for (const oc of objectCodes) {
    try {
      const assets = resolveObjectAssets(oc);
      if (assets.clusters && assets.clusters.length > 0) {
        clusterFiles.push(assets.clusters[0]);
      }
    } catch (e) {
      console.warn(`[mode:room-with-objects] Error resolving ${oc}:`, e.message);
    }
  }
  
  await downsampleAndPrepare({
    name: config.name,
    shell,
    clusters: clusterFiles,
    ratio: config.ratio,
    ratioShell: config.ratioShell,
    voxel: config.voxel,
    shellNoColor: config.shellNoColor,
    outDir: config.outDir,
  });
}

// ==================== Manifest post-processing ====================

async function updateManifestRoles(name, filePaths, role) {
  const manifestDir = path.join(__dirname, '..', 'public', 'manifests');
  const manifestPath = path.join(manifestDir, `${name}.json`);
  
  if (!fs.existsSync(manifestPath)) {
    console.warn(`[manifest] File not found: ${manifestPath}`);
    return;
  }
  
  const manifest = JSON.parse(await fsp.readFile(manifestPath, 'utf-8'));
  const fileNames = new Set(filePaths.map(p => path.basename(p)));
  
  for (const item of manifest.items) {
    const itemFileName = path.basename(item.source.url);
    if (fileNames.has(itemFileName)) {
      item.role = role;
    }
  }
  
  await fsp.writeFile(manifestPath, JSON.stringify(manifest, null, 2));
  console.log(`[manifest] Updated roles to '${role}' in ${manifestPath}`);
}

// ==================== Main ====================

async function main() {
  const args = parseArgs(process.argv);
  
  if (args.help || !args.mode || !args.name) {
    console.log(`
Usage: node scripts/prepare_visualization.mjs --mode <mode> --name <output_name> [options]

Modes:
  room              : Visualize entire room (shell + all clusters)
                      Required: --room <floor>-<room>
  
  clusters          : Visualize selected clusters
                      Required: --clusters <path> OR --objects <code,code,...>
  
  multi-rooms       : Visualize multiple room shells
                      Required: --rooms <code,code,...>
  
  room-with-objects : Room shell with selected objects
                      Required: --room <code>, --objects <code,code,...>

Common options:
  --name <string>       : Output name (used for data folder and manifest)
  --ratio <float>       : Cluster downsample ratio (default: 0.2)
  --ratioShell <float>  : Shell downsample ratio (default: 0.05)
  --voxel <float>       : Voxel size for spatial sampling (optional)
  --shellNoColor        : Strip color from shell (gray rendering)
  --outDir <path>       : Output directory (default: public/data)
  --no-clean            : Skip cleaning previous outputs
  --clean-all           : Clean ALL previous outputs (not just current name)
  --serve               : Automatically start dev server after preparation
  --port <number>       : Dev server port (default: 5173, only with --serve)

Examples:
  # Visualize entire room
  node scripts/prepare_visualization.mjs --mode room --room 0-7 --name room_007
  
  # Selected objects
  node scripts/prepare_visualization.mjs --mode clusters --objects "0-7-12,0-7-15" --name furniture
  
  # Multiple rooms layout
  node scripts/prepare_visualization.mjs --mode multi-rooms --rooms "0-1,0-2,0-3,0-4" --name floor_0
  
  # Clean all and visualize with auto-serve
  node scripts/prepare_visualization.mjs --mode room --room 0-7 --name room_007 --clean-all --serve
`);
    process.exit(args.help ? 0 : 1);
  }
  
  const config = {
    name: args.name,
    ratio: parseFloat(args.ratio) || 0.2,
    ratioShell: parseFloat(args.ratioShell) || 0.05,
    voxel: args.voxel ? parseFloat(args.voxel) : null,
    shellNoColor: Boolean(args.shellNoColor),
    outDir: args.outDir || path.join(__dirname, '..', 'public', 'data'),
    clean: args.clean !== 'false' && args['no-clean'] !== true,
    cleanAll: Boolean(args['clean-all']),
    serve: Boolean(args.serve),
    port: parseInt(args.port) || 5173,
  };
  
  console.log(`[config] Mode: ${args.mode}, Name: ${config.name}`);
  console.log(`[config] Ratio: ${config.ratio}, RatioShell: ${config.ratioShell}, Voxel: ${config.voxel || 'none'}`);
  
  // Clean all outputs if requested
  if (config.cleanAll) {
    await cleanAllOutputs(config.outDir);
  }
  
  // Clean previous outputs for this specific name
  if (config.clean && !config.cleanAll) {
    await cleanPreviousOutputs(config.name, config.outDir);
  }
  
  // Dispatch to mode handler
  switch (args.mode) {
    case 'room':
      await handleRoomMode(args, config);
      break;
    case 'clusters':
      await handleClustersMode(args, config);
      break;
    case 'multi-rooms':
      await handleMultiRoomsMode(args, config);
      break;
    case 'room-with-objects':
      await handleRoomWithObjectsMode(args, config);
      break;
    default:
      throw new Error(`Unknown mode: ${args.mode}`);
  }
  
  console.log('\n✅ Visualization preparation complete!');
  console.log(`\n🌐 View in browser:`);
  console.log(`   http://localhost:${config.port}/?manifest=/manifests/${config.name}.json`);
  console.log(`\n📁 Data location: ${path.join(config.outDir, config.name)}`);
  console.log(`📄 Manifest: public/manifests/${config.name}.json\n`);
  
  // Start dev server if requested
  if (config.serve) {
    await startDevServer(config.port, config.name);
  }
}

main().catch(e => {
  console.error('\n❌ Error:', e.message);
  if (e.stack) {
    console.error(e.stack);
  }
  process.exit(1);
});
