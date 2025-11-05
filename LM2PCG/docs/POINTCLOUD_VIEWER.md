# Point Cloud Viewer Documentation

A high-performance web-based point cloud visualization system built with deck.gl, React, and loaders.gl. Designed for interactive exploration of large-scale architectural point clouds with semantic segmentation.

## Table of Contents

- [Overview](#overview)
  - [Key Components](#key-components)
- [Features](#features)
  - [1. Multi-Mode Visualization](#1-multi-mode-visualization)
  - [2. Advanced Rendering](#2-advanced-rendering)
  - [3. Inspector UI](#3-inspector-ui)
  - [4. Performance Optimization](#4-performance-optimization)
- [Quick Start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Development Server](#development-server)
  - [Basic Usage](#basic-usage)
- [Interactive Object Selection](#interactive-object-selection)
  - [Server Setup](#server-setup)
  - [Selection Workflow](#selection-workflow)
  - [Object Code Format](#object-code-format)
  - [JSON Output Structure](#json-output-structure)
  - [Integration with AI_API.py](#integration-with-ai_apipy)
  - [API Endpoints](#api-endpoints)
  - [Troubleshooting](#troubleshooting)
- [Automated Workflow](#automated-workflow)
  - [Command Structure](#command-structure)
  - [Mode Examples](#mode-examples)
    - [1. Room Mode (Complete Room)](#1-room-mode-complete-room)
    - [2. Clusters Mode (Selected Objects)](#2-clusters-mode-selected-objects)
    - [3. Multi-Rooms Mode (Floor Overview)](#3-multi-rooms-mode-floor-overview)
    - [4. Room-with-Objects Mode (Contextual Analysis)](#4-room-with-objects-mode-contextual-analysis)
  - [Common Options](#common-options)
  - [Integration with ai_api.py](#integration-with-ai_apipy-1)
- [Architecture](#architecture)
  - [Data Flow](#data-flow)
  - [Manifest Schema (v1)](#manifest-schema-v1)
  - [Component Architecture](#component-architecture)
- [API Reference](#api-reference)
  - [prepare_visualization.mjs](#prepare_visualizationmjs)
  - [downsample_and_prepare_room.mjs](#downsample_and_prepare_roommjs)
- [Performance Optimization](#performance-optimization-1)
  - [Downsampling Guidelines](#downsampling-guidelines)
  - [Voxel vs Ratio Sampling](#voxel-vs-ratio-sampling)
  - [Browser Performance Tips](#browser-performance-tips)

---

## Overview

The Point Cloud Viewer is a complete visualization solution for the LM2PCG pipeline. It provides:

- **Interactive 3D visualization** of room shells and object clusters
- **Semantic inspection** with per-object visibility and styling controls
- **UOBB (Unoriented Bounding Box) rendering** for spatial understanding
- **Automated data preparation** with downsampling and manifest generation
- **Integration with ai_api.py** for semantic path resolution

### Key Components

```
web/pointcloud-viewer/
├── src/                          # React application
│   ├── App.tsx                   # Main viewer component
│   ├── components/
│   │   ├── Inspector.tsx         # UI controls and object list
│   │   ├── PointCloudLayer.tsx   # deck.gl point cloud rendering
│   │   └── UobbMeshLayer.tsx     # UOBB wireframe rendering
│   └── types/
│       └── manifest.ts           # TypeScript manifest schema
├── scripts/
│   ├── prepare_visualization.mjs       # Main automation script
│   └── downsample_and_prepare_room.mjs # Downsampling + manifest gen
└── public/
    ├── data/                     # Downsampled PLY files
    └── manifests/                # JSON visualization configs
```

---

## Features

### 1. Multi-Mode Visualization

| Mode | Description | Use Case |
|------|-------------|----------|
| **room** | Complete room with shell + all clusters | Full room inspection |
| **clusters** | Selected object clusters only | Object-focused analysis |
| **multi-rooms** | Multiple room shells without clusters | Floor layout overview |
| **room-with-objects** | Room shell + selected objects | Contextual object analysis |

### 2. Advanced Rendering

- **Point Cloud Rendering** 
  - Up to 10M+ points with 60 FPS performance
  - RGB color preservation from PLY files
  - Configurable point sizes (1-10 pixels)
  - Label-based semantic filtering

- **UOBB Visualization**
  - Wireframe mesh rendering
  - Automatic detection from `*_uobb.ply` files
  - **Non-pickable** (cannot be selected, clicks pass through to point clouds)
  - Toggle visibility per object
  - Customizable wireframe color
  - **Fixed center calculation** (properly handles rotated bounding boxes)

- **Interactive Controls**
  - Orbit camera with smooth transitions
  - Auto-calculated zoom based on scene bounds
  - Per-object visibility toggles
  - Smart title generation from filename patterns
  - **Object selection and confirmation**
  - **Download source PLY files** via API integration

### 3. Inspector UI

```
Inspector Panel Features:
├── Scene Stats (total points, items)
├── Global Controls
│   ├── Point size slider (1-10)
│   ├── Show/hide all toggle
│   └── UOBB visibility toggle
└── Per-Object Controls
    ├── Visibility checkbox
    ├── Smart title (e.g., "Couch 12" from "0-7-12_couch_cluster")
    ├── Point count display
    ├── UOBB toggle (if available)
    ├── Select/Confirm button (with backend integration)
    └── Download button (with API server)
```

### 4. Performance Optimization

- **Spatial Downsampling** - Voxel-based uniform sampling
- **Ratio Downsampling** - Deterministic random sampling
- **Attribute Preservation** - Maintains label and point_id fields
- **ASCII PLY Output** - Better compression and compatibility
- **Lazy Loading** - On-demand file loading via manifest

---

## Quick Start

### Prerequisites

```bash
# Required
Node.js >= 18
npm >= 9

# For ai_api.py integration
Python 3.7+
```

### Installation

```bash
cd web/pointcloud-viewer
npm install
```

### Development Server

```bash
npm run dev
# Opens http://localhost:5173
```

### Basic Usage

```bash
# Visualize a single room
npm run visualize -- \
  --mode room \
  --room 0-7 \
  --name my_room

# Then open browser to:
# http://localhost:5173/?manifest=/manifests/my_room.json
```

---

## Interactive Object Selection

The viewer supports interactive object selection with backend integration for AI operations and file downloads.

### Server Setup

**Quick Start (Recommended):**
```bash
cd web/pointcloud-viewer
./start_dev.sh
# Frontend: http://localhost:5173/
# API: http://localhost:8090/
```

**Stop Servers:**
```bash
./stop_dev.sh
```

**Or use automation pipeline:**
```bash
npm run visualize -- --mode room --room 0-7 --name room_007 --serve
# Automatically starts both servers
```

### Selection Workflow

1. **Click Object** → Point cloud highlights in yellow-gold
2. **View Info** → Inspector panel shows object_code (e.g., `0-7-12`)
3. **Click "Confirm All"** → Sends selection to backend API (localhost:8090)
4. **Backend Response** → Terminal displays selection JSON in real-time
5. **Auto-Close** → Servers shut down automatically (when using `ai_api.py VIS`)

**Interactive Mode (Default with VIS command):**
```bash
python3 scripts/ai_api.py VIS 0-7
# → Starts servers automatically
# → Opens browser for visualization
# → Waits for user selection (timeout: 5 minutes)
# → Prints selection JSON to terminal
# → Auto-closes servers
```

**Output Example:**
```
Status: success
Mode: room
Name: room_0_7
Viewer URL: http://localhost:5173/?manifest=room_0_7.json
Rooms: 0-7

[User clicks objects and confirms]

[
  {
    "itemCode": "0-7-12",
    "displayName": "couch (object_id: 0-7-12)",
    "type": "object",
    "sourceFile": "output/.../0-7-12_couch_cluster.ply",
    "timestamp": "2025-10-27T10:30:00.000Z"
  }
]
```

**Manual Mode:**
```bash
# Start servers manually (keeps running)
cd web/pointcloud-viewer
./start_dev.sh

# In another terminal, visualize without auto-close
python3 scripts/ai_api.py VIS 0-7 --no-wait

# Stop servers when done
./stop_dev.sh
```

### Object Code Format

- **Object:** `<floor>-<room>-<object>` (e.g., `0-7-12`)
- **Room:** `<floor>-<room>` (e.g., `0-7`)

Extracted from:
- Name pattern: `"couch (object_id: 0-7-12)"`
- File pattern: `"0-7-12_couch_cluster.ply"`

### JSON Output Structure

**Object Selection:**
```json
{
  "timestamp": "2025-10-24T09:00:00.000Z",
  "selection": {
    "code": "0-7-12",
    "type": "object",
    "name": "couch (object_id: 0-7-12)",
    "viewer_url": "/data/room_007/clusters/0-7-12_couch_cluster.ply"
  },
  "source_files": {
    "object_code": "0-7-12",
    "class": "couch",
    "cluster_path": "/path/to/output/.../0-7-12_couch_cluster.ply",
    "uobb_path": "/path/to/output/.../0-7-12_couch_uobb.ply",
    "mesh_path": "/path/to/output/.../0-7-12_couch_mesh.ply",
    "csv_data": { /* Full CSV row */ }
  }
}
```

**Room Selection:**
```json
{
  "timestamp": "2025-10-24T09:00:00.000Z",
  "selection": {
    "code": "0-7",
    "type": "shell",
    "name": "room_shell (room_id: 0-7)"
  },
  "source_files": {
    "room_code": "0-7",
    "shell_path": "/path/to/output/.../0-7-0_shell.ply",
    "shell_uobb_path": "/path/to/output/.../0-7-0_shell_uobb.ply",
    "clusters_dir": "/path/to/output/.../filtered_clusters"
  }
}
```

### Integration with AI_API.py

Use selection results with AI operations:

```python
from scripts.ai_api import Dispatcher
d = Dispatcher()

# Reconstruct mesh for selected object
result = d.op_RCN(object_code='0-7-12')

# Calculate volume
volume = d.op_VOL(object_code='0-7-12')

# Color analysis for selected room
result = d.op_CLR(room_code='0-7')
```

### API Endpoints

Backend API (`http://localhost:8090`):

- `GET /api/resolve-object?code=0-7-12` - Get object source files
- `GET /api/resolve-room?code=0-7` - Get room source files  
- `GET /api/download-file?path=<path>` - Stream file downloads (supports both absolute and relative paths)
- `GET /health` - Health check

**Response Format (Updated to use relative paths):**
```json
{
  "success": true,
  "data": {
    "object_code": "0-7-12",
    "cluster_path": "output/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply",
    "uobb_path": "output/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_uobb.ply",
    "csv_data": { /* Object metadata */ }
  }
}
```

**Key Features:**
- **Relative Paths**: API returns paths relative to project root for better portability
- **Dynamic Resolution**: Download endpoint resolves relative paths at runtime
- **Security**: Paths are validated to ensure they're within project directory

### Troubleshooting

**Buttons Disabled:**
- Ensure manifest uses correct naming: `"class (object_id: X-X-X)"`
- Regenerate manifest if needed

**API Connection Failed:**
```bash
# Check server status
curl http://localhost:8090/health

# Restart server
cd web/pointcloud-viewer
./stop_dev.sh && ./start_dev.sh
```

**Download Failed:**
- Server must be running (`./start_dev.sh`)
- Check `/tmp/api_server.log` for errors
- Verify file exists in output directory
- Ensure API server has been restarted after code changes

**Object Not Found:**
- Verify `/output` directory structure is complete
- Restart API server to reload PathIndex

---

## Automated Workflow

### Command Structure

```bash
npm run visualize -- \
  --mode <mode> \
  --name <output_name> \
  [mode-specific options] \
  [common options]
```

### Mode Examples

#### 1. Room Mode (Complete Room)

```bash
npm run visualize -- \
  --mode room \
  --room 0-7 \
  --name room_007 \
  --ratio 0.2 \
  --ratioShell 0.05 \
  --clean-all \
  --serve
```

**What it does:**
1. Resolves room `0-7` using `ai_api.py resolve-room`
2. Finds shell: `output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply`
3. Finds clusters: `output/Full House/floor_0/room_007/results/filtered_clusters/**/*_cluster.ply`
4. Downsamples shell to 5%, clusters to 20%
5. Copies shell UOBB if available
6. Generates manifest with 1 shell + N clusters
7. Starts dev server on port 5173

**Output:**
- `public/data/room_007/shell.ply` - Downsampled shell
- `public/data/room_007/shell_uobb.ply` - Shell bounding box
- `public/data/room_007/clusters/*.ply` - Downsampled clusters
- `public/manifests/room_007.json` - Unified manifest

#### 2. Clusters Mode (Selected Objects)

```bash
npm run visualize -- \
  --mode clusters \
  --objects "0-7-12,0-7-13,0-7-14" \
  --name sofas_room_007 \
  --ratio 0.25
```

**What it does:**
1. Resolves each object code via `ai_api.py resolve-object`
2. Extracts first cluster from each result
3. Downsamples to 25%
4. Generates manifest with 3 items

**Use case:** Focus on specific furniture pieces (e.g., all sofas in a room)

#### 3. Multi-Rooms Mode (Floor Overview)

```bash
npm run visualize -- \
  --mode multi-rooms \
  --rooms "0-1,0-2,0-3,0-4,0-5,0-6,0-7" \
  --name floor_0_layout \
  --ratio 0.1
```

**What it does:**
1. Resolves 7 room shells
2. Treats shells as "clusters" (no internal objects)
3. Downsamples each shell to 10%
4. Updates manifest roles to 'shell'

**Use case:** See overall floor plan without cluster details

#### 4. Room-with-Objects Mode (Contextual Analysis)

```bash
npm run visualize -- \
  --mode room-with-objects \
  --room 0-7 \
  --objects "0-7-12,0-7-15,0-7-6" \
  --name room_007_selected \
  --ratio 0.2 \
  --ratioShell 0.03
```

**What it does:**
1. Gets room shell (3% sampling)
2. Gets 3 object clusters (20% sampling)
3. Combines into single visualization

**Use case:** Highlight specific objects within room context

### Common Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--name` | string | **required** | Output directory and manifest name |
| `--ratio` | float | 0.2 | Cluster downsample ratio (1.0 = no sampling) |
| `--ratioShell` | float | 0.05 | Shell downsample ratio |
| `--voxel` | float | - | Voxel size for spatial sampling (overrides ratio) |
| `--shellNoColor` | flag | false | Strip color from shell (render as gray) |
| `--outDir` | path | public/data | Output directory for PLY files |
| `--no-clean` | flag | false | Skip cleaning previous outputs |
| `--clean-all` | flag | false | Remove ALL previous outputs before running |
| `--serve` | flag | false | Auto-start dev server after preparation |
| `--port` | number | 5173 | Dev server port (only with --serve) |

### Integration with ai_api.py

The automation scripts seamlessly integrate with the AI API:

```python
# Room resolution
python3 scripts/ai_api.py resolve-room 0-7 --json
# Returns: { "floor": 0, "room": 7, "csv": "...", "shell": ["..."], "shell_uobb": ["..."] }

# Object resolution
python3 scripts/ai_api.py resolve-object 0-7-12 --json
# Returns: { "clusters": ["..."], "uobbs": ["..."], "meshes": ["..."], "room_dir": "..." }
```

**Object/Room Code Format:**
- Room: `<floor>-<room>` (e.g., `0-7`)
- Object: `<floor>-<room>-<object>` (e.g., `0-7-12`)

---

## Architecture

### Data Flow

```
Pipeline Output                   Automation Scripts              Web Viewer
───────────────                   ──────────────────              ──────────
output/
└── Full House/
    └── floor_0/
        └── room_007/
            └── results/
                ├── shell/
                │   └── 0-7-0_shell.ply        ──┐
                │                                 │
                └── filtered_clusters/            │
                    ├── couch_007/                │
                    │   └── 0-7-12_*.ply      ────┤──> prepare_visualization.mjs
                    └── door_007/                 │         │
                        └── 0-7-15_*.ply      ────┘         │
                                                            │
                                                            ▼
                                              downsample_and_prepare_room.mjs
                                                            │
                                                            │
                              ┌─────────────────────────────┴──────────────┐
                              │                                            │
                              ▼                                            ▼
                        public/data/                              public/manifests/
                        room_007/                                 room_007.json
                        ├── shell.ply (5%)                        {
                        ├── shell_uobb.ply                          "version": 1,
                        └── clusters/                               "items": [
                            ├── 0-7-12_*.ply (20%)                    { "kind": "shell", ... },
                            └── 0-7-15_*.ply (20%)                    { "kind": "cluster", ... }
                                                                    ]
                              │                                     }
                              └─────────────────┬───────────────────┘
                                                │
                                                ▼
                                        http://localhost:5173/
                                        ?manifest=/manifests/room_007.json
                                                │
                                                ▼
                                          React App (App.tsx)
                                                │
                            ┌───────────────────┼───────────────────┐
                            │                   │                   │
                            ▼                   ▼                   ▼
                    Inspector.tsx      PointCloudLayer.tsx    UobbMeshLayer.tsx
```

### Manifest Schema (v1)

```typescript
interface UnifiedManifest {
  version: 1;
  title?: string;
  defaults?: {
    pointSize?: number;
    visible?: boolean;
  };
  items: ManifestItem[];
}

interface ManifestItem {
  id: string;              // Unique identifier
  name: string;            // Display name (e.g., "Shell", "Couch 12")
  kind: 'shell' | 'cluster';
  role?: 'shell' | 'cluster';
  source: string;          // Relative path to PLY file
  group?: string;          // Grouping category (e.g., "couch")
  visible?: boolean;       // Initial visibility
  style?: {
    pointSize?: number;
    color?: [number, number, number, number];
  };
  filters?: {
    labelIn?: number[];
    labelNotIn?: number[];
  };
  uobbSource?: string;     // Path to UOBB PLY (optional)
}
```

### Component Architecture

```typescript
// App.tsx - Main Controller
const App: FC = () => {
  const [manifest, setManifest] = useState<UnifiedManifest | null>(null);
  const [visibility, setVisibility] = useState<Record<string, boolean>>({});
  const [pointSize, setPointSize] = useState(3);
  
  // Load manifest from URL param
  useEffect(() => { /* fetch manifest */ }, []);
  
  // Render layers
  return (
    <DeckGL
      initialViewState={viewState}
      controller={{ type: OrbitView }}
      layers={[
        ...pointCloudLayers,  // One layer per item
        ...uobbLayers         // One layer per UOBB
      ]}
    >
      <Inspector
        items={manifest.items}
        visibility={visibility}
        onToggle={handleToggle}
        pointSize={pointSize}
        onPointSizeChange={setPointSize}
      />
    </DeckGL>
  );
};

// Inspector.tsx - UI Controls
const Inspector: FC<Props> = ({ items, visibility, onToggle }) => {
  // Render control panel
  return (
    <div className="inspector">
      <GlobalControls />
      {items.map(item => (
        <ItemControl
          key={item.id}
          item={item}
          visible={visibility[item.id]}
          onToggle={() => onToggle(item.id)}
        />
      ))}
    </div>
  );
};

// PointCloudLayer.tsx - Rendering
const PointCloudLayer: FC<Props> = ({ item, pointSize, visible }) => {
  const [data, setData] = useState<PLYData | null>(null);
  
  useEffect(() => {
    if (visible) {
      load(item.source, PLYLoader).then(setData);
    }
  }, [item.source, visible]);
  
  return new PointCloudLayerDeckGL({
    id: item.id,
    data: data?.attributes.POSITION.value,
    getColor: data?.attributes.COLOR_0.value,
    pointSize,
    visible
  });
};
```

---

## API Reference

### prepare_visualization.mjs

**Purpose:** Main automation script for end-to-end visualization preparation.

**Usage:**
```bash
node scripts/prepare_visualization.mjs [options]
```

**Key Functions:**

- `callAiApi(args: string[]): any` - Execute ai_api.py with JSON output
- `resolveObjectAssets(code: string): ObjectAssets` - Get clusters/UOBBs for object
- `resolveRoomAssets(code: string): RoomAssets` - Get shell/CSV for room
- `downsampleAndPrepare(options: DownsampleOptions): Promise<void>` - Spawn downsampler
- `cleanAllOutputs(outRoot: string): Promise<void>` - Remove all previous data
- `startDevServer(port: number, manifestName: string): Promise<void>` - Launch Vite

**Mode Handlers:**
- `handleRoomMode()` - Process complete room
- `handleClustersMode()` - Process selected clusters
- `handleMultiRoomsMode()` - Process multiple room shells
- `handleRoomWithObjectsMode()` - Process room + selected objects

### downsample_and_prepare_room.mjs

**Purpose:** Downsample PLY files and generate unified manifest.

**Usage:**
```bash
node scripts/downsample_and_prepare_room.mjs \
  --room <name> \
  [--shell <path>] \
  [--clustersDir <path>] \
  [--cluster <path>]... \
  [--ratio <float>] \
  [--voxel <float>]
```

**Key Functions:**

- `voxelSampleIndices(positions, voxelSize)` - Spatial uniform sampling
- `randomSampleIndices(n, ratio, seed)` - Deterministic random sampling
- `normalizeAttributes(data)` - Extract positions, colors, label, point_id
- `gatherByIndices(positions, colors, indices, extra)` - Preserve attributes
- `writeAsciiPLY(filePath, positions, colors, extra)` - Write ASCII PLY
- `downsamplePly(inputPath, outputPath, options)` - Main downsampling

**Sampling Strategies:**
- **Voxel:** Spatial binning with Map-based deduplication (key: `"x,y,z"`)
- **Ratio:** Fisher-Yates shuffle with LCG random generator (seed: 12345)

---

## Performance Optimization

### Downsampling Guidelines

| Scene Type | Shell Ratio | Cluster Ratio | Voxel Size | Expected Points |
|------------|-------------|---------------|------------|-----------------|
| Small room (<1M) | 0.1 (10%) | 0.3 (30%) | - | ~100-300K |
| Medium room (1-3M) | 0.05 (5%) | 0.2 (20%) | - | ~200-600K |
| Large room (>3M) | 0.03 (3%) | 0.15 (15%) | - | ~300-900K |
| Floor overview | 0.1 (10%) | N/A | - | ~500K-1M |
| Object focus | N/A | 0.25 (25%) | - | ~50-200K |

### Voxel vs Ratio Sampling

**Voxel Sampling** - Use when spatial uniformity is critical
```bash
--voxel 0.05  # 5cm voxel grid
```
- Pros: Perfect spatial distribution, no clustering artifacts
- Cons: Slower, variable output size

**Ratio Sampling** - Use for consistent point counts
```bash
--ratio 0.2  # Keep 20% of points
```
- Pros: Fast, predictable output size
- Cons: May oversample dense regions

### Browser Performance Tips

1. **Limit total points to <2M** for smooth 60 FPS
2. **Use `--shellNoColor`** for shells if color not needed (saves bandwidth)
3. **Enable `--clean-all`** periodically to prevent disk bloat
4. **Close unused manifests** in browser to free GPU memory
5. **Adjust point size** (1-3) for distant views, (3-5) for close-ups

---

For more documentation, see:
- [AI API Documentation](./AI_API.md)
- [Project Changelog](./CHANGELOG.md)
- [Root README](../README.md)
