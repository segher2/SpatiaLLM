# AI API: path resolution and head-code dispatcher

An AI-friendly control surface for the LM2PCG pipeline. It resolves paths (by object/room/filename) and dispatches short “head codes” to the compiled C++ tools with the right arguments.

## Table of Contents

- [Overview](#overview)
- [Capabilities](#capabilities)
- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Building the Project](#building-the-project)
- [Conventions](#conventions)
  - [Object/Room codes](#objectroom-codes)
  - [Directory layout](#directory-layout)
  - [File naming](#file-naming)
- [Path Resolution](#path-resolution)
  - [Resolve by filename](#resolve-by-filename)
  - [Resolve by room](#resolve-by-room)
  - [Resolve by object](#resolve-by-object)
- [Head Codes (operations)](#head-codes-operations)
  - [RCN — Reconstruct](#rcn--reconstruct)
  - [VOL — Volume](#vol--volume)
  - [ARE — Surface Area](#are--surface-area)
  - [CLR — Dominant Color](#clr--dominant-color)
  - [BBD — BBox Distance (pair)](#bbd--bbox-distance-pair)
  - [RMS — Room Manifest Summary](#rms--room-manifest-summary)
  - [VIS — Visualization](#vis--visualization)
- [Python API](#python-api)
- [JSON output](#json-output)
- [Related docs](#related-docs)

---

## Overview

The AI API is a lightweight Python layer at `scripts/ai_api.py` that:

- Scans `output/` once to build indices (by filename, room, object_code)
- Resolves inputs like `0-7-12_couch_cluster.ply`, `0-7`, or `0-7-12`
- Dispatches short, memorable operations (RCN/VOL/ARE/CLR/BBD/RMS) to the C++ apps
- Provides helpful error messages when executables are missing

It's designed for automation agents and local scripting. All results are printed to stdout and can be emitted as JSON.

## Capabilities

- Filename-only lookup anywhere under `output/`
- Room-level resolution: CSV + shell copies + shell UOBB
- Object-level resolution: clusters, UOBBs, recon meshes, and inferred `room_dir`
- Head code dispatcher that wraps `pcg_*` binaries with correct arguments
- Room manifest summary (RMS) operation for floor/room statistics

## Quick Start

```bash
# Check environment and tool availability
python3 scripts/ai_api.py check-env

# Resolve assets for one object
python3 scripts/ai_api.py resolve-object 0-7-12

# Room manifest summary
python3 scripts/ai_api.py RMS

# Reconstruct (RCN) and compute area (ARE)
python3 scripts/ai_api.py RCN 0-7-12
python3 scripts/ai_api.py ARE 0-7-12

# BBox distance between two objects
python3 scripts/ai_api.py BBD 0-7-12 0-7-14

# Color analysis
python3 scripts/ai_api.py CLR 0-7-12

# Interactive visualization
python3 scripts/ai_api.py VIS 0-7
```

**Note**: 
- All commands now use unified format: `<OPERATION> <ID>`
- JSON output is enabled by default (configured in `data/configs/default.yaml`)
- No `--object`, `--filename`, or `` flags needed
- The API no longer auto-builds. If executables are missing, you'll receive a helpful error message with build instructions.

## Prerequisites

- Python 3.7+
- C++ toolchain for the C++ apps: C++17 compiler + CMake
- Libraries: CGAL, Boost, Eigen3, PCL (and VTK/libpng/libjpeg/libtiff/zlib)

macOS hint (Homebrew):
```bash
brew install cmake cgal boost eigen pcl
```

```

Linux: use your distro packages for the equivalents.

## Building the Project

The AI API requires C++ executables to be built beforehand. The easiest way is to use the wrapper script:

```bash
# Auto-builds if needed, then processes your data
./pcg.sh "./data/rooms/Full House"
```

Alternatively, build manually:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

Check executable availability:

```bash
python3 scripts/ai_api.py check-env 
```

If executables are missing, the API will provide helpful error messages with build instructions.

## Conventions

## Install & Build

The API auto-builds binaries into `build/` on first use, or you can build explicitly.

### Auto-build (first run)

When a head code needs a missing executable, the API configures and builds Release targets automatically.

```bash
# Example: compute surface area; triggers configure+build on first run
python3 scripts/ai_api.py ARE 0-7-12 
```

Check availability anytime:

```bash
python3 scripts/ai_api.py check-env 
```

### Manual build

- Using the API itself:

```bash
python3 scripts/ai_api.py BUILD                 # configure (if needed) + build
python3 scripts/ai_api.py BUILD --reconfigure   # force reconfigure
```

- Using VS Code tasks (Terminal → Run Task…):
  - “Configure and build (Release)”
  - “Build CMake project (Release)”

Troubleshooting build:
- If libraries are missing, install them and re-run.
- If you switch compilers or upgrade libs, use `BUILD --reconfigure` or delete `build/`.

## Conventions

### Object/Room codes

- Room code: `<floor>-<room>` → e.g., `0-7`
- Object code: `<floor>-<room>-<object>` → e.g., `0-7-12`

### Directory layout

```
output/<site>/
└── floor_<f>/
    └── room_<rrr>/
        ├── <room_dirname>.csv
        └── results/
            ├── shell/
            │   └── shell_<rrr>/
            │       ├── 0-7-0_shell.ply
            │       └── 0-7-0_shell_uobb.ply
            ├── filtered_clusters/<stem>/
            │   ├── 0-7-12_<class>_cluster.ply
            │   └── 0-7-12_<class>_uobb.ply
            └── recon/<stem>/
                ├── 0-7-12_<class>_mesh.ply        (legacy)
                ├── 0-7-12_<class>_mesh_possion.ply (Poisson)
                └── 0-7-12_<class>_mesh_af.ply      (AF)
```

### File naming

- Cluster: `<object_code>_<class>_cluster.ply`
- UOBB: `<object_code>_<class>_uobb.ply`
- Mesh: `<object_code>_<class>_mesh[_possion|_af].ply`
- Room shell copy: `0-7-0_shell.ply` and `0-7-0_shell_uobb.ply`

## Path Resolution

The API scans `output/` once at initialization to build three types of indices for fast lookups:

1. **By filename**: Direct filename → absolute path(s) mapping
2. **By room code**: `(floor_id, room_id)` → CSV files, shell copies, shell UOBBs
3. **By object code**: `<floor>-<room>-<object>` → clusters, UOBBs, meshes, inferred room directory

### How Path Indexing Works

The `PathIndex` class recursively walks the `output/` directory tree and categorizes files based on naming conventions:

#### File Naming Patterns

1. **Object-level assets** (3+ parts):
   - Format: `<object_code>_<class>_<kind>.ply`
   - Examples:
     - `0-7-12_couch_cluster.ply` → cluster
     - `0-7-12_couch_uobb.ply` → UOBB
     - `0-7-12_couch_mesh.ply` → mesh (legacy)
     - `0-7-12_couch_mesh_possion.ply` → mesh (Poisson method)
     - `0-7-12_couch_mesh_af.ply` → mesh (Advancing Front method)

2. **Room-level shell** (2 parts):
   - Format: `<object_code>_shell.ply`
   - Example: `0-7-0_shell.ply` (object_id=0 indicates room-level)
   - Shell UOBB: `0-7-0_shell_uobb.ply`

3. **CSV metadata**:
   - Located in `room_XXX/` directories
   - Named as `<room_dirname>.csv`

#### Parsing Logic

- **Object code extraction**: First underscore-separated part (e.g., `0-7-12` from `0-7-12_couch_cluster.ply`)
- **Kind detection**:
  - Last part is `cluster`, `uobb`, or `mesh` → use as-is
  - Second-to-last is `mesh` (for method suffix) → kind = `mesh`
- **Room inference**: Walks up directory tree to find parent `room_XXX/` directory
- **Shell detection**:
  - 2-part name ending in `_shell` → room shell copy
  - 3-part name ending in `_shell_uobb` → room shell UOBB

### Resolve by filename

Returns all absolute paths matching a given filename (useful when the same file exists in multiple output runs).

```bash
python3 scripts/ai_api.py resolve-filename 0-7-12_couch_cluster.ply 
```

Output:
```json
{
  "matches": [
    "/abs/path/output/Full House/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply",
    "/abs/path/output/Full House (copy)/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply"
  ]
}
```

**Ambiguity handling**: When multiple matches exist (e.g., mirrored runs, backups), all candidates are returned. Some head codes support `--only-substr` to narrow results by path substring.

### Resolve by room

Returns room-level assets: CSV files, shell point clouds, and shell UOBBs.

```bash
python3 scripts/ai_api.py resolve-room 0-7 
```

Output:
```json
{
  "floor": 0,
  "room": 7,
  "csv": ["/abs/path/.../room_007/room_007.csv"],
  "shell": ["/abs/path/.../shell_007/0-7-0_shell.ply"],
  "shell_uobb": ["/abs/path/.../shell_007/0-7-0_shell_uobb.ply"]
}
```

Alternative syntax using separate integers:
```bash
python3 scripts/ai_api.py resolve-room-csv 0 7 
```

**Note**: The object_id in shell filenames is always `0` (e.g., `0-7-0_shell.ply` for room `0-7`), indicating room-level data rather than a specific object.

### Resolve by object

Returns all assets associated with an object code: clusters, UOBBs, meshes, and the inferred parent room directory.

```bash
python3 scripts/ai_api.py resolve-object 0-7-12 
```

Output:
```json
{
  "object_code": "0-7-12",
  "clusters": ["/abs/path/.../0-7-12_couch_cluster.ply"],
  "uobbs": ["/abs/path/.../0-7-12_couch_uobb.ply"],
  "meshes": [
    "/abs/path/.../0-7-12_couch_mesh_possion.ply",
    "/abs/path/.../0-7-12_couch_mesh_af.ply"
  ],
  "room_dir": "/abs/path/.../room_007"
}
```

**Room directory inference**: The API walks up to 8 parent directories from each asset file to locate the nearest `room_XXX/` directory. This is essential for operations requiring room context (e.g., reconstruction needs room-level configuration).

**Asset grouping**: All files sharing the same object code are grouped together, regardless of their physical location in subdirectories like `filtered_clusters/<stem>/` or `recon/<stem>/`.

## Head Codes (operations)

All operations use unified command format: `<OPERATION> <ID>`. JSON output is enabled by default via `json_output: true` in `data/configs/default.yaml`.

### RCN — Reconstruct

Reconstruct a mesh from a cluster PLY.

Usage:

```bash
python3 scripts/ai_api.py RCN 0-7-12
```

Behavior:
- Resolves the cluster and its `room_dir`, runs `pcg_reconstruct <cluster> <room_dir>`
- Returns the generated mesh path; method suffix differentiates Poisson/AF

Output with `` (from the API):

```json
{ "mesh": "/abs/path/..._mesh_possion.ply", "method": "poisson" }
```

Errors:
- Missing cluster for the object/filename
- Unable to infer `room_dir`

### VOL — Volume

Compute mesh volume and closedness.

Usage:

```bash
python3 scripts/ai_api.py VOL 0-7-12
python3 scripts/ai_api.py VOL 0-7-12_couch_cluster.ply
```

Behavior:
- If a mesh doesn’t exist and the input is a cluster, auto-reconstruct unless `--no-auto-recon`
- Calls `pcg_volume <mesh>` and parses JSON or legacy text

Output:

```json
{ "mesh": "/abs/path/..._mesh_af.ply", "closed": true, "volume": 0.0123 }
```

Errors:
- Mesh not found and `--no-auto-recon` set
- Volume not parsable from tool output

### ARE — Surface Area

Compute mesh surface area and closedness.

Usage:

```bash
python3 scripts/ai_api.py ARE 0-7-12
python3 scripts/ai_api.py ARE 0-7-12_couch_cluster.ply
```

Output:

```json
{ "mesh": "/abs/path/..._mesh_possion.ply", "closed": false, "area": 2.345 }
```

### CLR — Dominant Color

Run color GMM analysis on a PLY (prefers cluster when given an object code).

Usage:

```bash
python3 scripts/ai_api.py CLR 0-7-12 
python3 scripts/ai_api.py CLR 0-7-12_couch_cluster.ply 
```

Output (from tool JSON when available):

```json
{
  "file": ".../0-7-12_couch_cluster.ply",
  "M": 3,
  "components": [
    { "weight": 0.52, "mean": [r,g,b], "var": [vr,vg,vb] },
    { "weight": 0.34, "mean": [r,g,b], "var": [vr,vg,vb] },
    { "weight": 0.14, "mean": [r,g,b], "var": [vr,vg,vb] }
  ]
}
```

When JSON is not enabled in C++ tools, the API returns a light parse or raw text.

### BBD — BBox Distance (pair)

Distance and vector between two UOBB centers.

Usage:

```bash
python3 scripts/ai_api.py BBD 0-7-12 0-7-14 
```

Output:

```json
{ "distance": 1.234, "vector_1_to_2": { "x": 0.1, "y": -0.2, "z": 0.05 } }
```

Errors:
- Either object lacks a UOBB PLY
- Tool output not parsable

### RMS — Room Manifest Summary

Parse all `rooms_manifest.csv` files and return statistics about floors and rooms.

Usage:

```bash
# Auto-detect site from output directory
python3 scripts/ai_api.py RMS 

# Specify site name explicitly
python3 scripts/ai_api.py RMS "Full House" 
```

Output:

```json
{
  "site_name": "output",
  "total_floors": 2,
  "total_rooms": 10,
  "room_codes": ["0-1", "0-2", "0-3", "0-6", "0-7", "1-1", "1-2", "1-3", "1-4", "1-5"],
  "rooms": [
    {
      "floor_id": 0,
      "room_id": 1,
      "room_code": "0-1",
      "room_type": "bedroom",
      "source_manifest": "/abs/path/.../floor_0/rooms_manifest.csv"
    },
    ...
  ],
  "manifest_files": ["/abs/path/.../floor_0/rooms_manifest.csv", ...]
}
```

**Use cases:**
- Discover available rooms before visualization
- Generate floor/room statistics for reports
- Validate pipeline outputs across multiple floors

### VIS — Visualization

Prepare and optionally launch web-based visualization for rooms and objects. The VIS command features **automatic mode detection** and **name generation**, making it extremely easy to use.

#### Quick Start

```bash
# Single room - auto-detects 'room' mode, auto-serves
python3 scripts/ai_api.py VIS 0-7

# Multiple rooms - auto-detects 'multi-rooms' mode
python3 scripts/ai_api.py VIS 0-1 0-2 0-3

# Specific objects - auto-detects 'clusters' mode
python3 scripts/ai_api.py VIS 0-7-12 0-7-15

# Room with selected objects - auto-detects 'room-with-objects' mode
python3 scripts/ai_api.py VIS 0-7 0-7-12 0-7-15

# Non-interactive mode (visualization only, no waiting)
python3 scripts/ai_api.py VIS 0-7 --no-wait
```

**Output Example:**
```
Status: success
Mode: room
Name: room_0_7
Viewer URL: http://localhost:5173/?manifest=room_0_7.json
Rooms: 0-7

[User selects objects in browser and confirms]

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

Use `--no-clean-all` to keep previous outputs or `--no-serve` to skip server startup.

#### Configuration

VIS reads default parameters from `data/configs/default.yaml`:

```yaml
# Viewer visualization parameters
viewer_downsample_ratio: 0.2        # downsample ratio for cluster point clouds (0.0-1.0)
viewer_downsample_ratio_shell: 0.05 # downsample ratio for shell/room point clouds (0.0-1.0)
viewer_voxel_size: null             # optional: voxel size for spatial downsampling (in meters)
viewer_shell_no_color: false        # if true, render shell in constant gray color
```

You can override these defaults with command-line arguments.

#### Auto-Detection

The VIS command automatically detects the visualization mode based on input codes:

| Input Pattern | Detected Mode | Auto-Generated Name | Example |
|--------------|---------------|---------------------|---------|
| Single room code (`X-Y`) | `room` | `room_0_7` | `VIS 0-7` |
| Multiple room codes | `multi-rooms` | `multi_rooms_0_1_0_2_0_3` | `VIS 0-1 0-2 0-3` |
| Object codes only (`X-Y-Z`) | `clusters` | `clusters_0_7_12_0_7_15` | `VIS 0-7-12 0-7-15` |
| Mix of rooms and objects | `room-with-objects` | `room_0_7_with_2_objects` | `VIS 0-7 0-7-12 0-7-15` |

#### Usage

```bash
python3 scripts/ai_api.py VIS <codes>... \
  [--mode <mode>] \
  [--name <output_name>] \
  [--ratio <float>] \
  [--ratio-shell <float>] \
  [--voxel <float>] \
  [--shell-no-color] \
  [--no-clean] \
  [--no-clean-all] \
  [--no-serve] \
  [--port <int>] \
 
```

**Positional Arguments:**
- `codes`: Room codes (e.g., `0-7`) and/or object codes (e.g., `0-7-12`)
  - Mode is auto-detected if `--mode` is not specified
  - Name is auto-generated if `--name` is not specified

**Modes (auto-detected or manual):**

1. **`room`** — Visualize entire room (shell + all clusters)
   - Input: Single room code (e.g., `0-7`)
   - Automatically finds shell and all object clusters

2. **`clusters`** — Visualize selected clusters only
   - Input: Object codes only (e.g., `0-7-12 0-7-15`)
   - No room context, just the specified objects

3. **`multi-rooms`** — Visualize multiple room shells
   - Input: Multiple room codes (e.g., `0-1 0-2 0-3`)
   - Shell only, useful for floor layout overview

4. **`room-with-objects`** — Room shell with selected objects
   - Input: One room code + object codes (e.g., `0-7 0-7-12 0-7-15`)
   - Combines room context with specific object highlights

**Options:**
- `--mode`: Manual mode specification (auto-detected if omitted)
- `--name`: Custom output name (auto-generated if omitted)
- `--ratio`: Downsample ratio for clusters (default from config: 0.2)
- `--ratio-shell`: Downsample ratio for shell (default from config: 0.05)
- `--voxel`: Optional voxel size for spatial sampling
- `--shell-no-color`: Strip color from shell (gray rendering)
- `--no-clean`: Skip cleaning same-name output (default: clean same-name)
- `--no-clean-all`: Skip cleaning ALL outputs (default: clean all)
- `--no-serve`: Don't auto-start dev server (default: auto-start)
- `--port`: Dev server port (default: 5173)

#### Examples

**Simple room visualization:**
```bash
# Visualize room 0-7 (auto-detects room mode, auto-serves, auto-cleans)
python3 scripts/ai_api.py VIS 0-7
# Output name: room_0_7
# Viewer URL: http://localhost:5173/?manifest=room_0_7.json
```

**Multiple rooms overview:**
```bash
# Floor layout with multiple rooms
python3 scripts/ai_api.py VIS 0-1 0-2 0-3 0-6 0-7
# Output name: multi_rooms_0_1_0_2_0_3_0_6_0_7
```

**Selected objects:**
```bash
# Visualize specific furniture items
python3 scripts/ai_api.py VIS 0-7-12 0-7-15 0-7-3
# Output name: clusters_0_7_12_0_7_15_0_7_3

# With custom settings
python3 scripts/ai_api.py VIS 0-7-12 --ratio 0.5 --name couch_detail
```

**Room with highlighted objects:**
```bash
# Room context with specific objects
python3 scripts/ai_api.py VIS 0-7 0-7-12 0-7-15
# Output name: room_0_7_with_2_objects
```

**Manual mode specification:**
```bash
# Explicitly specify mode and name
python3 scripts/ai_api.py VIS 0-7 --mode room --name my_room
```

**Without server auto-start:**
```bash
# Prepare visualization without launching server
python3 scripts/ai_api.py VIS 0-7 --no-serve
# Then manually: cd web/pointcloud-viewer && npm run dev
```

**Output with ``:**

```json
{
  "status": "success",
  "mode": "room",
  "name": "room_0_7",
  "viewer_url": "http://localhost:5173/?manifest=room_0_7.json",
  "room_codes": ["0-7"]
}
```

#### Integration Details

**Architecture:**
```
User Input (CLI/Python)
    ↓
ai_api.py::op_VIS()
    ↓
PathIndex (path resolution)
    ↓
prepare_visualization.mjs (Node.js)
    ↓
downsample_and_prepare_room.mjs
    ↓
Web Viewer (deck.gl)
```

**What VIS Does:**
1. Auto-detects mode from input codes (or uses specified `--mode`)
2. Auto-generates descriptive name (or uses specified `--name`)
3. Resolves paths using PathIndex to find shells, clusters, UOBBs
4. Cleans previous visualization outputs (configurable)
5. Calls Node.js preparation script with resolved paths
6. Downsamples PLY files for performant web rendering
7. Generates manifest JSON for the viewer
8. Auto-starts development server (configurable)

**Output Location:**
- Data files: `web/pointcloud-viewer/public/data/<name>/`
- Manifest: `web/pointcloud-viewer/public/manifests/<name>.json`

**Prerequisites:**
- Node.js >= 18 installed
- Viewer dependencies installed: `cd web/pointcloud-viewer && npm install`

**Server Management:**
- Start: `cd web/pointcloud-viewer && bash start_dev.sh`
- Stop: `cd web/pointcloud-viewer && bash stop_dev.sh`
- Logs: `/tmp/vite_server.log`, `/tmp/api_server.log`

## Python API

```python
from scripts.ai_api import Dispatcher

d = Dispatcher()

# Path resolution
print(d.index.find_by_filename("0-7-12_couch_cluster.ply"))
print(d.index.find_csv(0, 7))
print(d.index.find_assets("0-7-12"))

# Operations
mesh_path = d.op_RCN(object_code="0-7-12")
mesh, vol, closed = d.op_VOL(object_code="0-7-12")
mesh, area, closed = d.op_ARE(object_code="0-7-12")
color = d.op_CLR(object_code="0-7-12")
dist, vec = d.op_BBD("0-7-12", "0-7-14")

# Room manifest summary
manifest_summary = d.op_RMS()  # or op_RMS("Full House")

# Visualization
vis_result = d.op_VIS(
    mode="room",
    name="room_007",
    room_codes=["0-7"],
    ratio=0.2,
    ratio_shell=0.05,
    auto_serve=True,
    port=5173
)
print(vis_result["viewer_url"])

# Visualize selected objects
vis_result = d.op_VIS(
    mode="clusters",
    name="furniture_items",
    object_codes=["0-7-12", "0-7-15", "0-7-3"],
    ratio=0.3
)
```

## JSON output

**Global Configuration:** Set `json_output: true` in `data/configs/default.yaml` (enabled by default). This controls both:
1. **CLI Output Format**: All `ai_api.py` commands return JSON by default
2. **C++ Tool Output**: When enabled, C++ tools emit structured JSON to stdout

**Configuration:**
```yaml
# data/configs/default.yaml
json_output: true           # Enable JSON output for all tools
viewer_downsample_ratio: 0.2
viewer_downsample_ratio_shell: 0.05
```

Per-tool JSON shapes (tool output):

- `pcg_volume` → `{ "file": "<path>", "closed": true|false, "volume": <float> }`
- `pcg_area`   → `{ "file": "<path>", "closed": true|false, "area": <float> }`
- `pcg_bbox`   →
  - gen: `{ "mode": "gen", "status": "ok|failed", "file": "<out.ply>" }`
  - pair: `{ "mode": "pair", "center1": {x,y,z}, "center2": {x,y,z}, "vector_1_to_2": {x,y,z}, "distance": <float> }`
    - errors: `{ "mode": "pair", "status": "not_found|read_failed|empty", ... }`
- `pcg_color`  → `{ "file": "<ply>", "M": <int>, "components": [ { "weight": <float>, "mean": [r,g,b], "var": [vr,vg,vb] }, ... ] }`
  - empty: `{ "file": "<ply>", "M": 0 }`
- `pcg_reconstruct` → one JSON object per input cluster:
  - success (Poisson): `{ "file": "<cluster.ply>", "method": "poisson", "mesh": "<mesh.ply>", "status": "ok" }`
  - success (AF):      `{ "file": "<cluster.ply>", "method": "af",      "mesh": "<mesh.ply>", "status": "ok" }`
  - read failure:      `{ "file": "<cluster.ply>", "status": "read_failed" }`
  - failed:            `{ "file": "<cluster.ply>", "status": "failed" }`

Notes:
- Tools that process multiple inputs emit newline-delimited objects (streaming), not an array.

## Related docs

- [Point Cloud Viewer](./POINTCLOUD_VIEWER.md)
- [Project Changelog](./CHANGELOG.md)
- [Root README](../README.md)

