# Indoor Point Cloud Pipeline   1.3.0

[![Release](https://img.shields.io/github/v/release/Jackson513ye/LM2PCG?sort=semver)](https://github.com/Jackson513ye/LM2PCG/releases)

A compact C++17 pipeline for indoor point-cloud processing with PCL and optional CGAL. Clusters object point clouds, computes upright OBBs, preserves vertex colors end-to-end, and exports standardized results.

## Key Features

- **Color-preserving PLY I/O**: Full XYZRGB support for clusters, UOBBs, and meshes
- **FEC-style clustering**: Radius-based with smart filtering
- **Upright bounding boxes**: Optimal OBBs using convex hull + rotating calipers
- **Reconstruction**: Poisson with acceptance checks + AF fallback
- **Accurate geometry analysis**: 
  - Volume calculation with automatic method selection (signed volume for closed, adaptive voxel for non-closed)
  - Automatic triangulation preprocessing (fixes 50% polygon face calculation error)
  - Surface area calculation with sub-1% accuracy on test geometries
  - Dominant color analysis using GMM with perceptual distance
  - Bounding box distance and point-to-bbox queries
- **Unified AI API**: Simplified Python interface with `<OPERATION> <ID>` format ([docs/AI_API.md](docs/AI_API.md))
- **Interactive Web Viewer**: Real-time 3D visualization with selection monitoring ([docs/POINTCLOUD_VIEWER.md](docs/POINTCLOUD_VIEWER.md))
- **JSON-First Output**: Configuration-based structured output for AI agents

## Quick Start

### Dependencies

| Component | Minimum Version | Tested Version | Status |
|-----------|----------------|----------------|--------|
| CMake | 3.16 | 4.0.2 | Required |
| C++ Standard | C++17 | C++17 | Required |
| PCL | 1.10 | 1.15.1_1 | Required |
| Eigen3 | (any) | 3.4.1 | Required |
| CGAL | (any) | 6.1 | Required |
| Boost | (any) | 1.89.0 | Required |
| Python | 3.x | 3.8+ | Required for AI API |
| Node.js | 16+ | 18+ | Required for web viewer |

**Required:**
- CMake 3.16+
- C++17 compiler (GCC 7+, Clang 5+, MSVC 2017+)
- PCL 1.10+
- Boost, Eigen3

**For mesh processing (pcg_reconstruct, pcg_volume, pcg_area):**
- **CGAL 5.3+** (required for unified IO headers)

```bash
# macOS
brew install cmake cgal boost eigen pcl

# Linux (Debian/Ubuntu 22.04+)
sudo apt-get install cmake build-essential libpcl-dev libcgal-dev libeigen3-dev libboost-all-dev

# Linux (Ubuntu 20.04 or older with CGAL < 5.3)
# You need to build CGAL from source:
wget https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6.tar.xz
tar xf CGAL-5.6.tar.xz && cd CGAL-5.6
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install
```

> **Note**: If CGAL < 5.3 is detected, mesh processing tools will be skipped during build. Only `pcg_room`, `pcg_color`, and `pcg_bbox` will be built.

### Build

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

### Quick Start with Wrapper Script (Recommended)

Use `pcg.sh` for automated building and execution:

```bash
# Auto-builds if needed, then processes the dataset
./pcg.sh "./data/rooms/Full House"
```

**Features**:
- Auto-build on first run or when executable is missing
- Fixed output to `./output/` (auto-cleared before each run)
- Automatic RMS (Room Manifest Summary) generation after processing
- Copies `rooms_manifest.csv` to output directories

### Manual Usage

```bash
# Process a room (clustering + UOBB + CSV)
./build/pcg_room "./data/rooms/Full House"

# Note: Output is now fixed to ./output/ and is auto-cleared before each run
# Room processing no longer accepts single room directories

# Reconstruct clusters to meshes
./build/pcg_reconstruct "output/Full House" "output/Full House"

# Analyze mesh properties
./build/pcg_volume output/Full\ House/**/results/recon/**/*_mesh.ply
./build/pcg_area output/Full\ House/**/results/recon/**/*_mesh.ply
```

## Core Tools

### 1. pcg_room - Clustering and Processing
```bash
./build/pcg_room <input_dir> [radius] [min_cluster_size]
```
Process entire sites with floor/room structure. Output is fixed to `./output/` (auto-cleared). Outputs colored cluster PLYs, UOBBs, and CSV summaries.

### 2. pcg_reconstruct - Mesh Generation
```bash
./build/pcg_reconstruct <input_root_or_room_dir> <output_root_dir>
```
Converts clusters to meshes using Poisson (with validation) or AF fallback.

### 3. pcg_volume - Volume Analysis
```bash
./build/pcg_volume <mesh_file> [mesh_file_2 ...]
./build/pcg_volume --voxel <mesh_file>  # Force voxel method
```
Computes mesh volume with automatic method selection:
- **Closed meshes**: Uses signed volume method (0% error for polyhedra)
- **Non-closed meshes**: Automatically uses adaptive voxel method (5-8% error)
- **Adaptive voxel**: AABB tree ray casting with 3-level boundary refinement

All meshes are automatically triangulated to fix polygon face calculation errors.

### 4. pcg_area - Surface Area Analysis
```bash
./build/pcg_area <mesh_file> [mesh_file_2 ...]
```
Computes surface area for both open and closed meshes. Automatically triangulates polygon faces for accurate calculation (fixes 50% calculation error from quad faces).

### 5. pcg_color - Dominant Color
```bash
./build/pcg_color <cluster.ply>
```
Analyzes dominant colors using GMM and perceptual color distance (ΔE*76).

### 6. pcg_bbox - Bounding Box Tools
```bash
# Compute distance between two objects
./build/pcg_bbox <bbox1_uobb.ply> <bbox2_uobb.ply>

# Generate test UOBB
./build/pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg

# Point to bbox distance
./build/pcg_bbox point x y z <bbox_uobb.ply>
```

## AI API

Unified Python interface with simplified command format. **[Complete guide →](docs/AI_API.md)**

### Quick Start

```bash
# All operations use simple format: <OPERATION> <ID>
python3 scripts/ai_api.py RCN 0-7-12     # Reconstruct mesh
python3 scripts/ai_api.py VOL 0-7-12     # Compute volume
python3 scripts/ai_api.py ARE 0-7-12     # Compute surface area
python3 scripts/ai_api.py CLR 0-7-12     # Analyze color
python3 scripts/ai_api.py BBD 0-7-12 0-7-15  # Distance between objects
python3 scripts/ai_api.py RMS            # Room manifest summary

# Interactive visualization (default mode)
python3 scripts/ai_api.py VIS 0-7        # Visualize room, wait for user selection
python3 scripts/ai_api.py VIS 0-7-12 0-7-15  # Visualize objects

# Non-interactive mode
python3 scripts/ai_api.py VIS 0-7 --no-wait
```

**Key Features**:
- **Unified Format**: All commands use `<OPERATION> <ID>` (no `--object`, `--filename`, or `--json` flags)
- **JSON by Default**: Configured in `data/configs/default.yaml` (`json_output: true`)
- **Interactive VIS**: Auto-starts servers, waits for selection, outputs JSON, auto-closes
- **Auto-Detection**: Automatically detects room vs object codes, visualization modes
- **AI Agent Ready**: Structured JSON output for automated workflows

**Operations**: `RCN` (reconstruct), `VOL` (volume), `ARE` (area), `CLR` (color), `BBD` (distance), `RMS` (room summary), `VIS` (visualization)

## Web Visualization

Interactive 3D viewer with real-time object selection monitoring. **[Complete guide →](docs/POINTCLOUD_VIEWER.md)**

```bash
# Quick start via AI API (recommended - fully automated)
python3 scripts/ai_api.py VIS 0-7
# → Auto-starts frontend (5173) and backend (8090) servers
# → Opens browser for visualization
# → Waits for user to select objects and click "Confirm All"
# → Outputs selection as JSON
# → Auto-closes servers

# Manual server management (from web/pointcloud-viewer)
./start_dev.sh                 # Start servers manually
./stop_dev.sh                  # Stop servers

# Or manual npm commands
npm run visualize -- --mode room --room 0-7 --name room_007
```

**4 Modes**: `room`, `clusters`, `multi-rooms`, `room-with-objects`  

**Key Features**: 
- **Real-time Selection Monitoring**: Interactive workflow with automatic JSON output
- **One-Click Download**: Download source PLY files via integrated API
- **Auto-Detection**: Automatically determines visualization mode from input codes
- **Smart Server Management**: Auto-start, auto-wait, auto-close by default
- **Non-pickable UOBB**: Bounding boxes don't block object selection
- Per-object visibility toggles with semantic naming
- 10M+ points @ 60 FPS performance with optimized downsampling
./start_dev.sh    # Start frontend (5173) + API server (8090)
./stop_dev.sh     # Stop both servers
```

## Configuration

Default settings in `data/configs/default.yaml`:

**Clustering**:
- `radius` (0.05): Neighbor radius in meters
- `min_cluster_size` (50): Minimum points per cluster
- `filter_factor` (0.70): Size-based filter threshold

**Reconstruction**:
- `poisson_min_oriented_fraction` (0.3): Normal orientation threshold
- `poisson_require_closed` (true): Require closed meshes
- `af_require_closed` (false): AF fallback policy

**Color Analysis**:
- `color_sample_n` (300): RGB sample size
- `color_deltaE_keep` (20.0): Perceptual color merge threshold

**Viewer**:
- `viewer_downsample_ratio` (0.2): Cluster downsampling rate (20%)
- `viewer_downsample_ratio_shell` (0.05): Shell downsampling rate (5%)
  - Note: RMS multi-rooms mode uses 1% (0.01) for performance
- `viewer_point_size` (3): Default point size for rendering
- `viewer_uobb_opacity` (0.3): UOBB transparency (0.0-1.0)
- `viewer_uobb_color` ([30, 144, 255]): UOBB color (RGB)

**JSON Output**:
```yaml
json_output: true  # Enable structured JSON output
```

## Output Structure

```
output/<site>/<floor>/<room>/
├── <room>.csv                          # Geometry summary
└── results/
    ├── filtered_clusters/<stem>/
    │   ├── <object_code>_<class>_cluster.ply
    │   └── <object_code>_<class>_uobb.ply
    └── recon/<object_stem>/
        └── <object_code>_<class>_mesh.ply
```

**Object codes**: `floor-room-object` (e.g., `0-7-12` = floor 0, room 7, object 12)

## Documentation

- **[AI API](docs/AI_API.md)** - Python orchestration and automation
- **[Point Cloud Viewer](docs/POINTCLOUD_VIEWER.md)** - Web visualization system
- **[Changelog](docs/CHANGELOG.md)** - Version history
