# Changelog

All notable changes to this project are documented here. This log mirrors the style of `docs/CHANGELOG.md` and focuses on the latest integrations for AI orchestration and structured outputs.

## 1.3.0 / 2025-11-03

### Geometry Calculation Accuracy Improvements

#### Critical Bug Fix: Polygon Face Triangulation
- **Fixed 50% calculation error in area and volume tools** caused by quad/polygon faces
  - Added automatic triangulation preprocessing in `pcg_volume` and `pcg_area`
  - CGAL PLY reader previously handled only ~50% of quad faces correctly
  - Now uses `CGAL::Polygon_mesh_processing::triangulate_faces()` after mesh loading
  - Validation: cube test shows 0.00% error (was ~50% before)
  - Reconstruction accuracy improved dramatically (e.g., gemstone0: +527% → +0.3%, sofa20: +92% → -0.1%)

#### New Feature: Adaptive Voxel Volume Calculation
- **Implemented robust volume calculation for non-closed meshes** (`mesh_volume_adaptive_voxel()`)
  - AABB tree-based ray casting for inside/outside testing
  - 3-level adaptive boundary refinement for accuracy
  - Auto-computed base voxel size (20 voxels along longest axis)
  - Typical accuracy: 5-8% underestimation (inherent to voxel methods)
  - Validation: sphere r=5 sub3 shows -0.86% error with signed volume, -5.60% with voxel method

#### Enhanced Volume Calculation Workflow
- **Automatic method selection for optimal accuracy**
  - Closed meshes: Use signed volume method (0% error for polyhedra)
  - Non-closed meshes: Automatically fall back to adaptive voxel method (5-8% error)
  - Removed skip logic: all meshes now get volume calculated
  - User can still force voxel method with `--voxel` flag

#### Validation and Documentation
- **Comprehensive testing with 8 test geometries**
  - Cubes, tetrahedron, spheres, cylinders, cone with known theoretical values
  - Results documented in `test_report_pcg_area_volume.tex`
  - Polyhedra: 0.00% error with triangulation + signed volume
  - Curved surfaces: sub-1% error due to mesh discretization
- **Method descriptions** documented in `method_description_volume_area.tex`
  - Signed volume method: Divergence theorem for closed meshes
  - Adaptive voxel method: Ray casting with boundary refinement
  - Usage recommendations based on mesh topology

### Files Modified
- `src/apps/pcg_volume.cpp`: Added triangulation preprocessing, auto voxel fallback
- `src/apps/pcg_area.cpp`: Added triangulation preprocessing
- `src/geometry/volume.cpp`: Implemented `mesh_volume_adaptive_voxel()` with AABB tree
- `include/pcg/geometry/volume.hpp`: Added function declaration for adaptive voxel method

## 1.3.0-beta.2 / 2025-10-30

### Build System Improvements

#### CGAL Version Requirement
- **Added explicit CGAL 5.3+ requirement** in CMakeLists.txt
  - `find_package(CGAL 5.3 QUIET COMPONENTS Core)` replaces unversioned lookup
  - Prevents compilation errors on systems with older CGAL versions
  - Enhanced error messages with installation instructions
  - Graceful degradation: mesh tools skipped if CGAL < 5.3

- **Documentation Updates**
  - Added CGAL version requirement to README.md
  - Included manual build instructions for Ubuntu 20.04 and older systems
  - Clear separation of required vs optional dependencies

### Major Improvements

#### AI Agent System Prompt Enhancement
- **Embedded Comprehensive System Prompt**: Moved complete system instructions from external documentation into code
  - Removed dependency on `COMPREHENSIVE_SYSTEM_PROMPT.md` that LLM cannot access at runtime
  - Embedded full system prompt (~350 lines) directly into `_create_system_prompt()` method
  - Includes all tool specifications, RGB color interpretation guides, response guidelines, and best practices
  - LLM now receives complete instructions in every API call

- **Enhanced System Prompt Structure**
  - Added visual section dividers (═══) for improved readability
  - 9 major sections: Core Identity, System Architecture, Data Model, Available Tools, Tool Invocation, Response Guidelines, Critical Instructions, Advanced Capabilities, Limitations
  - Detailed RGB color interpretation with 5 examples
  - Complete tool usage protocols for VOL, CLR, BBD, RCN, VIS
  - 10 comprehensive response guidelines with formatting examples

#### Database Schema Updates
- **Room Type Classification Support** (`room_database.py`)
  - Added `room_type` column to rooms table (VARCHAR default 'unknown')
  - Added automatic data migration for existing databases
  - Schema version tracking system (version 2)
  - Support for semantic room queries (kitchen, bedroom, hallway, etc.)

- **Enhanced Data Integrity**
  - Added NOT NULL constraints on essential columns (room_name, floor_name)
  - Improved error handling for schema migrations
  - Better validation for room and object data

#### Room Type Enrichment System
- **Vision-Based Room Classification** (`enrich_room_types.py`)
  - New automated room type classification using GPT-4o vision model
  - Processes room panorama images to determine room types
  - Updates database with classified types (kitchen, bedroom, living_room, etc.)
  - Handles rooms without images gracefully
  - Batch processing support for multiple rooms

#### API Wrapper Enhancements
- **Selection Session Support** (`ai_api_wrapper.py`)
  - Added `session_id` parameter to `visualize_point_cloud()` method
  - Enables user selection tracking in 3D viewer
  - Supports interactive workflows with confirmation dialogs

#### API Server Improvements (`scripts/api_server.py`)
- **Selection Confirmation Endpoint**
  - New `/api/confirm-selection` endpoint for viewer interactions
  - Accepts POST requests with selected object data
  - Writes selection to session-specific JSON files (`/tmp/viewer_selection_{session_id}.json`)
  - Enables agent to wait for and process user selections

- **Enhanced Path Resolution**
  - Improved object and room path resolution logic
  - Better error messages for missing resources
  - More robust file serving capabilities

### Added Features

#### Agent Capabilities
- **Interactive Selection Workflow** (`mutli_room_agent2.py`)
  - New `_wait_for_user_selection()` method
  - Polls for user selections with configurable timeout (default 60s)
  - Progress indicators every 10 seconds
  - Automatic cleanup of selection files
  - Returns structured object list or None on timeout

- **Auto-Triggered Visualization**
  - Detects room display queries automatically
  - Triggers VIS tool for "show", "display", "visualize" keywords
  - Works with room names (kitchen, bedroom) and room codes (0-1)
  - Seamless integration with query processing

- **Semantic Room Type Parsing**
  - Enhanced `_parse_room_reference()` with NLP context patterns
  - Matches room type names in natural language queries
  - Supports phrases like "show the kitchen", "visualize bedroom"
  - Fallback to word boundary matching for broader coverage

### Changed

#### System Prompt Documentation
- Removed references to external documentation files in docstrings
- Updated class and method documentation to reflect embedded prompts
- Simplified documentation structure

#### Code Organization
- Removed redundant tool specification comments
- Consolidated RGB color interpretation in single location
- Better separation of concerns in prompt generation

### Removed

#### Test Files Cleanup
- Deleted obsolete test files:
  - `scripts/test_ai_agent_workflow.py` (57 lines removed)
  - `scripts/test_manual_ops.py` (40 lines removed)
  - `scripts/test_realtime_selection.py` (95 lines removed)
- Removed empty `test_ai_wrapper.py` file

### Documentation

#### Updated Documentation Files
- **COMPREHENSIVE_SYSTEM_PROMPT.md**: Retained as developer reference
  - Still contains complete system documentation
  - Now explicitly marked as developer-only reference
  - LLM receives content through embedded prompt instead

### Technical Details

#### Statistics
- **Code Changes**:
  - `mutli_room_agent2.py`: +438 lines (major prompt enhancement)
  - `room_database.py`: +217 lines (schema updates and migrations)
  - `enrich_room_types.py`: +27 lines (vision classification)
  - `scripts/api_server.py`: +33 lines (selection endpoint)
  - Total: ~715 lines added, ~310 lines removed

#### Database Schema
```sql
-- New room_type column
ALTER TABLE rooms ADD COLUMN room_type VARCHAR DEFAULT 'unknown';

-- Schema version tracking
PRAGMA user_version = 2;
```

#### Selection File Format
```json
[
  {
    "id": "cluster-0-1-5",
    "name": "chair (object_id: 0-1-5)",
    "role": "object",
    "code": "0-1-5"
  }
]
```

### Developer Notes
- System prompt is now self-contained in code - no external file dependencies
- Database migrations handled automatically on connection
- Room type classification can be run separately via `enrich_room_types.py`
- Selection workflow requires both frontend (port 5173) and backend (port 8090) servers
- Session IDs must be unique for concurrent selection workflows

### Migration Guide
- Existing databases will auto-migrate to schema version 2 on first connection
- Run `enrich_room_types.py` to classify existing rooms with images
- No breaking changes to existing API calls
- Selection workflow is optional and backwards compatible

## 1.3.0-beta / 2025-10-27

### Breaking Changes
- **Unified Command Format**: All AI API operations now use simplified format `<OPERATION> <ID>`
  - Removed `--object`, `--filename`, and `--json` flags
  - Example: `python3 scripts/ai_api.py RCN 0-7-12` (no flags needed)
  - JSON output controlled by `json_output: true` in `data/configs/default.yaml`

### Added
- **Interactive VIS Workflow** (Default Mode)
  - Automatic server startup (frontend on 5173, backend on 8090)
  - Real-time selection monitoring with 5-minute timeout
  - Auto-cleanup of old selection files before each run
  - JSON output of user selections when "Confirm All" is clicked
  - Automatic server shutdown after selection or timeout
  - Added `--no-wait` flag for non-interactive visualization mode

- **Configuration-Based JSON Output**
  - Global `json_output` setting in `data/configs/default.yaml` (default: true)
  - All operations automatically return JSON format without flags
  - Consistent output structure across all commands

### Changed
- **Command Interface Simplification**
  - RCN: `--object 0-7-12` → `0-7-12`
  - VOL: `--object 0-7-12 --no-auto-recon` → `0-7-12 --no-auto-recon`
  - ARE: `--object 0-7-12` → `0-7-12`
  - CLR: `--object 0-7-12` → `0-7-12`
  - BBD: Already using simple format (no change)
  - VIS: Already using simple format (enhanced with interactive mode)

- **VIS Output Format**
  - Simplified initial output (Status, Mode, Name, Viewer URL, Rooms/Objects)
  - Removed verbose prompts and suggestions
  - User selection output as pure JSON array
  - Silent server shutdown (no extra messages)

- **Node.js Visualization Scripts**
  - Removed `--json` flag from `ai_api.py` calls in `prepare_visualization.mjs`
  - Automatic JSON parsing based on config setting

### Fixed
- **Selection File Caching**
  - Old selection files (`/tmp/viewer_selection.json`) now cleaned before each VIS run
  - Prevents immediate detection of previous selections
  - Ensures fresh user input on every visualization

- **Duplicate BBD Parser**
  - Removed duplicate BBD argument parser definition
  - Fixed `argparse.ArgumentError: conflicting subparser` error

### Documentation
- **Consolidated Documentation**
  - Merged `UNIFIED_API_USAGE.md` content into `AI_API.md`
  - Merged `REALTIME_SELECTION.md` content into `POINTCLOUD_VIEWER.md`
  - Removed redundant documentation files
  - Updated all command examples to use new unified format
  - Added interactive workflow examples and output samples

- **Enhanced Examples**
  - Added complete workflow examples for all operations
  - Included JSON output samples for each command
  - Documented interactive vs non-interactive VIS modes
  - Added configuration guidelines

### Developer Notes
- All API operations now prioritize simplicity: `<OPERATION> <ID>`
- JSON output is the default behavior (configured, not commanded)
- Interactive mode is default for VIS (use `--no-wait` to override)
- Selection workflow: prepare → serve → wait → detect → output → close

## 1.3.0-alpha.3 / 2025-10-25

### Added
- **VIS (Visualization) Operation**
  - New `VIS` command for automated point cloud visualization
  - Auto-detection of visualization mode from input codes (room, clusters, multi-rooms, room-with-objects)
  - Automatic name generation based on mode and codes
  - Default behavior: auto-clean all outputs and auto-start server
  - Configuration reading from `data/configs/default.yaml`
  - Support for `--no-clean-all` and `--no-serve` flags to override defaults

- **RMS Visualization**
  - Added `--visualize` flag to RMS operation
  - Automatic multi-rooms visualization after manifest summary
  - Optimized shell downsample ratio (0.01 = 1%) for performance

- **API Server Enhancements**
  - `/api/resolve-object` and `/api/resolve-room` now return relative paths
  - `/api/download-file` supports both absolute and relative paths
  - Improved path portability across different project locations

### Changed
- **Viewer Improvements**
  - UOBB bounding boxes are no longer selectable (pickable=false)
  - Fixed UOBB center calculation bug for rotated boxes
  - Fixed manifest path handling (auto-prepend /manifests/)
  - Server startup now uses independent start_dev.sh script

- **Pipeline Processing**
  - Fixed multi-rooms shell downsample ratio (now uses ratioShell parameter)
  - Shell downsampling: 0.05 (normal), 0.01 (RMS multi-rooms)

### Fixed
- **Path Resolution**
  - Fixed absolute path issue in download functionality
  - API now returns relative paths for better portability
  - Download handler resolves relative paths against project root

- **UOBB Calculation**
  - Fixed center position calculation for oriented bounding boxes
  - Center now computed before L/W dimension swap

### Documentation
- Merged VIS_EXAMPLES.md and VIS_IMPLEMENTATION.md into AI_API.md
- Updated POINTCLOUD_VIEWER.md with latest features
- Added comprehensive VIS usage examples

## 1.3.0-alpha.2 / 2025-10-25

### Changed
- **Output Structure Simplification**
  - `pcg_room` now outputs directly to `./output/floor_X/room_XXX/` (removed site name subdirectory)
  - Fixed output path to `./output/` with automatic clearing before each run
  - Removed support for processing single room directories (now requires full site structure)
  - Updated CLI: `pcg_room <input_dir> [radius] [min_cluster_size]` (removed output_dir parameter)

- **Manifest File Handling**
  - Automatic copying of `rooms_manifest.csv` from input to output directories
  - Copies manifest files to each `output/floor_X/` directory during processing

- **Wrapper Script (`pcg.sh`)**
  - Simplified to only accept `<input_path>` parameter
  - Auto-build functionality if `pcg_room` executable is missing
  - Automatically runs RMS (Room Manifest Summary) after `pcg_room` completes
  - JSON output for immediate room statistics

- **AI API (`scripts/ai_api.py`)**
  - **Removed auto-build functionality** - now provides helpful error messages with build instructions
  - Updated `op_RMS` to support new flat directory structure (`output/floor_X/` instead of `output/site/floor_X/`)
  - Improved directory structure detection with fallback to legacy structure for backward compatibility
  - Enhanced error messages for missing executables

### Added
- **RMS Operation Enhancement**
  - Automatic execution in `pcg.sh` workflow
  - Support for both new flat structure and legacy nested structure
  - Auto-detection of manifest files in `floor_X` directories

### Documentation
- **README.md Updates**
  - Added "Quick Start with Wrapper Script" section
  - Updated `pcg_room` usage examples
  - Clarified output structure and auto-clear behavior
  - Added RMS operation to operations list
  - Noted removal of auto-build from AI API

- **AI_API.md Updates**
  - Removed "Auto-build (first run)" section
  - Renamed "Install & Build" to "Building the Project"
  - Added recommendation to use `./pcg.sh` wrapper script
  - Updated quick start examples with RMS operation
  - Clarified that executables must be built beforehand

- **Removed BUILD_AND_RUN.md**
  - Content merged into README.md
  - Deprecated in favor of integrated documentation

### Fixed
- `op_RMS` manifest file detection for new output structure
- Path resolution in `ai_api.py` to work with flat directory layout

## 1.3.0-alpha.1 / 2025-10-24

### Added
- **Interactive Object Selection System**
  - Click-to-select objects in 3D viewer with visual highlight (yellow-gold color)
  - Real-time object information display in Inspector panel
  - Object code extraction supporting both 3-segment (object) and 2-segment (room) formats
  - Smart pattern matching for object codes from multiple naming conventions

- **Backend API Server (`scripts/api_server.py`)**
  - HTTP REST API for path resolution and file access
  - Endpoints:
    - `GET /api/resolve-object?code=<object_code>` - Resolve object paths
    - `GET /api/resolve-room?code=<room_code>` - Resolve room paths
    - `GET /api/download-file?path=<file_path>` - Stream file downloads
    - `GET /health` - Health check endpoint
  - Integration with `ai_api.py`'s PathIndex system
  - CORS support for development
  - Security: Path validation to prevent directory traversal

- **Source File Download Functionality**
  - "Confirm Object" button: Query backend API for source file metadata
  - "Download Object" button: Direct download of original .ply files from `/output`
  - Automatic path resolution (tries `cluster_path` then `shell_path`)
  - Server-side file streaming for large PLY files
  - One-click workflow: auto-resolve and download in single action

- **Development Automation**
  - `start_dev.sh` / `stop_dev.sh` scripts in `web/pointcloud-viewer/`
  - One-command server startup for both frontend (port 5173) and API (port 8090)
  - Automatic server cleanup before restart in visualization pipeline
  - Background process management with PID tracking
  - Log files: `/tmp/api_server.log` and `/tmp/vite_server.log`

- **Enhanced Visualization Pipeline**
  - `--serve` flag in `prepare_visualization.mjs` now starts both servers
  - Automatic old server termination before starting new instances
  - Project root path detection for cross-directory API server access
  - Complete workflow: data prep → server startup → browser access in one command

### Changed
- **UI Language**: All interface text converted to English
  - Button labels: "Confirm Object", "Download Object", "Clear Selection"
  - Status messages: "Confirming...", "Downloading...", "Object Confirmed"
  - Error messages and console output in English
  - Alert dialogs and tooltips in English

- **Object Selection Workflow**
  - Removed JSON preview display from UI
  - Simplified to two primary actions: Confirm (backend sync) and Download (file retrieval)
  - Console logging preserved for debugging (press F12)
  - Independent button operation (no dependency between Confirm and Download)

- **Download Button Behavior**
  - Changed from manifest-based to API-based file access
  - Downloads original files from `/output` instead of downsampled viewer files
  - Smart resolution: automatically fetches path if not already confirmed
  - Progress indication: Shows "⏳ Downloading..." during API calls

### Fixed
- **Path Resolution**: Corrected project root calculation in ESM modules
  - Fixed: `__dirname/../..` → `__dirname/../../..` for viewer scripts
  - Proper handling of `web/pointcloud-viewer/scripts/` directory structure
  - API server path correctly resolved across different execution contexts

- **TypeScript Type Safety**
  - Fixed union type handling for `ObjectInfo | RoomInfo`
  - Added proper type guards using `'cluster_path' in info` checks
  - Eliminated compile errors in object selection logic

### Technical Details

**New Files:**
- `scripts/api_server.py` - Backend HTTP server (254 lines)
- `web/pointcloud-viewer/start_dev.sh` - Unified server startup script
- `web/pointcloud-viewer/stop_dev.sh` - Server cleanup script
- `web/pointcloud-viewer/scripts/start_servers.mjs` - Node.js server orchestration (unused, replaced by shell scripts)

**Modified Files:**
- `web/pointcloud-viewer/src/viewer/PointCloudView2.tsx` - Selection and download logic
- `web/pointcloud-viewer/src/utils/api.ts` - API client functions
- `web/pointcloud-viewer/scripts/prepare_visualization.mjs` - Auto server management
- `web/pointcloud-viewer/package.json` - Added `dev:api` and `dev:all` scripts

**API Response Format:**
```json
{
  "success": true,
  "data": {
    "object_code": "0-7-12",
    "class": "couch",
    "cluster_path": "/absolute/path/to/0-7-12_couch_cluster.ply",
    "uobb_path": "/absolute/path/to/0-7-12_couch_uobb.ply",
    "mesh_path": "/absolute/path/to/0-7-12_couch_mesh.ply",
    "csv_data": { /* CSV row fields */ }
  }
}
```

**Security Features:**
- Path validation: Only allows downloads from project directory
- CORS: Wildcard allowed for development (should restrict in production)
- File streaming: Prevents memory overflow for large files (8KB chunks)
- Error handling: Graceful degradation with user-friendly messages

## 1.2.1 / 2025-10-24

### Added
- **UOBB Computation: Convex Hull + Rotating Calipers Algorithm**
  - Replaced PCA-based UOBB with strict optimal algorithm using convex hull (Andrew's monotone chain) and rotating calipers
  - Implemented in both C++ (`src/geometry/bbox.cpp`) and JavaScript (`web/pointcloud-viewer/scripts/uobb_compute.mjs`)
  - Provides true minimum area oriented bounding boxes in XY plane
  - Automatic UOBB generation from downsampled point clouds in visualization pipeline

- **Viewer: Multi-Rooms Mode UOBB Display**
  - Multi-rooms mode now displays UOBBs for all room shells
  - UOBBs computed on-the-fly from downsampled point clouds
  - All UOBBs grouped together in Layers panel for easier management

- **Viewer: Enhanced Display Names**
  - Improved naming conventions following `ai_api.py` path resolution rules:
    - Shell: `room_shell (room_id: 0-7)`
    - Shell UOBB: `room_uobb (room_id: 0-7)`
    - Cluster objects: `chair (object_id: 0-7-3)`, `couch (object_id: 0-7-12)`
  - Room code properly passed through visualization pipeline for accurate display names

- **Configuration: Viewer Parameters in YAML**
  - Added viewer-specific parameters to `data/configs/default.yaml`:
    - `viewer_downsample_ratio`: 0.2 (cluster downsampling)
    - `viewer_downsample_ratio_shell`: 0.05 (shell downsampling)
    - `viewer_voxel_size`: null (optional spatial downsampling)
    - `viewer_shell_no_color`: false (gray rendering option)
    - `viewer_point_size`: 3 (default point size)
    - `viewer_uobb_opacity`: 0.3 (UOBB transparency)
    - `viewer_uobb_color`: [30, 144, 255] (dodger blue)
    - `viewer_shell_color`: [180, 180, 180] (gray)

### Changed
- **UOBB Algorithm**: Complete replacement from approximate PCA to exact optimal solution
  - XY projection → 2D convex hull → rotating calipers → minimum area rectangle
  - Z-axis range preserved from original point cloud
  - Binary PLY format (422 bytes per UOBB mesh)

- **Visualization Pipeline**: Enhanced `downsample_and_prepare_room.mjs`
  - Added `roomCode` parameter for proper display name generation
  - Automatic UOBB computation integrated into downsampling workflow
  - Shell detection logic for multi-rooms mode

- **UOBB Grouping**: Unified grouping in Layers panel
  - Changed from individual room codes to unified `{name}_uobbs` group
  - All UOBBs for a visualization appear under single collapsible group

### Improved
- **Documentation: Path Resolution**
  - Comprehensive path resolution documentation in `docs/AI_API.md`
  - Detailed explanation of indexing mechanisms:
    - File naming patterns (object-level, room-level, CSV)
    - Parsing logic (object code extraction, kind detection, room inference)
    - Shell detection rules (2-part vs 3-part naming)
  - JSON output examples for all resolution methods
  - Edge case explanations (ambiguity handling, object_id=0 convention)

### Fixed
- Regular expression mismatch in shell UOBB generation (tested basename instead of full path)
- Display name issues in room mode (now correctly shows room code instead of output name)

### Technical Details
- **Convex Hull Implementation**: Andrew's monotone chain algorithm (O(n log n))
- **Rotating Calipers**: Optimal O(n) scan of convex hull for minimum area rectangle
- **UOBB Structure**: center, size, yaw, 8 corners, 12 triangular faces (binary PLY)
- **File Locations**: UOBBs generated alongside source files with `_uobb.ply` suffix

### Notes
- All UOBB computations tested with Full House dataset (7 rooms)
- JavaScript UOBB computation preferred over terminal commands for reliability
- Viewer parameter defaults now centralized in configuration file for easier tuning

## 1.2.0 / 2025-10-24

### Added
- **Point Cloud Viewer: Complete Automation System**
  - New `prepare_visualization.mjs` orchestration script with 4 visualization modes:
    - `room`: Complete room with shell + all clusters
    - `clusters`: Selected object clusters by code
    - `multi-rooms`: Multiple room shells for floor layout overview
    - `room-with-objects`: Room shell with selected objects
  - Full integration with `ai_api.py` for semantic path resolution
  - Automatic cleanup with `--clean-all` flag
  - Auto-serve with `--serve` flag for one-command workflow
  - Custom port support via `--port` option

- **Downsampling Enhancements**
  - `--acceptAnyPly` flag for processing non-cluster PLY files (shells, etc.)
  - Multiple `--cluster` arguments properly accumulated into array
  - Absolute path handling fixed for flexible output directories

- **Inspector UI Improvements**
  - Smart title generation from filename patterns (e.g., "Couch 12" from "0-7-12_couch_cluster")
  - Per-object UOBB visibility toggles
  - Global UOBB show/hide control
  - Scene statistics (total points, item count)

- **Documentation**
  - Comprehensive Point Cloud Viewer documentation: `docs/POINTCLOUD_VIEWER.md`
  - Architecture diagrams and data flow visualization
  - Performance optimization guidelines
  - Troubleshooting guide with common issues and solutions

### Changed
- **Viewer Coordinate System**: Switched from GLOBE to CARTESIAN without recentering
  - Preserves original coordinate values for spatial accuracy
  - Auto-calculates zoom based on scene bounds
  - Fixed camera positioning for better initial view

- **Multi-rooms Mode**: Now uses user-specified `--ratio` instead of hardcoded `ratioShell`
  - Allows flexible downsampling for floor overview (e.g., 10% instead of 5%)
  - Fixed path inference bug (removed extra `/results/` layer)

- **NPM Scripts**: Added convenience commands
  - `npm run visualize`: Run automation workflow
  - `npm run visualize:help`: Show usage information

### Removed
- **Deprecated Scripts** (replaced by new automation):
  - `scripts/generate_random_clusters_manifest.py`
  - `scripts/process_remaining_clusters.py`
  - `web/pointcloud-viewer/scripts/process_5_clusters.py`
  - Random mode from visualization (not practical for real use)

### Fixed
- Path handling for `--cluster` arguments in downsample script
- Clusters directory inference from shell path (removed double `/results/` nesting)
- Absolute vs relative path resolution in output directory creation
- Multi-rooms mode now properly accepts shells instead of clusters

### Documentation
- Updated `web/pointcloud-viewer/README.md` with new automation workflow
- Added comprehensive `docs/POINTCLOUD_VIEWER.md` with:
  - Complete API reference
  - Architecture and data flow diagrams
  - Performance optimization guidelines
  - Troubleshooting guide
  - Best practices and workflow patterns

### Notes
- Complete end-to-end workflow now achievable with single command:
  ```bash
  npm run visualize -- --mode room --room 0-7 --name my_room --clean-all --serve
  ```
- All modes tested and verified working with Full House dataset
- Integration with `ai_api.py` provides semantic object/room code resolution

## 1.1.0 / 2025-10-23

### Added
- Web Viewer (deck.gl): New UI toggle "No color (gray)" for the shell layer. When enabled, the viewer renders the shell with a constant gray and does not upload per-vertex color to the GPU (reduces memory/bandwidth).
- Loader: `loadPly(url, { dropColor: boolean })` option to omit COLOR attributes during load.
- Data prep script: `--shellNoColor` flag to write the shell PLY without RGB columns. This reduces file size and speeds up parsing; `label` and `point_id` are still preserved.

### Changed
- Viewer: The shell layer can be rendered colorless to improve interactivity on large rooms (e.g., room_007). The clusters remain colored.
- Docs: `web/pointcloud-viewer/README.md` updated with performance tips and new flags.

### Fixed
- Data prep: Ensured `label` (and `point_id` when present) are preserved in downsampled ASCII PLYs, enabling the "Hide labels 1,3" toggle to work on shell clouds.

### Notes
- For best performance, combine `--shellNoColor` at data generation time with the viewer’s "No color (gray)" toggle. You can also increase shell downsampling (e.g., `--ratioShell 0.05` or add `--voxelShell`) for further gains.

## 1.0.1 / 2025-10-23

### Added
- AI API: Room-level path resolution by room code (e.g., `0-7`). In a single query it returns:
  - The room CSV path
  - The room shell copy (e.g., `0-7-0_shell.ply`)
  - The room shell UOBB (e.g., `0-7-0_shell_uobb.ply`)
- CLI: New `resolve-room <floor-room>` subcommand (e.g., `resolve-room 0-7 --json`). The existing `resolve-room-csv <floor> <room>` now also returns `shell` and `shell_uobb` paths.

### Changed
- pcg_room: For files detected as shell clouds (filename contains "shell"), copy the original input `shell*.ply` to the output directory next to the corresponding `_shell_uobb.ply`. Naming and location example:
  - Input: `data/rooms/Full House/floor_0/room_007/shell_007.ply`
  - Output: `output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply` and `0-7-0_shell_uobb.ply`

### Notes
- Copying the raw shell cloud alongside its computed UOBB improves traceability.
- Documentation: `docs/AI_API.md` updated to describe room-level resolution and the new/extended CLI commands.

## 1.0.0 / 2025-10-22

### Fixed
- AI API: `CLR` head code now auto-builds `pcg_color` on first use (parity with `RCN`/`VOL`/`ARE`/`BBD`). Previously, `CLR` failed with “Missing executable” instead of triggering the first-run build.

### Clarified
- AI API: `VOL` supports auto-reconstruction by default when a mesh is missing. If invoked with an `object_code` that only has a cluster, `VOL` calls `RCN` first and then computes volume. Use `--no-auto-recon` to disable.

### Notes
- Release readiness: verified head codes end-to-end with JSON outputs enabled and CMake Release build. No changes required in `VOL` implementation; behavior verified via `python3 scripts/ai_api.py VOL --object <object_code> --json`.

## 2025-10-20 / 0.9.1

### Fixed / Improved
- AI API: Added automatic CMake build on first use. When a head code (e.g., ARE/VOL/RCN/BBD/CLR) needs a missing executable, the dispatcher configures and builds targets into `build/` automatically.
- New CLI: `python3 scripts/ai_api.py BUILD [--reconfigure]` to build explicitly or force a reconfigure.
- New CLI: `python3 scripts/ai_api.py check-env --json` shows availability of executables and key paths.

### Docs
- Updated `docs/AI_API.md` with installation, first-run (auto-build), manual build options, and troubleshooting.
- Updated README with quick examples for `check-env`, `BUILD`, and first-run behavior.

## 2025-10-20 / 0.9.0

### Added
- `pcg_area` CLI (CGAL): computes mesh surface area and reports closedness. Works for open and closed meshes; emits JSON when `json_output: true`.
- AI API head code `ARE`: resolves mesh by `object_code` or filename, auto-reconstructs if needed, and returns `{ mesh, closed, area }`.

### Changed
- README and AI_API docs updated to include `ARE` and `pcg_area` usage and JSON schemas.

### Notes
- This release finalizes feature additions for the 0.9 series; subsequent 0.9.x will focus on fixes and polish only.

## 2025-10-20 / 0.9.0-alpha.4

### Changed
- Reconstruction output filenames now encode the method:
  - Poisson: `<object_code>_<class>_mesh_possion.ply`
  - AF:      `<object_code>_<class>_mesh_af.ply`
  - Legacy `<object_code>_<class>_mesh.ply` remains recognized by the AI API.
- Volume computation is now performed only for closed meshes. For open meshes, `pcg_volume` skips volume (JSON `volume: null`; text shows a skip note). The AI API maps `null` to `0.0` in its VOL JSON for downstream simplicity.

### Fixed
- AI API `RCN` JSON now includes the `method` field inferred from the mesh filename suffix and returns mesh paths under the correct `results/recon/<stem>/` directory.

## 2025-10-20 / 0.9.0-alpha.3

### Added
- Python AI API layer `scripts/ai_api.py` for chatbot/automation workflows:
  - Path resolution by filename, `object_code` (e.g., `0-7-12`), and floor-room CSV.
  - Head-code dispatcher: `RCN` (reconstruct), `VOL` (mesh volume/closedness), `CLR` (dominant color), `BBD` (bbox distance between two objects).
  - `check-env` command to verify executables; `--json` option for CLI responses.
  - `--filename` accepts absolute/relative paths in addition to names under `output/`.
- Global structured JSON output mode:
  - New config toggle `json_output: true` in `data/configs/default.yaml`.
  - C++ apps emit JSON to stdout when enabled: `pcg_reconstruct`, `pcg_volume`, `pcg_color`, `pcg_bbox`.
- Documentation updates:
  - `docs/AI_API.md` explains head codes, usage examples, and JSON schemas.
  - README now includes AI API quick-start and JSON output instructions.

### Changed
- Standardized console outputs to English-only for machine readability.
- `pcg_bbox` exposes three modes (compute/gen/point) and supports JSON output when `json_output` is true.

### Fixed
- Addressed a duplicate variable declaration and lambda regression in `src/apps/pcg_bbox.cpp`.

### Notes
- Multi-item operations (e.g., reconstruction across many clusters) stream one JSON object per item to stdout.
- `ai_api.py` prefers parsing tool JSON and falls back to legacy text where necessary.
# Changelog

All notable changes to this project are documented here. This log focuses on the recent work to preserve color in point clouds, standardize outputs, and introduce dominant-color analysis.

## 2025-10-20 / 0.9.0-alpha.2

### Added
- `pcg_bbox` standalone CLI: compute the centers of two UOBB PLYs, the vector from the first to the second, and the Euclidean distance between the centers.
- `gen` subcommand for `pcg_bbox`: `pcg_bbox gen <out.ply> cx cy cz lx ly lz yaw_deg` for quickly generating a test UOBB PLY (Z-up; yaw in degrees).
- `point` subcommand for `pcg_bbox`: `pcg_bbox point x y z <bbox_uobb.ply>` to compute the vector and distance from a user-provided point to the bbox center.

### Changed
- Merged the former test generator `pcg_bbox_gen` into `pcg_bbox` as the `gen` subcommand to reduce maintenance and simplify usage.
- Docs: README updated with `pcg_bbox` usage and examples.

### Removed
- Deprecated and removed the standalone `pcg_bbox_gen` executable and source.

### Notes
- `pcg_bbox` compute mode remains simple: provide two `*_uobb.ply` files exported by `pcg_room`; the center is computed by averaging the box vertices (equals the geometric center). Output format is four lines: `center1`, `center2`, `vector_1_to_2`, `distance`.

## 2025-10-18 / 0.9.0-alpha.1

### Added
- Color-preserving PLY IO (XYZRGB): Load and export point clouds with RGB attributes; backward compatible with XYZ-only PLYs.
- Standardized results layout under `results/` for each room, with unified filenames.
- CSV schema extended: `object_code`, `class`, `object_id`, `room_id`, `floor_id` added.
- `pcg_color` standalone CLI for dominant color analysis of clusters/PLYs.
- Color GMM core library (diagonal, weighted EM; k-means++ init; log-sum-exp; predict API).

### Changed
- `pcg_room` clustering/export: per-cluster PLY exports now keep vertex colors and follow the naming pattern `<object_code>_<class>_{cluster|uobb|mesh}.ply`.
- All outputs consolidated beneath `<room>/results/` (previous ad-hoc directories removed).
- Default configuration updated to a single perceptual decision rule in Lab: keep components if ΔE*76 ≥ `color_deltaE_keep` (20 by default).
- `pcg_reconstruct` mesh naming aligned with the new pattern; one mesh per cluster (accepted Poisson or AF fallback).

### Removed
- Deprecated experimental apps and artifacts: `pcg_api`, synthetic generators, and temporary test data.
- Obsolete parameter `color_deltaE_merge` (replaced by a single `color_deltaE_keep` threshold).

### Fixed
- Loss of color during IO and exports due to XYZ-only handling. Introduced RGB-aware readers/writers and switched internal point type to `PointXYZRGB` where relevant.
- Inconsistent directory structure and filenames across modules. Unified under `results/` and standardized naming/CSV.

### Notes
- Dominant-color method: force K=3 in RGB, filter components by minimum weight and per-channel stddev (≤ `color_max_stddev`), convert remaining centroids to CIELab (D65), merge visually similar colors using ΔE*76 < `color_deltaE_keep`. If all components are filtered, the app reports `M=0` (no dominant color).
- `pcg_color` prints a concise summary: final component count `M` and for each component its weight, mean RGB, and variance.

