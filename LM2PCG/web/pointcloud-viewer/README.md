````markdown
# Point Cloud Viewer (deck.gl + loaders.gl)

Web viewer for local PLY point clouds with picking. Includes performance-oriented options for large rooms (shell color stripping, label-based filtering, downsampling).

## Dev quickstart

1) Ensure Node.js >= 18
2) Install deps and run the dev server

```bash
cd web/pointcloud-viewer
npm install
npm run dev
```

## 🚀 Automated Visualization Workflow

The new `prepare_visualization.mjs` script provides a complete automated workflow that integrates with the `ai_api.py` system:

### Quick Examples

```bash
# Show all options
npm run visualize:help

# Visualize entire room (shell + all clusters)
npm run visualize -- --mode room --room 0-7 --name room_007

# Visualize selected objects by code
npm run visualize -- --mode clusters --objects "0-7-12,0-7-15,0-7-3" --name selected_furniture

# Visualize multiple room shells
npm run visualize -- --mode multi-rooms --rooms "0-7,0-6,0-5" --name floor_0_overview

# Room with selected objects
npm run visualize -- --mode room-with-objects --room 0-7 --objects "0-7-12,0-7-15" --name room_007_furniture

# Clean all previous outputs and auto-start server
npm run visualize -- --mode room --room 0-7 --name room_007 --clean-all --serve
```

### Supported Modes

1. **`room`** - Visualize entire room (shell + all clusters)
   - Required: `--room <floor>-<room>` (e.g., `0-7`)
   - Automatically finds shell and clusters directory using `ai_api.py`

2. **`clusters`** - Visualize selected clusters
   - Option A: `--objects "0-7-12,0-7-15"` (object codes, resolved via `ai_api.py`)
   - Option B: `--clusters "path/to/file.ply"` (direct file paths)

3. **`multi-rooms`** - Visualize multiple room shells
   - Required: `--rooms "0-7,0-6,0-5"` (comma-separated room codes)
   - Visualizes only room shells without internal clusters - useful for floor layout overview

4. **`room-with-objects`** - Room shell with selected objects
   - Required: `--room <code>`, `--objects <codes>`
   - Combines room context with specific object clusters

### Common Options

- `--name <string>` - Output name (required, used for data folder and manifest)
- `--ratio <float>` - Cluster downsample ratio (default: 0.2)
- `--ratioShell <float>` - Shell downsample ratio (default: 0.05)
- `--voxel <float>` - Voxel size for spatial sampling (optional)
- `--shellNoColor` - Strip color from shell (gray rendering)
- `--no-clean` - Skip cleaning previous outputs (default: auto-clean enabled)
- `--clean-all` - Clean ALL previous outputs before preparation
- `--serve` - Automatically start dev server after preparation
- `--port <number>` - Dev server port (default: 5173, only with --serve)

### What it does

1. **Cleans cache** - Removes previous `.ply` files and manifests for the given name
2. **Resolves paths** - Uses `ai_api.py` to find shells, clusters, UOBBs by object/room codes
3. **Downsamples** - Calls `downsample_and_prepare_room.mjs` with appropriate parameters
4. **Generates manifest** - Creates unified manifest with proper item metadata
5. **Outputs URL** - Prints viewer URL with manifest parameter

### Integration with ai_api.py

The script fully integrates with the existing `ai_api.py` path resolution system:

- `resolve-object <code>` - Finds clusters, UOBBs, meshes by object code
- `resolve-room <code>` - Finds room CSV, shell, shell_uobb by room code
- Supports the same object/room code format (`<floor>-<room>-<object>`)

---

## 🔧 Manual Preparation (Advanced)

3) Prepare data with downsampling (recommended)

Use the provided script to copy shell and downsample clusters, then generate a manifest:

```bash
# show usage
npm run prepare:room

# example: prepare room_007 (downsample shell to 5%, clusters to 20%)
node scripts/downsample_and_prepare_room.mjs \
	--room room_007 \
	--shell "../../output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply" \
	--clustersDir "../../output/Full House/floor_0/room_007/results/filtered_clusters" \
	--ratio 0.2 \
	--ratioShell 0.05 \
	--outDir public/data \
	--clean  # ensure previous downsampled outputs for this room are cleared

# optional: write shell without RGB columns (smaller file, faster parsing)
node scripts/downsample_and_prepare_room.mjs \
	--room room_007 \
	--shell "../../output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply" \
	--clustersDir "../../output/Full House/floor_0/room_007/results/filtered_clusters" \
	--ratio 0.2 \
	--ratioShell 0.05 \
	--outDir public/data \
	--shellNoColor
```

Outputs:
- PLYs in `public/data/<room>/shell.ply` and `public/data/<room>/clusters/*.ply`
- Manifest in `public/manifests/<room>.json` (unified schema with `items`)

4) Open the app

- Dev: http://localhost:5173/?manifest=/manifests/room_007.json


## Manifest (unified)

The viewer expects a unified manifest with an `items` array that can represent shells, clusters, or arbitrary objects across one or more rooms.

Minimal schema:

```
{
	"version": 1,
	"title": "Optional title",
	"defaults": { "pointSize": 3 },
	"items": [
		{
			"id": "unique-id",
			"name": "Display name",
			"kind": "pointcloud",
			"role": "shell|cluster|object",
			"source": { "url": "/data/room_007/shell.ply" },
			"group": "room_007",            // optional, used for UI grouping
			"visible": true,
			"style": {
				"pointSize": 3,
				"colorMode": "file|constant",
				"color": [180, 180, 180]       // when colorMode = constant
			},
			"filters": {
				"labelInclude": [2,4],         // optional
				"labelExclude": [1,3]
			}
		}
	]
}
```

Examples are provided under `public/manifests/examples/`.

## What it does

- Loads arbitrary `items` (shells/clusters/objects) with grouping and per-item visibility
- Optional constant-gray rendering by item style (`colorMode = constant`)
- Label-based include/exclude filtering per item (`filters.labelInclude/labelExclude`)
- Picking: click a point to see its index and attributes (`point_id`, `label` where available)

## Performance notes

- For large rooms, consider both:
	- Generating shell without color columns (`--shellNoColor`), and
	- Enabling the viewer toggle "No color (gray)" for the shell layer.
- Downsample shell more aggressively than clusters (e.g., `--ratioShell 0.02` and `--ratio 0.2`).
- Prefer voxel downsampling first for spatial uniformity, then apply a ratio if needed.

## Notes

- Coordinates are CARTESIAN with `OrbitView` (units: meters recommended)
- PLY attributes are auto-detected; if files use different field names, adjust `src/loaders/ply.ts`
- For massive inputs, prefer voxel downsampling first, then ratio

## Production hosting

- Build with `npm run build`
- Serve `dist/` and make sure `/data` is reachable on the same origin
