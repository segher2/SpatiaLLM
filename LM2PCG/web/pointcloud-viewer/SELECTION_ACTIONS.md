# Point Cloud Selection & Actions

## Overview

The viewer now supports selecting point clouds and performing backend operations on them. When you click on a point, the entire point cloud is highlighted and detailed information is displayed in the Inspector panel.

## Features

### 1. Visual Highlighting
- Click any point in a point cloud
- The entire point cloud turns **bright yellow/gold** to indicate selection
- Other point clouds remain in their original colors

### 2. Information Display

The Inspector panel shows:
- **Selected Cloud**: Name, object_code, role, group, viewer file path
- **Backend Data** (automatically fetched):
  - Cluster files (original PLY files in `/output`)
  - UOBB files
  - Mesh files (if reconstructed)
  - Room directory
  - CSV metadata (class, size, center position, etc.)
- **Clicked Point**: Index, point_id, label

### 3. Action Buttons

#### 📋 Export as JSON
Exports complete selection information as JSON and copies to clipboard.

**Output format:**
```json
{
  "timestamp": "2025-10-24T...",
  "viewer_info": {
    "item_id": "cluster-0-7-12_couch_cluster",
    "name": "couch (object_id: 0-7-12)",
    "object_code": "0-7-12",
    "role": "object",
    "group": "room_007_fixed",
    "viewer_file": "/data/room_007_fixed/clusters/0-7-12_couch_cluster.ply"
  },
  "clicked_point": {
    "index": 1234,
    "point_id": 5678,
    "label": 3
  },
  "backend_data": {
    "success": true,
    "found": true,
    "source_files": {
      "clusters": ["/Users/.../output/Full House/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_cluster.ply"],
      "uobbs": ["/Users/.../output/Full House/floor_0/room_007/results/filtered_clusters/couch_007/0-7-12_couch_uobb.ply"],
      "meshes": [],
      "room_dir": "/Users/.../output/Full House/floor_0/room_007",
      "csv_path": "/Users/.../output/Full House/floor_0/room_007/room_007.csv"
    },
    "metadata": {
      "object_code": "0-7-12",
      "object_id": "12",
      "room_id": "7",
      "floor_id": "0",
      "class": "couch",
      "file": "couch_007.ply",
      "cluster_id": "0",
      "center_x": "3.881779",
      "center_y": "2.327255",
      "center_z": "0.450288",
      "size_x": "2.004373",
      "size_y": "1.926965",
      "size_z": "0.900277",
      "yaw_rad": "-1.980656"
    }
  }
}
```

#### 🔨 Reconstruct Mesh
Prepares reconstruction command for the selected cluster.

**Output:**
```json
{
  "action": "reconstruct_mesh",
  "object_code": "0-7-12",
  "cluster_file": "/Users/.../output/.../0-7-12_couch_cluster.ply",
  "timestamp": "2025-10-24T..."
}
```

**Backend command (example):**
```bash
./build/pcg_reconstruct /Users/.../0-7-12_couch_cluster.ply
```

#### 📐 Compute Volume
Prepares volume computation for the selected object.

**Output:**
```json
{
  "action": "compute_volume",
  "object_code": "0-7-12",
  "files": ["/Users/.../0-7-12_couch_cluster.ply"],
  "timestamp": "2025-10-24T..."
}
```

**Backend command (example):**
```bash
./build/pcg_volume --object_code 0-7-12
```

#### 🎨 Analyze Color
Prepares color analysis for the selected cluster.

**Output:**
```json
{
  "action": "analyze_color",
  "object_code": "0-7-12",
  "cluster_file": "/Users/.../0-7-12_couch_cluster.ply",
  "timestamp": "2025-10-24T..."
}
```

**Backend command (example):**
```bash
./build/pcg_color /Users/.../0-7-12_couch_cluster.ply
```

## Usage Workflow

### Basic Selection & Export

1. **Start services:**
   ```bash
   # Terminal 1: Start API server
   cd /Users/jacksonye/Documents/GitHub/Spariallm_geopipieline
   python3 scripts/api_server.py --port 8001
   
   # Terminal 2: Start viewer
   cd web/pointcloud-viewer
   npm run dev
   ```

2. **Open viewer:**
   ```
   http://localhost:5173/?manifest=/manifests/room_007_fixed.json
   ```

3. **Select a point cloud:**
   - Click on any furniture or object in the viewer
   - Wait for highlight and info to load (~1 second)

4. **Export selection:**
   - Click "📋 Export as JSON" button
   - JSON is copied to clipboard
   - Check browser console for full output

5. **Use the data:**
   - Paste JSON into your application
   - Extract `backend_data.source_files.clusters[0]` for file path
   - Use `viewer_info.object_code` for backend operations

### Performing Operations

1. **Select object** (e.g., a couch)
2. **Click operation button:**
   - 🔨 Reconstruct Mesh - if you want 3D mesh
   - 📐 Compute Volume - if you need volume/area
   - 🎨 Analyze Color - if you need color analysis
3. **Check console** for prepared command
4. **Copy command** and run in backend

### Integration with Backend

The action outputs can be directly used with the backend API:

```python
# Example: Using exported data with ai_api.py
from scripts.ai_api import Dispatcher

d = Dispatcher()

# Reconstruct mesh
mesh_path = d.op_RCN(object_code="0-7-12")
print(f"Mesh created: {mesh_path}")

# Compute volume
mesh, volume, closed = d.op_VOL(object_code="0-7-12")
print(f"Volume: {volume}, Closed: {closed}")

# Analyze color
color_info = d.op_CLR(object_code="0-7-12")
print(f"Color components: {color_info}")
```

## Room Selection

For room shells (not individual objects):
- Selection works the same way
- `object_code` will be extracted from shell filename (e.g., "0-7-0_shell" → "0-7")
- Room-level operations can be performed
- Backend data will show room directory and CSV path

## API Integration

### Automatic Backend Queries

When you select a point cloud with an `object_code`, the viewer automatically queries:
```
GET http://127.0.0.1:8001/api/resolve-object?code=<object_code>
```

Response includes:
- All cluster files for this object
- UOBB files
- Reconstructed mesh files (if any)
- Room directory
- CSV metadata

### Manual API Usage

You can also use the API directly:

```bash
# Query object
curl 'http://127.0.0.1:8001/api/resolve-object?code=0-7-12'

# Query room
curl 'http://127.0.0.1:8001/api/resolve-room?code=0-7'

# Health check
curl 'http://127.0.0.1:8001/api/health'
```

## Tips

1. **Console is your friend**: All actions output detailed JSON to browser console
2. **Clipboard export**: Use "Export as JSON" for easy copy-paste into scripts
3. **Check backend first**: API queries ensure file paths are correct
4. **Operations are safe**: Buttons only prepare commands, they don't execute automatically
5. **Multiple selections**: Clear selection before clicking another object

## Troubleshooting

### API not responding
- Check if `api_server.py` is running on port 8001
- Look for "Backend Data: Not found in backend" message

### No action buttons
- Make sure object has a valid `object_code` (format: `X-X-X`)
- Check if backend API successfully resolved the object

### Clipboard copy fails
- Check browser console for errors
- Try manually copying from console output

## Future Enhancements

Potential additions:
- Execute operations directly from viewer (with confirmation)
- Batch operations on multiple selections
- Operation history and undo
- Real-time operation status updates
- Custom operation scripts
