# Spatial Understanding GUI

A graphical user interface for spatial understanding and 3D point cloud segmentation using panoramic images and SAM2 (Segment Anything Model 2).

## Overview

This GUI application bridges natural language processing with 3D scans, enabling users to interactively select regions in panoramic images and automatically segment corresponding 3D point clouds.

## Architecture

### Core Components

1. **Streamlit GUI** (`GUI_streamlit.py`)
   - Web-based interface built with Streamlit
   - 360° panorama viewer using Pannellum.js
   - Interactive point selection (up to 5 points)
   - Real-time overlay display of segmentation results
   - Session management and navigation controls

2. **Flask Bridge Server** (`bridge_server_final.py`)
   - REST API server running on `http://localhost:5056`
   - Handles click coordinate conversion (pitch/yaw → pixel)
   - Manages point storage and triggers SAM2 segmentation
   - Associates panoramas with corresponding PLY point cloud files

3. **SAM2 Image Segmentation** (`sam2_predictor.py`)
   - Implements SAM2 for image segmentation
   - Uses SAM2.1 Hiera Large model
   - Generates binary masks and overlay images
   - Automatically triggered when 5 points are selected

4. **Point Cloud Filtering** (`select_points_in_mask.py`)
   - Projects 3D point clouds onto 2D panoramas
   - Filters points based on SAM2-generated masks
   - Outputs filtered point clouds in LAS format
   - Uses camera pose metadata for accurate projection

5. **Coordinate Conversion** (`conversion_2D_3D.py`)
   - Mathematical utilities for coordinate transformations
   - Quaternion to rotation matrix conversion
   - Pixel coordinates to 3D ray directions
   - Supports equirectangular panorama projection

## Linked Modules

### From SAM23D (3D Processing)

The following modules are symbolically linked from the `SAM23D` project:

- **`sam2_predictor.py`** → `../SAM23D/sam2_predictor.py`
  - SAM2 image segmentation
  - Generates binary masks and overlay images

- **`select_points_in_mask.py`** → `../SAM23D/select_points_in_mask.py`
  - Point cloud filtering based on masks
  - Projects 3D points to 2D panoramas

- **`conversion_2D_3D.py`** → `../SAM23D/conversion_2D_3D.py`
  - Coordinate transformation utilities
  - Quaternion and rotation matrix operations

- **`SAM2/`** → `../SAM23D/SAM2/`
  - SAM2 model checkpoints and configurations

- **`sam23d_outputs/`** → `../SAM23D/outputs/`
  - Generated masks and overlay images

### From LM2PCG (Spatial AI)

The following modules are symbolically linked from the `LM2PCG` project:

- **`ai_api_wrapper.py`** → `../LM2PCG/ai_api_wrapper.py`
  - Wrapper for external C++/Python pipeline API
  - Handles volume, color, and distance computations
  - Manages point cloud visualization

- **`mutli_room_agent2.py`** → `../LM2PCG/mutli_room_agent2.py`
  - Spatial AI agent for multi-room queries
  - Integrates with Azure OpenAI for natural language processing
  - Manages room database interactions

- **`room_database.py`** → `../LM2PCG/room_database.py`
  - SQLite database for spatial data management
  - Stores floor/room hierarchy, geometric objects, and metadata
  - Provides query interface for spatial information

- **`enrich_room_types.py`** → `../LM2PCG/enrich_room_types.py`
  - Automatic room type classification using vision models
  - Enriches database with semantic room information
  - Uses Azure OpenAI vision API

- **`spatial_rooms.db`** → `../LM2PCG/spatial_rooms.db`
  - Spatial database containing room and object information

- **`scripts/`** → `../LM2PCG/scripts/`
  - Helper scripts for API interactions and batch processing

## Data Structure

```
GUI/
├── data/output/
│   ├── floor_0/
│   └── floor_1/
│       └── room_*/
│           ├── panorama images (.jpg/.png)
│           └── combined1_*.ply (point cloud files)
│
├── extracted_data/
│   ├── images/              # Panoramic images
│   └── metadata/            # Camera pose JSON files
│       └── *.json          # {translation: {x,y,z}, rotation: {x,y,z,w}}
│
├── outputs/                 # SAM2 outputs
│   ├── *_overlay.png       # Segmentation overlays
│   └── *_binary_mask.png   # Binary masks
│
└── filtered_outputs/        # Filtered point clouds
    └── *_test.las          # LAS format point clouds
```

## Workflow

1. **Start Services**
   ```bash
   # Terminal 1: Start Flask bridge server
   python bridge_server_final.py
   
   # Terminal 2: Start Streamlit GUI
   streamlit run GUI_streamlit.py
   ```

2. **Navigate Panoramas**
   - Click "Virtual Mode" to enter panorama viewer
   - Use Previous/Next buttons to browse panoramas

3. **Select Points**
   - Click "Click mode" to enable point selection
   - Left-click on panorama to add points (max 5)
   - Right-click to reset all points

4. **Automatic Segmentation**
   - SAM2 automatically runs when 5 points are selected
   - Segmentation overlay is displayed on panorama
   - Binary mask is saved for point cloud filtering

5. **Point Cloud Filtering**
   - 3D points are projected onto panorama
   - Points within mask are extracted
   - Filtered point cloud saved as LAS file

## API Endpoints

### Bridge Server (`localhost:5056`)

- `GET /health` - Health check
- `POST /click` - Save click coordinates
  ```json
  {
    "pitch": 10.5,
    "yaw": 45.2,
    "image_filename": "panorama.jpg"
  }
  ```
- `POST /click/reset` - Clear all clicks
- `GET /clicks` - Get stored clicks
- `GET /get_latest_overlay` - Retrieve latest SAM2 overlay
- `POST /run_sam2` - Manually trigger SAM2 segmentation

## Requirements

See `requirements.txt` for Python dependencies:

- **Web Framework**: Flask, Streamlit
- **Deep Learning**: PyTorch, SAM2
- **3D Processing**: laspy, plyfile, numpy
- **Image Processing**: Pillow
- **Configuration**: Hydra, OmegaConf

### SAM2 Model

Required files in `SAM2/` directory:
- `checkpoints/sam2.1_hiera_large.pt`
- `configs/sam2.1/sam2.1_hiera_l.yaml`

## Camera Pose Format

JSON metadata files contain camera poses:

```json
{
  "name": "panorama_name.jpg",
  "translation": {
    "x": 9.69,
    "y": 8.13,
    "z": 2.07
  },
  "rotation": {
    "x": 0.003,
    "y": 0.005,
    "z": 0.804,
    "w": 0.594
  }
}
```

- **Translation**: Camera position in world coordinates (meters)
- **Rotation**: Quaternion (x, y, z, w) representing camera orientation

## Features

### Implemented ✅
- 360° panorama viewer with interactive navigation
- Multi-point selection interface (up to 5 points)
- Automatic SAM2 image segmentation
- 3D point cloud filtering based on 2D masks
- Real-time overlay display
- Session management

### Planned 🚧
- AI chat interface for spatial queries
- Multi-room spatial analysis
- Automatic room type classification
- Natural language query processing

## Technical Details

### Coordinate Systems

- **World Coordinates**: Right-handed, Z-up coordinate system
- **Camera Coordinates**: X=forward, Y=right, Z=up
- **Panorama Coordinates**: Equirectangular projection
  - Longitude: [-π, π], 0 at center
  - Latitude: [-π/2, π/2], 0 at horizon

### Point Cloud Projection

1. Transform world points to camera frame using pose quaternion
2. Convert to spherical coordinates (r, θ, φ)
3. Map to equirectangular coordinates (u, v)
4. Apply mask-based filtering
5. Export filtered points as LAS

## Development

### Project Structure

```
GUI/
├── GUI_streamlit.py              # Main GUI application
├── bridge_server_final.py        # Flask API server
├── requirements.txt              # GUI dependencies
│
├── sam2_predictor.py            # → SAM23D (symlink)
├── select_points_in_mask.py     # → SAM23D (symlink)
├── conversion_2D_3D.py          # → SAM23D (symlink)
├── SAM2/                        # → SAM23D (symlink)
├── sam23d_outputs/              # → SAM23D/outputs (symlink)
│
├── ai_api_wrapper.py            # → LM2PCG (symlink)
├── mutli_room_agent2.py         # → LM2PCG (symlink)
├── room_database.py             # → LM2PCG (symlink)
├── enrich_room_types.py         # → LM2PCG (symlink)
├── spatial_rooms.db             # → LM2PCG (symlink)
└── scripts/                     # → LM2PCG (symlink)
```

### Symbolic Links

This project uses symbolic links to share modules with related projects. The links are relative and should work correctly if all repositories are in the same parent directory:

```
Documents/GitHub/
├── GUI/          # This project (front-end + back-end)
├── SAM23D/       # 3D processing (SAM2 + point cloud filtering)
└── LM2PCG/       # Spatial AI (multi-room agent + database)
```

**Directory Organization:**
- **GUI**: Contains only front-end (Streamlit) and back-end (Flask) code
- **SAM23D**: Contains all image segmentation and 3D point cloud processing
- **LM2PCG**: Contains spatial AI agent, room database, and API wrapper

## Troubleshooting

### Bridge Server Not Found
Ensure Flask server is running:
```bash
python bridge_server_final.py
```

### SAM2 Model Not Found
Download SAM2 checkpoint to:
```
SAM2/checkpoints/sam2.1_hiera_large.pt
```

### Point Cloud Not Found
Verify data structure:
- Panoramas in `data/output/floor_*/room_*/`
- PLY files named `combined1_*.ply`
- Metadata JSON in `extracted_data/metadata/`

### Import Errors for Linked Modules
Verify symbolic links are correct:
```bash
ls -la *.py | grep "^l"
```

If links are broken, recreate them:
```bash
ln -sf ../LM2PCG/ai_api_wrapper.py ai_api_wrapper.py
ln -sf ../LM2PCG/mutli_room_agent2.py mutli_room_agent2.py
ln -sf ../LM2PCG/room_database.py room_database.py
ln -sf ../LM2PCG/enrich_room_types.py enrich_room_types.py
ln -sf ../LM2PCG/spatial_rooms.db spatial_rooms.db
ln -sf ../LM2PCG/scripts scripts
```

## License

This project is part of the LM2PCG spatial understanding system.

## Acknowledgments

- **SAM2**: Meta's Segment Anything Model 2
- **Pannellum**: 360° panorama viewer
- **Streamlit**: Interactive web application framework
