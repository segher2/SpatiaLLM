# SAM23D - SAM2 to 3D Point Cloud Pipeline

A pipeline for converting 2D image segmentation masks (from SAM2) to filtered 3D point clouds.

## Overview

This module provides the core functionality for:
1. **Image Segmentation**: Using SAM2 (Segment Anything Model 2) to segment objects in panoramic images
2. **Point Cloud Filtering**: Projecting 3D point clouds onto 2D panoramas and filtering based on segmentation masks
3. **Coordinate Conversion**: Mathematical utilities for transforming between different coordinate systems

## Components

### 1. SAM2 Image Segmentation (`sam2_predictor.py`)

Performs semantic segmentation on panoramic images using Meta's SAM2 model.

**Features:**
- Loads SAM2.1 Hiera Large model
- Accepts user-defined points (up to 5) for guided segmentation
- Generates binary masks and overlay images
- Automatically saves results to `outputs/` directory

**Usage:**
```python
from sam2_predictor import run_sam2_prediction
import numpy as np

# Define 5 points (x, y pixel coordinates)
points = np.array([[100, 200], [150, 250], [200, 300], [250, 350], [300, 400]])
image_filename = "panorama.jpg"

result = run_sam2_prediction(points, image_filename)
# Returns: {success, overlay_path, mask_path, mask_shape, score, image_size}
```

**Model Requirements:**
- Checkpoint: `SAM2/checkpoints/sam2.1_hiera_large.pt`
- Config: `SAM2/configs/sam2.1/sam2.1_hiera_l.yaml`

### 2. Point Cloud Filtering (`select_points_in_mask.py`)

Projects 3D point clouds onto 2D panoramic images and filters points based on SAM2 masks.

**Features:**
- Loads PLY point clouds and camera pose metadata
- Projects 3D points to equirectangular panorama coordinates
- Filters points inside segmentation mask
- Exports filtered point clouds as LAS files

**Usage:**
```python
from select_points_in_mask import filter_point_cloud_with_mask
from pathlib import Path

mask_path = "outputs/panorama_binary_mask.png"
image_stem = "panorama"
base_dir = Path("/path/to/GUI")

las_path = filter_point_cloud_with_mask(mask_path, image_stem, base_dir)
# Returns: path to filtered LAS file
```

**Data Requirements:**
- Binary mask image (from SAM2)
- PLY point cloud file (shell_*.ply or combined_*.ply)
- Camera pose JSON (translation + quaternion rotation)
- Original panoramic image

### 3. Coordinate Conversion (`conversion_2D_3D.py`)

Mathematical utilities for coordinate transformations.

**Functions:**
- `quat_to_matrix(qx, qy, qz, qw)`: Convert quaternion to rotation matrix
- `yaw_only_world_from_cam(R)`: Extract yaw rotation from full rotation matrix
- `pixel_to_ray(u, v, W, H)`: Convert pixel coordinates to 3D ray direction

**Usage:**
```python
from conversion_2D_3D import quat_to_matrix, pixel_to_ray

# Convert quaternion to rotation matrix
R = quat_to_matrix(0.003, 0.005, 0.804, 0.594)

# Convert pixel to ray direction (equirectangular)
ray = pixel_to_ray(2048, 1024, 4096, 2048)  # center pixel
```

## Directory Structure

```
SAM23D/
├── README.md                    # This file
├── sam2_predictor.py           # SAM2 segmentation
├── select_points_in_mask.py    # Point cloud filtering
├── conversion_2D_3D.py         # Coordinate utilities
├── SAM2/                       # SAM2 model files
│   ├── checkpoints/
│   │   └── sam2.1_hiera_large.pt
│   ├── configs/
│   │   └── sam2.1/
│   │       └── sam2.1_hiera_l.yaml
│   └── sam2/                   # SAM2 Python package
└── outputs/                    # Generated masks and overlays
    ├── *_binary_mask.png
    └── *_overlay.png
```

## Installation

### Dependencies

```bash
pip install torch torchvision
pip install pillow numpy
pip install laspy plyfile
pip install hydra-core omegaconf
```

### SAM2 Model

Download the SAM2.1 Hiera Large checkpoint:
```bash
mkdir -p SAM2/checkpoints
# Download sam2.1_hiera_large.pt to SAM2/checkpoints/
```

## Pipeline Workflow

```
User Selects 5 Points on Panorama
           ↓
    [sam2_predictor.py]
    - Load panoramic image
    - Run SAM2 segmentation
    - Generate binary mask
           ↓
    outputs/image_binary_mask.png
           ↓
    [select_points_in_mask.py]
    - Load 3D point cloud (PLY)
    - Load camera pose (JSON)
    - Project 3D points to 2D
    - Filter by mask
           ↓
    filtered_outputs/image_filtered.las
```

## Data Format

### Camera Pose JSON
```json
{
  "name": "panorama.jpg",
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

### Point Cloud (PLY)
Standard PLY format with at minimum:
- Vertex positions (x, y, z)
- Optional: RGB colors, normals

### Binary Mask (PNG)
- Grayscale or single-channel image
- White (255) = inside mask
- Black (0) = outside mask

## Coordinate Systems

### World Coordinates
- Right-handed coordinate system
- Z-axis points up
- Units: meters

### Camera Coordinates
- X-axis: forward (camera viewing direction)
- Y-axis: right
- Z-axis: up
- Origin: camera optical center

### Panorama Coordinates (Equirectangular)
- Width (u): [0, W] → Longitude [-π, π]
- Height (v): [0, H] → Latitude [-π/2, π/2]
- (0, 0) at top-left
- Center facing direction: u = W/2

## Mathematical Details

### Quaternion to Rotation Matrix
```
q = (qx, qy, qz, qw)
R = [1-2(yy+zz)  2(xy-wz)    2(xz+wy)  ]
    [2(xy+wz)    1-2(xx+zz)  2(yz-wx)  ]
    [2(xz-wy)    2(yz+wx)    1-2(xx+yy)]
```

### 3D Point Projection
```
1. Transform to camera frame:
   P_cam = R_cw @ (P_world - t_wc)

2. Convert to spherical coordinates:
   r = ||P_cam||
   lon = -atan2(Y, X)
   lat = asin(Z / r)

3. Map to pixel coordinates:
   u = (lon / 2π + 0.5) * W
   v = (0.5 - lat / π) * H
```

## Integration with GUI

This module is designed to be used by the GUI project via symbolic links:

```bash
# In GUI directory
ln -sf ../SAM23D/sam2_predictor.py sam2_predictor.py
ln -sf ../SAM23D/select_points_in_mask.py select_points_in_mask.py
ln -sf ../SAM23D/conversion_2D_3D.py conversion_2D_3D.py
ln -sf ../SAM23D/SAM2 SAM2
ln -sf ../SAM23D/outputs sam23d_outputs
```

The GUI's Flask bridge server imports these modules to provide:
- Automatic SAM2 segmentation when 5 points are clicked
- Point cloud filtering based on segmentation results
- Base64-encoded overlay images for real-time display

## Performance Notes

- **SAM2 Inference**: ~2-5 seconds on CPU, ~0.5-1 second on GPU
- **Point Cloud Filtering**: Depends on point cloud size
  - 1M points: ~1-2 seconds
  - 10M points: ~10-20 seconds
- **Memory Usage**: 
  - SAM2 model: ~2-4 GB GPU memory
  - Point clouds: ~100 MB per million points

## Troubleshooting

### SAM2 Model Not Found
```
FileNotFoundError: SAM2 checkpoint not found
```
**Solution**: Download sam2.1_hiera_large.pt to `SAM2/checkpoints/`

### Point Cloud Projection Misalignment
```
Warning: No points selected. Check alignment or mask.
```
**Solution**: Verify camera pose JSON matches the panoramic image

### Import Errors
```
ModuleNotFoundError: No module named 'sam2'
```
**Solution**: Ensure SAM2 directory is in Python path (handled automatically by predictor)

## Citation

If you use SAM2 in your research, please cite:
```bibtex
@article{ravi2024sam2,
  title={SAM 2: Segment Anything in Images and Videos},
  author={Ravi, Nikhila and others},
  journal={arXiv preprint arXiv:2408.00714},
  year={2024}
}
```

## License

This module is part of the Spatial Understanding GUI project.
SAM2 is licensed by Meta under Apache 2.0.

## Related Projects

- **GUI**: Front-end and back-end for user interaction
- **LM2PCG**: Multi-room spatial AI agent with database
- **SAM2**: Meta's Segment Anything Model 2 (external dependency)
