# 2D to 3D Point Cloud Masking Guide

## Overview

This guide explains how to run the 2D to 3D point cloud masking functionality using the SAM2-based segmentation system. The system allows you to select regions in panoramic images and automatically extract corresponding 3D point clouds.

## Prerequisites

### Python Environment
- Python 3.13+ (tested with Python 3.13.5)
- Virtual environment: miniconda3

### Required Packages
```bash
pip3 install flask flask-cors streamlit streamlit-js-eval
pip3 install torch torchvision
pip3 install laspy plyfile
pip3 install hydra-core omegaconf iopath
pip3 install numpy pillow requests psutil
```

## System Architecture

The system consists of three main components:

1. **Flask Bridge Server** (`bridge_server_final.py`) - Port 5056
   - Handles click coordinate collection
   - Triggers SAM2 segmentation
   - Manages 2D-to-3D point cloud filtering

2. **Streamlit GUI** (`GUI_streamlit.py`) - Port 8503
   - Interactive web interface for panorama visualization
   - Click-based region selection

3. **SAM2 Processing Backend** (`SAM23D/`)
   - SAM2 model for image segmentation
   - 2D-to-3D coordinate transformation
   - Point cloud filtering

## Running the System

### Step 1: Start the Flask Bridge Server

```bash
cd GUI
python3 bridge_server_final.py
```

The server will start on `http://localhost:5056` and display available endpoints.

### Step 2: Start the Streamlit GUI

In a separate terminal:

```bash
cd GUI
streamlit run GUI_streamlit.py --server.port 8503
```

The GUI will open at `http://localhost:8503` in your default browser.

### Step 3: Using the Masking Functionality

1. **Select a Panorama Image**
   - The GUI will display available panoramic images from `extracted_data/images/`
   - Select the panorama you want to work with

2. **Click to Define Region**
   - Click 5 points on the panorama to define the region of interest
   - Points are collected and displayed in real-time

3. **Automatic Processing**
   - After the 5th click, the system automatically:
     - Runs SAM2 segmentation to generate a mask
     - Projects the 2D mask into 3D space
     - Filters the point cloud based on the mask
     - Saves the result as a `.ply` file

4. **View Results**
   - The segmentation overlay is displayed in the GUI
   - Filtered point cloud is saved to `data/lm2pcg_data/floor_X/room_XXX/filtered_outputs/`

## Data Structure

### Input Data

The system expects the following directory structure:

```
GUI/
├── extracted_data/
│   ├── images/              # Panoramic images (.jpg)
│   └── metadata/            # Camera pose JSON files
├── data/
│   └── lm2pcg_data/
│       └── floor_X/
│           └── room_XXX/
│               ├── combined_XXX.ply    # Input point cloud (PRIMARY)
│               ├── combined1_XXX.ply   # Fallback option 1
│               ├── shell_XXX.ply       # Fallback option 2
│               └── panorama_images.jpg
```

### Point Cloud Priority

The system searches for point cloud files in the following order:

1. **`combined_*.ply`** (Primary) - Combined semantic point cloud
2. **`combined1_*.ply`** (Fallback 1) - Alternative combined format
3. **`shell_*.ply`** (Fallback 2) - Room shell point cloud

### Output Data

Filtered point clouds are saved to:

```
data/lm2pcg_data/floor_X/room_XXX/filtered_outputs/
└── [panorama_name]_test.ply
```

Segmentation masks are saved to:

```
SAM23D/outputs/
├── [panorama_name]_overlay.png      # Visualization overlay
└── [panorama_name]_binary_mask.png  # Binary mask
```

## Example Workflow

For panorama image `00028_de skatting 81_2025-06-20_15.08.22_G11-0265.jpg`:

1. **Input Point Cloud**: `combined_003.ply` (33MB)
   - Full path: `GUI/data/lm2pcg_data/floor_0/room_003/combined_003.ply`

2. **Camera Pose**: `00028_de skatting 81_2025-06-20_15.08.22_G11-0265.json`
   - Full path: `GUI/extracted_data/metadata/00028_de skatting 81_2025-06-20_15.08.22_G11-0265.json`

3. **Output Point Cloud**: `00028_de skatting 81_2025-06-20_15.08.22_G11-0265_test.ply`
   - Full path: `GUI/data/lm2pcg_data/floor_0/room_003/filtered_outputs/00028_de skatting 81_2025-06-20_15.08.22_G11-0265_test.ply`

## Technical Details

### SAM2 Model
- Model: `sam2.1_hiera_large.pt`
- Config: `sam2.1_hiera_l.yaml`
- Location: `SAM23D/SAM2/checkpoints/`

### Coordinate Transformation
The system performs the following transformations:

1. **User Clicks** → 2D pixel coordinates in panorama
2. **SAM2 Segmentation** → 2D binary mask
3. **3D Projection** → Transform point cloud to camera frame
4. **Ray Casting** → Project 3D points to panorama coordinates
5. **Mask Filtering** → Select points inside the 2D mask
6. **Output** → Filtered 3D point cloud

### Camera Model
- Camera position and rotation from pose JSON
- Quaternion rotation representation
- Equirectangular panorama projection

## Troubleshooting

### Server Won't Start
- Check if ports 5056 (Flask) or 8503 (Streamlit) are already in use
- Kill existing processes: `lsof -ti:5056 | xargs kill -9`

### No Panoramas Found
- Verify panoramas exist in `GUI/extracted_data/images/`
- Check file extensions (.jpg, .jpeg, .png)

### Point Cloud Not Found
- Ensure `combined_*.ply` files exist in room directories
- Check directory structure matches expected format

### Missing Dependencies
- Install SAM2 requirements: `pip3 install hydra-core omegaconf iopath`
- Verify torch installation: `python3 -c "import torch; print(torch.__version__)"`

## API Endpoints

### Flask Bridge Server (Port 5056)

- **POST** `/click` - Save click coordinate
- **POST** `/click/reset` - Clear all saved clicks
- **GET** `/clicks` - List all saved clicks
- **GET** `/get_latest_overlay` - Get most recent SAM2 overlay
- **POST** `/run_sam2` - Manually trigger SAM2 segmentation
- **GET** `/health` - Health check

## Output File Format

The filtered point cloud is saved in PLY format with XYZ coordinates:

```
ply
format binary_little_endian 1.0
element vertex [N]
property float x
property float y
property float z
end_header
[binary data]
```

You can open the `.ply` files with:
- CloudCompare
- MeshLab
- Open3D
- Any PLY-compatible viewer

## Performance Notes

- **Input Point Cloud Size**: ~33MB (combined_003.ply)
- **SAM2 Processing Time**: ~10-15 seconds on CPU
- **Point Cloud Filtering**: ~1-2 seconds
- **Output Point Cloud Size**: Typically 1-5% of input (depending on selected region)

## Contact & Support

For issues or questions, please refer to the project repository or contact the development team.
