# GUI Setup and Launch Guide

This guide will help you set up and run the SpatialLLM GUI for 2D to 3D segmentation.

## Prerequisites

- Python 3.x (avoid Python 3.14+, recommend Python 3.10-3.13)
- macOS, Linux, or Windows with bash/zsh
- At least 2GB of free disk space (for SAM2 model)
- Internet connection (for initial model download)
- Git (for cloning repository and submodules)

## Initial Setup

### 0. Clone the Repository with Submodules

⚠️ **IMPORTANT:** This project uses Git submodules. You must clone with the `--recursive` flag:

```bash
# Option 1: Clone with submodules in one command
git clone --recursive https://github.com/segher2/SpatialLLM.git
cd SpatialLLM
```

If you already cloned without `--recursive`, initialize the submodules:

```bash
# Option 2: Initialize submodules after cloning
cd SpatialLLM
git submodule update --init --recursive
```

**Verify submodules are present:**
```bash
ls -la
# You should see: LM2PCG/, SAM23D/, GUI_SpatialLLM/ directories
```

The project contains three submodules:
- `LM2PCG` - Point cloud processing pipeline
- `SAM23D` - 2D-to-3D segmentation module
- `GUI_SpatialLLM` - Interactive GUI interface

## Quick Start

### 1. Download SAM2 Model

The GUI requires the SAM2.1 Hiera Large model checkpoint. Download it once:

```bash
cd SAM23D/SAM2/checkpoints
curl -L -o sam2.1_hiera_large.pt https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt
```

**Note:** This is a large file (~856MB) and will take a few minutes to download.

### 2. Verify Data Structure

Ensure your data is properly set up. The GUI expects:

```
GUI_SpatialLLM/
├── data/
│   └── lm2pcg_data/  → symlink to ../../LM2PCG/data/rooms/Full House
│       ├── combined.ply  (complete point cloud)
│       ├── floor_0/
│       │   └── room_XXX/
│       │       ├── panorama_images.jpg
│       │       └── shell_XXX.ply
│       └── floor_1/
│           └── ...
└── extracted_data/
    ├── images/
    └── metadata/
        └── *.json (camera pose files)
```

### 3. Install Python Dependencies

Install required packages for the GUI:

```bash
cd GUI_SpatialLLM
pip install -r requirements.txt
```

Also install SAM23D dependencies:

```bash
cd ../SAM23D
pip install -r requirements.txt
```

### 4. Start the Bridge Server

The bridge server handles communication between the GUI and SAM2:

```bash
cd GUI_SpatialLLM
python bridge_server_final.py
```

**Expected output:**
```
Added SAM23D to Python path: ...
Bridge server running on http://localhost:5056
Endpoints:
  POST /click              - Save click coordinates
  POST /click/reset        - Clear all clicks
  ...
 * Running on http://127.0.0.1:5056
```

**Leave this terminal running.** Open a new terminal for the next step.

### 5. Launch the Streamlit GUI

In a new terminal:

```bash
cd GUI_SpatialLLM
streamlit run GUI_streamlit.py
```

**Expected output:**
```
  You can now view your Streamlit app in your browser.

  Local URL: http://localhost:8501
  Network URL: http://xxx.xxx.xxx.xxx:8501
```

### 6. Access the GUI

Open your web browser and navigate to:
```
http://localhost:8501
```

## Using the GUI for 2D to 3D Segmentation

### Workflow

1. **Select a Panorama**: Choose a panoramic image from the dropdown menu
2. **Click 5 Points**: Click exactly 5 points on the object you want to segment in the 360° viewer
3. **Automatic Processing**: 
   - SAM2 automatically runs when 5 points are selected
   - A segmentation mask is generated
   - The overlay appears on the panorama
4. **3D Point Cloud Filtering**: The system automatically:
   - Projects the 2D mask onto the 3D point cloud
   - Filters points that fall within the segmented region
   - Generates a filtered `.las` point cloud file
5. **View Results**: The segmented overlay is displayed on the panorama viewer

### Output Files

Processed results are saved to:

```
SAM23D/outputs/
├── {image_name}_overlay.png       # Visual overlay showing segmentation
├── {image_name}_binary_mask.png   # Binary mask
└── {image_name}_filtered.las      # Filtered 3D point cloud
```

### Tips

- **Point Selection**: Place points around the perimeter of the object you want to segment
- **Reset**: Use the "Reset Clicks" button to start over
- **Multiple Objects**: Process one object at a time for best results
- **Performance**: First run may be slower as SAM2 loads; subsequent runs are faster

## Troubleshooting

### Bridge Server Not Found

**Error Message:** `Bridge Server Not Found. Run it first: python bridge_server_final.py`

**Solution:** Start the bridge server in a separate terminal (see Step 4 above)

### SAM2 Checkpoint Not Found

**Error:** `SAM2 checkpoint not found: .../sam2.1_hiera_large.pt`

**Solution:** Download the model checkpoint (see Step 1 above)

### Combined PLY File Not Found

**Error:** `Combined PLY file not found at .../combined.ply`

**Solution:** 
1. Verify the symlink: `ls -la GUI_SpatialLLM/data/lm2pcg_data`
2. Check source file exists: `ls -lh LM2PCG/data/rooms/Full\ House/combined.ply`
3. If missing, ensure your LM2PCG data is properly set up

### Browser Console Warnings

Warnings like `Unrecognized feature: 'ambient-light-sensor'` or `ComponentRegistry.onMessageEvent` are **normal** and can be ignored. These are browser compatibility messages that don't affect functionality.

### Slow Performance

**Issue:** SAM2 processing takes a long time

**Solutions:**
- First run is always slower (model loading)
- SAM2 uses CPU by default; GPU acceleration can speed this up significantly
- Ensure you have sufficient RAM (8GB+ recommended)

### Port Already in Use

**Error:** `Address already in use`

**Solution:** 
- Kill existing processes: `lsof -ti:5056 | xargs kill` (for bridge server)
- Or: `lsof -ti:8501 | xargs kill` (for Streamlit)
- Then restart the services

## Architecture Overview

The system consists of three main components:

1. **Streamlit GUI** (`GUI_streamlit.py`): Web interface for user interaction
2. **Flask Bridge Server** (`bridge_server_final.py`): Coordinates between GUI and processing modules
3. **Processing Pipeline**:
   - `sam2_predictor.py`: SAM2 image segmentation
   - `select_points_in_mask.py`: 3D point cloud filtering
   - `conversion_2D_3D.py`: Coordinate transformations

### Data Flow

```
User clicks 5 points in GUI
    ↓
Bridge Server receives coordinates
    ↓
SAM2 generates segmentation mask
    ↓
Mask projected onto 3D point cloud
    ↓
Filtered point cloud saved as .las
    ↓
Results displayed in GUI
```

## System Requirements

### Minimum
- CPU: Dual-core processor
- RAM: 8GB
- Storage: 5GB free space
- GPU: Not required (CPU mode works)

### Recommended
- CPU: Quad-core processor or better
- RAM: 16GB
- Storage: 10GB+ free space
- GPU: CUDA-compatible GPU with 4GB+ VRAM (for faster processing)

## Additional Resources

- **SAM2 Documentation**: https://github.com/facebookresearch/segment-anything-2
- **Project README**: See main `README.md` for overall project structure
- **GUI Documentation**: `GUI_SpatialLLM/README.md` for detailed GUI features

## Working with Submodules

### Understanding Submodules

⚠️ **IMPORTANT:** Submodules are **independent Git repositories**. Changes to submodules require special handling.

### Making Changes to Submodules

If you modify code in `LM2PCG/`, `SAM23D/`, or `GUI_SpatialLLM/`:

**Step 1: Commit and push within the submodule**
```bash
# Navigate to the submodule directory
cd GUI_SpatialLLM  # or LM2PCG, or SAM23D

# Check status
git status

# Add and commit changes
git add .
git commit -m "Your commit message"

# Push to the submodule's repository
git push origin main  # or your branch name
```

**Step 2: Update the parent repository**
```bash
# Go back to the parent repository root
cd ..

# The parent repo will detect the submodule has changed
git status
# You'll see: modified:   GUI_SpatialLLM (new commits)

# Commit the submodule pointer update
git add GUI_SpatialLLM
git commit -m "Update GUI_SpatialLLM submodule"

# Push to parent repository
git push origin main  # or your branch name
```

### Pulling Updates from Submodules

When someone else updates a submodule:

```bash
# Pull parent repository
git pull

# Update submodules to match the recorded commits
git submodule update --recursive

# Or, to pull latest changes from submodule branches:
git submodule update --remote --recursive
```

### Common Mistakes to Avoid

❌ **DON'T** commit in parent repo without pushing submodule changes first
- Others won't be able to access the submodule commit you referenced

✅ **DO** follow this order:
1. Push submodule changes
2. Then update and push parent repository

❌ **DON'T** forget to run `git submodule update` after pulling parent repo
- Your local submodules may be out of sync

✅ **DO** use `git pull && git submodule update --recursive` together

## Maintenance

### Updating SAM2 Model

To update to a newer SAM2 model:
1. Download new checkpoint to `SAM23D/SAM2/checkpoints/`
2. Update checkpoint path in `sam2_predictor.py` if needed

### Cleaning Output Files

To clean up generated files:
```bash
rm -rf SAM23D/outputs/*
```

### Stopping Services

To gracefully stop running services:
1. Bridge Server: Press `Ctrl+C` in its terminal
2. Streamlit GUI: Press `Ctrl+C` in its terminal

## Getting Help

If you encounter issues:
1. Check the Troubleshooting section above
2. Review terminal output for error messages
3. Verify all dependencies are installed
4. Ensure data structure matches expected layout
5. Check that all required files exist (model checkpoint, combined.ply, etc.)
