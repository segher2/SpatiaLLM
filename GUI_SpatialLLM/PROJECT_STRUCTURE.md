# Project Structure Reorganization

This document explains the new modular organization of the Spatial Understanding system.

## Overview

The codebase has been reorganized into three separate projects:

1. **GUI** - Front-end and back-end for user interaction
2. **SAM23D** - Image segmentation and 3D point cloud processing
3. **LM2PCG** - Spatial AI agent with multi-room database

## Directory Structure

```
Documents/GitHub/
│
├── GUI/                                  # Front-end + Back-end
│   ├── GUI_streamlit.py                 # Streamlit web interface
│   ├── bridge_server_final.py           # Flask REST API server
│   ├── requirements.txt                 # GUI-specific dependencies
│   │
│   ├── data/                            # Data directories
│   │   └── lm2pcg_data/ → LM2PCG/data/rooms/Full House
│   │
│   ├── extracted_data/                  # Panorama images & metadata
│   │   ├── images/
│   │   └── metadata/
│   │
│   └── [Symbolic Links]
│       ├── sam2_predictor.py → ../SAM23D/sam2_predictor.py
│       ├── select_points_in_mask.py → ../SAM23D/select_points_in_mask.py
│       ├── conversion_2D_3D.py → ../SAM23D/conversion_2D_3D.py
│       ├── SAM2/ → ../SAM23D/SAM2/
│       ├── sam23d_outputs/ → ../SAM23D/outputs/
│       ├── ai_api_wrapper.py → ../LM2PCG/ai_api_wrapper.py
│       ├── mutli_room_agent2.py → ../LM2PCG/mutli_room_agent2.py
│       ├── room_database.py → ../LM2PCG/room_database.py
│       ├── enrich_room_types.py → ../LM2PCG/enrich_room_types.py
│       ├── spatial_rooms.db → ../LM2PCG/spatial_rooms.db
│       └── scripts/ → ../LM2PCG/scripts/
│
├── SAM23D/                               # 3D Processing Pipeline
│   ├── README.md                        # SAM23D documentation
│   ├── __init__.py                      # Python package initialization
│   ├── requirements.txt                 # 3D processing dependencies
│   │
│   ├── sam2_predictor.py               # SAM2 image segmentation
│   ├── select_points_in_mask.py        # Point cloud filtering
│   ├── conversion_2D_3D.py             # Coordinate transformations
│   │
│   ├── SAM2/                           # SAM2 model files
│   │   ├── checkpoints/
│   │   │   └── sam2.1_hiera_large.pt
│   │   ├── configs/
│   │   │   └── sam2.1/
│   │   └── sam2/                       # SAM2 Python package
│   │
│   └── outputs/                        # Generated masks & overlays
│       ├── *_binary_mask.png
│       └── *_overlay.png
│
└── LM2PCG/                              # Spatial AI System
    ├── ai_api_wrapper.py               # C++/Python API wrapper
    ├── mutli_room_agent2.py            # Multi-room AI agent
    ├── room_database.py                # SQLite database manager
    ├── enrich_room_types.py            # Room classification
    ├── spatial_rooms.db                # Spatial database
    │
    ├── scripts/                        # Helper scripts
    │   ├── ai_api.py
    │   └── api_server.py
    │
    └── data/
        └── rooms/
            └── Full House/             # Point cloud & panorama data
                ├── floor_0/
                │   ├── room_001/
                │   │   ├── *.jpg       # Panoramic images
                │   │   └── shell_*.ply # Point clouds
                │   └── room_002/
                └── floor_1/
```

## Project Responsibilities

### GUI (Front-end + Back-end)
**Purpose**: User interface and API endpoints

**Components**:
- **Streamlit UI** (`GUI_streamlit.py`)
  - 360° panorama viewer (Pannellum.js)
  - Point selection interface
  - Session management
  - Virtual mode navigation

- **Flask Server** (`bridge_server_final.py`)
  - REST API endpoints
  - Click coordinate handling
  - Trigger SAM2 segmentation
  - Manage point cloud associations

**Dependencies**:
- streamlit
- flask, flask-cors
- requests
- psutil
- streamlit-js-eval

**Runs**: 
- Flask: `http://localhost:5056`
- Streamlit: `http://localhost:8501`

### SAM23D (3D Processing)
**Purpose**: Image segmentation and point cloud filtering

**Components**:
- **SAM2 Predictor** (`sam2_predictor.py`)
  - Loads SAM2.1 Hiera Large model
  - Performs guided segmentation (5 points)
  - Generates binary masks and overlays

- **Point Cloud Filter** (`select_points_in_mask.py`)
  - Projects 3D points to 2D panoramas
  - Filters points using segmentation masks
  - Exports filtered LAS files

- **Coordinate Utils** (`conversion_2D_3D.py`)
  - Quaternion ↔ Rotation matrix
  - Pixel ↔ 3D ray direction
  - Equirectangular projection

**Dependencies**:
- torch, torchvision
- pillow, numpy
- laspy, plyfile
- hydra-core, omegaconf

**Outputs**:
- `outputs/*_binary_mask.png`
- `outputs/*_overlay.png`

### LM2PCG (Spatial AI)
**Purpose**: Natural language spatial understanding

**Components**:
- **AI Agent** (`mutli_room_agent2.py`)
  - Multi-room query processing
  - Azure OpenAI integration
  - Image analysis capabilities

- **Database** (`room_database.py`)
  - SQLite spatial database
  - Floor/room hierarchy
  - Object and plane storage

- **API Wrapper** (`ai_api_wrapper.py`)
  - External C++/Python pipeline
  - Volume, color, distance computations

- **Room Enrichment** (`enrich_room_types.py`)
  - Automatic room classification
  - Vision-based type detection

**Dependencies**:
- openai, python-dotenv
- sqlite3, pandas
- pydantic

**Database**: `spatial_rooms.db`

## Data Flow

```
User Interaction (GUI)
        ↓
    [Streamlit UI]
    - User clicks 5 points on panorama
    - Sends to Flask server
        ↓
    [Flask Bridge Server]
    - Converts pitch/yaw → pixels
    - Finds panorama and PLY files
    - Triggers SAM23D when 5 points reached
        ↓
    [SAM23D - sam2_predictor.py]
    - Loads panoramic image
    - Runs SAM2 segmentation
    - Generates mask & overlay
        ↓
    [SAM23D - select_points_in_mask.py]
    - Loads point cloud (PLY)
    - Projects 3D → 2D using camera pose
    - Filters points inside mask
    - Exports filtered LAS
        ↓
    [Flask Server]
    - Returns overlay (base64) to UI
    - Displays on panorama
        ↓
    [Optional: LM2PCG Agent]
    - User queries about space
    - AI agent analyzes and responds
```

## Why This Structure?

### Separation of Concerns
- **GUI**: Pure presentation and API layer
- **SAM23D**: Reusable 3D processing pipeline
- **LM2PCG**: Independent spatial intelligence system

### Modularity
- Each project can be developed independently
- Clear interfaces between components
- Easy to test in isolation

### Reusability
- SAM23D can be used by other projects
- LM2PCG can serve multiple GUIs
- Components are loosely coupled

### Maintainability
- Smaller, focused codebases
- Clear dependencies
- Easier to debug and extend

## Installation

### 1. Clone All Projects
```bash
cd ~/Documents/GitHub
git clone <GUI-repo>
git clone <SAM23D-repo>
git clone <LM2PCG-repo>
```

### 2. Install Dependencies

**GUI**:
```bash
cd GUI
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**SAM23D**:
```bash
cd SAM23D
pip install -r requirements.txt
# Download SAM2 checkpoint to SAM2/checkpoints/
```

**LM2PCG**:
```bash
cd LM2PCG
pip install -r requirements.txt
# Set up .env with Azure OpenAI credentials
```

### 3. Create Symbolic Links

**From GUI to SAM23D**:
```bash
cd GUI
ln -sf ../SAM23D/sam2_predictor.py sam2_predictor.py
ln -sf ../SAM23D/select_points_in_mask.py select_points_in_mask.py
ln -sf ../SAM23D/conversion_2D_3D.py conversion_2D_3D.py
ln -sf ../SAM23D/SAM2 SAM2
ln -sf ../SAM23D/outputs sam23d_outputs
```

**From GUI to LM2PCG**:
```bash
cd GUI
ln -sf ../LM2PCG/ai_api_wrapper.py ai_api_wrapper.py
ln -sf ../LM2PCG/mutli_room_agent2.py mutli_room_agent2.py
ln -sf ../LM2PCG/room_database.py room_database.py
ln -sf ../LM2PCG/enrich_room_types.py enrich_room_types.py
ln -sf ../LM2PCG/spatial_rooms.db spatial_rooms.db
ln -sf ../LM2PCG/scripts scripts
```

**Data Link**:
```bash
cd GUI
mkdir -p data
ln -sf ../../LM2PCG/data/rooms/Full\ House data/lm2pcg_data
```

## Running the System

### Start All Services

**Terminal 1 - Flask Server**:
```bash
cd GUI
source .venv/bin/activate
python bridge_server_final.py
```

**Terminal 2 - Streamlit UI**:
```bash
cd GUI
source .venv/bin/activate
streamlit run GUI_streamlit.py
```

**Access**:
- Streamlit: http://localhost:8501
- Flask API: http://localhost:5056

## Development Workflow

### Adding Features to GUI
1. Edit `GUI_streamlit.py` or `bridge_server_final.py`
2. Restart respective service
3. No changes needed in SAM23D or LM2PCG

### Improving 3D Processing
1. Edit files in `SAM23D/`
2. Changes automatically reflected in GUI (via symlinks)
3. Can test SAM23D independently

### Enhancing AI Agent
1. Edit files in `LM2PCG/`
2. Changes automatically available to GUI
3. Can develop LM2PCG separately

## Testing

### Test SAM23D Independently
```python
# In SAM23D directory
from sam2_predictor import run_sam2_prediction
import numpy as np

points = np.array([[100, 200], [150, 250], [200, 300], [250, 350], [300, 400]])
result = run_sam2_prediction(points, "test_image.jpg")
print(result)
```

### Test GUI API
```bash
curl http://localhost:5056/health
```

### Test LM2PCG Agent
```python
# In LM2PCG directory
from mutli_room_agent2 import FinalSpatialAIAgent

agent = FinalSpatialAIAgent()
response = agent.query("What rooms are on floor 0?")
print(response)
```

## Troubleshooting

### Broken Symbolic Links
```bash
# Check links
ls -la GUI/ | grep "^l"

# Recreate if needed
cd GUI
rm sam2_predictor.py  # remove broken link
ln -sf ../SAM23D/sam2_predictor.py sam2_predictor.py
```

### Module Import Errors
Ensure projects are in correct relative locations:
```
~/Documents/GitHub/
├── GUI/
├── SAM23D/
└── LM2PCG/
```

### SAM2 Not Found
```bash
cd SAM23D
ls -la SAM2/checkpoints/sam2.1_hiera_large.pt
# Download if missing
```

## Future Enhancements

### Potential Improvements
- **SAM23D**: Support for SAM2 video segmentation
- **GUI**: WebSocket for real-time updates
- **LM2PCG**: Multi-building support
- **Integration**: Unified configuration management

### API Standardization
Consider creating formal API contracts between:
- GUI ↔ SAM23D
- GUI ↔ LM2PCG
- LM2PCG ↔ SAM23D (future direct integration)

## Summary

This modular structure provides:
✅ Clear separation between UI, 3D processing, and AI
✅ Independent development and testing
✅ Reusable components
✅ Maintainable codebase
✅ Flexible deployment options

Each project has its own:
- README with specific documentation
- requirements.txt with dependencies
- Focused responsibility and scope
