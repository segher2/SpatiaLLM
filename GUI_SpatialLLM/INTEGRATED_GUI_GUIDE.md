# 🏗️ Integrated GUI Guide - Spatial LLM Virtual Mode

## Overview
The GUI has been upgraded to integrate **SAM23D segmentation** and **AI Agent** into a single unified **Virtual Mode** experience.

## Key Changes

### 1. **Simplified Sidebar (Single Button)**
- ❌ Removed: Start Session, Restart Session, End Session, Virtual Mode (4 separate buttons)
- ✅ Added: Single **"Start Virtual Mode Session"** button
  - Initializes both AI Agent + SAM23D functionality
  - Loads database and displays room summary table
  - Starts in Virtual Mode by default

### 2. **Integrated Features in Virtual Mode**

#### **Panorama Viewer**
- Navigate through panoramas with Previous/Next buttons
- Enable/disable click mode for point selection
- Right-click to reset points

#### **SAM2 Segmentation**
- Click 5 points on any object in the panorama
- Automatic segmentation on 5th click
- Overlay displayed directly on panorama
- 3D point cloud extraction with mask2cluster

#### **AI Agent Chat**
- Natural language queries about rooms
- Spatial relationship understanding
- Object detection and counting
- Multi-room analysis

### 3. **Workflow**

```
1. Open GUI → Click "Start Virtual Mode Session" in sidebar
   ↓
2. Agent initializes + Room database summary displayed
   ↓
3. Navigate panoramas (Previous/Next buttons)
   ↓
4. [OPTIONAL] Enable "Click mode" → Select 5 points → SAM2 segments object
   ↓
5. Ask questions in chat: "What's in this room?", "How many windows?", etc.
   ↓
6. [OPTIONAL] View 3D point cloud if SAM2 processing complete
   ↓
7. Click "Restart Session" to reset or "End Session" to stop
```

### 4. **Panorama Directory Support**

The GUI now checks two locations for panorama images:
1. `GUI_SpatialLLM/extracted_data/images/` (your new setup)
2. `data/input/panoramas/images/` (fallback for SAM23D workflow)

Place your panorama images in either location.

### 5. **Bridge Server Status**

- **If running**: SAM2 segmentation features enabled
- **If not running**: GUI still works with AI Agent (SAM2 disabled)
- Start bridge server: `cd GUI_SpatialLLM && source venv/bin/activate && python bridge_server_final.py`

## Features Status

| Feature | Status | Notes |
|---------|--------|-------|
| AI Agent | ✅ Active | Requires "Start Virtual Mode Session" |
| SAM2 Segmentation | ⚠️ Optional | Requires bridge server running |
| 3D Point Cloud Viewer | ✅ Active | After SAM2 completes |
| Chat Interface | ✅ Active | Natural language queries |
| Room Database | ✅ Active | Summary table displayed on start |

## Example Queries

After starting Virtual Mode Session, try these queries:

- **"What objects are in room 1-1-1?"**
- **"How many rooms are on floor 0?"**
- **"Tell me about the windows in room 1-1-2"**
- **"What's the room type of room 0-2-1?"**
- **"Show me all objects with 'door' in the name"**

## Dependencies

All agent files must be in `GUI_SpatialLLM/` directory:
- ✅ `mutli_room_agent2.py`
- ✅ `ai_api_wrapper.py`
- ✅ `enrich_room_types.py`
- ✅ `room_database.py`

Database location: `../LM2PCG/spatial_rooms.db`

## Running the GUI

```bash
cd GUI_SpatialLLM
source venv/bin/activate

# Start bridge server (optional, for SAM2)
python bridge_server_final.py &

# Start Streamlit GUI
streamlit run GUI_streamlit.py
```

Or use the helper script:
```bash
./run_gui.sh
```

## Troubleshooting

### Agent doesn't start
- Check that `spatial_rooms.db` exists in `../LM2PCG/`
- Verify all agent Python files are in `GUI_SpatialLLM/`
- Check `.env` file has `AZURE_OPENAI_API_KEY` set

### SAM2 not working
- Start bridge server: `python bridge_server_final.py`
- Verify SAM2 checkpoint: `SAM23D/SAM2/checkpoints/sam2.1_hiera_large.pt`
- Check panorama image is processed (exists in `data/output/`)

### No panoramas found
- Place images in `GUI_SpatialLLM/extracted_data/images/`
- Or use processed images from `data/input/panoramas/images/`

## Architecture

```
GUI_SpatialLLM/
├── GUI_streamlit.py          # Main GUI (upgraded)
├── bridge_server_final.py    # Flask server for SAM2
├── mutli_room_agent2.py      # AI Agent logic
├── ai_api_wrapper.py         # Azure OpenAI wrapper
├── room_database.py          # Database interface
├── enrich_room_types.py      # Room type enrichment
├── venv/                     # Virtual environment
└── extracted_data/           # Panorama images
    └── images/

LM2PCG/
└── spatial_rooms.db          # Room database

SAM23D/
├── sam2_predictor.py         # SAM2 integration
├── select_points_in_mask.py  # 3D filtering
└── SAM2/
    └── checkpoints/
        └── sam2.1_hiera_large.pt
```

## Next Steps

1. ✅ GUI upgraded with integrated Virtual Mode
2. ⏳ Test agent queries with your database
3. ⏳ Test SAM2 segmentation on processed images
4. ⏳ Verify 3D point cloud viewer works
5. ⏳ Add new segmented objects to database for agent queries
