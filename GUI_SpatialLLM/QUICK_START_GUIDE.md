# 🚀 GUI Quick Start Guide

## What You Have

Your GUI is a **Streamlit web application** that's already connected to your Spatial AI agent! The symbolic links are already set up, so your agent code (`mutli_room_agent2.py`) is automatically available in the GUI.

## Architecture Overview

```
GUI_SpatialLLM/
├── GUI_streamlit.py           ← Main web interface
├── bridge_server_final.py     ← Backend API server (for SAM2 segmentation)
│
├── mutli_room_agent2.py       ← 🔗 Linked to LM2PCG (YOUR AGENT!)
├── room_database.py           ← 🔗 Linked to LM2PCG 
├── ai_api_wrapper.py          ← 🔗 Linked to LM2PCG
├── enrich_room_types.py       ← 🔗 Linked to LM2PCG
│
└── (Other SAM23D links for 3D segmentation)
```

**The 🔗 symbol means these files are symbolic links to your LM2PCG code!**

## How It Works

1. **GUI (Streamlit)** provides the web interface
2. **Your agent code** (`mutli_room_agent2.py`) is imported and used directly
3. **Bridge server** (Flask) handles 3D segmentation features (optional for chat)
4. **Session management** initializes and connects everything

## 🎯 How to Run

### Step 1: Install GUI Dependencies

```bash
cd /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/GUI_SpatialLLM
source /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/venv/bin/activate
pip install -r requirements.txt
```

### Step 2: Start the GUI (Simple Mode - Chat Only)

```bash
streamlit run GUI_streamlit.py
```

This will open your browser at: **http://localhost:8501**

### Step 3 (Optional): Start Bridge Server (for 3D Segmentation)

If you want the full 3D segmentation features, open a **second terminal**:

```bash
cd /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/GUI_SpatialLLM
source /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/venv/bin/activate
python bridge_server_final.py
```

This runs on: **http://localhost:5056**

## 🖥️ Using the GUI

### Main Page (Home)

1. **Click "Start Session"** in the sidebar
   - This initializes your Spatial AI agent
   - It loads `room_database.py` and `mutli_room_agent2.py`
   - You'll see: ✅ "Spatial AI Agent initialized."

2. **Spatial AI Chat** appears at the bottom
   - Type your question: "What is in the kitchen?"
   - The agent uses your code from `mutli_room_agent2.py`
   - You'll get responses with room data, images, and analysis

3. **Special Commands:**
   - Type `overview` to see room summary
   - Type `quit` to end the chat

### Virtual Mode (3D Segmentation - Optional)

1. Click **"Virtual Mode"** in sidebar
2. Browse 360° panoramas
3. Click on images to select points
4. SAM2 automatically segments when 5 points selected

### Session Controls

- **Start Session**: Initialize the agent (first time)
- **Restart Session**: Reload the agent (if you made code changes)
- **End Session**: Close everything
- **Virtual Mode**: Enter 3D segmentation mode

## 🔧 How It's Connected to Your Agent

The GUI imports your agent like this (line 14 in `GUI_streamlit.py`):

```python
import mutli_room_agent2 as room_agent
```

When you click "Start Session" (lines 92-104):

```python
if start_btn:
    if st.session_state.agent:
        st.warning("Session already active.")
    else:
        with st.spinner("Initializing Spatial AI Agent..."):
            try:
                exec("room_database")
                exec("enrich_rooms")
                exec("ai_wrapper")
                st.session_state.agent = room_agent  # ← Your agent is loaded here!
                st.session_state.chat_ui = []
                st.success("Spatial AI Agent initialized.")
```

When you ask a question (lines 600-620):

```python
result = agent.query(user_q)  # ← Calls your agent's query() function!
```

## 📊 What Data Does It Use?

The GUI uses your **database and data from LM2PCG**:

- **Database**: `LM2PCG/spatial_rooms.db` (8 rooms, 2 floors)
- **Point Clouds**: `LM2PCG/data/rooms/Full House/floor_X/room_XXX/`
- **CSV Data**: `LM2PCG/output2/floor_X/room_XXX/`
- **Panoramas**: `GUI_SpatialLLM/data/rooms/` or `extracted_data/images/`

The symbolic links ensure the GUI can access everything from LM2PCG!

## 🎨 Features Your Agent Supports in GUI

Since your agent has these capabilities, they work in the GUI automatically:

1. ✅ **Multi-room queries**: "Compare kitchen and bedroom"
2. ✅ **Visual analysis**: "What's on the walls in the kitchen?"
3. ✅ **Cost estimation**: "What would it cost to paint the walls?"
4. ✅ **Floor area calculations**: "What's the total floor area?"
5. ✅ **Room summaries**: Type `overview` command
6. ✅ **Object queries**: "What objects are in room 2?"
7. ✅ **Dual-source analysis**: CSV data + panorama images

## 🔄 Making Changes

If you update your agent code (`LM2PCG/mutli_room_agent2.py`):

1. **Option 1**: Click "Restart Session" in the GUI sidebar
2. **Option 2**: Refresh the browser page (F5)
3. **Option 3**: Stop and restart Streamlit

The changes will be reflected immediately because the GUI uses symbolic links!

## 🚨 Troubleshooting

### "Bridge Server Not Found" Error

**Solution**: This is only needed for 3D segmentation. For chat, you can ignore it or:
```bash
# In a separate terminal:
python bridge_server_final.py
```

### "Failed to start AI Agent"

**Solution**: Make sure you're in the venv and have all dependencies:
```bash
source /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/venv/bin/activate
cd GUI_SpatialLLM
pip install -r requirements.txt
```

### "No module named 'mutli_room_agent2'"

**Solution**: Check the symbolic link:
```bash
ls -la mutli_room_agent2.py
# Should show: mutli_room_agent2.py -> ../LM2PCG/mutli_room_agent2.py
```

If broken, recreate it:
```bash
ln -sf ../LM2PCG/mutli_room_agent2.py mutli_room_agent2.py
```

### Agent not using updated database

**Solution**: The agent looks for database at:
- `LM2PCG/spatial_rooms.db` (your main database)

Make sure your database has data:
```bash
cd /Users/neelabhsingh/Documents/College/Synthesis/SpatialLLM/LM2PCG
python3 -c "import room_database; db = room_database.RoomDatabase('spatial_rooms.db'); print(f'Rooms: {len(db.get_all_rooms())}')"
```

## 📝 Example Session

```
1. Open terminal
2. cd GUI_SpatialLLM
3. source ../venv/bin/activate
4. streamlit run GUI_streamlit.py

→ Browser opens at http://localhost:8501

5. Click "Start Session" in sidebar
6. Wait for "Spatial AI Agent initialized" message
7. Type in chat: "What is in the kitchen?"
8. Get response with room details, objects, and panorama analysis!
```

## 🎉 That's It!

Your GUI is already connected to your agent via symbolic links. Just:
1. **Install dependencies** (`pip install -r requirements.txt`)
2. **Run Streamlit** (`streamlit run GUI_streamlit.py`)
3. **Start Session** (click button in sidebar)
4. **Ask questions!** (your agent handles everything)

No additional configuration needed! 🚀

---

## Advanced: Data Directory Structure

If you want to add your own panorama data:

```
GUI_SpatialLLM/
├── extracted_data/
│   ├── images/              ← Put panorama JPG/PNG files here
│   └── metadata/            ← Camera pose JSON files
│
├── data/
│   └── rooms/               ← Or organize by floor/room structure
│       └── Full House/
│           ├── floor_0/
│           │   └── room_XXX/
│           │       ├── *.jpg (panoramas)
│           │       └── combined1_*.ply (point clouds)
│           └── floor_1/
│               └── room_XXX/
└── outputs/                 ← SAM2 segmentation outputs
```

The GUI will automatically find panoramas in `extracted_data/images/` when you enter Virtual Mode.
