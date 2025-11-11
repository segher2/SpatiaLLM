# SpatiaLLM - AI-Powered Spatial Intelligence Agent

## Overview

This is the **LLM-based Multi-Room Agent** component of the SpatiaLLM project - an advanced spatial AI assistant that combines Large Language Models with 3D point cloud analysis for intelligent architectural space understanding and querying.

### Key Features
- Intelligent Natural Language Querying: Ask questions about rooms, objects, colors, distances in plain English
- Multi-Modal Analysis: Combines geometric data (point clouds) with visual data (panoramic images)
- Color Analysis: Automatic dominant color detection using Gaussian Mixture Models
- Spatial Measurements: Calculate volumes, distances, and dimensions
- SQLite Database: Structured storage of floors, rooms, objects, planes, and images
- Computer Vision Integration: GPT-4 Vision for room type classification from panoramic images
- Smart Query Scoping: Automatically determines if queries are single-room or multi-room

---

## My Component Files

```
LM2PCG/
├── mutli_room_agent2.py      # Main AI agent with LLM query processing (2,166 lines)
├── room_database.py           # SQLite database management (607 lines)
├── ai_api_wrapper.py          # Python wrapper for C++ tools (241 lines)
├── enrich_room_types.py       # Vision-based room classification (221 lines)
├── spatial_rooms.db           # SQLite database (auto-generated)
└── .env                       # Azure OpenAI credentials
```


---

## Installation & Setup

### Prerequisites
- Python 3.8+
- Azure OpenAI API access (GPT-4o-mini with vision capabilities)
- C++ build environment (for point cloud processing tools - see main README.md)

### Step 1: Install Python Dependencies

Create `requirements.txt` in `LM2PCG/` directory:

```txt
# Core LLM and API
openai>=1.0.0
python-dotenv>=0.21.0

# Data Processing
pandas>=1.5.0
pillow>=9.0.0

# Validation
pydantic>=2.0.0
```

Install dependencies:

```bash
cd LM2PCG
pip install -r requirements.txt
```

### Step 2: Configure Environment Variables

Create a `.env` file in the `LM2PCG/` directory:

```bash
# .env file
API_KEY=your_azure_openai_api_key_here
AZURE_OPENAI_ENDPOINT=https://azure-openai-scanplan.openai.azure.com/
```

### Step 3: Prepare Database

Ensure you have the data directory structure ready (see main README for data preparation):
```
../data/output/
├── floor_0/
│   ├── rooms_manifest.csv
│   └── room_001/, room_002/, ...
└── floor_1/
    └── ...
```

Then generate the database:

```bash
python room_database.py
```

**Expected output:**
```
📋 POPULATING DATABASE FROM ROOMS_MANIFEST.CSV
✓ Old tables dropped.
✓ Database tables created for new workflow
 Reading manifest: ../data/output/floor_0/rooms_manifest.csv
     Found 8 rooms in manifest
✓ Imported 45 objects from room_001.csv
✓ Imported 12 planes from planes_data.csv
✓ Added 3 panorama images
 Database populated successfully!
   Floors: 2
   Rooms: 16
   Objects: 234
   Planes: 96
   Images: 48
```

---

## Running the System for Presentation

### RECOMMENDED: Demo Mode

This is the best way to showcase the system during your presentation!

```bash
cd LM2PCG
python mutli_room_agent2.py
```

**What Happens:**
- Initializes the AI agent and database
- Prints database summary table
- Runs 21 predefined test queries automatically
- Showcases ALL features in 2-3 minutes
- No manual input needed - perfect for presentations

**Sample Demo Output:**

```
═══════════════════════════════════════════════════════════
 DATABASE SUMMARY
═══════════════════════════════════════════════════════════
Floor 0 (8 rooms, 125 objects, 42 planes, 24 images)
  └─ room_001 (Kitchen)       : 18 objects, 6 planes
  └─ room_002 (Living Room)   : 22 objects, 6 planes
  └─ room_003 (Bedroom)       : 15 objects, 6 planes
  ...

Floor 1 (8 rooms, 109 objects, 54 planes, 24 images)
  └─ room_001 (Office)        : 12 objects, 6 planes
  ...
═══════════════════════════════════════════════════════════

🤖 RUNNING DEMO MODE (21 TEST QUERIES)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Query 1/21: What color is object 0-2-3?
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔧 Executing Tool: CLR 0-2-3
✓ API Result: CLR (0-2-3): Found 2 colors: 
  [W: 0.70, RGB: (180, 195, 185)], [W: 0.30, RGB: (95, 110, 100)]

🤖 LLM Response:
Object 0-2-3 (door_2) displays two dominant colors:
- Light mint green (70% coverage) - RGB: (180, 195, 185)
- Darker forest green (30%) - RGB: (95, 110, 100)

This suggests a two-tone painted surface with lighter tones 
being more prominent.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Query 2/21: How far apart are objects 0-2-1 and 0-2-5?
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔧 Executing Tool: BBD 0-2-1 0-2-5
✓ API Result: Distance = 2.35 meters

🤖 LLM Response:
The distance between object 0-2-1 (chair) and object 0-2-5 
(table) is approximately 2.35 meters (7.7 feet).
...

[21 queries complete in ~120 seconds]
```

**Test Queries Covered:**
1. Color analysis of specific objects
2. Distance calculations between objects
3. Multi-room comparisons
4. Room information queries
5. Floor-level statistics
6. Object counting and filtering
7. Cost estimation queries
8. Visualization requests
9. Out-of-scope query rejection

---

### Alternative: Interactive Mode

For live Q&A during presentation (if time permits):

```bash
cd LM2PCG
python -c "from mutli_room_agent2 import interactive_session; interactive_session()"
```

**Example Interaction:**
```
═════════════════════════════════════════════════════════════
 STARTING INTERACTIVE SPATIAL AI AGENT SESSION
   Type 'quit' or 'exit' to end.
═════════════════════════════════════════════════════════════

> What color is the door in room 0-2?

🔧 Executing Tool: CLR 0-2-3
✓ Result: Light mint green with darker green accents

🤖 Response: The door in room 0-2 is painted in a two-tone 
green color scheme. The primary color is a light mint green...

> How many bedrooms are in the building?

🤖 Response: There are 3 bedrooms in the building:
- Floor 0: room_003 (Bedroom)
- Floor 1: room_005 (Bedroom), room_006 (Bedroom)

> quit
Goodbye!
```

---

## Presentation Script (10 minutes)

### Part 1: Introduction (2 min)

**Show the slide/screen:**
```
"SpatiaLLM: AI-Powered Spatial Intelligence"
My Component: LLM-based Multi-Room Agent
```

**Say:**
> "I developed the AI agent that allows users to query building information 
> using natural language. The system combines GPT-4o-mini with a custom 
> SQLite database containing geometric and visual data from point clouds."

**Run the demo:**
```bash
python mutli_room_agent2.py
```

**Point out the database summary table that appears:**
> "Here you can see our database contains 2 floors, 16 rooms, 234 objects, 
> and 48 panoramic images. The system automatically populates this from 
> point cloud data."

---

### Part 2: Feature Showcase (5 min)

**Let the demo run and highlight these queries as they execute:**

**1. Color Analysis (Query 1):**
> "Watch how the system analyzes the dominant color of an object. It calls 
> our C++ tool, gets RGB values, and the LLM interprets them into human-
> readable color names like 'light mint green'."

**2. Distance Measurement (Query 2):**
> "Here it calculates the distance between two objects - useful for space 
> planning and accessibility analysis."

**3. Multi-Room Intelligence (Query 5):**
> "This demonstrates the smart query scoping. The system automatically 
> detects this is a multi-room query and compares data across rooms."

**4. Cost Estimation (Query 7):**
> "The LLM uses room dimensions and object counts to estimate renovation 
> costs - showcasing the system's practical applications."

**5. Out-of-Scope Handling (Query 21):**
> "Notice how it politely rejects queries outside its knowledge domain, 
> like predicting future events. This prevents hallucinations."

---

### Part 3: Architecture Deep-Dive (2 min)

**Show architecture diagram (or explain):**

```
User Query: "What color is the kitchen table?"
    ↓
1. Parser extracts: room="kitchen", object_type="table"
    ↓
2. Scope Classifier (LLM): → SINGLE_ROOM
    ↓
3. Data Fetcher: Load room_001 (kitchen) data
    ↓
4. Tool Executor: 
   - Find object_code for "table" → 0-1-5
   - Execute: CLR 0-1-5 (C++ tool)
   - Result: RGB values [180, 160, 140]
    ↓
5. Answer Generator (LLM):
   - Context: room data + tool results
   - Interprets: RGB → "warm beige"
   - Response: "The kitchen table is primarily warm beige..."
```

**Key Points to Mention:**
- Two-stage LLM workflow (scope → answer)
- Hybrid approach: LLM reasoning + precise computational tools
- Object code system: `floor-room-object` (e.g., `0-1-5`)
- Multi-modal: geometric data + panoramic images

---

### Part 4: Vision Integration (1 min)

**Show room classification:**
```bash
python enrich_room_types.py
```

**Say:**
> "This script uses GPT-4 Vision to automatically classify room types from 
> panoramic images. It finds rooms labeled 'unknown', sends their photos to 
> the vision model, and updates the database with classifications like 
> 'kitchen', 'bedroom', 'bathroom', etc."

**Expected output:**
```
📸 PROCESSING ROOM 0-4 (unknown type)
   → Analyzing panoramic image...
   → GPT-4 Vision classification: kitchen
✓  Updated database: room 0-4 → kitchen

[Processed 3 rooms, updated 2 with new types]
```

---

## Technical Achievements

### Code Metrics
- **Total Lines:** 3,235 lines of Python
- **Main Agent:** 2,166 lines (24 methods, 21 test queries)
- **Database Manager:** 607 lines (complete CRUD operations)
- **API Wrapper:** 241 lines (5 tool integrations)
- **Vision Classifier:** 221 lines (automated room classification)

### Architecture Highlights
1. Two-Stage LLM Pipeline: Scope classification → Answer generation
2. Smart Context Management: Loads only relevant data per query
3. Tool Orchestration: Seamless Python to C++ communication
4. Multi-Modal Reasoning: Combines point clouds + images + metadata
5. Robust Error Handling: Graceful fallbacks and timeouts
6. Production-Ready: Clean code, comprehensive documentation, tested

### Database Schema
```sql
-- 5 interconnected tables
Floors (floor_id, floor_name, floor_number, total_rooms, total_area)
  └─ Rooms (room_id, floor_id, room_name, room_type, dimensions)
       ├─ Objects (object_id, room_id, object_code, class, geometry, color)
       ├─ Planes (plane_id, room_id, plane_class, normal, area)
       └─ Images (image_id, room_id, image_path)
```

---

## Quick Tests (If Demo Fails)

### Validate Components
```bash
# Test 1: Database connectivity
python -c "from room_database import SpatialDatabaseCorrect; db = SpatialDatabaseCorrect(); print('✓ DB OK')"

# Test 2: API wrapper
python -c "from ai_api_wrapper import AiApiWrapper; api = AiApiWrapper(); print(f'✓ API Ready: {api.is_ready}')"

# Test 3: Agent initialization
python -c "from mutli_room_agent2 import FinalSpatialAIAgent; agent = FinalSpatialAIAgent(); print('✓ Agent OK')"

# Test 4: Single query
python -c "from mutli_room_agent2 import FinalSpatialAIAgent; agent = FinalSpatialAIAgent(); print(agent.query('How many rooms are there?'))"
```

---

## Troubleshooting

### Issue: "spatial_rooms.db not found"
**Solution:**
```bash
python room_database.py
```

### Issue: "OpenAI API key not set"
**Solution:**
```bash
# Create .env file
echo "API_KEY=sk-..." > .env
echo "AZURE_OPENAI_ENDPOINT=https://..." >> .env
```

### Issue: "C++ tools not found"
**Solution:**
```bash
# Build the C++ tools first (from project root)
cd ..
./pcg.sh build
cd LM2PCG
```

### Issue: "No module named 'openai'"
**Solution:**
```bash
pip install -r requirements.txt
```

---

## Learning Outcomes

### Technical Skills Developed
1. **LLM Engineering**: Prompt design, context management, two-stage workflows
2. **Database Design**: SQLite schema design, indexing, query optimization
3. **System Integration**: Python ↔ C++ interop, subprocess management
4. **Multi-Modal AI**: Combining vision models with geometric processing
5. **API Design**: Clean abstractions, error handling, Pydantic validation
6. **Production Code**: Documentation, testing, code quality standards

### Challenges Overcome
1. **Context Window Management**: Implemented smart data filtering to fit LLM context limits
2. **Tool Orchestration**: Designed reliable subprocess communication with timeouts
3. **Query Parsing**: Built robust NLP-based room/object reference extraction
4. **Color Interpretation**: Developed RGB → human-readable color name mapping
5. **Multi-Room Reasoning**: Created scope classifier to handle queries spanning multiple rooms

---

## Dependencies Explained

### Core Libraries
```python
openai>=1.0.0          # Azure OpenAI API client for GPT-4o-mini
pandas>=1.5.0          # Data manipulation and database queries
pillow>=9.0.0          # Image processing for panorama encoding
pydantic>=2.0.0        # API response validation and schemas
python-dotenv>=0.21.0  # Environment variable management
```

### Built-in Modules (No Installation Needed)
```python
sqlite3      # Database operations
subprocess   # C++ tool execution
json         # API communication
re           # Query parsing with regex
base64       # Image encoding for vision API
os, glob     # File system operations
```

---

## My Contribution Summary

**Role:** LLM Integration & Spatial Intelligence System

**Deliverables:**
1. FinalSpatialAIAgent (2,166 lines) - Main AI agent with natural language processing
2. SpatialDatabaseCorrect (607 lines) - Database schema and population pipeline
3. AiApiWrapper (241 lines) - Python interface to C++ tools
4. Room Type Enrichment (221 lines) - Automated vision-based classification
5. Comprehensive Documentation - README, code comments, presentation guide

**Impact:**
- Enables non-technical users to query complex 3D building data
- Reduces query time from manual CSV inspection (5-10 min) to instant (<5 sec)
- Provides intelligent interpretation of raw geometric data
- Demonstrates practical AI application in architecture/construction domains

---

## Presentation Tips

### Before Presentation
1. Run `python room_database.py` to ensure fresh database
2. Test demo mode once: `python mutli_room_agent2.py`
3. Have `.env` file ready with valid API keys
4. Close unnecessary applications for smooth demo
5. Prepare backup: screenshots of demo output

### During Presentation
1. Start with demo mode - it's foolproof and impressive
2. Let queries 1, 2, 5, 7, and 21 run - they showcase best features
3. Explain the 2-stage architecture while demo runs
4. Highlight multi-modal reasoning (geometric + visual)
5. Show the database summary table at the start

### If Demo Fails
1. Show pre-recorded demo video/screenshots
2. Walk through code structure and architecture diagrams
3. Present metrics: 3,235 lines, 5 tools, 21 test queries
4. Explain the technical challenges overcome

---

**Last Updated:** November 11, 2025  
**Version:** 1.0 (Production-Ready for Presentation)  
**Author:** Neelabh Singh
