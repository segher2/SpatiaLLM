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
├── multi_room_agent2.py      # Main AI agent with LLM query processing (2,166 lines)
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

---

## Usage

### Running the System

#### Demo Mode
Execute the main agent with predefined test queries:

```bash
cd LM2PCG
python multi_room_agent2.py
```

This runs 21 test queries demonstrating all system capabilities:
- Color analysis of specific objects
- Distance calculations between objects
- Multi-room comparisons
- Room information queries
- Floor-level statistics
- Object counting and filtering
- Cost estimation queries
- Visualization requests
- Out-of-scope query rejection

#### Interactive Mode
Launch an interactive session for custom queries:

```bash
cd LM2PCG
python -c "from multi_room_agent2 import interactive_session; interactive_session()"
```

Example queries:
- "What color is the door in the kitchen?"
- "How many bedrooms are in the building?"
- "What is the distance between the chair and the table?"
- "Show me room 0-2"

---

## System Architecture

### Query Processing Pipeline

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

### Key Design Principles
- Two-stage LLM workflow (scope → answer)
- Hybrid approach: LLM reasoning + precise computational tools
- Object code system: `floor-room-object` (e.g., `0-1-5`)
- Multi-modal: geometric data + panoramic images

---

## Vision-Based Room Classification

The system includes automated room type classification using GPT-4 Vision:

```bash
python enrich_room_types.py
```

This script:
- Scans database for rooms with unknown or generic types
- Sends panoramic images to GPT-4 Vision API
- Classifies rooms into categories (kitchen, bedroom, bathroom, etc.)
- Updates database with inferred room types

**Expected Output:**
```
Processing room 0-4 (unknown type)
  → Analyzing panoramic image...
  → GPT-4 Vision classification: kitchen
✓ Updated database: room 0-4 → kitchen

[Processed 3 rooms, updated 2 with new types]
```

--- Technical Achievements

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

## Quick Tests

### Validate Components
```bash
# Test 1: Database connectivity
python -c "from room_database import SpatialDatabaseCorrect; db = SpatialDatabaseCorrect(); print('✓ DB OK')"

# Test 2: API wrapper
python -c "from ai_api_wrapper import AiApiWrapper; api = AiApiWrapper(); print(f'✓ API Ready: {api.is_ready}')"

# Test 3: Agent initialization
python -c "from multi_room_agent2 import FinalSpatialAIAgent; agent = FinalSpatialAIAgent(); print('✓ Agent OK')"

# Test 4: Single query
python -c "from multi_room_agent2 import FinalSpatialAIAgent; agent = FinalSpatialAIAgent(); print(agent.query('How many rooms are there?'))"
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

## Component Summary

This LLM-based spatial intelligence system provides:

**Deliverables:**
1. **FinalSpatialAIAgent** (2,166 lines) - Main AI agent with natural language processing
2. **SpatialDatabaseCorrect** (607 lines) - Database schema and population pipeline
3. **AiApiWrapper** (241 lines) - Python interface to C++ computational tools
4. **Room Type Enrichment** (221 lines) - Automated vision-based room classification
5. **Comprehensive Documentation** - Technical documentation and setup guides

**Impact:**
- Enables non-technical users to query complex 3D building data using natural language
- Reduces query time from manual CSV inspection (5-10 minutes) to instant responses (<5 seconds)
- Provides intelligent interpretation of raw geometric data
- Demonstrates practical AI application in architecture and construction domains

---

**Last Updated:** November 11, 2025  
**Version:** 1.0 (Production-Ready)


## Dependencies

### Core Libraries
```python
openai>=1.0.0          # Azure OpenAI API client for GPT-4o-mini
pandas>=1.5.0          # Data manipulation and database queries
pillow>=9.0.0          # Image processing for panorama encoding
pydantic>=2.0.0        # API response validation and schemas
python-dotenv>=0.21.0  # Environment variable management
```

### Built-in Modules
```python
sqlite3      # Database operations
subprocess   # C++ tool execution
json         # API communication
re           # Query parsing with regex
base64       # Image encoding for vision API
os, glob     # File system operations
```

