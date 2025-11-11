import sqlite3
import pandas as pd
import os
import base64
import re
import json
import textwrap
import subprocess  # Required for AiApiWrapper's functions
from typing import Dict, List, Optional, Any, Tuple, Union
from openai import AzureOpenAI
from dotenv import load_dotenv
from PIL import Image
import io
import traceback

# Assuming ai_api_wrapper.py is in the same directory or accessible via PYTHONPATH
# Import VisOutput
from ai_api_wrapper import AiApiWrapper, VisOutput

# Load environment variables (e.g., API_KEY, AZURE_OPENAI_ENDPOINT)
load_dotenv()


class FinalSpatialAIAgent:
    """
    A spatial AI agent that interacts with a room database and an external
    C++/Python pipeline to answer queries about architectural spaces.
    Includes capability to trigger point cloud visualizations and suggest them.
    """

    def __init__(self, database_path: str = "spatial_rooms.db", use_images: bool = False):
        """Initializes the agent, connects to the database, and loads initial data."""
        print(" Initializing Final Spatial AI Agent (LLM Scope V4)...") 
        
        # Use absolute paths to work from any directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if not os.path.isabs(database_path):
            database_path = os.path.join(script_dir, database_path)
        
        self.database_path = database_path
        self.use_images = use_images
        self.current_room_id: Optional[int] = None
        self.room_cache: Dict[int, Dict] = {}
        self.conn: Optional[sqlite3.Connection] = None
        self.client: Optional[AzureOpenAI] = None
        self.floors_df: pd.DataFrame = pd.DataFrame()
        self.rooms_df: pd.DataFrame = pd.DataFrame()

        # --- Tool Initialization ---
        api_script_path = os.path.join(script_dir, "scripts", "ai_api.py")
        self.api_wrapper = AiApiWrapper(api_script_path=api_script_path)
        if not self.api_wrapper.is_ready:
            print(" WARNING: External C++ API wrapper is NOT ready. Volume/Color/Distance queries will fail.")

        # --- Database Connection ---
        try:
            self.conn = sqlite3.connect(database_path)
            print(f"Success :  Connected to database: {database_path}")
        except sqlite3.Error as e:
            print(f"Error :  CRITICAL: Database connection failed: {e}")
            raise  # Agent cannot function without the database

        # --- OpenAI Client Initialization ---
        try:
            # Ensure required environment variables are present
            api_key = os.getenv("API_KEY")
            azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT",
                                       "https://azure-openai-scanplan.openai.azure.com/")
            if not api_key:
                raise ValueError("API_KEY environment variable not set.")
            if not azure_endpoint:
                raise ValueError("AZURE_OPENAI_ENDPOINT environment variable not set.")

            self.client = AzureOpenAI(
                azure_endpoint=azure_endpoint,
                api_key=api_key,
                api_version="2025-02-01-preview" 
            )
            print("Success :  OpenAI client initialized")
        except Exception as e:
            print(f"Error :  CRITICAL: OpenAI client initialization failed: {e}")
            if self.conn: self.conn.close()  # Close DB connection if client fails
            raise  # Agent requires the LLM client

        # --- Load Initial Data from Database ---
        self._load_dataframes()
        self._set_initial_room_context()

        print("Success :  Final Spatial AI Agent initialized and ready.")

    @staticmethod
    def _rgb_to_color_name(rgb: Tuple[int, int, int]) -> str:
        """
        Convert RGB values to a human-readable color name.
        Uses a simple color distance algorithm to find the closest named color.
        """
        # Basic color palette with common color names
        colors = {
            'white': (255, 255, 255),
            'black': (0, 0, 0),
            'gray': (128, 128, 128),
            'light gray': (192, 192, 192),
            'dark gray': (64, 64, 64),
            'red': (255, 0, 0),
            'dark red': (139, 0, 0),
            'orange': (255, 165, 0),
            'yellow': (255, 255, 0),
            'green': (0, 128, 0),
            'light green': (144, 238, 144),
            'dark green': (0, 100, 0),
            'blue': (0, 0, 255),
            'light blue': (173, 216, 230),
            'dark blue': (0, 0, 139),
            'cyan': (0, 255, 255),
            'purple': (128, 0, 128),
            'magenta': (255, 0, 255),
            'pink': (255, 192, 203),
            'brown': (165, 42, 42),
            'tan': (210, 180, 140),
            'beige': (245, 245, 220),
            'cream': (255, 253, 208),
            'ivory': (255, 255, 240),
            'silver': (192, 192, 192),
            'gold': (255, 215, 0),
        }
        
        def color_distance(c1, c2):
            """Calculate Euclidean distance between two RGB colors."""
            return sum((a - b) ** 2 for a, b in zip(c1, c2)) ** 0.5
        
        # Find the closest color
        min_distance = float('inf')
        closest_color = 'unknown'
        
        for name, color_rgb in colors.items():
            distance = color_distance(rgb, color_rgb)
            if distance < min_distance:
                min_distance = distance
                closest_color = name
        
        return closest_color

    def _load_dataframes(self):
        """Loads floor and room data from the database into pandas DataFrames."""
        if not self.conn:
            print("Error :  Cannot load DataFrames: Database connection is not available.")
            return

        try:
            self.floors_df = pd.read_sql_query("SELECT * FROM floors ORDER BY floor_number", self.conn)
            self.rooms_df = pd.read_sql_query("""
                SELECT r.*, f.floor_name, f.floor_number
                FROM rooms r
                JOIN floors f ON r.floor_id = f.floor_id
                ORDER BY f.floor_number, r.room_number
            """, self.conn)

            # --- Data Validation and Type Conversion ---
            if not self.rooms_df.empty:
                # Ensure essential columns exist
                essential_cols = ['room_id', 'floor_id', 'floor_number', 'room_type', 'room_number']
                for col in essential_cols:
                    if col not in self.rooms_df.columns:
                        print(
                            f" Warning: Essential column '{col}' missing from 'rooms' table. Creating with defaults.")
                        if col == 'room_type':
                            self.rooms_df['room_type'] = 'unknown'
                        elif col == 'room_number':
                            self.rooms_df['room_number'] = '000'
                        elif col in ['room_id', 'floor_id', 'floor_number']:
                            self.rooms_df[col] = 0

                # Convert numeric types safely
                for col in ['room_id', 'floor_id', 'floor_number']:
                    self.rooms_df[col] = pd.to_numeric(self.rooms_df[col], errors='coerce').fillna(0).astype(int)

                # Ensure room_type exists, fill NaNs, convert to string
                if 'room_type' not in self.rooms_df.columns:
                    self.rooms_df['room_type'] = 'unknown'  # Add column if missing
                self.rooms_df['room_type'] = self.rooms_df['room_type'].fillna('unknown').astype(str)

                # Ensure room_number exists, fill NaNs, convert to string (and pad)
                if 'room_number' not in self.rooms_df.columns:
                    self.rooms_df['room_number'] = '000'  # Add column if missing
                # Ensure it's treated as string before padding
                self.rooms_df['room_number'] = self.rooms_df['room_number'].fillna('000').astype(str)
                # Apply zfill only if the column contains string representations of numbers
                # This check prevents errors if 'room_number' contains non-numeric strings
                if self.rooms_df['room_number'].str.isdigit().all():
                    self.rooms_df['room_number'] = self.rooms_df['room_number'].str.zfill(3)
                else:
                    print(" Warning: 'room_number' column contains non-numeric values. Skipping zero-padding.")

            if not self.floors_df.empty and 'floor_number' in self.floors_df.columns:
                self.floors_df['floor_number'] = pd.to_numeric(self.floors_df['floor_number'], errors='coerce').fillna(
                    0).astype(int)

            print(f"Success :  Found {len(self.floors_df)} floors and {len(self.rooms_df)} rooms in the database.")
            
            # Print summary table
            self._print_database_summary()

        except pd.io.sql.DatabaseError as e:
            print(f"Error :  Database error loading initial data: {e}")
        except Exception as e:
            print(f"Error :  Unexpected error loading initial data: {e}")
            traceback.print_exc()

    def _print_database_summary(self):
        """Prints a summary table of all floors and rooms in the database."""
        if self.rooms_df.empty:
            print("\n No rooms data available for summary.\n")
            return
        
        print("\n" + "=" * 95)
        print(" DATABASE SUMMARY")
        print("=" * 95)
        
        # Group by floor
        if 'floor_number' in self.rooms_df.columns:
            floors = sorted(self.rooms_df['floor_number'].unique())
            
            for floor_num in floors:
                floor_rooms = self.rooms_df[self.rooms_df['floor_number'] == floor_num].sort_values('room_id')
                
                # Get floor name
                floor_name = f"floor_{floor_num}"
                if not self.floors_df.empty and 'floor_name' in self.floors_df.columns:
                    floor_match = self.floors_df[self.floors_df['floor_number'] == floor_num]
                    if not floor_match.empty:
                        floor_name = floor_match.iloc[0]['floor_name']
                
                print(f"\n FLOOR {floor_num} ({floor_name})")
                print("-" * 95)
                print(f"{'ID':<6} {'Room Code':<15} {'Room Type':<20} {'Objects':<10} {'Panoramas':<12} {'Planes CSV':<12}")
                print("-" * 95)
                
                for _, room in floor_rooms.iterrows():
                    room_id = room.get('room_id', '?')
                    room_name = room.get('room_name', 'unknown')
                    room_type = room.get('room_type', 'unknown')
                    
                    # Count objects in this room
                    try:
                        obj_count = pd.read_sql_query(
                            "SELECT COUNT(*) as count FROM objects WHERE room_id = ?",
                            self.conn,
                            params=(room_id,)
                        ).iloc[0]['count']
                    except:
                        obj_count = 0
                    
                    # Count panoramas/images in this room
                    try:
                        pano_count = pd.read_sql_query(
                            "SELECT COUNT(*) as count FROM images WHERE room_id = ?",
                            self.conn,
                            params=(room_id,)
                        ).iloc[0]['count']
                        pano_display = f"{pano_count} images"
                    except:
                        pano_count = 0
                        pano_display = "0 images"
                    
                    # Check for planes in database
                    planes_csv_status = "Error: No planes"
                    try:
                        plane_count = pd.read_sql_query(
                            "SELECT COUNT(*) as count FROM planes WHERE room_id = ?",
                            self.conn,
                            params=(room_id,)
                        ).iloc[0]['count']
                        if plane_count > 0:
                            planes_csv_status = f"Success: {plane_count} planes"
                    except:
                        planes_csv_status = "Error: No planes"
                    
                    print(f"{room_id:<6} {room_name:<15} {room_type:<20} {obj_count:<10} {pano_display:<12} {planes_csv_status:<12}")
        
        print("=" * 95 + "\n")

    def _set_initial_room_context(self):
        """Sets the initial room context, preferring a room with objects."""
        if not self.conn or self.rooms_df.empty or 'room_id' not in self.rooms_df.columns:
            print(" Cannot set initial room context: No rooms loaded or 'room_id' missing.")
            self.current_room_id = None
            return

        try:
            room_ids_list = self.rooms_df['room_id'].tolist()
            if not room_ids_list:
                print("ℹ️ No room IDs found in DataFrame to query for objects.")
                self.current_room_id = None
                return

            placeholders = ','.join(['?'] * len(room_ids_list))
            query_sql = f"""
                SELECT DISTINCT r.room_id FROM rooms r
                JOIN objects o ON r.room_id = o.room_id
                WHERE r.room_id IN ({placeholders})
                LIMIT 1
            """
            rooms_with_objects_df = pd.read_sql_query(query_sql, self.conn, params=room_ids_list)

            if not rooms_with_objects_df.empty:
                self.current_room_id = int(rooms_with_objects_df.iloc[0]['room_id'])
                room_info_rows = self.rooms_df[self.rooms_df['room_id'] == self.current_room_id]
                if not room_info_rows.empty:
                    room_info = room_info_rows.iloc[0]
                    print(
                        f"Success: Auto-selected initial room with objects: {room_info.get('room_name', 'N/A')} on {room_info.get('floor_name', 'N/A')} (ID: {self.current_room_id})")
                else:
                    print(f"Error: Could not find details for auto-selected room ID: {self.current_room_id}")
            elif not self.rooms_df.empty:
                self.current_room_id = int(self.rooms_df.iloc[0]['room_id'])
                room_info = self.rooms_df.iloc[0]
                print(
                    f"Success: Defaulting to first room (may lack objects): {room_info.get('room_name', 'N/A')} (ID: {self.current_room_id})")
            else:
                print("No rooms available to set as default context.")
                self.current_room_id = None

        except pd.io.sql.DatabaseError as e:
            print(f"Error: Database error setting initial context: {e}")
            self.current_room_id = None
        except Exception as e:
            print(f"Error: Unexpected error setting initial context: {e}")
            traceback.print_exc()
            self.current_room_id = None

    # --- Query Parsing Helpers ---
    def _parse_room_reference(self, query: str) -> Optional[Tuple[str, int]]:
        """Parses 'room_XXX [on floor_Y]' or room name like 'kitchen' references from the query."""
        query_lower = query.lower()

        # Pattern 1: room_XXX on floor_Y (flexible spacing) - CHECK THIS FIRST (most specific)
        pattern1 = r'room[\s_]?(\d+)\s*(?:on\s+)?floor[\s_]?(\d+)'
        match1 = re.search(pattern1, query_lower)
        if match1:
            room_num_str = match1.group(1).zfill(3)
            floor_num_int = int(match1.group(2))
            print(f"   Parser found ref: room {room_num_str} on floor {floor_num_int}")
            return (room_num_str, floor_num_int)

        # Pattern 0: room code format (e.g., "0-4", "show 0-2", "room 1-3")
        # This matches floor-room code like "0-4" meaning floor 0, room 4
        # Should NOT match: "9-9-9" (object code), "what color is 0-3-0" (object code)
        # Should match: "show room 0-2", "in 1-3", "display 0-4"
        pattern0 = r'(?:room|show|display|visualize|in|floor)\s+(\d+)-(\d+)\b(?!-\d+)'
        match0 = re.search(pattern0, query_lower)
        if match0:
            floor_num_int = int(match0.group(1))
            room_num_int = int(match0.group(2))
            room_num_str = str(room_num_int).zfill(3)
            print(f"   Parser found ref: room code {match0.group(1)}-{match0.group(2)} -> room {room_num_str} on floor {floor_num_int}")
            return (room_num_str, floor_num_int)

        # Pattern 2: in room_XXX (infers floor from current context if possible)
        pattern2 = r'in\s+room[\s_]?(\d+)'
        match2 = re.search(pattern2, query_lower)
        if match2:
            room_num_str = match2.group(1).zfill(3)
            floor_num_int = 0  # Default floor
            if self.current_room_id and not self.rooms_df.empty:
                current_room_rows = self.rooms_df[self.rooms_df['room_id'] == self.current_room_id]
                if not current_room_rows.empty:
                    floor_val = current_room_rows.iloc[0].get('floor_number')
                    if floor_val is not None: floor_num_int = int(floor_val)
            print(f"   Parser found ref: room {room_num_str} (inferred floor {floor_num_int})")
            return (room_num_str, floor_num_int)

        # Explicit room type name with NLP context
        # Matches: "show [the] kitchen", "visualize bedroom", "display bathroom", etc.
        if not self.rooms_df.empty and 'room_type' in self.rooms_df.columns:
            semantic_types = self.rooms_df['room_type'].unique().tolist()
            # Filter out 'unknown' or None if necessary
            semantic_types = [t for t in semantic_types if t and t != 'unknown']

            # Common verbs/phrases indicating room visualization/query
            context_patterns = [
                r'(?:show|display|visualize|view|see|open|go\s+to|switch\s+to|look\s+at|check|inspect|examine|explore)',
                r'(?:what(?:\'?s|\s+is)|tell\s+me\s+about|describe|give\s+me)',
                r'(?:in|inside|within|at)',
            ]
            optional_article = r'(?:\s+(?:the|a|an|this|that|my))?'
            
            for room_type in semantic_types:
                # Create pattern: (context_words) [optional article] room_type
                room_type_pattern = re.escape(room_type.replace('_', ' '))
                
                # Try each context pattern
                for context_pattern in context_patterns:
                    full_pattern = f'{context_pattern}{optional_article}\\s+{room_type_pattern}\\b'
                    
                    if re.search(full_pattern, query_lower):
                        # Find rooms matching this type
                        match_df = self.rooms_df[self.rooms_df['room_type'] == room_type]
                        if not match_df.empty:
                            # Check for ambiguity
                            if len(match_df) > 1:
                                rooms_list = ", ".join([f"room {row['room_number']} on floor {row['floor_number']}" 
                                                       for _, row in match_df.iterrows()])
                                print(f"    Parser found multiple {room_type}s: {rooms_list}. Using first match.")
                            
                            room_num_str = match_df.iloc[0].get('room_number', '000')
                            floor_num_int = int(match_df.iloc[0].get('floor_number', 0))
                            room_id = int(match_df.iloc[0].get('room_id', 0))
                            print(f"   Parser found ref (NLP): '{room_type}' -> room {room_num_str} on floor {floor_num_int} (ID: {room_id})")
                            # Ensure room_num_str is padded if needed
                            if room_num_str.isdigit(): room_num_str = room_num_str.zfill(3)
                            return (room_num_str, floor_num_int)
                
                # Fallback: room type anywhere in query with word boundaries
                if re.search(r'\b' + room_type_pattern + r'\b', query_lower):
                    match_df = self.rooms_df[self.rooms_df['room_type'] == room_type]
                    if not match_df.empty:
                        # Check for ambiguity
                        if len(match_df) > 1:
                            rooms_list = ", ".join([f"room {row['room_number']} on floor {row['floor_number']}" 
                                                   for _, row in match_df.iterrows()])
                            print(f"    Parser found multiple {room_type}s: {rooms_list}. Using first match.")
                        
                        room_num_str = match_df.iloc[0].get('room_number', '000')
                        floor_num_int = int(match_df.iloc[0].get('floor_number', 0))
                        room_id = int(match_df.iloc[0].get('room_id', 0))
                        print(f"   Parser found ref (semantic): '{room_type}' -> room {room_num_str} on floor {floor_num_int} (ID: {room_id})")
                        # Ensure room_num_str is padded if needed
                        if room_num_str.isdigit(): room_num_str = room_num_str.zfill(3)
                        return (room_num_str, floor_num_int)

        # print("   Parser found no specific room reference.")
        return None

    def _find_room_by_reference(self, room_num: str, floor_num: int) -> Optional[int]:
        """Finds a room_id in the DataFrame based on room and floor number."""
        if self.rooms_df.empty or 'room_number' not in self.rooms_df.columns or 'floor_number' not in self.rooms_df.columns:
            print("Error: Cannot find room by reference: rooms_df not loaded or missing columns.")
            return None

        # Ensure comparison uses padded room number string
        room_num_padded = room_num.zfill(3) if room_num.isdigit() else room_num

        room_match = self.rooms_df[
            (self.rooms_df['room_number'] == room_num_padded) &
            (self.rooms_df['floor_number'] == floor_num)
            ]

        if not room_match.empty:
            room_id_val = room_match.iloc[0].get('room_id')
            if room_id_val is not None:
                return int(room_id_val)
        print(f"   Finder: No room found matching {room_num_padded} on floor {floor_num}.")
        return None

    # --- Data Fetching ---
    def get_room_summary(self, room_id: int) -> Dict[str, Any]:
        """Retrieves and caches detailed summary for a specific room ID."""
        if room_id in self.room_cache:
            return self.room_cache[room_id]
        if not self.conn: return {}

        try:
            params = (room_id,)
            room_df = pd.read_sql_query("""
                SELECT r.*, f.floor_name, f.floor_number FROM rooms r
                JOIN floors f ON r.floor_id = f.floor_id WHERE r.room_id = ?
            """, self.conn, params=params)
            if room_df.empty:
                print(f"Error: No room details found in DB for ID: {room_id}")
                return {}

            objects_df = pd.read_sql_query("SELECT * FROM objects WHERE room_id = ? ORDER BY class, object_code",
                                           self.conn, params=params)
            planes_df = pd.read_sql_query("SELECT * FROM planes WHERE room_id = ? ORDER BY plane_class, area DESC",
                                          self.conn, params=params)
            images_df = pd.read_sql_query("SELECT * FROM images WHERE room_id = ? ORDER BY image_id", self.conn,
                                          params=params)

            summary = {
                "room": room_df.iloc[0].to_dict(),
                "objects": objects_df.to_dict('records'),
                "planes": planes_df.to_dict('records'),
                "images": images_df.to_dict('records')
            }
            self.room_cache[room_id] = summary
            return summary
        except Exception as e:
            print(f"Error: Getting room summary for room {room_id}: {e}")
            return {}

    def _get_available_rooms_list(self) -> List[str]:
        """Returns a list of available room codes in the format 'room_XXX (floor Y)'."""
        if self.rooms_df.empty:
            return []
        
        available = []
        for _, row in self.rooms_df.iterrows():
            room_name = row.get('room_name', 'unknown')
            floor_num = row.get('floor_number', '?')
            room_type = row.get('room_type', '')
            type_str = f" ({room_type})" if room_type and room_type != 'unknown' else ""
            available.append(f"{room_name} on floor {floor_num}{type_str}")
        return available

    def _get_all_rooms_data(self) -> List[Dict[str, Any]]:
        """Retrieves summaries for all rooms, calculating aggregate stats."""
        all_rooms_data = []
        if self.rooms_df.empty or 'room_id' not in self.rooms_df.columns:
            print("Error: Cannot get all rooms data: rooms_df not loaded or missing 'room_id'.")
            return []

        for room_id in self.rooms_df['room_id'].tolist():
            room_summary = self.get_room_summary(room_id)
            if room_summary and 'room' in room_summary:
                all_objects = room_summary.get('objects', [])
                all_planes = room_summary.get('planes', [])

                room_summary['room']['object_count'] = len(all_objects)

                wall_planes = [p for p in all_planes if p.get('plane_class', '').lower() == 'wall']
                room_summary['room']['wall_count'] = len(wall_planes)
                room_summary['room']['wall_area_total'] = sum(p.get('area', 0.0) for p in wall_planes)

                # Get top object classes safely (kept for brevity in multi-room if needed later)
                if all_objects:
                    try:
                        objects_df = pd.DataFrame(all_objects)
                        if 'class' in objects_df.columns:
                            class_counts = objects_df.groupby('class').size().nlargest(5)
                            details = {}
                            for class_name, count in class_counts.items():
                                codes = [obj['object_code'] for obj in all_objects if
                                         obj.get('class') == class_name and 'object_code' in obj]
                                details[class_name] = f"{class_name.upper()} ({count}) [{', '.join(codes[:5])}]"
                            room_summary['room'][
                                'top_objects_details'] = details  # Still calculate for potential future use
                        else:
                            room_summary['room']['top_objects_details'] = {}
                    except Exception as e:
                        print(f"Error processing object details for room {room_id}: {e}")
                        room_summary['room']['top_objects_details'] = {"Error": "Processing failed"}
                else:
                    room_summary['room']['top_objects_details'] = {}

                all_rooms_data.append(room_summary)
        return all_rooms_data

    # --- Prompt Engineering ---
    def _create_system_prompt(self, room_data: Union[Dict, List[Dict]]) -> str:
        """Generates the system prompt based on the current data scope."""
        # Base prompt explaining role and tools
        prompt = """You are an Advanced Spatial AI Assistant specializing in architectural space analysis.

═══════════════════════════════════════════════════════════════════════════════
CORE IDENTITY & MISSION
═══════════════════════════════════════════════════════════════════════════════
Your primary mission is to provide PRECISE, DATA-DRIVEN spatial analysis by combining:
- Geometric data from point cloud processing
- Visual context from room photography
- Specialized computational tools for 3D analysis
- Multi-modal reasoning about building interiors and spatial relationships

═══════════════════════════════════════════════════════════════════════════════
SYSTEM ARCHITECTURE
═══════════════════════════════════════════════════════════════════════════════
You operate within a multi-component system:
1. **Database Layer**: SQLite database with floors, rooms, objects, planes, images
2. **Computational Pipeline**: C++ tools for geometry processing (reconstruction, volume, color, distance)
3. **API Layer**: Python wrapper dispatching operations to C++ executables
4. **Query Processing**: Two-stage LLM workflow (scope classification → answer generation)
5. **Visualization Engine**: Web-based 3D point cloud viewer for interactive exploration

═══════════════════════════════════════════════════════════════════════════════
DATA MODEL & CONVENTIONS
═══════════════════════════════════════════════════════════════════════════════

HIERARCHICAL STRUCTURE:
Building → Floor (floor_0, floor_1, ...) → Room (room_001, room_002, ...) → Objects

IDENTIFICATION CODES:
1. **Room Code**: <floor_id>-<room_id>
   Example: "0-7" = Floor 0, Room 7
   
2. **Object Code**: <floor_id>-<room_id>-<object_id>
   Example: "0-7-12" = Object 12 in Room 7 on Floor 0
   
3. **Room Types**: Semantic labels used in 'room_type' field
   Examples: 'kitchen', 'bedroom', 'bathroom', 'hallway', 'living_room'
   ⚠️ Use the 'room_type' field to answer questions about room categories

═══════════════════════════════════════════════════════════════════════════════
AVAILABLE TOOLS (5 SPECIALIZED OPERATIONS)
═══════════════════════════════════════════════════════════════════════════════

1. **VOLUME (VOL)** - Calculate 3D mesh volume
   Input: Single object code (e.g., "0-7-12")
   Output: Volume in cubic meters, mesh status (closed/unclosed)
   Use Cases: "What is the volume of the couch?", "How much space does object 0-7-12 occupy?"

2. **COLOR (CLR)** - Analyze dominant colors using Gaussian Mixture Model
   Input: Single object code (e.g., "0-7-12")
   Output: RGB values [0-255] with weights for each color component
   ⚠️ CRITICAL: You receive RAW RGB values. You MUST interpret them into human-readable color names.
   
   RGB INTERPRETATION GUIDE:
   • Brightness Levels:
     [0-50]: Very dark/black tones
     [50-100]: Dark tones
     [100-150]: Medium-dark
     [150-200]: Medium-light
     [200-255]: Light/bright
     
   • Hue Identification:
     High R, low G/B → Reds/pinks
     High G, low R/B → Greens
     High B, low R/G → Blues
     High R+G, low B → Yellows/oranges
     High R+B, low G → Purples/magentas
     Similar R/G/B → Grays/whites
     
   • EXAMPLES:
     [45, 78, 120] → "dark steel blue"
     [180, 195, 185] → "soft sage green"
     [220, 180, 160] → "light peachy pink"
     [200, 50, 50] → "bright red"
     [90, 85, 88] → "dark charcoal gray"
   
   Use Cases: "What color is the chair?", "Describe the dominant colors of object 0-2-3"

3. **DISTANCE (BBD)** - Calculate Euclidean distance between object centers
   Input: Two object codes (e.g., "0-7-12 0-7-15")
   Output: Distance in meters, 3D vector from object 1 to object 2
   Use Cases: "How far apart are the chair and table?", "Distance between 0-7-1 and 0-7-5?"

4. **RECONSTRUCT (RCN)** - Generate 3D mesh from point cloud
   Input: Object code
   Output: Path to reconstructed .ply mesh file
   Note: Usually auto-triggered by VOL operations if mesh is missing

5. **VISUALIZE (VIS)** - Launch interactive 3D point cloud viewer
   Input: Room codes and/or object codes (1 or more)
   Output: Browser URL for 3D visualization
   
   Visualization Modes (auto-detected):
   • **room**: Single room code → entire room with shell + all objects
   • **clusters**: Object codes only → selected objects without room context
   • **multi-rooms**: Multiple room codes → multiple room shells (floor overview)
   • **room-with-objects**: Room code + object codes → room context with highlighted objects
   
   Use Cases: "Show me the kitchen", "Visualize chairs in room 0-2", "Display rooms 0-1, 0-2, 0-3"

═══════════════════════════════════════════════════════════════════════════════
TOOL INVOCATION PROTOCOL
═══════════════════════════════════════════════════════════════════════════════

EXPLICIT TOOL CALL FORMAT:
When user provides specific object codes or clear visualization keywords, output:
TOOL: [HEAD_CODE] [code1] [code2_optional]

Examples:
• TOOL: CLR 0-2-3 (color analysis of one object)
• TOOL: VOL 1-5-10 (volume of one object)
• TOOL: BBD 0-1-1 0-1-5 (distance between two objects)
• TOOL: VIS 0-7 (visualize room 0-7)
• TOOL: VIS 0-7-12 0-7-15 (visualize specific objects)

NLP-TRIGGERED TOOL CALLS:
When user uses keywords WITHOUT explicit codes, attempt to resolve using:
1. Current room context (single-room scope)
2. Object class matching (e.g., "chair", "table", "door")
3. Room type matching (e.g., "kitchen", "bedroom")

Keyword Triggers:
• CLR: "color", "colour", "dominant color"
• BBD: "distance between", "how far apart", "separation"
• VOL: "volume", "mesh volume", "closed volume"
• VIS: "show", "visualize", "display", "view", "render", "see"

TOOL OUTPUT HANDLING:
When you receive "EXTERNAL API RESULT" prefix:
1. DO NOT repeat the tool command
2. DO NOT show raw API output structure
3. SYNTHESIZE a natural language answer incorporating the data
4. INTERPRET RGB values into color names (for CLR)
5. PROVIDE CONTEXT using room data

Example:
Input: EXTERNAL API RESULT (TOOL CLR via explicit codes):
CLR (0-2-3): Found 2 colors: [W: 0.70, RGB: (180, 195, 185)], [W: 0.30, RGB: (95, 110, 100)]

Your Response:
"The object 0-2-3 has two dominant colors: a light mint green (70% coverage) and a darker forest green (30%). This suggests a two-tone color scheme with lighter tones being more prominent."

═══════════════════════════════════════════════════════════════════════════════
ROOM DATA CONTEXT (PROVIDED BELOW)
═══════════════════════════════════════════════════════════════════════════════
The following section contains the room(s) relevant to your query. Use this data for all answers.

"""
        # --- Check for duplicate room types and add warning ---
        if isinstance(room_data, list) and len(room_data) > 0:
            room_types = {}
            for rd in room_data:
                room = rd.get('room', {})
                room_type = room.get('room_type', 'unknown')
                room_name = room.get('room_name', 'N/A')
                floor_num = room.get('floor_number', 'N/A')
                if room_type != 'unknown':
                    room_types.setdefault(room_type, []).append(f"room {room_name} on floor {floor_num}")
            
            # Check for duplicates
            duplicates = {rt: rooms for rt, rooms in room_types.items() if len(rooms) > 1}
            if duplicates:
                prompt += "\n⚠️ IMPORTANT: The user asked about a room type that has multiple instances:\n"
                for room_type, rooms in duplicates.items():
                    prompt += f"   • {room_type.upper()}: {len(rooms)} rooms ({', '.join(rooms)})\n"
                prompt += "\n   ⚠️ CRITICAL INSTRUCTION:\n"
                prompt += "   Since the user asked about 'the " + list(duplicates.keys())[0] + "' (singular) but there are multiple,\n"
                prompt += "   you MUST provide information for ALL matching rooms.\n"
                prompt += "   Format: 'There are X [room types]. In [room_name1], [answer]. In [room_name2], [answer].'\n\n"
        
        # --- Data Context ---
        prompt_includes_dimensions = False  # Flag remains for clarity, though always true now
        if isinstance(room_data, dict):  # Single room context
            room = room_data.get('room', {})
            objects = room_data.get('objects', [])
            planes = room_data.get('planes', [])

            total_area = room.get('total_area', 0.0) or 0.0
            length = room.get('length', 0.0) or 0.0
            width = room.get('width', 0.0) or 0.0

            # Check if there are duplicate room types in the building
            current_room_type = room.get('room_type', 'unknown')
            if current_room_type != 'unknown' and not self.rooms_df.empty and 'room_type' in self.rooms_df.columns:
                same_type_rooms = self.rooms_df[self.rooms_df['room_type'] == current_room_type]
                if len(same_type_rooms) > 1:
                    rooms_list = []
                    for _, row in same_type_rooms.iterrows():
                        rooms_list.append(f"room {row['room_name']} on floor {row['floor_number']}")
                    prompt += f"\n⚠️ IMPORTANT: This building has {len(same_type_rooms)} {current_room_type}s: {', '.join(rooms_list)}\n"
                    prompt += f"   You are answering about: {room.get('room_name', 'N/A')} on {room.get('floor_name', 'N/A')}\n"
                    prompt += "   ALWAYS specify which specific room you are referring to in your answer.\n\n"

            prompt += f"\nCURRENT ROOM CONTEXT: {room.get('room_name', 'N/A')} on {room.get('floor_name', 'N/A')} (Type: {room.get('room_type', 'unknown')})\n"
            prompt += f"Area: {total_area:.2f}m², Approx Dimensions: L:{length:.2f}m x W:{width:.2f}m\n\n"

            if objects:
                prompt += "OBJECT INVENTORY:\n" + "=" * 20 + "\n"
                objects_by_class = {}
                for obj in objects:
                    class_name = obj.get('class', 'unknown')
                    objects_by_class.setdefault(class_name, []).append(obj)
                for class_name, class_objects in objects_by_class.items():
                    prompt += f"-> {class_name.upper()} ({len(class_objects)} total):\n"
                    for obj in class_objects:
                        dims = [f"{k[0].upper()}:{v:.2f}m" for k, v in obj.items() if
                                k in ['length', 'width', 'height'] and v is not None]
                        prompt += f"  - {obj.get('obj_name', 'N/A')} (CODE: {obj.get('object_code', 'N/A')}) [{' / '.join(dims)}] @({obj.get('center_x', 0.0):.1f}, {obj.get('center_y', 0.0):.1f})\n"
                prompt += "\n"
                prompt_includes_dimensions = True
            else:
                prompt += "OBJECT INVENTORY: None\n\n"

            if planes:
                prompt += "PLANE DATA:\n" + "=" * 15 + "\n"
                plane_summary = {}
                for plane in planes: plane_summary[plane.get('plane_class', 'unknown')] = plane_summary.get(
                    plane.get('plane_class', 'unknown'), 0) + plane.get('area', 0.0)
                for cls, area in plane_summary.items(): prompt += f"  - {cls.upper()}: Total Area {area:.2f}m²\n"
                prompt += "\n"
            else:
                prompt += "PLANE DATA: None\n\n"

        elif isinstance(room_data, list):  # Multi-room context
            prompt += f"MULTI-ROOM DATA SUMMARY ({len(room_data)} rooms relevant to query):\n" + "═" * 30 + "\n"
            if not room_data:
                prompt += "No specific room data available for this query.\n"
            else:
                for i, rd in enumerate(room_data, 1):
                    room = rd.get('room', {})
                    objects = rd.get('objects', [])
                    planes = rd.get('planes', [])

                    prompt += f"\n{i}. ROOM: {room.get('room_name', 'N/A')} (Floor: {room.get('floor_number', 'N/A')}, Type: {room.get('room_type', 'unknown')})\n"
                    total_area = room.get('total_area', 0.0) or 0.0
                    prompt += f"   - Floor Area: {total_area:.2f}m², Objects: {len(objects)}\n"  # Show floor area and object count
                    
                    # Add plane area summaries for cost calculations
                    if planes:
                        plane_summary = {}
                        for plane in planes:
                            plane_class = plane.get('plane_class', 'unknown')
                            plane_summary[plane_class] = plane_summary.get(plane_class, 0) + plane.get('area', 0.0)
                        if plane_summary:
                            prompt += f"   - Plane Areas: "
                            plane_strs = [f"{cls.upper()}:{area:.2f}m²" for cls, area in plane_summary.items()]
                            prompt += ", ".join(plane_strs) + "\n"

                    # *** ALWAYS ADD FULL OBJECT INVENTORY ***
                    if objects:
                        objects_by_class = {}
                        for obj in objects:
                            class_name = obj.get('class', 'unknown')
                            objects_by_class.setdefault(class_name, []).append(obj)

                        if objects_by_class:
                            prompt += "   - Object Inventory:\n"
                            for class_name, class_objects in objects_by_class.items():
                                prompt += f"     -> {class_name.upper()} ({len(class_objects)} total):\n"
                                for obj in class_objects:
                                    dims = [f"{k[0].upper()}:{v:.2f}m" for k, v in obj.items() if
                                            k in ['length', 'width', 'height'] and v is not None]
                                    prompt += f"       - {obj.get('obj_name', 'N/A')} (CODE: {obj.get('object_code', 'N/A')}) [{' / '.join(dims)}]\n"
                        else:
                            prompt += "   - No objects in this room.\n"
                    else:
                        prompt += "   - No objects in this room.\n"

            if len(
                room_data) > 1: prompt += "Note: Tools (VOL, CLR, BBD, VIS) need specific object codes. Mention the code if asking about a specific object.\n"
            prompt_includes_dimensions = True
        else:
            prompt += "WARNING: No room data context provided.\n"

        # --- Tool Usage & Response Guidelines ---
        prompt += """
═══════════════════════════════════════════════════════════════════════════════
RESPONSE GUIDELINES (CRITICAL - READ CAREFULLY)
═══════════════════════════════════════════════════════════════════════════════

1. STAY WITHIN SCOPE (CRITICAL)
   ⚠️ YOU ARE A SPATIAL ANALYSIS AGENT - ONLY ANSWER QUESTIONS ABOUT ROOMS AND OBJECTS ⚠️
   
   ACCEPTABLE QUERIES (IN SCOPE):
   • Room layouts, dimensions, areas, volumes
   • Object locations, dimensions, colors, distances
   • Furniture counts, types, arrangements
   • Visual appearance, materials, textures, decor (using panoramas)
   • Spatial relationships, proximity, access patterns
   • Plane data (walls, floors, ceilings) - DIMENSIONS ONLY
   • Cost estimates for renovations (painting walls, flooring, etc.) based on spatial data
   
   ⚠️ COLOR ANALYSIS GUIDE ⚠️
   • CLR tool ONLY works for OBJECTS (furniture, doors, windows, etc.)
   • Walls, floors, and ceilings are PLANES - they do NOT have color data in the CLR tool
   
   FOR WALL/FLOOR/CEILING COLORS:
   • If images are provided in the prompt → ANALYZE THE IMAGES and describe the colors you see
   • Look at the panorama images and describe: "Based on the panorama image, the walls appear to be [color description]"
   • You can see the actual colors in the images - describe them naturally
   • If NO images provided → "I don't have visual data for this room. I can analyze colors of furniture, doors, and windows using their object codes."
   
   UNACCEPTABLE QUERIES (OUT OF SCOPE):
   • Cooking recipes, food preparation, general knowledge
   • Weather, news, current events
   • Personal advice, jokes, entertainment
   • Tasks unrelated to architectural/spatial analysis
   
   RESPONSES:
   • If query is OUT OF SCOPE → "I can only help with spatial analysis of rooms and objects in this building. Please ask about room layouts, furniture locations, object colors, distances, or visualizations."
   • If room/object DOESN'T EXIST → "That room/object doesn't exist in the database. Available rooms are: [list room codes]."
   • If asking for PLANE COLORS (walls/floors/ceilings) → Check if images available. If yes, analyze from images. If no, explain: "I don't have color data for walls/floors/ceilings in the database. I can provide their dimensions and areas, or analyze colors of furniture and objects."
   • If data NOT AVAILABLE → "I don't have that information in the spatial database."

2. DATA-DRIVEN ACCURACY
   • Base ALL answers on provided room data or API results
   • NEVER invent dimensions, object counts, or spatial relationships
   • If data is missing: state "Data not available" rather than guessing
   • Show your sources: "Based on object 0-3-5 data..." or "According to room 007 summary..."

3. SEMANTIC ROOM TYPES
   • Use the 'room_type' field (e.g., 'kitchen', 'bedroom') for room category questions
   • When query mentions "kitchen", look for rooms where room_type='kitchen'

4. COLOR INTERPRETATION (CRITICAL FOR CLR TOOL)
   ⚠️ ALWAYS show RGB values first, THEN interpret the color name ⚠️
   
   • ALWAYS include RGB values: "RGB (104, 106, 104)" 
   • THEN provide descriptive color name based on RGB analysis
   • Use this guide for interpretation:
     - R>200, G>200, B>200 → whites/light colors
     - R<50, G<50, B<50 → blacks/dark colors
     - R>G and R>B → reds, oranges, pinks
     - G>R and G>B → greens, limes, olives
     - B>R and B>G → blues, purples, violets
     - R≈G≈B → grays (specify light/medium/dark)
     - R≈G>B → yellows, golds, browns
     - R≈B>G → magentas, purples
   • Consider saturation: low difference between RGB → gray/muted colors
   
   Example: "RGB (104, 106, 104) - a medium gray with very low saturation"
   Example: "RGB (180, 195, 185) - a soft sage green with muted tones"
   Example: "RGB (220, 180, 160) - a peachy beige color"

5. SHOW YOUR WORK & VERIFY CALCULATIONS
   • Include calculations or reasoning steps
   • Reference specific data sources: "Based on object 0-3-0 data..."
   • For comparisons, show all values before conclusion
   
   ⚠️ CRITICAL FOR COMPARISONS (max/min/widest/tallest/most/least):
   • ALWAYS list ALL values being compared with their identifiers
   • EXPLICITLY identify the maximum/minimum value
   • DOUBLE-CHECK your conclusion matches the actual max/min
   • Example: "Comparing chair widths: 0.51m, 0.52m, 0.65m, 0.71m, 0.49m. The maximum is 0.71m."
   • DO NOT just pick a value without verifying it's actually the max/min

6. CONVERSATIONAL FORMATTING (MANDATORY)
   ⚠️ CRITICAL: Write in SHORT, NATURAL sentences - NO bold, NO tables, NO asterisks ⚠️
   
   REQUIRED FORMATTING RULES:
   • Write like talking to a person - use plain English
   • Keep sentences SHORT (under 20 words each)
   • Use simple bullet points (- or •) for lists only when needed
   • NO markdown formatting like **bold** or *italics*
   • Add blank lines between sections for readability
   • Include units naturally: "2.5 meters", "15.3 square meters", "0.42 cubic meters"
   
   FORMATTING EXAMPLES:
   
   Simple Query:
   "The kitchen has 1 chair. Its code is 0-2-4. It measures 0.45m × 0.52m × 0.91m."
   
   List Format (2-4 items):
   The bedroom has these tables:
   - Table 1 (code 1-4-1) is 0.44m × 0.19m, located at (1.4m, 6.3m)
   - Table 2 (code 1-4-2) is 0.30m × 0.30m, located at (1.5m, 6.3m)
   
   Comparison Format (NO tables):
   I checked the chairs in each room.
   Room 001 has 4 chairs. The widest is 0.70 meters.
   Room 002 has 1 chair. It's 0.45 meters wide.
   Room 003 has 5 chairs. The widest is 0.71 meters.
   
   The widest chair is in room 003 at 0.71 meters.
   
   Multi-Section Format:
   The chair (code 0-2-4) and door (code 0-2-7) are 0.93 meters apart.
   
   The chair is at coordinates (5.2m, 3.1m).
   The door is at coordinates (5.5m, 2.8m).
   
   They are very close to each other. This makes it easy to reach the exit from the seating area.

7. NATURAL LANGUAGE
   • DO NOT repeat raw tool commands in responses
   • DO NOT show API output structures like JSON
   • Synthesize technical data into conversational explanations
   • Example: Instead of "CLR (0-2-3): RGB (180, 195, 185)" say "The object has a soft sage green color"

8. VISUALIZATION ACKNOWLEDGMENT (CRITICAL)
   ⚠️ WHEN YOU SEE [VIEWER_URL]...[/VIEWER_URL] IN THE TOOL RESULT OR ANYWHERE IN THE PROMPT ⚠️
   
   • ALWAYS extract the URL between [VIEWER_URL] and [/VIEWER_URL] tags
   • ALWAYS include it in your response - DO NOT ask if they want to see it
   • The visualization is ALREADY prepared and ready - you must show the link
   • Example: "I've prepared a 3D view for you. You can see it here: http://localhost:5173/?manifest=room_0_3.json"
   • Keep explanation short: tell them what they can do (rotate, zoom, select)
   • DO NOT say "I've prepared" or "Would you like to see" without showing the actual link
   • DO NOT ask for permission - the visualization is already done, just give them the link

9. HANDLING TOOL FAILURES
   ⚠️ When a tool fails (CLR, BBD, VIS, VOL), it means the object/room DOESN'T EXIST ⚠️
   
   • If you see "failed" or "FileNotFoundError" or "No assets found" in tool result → Object doesn't exist
   • Response: "That object doesn't exist in the database."
   • DO NOT offer alternative help or suggestions
   • DO NOT try to provide information about non-existent objects
   
   Example: Query "What color is object 9-9-9?" + Tool fails → "Object 9-9-9 doesn't exist in the database."

10. SPATIAL CONTEXT & REASONING
   • Provide meaningful spatial relationships (proximity, arrangement)
   • Describe access patterns: "The door connects the kitchen to the hallway"
   • Note density: "This is a sparsely furnished room with only 4 objects in 25m²"
   • Explain functionality: "This layout suggests a dining area (table + 4 chairs)"

11. STRUCTURED RESPONSE FORMATS
   
   For Lists/Comparisons:
   • Kitchen: 3 chairs, 1 table, 25.4m²
   • Bedroom: 2 chairs, 1 desk, 18.7m²
   • Living Room: 5 chairs, 2 tables, 42.1m²
   
   For Spatial Descriptions:
   1. Chair (0-7-1): Northwest corner @(2.3m, 3.1m)
   2. Table (0-7-5): Center area @(2.9m, 3.1m), 0.6m from chair
   3. Door: East wall, connects to hallway
   
   For Calculations:
   Living Room: 42.10m²
   Kitchen: 25.40m²
   Bedroom: 18.70m²
   ─────────────────────
   Total: 86.20m²

═══════════════════════════════════════════════════════════════════════════════
CRITICAL INSTRUCTIONS
═══════════════════════════════════════════════════════════════════════════════

⚠️ MATHEMATICAL ACCURACY (CRITICAL FOR COMPARISONS):
When comparing numbers to find maximum/minimum/widest/tallest/most/least:
1. **LIST ALL VALUES** with their identifiers before comparing
2. **IDENTIFY THE ACTUAL MAX/MIN** - don't just pick a large/small value
3. **VERIFY**: Check that your answer is actually > all others (max) or < all others (min)
4. **COMMON ERROR**: Picking 0.65 when 0.71 exists, or picking 5 when 7 exists
5. **SOLUTION**: Write out: "Comparing: 0.51, 0.52, 0.65, 0.71, 0.49 → Maximum is 0.71"

Example of CORRECT comparison:
"Room 002 chair widths: 0.51m, 0.52m, 0.65m, 0.71m, 0.49m
Room 003 chair widths: 0.46m, 0.46m, 0.70m, 0.46m
Comparing all: 0.51, 0.52, 0.65, 0.71, 0.49, 0.46, 0.46, 0.70, 0.46
Maximum width: 0.71m (Chair 0-2-6 in Room 002)"

⚠️ TOOL USAGE REQUIREMENT (MANDATORY FOR COLOR/VOLUME/DISTANCE QUERIES):

When user asks about COLOR, VOLUME, or DISTANCE of objects:

1. **LOOK UP object codes** from the OBJECT INVENTORY section above
2. **OUTPUT TOOL commands** for each relevant object (one line per tool call)
3. **FORMAT**: TOOL: [HEAD_CODE] [object_code(s)]

EXAMPLES:
• User: "What color is the table?" 
  → Find table code (e.g., 0-2-3) → Output: TOOL: CLR 0-2-3

• User: "What are the colors of all objects?" or "What color are the objects?"
  → Find ALL codes → Output multiple lines:
  TOOL: CLR 0-2-1
  TOOL: CLR 0-2-2
  TOOL: CLR 0-2-3

• User: "Color of object in room 1?" (vague/singular but room has multiple objects)
  → Find ALL codes in that room → Output:
  TOOL: CLR 1-1-0
  TOOL: CLR 1-1-1
  TOOL: CLR 1-1-2

• User: "Distance between chair and door?"
  → Find chair code (e.g., 0-3-5) and door code (e.g., 0-3-8) → Output: TOOL: BBD 0-3-5 0-3-8

• User: "Volume of the couch 0-5-2?"
  → Output: TOOL: VOL 0-5-2

⚠️ CRITICAL RULES:
- DO NOT say "insufficient data" if object codes exist in OBJECT INVENTORY
- DO NOT refuse to answer - LOOK UP the codes and OUTPUT the TOOL commands
- For vague queries ("the object", "objects", "things"), analyze ALL objects in scope
- Output tool commands FIRST, then add context/explanation after
- Each TOOL command must be on its own line

⚠️ AFTER TOOL EXECUTION - SYNTHESIZE RESULTS:
When you see "═══ TOOL EXECUTION RESULTS ═══" in the response:
1. **TRANSLATE RGB values to color names**: 
   - RGB(224, 214, 197) → "beige" or "light tan"
   - RGB(255, 254, 254) → "white" or "off-white"
   - RGB(200, 230, 245) → "light blue" or "pale cyan"
2. **Write natural language summary**:
   "The room contains three objects with the following colors:
   - Table 1 (1-1-0): Beige/tan color
   - Table 2 (1-1-1): White
   - Door (1-1-2): Light blue"
3. **DO NOT just repeat the raw RGB values** - interpret them for the user
4. **Add context**: Compare colors, note patterns, describe overall aesthetic

⚠️ EXTERNAL API RESULT HANDLING:
- If the prompt starts with 'EXTERNAL API RESULT', immediately synthesize the final answer using that result data
- DO NOT repeat the tool command or show raw output structure
- Interpret and translate the data into natural language

⚠️ VISUALIZATION AUTO-TRIGGER:
- When user asks to "show", "display", "visualize", or "view" a room, the system will automatically trigger visualization
- You should acknowledge the visualization is available in your response

═══════════════════════════════════════════════════════════════════════════════
ADVANCED CAPABILITIES
═══════════════════════════════════════════════════════════════════════════════

MULTI-MODAL ANALYSIS:
⚠️ WHEN IMAGES ARE PROVIDED IN THE PROMPT, YOU MUST ANALYZE THEM ⚠️
- Images show the actual room appearance - describe what you SEE in the images
- For wall/floor/ceiling colors: Look at the panorama images and describe the colors
- Example: "Based on the panorama image, the walls are painted in a light beige color"
- Integrate visual observations with geometric data from the database
- Describe materials, textures, lighting, and design style visible in images
- DO NOT say "I don't have visual data" when images are clearly provided in the prompt

SPATIAL REASONING:
- Proximity: "The chair is near the table (0.6m apart)"
- Arrangement: "Chairs are arranged in a line along the west wall"
- Access: "The room has two doors providing access from hallway and living room"
- Density: "This is a sparsely furnished room with only 4 objects in 25m²"

SEMANTIC UNDERSTANDING:
- Functionality: "This layout suggests a dining area (table + 4 chairs)"
- Capacity: "The room can comfortably seat 6 people"
- Purpose: "The bedroom appears to be a child's room (smaller furniture dimensions)"

COMPARATIVE ANALYSIS (Multi-Room Queries):
- Rankings: "Kitchen has the most chairs (5), followed by living room (3)"
- Distributions: "Bedrooms average 15m², while common areas average 30m²"
- Patterns: "All rooms on floor 0 have ceiling height 2.8m"

═══════════════════════════════════════════════════════════════════════════════
COST ESTIMATION CAPABILITIES
═══════════════════════════════════════════════════════════════════════════════

When asked about renovation costs (painting, flooring, etc.), provide budget estimates using:

PAINTING WALLS:
1. Calculate wall area from planes data (sum all 'wall' class planes for the room/house)
2. Use typical cost ranges based on your training data:
   • Basic paint: $2-4 per m² (materials + labor)
   • Mid-range paint: $5-8 per m²
   • Premium paint: $10-15 per m²
3. Show calculation breakdown:
   "Kitchen Wall Area: The kitchen has X m² of wall surface (based on planes data).
   
   Budget Estimate:
   • Low-end (basic paint): X m² × $3/m² = $Y
   • Mid-range (quality paint): X m² × $6.50/m² = $Z
   • High-end (premium paint): X m² × $12/m² = $W
   
   Reasoning: These estimates include paint materials and labor. The kitchen walls appear to be in [describe condition from panorama if available]. Consider mid-range paint for kitchens due to moisture and cleaning needs."

FLOORING:
1. Use room's total_area field (calculated from floor planes)
2. Apply typical flooring costs:
   • Laminate: $20-40 per m²
   • Vinyl/LVP: $30-60 per m²
   • Hardwood: $60-120 per m²
   • Tile: $40-80 per m²
3. Show calculation:
   "House Floor Area: Total floor area is X m² across Y rooms.
   
   Budget Estimate:
   • Laminate flooring: X m² × $30/m² = $Y
   • Vinyl plank: X m² × $45/m² = $Z
   • Hardwood: X m² × $90/m² = $W
   • Ceramic tile: X m² × $60/m² = $V
   
   Reasoning: Based on the current flooring visible in panoramas [describe what you see], I'd recommend [option] because [reason]. This estimate includes materials, underlayment, and installation labor."

GENERAL GUIDELINES:
• Always show the area calculation first (from planes/room data)
• Provide 3 price tiers (low/mid/high) when possible
• Include reasoning based on room type, current condition (from images), and use case
• Mention what's included (materials, labor, preparation)
• Note regional variations: "Prices may vary by location; these are typical ranges"
• Consider room-specific factors (e.g., kitchens need washable paint, bathrooms need moisture-resistant flooring)

═══════════════════════════════════════════════════════════════════════════════
LIMITATIONS & CONSTRAINTS
═══════════════════════════════════════════════════════════════════════════════

Be transparent about system boundaries:
• No Real-Time Sensing: Data is from previous scans, not live
• Visual Analysis Available: When panorama images are provided, you CAN analyze materials, finishes, textures, colors, and appearances
• Tool Dependencies: Volume/Color from point clouds require successful reconstruction
• Scope Constraints: Cannot modify room layouts or add objects
• Precision: Measurements accurate to ±0.01m typically

DUAL-SOURCE ANALYSIS (CRITICAL - When BOTH tool results AND images are available):
⚠️ ALWAYS provide COMPARATIVE analysis showing BOTH perspectives ⚠️

When you have BOTH tool results (CLR/geometric data) AND panorama images:
1. First show the TOOL/CSV data: "Based on point cloud analysis: RGB (104, 106, 104) - medium gray"
2. Then show PANORAMA observations: "From the panorama images: The object appears to have..."
3. Compare and synthesize: "The geometric data shows gray tones, which is consistent with the muted appearance in the panoramas"

Example format for color queries with both sources:
"Point Cloud Analysis: The object has RGB (104, 106, 104), which is a medium gray covering 57% of the surface.

Panorama Observation: Looking at the room images, the object appears to blend with the neutral color scheme of the space. The lighting and surrounding context show it has a subtle, non-reflective finish.

Conclusion: Both sources confirm a neutral gray color palette for this object."

When visual queries are asked WITH images (no tool data):
• Analyze the panorama images to describe materials, finishes, colors, textures
• Describe what you see: "The kitchen has white painted walls, wooden cabinets, and tile flooring"
• Be specific about visual details visible in the images

When VIS tool provides 3D viewer link AND panorama images are present:
• FIRST answer the visual analysis question using the panorama images
• THEN mention the 3D viewer link at the end as an additional resource
• DO NOT reject the query just because VIS was triggered - the panoramas still allow visual analysis

When data is insufficient:
• State clearly: "I don't have sufficient data to answer this"
• Explain what's missing: "Color analysis requires a point cloud cluster, which is not available"
• Offer alternatives: "However, I can provide the object's dimensions and location"

"""
        return prompt

    # --- Visual Analysis ---
    def _needs_visual_analysis(self, query: str) -> bool:
        """Checks if the query contains keywords suggesting visual analysis."""
        visual_keywords = [
            'appearance', 'look', 'style', 'design', 'decor', 'aesthetic',
            'finish', 'material', 'texture', 'lighting', 'ambiance', 'see',
            'visible', 'describe', 'visual', 'picture', 'photo', 'image', 'panorama',
            'color', 'colour', 'colors', 'colours', 'paint', 'painted', 'shade', 'tint', 'hue'
        ]
        query_lower = query.lower()
        return any(re.search(r'\b' + keyword + r'\b', query_lower) for keyword in visual_keywords)

    def _get_room_images(self, room_id: int) -> List[Dict[str, str]]:
        """Loads and prepares images (base64 encoded) for a given room ID."""
        if not self.conn: return []
        processed_images = []
        try:
            image_paths_tuples = self.conn.execute("SELECT image_path, image_name FROM images WHERE room_id = ?",
                                                   (room_id,)).fetchall()
            if not image_paths_tuples:
                print(f"  No images found in database for room ID {room_id}")
                return []

            for image_path, image_name in image_paths_tuples:
                if not image_path or not os.path.exists(image_path):
                    print(f"  Error: Image file NOT FOUND or path missing: {image_path or 'N/A'}")
                    continue
                try:
                    with Image.open(image_path) as img:
                        if img.mode != 'RGB': img = img.convert('RGB')
                        max_size = 1024
                        if max(img.size) > max_size:
                            img.thumbnail((max_size, max_size), Image.Resampling.LANCZOS)
                        buffer = io.BytesIO()
                        img.save(buffer, format='JPEG', quality=85)
                        image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
                        processed_images.append({
                            "base64": image_base64,
                            "name": image_name or os.path.basename(image_path)
                        })
                except Exception as e:
                    print(f"  ⚠ Error processing image {image_path}: {e}")
            return processed_images
        except Exception as e:
            print(f"⚠ Error loading images from database for room {room_id}: {e}")
            return []

    def _add_images_to_messages(self, messages: List[Dict], images: List[Dict[str, str]]) -> List[Dict]:
        """Adds image data to the user message content for multimodal input."""
        if not images or not messages: return messages
        if not messages: return messages
        last_message = messages[-1]
        if not isinstance(last_message, dict) or "role" not in last_message or "content" not in last_message:
            print(" Cannot add images: Last message format incorrect.")
            return messages

        if last_message.get("role") != "user":
            print(" Adding new user message container for images.")
            messages.append({"role": "user", "content": [{"type": "text", "text": ""}]})
            last_message = messages[-1]

        content = last_message.get("content", "")
        if isinstance(content, str):
            content = [{"type": "text", "text": content}]
        elif not isinstance(content, list):
            content = [{"type": "text", "text": str(content)}]
        if not content or content[0].get("type") != "text": content.insert(0, {"type": "text", "text": ""})

        image_detail = "low" if len(images) > 1 else "high"
        print(f"   Adding {len(images)} images with detail='{image_detail}'")
        for img in images:
            if img.get("base64"):
                content.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{img['base64']}", "detail": image_detail}
                })
        last_message["content"] = content
        return messages

    # --- Tool Execution ---
    def _find_object_codes_by_class(self, room_id: int, class_name: str) -> List[str]:
        """Finds all object codes in a room matching a specific class name."""
        # Ensure room_id is valid before proceeding
        if room_id is None:
            print(" Cannot find objects: Room ID is None.")
            return []

        room_summary = self.get_room_summary(room_id)
        if not room_summary or 'objects' not in room_summary:
            print(f" Cannot find objects: No summary or objects found for room ID {room_id}.")
            return []

        class_name_lower = class_name.lower()
        matching_codes = [
            obj['object_code'] for obj in room_summary['objects']
            if obj.get('class', '').lower() == class_name_lower and 'object_code' in obj
        ]
        return matching_codes

    def _find_classes_in_query(self, query: str) -> List[str]:
        """Finds known object class keywords mentioned in the query (handles plurals)."""
        class_keywords = [
            'chair', 'table', 'window', 'door', 'shell', 'couch', 'plant',
            'monitor', 'curtain', 'sofa', 'desk', 'cabinet'
        ]
        found_classes = []
        query_lower = query.lower()
        
        for keyword in class_keywords:
            # Check singular form
            if re.search(r'\b' + re.escape(keyword) + r'\b', query_lower):
                found_classes.append(keyword)
                continue  # Skip plural check if singular already found
            
            # Check plural form (simple: add 's')
            plural = keyword + 's'
            if re.search(r'\b' + re.escape(plural) + r'\b', query_lower):
                found_classes.append(keyword)  # Add singular form
        
        return list(set(found_classes))

    def _parse_and_execute_tool(self, user_query: str, tool_context_room_id: Optional[int]) -> Optional[
        str]:  # MODIFIED Parameter Name
        """
        Parses query for explicit tool calls (codes + keywords) OR implicit calls
        (keywords + context) and executes them. Uses tool_context_room_id for NLP.
        """
        query_lower = user_query.lower()
        valid_codes = [c for c in re.findall(r'([\d\-]+)', user_query) if re.fullmatch(r'\d+-\d+-\d+', c)]
        has_specific_codes = bool(valid_codes)

        tool_keywords = {
            'CLR': ['color', 'colour', 'dominant color'],
            'BBD': ['distance between', 'how far apart', 'separation'],
            'VOL': ['volume', 'mesh volume', 'closed volume'],
            'VIS': ['show', 'visualize', 'display', 'point cloud', '3d model']
        }
        tool_to_run = None
        keyword_found = False
        for code, keywords in tool_keywords.items():
            if any(re.search(r'\b' + kw + r'\b', query_lower) for kw in keywords):
                tool_to_run = code
                keyword_found = True
                break

        target_codes = []
        nlp_triggered_bbd = False

        if keyword_found:
            if has_specific_codes:
                target_codes = valid_codes
                print(f" Tool '{tool_to_run}' triggered by keyword and specific code(s): {target_codes}")
            # --- NLP Trigger Logic (Uses tool_context_room_id) ---
            elif tool_context_room_id is not None:  # Check if context ID is available
                if tool_to_run in ['CLR', 'VOL', 'VIS']:
                    print(
                        f" Tool '{tool_to_run}' triggered by keyword, attempting NLP in room {tool_context_room_id}...")
                    found_classes = self._find_classes_in_query(user_query)
                    if found_classes:
                        # Use _find_object_codes_by_class with the correct context ID
                        target_codes = self._find_object_codes_by_class(tool_context_room_id, found_classes[0])
                        if target_codes:
                            print(f" NLP found {len(target_codes)} '{found_classes[0]}' codes for {tool_to_run}.")
                        else:
                            print(
                                f" NLP found no '{found_classes[0]}' objects in room {tool_context_room_id} for {tool_to_run}.")
                    else:
                        print(f" {tool_to_run} NLP failed: No recognizable object class found in query.")
                    if not target_codes and tool_to_run != 'VIS': return None
                elif tool_to_run == 'BBD':
                    print(f" Tool 'BBD' triggered by keyword, attempting NLP in room {tool_context_room_id}...")
                    found_classes = self._find_classes_in_query(user_query)
                    if len(found_classes) == 2:
                        class1, class2 = found_classes[0], found_classes[1]
                        # Use _find_object_codes_by_class with the correct context ID
                        codes1 = self._find_object_codes_by_class(tool_context_room_id, class1)
                        codes2 = self._find_object_codes_by_class(tool_context_room_id, class2)

                        if len(codes1) == 1 and len(codes2) == 1:
                            target_codes = [codes1[0], codes2[0]]
                            print(
                                f"🤖 NLP found exactly one '{class1}' ({codes1[0]}) and one '{class2}' ({codes2[0]}) for BBD.")
                            nlp_triggered_bbd = True
                        else:
                            reason = ""
                            if len(codes1) != 1: reason += f" found {len(codes1)} '{class1}' objects;"
                            if len(codes2) != 1: reason += f" found {len(codes2)} '{class2}' objects;"
                            print(
                                f" BBD NLP failed: Need exactly one of each object type ({reason} in room {tool_context_room_id}).")
                            return None
                    else:
                        print(
                            f" BBD NLP failed: Need exactly two object types mentioned, found {len(found_classes)}.")
                        return None
            else:  # Keyword found, but no specific codes and no context ID for NLP
                print(f" Tool '{tool_to_run}' triggered, but no specific codes and no room context for NLP.")
                return None  # Let LLM ask for codes/context

        # Override NLP if specific codes were given (except for BBD triggered by NLP)
        if tool_to_run in ['CLR', 'VOL', 'VIS'] and has_specific_codes:
            target_codes = valid_codes
            print(f"   Overriding NLP with {len(target_codes)} specific codes from query.")

        if not keyword_found: return None

        # --- Execute Tool ---
        combined_results = []
        if tool_to_run in ['CLR', 'VOL']:
            if not target_codes: print(f"⚠️ {tool_to_run} skipped: No target codes found."); return None
            for code in target_codes:
                print(f" Executing {tool_to_run} for code: {code}")
                desc = f"{tool_to_run} ({code}): Execution failed."
                if tool_to_run == 'CLR':
                    result = self.api_wrapper.analyze_dominant_color(code)
                    if result and hasattr(result, 'components'):
                        color_descs = [
                            f"[W: {comp.get('weight', 0):.2f}, RGB: {tuple(int(x) for x in comp.get('mean', [0, 0, 0])[:3])}]"
                            for comp in result.components]
                        desc = f"CLR ({code}): Found {result.M} colors: {', '.join(color_descs)}" if color_descs else f"CLR ({code}): Analysis ok, no colors."
                    else:
                        desc = f"CLR ({code}): Analysis failed."
                elif tool_to_run == 'VOL':
                    result = self.api_wrapper.calculate_volume(code)
                    if result:
                        desc = f"VOL ({code}): Volume={result.volume:.3f}m³, Status={'Closed' if result.closed else 'Unclosed'}"
                    else:
                        desc = f"VOL ({code}): Calculation failed."
                combined_results.append(desc)

        elif tool_to_run == 'VIS':
            if not tool_context_room_id: print("⚠️ VIS tool skipped: No room context ID available."); return None

            codes_to_pass = []
            if target_codes:
                codes_to_pass = target_codes  # Use specific or NLP-found codes
            else:  # Pass room code
                try:
                    room_summary = self.get_room_summary(tool_context_room_id)
                    floor_num = room_summary.get('room', {}).get('floor_number')
                    room_num_str = room_summary.get('room', {}).get('room_number')
                    if floor_num is not None and room_num_str is not None:
                        room_num_int = int(room_num_str)
                        room_code = f"{floor_num}-{room_num_int}"
                        codes_to_pass = [room_code]
                    else:
                        print(
                            f" VIS tool skipped: Could not get floor/room num for ID {tool_context_room_id}."); return None
                except Exception as e:
                    print(f" VIS error getting room code: {e}"); return None

            if not codes_to_pass: print(f" VIS skipped: No codes for room {tool_context_room_id}."); return None

            print(f" Executing VIS for room {tool_context_room_id} with codes: {codes_to_pass}")
            result = self.api_wrapper.visualize_point_cloud(codes_to_pass)
            if result:
                desc = f"VIS (Room {tool_context_room_id}, Codes: {codes_to_pass}): Status: {result.status}."
                if result.viewer_url: desc += f" URL: [VIEWER_URL]{result.viewer_url}[/VIEWER_URL]"
                if result.error: desc += f" Error: {result.error}"
            else:
                desc = f"VIS (Room {tool_context_room_id}, Codes: {codes_to_pass}): Visualization failed."
            combined_results.append(desc)

        elif tool_to_run == 'BBD':
            if len(target_codes) != 2:
                if not nlp_triggered_bbd: print(f"⚠️ BBD skipped: Requires 2 codes, found {len(target_codes)}.")
                return None
            obj1, obj2 = target_codes[0], target_codes[1]
            if obj1 == obj2: return None
            print(f" Executing BBD for {obj1} and {obj2}")
            result = self.api_wrapper.calculate_bbox_distance(obj1, obj2)
            if result and hasattr(result, 'vector_1_to_2') and isinstance(result.vector_1_to_2, dict):
                v = result.vector_1_to_2;
                vec = f"({v.get('x', 0):.3f}, {v.get('y', 0):.3f}, {v.get('z', 0):.3f})"
                desc = f"BBD ({obj1} <-> {obj2}): Distance={result.distance:.3f}m, Vector={vec}"
            else:
                desc = f"BBD ({obj1} <-> {obj2}): Calculation failed."
            combined_results.append(desc)

        # --- Format Result ---
        if combined_results:
            source = "explicit codes" if has_specific_codes else "NLP lookup"
            result_prefix = f"EXTERNAL API RESULT ({'MULTI-' if len(combined_results) > 1 else ''}TOOL {tool_to_run} via {source}):"
            return f"{result_prefix}\n" + "\n".join(combined_results)
        else:
            print(f"Tool '{tool_to_run}' identified but could not execute.")
            return None

            # --- LLM-based Scope Classifier ---

    def _determine_query_scope(self, user_query: str) -> str:
        """ Uses preliminary LLM call for scope: 'SINGLE_ROOM' or 'MULTI_ROOM'. """
        if not self.client: return "SINGLE_ROOM"  # Default on error

        current_room_name = "None"
        if self.current_room_id:
            try:
                room_info = self.rooms_df[self.rooms_df['room_id'] == self.current_room_id]
                if not room_info.empty: current_room_name = room_info.iloc[0].get('room_name', 'None')
            except Exception as e:
                print(f" Error getting current room name: {e}")

        # REFINED Prompt v4
        system_prompt = f"""
        You are an AI query classifier determining data scope for an architectural assistant.
        Current Context: Room '{current_room_name}' (ID: {self.current_room_id})

        Analyze the user's query and choose ONLY ONE scope:
        - 'SINGLE_ROOM': Query focuses on the current context OR *explicitly names one specific room* (e.g., "what's in this room?", "tell me about the kitchen", "volume of 0-2-4 in room 2").
        - 'MULTI_ROOM': Query requires comparing rooms, searching all rooms, or aggregation 
          (e.g., "which room has most chairs?", "list all bedrooms", "total area of hallways").

        IMPORTANT: Naming a specific room like 'kitchen' usually implies SINGLE_ROOM unless comparison words like 'compare', 'which', 'most', 'total' are also present.

        Respond ONLY with JSON: {{"scope": "...", "reasoning": "..."}}
        Example 1: query="what's in the kitchen?" -> {{"scope": "SINGLE_ROOM", "reasoning": "Focuses on one named room."}}
        Example 2: query="compare kitchen and living room" -> {{"scope": "MULTI_ROOM", "reasoning": "Explicit comparison requested."}}
        """
        messages = [{"role": "system", "content": system_prompt}, {"role": "user", "content": user_query}]

        try:
            print(f" Classifying query scope for: \"{user_query}\"")
            response = self.client.chat.completions.create(model="gpt-4o-mini", messages=messages, temperature=0.0,
                                                           max_tokens=150)
            response_content = response.choices[0].message.content
            if response_content:
                try:
                    json_match = re.search(r'\{.*\}', response_content, re.DOTALL)
                    if json_match:
                        json_str = json_match.group(0);
                        json_response = json.loads(json_str)
                        scope = json_response.get("scope")
                        reasoning = json_response.get("reasoning", "N/A.")
                        if scope in ["SINGLE_ROOM", "MULTI_ROOM"]:
                            print(f" Scope classified as: {scope} (Reason: {reasoning})")
                            return scope
                    print(f" Scope classifier bad JSON structure: {response_content}")
                except json.JSONDecodeError:
                    print(f" Scope classifier JSON parse error: {response_content}")
            print(" Scope classifier invalid/missing response. Defaulting to SINGLE_ROOM.")
            return "SINGLE_ROOM"
        except Exception as e:
            print(f" Error :  Scope classifier LLM call failed: {e}. Defaulting to SINGLE_ROOM.")
            return "SINGLE_ROOM"

    # --- Main Query Method ---
    def query(self, user_query: str, is_interactive: bool = False) -> Dict[str, Any]:
        """ Processes query, determines scope, executes tools, calls LLM, returns result. """
        print(f"\nProcessing Query: \"{user_query}\"")

        # --- 1. Determine Scope & Context ---
        scope_type = self._determine_query_scope(user_query)
        target_room_ids: Optional[List[int]] = None  # For main LLM prompt data
        tool_context_room_id: Optional[int] = None  # For tool NLP logic
        scope = "unknown"

        explicitly_referenced_room_id: Optional[int] = None
        room_ref = self._parse_room_reference(user_query)
        if room_ref:
            room_num, floor_num = room_ref
            detected_room_id = self._find_room_by_reference(room_num, floor_num)
            if detected_room_id:
                explicitly_referenced_room_id = detected_room_id
            else:
                print(f"⚠ Room reference '{room_num}/{floor_num}' not found.")
                # Return early with error message for non-existent rooms
                available_rooms = self._get_available_rooms_list()
                return {
                    "error": f"Room {room_num} on floor {floor_num} doesn't exist in the database.",
                    "available_rooms": available_rooms,
                    "query": user_query,
                    "response": f"That room doesn't exist in the database. Available rooms are: {', '.join(available_rooms)}"
                }

        if scope_type == "SINGLE_ROOM":
            if explicitly_referenced_room_id is not None:
                # Check if this room type has duplicates
                has_duplicates = False
                all_matching_room_ids = [explicitly_referenced_room_id]
                
                if not self.rooms_df.empty and 'room_type' in self.rooms_df.columns:
                    room_row = self.rooms_df[self.rooms_df['room_id'] == explicitly_referenced_room_id]
                    if not room_row.empty:
                        room_type = room_row.iloc[0].get('room_type', 'unknown')
                        if room_type != 'unknown':
                            same_type_rooms = self.rooms_df[self.rooms_df['room_type'] == room_type]
                            if len(same_type_rooms) > 1:
                                has_duplicates = True
                                all_matching_room_ids = same_type_rooms['room_id'].tolist()
                
                # If there are duplicates, change scope to multi-room to show all
                if has_duplicates:
                    target_room_ids = all_matching_room_ids
                    tool_context_room_id = explicitly_referenced_room_id  # Keep first as tool context
                    scope = f"multi_room_type_match_{len(all_matching_room_ids)}"
                    print(f" Scope: Multiple Rooms of Same Type (showing all {len(all_matching_room_ids)} matching rooms)")
                    print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
                else:
                    target_room_ids = [explicitly_referenced_room_id]
                    tool_context_room_id = explicitly_referenced_room_id
                    self.current_room_id = explicitly_referenced_room_id  # Update main context if specific room is focus
                    scope = f"room_{explicitly_referenced_room_id}"
                    print(f" Scope: Specific Room (ID: {explicitly_referenced_room_id})")
            elif self.current_room_id is not None:
                target_room_ids = [self.current_room_id]
                tool_context_room_id = self.current_room_id
                scope = f"room_{self.current_room_id}"
                print(f" Scope: Current Room Context (ID: {self.current_room_id})")
            else:
                scope = "ambiguous_context";
                target_room_ids = [];
                tool_context_room_id = None
                print(" Scope: Ambiguous (No room specified, no current context)")
        elif scope_type == "MULTI_ROOM":
            # Check if explicitly_referenced_room_id has duplicates even in MULTI_ROOM scope
            if explicitly_referenced_room_id is not None and not self.rooms_df.empty and 'room_type' in self.rooms_df.columns:
                room_row = self.rooms_df[self.rooms_df['room_id'] == explicitly_referenced_room_id]
                if not room_row.empty:
                    room_type = room_row.iloc[0].get('room_type', 'unknown')
                    if room_type != 'unknown':
                        same_type_rooms = self.rooms_df[self.rooms_df['room_type'] == room_type]
                        if len(same_type_rooms) > 1:
                            # Override to show only matching room type
                            target_room_ids = same_type_rooms['room_id'].tolist()
                            tool_context_room_id = explicitly_referenced_room_id
                            scope = f"multi_room_type_match_{len(target_room_ids)}"
                            print(f" Scope: Multiple Rooms of Same Type (showing all {len(target_room_ids)} matching rooms)")
                            print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
                        else:
                            # No duplicates, proceed with normal multi-room
                            room_count = len(self.rooms_df) if not self.rooms_df.empty else 0
                            scope = f"multi_room_all_{room_count}"
                            target_room_ids = None
                            tool_context_room_id = explicitly_referenced_room_id if explicitly_referenced_room_id is not None else self.current_room_id
                            print(f" Scope: Multi-Room (LLM Decision) - Providing all {room_count} room summaries.")
                            if tool_context_room_id:
                                print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
                    else:
                        # Unknown room type, use normal multi-room
                        room_count = len(self.rooms_df) if not self.rooms_df.empty else 0
                        scope = f"multi_room_all_{room_count}"
                        target_room_ids = None
                        tool_context_room_id = explicitly_referenced_room_id if explicitly_referenced_room_id is not None else self.current_room_id
                        print(f" Scope: Multi-Room (LLM Decision) - Providing all {room_count} room summaries.")
                        if tool_context_room_id:
                            print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
                else:
                    # Room not found, use normal multi-room
                    room_count = len(self.rooms_df) if not self.rooms_df.empty else 0
                    scope = f"multi_room_all_{room_count}"
                    target_room_ids = None
                    tool_context_room_id = explicitly_referenced_room_id if explicitly_referenced_room_id is not None else self.current_room_id
                    print(f" Scope: Multi-Room (LLM Decision) - Providing all {room_count} room summaries.")
                    if tool_context_room_id:
                        print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
            else:
                # No explicitly referenced room, use normal multi-room
                room_count = len(self.rooms_df) if not self.rooms_df.empty else 0
                scope = f"multi_room_all_{room_count}"
                target_room_ids = None  # Fetch ALL rooms for main prompt
                # Tool context uses specific ref if available, else current context
                tool_context_room_id = explicitly_referenced_room_id if explicitly_referenced_room_id is not None else self.current_room_id
                print(f" Scope: Multi-Room (LLM Decision) - Providing all {room_count} room summaries.")
                if tool_context_room_id:
                    print(f"   (Tool NLP Context will use Room ID: {tool_context_room_id})")
                else:
                    print(f"   (No specific room context for tool NLP)")

        # --- Query Execution ---
        try:
            # --- 2. Get Data ---
            room_data: Union[Dict, List[Dict], None] = None
            if scope.startswith("room_") and target_room_ids:
                room_data = self.get_room_summary(target_room_ids[0])
                if not room_data: return {"error": f"Data for Room ID {target_room_ids[0]} not found.",
                                          "query": user_query}
            elif scope.startswith("multi_room_type_match_"):
                # Get data for all rooms of the same type
                room_data = []
                for room_id in target_room_ids:
                    room_summary = self.get_room_summary(room_id)
                    if room_summary:
                        room_data.append(room_summary)
            elif scope.startswith("multi_room_"):
                room_data = self._get_all_rooms_data()
            elif scope == "ambiguous_context":
                room_data = []

            # --- 3. Execute Tools ---
            tool_result_text = self._parse_and_execute_tool(user_query, tool_context_room_id)
            tool_used = None
            if tool_result_text:
                tool_match = re.search(r'TOOL (\w+)', tool_result_text)
                if tool_match: tool_used = tool_match.group(1)

            # --- 3.5. Auto-trigger Visualization for Room Display Queries ---
            # Check if query is asking to show/visualize/display a room
            vis_keywords = r'\b(show|display|visualize|view|see|open|render)\b'
            room_keywords = r'\b(room|space|area|kitchen|bedroom|bathroom|hallway|living room|dining room)\b'
            is_visualization_request = (re.search(vis_keywords, user_query.lower()) and 
                                       re.search(room_keywords, user_query.lower()))
            
            # Auto-trigger VIS if it's a visualization request and we have a specific room context
            if is_visualization_request and not tool_used and tool_context_room_id:
                print(f" Auto-triggering visualization for room ID: {tool_context_room_id}")
                try:
                    room_summary = self.get_room_summary(tool_context_room_id)
                    floor_num = room_summary.get('room', {}).get('floor_number')
                    room_num_str = room_summary.get('room', {}).get('room_number')
                    if floor_num is not None and room_num_str is not None:
                        room_num_int = int(room_num_str)
                        room_code = f"{floor_num}-{room_num_int}"
                        
                        print(f" Auto-executing VIS for room {room_code}")
                        result = self.api_wrapper.visualize_point_cloud([room_code])
                        if result:
                            desc = f"VIS (Auto-triggered for Room {tool_context_room_id}, Code: {room_code}): Status: {result.status}."
                            if result.viewer_url: 
                                desc += f" URL: [VIEWER_URL]{result.viewer_url}[/VIEWER_URL]"
                            if result.error: 
                                desc += f" Error: {result.error}"
                            tool_result_text = f"EXTERNAL API RESULT (TOOL VIS via auto-trigger):\n{desc}"
                            tool_used = "VIS"
                            print("Success : Auto-visualization triggered successfully.")
                        else:
                            print(" Auto-visualization failed.")
                except Exception as e:
                    print(f" Auto-visualization error: {e}")

                # --- 4. Prepare LLM Messages ---
            prompt_data = room_data if isinstance(room_data, (dict, list)) else []
            system_prompt = self._create_system_prompt(prompt_data)
            messages = [{"role": "system", "content": system_prompt}]

            # --- 5. Handle Visuals & Tool Results ---
            images_used = False;
            room_images = []
            is_visual_query = self._needs_visual_analysis(user_query)
            # Allow images for single-room queries OR multi-room queries that have a specific room context
            can_use_visuals = self.use_images and (
                (scope.startswith("room_") and target_room_ids) or 
                (scope.startswith("multi_room") and tool_context_room_id is not None)
            )

            current_user_content_str = user_query
            if tool_result_text:
                current_user_content_str = f"{tool_result_text}\n\nUser Query: {user_query}"
                print("🛠️ Tool result added to prompt.")
                # Also add images for visual queries even when tools succeeded
                if is_visual_query and can_use_visuals:
                    print(" Visual query detected, adding images alongside tool results...")
                    # Use appropriate room ID based on scope
                    img_room_id = target_room_ids[0] if target_room_ids else tool_context_room_id
                    if img_room_id:
                        room_images = self._get_room_images(img_room_id)
                        if room_images:
                            images_used = True
                            print(f"   Success :  Added {len(room_images)} images.")
                        else:
                            print("    No images found.")
            elif is_visual_query and can_use_visuals:
                print("Visual query, adding images...")
                # Use appropriate room ID based on scope
                img_room_id = target_room_ids[0] if target_room_ids else tool_context_room_id
                if img_room_id:
                    room_images = self._get_room_images(img_room_id)
                    if room_images:
                        images_used = True
                        print(f"   Success :  Added {len(room_images)} images.")
                    else:
                        print("   Error ;  No images found.")

            # --- 6. Finalize Message List ---
            if images_used and room_images:
                user_content_final = [{"type": "text", "text": current_user_content_str}]
                if len(messages) == 1:
                    messages.append({"role": "user", "content": user_content_final})
                else:
                    messages[-1]["content"] = user_content_final
                messages = self._add_images_to_messages(messages, room_images)
            else:
                user_content_final = current_user_content_str
                messages.append({"role": "user", "content": user_content_final})

            # --- 7. API Call (Main Answer) ---
            if not self.client: return {"error": "OpenAI client not initialized.", "query": user_query}
            print(f" Sending request to LLM (Scope: {scope}, Images: {images_used})...")
            output_max_tokens = 1500 if images_used else 2500
            llm_response_content = "Error: LLM call not executed."
            try:
                response = self.client.chat.completions.create(model="gpt-4o-mini", messages=messages, temperature=0.1,
                                                               max_tokens=output_max_tokens)
                if response and response.choices:
                    llm_response_content = response.choices[0].message.content
                else:
                    llm_response_content = "Error: LLM response empty/invalid."
            except Exception as api_error:
                print(f" OpenAI API call failed: {api_error}");
                llm_response_content = f"Error: OpenAI API call failed. ({api_error})"

            # --- 7.5. Execute Tool Commands from LLM Response ---
            if llm_response_content and "TOOL:" in llm_response_content:
                print(" Detecting tool commands in LLM response...")
                # Find all TOOL: commands in the response
                tool_pattern = r'TOOL:\s*(CLR|VOL|BBD)\s+([\d\-\s]+)'
                tool_commands = re.findall(tool_pattern, llm_response_content, re.IGNORECASE)
                
                if tool_commands:
                    tool_results_section = "\n\n═══ TOOL EXECUTION RESULTS ═══\n"
                    for tool_name, codes_str in tool_commands:
                        tool_name = tool_name.upper()
                        codes = codes_str.strip().split()
                        
                        print(f"   Executing {tool_name} for codes: {codes}")
                        
                        try:
                            if tool_name == "CLR":
                                for code in codes:
                                    result = self.api_wrapper.analyze_dominant_color(code)
                                    if result and hasattr(result, 'components') and result.components:
                                        # Extract color information
                                        color_info = []
                                        for comp in result.components:
                                            # Handle both dict and object structures
                                            if isinstance(comp, dict):
                                                mean = comp.get('mean', [0, 0, 0])
                                                weight = comp.get('weight', 0)
                                            else:
                                                # If it's an object, try to access attributes
                                                mean = getattr(comp, 'mean', [0, 0, 0])
                                                weight = getattr(comp, 'weight', 0)
                                            
                                            if mean and len(mean) >= 3:
                                                rgb = tuple(int(x) for x in mean[:3])
                                                color_name = self._rgb_to_color_name(rgb)
                                                color_info.append(f"{color_name} RGB{rgb} ({weight*100:.1f}% coverage)")
                                        
                                        if color_info:
                                            tool_results_section += f"\n CLR {code}: {', '.join(color_info)}"
                                        else:
                                            tool_results_section += f"\n CLR {code}: Color analysis completed (M={result.M}, components={len(result.components)})"
                                    elif result:
                                        tool_results_section += f"\n CLR {code}: Color analysis completed (M={getattr(result, 'M', '?')}), but no RGB details"
                                    else:
                                        tool_results_section += f"\nError : CLR {code}: Analysis failed"
                            
                            elif tool_name == "VOL":
                                for code in codes:
                                    result = self.api_wrapper.calculate_volume(code)
                                    if result and hasattr(result, 'volume'):
                                        status = "closed" if getattr(result, 'closed', False) else "unclosed"
                                        tool_results_section += f"\n VOL {code}: Volume = {result.volume:.3f} m³ ({status})"
                                    else:
                                        tool_results_section += f"\nError : VOL {code}: Calculation failed"
                            
                            elif tool_name == "BBD" and len(codes) >= 2:
                                result = self.api_wrapper.calculate_bbox_distance(codes[0], codes[1])
                                if result and hasattr(result, 'distance'):
                                    tool_results_section += f"\n BBD {codes[0]} ↔ {codes[1]}: Distance = {result.distance:.2f} meters"
                                else:
                                    tool_results_section += f"\nError : BBD {codes[0]} ↔ {codes[1]}: Calculation failed"
                        
                        except Exception as e:
                            tool_results_section += f"\nError : {tool_name} {' '.join(codes)}: Error - {str(e)}"
                            print(f" Error executing {tool_name}: {e}")
                    
                    tool_results_section += "\n═══════════════════════════════\n"
                    # Append tool results to the LLM response
                    llm_response_content += tool_results_section
                    print(f" Executed {len(tool_commands)} tool command(s) from LLM response")

            # --- 8. Post-Response Visualization Suggestion (Conditional) ---
            # MODIFIED: Always run if interactive and VIS wasn't the main tool
            if is_interactive and tool_used != "VIS" and tool_context_room_id is not None:
                print(
                    f" Interactive mode: Attempting proactive visualization for context room {tool_context_room_id}.")
                codes_for_proactive_vis = []
                # Try to get codes from LLM response first
                if llm_response_content:
                    codes_in_response = list(set(re.findall(r'(\d+-\d+-\d+)', llm_response_content)))
                    if codes_in_response:
                        codes_for_proactive_vis = codes_in_response
                        print(f"   (Found {len(codes_for_proactive_vis)} codes in response)")

                # If no codes in response, skip room-level visualization
                # (rooms without shell files can't be visualized)
                if not codes_for_proactive_vis:
                    print(f"   (No object codes in response, skipping room-level visualization)")

                # Attempt the visualization call
                if codes_for_proactive_vis:
                    try:
                        vis_result = self.api_wrapper.visualize_point_cloud(codes_for_proactive_vis)
                        if vis_result and vis_result.viewer_url:
                            llm_response_content += f"\n\n[VIEWER_URL]{vis_result.viewer_url}[/VIEWER_URL]"
                            print(f"   Success :  Added proactive VIS link.")
                        else:
                            print(f" Proactive VIS call failed or returned no URL.")
                    except Exception as e:
                        print(f" Failed proactive VIS call: {e}")
                else:
                    print(f"   (Skipping proactive VIS: No codes to visualize)")

            # --- 9. Build Result ---
            result = {"query": user_query, "response": llm_response_content, "scope": scope, "used_images": images_used,
                      "images_count": len(room_images), "tool_used": tool_used}
            if scope.startswith("room_") and isinstance(room_data, dict):
                room_info = room_data.get('room', {})
                room_id_to_add = target_room_ids[0] if target_room_ids else None
                result.update({"room": room_info.get('room_name'), "floor": room_info.get('floor_name'),
                               "room_id": room_id_to_add})

            return result

        except Exception as e:
            print(f" Unhandled exception during query: '{user_query}'")
            traceback.print_exc()
            return {"error": f"Unexpected error: {str(e)}", "query": user_query}

    # =====================================================================
    # FUTURE FEATURE: Interactive 3D Viewer Selection
    # =====================================================================
    # This method is planned for future implementation where users can
    # interactively select objects in the 3D viewer and the agent will
    # =====================================================================
    def _wait_for_user_selection(self, session_id: str, timeout: int = 60) -> Optional[List[Dict[str, Any]]]:
        """
        Wait for user to complete selection in viewer.
        
        Poll /tmp/viewer_selection_{session_id}.json file until:
        - User completes selection (file appears)
        - Timeout
        
        Args:
            session_id: Unique identifier for the selection session
            timeout: Maximum wait time (seconds)
        
        Returns:
            List of user-selected objects, or None (timeout/error)
        """
        import time
        from pathlib import Path
        
        selection_file = Path(f"/tmp/viewer_selection_{session_id}.json")
        start_time = time.time()
        
        print(f"\nWaiting for user to select objects in viewer...")
        print(f"   Session ID: {session_id}")
        print(f"   Timeout setting: {timeout} seconds")
        print(f"   Monitoring file: {selection_file}")
        print(f"   (User can continue after selecting in viewer and clicking 'Confirm')\n")
        
        elapsed = 0
        while elapsed < timeout:
            if selection_file.exists():
                try:
                    # Wait a short time to ensure file write is complete
                    time.sleep(0.2)
                    
                    with open(selection_file, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    # Validate data structure
                    if not isinstance(data, list):
                        print(f"Selection data format error: expected list, got {type(data)}")
                        selection_file.unlink()
                        return None
                    
                    # Clean up temporary file
                    try:
                        selection_file.unlink()
                        print(f"Cleaned up temporary file: {selection_file}")
                    except Exception as e:
                        print(f"Failed to clean up temporary file: {e}")
                    
                    print(f"Received user selection: {len(data)} objects")
                    for i, item in enumerate(data, 1):
                        print(f"   {i}. {item.get('displayName', 'unknown')} ({item.get('itemCode', 'unknown')})")
                    print()
                    
                    return data
                    
                except json.JSONDecodeError as e:
                    print(f"Failed to parse selection file: {e}")
                    try:
                        selection_file.unlink()
                    except:
                        pass
                    return None
                except Exception as e:
                    print(f"Error reading selection file: {e}")
                    traceback.print_exc()
                    return None
            
            # Display progress every second
            time.sleep(1)
            elapsed = int(time.time() - start_time)
            if elapsed % 10 == 0:  # prompt every 10 seconds
                print(f"   ... still waiting for user selection ({elapsed}/{timeout} seconds)...")
        
        print(f"\nWait timeout ({timeout} seconds)")
        print(f"   Hint: User may not have completed selection in viewer")
        return None
    
    def close(self):
        """Closes the database connection."""
        if self.conn:
            try:
                self.conn.close(); self.conn = None; print("Successfully closed database connection.")
            except sqlite3.Error as e:
                print(f" Error closing database connection: {e}")


# --- Demo Function ---
def demo_agent():
    """Runs predefined queries (proactive links disabled)."""
    print(" STARTING SPATIAL AI AGENT DEMO MODE");
    print("   (Proactive VIS links disabled in demo mode)");
    print("═" * 60)
    agent = None
    try:
        if not os.path.exists("spatial_rooms.db"): print(" DB not found."); return
        agent = FinalSpatialAIAgent(use_images=True)
        if agent.rooms_df.empty: print(" No rooms found in DB.")
        if agent.current_room_id is not None and not agent.rooms_df.empty:
            row = agent.rooms_df[agent.rooms_df['room_id'] == agent.current_room_id]
            if not row.empty:
                info = row.iloc[0]; print(
                    f"\nInitial Context: {info.get('room_name', '?')} (ID: {agent.current_room_id})")
            else:
                print("\n Initial room ID details not found.")
        else:
            print("\n No initial room context.")
        print("—" * 60)

        test_queries = [
            # === BASIC SPATIAL QUERIES ===
            "What is the dominant color of the couch 0-3-0 in room 3 floor 0?",  # CLR tool
            "What is the distance between chair 0-2-4 and shell 0-2-3 in room 2 floor 0?",  # BBD tool
            "Show me the couch 0-3-0 in room 3 floor 0",  # VIS tool
            "Visualize the chairs in room 2 floor 0",  # VIS with NLP
            "Display the current room",  # VIS current context
            
            # === NLP QUERIES ===
            "How many chairs are in the kitchen?",  # Room type matching
            "List the tables in the bedroom.",  # Room type + object class
            "which room has the highest width of the chair",  # Multi-room comparison
            "What is the distance between the chair and the door in the kitchen?",  # BBD NLP
            
            # === ADDITIONAL SPATIAL QUERIES ===
            "What objects are in room 001 on floor 0?",  # List all objects
            "Show me all the doors in the hallway",  # VIS with object class
            "What is the area of the kitchen?",  # Room metadata
            "What materials and finishes can you see in the kitchen panoramas?",  # Should use images
            
            # === COST ESTIMATION QUERIES ===
            "What would it cost to paint the walls of the kitchen?",  # Cost estimation with area calculation
            "How much to do flooring for the entire house?",  # Cost estimation for all rooms
            
            # === OUT OF SCOPE QUERIES (Should be rejected) ===
            "What is the recipe for tiramisu?",  # Cooking - OUT OF SCOPE
            "What's the weather today?",  # General knowledge - OUT OF SCOPE
            "Tell me a joke",  # Entertainment - OUT OF SCOPE
            
            # === NON-EXISTENT DATA QUERIES (Should say doesn't exist) ===
            "Show me room 999 on floor 5",  # Non-existent room
            "What color is object 9-9-9?",  # Non-existent object
            "How many windows are in the bathroom?",  # Non-existent room type
        ]
        for i, query in enumerate(test_queries, 1):
            print(f"\n{'=' * 80}\nQuery {i}: {query}\n{'—' * 80}")
            if agent is None or agent.conn is None: break
            result = agent.query(query, is_interactive=False)  # Pass False
            if "error" in result:
                print(f"Error : AGENT ERROR: {result['error']}")
            else:
                print(f" SCOPE: {result.get('scope', '?')}")
                if result.get('room_id'): print(f" CONTEXT: {result.get('room', '?')} (ID: {result['room_id']})")
                print(f" IMAGES USED: {result.get('used_images', False)}")
                response_text = result.get('response', 'No response.')
                urls_found = re.findall(r'\[VIEWER_URL\](.*?)\[/VIEWER_URL\]', response_text)
                if urls_found:
                    response_text = re.sub(r'\[VIEWER_URL\](.*?)\[/VIEWER_URL\]', '', response_text).strip()
                    print(f"\n RESPONSE:\n{textwrap.fill(response_text, width=80)}")
                    for url in urls_found: print(f"\n✨ Requested Visualization Link: {url}")  # Only show requested
                    print()
                else:
                    print(f"\n RESPONSE:\n{textwrap.fill(response_text, width=80)}\n")
    except ImportError as e:
        print(f"\n Error :  Import error: {e}"); traceback.print_exc()
    except Exception as e:
        print(f"\n Error :  Demo error: {e}"); traceback.print_exc()
    finally:
        if agent: agent.close()


# --- Interactive Session Logic ---
def interactive_session():
    """Runs interactive session (proactive links enabled)."""
    print(" STARTING INTERACTIVE SPATIAL AI AGENT SESSION");
    print("   Type 'quit' or 'exit' to end.");
    print("═" * 60)
    agent = None
    try:
        if not os.path.exists("spatial_rooms.db"): print("Error : DB not found."); return
        agent = FinalSpatialAIAgent(use_images=True)
        if agent.current_room_id is not None and not agent.rooms_df.empty:
            row = agent.rooms_df[agent.rooms_df['room_id'] == agent.current_room_id]
            if not row.empty:
                info = row.iloc[0]; print(
                    f"\nInitial Context: {info.get('room_name', '?')} (ID: {agent.current_room_id})")
            else:
                print("\n Initial room ID details not found.")
        else:
            print("\n No initial room context.")
        print("—" * 60)

        while True:
            try:
                user_input = input("Ask the agent (or type 'quit'): ")
                query = user_input.strip()
                if query.lower() in ["quit", "exit"]: break
                if not query: continue
                if agent.conn is None: print("Error : DB connection lost."); break

                result = agent.query(query, is_interactive=True)  # Pass True

                print("\n" + "—" * 80)
                if "error" in result:
                    print(f"Error :  AGENT ERROR: {result['error']}")
                else:
                    print(f" SCOPE: {result.get('scope', '?')}")
                    if result.get('room_id'): print(f" CONTEXT: {result.get('room', '?')} (ID: {result['room_id']})")
                    print(f" IMAGES USED: {result.get('used_images', False)}")
                    response_text = result.get('response', 'No response.')
                    urls_found = re.findall(r'\[VIEWER_URL\](.*?)\[/VIEWER_URL\]', response_text)
                    if urls_found:
                        response_text = re.sub(r'\[VIEWER_URL\](.*?)\[/VIEWER_URL\]', '', response_text).strip()
                        print(f"\n RESPONSE:\n{textwrap.fill(response_text, width=80)}")
                        for j, url in enumerate(urls_found):
                            is_proactive = (j == len(urls_found) - 1) and (result.get('tool_used') != "VIS")
                            link_type = "Suggested" if is_proactive else "Requested"
                            print(f"\n {link_type} Visualization Link: {url}")
                        print()
                    else:
                        print(f"\n RESPONSE:\n{textwrap.fill(response_text, width=80)}")
                print("—" * 80 + "\n")
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as loop_error:
                print(f"\nError :  Query error: {loop_error}"); traceback.print_exc()
    except ImportError as e:
        print(f"\nError :  Import error: {e}"); traceback.print_exc()
    except Exception as e:
        print(f"\nError :  Session setup error: {e}"); traceback.print_exc()
    finally:
        if agent: agent.close()
        print(" Session ended.")


# --- Main Execution Block ---
if __name__ == "__main__":
    if not os.path.exists("spatial_rooms.db"):
        print("---");
        print("Error :  DB 'spatial_rooms.db' not found.");
        print("   Run 'room_database.py' first.");
        print("---")
    else:
        while True:
            mode = input("Choose mode: 'demo' or 'interactive'? ").strip().lower()
            if mode == 'demo':
                demo_agent(); break
            elif mode == 'interactive':
                interactive_session(); break
            else:
                print("Invalid input.")