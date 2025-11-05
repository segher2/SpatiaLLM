import sqlite3
import os
import base64
import io
import time
import re
from typing import List, Dict, Optional, Tuple
from openai import AzureOpenAI
from dotenv import load_dotenv
from PIL import Image
import traceback

# Load environment variables (ensure API_KEY and endpoint are set)
load_dotenv()

# --- Configuration ---
DATABASE_PATH = "spatial_rooms.db"
# Define the possible room types for classification
POSSIBLE_ROOM_TYPES = [
    'living_room', 'kitchen', 'bedroom', 'bathroom', 'office',
    'hallway', 'dining_room', 'closet', 'balcony', 'garage', 'laundry_room', 'unknown'
]
# LLM Model to use for classification
LLM_MODEL = "gpt-4o-mini" # Or your preferred vision model
# Delay between API calls to avoid rate limits (adjust if needed)
API_DELAY_SECONDS = 1

# --- Helper Functions ---

def get_db_connection(db_path: str) -> Optional[Tuple[sqlite3.Connection, sqlite3.Cursor]]:
    """Establishes a connection to the SQLite database."""
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        print(f"✅ Connected to database: {db_path}")
        return conn, cursor
    except sqlite3.Error as e:
        print(f"❌ Database connection error: {e}")
        return None

def get_unknown_rooms(cursor: sqlite3.Cursor) -> List[Tuple[int, str]]:
    """Fetches room IDs and room codes for rooms with 'unknown' or generic types that need classification."""
    try:
        cursor.execute("""
            SELECT r.room_id, f.floor_number, r.room_number 
            FROM rooms r 
            JOIN floors f ON r.floor_id = f.floor_id 
            WHERE r.room_type = 'unknown' OR r.room_type IS NULL OR r.room_type = 'some room'
            ORDER BY f.floor_number, CAST(r.room_number AS INTEGER)
        """)
        rooms = cursor.fetchall()
        # Return list of (room_id, room_code) tuples
        return [(row[0], f"{row[1]}-{int(row[2])}") for row in rooms]
    except sqlite3.Error as e:
        print(f"❌ Error fetching unknown rooms: {e}")
        return []

def get_room_images_paths(cursor: sqlite3.Cursor, room_id: int) -> List[str]:
    """Fetches image paths for a given room ID."""
    try:
        cursor.execute("SELECT image_path FROM images WHERE room_id = ?", (room_id,))
        paths = cursor.fetchall()
        return [row[0] for row in paths]
    except sqlite3.Error as e:
        print(f"❌ Error fetching images for room {room_id}: {e}")
        return []

def process_image(image_path: str) -> Optional[str]:
    """Loads, resizes, and encodes an image to base64."""
    if not os.path.exists(image_path):
        print(f"  ❌ Image file NOT FOUND at path: {image_path}")
        return None
    try:
        with Image.open(image_path) as img:
            if img.mode != 'RGB':
                img = img.convert('RGB')
            max_size = 1024 # Resize for API efficiency
            if max(img.size) > max_size:
                img.thumbnail((max_size, max_size), Image.Resampling.LANCZOS)

            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=90)
            image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
            return image_base64
    except Exception as e:
        print(f"  ⚠ Error processing file {image_path}: {e}")
        return None

def classify_room_type(client: AzureOpenAI, image_base64_list: List[str]) -> str:
    """Uses OpenAI API to classify the room type based on images."""
    if not image_base64_list:
        return 'unknown' # Cannot classify without images

    prompt_text = (f"Analyze the following image(s) and classify the room type. "
                   f"Choose the best fit from this list: {', '.join(POSSIBLE_ROOM_TYPES)}. "
                   f"Respond ONLY with the chosen room type (e.g., 'living_room').")

    messages = [{"role": "system", "content": "You are an assistant that classifies room types from images."}]
    user_content = [{"type": "text", "text": prompt_text}]

    for img_b64 in image_base64_list:
        user_content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{img_b64}", "detail": "low"} # Use low detail for classification
        })

    messages.append({"role": "user", "content": user_content})

    try:
        response = client.chat.completions.create(
            model=LLM_MODEL,
            messages=messages,
            temperature=0.1,
            max_tokens=50 # Should be enough for just the room type name
        )
        llm_response = response.choices[0].message.content.strip().lower()

        # Clean the response - sometimes models add extra text
        # Look for the first matching room type in the response
        for room_type in POSSIBLE_ROOM_TYPES:
            if room_type in llm_response:
                return room_type # Return the first valid type found

        print(f"  ⚠️ LLM response ('{llm_response}') didn't match known types cleanly. Defaulting to unknown.")
        return 'unknown' # Default if no match or unexpected response

    except Exception as e:
        print(f"  ❌ OpenAI API call failed: {e}")
        traceback.print_exc()
        return 'unknown'

def update_room_type(conn: sqlite3.Connection, cursor: sqlite3.Cursor, room_id: int, new_type: str):
    """Updates the room_type in the database."""
    try:
        cursor.execute("UPDATE rooms SET room_type = ? WHERE room_id = ?", (new_type, room_id))
        conn.commit()
        print(f"  ✅ Updated room {room_id} type to '{new_type}'")
    except sqlite3.Error as e:
        print(f"  ❌ Error updating room {room_id}: {e}")

# --- Main Execution ---
def main():
    print("🚀 Starting Room Type Enrichment Script...")

    # 1. Initialize OpenAI Client
    try:
        client = AzureOpenAI(
            azure_endpoint="https://azure-openai-scanplan.openai.azure.com/",
            api_key=os.getenv("API_KEY"),
            api_version="2025-02-01-preview" # Or your API version
        )
        print("✅ OpenAI client initialized.")
    except Exception as e:
        print(f"❌ Failed to initialize OpenAI client: {e}. Ensure AZURE_OPENAI_ENDPOINT and API_KEY are set.")
        return # Cannot proceed without the client

    # 2. Connect to Database
    db_info = get_db_connection(DATABASE_PATH)
    if not db_info:
        return # Cannot proceed without DB
    conn, cursor = db_info

    # 3. Get Rooms to Process
    unknown_rooms = get_unknown_rooms(cursor)
    if not unknown_rooms:
        print("✅ No rooms needing classification found. Database is up-to-date.")
        conn.close()
        return

    print(f"🔍 Found {len(unknown_rooms)} rooms to classify (unknown/some room types).")

    # 4. Process Each Room
    processed_count = 0
    updated_count = 0
    for room_id, room_code in unknown_rooms:
        print(f"\nProcessing Room {room_code} (DB room_id: {room_id})...")
        processed_count += 1

        # Get image paths
        image_paths = get_room_images_paths(cursor, room_id)
        if not image_paths:
            print(f"  ℹ️ No images found for room {room_id}. Skipping classification.")
            continue # Skip if no images

        # Process images to base64 (take only the first one for simplicity, or modify to use more)
        image_base64_list = []
        if image_paths:
             img_b64 = process_image(image_paths[0]) # Using only the first image
             if img_b64:
                 image_base64_list.append(img_b64)

        if not image_base64_list:
            print(f"  ℹ️ Failed to process images for room {room_id}. Skipping classification.")
            continue

        # Classify using LLM
        print("  🧠 Classifying room type using vision model...")
        classified_type = classify_room_type(client, image_base64_list)

        # Update Database if classification is not 'unknown'
        if classified_type != 'unknown':
            update_room_type(conn, cursor, room_id, classified_type)
            updated_count += 1
        else:
            print(f"  ℹ️ Classification result was 'unknown' for room {room_id}. No update made.")

        # Optional delay
        if API_DELAY_SECONDS > 0:
            time.sleep(API_DELAY_SECONDS)

    # 5. Clean up
    conn.close()
    print("\n✅ Enrichment script finished.")
    print(f"   Processed {processed_count} rooms.")
    print(f"   Updated {updated_count} rooms with new types.")

if __name__ == "__main__":
    main()
