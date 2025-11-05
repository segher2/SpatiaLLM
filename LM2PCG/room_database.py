import sqlite3
import pandas as pd
import os
import glob
from typing import Dict, List, Tuple


class SpatialDatabaseCorrect:
    """
    DATABASE:
    - Stores structured room/floor data, geometric objects (from object CSV),
      planes/walls data (from planes_data.csv), and image paths.
    """

    def __init__(self, db_path: str = "spatial_rooms.db"):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        self._create_tables()  # This now runs the DROP commands internally

    def _create_tables(self):
        """
        Create database tables.
        CRITICAL FIX: Explicitly dropping tables to ensure the new 'object_code'
        column is created before the index is created.
        """
        print(" Ensuring clean schema by dropping old tables...")
        # Drop tables in dependency order
        self.cursor.execute("DROP TABLE IF EXISTS planes")
        self.cursor.execute("DROP TABLE IF EXISTS images")
        self.cursor.execute("DROP TABLE IF EXISTS objects")
        self.cursor.execute("DROP TABLE IF EXISTS rooms")
        self.cursor.execute("DROP TABLE IF EXISTS floors")
        self.conn.commit()
        print("✓ Old tables dropped.")

        # Table 1: Floors
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS floors (
                floor_id INTEGER PRIMARY KEY AUTOINCREMENT,
                floor_name TEXT NOT NULL,
                floor_number INTEGER,
                total_rooms INTEGER DEFAULT 0,
                total_area REAL DEFAULT 0
            )
        """)

        # Table 2: Rooms
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS rooms (
                room_id INTEGER PRIMARY KEY AUTOINCREMENT,
                floor_id INTEGER NOT NULL,
                room_name TEXT NOT NULL,
                room_number TEXT,
                room_type TEXT,
                total_area REAL,
                total_volume REAL,
                length REAL,
                width REAL,
                height REAL,
                geometric_csv_file TEXT,
                scan_date TEXT,
                notes TEXT,
                FOREIGN KEY (floor_id) REFERENCES floors(floor_id)
            )
        """)

        # Table 3: Objects
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS objects (
                object_id INTEGER PRIMARY KEY AUTOINCREMENT,
                room_id INTEGER NOT NULL,
                csv_id INTEGER,
                obj_name TEXT NOT NULL,
                class TEXT NOT NULL,
                object_code TEXT UNIQUE, -- NEW: The unique identifier for external API
                min_x REAL, min_y REAL, min_z REAL,
                max_x REAL, max_y REAL, max_z REAL,
                center_x REAL, center_y REAL, center_z REAL,
                length REAL, width REAL, height REAL,
                area REAL,
                volume REAL,
                FOREIGN KEY (room_id) REFERENCES rooms(room_id)
            )
        """)

        # Table 4: Images (panoramas for each room)
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS images (
                image_id INTEGER PRIMARY KEY AUTOINCREMENT,
                room_id INTEGER NOT NULL,
                image_path TEXT NOT NULL,
                image_name TEXT,
                image_type TEXT DEFAULT 'panorama',
                FOREIGN KEY (room_id) REFERENCES rooms(room_id)
            )
        """)

        # Table 5: Planes
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS planes (
                plane_id INTEGER PRIMARY KEY AUTOINCREMENT,
                room_id INTEGER NOT NULL,
                plane_name TEXT,     -- NEW: Stores CSV's 'group_name'
                plane_class TEXT,    -- NEW: Stores CSV's 'class' (wall, ceiling, floor)
                normal_x REAL, normal_y REAL, normal_z REAL,
                centroid_x REAL, centroid_y REAL, centroid_z REAL,
                area REAL,
                FOREIGN KEY (room_id) REFERENCES rooms(room_id)
            )
        """)

        # Indexes for performance
        # CRITICAL FIX: The table must exist with the columns before indexing.
        self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_rooms_floor ON rooms(floor_id)")
        self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_objects_room ON objects(room_id)")
        self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_objects_class ON objects(class)")
        self.cursor.execute("CREATE INDEX IF NOT EXISTS idx_images_room ON images(room_id)")
        self.cursor.execute("CREATE UNIQUE INDEX IF NOT EXISTS idx_objects_code ON objects(object_code)")  # NEW INDEX

        self.conn.commit()
        print("✓ Database tables created for new workflow")

    def scan_from_manifest(self, data_path: str = "Data") -> Dict:
        """
        NEW: Scan data directory based on rooms_manifest.csv files.
        This method reads all rooms_manifest.csv files under floor_* directories
        and uses them as the source of truth for room information.
        """
        print(f"🔍 Scanning data directory based on rooms_manifest.csv: {data_path}")

        structure = {}
        if not os.path.exists(data_path):
            print(f"❌ Data directory not found: {data_path}")
            return structure

        # Find all floor directories
        floor_pattern = os.path.join(data_path, "floor_*")
        floor_dirs = glob.glob(floor_pattern)

        for floor_dir in sorted(floor_dirs):
            floor_name = os.path.basename(floor_dir)
            floor_number = int(floor_name.split('_')[1]) if '_' in floor_name else 0

            # Look for rooms_manifest.csv in this floor directory
            manifest_path = os.path.join(floor_dir, "rooms_manifest.csv")
            
            if not os.path.exists(manifest_path):
                print(f"⚠️  No rooms_manifest.csv found in {floor_name}, skipping...")
                continue

            print(f"  📄 Reading manifest: {manifest_path}")
            
            structure[floor_name] = {'path': floor_dir, 'floor_number': floor_number, 'rooms': {}}

            try:
                # Read the manifest CSV
                manifest_df = pd.read_csv(manifest_path)
                manifest_df.columns = manifest_df.columns.str.strip()
                
                print(f"     Found {len(manifest_df)} rooms in manifest")

                # Process each room from the manifest
                for _, row in manifest_df.iterrows():
                    # Extract room_id from manifest (this is the room number)
                    room_id_from_manifest = int(row['room_id']) if 'room_id' in row else None
                    
                    if room_id_from_manifest is None:
                        print(f"     ⚠️  Skipping row without room_id")
                        continue
                    
                    # Generate room_name and room_number from manifest room_id
                    room_number = str(room_id_from_manifest).zfill(3)  # e.g., "007"
                    room_name = f"room_{room_number}"  # e.g., "room_007"
                    
                    # Construct expected room directory path
                    room_dir = os.path.join(floor_dir, room_name)
                    
                    # Check if room directory exists (optional - we can work without it)
                    room_exists = os.path.exists(room_dir)
                    
                    if not room_exists:
                        print(f"     ℹ️  Room directory not found: {room_name} (will create entry anyway)")
                    
                    # Look for object CSV
                    obj_csv_path = None
                    if room_exists:
                        obj_csv_pattern = os.path.join(room_dir, f"{room_name}.csv")
                        obj_csv_files = glob.glob(obj_csv_pattern)
                        obj_csv_path = obj_csv_files[0] if obj_csv_files else None
                    
                    # Look for planes CSV
                    plane_csv_path = None
                    if room_exists:
                        plane_csv_pattern = os.path.join(room_dir, "planes_data.csv")
                        plane_csv_files = glob.glob(plane_csv_pattern)
                        plane_csv_path = plane_csv_files[0] if plane_csv_files else None
                    
                    # Look for images
                    image_files = []
                    if room_exists:
                        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG']
                        image_files = [
                            f for ext in image_extensions 
                            for f in glob.glob(os.path.join(room_dir, ext))
                        ]
                    
                    # Extract additional info from manifest
                    room_type = row.get('room_type', 'unknown')
                    ply_path = row.get('ply_path', '')
                    
                    # Store room information
                    structure[floor_name]['rooms'][room_name] = {
                        'path': room_dir,
                        'room_number': room_number,
                        'room_id_from_manifest': room_id_from_manifest,
                        'room_type': room_type,
                        'ply_path': ply_path,
                        'obj_csv_path': obj_csv_path,
                        'plane_csv_path': plane_csv_path,
                        'images': sorted(image_files),
                        'has_obj_csv': bool(obj_csv_path),
                        'has_plane_csv': bool(plane_csv_path),
                        'image_count': len(image_files),
                        'room_exists': room_exists
                    }
                    
                    print(f"     ✓ {room_name} (room_id={room_id_from_manifest}, type={room_type}, images={len(image_files)})")

            except Exception as e:
                print(f"     ❌ Error reading manifest for {floor_name}: {e}")
                continue

        return structure
    
    def scan_data_directory(self, data_path: str = "Data") -> Dict:
        """
        DEPRECATED: Old method that scans directories directly.
        Use scan_from_manifest() instead for manifest-based scanning.
        """
        print(f"⚠️  WARNING: Using deprecated directory scanning method.")
        print(f"   Consider using scan_from_manifest() for manifest-based scanning.")
        print(f"🔍 Scanning data directory: {data_path}")

        structure = {}
        if not os.path.exists(data_path):
            print(f"❌ Data directory not found: {data_path}")
            return structure

        # Find all floor directories
        # Assumes floor folders are directly under the data_path (e.g., output2/Full House/floor_0)
        floor_pattern = os.path.join(data_path, "floor_*")
        floor_dirs = glob.glob(floor_pattern)

        for floor_dir in sorted(floor_dirs):
            floor_name = os.path.basename(floor_dir)
            floor_number = int(floor_name.split('_')[1]) if '_' in floor_name else 0

            structure[floor_name] = {'path': floor_dir, 'floor_number': floor_number, 'rooms': {}}

            room_pattern = os.path.join(floor_dir, "room_*")
            room_dirs = glob.glob(room_pattern)

            for room_dir in sorted(room_dirs):
                room_name = os.path.basename(room_dir)
                room_number = room_name.split('_')[1] if '_' in room_name else "001"

                # 1. Object CSV (e.g., room_001.csv)
                obj_csv_pattern = os.path.join(room_dir, f"{room_name}.csv")
                obj_csv_files = glob.glob(obj_csv_pattern)
                obj_csv_path = obj_csv_files[0] if obj_csv_files else None

                # 2. Planes CSV (planes_data.csv) - NEW
                plane_csv_pattern = os.path.join(room_dir, "planes_data.csv")
                plane_csv_files = glob.glob(plane_csv_pattern)
                plane_csv_path = plane_csv_files[0] if plane_csv_files else None

                # 3. Images
                image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG']
                image_files = [
                    f for ext in image_extensions for f in glob.glob(os.path.join(room_dir, ext))
                ]

                structure[floor_name]['rooms'][room_name] = {
                    'path': room_dir,
                    'room_number': room_number,
                    'obj_csv_path': obj_csv_path,
                    'plane_csv_path': plane_csv_path,
                    'images': sorted(image_files),
                    'has_obj_csv': bool(obj_csv_path),
                    'has_plane_csv': bool(plane_csv_path),
                    'image_count': len(image_files)
                }
        return structure

    def add_floor(self, floor_name: str, floor_number: int = None) -> int:
        """Add a new floor (Unchanged)"""
        if floor_number is None:
            try:
                floor_number = int(floor_name.split('_')[1])
            except:
                floor_number = 0
        self.cursor.execute("""
            INSERT INTO floors (floor_name, floor_number)
            VALUES (?, ?)
        """, (floor_name, floor_number))
        self.conn.commit()
        floor_id = self.cursor.lastrowid
        print(f"✓ Floor '{floor_name}' created (ID: {floor_id})")
        return floor_id

    def add_room(self, floor_id: int, room_name: str, room_number: str = None,
                 room_type: str = None, geometric_csv: str = None,
                 scan_date: str = None, notes: str = None) -> int:
        """Add a new room to a floor (Simplified for less verbose parameter list)"""
        self.cursor.execute("""
            INSERT INTO rooms (floor_id, room_name, room_number, room_type, 
                             geometric_csv_file, scan_date, notes)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (floor_id, room_name, room_number, room_type, geometric_csv, scan_date, notes))
        self.conn.commit()
        room_id = self.cursor.lastrowid
        print(f"✓ Room '{room_name}' created (ID: {room_id})")
        return room_id

    def import_geometric_csv(self, room_id: int, csv_path: str) -> int:
        """
        Import geometric CSV data, including the new object_code, and explicitly maps
        all necessary fields from the CSV row.
        """
        if not os.path.exists(csv_path):
            print(f"✗ File not found: {csv_path}")
            return 0

        print(f"Importing geometric data from {csv_path}...")

        try:
            df = pd.read_csv(csv_path)
            df.columns = df.columns.str.strip()
            print(f"CSV columns found: {list(df.columns)}")

            count = 0

            # --- CRITICAL CHECK ---
            required_cols = ['object_code', 'cluster_id', 'file', 'center_x', 'size_x', 'size_y', 'size_z']
            for col in required_cols:
                if col not in df.columns:
                    print(f"❌ ERROR: Missing CRITICAL column '{col}' in Object CSV. Cannot import objects.")
                    return 0
            # ----------------------

            for _, row in df.iterrows():
                filename = str(row['file'])
                obj_class = filename.split('_')[0].lower() if '_' in filename else filename.split('.')[0].lower()
                obj_name = filename

                # Insert object with object_code, using room_id and floor_id from the Python context
                self.cursor.execute("""
                    INSERT INTO objects (
                        room_id, csv_id, obj_name, class, object_code,
                        center_x, center_y, center_z,
                        length, width, height
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    room_id,  # Use room_id from the Python loop (not CSV's room_id)
                    int(row['cluster_id']),
                    obj_name,
                    obj_class,
                    str(row['object_code']),  # Mapped directly from CSV
                    float(row['center_x']),
                    float(row['center_y']),
                    float(row['center_z']),
                    float(row['size_x']),
                    float(row['size_y']),
                    float(row['size_z'])
                ))
                count += 1

            self.conn.commit()
            print(f"✓ Imported {count} objects from {os.path.basename(csv_path)}")
            return count

        except Exception as e:
            print(f"❌ Error importing OBJECT CSV: {e}")
            return 0

    def import_planes_csv(self, room_id: int, csv_path: str) -> int:
        """
        NEW: Import geometric plane data from planes_data.csv.
        FIX: Updated column mapping to use 'group_name' and 'class' from CSV.
        """
        if not os.path.exists(csv_path):
            print(f"✗ Plane File not found: {csv_path}")
            return 0

        print(f"Importing plane data from {csv_path}...")

        try:
            df = pd.read_csv(csv_path)
            df.columns = df.columns.str.strip()

            # The geometric and semantic columns now present in your CSV
            required_cols = ['group_name', 'class', 'normal_x', 'normal_y', 'normal_z',
                             'centroid_x', 'centroid_y', 'centroid_z', 'area']

            for col in required_cols:
                if col not in df.columns:
                    print(f"❌ ERROR: Missing CRITICAL column '{col}' in planes_data.csv. Skipping import.")
                    return 0

            count = 0
            for _, row in df.iterrows():
                self.cursor.execute("""
                    INSERT INTO planes (
                        room_id, plane_name, plane_class, normal_x, normal_y, normal_z,
                        centroid_x, centroid_y, centroid_z, area
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    room_id,
                    str(row['group_name']),  # Mapped from CSV
                    str(row['class']),  # Mapped from CSV
                    float(row['normal_x']), float(row['normal_y']), float(row['normal_z']),
                    float(row['centroid_x']), float(row['centroid_y']), float(row['centroid_z']),
                    float(row['area']),
                ))
                count += 1

            self.conn.commit()
            print(f"✓ Imported {count} planes from {os.path.basename(csv_path)}")
            return count

        except Exception as e:
            print(f"❌ Error importing PLANES CSV: {e}")
            return 0

    def add_room_images(self, room_id: int, image_paths: List[str]) -> int:
        """Add panorama images for a room (Unchanged)"""
        count = 0
        for image_path in image_paths:
            if os.path.exists(image_path):
                image_name = os.path.basename(image_path)
                self.cursor.execute("""
                    INSERT INTO images (room_id, image_path, image_name)
                    VALUES (?, ?, ?)
                """, (room_id, image_path, image_name))
                count += 1
            else:
                print(f"Warning: Image not found: {image_path}")

        self.conn.commit()
        if count > 0:
            print(f"✓ Added {count} panorama images")
        return count

    def populate_from_structure(self, structure: Dict, require_images: bool = False):
        """
        Populate database from discovered folder structure (Updated for manifest-based scanning).
        
        Args:
            structure: Dictionary containing floor and room information
            require_images: If True, skip rooms without images (default: False for manifest mode)
        """
        total_rooms = 0
        total_objects = 0
        total_images = 0
        total_planes = 0
        skipped_rooms = 0

        for floor_name, floor_data in structure.items():
            print(f"\n Processing {floor_name}...")

            floor_id = self.add_floor(floor_name, floor_data['floor_number'])

            for room_name, room_data in floor_data['rooms'].items():
                print(f"  🏠 Processing {room_name}...")

                # Optional: Skip rooms without images (configurable)
                if require_images and not room_data['images']:
                    print(f"    ⚠ SKIPPING {room_name}: No panorama images found.")
                    skipped_rooms += 1
                    continue

                # Use room_type from manifest if available, otherwise default to 'unknown'
                room_type = room_data.get('room_type', 'unknown')
                
                room_id = self.add_room(
                    floor_id=floor_id,
                    room_name=room_name,
                    room_number=room_data['room_number'],
                    room_type=room_type,  # Use room_type from manifest
                    geometric_csv=room_data['obj_csv_path'],
                    scan_date="2025-01-01",
                    notes=f"Auto-imported from {room_data['path']}"
                )

                # 1. Import geometric objects
                if room_data['has_obj_csv']:
                    obj_count = self.import_geometric_csv(room_id, room_data['obj_csv_path'])
                    total_objects += obj_count
                else:
                    print(f"    ℹ️  No Object CSV file found for {room_name}")

                # 2. Import planes data
                if room_data['has_plane_csv']:
                    plane_count = self.import_planes_csv(room_id, room_data['plane_csv_path'])
                    total_planes += plane_count
                else:
                    print(f"    ℹ️  No Planes CSV file found for {room_name}")

                # 3. Add images
                if room_data['images']:
                    img_count = self.add_room_images(room_id, room_data['images'])
                    total_images += img_count
                else:
                    print(f"    ℹ️  No images found for {room_name}")

                total_rooms += 1

        self._update_floor_statistics()

        print(f"\n✅ Database populated successfully!")
        print(f"   Floors: {len(structure)}")
        print(f"   Rooms: {total_rooms}")
        if skipped_rooms > 0:
            print(f"   Skipped: {skipped_rooms} rooms (no images)")
        print(f"   Objects: {total_objects}")
        print(f"   Planes: {total_planes}")
        print(f"   Images: {total_images}")

    # --- REMOVED _infer_room_type function ---

    def _update_floor_statistics(self):
        """Update floor statistics after population (Unchanged)"""
        self.cursor.execute("""
            UPDATE floors 
            SET total_rooms = (
                SELECT COUNT(*) FROM rooms WHERE rooms.floor_id = floors.floor_id
            ),
            total_area = (
                SELECT COALESCE(SUM(total_area), 0) FROM rooms 
                WHERE rooms.floor_id = floors.floor_id
            )
        """)
        self.conn.commit()

    def get_database_overview(self) -> str:
        """Get formatted overview of entire database (Updated for Planes)"""
        floors = pd.read_sql_query(
            "SELECT floor_name, floor_number, total_rooms, total_area FROM floors ORDER BY floor_number", self.conn)
        objects_total = pd.read_sql_query("SELECT COUNT(*) as total_objects FROM objects", self.conn)
        planes_total = pd.read_sql_query("SELECT COUNT(*) as total_planes FROM planes", self.conn)

        # New: Get plane classes (wall, ceiling, etc.)
        plane_classes = pd.read_sql_query(
            """SELECT plane_class, COUNT(*) as count FROM planes GROUP BY plane_class ORDER BY count DESC""", self.conn)
        objects_by_class = pd.read_sql_query(
            """SELECT class, COUNT(*) as count FROM objects GROUP BY class ORDER BY count DESC""", self.conn)

        overview = "SPATIAL DATABASE OVERVIEW (Updated Schema)\n"
        overview += "=" * 70 + "\n\n"

        overview += f"Total Floors: {len(floors)}\n"
        overview += f"Total Rooms: {floors['total_rooms'].sum()}\n"
        overview += f"Total Objects: {objects_total.iloc[0]['total_objects']}\n"
        overview += f"Total Planes/Walls: {planes_total.iloc[0]['total_planes']}\n"
        overview += f"Total Area: {floors['total_area'].sum():.2f}m²\n\n"

        overview += "PLANES BY CLASS:\n"
        for _, plane in plane_classes.iterrows():
            overview += f"  {plane['plane_class']}: {plane['count']} planes\n"

        overview += "\nOBJECTS BY CLASS:\n"
        for _, obj in objects_by_class.iterrows():
            overview += f"  {obj['class']}: {obj['count']} objects\n"

        overview += "\nFLOORS:\n"
        for _, floor in floors.iterrows():
            overview += f"  {floor['floor_name']} (Floor {floor['floor_number']})\n"
            overview += f"      Rooms: {floor['total_rooms']}, Area: {floor['total_area']:.2f}m²\n"

        return overview

    def get_room_summary(self, room_id: int) -> Dict:
        """Get complete room summary (Updated for Planes and Object Code)"""
        room = pd.read_sql_query("""
            SELECT r.*, f.floor_name 
            FROM rooms r
            JOIN floors f ON r.floor_id = f.floor_id
            WHERE r.room_id = ?
        """, self.conn, params=[room_id])

        if room.empty:
            return {}

        # Objects with Object Code
        objects = pd.read_sql_query("""
            SELECT 
                class, object_code, obj_name, center_x, center_y, center_z, length, width, height 
            FROM objects
            WHERE room_id = ?
            ORDER BY class, object_code
        """, self.conn, params=[room_id])

        # Planes
        planes = pd.read_sql_query("""
            SELECT 
                plane_class, area, normal_x, normal_y, normal_z 
            FROM planes
            WHERE room_id = ?
            ORDER BY plane_class, area DESC
        """, self.conn, params=[room_id])

        # Images
        images = pd.read_sql_query("""
            SELECT image_id, image_name, image_path 
            FROM images 
            WHERE room_id = ?
            ORDER BY image_id
        """, self.conn, params=[room_id])

        return {
            "room": room.iloc[0].to_dict(),
            "objects": objects.to_dict('records'),
            "planes": planes.to_dict('records'),
            "images": images.to_dict('records')
        }

    def close(self):
        """Close connection"""
        self.conn.close()
        print("✓ Database closed")

    def reset_database(self):
        """Delete all existing tables and recreate them"""
        print("🧹 Cleaning existing database...")
        self.cursor.execute("DROP TABLE IF EXISTS planes")
        self.cursor.execute("DROP TABLE IF EXISTS images")
        self.cursor.execute("DROP TABLE IF EXISTS objects")
        self.cursor.execute("DROP TABLE IF EXISTS rooms")
        self.cursor.execute("DROP TABLE IF EXISTS floors")
        self.conn.commit()
        print("✓ All existing tables dropped")

        self._create_tables()


def populate_database_fixed():
    """
    DEPRECATED: Old method using directory scanning.
    Use populate_database_from_manifest() instead.
    """
    print("⚠️  WARNING: This method is deprecated.")
    print("   Use populate_database_from_manifest() for manifest-based database generation.")
    print()

    # NOTE: The data path is explicitly set here to match your requested structure.
    DATA_ROOT = "output2"
    print(f" POPULATING DATABASE WITH UPDATED SCHEMA (Root: {DATA_ROOT})")
    print("=" * 70)

    # Initialize database
    db = SpatialDatabaseCorrect("spatial_rooms.db")

    # Scan data directory (old method)
    structure = db.scan_data_directory(DATA_ROOT)

    if not structure:
        print(f"❌ No floor structure found under {DATA_ROOT}. Please check folder structure.")
        db.close()
        return

    print(f"\n Discovered structure:")
    for floor_name, floor_data in structure.items():
        print(f"  {floor_name}: {len(floor_data['rooms'])} rooms")
        for room_name, room_data in floor_data['rooms'].items():
            status = "✓" if room_data['has_obj_csv'] else "⚠"
            plane_status = "P" if room_data['has_plane_csv'] else " "
            print(
                f"    {status}{plane_status} {room_name}: {room_data['image_count']} images, Object CSV: {room_data['has_obj_csv']}, Plane CSV: {room_data['has_plane_csv']}")

    # Populate database (with image requirement)
    db.populate_from_structure(structure, require_images=True)

    # Show final overview
    print("\n" + "=" * 70)
    print(" FINAL DATABASE OVERVIEW")
    print("=" * 70)
    print(db.get_database_overview())

    db.close()
    print(f"\n✅ Database created: spatial_rooms.db")


def populate_database_from_manifest():
    """
    NEW: Populate database based on rooms_manifest.csv files.
    This is the recommended method that uses manifest files as the source of truth.
    """
    DATA_ROOT = "output2"
    print(f"📋 POPULATING DATABASE FROM ROOMS_MANIFEST.CSV (Root: {DATA_ROOT})")
    print("=" * 70)

    # Initialize database
    db = SpatialDatabaseCorrect("spatial_rooms.db")

    # Scan data directory based on manifest files
    structure = db.scan_from_manifest(DATA_ROOT)

    if not structure:
        print(f"❌ No manifest files found under {DATA_ROOT}. Please check folder structure.")
        db.close()
        return

    print(f"\n📊 Discovered structure from manifests:")
    for floor_name, floor_data in structure.items():
        print(f"  {floor_name}: {len(floor_data['rooms'])} rooms")
        for room_name, room_data in floor_data['rooms'].items():
            status = "✓" if room_data['room_exists'] else "⚠"
            obj_status = "O" if room_data['has_obj_csv'] else " "
            plane_status = "P" if room_data['has_plane_csv'] else " "
            img_status = "I" if room_data['images'] else " "
            print(
                f"    {status}{obj_status}{plane_status}{img_status} {room_name} (ID:{room_data['room_id_from_manifest']}, type:{room_data['room_type']}, images:{room_data['image_count']})")

    # Populate database (no image requirement for manifest mode)
    db.populate_from_structure(structure, require_images=False)

    # Show final overview
    print("\n" + "=" * 70)
    print(" FINAL DATABASE OVERVIEW")
    print("=" * 70)
    print(db.get_database_overview())

    db.close()
    print(f"\n✅ Database created: spatial_rooms.db")
    print(f"   Source: rooms_manifest.csv files in floor_* directories")


if __name__ == "__main__":
    # Use the new manifest-based method by default
    populate_database_from_manifest()

