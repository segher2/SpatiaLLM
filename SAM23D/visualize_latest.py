#!/usr/bin/env python3
"""
Automated visualization script for SAM2 + mask2cluster results.

This script:
1. Cleans previous data and manifests
2. Finds the latest _test.ply file in SAM23D/outputs
3. Copies it to pointcloud-viewer/public/data
4. Creates a manifest JSON (Clusters Mode)
5. Starts the viewer server (if not running)
6. Prints the viewer URL

Usage:
    python visualize_latest.py
"""

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime


def find_latest_test_ply():
    """Find the most recently modified _test.ply file."""
    sam23d_dir = Path(__file__).parent
    outputs_dir = sam23d_dir / "outputs" / "filtered_point_clouds"
    
    if not outputs_dir.exists():
        print(f"❌ Error: Output directory not found: {outputs_dir}")
        return None
    
    # Find all _test.ply files
    test_files = list(outputs_dir.rglob("*_test.ply"))
    
    if not test_files:
        print(f"❌ Error: No _test.ply files found in {outputs_dir}")
        return None
    
    # Sort by modification time, newest first
    latest = max(test_files, key=lambda p: p.stat().st_mtime)
    
    return latest


def clean_outputs_directory():
    """Clean SAM23D/outputs directory."""
    sam23d_dir = Path(__file__).parent
    outputs_dir = sam23d_dir / "outputs"
    
    if outputs_dir.exists():
        # Remove entire outputs directory and recreate it
        shutil.rmtree(outputs_dir)
        outputs_dir.mkdir(parents=True, exist_ok=True)
        print("CLEANED: SAM23D/outputs")


def clean_viewer_data(viewer_base):
    """Clean previous data and manifests."""
    data_dir = viewer_base / "public" / "data"
    manifests_dir = viewer_base / "public" / "manifests"
    
    # Remove all files in data directory (keep directory structure)
    cleaned_count = 0
    if data_dir.exists():
        for item in data_dir.rglob("*"):
            if item.is_file():
                item.unlink()
                cleaned_count += 1
    
    # Remove all manifest files
    if manifests_dir.exists():
        for item in manifests_dir.glob("*.json"):
            item.unlink()
            cleaned_count += 1
    
    print(f"CLEANED_FILES: {cleaned_count}")


def get_point_count(ply_path):
    """Extract point count from PLY header."""
    try:
        with open(ply_path, 'rb') as f:
            # Read first 500 bytes to find header
            header = f.read(500).decode('latin-1')
            for line in header.split('\n'):
                if line.startswith('element vertex'):
                    return int(line.split()[2])
    except Exception as e:
        print(f"⚠️  Warning: Could not read point count: {e}")
    return None


def copy_to_viewer(ply_path, viewer_base):
    """Copy PLY file to viewer's public/data directory."""
    data_dir = viewer_base / "public" / "data" / "sam2_latest"
    data_dir.mkdir(parents=True, exist_ok=True)
    
    dest = data_dir / "cluster.ply"
    shutil.copy2(ply_path, dest)
    
    return dest


def create_manifest(ply_path, point_count, viewer_base):
    """Create manifest JSON in Clusters Mode format."""
    manifests_dir = viewer_base / "public" / "manifests"
    manifests_dir.mkdir(parents=True, exist_ok=True)
    
    # Extract image stem from path
    image_stem = ply_path.parent.name
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    
    manifest = {
        "version": 1,
        "title": f"SAM2 Cluster - {image_stem[:20]}...",
        "description": f"Single object cluster extracted from panorama {image_stem}",
        "defaults": {
            "pointSize": 4,
            "colorMode": "file"
        },
        "items": [
            {
                "id": "sam2_cluster",
                "name": f"Cluster ({point_count:,} points)" if point_count else "Cluster",
                "kind": "pointcloud",
                "role": "cluster",
                "source": {
                    "url": "/data/sam2_latest/cluster.ply"
                },
                "visible": True,
                "style": {
                    "pointSize": 4,
                    "colorMode": "file"
                },
                "meta": {
                    "sourceImage": image_stem,
                    "sourcePath": str(ply_path),
                    "generatedBy": "SAM2 + mask2cluster",
                    "timestamp": timestamp,
                    "pointCount": point_count
                }
            }
        ]
    }
    
    manifest_path = manifests_dir / "sam2_latest.json"
    with open(manifest_path, 'w', encoding='utf-8') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    
    return manifest_path


def kill_old_viewer_server(port=5174):
    """Kill any existing viewer server on the given port."""
    try:
        # Find PIDs using the port
        result = subprocess.run(
            ['lsof', '-ti', f':{port}'],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                try:
                    subprocess.run(['kill', '-9', pid], timeout=2)
                    print(f"KILLED_PID: {pid}")
                except Exception:
                    pass
            # Wait a moment for processes to die
            time.sleep(1)
    except Exception as e:
        print(f"ERROR_KILLING: {e}")



def start_viewer_server(viewer_base):
    """Kill old server and start a new viewer server."""
    # Try common Vite ports
    for port in [5174, 5173]:
        kill_old_viewer_server(port)
    
    # Start the server in the background
    # Capture output to a log file for debugging
    log_file = viewer_base / "server.log"
    with open(log_file, 'w') as log:
        process = subprocess.Popen(
            ["npm", "run", "dev"],
            cwd=viewer_base,
            stdin=subprocess.DEVNULL,  # Prevent stdin read errors
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True  # Detach from parent
        )
    
    print(f"SERVER_PID: {process.pid}")
    print(f"SERVER_LOG: {log_file}")
    
    # Wait for Vite to write its startup message
    print("⏳ Waiting for server to start...")
    time.sleep(3)
    
    # Parse the actual port from the log file
    actual_port = None
    if log_file.exists():
        with open(log_file) as f:
            content = f.read()
            # Look for "Local:   http://localhost:XXXX/"
            if 'Local:' in content and 'localhost:' in content:
                import re
                match = re.search(r'localhost:(\d+)', content)
                if match:
                    actual_port = int(match.group(1))
                    print(f"✅ Server started on port {actual_port}")
    
    if actual_port is None:
        print("⚠️  Could not detect port from server log, assuming 5173...")
        actual_port = 5173
    
    return True, actual_port


def main():
    # Locate viewer directory first
    sam23d_dir = Path(__file__).parent
    viewer_base = sam23d_dir.parent / "LM2PCG" / "web" / "pointcloud-viewer"
    
    if not viewer_base.exists():
        print(f"ERROR: Viewer directory not found: {viewer_base}", file=sys.stderr)
        sys.exit(1)
    
    print(f"VIEWER_DIR: {viewer_base}")
    
    clean_viewer_data(viewer_base)
    
    latest_ply = find_latest_test_ply()
    if not latest_ply:
        sys.exit(1)
    
    # Get file info
    file_size = latest_ply.stat().st_size / (1024 * 1024)  # MB
    mod_time = datetime.fromtimestamp(latest_ply.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")
    point_count = get_point_count(latest_ply)
    
    print(f"PLY_FILE: {latest_ply}")
    print(f"PLY_SIZE_MB: {file_size:.2f}")
    print(f"PLY_MODIFIED: {mod_time}")
    if point_count:
        print(f"POINT_COUNT: {point_count}")
    
    dest = copy_to_viewer(latest_ply, viewer_base)
    print(f"COPIED_TO: {dest}")
    
    manifest_path = create_manifest(latest_ply, point_count, viewer_base)
    print(f"MANIFEST: {manifest_path}")
    
    success, actual_port = start_viewer_server(viewer_base)
    
    url = f"http://localhost:{actual_port}/?manifest=sam2_latest.json"
    print(f"VIEWER_URL: {url}")
    print(url)
    
    if not success:
        print("\n⚠️  Server health check failed. Try accessing the URL manually.")
        sys.exit(1)


if __name__ == "__main__":
    main()
