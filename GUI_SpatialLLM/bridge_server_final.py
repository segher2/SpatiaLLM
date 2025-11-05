from flask import Flask, request, jsonify
from flask_cors import CORS
from PIL import Image
from pathlib import Path
import numpy as np
import base64
import sys

app = Flask(__name__)
CORS(app)

# Configuration
BASE_DIR = Path(__file__).resolve().parent
MAX_CLICKS = 5

# Add SAM23D to Python path (for sam2_predictor and select_points_in_mask)
SAM23D_DIR = BASE_DIR.parent / "SAM23D"
if str(SAM23D_DIR) not in sys.path:
    sys.path.insert(0, str(SAM23D_DIR))
    print(f"Added SAM23D to Python path: {SAM23D_DIR}")

# Add SAM2 to Python path (SAM2 is inside SAM23D)
SAM2_DIR = SAM23D_DIR / "SAM2"
if str(SAM2_DIR) not in sys.path:
    sys.path.insert(0, str(SAM2_DIR))
    print(f"Added SAM2 to Python path: {SAM2_DIR}")

# Store image title and points: {"image_title": "filename.jpg", "points": [[x1, y1], [x2, y2], ...]}
click_data = {
    "image_title": None,
    "points": np.array([], dtype=np.int32).reshape(0, 2)
}

# Store latest overlay path
latest_overlay = {
    "path": None,
    "base64": None
}

print("Bridge server started - using NumPy arrays")
print(f"Base directory: {BASE_DIR}")

# Helpers
def pitch_yaw_to_pixel(pitch: float, yaw: float, width: int, height: int):
    #Convert pitch/yaw (degrees) to pixel coordinates (x, y) for equirectangular panorama
    x = (yaw + 180.0) / 360.0 * width
    y = (90.0 - pitch) / 180.0 * height
    xi = int(round(max(0, min(width - 1, x))))
    yi = int(round(max(0, min(height - 1, y))))
    return xi, yi


def get_image_path(filename: str):
    """Recursively search in data/lm2pcg_data/floor_*/room_* for a panorama image
    that matches the given filename. Returns (image_path, room_dir) or (None, None) if not found."""
    if not filename:
        return None, None

    stem = Path(filename).stem
    # Try LM2PCG data directory first
    base_dir = BASE_DIR / "data" / "lm2pcg_data"
    
    if not base_dir.exists():
        # Fallback to old path
        base_dir = BASE_DIR / "data" / "output"
    
    if not base_dir.exists():
        print(f"Base directory not found: {base_dir}")
        return None, None

    for floor_dir in base_dir.glob("floor_*"):
        if not floor_dir.is_dir():
            continue

        for room_dir in floor_dir.glob("room_*"):
            if not room_dir.is_dir():
                continue

            # Try exact match
            for ext in ['.jpg', '.jpeg', '.png', '.tif', '.tiff']:
                potential_path = room_dir / f"{stem}{ext}"
                if potential_path.exists():
                    print(f"Found panorama in: {room_dir}")
                    return potential_path, room_dir

            # Try case-insensitive match
            for file in room_dir.glob("*"):
                if file.stem.lower() == stem.lower() and file.suffix.lower() in ['.jpg', '.jpeg', '.png', '.tif',
                                                                                 '.tiff']:
                    print(f"Found panorama (case-insensitive) in: {room_dir}")
                    return file, room_dir

    print(f"Panorama not found for: {filename}")
    return None, None

# API Routes
@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy"})

@app.route('/click', methods=['POST'])
def handle_click():
    global click_data, latest_overlay
    data = request.get_json()
    image_filename = data.get('image_filename')
    print(f"Received click for image: {image_filename}")

    if len(click_data["points"]) >= MAX_CLICKS:
        return jsonify({"ok": False, "error": f"Maximum {MAX_CLICKS} clicks reached"}), 400

    pitch = float(data.get('pitch', 0))
    yaw = float(data.get('yaw', 0))

    image_path, room_dir = get_image_path(image_filename)
    if not image_path or not image_path.exists():
        return jsonify({"ok": False, "error": f"Image not found: {image_filename}"}), 400

    with Image.open(image_path) as img:
        w, h = img.size
    x, y = pitch_yaw_to_pixel(pitch, yaw, w, h)

    if click_data["image_title"] is None:
        click_data["image_title"] = image_filename

    new_point = np.array([[x, y]], dtype=np.int32)
    click_data["points"] = (
        np.vstack([click_data["points"], new_point])
        if click_data["points"].size
        else new_point
    )

    print(f"Stored point {len(click_data['points'])}/{MAX_CLICKS} - pixel=({x}, {y})")

    # Try to find the associated PLY file in the same room
    ply_path = None
    if room_dir and room_dir.exists():
        # Try shell_*.ply first (LM2PCG format), then combined_*.ply (old format)
        ply_files = list(room_dir.glob("shell_*.ply"))
        if not ply_files:
            ply_files = list(room_dir.glob("combined1_*.ply"))
        if not ply_files:
            ply_files = list(room_dir.glob("combined_*.ply"))
        
        if ply_files:
            ply_path = ply_files[0]
            print(f"Associated PLY file found: {ply_path}")
        else:
            print("No associated PLY file found in that room folder.")

    # Auto-run SAM2 when 5 clicks reached
    if len(click_data["points"]) == MAX_CLICKS:
        print("Five points reached - running SAM2 predictor...")
        try:
            from sam2_predictor import run_sam2_prediction
            # Pass the full image path from GUI's extracted_data
            full_image_path = BASE_DIR / "extracted_data" / "images" / click_data["image_title"]
            result = run_sam2_prediction(click_data["points"], str(full_image_path))
            print("SAM2 run complete.")

            # Encode overlay to base64
            if result.get("success") and result.get("overlay_path"):
                overlay_path = Path(result["overlay_path"])
                if overlay_path.exists():
                    with open(overlay_path, "rb") as f:
                        overlay_bytes = f.read()
                        overlay_base64 = base64.b64encode(overlay_bytes).decode("utf-8")
                        result["overlay_base64"] = overlay_base64

                        # Store for later retrieval
                        latest_overlay["path"] = str(overlay_path)
                        latest_overlay["base64"] = overlay_base64
                        print(f"Overlay encoded: {len(overlay_base64)} chars")

            print("Starting point cloud filtering...")
            from select_points_in_mask import filter_point_cloud_with_mask
            image_stem = Path(click_data["image_title"]).stem
            las_path = filter_point_cloud_with_mask(
                result["mask_path"],
                image_stem,
                BASE_DIR
            )

            result["las_path"] = las_path
            print(f"LAS file saved at: {las_path}")
            print("Point cloud filtering complete.")
            print("Processing finished successfully.\n")

            response_data = {
                "ok": True,
                "sam2_auto_run": True,
                "sam2_result": result,
                "ply_file": str(ply_path) if ply_path else None
            }
            print(f"🔵 Returning response with overlay_base64 length: {len(result.get('overlay_base64', ''))}")
            print(f"🔵 Response keys: {list(response_data.keys())}")
            print(f"🔵 sam2_result keys: {list(result.keys())}")
            
            return jsonify(response_data)

        except Exception as e:
            print(f"SAM2 failed: {e}")
            import traceback
            traceback.print_exc()
            return jsonify({"ok": False, "sam2_auto_run": False, "error": str(e)}), 500

    return jsonify({
        "ok": True,
        "x": x,
        "y": y,
        "count": len(click_data["points"]),
        "ply_file": str(ply_path) if ply_path else None
    })

@app.route('/click/reset', methods=['POST'])
def reset_clicks():
    global click_data, latest_overlay
    click_data = {
        "image_title": None,
        "points": np.array([], dtype=np.int32).reshape(0, 2)
    }
    latest_overlay = {
        "path": None,
        "base64": None
    }
    print("All clicks reset")
    return jsonify({"ok": True})

@app.route('/clicks', methods=['GET'])
def get_clicks():
    points_list = click_data["points"].tolist()
    print(f"Current clicks: {len(points_list)} points for {click_data['image_title']}")
    return jsonify({
        "image_title": click_data["image_title"],
        "points": points_list,
        "count": len(points_list)
    })

@app.route('/get_latest_overlay', methods=['GET'])
def get_latest_overlay():
    """Return the most recent SAM2 overlay as base64"""
    try:
        if latest_overlay["base64"]:
            print(f"Returning cached overlay: {latest_overlay['path']}")
            return jsonify({
                "overlay_base64": latest_overlay["base64"],
                "filename": Path(latest_overlay["path"]).name if latest_overlay["path"] else None
            })

        # Fallback: search for most recent overlay file
        output_dir = BASE_DIR / "output"
        if output_dir.exists():
            overlay_files = sorted(
                output_dir.glob("*_overlay.png"),
                key=lambda x: x.stat().st_mtime,
                reverse=True
            )
            if overlay_files:
                latest = overlay_files[0]
                with open(latest, "rb") as f:
                    overlay_base64 = base64.b64encode(f.read()).decode("utf-8")

                # Cache it
                latest_overlay["path"] = str(latest)
                latest_overlay["base64"] = overlay_base64

                print(f"Found and cached overlay: {latest.name}")
                return jsonify({
                    "overlay_base64": overlay_base64,
                    "filename": latest.name
                })

        return jsonify({"error": "No overlay found"}), 404

    except Exception as e:
        print(f"Error retrieving overlay: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

@app.route('/run_sam2', methods=['POST'])
def run_sam2():
    """Manual SAM2 trigger endpoint"""
    print("Manual SAM2 trigger received")
    try:
        from sam2_predictor import run_sam2_prediction
        result = run_sam2_prediction(click_data["points"], click_data["image_title"])

        # Encode overlay to base64
        if result.get("success") and result.get("overlay_path"):
            overlay_path = Path(result["overlay_path"])
            if overlay_path.exists():
                with open(overlay_path, "rb") as f:
                    overlay_bytes = f.read()
                    result["overlay_base64"] = base64.b64encode(overlay_bytes).decode("utf-8")

                    # Store for later retrieval
                    latest_overlay["path"] = str(overlay_path)
                    latest_overlay["base64"] = result["overlay_base64"]

        if result.get("success"):
            print("Starting point cloud filtering...")
            from select_points_in_mask import filter_point_cloud_with_mask
            image_stem = Path(click_data["image_title"]).stem
            las_path = filter_point_cloud_with_mask(
                result["mask_path"],
                image_stem,
                BASE_DIR
            )
            result["las_path"] = las_path
            print(f"LAS file saved at: {las_path}")
            print("Point cloud filtering complete.")

        print("Manual SAM2 prediction completed")
        return jsonify(result)
    except Exception as e:
        print(f"Manual SAM2 prediction failed: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"success": False, "error": str(e)}), 500


if __name__ == '__main__':
    print("Bridge server running on http://localhost:5056")
    print("Endpoints:")
    print("  POST /click              - Save click coordinates")
    print("  POST /click/reset        - Clear all clicks")
    print("  GET  /clicks             - List saved clicks")
    print("  GET  /get_latest_overlay - Get most recent SAM2 overlay")
    print("  POST /run_sam2           - Manually trigger SAM2")
    print("  GET  /health             - Health check")
    app.run(host='0.0.0.0', port=5056, debug=False)