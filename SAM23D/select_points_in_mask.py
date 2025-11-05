import numpy as np
import json
from pathlib import Path
from PIL import Image
import laspy
import plyfile
from plyfile import PlyData
from conversion_2D_3D import quat_to_matrix

def filter_point_cloud_with_mask(mask_path, image_stem, base_dir):
    try:
        BASE_DIR = Path(base_dir)
        # Try LM2PCG data directory first
        base_output = BASE_DIR / "data" / "lm2pcg_data"
        if not base_output.exists():
            # Fallback to old path
            base_output = BASE_DIR / "data" / "output"
        
        metadata_dir = BASE_DIR / "extracted_data" / "metadata"

        # Find panorama's room folder and matching .ply
        pano_room = None
        pano_path = None
        ply_path = None

        # Find panorama (and room) by exact stem match, case-insensitive over common image types
        for floor_dir in base_output.glob("floor_*"):
            if not floor_dir.is_dir():
                continue
            for room_dir in floor_dir.glob("room_*"):
                if not room_dir.is_dir():
                    continue
                for ext in (".jpg", ".jpeg", ".png", ".tif", ".tiff"):
                    pano_file = room_dir / f"{image_stem}{ext}"
                    if pano_file.exists():
                        pano_path, pano_room = pano_file, room_dir
                        break
                if pano_room:
                    break
            if pano_room:
                break

        if not pano_room:
            raise FileNotFoundError(f"No panorama found for '{image_stem}' under {base_output}")

        # Use fixed combined.ply file from lm2pcg_data root
        ply_path = base_output / "combined.ply"
        if not ply_path.exists():
            raise FileNotFoundError(f"Combined PLY file not found at {ply_path}")

        print(f" Using panorama: {pano_path}")
        print(f" Using PLY file: {ply_path}")

        # Locate pose JSON (in extracted_data/metadata)
        pose_path = None
        expected_json = image_stem + ".json"

        # Look in extracted_data/metadata first
        metadata_json = metadata_dir / expected_json
        if metadata_json.exists():
            pose_path = metadata_json
        else:
            for file in metadata_dir.rglob("*.json"):
                if image_stem.lower() in file.stem.lower():
                    pose_path = file
                    break

        # Fallback: try the room directory (old location)
        if not pose_path:
            room_json = pano_room / expected_json
            if room_json.exists():
                pose_path = room_json
            else:
                for file in pano_room.rglob("*.json"):
                    if image_stem.lower() in file.stem.lower():
                        pose_path = file
                        break

        if not pose_path:
            raise FileNotFoundError(
                f"No matching JSON found for {image_stem}. Expected something like {expected_json} in {metadata_dir} or {pano_room}"
            )

        print(f" Using pose file: {pose_path}")

        # Load camera pose (translation + rotation)
        with open(pose_path, "r") as f:
            pose_json = json.load(f)

        t_wc = np.array([
            pose_json["translation"]["x"],
            pose_json["translation"]["y"],
            pose_json["translation"]["z"]
        ])
        qx, qy, qz, qw = (
            pose_json["rotation"]["x"],
            pose_json["rotation"]["y"],
            pose_json["rotation"]["z"],
            pose_json["rotation"]["w"]
        )
        R_wc = quat_to_matrix(qx, qy, qz, qw)
        R_cw = R_wc.T

        print(f" Camera position: [{t_wc[0]:.2f}, {t_wc[1]:.2f}, {t_wc[2]:.2f}]")

        # Load mask
        mask_img = Image.open(mask_path)
        mask = np.array(mask_img).astype(float)
        if mask.ndim == 3:
            mask = mask[..., 0]
        mask = mask / mask.max() > 0.5
        H, W = mask.shape
        print(f" Mask dimensions: {W}x{H}  ({np.sum(mask):,} True pixels)")

        # Load points from PLY
        ply_data = PlyData.read(str(ply_path))
        x = np.array(ply_data['vertex']['x'])
        y = np.array(ply_data['vertex']['y'])
        z = np.array(ply_data['vertex']['z'])
        points = np.vstack((x, y, z)).T
        print(f" Loaded {len(points):,} points from {ply_path.name}")

        # Transform to camera frame
        points_cam = (R_cw @ (points - t_wc).T).T
        X, Y, Z = points_cam[:, 0], points_cam[:, 1], points_cam[:, 2]
        r = np.linalg.norm(points_cam, axis=1)

        # Project into equirectangular panorama
        lon = -np.arctan2(Y, X)
        lat = np.arcsin(np.clip(Z / r, -1, 1))

        u = ((lon / (2 * np.pi)) + 0.5) * W
        v = ((0.5 - lat / np.pi)) * H
        u = np.clip(u.astype(int), 0, W - 1)
        v = np.clip(v.astype(int), 0, H - 1)

        print(f" Projected {len(points)} points into {W}x{H} panorama space")

        # Filter points inside the binary mask
        inside = mask[v, u]
        selected = points[inside]
        print(f" {len(selected):,} points inside mask ({len(selected)/len(points)*100:.2f}%)")

        # Save filtered points as PLY
        output_dir = pano_room / "filtered_outputs"
        output_dir.mkdir(exist_ok=True)
        out_path = output_dir / f"{image_stem}_test.ply"

        if len(selected) > 0:
            # Create PLY file with xyz coordinates
            vertex = np.array(
                [(selected[i, 0], selected[i, 1], selected[i, 2]) for i in range(len(selected))],
                dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]
            )
            ply_el = plyfile.PlyElement.describe(vertex, 'vertex')
            plyfile.PlyData([ply_el]).write(str(out_path))
            print(f" Saved {len(selected):,} filtered points to {out_path}")
            return str(out_path)
        else:
            print(" No points selected. Check alignment or mask.")
            return None

    except Exception as e:
        print(f" Point cloud filtering failed: {e}")
        raise e