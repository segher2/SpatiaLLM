import numpy as np
import json
import subprocess
from pathlib import Path
from PIL import Image
import laspy
import plyfile
from plyfile import PlyData
from conversion_2D_3D import quat_to_matrix

def _refine_with_clustering(input_ply, pose_json, output_ply, params=None):
    """
    Refine point cloud using mask2cluster FEC-based selection.
    
    Args:
        input_ply: Path to initial filtered point cloud
        pose_json: Path to camera pose JSON
        output_ply: Path for refined output
        params: dict with clustering parameters (eps, n, m, etc.)
    
    Returns:
        str: Path to refined output, or None if failed
    """
    if params is None:
        params = {}
    
    # Default parameters
    eps = params.get('eps', 0.05)
    n = params.get('n', 0.3)
    m = params.get('m', 50)
    min_pts = params.get('min_pts_core', 8)
    min_total = params.get('min_pts_total', 100)
    max_diameter = params.get('max_diameter', 0.0)
    
    # Find mask2cluster_cli executable
    script_dir = Path(__file__).parent
    cli_path = script_dir / "mask2cluster" / "build" / "mask2cluster_cli"
    
    if not cli_path.exists():
        print(f" Warning: mask2cluster_cli not found at {cli_path}")
        print(f" Please build it first: cd {script_dir}/mask2cluster && mkdir build && cd build && cmake .. && make")
        return None
    
    try:
        cmd = [
            str(cli_path),
            str(input_ply),
            str(pose_json),
            str(output_ply),
            "--eps", str(eps),
            "--n", str(n),
            "--m", str(m),
            "--min-pts", str(min_pts),
            "--min-total", str(min_total),
        ]
        
        if max_diameter > 0:
            cmd.extend(["--max-diameter", str(max_diameter)])
        
        print(f" Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        print(result.stdout)
        if result.stderr:
            print(f" Stderr: {result.stderr}")
        
        return str(output_ply) if Path(output_ply).exists() else None
        
    except subprocess.CalledProcessError as e:
        print(f" Clustering refinement failed: {e}")
        if e.stdout:
            print(f" Stdout: {e.stdout}")
        if e.stderr:
            print(f" Stderr: {e.stderr}")
        return None
    except Exception as e:
        print(f" Clustering refinement error: {e}")
        return None

def filter_point_cloud_with_mask(mask_path, image_stem, base_dir, use_clustering=False, 
                                 cluster_params=None):
    try:
        BASE_DIR = Path(base_dir)
        # Use the project root data/output directory
        base_output = BASE_DIR.parent / "data" / "output"
        if not base_output.exists():
            # Fallback to LM2PCG data directory
            base_output = BASE_DIR / "data" / "lm2pcg_data"
            if not base_output.exists():
                base_output = BASE_DIR / "data" / "output"
        
        # Metadata is now in data/input/panoramas/metadata
        metadata_dir = BASE_DIR.parent / "data" / "input" / "panoramas" / "metadata"

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

        # Find the room-specific PLY file (combined_non_structural_*.ply or shell_*.ply)
        room_number = pano_room.name.split('_')[-1]  # Extract room number from room_XXX
        ply_candidates = [
            pano_room / f"combined_non_structural_{room_number}.ply",
            pano_room / f"shell_{room_number}.ply",
            pano_room / f"combined1_{room_number}.ply",
            pano_room / f"combined_{room_number}.ply",
        ]
        
        ply_path = None
        for candidate in ply_candidates:
            if candidate.exists():
                ply_path = candidate
                break
        
        if not ply_path:
            raise FileNotFoundError(f"No PLY file found in {pano_room}. Tried: {[c.name for c in ply_candidates]}")

        print(f" Using panorama: {pano_path}")
        print(f" Using PLY file: {ply_path}")

        # Locate pose JSON (in data/input/panoramas/metadata)
        pose_path = None
        expected_json = image_stem + ".json"

        # Look in data/input/panoramas/metadata first
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
        
        # Load color channels if available
        colors = None
        if 'red' in ply_data['vertex'].data.dtype.names:
            red = np.array(ply_data['vertex']['red'])
            green = np.array(ply_data['vertex']['green'])
            blue = np.array(ply_data['vertex']['blue'])
            colors = np.vstack((red, green, blue)).T
            print(f" Loaded {len(points):,} points with RGB colors from {ply_path.name}")
        else:
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
        selected_colors = colors[inside] if colors is not None else None
        print(f" {len(selected):,} points inside mask ({len(selected)/len(points)*100:.2f}%)")

        # Save filtered points as PLY - use SAM23D/outputs directory
        script_dir = Path(__file__).parent
        output_dir = script_dir / "outputs" / "filtered_point_clouds" / image_stem
        output_dir.mkdir(parents=True, exist_ok=True)
        initial_path = output_dir / f"{image_stem}_initial.ply"
        out_path = output_dir / f"{image_stem}_test.ply"

        if len(selected) > 0:
            # Create PLY file with xyz coordinates and colors if available
            if selected_colors is not None:
                vertex = np.array(
                    [(selected[i, 0], selected[i, 1], selected[i, 2], 
                      selected_colors[i, 0], selected_colors[i, 1], selected_colors[i, 2]) 
                     for i in range(len(selected))],
                    dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), 
                           ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
                )
            else:
                vertex = np.array(
                    [(selected[i, 0], selected[i, 1], selected[i, 2]) for i in range(len(selected))],
                    dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]
                )
            ply_el = plyfile.PlyElement.describe(vertex, 'vertex')
            plyfile.PlyData([ply_el], text=False).write(str(initial_path))
            print(f" Saved {len(selected):,} filtered points to {initial_path}")
            
            # Optional: refine with clustering
            if use_clustering:
                refined_path = _refine_with_clustering(
                    initial_path, pose_path, out_path, cluster_params
                )
                if refined_path:
                    print(f" Clustering refinement successful: {refined_path}")
                    return str(refined_path)
                else:
                    print(" Clustering refinement failed, returning initial result")
                    return str(initial_path)
            else:
                # No clustering, just rename initial to final
                import shutil
                shutil.copy(initial_path, out_path)
                return str(out_path)
        else:
            print(" No points selected. Check alignment or mask.")
            return None

    except Exception as e:
        print(f" Point cloud filtering failed: {e}")
        raise e