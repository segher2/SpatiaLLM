from pathlib import Path
from typing import AnyStr
import argparse
import matplotlib.pyplot as plt
import shutil

import numpy as np
import open3d as o3d
import cv2

from floor_reader import main as floor_reader
from floor_slicer import main as floor_slicer
from run_metadata import main as run_metadata

# ---------------------------
# Core pipeline functions
# ---------------------------

def load_point_cloud(path: Path):
    """Load PLY and return full + downsampled point clouds and color array."""
    pcd = o3d.io.read_point_cloud(str(path))
    points = np.asarray(pcd.points)
    has_colors = (np.asarray(pcd.colors).size != 0)
    colors = np.asarray(pcd.colors) if has_colors else None
    return pcd, points, has_colors, colors

def voxel_downsample(pcd: o3d.geometry.PointCloud, voxel_size: float):
    pcd_down = pcd.voxel_down_sample(voxel_size=float(voxel_size))
    pts_down = np.asarray(pcd_down.points)
    return pcd_down, pts_down


def build_slice_grid(points_down: np.ndarray,
                     zmin: float,
                     zmax: float,
                     grid_res: float):
    """Slice by Z, rasterize to occupancy grid, return grid + metadata."""
    mask = (points_down[:, 2] > zmin) & (points_down[:, 2] < zmax)
    slice_points = points_down[mask]
    if slice_points.shape[0] == 0:
        raise ValueError("No points found in the selected Z range — adjust SLICE_HEIGHT_*.")

    x, y = slice_points[:, 0], slice_points[:, 1]
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    width = int(np.ceil((x_max - x_min) / grid_res))
    height = int(np.ceil((y_max - y_min) / grid_res))
    if width <= 0 or height <= 0:
        raise ValueError("Computed grid has non-positive width/height. Check grid resolution & data bounds.")

    grid = np.zeros((height, width), dtype=np.uint8)

    ix = ((x - x_min) / grid_res).astype(int)
    iy = ((y - y_min) / grid_res).astype(int)
    iy = height - iy - 1  # flip Y to image coords

    grid[iy, ix] = 255
    return grid, (x_min, x_max, y_min, y_max), (width, height)


def label_rooms_from_grid(grid: np.ndarray,
                          grid_res: float,
                          min_room_area_m2: float):
    """Make free-space image, label rooms, filter small ones, return labeled image."""
    # close small gaps in walls
    grid_closed = cv2.dilate(grid, np.ones((3, 3), np.uint8), iterations=1)

    free_space = cv2.bitwise_not(grid_closed)
    _, free_space_bin = cv2.threshold(free_space, 127, 255, cv2.THRESH_BINARY)

    # 🧹 Remove "outside" free space by flood-filling from the image borders
    h, w = free_space_bin.shape
    mask = np.zeros((h + 2, w + 2), np.uint8)
    flood_filled = free_space_bin.copy()

    # Flood fill from all borders to identify exterior (outside building)
    cv2.floodFill(flood_filled, mask, (0, 0), 0)  # top-left
    cv2.floodFill(flood_filled, mask, (w - 1, 0), 0)  # top-right
    cv2.floodFill(flood_filled, mask, (0, h - 1), 0)  # bottom-left
    cv2.floodFill(flood_filled, mask, (w - 1, h - 1), 0)  # bottom-right

    # "flood_filled" now has 0 where the outside was
    interior_space = flood_filled

    num_labels, labels = cv2.connectedComponents(interior_space)
    print(f"Detected {num_labels - 1} rooms (excluding background and outside).")

    # Filter rooms by area
    label_areas_px = {i: np.sum(labels == i) for i in range(1, num_labels)}
    area_m2 = {i: count * (grid_res ** 2) for i, count in label_areas_px.items()}
    valid = [i for i, a in area_m2.items() if a >= min_room_area_m2]
    removed_labels = [i for i in range(1, num_labels) if i not in valid]

    labels_filtered = labels.copy()
    for lbl in removed_labels:
        labels_filtered[labels_filtered == lbl] = 0  # reset to background

    # Relabel to compact indices 0..K (0 is background)
    num_lbls_filtered, labels_filtered = cv2.connectedComponents((labels_filtered > 0).astype(np.uint8))
    return labels_filtered, num_lbls_filtered - 1, labels  # (labels, count_without_background)


def fill_walls_evenly(labels_filtered: np.ndarray,
                      labels: np.ndarray,
                      pixel_expand:int,
                      width: int, height: int,
                      plot: bool, out_dir: Path):
    """Propagate labels into wall pixels to close gaps."""
    label_filled = labels_filtered.copy()
    # Prepare room membership grid (2D array of sets)
    room_memberships = [[set() for _ in range(width)] for _ in range(height)]

    unique_rooms = np.unique(label_filled[label_filled > 0])
    kernel = np.ones((3, 3), np.uint8)

    for lab in unique_rooms:
        mask = (label_filled == lab).astype(np.uint8)
        dilated = cv2.dilate(mask, kernel, iterations=pixel_expand)

        # Mark all pixels covered by this expanded room
        ys, xs = np.where(dilated > 0)
        for y, x in zip(ys, xs):
            room_memberships[y][x].add(int(lab))

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 3, 1)
    plt.imshow(labels, cmap="tab20b")
    plt.title("Initial Room segmentation")

    plt.subplot(1, 3, 2)
    plt.imshow(labels_filtered, cmap="tab20b")
    plt.title(f"Filtered (>{1} m²)")

    # For visualization: show number of overlapping rooms
    if plot:
        overlap_visual = np.zeros_like(label_filled, dtype=np.float32)
        for y in range(height):
            for x in range(width):
                overlap_visual[y, x] = len(room_memberships[y][x])
        plt.subplot(1, 3, 3)
        plt.imshow(overlap_visual, cmap="plasma")
        plt.title("Expanded Overlaps (yellow = shared walls)")
        plt.colorbar()
        out_path = Path(out_dir / "floorplan.png")
        plt.savefig(out_path)
        plt.show()

    return label_filled, room_memberships


def label_points_from_grid(points_xyz: np.ndarray,
                           room_memberships: list[list[set[AnyStr]]],
                           x_min: float, y_min: float,
                           grid_res: float,
                           width: int, height: int):
    """Assign each 3D point a label by projecting to the grid."""
    point_room_ids = []
    for i in range(points_xyz.shape[0]):
        xs, ys = points_xyz[i, 0], points_xyz[i, 1]
        ixs = int(np.floor((xs - x_min) / grid_res))
        iys = int(np.floor((ys - y_min) / grid_res))
        iy_img = height - iys - 1

        if 0 <= ixs < width and 0 <= iy_img < height:
            point_room_ids.append(room_memberships[iy_img][ixs].copy())
        else:
            point_room_ids.append(set())

    return np.asarray(point_room_ids)

def split_and_save_rooms(points_xyz: np.ndarray,
                         colors: np.ndarray | None,
                         point_room_ids: np.ndarray,
                         unique_labels: np.ndarray,
                         obj_type: str,
                         out_dir: Path):
    """Save each room as a PLY. Returns list of (room_id, path, n_points)."""
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest = []
    for lab in unique_labels:
        indices = [i for i, s in enumerate(point_room_ids) if lab in s]
        if not indices:
            continue
        room_pcd = o3d.geometry.PointCloud()
        room_pcd.points = o3d.utility.Vector3dVector(points_xyz[indices])
        if colors is not None:
            room_pcd.colors = o3d.utility.Vector3dVector(colors[indices])

        out_path = Path(out_dir / f"room_{int(lab):03d}")
        if out_path.exists():
            out_path_room = out_path / f"{obj_type}_{int(lab):03d}.ply"
            o3d.io.write_point_cloud(out_path_room, room_pcd)
            manifest.append((int(lab), str(out_path_room), int(len(indices))))

    return manifest

def save_pano_to_room(pano_locations: dict,
                        pano_coords: np.ndarray,
                        point_room_ids: np.ndarray,
                        source_dir: Path,
                        out_dir: Path):
    """Store the correct panoramas to the correct rooms"""

    workable_rooms = set()
    correct_order_map = {}
    first_label = 1

    for i in range(len(pano_coords)):
        if point_room_ids[i]:
            image = [k for k, v in pano_locations.items() if np.array_equal(v, pano_coords[i])]
            if image:
                room_id = next(iter(point_room_ids[i]))
                if room_id not in workable_rooms:
                    workable_rooms.add(room_id)
                    correct_order_map[room_id] = first_label
                    first_label += 1
                src_path = source_dir / f"{image[0]}"
                out_path = out_dir / f"room_{room_id:03d}"
                out_dir.mkdir(parents=True, exist_ok=True)
                out_path_image = out_path / f"{image[0]}"

                shutil.copy(src_path, out_path_image)

    return workable_rooms, correct_order_map


def write_manifest_rooms(manifest_rows, out_dir: Path,
                         floor_nr: int):
    """Write a simple CSV manifest for downstream use (optional but handy)."""

    mpath = out_dir / "rooms_manifest.csv"
    with mpath.open("w", encoding="utf-8") as f:
        f.write("room_id, floor_id, room_type, total_area, total_volume, length, width, height,ply_path,n_points\n")
        for lab, pth, n in manifest_rows:
            f.write(f"{lab},{floor_nr},some room,0,0,0,0,0,{pth},{n}\n")


def write_manifest_object(manifest_rows, out_dir: Path,
                          floor_nr: int, obj_type: str):
    """Write a simple CSV manifest for downstream use (optional but handy)."""

    for lab, pth, n in manifest_rows:
        current_room = lab
        mpath = out_dir / f"room_{int(lab):03d}/object_manifest.csv"
        write_header = not mpath.exists()
        with mpath.open("a", encoding="utf-8") as f:
            if write_header:
                f.write("object_id, room_id, floor_id, csv_id, obj_name, min_x, max_x, center_x,"
                    "length, area, volume, ply_path, n_points\n")
            for lab, pth, n in manifest_rows:
                if lab == current_room:
                    f.write(f"0,{lab},{floor_nr},0,{obj_type},0,0,0,0,0,0,{pth},{n}\n")


# ---------------------------
# Orchestrator
# ---------------------------

def run_split(
    input_dir: Path,
    output_dir: Path,
    obj_types: list[str],
    floor_nr: int,
    pano_locations: dict,
    voxel_size: float,
    slice_min: float,
    slice_max: float,
    grid_res: float,
    min_room_area_m2: float,
    expand_dist: float,
    plot: bool,
):
    input_grid_file = input_dir / f"floors/{floor}/combined_structural_{floor}.ply"
    output_dir = output_dir / f"floor_{floor_nr}"
    output_dir.mkdir(parents=True, exist_ok=True)

    pixel_expand = int(np.ceil(expand_dist / grid_res))

    # 1) Load & preprocess
    pcd, points, has_colors, colors = load_point_cloud(Path(input_grid_file))
    print(f"[load] #points: {len(points)}")

    pcd_down, pts_down = voxel_downsample(pcd, voxel_size)
    print(f"[voxel] #points: {len(pts_down)}")

    # 2) Slice and rasterize
    grid, (x_min, x_max, y_min, y_max), (width, height) = build_slice_grid(
        pts_down, slice_min, slice_max, grid_res
    )

    # 3) Label rooms in 2D, filter, and fill
    labels_2d, count_rooms, labels = label_rooms_from_grid(grid, grid_res, min_room_area_m2)
    print(f"[rooms] after filtering: {count_rooms}")

    labels_2d_filled, room_membership = fill_walls_evenly(
        labels_2d, labels, pixel_expand, width, height, plot, output_dir
    )

    pano_coords = np.asarray(list(pano_locations.values()))

    point_room_ids_pano = label_points_from_grid(
        pano_coords, room_membership, x_min, y_min, grid_res, width, height
    )

    input_dir_panos = input_dir / f"panoramas/images"

    workable_rooms, correct_order_map = save_pano_to_room(
        pano_locations, pano_coords, point_room_ids_pano, input_dir_panos, output_dir
    )

    for inner_list in room_membership:
        for i in range(len(inner_list)):
            s = inner_list[i]
            if not isinstance(s, set):
                try:
                    s = set(s)
                except TypeError:
                    s = set()
            filtered = s & workable_rooms
            if filtered:
                reworked_value = set()
                reworked_value.add(correct_order_map[next(iter(filtered))])
                inner_list[i] = reworked_value
            else:
                inner_list[i] = set()

    # 4) Project labels to 3D points
    point_room_ids = label_points_from_grid(
        points, room_membership, x_min, y_min, grid_res, width, height
    )

    unique_labels = np.array([0])
    for label in correct_order_map:
        unique_labels = unique_labels.append(label)

    # 5) Save each room PLY + optional combined colored
    obj_type = "shell"

    manifest = split_and_save_rooms(
        points, colors if has_colors else None, point_room_ids, unique_labels, obj_type, output_dir
    )
    print(f"[save] saved {len(manifest)} {obj_type} files to {output_dir}")

    write_manifest_rooms(manifest, output_dir, floor_nr)
    print(f"[save] {obj_type} manifest")

    # Test for objects
    for object in obj_types:
        path = input_dir / f"floors/{floor_nr}/{object}_{floor_nr}.ply"
        pcd_obj, points_obj, has_colors_obj, colors_obj = load_point_cloud(path)
        print(f"[load] #points {object}: {len(points_obj)}")

        point_room_ids_object = label_points_from_grid(
            points_obj, room_membership, x_min, y_min, grid_res, width, height
        )

        manifest2 = split_and_save_rooms(
            points_obj, colors_obj if has_colors_obj else None, point_room_ids_object, unique_labels, object, output_dir
        )
        print(f"[save] saved {len(manifest2)} {object}s files to {output_dir}")

        write_manifest_object(manifest2, output_dir, floor_nr, object)
        print(f"[save] {object} manifest")

    print("Done. Each dense room exported to: ", output_dir)


# ---------------------------
# CLI
# ---------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Split rooms from a PLY point cloud and save per-room PLYs.")
    p.add_argument("--input_dir",type=Path, default=Path("input_house"))
    p.add_argument("--output_dir", type=Path, default=Path("output_house"))
    p.add_argument("--obj_types", type=str, nargs="+", default=["chair","couch","curtain","door","monitor","plant","table","window"])
    p.add_argument("--voxel", type=float, default=0.15, help="Voxel downsample size in meters.")
    p.add_argument("--grid", type=float, default=0.10, help="Grid resolution in meters per pixel.")
    p.add_argument("--min-room-area", type=float, default=1.0, help="Minimum room area (m²) to keep.")
    p.add_argument("--expand_dist", type=float, default=0.30, help="Expand distance in meters.")
    p.add_argument("--plot", type=bool, default=False)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Main variabales
    floor = 0
    obj_types = args.obj_types

    pano_locations = run_metadata(
        input_dir=args.input_dir / "panoramas"
    )

    ground_floor_level, ceiling_level = floor_reader(
        input_dir=args.input_dir,
        plot=True,
    )

    filtered_pano_locations = {
        k: v for k, v in pano_locations.items()
        if ground_floor_level <= v[2] <= ceiling_level[0]
    }

    # Slice shells
    floor_slicer(
        input_dir=args.input_dir,
        obj_type="combined_structural",
        z_min=ground_floor_level[0]-0.10,
        z_max=ceiling_level[0],
        floor_nr=floor,
    )

    #Slice objects
    for object in obj_types:
        floor_slicer(
            input_dir=args.input_dir,
            obj_type=object,
            z_min=ground_floor_level[0]-0.10,
            z_max=ceiling_level[0],
            floor_nr=floor,
        )

    # First run for groundfloor
    print("Generating rooms from ground floor")
    run_split(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        obj_types=obj_types,
        floor_nr=floor,
        pano_locations=filtered_pano_locations,
        voxel_size=args.voxel,
        slice_min=ground_floor_level[0] + 0.15, # Change this between house and uni
        slice_max=ceiling_level[0] - 0.50, # Change this between house and uni
        grid_res=args.grid,
        min_room_area_m2=args.min_room_area,
        expand_dist=args.expand_dist,
        plot=args.plot,
    )

    floor += 1

    # If house has multiple floors
    for i in range(len(ceiling_level)):
        # Top floor
        if i == len(ceiling_level) - 1:
            filtered_pano_locations = {
                k: v for k, v in pano_locations.items()
                if ceiling_level[i] <= v[2]
            }

            floor_slicer(
                input_dir=args.input_dir,
                obj_type="combined_structural",
                z_min=ceiling_level[i]-0.20,
                z_max=15.0,
                floor_nr=floor,
            )

            # Slice objects
            for object in obj_types:
                floor_slicer(
                    input_dir=args.input_dir,
                    obj_type=object,
                    z_min=ceiling_level[i]-0.20,
                    z_max=15.0,
                    floor_nr=floor,
                )

            print(f"Generating rooms from floor_{floor}")
            run_split(
                input_dir=args.input_dir,
                output_dir=args.output_dir,
                obj_types=obj_types,
                floor_nr=floor,
                pano_locations=filtered_pano_locations,
                voxel_size=args.voxel,
                slice_min=ceiling_level[i] + 0.15,
                slice_max=ceiling_level[i] + 1.15,
                grid_res=0.25,
                min_room_area_m2=args.min_room_area,
                expand_dist=args.expand_dist,
                plot=args.plot,
            )
        else:
            filtered_pano_locations = {
                k: v for k, v in pano_locations.items()
                if ceiling_level[i] <= v[2] <= ceiling_level[i+1]
            }

            floor_slicer(
                input_dir=args.input_dir,
                obj_type="combined_structural",
                z_min=ceiling_level[i]-0.15,
                z_max=ceiling_level[i+1]+0.15,
                floor_nr=floor,
            )

            # Slice objects
            for object in obj_types:
                floor_slicer(
                    input_dir=args.input_dir,
                    obj_type=object,
                    z_min=ceiling_level[i]-0.15,
                    z_max=ceiling_level[i+1]+0.15,
                    floor_nr=floor,
                )

            print(f"Generating rooms from floor_{floor}")
            run_split(
                input_dir=args.input_dir,
                output_dir=args.output_dir,
                obj_types=obj_types,
                floor_nr=floor,
                pano_locations=filtered_pano_locations,
                voxel_size=args.voxel,
                slice_min=ceiling_level[i] + 0.20,
                slice_max=ceiling_level[i+1] - 0.20,
                grid_res=args.grid,
                min_room_area_m2=args.min_room_area,
                expand_dist=args.expand_dist,
                plot=args.plot,
            )
            floor += 1