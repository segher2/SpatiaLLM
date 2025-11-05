#!/usr/bin/env python3
"""
Slice a point cloud (.ply) between given height limits (Z-axis).

Usage:
    python slice_pointcloud.py --input building.ply --z_min 0.0 --z_max 3.0 --output slice_1.ply
"""

import argparse
import open3d as o3d
import numpy as np
from pathlib import Path

def load_point_cloud(path: Path):
    """Load PLY and return height values"""
    pcd = o3d.io.read_point_cloud(str(path))
    return pcd

def slice_pointcloud(pcd, z_min, z_max):
    """Return a new point cloud with points between z_min and z_max."""
    points = np.asarray(pcd.points)
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    sliced_points = points[mask]

    if len(sliced_points) == 0:
        print("⚠️ No points found in this slice range.")
        return o3d.geometry.PointCloud()

    sliced_pcd = o3d.geometry.PointCloud()
    sliced_pcd.points = o3d.utility.Vector3dVector(sliced_points)

    # copy colors if present
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[mask]
        sliced_pcd.colors = o3d.utility.Vector3dVector(colors)

    return sliced_pcd

def main(input_dir, obj_type, z_min, z_max, floor_nr):
    # Load point cloud

    input_obj = input_dir / f"classes/{obj_type}.ply"

    pcd = load_point_cloud(input_obj)

    # Make slices
    sliced_pcd = slice_pointcloud(pcd, z_min, z_max)

    # Store slices ply in dir
    out_path = input_dir / f"floors/{floor_nr}/{obj_type}_{floor_nr}.ply"
    out_path.parent.mkdir(parents=True, exist_ok=True)  # make sure folder exists
    o3d.io.write_point_cloud(out_path, sliced_pcd)
    print(f"\n {obj_type} PC of floor {floor_nr} written to {out_path}")

