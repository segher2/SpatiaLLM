#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path
import easy3d

def process_one(ply_path: Path, out_path: Path,
                min_support: int, dist_threshold: float,
                bitmap_resolution: float, normal_threshold: float,
                overlook_probability: float) -> None:
    pc = easy3d.PointCloudIO.load(str(ply_path))
    print(f"[easy3d] {ply_path.name}: loaded {pc.n_vertices()} pts")

    # normals for RANSAC
    easy3d.PointCloudNormals.estimate(pc, 16)

    # planes only
    ransac = easy3d.PrimitivesRansac()
    ransac.add_primitive_type(easy3d.PrimitivesRansac.PLANE)
    ransac.detect(
        pc,
        min_support=min_support,
        dist_threshold=dist_threshold,
        bitmap_resolution=bitmap_resolution,
        normal_threshold=normal_threshold,
        overlook_probability=overlook_probability
    )
    planes = ransac.get_planes()
    print(f"[easy3d] {ply_path.name}: planes={len(planes)}")

    # Save the (labeled) point cloud as .bvg
    out_path.parent.mkdir(parents=True, exist_ok=True)
    easy3d.PointCloudIO.save(str(out_path), pc)
    print(f"[easy3d] {ply_path.name} -> {out_path.name}")

def main():
    ap = argparse.ArgumentParser(description="Batch: convert *.ply to *.bvg with plane RANSAC (Easy3D).")
    ap.add_argument("--in-dir", type=Path, required=True, help="Input directory containing .ply files")
    ap.add_argument("--out-dir", type=Path, required=True, help="Output directory for .bvg files")
    ap.add_argument("--glob", type=str, default="*.ply", help="Glob for selecting inputs (default: *.ply)")
    # RANSAC params (same defaults as your snippet, except min_support=5000 like your latest)
    ap.add_argument("--min-support", type=int, default=5000)
    ap.add_argument("--dist-threshold", type=float, default=0.005)
    ap.add_argument("--bitmap-resolution", type=float, default=0.02)
    ap.add_argument("--normal-threshold", type=float, default=0.8)
    ap.add_argument("--overlook-probability", type=float, default=0.001)
    args = ap.parse_args()

    in_dir  = args.in_dir
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    easy3d.initialize(True)

    ply_files = sorted(in_dir.glob(args.glob))
    print(f"[easy3d] Found {len(ply_files)} files in {in_dir}")

    for i, ply_path in enumerate(ply_files, 1):
        bvg_path = out_dir / (ply_path.stem + ".bvg")
        print(f"[{i}/{len(ply_files)}] {ply_path.name}")
        process_one(
            ply_path, bvg_path,
            args.min_support, args.dist_threshold,
            args.bitmap_resolution, args.normal_threshold,
            args.overlook_probability
        )

    # Snakemake-friendly marker
    (out_dir / "_SUCCESS").write_text("OK\n", encoding="utf-8")
    print(f"[easy3d] Done. Wrote {(out_dir / '_SUCCESS')}")

if __name__ == "__main__":
    main()
