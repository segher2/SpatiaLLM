#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import laspy
import numpy as np
from plyfile import PlyData, PlyElement

# --- keep your existing helper functions here (resolve_dimension_name, ensure_list, etc.) ---

classes_dict = {
    0: "unclassified", 1: "ceiling", 2: "floor", 3: "wall", 4: "wall_ext",
    5: "beam", 6: "column", 7: "window", 8: "door", 9: "door_leaf",
    10: "plant", 11: "curtain", 12: "stairs", 13: "clutter", 14: "noise",
    15: "person", 16: "kitchen_cabinet", 17: "lamp", 18: "bed", 19: "table",
    20: "chair", 21: "couch", 22: "monitor", 23: "cupboard", 24: "shelves",
    25: "builtin_cabinet", 26: "tree", 27: "ground", 28: "car", 29: "grass", 30: "other"
}



def resolve_dimension_name(las: laspy.LasData, attr_name: str) -> str:
    """Return the actual (case-correct) dimension name or raise KeyError."""
    dims = [d.name for d in las.point_format.dimensions]
    for d in dims:
        if d.lower() == attr_name.lower():
            return d
    raise KeyError(f"Attribute '{attr_name}' not found. Available: {dims}")


def ensure_list(x):
    if isinstance(x, (list, tuple, np.ndarray)):
        return [int(v) for v in x]
    return [int(x)]


def build_vertex_array(las: laspy.LasData, mask: np.ndarray) -> np.ndarray:
    """Create a structured numpy array for PLY 'vertex' with available attributes."""
    idx = mask.astype(bool)
    n = int(idx.sum())
    if n == 0:
        return np.zeros(0, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    # Always include XYZ as float32 (use las.xyz to avoid ScaledArrayView issues)
    coords = las.xyz[idx].astype(np.float32, copy=False)  # (n, 3)

    fields = [('x', 'f4'), ('y', 'f4'), ('z', 'f4')]

    # Optional fields if present
    dims = {d.name.lower() for d in las.point_format.dimensions}
    include_color = {'red', 'green', 'blue'}.issubset(dims)
    include_intensity = 'intensity' in dims
    include_class = 'classification' in dims
    include_gpstime = 'gps_time' in dims

    if include_color:
        fields += [('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
    if include_intensity:
        fields += [('intensity', 'u2')]
    if include_class:
        fields += [('classification', 'u1')]
    if include_gpstime:
        fields += [('gps_time', 'f8')]

    verts = np.empty(n, dtype=fields)
    verts['x'] = coords[:, 0]
    verts['y'] = coords[:, 1]
    verts['z'] = coords[:, 2]

    if include_color:
        # Colors are usually uint16 in LAS; convert to uint8 for PLY
        r = np.asarray(las.red)[idx]
        g = np.asarray(las.green)[idx]
        b = np.asarray(las.blue)[idx]
        verts['red']   = np.clip((r / 256.0).round(), 0, 255).astype(np.uint8)
        verts['green'] = np.clip((g / 256.0).round(), 0, 255).astype(np.uint8)
        verts['blue']  = np.clip((b / 256.0).round(), 0, 255).astype(np.uint8)

    if include_intensity:
        verts['intensity'] = np.asarray(las.intensity)[idx].astype(np.uint16, copy=False)
    if include_class:
        verts['classification'] = np.asarray(las.classification)[idx].astype(np.uint8, copy=False)
    if include_gpstime:
        verts['gps_time'] = np.asarray(las.gps_time)[idx].astype(np.float64, copy=False)

    return verts


def write_subset_ply(las: laspy.LasData, mask: np.ndarray, out_path: Path) -> int:
    count = int(mask.sum())

    # build a vertex array with the right dtype even if empty
    vertex_array = build_vertex_array(las, mask)

    # write file regardless of count (Snakemake needs the file to exist)
    el = PlyElement.describe(vertex_array, 'vertex', comments=['filtered from LAS'])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    PlyData([el], text=False).write(str(out_path))  # binary little-endian PLY

    return count




def filter_to_ply(in_path: Path, out_path: Path, attr_name: str, target_ids, classes_dict: dict, mode: str = "combined"):
    las = laspy.read(in_path)
    dim = resolve_dimension_name(las, attr_name)
    ids = ensure_list(target_ids)
    values = las[dim]

    if mode.lower() == "combined":
        mask = np.isin(values, ids)
        kept = write_subset_ply(las, mask, Path(out_path).with_suffix('.ply'))
        if kept == 0:
            print(f"No points with {dim} in {ids}. Nothing written.")
        else:
            print(f"Wrote {kept} points with {dim} in {ids} to {Path(out_path).with_suffix('.ply')}")
        return

    # separate mode: here we expect exactly one ID and an explicit output file path
    if len(ids) != 1:
        raise ValueError("In --mode separate, pass exactly one ID (use --ids <id>).")
    idv = int(ids[0])
    mask = (values == idv)
    kept = write_subset_ply(las, mask, Path(out_path))
    kept = write_subset_ply(las, mask, Path(out_path))

    if kept == 0:
        print(f"No points with {dim} == {idv}. Wrote empty PLY to {out_path}.")
    else:
        print(f"Wrote {kept} points with {dim} == {idv} to {out_path}")


def parse_args():
    p = argparse.ArgumentParser(description="Filter LAS/LAZ by integer attribute values and export to PLY.")
    p.add_argument("--input", required=True, type=Path, help="Path to input .las/.laz")
    p.add_argument("--output", required=True, type=Path,
                   help="For --mode combined: the output file (e.g. output/combined.ply or a stem). "
                        "For --mode separate: the exact per-ID output file path (e.g. output/27.ply).")
    p.add_argument("--attribute", default="Classification", help="Dimension/attribute to filter on")
    p.add_argument("--ids", required=True, help="Comma-separated list of IDs (e.g. 1,2,3) or a single ID.")
    p.add_argument("--mode", choices=["combined", "separate"], default="combined",
                   help="combined: writes one file for all IDs; separate: expects one ID and one output path")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    # parse ids
    ids = [int(x) for x in args.ids.split(",") if x.strip() != ""]
    try:
        filter_to_ply(args.input, args.output, args.attribute, ids, classes_dict, args.mode)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
