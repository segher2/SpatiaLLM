#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path
import sys
import time
import polyfit
import numpy as np


def _obj_vertices_and_faces(obj_path: Path):
    """Parse an OBJ and return (verts, faces).
       Supports n-gons and negative indices; ignores vt/vn."""
    verts = []
    faces = []
    with obj_path.open("r", encoding="utf-8", errors="ignore") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            head, *rest = line.split(None, 1)
            if head == "v":
                parts = line.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif head == "f":
                part = rest[0] if rest else ""
                tokens = part.split()
                idxs = []
                for tok in tokens:
                    v_str = tok.split('/')[0]
                    if not v_str:
                        continue
                    idx = int(v_str)
                    # OBJ is 1-based; negative means "count from end"
                    if idx > 0:
                        idx0 = idx - 1
                    else:
                        idx0 = len(verts) + idx
                    idxs.append(idx0)
                if len(idxs) >= 3:
                    faces.append(idxs)
    return verts, faces


def _signed_volume_from_obj(obj_path: Path) -> float:
    """Compute signed volume from an OBJ (fan-triangulates n-gons)."""
    import numpy as np
    V = 0.0
    verts, faces = _obj_vertices_and_faces(obj_path)
    if not verts or not faces:
        return 0.0
    arr = np.asarray(verts, dtype=float)
    for face in faces:
        v0 = arr[face[0]]
        for k in range(1, len(face) - 1):
            v1 = arr[face[k]]
            v2 = arr[face[k + 1]]
            V += np.dot(np.cross(v0, v1), v2)
    return V / 6.0


def _write_flipped_obj(in_path: Path, out_path: Path):
    """Write an OBJ with all face windings reversed; preserves vt/vn tokens."""
    with in_path.open("r", encoding="utf-8", errors="ignore") as fin, \
         out_path.open("w", encoding="utf-8") as fout:
        for raw in fin:
            s = raw.lstrip()
            if s.startswith("f "):
                # keep original leading whitespace
                lead = raw[:len(raw) - len(s)]
                parts = s.strip().split()
                toks = parts[1:]
                # reverse order for any polygon (triangles included)
                toks_rev = toks[::-1]
                fout.write(f"{lead}f " + " ".join(toks_rev) + "\n")
            else:
                fout.write(raw)



def reconstruct_one(vg_path: Path, out_path: Path, solver_enum, w_data: float, w_cover: float, w_complex: float) -> tuple[bool, int]:
    """Return (success, faces)."""
    pc = polyfit.read_point_set(str(vg_path))
    if not pc:
        print(f"[polyfit] FAIL read: {vg_path}", file=sys.stderr, flush=True)
        return False, -1

    mesh = polyfit.reconstruct(pc, solver_enum, w_data, w_cover, w_complex)
    if not mesh:
        print(f"[polyfit] FAIL reconstruct: {vg_path}", file=sys.stderr, flush=True)
        return False, -1

    # --- temp export, check signed volume, flip if needed ---
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Save to *.tmp.obj so polyfit recognizes the .obj writer
    tmp_obj = out_path.parent / f"{out_path.stem}.tmp.obj"


    if not polyfit.save_mesh(str(tmp_obj), mesh):
        print(f"[polyfit] FAIL save (temp): {tmp_obj}", file=sys.stderr, flush=True)
        return False, -1

    try:
        vol = _signed_volume_from_obj(tmp_obj)
        eps = 1e-12
        if vol < -eps:
            print(f"[polyfit] Flipping normals for {vg_path.name} (signed volume={vol:.3e})", flush=True)
            _write_flipped_obj(tmp_obj, out_path)
            tmp_obj.unlink(missing_ok=True)
        else:
            # positive or ~zero: keep as-is
            tmp_obj.replace(out_path)
            if abs(vol) <= eps:
                print(f"[polyfit] Warning: near-zero volume estimate for {vg_path.name} (|V|<={eps})", flush=True)
    except Exception as e:
        print(f"[polyfit] Warning: could not check normals from OBJ: {e}", flush=True)
        # Fallback: keep the unmodified temp as final
        tmp_obj.replace(out_path)

    faces = mesh.size_of_facets()
    print(f"[polyfit] OK: {vg_path.name} -> {out_path.name} (faces={faces})", flush=True)

    # help GC
    del mesh
    del pc
    return True, faces


def main():
    ap = argparse.ArgumentParser(description="Batch PolyFit: convert *.vg to *.obj")
    ap.add_argument("--in-dir", type=Path, required=True, help="Directory with .vg files")
    ap.add_argument("--out-dir", type=Path, required=True, help="Directory to write .obj files")
    ap.add_argument("--glob", type=str, default="*.bvg", help="Glob pattern for inputs")
    ap.add_argument("--solver", type=str, default="SCIP", choices=["SCIP","GUROBI"], help="PolyFit solver enum name")
    ap.add_argument("--w-data", type=float, default=0.43, help="Weight: data fitting")
    ap.add_argument("--w-cover", type=float, default=0.27, help="Weight: model coverage")
    ap.add_argument("--w-complex", type=float, default=0.30, help="Weight: model complexity")
    args = ap.parse_args()

    in_dir  = args.in_dir
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    vg_paths = sorted(in_dir.glob(args.glob))
    print(f"[polyfit] Found {len(vg_paths)} .bvg files in {in_dir}", flush=True)

    # Initialize once
    polyfit.initialize()
    ok_all = True
    manifest = out_dir / "obj_manifest.csv"

    try:
        with manifest.open("w", encoding="utf-8") as mf:
            mf.write("vg_path,obj_path,faces,success\n")
            t0 = time.perf_counter()
            # map solver string to enum safely
            solver_enum = getattr(polyfit, args.solver, polyfit.SCIP)

            for i, vg in enumerate(vg_paths, 1):
                obj = out_dir / (vg.stem + ".obj")
                print(f"[polyfit] ({i}/{len(vg_paths)}) {vg.name}", flush=True)
                try:
                    success, faces = reconstruct_one(vg, obj, solver_enum, args.w_data, args.w_cover, args.w_complex)
                except Exception as e:
                    print(f"[polyfit] EXCEPTION on {vg.name}: {e}", file=sys.stderr, flush=True)
                    success, faces = False, -1
                mf.write(f"{vg},{obj},{faces},{int(success)}\n")
                mf.flush()
                ok_all = ok_all and success

            dt = time.perf_counter() - t0
            print(f"[polyfit] Done {len(vg_paths)} files in {dt:.1f}s (ok_all={ok_all})", flush=True)
    finally:
        # Write marker last, so Snakemake can depend on it
        (out_dir / "_SUCCESS").write_text(("OK\n" if ok_all else "SOME_FAIL\n"), encoding="utf-8")
        ok_all = True
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
