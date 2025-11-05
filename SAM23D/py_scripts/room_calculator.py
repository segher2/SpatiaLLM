#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import csv
import argparse
from collections import defaultdict
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import urlopen

# ---------- vector helpers ----------
def _norm3(v): return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
def _normalize(n):
    l = _norm3(n)
    return (0.0,0.0,0.0) if l == 0.0 else (n[0]/l, n[1]/l, n[2]/l)
def _dot(a,b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def _sub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _cross(a,b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _angle_deg_unit(n1, n2):
    c = max(-1.0, min(1.0, _dot(n1, n2)))
    return math.degrees(math.acos(c))

def _canonicalize_plane(n, d):
    nx, ny, nz = n
    flip = (nx < 0) or (nx == 0 and ny < 0) or (nx == 0 and ny == 0 and nz < 0)
    return ((-nx,-ny,-nz), -d) if flip else (n, d)

def _bbox_scale(verts):
    if not verts: return 1.0
    xs = [v[0] for v in verts]; ys = [v[1] for v in verts]; zs = [v[2] for v in verts]
    dx, dy, dz = max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)
    diag = math.sqrt(dx*dx + dy*dy + dz*dz)
    return diag if diag > 0 else 1.0

def _tri_area_normal(a,b,c):
    ab = _sub(b,a); ac = _sub(c,a)
    cr = _cross(ab, ac)
    area2 = _norm3(cr)
    return 0.5*area2, cr

def _poly_area_normal_and_centroid(verts, idxs):
    if len(idxs) < 3:
        return 0.0, (0.0,0.0,0.0), (0.0,0.0,0.0)
    a0 = verts[idxs[0]]
    acc_area = 0.0
    acc_normal = (0.0,0.0,0.0)
    acc_cent = (0.0,0.0,0.0)
    for i in range(1, len(idxs)-1):
        a = a0; b = verts[idxs[i]]; c = verts[idxs[i+1]]
        tri_area, cr = _tri_area_normal(a,b,c)
        if tri_area <= 0: continue
        acc_area += tri_area
        acc_normal = (acc_normal[0]+cr[0], acc_normal[1]+cr[1], acc_normal[2]+cr[2])
        tc = ((a[0]+b[0]+c[0])/3.0, (a[1]+b[1]+c[1])/3.0, (a[2]+b[2]+c[2])/3.0)
        acc_cent = (acc_cent[0]+tc[0]*tri_area, acc_cent[1]+tc[1]*tri_area, acc_cent[2]+tc[2]*tri_area)
    n_unit = _normalize(acc_normal)
    centroid = (acc_cent[0]/acc_area, acc_cent[1]/acc_area, acc_cent[2]/acc_area) if acc_area > 0 else (0.0,0.0,0.0)
    return acc_area, n_unit, centroid

# ---------- OBJ I/O ----------
def load_obj_from_url(obj_url: str):
    parsed = urlparse(obj_url)
    if parsed.scheme in ("http", "https", "file"):
        with urlopen(obj_url) as r:
            text = r.read().decode("utf-8", errors="ignore")
    else:
        text = Path(obj_url).read_text(encoding="utf-8", errors="ignore")

    vertices = []
    faces = []
    for line in text.splitlines():
        if not line or line.startswith('#'): continue
        s = line.strip()
        if not s: continue
        tag = s.split(None, 1)[0]
        if tag == "v":
            parts = s.split()
            if len(parts) >= 4:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                vertices.append((x,y,z))
        elif tag == "f":
            parts = s.split()[1:]
            idxs = []
            for tok in parts:
                vi_str = tok.split('/')[0]
                if not vi_str: continue
                vi = int(vi_str)
                if vi < 0:
                    vi = len(vertices) + 1 + vi
                idxs.append(vi-1)
            if len(idxs) >= 3:
                faces.append(idxs)
    return vertices, faces

def save_plane_obj(path: Path, vertices, faces_subset, comments=None):
    used = []
    remap = {}
    for idxs in faces_subset:
        for vi in idxs:
            if vi not in remap:
                remap[vi] = len(used)
                used.append(vi)
    local_vs = [vertices[vi] for vi in used]
    local_fs = [[remap[vi]+1 for vi in idxs] for idxs in faces_subset]

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        if comments:
            for c in comments:
                f.write(f"# {c}\n")
        for vx,vy,vz in local_vs:
            f.write(f"v {vx:.9g} {vy:.9g} {vz:.9g}\n")
        f.write("g plane\n")
        for face in local_fs:
            f.write("f " + " ".join(str(i) for i in face) + "\n")

# ---------- classification ----------
def classify_plane(normal, wall_tol_deg=10.0, floor_tol_deg=20.0):
    up = (0.0, 0.0, 1.0)
    ang = _angle_deg_unit(normal, up)  # 0=up, 90=vertical, 180=down
    if ang >= 180.0 - floor_tol_deg:
        return "floor"
    if abs(ang - 90.0) <= wall_tol_deg:
        return "wall"
    return "ceiling"

# ---------- grouping ----------
def group_coplanar(vertices, faces, angle_deg=1.0, dist_eps=None):
    scale = _bbox_scale(vertices)
    if dist_eps is None:
        dist_eps = 1e-4 * scale

    F = len(faces)
    face_area     = [0.0]*F
    face_normal   = [(0.0,0.0,0.0)]*F
    face_centroid = [(0.0,0.0,0.0)]*F
    face_point    = [(0.0,0.0,0.0)]*F
    bins = defaultdict(list)

    for fi, idxs in enumerate(faces):
        area, n, c = _poly_area_normal_and_centroid(vertices, idxs)
        if area <= 0 or _norm3(n) == 0:
            continue
        p0 = vertices[idxs[0]]
        d = -_dot(n, p0)
        n, d = _canonicalize_plane(n, d)
        face_area[fi] = area
        face_normal[fi] = n
        face_centroid[fi] = c
        face_point[fi] = p0
        qn = 1000
        qd = 1.0/max(dist_eps, 1e-12)
        key = (int(round(n[0]*qn)), int(round(n[1]*qn)), int(round(n[2]*qn)), int(round(d*qd)))
        bins[key].append(fi)

    angle_tol = angle_deg
    refined_groups = []
    for _, cand in bins.items():
        used = set()
        for fi in cand:
            if fi in used: continue
            sn = face_normal[fi]
            sp = face_point[fi]
            sd = -_dot(sn, sp)
            sn, sd = _canonicalize_plane(sn, sd)
            bucket = [fi]; used.add(fi)
            for fj in cand:
                if fj in used: continue
                nj = face_normal[fj]
                pj = face_point[fj]
                dj = -_dot(nj, pj)
                nj, dj = _canonicalize_plane(nj, dj)
                if _angle_deg_unit(sn, nj) <= angle_tol and abs(sd - dj) <= dist_eps:
                    bucket.append(fj); used.add(fj)
            refined_groups.append(bucket)

    groups = []
    for gi, face_ids in enumerate(refined_groups, 1):
        total_area = 0.0
        acc_n = (0.0,0.0,0.0)
        acc_c = (0.0,0.0,0.0)
        used_vids = set()
        for fi in face_ids:
            a = face_area[fi]; n = face_normal[fi]; c = face_centroid[fi]
            total_area += a
            acc_n = (acc_n[0]+n[0]*a, acc_n[1]+n[1]*a, acc_n[2]+n[2]*a)
            acc_c = (acc_c[0]+c[0]*a, acc_c[1]+c[1]*a, acc_c[2]+c[2]*a)
            for vid in faces[fi]:
                used_vids.add(vid)
        n_avg = _normalize(acc_n) if total_area > 0 else (0.0,0.0,0.0)
        centroid = (acc_c[0]/total_area, acc_c[1]/total_area, acc_c[2]/total_area) if total_area > 0 else (0.0,0.0,0.0)

        tmin = float("inf"); tmax = float("-inf")
        pmin = (0.0,0.0,0.0); pmax = (0.0,0.0,0.0)
        for vid in used_vids:
            p = vertices[vid]
            t = _dot(p, n_avg)
            if t < tmin: tmin, pmin = t, p
            if t > tmax: tmax, pmax = t, p

        groups.append({
            "name": f"plane_{gi:04d}",
            "face_ids": sorted(face_ids),
            "area": total_area,
            "normal": n_avg,
            "centroid": centroid,
            "lowest": pmin,
            "highest": pmax
        })

    groups.sort(key=lambda g: -g["area"])
    for i,g in enumerate(groups,1):
        g["name"] = f"plane_{i:04d}"
    return groups

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--obj", required=True, help="Path/URL to OBJ")
    ap.add_argument("--outdir", required=True, help="Directory where planes_report.csv and out_planes/ go")
    ap.add_argument("--angle", type=float, default=1.0, help="Max angle (deg) between normals for coplanarity")
    ap.add_argument("--dist_eps", type=float, default=None, help="Max |d| difference (same units). Default=1e-4*bbox_diag")
    ap.add_argument("--wall_tol_deg", type=float, default=10.0)
    ap.add_argument("--floor_tol_deg", type=float, default=20.0)
    args = ap.parse_args()

    obj_url = args.obj
    out_dir = Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[info] loading OBJ: {obj_url}")
    verts, faces = load_obj_from_url(obj_url)
    if not verts or not faces:
        raise SystemExit("[error] failed to read any vertices/faces from OBJ")

    scale = _bbox_scale(verts)
    dist_eps = args.dist_eps if args.dist_eps is not None else 1e-4 * scale
    print(f"[info] verts={len(verts)} faces={len(faces)} angle_tol={args.angle}° dist_eps={dist_eps:g}")

    groups = group_coplanar(verts, faces, angle_deg=args.angle, dist_eps=dist_eps)
    print(f"[info] planes found: {len(groups)}")

    planes_csv = out_dir / "planes_data.csv"
    with planes_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["group_name","class","face_count","area",
                    "centroid_x","centroid_y","centroid_z",
                    "normal_x","normal_y","normal_z",
                    "lowest_x","lowest_y","lowest_z",
                    "highest_x","highest_y","highest_z","obj_path"])

        planes_out_dir = out_dir / "out_planes"
        planes_out_dir.mkdir(parents=True, exist_ok=True)

        for g in groups:
            nx,ny,nz = g["normal"]
            klass = classify_plane(g["normal"], wall_tol_deg=args.wall_tol_deg, floor_tol_deg=args.floor_tol_deg)
            gname = f"{klass}_{g['name']}"

            plane_faces = [faces[fi] for fi in g["face_ids"]]
            cx,cy,cz = g["centroid"]; lx,ly,lz = g["lowest"];   hx,hy,hz = g["highest"]

            plane_path = planes_out_dir / f"{gname}.obj"
            comments = [
                f"group {gname}",
                f"class {klass}",
                f"faces {len(plane_faces)}",
                f"area {g['area']:.9g}",
                f"centroid {cx:.9g} {cy:.9g} {cz:.9g}",
                f"normal {nx:.9g} {ny:.9g} {nz:.9g}",
                f"lowest {lx:.9g} {ly:.9g} {lz:.9g} (along plane normal)",
                f"highest {hx:.9g} {hy:.9g} {hz:.9g} (along plane normal)"
            ]
            save_plane_obj(plane_path, verts, plane_faces, comments=comments)

            w.writerow([gname, klass, len(plane_faces), f"{g['area']:.9g}",
                        f"{cx:.9g}", f"{cy:.9g}", f"{cz:.9g}",
                        f"{nx:.9g}", f"{ny:.9g}", f"{nz:.9g}",
                        f"{lx:.9g}", f"{ly:.9g}", f"{lz:.9g}",
                        f"{hx:.9g}", f"{hy:.9g}", f"{hz:.9g}",
                        str(plane_path.resolve())])

    print(f"[ok] wrote {planes_csv.resolve()}")
    print(f"[ok] exported {len(groups)} plane OBJs to {planes_out_dir.resolve()}")

if __name__ == "__main__":
    main()
