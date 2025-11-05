configfile: "config.yaml"
import re
import sys
from pathlib import Path
from snakemake.shell import shell
shell.executable("bash")

# config values 
LAS         = config["input"]
OUTDIR      = config["outdir"]
ATTRIBUTE   = config.get("attribute", "Classification")
GROUPS      = config.get("groups", {})
IDS         = [int(i) for i in config["ids"]]

# Extract panoramas
PANORAMA_CFG  = config.get("extract_panoramas", {})
E57           = PANORAMA_CFG.get("input", "data/input/deSkatting.e57")
PANOS_DIR     = PANORAMA_CFG.get("outdir", "data/input/panoramas")
PANOS_DONE    = f"{PANOS_DIR}/_SUCCESS"

# Room segmentation 
ROOMS_CFG     = config.get("rooms", {})
COMBINED_GRP  = ROOMS_CFG.get("combined_group", None)
if COMBINED_GRP is None:
    raise ValueError("Set rooms.combined_group in config (e.g., 'structural').")
COMBINED_PLY  = ROOMS_CFG.get(
    "input_combined",
    f"{OUTDIR}/combined_{COMBINED_GRP}.ply"
)
ROOMS_INDIR   = ROOMS_CFG.get("input", "data/input/")
ROOMS_DIR     = ROOMS_CFG.get("outdir", f"{OUTDIR}/rooms")
VOX           = float(ROOMS_CFG.get("voxel", 0.15))
GRID          = float(ROOMS_CFG.get("grid", 0.10))
MIN_A         = float(ROOMS_CFG.get("min_area", 1.0))
EXPAND_DIST   = float(ROOMS_CFG.get("expand_dist", 0.3))

# BVG/PolyFit over room hierarchy of shell_*.ply 
PLY2BVG_CFG   = config.get("ply2bvg", {})
IN_ROOT       = PLY2BVG_CFG.get("in_root", "data/output")
ROOM_PATTERN  = PLY2BVG_CFG.get("room_pattern", "floor_{floor}/room_{room}")
PLY_GLOB      = PLY2BVG_CFG.get("ply_glob", "shell_*.ply")
BVG_ROOT      = PLY2BVG_CFG.get("out_root", "data/output/bvgfiles")

MIN_SUPPORT     = int(PLY2BVG_CFG.get("min_support", 5000))
DIST_THRESH     = float(PLY2BVG_CFG.get("dist_threshold", 0.005))
BITMAP_RES      = float(PLY2BVG_CFG.get("bitmap_resolution", 0.02))
NORMAL_THRESH   = float(PLY2BVG_CFG.get("normal_threshold", 0.8))
OVERLOOK_PROB   = float(PLY2BVG_CFG.get("overlook_probability", 0.001))

POLY_CFG     = config.get("polyfit", {})
OBJ_ROOT     = POLY_CFG.get("out_root", "data/output/objfiles")
SOLVER       = POLY_CFG.get("solver", "SCIP")
W_DATA       = float(POLY_CFG.get("w_data", 0.43))
W_COVER      = float(POLY_CFG.get("w_cover", 0.27))
W_COMPLEX    = float(POLY_CFG.get("w_complex", 0.30))

# CSV export over OBJ rooms 
CSV_CFG   = config.get("rooms_csvs", {})  
CSV_ROOT  = CSV_CFG.get("out_root", "planes") 
ANGLE_DEG = float(CSV_CFG.get("angle_deg", 1.0))
DIST_EPS  = CSV_CFG.get("dist_eps", 0.0005)
WALL_TOL  = float(CSV_CFG.get("wall_tol_deg", 10.0))
FLOOR_TOL = float(CSV_CFG.get("floor_tol_deg", 20.0))


# class dictionary (id -> name)
CLASSES = {
    0: "unclassified", 1: "ceiling", 2: "floor", 3: "wall", 4: "wall_ext",
    5: "beam", 6: "column", 7: "window", 8: "door", 9: "door_leaf",
    10: "plant", 11: "curtain", 12: "stairs", 13: "clutter", 14: "noise",
    15: "person", 16: "kitchen_cabinet", 17: "lamp", 18: "bed", 19: "table",
    20: "chair", 21: "couch", 22: "monitor", 23: "cupboard", 24: "shelves",
    25: "builtin_cabinet", 26: "tree", 27: "ground", 28: "car", 29: "grass", 30: "other"
}

# Build name lists and inverse mapping
NAMES   = [CLASSES[i] for i in IDS]
NAME2ID = {v: k for k, v in CLASSES.items()}

# Constrain {name} to the exact allowed names so it won't match "combined_*"
NAME_REGEX = "(" + "|".join(re.escape(n) for n in NAMES) + ")"
wildcard_constraints:
    name = NAME_REGEX

# Helper: only depend on per-class outputs for IDs that are actually in IDS
def group_name_deps(wc):
    gid_list = GROUPS[wc.gname]["ids"]
    present = [i for i in gid_list if i in IDS]
    return [f"{OUTDIR}/{CLASSES[i]}.ply" for i in present]

rule panoramas:
    input:
        e57 = E57
    output:
        success = PANOS_DONE
    params:
        outdir = PANOS_DIR
    shell:
        "python py_scripts/get_images_and_poses.py "
        "{input.e57} --outfolder {params.outdir} "
        "&& python -c \"open(r'{output.success}','w').write('OK\\n')\""

# --------------------------
# PER-CLASS (single-ID -> name.ply)
# --------------------------
rule per_class:
    input:
        las = LAS
    output:
        ply = f"{OUTDIR}/{{name}}.ply"
    params:
        attribute = ATTRIBUTE,
        id = lambda wc: NAME2ID[wc.name]
    shell:
        "python py_scripts/las_to_ply.py "
        "--input {input.las} --output {output.ply} "
        "--attribute {params.attribute} --ids {params.id} --mode separate"

# --------------------------
# COMBINED GROUPS (named combos)
# --------------------------
rule combined_group:
    input:
        group_name_deps
    output:
        ply = f"{OUTDIR}/combined_{{gname}}.ply"
    params:
        attribute = ATTRIBUTE,
        ids = lambda wc: ",".join(str(i) for i in GROUPS[wc.gname]["ids"]),
        las = LAS
    shell:
        "python py_scripts/las_to_ply.py "
        "--input {params.las} --output {output.ply} "
        "--attribute {params.attribute} --ids {params.ids} --mode combined"

# --------------------------
# ROOM-SPLITTING on the chosen combined file
# --------------------------
checkpoint split_rooms_from_combined:
    input:
        combined = COMBINED_PLY,   # forces dependency
        indir    = ROOMS_INDIR
    output:
        success = f"{ROOMS_DIR}/_SUCCESS"
    params:
        voxel=VOX, grid=GRID, min_area=MIN_A,
        expand_dist=EXPAND_DIST,
        outdir=ROOMS_DIR
    shell:
        "python py_scripts/segment_rooms_mark.py "
        "--input_dir {input.indir} --output_dir {params.outdir} "
        "--voxel {params.voxel} "
        "--grid {params.grid} "
        "--min-room-area {params.min_area} --expand_dist {params.expand_dist} "
        "&& python -c \"open(r'{output.success}','w').write('OK\\n')\""



from pathlib import Path
from snakemake.io import glob_wildcards

def _discover_rooms_after_checkpoint():
    # force checkpoint resolution first
    _ = checkpoints.split_rooms_from_combined.get().output.success

    floors, rooms, _shells = glob_wildcards(
        f"{IN_ROOT}/floor_{{floor}}/room_{{room}}/shell_{{shell}}.ply"
    )
    pairs = sorted(set(zip(floors, rooms)))

    kept = []
    for f, r in pairs:
        room_path = Path(f"{IN_ROOT}/floor_{f}/room_{r}")
        if any(room_path.glob("shell_*.ply")):  # only keep rooms that have shells
            kept.append((f, r))

    floors = [p[0] for p in kept]
    rooms  = [p[1] for p in kept]
    return floors, rooms



def discovered_room_success_targets(wc):
    floors, rooms = _discover_rooms_after_checkpoint()
    return expand(f"{BVG_ROOT}/floor_{{floor}}/room_{{room}}/_BVG_SUCCESS",
                  floor=floors, room=rooms)

def discovered_obj_success_targets(wc):
    floors, rooms = _discover_rooms_after_checkpoint()
    return expand(f"{OBJ_ROOT}/floor_{{floor}}/room_{{room}}/_OBJ_SUCCESS",
                  floor=floors, room=rooms)

def discovered_csv_success_targets(wc):
    floors, rooms = _discover_rooms_after_checkpoint()
    return expand(f"{CSV_ROOT}/floor_{{floor}}/room_{{room}}/_CSV_SUCCESS",
                  floor=floors, room=rooms)

rule all:
    input:
        PANOS_DONE,
        expand(f"{OUTDIR}/{{name}}.ply", name=NAMES),
        expand(f"{OUTDIR}/combined_{{gname}}.ply", gname=list(GROUPS.keys())),
        COMBINED_PLY,
        f"{ROOMS_DIR}/_SUCCESS",
        discovered_room_success_targets,
        discovered_obj_success_targets,
        discovered_csv_success_targets,  
        "data/output/_PCG_DONE"
# --------------------------
# BVG per room (only shell_*.ply inside each room)
# Writes to a mirrored structure and drops a per-room _SUCCESS marker.
# --------------------------
rule bvg_per_room:
    # no input here
    output:
        success = BVG_ROOT + "/floor_{floor}/room_{room}/_BVG_SUCCESS"
    params:
        room_dir = lambda wc: f"{IN_ROOT}/floor_{wc.floor}/room_{wc.room}",
        out_dir  = BVG_ROOT + "/floor_{floor}/room_{room}",
        ply_glob = PLY_GLOB,
        min_support = MIN_SUPPORT,
        dist_threshold = DIST_THRESH,
        bitmap_resolution = BITMAP_RES,
        normal_threshold = NORMAL_THRESH,
        overlook_probability = OVERLOOK_PROB
    run:
        from pathlib import Path
        from snakemake.shell import shell

        room = Path(params.room_dir)
        out_succ = Path(output.success)
        out_succ.parent.mkdir(parents=True, exist_ok=True)

        shells = list(room.glob(params.ply_glob)) if room.exists() else []
        if not shells:
            out_succ.write_text("EMPTY\n")
        else:
            shell(
                "python py_scripts/ransac_and_plane_detection.py "
                "--in-dir {params.room_dir} "
                "--out-dir {params.out_dir} "
                "--glob {params.ply_glob} "
                "--min-support {params.min_support} "
                "--dist-threshold {params.dist_threshold} "
                "--bitmap-resolution {params.bitmap_resolution} "
                "--normal-threshold {params.normal_threshold} "
                "--overlook-probability {params.overlook_probability}"
            )
            out_succ.write_text("OK\n")

# --------------------------
# PolyFit per room (convert BVGs in that room to OBJ)
# --------------------------
rule obj_per_room:
    input:
        bvg_done = BVG_ROOT + "/floor_{floor}/room_{room}/_BVG_SUCCESS"
    output:
        success  = OBJ_ROOT + "/floor_{floor}/room_{room}/_OBJ_SUCCESS"
    params:
        in_dir  = BVG_ROOT + "/floor_{floor}/room_{room}",
        out_dir = OBJ_ROOT + "/floor_{floor}/room_{room}",
        solver  = SOLVER,
        w_data  = W_DATA,
        w_cover = W_COVER,
        w_complex = W_COMPLEX
    shell:
        "python py_scripts/polyfit_for_rooms.py "
        "--in-dir {params.in_dir} --out-dir {params.out_dir} "
        "--solver {params.solver} --w-data {params.w_data} "
        "--w-cover {params.w_cover} --w-complex {params.w_complex} "
        "&& python -c \"import pathlib; pathlib.Path(r'{output.success}').parent.mkdir(parents=True, exist_ok=True); pathlib.Path(r'{output.success}').write_text('OK\\n')\""

# --------------------------
# Generate a csv for each room containing the planes
# --------------------------
rule csvs_per_room:
    input:
        obj_done = OBJ_ROOT + "/floor_{floor}/room_{room}/_OBJ_SUCCESS"
    output:
        success  = CSV_ROOT + "/floor_{floor}/room_{room}/_CSV_SUCCESS"
    params:
        obj_dir   = OBJ_ROOT + "/floor_{floor}/room_{room}",
        out_dir   = CSV_ROOT + "/floor_{floor}/room_{room}",  # planes/floor_X/room_Y/
        angle     = ANGLE_DEG,
        wall_tol  = WALL_TOL,
        floor_tol = FLOOR_TOL,
        dist_eps  = DIST_EPS   # float or None
    run:
        import sys, subprocess, csv
        from pathlib import Path

        obj_dir = Path(params.obj_dir)
        out_dir = Path(params.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        # find the single OBJ (if any)
        objs = list(obj_dir.glob("*.obj"))
        if not objs:
            # No OBJ: write header-only CSV so downstream is happy
            header = ["group_name","class","face_count","area",
                      "centroid_x","centroid_y","centroid_z",
                      "normal_x","normal_y","normal_z",
                      "lowest_x","lowest_y","lowest_z",
                      "highest_x","highest_y","highest_z","obj_path"]
            with (out_dir / "planes_report.csv").open("w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(header)
            Path(output.success).parent.mkdir(parents=True, exist_ok=True)
            Path(output.success).write_text("OK\n")
            return

        obj = objs[0]

        script = Path("py_scripts/room_calculator.py")

        cmd = [
            sys.executable, str(script),
            "--obj", str(obj),
            "--outdir", str(out_dir),              # writes planes_report.csv + out_planes/
            "--angle", str(params.angle),
            "--wall_tol_deg", str(params.wall_tol),
            "--floor_tol_deg", str(params.floor_tol),
        ]
        if params.dist_eps is not None:
            cmd += ["--dist_eps", str(params.dist_eps)]

        subprocess.check_call(cmd)

        Path(output.success).parent.mkdir(parents=True, exist_ok=True)
        Path(output.success).write_text("OK\n")


rule pcg_room_final:
    # depend on all previous final markers
    input:
        discovered_csv_success_targets
    output:
        touch("data/output/_PCG_DONE")
    run:
        import platform, subprocess, shutil
        from pathlib import Path
        pcg_cfg = config.get("pcg", {})
        IN  = pcg_cfg.get("in",  "data/output")
        OUT = pcg_cfg.get("out", "data/output")
        exes = pcg_cfg.get("exe", {})
        system = platform.system()

        def run_cmd(cmd_list):
            subprocess.check_call(cmd_list)

        if system == "Windows":
            exe_win = exes.get("windows")
            if exe_win and Path(exe_win).exists():
                Path(OUT).mkdir(parents=True, exist_ok=True)
                run_cmd([exe_win, IN, OUT])
            elif shutil.which("wsl"):
                exe_wsl = exes.get("wsl", "LM2PCG/build/pcg_room")
                IN_WSL  = subprocess.check_output(["wsl", "wslpath", "-a", IN]).decode().strip()
                OUT_WSL = subprocess.check_output(["wsl", "wslpath", "-a", OUT]).decode().strip()
                EXE_WSL = subprocess.check_output(["wsl", "wslpath", "-a", exe_wsl]).decode().strip()
                run_cmd(["wsl", "bash", "-lc", f'mkdir -p "{OUT_WSL}" && "{EXE_WSL}" "{IN_WSL}" "{OUT_WSL}"'])
            else:
                raise RuntimeError("Windows detected, but neither a native pcg_room.exe nor WSL is available.")
        else:
            exe_posix = exes.get("linux" if system == "Linux" else "macos",
                                 "LM2PCG/build/pcg_room")
            Path(OUT).mkdir(parents=True, exist_ok=True)
            run_cmd([exe_posix, IN, OUT])
