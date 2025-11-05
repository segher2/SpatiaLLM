#!/usr/bin/env python3
"""
AI-facing API layer for local point-cloud pipeline control.

Provides two main capabilities:
1) Path resolution (by object_code, filename, or room code)
2) Operation dispatch via 3-letter head codes (e.g., RCN, VOL, VIS)

Available operations:
- RCN: Reconstruct mesh from cluster
- VOL: Compute mesh volume
- ARE: Compute mesh surface area
- CLR: Analyze dominant color
- BBD: Compute distance between two objects
- RMS: Parse room manifest and summarize
- VIS: Prepare and launch visualization (NEW)

Visualization (VIS) modes:
- room: Visualize entire room (shell + all clusters)
- clusters: Visualize selected clusters only
- multi-rooms: Visualize multiple room shells
- room-with-objects: Room shell with selected objects

Conventions assumed from the C++ pipeline:
- Object code format: <floor_id>-<room_id>-<object_id> (e.g., 0-7-12)
- Room code format: <floor_id>-<room_id> (e.g., 0-7)
- Filenames: <object_code>_<class>_{cluster|uobb|mesh[,_possion|_af]}.ply
- Room outputs: output/<site>/floor_<f>/room_<rrr>/
- CSV path: output/.../floor_<f>/room_<rrr>/<room_dirname>.csv
- Cluster PLYs: output/.../results/filtered_clusters/<stem>/<object_code>_<class>_cluster.ply
- UOBB PLYs:    output/.../results/filtered_clusters/<stem>/<object_code>_<class>_uobb.ply
- Recon meshes: output/.../results/recon/<stem>/<object_code>_<class>_mesh[_possion|_af].ply

This script can be imported as a module or used as a small CLI.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ------------------------------
# Helpers and parsing
# ------------------------------

OBJ_RE = re.compile(r"^(?P<floor>\d+)-(?:\s*)?(?P<room>\d+)-(?:\s*)?(?P<object>\d+)$")
ROOM_RE = re.compile(r"^(?P<floor>\d+)[-_](?P<room>\d+)$")


def parse_object_code(s: str) -> Tuple[int, int, int]:
    m = OBJ_RE.match(s.strip())
    if not m:
        raise ValueError(f"Invalid object_code '{s}'. Expected '<floor>-<room>-<object>' like '0-7-12'.")
    return int(m.group("floor")), int(m.group("room")), int(m.group("object"))


def repo_root(start: Optional[Path] = None) -> Path:
    """Find repository root by looking for CMakeLists.txt upwards."""
    p = Path(start or __file__).resolve()
    for parent in [p] + list(p.parents):
        if (parent / "CMakeLists.txt").exists():
            return parent
    # Fallback: current working directory
    return Path.cwd()


def build_dir(root: Path) -> Path:
    return root / "build"


def output_root(root: Path) -> Path:
    # Check for OUTPUT_DIR environment variable, default to "output2"
    # (Changed from "output" to "output2" to match current database)
    output_dir_name = os.getenv("OUTPUT_DIR", "output2")
    return root / output_dir_name


def is_room_dir(p: Path) -> bool:
    # room directory named like room_007; contains .csv and results usually
    return p.is_dir() and p.name.startswith("room_")


def room_identifiers(room_dir: Path) -> Tuple[Optional[int], Optional[int]]:
    """Extract floor_id, room_id from parent names like floor_0 and room_007."""
    try:
        floor_name = room_dir.parent.name
        room_name = room_dir.name
        def extract_int(s: str) -> Optional[int]:
            digits = ''.join(ch for ch in s if ch.isdigit())
            return int(digits) if digits else None
        return extract_int(floor_name), extract_int(room_name)
    except Exception:
        return None, None


def auto_detect_vis_mode(codes: List[str]) -> Tuple[str, List[str], List[str]]:
    """
    Auto-detect visualization mode based on input codes.
    Returns: (mode, room_codes, object_codes)
    
    Logic:
    - All codes match room pattern (X-Y) -> single room if 1, multi-rooms if >1
    - All codes match object pattern (X-Y-Z) -> clusters
    - Mix of both -> room-with-objects
    """
    room_codes = []
    object_codes = []
    
    for code in codes:
        code = code.strip()
        if OBJ_RE.match(code):
            object_codes.append(code)
        elif ROOM_RE.match(code):
            room_codes.append(code)
        else:
            raise ValueError(f"Invalid code '{code}'. Expected room code 'X-Y' or object code 'X-Y-Z'")
    
    # Determine mode
    if room_codes and object_codes:
        mode = "room-with-objects"
    elif object_codes and not room_codes:
        mode = "clusters"
    elif room_codes and not object_codes:
        mode = "room" if len(room_codes) == 1 else "multi-rooms"
    else:
        raise ValueError("No valid codes provided")
    
    return mode, room_codes, object_codes


def generate_vis_name(mode: str, room_codes: List[str], object_codes: List[str]) -> str:
    """
    Generate a visualization name based on mode and codes.
    
    Examples:
    - room: "room_0-7"
    - multi-rooms: "multi_rooms_0-7_0-8"
    - clusters: "clusters_0-7-12_0-7-15"
    - room-with-objects: "room_0-7_with_objects"
    """
    if mode == "room":
        return f"room_{room_codes[0].replace('-', '_')}"
    elif mode == "multi-rooms":
        # Limit to first few room codes to avoid overly long names
        preview = "_".join(rc.replace("-", "_") for rc in room_codes[:3])
        if len(room_codes) > 3:
            preview += f"_plus{len(room_codes) - 3}"
        return f"multi_rooms_{preview}"
    elif mode == "clusters":
        # Limit to first few object codes
        preview = "_".join(oc.replace("-", "_") for oc in object_codes[:3])
        if len(object_codes) > 3:
            preview += f"_plus{len(object_codes) - 3}"
        return f"clusters_{preview}"
    elif mode == "room-with-objects":
        # Use first room and count of objects
        room_preview = room_codes[0].replace("-", "_") if room_codes else "unknown"
        return f"room_{room_preview}_with_{len(object_codes)}_objects"
    else:
        return "visualization"


# ------------------------------
# Path index and resolvers
# ------------------------------

@dataclass
class ObjectAssets:
    object_code: str
    clusters: List[Path]
    uobbs: List[Path]
    meshes: List[Path]
    room_dir: Optional[Path]


class PathIndex:
    """Scan output tree once to build lookups by filename, object_code, and room code."""
    def __init__(self, out_root: Path):
        self.out_root = out_root
        self.by_filename: Dict[str, List[Path]] = {}
        self.csv_by_room: Dict[Tuple[int, int], List[Path]] = {}
        self.assets_by_object: Dict[str, ObjectAssets] = {}
        # Room-level shell exports
        self.shell_by_room: Dict[Tuple[int, int], Dict[str, List[Path]]] = {}

    def build(self) -> None:
        if not self.out_root.exists():
            return
        for p in self.out_root.rglob("*"):
            if p.is_file():
                # filename lookup
                self.by_filename.setdefault(p.name, []).append(p)

                # CSV mapping by room
                if p.suffix == ".csv" and is_room_dir(p.parent):
                    f_id, r_id = room_identifiers(p.parent)
                    if f_id is not None and r_id is not None:
                        self.csv_by_room.setdefault((f_id, r_id), []).append(p)

                # PLY asset mapping by object_code
                if p.suffix.lower() == ".ply":
                    stem = p.stem  # usually <object_code>_<class>_<kind>, but shell copy is <object_code>_shell
                    parts = stem.split("_")
                    if len(parts) >= 2:
                        object_code = parts[0]
                        # Allow mesh with method suffix: _mesh_possion or _mesh_af
                        last = parts[-1].lower()
                        second_last = parts[-2].lower() if len(parts) >= 2 else ""

                        # Map assets_by_object for standard 3+ part names
                        if len(parts) >= 3:
                            if last in ("cluster", "uobb", "mesh"):
                                kind = last
                            elif second_last == "mesh":
                                kind = "mesh"
                            else:
                                kind = last
                            assets = self.assets_by_object.get(object_code)
                            if not assets:
                                assets = ObjectAssets(object_code, [], [], [], None)
                                self.assets_by_object[object_code] = assets
                            if kind == "cluster":
                                assets.clusters.append(p)
                            elif kind == "uobb":
                                assets.uobbs.append(p)
                            elif kind == "mesh":
                                assets.meshes.append(p)
                            # infer room_dir for this asset
                            rd = self._infer_room_dir_from_asset(p)
                            if rd is not None:
                                assets.room_dir = rd

                        # Room-level shell and shell_uobb indexing (handle 2-part shell copy and 3-part shell uobb)
                        try:
                            f_id, r_id, _ = parse_object_code(object_code)
                        except Exception:
                            f_id = r_id = None  # type: ignore[assignment]
                        if f_id is not None and r_id is not None:
                            key = (f_id, r_id)
                            entry = self.shell_by_room.setdefault(key, {"shell": [], "uobb": []})
                            is_shell_copy = (len(parts) == 2 and last == "shell")
                            is_shell_uobb = (stem.endswith("_shell_uobb") or (len(parts) >= 3 and parts[-2].lower() == "shell" and last == "uobb"))
                            if is_shell_copy:
                                entry["shell"].append(p)
                            if is_shell_uobb:
                                entry["uobb"].append(p)

    @staticmethod
    def _infer_room_dir_from_asset(p: Path) -> Optional[Path]:
        """Walk up to find a 'room_XXX' directory."""
        cur = p.parent
        for _ in range(8):
            if cur is None:
                break
            if is_room_dir(cur):
                return cur
            cur = cur.parent if cur != cur.parent else None
        return None

    # Public resolvers
    def find_by_filename(self, name: str) -> List[Path]:
        return self.by_filename.get(name, [])

    def find_csv(self, floor_id: int, room_id: int) -> List[Path]:
        return self.csv_by_room.get((floor_id, room_id), [])

    def find_assets(self, object_code: str) -> Optional[ObjectAssets]:
        return self.assets_by_object.get(object_code)

    def find_room_shells(self, floor_id: int, room_id: int) -> Tuple[List[Path], List[Path]]:
        ent = self.shell_by_room.get((floor_id, room_id))
        if not ent:
            return [], []
        return ent.get("shell", []), ent.get("uobb", [])


# ------------------------------
# Dispatcher for head codes
# ------------------------------

class Dispatcher:
    def __init__(self, root: Optional[Path] = None):
        self.root = repo_root(root)
        self.out_root = output_root(self.root)
        self.bin_dir = build_dir(self.root)
        self.index = PathIndex(self.out_root)
        self.index.build()

    def env_status(self) -> Dict[str, object]:
        """Report availability of required executables and basic paths."""
        bins = {
            "pcg_reconstruct": (self.bin_dir / "pcg_reconstruct").exists(),
            "pcg_volume": (self.bin_dir / "pcg_volume").exists(),
            "pcg_area": (self.bin_dir / "pcg_area").exists(),
            "pcg_bbox": (self.bin_dir / "pcg_bbox").exists(),
            "pcg_room": (self.bin_dir / "pcg_room").exists(),
            "pcg_color": (self.bin_dir / "pcg_color").exists(),
        }
        return {
            "repo_root": str(self.root),
            "build_dir": str(self.bin_dir),
            "output_root": str(self.out_root),
            "executables": bins,
        }

    # --- Head code: RCN (reconstruction) ---
    def op_RCN(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               only_substring: Optional[str] = None) -> Path:
        """Reconstruct mesh for a cluster. Input: object_code or filename.
        Returns the expected mesh path (created or updated).
        """
        if (object_code is None) == (filename is None):
            raise ValueError("RCN requires exactly one of object_code or filename.")

        if object_code:
            assets = self.index.find_assets(object_code)
            if not assets or not assets.clusters:
                raise FileNotFoundError(f"No cluster PLY found for object_code '{object_code}'.")
            # If multiple clusters found (rare), try narrowing by substring or pick the first
            cluster_path = self._choose_one(assets.clusters, only_substring)
            room_dir = assets.room_dir or self.index._infer_room_dir_from_asset(cluster_path)
        else:
            # accept direct path
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                cluster_path = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                cluster_path = self._choose_one(matches, only_substring)
            # Validate looks like a cluster file
            if not cluster_path.stem.endswith("_cluster"):
                raise ValueError(f"RCN expects a cluster PLY, got '{cluster_path.name}'.")
            room_dir = self.index._infer_room_dir_from_asset(cluster_path)

        if room_dir is None:
            raise RuntimeError(f"Cannot infer room directory for '{cluster_path}'.")

        # Call pcg_reconstruct <cluster_ply> <room_dir>
        exe = self.bin_dir / "pcg_reconstruct"
        if not exe.exists():
            raise FileNotFoundError(
                f"pcg_reconstruct not found at {exe}. Please build the project first:\n"
                "  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j"
            )
        self._run([str(exe), str(cluster_path), str(room_dir)])

        # Find the generated mesh path after reconstruction. New naming includes method suffix.
        oc, klass, kind = self._parse_asset_name(cluster_path)
        # <stem> is the immediate directory under filtered_clusters (e.g., couch_007)
        stem_dir = cluster_path.parent.name
        mesh_dir = room_dir / "results" / "recon" / stem_dir
        prefix = f"{oc}_{klass}_mesh"
        candidates = sorted([p for p in mesh_dir.glob(prefix + "*.ply")])
        if not candidates:
            # Fallback to legacy name without suffix
            legacy = mesh_dir / f"{prefix}.ply"
            return legacy
        return candidates[0]

    # --- Head code: VOL (mesh volume) ---
    def op_VOL(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               auto_reconstruct: bool = True) -> Tuple[Path, float, bool]:
        """Compute volume (and closedness) for a reconstructed mesh.
        Input: object_code or mesh filename. If mesh missing and auto_reconstruct, try RCN.
        Returns: (mesh_path, volume, is_closed)
        """
        if (object_code is None) == (filename is None):
            raise ValueError("VOL requires exactly one of object_code or filename.")

        mesh_path: Optional[Path] = None
        if object_code:
            assets = self.index.find_assets(object_code)
            if assets and assets.meshes:
                mesh_path = self._choose_one(assets.meshes)
            elif auto_reconstruct:
                # Try reconstruct from cluster first
                mesh_path = self.op_RCN(object_code=object_code)
            else:
                raise FileNotFoundError(f"No mesh found for object_code '{object_code}'.")
        else:
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                candidate = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                candidate = self._choose_one(matches)
            if self._is_mesh_stem(candidate.stem):
                mesh_path = candidate
            elif auto_reconstruct and candidate.stem.endswith("_cluster"):
                # it's a cluster, reconstruct first
                mesh_path = self.op_RCN(filename=candidate.name)
            else:
                raise ValueError("VOL expects a mesh PLY, or a cluster with auto_reconstruct=True.")

        if mesh_path is None:
            raise RuntimeError("Internal error: mesh_path is None after resolution.")

        exe = self.bin_dir / "pcg_volume"
        if not exe.exists():
            raise FileNotFoundError(
                f"pcg_volume not found at {exe}. Please build the project first (CGAL required):\n"
                "  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j"
            )
        # Run and parse output
        out = self._run([str(exe), str(mesh_path)])
        # Try JSON first
        j = self._try_parse_json(out)
        if j and "closed" in j:
            vol_val = j.get("volume", None)
            vol_f = float(vol_val) if isinstance(vol_val, (int, float)) else 0.0
            return mesh_path, vol_f, bool(j["closed"])
        # Fallback to legacy text parsing
        is_closed = "closed: true" in out
        vol = None
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("volume:"):
                try:
                    vol = float(line.split(":", 1)[1].strip())
                except Exception:
                    pass
        if vol is None:
            raise RuntimeError("Failed to parse volume from pcg_volume output.")
        return mesh_path, vol, is_closed

    # --- Head code: CLR (dominant color analysis) ---
    def op_CLR(self, object_code: Optional[str] = None, filename: Optional[str] = None,
               auto_pick_cluster: bool = True) -> Dict[str, object]:
        """Run color analysis using pcg_color.
        Input: object_code (prefers cluster), or explicit filename (any PLY with RGB).
        Returns a dict with parsed output if possible; otherwise raw text.
        """
        if (object_code is None) == (filename is None):
            raise ValueError("CLR requires exactly one of object_code or filename.")

        target: Optional[Path] = None
        if object_code:
            assets = self.index.find_assets(object_code)
            if not assets:
                raise FileNotFoundError(f"No assets found for object_code '{object_code}'.")
            # Prefer cluster if available, else fall back to any mesh/ply
            if auto_pick_cluster and assets.clusters:
                target = self._choose_one(assets.clusters)
            elif assets.meshes:
                target = self._choose_one(assets.meshes)
            elif assets.uobbs:
                target = self._choose_one(assets.uobbs)
            else:
                raise FileNotFoundError(f"No usable PLY for color analysis under '{object_code}'.")
        else:
            # allow direct file path or lookup by name
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                target = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                target = self._choose_one(matches)
        # Check if executable exists
        exe = self.bin_dir / "pcg_color"
        if not exe.exists():
            raise FileNotFoundError(
                f"pcg_color not found at {exe}. Please build the project first:\n"
                "  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j"
            )
        out = self._run([str(exe), str(target)])

        # Prefer JSON
        j = self._try_parse_json(out)
        if j:
            return j

        # Fallback: light parse
        result: Dict[str, object] = {"file": str(target), "raw": out}
        try:
            lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
            m_val: Optional[int] = None
            comps: List[Dict[str, object]] = []
            for ln in lines:
                if ln.lower().startswith("final m="):
                    try:
                        m_val = int(ln.split("=", 1)[1].strip())
                    except Exception:
                        pass
                if ln.lower().startswith("component"):
                    comps.append({"line": ln})
            if m_val is not None:
                result["M"] = m_val
            if comps:
                result["components"] = comps
        except Exception:
            pass
        return result

    # --- Head code: ARE (mesh surface area) ---
    def op_ARE(self, object_code: Optional[str] = None, filename: Optional[str] = None,
                auto_reconstruct: bool = True) -> Tuple[Path, float, bool]:
        """Compute surface area (and closedness) for a reconstructed mesh (.ply with '_mesh').
        Input: object_code or mesh filename. If mesh missing and auto_reconstruct, try RCN.
        Returns: (mesh_path, area, is_closed)
        """
        if (object_code is None) == (filename is None):
            raise ValueError("ARE requires exactly one of object_code or filename.")

        mesh_path: Optional[Path] = None
        if object_code:
            assets = self.index.find_assets(object_code)
            if assets and assets.meshes:
                mesh_path = self._choose_one(assets.meshes)
            elif auto_reconstruct:
                mesh_path = self.op_RCN(object_code=object_code)
            else:
                raise FileNotFoundError(f"No mesh found for object_code '{object_code}'.")
        else:
            fp = Path(filename)  # type: ignore[arg-type]
            if fp.exists():
                candidate = fp
            else:
                matches = self.index.find_by_filename(filename)  # type: ignore[arg-type]
                matches = [p for p in matches if p.suffix == ".ply"]
                if not matches:
                    raise FileNotFoundError(f"No file named '{filename}' under '{self.out_root}'.")
                candidate = self._choose_one(matches)
            if self._is_mesh_stem(candidate.stem):
                mesh_path = candidate
            elif auto_reconstruct and candidate.stem.endswith("_cluster"):
                mesh_path = self.op_RCN(filename=candidate.name)
            else:
                raise ValueError("ARE expects a mesh PLY, or a cluster with auto_reconstruct=True.")

        exe = self.bin_dir / "pcg_area"
        if not exe.exists():
            raise FileNotFoundError(
                f"pcg_area not found at {exe}. Please build the project first (CGAL required):\n"
                "  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j"
            )
        out = self._run([str(exe), str(mesh_path)])
        j = self._try_parse_json(out)
        if j and "area" in j:
            return mesh_path, float(j.get("area", 0.0)), bool(j.get("closed", False))
        # Fallback text parsing
        is_closed = "closed: true" in out
        area_val = None
        for line in out.splitlines():
            s = line.strip()
            if s.startswith("area:"):
                try:
                    area_val = float(s.split(":", 1)[1].strip())
                except Exception:
                    pass
        if area_val is None:
            raise RuntimeError("Failed to parse area from pcg_area output.")
        return mesh_path, float(area_val), is_closed

    # --- Example two-object op: BBD (bbox distance) ---
    def op_BBD(self, object_code_1: str, object_code_2: str) -> Tuple[float, Tuple[float, float, float]]:
        """Compute distance and vector between two UOBB centers (example two-object op)."""
        assets1 = self.index.find_assets(object_code_1)
        assets2 = self.index.find_assets(object_code_2)
        if not assets1 or not assets1.uobbs:
            raise FileNotFoundError(f"No UOBB for object_code '{object_code_1}'.")
        if not assets2 or not assets2.uobbs:
            raise FileNotFoundError(f"No UOBB for object_code '{object_code_2}'.")
        u1 = self._choose_one(assets1.uobbs)
        u2 = self._choose_one(assets2.uobbs)
        exe = self.bin_dir / "pcg_bbox"
        if not exe.exists():
            raise FileNotFoundError(
                f"pcg_bbox not found at {exe}. Please build the project first:\n"
                "  cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . -j"
            )
        out = self._run([str(exe), str(u1), str(u2)])
        # Try JSON
        j = self._try_parse_json(out)
        if j and "distance" in j and "vector_1_to_2" in j:
            v = j["vector_1_to_2"]
            vec = (float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0)))
            return float(j["distance"]), vec
        # Fallback to legacy text
        vec = (0.0, 0.0, 0.0)
        dist = None
        for line in out.splitlines():
            s = line.strip()
            if s.startswith("vector_1_to_2:"):
                nums = s.split(":", 1)[1].split(",")
                if len(nums) == 3:
                    vec = tuple(float(x) for x in map(str.strip, nums))  # type: ignore[assignment]
            if s.startswith("distance:"):
                try:
                    dist = float(s.split(":", 1)[1].strip())
                except Exception:
                    pass
        if dist is None:
            raise RuntimeError("Failed to parse output from pcg_bbox.")
        return dist, vec  # type: ignore[return-value]

    # --- Head code: VIS (visualization) ---
    def op_VIS(self, mode: str, name: str,
               room_codes: Optional[List[str]] = None,
               object_codes: Optional[List[str]] = None,
               ratio: Optional[float] = None,
               ratio_shell: Optional[float] = None,
               voxel: Optional[float] = None,
               shell_no_color: Optional[bool] = None,
               clean: bool = True,
               clean_all: bool = False,
               auto_serve: bool = True,
               port: int = 5173) -> Dict[str, object]:
        """Prepare and optionally launch visualization for rooms/objects.
        
        Modes:
        - 'room': Visualize entire room (shell + all clusters)
          Required: room_codes (single room code like '0-7')
        - 'clusters': Visualize selected clusters only
          Required: object_codes (list of object codes like ['0-7-12', '0-7-15'])
        - 'multi-rooms': Visualize multiple room shells
          Required: room_codes (list of room codes like ['0-7', '0-6'])
        - 'room-with-objects': Room shell with selected objects
          Required: room_codes (single), object_codes (list)
        
        Args:
            mode: Visualization mode (room, clusters, multi-rooms, room-with-objects)
            name: Output name for the visualization (used for data folder and manifest)
            room_codes: List of room codes in format '<floor>-<room>' (e.g., ['0-7'])
            object_codes: List of object codes in format '<floor>-<room>-<object>' (e.g., ['0-7-12'])
            ratio: Downsample ratio for clusters (None = use config default)
            ratio_shell: Downsample ratio for shell (None = use config default)
            voxel: Optional voxel size for spatial sampling (None = use config default)
            shell_no_color: Strip color from shell (None = use config default)
            clean: Clean previous outputs before preparation
            clean_all: Clean ALL previous outputs (not just same name)
            auto_serve: Automatically start dev server after preparation
            port: Dev server port (default 5173)
        
        Returns:
            Dict with status, viewer_url, and details about prepared files
        """
        # Load defaults from config file if parameters not specified
        # Use simple key-value parsing similar to C++ params.cpp
        config_path = self.root / "data" / "configs" / "default.yaml"
        config_defaults = {
            'ratio': 0.2,
            'ratio_shell': 0.05,
            'voxel': None,
            'shell_no_color': False,
        }
        if config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        # Parse key: value or key = value
                        sep_pos = line.find(':')
                        if sep_pos == -1:
                            sep_pos = line.find('=')
                        if sep_pos == -1:
                            continue
                        key = line[:sep_pos].strip()
                        value = line[sep_pos + 1:].strip()
                        # Remove inline comments
                        comment_pos = value.find('#')
                        if comment_pos != -1:
                            value = value[:comment_pos].strip()
                        
                        # Parse viewer parameters
                        if key == 'viewer_downsample_ratio':
                            try:
                                config_defaults['ratio'] = float(value)
                            except ValueError:
                                pass
                        elif key == 'viewer_downsample_ratio_shell':
                            try:
                                config_defaults['ratio_shell'] = float(value)
                            except ValueError:
                                pass
                        elif key == 'viewer_voxel_size':
                            if value.lower() not in ('null', 'none', ''):
                                try:
                                    config_defaults['voxel'] = float(value)
                                except ValueError:
                                    pass
                        elif key == 'viewer_shell_no_color':
                            v = value.lower()
                            config_defaults['shell_no_color'] = v in ('1', 'true', 'yes')
            except Exception:
                # If config parsing fails, use hardcoded defaults
                pass
        
        # Apply defaults from config if not specified
        if ratio is None:
            ratio = config_defaults['ratio']
        if ratio_shell is None:
            ratio_shell = config_defaults['ratio_shell']
        if voxel is None:
            voxel = config_defaults['voxel']
        if shell_no_color is None:
            shell_no_color = config_defaults['shell_no_color']
        
        # Validate mode
        valid_modes = ['room', 'clusters', 'multi-rooms', 'room-with-objects']
        if mode not in valid_modes:
            raise ValueError(f"Invalid mode '{mode}'. Must be one of {valid_modes}.")
        
        # Validate required parameters per mode
        if mode == 'room':
            if not room_codes or len(room_codes) != 1:
                raise ValueError("Mode 'room' requires exactly one room_code.")
        elif mode == 'clusters':
            if not object_codes or len(object_codes) == 0:
                raise ValueError("Mode 'clusters' requires at least one object_code.")
        elif mode == 'multi-rooms':
            if not room_codes or len(room_codes) == 0:
                raise ValueError("Mode 'multi-rooms' requires at least one room_code.")
        elif mode == 'room-with-objects':
            if not room_codes or len(room_codes) != 1:
                raise ValueError("Mode 'room-with-objects' requires exactly one room_code.")
            if not object_codes or len(object_codes) == 0:
                raise ValueError("Mode 'room-with-objects' requires at least one object_code.")
        
        # Find viewer directory
        viewer_dir = self.root / "web" / "pointcloud-viewer"
        if not viewer_dir.exists():
            raise FileNotFoundError(f"Viewer directory not found at {viewer_dir}.")
        
        script_path = viewer_dir / "scripts" / "prepare_visualization.mjs"
        if not script_path.exists():
            raise FileNotFoundError(f"Visualization script not found at {script_path}.")
        
        # Build command
        cmd = ["node", str(script_path), "--mode", mode, "--name", name]
        
        # Add room codes
        if room_codes:
            if mode in ['room', 'room-with-objects']:
                cmd.extend(["--room", room_codes[0]])
            elif mode == 'multi-rooms':
                cmd.extend(["--rooms", ",".join(room_codes)])
        
        # Add object codes
        if object_codes:
            cmd.extend(["--objects", ",".join(object_codes)])
        
        # Add optional parameters
        cmd.extend(["--ratio", str(ratio)])
        cmd.extend(["--ratioShell", str(ratio_shell)])
        if voxel is not None:
            cmd.extend(["--voxel", str(voxel)])
        if shell_no_color:
            cmd.append("--shellNoColor")
        if not clean:
            cmd.append("--no-clean")
        if clean_all:
            cmd.append("--clean-all")
        # Don't pass --serve to prepare_visualization.mjs
        # We'll start the server separately using start_dev.sh
        
        # Execute visualization preparation (data processing only)
        try:
            cwd_original = Path.cwd()
            import os
            os.chdir(str(viewer_dir))
            
            # Run data preparation synchronously
            proc = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            out = proc.stdout
            
            # If auto_serve is True, start the dev server independently
            if auto_serve:
                start_script = viewer_dir / "start_dev.sh"
                if not start_script.exists():
                    raise RuntimeError(f"start_dev.sh not found at {start_script}")
                
                # Execute start_dev.sh in background using nohup to make it truly independent
                subprocess.Popen(
                    ['bash', str(start_script)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                    cwd=str(viewer_dir)
                )
                
                out += f"\n🚀 Development servers starting in background...\n"
                out += f"   Frontend: http://localhost:{port}\n"
                out += f"   Backend API: http://localhost:8090\n"
                out += f"   Logs: /tmp/vite_server.log, /tmp/api_server.log\n"
                out += f"   To stop: bash {viewer_dir}/stop_dev.sh\n"
            
            os.chdir(str(cwd_original))
            
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Visualization preparation failed:\n{e.stdout}")
        except Exception as e:
            raise RuntimeError(f"Visualization preparation error: {str(e)}")
        
        # Parse output for URL
        viewer_url = f"http://localhost:{port}/?manifest={name}.json"
        
        # Build result
        result: Dict[str, object] = {
            "status": "success",
            "mode": mode,
            "name": name,
            "viewer_url": viewer_url,
            "output": out,
        }
        
        if room_codes:
            result["room_codes"] = room_codes
        if object_codes:
            result["object_codes"] = object_codes
        
        return result

    # --- Head code: RMS (Room Manifest Summary) ---
    def op_RMS(self, site_name: Optional[str] = None, visualize: bool = False, 
               vis_name: Optional[str] = None, auto_serve: bool = True, 
               clean_all: bool = True) -> Dict[str, object]:
        """Parse all rooms_manifest.csv files in subdirectories and return summary of floors and rooms.
        Input: site_name (optional; if not provided, auto-detect from /output directory)
               visualize: if True, also prepare visualization of all rooms (multi-rooms mode)
               vis_name: output name for visualization (default: 'all_rooms')
               auto_serve: if True, auto-start dev server after visualization (default: True)
               clean_all: if True, clean all previous outputs before visualization (default: True)
        Returns: dict with total_floors, total_rooms, list of room_codes, and optional viewer_url
        """
        # Auto-detect site if not provided
        if site_name is None:
            if not self.out_root.exists():
                raise FileNotFoundError(f"Output directory '{self.out_root}' does not exist.")
            
            # Check if rooms_manifest.csv exists directly under floor_X directories in output root
            manifest_files = []
            for item in self.out_root.iterdir():
                if item.is_dir() and item.name.startswith("floor_"):
                    manifest_path = item / "rooms_manifest.csv"
                    if manifest_path.exists():
                        manifest_files.append(manifest_path)
            
            # If found manifests directly under floor_X, use output root as site_dir
            if manifest_files:
                site_dir = self.out_root
                site_name = "output"
            else:
                # Otherwise, look for the old structure: output/site_name/floor_X
                candidates = []
                for item in self.out_root.iterdir():
                    if item.is_dir():
                        # Check if any subdirectory contains rooms_manifest.csv
                        has_manifest = False
                        for subitem in item.iterdir():
                            if subitem.is_dir() and (subitem / "rooms_manifest.csv").exists():
                                has_manifest = True
                                break
                        if has_manifest:
                            candidates.append(item)
                if not candidates:
                    raise FileNotFoundError(f"No rooms_manifest.csv found in any subdirectory structure under '{self.out_root}'.")
                site_dir = candidates[0]
                site_name = site_dir.name
        else:
            site_dir = self.out_root / site_name
        
        if not site_dir.exists():
            raise FileNotFoundError(f"Site directory '{site_dir}' does not exist.")
        
        # Find all rooms_manifest.csv files in subdirectories
        manifest_files = []
        for subdir in site_dir.iterdir():
            if subdir.is_dir():
                manifest_path = subdir / "rooms_manifest.csv"
                if manifest_path.exists():
                    manifest_files.append(manifest_path)
        
        if not manifest_files:
            raise FileNotFoundError(f"No rooms_manifest.csv found in any subdirectory of '{site_dir}'.")
        
        import csv
        rooms_info = []
        floors_set = set()
        
        # Read and combine all manifest files
        for manifest_path in manifest_files:
            with open(manifest_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        # Strip whitespace from keys and values
                        row = {k.strip(): v.strip() for k, v in row.items()}
                        room_id = int(row['room_id'])
                        floor_id = int(row['floor_id'])
                        floors_set.add(floor_id)
                        # Room code format: <floor_id>-<room_id> (e.g., '0-7')
                        room_code = f"{floor_id}-{room_id}"
                        rooms_info.append({
                            "floor_id": floor_id,
                            "room_id": room_id,
                            "room_code": room_code,
                            "room_type": row.get('room_type', 'unknown'),
                            "source_manifest": str(manifest_path)
                        })
                    except (ValueError, KeyError) as e:
                        # Skip malformed rows
                        continue
        
        # Sort room codes for consistent output
        rooms_info.sort(key=lambda x: (x['floor_id'], x['room_id']))
        room_codes = [r['room_code'] for r in rooms_info]
        
        result = {
            "site_name": site_name,
            "total_floors": len(floors_set),
            "total_rooms": len(rooms_info),
            "room_codes": room_codes,
            "rooms": rooms_info,
            "manifest_files": [str(p) for p in manifest_files]
        }
        
        # Optional: visualize all rooms
        if visualize and room_codes:
            if vis_name is None:
                vis_name = f"all_rooms_{site_name}"
            
            vis_result = self.op_VIS(
                mode="multi-rooms",
                name=vis_name,
                room_codes=room_codes,
                ratio_shell=0.01,  # Force low shell downsample ratio (1%) for RMS multi-rooms performance
                clean_all=clean_all,
                auto_serve=auto_serve
            )
            result["visualization"] = {
                "status": vis_result["status"],
                "viewer_url": vis_result["viewer_url"],
                "name": vis_result["name"]
            }
        
        return result

    # ------------------------------
    # Utilities
    # ------------------------------
    @staticmethod
    def _choose_one(paths: List[Path], only_substring: Optional[str] = None) -> Path:
        if not paths:
            raise FileNotFoundError("No candidates to choose from.")
        if only_substring:
            sub = [p for p in paths if only_substring in p.name]
            if sub:
                paths = sub
        if len(paths) > 1:
            # Prefer paths under filtered_clusters for clusters, recon for meshes
            def score(p: Path) -> int:
                s = 0
                parts = [str(x) for x in p.parts]
                if "filtered_clusters" in parts:
                    s += 2
                if "recon" in parts:
                    s += 1
                return -s
            paths = sorted(paths, key=score)
        return paths[0]

    @staticmethod
    def _parse_asset_name(p: Path) -> Tuple[str, str, str]:
        # returns (object_code, class, kind)
        parts = p.stem.split("_")
        if len(parts) < 3:
            raise ValueError(f"Unexpected asset name format: {p.name}")
        # If mesh has method suffix (_mesh_possion/_mesh_af), coerce kind to 'mesh'
        kind = parts[-1]
        if len(parts) >= 2 and parts[-2] == "mesh":
            kind = "mesh"
        return parts[0], parts[-2] if len(parts) >= 2 else "", kind

    @staticmethod
    def _is_mesh_stem(stem: str) -> bool:
        return stem.endswith("_mesh") or stem.endswith("_mesh_possion") or stem.endswith("_mesh_af")

    @staticmethod
    def _run(cmd: List[str]) -> str:
        try:
            proc = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            return proc.stdout
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Command failed ({e.returncode}): {' '.join(cmd)}\nOutput:\n{e.stdout}")

    @staticmethod
    def _try_parse_json(text: str) -> Optional[Dict[str, object]]:
        s = text.strip()
        if not s:
            return None
        # Fast path
        if s.startswith("{") and s.endswith("}"):
            try:
                return json.loads(s)
            except Exception:
                pass
        # Try to find the last JSON object in the text
        last_open = s.rfind("{")
        last_close = s.rfind("}")
        if last_open != -1 and last_close != -1 and last_close > last_open:
            frag = s[last_open:last_close+1]
            try:
                return json.loads(frag)
            except Exception:
                return None
        return None


# ------------------------------
# Minimal CLI for agents
# ------------------------------

def _cli() -> int:
    parser = argparse.ArgumentParser(description="AI API for local PCG pipeline")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # Load json_output setting from config
    config_path = repo_root() / "data" / "configs" / "default.yaml"
    default_json_output = True  # fallback
    if config_path.exists():
        try:
            with open(config_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    if ':' in line or '=' in line:
                        sep_pos = line.find(':')
                        if sep_pos == -1:
                            sep_pos = line.find('=')
                        key = line[:sep_pos].strip()
                        value = line[sep_pos + 1:].strip()
                        comment_pos = value.find('#')
                        if comment_pos != -1:
                            value = value[:comment_pos].strip()
                        if key == 'json_output':
                            v = value.lower()
                            default_json_output = v in ('1', 'true', 'yes')
                            break
        except Exception:
            pass

    # Resolve by filename
    p_res_fn = sub.add_parser("resolve-filename", help="Find file by name under output/")
    p_res_fn.add_argument("name")

    # Resolve by room
    p_res_room = sub.add_parser("resolve-room-csv", help="Find CSV for floor-room")
    p_res_room.add_argument("floor", type=int)
    p_res_room.add_argument("room", type=int)

    # Resolve by room code (e.g., 0-7)
    p_res_room2 = sub.add_parser("resolve-room", help="Find CSV and shell/bbox for room code like 0-7")
    p_res_room2.add_argument("room_code")

    # Resolve by object_code
    p_res_obj = sub.add_parser("resolve-object", help="Find assets for object_code")
    p_res_obj.add_argument("object_code")

    # RCN - Unified format: RCN <object_code>
    p_rcn = sub.add_parser("RCN", help="Reconstruct mesh from object code")
    p_rcn.add_argument("object_code", help="Object code (e.g., 0-7-12)")

    # VOL - Unified format: VOL <object_code>
    p_vol = sub.add_parser("VOL", help="Compute mesh volume from object code")
    p_vol.add_argument("object_code", help="Object code (e.g., 0-7-12)")
    p_vol.add_argument("--no-auto-recon", action="store_true", help="Skip auto-reconstruction if mesh missing")

    # ARE - Unified format: ARE <object_code>
    p_area = sub.add_parser("ARE", help="Compute mesh surface area from object code")
    p_area.add_argument("object_code", help="Object code (e.g., 0-7-12)")
    p_area.add_argument("--no-auto-recon", action="store_true", help="Skip auto-reconstruction if mesh missing")

    # CLR - Unified format: CLR <object_code>
    # CLR - Unified format: CLR <object_code>
    p_clr = sub.add_parser("CLR", help="Analyze dominant color from object code")
    p_clr.add_argument("object_code", help="Object code (e.g., 0-7-12)")

    # BBD - Unified format: BBD <object_code_1> <object_code_2>
    p_bbd = sub.add_parser("BBD", help="Compute distance between two objects")
    p_bbd.add_argument("object_code_1", help="First object code (e.g., 0-7-12)")
    p_bbd.add_argument("object_code_2", help="Second object code (e.g., 0-7-15)")

    # VIS - Unified format: VIS <codes...>
    p_vis = sub.add_parser("VIS", help="Prepare and launch visualization for rooms/objects")
    p_vis.add_argument("codes", nargs="+", help="Room codes (e.g., '0-7') and/or object codes (e.g., '0-7-12')")
    p_vis.add_argument("--mode", 
                       choices=["room", "clusters", "multi-rooms", "room-with-objects"],
                       help="Visualization mode (auto-detected if omitted)")
    p_vis.add_argument("--name", help="Output name for visualization (auto-generated if omitted)")
    p_vis.add_argument("--ratio", type=float, help="Downsample ratio for clusters (default: from config)")
    p_vis.add_argument("--ratio-shell", type=float, help="Downsample ratio for shell (default: from config)")
    p_vis.add_argument("--voxel", type=float, help="Voxel size for spatial sampling (default: from config)")
    p_vis.add_argument("--shell-no-color", action="store_true", default=None, help="Strip color from shell (default: from config)")
    p_vis.add_argument("--no-clean", action="store_true", help="Skip cleaning previous outputs (default: clean)")
    p_vis.add_argument("--no-clean-all", action="store_true", help="Do NOT clean all previous outputs (default: clean all)")
    p_vis.add_argument("--no-serve", action="store_true", help="Do NOT auto-start dev server (default: auto-start)")
    p_vis.add_argument("--no-wait", action="store_true", help="Do NOT wait for user selection (non-interactive mode)")
    p_vis.add_argument("--port", type=int, default=5173, help="Dev server port (default: 5173)")

    # RMS - Unified format: RMS [site_name]
    p_rms = sub.add_parser("RMS", help="Parse rooms_manifest.csv and summarize floors/rooms")
    p_rms.add_argument("site_name", nargs="?", default=None, help="Site name (optional; auto-detected if omitted)")
    p_rms.add_argument("--visualize", action="store_true", help="Visualize all rooms in multi-rooms mode")
    p_rms.add_argument("--vis-name", help="Visualization output name (default: 'all_rooms_<site>')")
    p_rms.add_argument("--no-serve", action="store_true", help="Do NOT auto-start dev server (default: auto-start)")
    p_rms.add_argument("--no-clean-all", action="store_true", help="Do NOT clean all previous outputs (default: clean all)")

    # check environment
    p_chk = sub.add_parser("check-env", help="Show availability of required executables")

    args = parser.parse_args()
    d = Dispatcher()
    
    # Use json_output from config as default
    use_json = default_json_output

    args = parser.parse_args()
    d = Dispatcher()
    
    # Use json_output from config as default
    use_json = default_json_output

    if args.cmd == "resolve-filename":
        paths = [str(p) for p in d.index.find_by_filename(args.name)]
        if use_json:
            print(json.dumps({"matches": paths}, ensure_ascii=False))
        else:
            for p in paths:
                print(p)
        return 0 if paths else 1
    if args.cmd == "resolve-room-csv":
        csvs = [str(p) for p in d.index.find_csv(args.floor, args.room)]
        shells, uobbs = d.index.find_room_shells(args.floor, args.room)
        out = {
            "floor": args.floor,
            "room": args.room,
            "csv": csvs,
            "shell": [str(p) for p in shells],
            "shell_uobb": [str(p) for p in uobbs],
        }
        if use_json:
            print(json.dumps(out, ensure_ascii=False))
        else:
            for p in out["csv"]:
                print(p)
            if out["shell"]:
                print("# shell:")
                for p in out["shell"]:
                    print(p)
            if out["shell_uobb"]:
                print("# shell_uobb:")
                for p in out["shell_uobb"]:
                    print(p)
        return 0 if (csvs or shells or uobbs) else 1
    if args.cmd == "resolve-room":
        code = args.room_code.strip()
        m = ROOM_RE.match(code)
        if not m:
            print("Invalid room code. Expected '<floor>-<room>' like '0-7'.")
            return 2
        f_id = int(m.group("floor"))
        r_id = int(m.group("room"))
        csvs = [str(p) for p in d.index.find_csv(f_id, r_id)]
        shells, uobbs = d.index.find_room_shells(f_id, r_id)
        out = {
            "floor": f_id,
            "room": r_id,
            "csv": csvs,
            "shell": [str(p) for p in shells],
            "shell_uobb": [str(p) for p in uobbs],
        }
        if use_json:
            print(json.dumps(out, ensure_ascii=False))
        else:
            for p in out["csv"]:
                print(p)
            if out["shell"]:
                print("# shell:")
                for p in out["shell"]:
                    print(p)
            if out["shell_uobb"]:
                print("# shell_uobb:")
                for p in out["shell_uobb"]:
                    print(p)
        return 0 if (csvs or shells or uobbs) else 1
    if args.cmd == "resolve-object":
        assets = d.index.find_assets(args.object_code)
        if not assets:
            if use_json:
                print(json.dumps({"object_code": args.object_code, "found": False}, ensure_ascii=False))
            else:
                print("<none>")
            return 1
        if use_json:
            print(json.dumps({
                "object_code": assets.object_code,
                "clusters": [str(p) for p in assets.clusters],
                "uobbs": [str(p) for p in assets.uobbs],
                "meshes": [str(p) for p in assets.meshes],
                "room_dir": str(assets.room_dir) if assets.room_dir else None
            }, ensure_ascii=False))
        else:
            print("# clusters:")
            for p in assets.clusters:
                print(p)
            print("# uobbs:")
            for p in assets.uobbs:
                print(p)
            print("# meshes:")
            for p in assets.meshes:
                print(p)
            if assets.room_dir:
                print(f"# room_dir: {assets.room_dir}")
        return 0
    if args.cmd == "RCN":
        mesh = d.op_RCN(object_code=args.object_code)
        if use_json:
            mstem = Path(mesh).stem
            method = "unknown"
            if mstem.endswith("_mesh_possion"):
                method = "poisson"
            elif mstem.endswith("_mesh_af"):
                method = "af"
            elif mstem.endswith("_mesh"):
                method = "unknown"  # legacy
            print(json.dumps({"mesh": str(mesh), "method": method}, ensure_ascii=False))
        else:
            print(mesh)
        return 0
    if args.cmd == "VOL":
        mesh, vol, closed = d.op_VOL(object_code=args.object_code, auto_reconstruct=not args.no_auto_recon)
        if use_json:
            print(json.dumps({"mesh": str(mesh), "closed": closed, "volume": vol}, ensure_ascii=False))
        else:
            print(mesh)
            print(f"closed: {str(closed).lower()}")
            print(f"volume: {vol}")
        return 0
    if args.cmd == "ARE":
        mesh, area, closed = d.op_ARE(object_code=args.object_code, auto_reconstruct=not args.no_auto_recon)
        if use_json:
            print(json.dumps({"mesh": str(mesh), "closed": closed, "area": area}, ensure_ascii=False))
        else:
            print(mesh)
            print(f"closed: {str(closed).lower()}")
            print(f"area: {area}")
        return 0
    if args.cmd == "CLR":
        res = d.op_CLR(object_code=args.object_code)
        if use_json:
            print(json.dumps(res, ensure_ascii=False))
        else:
            # print raw if available
            if "raw" in res:
                print(res["raw"])  # type: ignore[index]
            else:
                print(res)
        return 0
    if args.cmd == "BBD":
        dist, vec = d.op_BBD(args.object_code_1, args.object_code_2)
        if use_json:
            print(json.dumps({
                "distance": dist,
                "vector_1_to_2": {"x": vec[0], "y": vec[1], "z": vec[2]}
            }, ensure_ascii=False))
        else:
            print(f"vector_1_to_2: {vec[0]}, {vec[1]}, {vec[2]}")
            print(f"distance: {dist}")
        return 0
    if args.cmd == "VIS":
        # Auto-detect mode and parse codes
        if args.mode:
            # Manual mode specified
            mode = args.mode
            # Parse codes based on mode
            room_codes = []
            object_codes = []
            for code in args.codes:
                code = code.strip()
                if OBJ_RE.match(code):
                    object_codes.append(code)
                elif ROOM_RE.match(code):
                    room_codes.append(code)
                else:
                    print(f"Error: Invalid code '{code}'. Expected room code 'X-Y' or object code 'X-Y-Z'")
                    return 2
        else:
            # Auto-detect mode from codes
            try:
                mode, room_codes, object_codes = auto_detect_vis_mode(args.codes)
            except ValueError as e:
                print(f"Error: {e}")
                return 2
        
        # Generate name if not provided
        name = args.name if args.name else generate_vis_name(mode, room_codes, object_codes)
        
        result = d.op_VIS(
            mode=mode,
            name=name,
            room_codes=room_codes if room_codes else None,
            object_codes=object_codes if object_codes else None,
            ratio=args.ratio,
            ratio_shell=args.ratio_shell,
            voxel=args.voxel,
            shell_no_color=args.shell_no_color,
            clean=not args.no_clean,
            clean_all=not args.no_clean_all,
            auto_serve=not args.no_serve,
            port=args.port
        )
        
        # Output JSON or human-readable format
        if use_json:
            # JSON output for API wrapper
            json_output = {
                "status": result['status'],
                "mode": result['mode'],
                "name": result['name'],
                "viewer_url": result['viewer_url']
            }
            if 'room_codes' in result:
                json_output['room_codes'] = result['room_codes']
            if 'object_codes' in result:
                json_output['objects'] = result['object_codes']
            print(json.dumps(json_output, ensure_ascii=False))
        else:
            # Human-readable output for terminal
            print(f"Status: {result['status']}")
            print(f"Mode: {result['mode']}")
            print(f"Name: {result['name']}")
            print(f"Viewer URL: {result['viewer_url']}")
            if 'room_codes' in result:
                print(f"Rooms: {', '.join(result['room_codes'])}")
            if 'object_codes' in result:
                print(f"Objects: {', '.join(result['object_codes'])}")
        
        # Skip interactive waiting if in JSON mode (for API wrapper)
        if use_json:
            return 0
        
        if args.no_serve:
            # Manual mode: user needs to start server
            pass
        elif args.no_wait:
            # Non-interactive mode: servers started, exit immediately
            pass
        else:
            # Interactive mode: wait for user selection
            import time
            selection_file = Path("/tmp/viewer_selection.json")
            
            # Clear old selection file before waiting
            if selection_file.exists():
                selection_file.unlink()
            
            timeout = 300  # 5 minutes
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if selection_file.exists():
                    try:
                        with open(selection_file, 'r') as f:
                            data = json.load(f)
                        
                        # Print selection as JSON
                        print("\n" + json.dumps(data, ensure_ascii=False, indent=2))
                        
                        # Save for later use
                        if len(data) > 0:
                            codes = [item.get('itemCode', 'unknown') for item in data]
                            last_selection = Path("/tmp/last_selection.json")
                            with open(last_selection, 'w') as f:
                                json.dump({
                                    'codes': codes,
                                    'timestamp': __import__('datetime').datetime.now().isoformat()
                                }, f, indent=2)
                        
                        # Auto-close servers
                        viewer_dir = d.root / "web" / "pointcloud-viewer"
                        stop_script = viewer_dir / "stop_dev.sh"
                        if stop_script.exists():
                            subprocess.run(['bash', str(stop_script)], 
                                         stdout=subprocess.DEVNULL, 
                                         stderr=subprocess.DEVNULL)
                        break
                    except Exception as e:
                        print(f"Error reading selection file: {e}")
                
                time.sleep(1)
            else:
                # Timeout occurred
                viewer_dir = d.root / "web" / "pointcloud-viewer"
                stop_script = viewer_dir / "stop_dev.sh"
                if stop_script.exists():
                    subprocess.run(['bash', str(stop_script)], 
                                 stdout=subprocess.DEVNULL, 
                                 stderr=subprocess.DEVNULL)
        return 0
    if args.cmd == "RMS":
        result = d.op_RMS(
            site_name=args.site_name,
            visualize=args.visualize,
            vis_name=args.vis_name,
            auto_serve=not args.no_serve,
            clean_all=not args.no_clean_all
        )
        if use_json:
            print(json.dumps(result, ensure_ascii=False))
        else:
            print(f"Site: {result['site_name']}")
            print(f"Total floors: {result['total_floors']}")
            print(f"Total rooms: {result['total_rooms']}")
            print("Room codes:")
            for code in result['room_codes']:
                print(f"  {code}")
            
            if 'visualization' in result:
                print(f"\n✓ Visualization prepared: {result['visualization']['name']}")
                print(f"  Viewer URL: {result['visualization']['viewer_url']}")
                if args.no_serve:
                    print("\n  To view, run:")
                    print(f"    cd web/pointcloud-viewer && npm run dev")
                else:
                    print(f"\n🚀 Server starting automatically...")
        return 0
    if args.cmd == "check-env":
        info = d.env_status()
        if use_json:
            print(json.dumps(info, ensure_ascii=False))
        else:
            print(f"repo_root: {info['repo_root']}")
            print(f"build_dir: {info['build_dir']}")
            print(f"output_root: {info['output_root']}")
            print("executables:")
            for k, v in info["executables"].items():
                print(f"  {k}: {'yes' if v else 'no'}")
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(_cli())
