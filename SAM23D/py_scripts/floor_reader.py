from scipy.signal import find_peaks
from pathlib import Path
import argparse
import matplotlib.pyplot as plt

import numpy as np
import open3d as o3d

def load_point_cloud(path: Path):
    """Load PLY and return height values"""
    pcd = o3d.io.read_point_cloud(str(path))
    pcd = np.asarray(pcd.points)[:, 2]
    return pcd


def detect_strong_z_levels(z: np.array, bin_size: float,
                           prominence: int, min_peak_ratio: float,
                           min_separation: float,
                           plot: bool, max_one: bool):
    """
    Detect dominant z-levels (floors/ceilings) from height distribution.

    Args:
        z: np.array of height values
        bin_size: histogram bin width
        prominence: minimum prominence for peak detection
        min_peak_ratio: fraction of max peak height to keep
        plot: whether to visualize histogram
        max_one: if True, return only the single strongest peak (for floor)

    Returns:
        np.array of detected z-levels (sorted)
    """
    bins = np.arange(np.min(z), np.max(z) + bin_size, bin_size)
    hist, edges = np.histogram(z, bins=bins)
    centers = (edges[:-1] + edges[1:]) / 2

    peaks, props = find_peaks(hist, prominence=prominence)
    if len(peaks) == 0:
        return np.array([])

    peak_heights = hist[peaks]
    max_height = np.max(peak_heights)
    strong_idx = peak_heights > (min_peak_ratio * max_height)
    strong_peaks = peaks[strong_idx]

    if len(strong_peaks) == 0:
        strong_peaks = [peaks[np.argmax(peak_heights)]]  # fallback to strongest

    # --- Merge nearby peaks ---
    sorted_idx = np.argsort(centers[strong_peaks])
    merged = []
    current_peak = strong_peaks[sorted_idx[0]]

    for idx in sorted_idx[1:]:
        peak = strong_peaks[idx]
        peak_range = centers[current_peak] + min_separation # Careful with this approach
        if centers[peak] < peak_range:
            # keep the stronger (higher histogram count)
            if hist[peak] > hist[current_peak]:
                current_peak = peak
        else:
            merged.append(current_peak)
            current_peak = peak
    merged.append(current_peak)

    # If only one main floor needed
    if max_one:
        merged = [merged[np.argmax(hist[merged])]]

    strong_levels = np.sort(centers[merged])

    # --- Visualization ---
    if plot:
        plt.figure(figsize=(10, 4))
        plt.plot(centers, hist, color='steelblue')
        plt.scatter(centers[peaks], hist[peaks], c='orange', s=30, label="All peaks")
        plt.scatter(centers[strong_peaks], hist[strong_peaks], c='red', s=60, label="Strong peaks")
        plt.xlabel("Height (m)")
        plt.ylabel("Count")
        plt.title("Height histogram with detected peaks")
        plt.legend()
        plt.savefig("data/input/classes/height_histogram.png")
        plt.show()

    return np.sort(strong_levels)

def run_heights(
    input_floor: Path,
    input_ceil: Path,
    plot: bool,
):
    pcd_floor = load_point_cloud(Path(input_floor))
    pcd_ceil = load_point_cloud(Path(input_ceil))

    # Floor → only one dominant level
    floor_levels = detect_strong_z_levels(
        pcd_floor, bin_size=0.05, prominence=80, min_peak_ratio=0.25,
        min_separation=1.0, plot=plot, max_one=True
    )

    # Ceiling → possibly multiple levels
    ceiling_levels = detect_strong_z_levels(
        pcd_ceil, bin_size=0.05, prominence=80, min_peak_ratio=0.25,
        min_separation=1.0, plot=plot, max_one=False
    )

    print("Detected floor level:", floor_levels)
    print("Detected ceiling levels:", ceiling_levels)

    # Pair nearest ceiling above the single floor
    if len(floor_levels) > 0 and len(ceiling_levels) > 0:
        f = floor_levels[0]
        ceilings_above = ceiling_levels[ceiling_levels > f]
        if len(ceilings_above) > 0:
            first_ceiling = ceilings_above[0]
            height = first_ceiling - f
            print(f"Estimated first floor height: {height:.2f} m \n")

    return floor_levels, ceiling_levels

def main(input_dir, plot):
    input_height = input_dir / "classes"

    input_floor = input_height / "floor.ply"
    input_ceil = input_height / "ceiling.ply"

    ground_floor_level, ceiling_levels = run_heights(input_floor, input_ceil, plot)
    return ground_floor_level, ceiling_levels