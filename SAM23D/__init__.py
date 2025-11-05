"""
SAM23D - SAM2 to 3D Point Cloud Pipeline

A pipeline for converting 2D image segmentation masks (from SAM2) 
to filtered 3D point clouds.

Modules:
- sam2_predictor: SAM2 image segmentation
- select_points_in_mask: Point cloud filtering
- conversion_2D_3D: Coordinate transformations
"""

__version__ = "1.0.0"
__author__ = "Spatial Understanding Team"

# Import main functions for convenience
try:
    from .sam2_predictor import run_sam2_prediction
except ImportError:
    pass

try:
    from .select_points_in_mask import filter_point_cloud_with_mask
except ImportError:
    pass

try:
    from .conversion_2D_3D import quat_to_matrix, pixel_to_ray, yaw_only_world_from_cam
except ImportError:
    pass

__all__ = [
    'run_sam2_prediction',
    'filter_point_cloud_with_mask',
    'quat_to_matrix',
    'pixel_to_ray',
    'yaw_only_world_from_cam'
]
