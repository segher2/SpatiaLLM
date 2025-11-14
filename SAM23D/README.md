# SAM23D 
SAM23D is a lightweight pipeline that transforms user-selected regions in panoramic images into 3D object point-cloud clusters, enriched with semantic labels, upright bounding boxes, and optional interactive visualization.  
It integrates SAM2 (Segment Anything Model 2), mask2cluster, camera geometry, and Azure OpenAI Vision to generate 3D object assets directly from a single panorama.

---

## Overview

### 1. 2D Object Selection (SAM2)
`sam2_predictor.py`:
- User selects one or more points on a panorama.
- SAM2 generates:
  - Segmentation mask
  - Overlay visualization
  - Binary mask
  - Cropped object panorama
- Outputs saved in `SAM23D/outputs/`.

---

### 2. Mask → 3D Point Cloud Filtering
`select_points_in_mask.py`:
- Loads the room’s point cloud.
- Reads the camera pose and reprojects points into the panorama.
- Retains only points inside the SAM2 mask.
- Saves:
  - `<stem>_initial.ply`
  - `<stem>_test.ply` (optionally refined with mask2cluster)

---

### 3. Optional: 3D Visualization
`visualize_latest.py`:
- Finds latest `_test.ply`
- Copies it into the LM2PCG pointcloud viewer
- Generates a manifest
- Starts the viewer server
- Prints the URL for inspection

---

### 4. Semantic Labeling & Room Integration
`integrate_object_to_room.py`:
- Loads latest cropped panorama (`*_cropped.png`)
- Sends to Azure OpenAI Vision → single-word semantic label
- Locates corresponding room folder
- Copies cluster PLY into room structure
- Computes Upright Oriented Bounding Box (UOBB)
- Appends object entry to room CSV

