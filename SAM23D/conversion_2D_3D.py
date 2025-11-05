# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
# import os
#
# # -------------------------------
# # Math helpers
#
# def quat_to_matrix(qx, qy, qz, qw):
#     """Quaternion (x,y,z,w) -> rotation matrix R (world_from_cam)."""
#     # normalize
#     n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#     qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
#     xx, yy, zz = qx*qx, qy*qy, qz*qz
#     xy, xz, yz = qx*qy, qx*qz, qy*qz
#     wx, wy, wz = qw*qx, qw*qy, qw*qz
#     R = np.array([
#         [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
#         [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
#         [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
#     ])
#     return R
#
# def yaw_only_world_from_cam(R_world_from_cam):
#     """
#     Keep only yaw about world Z from the full rotation.
#     We compute yaw as the heading of the camera's +X axis projected onto XY.
#     """
#     # Camera +X axis expressed in world coords is the first column of R
#     cam_x_in_world = R_world_from_cam[:, 0]
#     # Heading in XY plane
#     yaw = np.arctan2(cam_x_in_world[1], cam_x_in_world[0])  # atan2(y, x)
#     cy, sy = np.cos(yaw), np.sin(yaw)
#     Rz = np.array([
#         [ cy, -sy, 0],
#         [ sy,  cy, 0],
#         [  0,   0, 1]
#     ])
#     return Rz, yaw
#
# def pixel_to_ray(u, v, W, H):
#     """
#     Pixel (u,v) in equirectangular panorama -> unit ray in camera coords,
#     with camera axes: X=forward, Y=right, Z=up (right-handed, Z-up).
#     lon=0 faces +X, lon increases to the right (Y+), lat increases upward (Z+).
#     """
#     lon = 2 * np.pi * (u / W - 0.5)   # [-pi, pi], 0 at image center
#     lat = np.pi * (0.5 - v / H)       # [-pi/2, pi/2], 0 at center
#     x = np.cos(lat) * np.cos(lon)     # forward
#     y = np.cos(lat) * np.sin(lon)     # right
#     z = np.sin(lat)                   # up
#     d = np.array([x, y, z], dtype=float)
#     return d / np.linalg.norm(d)
#
# # -------------------------------
# # Load your panorama (for W,H only)
# pano_path = "extracted_data/images/00000-pano.jpg"
# if os.path.exists(pano_path):
#     pano = mpimg.imread(pano_path)
#     H, W = pano.shape[0], pano.shape[1]
# else:
#     # Fallback if running without the image available (edit if needed)
#     W, H = 4096, 2048
#     pano = None
#
# # Pose from JSON (world_from_cam)
# pose = {
#     "translation": {"x": -5.201647051482847,
#                     "y": -6.979796067950823,
#                     "z": 0.9797140526741788},
#     "rotation": {"x": -0.008890023587453206,
#                  "y": -0.03365608061995312,
#                  "z": -0.7481927333364353,
#                  "w": 0.6625676339065104}
# }
#
# t_wc = np.array([pose["translation"]["x"],
#                  pose["translation"]["y"],
#                  pose["translation"]["z"]], dtype=float)
#
# qx, qy, qz, qw = (pose["rotation"]["x"],
#                   pose["rotation"]["y"],
#                   pose["rotation"]["z"],
#                   pose["rotation"]["w"])
#
# # Full rotation and yaw-only (horizon leveled)
# R_full = quat_to_matrix(qx, qy, qz, qw)
# R_yaw, yaw = yaw_only_world_from_cam(R_full)  # world_from_cam with only yaw about Z
#
# # -------------------------------
# # Pick sample pixels (edit freely)
# pixels = [
#     (W//2, H//2),     # center
#     (W//4, H//2),     # left
#     (3*W//4, H//2),   # right
#     (W//2, H//4),     # top
#     (W//2, 3*H//4)    # bottom
# ]
#
# # Compute rays in world (leveled)
# ray_length = 5.0  # meters
# rays = []
# for i, (u, v) in enumerate(pixels):
#     dir_c = pixel_to_ray(u, v, W, H)          # camera coords (X fwd, Y right, Z up)
#     dir_w = R_yaw @ dir_c                     # world coords (yaw-only, leveled)
#     dir_w /= np.linalg.norm(dir_w)
#     p0 = t_wc
#     p1 = t_wc + ray_length * dir_w
#     rays.append((i, p0, p1))
#
# # -------------------------------
# # Optional quick viz (panorama dots + 3D rays)
# if pano is not None:
#     import matplotlib.pyplot as plt
#     plt.figure(figsize=(12,6))
#     plt.imshow(pano)
#     for (u,v) in pixels:
#         plt.scatter(u, v, s=50)
#     plt.title("Panorama with sampled pixels")
#     plt.show()
#
# fig = plt.figure(figsize=(8,8))
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(*t_wc, s=50, label="Camera")
# for i, p0, p1 in rays:
#     line = np.vstack([p0, p1])
#     ax.plot(line[:,0], line[:,1], line[:,2], label=f"ray {i}")
# ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
# ax.set_title("World rays (Z-up, yaw-leveled)")
# ax.legend()
# plt.show()
#
# # -------------------------------
# # Export: OBJ polylines (best for CloudCompare)
# obj_lines = []
# verts = []
# for i, (pid, p0, p1) in enumerate(rays):
#     verts.append(p0)
#     verts.append(p1)
# # Write vertices
# obj_lines.append("# Rays as line segments\n")
# for v in verts:
#     obj_lines.append(f"v {v[0]} {v[1]} {v[2]}\n")
# # Write line indices (1-based)
# for i in range(len(rays)):
#     a = 2*i + 1
#     b = 2*i + 2
#     obj_lines.append(f"l {a} {b}\n")
#
# with open("rays.obj", "w") as f:
#     f.writelines(obj_lines)
#
# # Export: CSV (origin & endpoint)
# import csv
# with open("rays.csv", "w", newline="") as f:
#     w = csv.writer(f)
#     w.writerow(["ray_id","x0","y0","z0","x1","y1","z1"])
#     for i, p0, p1 in rays:
#         w.writerow([i, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]])
#
# print("Wrote rays.obj (polylines) and rays.csv (segments).")
# print(f"Yaw (deg): {np.degrees(yaw):.2f}")

import numpy as np
import os

# -------------------------------
# Math helpers

def quat_to_matrix(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> rotation matrix R (world_from_cam)."""
    # normalize
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ])
    return R

def yaw_only_world_from_cam(R_world_from_cam):
    """
    Keep only yaw about world Z from the full rotation.
    We compute yaw as the heading of the camera's +X axis projected onto XY.
    """
    # Camera +X axis expressed in world coords is the first column of R
    cam_x_in_world = R_world_from_cam[:, 0]
    # Heading in XY plane
    yaw = np.arctan2(cam_x_in_world[1], cam_x_in_world[0])  # atan2(y, x)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([
        [ cy, -sy, 0],
        [ sy,  cy, 0],
        [  0,   0, 1]
    ])
    return Rz, yaw

def pixel_to_ray(u, v, W, H):
    """
    Pixel (u,v) in equirectangular panorama -> unit ray in camera coords,
    with camera axes: X=forward, Y=right, Z=up (right-handed, Z-up).
    lon=0 faces +X, lon increases to the right (Y+), lat increases upward (Z+).
    """
    lon = 2 * np.pi * (u / W - 0.5)   # [-pi, pi], 0 at image center
    lat = np.pi * (0.5 - v / H)       # [-pi/2, pi/2], 0 at center
    x = np.cos(lat) * np.cos(lon)     # forward
    y = np.cos(lat) * np.sin(lon)     # right
    z = np.sin(lat)                   # up
    d = np.array([x, y, z], dtype=float)
    return d / np.linalg.norm(d)

# -------------------------------
# Test/Demo code - only runs when script is executed directly
if __name__ == "__main__":
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

    # Load your panorama (for W,H only)
    pano_path = "extracted_data/images/00000-pano.jpg"
    if os.path.exists(pano_path):
        pano = mpimg.imread(pano_path)
        H, W = pano.shape[0], pano.shape[1]
    else:
        # Fallback if running without the image available (edit if needed)
        W, H = 4096, 2048
        pano = None

    # Pose from JSON (world_from_cam)
    pose = {
        "translation": {"x": -5.201647051482847,
                        "y": -6.979796067950823,
                        "z": 0.9797140526741788},
        "rotation": {"x": -0.008890023587453206,
                     "y": -0.03365608061995312,
                     "z": -0.7481927333364353,
                     "w": 0.6625676339065104}
    }

    t_wc = np.array([pose["translation"]["x"],
                     pose["translation"]["y"],
                     pose["translation"]["z"]], dtype=float)

    qx, qy, qz, qw = (pose["rotation"]["x"],
                      pose["rotation"]["y"],
                      pose["rotation"]["z"],
                      pose["rotation"]["w"])

    # Full rotation and yaw-only (horizon leveled)
    R_full = quat_to_matrix(qx, qy, qz, qw)
    R_yaw, yaw = yaw_only_world_from_cam(R_full)  # world_from_cam with only yaw about Z

    # -------------------------------
    # Pick sample pixels (edit freely)
    pixels = [
        (W//2, H//2),     # center
        (W//4, H//2),     # left
        (3*W//4, H//2),   # right
        (W//2, H//4),     # top
        (W//2, 3*H//4)    # bottom
    ]

    # Compute rays in world (leveled)
    ray_length = 5.0  # meters
    rays = []
    for i, (u, v) in enumerate(pixels):
        dir_c = pixel_to_ray(u, v, W, H)          # camera coords (X fwd, Y right, Z up)
        dir_w = R_yaw @ dir_c                     # world coords (yaw-only, leveled)
        dir_w /= np.linalg.norm(dir_w)
        p0 = t_wc
        p1 = t_wc + ray_length * dir_w
        rays.append((i, p0, p1))

    # -------------------------------
    # Optional quick viz (panorama dots + 3D rays)
# Disabled to avoid GUI issues when running in Flask worker thread
# if pano is not None:
#     import matplotlib.pyplot as plt
#     plt.figure(figsize=(12,6))
#     plt.imshow(pano)
#     for (u,v) in pixels:
#         plt.scatter(u, v, s=50)
#     plt.title("Panorama with sampled pixels")
#     plt.show()

# fig = plt.figure(figsize=(8,8))
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(*t_wc, s=50, label="Camera")
# for i, p0, p1 in rays:
#     line = np.vstack([p0, p1])
#     ax.plot(line[:,0], line[:,1], line[:,2], label=f"ray {i}")
# ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
# ax.set_title("World rays (Z-up, yaw-leveled)")
# ax.legend()
# plt.show()

    # -------------------------------
    # Export: OBJ polylines (best for CloudCompare)
    obj_lines = []
    verts = []
    for i, (pid, p0, p1) in enumerate(rays):
        verts.append(p0)
        verts.append(p1)
    # Write vertices
    obj_lines.append("# Rays as line segments\n")
    for v in verts:
        obj_lines.append(f"v {v[0]} {v[1]} {v[2]}\n")
    # Write line indices (1-based)
    for i in range(len(rays)):
        a = 2*i + 1
        b = 2*i + 2
        obj_lines.append(f"l {a} {b}\n")

    with open("rays.obj", "w") as f:
        f.writelines(obj_lines)

    # Export: CSV (origin & endpoint)
    import csv
    with open("rays.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ray_id","x0","y0","z0","x1","y1","z1"])
        for i, p0, p1 in rays:
            w.writerow([i, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]])

    print("Wrote rays.obj (polylines) and rays.csv (segments).")
    print(f"Yaw (deg): {np.degrees(yaw):.2f}")