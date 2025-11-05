#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace pcg {

// Minimal PLY loader for the provided header schema. Reads only vertex positions (x,y,z).
// Supports binary_little_endian 1.0 with the specific fields listed, ignores color and scalars.
bool load_ply_xyz(const std::string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                  std::string* error = nullptr);

// Color-aware PLY loader. Always fills XYZ. If RGB fields (red, green, blue) exist, also fills
// cloud_rgb_out and sets has_color=true. On files without RGB, cloud_rgb_out is reset and has_color=false.
// Still only supports binary_little_endian 1.0.
bool load_ply_xyz_or_rgb(const std::string& filepath,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz_out,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_rgb_out,
                         bool& has_color,
                         std::string* error = nullptr);

} // namespace pcg
