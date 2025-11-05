#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace pcg {

struct UOBB {
    Eigen::Vector3f center;              // box center
    Eigen::Vector3f size;                // lengths along e1,e2,up (lx, ly, lz)
    Eigen::Matrix3f R;                   // columns are e1,e2,up; up is (0,0,1)
    std::array<Eigen::Vector3f, 8> corners; // 8 corner vertices
    double area = 0.0;                   // base area lx*ly
    double volume = 0.0;                 // volume lx*ly*lz
    double yaw = 0.0;                    // rotation angle around up from +X
};

// Compute vertically-aligned OBB for a cluster.
// up = (0,0,1). Indices refer to points in cloud.
UOBB compute_uobb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                  const std::vector<int>& indices);

// Save UOBB as a triangular mesh .ply (binary) at path.
bool save_uobb_ply(const std::string& filepath, const UOBB& box);

} // namespace pcg
