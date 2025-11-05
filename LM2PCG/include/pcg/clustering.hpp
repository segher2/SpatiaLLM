#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>
#include <string>

namespace pcg {

struct ClusteringParams {
    double radius = 0.02;       // radius for neighbor search
    int max_neighbors = 100;    // cap neighbors per query
    int min_cluster_size = 50;  // default aligns with triangulate.cpp
};

// Run clustering on a PCL XYZ cloud using FEC, return clusters
std::vector<pcl::PointIndices> cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const ClusteringParams& params);

// Filter clusters by dynamic threshold = factor * average cluster size
// Returns a new vector with clusters whose size >= threshold
std::vector<pcl::PointIndices> filter_clusters_by_average(
    const std::vector<pcl::PointIndices>& clusters,
    double factor /* e.g., 0.1 for 10% */);

} // namespace pcg
