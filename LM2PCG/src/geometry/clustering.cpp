#include "pcg/clustering.hpp"
#include "pcg/FEC.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcg {

std::vector<pcl::PointIndices> cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const ClusteringParams& params) {
    if (!cloud || cloud->empty()) return {};
    return FEC(cloud, params.min_cluster_size, params.radius, params.max_neighbors);
}

std::vector<pcl::PointIndices> filter_clusters_by_average(
    const std::vector<pcl::PointIndices>& clusters,
    double factor) {
    if (clusters.empty()) return {};
    std::size_t total_points = 0;
    for (const auto& c : clusters) total_points += c.indices.size();
    const double avg = static_cast<double>(total_points) / static_cast<double>(clusters.size());
    const double threshold = avg * factor;

    std::vector<pcl::PointIndices> kept;
    kept.reserve(clusters.size());
    for (const auto& c : clusters) {
        if (static_cast<double>(c.indices.size()) >= threshold) {
            kept.push_back(c);
        }
    }
    return kept;
}

} // namespace pcg
