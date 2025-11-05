// Lightweight, namespaced version of the original FEC header-only clustering.
#pragma once
#ifndef PCG_SEGMENT_FEC_H
#define PCG_SEGMENT_FEC_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cstring>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

namespace pcg {

// Store index and label information for each point
struct PointIndex_NumberTag {
    float nPointIndex;
    float nNumberTag;
};

inline bool NumberTagLess(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1) {
    return p0.nNumberTag < p1.nNumberTag;
}

// FEC clustering: radius-based fast equivalent class labeling
inline std::vector<pcl::PointIndices> FEC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          int min_component_size,
                                          double tolerance,
                                          int max_n) {
    using std::size_t;
    size_t i, j;
    const size_t cloud_size = cloud ? cloud->size() : 0;
    std::vector<pcl::PointIndices> empty;
    if (!cloud || cloud_size == 0) { return empty; }

    pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdtreeflann;
    cloud_kdtreeflann.setInputCloud(cloud);

    std::vector<int> marked_indices(cloud_size, 0);
    std::vector<int> pointIdx;
    std::vector<float> pointSquaredDistance;

    int tag_num = 1;
    int temp_tag_num = -1;

    for (i = 0; i < cloud_size; ++i) {
        // Clustering process
        if (marked_indices[i] == 0) { // not yet labeled
            pointIdx.clear();
            pointSquaredDistance.clear();
            cloud_kdtreeflann.radiusSearch(cloud->points[i], tolerance, pointIdx, pointSquaredDistance, max_n);

            int min_tag_num = tag_num;
            for (j = 0; j < pointIdx.size(); ++j) {
                // find smallest existing label among neighbors
                if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num)) {
                    min_tag_num = marked_indices[pointIdx[j]];
                }
            }

            for (j = 0; j < pointIdx.size(); ++j) {
                temp_tag_num = marked_indices[pointIdx[j]];
                if (temp_tag_num > min_tag_num) {
                    for (size_t k = 0; k < cloud_size; ++k) {
                        if (marked_indices[k] == temp_tag_num) {
                            marked_indices[k] = min_tag_num;
                        }
                    }
                }
                marked_indices[pointIdx[j]] = min_tag_num;
            }
            ++tag_num;
        }
    }

    std::vector<PointIndex_NumberTag> indices_tags(cloud_size);
    for (i = 0; i < cloud_size; ++i) {
        indices_tags[i] = PointIndex_NumberTag{static_cast<float>(i), static_cast<float>(marked_indices[i])};
    }
    std::sort(indices_tags.begin(), indices_tags.end(), NumberTagLess);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    size_t begin_index = 0;
    for (i = 0; i < indices_tags.size(); ++i) {
        // Relabel each cluster
        if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag) {
            if ((i - begin_index) >= static_cast<size_t>(min_component_size)) {
                size_t m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; ++j) {
                    inliers->indices[m++] = static_cast<int>(indices_tags[j].nPointIndex);
                }
                cluster_indices.push_back(*inliers);
            }
            begin_index = i;
        }
    }

    if ((i - begin_index) >= static_cast<size_t>(min_component_size)) {
        size_t m = 0;
        inliers->indices.resize(i - begin_index);
        for (j = begin_index; j < i; ++j) {
            inliers->indices[m++] = static_cast<int>(indices_tags[j].nPointIndex);
        }
        cluster_indices.push_back(*inliers);
    }

    return cluster_indices;
}

} // namespace pcg

#endif // PCG_SEGMENT_FEC_H
