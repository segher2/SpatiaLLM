#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace m2c {

using PointT = pcl::PointXYZ;      // Basic XYZ point used across the pipeline.
using CloudT = pcl::PointCloud<PointT>;  // Shared point cloud container alias.

struct Pose {
	Eigen::Vector3f C;  // Reference point derived solely from pose translation.
};

struct Params {
	float eps;          // DBSCAN neighborhood radius (meters).
	int minPts_core;    // Minimum neighbors for a core point.
	int minPts_total;   // Minimum total points required for an accepted cluster.
	float maxDiameter;  // Maximum spatial diameter allowed for accepted clusters.
	int maxPts;         // Safety cap on processed points per cloud.
	int max_trials;     // Maximum number of seed attempts before aborting.
	float voxel;        // Optional voxel grid size; zero disables downsampling.
	// FEC-based selection parameters
	float n;            // Fraction multiplier for mean cluster size: floor(n * mean_size).
	int m;              // Top-M nearest points to C for voting among clusters.
};

}  // namespace m2c
