#include "m2c/pipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl/PointIndices.h>

#include "m2c/kdtree.h"
#include "m2c/validator.h"
#include "pcg/FEC.hpp"

namespace m2c {
namespace {

struct Candidate {
  int index;
  float distance;
};

}  // namespace

Result selectCluster(const CloudT& cloud, const Pose& pose, const Params& params) {
  Result result;

  if (cloud.empty()) {
    return result;
  }

  // 1) Convert RGB cloud to XYZ for FEC clustering (FEC expects PointXYZ)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_xyz->reserve(cloud.size());
  for (const auto& pt : cloud) {
    pcl::PointXYZ p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    cloud_xyz->push_back(p);
  }
  cloud_xyz->width = cloud.width;
  cloud_xyz->height = cloud.height;
  cloud_xyz->is_dense = cloud.is_dense;

  // 2) FEC clustering on the XYZ cloud
  const int min_component_size = 1;           // initial FEC labeling without size filter
  const double tolerance = static_cast<double>(std::max(params.eps, 1e-6f));  // reuse eps as tolerance
  const int max_n = std::max(8, params.minPts_core);  // neighbor cap in radiusSearch

  const std::vector<pcl::PointIndices> fec_clusters = pcg::FEC(cloud_xyz, min_component_size, tolerance, max_n);
  if (fec_clusters.empty()) {
    return result;
  }

  // 3) Compute average cluster size and derive minimum size threshold = floor(n * k)
  std::size_t sum_sizes = 0;
  for (const auto& ci : fec_clusters) sum_sizes += ci.indices.size();
  const double k = static_cast<double>(sum_sizes) / static_cast<double>(fec_clusters.size());
  const int min_keep = std::max(1, static_cast<int>(std::floor(params.n * k)));

  // Map point -> clusterId for clusters that pass the threshold
  std::vector<int> point_to_cluster(cloud.size(), -1);
  std::vector<int> kept_cluster_ids;
  kept_cluster_ids.reserve(fec_clusters.size());

  for (std::size_t cid = 0; cid < fec_clusters.size(); ++cid) {
    const auto& ci = fec_clusters[cid];
    if (static_cast<int>(ci.indices.size()) < min_keep) {
      continue;
    }
    kept_cluster_ids.push_back(static_cast<int>(cid));
    for (int idx : ci.indices) {
      if (idx >= 0 && static_cast<std::size_t>(idx) < point_to_cluster.size()) {
        point_to_cluster[idx] = static_cast<int>(cid);
      }
    }
  }

  if (kept_cluster_ids.empty()) {
    return result;
  }

  // 4) Find the m points (across kept clusters) nearest to C
  struct NearRec { float dist; int idx; int cid; };
  std::vector<NearRec> pool;
  pool.reserve(cloud.size());

  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const int cid = point_to_cluster[i];
    if (cid < 0) continue;  // skip filtered-out clusters
    const PointT& p = cloud[i];
    const float dx = p.x - pose.C.x();
    const float dy = p.y - pose.C.y();
    const float dz = p.z - pose.C.z();
    const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    pool.push_back({dist, static_cast<int>(i), cid});
  }

  if (pool.empty()) {
    return result;
  }

  const std::size_t take = std::min<std::size_t>(static_cast<std::size_t>(std::max(1, params.m)), pool.size());
  std::nth_element(pool.begin(), pool.begin() + take, pool.end(), [](const NearRec& a, const NearRec& b){ return a.dist < b.dist; });
  pool.resize(take);

  // 4) Vote: cluster with the most occurrences among the top-m nearest points
  std::unordered_map<int, int> counts;          // cid -> count
  std::unordered_map<int, double> dist_sums;    // cid -> sum of distances (for tie-breaker)
  int best_cid = -1;
  int best_count = -1;
  double best_sum = std::numeric_limits<double>::infinity();

  for (const auto& rec : pool) {
    int& c = counts[rec.cid];
    c += 1;
    dist_sums[rec.cid] += rec.dist;
    if (c > best_count || (c == best_count && dist_sums[rec.cid] < best_sum)) {
      best_count = c;
      best_sum = dist_sums[rec.cid];
      best_cid = rec.cid;
    }
  }

  if (best_cid < 0) {
    return result;
  }

  // Compose result from the selected cluster
  const auto& chosen = fec_clusters[static_cast<std::size_t>(best_cid)];
  Cluster out;
  out.indices = chosen.indices;

  // estimate diameter via AABB
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();
  for (int idx : out.indices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) continue;
    const PointT& p = cloud[idx];
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
  }
  const float dx = max_x - min_x;
  const float dy = max_y - min_y;
  const float dz = max_z - min_z;
  out.diameter = std::sqrt(dx * dx + dy * dy + dz * dz);

  result.found = true;
  result.trials = 1;
  result.cluster = std::move(out);
  return result;
}

}  // namespace m2c
