#include "m2c/kdtree.h"

#include <stdexcept>

#include <pcl/search/kdtree.h>

namespace m2c {

struct KD::State {
  CloudT::ConstPtr input_cloud;
  pcl::search::KdTree<PointT>::Ptr tree;
};

KD::KD(const CloudT& cloud) : state_(std::make_shared<State>()) {
  if (cloud.empty()) {
    throw std::invalid_argument("Cannot build KDTree on an empty cloud");
  }

  state_->input_cloud = CloudT::ConstPtr(&cloud, [](const CloudT*) {});
  state_->tree.reset(new pcl::search::KdTree<PointT>);
  state_->tree->setInputCloud(state_->input_cloud);
}

void KD::radius(int idx, float r, std::vector<int>& out) const {
  if (!state_ || !state_->tree) {
    throw std::runtime_error("KD tree state not initialized");
  }
  if (idx < 0 || static_cast<std::size_t>(idx) >= state_->input_cloud->size()) {
    throw std::out_of_range("Query index out of bounds");
  }
  if (r <= 0.0f) {
    out.clear();
    return;
  }

  out.clear();
  std::vector<float> distances;
  const bool ok = state_->tree->radiusSearch(idx, static_cast<double>(r), out, distances);
  if (!ok) {
    out.clear();
  }
}

}  // namespace m2c
