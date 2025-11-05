#include "m2c/validator.h"

namespace m2c {

bool Validator::pass(const Cluster& cluster) const {
  if (minPtsTotal <= 0) {
    return false;
  }
  if (static_cast<int>(cluster.indices.size()) < minPtsTotal) {
    return false;
  }
  if (maxDiameter > 0.0f && cluster.diameter > maxDiameter) {
    return false;
  }
  return true;
}

bool Validator::sureNoise(const Cluster& cluster) const {
  if (minPtsTotal <= 0) {
    return true;
  }
  const float threshold = 0.3f * static_cast<float>(minPtsTotal);
  return static_cast<float>(cluster.indices.size()) < threshold;
}

}  // namespace m2c
