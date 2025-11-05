#pragma once

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <limits>
#include <utility>

namespace pcg {

struct GMMResult {
    int K = 0;                                  // chosen number of components (1..3)
    std::vector<Eigen::Vector3f> means;         // size K, in [0,255]
    std::vector<Eigen::Vector3f> variances;     // diagonal variances per channel, size K
    std::vector<float> weights;                 // mixture weights, sum to 1

    // Predict hard assignment for each point; returns component index [0..K-1] per input
    std::vector<int> predict(const std::vector<Eigen::Vector3f>& X) const;
};

struct ColorGMMOptions {
    // Adds an extra penalty to BIC per additional component beyond K=1
    // Effective BIC used for selection: BIC(K) + bic_extra_per_component * (K - 1)
    // Set to 0 for default behavior.
    float bic_extra_per_component = 0.0f;
    // If set to 1,2,3 will force the EM to evaluate only that K and return it (no model selection)
    int force_K = 0;
};

// Weighted diagonal-covariance GMM (K=1..3) with EM and BIC model selection.
// - X: RGB vectors in [0,255]
// - w: integer weights per sample (optional; if empty, all weights=1)
// - K candidates = {1,2,3}
// Implementation details:
//   * k-means++ initialization
//   * Diagonal covariances with variance floor
//   * Dirichlet prior (small) for weights stability
//   * Log-sum-exp for numerical stability
GMMResult fit_color_gmm_bic(const std::vector<Eigen::Vector3f>& X,
                            const std::vector<int>& w = {});

// Overload with options
GMMResult fit_color_gmm_bic(const std::vector<Eigen::Vector3f>& X,
                            const ColorGMMOptions& opts,
                            const std::vector<int>& w = {});

} // namespace pcg
