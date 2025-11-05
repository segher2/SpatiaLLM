#pragma once

#include <string>

namespace pcg {

struct ParamsConfig {
    double radius = 0.02;
    int min_cluster_size = 50;
    int max_neighbors = 100;
    double filter_factor = 0.30; // 30% of average
    // If max_cluster_size / min_cluster_size <= no_filter_ratio, skip filtering entirely
    double no_filter_ratio = 2.0;
    // Color sampling parameters
    int color_sample_n = 300; // number of RGB points to sample per cluster (cap at size)
    // Color GMM model selection bias (additional BIC penalty per extra component)
    // Effective BIC: BIC(K) + color_bic_k_penalty * (K - 1)
    double color_bic_k_penalty = 0.0;
    // Color GMM post rules
    double color_min_weight = 0.10;   // discard components with weight < this
    double color_max_stddev = 60.0;   // discard components with any channel stddev > this
    double color_deltaE_keep  = 20.0; // Single breakpoint: if ΔE*76 < this -> merge (drop lower-weight), else keep both

    // Reconstruction parameters
    // Poisson
    int poisson_spacing_neighbors = 6;   // neighbors used to compute average spacing
    int poisson_normal_neighbors  = 18;  // neighbors for jet normal estimation and MST orientation
    // Poisson acceptance / validation
    double poisson_min_oriented_fraction = 0.3; // MST-oriented normals fraction threshold
    bool   poisson_require_closed = true;       // require closed mesh from Poisson
    double poisson_invalid_ratio_vs_hull = 1.6; // Poisson volume must be <= this * hull volume
    // Advancing Front (AF)
    int  af_min_points = 3;          // minimum points required to attempt AF
    bool af_require_closed = false;  // if true, reject AF result when not closed

    // Global output formatting
    bool json_output = false;  // if true, apps emit structured JSON output instead of human-readable text
};

// Loads a very small subset of YAML/INI: lines like `key: value` or `key=value`
// Supported keys: radius (double), min_cluster_size (int), max_neighbors (int), filter_factor (double)
// Returns true on success (file opened). Unknown keys are ignored.
bool load_params_from_file(const std::string& filepath, ParamsConfig& out, std::string* error = nullptr);

} // namespace pcg
